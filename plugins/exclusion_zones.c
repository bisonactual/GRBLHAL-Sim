/*
  exclusion_zones.c — Multi-zone 3D exclusion/inclusion plugin for grblHAL.

  Compatible with the Sienci ATCi keepout plugin hook architecture.
  Designed for integration with FlexiSender (web sender for grblHAL).

  Zones are axis-aligned 3D boxes (X/Y/Z min/max).
  Each zone has per-operation permission flags:
    bit 0 = allow_gcode      (G-code moves may enter)
    bit 1 = allow_jog         (Jog moves may enter)
    bit 2 = allow_toolchange  (Tool-change macros may enter)
    bit 3 = enabled           (Zone is active)

  Zone 0 is reserved for the ATCi plugin — it mirrors settings $684-$687
  (X min, Y min, X max, Y max) with Z undefined (blocks all depths).
  It bans gcode and jog but allows toolchange. Cannot be written or deleted
  via $ZONE commands.

  If a zone has z_min == 0 and z_max == 0, the Z axis is ignored and the
  zone blocks at all depths.

  Commands:
    $EXCLUSION=1 / $EXCLUSION=0                              — global enable/disable
    $ZONE                                                    — list active zones
    $ZONE=n,xmin,ymin,zmin,xmax,ymax,zmax[,flags]           — set/update zone (n >= 1)
    $ZONE-n                                                  — delete zone n (n >= 1)

  Realtime report appends: |EZ:EZ  (E=enabled, Z=inside a zone)
*/

#include "driver.h"

#if EXCLUSION_ZONES_ENABLE

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "grbl/hal.h"
#include "grbl/nvs_buffer.h"
#include "grbl/gcode.h"
#include "grbl/system.h"
#include "grbl/motion_control.h"
#include "grbl/settings.h"
#include "grbl/plugins.h"
#include "grbl/task.h"

#include "exclusion_zones.h"

/* ── ATCi setting IDs (must match sienci-atci-plugin) ─────────────────────── */
#define ATCI_SETTING_X_MIN 684
#define ATCI_SETTING_Y_MIN 685
#define ATCI_SETTING_X_MAX 686
#define ATCI_SETTING_Y_MAX 687

/* ── Static state ─────────────────────────────────────────────────────────── */

static ez_config_t config;
static nvs_address_t nvs_addr;
static bool tc_macro_running = false;

static on_report_options_ptr   on_report_options   = NULL;
static on_realtime_report_ptr  on_realtime_report  = NULL;
static on_tool_selected_ptr    prev_on_tool_selected = NULL;
static on_tool_changed_ptr     prev_on_tool_changed  = NULL;
static on_unknown_sys_command_ptr prev_on_unknown_sys_command = NULL;

typedef bool (*travel_limits_ptr)(float *target, axes_signals_t axes, bool is_cartesian, work_envelope_t *envelope);
typedef void (*apply_travel_limits_ptr)(float *target, float *position, work_envelope_t *envelope);
static travel_limits_ptr       prev_check_travel_limits = NULL;
static apply_travel_limits_ptr prev_apply_travel_limits = NULL;

static void ez_save(void);
static void ez_load(void);
static void ez_restore(void);

/* ── Zone 0 ATCi sync ────────────────────────────────────────────────────── */

static void sync_zone0_from_atci(void)
{
    setting_id_t ids[4] = { ATCI_SETTING_X_MIN, ATCI_SETTING_Y_MIN,
                            ATCI_SETTING_X_MAX, ATCI_SETTING_Y_MAX };
    float vals[4] = { 0 };

    for (int i = 0; i < 4; i++) {
        setting_detail_t *s = setting_get_details(ids[i], NULL);
        if (s)
            vals[i] = setting_get_float_value(s, 0);
    }

    zone_t *z = &config.zones[0];
    z->x_min = fminf(vals[0], vals[2]);
    z->y_min = fminf(vals[1], vals[3]);
    z->z_min = 0.0f;
    z->x_max = fmaxf(vals[0], vals[2]);
    z->y_max = fmaxf(vals[1], vals[3]);
    z->z_max = 0.0f;
    z->flags.value = 0x0C;

    if (config.count < 1)
        config.count = 1;
}

static void sync_zone0_delayed(void *data)
{
    sync_zone0_from_atci();
}

/* ── Zone geometry helpers ────────────────────────────────────────────────── */

static inline bool zone_z_undefined(const zone_t *z)
{
    return z->z_min == 0.0f && z->z_max == 0.0f;
}

static inline void zone_normalize(zone_t *z)
{
    float tmp;
    if (z->x_min > z->x_max) { tmp = z->x_min; z->x_min = z->x_max; z->x_max = tmp; }
    if (z->y_min > z->y_max) { tmp = z->y_min; z->y_min = z->y_max; z->y_max = tmp; }
    if (z->z_min > z->z_max) { tmp = z->z_min; z->z_min = z->z_max; z->z_max = tmp; }
}

static bool point_in_zone(const zone_t *z, float x, float y, float zv)
{
    bool xy = x >= z->x_min && x <= z->x_max &&
              y >= z->y_min && y <= z->y_max;
    if (!xy) return false;
    if (zone_z_undefined(z)) return true;
    return zv >= z->z_min && zv <= z->z_max;
}

static bool point_deep_in_zone(const zone_t *z, float x, float y, float zv)
{
    bool xy = x > (z->x_min + EZ_TOLERANCE) && x < (z->x_max - EZ_TOLERANCE) &&
              y > (z->y_min + EZ_TOLERANCE) && y < (z->y_max - EZ_TOLERANCE);
    if (!xy) return false;
    if (zone_z_undefined(z)) return true;
    return zv > (z->z_min + EZ_TOLERANCE) && zv < (z->z_max - EZ_TOLERANCE);
}

static bool segment_intersects_zone(const zone_t *z, const float *p0, const float *p1)
{
    float t0 = 0.0f, t1 = 1.0f;
    int dims = zone_z_undefined(z) ? 2 : 3;
    float d[3]  = { p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2] };
    float lo[3] = { z->x_min + EZ_TOLERANCE, z->y_min + EZ_TOLERANCE, z->z_min + EZ_TOLERANCE };
    float hi[3] = { z->x_max - EZ_TOLERANCE, z->y_max - EZ_TOLERANCE, z->z_max - EZ_TOLERANCE };

    for (int i = 0; i < dims; i++) {
        float p_neg = -d[i], q_neg = p0[i] - lo[i];
        float p_pos =  d[i], q_pos = hi[i] - p0[i];
        if (p_neg == 0.0f && p_pos == 0.0f) {
            if (q_neg < 0.0f || q_pos < 0.0f) return false;
        } else {
            float t;
            t = q_neg / p_neg;
            if (p_neg < 0.0f) { if (t > t1) return false; if (t > t0) t0 = t; }
            else               { if (t < t0) return false; if (t < t1) t1 = t; }
            t = q_pos / p_pos;
            if (p_pos < 0.0f) { if (t > t1) return false; if (t > t0) t0 = t; }
            else               { if (t < t0) return false; if (t < t1) t1 = t; }
        }
    }
    return t0 < t1;
}

static bool clip_to_zone_boundary(const zone_t *z, const float *start, const float *end, float *clipped)
{
    float t0 = 0.0f, t1 = 1.0f;
    int dims = zone_z_undefined(z) ? 2 : 3;
    float d[3]  = { end[0] - start[0], end[1] - start[1], end[2] - start[2] };
    float lo[3] = { z->x_min, z->y_min, z->z_min };
    float hi[3] = { z->x_max, z->y_max, z->z_max };

    for (int i = 0; i < dims; i++) {
        float p_neg = -d[i], q_neg = start[i] - lo[i];
        float p_pos =  d[i], q_pos = hi[i] - start[i];
        if (p_neg == 0.0f && p_pos == 0.0f) {
            if (q_neg < 0.0f || q_pos < 0.0f) return false;
        } else {
            float t;
            t = q_neg / p_neg;
            if (p_neg < 0.0f) { if (t > t1) return false; if (t > t0) t0 = t; }
            else               { if (t < t0) return false; if (t < t1) t1 = t; }
            t = q_pos / p_pos;
            if (p_pos < 0.0f) { if (t > t1) return false; if (t > t0) t0 = t; }
            else               { if (t < t0) return false; if (t < t1) t1 = t; }
        }
    }

    if (t0 > 0.0f) {
        memcpy(clipped, end, sizeof(float) * N_AXIS);
        clipped[X_AXIS] = start[0] + t0 * d[0];
        clipped[Y_AXIS] = start[1] + t0 * d[1];
        if (!zone_z_undefined(z))
            clipped[Z_AXIS] = start[2] + t0 * d[2];
        return true;
    }
    return false;
}

/* ── Permission helpers ───────────────────────────────────────────────────── */

static bool op_allowed_in_zone(const zone_t *z, ez_op_t op)
{
    switch (op) {
        case OP_GCODE:      return z->flags.allow_gcode;
        case OP_JOG:        return z->flags.allow_jog;
        case OP_TOOLCHANGE: return z->flags.allow_toolchange;
    }
    return true;
}

static ez_op_t current_op(void)
{
    return tc_macro_running ? OP_TOOLCHANGE : OP_GCODE;
}

/* ── Travel-limit hooks ───────────────────────────────────────────────────── */

static bool ez_check_travel(float *target, axes_signals_t axes, bool is_cartesian, work_envelope_t *envelope)
{
    if (config.global_enabled) {
        float *pos = plan_get_position();
        float x0 = pos ? pos[X_AXIS] : 0.0f;
        float y0 = pos ? pos[Y_AXIS] : 0.0f;
        float z0 = pos ? pos[Z_AXIS] : 0.0f;
        float xt = target[X_AXIS];
        float yt = target[Y_AXIS];
        float zt = target[Z_AXIS];
        ez_op_t op = current_op();

        for (uint8_t i = 0; i < config.count; i++) {
            zone_t *z = &config.zones[i];
            if (!z->flags.enabled)
                continue;
            if (op_allowed_in_zone(z, op))
                continue;

            if (point_deep_in_zone(z, xt, yt, zt)) {
                if (point_deep_in_zone(z, x0, y0, z0))
                    report_message("EZ: Inside exclusion zone — disable zone to move out", Message_Warning);
                else
                    report_message("EZ: Target inside exclusion zone", Message_Warning);
                return false;
            }

            float p0[3] = { x0, y0, z0 };
            float p1[3] = { xt, yt, zt };
            if (segment_intersects_zone(z, p0, p1)) {
                if (point_deep_in_zone(z, x0, y0, z0))
                    report_message("EZ: Inside exclusion zone — disable zone to move out", Message_Warning);
                else
                    report_message("EZ: Move crosses exclusion zone", Message_Warning);
                return false;
            }
        }
    }
    return prev_check_travel_limits
        ? prev_check_travel_limits(target, axes, is_cartesian, envelope)
        : true;
}

static void ez_apply_travel(float *target, float *position, work_envelope_t *envelope)
{
    if (config.global_enabled) {
        float x0 = position[X_AXIS];
        float y0 = position[Y_AXIS];
        float z0 = position[Z_AXIS];

        for (uint8_t i = 0; i < config.count; i++) {
            zone_t *z = &config.zones[i];
            if (!z->flags.enabled)
                continue;
            if (op_allowed_in_zone(z, OP_JOG))
                continue;

            if (point_deep_in_zone(z, x0, y0, z0)) {
                report_message("EZ: Inside exclusion zone — disable zone to jog out", Message_Warning);
                memcpy(target, position, sizeof(float) * N_AXIS);
                return;
            }

            float xt = target[X_AXIS];
            float yt = target[Y_AXIS];
            float zt = target[Z_AXIS];

            bool target_inside = point_deep_in_zone(z, xt, yt, zt);
            float p0[3] = { x0, y0, z0 };
            float p1[3] = { xt, yt, zt };
            bool intersects = segment_intersects_zone(z, p0, p1);

            if (target_inside || intersects) {
                float clipped[N_AXIS];
                if (clip_to_zone_boundary(z, position, target, clipped)) {
                    report_message("EZ: Jog clipped at exclusion boundary", Message_Warning);
                    memcpy(target, clipped, sizeof(float) * N_AXIS);
                } else {
                    report_message("EZ: Jog blocked at exclusion boundary", Message_Warning);
                    memcpy(target, position, sizeof(float) * N_AXIS);
                    return;
                }
            }
        }
    }

    if (prev_apply_travel_limits)
        prev_apply_travel_limits(target, position, envelope);
}

/* ── Tool-change callbacks ────────────────────────────────────────────────── */

static void ez_tool_selected(tool_data_t *tool)
{
    tc_macro_running = true;
    if (prev_on_tool_selected) prev_on_tool_selected(tool);
}

static void ez_tool_changed(tool_data_t *tool)
{
    tc_macro_running = false;
    if (prev_on_tool_changed) prev_on_tool_changed(tool);
}

/* ── $ command handler ───────────────────────────────────────────────────── */

static status_code_t ez_sys_command(sys_state_t state, char *line)
{
    char buf[160];

    if (strncmp(line, "EXCLUSION=", 10) == 0) {
        config.global_enabled = (line[10] != '0');
        ez_save();
        snprintf(buf, sizeof(buf), "[EZ:%s]", config.global_enabled ? "enabled" : "disabled");
        hal.stream.write(buf);
        hal.stream.write(ASCII_EOL);
        return Status_OK;
    }

    if (strncmp(line, "ZONE-", 5) == 0) {
        uint8_t slot = (uint8_t)atoi(line + 5);
        if (slot == 0) {
            report_message("EZ: Zone 0 is reserved for ATCi", Message_Warning);
            return Status_InvalidStatement;
        }
        if (slot >= config.count)
            return Status_InvalidStatement;
        memmove(&config.zones[slot],
                &config.zones[slot + 1],
                sizeof(zone_t) * (config.count - slot - 1));
        config.count--;
        memset(&config.zones[config.count], 0, sizeof(zone_t));
        ez_save();
        snprintf(buf, sizeof(buf), "[EZ:zone %d deleted, %d remaining]", slot, config.count);
        hal.stream.write(buf);
        hal.stream.write(ASCII_EOL);
        return Status_OK;
    }

    if (strncmp(line, "ZONE=", 5) == 0) {
        unsigned slot, flags = 0;
        float xmin, ymin, zmin, xmax, ymax, zmax;
        int parsed = sscanf(line + 5, "%u,%f,%f,%f,%f,%f,%f,%u",
                            &slot, &xmin, &ymin, &zmin, &xmax, &ymax, &zmax, &flags);
        if (parsed < 7)
            return Status_InvalidStatement;
        if (slot == 0) {
            report_message("EZ: Zone 0 is reserved for ATCi", Message_Warning);
            return Status_InvalidStatement;
        }
        if (slot >= EZ_MAX_ZONES)
            return Status_InvalidStatement;

        zone_t *z = &config.zones[slot];
        z->x_min = xmin; z->y_min = ymin; z->z_min = zmin;
        z->x_max = xmax; z->y_max = ymax; z->z_max = zmax;
        z->flags.value = (parsed < 8 || flags < 8) ? ((uint8_t)flags | 0x08) : (uint8_t)flags;
        zone_normalize(z);
        if (slot >= config.count)
            config.count = slot + 1;
        ez_save();
        snprintf(buf, sizeof(buf), "[EZ:zone %d set]", slot);
        hal.stream.write(buf);
        hal.stream.write(ASCII_EOL);
        return Status_OK;
    }

    if (strcmp(line, "ZONE") == 0) {
        sync_zone0_from_atci();

        uint8_t active = 0;
        for (uint8_t i = 0; i < config.count; i++) {
            if (config.zones[i].flags.enabled) active++;
        }
        snprintf(buf, sizeof(buf), "[EZ:%s,%d active]",
                 config.global_enabled ? "enabled" : "disabled", active);
        hal.stream.write(buf);
        hal.stream.write(ASCII_EOL);
        for (uint8_t i = 0; i < config.count; i++) {
            zone_t *z = &config.zones[i];
            if (!z->flags.enabled) continue;
            snprintf(buf, sizeof(buf),
                     "[ZONE:%d|%.2f,%.2f,%.2f,%.2f,%.2f,%.2f|%u]",
                     i, z->x_min, z->y_min, z->z_min,
                     z->x_max, z->y_max, z->z_max,
                     (unsigned)z->flags.value);
            hal.stream.write(buf);
            hal.stream.write(ASCII_EOL);
        }
        return Status_OK;
    }

    return prev_on_unknown_sys_command
        ? prev_on_unknown_sys_command(state, line)
        : Status_Unhandled;
}

/* ── Reporting ────────────────────────────────────────────────────────────── */

static void ez_report_options(bool newopt)
{
    if (on_report_options) on_report_options(newopt);
    if (!newopt)
        report_plugin("Exclusion Zones", "1.1.0");
}

static void ez_realtime_report(stream_write_ptr stream_write, report_tracking_flags_t report)
{
    if (config.global_enabled) {
        char buf[16] = "|EZ:";
        char *p = buf + 4;

        *p++ = 'E';

        float *pos = plan_get_position();
        if (pos) {
            for (uint8_t i = 0; i < config.count; i++) {
                if (config.zones[i].flags.enabled &&
                    point_in_zone(&config.zones[i], pos[X_AXIS], pos[Y_AXIS], pos[Z_AXIS])) {
                    *p++ = 'Z';
                    break;
                }
            }
        }
        *p = '\0';
        stream_write(buf);
    }

    if (on_realtime_report)
        on_realtime_report(stream_write, report);
}

/* ── NVS persistence ──────────────────────────────────────────────────────── */

static void ez_save(void)
{
    hal.nvs.memcpy_to_nvs(nvs_addr, (uint8_t *)&config, sizeof(config), true);
}

static void ez_restore(void)
{
    memset(&config, 0, sizeof(config));
    config.global_enabled = true;
    ez_save();
}

static void ez_load(void)
{
    if (hal.nvs.memcpy_from_nvs((uint8_t *)&config, nvs_addr, sizeof(config), true) != NVS_TransferResult_OK)
        ez_restore();

    for (uint8_t i = 0; i < config.count; i++)
        zone_normalize(&config.zones[i]);

    task_add_delayed(sync_zone0_delayed, NULL, 5000);

    if (prev_check_travel_limits == NULL) {
        prev_check_travel_limits = grbl.check_travel_limits;
        grbl.check_travel_limits = ez_check_travel;

        prev_apply_travel_limits = grbl.apply_travel_limits;
        grbl.apply_travel_limits = ez_apply_travel;

        prev_on_unknown_sys_command = grbl.on_unknown_sys_command;
        grbl.on_unknown_sys_command = ez_sys_command;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = ez_report_options;

        on_realtime_report = grbl.on_realtime_report;
        grbl.on_realtime_report = ez_realtime_report;

        prev_on_tool_selected = grbl.on_tool_selected;
        grbl.on_tool_selected = ez_tool_selected;

        prev_on_tool_changed = grbl.on_tool_changed;
        grbl.on_tool_changed = ez_tool_changed;
    }
}

/* ── Public init ──────────────────────────────────────────────────────────── */

void exclusion_zones_init(void)
{
    static setting_details_t settings = {
        .load    = ez_load,
        .save    = ez_save,
        .restore = ez_restore
    };

    if ((nvs_addr = nvs_alloc(sizeof(config)))) {
        settings_register(&settings);
        report_message("Exclusion Zones v1.1.0 initialized", Message_Info);
    }
}

#endif /* EXCLUSION_ZONES_ENABLE */