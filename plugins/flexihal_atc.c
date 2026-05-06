/*
  flexihal_atc.c - Tool change routine to support ATC spindles

  Part of grblHAL

  Vendored from Expatria-Technologies/flexihal_atc (dev branch).

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
*/

#if ATC_ENABLE == 2

#include <string.h>
#include <stdio.h>

#include "grbl/hal.h"
#include "grbl/motion_control.h"
#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"
#include "grbl/nuts_bolts.h"
#include "grbl/state_machine.h"
#if TOOLTABLE_ENABLE == 2
#include "grbl/ngc_params.h"
#include "tooltable.h"
#endif

#define FLEXIHAL_DEBUG 1
#define RELAY_DEBOUNCE 50

static uint8_t n_input_ports;
static uint8_t n_output_ports;

#if FLEXIHAL_DEBUG
#define FLEXIHAL_DEBUG_PRINT(message) \
    hal.stream.write("[ATC]: "); \
    hal.stream.write(message); \
    hal.stream.write(ASCII_EOL)
#else
#define FLEXIHAL_DEBUG_PRINT(...)
#endif

typedef struct {
    uint8_t userinput;
    uint8_t tool_present;
    uint8_t drawbar_status;
    uint8_t drawbar_control;
    uint8_t air_seal;
    uint8_t taper_clear;
    uint8_t tlo_clear;
} atc_ports_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t
        drawbar_control     :1,
        airseal_control     :1,
        taperclear_control  :1,
        tloclear_control    :1,
        drawbar_status      :1,
        toolpresent_status  :1,
        userinput_status    :1,
        reserved            :1;
    };
} atc_status_flags_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t
        user_input_active       :1,
        tool_present_active     :1,
        drawbar_status_active   :1,
        drawbar_control_active  :1,
        air_seal_active         :1,
        taper_clear_active      :1,
        tlo_clear_active        :1,
        reserved                :2;
    };
} atc_settings_flags_t;

typedef struct {
    uint16_t drawbar_delay;
    atc_ports_t ports;
    atc_settings_flags_t flags;
#if TOOLTABLE_ENABLE == 2
    int16_t number_of_pockets;
#endif
} atc_settings_t;

static nvs_address_t nvs_address;
static atc_settings_t atc;
static atc_status_flags_t atc_status;

static on_spindle_select_ptr on_spindle_select;
static on_probe_toolsetter_ptr on_probe_fixture;
static probe_configure_ptr on_probe_configure = NULL;
static spindle_set_state_ptr on_spindle_set_state = NULL;
static driver_reset_ptr driver_reset = NULL;
static on_report_options_ptr on_report_options;
static on_macro_return_ptr on_macro_return = NULL;

static bool is_away = false;

static uint8_t n_in_ports;
static uint8_t n_out_ports;
static char max_in_port[4]  = "0";
static char max_out_port[4] = "0";

static atc_ports_t active_ports;

extern void atci_set_keepout_enabled(bool enabled);

static void read_atc_ports(void);

static const setting_group_detail_t atc_groups[] = {
    { Group_Root, Group_UserSettings, "FlexiHAL ATC" }
};

#define ATC_MACRO_ID_TCFETCH      390
#define ATC_MACRO_ID_TCRETURN     391
#define ATC_MACRO_ID_HANDFETCH    392
#define ATC_MACRO_ID_HANDRETURN   393
#define ATC_MACRO_ID_MEASURE      394

/* ── DRAWBAR ─────────────────────────────────────────────────────────── */

status_code_t drawbar_open (sys_state_t state, char *args)
{
    spindle_ptrs_t *spindle;
    spindle_state_t spindle_state = {0};
    spindle_data_t spinddata;
    spindle_data_t *spindledata = &spinddata;

    report_message("ATC plugin: Drawbar Open", Message_Info);

    spindle = spindle_get(0);
    spindledata->rpm = 0.0f;

    if(spindle->get_data)
        spindledata = spindle->get_data(SpindleData_RPM);
    if(spindle->get_state)
        spindle_state = spindle->get_state(spindle);

    if(spindle_state.on || (spindledata->rpm > 0.0f)) {
        report_message("Drawbar cannot open while spindle is running", Message_Warning);
        return Status_OK;
    }

    spindle_state.value = 0;
    spindle_set_state(spindle, spindle_state, 0);
    hal.delay_ms(RELAY_DEBOUNCE, NULL);

    switch(state_get()) {
        case STATE_IDLE: case STATE_TOOL_CHANGE: case STATE_HOLD: break;
        default:
            report_message("Drawbar can only open in IDLE or TOOL state", Message_Warning);
            return Status_OK;
    }

    if(atc.flags.drawbar_control_active)
        hal.port.digital_out(active_ports.drawbar_control, 1);
    atc_status.drawbar_control = 1;

    if(atc.flags.taper_clear_active)
        hal.port.digital_out(active_ports.taper_clear, 1);
    atc_status.taperclear_control = 1;

    hal.delay_ms(atc.drawbar_delay, NULL);
    read_atc_ports();

    return Status_OK;
}

status_code_t drawbar_close (sys_state_t state, char *args)
{
    spindle_ptrs_t *spindle;
    spindle_state_t spindle_state = {0};
    spindle_data_t spinddata;
    spindle_data_t *spindledata = &spinddata;

    spindle = spindle_get(0);
    spindledata->rpm = 0.0f;

    if(spindle->get_state)
        spindle_state = spindle->get_state(spindle);
    if(spindle->get_data)
        spindledata = spindle->get_data(SpindleData_RPM);

    if(spindle_state.on || (spindledata->rpm > 0.0f))
        return Status_OK;

    switch(state_get()) {
        case STATE_IDLE: case STATE_TOOL_CHANGE: case STATE_HOLD: break;
        default: return Status_OK;
    }

    report_message("ATC plugin: Drawbar Close", Message_Info);

    if(atc.flags.drawbar_control_active)
        hal.port.digital_out(active_ports.drawbar_control, 0);
    atc_status.drawbar_control = 0;

    if(atc.flags.taper_clear_active)
        hal.port.digital_out(active_ports.taper_clear, 0);
    atc_status.taperclear_control = 0;

    hal.delay_ms(atc.drawbar_delay, NULL);
    read_atc_ports();

    return Status_OK;
}

/* ── CAROUSEL ADMIN ───────────────────────────────────────────────────── */

#if TOOLTABLE_ENABLE == 2

static void macro_exit (void)
{
    atci_set_keepout_enabled(true);
    grbl.on_macro_return = on_macro_return;
    on_macro_return = NULL;
    if(grbl.on_macro_return)
        grbl.on_macro_return();
}

static status_code_t carousel_add (sys_state_t state, char *args)
{
    if(state_get() != STATE_IDLE) {
        report_message("TCADD: machine must be IDLE", Message_Warning);
        return Status_InvalidStatement;
    }
    if((sys.homed.mask & (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT)) != (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT)) {
        report_message("TCADD: machine must be homed", Message_Warning);
        return Status_HomingRequired;
    }

    uint32_t tool_id;
    const char *name = NULL;

    if(!args || !*args) {
        tool_id = (uint32_t)gc_state.tool->tool_id;
        if(tool_id == 0) {
            report_message("TCADD: no tool selected and no argument given", Message_Warning);
            return Status_BadNumberFormat;
        }
    } else {
        if(*args != 'T' && *args != 't') {
            report_message("TCADD: usage is $TCADD [Tn] [;name]", Message_Warning);
            return Status_BadNumberFormat;
        }
        uint_fast8_t cc = 1;
        status_code_t parse_status = read_uint(args, &cc, &tool_id);
        if(parse_status != Status_OK) return parse_status;
        while(args[cc] == ' ' || args[cc] == '\t') cc++;
        if(args[cc] == ';') name = &args[cc + 1];
    }

    if(atc.flags.tool_present_active) {
        read_atc_ports();
        if(!atc_status.toolpresent_status) {
            report_message("TCADD: no tool detected in spindle", Message_Warning);
            return Status_GcodeValueOutOfRange;
        }
    }

    spindle_ptrs_t *spindle = spindle_get(0);
    if(spindle && spindle->get_state && spindle->get_state(spindle).on) {
        report_message("TCADD: spindle must be off", Message_Warning);
        return Status_GcodeValueOutOfRange;
    }

    pocket_id_t assigned_pocket = 0;
    carousel_op_result_t result = tooltable_carousel_add((tool_id_t)tool_id, atc.number_of_pockets, name, &assigned_pocket);

    switch(result) {
        case CarouselOp_OK: break;
        case CarouselOp_ToolAlreadyInPocket:
            report_message("TCADD: tool already has a pocket assigned", Message_Warning);
            return Status_GcodeValueOutOfRange;
        case CarouselOp_NoPocketAvailable: {
            char msg[60];
            sprintf(msg, "TCADD: carousel is full (%u pockets configured)", (unsigned)atc.number_of_pockets);
            report_message(msg, Message_Warning);
            return Status_GcodeValueOutOfRange;
        }
        case CarouselOp_TableNotLoaded:
            report_message("TCADD: tool table not loaded", Message_Warning);
            return Status_GcodeValueOutOfRange;
        case CarouselOp_WriteError:
            report_message("TCADD: failed to write tool table", Message_Warning);
            return Status_FileReadError;
        default:
            report_message("TCADD: unknown error", Message_Warning);
            return Status_GcodeValueOutOfRange;
    }

    ngc_param_set(4900, (float)tool_id);
    ngc_param_set(4901, (float)assigned_pocket);
    ngc_param_set(4905, 1.0f);
    ngc_param_set(4906, 0.0f);

    atci_set_keepout_enabled(false);
    on_macro_return = grbl.on_macro_return;
    grbl.on_macro_return = macro_exit;

    if(!grbl.enqueue_gcode("G65P391")) {
        tooltable_carousel_remove((tool_id_t)tool_id);
        report_message("TCADD: failed to enqueue deposit motion -- pocket assignment rolled back", Message_Warning);
        return Status_EStop;
    }
    return Status_OK;
}

static status_code_t carousel_return (sys_state_t state, char *args) { return carousel_add(state, NULL); }

static status_code_t carousel_remove (sys_state_t state, char *args)
{
    if(state_get() != STATE_IDLE) {
        report_message("TCRM: machine must be IDLE", Message_Warning);
        return Status_InvalidStatement;
    }
    uint32_t tool_id;
    if(!args || !*args) {
        tool_id = (uint32_t)gc_state.tool->tool_id;
        if(tool_id == 0) { report_message("TCRM: no tool selected", Message_Warning); return Status_BadNumberFormat; }
    } else {
        if(*args != 'T' && *args != 't') { report_message("TCRM: usage is $TCRM [Tn]", Message_Warning); return Status_BadNumberFormat; }
        uint_fast8_t cc = 1;
        status_code_t s = read_uint(args, &cc, &tool_id);
        if(s != Status_OK) return s;
    }
    carousel_op_result_t result = tooltable_carousel_remove((tool_id_t)tool_id);
    if(result == CarouselOp_OK) {
        char msg[40]; sprintf(msg, "Tool %lu removed from carousel", (unsigned long)tool_id);
        report_message(msg, Message_Info);
        return Status_OK;
    }
    report_message("TCRM: tool not found in carousel", Message_Warning);
    return Status_GcodeValueOutOfRange;
}

static status_code_t carousel_measure (sys_state_t state, char *args)
{
    if(state_get() != STATE_IDLE) { report_message("TCMEASURE: machine must be IDLE", Message_Warning); return Status_InvalidStatement; }
    if((sys.homed.mask & (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT)) != (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT)) { report_message("TCMEASURE: machine must be homed", Message_Warning); return Status_HomingRequired; }
    if(gc_state.tool->tool_id == 0) { report_message("TCMEASURE: no tool in spindle", Message_Warning); return Status_GcodeValueOutOfRange; }
    atci_set_keepout_enabled(false);
    on_macro_return = grbl.on_macro_return;
    grbl.on_macro_return = macro_exit;
    if(!grbl.enqueue_gcode("G65P394")) { report_message("TCMEASURE: failed to enqueue measure macro", Message_Warning); return Status_EStop; }
    return Status_OK;
}

static status_code_t carousel_remeasure (sys_state_t state, char *args)
{
    if(state_get() != STATE_IDLE) { report_message("TCREMEASURE: machine must be IDLE", Message_Warning); return Status_InvalidStatement; }
    if((sys.homed.mask & (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT)) != (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT)) { report_message("TCREMEASURE: machine must be homed", Message_Warning); return Status_HomingRequired; }
    if(gc_state.tool->tool_id == 0) { report_message("TCREMEASURE: no tool in spindle", Message_Warning); return Status_GcodeValueOutOfRange; }
    tool_data_t tool = *gc_state.tool;
    memset(&tool.offset, 0, sizeof(tool.offset));
    grbl.tool_table.set_tool(&tool);
    atci_set_keepout_enabled(false);
    on_macro_return = grbl.on_macro_return;
    grbl.on_macro_return = macro_exit;
    if(!grbl.enqueue_gcode("G65P394")) { report_message("TCREMEASURE: failed", Message_Warning); return Status_EStop; }
    return Status_OK;
}

#endif /* TOOLTABLE_ENABLE == 2 */

/* ── COMMAND TABLE ────────────────────────────────────────────────────── */

const sys_command_t atc_command_list[] = {
    { "DRBO", drawbar_open,   { .noargs = On  }, { .str = "Open the drawbar" } },
    { "DRBC", drawbar_close,  { .noargs = On  }, { .str = "Close the drawbar" } },
#if TOOLTABLE_ENABLE == 2
    { "TCADD",       carousel_add,       { .noargs = Off }, { .str = "Deposit tool into carousel: $TCADD [Tn] [;name]" } },
    { "TCRETURN",    carousel_return,    { .noargs = On  }, { .str = "Return spindle tool to carousel pocket" } },
    { "TCRM",        carousel_remove,    { .noargs = Off }, { .str = "Clear tool carousel pocket: $TCRM [Tn]" } },
    { "TCMEASURE",   carousel_measure,   { .noargs = On  }, { .str = "Measure current tool length" } },
    { "TCREMEASURE", carousel_remeasure, { .noargs = On  }, { .str = "Clear offset and re-measure tool" } },
#endif
};

static sys_commands_t atc_commands = {
    .n_commands = sizeof(atc_command_list) / sizeof(sys_command_t),
    .commands   = atc_command_list
};

sys_commands_t *atc_get_commands (void) { return &atc_commands; }

/* ── POLLING ─────────────────────────────────────────────────────────── */

static void atc_poll (void *data)
{
    #define DEBOUNCE_THRESHOLD 3
    #define ZERO_THRESHOLD     10
    static uint8_t val = 0, prev_val = 99, latch = 0;
    static int zero_count = 0, one_count = 0;

    read_atc_ports();
    prev_val = val;
    val = atc_status.userinput_status;

    if(val == 0) { zero_count++; one_count = 0; }
    else { one_count++; zero_count = 0; }

    if((prev_val == 0) && (val == 0) && (latch == 0)) {
        if(zero_count >= DEBOUNCE_THRESHOLD) { latch = 1; drawbar_open(state_get(), NULL); zero_count = 0; }
    } else if(((prev_val == 1) && (val == 1) && (latch == 1)) || (zero_count >= ZERO_THRESHOLD)) {
        if(one_count >= 1 || zero_count >= ZERO_THRESHOLD) { latch = 0; drawbar_close(state_get(), NULL); one_count = 0; zero_count = 0; }
    } else { zero_count = 0; one_count = 0; }

    task_delete(atc_poll, NULL);
    task_add_delayed(atc_poll, NULL, 100);
}

/* ── PORT I/O ────────────────────────────────────────────────────────── */

static void read_atc_ports (void)
{
    uint8_t val;
    if(atc.flags.drawbar_status_active) {
        val = hal.port.wait_on_input(Port_Digital, active_ports.drawbar_status, WaitMode_Immediate, 0.0f);
        atc_status.drawbar_status = (val == 1);
    }
    if(atc.flags.tool_present_active) {
        val = hal.port.wait_on_input(Port_Digital, active_ports.tool_present, WaitMode_Immediate, 0.0f);
        atc_status.toolpresent_status = (val == 1);
    }
    if(atc.flags.user_input_active) {
        val = hal.port.wait_on_input(Port_Digital, active_ports.userinput, WaitMode_Immediate, 0.0f);
        atc_status.userinput_status = (val == 1);
    }
}

/* ── SPINDLE HOOKS ───────────────────────────────────────────────────── */

static void onSpindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    read_atc_ports();
    if(state.value != 0) atc_status.airseal_control = 1;
    else atc_status.airseal_control = 0;
    if(atc.flags.air_seal_active)
        hal.port.digital_out(active_ports.air_seal, atc_status.airseal_control);
    on_spindle_set_state(spindle, state, rpm);
}

static bool onSpindleSelect (spindle_ptrs_t *spindle)
{
    on_spindle_set_state = spindle->set_state;
    spindle->set_state   = onSpindleSetState;
    return on_spindle_select == NULL || on_spindle_select(spindle);
}

static bool probe_fixture (tool_data_t *tool, coord_data_t *position, bool at_g59_3, bool on)
{
    bool status = true;
    if(at_g59_3 && on) {
        report_message("ATC tool probe", Message_Info);
        is_away = false;
        if(atc.flags.tlo_clear_active && !is_away) {
            hal.port.digital_out(active_ports.tlo_clear, 1);
            hal.delay_ms(atc.drawbar_delay, NULL);
            hal.port.digital_out(active_ports.tlo_clear, 0);
        }
    }
    if(on_probe_fixture)
        status = on_probe_fixture(tool, position, at_g59_3, on);
    return status;
}

static void probeConfigure (bool is_probe_away, bool probing)
{
    if(on_probe_configure) on_probe_configure(is_probe_away, probing);
    if(probing) is_away = false;
    is_away = is_probe_away;
}

static tool_change_ptr on_tool_change = NULL;

static status_code_t atc_tool_change (parser_state_t *gc_state)
{
    coolant_state_t mode = {0};
    spindle_all_off(false);
    coolant_set_state(mode);
    spindle_ptrs_t *spindle = spindle_get(0);
    if(spindle && spindle->get_data) {
        uint32_t ms = hal.get_elapsed_ticks();
        while(spindle->get_data(SpindleData_RPM)->rpm > 0.0f) {
            if(hal.get_elapsed_ticks() - ms > 20000) {
                report_message("ATC: spindle did not stop -- tool change aborted", Message_Warning);
                return Status_EStop;
            }
            hal.delay_ms(100, NULL);
        }
    }
    return on_tool_change ? on_tool_change(gc_state) : Status_Unhandled;
}

/* ── SETTINGS ─────────────────────────────────────────────────────────── */

static const setting_detail_t atc_settings[] = {
    { 953, Group_Toolchange, "ATC Drawbar Delay", "milliseconds", Format_Int16, "##0", NULL, NULL, Setting_NonCore, &atc.drawbar_delay, NULL, NULL },
    { 954, Group_Toolchange, "ATC User Input Port", NULL, Format_Int8, "#0", "0", max_in_port, Setting_NonCore, &atc.ports.userinput, NULL, NULL, { .reboot_required = On } },
    { 955, Group_Toolchange, "ATC Tool Present Port", NULL, Format_Int8, "#0", "0", max_in_port, Setting_NonCore, &atc.ports.tool_present, NULL, NULL, { .reboot_required = On } },
    { 956, Group_Toolchange, "ATC Drawbar Status Port", NULL, Format_Int8, "#0", "0", max_in_port, Setting_NonCore, &atc.ports.drawbar_status, NULL, NULL, { .reboot_required = On } },
    { 957, Group_Toolchange, "ATC Drawbar Control Port", NULL, Format_Int8, "#0", "0", max_out_port, Setting_NonCore, &atc.ports.drawbar_control, NULL, NULL, { .reboot_required = On } },
    { 958, Group_Toolchange, "ATC Air Seal Port", NULL, Format_Int8, "#0", "0", max_out_port, Setting_NonCore, &atc.ports.air_seal, NULL, NULL, { .reboot_required = On } },
    { 959, Group_Toolchange, "ATC Taper Clear Port", NULL, Format_Int8, "#0", "0", max_out_port, Setting_NonCore, &atc.ports.taper_clear, NULL, NULL, { .reboot_required = On } },
    { 960, Group_Toolchange, "ATC TLO Clear Port", NULL, Format_Int8, "#0", "0", max_out_port, Setting_NonCore, &atc.ports.tlo_clear, NULL, NULL, { .reboot_required = On } },
    { 961, Group_Toolchange, "ATC Flags", NULL, Format_Bitfield, "User Input Enabled, Tool Detect Enabled, Drawbar Status Enabled, Drawbar Control Enabled, Air Seal Control Enabled, Taper Clear Enabled, Toolsetter Clear Enabled", NULL, NULL, Setting_NonCore, &atc.flags, NULL, NULL },
#if TOOLTABLE_ENABLE == 2
    { 962, Group_Toolchange, "ATC Number of Pockets", "pockets", Format_Int16, "##0", "1", "9999", Setting_NonCore, &atc.number_of_pockets, NULL, NULL },
#endif
};

#ifndef NO_SETTINGS_DESCRIPTIONS
static const setting_descr_t atc_descriptions[] = {
    { 953, "Delay between operating the drawbar and reading the sensors." },
    { 954, "Aux input port for drawbar user input" },
    { 955, "Aux input port for tool detection" },
    { 956, "Aux input port for drawbar status" },
    { 957, "Aux output port for drawbar control" },
    { 958, "Aux output port for air seal control" },
    { 959, "Aux output port for taper clear control" },
    { 960, "Aux output port for toolsetter clearing" },
    { 961, "Aux input for ATC button is enabled.\\n"
           "Aux input for tool clamp sensor is enabled.\\n"
           "Aux input for drawbar status is enabled.\\n\\n"
           "Aux output for drawbar control is enabled.\\n"
           "Aux output for air seal is enabled.\\n"
           "Aux output for taper clear is enabled.\\n"
           "Aux output for toolsetter clear is enabled.\\n" },
#if TOOLTABLE_ENABLE == 2
    { 962, "Number of physical pockets in the carousel." },
#endif
};
#endif

static void warning_no_port (void *data)
{
    report_message("ATC plugin: configured port number is not available", Message_Warning);
}

static void atc_settings_restore (void)
{
    memset(&atc, 0, sizeof(atc_settings_t));
#if TOOLTABLE_ENABLE == 2
    atc.number_of_pockets = 12;
#endif
    atc.ports.userinput      = hal.port.num_digital_in  ? hal.port.num_digital_in  - 1 : 0;
    atc.ports.tool_present   = hal.port.num_digital_in  ? hal.port.num_digital_in  - 1 : 0;
    atc.ports.drawbar_status = hal.port.num_digital_in  ? hal.port.num_digital_in  - 1 : 0;
    atc.ports.drawbar_control = hal.port.num_digital_out ? hal.port.num_digital_out - 1 : 0;
    atc.ports.air_seal        = hal.port.num_digital_out ? hal.port.num_digital_out - 1 : 0;
    atc.ports.taper_clear     = hal.port.num_digital_out ? hal.port.num_digital_out - 1 : 0;
    atc.ports.tlo_clear       = hal.port.num_digital_out ? hal.port.num_digital_out - 1 : 0;
    atc.drawbar_delay = 352;
    atc.flags.value   = 0;
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&atc, sizeof(atc_settings_t), true);
}

static void atc_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&atc, sizeof(atc_settings_t), true);
}

static void atc_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&atc, nvs_address, sizeof(atc_settings_t), true) != NVS_TransferResult_OK)
        atc_settings_restore();

    active_ports.tool_present    = atc.ports.tool_present;
    active_ports.drawbar_status  = atc.ports.drawbar_status;
    active_ports.userinput       = atc.ports.userinput;
    active_ports.drawbar_control = atc.ports.drawbar_control;
    active_ports.taper_clear     = atc.ports.taper_clear;
    active_ports.air_seal        = atc.ports.air_seal;
    active_ports.tlo_clear       = atc.ports.tlo_clear;

    if(atc.flags.user_input_active)
        if(!ioport_claim(Port_Digital, Port_Input, &active_ports.userinput, "ATC User Input"))
            task_add_immediate(warning_no_port, NULL);
    if(atc.flags.tool_present_active)
        if(!ioport_claim(Port_Digital, Port_Input, &active_ports.tool_present, "Tool Present"))
            task_add_immediate(warning_no_port, NULL);
    if(atc.flags.drawbar_status_active)
        if(!ioport_claim(Port_Digital, Port_Input, &active_ports.drawbar_status, "Drawbar Open/Closed"))
            task_add_immediate(warning_no_port, NULL);
    if(atc.flags.drawbar_control_active)
        if(!ioport_claim(Port_Digital, Port_Output, &active_ports.drawbar_control, "Drawbar Control"))
            task_add_immediate(warning_no_port, NULL);
    if(atc.flags.taper_clear_active)
        if(!ioport_claim(Port_Digital, Port_Output, &active_ports.taper_clear, "Taper Clear"))
            task_add_immediate(warning_no_port, NULL);
    if(atc.flags.air_seal_active)
        if(!ioport_claim(Port_Digital, Port_Output, &active_ports.air_seal, "Air Seal"))
            task_add_immediate(warning_no_port, NULL);
    if(atc.flags.tlo_clear_active)
        if(!ioport_claim(Port_Digital, Port_Output, &active_ports.tlo_clear, "Toolsetter Clear"))
            task_add_immediate(warning_no_port, NULL);

#if TOOLTABLE_ENABLE == 2
    tooltable_set_max_pockets(atc.number_of_pockets);
#endif
}

static setting_details_t setting_details = {
    .groups       = atc_groups,
    .n_groups     = sizeof(atc_groups) / sizeof(setting_group_detail_t),
    .settings     = atc_settings,
    .n_settings   = sizeof(atc_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions   = atc_descriptions,
    .n_descriptions = sizeof(atc_descriptions) / sizeof(setting_descr_t),
#endif
    .save    = atc_settings_save,
    .load    = atc_settings_load,
    .restore = atc_settings_restore
};

/* ── INIT ─────────────────────────────────────────────────────────────── */

static void report_options (bool newopt)
{
    on_report_options(newopt);
    if(!newopt)
        hal.stream.write("[PLUGIN: FlexiHAL ATC v0.03]" ASCII_EOL);
}

static void atc_reset (void) { driver_reset(); }

void atc_init (void)
{
    protocol_enqueue_foreground_task(report_info, "FlexiHAL ATC plugin trying to initialize!");

    bool ok = (n_input_ports  = ioports_available(Port_Digital, Port_Input));
    ok      = (n_output_ports = ioports_available(Port_Digital, Port_Output));

    if(!ioport_can_claim_explicit()) {
        protocol_enqueue_foreground_task(report_warning, "FlexiHAL: Failed to initialize, unable to claim required ioports!");
        return;
    } else {
        if((ok = (n_in_ports  = ioports_available(Port_Digital, Port_Input))  >= 1))
            strcpy(max_in_port,  uitoa(n_in_ports  - 1));
        if((ok = ok && (n_out_ports = ioports_available(Port_Digital, Port_Output)) >= 1))
            strcpy(max_out_port, uitoa(n_out_ports - 1));
    }

    if(!ok) {
        protocol_enqueue_foreground_task(report_warning, "FlexiHAL: Failed to initialize, unable to claim required ioports!");
        return;
    }

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = report_options;

    on_spindle_select = grbl.on_spindle_select;
    grbl.on_spindle_select = onSpindleSelect;

    on_probe_configure = hal.probe.configure;
    hal.probe.configure = probeConfigure;

    on_probe_fixture = grbl.on_probe_toolsetter;
    grbl.on_probe_toolsetter = probe_fixture;

    driver_reset = hal.driver_reset;
    hal.driver_reset = atc_reset;

    on_tool_change   = hal.tool.change;
    hal.tool.change  = atc_tool_change;

    system_register_commands(&atc_commands);
    task_add_delayed(atc_poll, NULL, 1000);

    if((nvs_address = nvs_alloc(sizeof(atc_settings_t)))) {
        settings_register(&setting_details);
    } else {
        protocol_enqueue_foreground_task(report_warning, "FlexiHAL: Failed to initialize, no NVS storage for settings!");
    }
}

#endif /* ATC_ENABLE == 2 */