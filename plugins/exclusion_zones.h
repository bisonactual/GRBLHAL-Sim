/*
  exclusion_zones.h — Public header for the grblHAL exclusion zones plugin.

  Multi-zone 3D exclusion/inclusion system with per-operation permission flags.
  Compatible with the Sienci ATCi keepout plugin hook architecture.

  Zone 0 is reserved for the ATCi plugin — it mirrors settings $684-$687
  and blocks gcode/jog at all depths, allowing only toolchange.

  If a zone has z_min == 0 and z_max == 0, the Z axis is ignored and the
  zone blocks at all depths.

  Commands:
    $EXCLUSION=1 / $EXCLUSION=0                              — global enable/disable
    $ZONE                                                    — list active zones
    $ZONE=n,xmin,ymin,zmin,xmax,ymax,zmax,flags             — set/update zone (n >= 1)
    $ZONE-n                                                  — delete zone n (n >= 1)
*/

#ifndef _EXCLUSION_ZONES_H_
#define _EXCLUSION_ZONES_H_

/* Maximum number of zones — override at compile time if needed */
#ifndef EZ_MAX_ZONES
#define EZ_MAX_ZONES 16
#endif

/* Tolerance buffer so boundary-touching moves aren't falsely blocked */
#ifndef EZ_TOLERANCE
#define EZ_TOLERANCE 0.5f
#endif

/* ── Data types ───────────────────────────────────────────────────────────── */

/* Per-zone permission flags — if bit is SET, that operation is ALLOWED inside the zone */
typedef union {
    uint8_t value;
    struct {
        uint8_t allow_gcode      :1,
                allow_jog        :1,
                allow_toolchange :1,
                enabled          :1,
                unused           :4;
    };
} zone_flags_t;

typedef struct {
    float        x_min, y_min, z_min;
    float        x_max, y_max, z_max;
    zone_flags_t flags;
} zone_t;

typedef struct {
    uint8_t count;
    bool    global_enabled;
    zone_t  zones[EZ_MAX_ZONES];
} ez_config_t;

typedef enum {
    OP_GCODE,
    OP_JOG,
    OP_TOOLCHANGE
} ez_op_t;

/* ── Public API ───────────────────────────────────────────────────────────── */

/*
   Call once from the driver's plugin init to register the exclusion zones
   plugin with grblHAL (hooks travel limits, $ commands, reporting, NVS).
*/
void exclusion_zones_init(void);

#endif /* _EXCLUSION_ZONES_H_ */
