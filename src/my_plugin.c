/* Bundled FlexiHAL simulator plugin entry points. */
#define ADD_MY_PLUGIN
#include "grbl/hal.h"

extern void atci_init(void);
extern void exclusion_zones_init(void);
#if FLIGHT_RECORDER_ENABLE
extern void flight_recorder_init(void);
#endif

void my_plugin_init(void)
{
    atci_init();
    exclusion_zones_init();
#if FLIGHT_RECORDER_ENABLE
    flight_recorder_init();
#endif
}
