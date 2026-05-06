/*
  grbl_interface.h - hooks to link sim to grblHAL app

  Part of grblHAL FlexiHAL Simulator
*/

#ifndef _GRBL_INTERFACE_H_
#define _GRBL_INTERFACE_H_

#define SIM_STDIN_REPORT_STATE 0x12
#define SIM_STDIN_FORCE_IDLE 'i'

void grbl_app_init(void);
void grbl_per_tick(void);
void grbl_per_byte(void);
void grbl_app_exit(void);

#endif
