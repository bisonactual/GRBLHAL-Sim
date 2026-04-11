/*
  sim_stubs.h - Stub definitions for hardware macros used by plugins

  Part of grblHAL FlexiHAL Simulator

  Plugins written for real boards reference GPIO ports, pins, and
  hardware read macros. In the simulator these don't exist, so we
  provide safe no-op stubs that let the code compile and run without
  touching real hardware.
*/

#ifndef _SIM_STUBS_H_
#define _SIM_STUBS_H_

#include <stdint.h>
#include <stdbool.h>

/* ── GPIO port/pin stubs ─────────────────────────────────────────────────
   Real board maps define these as pointers to hardware registers (e.g. GPIOA).
   We define them as integers so pin references compile but do nothing.
*/

#ifndef AUXINPUT0_PORT
#define AUXINPUT0_PORT  0
#define AUXINPUT0_PIN   0
#endif
#ifndef AUXINPUT1_PORT
#define AUXINPUT1_PORT  0
#define AUXINPUT1_PIN   1
#endif
#ifndef AUXINPUT2_PORT
#define AUXINPUT2_PORT  0
#define AUXINPUT2_PIN   2
#endif
#ifndef AUXINPUT3_PORT
#define AUXINPUT3_PORT  0
#define AUXINPUT3_PIN   3
#endif
#ifndef AUXINPUT4_PORT
#define AUXINPUT4_PORT  0
#define AUXINPUT4_PIN   4
#endif
#ifndef AUXINPUT5_PORT
#define AUXINPUT5_PORT  0
#define AUXINPUT5_PIN   5
#endif
#ifndef AUXINPUT6_PORT
#define AUXINPUT6_PORT  0
#define AUXINPUT6_PIN   6
#endif
#ifndef AUXINPUT7_PORT
#define AUXINPUT7_PORT  0
#define AUXINPUT7_PIN   7
#endif
#ifndef AUXINPUT8_PORT
#define AUXINPUT8_PORT  0
#define AUXINPUT8_PIN   8
#endif
#ifndef AUXINPUT9_PORT
#define AUXINPUT9_PORT  0
#define AUXINPUT9_PIN   9
#endif
#ifndef AUXINPUT10_PORT
#define AUXINPUT10_PORT 0
#define AUXINPUT10_PIN  10
#endif

#ifndef AUXOUTPUT0_PORT
#define AUXOUTPUT0_PORT 0
#define AUXOUTPUT0_PIN  0
#endif
#ifndef AUXOUTPUT1_PORT
#define AUXOUTPUT1_PORT 0
#define AUXOUTPUT1_PIN  1
#endif
#ifndef AUXOUTPUT2_PORT
#define AUXOUTPUT2_PORT 0
#define AUXOUTPUT2_PIN  2
#endif
#ifndef AUXOUTPUT3_PORT
#define AUXOUTPUT3_PORT 0
#define AUXOUTPUT3_PIN  3
#endif
#ifndef AUXOUTPUT4_PORT
#define AUXOUTPUT4_PORT 0
#define AUXOUTPUT4_PIN  4
#endif
#ifndef AUXOUTPUT5_PORT
#define AUXOUTPUT5_PORT 0
#define AUXOUTPUT5_PIN  5
#endif
#ifndef AUXOUTPUT6_PORT
#define AUXOUTPUT6_PORT 0
#define AUXOUTPUT6_PIN  6
#endif
#ifndef AUXOUTPUT7_PORT
#define AUXOUTPUT7_PORT 0
#define AUXOUTPUT7_PIN  7
#endif
#ifndef AUXOUTPUT8_PORT
#define AUXOUTPUT8_PORT 0
#define AUXOUTPUT8_PIN  8
#endif

/* ── Hardware read/write macros ──────────────────────────────────────────
   DIGITAL_IN reads a GPIO pin. In sim, always returns 0 (not asserted).
   DIGITAL_OUT writes a GPIO pin. In sim, does nothing.
*/
#ifndef DIGITAL_IN
#define DIGITAL_IN(port, pin) ((bool)0)
#endif

#ifndef DIGITAL_OUT
#define DIGITAL_OUT(port, pin, val) ((void)0)
#endif

#endif /* _SIM_STUBS_H_ */
