/*
  driver.h - FlexiHAL simulator driver definitions

  Part of grblHAL FlexiHAL Simulator
*/

#ifndef _DRIVER_H_
#define _DRIVER_H_

/* Provide stub definitions for hardware macros that plugins may reference */
#include "sim_stubs.h"

/* Pull in the full grblHAL API — plugins expect driver.h to provide this */
#include "grbl/hal.h"

/* Simulated SD card stubs (provides sdcard_early_mount inline no-op) */
#if SDCARD_ENABLE
#include "fs_sim.h"
#endif

#define portINT(p) portQ(p)
#define portQ(p) GPIO ## p ## _IRQ

#define STEPPER_TIMER 0

// GPIO port assignments for virtual MCU
#define LIMITS_PORT0 0
#define LIMITS_IRQ0  portINT(LIMITS_PORT0)
#define LIMITS_PORT1 1
#define LIMITS_IRQ1  portINT(LIMITS_PORT1)

#define CONTROL_PORT 2
#define CONTROL_IRQ  portINT(CONTROL_PORT)

#define SPINDLE_PORT 3
#define COOLANT_PORT 4
#define PROBE_PORT   5

#define STEP_PORT0   6
#define STEP_PORT1   7
#define DIR_PORT     8
#define STEPPER_ENABLE_PORT 9

// FlexiHAL supports up to 5 axes (X, Y, Z + 2 ABC motors)
#define SPINDLE_MASK 0x07  // enable + direction + PWM
#define COOLANT_MASK 0x03  // flood + mist

// Control signal bits — must match control_signals_t bit layout in nuts_bolts.h
#define RESET_PIN       0
#define FEED_HOLD_PIN   1
#define CYCLE_START_PIN 2
#define SAFETY_DOOR_PIN 3
#define ESTOP_PIN       6
#define PROBE_DISCONNECTED_PIN 7
#define RESET_BIT       (1 << RESET_PIN)
#define FEED_HOLD_BIT   (1 << FEED_HOLD_PIN)
#define CYCLE_START_BIT (1 << CYCLE_START_PIN)
#define SAFETY_DOOR_BIT (1 << SAFETY_DOOR_PIN)
#define ESTOP_BIT       (1 << ESTOP_PIN)
#define PROBE_DISCONNECTED_BIT (1 << PROBE_DISCONNECTED_PIN)
#define CONTROL_MASK    (RESET_BIT | FEED_HOLD_BIT | CYCLE_START_BIT | SAFETY_DOOR_BIT | ESTOP_BIT | PROBE_DISCONNECTED_BIT)

// Probe
#define PROBE_PIN           0
#define PROBE_CONNECTED_PIN 1
#define PROBE_BIT           (1 << PROBE_PIN)
#define PROBE_CONNECTED_BIT (1 << PROBE_CONNECTED_PIN)
#define PROBE_MASK          (PROBE_BIT | PROBE_CONNECTED_BIT)

#endif
