/*
  fs_sim.h - Simulated filesystem backed by a host directory

  Part of grblHAL FlexiHAL Simulator
*/

#pragma once

/* Initialise the simulated SD card.
   Creates the storage directory if it doesn't exist, registers the VFS
   backend, mounts it at "/", and calls fs_stream_init().
   Call once from driver_init(). */
void fs_sim_init (void);

/* Stub for sdcard_early_mount() — called by tooltable_init() when
   SDCARD_ENABLE is set.  In the simulator the FS is already mounted
   by fs_sim_init() during driver_init(), so this is a no-op. */
static inline void sdcard_early_mount (void) {}
