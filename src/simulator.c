/*
  simulator.c - simulation engine

  Part of grblHAL FlexiHAL Simulator

  Based on grblHAL Simulator by Terje Io, Jens Geisler, Adam Shelly
*/

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#include "simulator.h"
#include "eeprom.h"
#include "mcu.h"
#include "driver.h"

#include "grbl/grbl.h"

void sim_nop(void) {}

sim_vars_t sim = {
    .on_init = sim_nop,
    .on_tick = sim_nop,
    .on_byte = sim_nop,
    .on_shutdown = sim_nop
};

void init_simulator(void)
{
    sim.baud_ticks = F_CPU / 115200;
    sim.on_init();
}

void shutdown_simulator(void)
{
    sim.on_shutdown();
}

void simulate_hardware(bool do_serial)
{
    sim.masterclock++;
    sim.sim_time = (float)sim.masterclock / (float)F_CPU;
    mcu_master_clock();
    if (do_serial)
        simulate_serial();
}

void sim_loop(void)
{
    const uint32_t control_frame_ns = 100 * 1000 * 1000;

    int32_t sleep_time_us = 0;
    uint32_t ticks_per_frame = F_CPU / 100;
    uint64_t target_ticks = ticks_per_frame;
    uint32_t ns_prev = platform_ns();
    uint64_t next_byte_tick = F_CPU;

    while (sim.exit != exit_OK) {
        while (sim.masterclock < target_ticks) {
            bool read_serial = (sim.masterclock >= next_byte_tick);
            simulate_hardware(read_serial);
            sim.on_tick();

            if (read_serial) {
                next_byte_tick += sim.baud_ticks * 10;
                sim.on_byte();
            }

            if (sim.exit == exit_OK)
                return;
        }

        uint32_t ns_now = platform_ns();
        uint32_t ns_elapsed = (ns_now - ns_prev);
        ns_prev = ns_now;
        uint32_t rt_ticks_per_frame = F_CPU / 1e9 * ns_elapsed;
        sim.speedup = (double)ticks_per_frame / rt_ticks_per_frame;

        if (args.speedup) {
            float speedup_error = args.speedup - sim.speedup;
            int32_t time_error_us = speedup_error * ns_elapsed / 1000;
            int32_t sleep_error_us = time_error_us - sleep_time_us;
            sleep_time_us = -1 * sleep_error_us;
            if (sleep_time_us > 0) {
                platform_sleep(sleep_time_us);
                ticks_per_frame = F_CPU / 1e9 * control_frame_ns * args.speedup;
            } else {
                sleep_time_us = 0;
                ticks_per_frame *= (double)control_frame_ns / ns_elapsed;
            }
        } else {
            ticks_per_frame *= (double)control_frame_ns / ns_elapsed;
        }

        target_ticks += ticks_per_frame;
    }
}

void sim_serial_out(uint8_t data)
{
    static uint8_t buf[128] = {0};
    static uint8_t len = 0;

    buf[len++] = data;
    if (data == '\n' || data == '\r' || len >= 127) {
        if (args.comment_char)
            fprintf(args.serial_out_file, "%c ", args.comment_char);
        buf[len] = '\0';
        fprintf(args.serial_out_file, "%s", buf);
        len = 0;
    }
}
