/*
  main.c - grblHAL FlexiHAL Simulator

  Part of grblHAL FlexiHAL Simulator

  Compiles the real grblHAL core with a virtual HAL driver that emulates
  the Expatria FlexiHAL controller. Connects to senders via WebSocket
  and accepts UF2 firmware uploads via HTTP.

  Based on grblHAL Simulator by Terje Io, Jens Geisler, Adam Shelly
*/

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#ifdef WIN32
#include <direct.h>
#else
#include <unistd.h>
#endif

#include "simulator.h"
#include "mcu.h"
#include "eeprom.h"
#include "grbl_interface.h"
#include "websocket.h"
#include "uf2.h"

#include "grbl/grbllib.h"

arg_vars_t args;
static const char *progname;
static int saved_argc;
static char **saved_argv;
static char exe_path[512] = {0};

static void print_usage(const char *badarg)
{
    if (badarg)
        printf("Unrecognized option %s\n", badarg);

    printf(
        "grblHAL FlexiHAL Simulator\n\n"
        "Usage: %s [options]\n"
        "  Options:\n"
        "    -w <port>   WebSocket port for sender connection (default: 8080)\n"
        "    -u <port>   HTTP port for UF2 firmware upload (default: 8081)\n"
        "    -t <speed>  Realtime speed multiplier (default: 1.0, 0=max)\n"
        "    -e <file>   EEPROM/settings file (default: EEPROM.DAT)\n"
        "    -r <time>   Step report interval in seconds (0=off)\n"
        "    -n          No comment prefix on serial output\n"
        "    -h          This help\n"
        "\n"
        "  Keyboard controls (when WebSocket client connected):\n"
        "    e=E-Stop  r=Reset  h=FeedHold  s=CycleStart  d=SafetyDoor\n"
        "    p=Probe   x/y/z=Toggle limits  ?=Status report\n"
        "\n", progname);
}

PLAT_THREAD_FUNC(grbl_main_thread, exit)
{
    grbl_enter();
    return 0;
}

static void exithandler(int signum)
{
    (void)signum;
    eeprom_close();
}

int main(int argc, char *argv[])
{
    saved_argc = argc;
    saved_argv = argv;

    // Resolve our executable path now, before a rebuild replaces the binary
#ifndef WIN32
    {
        ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
        if (len > 0)
            exe_path[len] = '\0';
        else
            strncpy(exe_path, argv[0], sizeof(exe_path) - 1);
    }
#else
    strncpy(exe_path, argv[0], sizeof(exe_path) - 1);
#endif

    // Defaults
    args.step_out_file = stderr;
    args.block_out_file = stdout;
    args.serial_out_file = stdout;
    args.comment_char = '#';
    args.speedup = 1.0f;
    args.step_time = 0.0f;
    args.ws_port = 8080;
    args.http_port = 8081;

    set_eeprom_name("EEPROM.DAT");
    progname = argv[0];

    while (argc > 1) {
        argv++; argc--;
        if (argv[0][0] == '-') {
            switch (argv[0][1]) {
                case 'w':
                    argv++; argc--;
                    args.ws_port = (uint16_t)atoi(*argv);
                    break;
                case 'u':
                    argv++; argc--;
                    args.http_port = (uint16_t)atoi(*argv);
                    break;
                case 't':
                    argv++; argc--;
                    args.speedup = (float)atof(*argv);
                    break;
                case 'e':
                    argv++; argc--;
                    set_eeprom_name(*argv);
                    break;
                case 'r':
                    argv++; argc--;
                    args.step_time = atof(*argv);
                    break;
                case 'n':
                    args.comment_char = 0;
                    break;
                case 'h':
                    print_usage(NULL);
                    return EXIT_SUCCESS;
                default:
                    print_usage(*argv);
                    return EXIT_FAILURE;
            }
        }
    }

    printf("=== grblHAL FlexiHAL Simulator ===\n");
    printf("Emulating: Expatria Flexi-HAL (STM32F446)\n");
    printf("Axes: %d (X, Y, Z + 2 ABC motors)\n\n", N_AXIS);

    platform_init();

    // Figure out our directory paths from the executable location
    // We expect to be run from the build/ dir, so root is one level up
    {
        char cwd[512] = {0};
#ifdef WIN32
        _getcwd(cwd, sizeof(cwd));
#else
        getcwd(cwd, sizeof(cwd));
#endif
        // If we're in build/, root is parent. Otherwise assume cwd is root.
        size_t len = strlen(cwd);
        if (len > 6 && strcmp(cwd + len - 5, "build") == 0) {
            char root[512];
            strncpy(root, cwd, len - 6);
            root[len - 6] = '\0';
            uf2_set_paths(root, cwd);
        } else {
            char build_path[600];
            snprintf(build_path, sizeof(build_path), "%s/build", cwd);
            uf2_set_paths(cwd, build_path);
        }
    }

    // Initialize WebSocket server
    if (ws_init(args.ws_port) != 0) {
        printf("Fatal: Could not start WebSocket server on port %d\n", args.ws_port);
        return EXIT_FAILURE;
    }

    // Initialize HTTP server (firmware upload + plugin manager)
    if (uf2_http_init(args.http_port) != 0) {
        printf("Fatal: Could not start HTTP server on port %d\n", args.http_port);
        return EXIT_FAILURE;
    }

    // Wire up sim I/O to WebSocket
    sim.getchar = ws_getchar;
    sim.putchar = ws_putchar;
    sim.socket_fd = 1; // Non-zero signals "network mode" to grbl_interface

    sim.on_init = grbl_app_init;
    sim.on_shutdown = grbl_app_exit;
    sim.on_tick = grbl_per_tick;
    sim.on_byte = grbl_per_byte;

    init_simulator();

    // Launch grblHAL core in its own thread
    plat_thread_t *th = platform_start_thread(grbl_main_thread);
    if (!th) {
        printf("Fatal: Unable to start grblHAL thread\n");
        return EXIT_FAILURE;
    }

    atexit(eeprom_close);
    signal(SIGTERM, exithandler);
    signal(SIGINT, exithandler);

    printf("\nReady. Connect your sender to ws://localhost:%d\n", args.ws_port);
    printf("Manage firmware & plugins at http://localhost:%d\n\n", args.http_port);

    // Main loop: run simulation + poll for UF2 uploads
    while (sim.exit != exit_OK) {
        // Run one frame of simulation
        // (inlined from sim_loop to interleave UF2 polling)
        uint64_t frame_ticks = F_CPU / 100;
        uint64_t target = sim.masterclock + frame_ticks;
        uint64_t next_byte_tick = sim.masterclock + sim.baud_ticks * 10;

        while (sim.masterclock < target && sim.exit != exit_OK) {
            bool read_serial = (sim.masterclock >= next_byte_tick);

            sim.masterclock++;
            sim.sim_time = (double)sim.masterclock / (double)F_CPU;
            mcu_master_clock();
            if (read_serial)
                simulate_serial();

            sim.on_tick();

            if (read_serial) {
                next_byte_tick += sim.baud_ticks * 10;
                sim.on_byte();
            }
        }

        // Check for UF2 upload
        if (uf2_poll()) {
            printf("[SIM] Firmware upload detected — simulating controller restart...\n");
            printf("[SIM] Controller restarted with new firmware.\n");
        }

        // Check if a plugin rebuild completed — auto-restart with new binary
        if (uf2_rebuild_requested()) {
            printf("[SIM] Rebuild completed — restarting with new plugins...\n");

            // Graceful cleanup
            eeprom_close();
            sim.exit = exit_REQ;
            shutdown_simulator();
            platform_kill_thread(th);
            ws_shutdown();
            uf2_shutdown();

            // Small delay to let sockets fully close
            platform_sleep(200000);

            // Re-exec ourselves with the original arguments
            execv(exe_path, saved_argv);

            // execv only returns on failure
            perror("[SIM] execv failed");
            printf("[SIM] Please restart manually.\n");
            return EXIT_FAILURE;
        }

        // Pace to approximate realtime
        if (args.speedup > 0) {
            uint32_t sleep_us = (uint32_t)(1e6 / (100.0 * args.speedup));
            if (sleep_us > 0)
                platform_sleep(sleep_us);
        }
    }

    // Graceful shutdown
    shutdown_simulator();
    platform_kill_thread(th);

    ws_shutdown();
    uf2_shutdown();

    platform_terminate();

    printf("Simulator exited.\n");
    return EXIT_SUCCESS;
}
