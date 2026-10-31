/*
  main.c - grblHAL FlexiHAL Simulator

  Part of grblHAL FlexiHAL Simulator

  Compiles the real grblHAL core with a virtual HAL driver that emulates
  the Expatria FlexiHAL controller. Connects to senders over USB serial
  (a host serial port standing in for the board's USB CDC port) and over
  WebSocket, like the real board's USB and network streams.

  Based on grblHAL Simulator by Terje Io, Jens Geisler, Adam Shelly
*/

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#include "simulator.h"
#include "mcu.h"
#include "eeprom.h"
#include "grbl_interface.h"
#include "usb_host.h"
#include "websocket.h"

#include "grbl/grbllib.h"

arg_vars_t args;
static const char *progname;

static void print_usage(const char *badarg)
{
    if (badarg)
        printf("Unrecognized option %s\n", badarg);

    printf(
        "grblHAL FlexiHAL Simulator\n\n"
        "Usage: %s [options]\n"
        "  Options:\n"
        "    -w <port>   WebSocket port for sender connection (default: 8080, 0=off)\n"
#ifdef WIN32
        "    -u <port>   USB serial: COM port for the sender side (e.g. COM7 of a com0com pair)\n"
#else
        "    -u <path>   USB serial: symlink to create for the sender (e.g. /tmp/flexihal-usb)\n"
#endif
        "    -t <speed>  Realtime speed multiplier (default: 1.0, 0=max)\n"
        "    -e <file>   EEPROM/settings file (default: EEPROM.DAT)\n"
        "    -r <time>   Step report interval in seconds (0=off)\n"
        "    -n          No comment prefix on serial output\n"
        "    -h          This help\n"
        "\n"
        "  Keyboard controls:\n"
        "    e=E-Stop  r=Reset  h=FeedHold  s=CycleStart  d=SafetyDoor\n"
        "    p=Probe   x/y/z=Toggle limits  ?=Status report\n"
        "    1-9/0=Inject alarms 1-9/10  m=MotorFault alarm 17\n"
        "    n=Toggle no-response mode  k=Kick WebSocket client\n"
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
    // Defaults
    args.step_out_file = stderr;
    args.block_out_file = stdout;
    args.serial_out_file = stdout;
    args.comment_char = '#';
    args.speedup = 1.0f;
    args.step_time = 0.0f;
    args.ws_port = 8080;

    const char *usb_path = NULL;

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
                    usb_path = *argv;
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

    // The WebSocket client gets a grblHAL stream of its own.
    if (args.ws_port && ws_init(args.ws_port) != 0) {
        printf("Fatal: Could not start WebSocket server on port %d\n", args.ws_port);
        return EXIT_FAILURE;
    }

    // The emulated UART is the board's USB stream. Without -u it acts like a USB
    // port with nothing plugged in.
    if (usb_path && !usb_host_init(usb_path)) {
        printf("Fatal: Could not open USB serial port %s\n", usb_path);
        return EXIT_FAILURE;
    }

    sim.getchar = usb_host_getchar;     // with no port there's never DTR, so nothing goes in or out
    sim.putchar = usb_host_putchar;
    sim.socket_fd = 1; // Non-zero: stdin is free for simulator controls

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

    printf("\nReady. Connect your sender to");
    if (args.ws_port)
        printf(" ws://localhost:%d", args.ws_port);
    if (usb_path)
        printf("%s %s", args.ws_port ? " or" : "", usb_path);
    printf("\nUse the console keyboard or launcher buttons for simulator inputs.\n\n");

    // Main loop: run simulation while polling stdin controls.
    while (sim.exit != exit_OK) {
        // Service the sender ports
        if (usb_path)
            usb_host_service();
        ws_service();

        // Run one frame of simulation
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
    usb_host_shutdown();

    platform_terminate();

    printf("Simulator exited.\n");
    return EXIT_SUCCESS;
}
