/*
  simulator.h - simulation engine

  Part of grblHAL FlexiHAL Simulator

  Based on grblHAL Simulator by Terje Io, Jens Geisler, Adam Shelly
*/

#ifndef simulator_h
#define simulator_h

#include <stdio.h>
#include "platform.h"

typedef void (*sim_hook_fp)(void);

typedef struct sim_vars {
    uint64_t masterclock;
    double sim_time;
    uint8_t started;
    enum { exit_NO, exit_REQ, exit_OK } exit;
    float speedup;
    int32_t baud_ticks;
#ifdef WIN32
    SOCKET socket_fd;
#else
    int socket_fd;
#endif
    uint8_t (*getchar)(void);
    void (*putchar)(uint8_t);
    sim_hook_fp on_init;
    sim_hook_fp on_tick;
    sim_hook_fp on_byte;
    sim_hook_fp on_shutdown;
} sim_vars_t;

extern sim_vars_t sim;

typedef struct arg_vars {
    FILE *block_out_file;
    FILE *step_out_file;
    FILE *serial_out_file;
    float speedup;
    double step_time;
    uint8_t comment_char;
    uint16_t ws_port;       // WebSocket port (replaces raw telnet)
} arg_vars_t;

extern arg_vars_t args;

void init_simulator(void);
void shutdown_simulator(void);
void sim_loop(void);
void simulate_serial(void);
void sim_serial_out(uint8_t data);
void sim_socket_out(uint8_t data);

#endif
