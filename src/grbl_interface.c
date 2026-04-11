/*
  grbl_interface.c - hooks to interact with grblHAL running on simulator

  Part of grblHAL FlexiHAL Simulator

  Based on grblHAL Simulator by Terje Io, Jens Geisler, Adam Shelly
*/

#include <stdio.h>

#include "mcu.h"
#include "driver.h"
#include "simulator.h"

#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"

static int block_position[N_AXIS] = {0};
static uint32_t block_number = 0;
static double next_print_time;

static void print_steps(bool force);
static void printBlock(void);

void grbl_app_init(void)
{
    next_print_time = args.step_time;
}

void grbl_per_tick(void)
{
    print_steps(0);
}

void grbl_per_byte(void)
{
    // When using WebSocket, stdin is free for hardware event simulation
    if (sim.socket_fd) {
        switch (platform_poll_stdin()) {
            case 'e': case 'E':
                mcu_gpio_toggle_in(&gpio[CONTROL_PORT], ESTOP_BIT);
                break;
            case 'r': case 'R':
                mcu_gpio_toggle_in(&gpio[CONTROL_PORT], RESET_BIT);
                break;
            case 'h': case 'H':
                mcu_gpio_toggle_in(&gpio[CONTROL_PORT], FEED_HOLD_BIT);
                break;
            case 's': case 'S':
                mcu_gpio_toggle_in(&gpio[CONTROL_PORT], CYCLE_START_BIT);
                break;
            case 'd': case 'D':
                mcu_gpio_toggle_in(&gpio[CONTROL_PORT], SAFETY_DOOR_BIT);
                break;
            case 'p': case 'P':
                mcu_gpio_toggle_in(&gpio[PROBE_PORT], PROBE_BIT);
                break;
            case 'o': case 'O':
                mcu_gpio_toggle_in(&gpio[PROBE_PORT], PROBE_CONNECTED_BIT);
                break;
            case 'x':
                mcu_gpio_toggle_in(&gpio[LIMITS_PORT0], X_AXIS_BIT);
                break;
            case 'y':
                mcu_gpio_toggle_in(&gpio[LIMITS_PORT0], Y_AXIS_BIT);
                break;
            case 'z':
                mcu_gpio_toggle_in(&gpio[LIMITS_PORT0], Z_AXIS_BIT);
                break;
            case '?':
                protocol_enqueue_realtime_command(CMD_STATUS_REPORT);
                break;
            case 0x06:
                sim.exit = exit_REQ;
                break;
        }
    }
    printBlock();
}

void grbl_app_exit(void)
{
    print_steps(1);
}

static void print_steps(bool force)
{
    static plan_block_t *printed_block = NULL;
    plan_block_t *current_block = plan_get_current_block();

    if (sim.exit == exit_REQ && state_get() < STATE_HOMING)
        sim.exit = exit_OK;

    if (next_print_time == 0.0)
        return;

    if (current_block != printed_block) {
        if (block_number)
            fprintf(args.step_out_file, "%12.5f %d, %d, %d\n",
                    sim.sim_time, sys.position[X_AXIS], sys.position[Y_AXIS], sys.position[Z_AXIS]);
        printed_block = current_block;
        if (current_block == NULL)
            return;
        fprintf(args.step_out_file, "# block number %d\n", block_number++);
    } else if ((current_block && sim.sim_time >= next_print_time) || force) {
        fprintf(args.step_out_file, "%12.5f %d, %d, %d\n",
                sim.sim_time, sys.position[X_AXIS], sys.position[Y_AXIS], sys.position[Z_AXIS]);
        fflush(args.step_out_file);
        while (next_print_time <= sim.sim_time)
            next_print_time += args.step_time;
    }
}

static void printBlock(void)
{
    static plan_block_t *last_block;
    plan_block_t *b = plan_get_recent_block();

    if (b != last_block && b != NULL) {
        int i;
        for (i = 0; i < N_AXIS; i++) {
            if (b->direction.bits & bit(i))
                block_position[i] -= b->steps.value[i];
            else
                block_position[i] += b->steps.value[i];
            fprintf(args.block_out_file, "%d, ", block_position[i]);
        }
        fprintf(args.block_out_file, "%f\n", b->entry_speed_sqr);
        fflush(args.block_out_file);
        last_block = b;
    }
}
