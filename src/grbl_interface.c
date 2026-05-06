/*
  grbl_interface.c - hooks to interact with grblHAL running on simulator

  Part of grblHAL FlexiHAL Simulator

  Based on grblHAL Simulator by Terje Io, Jens Geisler, Adam Shelly
*/

#include <stdio.h>
#include <string.h>

#include "grbl_interface.h"
#include "mcu.h"
#include "driver.h"
#include "simulator.h"
#include "websocket.h"

#include "grbl/alarms.h"
#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/report.h"
#include "grbl/state_machine.h"
#include "grbl/system.h"

static int block_position[N_AXIS] = {0};
static uint32_t block_number = 0;
static double next_print_time;

static void print_steps(bool force);
static void printBlock(void);
static void inject_alarm(alarm_code_t alarm, const char *label);
static void force_controller_idle(void);
static void report_controller_state(void);

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
                mcu_gpio_toggle_in(&gpio[CONTROL_PORT], PROBE_DISCONNECTED_BIT);
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
            case '1':
                inject_alarm(Alarm_HardLimit, "hard limit");
                break;
            case '2':
                inject_alarm(Alarm_SoftLimit, "soft limit");
                break;
            case '3':
                inject_alarm(Alarm_AbortCycle, "abort during cycle");
                break;
            case '4':
                inject_alarm(Alarm_ProbeFailInitial, "probe fail initial");
                break;
            case '5':
                inject_alarm(Alarm_ProbeFailContact, "probe fail contact");
                break;
            case '6':
                inject_alarm(Alarm_HomingFailReset, "homing fail reset");
                break;
            case '7':
                inject_alarm(Alarm_HomingFailDoor, "homing fail door");
                break;
            case '8':
                inject_alarm(Alarm_FailPulloff, "homing pull-off fail");
                break;
            case '9':
                inject_alarm(Alarm_HomingFailApproach, "homing approach fail");
                break;
            case '0':
                inject_alarm(Alarm_EStop, "E-stop asserted");
                break;
            case 'm': case 'M':
                inject_alarm(Alarm_MotorFault, "motor fault");
                break;
            case 'n': case 'N': {
                bool muted = ws_toggle_tx_muted();
                printf("[SIM] WebSocket TX mute %s: controller will %s sender replies\n",
                       muted ? "ON" : "OFF",
                       muted ? "drop" : "resume");
                break;
            }
            case 'k': case 'K':
                printf("[SIM] WebSocket client kick %s\n",
                       ws_kick_client() ? "sent" : "ignored; no client connected");
                break;
            case '?':
                protocol_enqueue_realtime_command(CMD_STATUS_REPORT);
                break;
            case SIM_STDIN_REPORT_STATE:
                report_controller_state();
                break;
            case SIM_STDIN_FORCE_IDLE: case 'I':
                force_controller_idle();
                break;
            case 0x06:
                sim.exit = exit_REQ;
                break;
        }
    }
    printBlock();
}

static void inject_alarm(alarm_code_t alarm, const char *label)
{
    printf("[SIM] Injecting alarm %u (%s)\n", (unsigned)alarm, label);
    system_set_exec_alarm(alarm);
}

static void force_controller_idle(void)
{
    if (!sys.driver_started) {
        printf("[SIM] Controller Idle request ignored; driver is still starting\n");
        report_controller_state();
        return;
    }

    system_clear_exec_alarm();
    system_clear_exec_states();
    sys.abort = false;
    sys.reset_pending = false;
    sys.blocking_event = false;
    sys.alarm_pending = Alarm_None;
    state_set(STATE_IDLE);

    printf("[SIM] Controller state forced to Idle\n");
    report_controller_state();
}

static char state_report_buf[1024];
static size_t state_report_len;

static void state_report_write(const char *s)
{
    size_t len = strlen(s);
    size_t available = sizeof(state_report_buf) - state_report_len - 1;

    if (len > available)
        len = available;

    if (len > 0) {
        memcpy(state_report_buf + state_report_len, s, len);
        state_report_len += len;
        state_report_buf[state_report_len] = '\0';
    }
}

static void report_controller_state(void)
{
    if (!sys.driver_started) {
        printf("[SIMSTATE]<Starting>\n");
        fflush(stdout);
        return;
    }

    status_report_tracking_t report = {0};
    report.flags = report_get_rt_flags_all();

    state_report_len = 0;
    state_report_buf[0] = '\0';

    report_realtime_status(state_report_write, &report);

    while (state_report_len > 0 &&
           (state_report_buf[state_report_len - 1] == '\n' ||
            state_report_buf[state_report_len - 1] == '\r')) {
        state_report_buf[--state_report_len] = '\0';
    }

    printf("[SIMSTATE]%s\n", state_report_buf);
    fflush(stdout);
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
