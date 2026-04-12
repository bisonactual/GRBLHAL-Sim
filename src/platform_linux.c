/*
  platform_linux.c - Linux platform specifics

  Part of grblHAL FlexiHAL Simulator

  Based on grblHAL Simulator by Adam Shelly
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include "platform.h"

#define MS_PER_SEC 1000000

void platform_init(void) {}
void platform_terminate(void) {}

uint32_t platform_ns(void)
{
    static uint32_t gTimeBase = 0;
    uint32_t timestamp;
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    timestamp = ts.tv_sec * (uint32_t)1e9 + ts.tv_nsec;
    if (gTimeBase == 0)
        gTimeBase = timestamp;
    return timestamp - gTimeBase;
}

void platform_sleep(long microsec)
{
    struct timespec ts = {0};
    while (microsec >= MS_PER_SEC) {
        ts.tv_sec++;
        microsec -= MS_PER_SEC;
    }
    ts.tv_nsec = microsec * 1000;
    nanosleep(&ts, NULL);
}

static void enable_kbhit(int dir)
{
    static struct termios oldt, newt;
    if (dir == 1) {
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    } else
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

static int kbhit(void)
{
    struct timeval tv = {0};
    fd_set rdfs = {{0}};
    FD_SET(STDIN_FILENO, &rdfs);
    select(STDIN_FILENO + 1, &rdfs, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &rdfs);
}

plat_thread_t *platform_start_thread(plat_threadfunc_t threadfunc)
{
    plat_thread_t *th = malloc(sizeof(plat_thread_t));
    if (pthread_create(&th->tid, NULL, threadfunc, &th->exit)) {
        free(th);
        return NULL;
    }
    return th;
}

void platform_stop_thread(plat_thread_t *th)
{
    th->exit = 1;
    pthread_join(th->tid, NULL);
}

void platform_kill_thread(plat_thread_t *th)
{
    th->exit = 1;
    pthread_cancel(th->tid);
}

uint8_t platform_poll_stdin(void)
{
    uint8_t char_in = 0;
    enable_kbhit(1);
    if (kbhit())
        char_in = getchar();
    enable_kbhit(0);
    return char_in;
}
