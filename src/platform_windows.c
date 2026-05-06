/*
  platform_windows.c - Windows platform specifics

  Part of grblHAL FlexiHAL Simulator

  Based on grblHAL Simulator by Adam Shelly
*/

#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <windows.h>
#include "platform.h"

#define NS_PER_SEC 1000000000
#define MICRO_PER_MILLI 1000

static double ns_per_perfcount;

void platform_init(void)
{
    __int64 counts_per_sec;
    QueryPerformanceFrequency((LARGE_INTEGER *)&counts_per_sec);
    ns_per_perfcount = (float)NS_PER_SEC / counts_per_sec;
}

void platform_terminate(void) {}

uint32_t platform_ns(void)
{
    static uint32_t gTimeBase = 0;
    __int64 counts;
    uint32_t ns;

    QueryPerformanceCounter((LARGE_INTEGER *)&counts);
    ns = (uint32_t)(counts * ns_per_perfcount);
    if (gTimeBase == 0)
        gTimeBase = ns;
    return ns - gTimeBase;
}

void platform_sleep(long microsec)
{
    Sleep((DWORD)(microsec / MICRO_PER_MILLI));
}

plat_thread_t *platform_start_thread(plat_threadfunc_t threadfunc)
{
    plat_thread_t *th = malloc(sizeof(plat_thread_t));
    th->tid = CreateThread(NULL, 0, threadfunc, &th->exit, 0, NULL);
    if (!th->tid) {
        free(th);
        return NULL;
    }
    return th;
}

void platform_stop_thread(plat_thread_t *th)
{
    th->exit = 1;
    WaitForSingleObject(th->tid, INFINITE);
}

void platform_kill_thread(plat_thread_t *th)
{
    th->exit = 1;
    TerminateThread(th->tid, 0);
}

uint8_t platform_poll_stdin(void)
{
    HANDLE input = GetStdHandle(STD_INPUT_HANDLE);
    DWORD type = GetFileType(input);

    if (type == FILE_TYPE_CHAR) {
        if (_kbhit())
            return (uint8_t)getch();
        return 0;
    }

    if (type == FILE_TYPE_PIPE) {
        DWORD available = 0;
        if (PeekNamedPipe(input, NULL, 0, NULL, &available, NULL) && available > 0) {
            char c = 0;
            DWORD read = 0;
            if (ReadFile(input, &c, 1, &read, NULL) && read == 1)
                return (uint8_t)c;
        }
    }

    return 0;
}
