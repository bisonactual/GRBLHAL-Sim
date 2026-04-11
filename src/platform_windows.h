#ifndef _PLATFORM_WINDOWS_H_
#define _PLATFORM_WINDOWS_H_

#include <windows.h>

#define PLAT_THREAD_FUNC(name, exitvar) DWORD WINAPI name(LPVOID exitvar)
typedef LPTHREAD_START_ROUTINE plat_threadfunc_t;

typedef struct {
    HANDLE tid;
    int exit;
} plat_thread_t;

#endif
