#ifndef _PLATFORM_LINUX_H_
#define _PLATFORM_LINUX_H_

#include <pthread.h>

#define PLAT_THREAD_FUNC(name, exitvar) void *name(void *exitvar)
typedef void *(*plat_threadfunc_t)(void *);

typedef struct {
    pthread_t tid;
    int exit;
} plat_thread_t;

#endif
