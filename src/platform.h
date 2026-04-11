#ifndef platform_h

#ifdef PLATFORM_LINUX
#include "platform_linux.h"
#elif PLATFORM_OSX
#include "platform_osx.h"
#else
#include "platform_windows.h"
#endif

#define platform_h
#include <inttypes.h>

void platform_init(void);
void platform_terminate(void);

plat_thread_t *platform_start_thread(plat_threadfunc_t func);
void platform_stop_thread(plat_thread_t *thread);
void platform_kill_thread(plat_thread_t *thread);

uint32_t platform_ns(void);
void platform_sleep(long microsec);
uint8_t platform_poll_stdin(void);

#endif
