/*
  shared.h - values shared between grblHAL's thread and the simulator's main thread

  Part of grblHAL FlexiHAL Simulator

  Each ring buffer has one thread writing to it and one reading from it. The
  writer puts the data in and then publishes the new index with a release
  store, and the reader loads the index with acquire before it reads the data.
  The connection flags are handled the same way.

  MSVC builds are x86/x64 only, and there the default /volatile:ms already gives
  volatile accesses these semantics (all the shared values are volatile).
*/

#ifndef shared_h
#define shared_h

#ifdef _MSC_VER
#define shared_load(value)          (value)
#define shared_store(value, v)      ((value) = (v))
#else
#define shared_load(value)          __atomic_load_n(&(value), __ATOMIC_ACQUIRE)
#define shared_store(value, v)      __atomic_store_n(&(value), (v), __ATOMIC_RELEASE)
#endif

#endif
