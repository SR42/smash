#pragma once
// thread_stack.h - stack sizes for smash's own threads.
//
// glibc places a new thread's static TLS block (the initial-exec TLS of every
// loaded module) and its guard page inside the thread's stack. smash used to
// create its coordinator and helper threads with a fixed 128 KiB stack, so an
// application whose libraries carry ~128 KiB or more of static TLS — large C++
// and ML stacks do — made every one of those pthread_create calls fail with
// EINVAL at library load: "[smash WARN] pthread_create(coordinator) failed;
// compression disabled", and smash silently ran as a plain allocator. Measured
// with a test app: 100 KiB of static TLS worked, 128 KiB and 256 KiB did not.
// A 64 KiB-page kernel makes it worse, since the guard page alone is 64 KiB.
//
// So a smash thread's stack is what the thread itself needs PLUS whatever glibc
// needs on top of PTHREAD_STACK_MIN (static TLS + guard), rounded to the
// RUNTIME page size. glibc exposes that minimum as __pthread_get_minstack
// (GLIBC_PRIVATE; Chromium sizes its thread stacks the same way). When it is
// unavailable, callers fall back to default thread attributes, which glibc
// always sizes to fit.
#include <cstddef>
#include <pthread.h>
#include <limits.h>
#include <unistd.h>
#if defined(__linux__) && defined(__GLIBC__)
#include <dlfcn.h>
#endif

namespace smash {

inline size_t runtimePageSize() {
    static const size_t p = [] {
        long v = sysconf(_SC_PAGESIZE);
        return v > 0 ? static_cast<size_t>(v) : size_t{4096};
    }();
    return p;
}

// Bytes glibc needs in a thread's stack beyond PTHREAD_STACK_MIN (static TLS
// of all loaded modules + guard), or 0 if glibc does not say.
inline size_t staticTlsStackOverhead() {
    static const size_t v = []() -> size_t {
#if defined(__linux__) && defined(__GLIBC__)
        using fn_t = size_t (*)(const pthread_attr_t*);
        auto fn = reinterpret_cast<fn_t>(dlsym(RTLD_DEFAULT, "__pthread_get_minstack"));
        if (!fn) return 0;
        pthread_attr_t a;
        if (pthread_attr_init(&a) != 0) return 0;
        const size_t m = fn(&a);
        pthread_attr_destroy(&a);
        const size_t floor = static_cast<size_t>(PTHREAD_STACK_MIN);
        return m > floor ? m - floor : 0;
#else
        return 0;
#endif
    }();
    return v;
}

// Stack size for a smash thread that itself needs `usable` bytes of stack.
inline size_t smashThreadStackSize(size_t usable) {
    size_t s = usable + staticTlsStackOverhead();
    const size_t floor = static_cast<size_t>(PTHREAD_STACK_MIN);
    if (s < floor) s = floor;
    const size_t p = runtimePageSize();
    return (s + p - 1) / p * p;
}

// pthread_create for smash's threads: a stack of `usable` bytes on top of what
// glibc needs; if that is still refused, retry once with default attributes
// (glibc sizes those to fit, at the cost of a larger virtual reservation).
// Returns the error of the last attempt (0 on success).
inline int createSmashThread(pthread_t* t, size_t usable,
                             void* (*fn)(void*), void* arg) {
    pthread_attr_t attr;
    int rc = pthread_attr_init(&attr);
    if (rc == 0) {
        rc = pthread_attr_setstacksize(&attr, smashThreadStackSize(usable));
        if (rc == 0) rc = pthread_create(t, &attr, fn, arg);
        pthread_attr_destroy(&attr);
        if (rc == 0) return 0;
    }
    return pthread_create(t, nullptr, fn, arg);
}

}  // namespace smash
