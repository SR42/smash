// slowdlsym_shim.c - a dlsym() that takes 3 ms.
//
// Widens the window in which one thread's lazy dlsym() of a wrapper's real
// target overlaps another thread's first call to a different wrapper, which
// is what tests/lazy_resolve_race_test.cpp needs to hit deterministically.
// Preloaded after libsmash.
#define _GNU_SOURCE
#include <dlfcn.h>
#include <time.h>
#include <unistd.h>

__attribute__((visibility("default")))
void* dlsym(void* handle, const char* name) {
    static void* (*real)(void*, const char*);
    if (!real) {
        real = (void* (*)(void*, const char*))dlvsym(RTLD_NEXT, "dlsym", "GLIBC_2.34");
        if (!real) real = (void* (*)(void*, const char*))dlvsym(RTLD_NEXT, "dlsym", "GLIBC_2.2.5");
        if (!real) real = (void* (*)(void*, const char*))dlvsym(RTLD_NEXT, "dlsym", "GLIBC_2.17");
        if (!real) { static const char m[] = "slowdlsym_shim: no real dlsym\n"; (void)!write(2, m, sizeof m - 1); _exit(98); }
    }
    struct timespec ts = {0, 3 * 1000 * 1000};
    nanosleep(&ts, 0);
    return real(handle, name);
}
