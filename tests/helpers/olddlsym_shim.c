// olddlsym_shim.c - make this glibc's dlsym() behave like glibc < 2.34's.
//
// Before 2.34, dlsym lived in libdl and _dlerror_run() callocs a per-thread
// dl_action_result on a thread's first dlsym, before the lookup, storing it
// only after calloc returns (so a dlsym from inside that calloc callocs
// again). RHEL 8 (2.28, the release floor), Ubuntu 20.04 and Debian 11 (2.31)
// all do this. Preloaded after libsmash, this shim reproduces it on any glibc:
// the calloc goes to whatever calloc the process has, which is smash's.
//
// Guard against the recursion this used to cause: 200 nested callocs from one
// dlsym exits 99 with a message, so a regression fails the test instead of
// overflowing the stack.
#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdlib.h>
#include <unistd.h>

static __thread void* result;
static __thread int depth;

static void* (*realDlsym(void))(void*, const char*) {
    static void* (*real)(void*, const char*);
    if (!real) {
        // dlvsym is a different symbol, so looking the real dlsym up through it
        // never re-enters this function. The version is the one glibc gives
        // dlsym since 2.34 (when it moved into libc) on every architecture.
        real = (void* (*)(void*, const char*))dlvsym(RTLD_NEXT, "dlsym", "GLIBC_2.34");
        if (!real) real = (void* (*)(void*, const char*))dlvsym(RTLD_NEXT, "dlsym", "GLIBC_2.2.5");
        if (!real) real = (void* (*)(void*, const char*))dlvsym(RTLD_NEXT, "dlsym", "GLIBC_2.17");
        if (!real) { static const char m[] = "olddlsym_shim: no real dlsym\n"; (void)!write(2, m, sizeof m - 1); _exit(98); }
    }
    return real;
}

__attribute__((visibility("default")))
void* dlsym(void* handle, const char* name) {
    void* (*real)(void*, const char*) = realDlsym();
    if (!result) {
        if (++depth > 200) {
            static const char m[] = "olddlsym_shim: dlsym recursed through calloc 200 deep\n";
            (void)!write(2, m, sizeof m - 1);
            _exit(99);
        }
        void* r = calloc(1, 40);
        --depth;
        result = r;
    }
    return real(handle, name);
}
