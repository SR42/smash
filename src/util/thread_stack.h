#pragma once
// thread_stack.h - starting smash's own threads: the mark every one of them
// carries, and their stack sizes.
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
#include <atomic>
#include <cstddef>
#include <pthread.h>
#include <limits.h>
#include <sched.h>
#include <unistd.h>
#if defined(__linux__) && defined(__GLIBC__)
#include <dlfcn.h>
#endif

namespace smash {

// True on a thread smash created: the compressor's coordinator, helpers and
// node workers, and VmRegion's decommit thread. Such a thread never allocates
// from the heap smash manages; its malloc, calloc, realloc and memalign go to
// the system allocator, and it has no thread cache.
//
// The compressor is what takes arena pages away. If one of its threads kept
// its own data there, the page would be compressed like any other and the
// thread would fault on it at moments when the fault cannot be served: while
// it holds that page's lock (not recursive: the handler refuses and the
// SIGSEGV is fatal), while it holds another page's lock (two workers doing
// that to each other deadlock), or inside the fault handler.
//
// The mark has to be there before anything on the thread can allocate. It
// was set by the thread's entry function, which runs after the thread hook
// (SmashHeap::threadInit) has given every thread a thread cache, and malloc's
// fast path looks at nothing but the cache: all 16 compressor threads had
// one, and every small malloc and calloc on them came from the arena, as did
// memalign of any size (issue #7). startSmashThread() now sets it first.
//
// inline thread_local (C++17) gives a single definition across all TUs, both
// libsmash.so and test executables that include the headers directly.
inline __attribute__((tls_model("initial-exec")))
    thread_local bool g_smash_thread = false;

// What a smash thread still needs from the allocator's thread hook (the exit
// watch and the alternate signal stack: SmashHeap::threadInit, which skips
// the thread cache for a marked thread), and how to hand back a thread cache
// given before the mark (SmashHeap::threadCleanup). Set by SmashHeap; null in
// a program that uses these headers without it.
inline std::atomic<void (*)()> g_smash_thread_init{nullptr};
inline std::atomic<void (*)()> g_smash_thread_drop_cache{nullptr};

// Marks the calling thread as one of smash's. Every smash thread's entry
// function calls it first. A thread started by startSmashThread() is marked
// already; one that came through the application's thread hook instead (no
// trampoline on this platform, or the real pthread_create not found) has
// been given a thread cache, which goes back here.
inline void enterSmashThread() {
    if (g_smash_thread) return;
    g_smash_thread = true;
    if (auto drop = g_smash_thread_drop_cache.load(std::memory_order_acquire)) drop();
}

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

#if defined(__linux__) && defined(__GLIBC__)
namespace detail {
// A thread's function and argument, on their way to the trampoline. A fixed
// table, because the thread that starts must not allocate and the one that
// creates it may be an application thread (whose block would be the arena's).
// An entry is in use from the create call until the new thread has read it.
struct ThreadStart {
    std::atomic<int> busy{0};
    void* (*fn)(void*) = nullptr;
    void* arg = nullptr;
};
inline constexpr int kThreadStarts = 64;
inline ThreadStart g_thread_starts[kThreadStarts];

inline void* smashThreadTrampoline(void* p) {
    auto* start = static_cast<ThreadStart*>(p);
    void* (*fn)(void*) = start->fn;
    void* arg = start->arg;
    start->busy.store(0, std::memory_order_release);
    // First, before anything on this thread can allocate.
    enterSmashThread();
    if (auto init = g_smash_thread_init.load(std::memory_order_acquire)) init();
    return fn(arg);
}

// libc's pthread_create. Inside libsmash the plain name is alloc8's
// interposer (-Bsymbolic), whose trampoline runs the application's thread
// hook on the new thread before the thread's own function.
using pthread_create_fn = int (*)(pthread_t*, const pthread_attr_t*, void* (*)(void*), void*);
inline pthread_create_fn realPthreadCreate() {
    static const pthread_create_fn real =
        reinterpret_cast<pthread_create_fn>(dlsym(RTLD_NEXT, "pthread_create"));
    return real;
}
}  // namespace detail
#endif

// pthread_create for a thread of smash's own, with the attributes given
// (nullptr: the defaults). The thread is marked (g_smash_thread) before
// anything else runs on it. Every thread smash creates starts here.
inline int startSmashThread(pthread_t* t, const pthread_attr_t* attr,
                            void* (*fn)(void*), void* arg) {
#if defined(__linux__) && defined(__GLIBC__)
    if (detail::pthread_create_fn real = detail::realPthreadCreate()) {
        // An entry is free again as soon as its thread has started, and smash
        // creates its threads one after another, so the first pass finds one.
        // The retries are for a table left busy some other way (a fork while
        // a thread was being born); they end, and the fallback below starts
        // the thread all the same.
        for (int attempt = 0; attempt < 1000; ++attempt) {
            for (detail::ThreadStart& start : detail::g_thread_starts) {
                int expected = 0;
                if (!start.busy.compare_exchange_strong(expected, 1, std::memory_order_acq_rel,
                                                        std::memory_order_relaxed))
                    continue;
                start.fn = fn;
                start.arg = arg;
                const int rc = real(t, attr, detail::smashThreadTrampoline, &start);
                if (rc != 0) start.busy.store(0, std::memory_order_release);
                return rc;
            }
            sched_yield();
        }
    }
#endif
    // Through the application's thread hook; the entry function's
    // enterSmashThread() hands back the cache that gives it.
    return pthread_create(t, attr, fn, arg);
}

// startSmashThread with a stack of `usable` bytes on top of what glibc
// needs; if that is still refused, retry once with default attributes
// (glibc sizes those to fit, at the cost of a larger virtual reservation).
// Returns the error of the last attempt (0 on success).
inline int createSmashThread(pthread_t* t, size_t usable,
                             void* (*fn)(void*), void* arg) {
    pthread_attr_t attr;
    int rc = pthread_attr_init(&attr);
    if (rc == 0) {
        rc = pthread_attr_setstacksize(&attr, smashThreadStackSize(usable));
        if (rc == 0) rc = startSmashThread(t, &attr, fn, arg);
        pthread_attr_destroy(&attr);
        if (rc == 0) return 0;
    }
    return startSmashThread(t, nullptr, fn, arg);
}

}  // namespace smash
