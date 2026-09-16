#pragma once
// exit_guard.h - let the process exit when the application's last thread does.
//
// glibc ends a process when its thread count reaches zero: the last thread to
// exit calls exit(0), which runs atexit handlers and static destructors.
// pthread_exit() from main does NOT call exit() — it retires the main thread
// and leaves the process alive until that count drops. smash's background
// threads never exit on their own, so the count never drops and the process
// hangs forever: a zombie thread-group leader beside ~16 sleeping smash
// threads.
//
// Measured on this tree. A program whose main thread ends with
// pthread_exit(NULL) hangs under libsmash (needs SIGKILL) and its atexit
// handlers never run; the same binary with SMASH_NO_COMPRESSOR=1 exits 0 with
// handlers run, and with SMASH_LARGE_ONLY=1 it hangs again — which isolates it
// to smash's threads rather than to anything else the library does. smash's
// own shutdown paths cannot help: both the atexit handler in
// compressor_thread.h and the destructor(101) in smash_heap.cpp hang off
// exit(), which this path never reaches.
//
// The fix is not to emulate exit() but to remove smash's threads once the
// application has none left, and let glibc's own last-thread logic run. That
// gives stock semantics — status 0, atexit handlers, static destructors — with
// nothing reproduced here. Calling exit() ourselves would mean choosing a
// status and running the application's handlers on a compressor thread.
//
// Detection is authoritative rather than bookkeeping: /proc/self/task is
// scanned, and any task that is not the caller, not one of smash's registered
// threads, and not a zombie (the retired main thread lingers as one) counts as
// a live application thread. A counter of wrapped threads would inherit the
// failure this library just fixed, where interposition silently stopped
// wrapping application threads for years. Nothing is scanned until the main
// thread is seen to retire, so an ordinary process pays one relaxed atomic
// load per thread exit and nothing else.
//
// Everything here runs on a thread that is already exiting, so it uses raw
// syscalls only: no malloc, no stdio, no reentry into the allocator.

#include <atomic>
#include <cstdlib>

#if defined(__linux__)
#include <fcntl.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace smash {
namespace exit_guard {

#if defined(__linux__)

// Bounded registry of the threads smash created. Slots are reused, so a
// thread must release its slot on the way out: a stale tid could later collide
// with a recycled tid belonging to the application, and an application thread
// mistaken for one of ours is the one error that could end a process early.
inline constexpr int kMaxSmashThreads = 64;
inline std::atomic<int> g_smash_tids[kMaxSmashThreads];
inline std::atomic<bool> g_main_retired{false};
inline std::atomic<bool> g_shutdown_started{false};
inline std::atomic<void (*)()> g_shutdown_hook{nullptr};

inline int selfTid() { return static_cast<int>(::syscall(SYS_gettid)); }

inline void registerSmashThread() {
    const int tid = selfTid();
    for (int i = 0; i < kMaxSmashThreads; ++i) {
        int expected = 0;
        if (g_smash_tids[i].compare_exchange_strong(expected, tid,
                std::memory_order_acq_rel, std::memory_order_relaxed))
            return;
    }
    // Registry full: this thread reads as an application thread from now on,
    // which can only delay the process exit — never cause one while the
    // application is still running.
}

inline void unregisterSmashThread() {
    const int tid = selfTid();
    for (int i = 0; i < kMaxSmashThreads; ++i) {
        int expected = tid;
        if (g_smash_tids[i].compare_exchange_strong(expected, 0,
                std::memory_order_acq_rel, std::memory_order_relaxed))
            return;
    }
}

// RAII so every return path out of a thread entry releases the slot.
struct ThreadRegistration {
    ThreadRegistration() { registerSmashThread(); }
    ~ThreadRegistration() { unregisterSmashThread(); }
    ThreadRegistration(const ThreadRegistration&) = delete;
    ThreadRegistration& operator=(const ThreadRegistration&) = delete;
};

inline bool isSmashTid(int tid) {
    for (int i = 0; i < kMaxSmashThreads; ++i)
        if (g_smash_tids[i].load(std::memory_order_acquire) == tid) return true;
    return false;
}

// True when the task is gone or is a zombie. The retired main thread stays in
// /proc/self/task as state Z for the life of the process and is counted in
// "Threads:", so skipping zombies is what makes the scan meaningful.
inline bool taskIsDeadOrZombie(const char* tid_name) {
    char path[64];
    int n = 0;
    for (const char* p = "/proc/self/task/"; *p; ++p) path[n++] = *p;
    for (const char* q = tid_name; *q && n < 50; ++q) path[n++] = *q;
    for (const char* s = "/stat"; *s; ++s) path[n++] = *s;
    path[n] = '\0';

    const int fd = static_cast<int>(::syscall(SYS_openat, AT_FDCWD, path, O_RDONLY, 0));
    if (fd < 0) return true;                    // exited between readdir and now
    char buf[256];
    const long r = ::syscall(SYS_read, fd, buf, sizeof(buf) - 1);
    ::syscall(SYS_close, fd);
    if (r <= 0) return true;
    buf[r] = '\0';
    // "pid (comm) S ..." — comm can contain spaces and parentheses, so the
    // state is the character two past the LAST ')'.
    const char* last = nullptr;
    for (char* c = buf; *c; ++c)
        if (*c == ')') last = c;
    if (!last || !last[1]) return false;
    return last[2] == 'Z';
}

// The kernel's getdents64 record. d_name is a NUL-terminated name that runs to
// d_reclen; declared [1] rather than [] because a flexible array member is a
// C99 extension in C++.
struct LinuxDirent64 {
    unsigned long long d_ino;
    long long d_off;
    unsigned short d_reclen;
    unsigned char d_type;
    char d_name[1];
};

// Is any application thread other than the caller still alive?
inline bool appThreadsAlive() {
    const int self = selfTid();
    const int fd = static_cast<int>(
        ::syscall(SYS_openat, AT_FDCWD, "/proc/self/task", O_RDONLY | O_DIRECTORY, 0));
    if (fd < 0) return true;   // cannot tell: never end a process on a guess

    bool alive = false;
    alignas(8) char buf[4096];
    while (!alive) {
        const long n = ::syscall(SYS_getdents64, fd, buf, sizeof(buf));
        if (n <= 0) break;
        for (long off = 0; off < n && !alive;) {
            auto* d = reinterpret_cast<LinuxDirent64*>(buf + off);
            off += d->d_reclen;
            if (d->d_name[0] < '0' || d->d_name[0] > '9') continue;
            int tid = 0;
            for (const char* c = d->d_name; *c >= '0' && *c <= '9'; ++c)
                tid = tid * 10 + (*c - '0');
            if (tid == self || isSmashTid(tid)) continue;
            if (taskIsDeadOrZombie(d->d_name)) continue;
            alive = true;
        }
    }
    ::syscall(SYS_close, fd);
    return alive;
}

// SMASH_NO_EXIT_GUARD=1 turns the guard off: smash's threads then keep the
// process alive after the application's last thread exits, which is the
// pre-fix behaviour. Kept as an escape hatch for an application that really
// does rely on a library's background threads outliving its own; setting it is
// also how test_main_pthread_exit_* is checked to be a real discriminator
// (with the guard off the test times out and fails).
inline bool guardDisabled() {
    static const bool off = [] {
        const char* v = std::getenv("SMASH_NO_EXIT_GUARD");
        return v && v[0] == '1';
    }();
    return off;
}

inline void setShutdownHook(void (*fn)()) {
    g_shutdown_hook.store(fn, std::memory_order_release);
}

// fork() clones only the calling thread, so none of the parent's smash threads
// exist in the child. Leaving their tids registered would let a recycled tid
// in the child read as one of smash's own — the single error that could end a
// process while the application is still running. The child re-registers its
// threads when the compressor restarts.
inline void resetAfterFork() {
    for (int i = 0; i < kMaxSmashThreads; ++i)
        g_smash_tids[i].store(0, std::memory_order_relaxed);
    g_main_retired.store(false, std::memory_order_relaxed);
    g_shutdown_started.store(false, std::memory_order_relaxed);
}

// Runs the hook once. Idempotent: the atexit handler and destructor(101) that
// smash already registers will run afterwards on the normal exit path and must
// not repeat the joins.
inline void runShutdown() {
    bool expected = false;
    if (!g_shutdown_started.compare_exchange_strong(expected, true,
            std::memory_order_acq_rel, std::memory_order_relaxed))
        return;
    if (auto fn = g_shutdown_hook.load(std::memory_order_acquire)) fn();
}

// Called from a TLS destructor on every thread smash sees exit. Before the
// main thread retires this is a single relaxed load; a process that never
// calls pthread_exit() from main never scans anything.
inline void noteThreadExiting() {
    if (guardDisabled()) return;
    if (!g_main_retired.load(std::memory_order_acquire)) {
        // Only the main thread retiring can strand the process: any other
        // thread exiting still leaves main to reach exit() normally.
        if (selfTid() != static_cast<int>(::getpid())) return;
        g_main_retired.store(true, std::memory_order_release);
    }
    if (appThreadsAlive()) return;
    // Last application thread. Joining smash's threads here makes the caller
    // the last thread standing, so glibc ends the process from this thread
    // exactly as it would without libsmash loaded.
    runShutdown();
}

// Fallback for the compressor coordinator: an application thread that smash
// never saw start cannot run the TLS-destructor trigger above.
inline bool applicationIsGone() {
    if (guardDisabled()) return false;
    if (!g_main_retired.load(std::memory_order_acquire)) return false;
    if (g_shutdown_started.load(std::memory_order_acquire)) return false;
    return !appThreadsAlive();
}

#else  // !__linux__

struct ThreadRegistration {};
inline void registerSmashThread() {}
inline void unregisterSmashThread() {}
inline void setShutdownHook(void (*)()) {}
inline void runShutdown() {}
inline void noteThreadExiting() {}
inline bool applicationIsGone() { return false; }
inline void resetAfterFork() {}

#endif

}  // namespace exit_guard
}  // namespace smash
