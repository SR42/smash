#pragma once
// park.h - sleep until a 32-bit word changes (futex on Linux).
//
// The default compressor helpers used to wait for work with usleep(100) in a
// loop: ~6 000 wakeups a second per helper, all 7 helpers, all the time, even
// when one worker was active. Measured on an 8-CPU VM: 38 s of kernel time in
// a 180 s run, the largest single kernel cost of the process. A parked thread
// costs nothing until it is woken.
//
// park() may return spuriously; callers loop on their own condition.
//
// macOS has the same primitive as __ulock_wait/__ulock_wake (libSystem; what
// libc++'s std::atomic::wait uses there). It used to fall back to a 100 us
// sleep, which kept the polling this replaced on macOS -- the problem
// upstream's Waker (#93, a condition variable) fixed there. A condition
// variable is not an option here: the page locks park from the fault handler
// (page_state.h), and a ulock is a syscall, safe in a signal handler and with
// nothing to re-initialise after fork(). Elsewhere park() still sleeps 100 us.
#include <atomic>
#include <climits>
#include <cstdint>
#include <ctime>
#include <unistd.h>
#if defined(__linux__)
#include <linux/futex.h>
#include <sys/syscall.h>
#elif defined(__APPLE__)
extern "C" int __ulock_wait(uint32_t operation, void* addr, uint64_t value, uint32_t timeout_us);
extern "C" int __ulock_wake(uint32_t operation, void* addr, uint64_t wake_value);
#endif

namespace smash {

// Sleeps while `word` holds `expected`.
#if defined(__APPLE__)
// <sys/ulock.h> is not in the SDK; these are its values.
inline constexpr uint32_t kUlCompareAndWait = 1;
inline constexpr uint32_t kUlfWakeAll = 0x00000100;
inline constexpr uint32_t kUlfNoErrno = 0x01000000;
#endif

inline void park(std::atomic<uint32_t>& word, uint32_t expected) {
#if defined(__linux__)
    syscall(SYS_futex, &word, FUTEX_WAIT_PRIVATE, expected, nullptr, nullptr, 0);
#elif defined(__APPLE__)
    (void)__ulock_wait(kUlCompareAndWait | kUlfNoErrno, &word, expected, 0);
#else
    if (word.load(std::memory_order_acquire) == expected) usleep(100);
#endif
}

// Sleeps while `word` holds `expected`, for at most `timeout_ms` (0: no
// limit). Without a futex it sleeps 100 us, as park() does.
inline void parkFor(std::atomic<uint32_t>& word, uint32_t expected, uint64_t timeout_ms) {
#if defined(__linux__)
    if (timeout_ms == 0) {
        park(word, expected);
        return;
    }
    struct timespec ts;
    ts.tv_sec = static_cast<time_t>(timeout_ms / 1000);
    ts.tv_nsec = static_cast<long>(timeout_ms % 1000) * 1000000L;
    syscall(SYS_futex, &word, FUTEX_WAIT_PRIVATE, expected, &ts, nullptr, 0);
#elif defined(__APPLE__)
    // Microseconds, 0 = no limit; anything past ~71 minutes waits that long.
    const uint64_t us = timeout_ms * 1000;
    const uint32_t t = us == 0 ? 0 : (us > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(us));
    (void)__ulock_wait(kUlCompareAndWait | kUlfNoErrno, &word, expected, t);
#else
    (void)timeout_ms;
    park(word, expected);
#endif
}

// Wakes up to `count` threads parked on `word`.
inline void unpark(std::atomic<uint32_t>& word, int count) {
#if defined(__linux__)
    syscall(SYS_futex, &word, FUTEX_WAKE_PRIVATE, count, nullptr, nullptr, 0);
#elif defined(__APPLE__)
    // A ulock wakes one waiter or all of them.
    (void)__ulock_wake(kUlCompareAndWait | kUlfNoErrno | (count > 1 ? kUlfWakeAll : 0), &word, 0);
#else
    (void)word;
    (void)count;
#endif
}

}  // namespace smash
