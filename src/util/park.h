#pragma once
// park.h - sleep until a 32-bit word changes (futex on Linux).
//
// The default compressor helpers used to wait for work with usleep(100) in a
// loop: ~6 000 wakeups a second per helper, all 7 helpers, all the time, even
// when one worker was active. Measured on an 8-CPU VM: 38 s of kernel time in
// a 180 s run, the largest single kernel cost of the process. A parked thread
// costs nothing until it is woken.
//
// park() may return spuriously; callers loop on their own condition. Where
// there is no futex (macOS), park() sleeps 100 us, which is the old polling.
#include <atomic>
#include <cstdint>
#include <unistd.h>
#if defined(__linux__)
#include <linux/futex.h>
#include <sys/syscall.h>
#endif

namespace smash {

// Sleeps while `word` holds `expected`.
inline void park(std::atomic<uint32_t>& word, uint32_t expected) {
#if defined(__linux__)
    syscall(SYS_futex, &word, FUTEX_WAIT_PRIVATE, expected, nullptr, nullptr, 0);
#else
    if (word.load(std::memory_order_acquire) == expected) usleep(100);
#endif
}

// Wakes up to `count` threads parked on `word`.
inline void unpark(std::atomic<uint32_t>& word, int count) {
#if defined(__linux__)
    syscall(SYS_futex, &word, FUTEX_WAKE_PRIVATE, count, nullptr, nullptr, 0);
#else
    (void)word;
    (void)count;
#endif
}

}  // namespace smash
