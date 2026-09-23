// smash/src/util/spinlock.h - Lightweight spinlock (no malloc dependency)
#pragma once

#include <atomic>
#include <cstdint>
#include "thread_safety.h"

#if defined(SMASH_SIM)
#include <time.h>
#endif

namespace smash {

#if defined(SMASH_SIM)
// Simulator build only (tools/smash_sim.cpp): contended acquisitions and the
// time spent waiting, per call site — the return address into the function
// that took the lock — so the simulator can say which lock is hot without
// perf. A small open-addressed table; a full table drops new sites.
namespace sim {
struct LockSite {
    std::atomic<uintptr_t> ra{0};
    std::atomic<uint64_t> contended{0};
    std::atomic<uint64_t> wait_ns{0};
};
inline constexpr unsigned kLockSites = 1024;
inline LockSite g_lock_sites[kLockSites];
inline LockSite* lockSite(uintptr_t ra) {
    unsigned h = static_cast<unsigned>((ra >> 4) * 2654435761u) & (kLockSites - 1);
    for (unsigned i = 0; i < kLockSites; ++i, h = (h + 1) & (kLockSites - 1)) {
        uintptr_t cur = g_lock_sites[h].ra.load(std::memory_order_acquire);
        if (cur == ra) return &g_lock_sites[h];
        if (cur == 0 && g_lock_sites[h].ra.compare_exchange_strong(cur, ra))
            return &g_lock_sites[h];
        if (cur == ra) return &g_lock_sites[h];
    }
    return nullptr;
}
inline uint64_t nowNs() {
    timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return static_cast<uint64_t>(t.tv_sec) * 1000000000ull + static_cast<uint64_t>(t.tv_nsec);
}
}  // namespace sim
#define SMASH_SIM_NOINLINE __attribute__((noinline))
#else
#define SMASH_SIM_NOINLINE
#endif

// CPU relax hint for the spin-wait body.
static inline void spinRelax() {
#if defined(__x86_64__) || defined(_M_X64)
    __builtin_ia32_pause();
#elif defined(__aarch64__) || defined(_M_ARM64)
    asm volatile("yield");
#endif
}

// Test-and-test-and-set spinlock with bounded exponential backoff.
//
// The naive test-and-set loop (flag_.test_and_set every iteration) issues a
// bus-locked read-for-ownership on EVERY spin, so N contending threads generate
// an O(N) cache-line ping-pong storm on the single lock word. On a 192-core
// EPYC this collapsed the neuron-cc walrus backend's slab-refill path: profiling
// showed 99.6% of allocateBatch/drain self-time on the `test %cl,%cl` of the
// TAS spin, IPC halved (0.26 vs 0.49) at equal cycles — pure coherence traffic.
//
// TTAS fixes the mechanism: spin on a plain relaxed LOAD (the line stays Shared,
// generating no coherence traffic while the lock is held), and only issue the
// atomic exchange when the lock APPEARS free. Exponential backoff further thins
// the herd of exchanges the instant the lock is released.
class SMASH_CAPABILITY("mutex") Spinlock {
    std::atomic<uint8_t> state_{0};   // 0 = free, 1 = held

public:
    SMASH_SIM_NOINLINE void lock() SMASH_ACQUIRE() {
        // Fast path: uncontended acquire.
        if (tryLockRelaxed()) return;
#if defined(SMASH_SIM)
        const uintptr_t ra = reinterpret_cast<uintptr_t>(__builtin_return_address(0));
        const uint64_t t0 = sim::nowNs();
        struct Record {
            uintptr_t ra; uint64_t t0;
            ~Record() {
                if (sim::LockSite* site = sim::lockSite(ra)) {
                    site->contended.fetch_add(1, std::memory_order_relaxed);
                    site->wait_ns.fetch_add(sim::nowNs() - t0, std::memory_order_relaxed);
                }
            }
        } record{ra, t0};
#endif
        unsigned backoff = 1;
        for (;;) {
            // Read-spin: no atomic RMW, line stays Shared, zero coherence
            // traffic while another core holds the lock.
            while (state_.load(std::memory_order_relaxed) != 0)
                spinRelax();
            // Lock looks free — attempt the one atomic exchange.
            if (tryLockRelaxed()) return;
            // Lost the race; back off to spread out the retry storm.
            for (unsigned i = 0; i < backoff; ++i) spinRelax();
            if (backoff < 1024) backoff <<= 1;
        }
    }

    void unlock() SMASH_RELEASE() {
        state_.store(0, std::memory_order_release);
    }

    bool tryLock() SMASH_TRY_ACQUIRE(true) {
        return tryLockRelaxed();
    }

private:
    // Single atomic acquire attempt. Acquire ordering on success.
    bool tryLockRelaxed() {
        uint8_t expected = 0;
        return state_.compare_exchange_strong(
            expected, 1, std::memory_order_acquire, std::memory_order_relaxed);
    }
};

class SMASH_SCOPED_CAPABILITY LockGuard {
    Spinlock& lock_;
public:
    explicit LockGuard(Spinlock& lock) SMASH_ACQUIRE(lock) : lock_(lock) { lock_.lock(); }
    ~LockGuard() SMASH_RELEASE() { lock_.unlock(); }
    LockGuard(const LockGuard&) = delete;
    LockGuard& operator=(const LockGuard&) = delete;
};

} // namespace smash
