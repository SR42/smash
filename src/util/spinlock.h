// smash/src/util/spinlock.h - Lightweight spinlock (no malloc dependency)
#pragma once

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <time.h>
#include "park.h"
#include "thread_safety.h"

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

// A lock for short critical sections: test-and-test-and-set, a bounded spin,
// then the waiter sleeps until the holder wakes it. (The name is historical:
// it only spun until issue #49.)
//
// The naive test-and-set loop (an atomic exchange every iteration) issues a
// bus-locked read-for-ownership on EVERY spin, so N contending threads generate
// an O(N) cache-line ping-pong storm on the single lock word. On a 192-core
// EPYC this collapsed the neuron-cc walrus backend's slab-refill path: profiling
// showed 99.6% of allocateBatch/drain self-time on the `test %cl,%cl` of the
// TAS spin, IPC halved (0.26 vs 0.49) at equal cycles — pure coherence traffic.
//
// TTAS fixes the mechanism: spin on a plain relaxed LOAD (the line stays Shared,
// generating no coherence traffic while the lock is held), and only issue the
// atomic compare-exchange when the lock APPEARS free. Exponential backoff
// further thins the herd of exchanges the instant the lock is released.
//
// Spinning only pays while the holder is running and about to let go. It used
// to be all a waiter did (with a sched_yield() every 1024 pauses), on the
// assumption that no critical section outlasts the spin. Two things broke
// that: a holder that does something slow under the lock (the slab purge made
// one madvise per span: every thread on the slab burnt a CPU for 90-820 ms),
// and a holder the scheduler has taken off its CPU, which is ordinary on a
// host with more threads than CPUs: waiters that yield and spin again stay
// runnable and take CPU time the holder needs (16 threads on 2 CPUs did 0.84
// of the work of 2 threads; 0.90 with waiters that sleep). So after kSpinNs
// the waiter marks the word and sleeps on it (a futex; util/park.h), as the
// page locks do, and unlock() wakes one sleeper when the word is marked.
//
// What sleeping does not change is how long a waiter waits for a descheduled
// holder: until the scheduler runs it again, milliseconds on a busy host.
// Nothing a waiter does can hand the holder its CPU.
//
// The spin is bounded by time, not by a count of pause instructions: one
// takes about a cycle on some arm64 cores, 16 ns on a Zen 3 and more on
// recent Intel.
//
// Signal-safe (a futex is a system call, and the library is linked -z now, so
// clock_gettime needs no lazy binding), nothing to re-initialise after
// fork() but the word itself (ForkLockSet::resetAll), and all-zero is free.
class SMASH_CAPABILITY("mutex") Spinlock {
    // 0 = free, 1 = held, 2 = held and a thread may be asleep on the word.
    std::atomic<uint32_t> state_{0};
    // How long a waiter spins before it sleeps: several times a slab refill
    // or drain (a microsecond or two), about what going to sleep and being
    // woken costs.
    static constexpr uint64_t kSpinNs = 20000;
    // Pause instructions between two looks at the clock.
    static constexpr unsigned kSpinRound = 64;
    // The longest pause after losing the race for a free lock: what the
    // pure spinlock used (tuned on the 192-core host).
    static constexpr unsigned kMaxBackoff = 1024;

public:
    SMASH_SIM_NOINLINE void lock() SMASH_ACQUIRE() {
        // Fast path: uncontended acquire.
        if (tryAcquire(1)) [[likely]] return;
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
        lockSlow();
    }

    void unlock() SMASH_RELEASE() {
        if (state_.exchange(0, std::memory_order_release) == 2) [[unlikely]] wake();
    }

    bool tryLock() SMASH_TRY_ACQUIRE(true) {
        return tryAcquire(1);
    }

private:
    // Single atomic acquire attempt. Acquire ordering on success.
    bool tryAcquire(uint32_t as) {
        uint32_t expected = 0;
        return state_.compare_exchange_strong(
            expected, as, std::memory_order_acquire, std::memory_order_relaxed);
    }

    static uint64_t nowNs() {
        timespec t;
        clock_gettime(CLOCK_MONOTONIC, &t);
        return static_cast<uint64_t>(t.tv_sec) * 1000000000ull + static_cast<uint64_t>(t.tv_nsec);
    }

    [[gnu::noinline]] void wake() {
        // One sleeper: it takes the lock marked (see lockSlow), so its own
        // unlock wakes the next.
        unpark(state_, 1);
    }

    // Sleep while the word is 2. free() must leave errno alone, and a futex
    // wait that is interrupted, or finds the word already changed, sets it.
    [[gnu::noinline]] void sleepOnWord() {
        const int saved_errno = errno;
        park(state_, 2);
        errno = saved_errno;
    }

    [[gnu::noinline]] void lockSlow() {
        // A thread that has slept takes the lock as 2: it cannot know whether
        // it was the only sleeper, and taking it as 1 would leave the others
        // asleep with nobody to wake them.
        uint32_t as = 1;
        unsigned backoff = 1;
        unsigned since_clock = 0;
        uint64_t deadline = 0;
        for (;;) {
            // Read-spin: no atomic RMW, the line stays Shared, zero coherence
            // traffic while another core holds the lock.
            if (state_.load(std::memory_order_relaxed) == 0) {
                // Lock looks free — attempt the one compare-exchange.
                if (tryAcquire(as)) return;
                // Lost the race; back off to spread out the retry storm.
                for (unsigned i = 0; i < backoff; ++i) spinRelax();
                since_clock += backoff;
                if (backoff < kMaxBackoff) backoff <<= 1;
            } else {
                spinRelax();
                ++since_clock;
            }
            if (since_clock < kSpinRound) continue;
            since_clock = 0;
            const uint64_t now = nowNs();
            if (deadline == 0) { deadline = now + kSpinNs; continue; }
            if (now < deadline) continue;
            // Spun long enough: the holder is slow or not running. Mark the
            // word and sleep on it. An unlock before the mark leaves 0 here
            // (the lock is ours, marked, which costs one spare wake); one
            // after it sees the mark and wakes; one in between changes the
            // word, and the futex wait returns at once.
            if (state_.exchange(2, std::memory_order_acquire) == 0) return;
            sleepOnWord();
            as = 2;
            backoff = 1;
            deadline = 0;
        }
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
