// fork_locks.h - every allocator lock, for fork().
//
// A fork child inherits each lock word as it was at the instant of the clone.
// A lock some other parent thread held at that instant is held in the child
// by a thread that does not exist there, for ever. The prepare handler
// therefore takes every lock listed here, the parent handler releases them
// and the child handler re-initialises them (placement-new: no thread but
// the forking one exists in the child, so nothing can be waiting on them).
//
// Acquisition is in rounds of trylock: every lock in order, each given a
// short spin; the first one that stays held makes the round release
// everything it took and start again after a yield. A round can never
// deadlock against a thread that holds an inner lock and wants an outer one,
// whatever order the list is in, because the prepare handler never blocks
// while holding anything. Rounds are bounded too (a lock a thread holds across
// a long operation would otherwise block fork() for that long): past the
// bound fork() proceeds and the child's re-initialisation covers the rest,
// which is what every fork did before this existed.
#pragma once

#include <cstddef>
#include <new>
#include <sched.h>
#include <time.h>
#include "spinlock.h"

namespace smash {

class ForkLockSet {
    // Slabs (arenas x classes x 3), mid classes, span and entry and run-cache
    // shards (256 each), sub-arenas (64 per set), node arenas, store shards.
    static constexpr size_t kMax = 4096;
    Spinlock* locks_[kMax];
    size_t n_ = 0;
    size_t held_ = 0;

public:
    void clear() { n_ = held_ = 0; }
    size_t size() const { return n_; }
    bool full() const { return n_ >= kMax; }
    void add(Spinlock& l) {
        if (n_ < kMax) locks_[n_++] = &l;
    }

    // Takes every lock, in rounds (see above), for up to `max_ms`. Returns
    // false if time ran out with some lock still held by another thread (a
    // thread that holds an allocator lock for seconds is a bug of its own;
    // fork() then proceeds and the child re-initialises everything); `held()`
    // is then 0 and unlockAll() releases nothing.
    bool lockAll(unsigned max_ms = 3000) {
        struct timespec t0{};
        clock_gettime(CLOCK_MONOTONIC, &t0);
        for (unsigned round = 0;; ++round) {
            if ((round & 63) == 63) {
                struct timespec now{};
                clock_gettime(CLOCK_MONOTONIC, &now);
                const long ms = (now.tv_sec - t0.tv_sec) * 1000 + (now.tv_nsec - t0.tv_nsec) / 1000000;
                if (ms >= static_cast<long>(max_ms)) break;
            }
            size_t got = 0;
            for (; got < n_; ++got) {
                bool ok = false;
                for (unsigned spin = 0; spin < 200 && !ok; ++spin) {
                    ok = locks_[got]->tryLock();
                    if (!ok) spinRelax();
                }
                if (!ok) break;
            }
            if (got == n_) { held_ = n_; return true; }
            for (size_t i = got; i-- > 0;) locks_[i]->unlock();
            sched_yield();
        }
        held_ = 0;
        return false;
    }
    size_t held() const { return held_; }

    void unlockAll() {
        for (size_t i = held_; i-- > 0;) locks_[i]->unlock();
        held_ = 0;
    }

    // The child: every lock becomes free, whoever held it in the parent.
    void resetAll() {
        for (size_t i = 0; i < n_; ++i) new (locks_[i]) Spinlock();
        held_ = 0;
    }
};

}  // namespace smash
