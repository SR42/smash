// smash/src/util/waker.h - block a background thread until there is work
//
// smash's background threads (compressor helpers, the VmRegion decommit
// drainer) used to poll with usleep(100): ~10,000 wake-ups per second each.
// On a machine whose cores are all busy with application threads, every
// wake-up can preempt an application thread -- and if that thread holds a
// slab spinlock, every other thread then spins on it until it runs again
// (8-thread random-replace churn: 1.49 s without the compressor, 1.78 s
// with it). A Waker parks the thread on a condition variable instead.
//
// No allocation (statically initialised pthread objects), so it is safe in
// smash internals. The wait is bounded (timeout) as a backstop: a waiter
// re-checks its predicate at least that often even if a wake-up is lost.
#pragma once

#include <pthread.h>
#include <time.h>

namespace smash {

class Waker {
    pthread_mutex_t mutex_ = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t cond_ = PTHREAD_COND_INITIALIZER;

public:
    // Block until ready() is true, re-checking at least every timeout_ms.
    // ready() must read state that the notifier changes BEFORE notifyAll().
    template <class Ready>
    void waitUntil(Ready ready, long timeout_ms) {
        pthread_mutex_lock(&mutex_);
        while (!ready()) {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec += timeout_ms / 1000;
            ts.tv_nsec += (timeout_ms % 1000) * 1000000L;
            if (ts.tv_nsec >= 1000000000L) { ts.tv_sec += 1; ts.tv_nsec -= 1000000000L; }
            if (pthread_cond_timedwait(&cond_, &mutex_, &ts) != 0) break;  // timeout
        }
        pthread_mutex_unlock(&mutex_);
    }

    // Wake every waiter. Taking the mutex orders this after any waiter's
    // predicate check, so a waiter cannot miss the state change.
    void notifyAll() {
        pthread_mutex_lock(&mutex_);
        pthread_mutex_unlock(&mutex_);
        pthread_cond_broadcast(&cond_);
    }

    // fork() child: the parent's waiters are gone and the mutex may have
    // been held by a thread that no longer exists.
    void reinitAfterFork() {
        pthread_mutex_init(&mutex_, nullptr);
        pthread_cond_init(&cond_, nullptr);
    }
};

}  // namespace smash
