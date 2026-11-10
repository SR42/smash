// spinlock_park_test.cpp - what a thread waiting for a Spinlock costs, and
// that none is left waiting (src/util/spinlock.h, issue #49).
//
// The lock was a pure spin: a waiter read the word in a loop and called
// sched_yield() every 1024 pauses, for as long as the holder took. Behind a
// holder that is slow (the slab purge held a slab lock across one madvise
// per span: 90-820 ms) every waiter burnt a CPU for the whole hold, and
// behind a holder the scheduler had taken off its CPU the waiters competed
// with it for the CPU it needed. A waiter now sleeps on a futex after about
// 20 us and the unlock wakes one sleeper.
//
//   1. waiter CPU: a thread holds the lock for 400 ms; the thread waiting
//      for it must use under 40 ms of CPU (a spinning waiter uses all 400).
//   2. the wake chain: 12 threads sleep on a lock held for 200 ms; after the
//      unlock each takes it in turn, holding it 1 ms. All 12 must finish. A
//      sleeper that takes the lock unmarked wakes nobody when it lets go, and
//      the rest sleep for ever.
//   3. mutual exclusion under more threads than CPUs, with holds long enough
//      that waiters sleep: the count must come out exact.
//   4. errno: free() must leave it alone, and a futex wait a signal
//      interrupts sets EINTR. A waiter that is signalled while asleep must
//      come back with the errno it went in with.
//   5. tryLock() fails on a held lock and succeeds on a free one, and an
//      all-zero lock is free (bootstrap memory holds locks that are never
//      constructed).
#include "util/spinlock.h"

#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <pthread.h>
#include <thread>
#include <unistd.h>
#include <vector>

using smash::Spinlock;

namespace {
int failures = 0;
#define CHECK(cond, ...) do { \
    if (!(cond)) { std::fprintf(stderr, "FAIL: " __VA_ARGS__); std::fputc('\n', stderr); ++failures; } \
} while (0)

double seconds(clockid_t c) {
    timespec t;
    clock_gettime(c, &t);
    return static_cast<double>(t.tv_sec) + static_cast<double>(t.tv_nsec) * 1e-9;
}

// Polls `done` for up to `limit_s`. A thread that is still waiting for the
// lock cannot be joined: report and leave the process.
void awaitOrDie(const std::atomic<int>& done, int want, double limit_s, const char* what) {
    const double t0 = seconds(CLOCK_MONOTONIC);
    while (done.load() < want) {
        if (seconds(CLOCK_MONOTONIC) - t0 > limit_s) {
            std::fprintf(stderr, "FAIL: %s: %d of %d threads finished within %.0f s\n",
                         what, done.load(), want, limit_s);
            _exit(1);
        }
        usleep(1000);
    }
}

void testWaiterCpu() {
    Spinlock lock;
    std::atomic<int> held{0}, done{0};
    double cpu = 0, wall = 0;
    std::thread holder([&] {
        lock.lock();
        held.store(1);
        usleep(400000);
        lock.unlock();
    });
    while (!held.load()) usleep(100);
    std::thread waiter([&] {
        const double w0 = seconds(CLOCK_MONOTONIC), c0 = seconds(CLOCK_THREAD_CPUTIME_ID);
        lock.lock();
        cpu = seconds(CLOCK_THREAD_CPUTIME_ID) - c0;
        wall = seconds(CLOCK_MONOTONIC) - w0;
        lock.unlock();
        done.store(1);
    });
    awaitOrDie(done, 1, 30, "waiter CPU");
    holder.join();
    waiter.join();
    std::printf("waiter CPU: waited %.0f ms, used %.2f ms of CPU\n", wall * 1e3, cpu * 1e3);
    CHECK(wall > 0.2, "the waiter got the lock after %.0f ms of a 400 ms hold", wall * 1e3);
    CHECK(cpu < 0.040, "a thread waiting %.0f ms for the lock used %.0f ms of CPU (limit 40): it spins",
          wall * 1e3, cpu * 1e3);
}

void testWakeChain() {
    constexpr int kWaiters = 12;
    Spinlock lock;
    std::atomic<int> done{0};
    int count = 0;
    lock.lock();
    std::vector<std::thread> threads;
    for (int i = 0; i < kWaiters; ++i)
        threads.emplace_back([&] {
            lock.lock();
            ++count;
            usleep(1000);
            lock.unlock();
            done.fetch_add(1);
        });
    usleep(200000);   // all of them are asleep by now
    lock.unlock();
    awaitOrDie(done, kWaiters, 20, "wake chain");
    for (auto& t : threads) t.join();
    std::printf("wake chain: %d of %d sleepers got the lock\n", count, kWaiters);
    CHECK(count == kWaiters, "%d of %d threads ran the critical section", count, kWaiters);
}

void testExclusion() {
    long ncpu = sysconf(_SC_NPROCESSORS_ONLN);
    const int kThreads = static_cast<int>(ncpu < 4 ? 8 : 2 * ncpu > 32 ? 32 : 2 * ncpu);
    constexpr int kIters = 20000;
    Spinlock lock;
    std::atomic<int> done{0};
    long count = 0;
    std::vector<std::thread> threads;
    for (int t = 0; t < kThreads; ++t)
        threads.emplace_back([&, t] {
            for (int i = 0; i < kIters; ++i) {
                lock.lock();
                const long seen = count;
                // One hold in 512 outlasts the spin, so the others sleep.
                if (((i + t * 37) & 511) == 0) usleep(60);
                count = seen + 1;
                lock.unlock();
            }
            done.fetch_add(1);
        });
    awaitOrDie(done, kThreads, 120, "mutual exclusion");
    for (auto& t : threads) t.join();
    std::printf("mutual exclusion: %d threads, count %ld of %ld\n", kThreads, count,
                static_cast<long>(kThreads) * kIters);
    CHECK(count == static_cast<long>(kThreads) * kIters, "lost updates: %ld of %ld", count,
          static_cast<long>(kThreads) * kIters);
}

std::atomic<int> g_signals{0};
void onSignal(int) { g_signals.fetch_add(1); }

void testErrno() {
    struct sigaction sa;
    std::memset(&sa, 0, sizeof sa);
    sa.sa_handler = onSignal;   // no SA_RESTART: the futex wait returns EINTR
    sigaction(SIGUSR1, &sa, nullptr);
    constexpr int kSentinel = 4242;
    Spinlock lock;
    std::atomic<int> waiting{0}, done{0};
    int after = 0;
    lock.lock();
    std::thread waiter([&] {
        errno = kSentinel;
        waiting.store(1);
        lock.lock();
        after = errno;
        lock.unlock();
        done.store(1);
    });
    while (!waiting.load()) usleep(100);
    for (int i = 0; i < 3; ++i) {
        usleep(50000);   // asleep on the lock by now
        pthread_kill(waiter.native_handle(), SIGUSR1);
    }
    usleep(50000);
    lock.unlock();
    awaitOrDie(done, 1, 30, "errno");
    waiter.join();
    std::printf("errno: %d of 3 signals delivered to the sleeping waiter, errno %d after lock()\n",
                g_signals.load(), after);
    // Standard signals do not queue: on a busy host two can arrive as one.
    CHECK(g_signals.load() >= 1, "no signal reached the waiter");
    CHECK(after == kSentinel, "lock() changed errno from %d to %d (%s)", kSentinel, after,
          std::strerror(after));
}

void testTryLock() {
    Spinlock lock;
    CHECK(lock.tryLock(), "tryLock() failed on a free lock");
    CHECK(!lock.tryLock(), "tryLock() took a held lock");
    lock.unlock();
    CHECK(lock.tryLock(), "tryLock() failed after unlock()");
    lock.unlock();
    alignas(Spinlock) unsigned char raw[sizeof(Spinlock)] = {};
    auto* zero = reinterpret_cast<Spinlock*>(raw);
    CHECK(zero->tryLock(), "an all-zero lock is not free");
    zero->unlock();
}
}  // namespace

int main() {
    testTryLock();
    testWaiterCpu();
    testWakeChain();
    testExclusion();
    testErrno();
    if (failures) {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("OK\n");
    return 0;
}
