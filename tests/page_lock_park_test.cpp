// page_lock_park_test.cpp - what threads waiting for a page lock cost
// (src/vm/page_state.h PageLockTable).
//
// A page lock was a test-and-set byte spun on for ever: a thread faulting on
// a page whose lock another thread held burnt a CPU until the holder let go,
// and it is faulting application threads that wait -- a restore in progress,
// a compressor worker in the non-deferred mode, 128 threads reading one hot
// page as it comes back. The lock now spins for the microseconds a normal
// hold lasts and then parks the thread on a futex (hashed wait words), to be
// woken by the unlock.
//
//   a thread holds one compressed page's lock for 1 s
//   (smash_debug_hold_page_lock); 16 threads then read the page through
//   the fault handler and wait on that lock. The process's CPU time over
//   the hold must be under a fifth of a CPU per waiting thread (spinning
//   waiters burn min(16, CPUs) CPUs the whole second), every waiter must
//   be done within 500 ms of the release, and must have read the right
//   bytes (the restore ran once, after the lock).
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <sys/resource.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {
double cpuSeconds() {
    struct rusage ru{};
    getrusage(RUSAGE_SELF, &ru);
    return ru.ru_utime.tv_sec + ru.ru_utime.tv_usec / 1e6 + ru.ru_stime.tv_sec + ru.ru_stime.tv_usec / 1e6;
}
}  // namespace

int main() {
    using info_fn = int (*)(const void*, int*, int*, int*);
    using hold_fn = int (*)(const void*, unsigned);
    auto page_info = reinterpret_cast<info_fn>(dlsym(RTLD_DEFAULT, "smash_page_info"));
    auto hold = reinterpret_cast<hold_fn>(dlsym(RTLD_DEFAULT, "smash_debug_hold_page_lock"));
    if (!page_info || !hold) { std::fprintf(stderr, "FAIL: diagnostic exports missing\n"); return 1; }
    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    const int ncpu = static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN));
    const int kWaiters = 16;
    const unsigned kHoldMs = 1000;
    int failures = 0;

    auto* block = static_cast<unsigned char*>(std::malloc(1u << 20));
    if (!block) return 1;
    for (size_t off = 0; off < (1u << 20); off += 64) block[off] = static_cast<unsigned char>(off >> 12);
    // A compressed page the waiters will fault on.
    unsigned char* page = nullptr;
    for (int waited = 0; waited < 30000 && !page; waited += 200) {
        usleep(200000);
        for (size_t off = 0; off < (1u << 20); off += ps) {
            int st = 0, c = 0, r = 0;
            if (page_info(block + off, &st, &c, &r) == 0 && st == 2) { page = block + off; break; }
        }
    }
    if (!page) { std::fprintf(stderr, "FAIL: no page compressed in 30 s\n"); return 1; }
    const unsigned char expected = static_cast<unsigned char>((page - block) >> 12);

    std::atomic<int> hold_rc{-1};
    std::thread holder([&] { hold_rc.store(hold(page, kHoldMs)); });
    usleep(30000);

    std::atomic<int> wrong{0};
    std::atomic<int> done{0};
    const double cpu0 = cpuSeconds();
    const auto t0 = std::chrono::steady_clock::now();
    std::vector<std::thread> waiters;
    for (int i = 0; i < kWaiters; ++i)
        waiters.emplace_back([&] {
            volatile unsigned char v = *page;
            if (v != expected) wrong.fetch_add(1);
            done.fetch_add(1);
        });
    holder.join();
    const double cpu_hold = cpuSeconds() - cpu0;
    const auto t_release = std::chrono::steady_clock::now();
    for (auto& t : waiters) t.join();
    const double after_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_release).count();
    const double hold_s = std::chrono::duration<double>(t_release - t0).count();

    const double budget = 0.2 * (kWaiters < ncpu ? kWaiters : ncpu) * hold_s;
    std::printf("page_lock_park: %d waiters on a lock held %.2f s: %.2f s of CPU (spinning would burn about %.1f s; budget %.2f s); all done %.0f ms after the release\n",
                kWaiters, hold_s, cpu_hold, (double)(kWaiters < ncpu ? kWaiters : ncpu) * hold_s, budget, after_ms);
    if (hold_rc.load() != 0) { std::fprintf(stderr, "FAIL: smash_debug_hold_page_lock returned %d\n", hold_rc.load()); ++failures; }
    if (cpu_hold > budget) { std::fprintf(stderr, "FAIL: waiters spun: %.2f s of CPU over a %.2f s hold\n", cpu_hold, hold_s); ++failures; }
    if (after_ms > 500) { std::fprintf(stderr, "FAIL: waiters took %.0f ms to run after the release (missed wake-up)\n", after_ms); ++failures; }
    if (wrong.load()) { std::fprintf(stderr, "FAIL: %d waiters read the wrong byte\n", wrong.load()); ++failures; }
    std::free(block);

    if (failures) { std::fprintf(stderr, "page_lock_park: %d failure(s)\n", failures); return 1; }
    std::printf("page_lock_park: all tests passed\n");
    return 0;
}
