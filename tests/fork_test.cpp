// fork_test.cpp - fork() while other threads are inside the allocator
// (src/smash_heap.cpp atfork handlers; SmashHeap::forkPrepare / forkParent /
// forkChild; CompressorThread::pauseForFork).
//
// A child inherits every lock word as it was at the instant of the fork. The
// prepare handler paused only the compressor's tick (and gave up after 50 ms
// if one was running); no allocator lock was taken, so a child whose first
// large malloc landed on a sub-arena another parent thread had locked, or
// whose free() needed a span shard or a slab lock held at the fork, spun
// for ever. The prepare handler now takes every allocator lock (trylock, in
// rounds, so it cannot deadlock against the threads it is waiting out),
// the parent releases them and the child re-initialises them; a tick in
// progress stops at the next page boundary.
//
//   held: a thread holds every allocator lock for 400 ms (the diagnostic
//         export smash_debug_hold_locks, exactly what a thread caught mid-
//         operation does) while main forks 50 ms in; the child's malloc and
//         free must complete -- the prepare handler waits the holder out.
//         Without that, the child inherits the locks held and spins
//   page lock: a thread holds one compressed page's lock for 400 ms
//         (smash_debug_hold_page_lock: what a thread inside the fault
//         handler holds, across the decompress, the write and the mprotect)
//         while main forks 50 ms in; the child reads that page. The prepare
//         handler does not wait for page locks (the handler is microseconds
//         and the fault slot is per thread), so the child must re-initialise
//         them: before it did so only after a tick pause had timed out, and
//         a child that inherited a fault handler's lock spun on it for ever
//         (2 of 200 hung in a SMASH_SOFTDIRTY=0 run)
//   churn: 8 threads allocate and free 64 B, 20 KiB and 1 MiB blocks (slab
//          locks, span shards, sub-arena locks) and read a compressed block
//          (page locks, store) throughout
//   forks: 200 of them; each child pins itself to every CPU in turn and
//          allocates and frees a 1 MiB and a 64 B block there (every
//          sub-arena of the shared set), reads the compressed block, then
//          exits. A child alive after 5 s is hung.
//
// Registered with SMASH_SUBARENAS=1 in large-only and full mode, and once
// with SMASH_NO_COMPRESSOR=1: there the held-locks phase is decisive (with a
// compressor, the holder also blocks the tick, which the pause waits out).
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <dlfcn.h>
#include <pthread.h>
#include <sched.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {
std::atomic<bool> g_stop{false};
unsigned char* g_cold = nullptr;   // compressed before the forks start
const size_t kCold = 8u << 20;

void churn(unsigned seed) {
    static const size_t sizes[] = {64, 20000, 1u << 20, 1u << 20, 3u << 20};
    std::vector<void*> held;
    while (!g_stop.load(std::memory_order_relaxed)) {
        seed = seed * 1103515245u + 12345u;
        const size_t n = sizes[(seed >> 16) % 5];
        auto* p = static_cast<unsigned char*>(std::malloc(n));
        if (p) { p[0] = 1; p[n - 1] = 2; held.push_back(p); }
        if (held.size() > 8) { std::free(held.front()); held.erase(held.begin()); }
        volatile unsigned char v = g_cold[(seed >> 8) % kCold];
        (void)v;
    }
    for (void* p : held) std::free(p);
}

int childBody() {
    const int ncpu = static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN));
    for (int cpu = 0; cpu < ncpu && cpu < 64; ++cpu) {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET(cpu, &set);
        sched_setaffinity(0, sizeof set, &set);
        auto* big = static_cast<unsigned char*>(std::malloc(1u << 20));
        auto* small = static_cast<unsigned char*>(std::malloc(64));
        if (!big || !small) return 2;
        big[0] = 1; big[(1u << 20) - 1] = 2; small[0] = 3;
        std::free(big);
        std::free(small);
    }
    size_t sum = 0;
    for (size_t i = 0; i < kCold; i += 4096) sum += g_cold[i];
    return sum == (kCold / 4096) * 0x5A ? 0 : 4;
}
}  // namespace

int main(int argc, char** argv) {
    const int forks = argc > 1 ? std::atoi(argv[1]) : 200;
    using info_fn = int (*)(const void*, int*, int*, int*);
    auto page_info = reinterpret_cast<info_fn>(dlsym(RTLD_DEFAULT, "smash_page_info"));
    if (!page_info) { std::fprintf(stderr, "FAIL: smash_page_info not exported\n"); return 1; }

    g_cold = static_cast<unsigned char*>(std::malloc(kCold));
    std::memset(g_cold, 0x5A, kCold);
    const char* nc = std::getenv("SMASH_NO_COMPRESSOR");
    const bool compressor_off = nc && nc[0] == '1';
    int compressed = 0;
    // With the compressor off nothing compresses, and nothing but the prepare
    // handler's locking can make fork() wait for the holder: that is the
    // registration in which the held-locks phase tells the fix from no fix.
    for (int waited = 0; !compressor_off && waited < 20000 && compressed < 1000; waited += 200) {
        usleep(200000);
        compressed = 0;
        for (size_t i = 0; i < kCold; i += 4096) {
            int st = 0, c = 0, r = 0;
            if (page_info(g_cold + i, &st, &c, &r) == 0 && st == 2) ++compressed;
        }
    }
    std::printf("fork: %d of %zu cold pages compressed before the forks\n", compressed, kCold / 4096);

    // Deterministic: fork while another thread holds every lock.
    using hold_fn = int (*)(unsigned);
    auto hold = reinterpret_cast<hold_fn>(dlsym(RTLD_DEFAULT, "smash_debug_hold_locks"));
    if (!hold) { std::fprintf(stderr, "FAIL: smash_debug_hold_locks not exported\n"); return 1; }
    int hung = 0, bad = 0;
    for (int round = 0; round < 3; ++round) {
        std::thread holder([&] { hold(400); });
        usleep(50000);
        const auto t0 = std::chrono::steady_clock::now();
        pid_t pid = fork();
        if (pid == 0) _exit(childBody());
        const double fork_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        int st = 0;
        bool done = false;
        for (int w = 0; w < 500; ++w) {
            if (waitpid(pid, &st, WNOHANG) == pid) { done = true; break; }
            usleep(10000);
        }
        holder.join();
        if (!done) { ++hung; kill(pid, SIGKILL); waitpid(pid, &st, 0); }
        else if (!WIFEXITED(st) || WEXITSTATUS(st) != 0) ++bad;
        std::printf("fork: with the locks held, fork() took %.0f ms; child %s\n", fork_ms,
                    !done ? "HUNG" : (WIFEXITED(st) && WEXITSTATUS(st) == 0 ? "ok" : "failed"));
    }

    // Deterministic: fork while another thread holds a compressed page's lock.
    using hold_page_fn = int (*)(const void*, unsigned);
    auto hold_page = reinterpret_cast<hold_page_fn>(dlsym(RTLD_DEFAULT, "smash_debug_hold_page_lock"));
    if (!hold_page) { std::fprintf(stderr, "FAIL: smash_debug_hold_page_lock not exported\n"); return 1; }
    for (int round = 0; round < 3 && !compressor_off; ++round) {
        // A page the child will read through the fault handler.
        const unsigned char* page = g_cold;
        for (size_t i = 0; i < kCold; i += 4096) {
            int st = 0, c = 0, r = 0;
            if (page_info(g_cold + i, &st, &c, &r) == 0 && st == 2) { page = g_cold + i; break; }
        }
        std::atomic<int> held{-1};
        std::thread holder([&] { held.store(hold_page(page, 400)); });
        usleep(50000);
        const auto t0 = std::chrono::steady_clock::now();
        pid_t pid = fork();
        if (pid == 0) _exit(childBody());
        const double fork_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        int st = 0;
        bool done = false;
        for (int w = 0; w < 500; ++w) {
            if (waitpid(pid, &st, WNOHANG) == pid) { done = true; break; }
            usleep(10000);
        }
        holder.join();
        if (held.load() != 0) { std::fprintf(stderr, "FAIL: smash_debug_hold_page_lock returned %d\n", held.load()); ++bad; }
        if (!done) { ++hung; kill(pid, SIGKILL); waitpid(pid, &st, 0); }
        else if (!WIFEXITED(st) || WEXITSTATUS(st) != 0) ++bad;
        std::printf("fork: with a page lock held, fork() took %.0f ms; child %s\n", fork_ms,
                    !done ? "HUNG" : (WIFEXITED(st) && WEXITSTATUS(st) == 0 ? "ok" : "failed"));
    }

    std::vector<std::thread> th;
    for (unsigned t = 0; t < 8; ++t) th.emplace_back(churn, t + 1);
    usleep(100000);

    for (int i = 0; i < forks; ++i) {
        pid_t pid = fork();
        if (pid == 0) _exit(childBody());
        if (pid < 0) { std::fprintf(stderr, "FAIL: fork failed\n"); return 1; }
        int st = 0;
        bool done = false;
        for (int w = 0; w < 500; ++w) {
            if (waitpid(pid, &st, WNOHANG) == pid) { done = true; break; }
            usleep(10000);
        }
        if (!done) { ++hung; kill(pid, SIGKILL); waitpid(pid, &st, 0); }
        else if (!WIFEXITED(st) || WEXITSTATUS(st) != 0) ++bad;
    }
    g_stop.store(true);
    for (auto& t : th) t.join();
    std::printf("fork: %d forks, %d hung, %d bad exits\n", forks, hung, bad);
    if (hung || bad) {
        std::fprintf(stderr, "FAIL: %d children hung, %d exited badly\n", hung, bad);
        return 1;
    }
    std::printf("fork: all tests passed\n");
    return 0;
}
