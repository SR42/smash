// numa_arena_churn_test.cpp - malloc_numa must keep reusing its node arena.
//
// A node arena serves a request from its own free runs, then from the unused
// tail of its newest 64 MiB chunk, then from a freshly carved chunk. free()
// does not put a run back in the free map directly: it queues it, and the run
// joins the map only when the decommit thread processes that entry. So a tight
// allocate/free loop keeps missing the map by a few microseconds, and the
// miss used to go straight to fresh region.
//
// The result was that churn consumed the whole arena and then served
// allocations from separate bound mappings, which smash never compresses —
// silently, since the block still works. Measured before the fix with a 1 GiB
// arena: 64 KiB blocks touched all 1023.9 MiB of it and 4.5% of single-thread
// allocations fell out; at 16 threads over 4 nodes, 87% did. The same loop
// with 50 us of slack per iteration reused a 0.8 MiB window and never fell
// out, which is what identified the queue rather than fragmentation.
//
// An allocation that misses now drains the queue and looks again before taking
// anything new. This test is the guard: with a 1 GiB arena, every block of a
// churn loop must stay inside the compressible arena, single- and
// multi-threaded. SMASH_VM_GIB=1 is load-bearing — with the default 16 GiB the
// loop would not reach exhaustion and the check would pass vacuously.
//
// Skips when the kernel or a seccomp filter refuses mbind (ENOSYS/EPERM).

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <thread>
#include <time.h>
#include <vector>

static double nowSec() {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}

#if defined(__linux__)
#include <unistd.h>
#include "numa_test_util.h"
#endif

using numa_fn = void* (*)(uint32_t, size_t);
using in_arena_fn = int (*)(const void*);

static numa_fn g_numa = nullptr;
static in_arena_fn g_in_arena = nullptr;
static int g_failures = 0;

#if defined(__linux__)

// One churn pass: allocate, touch, free. Returns how many blocks came from
// outside the compressible arena.
static long churn(uint32_t node, size_t size, long iters, long* nulls) {
    long out = 0;
    for (long i = 0; i < iters; ++i) {
        void* p = g_numa(node, size);
        if (!p) { ++*nulls; continue; }
        if (!g_in_arena(p)) ++out;
        *static_cast<volatile unsigned char*>(p) = 1;
        free(p);
    }
    return out;
}

static void runLinux() {
    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    errno = 0;
    void* probe = g_numa(0, ps);
    if (!probe && (errno == ENOSYS || errno == EPERM)) {
        printf("SKIP: mbind refused (errno=%d %s)\n", errno, strerror(errno));
        exit(77);
    }
    free(probe);

    const std::vector<int> nodes = numa_test::memoryNodes();

    // Host context, printed before any assertion: this test is sensitive to
    // node count (every node's arena carves its own 64 MiB chunks out of one
    // SMASH_VM_GIB arena) and to the kernel page size (smash's kPageSize is a
    // compile-time 4096; an aarch64 kernel with 16 or 64 KiB pages makes every
    // mprotect and madvise sub-page-aligned — see test_page_size). A failure
    // report is hard to act on without these.
    const char* vm_gib = getenv("SMASH_VM_GIB");
    printf("host: %zu memory node(s), %ld online CPU(s), kernel page %ld B, "
           "SMASH_VM_GIB=%s\n",
           nodes.size(), sysconf(_SC_NPROCESSORS_ONLN), ps,
           vm_gib ? vm_gib : "(default)");
    if (ps != 4096)
        printf("NOTE: kernel page is %ld B, not 4096 — smash is built with a "
               "compile-time 4096-byte page, so this test's arena arithmetic "
               "and smash's own page operations disagree with the kernel.\n", ps);

    // Single-threaded, per node. 100k round trips of 64 KiB is 6.25 GiB of
    // allocation through a 1 GiB arena, so it cannot be satisfied without
    // reuse: before the fix this exhausted the arena at round trip ~55000.
    for (int node : nodes) {
        long nulls = 0;
        const double t0 = nowSec();
        const long out = churn(static_cast<uint32_t>(node), 64 * 1024, 100000, &nulls);
        const double dt = nowSec() - t0;
        if (dt > 20.0)
            printf("  node %d: 100000 round trips took %.1f s\n", node, dt);
        if (out || nulls) {
            fprintf(stderr, "FAIL: node %d: 100000 single-threaded round trips gave "
                    "%ld block(s) outside the compressible arena and %ld NULL — the "
                    "node arena is taking fresh region instead of reusing its own "
                    "freed runs\n", node, out, nulls);
            ++g_failures;
        }
    }
    if (!g_failures)
        printf("single-threaded churn: %zu node(s) x 100000 round trips, all in arena\n",
               nodes.size());

    // Concurrent, spread over the nodes: the same miss, with the threads also
    // contending for the arena lock and the free map.
    // Scaled to the host, not fixed at 16. The race this guards only appears
    // with real parallelism: on an 8-CPU box 16 threads showed 0 fallbacks
    // whether or not the fix was present, while the same 16 threads on a
    // many-core aarch64 host showed 20 — so a fixed count makes the test blind
    // on small machines and is what let this regression reach a green suite.
    // 8x the CPU count reproduces it everywhere (203 fallbacks unfixed at 64
    // threads here, against 0-8 fixed over five runs).
    const int kThreads = [] {
        // SMASH_TEST_THREADS: reproduce a many-core host's count on a small one.
        if (const char* t = getenv("SMASH_TEST_THREADS"); t && atoi(t) > 0)
            return atoi(t);
        const long cpus = sysconf(_SC_NPROCESSORS_ONLN);
        const long want = (cpus > 0 ? cpus : 2) * 8;
        return static_cast<int>(want < 16 ? 16 : (want > 256 ? 256 : want));
    }();
    const long kPerThread = 320000 / kThreads;
    std::atomic<long> out_total{0}, null_total{0};
    std::vector<std::thread> th;
    for (int t = 0; t < kThreads; ++t) {
        th.emplace_back([&, t] {
            long nulls = 0;
            const long out = churn(static_cast<uint32_t>(nodes[t % nodes.size()]),
                                   64 * 1024, kPerThread, &nulls);
            out_total += out;
            null_total += nulls;
        });
    }
    for (auto& t : th) t.join();
    // The concurrent phase asserts a small bound, not zero, and the single-
    // threaded phase above asserts zero. The difference is real rather than
    // lenience: a run that has left its owner and not yet reached the free map
    // is invisible to an allocation looking for it, so an allocation that
    // misses waits for in-flight frees — but that wait is bounded, and under
    // enough concurrency it can expire with pages still in transit. Falling
    // back then is a footprint loss, never a correctness one.
    //
    // What this phase can and cannot guard, stated plainly because getting it
    // wrong cost three rounds of threshold-chasing across two machines.
    //
    // It CAN catch a catastrophic regression: with no drain at all, 285955 of
    // 320000 blocks (87%) fell out. It CANNOT reliably catch the subtler race
    // it was originally written for — one thread taking the whole decommit
    // queue while others conclude nothing is recoverable. That residual scales
    // with real parallelism and varies run to run: 0-8 of 320000 at 64 threads
    // on an 8-CPU host, 34-303 at 256 threads there, and 1537 at 256 threads on
    // a 128-CPU one. The unfixed code overlaps that range from below (203 at 64
    // threads on the small host), so no fixed threshold separates them on every
    // machine, and a ratio does not either — measured 3x on one run and 37x on
    // the next.
    //
    // So the ceiling is 2%, which is a catastrophic-regression guard, and the
    // single-threaded phase above carries the precise assertion: it is
    // deterministic, it caught the original defect at 4.5%, and it is 0 on both
    // hosts. For the race itself, measure deliberately with SMASH_NO_DRAIN_WAIT
    // rather than trusting this number.
    const long total = static_cast<long>(kThreads) * kPerThread;
    const long allowed = total / 50;   // 2%
    if (out_total > allowed || null_total) {
        fprintf(stderr, "FAIL: %d threads x %ld round trips gave %ld block(s) outside "
                "the compressible arena (allowed %ld of %ld) and %ld NULL\n",
                kThreads, kPerThread, out_total.load(), allowed, total,
                null_total.load());
        ++g_failures;
    } else if (out_total) {
        printf("concurrent churn: %ld of %ld blocks fell back (%.3f%%, ceiling %ld) — "
               "in-flight frees the bounded wait did not catch; compare with "
               "SMASH_NO_DRAIN_WAIT=1 to see what the wait is worth here\n",
               out_total.load(), total, 100.0 * out_total.load() / total, allowed);
    }
    if (!out_total && !null_total)
        printf("concurrent churn: %d threads x %ld round trips over %zu node(s), "
               "all in arena\n", kThreads, kPerThread, nodes.size());
}

#endif

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    g_numa = reinterpret_cast<numa_fn>(dlsym(RTLD_DEFAULT, "malloc_numa"));
    g_in_arena = reinterpret_cast<in_arena_fn>(dlsym(RTLD_DEFAULT, "smash_in_arena"));
    if (!g_numa || !g_in_arena) {
        fprintf(stderr, "FAIL: malloc_numa not found (not running under libsmash)\n");
        return 1;
    }
#if defined(__linux__)
    runLinux();
#else
    printf("SKIP: node arenas are Linux-only\n");
    return 77;
#endif
    if (g_failures) {
        fprintf(stderr, "FAIL: %d check(s) failed\n", g_failures);
        return 1;
    }
    printf("PASS: malloc_numa churn stays inside the compressible arena\n");
    return 0;
}
