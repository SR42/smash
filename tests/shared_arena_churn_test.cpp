// shared_arena_churn_test.cpp - the shared arena must reuse freed pages.
//
// Two phases, guarding two different ways a large allocation can end up on a
// direct mmap that smash never compresses.
//
// PHASE A - freed runs that are still in flight.
//   free() does not return a run to the free map directly: it queues a
//   decommit, and the run joins the map only when that entry is processed. An
//   allocation that misses the map therefore has to distinguish "gone" from
//   "not back yet". VmRegion::allocatePages() did neither — it called the
//   exhausted hook, retried once and gave up — so concurrent large-block churn
//   through a small arena spilled onto uncompressible mappings. Measured with
//   a 1 GiB arena and 2 MiB blocks: 0 of 40000 single-threaded, then 1728 of
//   40000 at 4 threads, 46263 of 160000 at 8, 93726 at 16 and 144241 at 64.
//   The same defect was fixed for the NUMA node arenas first
//   (numa_arena_churn_test); this is the shared arena every ordinary large
//   allocation uses.
//
// PHASE B - empty slab spans reclaimed under concurrency.
//   When the arena runs out, the heap hands back every slab's empty spans
//   through VmRegion's exhausted hook, and Slab::reclaimEmptySpans() takes each
//   slab with tryLock() — it may already hold another slab's lock, so blocking
//   could deadlock, and a busy slab is skipped. A skipped slab is a negative
//   answer that is not authoritative, which is the same shape as phase A's bug.
//   This phase provokes it: fill the arena with small objects, free them so the
//   pages are held as empty spans, then take large blocks while other threads
//   churn small objects across many size classes. It PASSES today (measured 0
//   of 200 outside the arena with 0, 8 and 32 noisy threads) and is kept as a
//   guard, not as a reproduction.
//
//   It is self-evidencing: phase B first asserts it really exhausted the arena,
//   because a phase that never fills it would pass while proving nothing.
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <thread>
#include <unistd.h>
#include <vector>

using in_arena_fn = int (*)(const void*);
static in_arena_fn g_in_arena = nullptr;
static int g_failures = 0;
static bool inArena(void* p) { return p && g_in_arena(p); }

// Threads scale with the host, but the LIVE SET must stay a small fraction of
// the arena or the test measures capacity instead of the defect. Each thread
// holds one block at a time, so the cap is arena / (16 * block): on a 128-CPU
// host the unbounded rule picked 256 threads, which is 512 MiB live in a 1 GiB
// arena, and 17389 of 160000 blocks legitimately fell back. The same run at 32
// and 64 threads gives 99 and 221 — the failure tracked the live-set fraction,
// not the bug.
static int threadCount(size_t block) {
    // SMASH_TEST_THREADS reproduces a many-core host's thread count on a small
    // one, which is how the 128-CPU failures were diagnosed without one.
    if (const char* t = std::getenv("SMASH_TEST_THREADS"); t && std::atoi(t) > 0)
        return std::atoi(t);
    const long cpus = sysconf(_SC_NPROCESSORS_ONLN);
    long want = (cpus > 0 ? cpus : 2) * 8;
    if (want > 256) want = 256;
    const char* gib = std::getenv("SMASH_VM_GIB");
    const long arena_gib = gib ? std::atol(gib) : 16;
    const long by_arena =
        static_cast<long>((arena_gib > 0 ? arena_gib : 16) * (1u << 30) / (16 * block));
    if (want > by_arena) want = by_arena;
    return static_cast<int>(want < 4 ? 4 : want);
}

// PHASE A: concurrent churn of blocks above kLargeAllocVmThreshold.
static void phaseA() {
    constexpr size_t kBlock = 2u << 20;
    constexpr long kTotal = 160000;
    const int threads = threadCount(kBlock);
    const long per_thread = kTotal / threads;
    std::atomic<long> out{0}, nulls{0};
    std::vector<std::thread> th;
    for (int t = 0; t < threads; ++t) {
        th.emplace_back([&] {
            long o = 0, n = 0;
            for (long i = 0; i < per_thread; ++i) {
                void* p = malloc(kBlock);
                if (!p) { ++n; continue; }
                if (!inArena(p)) ++o;
                *static_cast<volatile unsigned char*>(p) = 1;
                static_cast<volatile unsigned char*>(p)[kBlock - 1] = 1;
                free(p);
            }
            out += o;
            nulls += n;
        });
    }
    for (auto& t : th) t.join();

    // A bound rather than zero, for the reason numa_arena_churn_test gives: the
    // wait for in-flight frees is bounded, so under enough concurrency it can
    // expire with runs still in transit. 0.01% sits far above what the fixed
    // build produces and three orders of magnitude below the unfixed 29-90%.
    // The allowance comes from measurement. Fixed build, 160000 round trips of
    // 2 MiB through a 1 GiB arena: 70 at 4 threads, 84 at 8, 129 at 16, 174 at
    // 64, 99 at 32 — a few hundredths of a percent, flat in thread count, so it
    // is not the race. What is left is a bounded wait expiring and, for a
    // 512-page contiguous request, the free map not holding a run that long at
    // that instant. Unfixed the same runs gave 1728 / 46263 / 93726 / 144241,
    // i.e. 4% to 90%. 0.5% sits ~6x above the residual and ~60x below the
    // smallest unfixed figure.
    const long total = per_thread * threads;
    const long allowed = total / 200;
    if (out > allowed || nulls) {
        std::fprintf(stderr,
            "FAIL: phase A: %d threads x %ld round trips of 2 MiB gave %ld block(s) "
            "outside the compressible arena (allowed %ld of %ld) and %ld NULL — the "
            "shared arena is not waiting for freed runs that are still in flight\n",
            threads, per_thread, out.load(), allowed, total, nulls.load());
        ++g_failures;
    } else {
        std::printf("phase A: %d threads x %ld round trips of 2 MiB (live set %ld MiB), "
                    "%ld outside the arena (allowed %ld)\n", threads, per_thread,
                    static_cast<long>(threads) * static_cast<long>(kBlock) / (1 << 20),
                    out.load(), allowed);
    }
}

// PHASE B: large blocks must reclaim empty slab spans while slabs are busy.
// Full mode only — in large-only a 256-byte allocation goes to the system
// allocator, so the arena never fills with slab spans and the phase would pass
// while exercising nothing.
static void phaseB() {
    constexpr size_t kSmall = 256;
    constexpr long kSmallCap = 5000000;   // 1.28 GiB of payload vs a 1 GiB arena
    constexpr int kBig = 200;
    constexpr size_t kBigSize = 2u << 20;

    std::vector<void*> small;
    small.reserve(kSmallCap / 4);
    long small_out = 0;
    for (long i = 0; i < kSmallCap && small_out < 1000; ++i) {
        void* p = malloc(kSmall);
        if (!p) break;
        if (!inArena(p)) ++small_out;   // slab spans have started coming from mmap
        *static_cast<volatile unsigned char*>(p) = 1;
        small.push_back(p);
    }
    if (small_out == 0) {
        std::fprintf(stderr,
            "FAIL: phase B never exhausted the arena (%zu small objects all fit), so "
            "it would pass without exercising the reclaim path at all.\n", small.size());
        ++g_failures;
    }

    // Control, with the small objects still LIVE: the arena is full and the
    // hook has nothing to give, so large blocks must mostly fall back. Without
    // this, a phase B that passes proves only that the arena had room.
    long ctrl_out = 0;
    std::vector<void*> ctrl;
    for (int i = 0; i < kBig; ++i) {
        void* p = malloc(kBigSize);
        if (!p) continue;
        if (!inArena(p)) ++ctrl_out;
        ctrl.push_back(p);
    }
    for (void* p : ctrl) free(p);
    if (ctrl_out * 2 < kBig) {
        std::fprintf(stderr,
            "FAIL: phase B control: only %ld of %d large blocks fell back while the "
            "arena was full of live small objects — the arena is not actually "
            "exhausted, so the reclaim check below would prove nothing\n",
            ctrl_out, kBig);
        ++g_failures;
    }

    // Now free them: the pages are held as empty slab spans, which only the
    // exhausted hook can hand back.
    for (void* p : small) free(p);
    small.clear();
    small.shrink_to_fit();

    std::atomic<bool> stop{false};
    std::vector<std::thread> noisy;
    const int kNoisy = 8;
    for (int t = 0; t < kNoisy; ++t) {
        noisy.emplace_back([&] {
            std::vector<void*> keep;
            while (!stop.load(std::memory_order_relaxed)) {
                for (int c = 0; c < 8; ++c) {
                    void* p = malloc(size_t{64} << c);
                    if (!p) continue;
                    *static_cast<volatile unsigned char*>(p) = 1;
                    if (keep.size() < 64) keep.push_back(p); else free(p);
                }
            }
            for (void* p : keep) free(p);
        });
    }

    long big_out = 0, big_null = 0;
    std::vector<void*> big;
    for (int i = 0; i < kBig; ++i) {
        void* p = malloc(kBigSize);
        if (!p) { ++big_null; continue; }
        if (!inArena(p)) ++big_out;
        *static_cast<volatile unsigned char*>(p) = 1;
        big.push_back(p);
    }
    stop = true;
    for (auto& t : noisy) t.join();
    for (void* p : big) free(p);

    if (big_out || big_null) {
        std::fprintf(stderr,
            "FAIL: phase B: %ld of %d large blocks landed outside the arena and %ld "
            "were NULL while %d threads churned small objects — empty slab spans are "
            "not reaching the arena, check reclaimEmptySpans()'s tryLock skip\n",
            big_out, kBig, big_null, kNoisy);
        ++g_failures;
    } else {
        std::printf("phase B: arena exhausted (control: %ld of %d fell back), then "
                    "%d large blocks all reclaimed from empty spans with %d noisy "
                    "threads\n", ctrl_out, kBig, kBig, kNoisy);
    }
}

// The two phases run as SEPARATE ctest registrations, selected by
// SMASH_TEST_PHASE, rather than one after the other in one process.
//
// They cannot share a process: phase A's 160000 round trips of 2 MiB leave the
// free map without the long runs phase B needs (measured 171 of 200 outside
// the arena, against 0 of 200 for the same phase in isolation), and phase B's
// leftovers fragment the arena for phase A (its residual went from a steady
// 70-174 to an occasional 1293).
//
// And they cannot be split with fork(), which was the first attempt: smash's
// own background threads can hold a slab lock at the instant of the fork, and
// the child inherits it locked by a thread that does not exist there, so
// Slab::reclaimEmptySpans()'s tryLock can never succeed for that slab. Phase B
// then failed 3 runs in 6 with ~170 of 200 blocks outside the arena, while the
// identical workload in its own process passes every time. That looked exactly
// like the tryLock defect it was written to catch, which is the trap: the
// harness reproduced the symptom of the bug it was hunting.
int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    g_in_arena = reinterpret_cast<in_arena_fn>(dlsym(RTLD_DEFAULT, "smash_in_arena"));
    if (!g_in_arena) {
        std::printf("SKIP: smash_in_arena not found (not running under libsmash)\n");
        return 0;
    }
    const char* lo = std::getenv("SMASH_LARGE_ONLY");
    const bool large_only = (lo && lo[0] == '1');
    const char* phase = std::getenv("SMASH_TEST_PHASE");
    const char which = (phase && phase[0]) ? phase[0] : 'A';
    std::printf("shared_arena_churn_test: phase=%c mode=%s, %ld online CPU(s)\n",
                which, large_only ? "large-only" : "full",
                sysconf(_SC_NPROCESSORS_ONLN));

    if (which == 'B') {
        if (large_only) {
            std::printf("SKIP: phase B needs full mode — a 256-byte allocation goes to "
                        "the system allocator in large-only, so the arena never fills "
                        "with slab spans\n");
            return 0;
        }
        phaseB();
    } else {
        phaseA();
    }

    if (g_failures) {
        std::fprintf(stderr, "FAIL: %d check(s) failed\n", g_failures);
        return 1;
    }
    std::printf("PASS: phase %c\n", which);
    return 0;
}
