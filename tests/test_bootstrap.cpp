// Test bootstrap allocator
#include "core/bootstrap_alloc.h"
#include "smash/config.h"

#include <csignal>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>
#include <unistd.h>

using namespace smash;

static int failures = 0;

#define CHECK(cond, msg, ...) do { \
    if (!(cond)) { \
        fprintf(stderr, "FAIL: " msg "\n", ##__VA_ARGS__); \
        ++failures; \
    } \
} while (0)

// Growing past the initial region must not deadlock.
//
// expandAndAllocate() holds expand_lock_ and calls tryAllocFrom(). While
// tryAllocFrom() also called ensureCommitted() — which took that same
// non-recursive spinlock to mprotect the next pages — the first allocation
// that had to grow the arena spun forever holding the lock, wedging every
// other thread behind it. Regions are mapped read/write up front now, so the
// allocation path takes no lock at all.
//
// This is a pure allocation-volume trigger, not a concurrency one: a single
// thread reaches it. Under smash the arena fills with 92 KiB ThreadCaches
// (~728 of them exhaust the 64 MiB initial region); here we just allocate
// past it directly. Before the fix this function hangs; after it, it returns
// in milliseconds.
static void testGrowPastFirstRegion() {
    auto& ba = BootstrapAlloc::instance();
    constexpr size_t kChunk = 1024 * 1024;                       // 1 MiB a time
    const size_t chunks = (kBootstrapInitialSize / kChunk) + 8;  // past the end

    void* first = nullptr;
    void* last = nullptr;
    for (size_t i = 0; i < chunks; ++i) {
        void* p = ba.allocate(kChunk, 16);
        CHECK(p != nullptr, "allocation %zu of %zu failed while growing", i, chunks);
        if (!p) return;
        // Touch both ends: the pages are mapped read/write but only backed on
        // first touch, so this also proves the new region is writable.
        static_cast<char*>(p)[0] = 1;
        static_cast<char*>(p)[kChunk - 1] = 2;
        CHECK(ba.owns(p), "grown region's pointer %p not owned", p);
        if (!first) first = p;
        last = p;
    }
    CHECK(first != last, "expected several distinct allocations");
    fprintf(stderr, "bootstrap: grew past the %zu MiB initial region (%zu MiB allocated)\n",
            kBootstrapInitialSize / (1024 * 1024), (chunks * kChunk) / (1024 * 1024));
}

static void testBasicAlloc() {
    auto& ba = BootstrapAlloc::instance();

    void* p1 = ba.allocate(64);
    CHECK(p1 != nullptr, "first allocation failed");
    CHECK(ba.owns(p1), "should own allocated ptr");

    void* p2 = ba.allocate(128);
    CHECK(p2 != nullptr, "second allocation failed");
    CHECK(p2 != p1, "allocations should be distinct");
    CHECK(ba.owns(p2), "should own second ptr");

    // Should be able to write to allocated memory
    memset(p1, 0xAA, 64);
    memset(p2, 0xBB, 128);

    // Verify ownership check
    int stack_var = 42;
    CHECK(!ba.owns(&stack_var), "should not own stack memory");
    CHECK(!ba.owns(nullptr), "should not own nullptr");
}

static void testAlignment() {
    auto& ba = BootstrapAlloc::instance();

    for (size_t align = 16; align <= 4096; align *= 2) {
        void* p = ba.allocate(64, align);
        CHECK(p != nullptr, "aligned alloc failed for align=%zu", align);
        CHECK(reinterpret_cast<uintptr_t>(p) % align == 0,
              "alignment %zu not satisfied", align);
    }
}

static void testManyAllocations() {
    auto& ba = BootstrapAlloc::instance();

    // Allocate many small blocks
    constexpr int N = 10000;
    void* ptrs[N];
    for (int i = 0; i < N; ++i) {
        ptrs[i] = ba.allocate(32);
        CHECK(ptrs[i] != nullptr, "allocation %d failed", i);
    }

    // All should be owned
    for (int i = 0; i < N; ++i) {
        CHECK(ba.owns(ptrs[i]), "allocation %d not owned", i);
    }

    // All should be distinct
    for (int i = 0; i < N; ++i) {
        for (int j = i + 1; j < N && j < i + 10; ++j) {
            CHECK(ptrs[i] != ptrs[j], "allocations %d and %d collide", i, j);
        }
    }
}

// The metadata arena must scale with the heap. Every slab span costs a
// descriptor plus a bitmap here, so its capacity bounds how large a heap smash
// can manage: when it runs out, allocations fail (and before the null checks,
// the process died dereferencing null — reported at ~169 GB RSS). Fixed
// 16 MiB expansions capped it at 64 MiB + 63 x 16 MiB = 1.07 GiB, roughly a
// 100 GB heap. Regions now double up to kBootstrapMaxExpandSize.
static void testCapacityScales() {
    size_t cap = smash::kBootstrapInitialSize;
    for (int i = 1; i < smash::kBootstrapMaxRegions; ++i) {
        size_t r = smash::kBootstrapExpandSize << (i - 1 < 4 ? i - 1 : 4);
        if (r > smash::kBootstrapMaxExpandSize) r = smash::kBootstrapMaxExpandSize;
        cap += r;
    }
    const size_t kWant = size_t{8} << 30;   // 8 GiB of metadata
    CHECK(cap >= kWant,
          "metadata arena capacity is only %zu MiB; with ~112 bytes per slab span "
          "that bounds the heap to roughly %zu GB. Regions must keep growing "
          "geometrically (kBootstrapMaxExpandSize).",
          cap >> 20, (cap / 112) * 8 * 4096 / 1000000000);
    printf("  metadata arena capacity: %zu MiB across %d regions\n",
           cap >> 20, smash::kBootstrapMaxRegions);
}

// Growing past the initial region (from upstream #92): expandAndAllocate()
// holds expand_lock_ while it adds a region and allocates from it. A commit
// step that took expand_lock_ again there (Spinlock is not recursive)
// self-deadlocked the first expansion, and every other thread spun behind it.
// Here regions are mapped read/write and have no commit step (see addRegion),
// so this guards against one coming back. Allocate well past
// kBootstrapInitialSize, single-threaded and then from several threads at
// once, and touch every block; the watchdog turns a hang into a failure.
static void testExpansion() {
    auto& ba = BootstrapAlloc::instance();
    constexpr size_t kChunk = 1u << 20;
    const size_t n = (kBootstrapInitialSize + 2 * kBootstrapExpandSize) / kChunk + 4;
    for (size_t i = 0; i < n; ++i) {
        auto* p = static_cast<unsigned char*>(ba.allocate(kChunk));
        CHECK(p != nullptr, "expansion allocation %zu failed", i);
        if (!p) return;
        p[0] = 1;
        p[kChunk - 1] = 2;
        CHECK(ba.owns(p), "expansion allocation %zu not owned", i);
    }
}

static void testConcurrentExpansion() {
    auto& ba = BootstrapAlloc::instance();
    std::vector<std::thread> ts;
    for (int t = 0; t < 4; ++t) {
        ts.emplace_back([&ba] {
            for (int i = 0; i < 40; ++i) {
                auto* p = static_cast<unsigned char*>(ba.allocate(1u << 20));
                if (!p) { ++failures; return; }
                p[0] = 1;
                p[(1u << 20) - 1] = 2;
            }
        });
    }
    for (auto& t : ts) t.join();
}

int main() {
    // A deadlock must fail the test, not hang CI until its timeout.
    signal(SIGALRM, [](int) {
        static const char m[] = "FAIL: bootstrap test timed out (deadlock?)\n";
        (void)!write(2, m, sizeof(m) - 1);
        _exit(2);
    });
    alarm(20);

    testBasicAlloc();
    testAlignment();
    testManyAllocations();
    // Last: these consume the initial region on purpose.
    testGrowPastFirstRegion();
    testCapacityScales();
    testExpansion();
    testConcurrentExpansion();

    if (failures == 0) {
        fprintf(stderr, "bootstrap: all tests passed\n");
        return 0;
    } else {
        fprintf(stderr, "bootstrap: %d failures\n", failures);
        return 1;
    }
}
