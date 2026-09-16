// Test bootstrap allocator
#include "core/bootstrap_alloc.h"
#include "smash/config.h"

#include <cstdio>
#include <cstring>

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

int main() {
    testBasicAlloc();
    testAlignment();
    testManyAllocations();
    // Last: it consumes the initial region on purpose.
    testGrowPastFirstRegion();

    if (failures == 0) {
        fprintf(stderr, "bootstrap: all tests passed\n");
        return 0;
    } else {
        fprintf(stderr, "bootstrap: %d failures\n", failures);
        return 1;
    }
}
