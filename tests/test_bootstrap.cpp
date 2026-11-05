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

// Growing past the initial region: expandAndAllocate() holds expand_lock_ and
// commits the new region's first pages. ensureCommitted() used to take
// expand_lock_ again there (Spinlock is not recursive), so the first
// expansion self-deadlocked -- and every other thread needing a commit
// spun behind it. Allocate well past kBootstrapInitialSize, single-threaded
// and then from several threads at once, and touch every block.
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
