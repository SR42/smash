// Test span bitmap-based allocation
#include "core/span.h"
#include "core/size_classes.h"
#include "core/bootstrap_alloc.h"
#include "vm/platform_mem.h"
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

static void testSpanAllocFree() {
    // Test with a small size class (class 0 = 16 bytes)
    const auto& info = kSizeClasses[0];
    size_t span_bytes = info.pages * kPageSize;
    void* mem = vm::mapPages(span_bytes);
    CHECK(mem != nullptr, "mmap failed");

    Span* span = newSpanDescriptor();
    span->init(mem, info.pages, 0);

    CHECK(!span->full(), "new span should not be full");
    CHECK(span->empty(), "new span should be empty");
    CHECK(span->object_count == info.objects, "object count mismatch: got %u, expected %u",
          span->object_count, info.objects);

    // Allocate all objects
    void* ptrs[info.objects];
    for (uint32_t i = 0; i < info.objects; ++i) {
        ptrs[i] = span->allocate();
        CHECK(ptrs[i] != nullptr, "allocation %u failed", i);
        CHECK(span->contains(ptrs[i]), "allocation %u out of span bounds", i);
    }

    CHECK(span->full(), "span should be full after allocating all objects");
    CHECK(span->allocate() == nullptr, "allocation from full span should fail");

    // Free all objects
    for (uint32_t i = 0; i < info.objects; ++i) {
        span->deallocate(ptrs[i]);
    }

    CHECK(span->empty(), "span should be empty after freeing all objects");

    vm::unmapPages(mem, span_bytes);
}

static void testSpanNoDuplicates() {
    const auto& info = kSizeClasses[3]; // 64 bytes
    size_t span_bytes = info.pages * kPageSize;
    void* mem = vm::mapPages(span_bytes);
    CHECK(mem != nullptr, "mmap failed");

    Span* span = newSpanDescriptor();
    span->init(mem, info.pages, 3);

    // Allocate all and verify uniqueness
    void* ptrs[info.objects];
    for (uint32_t i = 0; i < info.objects; ++i) {
        ptrs[i] = span->allocate();
        for (uint32_t j = 0; j < i; ++j) {
            CHECK(ptrs[i] != ptrs[j], "duplicate allocation at %u and %u", i, j);
        }
    }

    vm::unmapPages(mem, span_bytes);
}

static void testSpanAllocFreeCycle() {
    const auto& info = kSizeClasses[1]; // 32 bytes
    size_t span_bytes = info.pages * kPageSize;
    void* mem = vm::mapPages(span_bytes);
    CHECK(mem != nullptr, "mmap failed");

    Span* span = newSpanDescriptor();
    span->init(mem, info.pages, 1);

    // Allocate half, free half, allocate again
    uint32_t half = info.objects / 2;
    void* ptrs[info.objects];

    for (uint32_t i = 0; i < half; ++i) {
        ptrs[i] = span->allocate();
        CHECK(ptrs[i] != nullptr, "initial alloc %u failed", i);
    }

    CHECK(!span->full(), "span shouldn't be full at half capacity");
    CHECK(!span->empty(), "span shouldn't be empty at half capacity");

    // Free them
    for (uint32_t i = 0; i < half; ++i) {
        span->deallocate(ptrs[i]);
    }
    CHECK(span->empty(), "span should be empty");

    // Allocate all again
    for (uint32_t i = 0; i < info.objects; ++i) {
        ptrs[i] = span->allocate();
        CHECK(ptrs[i] != nullptr, "re-allocation %u failed", i);
    }
    CHECK(span->full(), "span should be full");

    vm::unmapPages(mem, span_bytes);
}

// Span::slotOf() replaces offset / object_size with a reciprocal multiply;
// it must agree with the division for every byte offset of every class's
// span (and well beyond, up to 64 MiB, far past any span size).
static void testSlotOfMatchesDivision() {
    for (int sc = 0; sc < kNumClasses; ++sc) {
        Span span{};
        span.object_size = kSizeClasses[sc].size;
        span.div_magic = ((uint64_t{1} << Span::kDivShift) + span.object_size - 1)
                         / span.object_size;
        const size_t limit = size_t{64} << 20;
        size_t bad = 0;
        for (size_t off = 0; off < limit && bad == 0; ++off)
            if (span.slotOf(off) != off / span.object_size) bad = off + 1;
        CHECK(bad == 0, "slotOf mismatch for class %d (size %u) at offset %zu",
              sc, span.object_size, bad - 1);
    }
}

int main() {
    testSlotOfMatchesDivision();
    testSpanAllocFree();
    testSpanNoDuplicates();
    testSpanAllocFreeCycle();

    if (failures == 0) {
        fprintf(stderr, "span: all tests passed\n");
        return 0;
    } else {
        fprintf(stderr, "span: %d failures\n", failures);
        return 1;
    }
}
