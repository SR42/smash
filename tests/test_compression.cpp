// Test end-to-end compression: compress cold pages, decompress on fault
#include "vm/vm_region.h"
#include "vm/page_state.h"
#include "compress/compress_store.h"
#include "compress/compress_engine.h"
#include "compress/compressor_thread.h"
#include "smash/config.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>

using namespace smash;

static int failures = 0;

#define CHECK(cond, msg, ...) do { \
    if (!(cond)) { \
        fprintf(stderr, "FAIL: " msg "\n", ##__VA_ARGS__); \
        ++failures; \
    } \
} while (0)

static void testCompressEngine() {
    CompressEngine engine;
    engine.init();

    // Compress a buffer of repeated bytes (highly compressible)
    auto* src = static_cast<char*>(
        BootstrapAlloc::instance().allocate(kPageSize, kPageSize));
    memset(src, 0x42, kPageSize);

    size_t max_comp = CompressEngine::maxCompressedSize(kPageSize);
    auto* dst = static_cast<char*>(
        BootstrapAlloc::instance().allocate(max_comp, 16));

    size_t comp_size = engine.compress(src, dst, kPageSize, max_comp);
    CHECK(comp_size > 0, "compression failed");
    CHECK(comp_size < kPageSize, "compressed size %zu >= original %zu", comp_size, kPageSize);

    // Decompress and verify
    auto* restored = static_cast<char*>(
        BootstrapAlloc::instance().allocate(kPageSize, kPageSize));
    size_t decomp_size = engine.decompress(dst, restored, comp_size, kPageSize);
    CHECK(decomp_size == kPageSize, "decompressed size %zu != %zu", decomp_size, kPageSize);
    CHECK(memcmp(src, restored, kPageSize) == 0, "decompressed data doesn't match original");
}

static void testCompressStore() {
    CompressStore store;
    store.init();

    // Store some data
    char data[256];
    memset(data, 0xBB, sizeof(data));

    size_t alloc_size = 0;
    void* stored = store.store(data, sizeof(data), &alloc_size);
    CHECK(stored != nullptr, "store failed");
    CHECK(alloc_size >= sizeof(data), "alloc_size %zu < data size %zu", alloc_size, sizeof(data));
    CHECK(memcmp(stored, data, sizeof(data)) == 0, "stored data doesn't match");

    // Release
    store.release(stored, alloc_size);

    // Store again (should reuse from free list)
    void* stored2 = store.store(data, sizeof(data), &alloc_size);
    CHECK(stored2 != nullptr, "second store failed");
    store.release(stored2, alloc_size);
}

static void testCompressorRoundtrip() {
    // Set up all components
    VmRegion vm;
    bool ok = vm.init(64 * kPageSize);
    CHECK(ok, "VmRegion init failed");

    PageStateTable states;
    states.init(vm.totalPages());

    PageLockTable locks;
    locks.init(vm.totalPages());

    CompressStore store;
    store.init();

    CompressEngine engine;
    engine.init();

    CompressorThread compressor;
    compressor.init(&vm, &states, &locks, &store, &engine);

    // Allocate a page and fill with compressible data
    void* page = vm.allocatePages(1);
    CHECK(page != nullptr, "page alloc failed");

    size_t page_idx = vm.pageIndex(reinterpret_cast<uintptr_t>(page));
    states.set(page_idx, PageState::ACTIVE);

    // Fill with a repeating pattern (highly compressible)
    memset(page, 0x42, kPageSize);

    // Tick 1: cold_count goes to 1, page becomes ACTIVE_MONITORING (PROT_READ)
    compressor.compressTick();
    CHECK(states.get(page_idx) == PageState::ACTIVE_MONITORING,
          "after tick 1: expected ACTIVE_MONITORING, got %d",
          static_cast<int>(states.get(page_idx)));

    // Tick 2: cold_count reaches kColdTicks, two-level monitoring escalates
    // to PROT_NONE (deep monitoring) to detect reads as well as writes
    compressor.compressTick();

    // Tick 3: survived deep monitoring, truly cold — compress
    compressor.compressTick();
    CHECK(states.get(page_idx) == PageState::COMPRESSED || states.get(page_idx) == PageState::COMPRESSED_SHADOW,
          "after tick 3: expected COMPRESSED, got %d",
          static_cast<int>(states.get(page_idx)));

    // Simulate a fault: decompress the page
    bool handled = compressor.handleFault(reinterpret_cast<uintptr_t>(page));
    CHECK(handled, "handleFault should return true for compressed page");
    CHECK(states.get(page_idx) == PageState::ACTIVE,
          "after fault: expected ACTIVE, got %d",
          static_cast<int>(states.get(page_idx)));

    // Verify data integrity after decompression
    auto* bytes = static_cast<uint8_t*>(page);
    bool data_ok = true;
    for (size_t i = 0; i < kPageSize; ++i) {
        if (bytes[i] != 0x42) {
            data_ok = false;
            fprintf(stderr, "FAIL: data mismatch at offset %zu: got 0x%02x expected 0x42\n",
                    i, bytes[i]);
            break;
        }
    }
    CHECK(data_ok, "decompressed data doesn't match original");
}

static void testIncompressibleData() {
    VmRegion vm;
    vm.init(64 * kPageSize);

    PageStateTable states;
    states.init(vm.totalPages());

    PageLockTable locks;
    locks.init(vm.totalPages());

    CompressStore store;
    store.init();

    CompressEngine engine;
    engine.init();

    CompressorThread compressor;
    compressor.init(&vm, &states, &locks, &store, &engine);

    // Allocate a page and fill with pseudo-random data (incompressible)
    void* page = vm.allocatePages(1);
    CHECK(page != nullptr, "page alloc failed");

    size_t page_idx = vm.pageIndex(reinterpret_cast<uintptr_t>(page));
    states.set(page_idx, PageState::ACTIVE);

    // Fill with pseudo-random bytes
    auto* bytes = static_cast<uint8_t*>(page);
    uint32_t seed = 12345;
    for (size_t i = 0; i < kPageSize; ++i) {
        seed = seed * 1103515245 + 12345;
        bytes[i] = static_cast<uint8_t>(seed >> 16);
    }

    // Tick three times to attempt compression (two-level monitoring requires 3)
    compressor.compressTick();
    compressor.compressTick();
    compressor.compressTick();

    // With random data, compression ratio likely exceeds kMinCompressRatio,
    // so the page should NOT be compressed. After failed compression, tick's
    // Phase 3 sets remaining ACTIVE pages to ACTIVE_MONITORING.
    PageState st = states.get(page_idx);
    CHECK(st != PageState::COMPRESSED,
          "incompressible page should NOT be COMPRESSED, got %d", static_cast<int>(st));

    // Verify data is still intact
    seed = 12345;
    bool data_ok = true;
    for (size_t i = 0; i < kPageSize; ++i) {
        seed = seed * 1103515245 + 12345;
        uint8_t expected = static_cast<uint8_t>(seed >> 16);
        if (bytes[i] != expected) {
            data_ok = false;
            break;
        }
    }
    CHECK(data_ok, "data corrupted after failed compression attempt");
}

static void testAccessTracking() {
    // Walks the tick-by-tick schedule of a page written once right after it
    // was armed. The re-arm backoff (testRearmBackoff) would hold that page
    // unarmed for two ticks; this test is about the fault resetting the cold
    // count, so it runs without it. Read by init().
    setenv("SMASH_REARM_BACKOFF_MAX", "0", 1);
    VmRegion vm;
    vm.init(64 * kPageSize);

    PageStateTable states;
    states.init(vm.totalPages());

    PageLockTable locks;
    locks.init(vm.totalPages());

    CompressStore store;
    store.init();

    CompressEngine engine;
    engine.init();

    CompressorThread compressor;
    compressor.init(&vm, &states, &locks, &store, &engine);

    // Allocate a page
    void* page = vm.allocatePages(1);
    CHECK(page != nullptr, "page alloc failed");

    size_t page_idx = vm.pageIndex(reinterpret_cast<uintptr_t>(page));
    states.set(page_idx, PageState::ACTIVE);
    memset(page, 0x42, kPageSize);

    // Tick once: page becomes ACTIVE_MONITORING
    compressor.compressTick();
    CHECK(states.get(page_idx) == PageState::ACTIVE_MONITORING,
          "expected ACTIVE_MONITORING");

    // Simulate a write fault (user writes to monitored page)
    bool handled = compressor.handleFault(reinterpret_cast<uintptr_t>(page));
    CHECK(handled, "handleFault for ACTIVE_MONITORING should succeed");
    CHECK(states.get(page_idx) == PageState::ACTIVE,
          "after write fault: expected ACTIVE, got %d",
          static_cast<int>(states.get(page_idx)));

    // Now tick four more times — the fault set accessed_=true, so:
    // Tick 1: accessed_=true → cold_count stays 0, cleared. Phase 3: → ACTIVE_MONITORING
    // Tick 2: accessed_=false → cold_count=1. Phase 2: skip (< kColdTicks)
    // Tick 3: accessed_=false → cold_count=2 (== kColdTicks). Phase 2: escalate to deep monitoring
    // Tick 4: accessed_=false → cold_count=3 (> kColdTicks). Phase 2: compress!
    compressor.compressTick();
    CHECK(states.get(page_idx) == PageState::ACTIVE_MONITORING,
          "after tick 1: expected ACTIVE_MONITORING");

    compressor.compressTick();
    // cold_count=1, not yet compressed
    CHECK(states.get(page_idx) != PageState::COMPRESSED,
          "after tick 2: should not be COMPRESSED yet");

    compressor.compressTick();
    // cold_count=2, deep monitoring escalation — not yet compressed
    CHECK(states.get(page_idx) != PageState::COMPRESSED,
          "after tick 3: should not be COMPRESSED yet (deep monitoring)");

    compressor.compressTick();
    // cold_count=3, survived deep monitoring — should now be compressed
    CHECK(states.get(page_idx) == PageState::COMPRESSED || states.get(page_idx) == PageState::COMPRESSED_SHADOW,
          "after 4 ticks without access: expected COMPRESSED, got %d",
          static_cast<int>(states.get(page_idx)));

    // Decompress and verify
    handled = compressor.handleFault(reinterpret_cast<uintptr_t>(page));
    CHECK(handled, "handleFault for COMPRESSED should succeed");

    auto* bytes = static_cast<uint8_t*>(page);
    bool ok = true;
    for (size_t i = 0; i < kPageSize; ++i) {
        if (bytes[i] != 0x42) { ok = false; break; }
    }
    CHECK(ok, "data mismatch after access tracking round-trip");
}

// Re-arm backoff (page-protection write tracking): a page written within a
// tick of being armed waits 2^exp ticks unarmed, exp rising per hot fault up
// to SMASH_REARM_BACKOFF_MAX, and is never counted idle (so never compressed)
// while it waits.
static void testRearmBackoff() {
    setenv("SMASH_REARM_BACKOFF_MAX", "2", 1);
    VmRegion vm;
    vm.init(64 * kPageSize);
    PageStateTable states;
    states.init(vm.totalPages());
    PageLockTable locks;
    locks.init(vm.totalPages());
    CompressStore store;
    store.init();
    CompressEngine engine;
    engine.init();
    CompressorThread compressor;
    compressor.init(&vm, &states, &locks, &store, &engine);

    void* page = vm.allocatePages(1);
    CHECK(page != nullptr, "page alloc failed");
    const size_t page_idx = vm.pageIndex(reinterpret_cast<uintptr_t>(page));
    states.set(page_idx, PageState::ACTIVE);
    memset(page, 0x42, kPageSize);

    compressor.compressTick();
    CHECK(states.get(page_idx) == PageState::ACTIVE_MONITORING, "backoff: first arm");

    // Hot fault #1 -> wait 2 ticks; hot fault #2 -> 4; #3 stays at the cap, 4.
    const int waits[] = {2, 4, 4};
    for (int round = 0; round < 3; ++round) {
        CHECK(compressor.handleFault(reinterpret_cast<uintptr_t>(page)),
              "backoff: write fault %d not handled", round);
        for (int t = 0; t < waits[round]; ++t) {
            compressor.compressTick();
            CHECK(states.get(page_idx) == PageState::ACTIVE,
                  "backoff round %d tick %d: expected ACTIVE (waiting), got %d", round, t,
                  static_cast<int>(states.get(page_idx)));
        }
        compressor.compressTick();
        CHECK(states.get(page_idx) == PageState::ACTIVE_MONITORING,
              "backoff round %d: expected re-armed after %d ticks, got %d", round,
              waits[round], static_cast<int>(states.get(page_idx)));
    }

    // Left alone, the page still goes cold and is compressed.
    bool compressed = false;
    for (int t = 0; t < 12 && !compressed; ++t) {
        compressor.compressTick();
        const PageState st = states.get(page_idx);
        compressed = st == PageState::COMPRESSED || st == PageState::COMPRESSED_SHADOW;
    }
    CHECK(compressed, "backoff: page never compressed after its writes stopped");
    CHECK(compressor.handleFault(reinterpret_cast<uintptr_t>(page)), "backoff: restore fault");
    CHECK(static_cast<uint8_t*>(page)[kPageSize - 1] == 0x42, "backoff: data mismatch");
    unsetenv("SMASH_REARM_BACKOFF_MAX");
}

static void testReleaseCompressedPages() {
    VmRegion vm;
    vm.init(64 * kPageSize);

    PageStateTable states;
    states.init(vm.totalPages());

    PageLockTable locks;
    locks.init(vm.totalPages());

    CompressStore store;
    store.init();

    CompressEngine engine;
    engine.init();

    CompressorThread compressor;
    compressor.init(&vm, &states, &locks, &store, &engine);

    // Allocate, compress, then release
    void* page = vm.allocatePages(1);
    CHECK(page != nullptr, "page alloc failed");

    size_t page_idx = vm.pageIndex(reinterpret_cast<uintptr_t>(page));
    states.set(page_idx, PageState::ACTIVE);
    memset(page, 0x42, kPageSize);

    // Compress it (3 ticks: monitoring + deep monitoring + compress)
    compressor.compressTick();
    compressor.compressTick();
    compressor.compressTick();
    CHECK(states.get(page_idx) == PageState::COMPRESSED || states.get(page_idx) == PageState::COMPRESSED_SHADOW,
          "expected COMPRESSED");

    // Release the compressed page (simulates span deallocation)
    compressor.releaseCompressedPages(page_idx, 1);
    CHECK(states.get(page_idx) == PageState::EMPTY,
          "after release: expected EMPTY, got %d",
          static_cast<int>(states.get(page_idx)));
}

int main() {
    testCompressEngine();
    testCompressStore();
    testCompressorRoundtrip();
    testIncompressibleData();
    testAccessTracking();
    unsetenv("SMASH_REARM_BACKOFF_MAX");
    testRearmBackoff();
    testReleaseCompressedPages();

    if (failures == 0) {
        fprintf(stderr, "compression: all tests passed\n");
        return 0;
    } else {
        fprintf(stderr, "compression: %d failures\n", failures);
        return 1;
    }
}
