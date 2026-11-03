// Test end-to-end compression: compress cold pages, decompress on fault
#include "vm/vm_region.h"
#include "vm/page_state.h"
#include "compress/compress_store.h"
#include "compress/compress_engine.h"
#include "compress/compressor_thread.h"
#include "core/page_map.h"
#include "core/span.h"
#include "smash/config.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <unistd.h>

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

// Group restore (SMASH_RESTORE_GRANULE_PAGES): a write fault on a watched
// page makes its whole aligned group writable in one mprotect per contiguous
// stretch of ACTIVE / ACTIVE_MONITORING pages — and never touches a
// compressed or shadow neighbour, whose page must stay unwritable.
static bool mapsPerms(const void* addr, char out[5]) {
    FILE* f = fopen("/proc/self/maps", "r");
    if (!f) return false;
    const auto a = reinterpret_cast<uintptr_t>(addr);
    char line[512];
    bool found = false;
    while (fgets(line, sizeof(line), f)) {
        unsigned long lo = 0, hi = 0;
        char perms[5] = {};
        if (sscanf(line, "%lx-%lx %4s", &lo, &hi, perms) == 3 && a >= lo && a < hi) {
            memcpy(out, perms, 5);
            found = true;
            break;
        }
    }
    fclose(f);
    return found;
}

static void testRestoreGranule() {
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
    unsetenv("SMASH_REARM_BACKOFF_MAX");
    compressor.setRestoreGranulePages(1);   // one page per fault while setting up

    auto* mem = static_cast<uint8_t*>(vm.allocatePages(8));
    CHECK(mem != nullptr, "granule: alloc failed");
    const size_t idx0 = vm.pageIndex(reinterpret_cast<uintptr_t>(mem));
    const size_t base = (idx0 + 3) & ~size_t{3};          // a 4-aligned group inside
    auto pg = [&](size_t k) { return static_cast<uint8_t*>(vm.pageAddress(base + k)); };
    for (size_t i = 0; i < 8; ++i) {
        states.set(idx0 + i, PageState::ACTIVE);
        memset(mem + i * kPageSize, 0x40 + static_cast<int>(i), kPageSize);
    }
    const uint8_t v1 = pg(1)[0];
    compressor.compressTick();
    CHECK(states.get(base) == PageState::ACTIVE_MONITORING, "granule: first arm");

    // Page 1 goes cold and is compressed while 0, 2, 3 keep being written
    // (group restore off: one page per fault).
    PageState s1 = PageState::ACTIVE_MONITORING;
    for (int t = 0; t < 16; ++t) {
        for (size_t k : {0u, 2u, 3u})
            if (states.get(base + k) == PageState::ACTIVE_MONITORING)
                compressor.handleFault(reinterpret_cast<uintptr_t>(pg(k)));
        compressor.compressTick();
        s1 = states.get(base + 1);
        if (s1 == PageState::COMPRESSED || s1 == PageState::COMPRESSED_SHADOW) break;
    }
    CHECK(s1 == PageState::COMPRESSED || s1 == PageState::COMPRESSED_SHADOW,
          "granule: page 1 never compressed (state %d)", static_cast<int>(s1));
    compressor.compressTick();   // re-arm 0, 2, 3
    for (size_t k : {0u, 2u, 3u})
        CHECK(states.get(base + k) == PageState::ACTIVE_MONITORING,
              "granule: page %zu not re-armed (state %d)", k,
              static_cast<int>(states.get(base + k)));
    s1 = states.get(base + 1);

    compressor.setRestoreGranulePages(4);
    CHECK(compressor.handleFault(reinterpret_cast<uintptr_t>(pg(2))), "granule: fault on 2");
    for (size_t k : {0u, 2u, 3u}) {
        CHECK(states.get(base + k) == PageState::ACTIVE,
              "granule: page %zu not restored with its group (state %d)", k,
              static_cast<int>(states.get(base + k)));
        char perms[5] = {};
        CHECK(mapsPerms(pg(k), perms) && perms[1] == 'w',
              "granule: page %zu not writable after the group restore (%s)", k, perms);
    }
    CHECK(states.get(base + 1) == s1, "granule: compressed neighbour's state changed (%d -> %d)",
          static_cast<int>(s1), static_cast<int>(states.get(base + 1)));
    char perms1[5] = {};
    CHECK(mapsPerms(pg(1), perms1) && perms1[1] != 'w',
          "granule: compressed neighbour made writable (%s)", perms1);
    // Its contents come back through its own fault.
    if (states.get(base + 1) == PageState::COMPRESSED)
        compressor.handleFault(reinterpret_cast<uintptr_t>(pg(1)));
    CHECK(pg(1)[0] == v1 && pg(1)[kPageSize - 1] == v1, "granule: page 1 data mismatch");
    compressor.setRestoreGranulePages(1);
}

// Idle time in nominal ticks: a tick that took 5 s advances the cold count
// by 5, but stops at the cold floor so the page is still escalated (watched
// for reads) before it is compressed; and a shadow page 5 nominal ticks old
// is released at the next tick instead of after two.
static void testNominalTicks() {
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
    unsetenv("SMASH_REARM_BACKOFF_MAX");
    compressor.setRestoreGranulePages(1);

    auto* page = static_cast<uint8_t*>(vm.allocatePages(1));
    CHECK(page != nullptr, "nominal: alloc failed");
    const size_t idx = vm.pageIndex(reinterpret_cast<uintptr_t>(page));
    states.set(idx, PageState::ACTIVE);
    memset(page, 0x5c, kPageSize);

    compressor.compressTick();                       // armed
    CHECK(states.get(idx) == PageState::ACTIVE_MONITORING, "nominal: not armed");
    compressor.pretendNextTickAfterMs(5000);
    compressor.compressTick();                       // cold count jumps: stops at the floor
    char perms[5] = {};
    CHECK(states.get(idx) == PageState::ACTIVE_MONITORING,
          "nominal: after a 5 s tick the page was compressed (state %d) without the read watch",
          static_cast<int>(states.get(idx)));
    // Watched for reads either way: PROT_NONE, or as root a verified idle mark
    // on a PROT_READ page. Which one depends on whether the kernel took the
    // mark, i.e. whether the page has left the per-CPU LRU batch yet.
    CHECK(mapsPerms(page, perms) && (perms[0] == '-' || compressor.idleWatchedForTest(idx)),
          "nominal: after a 5 s tick the page is not watched for reads (%s, no idle mark)", perms);
    compressor.pretendNextTickAfterMs(5000);
    compressor.compressTick();                       // survived the read watch: compressed
    const PageState s3 = states.get(idx);
    CHECK(s3 == PageState::COMPRESSED_SHADOW || s3 == PageState::COMPRESSED,
          "nominal: not compressed after the read-watch tick (state %d)", static_cast<int>(s3));
    if (s3 == PageState::COMPRESSED_SHADOW) {
        compressor.pretendNextTickAfterMs(5000);
        compressor.compressTick();                   // 5 nominal ticks in shadow >= delay 2
        CHECK(states.get(idx) == PageState::COMPRESSED,
              "nominal: shadow 5 nominal ticks old not released (state %d)",
              static_cast<int>(states.get(idx)));
    }
    CHECK(compressor.handleFault(reinterpret_cast<uintptr_t>(page)), "nominal: restore fault");
    CHECK(page[0] == 0x5c && page[kPageSize - 1] == 0x5c, "nominal: data mismatch");
}

// Recompression-thrash back-off: a page faulted back from COMPRESSED must
// stay idle 2^rc times the floor before it is compressed again (rc = its
// recompress count, shift capped at kMaxBackoffShift), and the count must
// decay only once the page has stayed compressed -- one step per
// kRcDecayColdTicks cold ticks. With SMASH_RECOMPRESS_BACKOFF=0 every round
// takes the same time (test_compression_backoff_off).
//
// The decay used to run on idle ACTIVE pages too, i.e. during the very wait
// the back-off imposes: at the floor of 2 this test runs with, the waits were
// 4, 8, 8, 8 ticks instead of 4, 8, 16, 32, and at the default floor of 10 a
// page faulted back once waited no longer than one never compressed.
static int ticksUntilCompressed(CompressorThread& c, PageStateTable& states, size_t idx, int max) {
    for (int t = 1; t <= max; ++t) {
        c.compressTick();
        const PageState st = states.get(idx);
        if (st == PageState::COMPRESSED || st == PageState::COMPRESSED_SHADOW) return t;
    }
    return -1;
}

static void testRecompressBackoff() {
    const char* off_env = getenv("SMASH_RECOMPRESS_BACKOFF");
    const bool backoff = !(off_env && atoi(off_env) == 0);
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
    unsetenv("SMASH_REARM_BACKOFF_MAX");
    compressor.setRestoreGranulePages(1);

    auto* page = static_cast<uint8_t*>(vm.allocatePages(1));
    CHECK(page != nullptr, "recompress: alloc failed");
    const size_t idx = vm.pageIndex(reinterpret_cast<uintptr_t>(page));
    states.set(idx, PageState::ACTIVE);
    memset(page, 0x3d, kPageSize);
    const uint8_t* rc = compressor.recompressCounts();
    const int floor = static_cast<int>(ROIConfig::instance().cold_ticks_floor);

    CHECK(ticksUntilCompressed(compressor, states, idx, 40) > 0, "recompress: first compression never happened");
    // Ticks from a restore fault to the next compression, per recompress count.
    int wait[6] = {};
    for (int r = 1; r <= 5; ++r) {
        CHECK(compressor.handleFault(reinterpret_cast<uintptr_t>(page)), "recompress: restore fault %d", r);
        CHECK(page[0] == 0x3d && page[kPageSize - 1] == 0x3d, "recompress: data mismatch after fault %d", r);
        CHECK(rc[idx] == r, "recompress: count %d after fault %d", rc[idx], r);
        wait[r] = ticksUntilCompressed(compressor, states, idx, 400);
        fprintf(stderr, "recompress: rc=%d -> compressed again after %d ticks (floor %d, backoff %s)\n", r,
                wait[r], floor, backoff ? "on" : "off");
        CHECK(wait[r] > 0, "recompress: rc=%d never compressed again", r);
    }
    for (int r = 2; r <= 5; ++r) {
        // Each fault doubles the idle streak required, so the wait grows by
        // exactly the difference of the effective floors.
        const int want = backoff ? wait[1] + (floor << r) - (floor << 1) : wait[1];
        CHECK(wait[r] == want, "recompress: rc=%d waited %d ticks, want %d", r, wait[r], want);
    }
    CHECK(!backoff || wait[1] >= floor << 1, "recompress: rc=1 waited %d ticks, under 2x the floor of %d",
          wait[1], floor);

    // Left compressed, the count decays one step per kRcDecayColdTicks cold
    // ticks, to 0; the next fault then starts the schedule over.
    for (int t = 0; t < 5 * kRcDecayColdTicks + 8; ++t) compressor.compressTick();
    CHECK(states.get(idx) == PageState::COMPRESSED, "recompress: page left idle is not compressed (state %d)",
          static_cast<int>(states.get(idx)));
    CHECK(rc[idx] == 0, "recompress: count %d after %d compressed ticks, want 0", rc[idx],
          5 * kRcDecayColdTicks + 8);
    CHECK(compressor.handleFault(reinterpret_cast<uintptr_t>(page)), "recompress: restore fault after decay");
    CHECK(rc[idx] == 1, "recompress: count %d after the fault that followed the decay", rc[idx]);
    const int again = ticksUntilCompressed(compressor, states, idx, 400);
    CHECK(again == wait[1], "recompress: after decay waited %d ticks, want %d (the rc=1 wait)", again, wait[1]);
    CHECK(compressor.handleFault(reinterpret_cast<uintptr_t>(page)), "recompress: final restore fault");
    CHECK(page[0] == 0x3d && page[kPageSize - 1] == 0x3d, "recompress: data mismatch at the end");
}

// A restore of a page the allocator marked (it handed out a block there while
// the page was compressed: PageStateTable::markAllocReopened) must not count
// as recompression thrash, and must take the mark; any other restore must
// count. A mark on a neighbouring page changes nothing. Both restore paths:
// from COMPRESSED (decompress) and from COMPRESSED_SHADOW (the write fault on
// a page awaiting deferred reclaim).
static int ticksUntilState(CompressorThread& c, PageStateTable& states, size_t idx, PageState want, int max) {
    for (int t = 1; t <= max; ++t) {
        c.compressTick();
        if (states.get(idx) == want) return t;
    }
    return -1;
}

static void testAllocReopen() {
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
    unsetenv("SMASH_REARM_BACKOFF_MAX");
    compressor.setRestoreGranulePages(1);

    auto* mem = static_cast<uint8_t*>(vm.allocatePages(3));
    CHECK(mem != nullptr, "reopen: alloc failed");
    uint8_t* page = mem + kPageSize;
    const size_t idx = vm.pageIndex(reinterpret_cast<uintptr_t>(page));
    for (size_t k = 0; k < 3; ++k) states.set(idx - 1 + k, PageState::ACTIVE);
    memset(mem, 0x6b, 3 * kPageSize);
    const uint8_t* rc = compressor.recompressCounts();

    struct Case { const char* what; PageState at; long mark; int want_rc; };
    const Case cases[] = {
        {"compressed, page marked", PageState::COMPRESSED, 0, 0},
        {"compressed, nothing marked", PageState::COMPRESSED, -2, 1},
        {"compressed, next page marked", PageState::COMPRESSED, 1, 1},
        {"compressed, page marked again", PageState::COMPRESSED, 0, 0},
        {"shadow, page marked", PageState::COMPRESSED_SHADOW, 0, 0},
        {"shadow, nothing marked", PageState::COMPRESSED_SHADOW, -2, 1},
    };
    for (const Case& c : cases) {
        CHECK(ticksUntilState(compressor, states, idx, c.at, 400) > 0, "reopen: %s: page never reached state %d",
              c.what, static_cast<int>(c.at));
        const int before = rc[idx];
        if (c.mark > -2) states.markAllocReopened(idx + c.mark);
        CHECK(compressor.handleFault(reinterpret_cast<uintptr_t>(page)), "reopen: %s: restore fault", c.what);
        CHECK(rc[idx] - before == c.want_rc, "reopen: %s: recompress count went %d -> %d, want +%d", c.what, before,
              rc[idx], c.want_rc);
        CHECK(!states.takeAllocReopened(idx), "reopen: %s: the restore left the page's mark", c.what);
        CHECK(page[0] == 0x6b && page[kPageSize - 1] == 0x6b, "reopen: %s: data mismatch", c.what);
        if (c.mark == 1) (void)states.takeAllocReopened(idx + 1);
    }
}

// Phase B (reclaimShadowRange) must touch exactly the pages of its range.
// Worker ranges are cut at committed / workers, not on 64-page chunk
// boundaries, and the loop walked whole chunks from start / 64: a worker
// reclaimed the previous worker's pages of a shared chunk too, and its
// look-ahead, which arms a run PROT_READ without the pages' locks, could land
// after the other worker had made them PROT_NONE and dropped them -- COMPRESSED
// pages that read zeros without a fault (seen under load: whole runs of a
// test's objects read back as zeros, state 2). It also counted chunks from
// end - start, one short when start is mid-chunk, so the chunk holding the
// range's last pages was never visited. SMASH_DEFERRED_RECLAIM_DELAY=1000
// (test_compression_reclaim_range) keeps the ticks from reclaiming; the
// range runs with no delay.
static void testReclaimRange() {
    setenv("SMASH_REARM_BACKOFF_MAX", "0", 1);
    VmRegion vm;
    vm.init(512 * kPageSize);
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
    unsetenv("SMASH_REARM_BACKOFF_MAX");
    compressor.setRestoreGranulePages(1);

    auto* mem = static_cast<uint8_t*>(vm.allocatePages(256));
    CHECK(mem != nullptr, "reclaim range: alloc failed");
    const size_t first = vm.pageIndex(reinterpret_cast<uintptr_t>(mem));
    const size_t base = (first + 63) & ~size_t{63};   // a 64-page chunk inside the block
    CHECK(base + 128 <= first + 256, "reclaim range: no two whole chunks in the block");
    for (size_t k = 0; k < 256; ++k) {
        states.set(first + k, PageState::ACTIVE);
        memset(mem + k * kPageSize, static_cast<int>(0x11 + k % 200), kPageSize);
    }
    int shadow = 0;
    for (int t = 0; t < 40 && shadow < 128; ++t) {
        compressor.compressTick();
        shadow = 0;
        for (size_t k = 0; k < 128; ++k) shadow += states.get(base + k) == PageState::COMPRESSED_SHADOW;
    }
    CHECK(shadow == 128, "reclaim range: only %d of 128 pages became COMPRESSED_SHADOW", shadow);

    const size_t lo = base + 10, hi = base + 74;   // starts and ends mid-chunk
    compressor.reclaimShadowRangeForTest(lo, hi);
    int below = 0, inside = 0, above = 0;
    for (size_t k = base; k < base + 128; ++k) {
        const bool reclaimed = states.get(k) == PageState::COMPRESSED;
        if (k < lo) below += reclaimed;
        else if (k < hi) inside += reclaimed;
        else above += reclaimed;
    }
    fprintf(stderr, "reclaim range [+10, +74): reclaimed %d below, %d of 64 inside, %d above\n", below, inside,
            above);
    CHECK(below == 0, "reclaim range: %d pages below the range reclaimed", below);
    CHECK(inside == 64, "reclaim range: %d of 64 pages of the range reclaimed", inside);
    CHECK(above == 0, "reclaim range: %d pages above the range reclaimed", above);
    for (size_t k = 0; k < 256; ++k) {
        const size_t idx = first + k;
        const PageState st = states.get(idx);
        if (st == PageState::COMPRESSED || st == PageState::COMPRESSED_SHADOW)
            CHECK(compressor.handleFault(reinterpret_cast<uintptr_t>(mem + k * kPageSize)),
                  "reclaim range: restore of page %zu", k);
        CHECK(mem[k * kPageSize] == static_cast<uint8_t>(0x11 + k % 200) &&
              mem[k * kPageSize + kPageSize - 1] == static_cast<uint8_t>(0x11 + k % 200),
              "reclaim range: page %zu data mismatch", k);
    }
}

// The cold timeout setting: SMASH_COLD_TIMEOUT_SEC (seconds) wins over
// SMASH_COLD_TICKS (ticks); text that is not a positive number keeps the
// default with a warning; a value above kMaxColdTicks is clamped to it with a
// warning -- a page's idle count stops at 255, and Phase 2 compresses only
// above the floor, so a floor of 255 or more compressed nothing.
static void testColdTimeoutParse() {
    struct Case { const char* sec; const char* ticks; int want; bool overridden; const char* warning; };
    const Case cases[] = {
        {nullptr, nullptr, kColdTicksDefault, false, ""},
        {"3", nullptr, 3, true, ""},
        {"100", nullptr, 100, true, ""},
        {"2.4", nullptr, 2, true, ""},
        {"2.5", nullptr, 3, true, ""},
        {"0.2", nullptr, 1, true, ""},
        {"254", nullptr, 254, true, ""},
        {"254.4", nullptr, 254, true, ""},
        {"254.6", nullptr, kMaxColdTicks, true, "is above the maximum of 254 s"},
        {"255", nullptr, kMaxColdTicks, true, "is above the maximum of 254 s"},
        {"99999", nullptr, kMaxColdTicks, true, "SMASH_NO_COMPRESSOR=1"},
        {"1e12", nullptr, kMaxColdTicks, true, "is above the maximum"},
        {"5 ", nullptr, 5, true, ""},
        {"0", nullptr, kColdTicksDefault, false, "is not a positive number"},
        {"-3", nullptr, kColdTicksDefault, false, "is not a positive number"},
        {"", nullptr, kColdTicksDefault, false, "is not a positive number"},
        {"ten", nullptr, kColdTicksDefault, false, "is not a positive number"},
        {"10s", nullptr, kColdTicksDefault, false, "is not a positive number"},
        {"nan", nullptr, kColdTicksDefault, false, "is not a positive number"},
        {"inf", nullptr, kColdTicksDefault, false, "is not a positive number"},
        {"7", "2", 7, true, ""},
        {nullptr, "2", 2, true, ""},
        {nullptr, "300", kMaxColdTicks, true, "SMASH_COLD_TICKS=300 is above the maximum of 254 ticks"},
        {nullptr, "-1", kColdTicksDefault, false, "SMASH_COLD_TICKS=-1 is not a positive number"},
    };
    for (const Case& c : cases) {
        const ColdTicksSetting r = parseColdTicks(c.sec, c.ticks);
        const char* label = c.sec ? c.sec : c.ticks ? c.ticks : "(unset)";
        CHECK(r.ticks == c.want, "cold timeout %s: %d ticks, want %d", label, r.ticks, c.want);
        CHECK(r.overridden == c.overridden, "cold timeout %s: overridden=%d", label, r.overridden);
        if (c.warning[0])
            CHECK(strstr(r.warning, c.warning) != nullptr, "cold timeout %s: warning \"%s\" lacks \"%s\"", label,
                  r.warning, c.warning);
        else
            CHECK(r.warning[0] == '\0', "cold timeout %s: unexpected warning \"%s\"", label, r.warning);
    }
}

// SMASH_COLD_TICKS=300 (test_compression_cold_max): clamped to the maximum
// floor of 254, and a page idle that long is compressed. Unclamped, the floor
// was 300 and the page never was.
static void testColdTicksMax() {
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
    unsetenv("SMASH_REARM_BACKOFF_MAX");
    // ROIConfig reads the environment when the compressor initialises it.
    CHECK(ROIConfig::instance().cold_ticks_floor == static_cast<uint32_t>(kMaxColdTicks),
          "cold max: floor %u, want %d", ROIConfig::instance().cold_ticks_floor, kMaxColdTicks);
    auto* page = static_cast<uint8_t*>(vm.allocatePages(1));
    CHECK(page != nullptr, "cold max: alloc failed");
    const size_t idx = vm.pageIndex(reinterpret_cast<uintptr_t>(page));
    states.set(idx, PageState::ACTIVE);
    memset(page, 0x2e, kPageSize);
    const int t = ticksUntilCompressed(compressor, states, idx, 400);
    fprintf(stderr, "cold max: floor %u, compressed after %d ticks\n", ROIConfig::instance().cold_ticks_floor, t);
    CHECK(t > kMaxColdTicks && t <= kMaxColdTicks + 3, "cold max: compressed after %d ticks, want %d..%d", t,
          kMaxColdTicks + 1, kMaxColdTicks + 3);
    CHECK(compressor.handleFault(reinterpret_cast<uintptr_t>(page)), "cold max: restore fault");
    CHECK(page[0] == 0x2e && page[kPageSize - 1] == 0x2e, "cold max: data mismatch");
}

// Coarse watching (SMASH_WATCH_GROUP_PAGES): a group is idle only when all
// its pages are, so it is escalated and compressed in one tick; a fault on
// a compressed page restores the whole group; a group is not armed while one
// of its pages waits out a re-arm backoff.
static void testWatchGroup() {
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
    unsetenv("SMASH_REARM_BACKOFF_MAX");
    compressor.setRestoreGranulePages(1);
    compressor.setWatchGroupPages(4);

    auto* mem = static_cast<uint8_t*>(vm.allocatePages(12));
    CHECK(mem != nullptr, "group: alloc failed");
    const size_t idx0 = vm.pageIndex(reinterpret_cast<uintptr_t>(mem));
    const size_t base = (idx0 + 3) & ~size_t{3};               // a 4-aligned group
    auto pg = [&](size_t k) { return static_cast<uint8_t*>(vm.pageAddress(base + k)); };
    auto st = [&](size_t k) { return states.get(base + k); };
    auto compressed = [&](size_t k) {
        return st(k) == PageState::COMPRESSED || st(k) == PageState::COMPRESSED_SHADOW;
    };
    for (size_t i = 0; i < 12; ++i) {
        states.set(idx0 + i, PageState::ACTIVE);
        memset(mem + i * kPageSize, 0x30 + static_cast<int>(i), kPageSize);
    }

    // Page 2 is written every tick (after the first arming): none of the
    // group may be compressed, though pages 0, 1 and 3 are idle.
    compressor.compressTick();
    for (int t = 0; t < 8; ++t) {
        if (st(2) == PageState::ACTIVE_MONITORING)
            compressor.handleFault(reinterpret_cast<uintptr_t>(pg(2)));
        compressor.compressTick();
        for (size_t k : {0u, 1u, 3u})
            CHECK(!compressed(k), "group: page %zu compressed at tick %d while page 2 is "
                  "written every tick", k, t);
        // While page 2 waits out its backoff (ACTIVE), its group is not armed.
        if (st(2) == PageState::ACTIVE)
            for (size_t k : {0u, 1u, 3u})
                CHECK(st(k) != PageState::ACTIVE_MONITORING,
                      "group: page %zu armed while page 2 waits out its backoff", k);
    }
    // Page 2 stops: the group compresses, all four in the same tick.
    int first = -1, last = -1;
    for (int t = 0; t < 40 && last < 0; ++t) {
        compressor.compressTick();
        int n = 0;
        for (size_t k = 0; k < 4; ++k) n += compressed(k);
        if (n > 0 && first < 0) first = t;
        if (n == 4) last = t;
    }
    CHECK(last >= 0, "group: never fully compressed after its writes stopped");
    CHECK(first == last, "group: pages compressed in different ticks (%d .. %d)", first, last);

    // A fault on one compressed page restores the whole group.
    if (st(1) == PageState::COMPRESSED_SHADOW) {
        compressor.pretendNextTickAfterMs(5000);
        compressor.compressTick();                           // release the shadows
    }
    CHECK(st(1) == PageState::COMPRESSED, "group: page 1 not released (state %d)",
          static_cast<int>(st(1)));
    CHECK(compressor.handleFault(reinterpret_cast<uintptr_t>(pg(1))), "group: restore fault");
    for (size_t k = 0; k < 4; ++k)
        CHECK(st(k) == PageState::ACTIVE, "group: page %zu not restored with its group "
              "(state %d)", k, static_cast<int>(st(k)));
    for (size_t k = 0; k < 4; ++k)
        CHECK(pg(k)[0] == 0x30 + (base + k - idx0) && pg(k)[kPageSize - 1] == 0x30 + (base + k - idx0),
              "group: page %zu data mismatch", k);
}

// The time budget and the thrash rate must count every page of a large
// block. The fault handler found a restored page's bucket through the page
// map, which registers a large block's first page only, while compressPage
// counted the compression of every page: a call site whose blocks came back
// whole showed one decompression per block, a thrash rate up to the block's
// page count too low, and a saved profile marked it stable_cold -- the next
// run compressed it after a quarter of the cold timeout. The profile save
// also took the rate from the one worker with the most samples, over its own
// compressions, though every decompression is charged to worker 0.
//
// A 64-page block registered by its first page only, as LargeAlloc does:
// pages 0-62 hold data, page 63 zeros (a zero page counts no compression, so
// its restore counts no decompression). All 64 compressed, then the interior
// pages restored, then the first. SMASH_PREFETCH_WINDOW=0: a prefetched
// neighbour is restored without a fault.
static void testInteriorBucketStats() {
    char path[] = "/tmp/smash_bucket_profile_XXXXXX";
    const int fd = mkstemp(path);
    CHECK(fd >= 0, "bucket stats: mkstemp failed");
    if (fd < 0) return;
    close(fd);
    setenv("SMASH_PROFILE_FILE", path, 1);
    setenv("SMASH_PROFILE_FILE_SAVE", "1", 1);
    setenv("SMASH_PREFETCH_WINDOW", "0", 1);
    setenv("SMASH_REARM_BACKOFF_MAX", "0", 1);
    VmRegion vm;
    vm.init(256 * kPageSize);
    PageStateTable states;
    states.init(vm.totalPages());
    PageLockTable locks;
    locks.init(vm.totalPages());
    CompressStore store;
    store.init();
    CompressEngine engine;
    engine.init();
    PageMap page_map;
    page_map.init();
    CompressorThread compressor;
    compressor.init(&vm, &states, &locks, &store, &engine, &page_map);
    unsetenv("SMASH_REARM_BACKOFF_MAX");
    compressor.setRestoreGranulePages(1);

    constexpr uint32_t kPages = 64;
    constexpr uint8_t kArena = 2;
    const uint8_t sc = largeSizeClass(kPages);
    auto* mem = static_cast<uint8_t*>(vm.allocatePages(kPages));
    CHECK(mem != nullptr, "bucket stats: alloc failed");
    if (!mem) return;
    Span* span = newSpanDescriptor();
    span->init(mem, kPages, 0, kArena);
    span->is_large = true;
    page_map.setRange(reinterpret_cast<uintptr_t>(mem), 1, span);
    const size_t first = vm.pageIndex(reinterpret_cast<uintptr_t>(mem));
    // 16 letters at random: compresses to about half, so each restore takes
    // a measurable decompression.
    auto want = [](size_t k, size_t i) {
        if (k == kPages - 1) return uint8_t{0};
        uint64_t x = (k * kPageSize + i + 1) * 0x9E3779B97F4A7C15ull;
        x ^= x >> 29;
        return static_cast<uint8_t>('a' + (x >> 60));
    };
    for (size_t k = 0; k < kPages; ++k) {
        states.set(first + k, PageState::ACTIVE);
        for (size_t i = 0; i < kPageSize; ++i) mem[k * kPageSize + i] = want(k, i);
    }
    size_t done = 0;
    for (int t = 0; t < 400 && done < kPages; ++t) {
        compressor.compressTick();
        done = 0;
        for (size_t k = 0; k < kPages; ++k) done += states.get(first + k) == PageState::COMPRESSED;
    }
    CHECK(done == kPages, "bucket stats: %zu of %u pages compressed", done, kPages);

    auto sums = [&](uint8_t arena, uint64_t* cc, uint64_t* dc, uint64_t* tc) {
        *cc = *dc = *tc = 0;
        for (int w = 0; w < kMaxCompressorWorkers; ++w) {
            const auto& st = compressor.bucketStatsForTest(w, arena, sc);
            *cc += st.compress_count.load();
            *dc += st.decompress_count.load();
            *tc += st.time_cost_total_us.load();
        }
    };
    uint64_t cc0, dc0, tc0, cc1, dc1, tc1, cc2, dc2, tc2;
    sums(kArena, &cc0, &dc0, &tc0);
    for (size_t k = 1; k < kPages; ++k)
        CHECK(compressor.handleFault(reinterpret_cast<uintptr_t>(mem + k * kPageSize)),
              "bucket stats: restore of page %zu", k);
    sums(kArena, &cc1, &dc1, &tc1);
    CHECK(compressor.handleFault(reinterpret_cast<uintptr_t>(mem)), "bucket stats: restore of page 0");
    sums(kArena, &cc2, &dc2, &tc2);
    fprintf(stderr, "bucket stats: %llu compressions; decompressions %llu, +%llu after 63 interior restores "
            "(+%llu us), +%llu after the first page\n", (unsigned long long)cc0, (unsigned long long)dc0,
            (unsigned long long)(dc1 - dc0), (unsigned long long)(tc1 - tc0), (unsigned long long)(dc2 - dc1));
    CHECK(cc0 == kPages - 1, "bucket stats: %llu compressions counted, want %u (the zero page counts none)",
          (unsigned long long)cc0, kPages - 1);
    CHECK(dc0 == 0, "bucket stats: %llu decompressions before any restore", (unsigned long long)dc0);
    CHECK(dc1 - dc0 == kPages - 2, "bucket stats: interior restores counted %llu decompressions, want %u",
          (unsigned long long)(dc1 - dc0), kPages - 2);
    CHECK(tc1 > tc0, "bucket stats: interior restores charged no decompression time");
    CHECK(dc2 - dc1 == 1, "bucket stats: the first page's restore counted %llu", (unsigned long long)(dc2 - dc1));
    size_t bad = 0;
    for (size_t k = 0; k < kPages; ++k)
        for (size_t i = 0; i < kPageSize; ++i) bad += mem[k * kPageSize + i] != want(k, i);
    CHECK(bad == 0, "bucket stats: %zu bytes read back wrong", bad);

    // Bucket (kArena + 1, sc), set by hand: worker 1 holds the most samples
    // and the compressions, worker 0 the decompressions, as the fault
    // handler records them. 80 of 100 came back.
    auto& w0 = compressor.bucketStatsForTest(0, kArena + 1, sc);
    auto& w1 = compressor.bucketStatsForTest(1, kArena + 1, sc);
    w0.count = 8;
    w0.decompress_count.store(80);
    w1.count = 64;
    w1.compress_count.store(100);

    compressor.saveProfileFileForTest();
    using Persist = std::remove_reference_t<decltype(w0)>::Persist;
    struct Header { uint32_t magic, version; uint16_t num_arenas, num_classes; uint32_t flags; };
    static_assert(sizeof(Header) == 16, "profile header layout");
    Persist rec[2] = {};
    bool read_ok = false;
    if (FILE* f = fopen(path, "rb")) {
        Header h{};
        read_ok = fread(&h, sizeof h, 1, f) == 1 && h.num_classes == kTotalBucketsPerArena &&
                  fseek(f, static_cast<long>(sizeof(Persist) * (kArena * kTotalBucketsPerArena + sc)), SEEK_CUR) == 0 &&
                  fread(&rec[0], sizeof(Persist), 1, f) == 1 &&
                  fseek(f, static_cast<long>(sizeof(Persist) * (kTotalBucketsPerArena - 1)), SEEK_CUR) == 0 &&
                  fread(&rec[1], sizeof(Persist), 1, f) == 1;
        fclose(f);
    }
    unlink(path);
    char lock[sizeof path + 8];
    snprintf(lock, sizeof lock, "%s.lock", path);
    unlink(lock);
    CHECK(read_ok, "bucket stats: profile file %s not readable", path);
    fprintf(stderr, "bucket stats: saved thrash rate %u/255 stable_cold %u (whole block restored); %u/255 "
            "stable_cold %u (80 of 100, counts split over two workers)\n", rec[0].thrash_rate, rec[0].stable_cold,
            rec[1].thrash_rate, rec[1].stable_cold);
    CHECK(rec[0].thrash_rate == 255 && !rec[0].stable_cold,
          "bucket stats: profile saved thrash rate %u/255, stable_cold %u for a block restored whole; want 255, 0",
          rec[0].thrash_rate, rec[0].stable_cold);
    CHECK(rec[1].thrash_rate == 80 * 255 / 100 && !rec[1].stable_cold,
          "bucket stats: profile saved thrash rate %u/255, stable_cold %u with the counts on two workers; "
          "want %d, 0", rec[1].thrash_rate, rec[1].stable_cold, 80 * 255 / 100);
}

// A worker allocated after worker 0 starts from worker 0's knowledge of each
// bucket (ratio window, cost EMAs, the persisted hints a profile loads into
// the workers that exist then), never from its totals: every reader sums
// those over all workers, and a copy counted worker 0's compressions once
// more per late worker (63 became 1953 over 31 workers).
static void testSeedBucketStats() {
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

    constexpr uint8_t kArena = 1, kSc = 3;
    auto& w0 = compressor.bucketStatsForTest(0, kArena, kSc);
    w0.count = 12;
    w0.head = 12;
    for (int i = 0; i < 12; ++i) w0.ratios[i] = 200;
    w0.sum = 12 * 200;
    w0.cost_ema_x16[0] = 1600;
    w0.cost_count[0] = 9;
    w0.persist_hint = 1;
    w0.profile_stable_cold = 1;
    w0.profile_best_tier = 0;
    w0.compress_count.store(63);
    w0.decompress_count.store(21);
    w0.bytes_saved_total.store(123456);
    w0.time_cost_total_us.store(777);

    // The last worker of the pool: allocated now, by this call.
    const auto& late = compressor.bucketStatsForTest(kMaxCompressorWorkers - 1, kArena, kSc);
    fprintf(stderr, "seed stats: late worker count %u sum %u cost %u/%u hint %u stable %u tier %u; "
            "totals %llu %llu %llu %llu\n", late.count, late.sum, late.cost_ema_x16[0], late.cost_count[0],
            late.persist_hint, late.profile_stable_cold, late.profile_best_tier,
            (unsigned long long)late.compress_count.load(), (unsigned long long)late.decompress_count.load(),
            (unsigned long long)late.bytes_saved_total.load(), (unsigned long long)late.time_cost_total_us.load());
    CHECK(late.count == 12 && late.head == 12 && late.sum == 12 * 200 && late.ratios[11] == 200,
          "seed stats: a late worker's ratio window is %u samples, sum %u; want worker 0's 12, %u",
          late.count, late.sum, 12 * 200);
    CHECK(late.cost_ema_x16[0] == 1600 && late.cost_count[0] == 9,
          "seed stats: a late worker's cost EMA %u (%u samples); want worker 0's 1600 (9)",
          late.cost_ema_x16[0], late.cost_count[0]);
    CHECK(late.persist_hint == 1 && late.profile_stable_cold == 1 && late.profile_best_tier == 0,
          "seed stats: a late worker's profile hints %u/%u/%u; want worker 0's 1/1/0",
          late.persist_hint, late.profile_stable_cold, late.profile_best_tier);
    CHECK(late.compress_count.load() == 0, "seed stats: a late worker starts with %llu compressions, want 0",
          (unsigned long long)late.compress_count.load());
    CHECK(late.decompress_count.load() == 0, "seed stats: a late worker starts with %llu decompressions, want 0",
          (unsigned long long)late.decompress_count.load());
    CHECK(late.bytes_saved_total.load() == 0, "seed stats: a late worker starts with %llu bytes saved, want 0",
          (unsigned long long)late.bytes_saved_total.load());
    CHECK(late.time_cost_total_us.load() == 0, "seed stats: a late worker starts with %llu us spent, want 0",
          (unsigned long long)late.time_cost_total_us.load());
}

// init() initializes the function-local statics that read the environment
// (warmupClassStatics), so none of them first runs getenv later, inside a
// tick or a fault. The warm-up used to call getenv itself, which initialized
// none of them: each took whatever the environment held at its first use.
// Each variable is set before init() and changed after it; the value in
// effect must be the first. Its own process: these statics live for good.
static void testWarmStatics() {
    setenv("SMASH_PROTECT_CHUNK_PAGES", "7", 1);
    setenv("SMASH_P2_CHUNK", "0", 1);
    setenv("SMASH_SOFTDIRTY", "0", 1);
    setenv("SMASH_IDLE_READ_TRACK", "0", 1);
    setenv("SMASH_CPU_PRESSURE_CAP", "0", 1);
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
    setenv("SMASH_PROTECT_CHUNK_PAGES", "9", 1);
    setenv("SMASH_P2_CHUNK", "1", 1);
    setenv("SMASH_SOFTDIRTY", "1", 1);
    setenv("SMASH_IDLE_READ_TRACK", "1", 1);
    setenv("SMASH_CPU_PRESSURE_CAP", "1", 1);

    size_t chunk = 0;
    bool p2 = true, sd = true, cap = true;
    int idle_mode = 0;
    CompressorThread::envStaticsForTest(&chunk, &p2, &sd, &idle_mode, &cap);
    fprintf(stderr, "warm statics: protect chunk %zu, p2 chunk %d, softdirty %d, idle read mode %d, "
            "cpu pressure cap %d\n", chunk, p2, sd, idle_mode, cap);
    CHECK(chunk == 7, "warm statics: SMASH_PROTECT_CHUNK_PAGES in effect %zu, want 7 (set before init)", chunk);
    CHECK(!p2, "warm statics: SMASH_P2_CHUNK=0 set before init is not in effect");
    CHECK(!sd, "warm statics: SMASH_SOFTDIRTY=0 set before init is not in effect");
    CHECK(idle_mode == -1, "warm statics: SMASH_IDLE_READ_TRACK mode %d, want -1 (0 set before init)", idle_mode);
    CHECK(!cap, "warm statics: SMASH_CPU_PRESSURE_CAP=0 set before init is not in effect");
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

int main(int argc, char** argv) {
    if (argc > 1 && strcmp(argv[1], "bucketstats") == 0) {
        testInteriorBucketStats();
        fprintf(stderr, failures ? "compression: %d failures\n" : "compression: all tests passed\n", failures);
        return failures ? 1 : 0;
    }
    if (argc > 1 && strcmp(argv[1], "warmstatics") == 0) {
        testWarmStatics();
        fprintf(stderr, failures ? "compression: %d failures\n" : "compression: all tests passed\n", failures);
        return failures ? 1 : 0;
    }
    if (argc > 1 && strcmp(argv[1], "seedstats") == 0) {
        testSeedBucketStats();
        fprintf(stderr, failures ? "compression: %d failures\n" : "compression: all tests passed\n", failures);
        return failures ? 1 : 0;
    }
    if (argc > 1 && strcmp(argv[1], "reclaimrange") == 0) {
        testReclaimRange();
        fprintf(stderr, failures ? "compression: %d failures\n" : "compression: all tests passed\n", failures);
        return failures ? 1 : 0;
    }
    if (argc > 1 && strcmp(argv[1], "coldmax") == 0) {
        testColdTicksMax();
        fprintf(stderr, failures ? "compression: %d failures\n" : "compression: all tests passed\n", failures);
        return failures ? 1 : 0;
    }
    if (argc > 1 && strcmp(argv[1], "recompress") == 0) {
        testRecompressBackoff();
        fprintf(stderr, failures ? "compression: %d failures\n" : "compression: all tests passed\n", failures);
        return failures ? 1 : 0;
    }
    testCompressEngine();
    testCompressStore();
    testCompressorRoundtrip();
    testIncompressibleData();
    testAccessTracking();
    unsetenv("SMASH_REARM_BACKOFF_MAX");
    testRearmBackoff();
    testRestoreGranule();
    testNominalTicks();
    testWatchGroup();
    testRecompressBackoff();
    testAllocReopen();
    testColdTimeoutParse();
    testReleaseCompressedPages();

    if (failures == 0) {
        fprintf(stderr, "compression: all tests passed\n");
        return 0;
    } else {
        fprintf(stderr, "compression: %d failures\n", failures);
        return 1;
    }
}
