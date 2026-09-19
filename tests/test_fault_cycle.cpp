// Test real fault handler: allocate → compress → read (triggers SIGSEGV) → verify data
//
// The read must hit a page that is genuinely inaccessible. Three ticks are not
// enough for that in either reclaim mode, and the test used to assume they
// were:
//  - deferred reclaim (default): after 3 ticks the page is COMPRESSED_SHADOW,
//    still mapped read/write, so the read never faulted and the fault handler
//    was not exercised at all — the test passed without testing its subject;
//  - SMASH_DEFERRED_RECLAIM=0: the page is COMPRESSED but its PROT_NONE is
//    queued for the deferred-madvise sweeper, so it is still readable. The
//    read succeeded without a fault and the page stayed COMPRESSED, which
//    failed the old "state after read" check 20/20. That is the failure that
//    got the test disabled in June as "signal-delivery timing"; there was no
//    timing involved, and the data was always correct.
// So each cycle ticks until the page is really inaccessible (PROT_NONE in the
// process's own mapping table), then asserts the read went through the fault
// handler, the data is intact and the page is live again. Registered in both
// reclaim modes.
#include "vm/vm_region.h"
#include "vm/page_state.h"
#include "vm/fault_handler.h"
#include "compress/compress_store.h"
#include "compress/compress_engine.h"
#include "compress/compressor_thread.h"
#include "core/page_map.h"
#include "core/span.h"
#include "smash/config.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <signal.h>
#if defined(__APPLE__)
#include <mach/mach.h>
#include <mach/mach_vm.h>
#endif

using namespace smash;

static int failures = 0;

#define CHECK(cond, msg, ...) do { \
    if (!(cond)) { \
        fprintf(stderr, "FAIL: " msg "\n", ##__VA_ARGS__); \
        ++failures; \
    } \
} while (0)

// Tick budget for a page to become inaccessible. Deferred reclaim needs the
// shadow delay, eager reclaim the sweeper's TTL; both are a few ticks.
static constexpr int kMaxTicks = 200;

// True if `addr`'s page currently has no read permission.
static bool pageInaccessible(const void* addr) {
#if defined(__linux__)
    FILE* f = fopen("/proc/self/maps", "r");
    if (!f) return false;
    char line[512];
    bool none = false;
    auto a = reinterpret_cast<unsigned long>(addr);
    while (fgets(line, sizeof line, f)) {
        unsigned long lo, hi;
        char perms[5];
        if (sscanf(line, "%lx-%lx %4s", &lo, &hi, perms) == 3 && a >= lo && a < hi) {
            none = perms[0] != 'r';
            break;
        }
    }
    fclose(f);
    return none;
#elif defined(__APPLE__)
    mach_vm_address_t region = reinterpret_cast<mach_vm_address_t>(addr);
    mach_vm_size_t size = 0;
    vm_region_basic_info_data_64_t info;
    mach_msg_type_number_t count = VM_REGION_BASIC_INFO_COUNT_64;
    mach_port_t object = MACH_PORT_NULL;
    if (mach_vm_region(mach_task_self(), &region, &size, VM_REGION_BASIC_INFO_64,
                       reinterpret_cast<vm_region_info_t>(&info), &count, &object) != KERN_SUCCESS)
        return false;
    if (region > reinterpret_cast<mach_vm_address_t>(addr)) return false;
    return (info.protection & VM_PROT_READ) == 0;
#else
    (void)addr;
    return false;
#endif
}

// Tick until the page is really inaccessible. Returns the ticks taken, or -1.
static int tickUntilInaccessible(CompressorThread& c, const void* page) {
    for (int t = 1; t <= kMaxTicks; ++t) {
        c.compressTick();
        if (pageInaccessible(page)) return t;
    }
    return -1;
}

// Global state for fault handler callback
static CompressorThread* g_compressor = nullptr;
static volatile sig_atomic_t g_faults = 0;   // fault-handler invocations

static bool faultCallback(uintptr_t fault_addr, void* ctx) {
    (void)ctx;
    g_faults = g_faults + 1;
    return g_compressor->handleFault(fault_addr);
}

static void testFaultCycleBasic() {
    // Set up components
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

    PageMap page_map;
    page_map.init();

    CompressorThread compressor;
    compressor.init(&vm, &states, &locks, &store, &engine, &page_map);
    g_compressor = &compressor;

    // Register fault handler
    vm::FaultHandler fault_handler;
    fault_handler.start(faultCallback, nullptr);

    // Allocate a page and fill with known data
    void* page = vm.allocatePages(1);
    CHECK(page != nullptr, "page alloc failed");

    size_t page_idx = vm.pageIndex(reinterpret_cast<uintptr_t>(page));

    // Register in page map so prefetch boundary checks work.
    // Mark as large so zeroFreeSlots skips it (this is raw test data, not a slab page).
    Span* span = newSpanDescriptor();
    span->init(page, 1, 0);
    span->is_large = true;
    page_map.setRange(reinterpret_cast<uintptr_t>(page), 1, span);

    states.set(page_idx, PageState::ACTIVE);

    // Fill with a pattern
    auto* bytes = static_cast<volatile uint8_t*>(page);
    for (size_t i = 0; i < kPageSize; ++i) {
        bytes[i] = static_cast<uint8_t>(i & 0xFF);
    }

    // Compress until the page is really inaccessible.
    int ticks = tickUntilInaccessible(compressor, page);
    CHECK(ticks > 0, "page never became inaccessible within %d ticks (state %d)",
          kMaxTicks, static_cast<int>(states.get(page_idx)));
    CHECK(states.get(page_idx) == PageState::COMPRESSED,
          "an inaccessible page should be COMPRESSED, got %d",
          static_cast<int>(states.get(page_idx)));

    // Now READ the page — this must raise a real SIGSEGV/SIGBUS, and the fault
    // handler must decompress and restore it.
    const int faults_before = g_faults;
    volatile uint8_t val = bytes[0];
    CHECK(g_faults > faults_before,
          "reading an inaccessible page did not go through the fault handler");
    CHECK(val == 0x00, "first byte should be 0x00, got 0x%02x", val);

    val = bytes[255];
    CHECK(val == 0xFF, "byte 255 should be 0xFF, got 0x%02x", val);

    // Verify full page data integrity
    bool data_ok = true;
    for (size_t i = 0; i < kPageSize; ++i) {
        if (bytes[i] != static_cast<uint8_t>(i & 0xFF)) {
            data_ok = false;
            fprintf(stderr, "FAIL: data mismatch at offset %zu: got 0x%02x expected 0x%02x\n",
                    i, bytes[i], static_cast<uint8_t>(i & 0xFF));
            break;
        }
    }
    CHECK(data_ok, "full page data verification failed");
    CHECK(states.get(page_idx) == PageState::ACTIVE ||
          states.get(page_idx) == PageState::ACTIVE_MONITORING,
          "page should be live again after the fault, got %d",
          static_cast<int>(states.get(page_idx)));
    fprintf(stderr, "basic: inaccessible after %d ticks, fault handled\n", ticks);

    // Clean up
    fault_handler.stop();
    g_compressor = nullptr;
}

static void testFaultCycleMultipleRounds() {
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

    PageMap page_map;
    page_map.init();

    CompressorThread compressor;
    compressor.init(&vm, &states, &locks, &store, &engine, &page_map);
    g_compressor = &compressor;

    vm::FaultHandler fault_handler;
    fault_handler.start(faultCallback, nullptr);

    void* page = vm.allocatePages(1);
    CHECK(page != nullptr, "page alloc failed");

    Span* span = newSpanDescriptor();
    span->init(page, 1, 0);
    span->is_large = true;
    page_map.setRange(reinterpret_cast<uintptr_t>(page), 1, span);

    size_t page_idx = vm.pageIndex(reinterpret_cast<uintptr_t>(page));
    states.set(page_idx, PageState::ACTIVE);

    auto* bytes = static_cast<volatile uint8_t*>(page);

    // Multiple compress → fault → verify cycles
    for (int round = 0; round < 5; ++round) {
        // Write new pattern
        for (size_t i = 0; i < kPageSize; ++i) {
            bytes[i] = static_cast<uint8_t>((i + round) & 0xFF);
        }

        // Compress until the page is really inaccessible.
        int ticks = tickUntilInaccessible(compressor, page);
        CHECK(ticks > 0, "round %d: page never became inaccessible within %d ticks (state %d)",
              round, kMaxTicks, static_cast<int>(states.get(page_idx)));

        // Trigger fault by reading
        const int faults_before = g_faults;
        volatile uint8_t check = bytes[42];
        uint8_t expected = static_cast<uint8_t>((42 + round) & 0xFF);
        CHECK(g_faults > faults_before,
              "round %d: read of an inaccessible page did not fault", round);
        CHECK(check == expected,
              "round %d: byte 42 should be 0x%02x, got 0x%02x",
              round, expected, check);

        CHECK(states.get(page_idx) == PageState::ACTIVE ||
              states.get(page_idx) == PageState::ACTIVE_MONITORING,
              "round %d: page should be live again after the fault, got %d",
              round, static_cast<int>(states.get(page_idx)));

        // Verify full data
        bool data_ok = true;
        for (size_t i = 0; i < kPageSize; ++i) {
            if (bytes[i] != static_cast<uint8_t>((i + round) & 0xFF)) {
                data_ok = false;
                break;
            }
        }
        CHECK(data_ok, "round %d: data integrity check failed", round);
    }

    fault_handler.stop();
    g_compressor = nullptr;
}

int main() {
    testFaultCycleBasic();
    testFaultCycleMultipleRounds();

    if (failures == 0) {
        fprintf(stderr, "fault_cycle: all tests passed\n");
        return 0;
    } else {
        fprintf(stderr, "fault_cycle: %d failures\n", failures);
        return 1;
    }
}
