// idle_read_tracking_test.cpp - idle-page read tracking (root-only path).
//
// As root, smash can read real PFNs from /proc/self/pagemap and switches on
// idle-page read tracking: a monitored page due for deep monitoring is marked
// idle through /sys/kernel/mm/page_idle/bitmap instead of being armed
// PROT_NONE, and phase 1 treats "no longer idle" as "read".
//
// That path had two defects, both invisible to an unprivileged run (where it
// never activates) and so to CI:
//  1. The idle bit was consulted for EVERY monitored page, including pages
//     never marked. An unmarked page reads "not idle", so each one counted as
//     read at its first check and never reached escalation: nothing compressed.
//     test_prefetch, test_compression and test_syscall_efault all failed as
//     root for this reason (first reported on aarch64; it is not arch-specific).
//  2. A failed mark was trusted. The kernel ignores an idle mark on a page not
//     on the LRU — the shared zero page never is, and freshly written pages
//     are not until the per-CPU LRU-add batch drains — so such a page read
//     "not idle" forever.
// The fix records per page whether the mark was verified (idle_armed_), trusts
// the bit only then, and falls back to the PROT_NONE arm otherwise.
//
// Asserted, each driving the compressor in-process with SMASH_COLD_TICKS=2:
//  A. written pages: the idle arm is used (page stays PROT_READ, no PROT_NONE)
//     and every page compresses at tick 3;
//  B. the same, read once after the arm: the read is detected without a fault
//     (nothing compressed at tick 3), and the pages still compress once quiet;
//  C. zero-page-backed pages: the mark cannot take, so the arm falls back to
//     PROT_NONE and every page still compresses at tick 3.
// Verified as a real discriminator: against the pre-fix compressor all three
// scenarios fail as root (no page compresses within 16 ticks).
//
// Skips when not root, or when the kernel lacks CONFIG_IDLE_PAGE_TRACKING.
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
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <unistd.h>

using namespace smash;

static int failures = 0;
#define CHECK(cond, msg, ...) do { \
    if (!(cond)) { fprintf(stderr, "FAIL: " msg "\n", ##__VA_ARGS__); ++failures; } \
} while (0)

static constexpr int kPages = 8;

// Protection of the mapping containing `a`, as the "rwxp" field of /proc/self/maps.
static void protOf(void* a, char out[5]) {
    FILE* f = fopen("/proc/self/maps", "r");
    char line[512];
    strcpy(out, "????");
    while (f && fgets(line, sizeof line, f)) {
        unsigned long lo, hi;
        char p[5];
        if (sscanf(line, "%lx-%lx %4s", &lo, &hi, p) == 3 &&
            reinterpret_cast<unsigned long>(a) >= lo &&
            reinterpret_cast<unsigned long>(a) < hi) {
            memcpy(out, p, 5);
            break;
        }
    }
    if (f) fclose(f);
}

// Fault in enough other memory that our pages leave the per-CPU LRU-add batch,
// so an idle mark on them can take.
static void drainLruBatch() {
    const size_t big = 64u << 20;
    void* b = mmap(nullptr, big, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (b == MAP_FAILED) return;
    memset(b, 1, big);
    munmap(b, big);
}

static bool isCompressed(PageState s) {
    return s == PageState::COMPRESSED || s == PageState::COMPRESSED_SHADOW;
}

enum class Kind { kWritten, kReadAfterArm, kZeroPage };

static void scenario(const char* name, Kind kind) {
    VmRegion vm;
    CHECK(vm.init(64 * kPageSize), "%s: VmRegion init failed", name);
    PageStateTable states; states.init(vm.totalPages());
    PageLockTable locks; locks.init(vm.totalPages());
    CompressStore store; store.init();
    CompressEngine engine; engine.init();
    PageMap page_map; page_map.init();
    CompressorThread c;
    c.init(&vm, &states, &locks, &store, &engine, &page_map);

    void* base = vm.allocatePages(kPages);
    CHECK(base != nullptr, "%s: page alloc failed", name);
    if (!base) return;
    Span* span = newSpanDescriptor();
    span->init(base, kPages, 0);
    span->is_large = true;  // keep zeroFreeSlots away from test data
    page_map.setRange(reinterpret_cast<uintptr_t>(base), kPages, span);
    const size_t b = vm.pageIndex(reinterpret_cast<uintptr_t>(base));

    for (int p = 0; p < kPages; ++p) {
        states.set(b + p, PageState::ACTIVE);
        auto* page = static_cast<unsigned char*>(vm.pageAddress(b + p));
        if (kind == Kind::kZeroPage) {
            volatile unsigned char x = *page;  // read only: maps the shared zero page
            (void)x;
        } else {
            memset(page, 0x30 + p, kPageSize);
        }
    }
    drainLruBatch();

    c.compressTick();  // ACTIVE -> ACTIVE_MONITORING
    c.compressTick();  // cold == floor -> escalate: idle arm or PROT_NONE arm

    char prot[5];
    protOf(vm.pageAddress(b), prot);
    if (kind == Kind::kZeroPage) {
        CHECK(prot[0] == '-', "%s: a zero-page-backed page cannot take an idle mark, "
              "so the arm must fall back to PROT_NONE; mapping is %s", name, prot);
    } else {
        CHECK(prot[0] == 'r', "%s: expected the idle arm (page stays PROT_READ), "
              "got %s — idle tracking was not used", name, prot);
    }

    if (kind == Kind::kReadAfterArm && prot[0] == 'r') {
        volatile unsigned sum = 0;
        for (int p = 0; p < kPages; ++p)
            for (size_t o = 0; o < kPageSize; o += 64)
                sum += static_cast<volatile unsigned char*>(vm.pageAddress(b + p))[o];
        (void)sum;
    }

    int done_at = -1;
    for (int t = 3; t <= 16 && done_at < 0; ++t) {
        c.compressTick();
        int n = 0;
        for (int p = 0; p < kPages; ++p) n += isCompressed(states.get(b + p));
        if (t == 3 && kind == Kind::kReadAfterArm)
            CHECK(n == 0, "%s: the read after the arm was not detected "
                  "(%d/%d pages compressed at tick 3)", name, n, kPages);
        if (n == kPages) done_at = t;
    }
    if (kind == Kind::kReadAfterArm) {
        CHECK(done_at > 3, "%s: pages never compressed after the read — the "
              "counted read kept them hot", name);
    } else {
        CHECK(done_at == 3, "%s: expected all %d pages compressed at tick 3, got %s",
              name, kPages, done_at < 0 ? "never (16 ticks)" : std::to_string(done_at).c_str());
    }
    printf("  %-28s arm=%s  all compressed at tick %d\n", name, prot, done_at);
}

// The path under test exists only as root on a kernel with idle page tracking.
static bool idleTrackingAvailable() {
    if (geteuid() != 0) return false;
    int fd = open("/sys/kernel/mm/page_idle/bitmap", O_RDWR | O_CLOEXEC);
    if (fd < 0) return false;
    close(fd);
    return true;
}

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    if (!idleTrackingAvailable()) {
        printf("SKIP: idle-page read tracking needs root and CONFIG_IDLE_PAGE_TRACKING\n");
        return 0;
    }
    scenario("A: written, untouched", Kind::kWritten);
    scenario("B: written, read after arm", Kind::kReadAfterArm);
    scenario("C: zero-page backed", Kind::kZeroPage);
    if (failures) {
        printf("idle_read_tracking_test: %d FAILURES\n", failures);
        return 1;
    }
    printf("idle_read_tracking_test: all checks passed\n");
    return 0;
}
