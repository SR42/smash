// VM primitives must work at smash's kPageSize granularity.
//
// Mechanism-level companion to test_page_size. Where that test compares
// constants, this one issues the exact syscall sequence VmRegion and the
// compressor rely on — reserve a region, commit an interior page, flip its
// protection, decommit it — and asserts each step succeeds.
//
// On a kernel whose page size is larger than smash's compile-time kPageSize
// (a 4 KiB build on a 16 KiB / 64 KiB aarch64 kernel), every one of these
// calls targets an address the kernel does not consider a page boundary and
// fails with EINVAL. VmRegion::ensureCommitted() ignores commitPages()'
// return value and advances its committed_pages_ watermark regardless, so the
// caller receives an address that is still PROT_NONE; the compressor's
// protectPages()/decommitPages() likewise no-op. The allocator then behaves
// as if it were out of memory.
//
// Runs standalone (no interposition) — these are raw OS primitives.

#include "smash/config.h"
#include "vm/platform_mem.h"

#include <cerrno>
#include <cstdint>
#include <initializer_list>
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

namespace {

constexpr size_t kRegionPages = 64;

// Page offsets exercised individually. Anything other than 0 is a sub-kernel-
// page offset when kPageSize < the real page size, which is what makes this a
// discriminator rather than a smoke test.
constexpr size_t kProbePages[] = {0, 1, 2, 3, 7, 15, 31, 63};

struct Region {
    char* base = nullptr;
    size_t bytes = 0;

    ~Region() {
        if (base) {
            vm::setVmBounds(nullptr, 0);
            vm::unmapPages(base, bytes);
        }
    }
};

// Reserve a region and publish it as smash's VM bounds. protectPages() and
// decommitPages() hard-trap (__builtin_trap) on addresses outside the bounds,
// so the bounds must be set before either is called.
bool makeRegion(Region& r, size_t pages) {
    r.bytes = pages * kPageSize;
    r.base = static_cast<char*>(vm::reservePages(r.bytes));
    if (!r.base) return false;
    vm::setVmBounds(r.base, r.bytes);
    return true;
}

} // namespace

// ── reservePages returns a kPageSize-aligned region ─────────────────────────
static void testReserveAlignment() {
    Region r;
    CHECK(makeRegion(r, kRegionPages), "reservePages(%zu pages) returned null",
          kRegionPages);
    if (!r.base) return;

    auto addr = reinterpret_cast<uintptr_t>(r.base);
    CHECK((addr & (kPageSize - 1)) == 0,
          "reservePages returned %p, not aligned to kPageSize (%zu)",
          static_cast<void*>(r.base), kPageSize);
}

// ── committing a single interior page ───────────────────────────────────────
// This is VmRegion::ensureCommitted()'s mprotect, page by page.
static void testCommitInteriorPage() {
    Region r;
    if (!makeRegion(r, kRegionPages)) {
        CHECK(false, "reservePages(%zu pages) returned null", kRegionPages);
        return;
    }

    for (size_t page : kProbePages) {
        void* p = r.base + page * kPageSize;
        errno = 0;
        bool ok = vm::commitPages(p, kPageSize);
        CHECK(ok,
              "commitPages(base + %zu*%zu, %zu) failed (errno=%d %s). smash "
              "cannot commit memory at %zu-byte granularity on this kernel; "
              "VmRegion hands out PROT_NONE memory and the allocator behaves "
              "as if out of memory",
              page, kPageSize, kPageSize, errno, std::strerror(errno),
              kPageSize);
        if (!ok) continue;

        // A successful commit must actually be writable — ensureCommitted
        // never checks, so a silent failure surfaces only as a SIGSEGV here.
        auto* bytes = static_cast<volatile unsigned char*>(p);
        bytes[0] = 0x5A;
        bytes[kPageSize - 1] = 0xA5;
        CHECK(bytes[0] == 0x5A && bytes[kPageSize - 1] == 0xA5,
              "page %zu did not retain writes after commitPages", page);
    }
}

// ── committing a multi-page run at a non-zero offset ────────────────────────
static void testCommitRun() {
    Region r;
    if (!makeRegion(r, kRegionPages)) {
        CHECK(false, "reservePages(%zu pages) returned null", kRegionPages);
        return;
    }

    const size_t start = 5;      // deliberately not a power-of-two multiple
    const size_t run = 11;
    void* p = r.base + start * kPageSize;
    errno = 0;
    bool ok = vm::commitPages(p, run * kPageSize);
    CHECK(ok, "commitPages(base + %zu*%zu, %zu*%zu) failed (errno=%d %s)",
          start, kPageSize, run, kPageSize, errno, std::strerror(errno));
    if (!ok) return;

    std::memset(p, 0x33, run * kPageSize);
    auto* bytes = static_cast<unsigned char*>(p);
    CHECK(bytes[0] == 0x33 && bytes[run * kPageSize - 1] == 0x33,
          "multi-page commit did not retain writes");
}

// ── protect / restore a single page ─────────────────────────────────────────
// The compressor drops a cold page to PROT_READ (Phase 3) and later to
// PROT_NONE (Phase 2), then the fault handler restores PROT_RW. Each of those
// is a single-page mprotect at a kPageSize offset.
static void testProtectSinglePage() {
    Region r;
    if (!makeRegion(r, kRegionPages)) {
        CHECK(false, "reservePages(%zu pages) returned null", kRegionPages);
        return;
    }

    for (size_t page : kProbePages) {
        void* p = r.base + page * kPageSize;
        if (!vm::commitPages(p, kPageSize)) continue;  // reported above
        std::memset(p, 0x77, kPageSize);

        errno = 0;
        CHECK(vm::protectPages(p, kPageSize, /*read=*/true, /*write=*/false),
              "protectPages(page %zu, PROT_READ) failed (errno=%d %s) — the "
              "compressor's Phase-3 access tracking cannot run at %zu-byte "
              "granularity", page, errno, std::strerror(errno), kPageSize);

        // Read-only must still read the original contents.
        CHECK(static_cast<const volatile unsigned char*>(p)[0] == 0x77,
              "page %zu lost its contents across a PROT_READ transition", page);

        errno = 0;
        CHECK(vm::protectPages(p, kPageSize, /*read=*/true, /*write=*/true),
              "protectPages(page %zu, PROT_RW) failed (errno=%d %s) — the "
              "fault handler could not restore write access",
              page, errno, std::strerror(errno));

        static_cast<volatile unsigned char*>(p)[0] = 0x11;
        CHECK(static_cast<const volatile unsigned char*>(p)[0] == 0x11,
              "page %zu not writable after PROT_RW restore", page);
    }
}

// ── decommit reclaims a single page ─────────────────────────────────────────
// decommitPages() is madvise(MADV_DONTNEED) on Linux. On a page-size mismatch
// it returns EINVAL and the page keeps its contents — so smash believes it
// reclaimed memory it still holds. Detect that by writing a pattern,
// decommitting, and requiring the page to read back as zero.
static void testDecommitReclaims() {
#if defined(__linux__)
    if (getenv("SMASH_NO_DECOMMIT") || getenv("SMASH_MADV_FREE")) {
        printf("SKIP testDecommitReclaims: decommit behavior overridden by env\n");
        return;
    }

    Region r;
    if (!makeRegion(r, kRegionPages)) {
        CHECK(false, "reservePages(%zu pages) returned null", kRegionPages);
        return;
    }

    for (size_t page : kProbePages) {
        void* p = r.base + page * kPageSize;
        if (!vm::commitPages(p, kPageSize)) continue;  // reported above
        std::memset(p, 0xEE, kPageSize);

        vm::decommitPages(p, kPageSize);

        // MADV_DONTNEED on an anonymous private mapping drops the physical
        // page; the next read faults in a fresh zero page. If the madvise was
        // rejected (EINVAL from a granularity mismatch) the old bytes survive.
        auto* bytes = static_cast<const unsigned char*>(p);
        bool zeroed = true;
        for (size_t i = 0; i < kPageSize; ++i) {
            if (bytes[i] != 0) { zeroed = false; break; }
        }
        CHECK(zeroed,
              "decommitPages(page %zu, %zu bytes) did not reclaim the page — "
              "it still reads 0xEE. madvise(MADV_DONTNEED) was rejected at "
              "this granularity, so smash's RSS accounting is fiction and "
              "cold pages are never actually released",
              page, kPageSize);
    }
#else
    printf("SKIP testDecommitReclaims: MADV_FREE_REUSABLE is advisory on this "
           "platform\n");
#endif
}

// ── mapPages returns usable, aligned, page-granular memory ──────────────────
// LargeAlloc's direct-mmap fallback — the path every allocation takes once
// VmRegion is exhausted — goes through vm::mapPages().
static void testMapPages() {
    for (size_t pages : {size_t{1}, size_t{4}, size_t{33}}) {
        size_t bytes = pages * kPageSize;
        void* p = vm::mapPages(bytes);
        CHECK(p != nullptr, "mapPages(%zu) returned null", bytes);
        if (!p) continue;

        auto addr = reinterpret_cast<uintptr_t>(p);
        CHECK((addr & (kPageSize - 1)) == 0,
              "mapPages(%zu) returned %p, not kPageSize-aligned", bytes, p);

        std::memset(p, 0x5C, bytes);
        auto* bytes_p = static_cast<unsigned char*>(p);
        CHECK(bytes_p[0] == 0x5C && bytes_p[bytes - 1] == 0x5C,
              "mapPages(%zu) region not fully writable", bytes);

        vm::unmapPages(p, bytes);
    }
}

int main() {
    printf("kPageSize=%zu\n", kPageSize);

    testReserveAlignment();
    testCommitInteriorPage();
    testCommitRun();
    testProtectSinglePage();
    testDecommitReclaims();
    testMapPages();

    if (failures) {
        printf("test_vm_granularity: %d FAILURES\n", failures);
        return 1;
    }
    printf("test_vm_granularity: all checks passed\n");
    return 0;
}
