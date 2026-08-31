// Page-size and address-space invariants.
//
// smash's page geometry is a COMPILE-TIME constant (kPageSize =
// ALLOC8_PAGE_SIZE), but every mmap / mprotect / madvise it issues is
// validated by the kernel against the RUNNING kernel's page size. alloc8
// hard-codes 4096 for every Linux target — including aarch64, where 16 KiB
// and 64 KiB base pages are common (RHEL/Rocky/Oracle aarch64 ship 64 KiB;
// Asahi and some Ubuntu images ship 16 KiB).
//
// When the two disagree, smash keeps handing out addresses at 4 KiB
// granularity inside a region the kernel manages at 64 KiB granularity:
//
//   * mprotect/madvise on a sub-page-aligned address returns EINVAL, so
//     VmRegion::ensureCommitted() silently fails to commit (it ignores
//     commitPages()' return value and advances its watermark anyway) and
//     the allocation is handed back still PROT_NONE;
//   * every over-aligned allocation routed through LargeAlloc burns a full
//     *kernel* page (64 KiB) for what smash accounted as 4 KiB, a 16x
//     footprint blow-up that exhausts memory and turns malloc into nullptr;
//   * nullptr out of malloc is what the operator-new wrappers in
//     linux_syscall_wrappers.cpp turn into `throw std::bad_alloc()`.
//
// So the user-visible symptom of a page-size mismatch is std::bad_alloc from
// an app that has plenty of memory. This test makes that mismatch a loud,
// specific test failure instead of a mysterious runtime one.
//
// Runs standalone (no interposition needed) — the invariants are properties
// of the build + the kernel it is running on, not of any allocation.

#include "smash/config.h"
#include "core/size_classes.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>

#if !defined(_WIN32)
#include <sys/mman.h>
#endif

using namespace smash;

static int failures = 0;

#define CHECK(cond, msg, ...) do { \
    if (!(cond)) { \
        fprintf(stderr, "FAIL: " msg "\n", ##__VA_ARGS__); \
        ++failures; \
    } \
} while (0)

// ── The kPageShift <-> kPageSize agreement ──────────────────────────────────
// config.h derives the shift with a ternary that only knows about 4 KiB and
// 16 KiB:
//     kPageShift = (kPageSize == 16384) ? 14 : 12;
// A 64 KiB build would therefore get shift 12 while pages are 65536 bytes,
// and every page index in PageMap / VmRegion / PageStateTable would be wrong
// by 16x. Assert the two are consistent by construction.
static void testPageShiftMatchesPageSize() {
    CHECK((static_cast<size_t>(1) << kPageShift) == kPageSize,
          "kPageShift (%d) does not describe kPageSize (%zu): 1<<%d = %zu. "
          "config.h's kPageShift ternary only handles 4 KiB and 16 KiB pages",
          kPageShift, kPageSize, kPageShift,
          static_cast<size_t>(1) << kPageShift);

    CHECK(kPageSize != 0 && (kPageSize & (kPageSize - 1)) == 0,
          "kPageSize (%zu) is not a power of two", kPageSize);
}

// ── The compile-time page size vs. the running kernel's ─────────────────────
// THE aarch64 check. A build made for 4 KiB pages running on a 16 KiB or
// 64 KiB kernel is the documented cause of the std::bad_alloc reports.
static void testPageSizeMatchesKernel() {
#if defined(_SC_PAGESIZE)
    long sc = sysconf(_SC_PAGESIZE);
    CHECK(sc > 0, "sysconf(_SC_PAGESIZE) failed (%ld)", sc);
    if (sc <= 0) return;

    size_t kernel_page = static_cast<size_t>(sc);
    CHECK(kernel_page == kPageSize,
          "PAGE SIZE MISMATCH: smash was compiled for %zu-byte pages but this "
          "kernel uses %zu-byte pages.\n"
          "       Every mprotect/madvise smash issues at %zu-byte granularity "
          "will be rejected with EINVAL, VmRegion hands back uncommitted "
          "(PROT_NONE) memory, and over-aligned allocations cost a full "
          "%zu-byte kernel page each. malloc then returns nullptr and "
          "operator new throws std::bad_alloc.\n"
          "       Fix: teach ALLOC8_PAGE_SIZE (alloc8/platform.h) the running "
          "page size for this target, or build on a host whose page size "
          "matches the deployment kernel.",
          kPageSize, kernel_page, kPageSize, kernel_page);
#endif
}

// ── Real mappings honour smash's page granularity ───────────────────────────
// If the kernel's page is larger than kPageSize, an mmap of exactly kPageSize
// still returns a kernel-page-aligned address, but the *next* kPageSize-sized
// slot smash believes it owns is inside the same kernel page. Detect the
// discrepancy by mapping two kPageSize regions worth of space and checking
// that a kPageSize-granular offset inside it is itself a valid page boundary
// as far as the kernel is concerned.
static void testMappingGranularity() {
#if !defined(_WIN32)
    const size_t bytes = 8 * kPageSize;
    void* p = mmap(nullptr, bytes, PROT_READ | PROT_WRITE,
                   MAP_PRIVATE | MAP_ANON, -1, 0);
    CHECK(p != MAP_FAILED, "mmap(%zu) failed", bytes);
    if (p == MAP_FAILED) return;

    auto addr = reinterpret_cast<uintptr_t>(p);
    CHECK((addr & (kPageSize - 1)) == 0,
          "mmap returned %p, not aligned to smash's kPageSize (%zu)",
          p, kPageSize);

    // mprotect a single interior smash-page. This is precisely what
    // VmRegion::ensureCommitted and the compressor's protectPages() do, and
    // it is the call that returns EINVAL under a page-size mismatch.
    void* interior = static_cast<char*>(p) + kPageSize;
    int rc = mprotect(interior, kPageSize, PROT_READ);
    CHECK(rc == 0,
          "mprotect(base + kPageSize, kPageSize) failed (rc=%d). smash cannot "
          "manage memory at %zu-byte granularity on this kernel",
          rc, kPageSize);
    if (rc == 0) mprotect(interior, kPageSize, PROT_READ | PROT_WRITE);

    munmap(p, bytes);
#endif
}

// ── PageMap's L1 table actually covers this machine's address space ─────────
// PageMap indexes level1_[addr >> kPageShift >> kPageMapL2Bits] with NO bounds
// check, sized from a hard-coded kAddressBits = 48. aarch64 kernels can be
// built with 52-bit user VA (LPA2 + 64 KiB pages); an address above 2^48 would
// index past the end of the L1 table and read a garbage Span*, which free()
// then dereferences. Verify the addresses this process actually sees fit.
static void checkAddressFits(const void* p, const char* what) {
    auto addr = reinterpret_cast<uintptr_t>(p);
    size_t l1 = (addr >> kPageShift) >> kPageMapL2Bits;
    CHECK(l1 < kPageMapL1Size,
          "%s address %p yields PageMap L1 index %zu, but the L1 table only "
          "has %zu entries (kAddressBits=%d). PageMap::get() would read out of "
          "bounds and return a garbage Span*",
          what, p, l1, kPageMapL1Size, kAddressBits);
}

static void testAddressSpaceFitsPageMap() {
    int stack_var = 0;
    static int static_var = 0;
    checkAddressFits(&stack_var, "stack");
    checkAddressFits(&static_var, "static");
    checkAddressFits(reinterpret_cast<const void*>(&testAddressSpaceFitsPageMap),
                     "text");

#if !defined(_WIN32)
    void* heap = mmap(nullptr, kPageSize, PROT_READ | PROT_WRITE,
                      MAP_PRIVATE | MAP_ANON, -1, 0);
    if (heap != MAP_FAILED) {
        checkAddressFits(heap, "mmap");
        munmap(heap, kPageSize);
    }

    // Ask for a mapping above 2^48 explicitly. Kernels without large-VA
    // support ignore the hint and return something low, which is fine — we
    // only assert on what we are actually handed.
    void* hint = reinterpret_cast<void*>(static_cast<uintptr_t>(1) << 49);
    void* high = mmap(hint, kPageSize, PROT_READ | PROT_WRITE,
                      MAP_PRIVATE | MAP_ANON, -1, 0);
    if (high != MAP_FAILED) {
        checkAddressFits(high, "high-VA hint");
        munmap(high, kPageSize);
    }
#endif
}

// ── Derived constants stay coherent at this page size ───────────────────────
static void testDerivedConstants() {
    CHECK(kPageMapL1Bits > 0,
          "kPageMapL1Bits is %d — kAddressBits(%d) - kPageShift(%d) - "
          "kPageMapL2Bits(%d) must stay positive",
          kPageMapL1Bits, kAddressBits, kPageShift, kPageMapL2Bits);

    CHECK(kVmRegionSize / kPageSize == kVmMaxPages,
          "kVmRegionSize (%zu) / kPageSize (%zu) = %zu != kVmMaxPages (%zu)",
          kVmRegionSize, kPageSize, kVmRegionSize / kPageSize, kVmMaxPages);

    size_t runtime_vm = getVmRegionSize();
    CHECK(runtime_vm % kPageSize == 0,
          "getVmRegionSize() (%zu) is not a multiple of kPageSize (%zu) — "
          "VmRegion::contig_pages_ would truncate",
          runtime_vm, kPageSize);
    CHECK(runtime_vm <= kVmRegionSize,
          "getVmRegionSize() (%zu) exceeds the compile-time max (%zu)",
          runtime_vm, kVmRegionSize);

    CHECK(kLargeAllocVmThreshold % kPageSize == 0,
          "kLargeAllocVmThreshold (%zu) is not a multiple of kPageSize (%zu)",
          kLargeAllocVmThreshold, kPageSize);
}

// ── Size-class span geometry is valid at this page size ─────────────────────
// computeClassInfo() derives pages-per-span from kPageSize and clamps to
// kMaxSpanPages. A larger page size changes every span's object count; a
// class whose object no longer fits in its span silently produces a slab that
// can hand out zero objects, so Slab::allocate() returns nullptr forever ->
// malloc nullptr -> std::bad_alloc.
static void testSizeClassGeometry() {
    for (int i = 0; i < kNumClasses; ++i) {
        const SizeClassInfo& info = kSizeClasses[i];
        CHECK(info.pages >= 1 && info.pages <= static_cast<uint32_t>(kMaxSpanPages),
              "class %d (size %u): pages=%u outside [1, %d]",
              i, info.size, info.pages, kMaxSpanPages);
        CHECK(info.objects >= 1,
              "class %d (size %u): span of %u page(s) (%zu bytes) holds %u "
              "objects — a zero-capacity span makes Slab::allocate() return "
              "nullptr forever",
              i, info.size, info.pages,
              static_cast<size_t>(info.pages) * kPageSize, info.objects);
        CHECK(static_cast<size_t>(info.objects) * info.size
                  <= static_cast<size_t>(info.pages) * kPageSize,
              "class %d (size %u): %u objects x %u bytes overruns a %zu-byte "
              "span", i, info.size, info.objects, info.size,
              static_cast<size_t>(info.pages) * kPageSize);
    }

    CHECK(kMaxSmallSize <= static_cast<size_t>(kMaxSpanPages) * kPageSize,
          "kMaxSmallSize (%zu) does not fit in the largest span (%d pages = "
          "%zu bytes)", kMaxSmallSize, kMaxSpanPages,
          static_cast<size_t>(kMaxSpanPages) * kPageSize);

    // Every small size must map to a class that can hold it.
    for (size_t s = 1; s <= kMaxSmallSize; ++s) {
        uint8_t sc = sizeToClass(s);
        if (sc >= kNumClasses) {
            CHECK(false, "sizeToClass(%zu) = %u is out of range [0, %d)",
                  s, sc, kNumClasses);
            break;
        }
        if (classSize(sc) < s) {
            CHECK(false, "sizeToClass(%zu) -> class %u of size %u (too small)",
                  s, sc, classSize(sc));
            break;
        }
    }
}

int main() {
    printf("kPageSize=%zu kPageShift=%d kAddressBits=%d "
           "kPageMapL1Size=%zu kMaxSpanPages=%d\n",
           kPageSize, kPageShift, kAddressBits, kPageMapL1Size, kMaxSpanPages);
#if defined(_SC_PAGESIZE)
    printf("kernel page size (sysconf) = %ld\n", sysconf(_SC_PAGESIZE));
#endif

    testPageShiftMatchesPageSize();
    testPageSizeMatchesKernel();
    testMappingGranularity();
    testAddressSpaceFitsPageMap();
    testDerivedConstants();
    testSizeClassGeometry();

    if (failures) {
        printf("test_page_size: %d FAILURES\n", failures);
        return 1;
    }
    printf("test_page_size: all checks passed\n");
    return 0;
}
