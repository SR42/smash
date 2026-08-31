// Over-aligned small allocations must not cost an unbounded multiple of what
// was requested.
//
// SmashHeap::memalign() routes EVERY request whose alignment exceeds
// kMinAlignment (16 bytes) to LargeAlloc:
//
//     if (alignment <= kMinAlignment) return this->malloc(size);
//     return large_alloc_.allocate(size, alignment, arena);
//
// and LargeAlloc::allocate() immediately rounds the alignment up to a whole
// page:
//
//     if (alignment < kPageSize) alignment = kPageSize;
//     size_t alloc_size = roundUp(size, kPageSize);
//
// So a 64-byte object declared `alignas(64)` — utterly ordinary in C++;
// `std::hardware_destructive_interference_size`, SIMD types, lock-free
// structures and Eigen all produce them, and C++17 routes them through
// `operator new(size_t, align_val_t)` — consumes a full page.
//
// The cost of that scales with the KERNEL's page size, not smash's:
//
//     x86-64 / 4 KiB pages   ->    64x the requested bytes
//     macOS arm64 / 16 KiB   ->   256x
//     aarch64 / 64 KiB pages -> 1,024x
//
// 64 KiB base pages are the default on RHEL/Rocky/Oracle aarch64. An
// application allocating a few hundred thousand aligned objects there needs
// tens of gigabytes for a few tens of megabytes of payload, runs the machine
// out of memory, gets nullptr from malloc, and sees std::bad_alloc from the
// operator-new wrappers. That is the reported aarch64 failure, reproduced
// from the allocator's side.
//
// Two tiers, following the pattern test_compression_ratio uses:
//   * a hard budget that FAILS the test — set so a 4 KiB or 16 KiB page size
//     passes with roughly 2x headroom while a 64 KiB page size blows through
//     it by 2x. This is the aarch64 discriminator.
//   * a WARN when the per-object cost exceeds 16x the requested bytes, which
//     fires on every platform today and documents the routing cost without
//     turning CI red for a known design property.
//
// Run under LD_PRELOAD / DYLD_INSERT_LIBRARIES in both SMASH_LARGE_ONLY=0
// (full) and SMASH_LARGE_ONLY=1 (production) modes. Large-only passes small
// aligned requests through to the system allocator, so it should show
// essentially no amplification — which is the measurement backing CLAUDE.md's
// recommendation of large-only as the production configuration.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <new>
#include <unistd.h>
#include <vector>

#if defined(__APPLE__)
#include <mach/mach.h>
#else
#include <fcntl.h>
#include <sys/syscall.h>
#endif

namespace {

int failures = 0;

// ── Tuning ──────────────────────────────────────────────────────────────────
constexpr size_t kNumObjects = 12000;
constexpr size_t kObjectSize = 64;
constexpr size_t kObjectAlign = 64;

// Hard budget. Expected growth is kNumObjects x (kernel page size):
//    4 KiB ->  47 MiB      16 KiB -> 188 MiB      64 KiB -> 750 MiB
// 384 MiB clears the first two with ~2x headroom and is exceeded ~2x by the
// third.
constexpr size_t kHardBudgetBytes = 384ull * 1024 * 1024;

// Advisory ratio: cost per object relative to the bytes actually requested.
constexpr size_t kWarnRatio = 16;

size_t getCurrentRSSBytes() {
#if defined(__APPLE__)
    mach_task_basic_info_data_t info;
    mach_msg_type_number_t count = MACH_TASK_BASIC_INFO_COUNT;
    if (task_info(mach_task_self(), MACH_TASK_BASIC_INFO,
                  reinterpret_cast<task_info_t>(&info), &count) == KERN_SUCCESS)
        return info.resident_size;
    return 0;
#elif defined(__linux__)
    int fd = open("/proc/self/status", O_RDONLY);
    if (fd < 0) return 0;
    char buf[4096];
    ssize_t n = syscall(SYS_read, fd, buf, sizeof(buf) - 1);
    close(fd);
    if (n <= 0) return 0;
    buf[n] = '\0';
    const char* p = strstr(buf, "VmRSS:");
    if (!p) return 0;
    size_t kb = 0;
    if (sscanf(p, "VmRSS: %zu kB", &kb) != 1) return 0;
    return kb * 1024;
#else
    return 0;
#endif
}

// Is libsmash actually interposed? These tests are meaningful standalone (the
// contract holds for any allocator) but only prove something about smash when
// the preload took effect, so say so plainly in the banner. Matches
// arena_routing_test.cpp, which probes the same export.
bool smashInterposed() {
    return dlsym(RTLD_DEFAULT, "smash_arena_for") != nullptr;
}

void printInterposeBanner() {
    if (smashInterposed()) return;
    printf("WARNING: libsmash is not interposed (smash_arena_for not found) — "
           "this run exercises the system allocator and proves nothing about "
           "smash. The ctest registration sets LD_PRELOAD / "
           "DYLD_INSERT_LIBRARIES.\n");
}

const char* modeName() {
    const char* e = std::getenv("SMASH_LARGE_ONLY");
    return (e && e[0] == '1') ? "large-only" : "full";
}

} // namespace

int main() {
    const long page = sysconf(_SC_PAGESIZE);
    printf("aligned_alloc_footprint_test: mode=%s kernel_page=%ld\n",
           modeName(), page);
    printInterposeBanner();

    if (getCurrentRSSBytes() == 0) {
        printf("SKIP: RSS sampling unavailable on this platform\n");
        return 0;
    }

    std::vector<void*> live;
    live.reserve(kNumObjects);

    // Warm the allocator and the vector's own storage so the measured window
    // contains only the aligned allocations.
    for (size_t i = 0; i < 256; ++i) {
        void* p = ::operator new(kObjectSize, std::align_val_t(kObjectAlign));
        *static_cast<unsigned char*>(p) = 1;
        live.push_back(p);
    }

    const size_t rss_before = getCurrentRSSBytes();

    for (size_t i = live.size(); i < kNumObjects; ++i) {
        void* p = nullptr;
        try {
            p = ::operator new(kObjectSize, std::align_val_t(kObjectAlign));
        } catch (const std::bad_alloc&) {
            fprintf(stderr,
                    "FAIL: std::bad_alloc after %zu aligned allocations of "
                    "%zu bytes (align %zu) — %zu bytes of payload exhausted "
                    "this machine\n",
                    i, kObjectSize, kObjectAlign, i * kObjectSize);
            ++failures;
            break;
        }
        // Touch the object so it is genuinely resident. Only the first byte:
        // the point is that the allocator committed a whole page for it.
        *static_cast<unsigned char*>(p) = static_cast<unsigned char>(i & 0xff);
        live.push_back(p);
    }

    const size_t rss_after = getCurrentRSSBytes();
    const size_t grew = rss_after > rss_before ? rss_after - rss_before : 0;
    const size_t objects = live.size() - 256;
    const size_t requested = objects * kObjectSize;
    const double per_object = objects ? double(grew) / double(objects) : 0.0;
    const double ratio = requested ? double(grew) / double(requested) : 0.0;

    printf("allocated %zu objects x %zu B (align %zu) = %.2f MiB requested\n",
           objects, kObjectSize, kObjectAlign, requested / (1024.0 * 1024.0));
    printf("RSS growth: %.2f MiB (%.0f B/object, %.1fx requested)\n",
           grew / (1024.0 * 1024.0), per_object, ratio);

    if (grew > kHardBudgetBytes) {
        fprintf(stderr,
                "FAIL: %zu aligned allocations of %zu bytes grew RSS by "
                "%.2f MiB, over the %.0f MiB budget (%.0f bytes per "
                "%zu-byte object).\n"
                "      SmashHeap::memalign() sends every alignment > 16 to "
                "LargeAlloc, which rounds up to a full page — so each object "
                "costs one %ld-byte kernel page. On a large-page kernel this "
                "exhausts memory and malloc starts returning nullptr, which "
                "the operator-new wrappers turn into std::bad_alloc.\n"
                "      Workaround: SMASH_LARGE_ONLY=1 passes small aligned "
                "requests through to the system allocator.\n",
                objects, kObjectSize, grew / (1024.0 * 1024.0),
                kHardBudgetBytes / (1024.0 * 1024.0), per_object, kObjectSize,
                page);
        ++failures;
    } else if (ratio > double(kWarnRatio)) {
        printf("WARN: %.1fx amplification (%.0f B of RSS per %zu-byte object). "
               "Over-aligned small allocations each consume a whole page via "
               "LargeAlloc. Under the %.0f MiB hard budget on this %ld-byte-"
               "page kernel, but the same routing costs %.0fx on a 64 KiB-page "
               "aarch64 kernel.\n",
               ratio, per_object, kObjectSize,
               kHardBudgetBytes / (1024.0 * 1024.0), page,
               65536.0 / double(kObjectSize));
    }

    for (void* p : live)
        ::operator delete(p, std::align_val_t(kObjectAlign));

    if (failures) {
        printf("aligned_alloc_footprint_test (%s): %d FAILURES\n",
               modeName(), failures);
        return 1;
    }
    printf("aligned_alloc_footprint_test (%s): passed\n", modeName());
    return 0;
}
