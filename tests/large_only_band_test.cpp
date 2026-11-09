// large_only_band_test.cpp - where the (16 KiB, 1 MiB) band goes.
//
// The band is below SMASH_LARGE_ALLOC_VM_THRESHOLD, so it is never compressed.
// In large-only mode it used to be one mapping per malloc and one munmap per
// free all the same: every malloc faulted all its pages in again, each free
// shot down TLBs, and each pair took mmap_lock for writing, for memory glibc
// already serves everything below 16 KiB from. Large-only now hands the band
// to glibc: a churned block costs no page faults once glibc's dynamic mmap
// threshold has adapted.
//
// BAND_EXPECT says what the registration configures:
//   system  the band is glibc's (SMASH_LARGE_ONLY=1)
//   mapped  a direct mapping per block (full mode)
//   arena   the band is smash's arena (SMASH_MID_BINS=1, or
//           SMASH_LARGE_ALLOC_VM_THRESHOLD=16384)
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <malloc.h>
#include <string>
#include <sys/resource.h>
#include <unistd.h>

#define CHECK(cond, ...)                         \
    do {                                         \
        if (!(cond)) {                           \
            std::printf("FAIL: " __VA_ARGS__);   \
            std::printf("\n");                   \
            return 1;                            \
        }                                        \
    } while (0)

using InArena = int (*)(const void*);
static InArena in_arena;

static long minflt() {
    rusage r{};
    getrusage(RUSAGE_SELF, &r);
    return r.ru_minflt;
}

template <typename Alloc>
static double faultsPerCycle(size_t size, Alloc alloc) {
    // The first free of a glibc mmapped chunk raises its mmap threshold, so
    // warm up before counting.
    for (int i = 0; i < 8; ++i) {
        void* p = alloc();
        std::memset(p, 1, size);
        std::free(p);
    }
    const int kCycles = 200;
    const long before = minflt();
    for (int i = 0; i < kCycles; ++i) {
        void* p = alloc();
        std::memset(p, 1, size);
        std::free(p);
    }
    return static_cast<double>(minflt() - before) / kCycles;
}

int main() {
    in_arena = reinterpret_cast<InArena>(dlsym(RTLD_DEFAULT, "smash_in_arena"));
    if (!in_arena) { std::fprintf(stderr, "FAIL: not running under libsmash\n"); return 1; }
    const char* e = std::getenv("BAND_EXPECT");
    const std::string expect = e ? e : "system";

    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    const size_t kBand = 320u << 10;
    const size_t kPages = kBand / ps;

    // Where a band block lives. glibc's chunks, mmapped ones included, sit
    // behind a header, so never on a page boundary; smash's are page aligned.
    void* p = std::malloc(kBand);
    CHECK(p, "malloc(320 KiB) failed");
    const bool aligned = (reinterpret_cast<uintptr_t>(p) & (ps - 1)) == 0;
    const bool arena = in_arena(p);
    std::free(p);
    std::printf("320 KiB block: %s, %s\n", aligned ? "page aligned" : "behind a header",
                arena ? "in the arena" : "outside the arena");
    if (expect == "system")
        CHECK(!aligned && !arena, "the band is not glibc's: page aligned=%d, arena=%d", aligned, arena);
    else if (expect == "mapped")
        CHECK(aligned && !arena, "full mode's band is not a direct mapping: page aligned=%d, arena=%d", aligned, arena);
    else
        CHECK(arena, "the band is not in the arena");

    // Churn: the page faults a block costs each time round.
    const double f_malloc = faultsPerCycle(kBand, [&] { return std::malloc(kBand); });
    const double f_calloc = faultsPerCycle(kBand, [&] { return std::calloc(1, kBand); });
    const double f_memalign = faultsPerCycle(kBand, [&] {
        void* q = nullptr;
        if (posix_memalign(&q, 4096, kBand)) std::abort();
        return q;
    });
    std::printf("faults per cycle (a block is %zu pages): malloc %.1f, calloc %.1f, posix_memalign %.1f\n",
                kPages, f_malloc, f_calloc, f_memalign);
    if (expect == "system") {
        CHECK(f_malloc < 4, "malloc: %.1f faults per 320 KiB cycle", f_malloc);
        CHECK(f_calloc < 4, "calloc: %.1f faults per 320 KiB cycle", f_calloc);
        CHECK(f_memalign < 4, "posix_memalign: %.1f faults per 320 KiB cycle", f_memalign);
    }

    if (const char* t = std::getenv("SMASH_LARGE_ALLOC_VM_THRESHOLD"); !t) {
        // The edges of the band: the threshold compares the block's pages.
        const size_t mib = 1u << 20;
        void* at = std::malloc(mib);
        void* rounded = std::malloc(mib - 1);
        void* below = std::malloc(mib - ps);
        CHECK(at && rounded && below, "edge allocations failed");
        std::printf("arena: 1 MiB %d, 1 MiB - 1 %d, 1 MiB - 1 page %d\n", in_arena(at) != 0,
                    in_arena(rounded) != 0, in_arena(below) != 0);
        CHECK(in_arena(at), "a 1 MiB block must stay compressible");
        CHECK(in_arena(rounded), "1 MiB - 1 byte takes 256 pages and must stay compressible");
        if (expect == "system") CHECK(!in_arena(below), "1 MiB - 1 page is below the threshold");
        std::free(at); std::free(rounded); std::free(below);
    }

    // A block growing past the threshold has to reach the arena, or a buffer
    // grown by realloc is never compressed, whatever size it gets to.
    {
        const size_t small = 512u << 10, big = 8u << 20;
        auto* q = static_cast<unsigned char*>(std::malloc(small));
        CHECK(q, "malloc(512 KiB) failed");
        for (size_t i = 0; i < small; ++i) q[i] = static_cast<unsigned char>(i * 31 + 7);
        auto* r = static_cast<unsigned char*>(std::realloc(q, big));
        CHECK(r, "realloc to 8 MiB failed");
        for (size_t i = 0; i < small; ++i)
            CHECK(r[i] == static_cast<unsigned char>(i * 31 + 7), "byte %zu lost by the grow", i);
        std::printf("512 KiB grown to 8 MiB: in the arena %d\n", in_arena(r) != 0);
        CHECK(in_arena(r), "a block grown past the threshold is not compressible");
        std::memset(r + small, 2, big - small);

        // And back down: contents survive, whichever allocator ends up with it.
        auto* s = static_cast<unsigned char*>(std::realloc(r, small / 2));
        CHECK(s, "realloc down failed");
        for (size_t i = 0; i < small / 2; ++i)
            CHECK(s[i] == static_cast<unsigned char>(i * 31 + 7), "byte %zu lost by the shrink", i);
        std::free(s);
    }
    // A grow within the band stays where it is cheap, with its contents.
    {
        auto* q = static_cast<unsigned char*>(std::malloc(64u << 10));
        CHECK(q, "malloc(64 KiB) failed");
        std::memset(q, 9, 64u << 10);
        auto* r = static_cast<unsigned char*>(std::realloc(q, 512u << 10));
        CHECK(r, "realloc within the band failed");
        for (size_t i = 0; i < (64u << 10); ++i) CHECK(r[i] == 9, "byte %zu lost within the band", i);
        if (expect == "system") CHECK(!in_arena(r), "a grow within the band moved to the arena");
        std::free(r);
    }
    std::printf("PASS\n");
    return 0;
}
