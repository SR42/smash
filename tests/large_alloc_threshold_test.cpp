// large_alloc_threshold_test.cpp - SMASH_LARGE_ALLOC_VM_THRESHOLD decides which
// allocations are compressible at all, at runtime.
//
// An allocation below the threshold goes to a direct mapping that the
// compressor never walks, so it stays 100% resident however cold it gets;
// at or above it, the block lands in the VmRegion arena and is compressed like
// any other page. The default is 1 MiB, which leaves a workload whose
// footprint is mid-sized blocks (say 320 KB) with no compression at all —
// hence the runtime override.
//
// The expectation is computed from the env var, so both registrations run
// byte-identical assertions and cannot drift: whatever the threshold is,
// half of it must be outside the arena and twice it must be inside.
//
// SMASH_MID_BINS=1 deliberately changes that: it serves the (16 KiB, 1 MiB)
// band from binned chunks inside the arena, so a sub-threshold size in that
// band IS arena-backed. The expectation follows both settings rather than
// failing on the feature.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <vector>

namespace {

using InArenaFn = int (*)(const void*);
InArenaFn in_arena = nullptr;
int failures = 0;

constexpr size_t kPage = 4096;
constexpr size_t kDefaultThreshold = 1024 * 1024;
constexpr size_t kMaxSmall = 16384;          // kMaxSmallSize
constexpr size_t kMidBandTop = 1024 * 1024;  // top of the binned band

bool midBinsOn() {
    const char* v = std::getenv("SMASH_MID_BINS");
    return v && v[0] == '1';
}

// True when mid-size bins claim this size, i.e. it is in the band AND below
// the threshold (at or above it, LargeAlloc takes the block directly).
bool binnedByMidAlloc(size_t size, size_t threshold) {
    return midBinsOn() && size > kMaxSmall && size < kMidBandTop && size < threshold;
}

// Same rounding the library applies: up to a whole page, floored at one page.
size_t effectiveThreshold() {
    size_t t = kDefaultThreshold;
    if (const char* env = std::getenv("SMASH_LARGE_ALLOC_VM_THRESHOLD")) {
        long long parsed = std::atoll(env);
        if (parsed > 0) t = static_cast<size_t>(parsed);
    }
    t = (t + kPage - 1) & ~(kPage - 1);
    return t < kPage ? kPage : t;
}

// `want_arena` is what the threshold implies for this size.
void check(const char* label, size_t size, bool want_arena) {
    constexpr int kCopies = 8;
    std::vector<void*> blocks;
    int inside = 0;
    for (int i = 0; i < kCopies; ++i) {
        void* p = std::malloc(size);
        if (!p) { std::fprintf(stderr, "FAIL: malloc(%zu) returned nullptr\n", size); ++failures; break; }
        std::memset(p, 0x5A, size > 4096 ? 4096 : size);
        inside += in_arena(p) ? 1 : 0;
        blocks.push_back(p);
    }
    const int want = want_arena ? kCopies : 0;
    std::printf("  %-26s %7zu B  %d/%d in the arena (want %d)\n",
                label, size, inside, kCopies, want);
    if (inside != want) {
        std::fprintf(stderr,
            "FAIL: %s: %zu-byte blocks are %sin the compressible arena, expected %s. "
            "Blocks below the threshold must take the direct mapping (never "
            "compressed); blocks at or above it must enter the VmRegion.\n",
            label, size, inside ? "" : "not ", want_arena ? "in" : "out");
        ++failures;
    }
    for (void* p : blocks) std::free(p);
}

}  // namespace

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    in_arena = reinterpret_cast<InArenaFn>(dlsym(RTLD_DEFAULT, "smash_in_arena"));
    if (!in_arena) { std::printf("SKIP: not running under libsmash\n"); return 0; }

    const size_t t = effectiveThreshold();
    std::printf("large_alloc_threshold_test: effective threshold %zu B\n", t);

    // Below: never compressible. At and above: compressible.
    if (t / 2 > 16384)                       // below kMaxSmallSize it would be a slab object
        check("half the threshold", t / 2, binnedByMidAlloc(t / 2, t));
    check("exactly the threshold", t, true);
    check("twice the threshold", t * 2, true);

    // The size the runtime knob exists for: 320 KB is compressible only when
    // the threshold has been lowered to reach it.
    check("320 KB probe", 320u << 10,
          (320u << 10) >= t || binnedByMidAlloc(320u << 10, t));

    if (failures) { std::printf("large_alloc_threshold_test: %d FAILURES\n", failures); return 1; }
    std::printf("large_alloc_threshold_test: all checks passed\n");
    return 0;
}
