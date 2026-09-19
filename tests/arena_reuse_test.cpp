// arena_reuse_test.cpp - freed memory must be reusable by the compressible
// arena, across size classes and allocation sizes.
//
// Memory that lands outside the VmRegion arena is never compressed. Two things
// used to keep freed arena memory from being reused:
//  - an emptied slab span kept its pages for its own (arena, size class), so a
//    program moving from 64-byte to 128-byte objects consumed a fresh share of
//    the arena for each class;
//  - freed page runs were never coalesced, so a run freed by one size could
//    not serve a larger request (1 MiB runs could not hold a 2 MiB block).
// A long-running program therefore drifted out of the arena for good: before
// the fallback in Slab::allocateNewSpan that was a std::bad_alloc, after it
// simply memory that is never compressed.
//
// With a 1 GiB arena (SMASH_VM_GIB=1), each phase allocates ~70% of it, then
// frees everything before the next. Every phase must land (almost) entirely
// in the arena, measured with the exported smash_in_arena() diagnostic.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <vector>

using InArenaFn = int (*)(const void*);
static InArenaFn in_arena = nullptr;
static int failures = 0;

// Allocate `count` blocks of `size`, touch each, report the fraction in the
// arena, then free them all.
static void phase(const char* name, size_t size, size_t count, double min_frac) {
    std::vector<char*> v;
    v.reserve(count);
    size_t inside = 0;
    for (size_t i = 0; i < count; ++i) {
        char* p = static_cast<char*>(std::malloc(size));
        if (!p) { std::fprintf(stderr, "FAIL: %s: malloc(%zu) returned nullptr\n", name, size); ++failures; break; }
        p[0] = static_cast<char>(i);
        p[size - 1] = static_cast<char>(i);
        inside += in_arena(p) ? 1 : 0;
        v.push_back(p);
    }
    const double frac = v.empty() ? 0 : static_cast<double>(inside) / v.size();
    std::printf("  %-26s %5.1f%% in the arena\n", name, 100 * frac);
    if (frac < min_frac) {
        std::fprintf(stderr, "FAIL: %s: only %.1f%% of the blocks are in the compressible arena "
                             "(need %.0f%%): memory freed by earlier phases was not reused\n",
                     name, 100 * frac, 100 * min_frac);
        ++failures;
    }
    for (char* p : v) std::free(p);
}

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    in_arena = reinterpret_cast<InArenaFn>(dlsym(RTLD_DEFAULT, "smash_in_arena"));
    if (!in_arena) { std::printf("SKIP: not running under libsmash (no smash_in_arena)\n"); return 0; }
    if (const char* lo = std::getenv("SMASH_LARGE_ONLY"); lo && lo[0] == '1') {
        std::printf("SKIP: small objects bypass the arena in large-only mode\n");
        return 0;
    }
    const size_t budget = 700u << 20;   // ~70% of the 1 GiB arena per phase
    const double need = 0.95;
    // Small objects: a different size class each phase.
    phase("small 64 B", 64, budget / 64, need);
    phase("small 128 B", 128, budget / 128, need);
    phase("small 256 B", 256, budget / 256, need);
    phase("small 1 KiB", 1024, budget / 1024, need);
    // Large blocks: each phase needs longer runs than the previous one freed.
    phase("large 1 MiB", 1u << 20, budget >> 20, need);
    phase("large 2 MiB", 2u << 20, budget >> 21, need);
    phase("large 3 MiB", 3u << 20, budget / (3u << 20), need);
    // And back to small objects after the large ones.
    phase("small 64 B again", 64, budget / 64, need);
    if (failures) { std::printf("arena_reuse_test: %d FAILURES\n", failures); return 1; }
    std::printf("arena_reuse_test: all checks passed\n");
    return 0;
}
