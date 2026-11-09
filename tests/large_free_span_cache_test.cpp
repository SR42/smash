// large_free_span_cache_test.cpp - a free inside a freed large block's old
// range must reach its own span, never whatever a stale address-keyed entry
// names.
//
// Written for free()'s per-thread last-span cache (the Span* and its
// [base, end) range, stamped with a global page-map generation that large
// frees did not bump: that bump was 49% of smash_sim's time at 128 threads on
// the 128-core aarch64 box). Upstream #93 removed that cache; the lean free()
// now routes by a per-page tag (VmRegion::page_tag_), which is keyed by
// address the same way, so the scenario still applies.
//
// Span descriptors are recycled, so a stale entry's Span* soon describes some
// other block. Here: free a 1 MiB block A, allocate a 2 MiB block B at once
// (it takes A's recycled descriptor), let small objects fill A's old range,
// and free one of them. A stale entry routes that free to the descriptor --
// now B -- and B is freed while live: its pages are decommitted and handed
// out again. B must still hold its pattern. B is larger than A so that it
// cannot take A's run: until 2026-11-09 B was 1 MiB too, on the assumption
// that A's pages were still in flight to the free map, but since the upstream
// merge B came back at A's address, no small object could land in A's range,
// and the test skipped on every run (reported as a pass).
//
// Full mode (the lean free and the TLS cache are full-mode paths), 1 GiB arena.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

int main() {
    auto in_arena = reinterpret_cast<int (*)(const void*)>(dlsym(RTLD_DEFAULT, "smash_in_arena"));
    if (!in_arena) { std::fprintf(stderr, "FAIL: not running under libsmash\n"); return 1; }
    const char* lo = std::getenv("SMASH_LARGE_ONLY");
    if (lo && lo[0] == '1') { std::printf("SKIP: needs full mode\n"); return 77; }

    constexpr size_t kBig = 1u << 20;
    constexpr size_t kBigB = 2u << 20;
    constexpr size_t kSmall = 64;
    auto* a = static_cast<unsigned char*>(malloc(kBig));
    if (!a || !in_arena(a)) { std::fprintf(stderr, "FAIL: the 1 MiB block is not in the arena (%p)\n", static_cast<void*>(a)); return 1; }
    std::memset(a, 1, kBig);
    const auto lo_addr = reinterpret_cast<uintptr_t>(a);
    const auto hi_addr = lo_addr + kBig;
    // The cache refills only every Nth miss after a run of misses, and a hit
    // resets that: free 64 neighbours from one span (the later frees hit), so
    // the free of A below is a miss that fills the cache if large spans are
    // allowed in.
    {
        void* warm[64];
        for (auto& w : warm) w = malloc(kSmall);
        for (auto& w : warm) free(w);
    }
    free(a);
    auto* b = static_cast<unsigned char*>(malloc(kBigB));   // takes A's descriptor
    if (!b) { std::fprintf(stderr, "FAIL: the 2 MiB malloc returned nullptr\n"); return 1; }
    std::memset(b, 0xAB, kBigB);
    const bool b_elsewhere = reinterpret_cast<uintptr_t>(b) + kBigB <= lo_addr ||
                             reinterpret_cast<uintptr_t>(b) >= hi_addr;
    usleep(100 * 1000);   // A's run reaches the free map through the decommit queue

    // Small objects until one lands in A's old range; their pointers live in
    // a mapping of our own, not the heap.
    constexpr size_t kMax = 8u << 20;
    auto** objs = static_cast<void**>(mmap(nullptr, kMax * sizeof(void*), PROT_READ | PROT_WRITE,
                                           MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0));
    if (objs == MAP_FAILED) { std::printf("SKIP: mmap failed\n"); return 77; }
    size_t n = 0, inside = kMax;
    for (; n < kMax; ++n) {
        void* p = malloc(kSmall);
        if (!p) { std::fprintf(stderr, "FAIL: malloc(%zu) returned nullptr after %zu objects\n", kSmall, n); return 1; }
        objs[n] = p;
        const auto x = reinterpret_cast<uintptr_t>(p);
        if (x >= lo_addr && x < hi_addr) { inside = n++; break; }
    }
    if (inside == kMax) {
        // The scenario did not arise: nothing was checked.
        std::printf("SKIP: no small object landed in the freed block's range (%zu tried; B at %p, A at %p)\n",
                    n, static_cast<void*>(b), static_cast<void*>(a));
        return 77;
    }
    std::printf("B %s A's range; small object %zu landed in A's old range\n",
                b_elsewhere ? "is outside" : "overlaps", inside);
    free(objs[inside]);   // must go to its own slab, not to whatever the stale entry names
    objs[inside] = nullptr;

    usleep(100 * 1000);   // a wrongly freed B would be decommitted by now
    std::vector<unsigned char*> more;
    more.reserve(64);
    for (int i = 0; i < 64; ++i) {
        auto* m = static_cast<unsigned char*>(malloc(kBig));
        if (!m) break;
        std::memset(m, 0xEE, kBig);
        more.push_back(m);
    }
    size_t bad = 0;
    for (size_t i = 0; i < kBigB; ++i) bad += b[i] != 0xAB;
    std::printf("live 2 MiB block B: %zu of %zu bytes changed\n", bad, kBigB);
    for (auto* m : more) free(m);
    for (size_t i = 0; i < n; ++i) if (objs[i]) free(objs[i]);
    if (bad) {
        std::printf("FAIL: a free inside a freed large block's range reached a stale span cache "
                    "entry and freed the live block B that its descriptor now describes\n");
        return 1;
    }
    free(b);
    std::printf("large_free_span_cache_test: all checks passed\n");
    return 0;
}
