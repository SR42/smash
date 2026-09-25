// large_free_span_cache_test.cpp - a large free must never leave a stale
// entry in free()'s per-thread span cache.
//
// free() keeps the last span it resolved in a TLS cache (the Span* and its
// [base, end) range), stamped with g_pagemap_generation; a page-map teardown
// bumps the generation so a stale entry misses. Large blocks do not enter
// that cache, so freeing one does not bump the generation: the bump was one
// global cache line written by every large free, and on the 128-core aarch64
// box it was 49% of smash_sim's time (PageMap::set) at 128 threads.
//
// What breaks if a large span enters the cache while large frees skip the
// bump: span descriptors are recycled, so the stale entry's Span* soon
// describes some other block. Here: free a 1 MiB block A (cached), allocate a
// 1 MiB block B at once (A's pages are still in flight to the free map, so B
// lands elsewhere, and it takes A's recycled descriptor), let small objects
// fill A's old range, and free one of them. The stale entry routes that free
// to the descriptor — now B — and B is freed while live: its pages are
// decommitted and handed out again. B must still hold its pattern.
//
// Full mode (the TLS cache is on the full free() fast path), 1 GiB arena.
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
    if (!in_arena) { std::printf("SKIP: not running under libsmash\n"); return 0; }
    const char* lo = std::getenv("SMASH_LARGE_ONLY");
    if (lo && lo[0] == '1') { std::printf("SKIP: needs full mode\n"); return 0; }

    constexpr size_t kBig = 1u << 20;
    constexpr size_t kSmall = 64;
    auto* a = static_cast<unsigned char*>(malloc(kBig));
    if (!a || !in_arena(a)) { std::printf("SKIP: the 1 MiB block is not in the arena\n"); return 0; }
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
    auto* b = static_cast<unsigned char*>(malloc(kBig));   // takes A's descriptor
    if (!b) { std::printf("SKIP: second 1 MiB block failed\n"); return 0; }
    std::memset(b, 0xAB, kBig);
    const bool b_elsewhere = reinterpret_cast<uintptr_t>(b) + kBig <= lo_addr ||
                             reinterpret_cast<uintptr_t>(b) >= hi_addr;
    usleep(100 * 1000);   // A's run reaches the free map through the decommit queue

    // Small objects until one lands in A's old range; their pointers live in
    // a mapping of our own, not the heap.
    constexpr size_t kMax = 8u << 20;
    auto** objs = static_cast<void**>(mmap(nullptr, kMax * sizeof(void*), PROT_READ | PROT_WRITE,
                                           MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0));
    if (objs == MAP_FAILED) { std::printf("SKIP: mmap failed\n"); return 0; }
    size_t n = 0, inside = kMax;
    for (; n < kMax; ++n) {
        void* p = malloc(kSmall);
        if (!p) break;
        objs[n] = p;
        const auto x = reinterpret_cast<uintptr_t>(p);
        if (x >= lo_addr && x < hi_addr) { inside = n++; break; }
    }
    if (inside == kMax) {
        std::printf("SKIP: no small object landed in the freed block's range (%zu tried)\n", n);
        return 0;
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
    for (size_t i = 0; i < kBig; ++i) bad += b[i] != 0xAB;
    std::printf("live 1 MiB block B: %zu of %zu bytes changed\n", bad, kBig);
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
