// realloc_test.cpp - realloc without a copy (src/smash_heap.h realloc;
// src/core/large_alloc.h resizeInPlace; src/vm/vm_region.h tryExtendRun).
//
// realloc of a smash block always did malloc + memcpy + free: shrinking a
// 1 GiB block to 512 MiB copied 512 MiB and briefly held 1.5 GiB, growing a
// buffer copied it however much room followed it, and realloc(p, same size)
// copied too. A large block now shrinks in place (its tail pages go back the
// way a free's would), grows in place when the pages after it are free (the
// arena's bump pointer or a sub-arena's tail, a free run of the same owner,
// or mremap for a block outside the arena), and stays put when the new size
// fits its pages; a small or mid block stays put when the new size fits and
// uses more than half of it. Strictly less memory than before: the copy's
// second block never exists.
//
//   1. the process's first large block, 8 MiB -> 12 MiB, nothing behind it:
//      same pointer, data intact (later blocks may have a slab span or a
//      free run still queued for decommit behind them, so this comes first)
//   2. 8 MiB - 100 -> 8 MiB: same pointer (fits its pages)
//   3. 8 MiB -> 4 MiB: same pointer, usable size 4 MiB, the data intact, the
//      arena has 1024 more free pages once the tail's decommit has run
//   4. 8 MiB -> 12 MiB with a block right behind it: moved, data intact
//   5. full mode: a 100 B block to its usable size and to half of it plus
//      one: same pointer
//   6. REALLOC_COMMIT_CASE=1 (run alone, SMASH_COLD_TIMEOUT_SEC=1): the tail
//      a grow takes past the arena's bump pointer must be committed like any
//      fresh pages (VmRegion::ensureCommitted). It was not: the pages stayed
//      PROT_NONE with the commit watermark below them (the fault handler
//      healed each first touch, so nothing showed), and the next
//      allocation's commit swept mprotect(PROT_RW) over them -- by then
//      compressed and dropped. Reads returned zeros without a fault, writes
//      landed on memory the compressor owned and were lost at the next drop,
//      and a later restore brought the old blob back: that is how
//      test_integrity_stress_small_heap failed once on claudepc (units 51-87
//      of a block grown at round 18432 read zeros, then versions behind).
//      Here: the first block grown 8 -> 12 MiB, its tail filled and left
//      until every tail page is compressed and non-resident (mincore), then
//      a 1 MiB malloc (the commit); the tail must read back intact.
//
// Registered large-only, large-only with SMASH_SUBARENAS=1 (the sub-arena
// tail path), in full mode (the small-block path too), and case 6 alone.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <malloc.h>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

namespace {
int failures = 0;
void fail(const char* what) { std::fprintf(stderr, "FAIL: %s\n", what); ++failures; }
unsigned char pat(size_t off) { return static_cast<unsigned char>((off >> 12) * 7 + (off >> 6) + 1); }
void fill(unsigned char* p, size_t n) { for (size_t off = 0; off < n; off += 64) p[off] = pat(off); }
size_t wrong(const unsigned char* p, size_t n) { size_t b = 0; for (size_t off = 0; off < n; off += 64) b += p[off] != pat(off); return b; }
}  // namespace

int main() {
    using arena_fn = int (*)(size_t*, size_t*);
    auto arena = reinterpret_cast<arena_fn>(dlsym(RTLD_DEFAULT, "smash_shared_arena_info"));
    if (!arena) { std::fprintf(stderr, "FAIL: smash_shared_arena_info missing\n"); return 1; }
    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    const size_t kBig = 8u << 20;
    const char* lo = std::getenv("SMASH_LARGE_ONLY");
    const bool full_mode = !(lo && lo[0] == '1');

    // 6. the grown tail past the bump pointer is committed; a later commit
    //    must not make its compressed pages readable (see the header)
    if (const char* cc = std::getenv("REALLOC_COMMIT_CASE"); cc && cc[0] == '1') {
        using info_fn = int (*)(const void*, int*, int*, int*);
        auto page_info = reinterpret_cast<info_fn>(dlsym(RTLD_DEFAULT, "smash_page_info"));
        if (!page_info) { std::fprintf(stderr, "FAIL: smash_page_info missing\n"); return 1; }
        auto* p = static_cast<unsigned char*>(std::malloc(kBig));
        fill(p, kBig);
        auto* q = static_cast<unsigned char*>(std::realloc(p, kBig + kBig / 2));
        if (q != p) { std::printf("realloc: the first block did not grow in place; nothing to test\n"); return 1; }
        unsigned char* tail = q + kBig;
        const size_t tail_bytes = kBig / 2, tail_pages = tail_bytes / ps;
        fill(tail, tail_bytes);
        // Wait until every tail page is compressed and its memory dropped.
        std::vector<unsigned char> res(tail_pages);
        size_t ready = 0;
        int waited = 0;
        for (; waited < 90000 && ready < tail_pages; waited += 250) {
            usleep(250000);
            ready = 0;
            if (mincore(tail, tail_bytes, res.data()) != 0) { perror("mincore"); return 1; }
            for (size_t i = 0; i < tail_pages; ++i) {
                int st = 0, c = 0, r = 0;
                if (page_info(tail + i * ps, &st, &c, &r) == 0 && st == 2 && !(res[i] & 1)) ++ready;
            }
        }
        std::printf("realloc: %zu of %zu tail pages compressed and dropped after %d ms\n", ready, tail_pages, waited);
        if (ready < tail_pages) fail("the grown tail did not get compressed and dropped in 90 s");
        // The next fresh allocation commits the arena past the tail.
        auto* r = static_cast<unsigned char*>(std::malloc(1u << 20));
        r[0] = 1;
        size_t zero = 0;
        for (size_t off = 0; off < tail_bytes; off += ps) zero += tail[off] == 0 && tail[off + 64] == 0;
        const size_t bad = wrong(tail, tail_bytes);
        std::printf("realloc: after a 1 MiB malloc the tail reads back with %zu wrong bytes (%zu pages all zero)\n", bad, zero);
        if (bad) fail("the grown tail read back wrong after a later commit: its pages were left above the commit watermark");
        std::free(r);
        std::free(q);
        if (failures) { std::fprintf(stderr, "realloc: %d failure(s)\n", failures); return 1; }
        std::printf("realloc: all tests passed\n");
        return 0;
    }

    // 1. grow in place into free pages behind the block: the process's first
    //    large block is the last one carved, nothing sits behind it yet
    {
        auto* p = static_cast<unsigned char*>(std::malloc(kBig));
        fill(p, kBig);
        auto* q = static_cast<unsigned char*>(std::realloc(p, kBig + kBig / 2));
        std::printf("realloc: 8 MiB -> 12 MiB, nothing behind it: %s pointer\n", q == p ? "same" : "moved");
        if (q != p) fail("growing into free pages behind the block moved it (copied 8 MiB)");
        if (wrong(q, kBig)) fail("data wrong after growing in place");
        fill(q, kBig + kBig / 2);
        if (wrong(q, kBig + kBig / 2)) fail("the grown block does not hold its new size");
        std::free(q);
    }

    // 2. grow within the block's own pages
    {
        auto* p = static_cast<unsigned char*>(std::malloc(kBig - 100));
        fill(p, kBig - 100);
        auto* q = static_cast<unsigned char*>(std::realloc(p, kBig));
        std::printf("realloc: 8 MiB - 100 -> 8 MiB: %s pointer\n", q == p ? "same" : "moved");
        if (q != p) fail("growing within the block's own pages moved it");
        if (wrong(q, kBig - 100)) fail("data wrong after growing within the pages");
        std::free(q);
    }

    // 3. shrink in place
    {
        auto* p = static_cast<unsigned char*>(std::malloc(kBig));
        if (!p) return 1;
        fill(p, kBig);
        size_t used0 = 0, free0 = 0;
        arena(&used0, &free0);
        auto* q = static_cast<unsigned char*>(std::realloc(p, kBig / 2));
        // The tail goes through the decommit queue before it is free again.
        size_t used1 = 0, free1 = 0;
        for (int w = 0; w < 3000; w += 50) {
            arena(&used1, &free1);
            if (free1 >= free0 + (kBig / 2) / ps) break;
            usleep(50000);
        }
        const size_t usable = malloc_usable_size(q);
        std::printf("realloc: 8 MiB -> 4 MiB: %s pointer, usable %zu KiB, arena free pages %zu -> %zu\n",
                    q == p ? "same" : "moved", usable >> 10, free0, free1);
        if (q != p) fail("shrinking a large block moved it (copied 4 MiB)");
        if (usable < kBig / 2 || usable >= kBig) fail("usable size after the shrink is not 4 MiB");
        if (wrong(q, kBig / 2)) fail("data wrong after the shrink");
        if (free1 < free0 + (kBig / 2) / ps) fail("the shrunk tail did not go back to the arena");
        std::free(q);
    }

    // 4. grow blocked by a neighbour: moved, data intact
    {
        auto* p = static_cast<unsigned char*>(std::malloc(kBig));
        auto* r = static_cast<unsigned char*>(std::malloc(1u << 20));
        fill(p, kBig);
        r[0] = 1;
        auto* q = static_cast<unsigned char*>(std::realloc(p, kBig + kBig / 2));
        const bool behind = r > p && r < p + kBig + kBig / 2;
        std::printf("realloc: 8 MiB -> 12 MiB with a block %s: %s pointer\n", behind ? "right behind it" : "elsewhere", q == p ? "same" : "moved");
        if (wrong(q, kBig)) fail("data wrong after a moving grow");
        if (behind && q == p) fail("grew in place over a live neighbour");
        std::free(q);
        std::free(r);
    }

    // 5. small blocks stay put when the new size fits (full mode)
    if (full_mode) {
        auto* p = static_cast<unsigned char*>(std::malloc(100));
        const size_t u = malloc_usable_size(p);
        std::memset(p, 0x5A, 100);
        auto* q = static_cast<unsigned char*>(std::realloc(p, u));
        auto* r = static_cast<unsigned char*>(std::realloc(q, u / 2 + 1));
        std::printf("realloc: 100 B (usable %zu) -> %zu -> %zu: %s, %s\n", u, u, u / 2 + 1, q == p ? "same" : "moved", r == q ? "same" : "moved");
        if (q != p) fail("a small block grown to its usable size moved");
        if (r != q) fail("a small block shrunk to over half its usable size moved");
        for (size_t i = 0; i < 50; ++i) if (r[i] != 0x5A) { fail("small block data wrong"); break; }
        std::free(r);
    }

    if (failures) { std::fprintf(stderr, "realloc: %d failure(s)\n", failures); return 1; }
    std::printf("realloc: all tests passed\n");
    return 0;
}
