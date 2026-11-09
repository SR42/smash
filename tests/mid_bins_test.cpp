// mid_bins_test.cpp - SMASH_MID_BINS=1 must make the (16 KiB, 1 MiB) band
// compressible without breaking the allocator or the arena.
//
// Without bins, a block in that band takes LargeAlloc's direct-mmap path: it
// is never placed in the arena, so it is never compressed however cold it
// gets. With bins it comes from a size-binned chunk carved out of the arena.
//
// Three properties, all of which the feature exists for:
//   1. blocks in the band land in the arena and actually compress;
//   2. the data survives — every block keeps its bytes across allocation,
//      neighbouring frees and reuse, and reports a usable size >= requested;
//   3. an emptied chunk goes back to the arena, so a workload that frees a
//      batch of mid-size blocks does not keep the space from a later large
//      allocation;
//   4. realloc copies a binned block's real size — its size lives in the
//      chunk's Span::object_size, not in the slab size-class table.
// The expectation follows the env var, so the off-registration asserts the
// opposite of the on-registration from the same source.
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <malloc.h>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

namespace {

using InArenaFn = int (*)(const void*);
InArenaFn in_arena = nullptr;
int failures = 0;
bool bins_on = false;

void fail(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    std::fprintf(stderr, "FAIL: ");
    std::vfprintf(stderr, fmt, ap);
    std::fputc('\n', stderr);
    va_end(ap);
    ++failures;
}

double residentFraction(unsigned char* p, size_t bytes) {
    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    const size_t pages = bytes / ps;
    if (!pages) return -1;
    std::vector<unsigned char> vec(pages);
    if (mincore(p, pages * ps, vec.data()) != 0) return -1;
    size_t resident = 0;
    for (unsigned char v : vec) resident += (v & 1);
    return static_cast<double>(resident) / pages;
}

}  // namespace

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    in_arena = reinterpret_cast<InArenaFn>(dlsym(RTLD_DEFAULT, "smash_in_arena"));
    if (!in_arena) { std::fprintf(stderr, "FAIL: not running under libsmash\n"); return 1; }
    const char* env = std::getenv("SMASH_MID_BINS");
    bins_on = env && env[0] == '1';
    std::printf("mid_bins_test: SMASH_MID_BINS=%s\n", bins_on ? "1" : "(unset)");

    // ── 1. placement and compression ────────────────────────────────────────
    const size_t sizes[] = { 20u << 10, 100u << 10, 320u << 10, 700u << 10 };
    constexpr int kCopies = 16;
    std::vector<unsigned char*> blocks;
    for (size_t sz : sizes) {
        int inside = 0;
        for (int i = 0; i < kCopies; ++i) {
            auto* p = static_cast<unsigned char*>(std::malloc(sz));
            if (!p) { fail("malloc(%zu) returned nullptr", sz); break; }
            std::memset(p, 0xC3, sz);                 // compressible, every page touched
            if (malloc_usable_size(p) < sz)
                fail("malloc_usable_size(%zu) reported %zu, less than requested",
                     sz, malloc_usable_size(p));
            inside += in_arena(p) ? 1 : 0;
            blocks.push_back(p);
        }
        std::printf("  %4zu KB: %2d/%d in the arena\n", sz >> 10, inside, kCopies);
        if (bins_on && inside != kCopies)
            fail("%zu KB: only %d of %d blocks are in the arena — the band is not "
                 "being binned", sz >> 10, inside, kCopies);
        if (!bins_on && inside != 0)
            fail("%zu KB: %d blocks are in the arena with bins off", sz >> 10, inside);
    }

    // ── 2. the data survives neighbouring churn ─────────────────────────────
    // Tag every block, free half of them (so their slots are reused), allocate
    // a fresh batch into those slots, and require the survivors to be intact.
    for (size_t i = 0; i < blocks.size(); ++i)
        std::memset(blocks[i], static_cast<int>(i & 0xff), 4096);
    std::vector<unsigned char*> reused;
    for (size_t i = 0; i < blocks.size(); i += 2) { std::free(blocks[i]); blocks[i] = nullptr; }
    for (size_t i = 0; i < blocks.size() / 2; ++i) {
        auto* p = static_cast<unsigned char*>(std::malloc(320u << 10));
        if (!p) { fail("malloc during reuse returned nullptr"); break; }
        std::memset(p, 0x5A, 320u << 10);
        reused.push_back(p);
    }
    size_t corrupted = 0;
    for (size_t i = 1; i < blocks.size(); i += 2) {
        if (!blocks[i]) continue;
        unsigned char want = static_cast<unsigned char>(i & 0xff);
        for (size_t o = 0; o < 4096; o += 512)
            if (blocks[i][o] != want) { ++corrupted; break; }
    }
    std::printf("  survivors intact after reuse: %zu corrupted\n", corrupted);
    if (corrupted) fail("%zu surviving blocks were corrupted by reuse of their neighbours",
                        corrupted);

    // ── 3. compression, which is the entire point ───────────────────────────
    if (bins_on) {
        auto* cold = static_cast<unsigned char*>(std::malloc(320u << 10));
        if (!cold) { fail("malloc for the cold probe returned nullptr"); }
        else {
            std::memset(cold, 0x11, 320u << 10);
            double resident = 1.0;
            for (int i = 0; i < 80; ++i) {          // up to 20 s
                usleep(250 * 1000);
                resident = residentFraction(cold, 320u << 10);
                if (resident >= 0 && resident < 0.5) break;
            }
            std::printf("  cold 320 KB block: %.0f%% resident\n", 100 * resident);
            if (!(resident >= 0 && resident < 0.5))
                fail("a cold 320 KB block is still %.0f%% resident — binned blocks are "
                     "in the arena but are not being compressed", 100 * resident);
            // It must still read back correctly once faulted in.
            for (size_t o = 0; o < (320u << 10); o += 4096)
                if (cold[o] != 0x11) { fail("cold block corrupted at offset %zu", o); break; }
            std::free(cold);
        }
    }

    // ── 4. realloc keeps a binned block's bytes ─────────────────────────────
    // realloc has to know the old block's size to copy it, and a binned block
    // is neither a slab object nor a LargeAlloc block: its size lives in
    // Span::object_size. Asking the slab size-class table instead indexes past
    // its end and copies a garbage length, truncating or over-reading the data.
    // Every mid class, grown within the band, grown out of it and shrunk below
    // half (the shrink that actually moves).
    {
        const size_t from[] = { 17u << 10, 20u << 10, 28u << 10, 48u << 10, 100u << 10,
                                192u << 10, 300u << 10, 500u << 10, 700u << 10, 1000u << 10 };
        size_t bad = 0, checked = 0;
        for (size_t sz : from) {
            const size_t to[] = { sz + (sz >> 1), 2u << 20, sz / 3 };
            for (size_t nsz : to) {
                auto* p = static_cast<unsigned char*>(std::malloc(sz));
                if (!p) { fail("malloc(%zu) for realloc returned nullptr", sz); continue; }
                for (size_t o = 0; o < sz; ++o)
                    p[o] = static_cast<unsigned char>((o * 131 + sz) >> 3);
                auto* q = static_cast<unsigned char*>(std::realloc(p, nsz));
                if (!q) { fail("realloc(%zu -> %zu) returned nullptr", sz, nsz); std::free(p); continue; }
                const size_t keep = sz < nsz ? sz : nsz;
                size_t o = 0;
                for (; o < keep; ++o)
                    if (q[o] != static_cast<unsigned char>((o * 131 + sz) >> 3)) break;
                ++checked;
                if (o != keep) {
                    ++bad;
                    fail("realloc(%zu -> %zu) lost the block's data from offset %zu of %zu",
                         sz, nsz, o, keep);
                }
                std::free(q);
            }
        }
        std::printf("  realloc of band blocks: %zu of %zu kept their bytes\n",
                    checked - bad, checked);
    }

    // ── 5. realloc reads only the old block ─────────────────────────────────
    // The garbage length above is not necessarily short. In the build where
    // this was found, the out-of-table read gave ~4 GiB, so memcpy copied the
    // whole NEW size: growing a block read straight through its neighbours
    // (and near the arena's commit edge, into reserved PROT_NONE pages). The
    // bytes check in section 4 cannot see that, so watch the neighbours: fill
    // one whole chunk, let it go cold until its pages leave RSS, grow the
    // first block, and require the others to stay out of RSS. A copy of the
    // right length touches only the first block — plus what the fault handler
    // prefetches around it: up to kPrefetchWindow (2) pages either side,
    // within the span, and a chunk is one span. As root that prefetch shows
    // up as exactly 2 neighbour pages (claudepc, 2026-09-23), so pages within
    // two of the block are allowed; the defect brought back a whole neighbour
    // (60 pages).
    if (bins_on) {
        constexpr size_t kSz = 240u << 10;       // a class nothing above uses: 8 per chunk
        constexpr int kN = 8;
        const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
        unsigned char* nb[kN] = {};
        for (int i = 0; i < kN; ++i) {
            nb[i] = static_cast<unsigned char*>(std::malloc(kSz));
            if (!nb[i]) { fail("malloc(240 KiB) returned nullptr"); break; }
            for (size_t o = 0; o < kSz; ++o) nb[i][o] = static_cast<unsigned char>(i * 17 + o / 4096);
        }
        constexpr size_t kPrefetchWindow = 2;    // include/smash/config.h default
        // Resident pages of the neighbours, excluding those within the
        // prefetch window of the first block (nb[0] as it was, before realloc).
        const auto* first = nb[0];
        auto residentPages = [&](int from) {
            size_t r = 0;
            std::vector<unsigned char> vec(kSz / ps);
            for (int i = from; i < kN; ++i) {
                if (!nb[i] || mincore(nb[i], kSz, vec.data()) != 0) continue;
                for (size_t k = 0; k < vec.size(); ++k) {
                    const unsigned char* a = nb[i] + k * ps;
                    const bool near = first && a + kPrefetchWindow * ps >= first &&
                                      a < first + kSz + kPrefetchWindow * ps;
                    if (!near) r += vec[k] & 1;
                }
            }
            return r;
        };
        size_t before = residentPages(1);
        for (int i = 0; i < 120 && before > 0; ++i) {  // up to 30 s
            usleep(250 * 1000);
            before = residentPages(1);
        }
        const size_t all = (kN - 1) * (kSz / ps);
        std::printf("  neighbours before realloc: %zu of %zu pages resident\n", before, all);
        if (before * 4 > all) {
            fail("the chunk's blocks did not leave RSS (%zu of %zu pages resident); the "
                 "over-read check below would prove nothing", before, all);
        } else if (nb[0]) {
            auto* q = static_cast<unsigned char*>(std::realloc(nb[0], 2 * kSz));
            const size_t after = residentPages(1);
            std::printf("  neighbours after growing the first block: %zu pages resident "
                        "beyond the prefetch window\n", after);
            if (after > before)
                fail("growing one 240 KiB block faulted %zu pages of its neighbours back "
                     "in — realloc copied past the end of the old block", after - before);
            if (!q) fail("realloc(240 KiB -> 480 KiB) returned nullptr");
            else {
                for (size_t o = 0; o < kSz; ++o)
                    if (q[o] != static_cast<unsigned char>(o / 4096)) {
                        fail("realloc lost the block's data at offset %zu", o);
                        break;
                    }
                nb[0] = q;
            }
        }
        for (int i = 0; i < kN; ++i) std::free(nb[i]);
    }

    for (auto* p : blocks) std::free(p);
    for (auto* p : reused) std::free(p);
    if (failures) { std::printf("mid_bins_test: %d FAILURES\n", failures); return 1; }
    std::printf("mid_bins_test: all checks passed\n");
    return 0;
}
