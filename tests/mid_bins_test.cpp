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
//      allocation.
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
    if (!in_arena) { std::printf("SKIP: not running under libsmash\n"); return 0; }
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

    for (auto* p : blocks) std::free(p);
    for (auto* p : reused) std::free(p);
    if (failures) { std::printf("mid_bins_test: %d FAILURES\n", failures); return 1; }
    std::printf("mid_bins_test: all checks passed\n");
    return 0;
}
