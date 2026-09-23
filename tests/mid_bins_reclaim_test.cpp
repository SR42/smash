// mid_bins_reclaim_test.cpp - an emptied mid-size chunk that the arena takes
// back must take its compressed state with it.
//
// SMASH_MID_BINS=1 keeps an emptied chunk until the arena runs out, and then
// VmRegion's exhausted hook hands it back (MidAlloc::reclaimEmptyChunks). The
// chunk's pages are exactly the ones most likely to be compressed — they sat
// idle — so returning them must also release their blobs and reset their page
// state, as LargeAlloc::deallocate and Slab::releaseSpan do through the
// compressor's release hook. Otherwise every blob leaks, and the pages go back
// to the arena still marked COMPRESSED for whoever takes them next.
//
// Sequence: fill mid-size blocks, wait until the compressor holds blobs for
// them, free them all (the chunks stay), then exhaust a 1 GiB arena with
// blocks too large for the chunks' runs, so the hook runs and reclaims the
// chunks but nothing reuses them yet. At that point no page of the old chunks
// may still report a blob. Then take the pages again with fresh blocks and
// require them to survive a compress / fault cycle byte-exact.
//
// Registered with SMASH_LARGE_ONLY=1 SMASH_MID_BINS=1 SMASH_VM_GIB=1
// SMASH_COLD_TIMEOUT_SEC=1. One phase per process: arena exhaustion leaves the
// free map in a shape no later phase should inherit.
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <sys/mman.h>
#include <unistd.h>

namespace {

using InArenaFn = int (*)(const void*);
using BlobFn = void* (*)(const void*);
int failures = 0;

void fail(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    std::fprintf(stderr, "FAIL: ");
    std::vfprintf(stderr, fmt, ap);
    std::fputc('\n', stderr);
    va_end(ap);
    ++failures;
}

constexpr size_t kBlock = 320u << 10;   // class 376 KiB: five per 2 MiB chunk
constexpr int kBlocks = 40;             // eight chunks, 16 MiB
constexpr size_t kHog = 64u << 20;      // larger than every chunk run put together
constexpr int kMaxHogs = 64;            // 4 GiB: well past a 1 GiB arena
constexpr size_t kReuse = 2u << 20;     // LargeAlloc blocks that fit a chunk's run
constexpr int kReuseBlocks = 8;

unsigned char pattern(size_t block, size_t off) {
    return static_cast<unsigned char>((off * 7 + block * 29) >> 4);
}

// Pages of the given blocks that the compressor holds a blob for.
size_t blobbed(BlobFn blob, unsigned char* const* ptrs, int n, size_t bytes, size_t ps) {
    size_t c = 0;
    for (int b = 0; b < n; ++b)
        for (size_t o = 0; o < bytes; o += ps)
            c += blob(ptrs[b] + o) != nullptr;
    return c;
}

}  // namespace

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    auto in_arena = reinterpret_cast<InArenaFn>(dlsym(RTLD_DEFAULT, "smash_in_arena"));
    auto blob = reinterpret_cast<BlobFn>(dlsym(RTLD_DEFAULT, "smash_compressed_blob_for"));
    if (!in_arena || !blob) { std::printf("SKIP: not running under libsmash\n"); return 0; }
    const char* bins = std::getenv("SMASH_MID_BINS");
    if (!bins || bins[0] != '1') { std::printf("SKIP: needs SMASH_MID_BINS=1\n"); return 0; }
    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    const size_t block_pages = kBlock / ps;

    // ── 1. mid-size blocks go cold and compress ─────────────────────────────
    unsigned char* mid[kBlocks] = {};
    for (int b = 0; b < kBlocks; ++b) {
        mid[b] = static_cast<unsigned char*>(std::malloc(kBlock));
        if (!mid[b] || !in_arena(mid[b])) {
            fail("mid-size block %d is not in the arena (%p) — bins are not active", b, mid[b]);
            return 1;
        }
        for (size_t o = 0; o < kBlock; ++o) mid[b][o] = pattern(b, o);
    }
    const size_t total_pages = block_pages * kBlocks;
    size_t held = 0;
    for (int i = 0; i < 120 && held < total_pages / 2; ++i) {   // up to 30 s
        usleep(250 * 1000);
        held = blobbed(blob, mid, kBlocks, kBlock, ps);
    }
    std::printf("mid-size blocks: %zu of %zu pages compressed\n", held, total_pages);
    if (held < total_pages / 2) {
        fail("only %zu of %zu pages of cold mid-size blocks were compressed in 30 s; "
             "the reclaim check below would prove nothing", held, total_pages);
        return 1;
    }

    // ── 2. free them all: the chunks are kept, blobs and all ────────────────
    for (int b = 0; b < kBlocks; ++b) std::free(mid[b]);
    const size_t after_free = blobbed(blob, mid, kBlocks, kBlock, ps);
    std::printf("after free, before reclaim: %zu pages still hold a blob\n", after_free);
    if (after_free == 0) {
        fail("freeing the blocks already dropped every blob, so the reclaim check "
             "below cannot discriminate");
        return 1;
    }

    // ── 3. exhaust the arena: the hook reclaims the empty chunks ────────────
    // Each 64 MiB request is larger than the reclaimed runs, so the miss that
    // triggers the hook still falls back to a direct mapping and nothing takes
    // the chunks' pages before they are inspected.
    void* hogs[kMaxHogs] = {};
    int nhogs = 0;
    bool exhausted = false;
    while (nhogs < kMaxHogs) {
        void* h = std::malloc(kHog);
        if (!h) { fail("malloc(64 MiB) returned nullptr"); break; }
        hogs[nhogs++] = h;
        if (!in_arena(h)) { exhausted = true; break; }
    }
    std::printf("arena exhausted after %d x 64 MiB: %s\n", nhogs, exhausted ? "yes" : "no");
    if (!exhausted) {
        fail("4 GiB of 64 MiB blocks never left the arena — is SMASH_VM_GIB=1 set?");
        return 1;
    }
    const size_t stale = blobbed(blob, mid, kBlocks, kBlock, ps);
    std::printf("after reclaim: %zu pages of the returned chunks still hold a blob\n", stale);
    if (stale)
        fail("%zu pages went back to the arena still holding compressed blobs — the "
             "chunk reclaim skipped the compressor's release (blobs leak, and the pages "
             "are handed out again marked compressed)", stale);

    // ── 4. whoever takes the pages next gets them intact ────────────────────
    unsigned char* reuse[kReuseBlocks] = {};
    int in = 0;
    for (int b = 0; b < kReuseBlocks; ++b) {
        reuse[b] = static_cast<unsigned char*>(std::malloc(kReuse));
        if (!reuse[b]) { fail("malloc(2 MiB) after reclaim returned nullptr"); continue; }
        in += in_arena(reuse[b]);
        for (size_t o = 0; o < kReuse; ++o) reuse[b][o] = pattern(b + 100, o);
    }
    std::printf("reused the reclaimed runs: %d of %d blocks in the arena\n", in, kReuseBlocks);
    if (in == 0) fail("no 2 MiB block landed in the reclaimed runs");
    const size_t reuse_pages = (kReuse / ps) * kReuseBlocks;
    size_t cold = 0;
    for (int i = 0; i < 120 && cold < reuse_pages / 2; ++i) {
        usleep(250 * 1000);
        cold = blobbed(blob, reuse, kReuseBlocks, kReuse, ps);
    }
    std::printf("reused blocks: %zu of %zu pages compressed\n", cold, reuse_pages);
    size_t differ = 0;
    for (int b = 0; b < kReuseBlocks; ++b) {
        if (!reuse[b]) continue;
        for (size_t o = 0; o < kReuse; ++o) differ += reuse[b][o] != pattern(b + 100, o);
        std::free(reuse[b]);
    }
    if (differ) fail("%zu bytes of blocks placed on reclaimed pages differ after a "
                     "compress / fault cycle", differ);

    for (int h = 0; h < nhogs; ++h) std::free(hogs[h]);
    if (failures) { std::printf("mid_bins_reclaim_test: %d FAILURES\n", failures); return 1; }
    std::printf("mid_bins_reclaim_test: all checks passed\n");
    return 0;
}
