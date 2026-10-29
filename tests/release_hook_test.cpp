// release_hook_test.cpp - what a large free pays in the compressor's release
// hook (CompressorThread::releaseCompressedPages).
//
// Every large block freed back to the arena goes through that hook, which
// walked every page of the block under its page lock and reset ~15 per-page
// arrays — 16 384 lock/unlock pairs and some 250 000 scattered stores for a
// 64 MiB block — whether or not the compressor had ever run. With
// SMASH_NO_COMPRESSOR=1 (the control an operator reaches for when an
// application slows down) nothing but the allocator had ever written any of
// that state, so the walk was pure overhead in the very run meant to measure
// the allocator alone. Now a free made before the compressor has ever started
// resets only the page-state bytes, in bulk, and skips the walk.
//
//   1. accounting: the pages of freed blocks go through the bulk path when the
//      compressor never started (SMASH_NO_COMPRESSOR=1) and through the walk
//      when it runs — the expectation follows the environment, so one body
//      serves both registrations and "off" must mean off
//   2. reuse after a bulk release: the freed run comes back usable, and calloc
//      of it is zero (a dirty sub-arena run is cleared, a decommitted one is
//      not and need not be)
//
// Needs smash_release_hook_info (diagnostic export) and smash_in_arena.
#include <chrono>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <unistd.h>
#include <vector>

namespace {

using info_fn = int (*)(uint64_t*, uint64_t*);
using in_arena_fn = int (*)(const void*);
info_fn g_info = nullptr;
in_arena_fn g_in_arena = nullptr;
int failures = 0;
const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));

void fail(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    std::fprintf(stderr, "FAIL: ");
    std::vfprintf(stderr, fmt, ap);
    std::fputc('\n', stderr);
    va_end(ap);
    ++failures;
}

bool envIs1(const char* name) {
    const char* v = std::getenv(name);
    return v && v[0] == '1';
}

void counters(uint64_t* walked, uint64_t* bulk) {
    *walked = *bulk = 0;
    if (g_info(walked, bulk) != 0) fail("smash_release_hook_info returned nonzero");
}

// Phase 1: which path the pages of freed arena blocks took.
void testAccounting(bool compressor_off) {
    constexpr size_t kBlocks = 32;
    constexpr size_t kBlockBytes = 8u << 20;   // 8 MiB: above the 1 MiB arena threshold
    const size_t pages_per_block = kBlockBytes / ps;

    std::vector<void*> blocks;
    size_t in_arena = 0;
    for (size_t i = 0; i < kBlocks; ++i) {
        auto* p = static_cast<unsigned char*>(std::malloc(kBlockBytes));
        if (!p) { fail("malloc(%zu) returned null", kBlockBytes); return; }
        for (size_t off = 0; off < kBlockBytes; off += ps) p[off] = static_cast<unsigned char>(i);
        if (g_in_arena(p)) ++in_arena;
        blocks.push_back(p);
    }
    if (in_arena != kBlocks) {
        fail("%zu of %zu blocks landed in the arena; the hook only runs for arena blocks",
             in_arena, kBlocks);
    }

    uint64_t walked0, bulk0;
    counters(&walked0, &bulk0);
    const auto t0 = std::chrono::steady_clock::now();
    for (void* p : blocks) std::free(p);
    const auto t1 = std::chrono::steady_clock::now();
    uint64_t walked1, bulk1;
    counters(&walked1, &bulk1);

    const uint64_t walked = walked1 - walked0, bulk = bulk1 - bulk0;
    const uint64_t expect = static_cast<uint64_t>(in_arena) * pages_per_block;
    const double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    std::printf("release of %zu x %zu MiB: walked=%llu bulk=%llu pages, %.0f us total (%.2f us per block)\n",
                in_arena, kBlockBytes >> 20, static_cast<unsigned long long>(walked),
                static_cast<unsigned long long>(bulk), us, us / (in_arena ? in_arena : 1));

    if (compressor_off) {
        // Nothing but the allocator ever wrote per-page state: no walk at all.
        if (walked != 0) fail("compressor never started, but %llu pages were walked",
                              static_cast<unsigned long long>(walked));
        if (bulk != expect) fail("expected %llu pages through the bulk path, got %llu",
                                 static_cast<unsigned long long>(expect),
                                 static_cast<unsigned long long>(bulk));
    } else {
        // The compressor runs: its per-page state may be live for any page, so
        // every page is walked and none is bulk-released.
        if (walked != expect) fail("compressor running: expected %llu pages walked, got %llu",
                                   static_cast<unsigned long long>(expect),
                                   static_cast<unsigned long long>(walked));
        if (bulk != 0) fail("compressor running, but %llu pages took the bulk path",
                            static_cast<unsigned long long>(bulk));
    }
}

// Phase 2: a run released in bulk is reusable and calloc of it is zero.
void testReuse() {
    constexpr size_t kBlockBytes = 8u << 20;
    auto* p = static_cast<unsigned char*>(std::malloc(kBlockBytes));
    if (!p) { fail("malloc returned null"); return; }
    std::memset(p, 0xA5, kBlockBytes);
    std::free(p);

    // The same call site twice: the second block most likely reuses the run.
    for (int round = 0; round < 2; ++round) {
        auto* z = static_cast<unsigned char*>(std::calloc(kBlockBytes / 64, 64));
        if (!z) { fail("calloc returned null"); return; }
        size_t nonzero = 0;
        for (size_t off = 0; off < kBlockBytes; off += 64)
            if (z[off] != 0) ++nonzero;
        if (nonzero) fail("calloc block had %zu nonzero samples after a bulk release", nonzero);
        std::memset(z, 0x5A, kBlockBytes);   // usable end to end
        std::free(z);
    }
}

}  // namespace

int main() {
    g_info = reinterpret_cast<info_fn>(dlsym(RTLD_DEFAULT, "smash_release_hook_info"));
    g_in_arena = reinterpret_cast<in_arena_fn>(dlsym(RTLD_DEFAULT, "smash_in_arena"));
    if (!g_info || !g_in_arena) {
        std::fprintf(stderr, "FAIL: smash_release_hook_info / smash_in_arena not exported (is libsmash preloaded?)\n");
        return 1;
    }
    const bool compressor_off = envIs1("SMASH_NO_COMPRESSOR");
    std::printf("release_hook: compressor %s, %s mode\n", compressor_off ? "off" : "on",
                envIs1("SMASH_LARGE_ONLY") ? "large-only" : "full");

    testAccounting(compressor_off);
    testReuse();

    if (failures) {
        std::fprintf(stderr, "release_hook: %d failure(s)\n", failures);
        return 1;
    }
    std::printf("release_hook: all tests passed\n");
    return 0;
}
