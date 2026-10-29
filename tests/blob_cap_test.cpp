// blob_cap_test.cpp - the blob store's cap and what a refused page does next
// (src/compress/compress_store.h SMASH_BLOB_MAX_MIB; compressPage's
// store-failure path).
//
// The store had no cap: compressed pages were kept until mmap failed. And a
// page whose blob the store could not take went back to ACTIVE with its cold
// streak intact and no failure counted, so it was selected, snapshotted,
// compressed and refused again on every tick, for ever, with nothing printed.
// A refusal now counts as a compression failure (the cold streak restarts and
// the effective floor doubles, as after a ratio-gate failure), and the first
// one warns.
//
//   1. cap: 64 MiB of half-compressible data against an 8 MiB cap; the bytes
//      the stores hold stay within the cap (plus one region's slack), the
//      data reads back intact, and at least one refusal was counted
//   2. back-off: over a later 16 s window each page the store refused is
//      refused at most twice more (its retries come at doubling idle
//      intervals: nominal ticks 8 and 16); the test allows 4 per page.
//      Without the back-off every tick refused every one of them again --
//      8 per page measured, ticks running long on the re-compression
//
// Needs smash_resilience_info, smash_blob_usage and smash_page_info.
// Registered with SMASH_BLOB_MAX_MIB=8 SMASH_COLD_TIMEOUT_SEC=1
// SMASH_DEFER_PHASES_MS=0.
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <thread>
#include <unistd.h>

namespace {
int failures = 0;
void fail(const char* what) { std::fprintf(stderr, "FAIL: %s\n", what); ++failures; }
using res_fn = int (*)(uint64_t*, uint64_t*);
using use_fn = int (*)(size_t*, size_t*);
using info_fn = int (*)(const void*, int*, int*, int*);
res_fn g_res = nullptr;
use_fn g_use = nullptr;
info_fn g_info = nullptr;

uint64_t storeFails() { uint64_t f = 0, r = 0; g_res(&f, &r); return f; }

// Half random, half zero per 64 bytes: ~50 % with LZ4/zstd, under the 75 %
// gate and far from the 16:1 a repeated pattern gives.
void fillHalfCompressible(unsigned char* p, size_t bytes, uint32_t seed) {
    uint32_t x = seed | 1;
    for (size_t i = 0; i < bytes; i += 64) {
        for (int k = 0; k < 32; ++k) { x = x * 1664525u + 1013904223u; p[i + k] = static_cast<unsigned char>(x >> 24); }
        std::memset(p + i + 32, 0, 32);
    }
}
bool checkHalfCompressible(const unsigned char* p, size_t bytes, uint32_t seed) {
    uint32_t x = seed | 1;
    for (size_t i = 0; i < bytes; i += 64) {
        for (int k = 0; k < 32; ++k) { x = x * 1664525u + 1013904223u; if (p[i + k] != static_cast<unsigned char>(x >> 24)) return false; }
        for (int k = 32; k < 64; ++k) if (p[i + k] != 0) return false;
    }
    return true;
}
}  // namespace

int main() {
    g_res = reinterpret_cast<res_fn>(dlsym(RTLD_DEFAULT, "smash_resilience_info"));
    g_use = reinterpret_cast<use_fn>(dlsym(RTLD_DEFAULT, "smash_blob_usage"));
    g_info = reinterpret_cast<info_fn>(dlsym(RTLD_DEFAULT, "smash_page_info"));
    if (!g_res || !g_use || !g_info) { std::fprintf(stderr, "FAIL: diagnostic exports missing\n"); return 1; }
    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    const char* capv = std::getenv("SMASH_BLOB_MAX_MIB");
    const size_t cap = (capv ? static_cast<size_t>(std::atol(capv)) : 0) << 20;
    if (!cap) { std::fprintf(stderr, "FAIL: SMASH_BLOB_MAX_MIB must be set\n"); return 1; }

    const size_t bytes = 64u << 20;
    auto* p = static_cast<unsigned char*>(std::malloc(bytes));
    if (!p) { fail("malloc"); return 1; }
    fillHalfCompressible(p, bytes, 42);

    // Window 1: the first ticks after the block goes cold.
    std::this_thread::sleep_for(std::chrono::seconds(2));
    const uint64_t f0 = storeFails();
    std::this_thread::sleep_for(std::chrono::seconds(4));
    const uint64_t f1 = storeFails();
    size_t live = 0, held = 0;
    g_use(&live, &held);
    std::printf("  after 6 s: %llu refusals so far (%llu in the last 4 s), blobs live %zu MiB, held %zu MiB (cap %zu)\n",
                static_cast<unsigned long long>(f1), static_cast<unsigned long long>(f1 - f0),
                live >> 20, held >> 20, cap >> 20);
    if (f1 == 0) fail("no refusal counted: 32 MiB of blobs must not fit an 8 MiB cap");
    if (held > cap + (16u << 20)) fail("the stores hold more than the cap plus one region");

    // Window 2: with the back-off a refused page comes back at doubling idle
    // intervals (nominal ticks 2 and 4 fall in the first window, 8 and 16 in
    // this one, one fewer under write-protect monitoring), so each page the
    // store still refuses is refused about twice more here. Without it every
    // tick refused every one of them again: 8 per page measured.
    std::this_thread::sleep_for(std::chrono::seconds(16));
    const uint64_t f2 = storeFails();
    size_t stored = 0;
    for (size_t off = 0; off < bytes; off += ps) {
        int st = 0, c = 0, r = 0;
        if (g_info(p + off, &st, &c, &r) == 0 && st == 2) ++stored;
    }
    const size_t refused_pages = bytes / ps > stored ? bytes / ps - stored : 1;
    const double per_page = static_cast<double>(f2 - f1) / static_cast<double>(refused_pages);
    std::printf("  next 16 s: %llu refusals (first 4 s window: %llu); %zu pages stored, %zu refused: %.1f refusals per refused page\n",
                static_cast<unsigned long long>(f2 - f1), static_cast<unsigned long long>(f1 - f0),
                stored, refused_pages, per_page);
    if (per_page > 4.0)
        fail("refusals did not back off: a refused page is re-selected every tick");

    if (!checkHalfCompressible(p, bytes, 42)) fail("data changed under the cap");
    g_use(&live, &held);
    if (held > cap + (16u << 20)) fail("the stores hold more than the cap plus one region (after read-back)");
    std::free(p);

    if (failures) { std::fprintf(stderr, "blob_cap: %d failure(s)\n", failures); return 1; }
    std::printf("blob_cap: all tests passed\n");
    return 0;
}
