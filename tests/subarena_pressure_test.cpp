// subarena_pressure_test.cpp - dirty memory under a burst of frees nothing
// reallocates (SMASH_SUBARENAS; design: agents/design-subarenas.md).
//
// A sub-arena's dirty runs are purged by one thread per set — the node's or
// the shared set's decommit thread — which a free that crosses the cap wakes.
// Churn that reallocates what it frees never needs it (the dirty run is taken
// back at once), but a burst of frees that nothing takes — a working set
// shrinking, or blocks freed on CPUs that never allocate — handed that one
// thread the whole burst: with 8 threads each freeing 128 MiB at once against
// an 8 MiB cap, the dirty bytes of the set stood at the full gigabyte until
// that thread had madvised all of it, with no bound and no warning. Now a
// free that takes its sub-arena past twice the cap purges the excess on the
// freeing thread, under the lock it already holds, so no sub-arena is ever
// more than twice its cap plus one block over.
//
//   1. burst: threads allocate a batch of blocks, touch them, free them all,
//      repeat; the most any sub-arena held dirty at once must be at most
//      2 x cap + one block (smash_subarena_pressure), and at least one purge
//      must have run inline — a purger thread alone cannot keep that bound
//   2. still correct: after the bursts every freed block is gone from the
//      sub-arenas within the decay time (dirty bytes drain to 0), and blocks
//      allocated after the bursts are usable and calloc of them is zero
//
// Registered with SMASH_DIRTY_MAX_MIB=8 and SMASH_NO_COMPRESSOR=1 (the
// compressor plays no part in dirty reuse; keeping it out keeps the bound
// exact). Needs smash_subarena_pressure and smash_subarena_info.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

using info_fn = int (*)(uint64_t*, uint64_t*);
info_fn g_info = nullptr;
info_fn g_pressure = nullptr;
int failures = 0;
const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
size_t cap_bytes = 0;

void fail(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    std::fprintf(stderr, "FAIL: ");
    std::vfprintf(stderr, fmt, ap);
    std::fputc('\n', stderr);
    va_end(ap);
    ++failures;
}

constexpr size_t kBlock = 4u << 20;      // 4 MiB, in the arena
constexpr size_t kBatch = 32;            // 128 MiB per thread per round
constexpr int kRounds = 6;

// Phase 1: bursts of frees, and the bound on dirty memory.
void testBurst() {
    unsigned n = std::thread::hardware_concurrency();
    n = std::clamp<unsigned>(n, 4, 16);
    std::atomic<unsigned> ready{0};
    std::atomic<bool> go{false};
    std::atomic<uint64_t> free_ns_max{0};
    std::vector<std::thread> th;
    for (unsigned t = 0; t < n; ++t) th.emplace_back([&, t] {
        ready.fetch_add(1);
        while (!go.load()) std::this_thread::yield();
        std::vector<unsigned char*> blocks(kBatch);
        for (int r = 0; r < kRounds; ++r) {
            for (size_t i = 0; i < kBatch; ++i) {
                blocks[i] = static_cast<unsigned char*>(std::malloc(kBlock));
                if (!blocks[i]) { fail("malloc returned null"); return; }
                for (size_t off = 0; off < kBlock; off += ps) blocks[i][off] = static_cast<unsigned char>(t);
            }
            const auto t0 = std::chrono::steady_clock::now();
            for (size_t i = 0; i < kBatch; ++i) std::free(blocks[i]);
            const auto ns = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t0).count());
            uint64_t prev = free_ns_max.load();
            while (ns > prev && !free_ns_max.compare_exchange_weak(prev, ns)) {}
            // A pause nothing allocates in: the burst stays freed.
            usleep(2000);
        }
    });
    while (ready.load() < n) std::this_thread::yield();
    go.store(true);

    for (auto& t : th) t.join();
    uint64_t d = 0, p = 0;
    g_info(&d, &p);
    uint64_t peak = 0, inl = 0;
    g_pressure(&peak, &inl);
    const uint64_t bound = 2 * cap_bytes + kBlock;
    std::printf("  %u threads x %d rounds of %zu x %zu MiB: peak dirty in one sub-arena %llu MiB "
                "(bound %llu MiB), %llu inline purges, %llu purged runs, slowest burst of %zu frees %.1f ms\n",
                n, kRounds, kBatch, kBlock >> 20, static_cast<unsigned long long>(peak >> 20),
                static_cast<unsigned long long>(bound >> 20), static_cast<unsigned long long>(inl),
                static_cast<unsigned long long>(p), kBatch, free_ns_max.load() / 1e6);
    if (peak > bound)
        fail("a sub-arena held %llu bytes dirty at once; the bound is 2 x cap + one block = %llu",
             static_cast<unsigned long long>(peak), static_cast<unsigned long long>(bound));
    if (inl == 0)
        fail("no purge ran on a freeing thread: %zu MiB freed at once per thread against an %zu MiB cap "
             "must take some past twice the cap", (kBatch * kBlock) >> 20, cap_bytes >> 20);
}

// Phase 2: the sub-arenas drain and stay correct.
void testAfter() {
    const char* decay = std::getenv("SMASH_DIRTY_DECAY_MS");
    const int decay_ms = decay ? std::atoi(decay) : 1000;
    uint64_t d = 0, p = 0;
    for (int waited = 0; waited < decay_ms * 4 + 500; waited += 50) {
        g_info(&d, &p);
        if (d == 0) break;
        usleep(50000);
    }
    if (d != 0) fail("%llu bytes still dirty %d ms after the last free (decay %d ms)",
                     static_cast<unsigned long long>(d), decay_ms * 4 + 500, decay_ms);

    for (int round = 0; round < 4; ++round) {
        auto* z = static_cast<unsigned char*>(std::calloc(kBlock / 64, 64));
        if (!z) { fail("calloc returned null"); return; }
        size_t nonzero = 0;
        for (size_t off = 0; off < kBlock; off += 64) nonzero += z[off] != 0;
        if (nonzero) fail("calloc block had %zu nonzero samples", nonzero);
        std::memset(z, 0xA5, kBlock);
        std::free(z);
    }
}

}  // namespace

int main() {
    g_info = reinterpret_cast<info_fn>(dlsym(RTLD_DEFAULT, "smash_subarena_info"));
    g_pressure = reinterpret_cast<info_fn>(dlsym(RTLD_DEFAULT, "smash_subarena_pressure"));
    if (!g_info || !g_pressure) {
        std::fprintf(stderr, "FAIL: smash_subarena_info / smash_subarena_pressure not exported\n");
        return 1;
    }
    uint64_t d = 0, p = 0;
    if (g_info(&d, &p) != 0) {
        std::fprintf(stderr, "FAIL: sub-arenas are off (SMASH_SUBARENAS=1 required)\n");
        return 1;
    }
    const char* cap = std::getenv("SMASH_DIRTY_MAX_MIB");
    cap_bytes = cap ? static_cast<size_t>(std::atol(cap)) << 20 : 256u << 20;
    std::printf("subarena_pressure: cap %zu MiB per sub-arena\n", cap_bytes >> 20);

    testBurst();
    testAfter();

    if (failures) {
        std::fprintf(stderr, "subarena_pressure: %d failure(s)\n", failures);
        return 1;
    }
    std::printf("subarena_pressure: all tests passed\n");
    return 0;
}
