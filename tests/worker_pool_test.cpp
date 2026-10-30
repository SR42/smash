// worker_pool_test.cpp - the compressor can run more than 8 default workers
// (include/smash/config.h kMaxCompressorWorkers, compressor_thread.h
// adaptWorkerCount / SMASH_MAX_WORKERS).
//
// The worker pool was a compile-time 8: on a 128-core host the Little's-law
// estimate asked for more every tick and got 8, next to 2 threads per node,
// and ticks of 724 s and 1448 s were recorded. The pool is 32 now, allocated
// lazily (a worker's state is ~190 KiB), and the runtime cap is
// SMASH_MAX_WORKERS, by default clamp(nproc / 4, 8, 32), so hosts of up to
// 32 CPUs still run at most 8. SMASH_TEST_WORKERS raises the cap with it.
//
//   256 MiB live (65536 pages: 16 slices of 16 MiB) at the bottom of a 1 GiB
//   high-water mark, 12 workers (SMASH_TEST_WORKERS=12), no NUMA groups.
//   Once the live set is compressed, smash_worker_balance_info must report
//   12 active workers and no worker did more than half the compressions.
//   With the 8-worker pool: 8 workers (SMASH_TEST_WORKERS clamped).
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <dlfcn.h>
#include <unistd.h>
#include <vector>

int main() {
    using info_fn = int (*)(const void*, int*, int*, int*);
    using bal_fn = int (*)(uint64_t*, uint64_t*, uint32_t*);
    auto page_info = reinterpret_cast<info_fn>(dlsym(RTLD_DEFAULT, "smash_page_info"));
    auto balance = reinterpret_cast<bal_fn>(dlsym(RTLD_DEFAULT, "smash_worker_balance_info"));
    if (!page_info || !balance) { std::fprintf(stderr, "FAIL: diagnostic exports missing\n"); return 1; }
    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    const size_t kBlock = 16u << 20;
    int failures = 0;

    std::vector<unsigned char*> blocks;
    for (int i = 0; i < 64; ++i) {
        auto* p = static_cast<unsigned char*>(std::malloc(kBlock));
        if (!p) { std::fprintf(stderr, "FAIL: malloc\n"); return 1; }
        for (size_t off = 0; off < kBlock; off += 64) p[off] = static_cast<unsigned char>((off >> 12) * 5 + i);
        blocks.push_back(p);
    }
    std::sort(blocks.begin(), blocks.end());
    const size_t keep = 16;
    for (size_t i = keep; i < blocks.size(); ++i) std::free(blocks[i]);
    blocks.resize(keep);
    const size_t live_pages = keep * kBlock / ps;

    size_t compressed = 0;
    int waited = 0;
    for (; waited < 120000 && compressed < live_pages * 9 / 10; waited += 250) {
        usleep(250000);
        compressed = 0;
        for (auto* p : blocks)
            for (size_t off = 0; off < kBlock; off += ps) {
                int st = 0, c = 0, r = 0;
                if (page_info(p + off, &st, &c, &r) == 0 && st == 2) ++compressed;
            }
    }
    uint64_t total = 0, max_one = 0;
    uint32_t workers = 0;
    balance(&total, &max_one, &workers);
    std::printf("worker_pool: %zu of %zu live pages compressed after %d ms; %u workers; busiest did %llu of %llu compressions (%.0f %%)\n",
                compressed, live_pages, waited, workers, static_cast<unsigned long long>(max_one),
                static_cast<unsigned long long>(total), total ? 100.0 * static_cast<double>(max_one) / static_cast<double>(total) : 0.0);
    if (compressed < live_pages * 9 / 10) { std::fprintf(stderr, "FAIL: the live set did not compress in 120 s\n"); ++failures; }
    if (workers < 12) { std::fprintf(stderr, "FAIL: %u workers active, SMASH_TEST_WORKERS=12 asked for 12: the pool is capped below it\n", workers); ++failures; }
    if (total == 0) { std::fprintf(stderr, "FAIL: the default workers compressed nothing\n"); ++failures; }
    else if (max_one * 2 > total) { std::fprintf(stderr, "FAIL: one worker did %.0f %% of the compressions\n", 100.0 * static_cast<double>(max_one) / static_cast<double>(total)); ++failures; }
    for (auto* p : blocks) std::free(p);
    if (failures) { std::fprintf(stderr, "worker_pool: %d failure(s)\n", failures); return 1; }
    std::printf("worker_pool: all tests passed\n");
    return 0;
}
