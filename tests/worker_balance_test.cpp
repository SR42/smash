// worker_balance_test.cpp - how a tick's work is shared among the default
// workers (src/compress/compressor_thread.h dispatch / executePhase).
//
// Each phase used to split the committed range into one static slice per
// worker: whichever worker's slice held the live pages carried the phase
// while the others idled at the barrier -- a heap whose live set sits at the
// bottom of a 512 GiB arena was compressed by one thread. The range is now
// handed out in slices (16 MiB, or a sixteenth of a worker's share) to
// whichever worker asks next.
//
//   a 1 GiB high-water mark is built from 64 x 16 MiB blocks; the 8 lowest
//   stay live (128 MiB, compressible), the rest are freed; 4 workers
//   (SMASH_TEST_WORKERS=4), no NUMA groups (SMASH_NUMA_COMPRESSOR=0, so the
//   default workers do the compressing). Once the live set is compressed,
//   smash_worker_balance_info must show the busiest worker did at most 75 %
//   of the compressions. With static slices worker 0 did 100 %.
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
        for (size_t off = 0; off < kBlock; off += 64) p[off] = static_cast<unsigned char>((off >> 12) * 3 + i);
        blocks.push_back(p);
    }
    std::sort(blocks.begin(), blocks.end());
    const size_t keep = 8;
    for (size_t i = keep; i < blocks.size(); ++i) std::free(blocks[i]);
    blocks.resize(keep);
    const size_t live_pages = keep * kBlock / ps;

    size_t compressed = 0;
    int waited = 0;
    for (; waited < 60000 && compressed < live_pages * 9 / 10; waited += 250) {
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
    std::printf("worker_balance: %zu of %zu live pages compressed after %d ms; %u workers; busiest did %llu of %llu compressions (%.0f %%)\n",
                compressed, live_pages, waited, workers, static_cast<unsigned long long>(max_one),
                static_cast<unsigned long long>(total), total ? 100.0 * max_one / total : 0.0);
    if (compressed < live_pages * 9 / 10) { std::fprintf(stderr, "FAIL: the live set did not compress in 60 s\n"); ++failures; }
    if (workers < 2) { std::fprintf(stderr, "FAIL: fewer than 2 workers active (SMASH_TEST_WORKERS not honoured)\n"); ++failures; }
    if (total == 0) { std::fprintf(stderr, "FAIL: the default workers compressed nothing (NUMA groups took the pages?)\n"); ++failures; }
    else if (max_one * 4 > total * 3) { std::fprintf(stderr, "FAIL: one worker did %.0f %% of the compressions: the phase was not shared\n", 100.0 * max_one / total); ++failures; }
    for (auto* p : blocks) std::free(p);
    if (failures) { std::fprintf(stderr, "worker_balance: %d failure(s)\n", failures); return 1; }
    std::printf("worker_balance: all tests passed\n");
    return 0;
}
