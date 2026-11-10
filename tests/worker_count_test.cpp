// worker_count_test.cpp - the compressor runs with as many workers as its
// work needs, measured in the running library (issue #8).
//
// The policy is test_worker_scaling's. This checks that the library measures
// the work and applies the count, on the two loads the old count got wrong.
// It was ceil(eligible pages / pages compressed per worker), which is
// workers x eligible / compressed.
//
//   light: every 250 ms a 2 MiB block of compressible bytes and a 2 MiB block
//     of random bytes are allocated and left alone, for 10 s. That is about
//     20 ms of compressor work a second. Half the eligible pages are not
//     compressed (the random ones), so the old count doubled every tick up to
//     the cap: 8 workers woken for every phase of every tick. The count must
//     stay at 2 or below from the fourth second on. (On a host with several
//     memory nodes the old count went to the cap without the random blocks:
//     pages handed to a node's group were eligible and not compressed by the
//     default workers.)
//
//   burst: 512 MiB of compressible bytes go cold at once, with ticks of
//     200 ms: several ticks' worth of CPU time in one tick. The old count
//     stayed at 1, every eligible page being compressed, while one thread
//     worked through the lot. The count must reach 2 or more while the block
//     is being compressed, and be back at 1 within 30 ticks of the end.
//
// SMASH_CPU_PRESSURE_CAP=0 in both: the cap would hide the count on a loaded
// 2-CPU host. SMASH_NUMA_COMPRESSOR=0 in burst: with node groups the default
// workers only route, which is little work, and the old count went to the
// cap there for the other reason.
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <dlfcn.h>
#include <unistd.h>
#include <vector>

namespace {
using info_fn = int (*)(const void*, int*, int*, int*);
using bal_fn = int (*)(uint64_t*, uint64_t*, uint32_t*);
info_fn page_info = nullptr;
bal_fn balance = nullptr;

uint64_t rng = 88172645463325252ull;
inline uint64_t next() { rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17; return rng; }

// Four bits of entropy a byte: the codec has real work and saves about half.
void fillCompressible(unsigned char* p, size_t n) {
    auto* w = reinterpret_cast<uint64_t*>(p);
    for (size_t i = 0; i < n / 8; ++i) w[i] = next() & 0x0f0f0f0f0f0f0f0full;
}
void fillRandom(unsigned char* p, size_t n) {
    auto* w = reinterpret_cast<uint64_t*>(p);
    for (size_t i = 0; i < n / 8; ++i) w[i] = next();
}
double now() {
    timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return static_cast<double>(t.tv_sec) + static_cast<double>(t.tv_nsec) * 1e-9;
}
uint32_t workers() {
    uint64_t total = 0, max_one = 0;
    uint32_t w = 0;
    balance(&total, &max_one, &w);
    return w;
}
// Pages of [p, p + n) that are compressed (state 2) or compressed with the
// page still mapped (5).
size_t compressedPages(const unsigned char* p, size_t n) {
    size_t c = 0;
    for (size_t off = 0; off < n; off += 4096) {
        int st = 0, cold = 0, rc = 0;
        if (page_info(p + off, &st, &cold, &rc) == 0 && (st == 2 || st == 5)) ++c;
    }
    return c;
}

int light() {
    constexpr size_t kBlock = size_t{2} << 20;
    std::vector<unsigned char*> good, bad;
    uint32_t max_workers = 0;
    int above = 0, samples = 0;
    const double t0 = now();
    double next_alloc = 0;
    while (now() - t0 < 13.0) {
        const double t = now() - t0;
        if (t < 10.0 && t >= next_alloc) {
            next_alloc += 0.25;
            auto* a = static_cast<unsigned char*>(std::malloc(kBlock));
            auto* b = static_cast<unsigned char*>(std::malloc(kBlock));
            if (!a || !b) { std::fprintf(stderr, "FAIL: malloc\n"); return 1; }
            fillCompressible(a, kBlock);
            fillRandom(b, kBlock);
            good.push_back(a);
            bad.push_back(b);
        }
        if (t >= 4.0) {
            const uint32_t w = workers();
            max_workers = std::max(max_workers, w);
            ++samples;
            if (w > 2) ++above;
        }
        usleep(20000);
    }
    size_t good_c = 0, bad_c = 0;
    for (auto* p : good) good_c += compressedPages(p, kBlock);
    for (auto* p : bad) bad_c += compressedPages(p, kBlock);
    const size_t pages = good.size() * kBlock / 4096;
    std::printf("light: %zu of %zu compressible pages and %zu of %zu random pages compressed; "
                "most workers from 4 s on: %u (%d of %d samples above 2)\n",
                good_c, pages, bad_c, pages, max_workers, above, samples);
    if (good_c < pages / 2) {
        std::fprintf(stderr, "FAIL: %zu of %zu compressible pages were compressed in 13 s: the compressor did "
                     "not do the work this test measures\n", good_c, pages);
        return 1;
    }
    if (bad_c > pages / 2) {
        std::printf("SKIP: %zu of %zu random pages were compressed, so there were no eligible pages left "
                    "uncompressed\n", bad_c, pages);
        return 77;
    }
    if (max_workers > 2) {
        std::fprintf(stderr, "FAIL: the compressor ran with %u workers for 8 MiB of new cold data a second "
                     "(%d of %d samples above 2)\n", max_workers, above, samples);
        return 1;
    }
    return 0;
}

int burst() {
    constexpr size_t kBlock = size_t{512} << 20;
    auto* p = static_cast<unsigned char*>(std::malloc(kBlock));
    if (!p) { std::fprintf(stderr, "FAIL: malloc\n"); return 1; }
    fillCompressible(p, kBlock);
    const size_t pages = kBlock / 4096;
    uint32_t max_workers = 0;
    const double t0 = now();
    double done_at = -1;
    size_t c = 0;
    while (now() - t0 < 120.0) {
        max_workers = std::max(max_workers, workers());
        usleep(10000);
        // The states cost a call per page: look twice a second.
        static double last_look = 0;
        if (now() - last_look >= 0.5) {
            last_look = now();
            c = compressedPages(p, kBlock);
            if (c >= pages * 9 / 10) { done_at = now() - t0; break; }
        }
    }
    if (done_at < 0) {
        std::fprintf(stderr, "FAIL: %zu of %zu pages compressed in 120 s\n", c, pages);
        return 1;
    }
    // 30 ticks of 200 ms for the count to come down.
    uint32_t last = 0;
    const double t1 = now();
    double back_at = -1;
    while (now() - t1 < 6.0) {
        last = workers();
        if (last <= 1) { back_at = now() - t1; break; }
        usleep(20000);
    }
    std::printf("burst: %zu of %zu pages compressed after %.1f s; most workers meanwhile: %u; back at one "
                "worker %.1f s later\n", c, pages, done_at, max_workers, back_at);
    if (max_workers < 2) {
        std::fprintf(stderr, "FAIL: 512 MiB went cold at once and the compressor never ran with more than "
                     "%u worker(s)\n", max_workers);
        return 1;
    }
    if (back_at < 0) {
        std::fprintf(stderr, "FAIL: %u workers 6 s (30 ticks) after the work ended\n", last);
        return 1;
    }
    return 0;
}
}  // namespace

int main(int argc, char** argv) {
    setvbuf(stdout, nullptr, _IOLBF, 0);
    page_info = reinterpret_cast<info_fn>(dlsym(RTLD_DEFAULT, "smash_page_info"));
    balance = reinterpret_cast<bal_fn>(dlsym(RTLD_DEFAULT, "smash_worker_balance_info"));
    if (!page_info || !balance) {
        std::fprintf(stderr, "FAIL: not running under libsmash (diagnostic exports missing)\n");
        return 1;
    }
    const char* mode = argc > 1 ? argv[1] : "";
    int rc;
    if (!std::strcmp(mode, "light")) rc = light();
    else if (!std::strcmp(mode, "burst")) rc = burst();
    else { std::fprintf(stderr, "usage: %s light|burst\n", argv[0]); return 2; }
    if (rc == 0) std::printf("OK\n");
    return rc;
}
