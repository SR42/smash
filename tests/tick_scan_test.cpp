// tick_scan_test.cpp - what a tick reads that is not a live page
// (src/vm/page_state.h live bitmap; compressor_thread.h sweepDeferredMadvise).
//
// Each tick began by reading the state byte of every page from 0 to the
// arena's high-water mark to rebuild the live-chunk bitmap, and ended by
// reading every page's deferred-madvise bit over the same range -- both on
// the coordinator alone, both proportional to how much of the arena has ever
// been walked, not to what is live (134M bytes each on a 512 GiB arena), and
// the second for nothing in the default deferred-reclaim mode, where no page
// ever has the bit set. The bitmap is now kept by every state store, and the
// sweep walks nothing while no page is pending.
//
//   1. a 1 GiB high-water mark is left behind by allocating and freeing
//      64 x 16 MiB with one 8 MiB block kept; after three ticks
//      smash_tick_cost_info must show the sweep scanned no pages (nothing
//      is pending) -- before the fix it scanned 262144 per tick
//   2. the stats line says how much of what was walked is free: bump= against
//      arena_free= (SMASH_STATS_SIGNAL=1), the whole 1 GiB of it here, sub-arenas
//      or not
//   3. the kept block still compresses and reads back (the live bitmap the
//      phases now rely on is right)
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

int main() {
    using cost_fn = int (*)(uint64_t*, uint64_t*, uint64_t*);
    using info_fn = int (*)(const void*, int*, int*, int*);
    auto cost = reinterpret_cast<cost_fn>(dlsym(RTLD_DEFAULT, "smash_tick_cost_info"));
    auto page_info = reinterpret_cast<info_fn>(dlsym(RTLD_DEFAULT, "smash_page_info"));
    if (!cost || !page_info) { std::fprintf(stderr, "FAIL: diagnostic exports missing\n"); return 1; }
    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    int failures = 0;

    // A high-water mark far above the live set.
    std::vector<unsigned char*> blocks;
    for (int i = 0; i < 64; ++i) {
        auto* p = static_cast<unsigned char*>(std::malloc(16u << 20));
        if (!p) { std::fprintf(stderr, "FAIL: malloc\n"); return 1; }
        for (size_t off = 0; off < (16u << 20); off += ps) p[off] = 1;
        blocks.push_back(p);
    }
    auto* kept = static_cast<unsigned char*>(std::malloc(8u << 20));
    for (size_t off = 0; off < (8u << 20); off += 64) kept[off] = static_cast<unsigned char>(off >> 12);
    for (auto* p : blocks) std::free(p);

    uint64_t sweep0 = 0, mem0 = 0, t0 = 0;
    cost(&sweep0, &mem0, &t0);
    // Three more ticks.
    for (int waited = 0; waited < 20000; waited += 100) {
        uint64_t s = 0, m = 0, t = 0;
        cost(&s, &m, &t);
        if (t >= t0 + 3) break;
        usleep(100000);
    }
    uint64_t sweep1 = 0, mem1 = 0, t1 = 0;
    cost(&sweep1, &mem1, &t1);

    // bump= and arena_free= on the stats line, read from a file stderr points at.
    {
        char path[] = "/tmp/smash_tickscan_XXXXXX";
        const int fd = mkstemp(path);
        if (fd < 0) { std::fprintf(stderr, "FAIL: mkstemp\n"); return 1; }
        unlink(path);
        const int saved = dup(2);
        dup2(fd, 2);
        raise(SIGUSR2);
        std::string log;
        for (int waited = 0; waited < 5000 && log.find("arena_free=") == std::string::npos; waited += 100) {
            usleep(100000);
            char buf[8192];
            const ssize_t n = pread(fd, buf, sizeof(buf) - 1, 0);
            if (n > 0) log.assign(buf, static_cast<size_t>(n));
        }
        dup2(saved, 2);
        close(saved);
        close(fd);
        const size_t b = log.find(" bump="), f = log.find(" arena_free=");
        if (b == std::string::npos || f == std::string::npos) {
            std::fprintf(stderr, "FAIL: no bump= and arena_free= on the stats line: %s\n", log.c_str());
            ++failures;
        } else {
            const unsigned long long bump = std::strtoull(log.c_str() + b + 6, nullptr, 10),
                                     fr = std::strtoull(log.c_str() + f + 12, nullptr, 10);
            std::printf("stats line: bump=%llu pages, arena_free=%llu pages\n", bump, fr);
            const unsigned long long want = (900ull << 20) / ps;
            if (bump < want || fr < want) {
                std::fprintf(stderr, "FAIL: 1 GiB walked and freed, the stats line says bump=%llu arena_free=%llu pages\n",
                             bump, fr);
                ++failures;
            }
        }
    }
    std::printf("tick_scan: %llu ticks; sweep scanned %llu page bits (1 GiB high-water mark = %zu pages)\n",
                static_cast<unsigned long long>(t1 - t0), static_cast<unsigned long long>(sweep1 - sweep0),
                (1u << 30) / ps);
    if (t1 - t0 < 3) { std::fprintf(stderr, "FAIL: fewer than 3 ticks in 20 s\n"); ++failures; }
    if (sweep1 - sweep0 != 0) {
        std::fprintf(stderr, "FAIL: the sweep scanned %llu page bits with nothing pending\n",
                     static_cast<unsigned long long>(sweep1 - sweep0));
        ++failures;
    }

    // The kept block compresses (the live bitmap names its pages) and reads back.
    int compressed = 0;
    for (int waited = 0; waited < 20000 && compressed == 0; waited += 200) {
        usleep(200000);
        for (size_t off = 0; off < (8u << 20); off += ps) {
            int st = 0, c = 0, r = 0;
            if (page_info(kept + off, &st, &c, &r) == 0 && st == 2) ++compressed;
        }
    }
    if (compressed == 0) { std::fprintf(stderr, "FAIL: the kept block never compressed\n"); ++failures; }
    size_t bad = 0;
    for (size_t off = 0; off < (8u << 20); off += 64) bad += kept[off] != static_cast<unsigned char>(off >> 12);
    if (bad) { std::fprintf(stderr, "FAIL: %zu bytes wrong after decompression\n", bad); ++failures; }
    std::printf("tick_scan: %d of %zu kept pages were compressed; data intact\n", compressed, (8u << 20) / ps);
    std::free(kept);

    if (failures) { std::fprintf(stderr, "tick_scan: %d failure(s)\n", failures); return 1; }
    std::printf("tick_scan: all tests passed\n");
    return 0;
}
