// idle_scan_live_test.cpp - the idle tracker reads the pagemap of live pages
// only (compressor_thread.h idleScan, anyLivePages).
//
// SMASH_WRITE_TRACK=idle reads one pagemap entry per page it scans, a pread
// per 1024 pages. Freed pages stay below the arena's high-water mark for good,
// and the scan used to read them as well: with 8 GiB freed, 2048 pagemap reads
// per tick for nothing. A batch whose chunks hold no live page is skipped now,
// without a system call.
//
// 64 x 16 MiB are allocated, touched and freed with one 8 MiB block kept
// (a 1 GiB high-water mark), SMASH_IDLE_SCAN_TICKS=1 so every slice is due
// every tick. Over the next ticks smash_idle_scan_info must show at most 8
// pagemap reads per tick (the kept block needs 2 or 3; the freed gigabyte
// adds 256), and at least 1024 resident pages examined per tick (the kept
// block's 2048: the skip must not reach live pages).
//
// Root only: without CAP_SYS_ADMIN and the page_idle bitmap the tracker is
// not `idle`, and the test prints SKIP and passes.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <unistd.h>
#include <vector>

int main() {
    using scan_fn = int (*)(uint64_t*, uint64_t*, uint64_t*);
    auto scan = reinterpret_cast<scan_fn>(dlsym(RTLD_DEFAULT, "smash_idle_scan_info"));
    if (!scan) { std::fprintf(stderr, "FAIL: smash_idle_scan_info missing\n"); return 1; }
    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));

    // A high-water mark far above the live set.
    std::vector<unsigned char*> blocks;
    for (int i = 0; i < 64; ++i) {
        auto* p = static_cast<unsigned char*>(std::malloc(16u << 20));
        if (!p) { std::fprintf(stderr, "FAIL: malloc\n"); return 1; }
        for (size_t off = 0; off < (16u << 20); off += ps) p[off] = 1;
        blocks.push_back(p);
    }
    constexpr size_t kKept = 8u << 20;
    auto* kept = static_cast<unsigned char*>(std::malloc(kKept));
    for (size_t off = 0; off < kKept; off += 64) kept[off] = static_cast<unsigned char>(off >> 12);
    for (auto* p : blocks) std::free(p);

    // The tracker is chosen once compression has started.
    using track_fn = const char* (*)();
    auto track = reinterpret_cast<track_fn>(dlsym(RTLD_DEFAULT, "smash_write_tracker"));
    if (!track) { std::fprintf(stderr, "FAIL: smash_write_tracker missing\n"); return 1; }
    const char* name = track();
    for (int w = 0; w < 10000 && std::strcmp(name, "unprobed") == 0; w += 100) { usleep(100000); name = track(); }
    uint64_t r = 0, pg = 0, t = 0;
    const int on = scan(&r, &pg, &t);
    if (on != 1) {
        std::printf("SKIP: the write tracker is %s, not idle (smash_idle_scan_info %d; needs root and "
                    "/sys/kernel/mm/page_idle/bitmap)\n", name, on);
        return 0;
    }
    auto waitTicks = [&](uint64_t n) {
        uint64_t r1 = 0, p1 = 0, t1 = 0, start = 0;
        scan(&r1, &p1, &start);
        for (int waited = 0; waited < 30000; waited += 50) {
            scan(&r1, &p1, &t1);
            if (t1 >= start + n) return true;
            usleep(50000);
        }
        return false;
    };
    // Two ticks for the frees to settle, then eight measured.
    if (!waitTicks(2)) { std::fprintf(stderr, "FAIL: fewer than 2 ticks in 30 s\n"); return 1; }
    uint64_t r0 = 0, p0 = 0, t0 = 0, r1 = 0, p1 = 0, t1 = 0;
    scan(&r0, &p0, &t0);
    if (!waitTicks(8)) { std::fprintf(stderr, "FAIL: fewer than 8 ticks in 30 s\n"); return 1; }
    scan(&r1, &p1, &t1);
    const double ticks = static_cast<double>(t1 - t0);
    const double reads = static_cast<double>(r1 - r0) / ticks, pages = static_cast<double>(p1 - p0) / ticks;
    std::printf("idle_scan_live: %llu ticks; %.1f pagemap reads and %.0f resident pages examined per tick "
                "(8 MiB live, 1 GiB freed)\n", static_cast<unsigned long long>(t1 - t0), reads, pages);

    int failures = 0;
    if (reads > 8) {
        std::fprintf(stderr, "FAIL: %.1f pagemap reads per tick: the scan reads freed address space\n", reads);
        ++failures;
    }
    if (pages < 1024) {
        std::fprintf(stderr, "FAIL: %.0f resident pages examined per tick, want the kept block's %zu\n",
                     pages, kKept / ps);
        ++failures;
    }
    size_t bad = 0;
    for (size_t off = 0; off < kKept; off += 64) bad += kept[off] != static_cast<unsigned char>(off >> 12);
    if (bad) { std::fprintf(stderr, "FAIL: %zu bytes of the kept block wrong\n", bad); ++failures; }
    std::free(kept);

    if (failures) { std::fprintf(stderr, "idle_scan_live: %d failure(s)\n", failures); return 1; }
    std::printf("idle_scan_live: all checks passed\n");
    return 0;
}
