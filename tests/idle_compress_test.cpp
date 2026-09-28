// idle_compress_test.cpp - an idle working set must actually be compressed
// out of RSS, in every write-tracking mode, with default timings.
//
// The tests that run under SMASH_SOFTDIRTY=0 (page-protection write
// tracking) check the mechanics on a page or a few MiB with shortened
// timeouts. None checked that a large idle set goes through the whole
// pipeline — watched (PROT_READ), escalated, compressed, decommitted — in
// reasonable time. An application on a 6.6 arm64 kernel reported 115 GiB
// watched and nothing compressed under SMASH_SOFTDIRTY=0, and nothing could
// say whether that was early or stuck.
//
// 1 GiB of compressible rows in 4 MiB blocks, left alone. mincore() (not
// smash's counters) measures how much has left RSS; at least 90% must within
// the deadline. Then every block is read back and checked, so the round trip
// through compression is exact. Measured on the 8-CPU host with 8 GiB: all
// of it compressed within 40 s in both protection and soft-dirty modes.
//
// IDLE_COMPRESS_DEADLINE_S overrides the deadline (default 120 s).
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

namespace {
constexpr size_t kBlock = 4u << 20;
constexpr size_t kBlocks = 256;   // 1 GiB

void fill(char* p, size_t i) {
    for (size_t o = 0; o < kBlock; o += 64)
        std::snprintf(p + o, 64, "row %zu value %zu ", i, o % 977);
}
bool check(const char* p, size_t i) {
    char want[64];
    for (size_t o = 0; o < kBlock; o += 64) {
        std::snprintf(want, sizeof(want), "row %zu value %zu ", i, o % 977);
        if (std::memcmp(p + o, want, std::strlen(want) + 1) != 0) return false;
    }
    return true;
}
}  // namespace

int main() {
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) { std::printf("SKIP: not running under libsmash\n"); return 0; }
    const char* d = std::getenv("IDLE_COMPRESS_DEADLINE_S");
    const int deadline_s = d ? std::atoi(d) : 120;
    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));

    std::vector<char*> blocks;
    for (size_t i = 0; i < kBlocks; ++i) {
        auto* p = static_cast<char*>(malloc(kBlock));
        if (!p) { std::printf("FAIL: allocation %zu failed\n", i); return 1; }
        fill(p, i);
        blocks.push_back(p);
    }
    const size_t pages = kBlocks * kBlock / ps;
    std::vector<unsigned char> vec(kBlock / ps);
    auto outOfRss = [&] {
        size_t gone = 0;
        for (char* p : blocks)
            if (mincore(p, kBlock, vec.data()) == 0)
                for (unsigned char v : vec) gone += !(v & 1);
        return gone;
    };

    const auto t0 = std::chrono::steady_clock::now();
    size_t gone = 0;
    int secs = 0;
    for (; secs <= deadline_s; ++secs) {
        gone = outOfRss();
        if (secs % 10 == 0)
            std::printf("t=%3ds: %5.1f%% of 1 GiB out of RSS\n", secs, 100.0 * gone / pages);
        if (gone * 10 >= pages * 9) break;
        sleep(1);
    }
    const double took = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("%.1f%% out of RSS after %.0f s\n", 100.0 * gone / pages, took);

    size_t bad = 0;
    for (size_t i = 0; i < kBlocks; ++i) bad += !check(blocks[i], i);
    for (char* p : blocks) free(p);
    std::printf("read back: %zu of %zu blocks wrong\n", bad, kBlocks);

    int failures = 0;
    if (gone * 10 < pages * 9) {
        std::printf("FAIL: only %.1f%% of an idle 1 GiB left RSS within %d s: the compression "
                    "pipeline stalled\n", 100.0 * gone / pages, deadline_s);
        ++failures;
    }
    if (bad) {
        std::printf("FAIL: %zu blocks read back wrong after compression\n", bad);
        ++failures;
    }
    if (failures) return 1;
    std::printf("idle_compress_test: all checks passed\n");
    return 0;
}
