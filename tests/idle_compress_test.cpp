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
// IDLE_COMPRESS_NUMA=1 allocates through malloc_numa, round-robin over the
// memory nodes: node-arena pages, compressed by the per-node groups.
// IDLE_COMPRESS_EXTRA_VMAS=N first splits a PROT_NONE reservation into about
// N mappings (every other page PROT_READ, so none merge) — the shape of a
// large application under page-protection tracking (503842 mappings seen).
#include <chrono>
#include <cstdint>
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
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) { std::fprintf(stderr, "FAIL: not running under libsmash\n"); return 1; }
    const char* d = std::getenv("IDLE_COMPRESS_DEADLINE_S");
    const int deadline_s = d ? std::atoi(d) : 120;
    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));

    using numa_fn = void* (*)(uint32_t, size_t);
    const char* nv = std::getenv("IDLE_COMPRESS_NUMA");
    auto numa = nv && nv[0] == '1' ? reinterpret_cast<numa_fn>(dlsym(RTLD_DEFAULT, "malloc_numa"))
                                   : nullptr;
    std::vector<int> nodes;
    if (numa) {
        for (int n = 0; n < 1024; ++n) {
            char path[80];
            std::snprintf(path, sizeof(path), "/sys/devices/system/node/node%d/meminfo", n);
            if (access(path, F_OK) == 0) nodes.push_back(n);
        }
        if (nodes.empty()) nodes.push_back(0);
        std::printf("allocating through malloc_numa over %zu node(s)\n", nodes.size());
    }
    if (const char* ev = std::getenv("IDLE_COMPRESS_EXTRA_VMAS")) {
        const size_t n = std::strtoull(ev, nullptr, 10);
        long limit = 0;
        if (FILE* mc = std::fopen("/proc/sys/vm/max_map_count", "r")) {
            if (std::fscanf(mc, "%ld", &limit) != 1) limit = 0;
            std::fclose(mc);
        }
        if (limit < static_cast<long>(n) + 100000) {
            std::printf("SKIP: vm.max_map_count=%ld is too low for %zu extra mappings\n", limit, n);
            return 77;
        }
        auto* res = static_cast<char*>(mmap(nullptr, n * ps, PROT_NONE,
                                            MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0));
        if (res == MAP_FAILED) { std::printf("SKIP: reservation for extra mappings failed\n"); return 77; }
        size_t made = 0;
        for (size_t k = 0; k < n; k += 2)
            made += mprotect(res + k * ps, ps, PROT_READ) == 0;
        FILE* m = std::fopen("/proc/self/maps", "r");
        size_t lines = 0;
        char buf[512];
        const auto m0 = std::chrono::steady_clock::now();
        while (m && std::fgets(buf, sizeof(buf), m)) ++lines;
        if (m) std::fclose(m);
        std::printf("extra mappings: %zu pages armed, /proc/self/maps has %zu lines "
                    "(one read: %.0f ms)\n", made, lines,
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - m0).count());
    }
    std::vector<char*> blocks;
    for (size_t i = 0; i < kBlocks; ++i) {
        auto* p = static_cast<char*>(
            numa ? numa(static_cast<uint32_t>(nodes[i % nodes.size()]), kBlock) : malloc(kBlock));
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
