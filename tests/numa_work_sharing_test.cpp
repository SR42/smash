// numa_work_sharing_test.cpp - every compressor thread works every pool of a
// phase (src/compress/compressor_thread.h executePhase, executeNodePhase).
//
// On a NUMA host the compressor has two kinds of thread: the default workers
// and, per memory node, a group pinned to that node. Phases 1-4 used to be
// split by ownership -- node arena chunks to the group, everything else to
// the default workers -- and phase 5 (the pages phase 2 routed to each node's
// queue) ran on the group alone. An ordinary heap carries no binding, so the
// default workers alone walked phases 1-4 while every group idled at the
// barrier, and in phase 5 the default workers idled while the groups
// compressed the whole cold set; whichever side owned the heap carried the
// tick. Now each pool (the shared range in slices, a group's chunks in units,
// a group's queue in batches) is claimed by whichever thread asks next, and a
// page's blob still goes to its node's store whoever compresses it.
//
//   The test re-executes itself pinned to node 0's CPUs, so the default
//   workers (2: SMASH_TEST_WORKERS=2) run there, then builds a 1 GiB
//   high-water mark from 64 x 16 MiB blocks first touched on node 1's CPUs
//   and frees all but the 16 lowest (256 MiB live, 65536 pages, on node 1).
//   Each group has one thread (SMASH_NUMA_WORKERS=1). Once 90 % of the live
//   set is compressed:
//     - smash_numa_compressor_info(1) must show that at least a quarter of
//       node 1's compressions were done by threads outside its group
//       (helped_compressions), and smash_numa_route_info that the pages went
//       through the queue (the test proved nothing otherwise);
//     - smash_work_share_info must show group threads took slices of the
//       shared range (64 slices of 16 MiB against 2 default workers);
//     - at most 10 % of the blobs may be off node 1: the default workers
//       compress into node 1's store from node 0.
//   Before the fix: 0 helped, 0 shared slices by group threads.
//
// Needs two or more memory nodes with CPUs (skips otherwise). Registered
// large-only, cold timeout 1 s.
#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <sched.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <vector>

namespace {
int failures = 0;
void fail(const char* what) { std::fprintf(stderr, "FAIL: %s\n", what); ++failures; }

// Same layout as the struct in src/smash_heap.cpp; `size` guards it.
struct SmashNumaCompressorInfo {
    uint32_t size;
    uint32_t workers;
    int32_t  tids[8];
    uint64_t pages_compressed;
    uint64_t off_node_compressions;
    void*    memory[8][4];
    uint64_t helped_compressions;
};

bool nodeCpus(int node, cpu_set_t* set) {
    char path[128];
    std::snprintf(path, sizeof path, "/sys/devices/system/node/node%d/cpulist", node);
    FILE* f = std::fopen(path, "r");
    if (!f) return false;
    char buf[4096] = {};
    const bool ok = std::fgets(buf, sizeof buf, f) != nullptr;
    std::fclose(f);
    if (!ok) return false;
    CPU_ZERO(set);
    for (char* tok = std::strtok(buf, ",\n"); tok; tok = std::strtok(nullptr, ",\n")) {
        int a = 0, b = 0;
        if (std::sscanf(tok, "%d-%d", &a, &b) == 2) for (int c = a; c <= b; ++c) CPU_SET(c, set);
        else if (std::sscanf(tok, "%d", &a) == 1) CPU_SET(a, set);
    }
    return CPU_COUNT(set) > 0;
}

// By address, not index: the blocks are sorted after they are written.
unsigned char pattern(const unsigned char* block, size_t off) {
    return static_cast<unsigned char>((off >> 12) * 7 + (off >> 6) + (reinterpret_cast<uintptr_t>(block) >> 24) * 3);
}
}  // namespace

int main(int, char** argv) {
    // First run: pin to node 0 and start over, so the preload's threads
    // inherit the mask.
    if (!std::getenv("NUMA_WORK_SHARING_PINNED")) {
        cpu_set_t node0, allowed0;
        if (!nodeCpus(0, &node0) || sched_getaffinity(0, sizeof allowed0, &allowed0) != 0) {
            std::printf("numa_work_sharing: SKIP (no node 0 with CPUs)\n");
            return 0;
        }
        CPU_AND(&node0, &node0, &allowed0);
        if (CPU_COUNT(&node0) == 0) { std::printf("numa_work_sharing: SKIP (node 0's CPUs are not in the affinity mask)\n"); return 0; }
        if (sched_setaffinity(0, sizeof node0, &node0) != 0) { perror("sched_setaffinity"); return 1; }
        setenv("NUMA_WORK_SHARING_PINNED", "1", 1);
        execv("/proc/self/exe", argv);
        perror("execv");
        return 1;
    }
    using page_fn = int (*)(const void*, int*, int*, int*);
    using blob_fn = void* (*)(const void*);
    using route_fn = int (*)(uint64_t*, uint64_t*);
    using info_fn = int (*)(uint32_t, SmashNumaCompressorInfo*);
    using share_fn = int (*)(uint64_t*, uint64_t*, uint64_t*);
    auto page_info = reinterpret_cast<page_fn>(dlsym(RTLD_DEFAULT, "smash_page_info"));
    auto blob_for = reinterpret_cast<blob_fn>(dlsym(RTLD_DEFAULT, "smash_compressed_blob_for"));
    auto route = reinterpret_cast<route_fn>(dlsym(RTLD_DEFAULT, "smash_numa_route_info"));
    auto info = reinterpret_cast<info_fn>(dlsym(RTLD_DEFAULT, "smash_numa_compressor_info"));
    auto share = reinterpret_cast<share_fn>(dlsym(RTLD_DEFAULT, "smash_work_share_info"));
    if (!page_info || !blob_for || !route || !info || !share) { std::fprintf(stderr, "FAIL: diagnostic exports missing\n"); return 1; }

    cpu_set_t node1, allowed;
    if (!nodeCpus(1, &node1)) { std::printf("numa_work_sharing: SKIP (no node 1 with CPUs)\n"); return 0; }
    if (sched_getaffinity(0, sizeof allowed, &allowed) != 0) return 1;   // node 0: where the workers run
    if (sched_setaffinity(0, sizeof node1, &node1) != 0) {
        std::printf("numa_work_sharing: SKIP (cannot run on node 1's CPUs: %s)\n", std::strerror(errno));
        return 0;
    }
    {
        SmashNumaCompressorInfo in{};
        in.size = sizeof in;
        const int rc = info(1, &in);
        if (rc != 0) {
            if (rc == -2) { fail("smash_numa_compressor_info: layout mismatch (the test's struct is stale)"); return 1; }
            std::printf("numa_work_sharing: SKIP (node 1 has no compressor group, rc=%d)\n", rc);
            return 0;
        }
    }

    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    const size_t kBlock = 16u << 20;
    const size_t kBlocks = 64, kKeep = 16;

    // On node 1 for the first touch: the pages land there.
    std::vector<unsigned char*> blocks;
    for (size_t i = 0; i < kBlocks; ++i) {
        auto* p = static_cast<unsigned char*>(std::malloc(kBlock));
        if (!p) { fail("malloc"); return 1; }
        for (size_t off = 0; off < kBlock; off += 64) p[off] = pattern(p, off);
        blocks.push_back(p);
    }
    std::sort(blocks.begin(), blocks.end());
    for (size_t i = kKeep; i < blocks.size(); ++i) std::free(blocks[i]);
    blocks.resize(kKeep);
    sched_setaffinity(0, sizeof allowed, &allowed);
    const size_t pages = kKeep * kBlock / ps;

    // Where the pages are (the blobs must follow them).
    std::vector<void*> ptrs(pages);
    std::vector<int> status(pages, INT_MIN);
    for (size_t i = 0; i < pages; ++i) ptrs[i] = blocks[i / (kBlock / ps)] + (i % (kBlock / ps)) * ps;
    if (syscall(SYS_move_pages, 0, pages, ptrs.data(), nullptr, status.data(), 0) != 0) { perror("move_pages"); return 1; }
    size_t on1 = 0;
    for (size_t i = 0; i < pages; ++i) on1 += status[i] == 1;
    if (on1 < pages * 9 / 10) {
        std::printf("numa_work_sharing: SKIP (only %zu of %zu pages landed on node 1)\n", on1, pages);
        return 0;
    }

    uint64_t q0 = 0, f0 = 0;
    route(&q0, &f0);
    SmashNumaCompressorInfo before{};
    before.size = sizeof before;
    info(1, &before);
    uint64_t h0 = 0, s0 = 0, u0 = 0;
    share(&h0, &s0, &u0);

    size_t compressed = 0;
    int waited = 0;
    for (; waited < 120000 && compressed < pages * 9 / 10; waited += 500) {
        usleep(500000);
        compressed = 0;
        for (size_t i = 0; i < pages; ++i) {
            int st = 0, c = 0, r = 0;
            if (page_info(ptrs[i], &st, &c, &r) == 0 && st == 2) ++compressed;
        }
    }
    uint64_t q1 = 0, f1 = 0;
    route(&q1, &f1);
    SmashNumaCompressorInfo after{};
    after.size = sizeof after;
    info(1, &after);
    uint64_t h1 = 0, s1 = 0, u1 = 0;
    share(&h1, &s1, &u1);
    const uint64_t own = after.pages_compressed - before.pages_compressed;
    const uint64_t helped = after.helped_compressions - before.helped_compressions;

    // Blob placement.
    std::vector<void*> blob_ptrs;
    for (size_t i = 0; i < pages; ++i) {
        int st = 0, c = 0, r = 0;
        if (page_info(ptrs[i], &st, &c, &r) != 0 || st != 2) continue;
        if (void* b = blob_for(ptrs[i])) blob_ptrs.push_back(b);
    }
    size_t off_node = 0;
    int first = 0;
    if (!blob_ptrs.empty()) {
        std::vector<int> bst(blob_ptrs.size(), INT_MIN);
        if (syscall(SYS_move_pages, 0, blob_ptrs.size(), blob_ptrs.data(), nullptr, bst.data(), 0) != 0) { perror("move_pages blobs"); return 1; }
        for (size_t i = 0; i < blob_ptrs.size(); ++i)
            if (bst[i] != 1) { if (off_node++ == 0) first = bst[i]; }
    }
    std::printf("numa_work_sharing: %zu of %zu pages on node 1; %zu compressed after %d ms; queued %llu (queue full %llu);"
                " node 1: %llu by its group, %llu by other threads (%.0f %%); shared slices by group threads %llu,"
                " node units by others %llu; %zu of %zu blobs off node 1 (first status %d)\n",
                on1, pages, compressed, waited, static_cast<unsigned long long>(q1 - q0),
                static_cast<unsigned long long>(f1 - f0), static_cast<unsigned long long>(own),
                static_cast<unsigned long long>(helped),
                own + helped ? 100.0 * static_cast<double>(helped) / static_cast<double>(own + helped) : 0.0,
                static_cast<unsigned long long>(s1 - s0), static_cast<unsigned long long>(u1 - u0),
                off_node, blob_ptrs.size(), first);
    if (compressed < pages * 9 / 10) fail("the live set did not compress in 120 s");
    if (q1 - q0 < pages / 4) fail("fewer than a quarter of the pages went through node 1's queue: the test did not exercise phase 5");
    if (helped * 4 < (own + helped)) fail("threads outside node 1's group did less than a quarter of its compressions: phase 5 was not shared");
    if (s1 - s0 == 0) fail("no group thread took a slice of the shared range: phases 1-4 were not shared");
    if (blob_ptrs.empty() || off_node * 10 > blob_ptrs.size()) fail("blobs compressed by other threads landed off their node");

    size_t bad = 0;
    for (size_t i = 0; i < kKeep; ++i)
        for (size_t off = 0; off < kBlock; off += 64) bad += blocks[i][off] != pattern(blocks[i], off);
    if (bad) fail("data did not read back");
    for (auto* p : blocks) std::free(p);
    if (failures) { std::fprintf(stderr, "numa_work_sharing: %d failure(s)\n", failures); return 1; }
    std::printf("numa_work_sharing: all tests passed\n");
    return 0;
}
