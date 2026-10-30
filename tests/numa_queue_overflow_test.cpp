// numa_queue_overflow_test.cpp - where an ordinary page's blob goes when its
// node's queue is full (src/compress/compressor_thread.h routeByNode,
// enqueueForNode, flushNodeQueries).
//
// Phase 2 hands every eligible ordinary-heap page to its node's group through
// a per-node queue of 64K entries per tick. A page that found the queue full
// was compressed by the routing worker into the SHARED store: its blob
// landed on whatever node that worker happened to run on, for as long as it
// stayed compressed, and nothing counted it. A backlog of more than 256 MiB
// per node per tick -- the first cold wave of any large process -- put most
// of its blobs off-node. A page that finds the queue full is now still
// compressed by the routing worker, but into its own node's store.
//
//   The test re-executes itself pinned to node 0's CPUs, so the default
//   compressor workers (which inherit the process's mask at start) run
//   there, and the shared store's memory lands there. Then 512 MiB of
//   ordinary malloc memory is first touched from node 1's CPUs (131072
//   pages: twice the queue). Once 90 % is compressed, at most 10 % of the
//   blobs may be off node 1, and smash_numa_route_info must show the queue
//   did overflow (otherwise the test proved nothing). Without the pinning
//   the workers happened to run on node 1 too and the shared store landed
//   there: the mutant passed.
//
// Needs two or more memory nodes with CPUs (skips otherwise), and a
// compressor group on node 1. Registered large-only, cold timeout 1 s.
#include <cerrno>
#include <climits>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <vector>

namespace {
int failures = 0;
void fail(const char* what) { std::fprintf(stderr, "FAIL: %s\n", what); ++failures; }

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
}  // namespace

int main(int, char** argv) {
    // First run: pin to node 0 and start over, so the preload's threads
    // inherit the mask. The affinity can be widened again later (it is a
    // mask, not a cpuset).
    if (!std::getenv("NUMA_QUEUE_OVERFLOW_PINNED")) {
        cpu_set_t node0, allowed0;
        if (!nodeCpus(0, &node0) || sched_getaffinity(0, sizeof allowed0, &allowed0) != 0) {
            std::printf("numa_queue_overflow: SKIP (no node 0 with CPUs)\n");
            return 0;
        }
        CPU_AND(&node0, &node0, &allowed0);
        if (CPU_COUNT(&node0) == 0) { std::printf("numa_queue_overflow: SKIP (node 0's CPUs are not in the affinity mask)\n"); return 0; }
        if (sched_setaffinity(0, sizeof node0, &node0) != 0) { perror("sched_setaffinity"); return 1; }
        setenv("NUMA_QUEUE_OVERFLOW_PINNED", "1", 1);
        execv("/proc/self/exe", argv);
        perror("execv");
        return 1;
    }
    using info_fn = int (*)(const void*, int*, int*, int*);
    using blob_fn = void* (*)(const void*);
    using route_fn = int (*)(uint64_t*, uint64_t*);
    auto page_info = reinterpret_cast<info_fn>(dlsym(RTLD_DEFAULT, "smash_page_info"));
    auto blob_for = reinterpret_cast<blob_fn>(dlsym(RTLD_DEFAULT, "smash_compressed_blob_for"));
    auto route = reinterpret_cast<route_fn>(dlsym(RTLD_DEFAULT, "smash_numa_route_info"));
    if (!page_info || !blob_for || !route) { std::fprintf(stderr, "FAIL: diagnostic exports missing\n"); return 1; }

    cpu_set_t node1, allowed;
    if (!nodeCpus(1, &node1)) { std::printf("numa_queue_overflow: SKIP (no node 1 with CPUs)\n"); return 0; }
    if (sched_getaffinity(0, sizeof allowed, &allowed) != 0) return 1;   // node 0: where the workers run
    if (sched_setaffinity(0, sizeof node1, &node1) != 0) {
        std::printf("numa_queue_overflow: SKIP (cannot run on node 1's CPUs: %s)\n", std::strerror(errno));
        return 0;
    }

    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    const size_t bytes = 512u << 20;
    const size_t pages = bytes / ps;

    // On node 1 for the first touch: the pages land there.
    auto* p = static_cast<unsigned char*>(std::malloc(bytes));
    if (!p) return 1;
    for (size_t off = 0; off < bytes; off += 64) p[off] = static_cast<unsigned char>((off >> 12) * 11 + (off >> 6));
    sched_setaffinity(0, sizeof allowed, &allowed);

    // Where the pages are (the blobs must follow them).
    std::vector<void*> ptrs(pages);
    std::vector<int> status(pages, INT_MIN);
    for (size_t i = 0; i < pages; ++i) ptrs[i] = p + i * ps;
    if (syscall(SYS_move_pages, 0, pages, ptrs.data(), nullptr, status.data(), 0) != 0) { perror("move_pages"); return 1; }
    size_t on1 = 0;
    for (size_t i = 0; i < pages; ++i) on1 += status[i] == 1;
    if (on1 < pages * 9 / 10) {
        std::printf("numa_queue_overflow: SKIP (only %zu of %zu pages landed on node 1)\n", on1, pages);
        return 0;
    }

    uint64_t q0 = 0, f0 = 0;
    route(&q0, &f0);
    size_t compressed = 0;
    int waited = 0;
    for (; waited < 120000 && compressed < pages * 9 / 10; waited += 500) {
        usleep(500000);
        compressed = 0;
        for (size_t i = 0; i < pages; ++i) {
            int st = 0, c = 0, r = 0;
            if (page_info(p + i * ps, &st, &c, &r) == 0 && st == 2) ++compressed;
        }
    }
    uint64_t q1 = 0, f1 = 0;
    route(&q1, &f1);

    // Blob placement.
    size_t blobs = 0;
    for (size_t i = 0; i < pages; ++i) {
        int st = 0, c = 0, r = 0;
        if (page_info(p + i * ps, &st, &c, &r) != 0 || st != 2) continue;
        if (void* b = blob_for(p + i * ps)) ptrs[blobs++] = b;
    }
    size_t off_node = 0;
    int first = 0;
    if (blobs) {
        for (size_t i = 0; i < blobs; ++i) status[i] = INT_MIN;
        if (syscall(SYS_move_pages, 0, blobs, ptrs.data(), nullptr, status.data(), 0) != 0) { perror("move_pages blobs"); return 1; }
        for (size_t i = 0; i < blobs; ++i)
            if (status[i] != 1) { if (off_node++ == 0) first = status[i]; }
    }
    std::printf("numa_queue_overflow: %zu of %zu pages on node 1; %zu compressed after %d ms; queued %llu, queue full %llu; %zu of %zu blobs off node 1 (first status %d)\n",
                on1, pages, compressed, waited, static_cast<unsigned long long>(q1 - q0),
                static_cast<unsigned long long>(f1 - f0), off_node, blobs, first);
    if (compressed < pages * 9 / 10) fail("the block did not compress in 120 s");
    if (f1 - f0 == 0) fail("node 1's queue never overflowed: the test did not exercise the overflow path");
    if (blobs == 0 || off_node * 10 > blobs) fail("blobs of pages that overflowed the node queue landed off their node");

    size_t bad = 0;
    for (size_t off = 0; off < bytes; off += 64) bad += p[off] != static_cast<unsigned char>((off >> 12) * 11 + (off >> 6));
    if (bad) fail("data did not read back");
    std::free(p);
    if (failures) { std::fprintf(stderr, "numa_queue_overflow: %d failure(s)\n", failures); return 1; }
    std::printf("numa_queue_overflow: all tests passed\n");
    return 0;
}
