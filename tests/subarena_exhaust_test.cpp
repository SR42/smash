// subarena_exhaust_test.cpp - sub-arenas must not strand free space when the
// compressible region runs out (SMASH_SUBARENAS; agents/design-subarenas.md).
//
// Every sub-arena keeps the unused end of its newest chunk as a private tail.
// With many sub-arenas that is up to one mostly free 64 MiB chunk each, so a
// region that is "out" can still hold a lot of free space. Run with a 1 GiB
// region (SMASH_VM_GIB=1), SMASH_SUBARENAS=1, SMASH_SUBARENAS_PER_NODE=2.
//
// PHASE A (SUBARENA_EXHAUST_PHASE=A) - a node arena: CPU a's sub-arena takes a
//   1 MiB block, so its chunk's other 63 MiB is its tail. CPU b, the node's
//   other sub-arena, then takes 32 MiB blocks until the region is out. Before
//   any of b's blocks leaves the arena, b must have been given space from a's
//   tail: the last-resort retry takes siblings' tails, not only its own.
//
// PHASE B (SUBARENA_EXHAUST_PHASE=B, full mode) - the shared arena: 8 threads
//   churn 2 MiB blocks, which leaves up to 8 sub-arenas each holding a tail
//   and pooled chunks; after they are freed, 256-byte objects (slab spans,
//   which never use sub-arenas) must still fill at least 900 MiB of the
//   region. The churn runs on two CPUs of one node, so its threads share two
//   sub-arenas and are preempted inside them. Measured on the 8-CPU host: 978
//   MiB with sub-arenas off, 960 MiB with sub-arenas on (the rest is the chunk
//   holding the test's own 36 MB pointer array); 448 MiB without donating the
//   pooled chunks, 576 MiB without retiring the tails into them, and 768-896
//   MiB in about half the runs when a chunk left wholly free by a tail change
//   was not pooled (also seen on the 128-core aarch64 box: 896).
//
// PHASE C (SUBARENA_EXHAUST_PHASE=C) - blocks larger than a chunk get whole
//   chunks of their own, which go back to the pool when freed; the next such
//   block must reuse them rather than carve fresh ones. 100 rounds of
//   allocate/free of a 100 MiB block, through malloc_numa and through plain
//   malloc, must all stay in the 1 GiB region. With fresh chunks every time,
//   the region runs out after a handful of rounds. 300 rounds, the process on
//   one CPU: a freed block's chunks reach the pool through the decommit queue,
//   and when the queue lags the next block must wait for them rather than
//   fall out (1-7 of 100 fell out on the 2-CPU claudepc before it did).
//
// PHASE D (SUBARENA_EXHAUST_PHASE=D) - a full region whose free space is a
//   sub-arena's dirty run next to its clean run. One CPU fills the region with
//   4 MiB blocks. In one of its chunks it frees a block and waits for the
//   purger to make that run clean, then frees the neighbour (dirty) and asks
//   for 8 MiB: the two runs hold it together, though neither alone does, as
//   dirty and clean runs never merge. The block must come from them, in the
//   arena, without purging anything. Then a genuine miss: one more block freed
//   (dirty) and a 32 MiB request that nothing in the region can hold. It
//   leaves the arena, but the dirty run must stay dirty. Before the fix, every
//   such miss purged every sub-arena's dirty runs, so each later block was
//   faulted in again. Churning 1-3 MiB blocks at 70-75 % of a 1 GiB region
//   (8 threads) then fell from about 200k to about 10k blocks a second for
//   good, and the page faults rose from 0.2M/s to 1.7M/s.

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <sched.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "numa_test_util.h"

namespace {

using numa_fn = void* (*)(uint32_t, size_t);
using in_arena_fn = int (*)(const void*);
numa_fn g_numa = nullptr;
in_arena_fn g_in_arena = nullptr;
int failures = 0;

void pinTo(int cpu) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    sched_setaffinity(0, sizeof(set), &set);
}

int phaseA() {
    const std::vector<int> nodes = numa_test::memoryNodes();
    if (nodes.empty()) { std::printf("SKIP: no memory nodes\n"); return 0; }
    const int node = nodes[0];
    char path[96];
    std::snprintf(path, sizeof(path), "/sys/devices/system/node/node%d/cpulist", node);
    cpu_set_t allowed;
    sched_getaffinity(0, sizeof(allowed), &allowed);
    std::vector<int> cpus;
    for (int c : numa_test::readList(path))
        if (CPU_ISSET(c, &allowed)) cpus.push_back(c);
    if (cpus.size() < 2) { std::printf("SKIP: fewer than two CPUs on node %d\n", node); return 0; }

    constexpr size_t kChunk = 64u << 20;
    pinTo(cpus[0]);
    errno = 0;
    auto* a = static_cast<unsigned char*>(g_numa(static_cast<uint32_t>(node), 1u << 20));
    if (!a && (errno == EPERM || errno == ENOSYS)) { std::printf("SKIP: mbind refused\n"); return 0; }
    if (!a || !g_in_arena(a)) { std::printf("FAIL: the first node block is not in the arena\n"); return 1; }
    a[0] = 1;
    const auto a_chunk = reinterpret_cast<uintptr_t>(a) / kChunk * kChunk;

    pinTo(cpus[1]);
    std::vector<void*> blocks;
    long in = 0, in_a_chunk = 0, out = 0;
    for (int i = 0; i < 64 && !out; ++i) {
        void* p = g_numa(static_cast<uint32_t>(node), 32u << 20);
        if (!p) break;
        blocks.push_back(p);
        if (!g_in_arena(p)) { ++out; break; }
        ++in;
        if (reinterpret_cast<uintptr_t>(p) / kChunk * kChunk == a_chunk) ++in_a_chunk;
    }
    std::printf("phase A: CPU %d took %ld x 32 MiB in the arena, %ld of them in CPU %d's "
                "chunk, before %s\n", cpus[1], in, in_a_chunk, cpus[0],
                out ? "one fell out of the arena" : "stopping");
    if (!out) std::printf("  (the region never ran out: this phase proves nothing)\n");
    if (out && in_a_chunk == 0) {
        std::printf("FAIL: phase A: the node's region ran out while its other sub-arena "
                    "still held a 63 MiB tail, and the allocation left the arena instead\n");
        ++failures;
    }
    for (void* p : blocks) free(p);
    free(a);
    return failures ? 1 : 0;
}

int phaseB() {
    const char* lo = std::getenv("SMASH_LARGE_ONLY");
    if (lo && lo[0] == '1') { std::printf("SKIP: phase B needs full mode\n"); return 0; }
    // Eight threads on two CPUs of one node: they share two sub-arenas and
    // are preempted inside them, which is what stranded whole free chunks in
    // a sub-arena's clean runs (768-896 MiB instead of 960, about half the
    // runs, before the fix; the 128-core box's 896 was the same). Spread
    // over more CPUs it did not show.
    cpu_set_t allowed, two;
    sched_getaffinity(0, sizeof(allowed), &allowed);
    CPU_ZERO(&two);
    {
        const std::vector<int> nodes = numa_test::memoryNodes();
        char path[96];
        std::snprintf(path, sizeof(path), "/sys/devices/system/node/node%d/cpulist",
                      nodes.empty() ? 0 : nodes[0]);
        int picked = 0;
        for (int c : numa_test::readList(path))
            if (CPU_ISSET(c, &allowed) && picked < 2) { CPU_SET(c, &two); ++picked; }
        if (picked == 0) two = allowed;
    }
    // The whole process from here, before its first large block, so smash's
    // lazily started purger thread inherits the two CPUs as well: confining
    // only the churn threads showed the stranding in 2 runs of 20, the whole
    // process in about half.
    sched_setaffinity(0, sizeof(two), &two);
    std::vector<std::thread> th;
    for (int t = 0; t < 8; ++t) {
        th.emplace_back([] {
            std::vector<void*> v;
            for (int i = 0; i < 16; ++i) {
                auto* p = static_cast<char*>(malloc(2u << 20));
                if (p) { p[0] = 1; v.push_back(p); }
            }
            for (void* p : v) free(p);
        });
    }
    for (auto& t : th) t.join();
    usleep(800 * 1000);   // past the decay: dirty runs purged, whole chunks pooled

    std::vector<void*> small;
    small.reserve(4500000);
    long in = 0;
    for (long i = 0; i < 4500000; ++i) {
        void* p = malloc(256);
        if (!p || !g_in_arena(p)) { if (p) free(p); break; }
        *static_cast<volatile char*>(p) = 1;
        small.push_back(p);
        ++in;
    }
    const double mib = static_cast<double>(in) * 256 / (1 << 20);
    std::printf("phase B: %.0f MiB of 256-byte objects in the 1 GiB arena after large-block "
                "churn\n", mib);
    if (mib < 900) {
        std::printf("FAIL: phase B: only %.0f MiB of small objects fit; the shared arena's "
                    "sub-arenas are holding free chunks slab spans cannot use\n", mib);
        ++failures;
    }
    for (void* p : small) free(p);
    return failures ? 1 : 0;
}

int phaseC() {
    const std::vector<int> nodes = numa_test::memoryNodes();
    const int node = nodes.empty() ? 0 : nodes[0];
    constexpr size_t kBig = 100u << 20;
    constexpr int kRounds = 300;
    cpu_set_t allowed;
    sched_getaffinity(0, sizeof(allowed), &allowed);
    for (int c = 0; c < CPU_SETSIZE; ++c)
        if (CPU_ISSET(c, &allowed)) { pinTo(c); break; }   // before smash's queue thread starts
    for (int plain = 0; plain < 2; ++plain) {
        long in = 0, out = 0, nulls = 0;
        for (int i = 0; i < kRounds; ++i) {
            errno = 0;
            auto* p = static_cast<unsigned char*>(
                plain ? malloc(kBig) : g_numa(static_cast<uint32_t>(node), kBig));
            if (!p) {
                if (!plain && (errno == EPERM || errno == ENOSYS)) break;   // no mbind
                ++nulls;
                continue;
            }
            p[0] = 1;
            p[kBig - 1] = 1;
            if (g_in_arena(p)) ++in; else ++out;
            free(p);
        }
        std::printf("phase C: %s: %d rounds of a 100 MiB block, %ld in the arena, %ld out, "
                    "%ld NULL\n", plain ? "malloc" : "malloc_numa", kRounds, in, out, nulls);
        if (out || nulls) {
            std::printf("FAIL: phase C: %s blocks larger than a chunk left the 1 GiB region: "
                        "freed chunks are not being reused\n", plain ? "plain" : "node");
            ++failures;
        }
    }
    return failures ? 1 : 0;
}

using subarena_info_fn = int (*)(uint64_t*, uint64_t*);
subarena_info_fn g_subarena_info = nullptr;

int phaseD() {
    constexpr size_t kChunk = 64u << 20, kBlock = 4u << 20;
    constexpr int kMax = 512;
    static void* blocks[kMax];   // static: a vector this size would come from smash
    cpu_set_t allowed;
    sched_getaffinity(0, sizeof(allowed), &allowed);
    for (int c = 0; c < CPU_SETSIZE; ++c)
        if (CPU_ISSET(c, &allowed)) { pinTo(c); break; }   // one sub-arena, and its purger
    int n = 0;
    bool out = false;
    while (n < kMax) {
        auto* p = static_cast<unsigned char*>(malloc(kBlock));
        if (!p) break;
        p[0] = 1;
        if (!g_in_arena(p)) { free(p); out = true; break; }
        blocks[n++] = p;
    }
    if (!out) { std::printf("FAIL: phase D: the region never ran out (%d blocks)\n", n); return 1; }
    // A chunk wholly filled by 16 of our blocks, in address order. The chunk
    // grid starts at the region's base, which is the lowest block: the blocks
    // fill the region.
    uintptr_t x = 0, base = UINTPTR_MAX;
    for (int i = 0; i < n; ++i) base = std::min(base, reinterpret_cast<uintptr_t>(blocks[i]));
    auto have = [&](uintptr_t a) {
        for (int i = 0; i < n; ++i) if (reinterpret_cast<uintptr_t>(blocks[i]) == a) return true;
        return false;
    };
    for (int i = 0; i < n && !x; ++i) {
        const auto a = reinterpret_cast<uintptr_t>(blocks[i]);
        if ((a - base) % kChunk) continue;
        bool full = true;
        for (size_t k = 1; k < kChunk / kBlock && full; ++k) full = have(a + k * kBlock);
        if (full) x = a;
    }
    if (!x) { std::printf("FAIL: phase D: no chunk filled by 16 of the %d blocks\n", n); return 1; }
    auto release = [&](uintptr_t a) {
        for (int i = 0; i < n; ++i)
            if (reinterpret_cast<uintptr_t>(blocks[i]) == a) { free(blocks[i]); blocks[i] = blocks[--n]; return; }
    };
    uint64_t dirty = 0, purged0 = 0, purged1 = 0;

    // Clean [20, 24) MiB of the chunk: free, then wait out the decay.
    release(x + 5 * kBlock);
    for (int i = 0; i < 500; ++i) {
        g_subarena_info(&dirty, &purged0);
        if (dirty == 0) break;
        usleep(10 * 1000);
    }
    if (dirty) { std::printf("FAIL: phase D: the freed block was never purged\n"); return 1; }
    // Dirty [24, 28) MiB, and 8 MiB asked for at once.
    release(x + 6 * kBlock);
    auto* mixed = static_cast<unsigned char*>(malloc(2 * kBlock));
    g_subarena_info(&dirty, &purged1);
    const bool mixed_in = mixed && g_in_arena(mixed);
    std::printf("phase D: 8 MiB over a clean and a dirty 4 MiB run: %s, %s the runs; "
                "%llu run(s) purged by the allocation\n",
                mixed_in ? "in the arena" : "out of the arena",
                reinterpret_cast<uintptr_t>(mixed) == x + 5 * kBlock ? "from" : "not from",
                static_cast<unsigned long long>(purged1 - purged0));
    if (!mixed_in || reinterpret_cast<uintptr_t>(mixed) != x + 5 * kBlock) {
        std::printf("FAIL: phase D: the region ran out and the request did not get the clean "
                    "and dirty runs that hold it together\n");
        ++failures;
    }
    if (mixed) mixed[0] = mixed[2 * kBlock - 1] = 1;

    // A genuine miss: dirty [40, 44) MiB, then 32 MiB, which nothing holds.
    release(x + 10 * kBlock);
    g_subarena_info(&dirty, &purged0);
    const uint64_t dirty0 = dirty;
    auto* big = static_cast<unsigned char*>(malloc(8 * kBlock));
    g_subarena_info(&dirty, &purged1);
    std::printf("phase D: 32 MiB with no room: %s; dirty %llu -> %llu MiB, %llu run(s) purged\n",
                big && g_in_arena(big) ? "in the arena" : "out of the arena",
                static_cast<unsigned long long>(dirty0 >> 20),
                static_cast<unsigned long long>(dirty >> 20),
                static_cast<unsigned long long>(purged1 - purged0));
    if (purged1 != purged0 || dirty < kBlock) {
        std::printf("FAIL: phase D: a miss that no purge could serve purged the sub-arenas' "
                    "dirty runs, and every block reusing them is faulted in again\n");
        ++failures;
    }
    free(big);
    free(mixed);
    while (n) free(blocks[--n]);
    return failures ? 1 : 0;
}

}  // namespace

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    g_numa = reinterpret_cast<numa_fn>(dlsym(RTLD_DEFAULT, "malloc_numa"));
    g_in_arena = reinterpret_cast<in_arena_fn>(dlsym(RTLD_DEFAULT, "smash_in_arena"));
    if (!g_numa || !g_in_arena) { std::printf("SKIP: not running under libsmash\n"); return 0; }
    g_subarena_info = reinterpret_cast<subarena_info_fn>(dlsym(RTLD_DEFAULT, "smash_subarena_info"));
    const char* ph = std::getenv("SUBARENA_EXHAUST_PHASE");
    if (ph && ph[0] == 'D' && (!g_subarena_info || g_subarena_info(nullptr, nullptr) != 0)) {
        std::printf("FAIL: phase D needs SMASH_SUBARENAS=1 and smash_subarena_info\n");
        return 1;
    }
    const int rc = ph && ph[0] == 'B'   ? phaseB()
                   : ph && ph[0] == 'C' ? phaseC()
                   : ph && ph[0] == 'D' ? phaseD()
                                        : phaseA();
    std::printf("subarena_exhaust_test: %s\n", rc ? "FAILED" : "done");
    return rc;
}
