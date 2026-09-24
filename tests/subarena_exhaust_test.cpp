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
//   region. Measured on the 8-CPU host: 978 MiB with sub-arenas off, 960 MiB
//   with sub-arenas on; 448 MiB without donating the pooled chunks and 576
//   MiB without retiring the tails into them (both mutations fail).
//
// PHASE C (SUBARENA_EXHAUST_PHASE=C) - blocks larger than a chunk get whole
//   chunks of their own, which go back to the pool when freed; the next such
//   block must reuse them rather than carve fresh ones. 100 rounds of
//   allocate/free of a 100 MiB block, through malloc_numa and through plain
//   malloc, must all stay in the 1 GiB region. With fresh chunks every time,
//   the region runs out after a handful of rounds.
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
    for (int plain = 0; plain < 2; ++plain) {
        long in = 0, out = 0, nulls = 0;
        for (int i = 0; i < 100; ++i) {
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
        std::printf("phase C: %s: 100 rounds of a 100 MiB block, %ld in the arena, %ld out, "
                    "%ld NULL\n", plain ? "malloc" : "malloc_numa", in, out, nulls);
        if (out || nulls) {
            std::printf("FAIL: phase C: %s blocks larger than a chunk left the 1 GiB region: "
                        "freed chunks are not being reused\n", plain ? "plain" : "node");
            ++failures;
        }
    }
    return failures ? 1 : 0;
}

}  // namespace

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    g_numa = reinterpret_cast<numa_fn>(dlsym(RTLD_DEFAULT, "malloc_numa"));
    g_in_arena = reinterpret_cast<in_arena_fn>(dlsym(RTLD_DEFAULT, "smash_in_arena"));
    if (!g_numa || !g_in_arena) { std::printf("SKIP: not running under libsmash\n"); return 0; }
    const char* ph = std::getenv("SUBARENA_EXHAUST_PHASE");
    const int rc = ph && ph[0] == 'B' ? phaseB() : ph && ph[0] == 'C' ? phaseC() : phaseA();
    std::printf("subarena_exhaust_test: %s\n", rc ? "FAILED" : "done");
    return rc;
}
