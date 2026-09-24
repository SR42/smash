// subarena_test.cpp - per-CPU sub-arenas with immediate dirty reuse
// (SMASH_SUBARENAS; design: agents/design-subarenas.md).
//
// With sub-arenas a warm block freed from a node arena goes straight into its
// chunk's sub-arena as "dirty" — resident, no syscall — and any later
// allocation that fits can be carved from it. Without them it is decommitted
// first, so its next owner takes a zero-fill fault per page. Minor faults are
// the discriminator, as in test_run_cache; they need no smash internals.
//
// The expectation follows SMASH_SUBARENAS, so the "off" registration asserts
// today's behaviour from the same body. Registered with
// SMASH_SUBARENAS_PER_NODE=2, SMASH_DIRTY_DECAY_MS=200, SMASH_DIRTY_MAX_MIB=32.
//
//   1. reuse at a different size: 4 MiB freed warm, 3 MiB allocated on the
//      same CPU -> the same pages, no faults (the exact-size run cache cannot)
//   2. frees go home: freed on a CPU of the other sub-arena, reallocated on
//      the first -> the same block, no faults
//   3. cold: a block compressed before it is freed is not reused dirty, and
//      is fully writable afterwards
//   4. dirty memory stays under the cap (within 5 ms of the frees, not only
//      at the purger's next epoch) and drains to 0 after the decay time
//   5. sibling scan: a sub-arena with nothing free takes a sibling's dirty run
//      instead of carving a new chunk
//   6. churn: tagged blocks from many threads never overlap and stay on node
//   7. nothing lost: blocks larger than a chunk (one aligned past a whole
//      chunk), then every node's free pages equal what it carved
//
// SUBARENA_TEST_PLAIN=1 runs the same phases through plain malloc(), whose
// large blocks come from the shared arena's sub-arenas (unbound chunks)
// instead of a node's. Node placement checks and the aligned node block are
// skipped there; "nothing lost" compares the shared arena's pages in use with
// its free pages (smash_shared_arena_info), in large-only mode only, where
// nothing else lives in the arena.
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include "numa_test_util.h"

namespace {

using numa_fn = void* (*)(uint32_t, size_t);
using info_fn = int (*)(uint64_t*, uint64_t*);
using arena_info_fn = int (*)(uint32_t, size_t*, size_t*);
using shared_info_fn = int (*)(size_t*, size_t*);
numa_fn g_numa = nullptr;
shared_info_fn g_shared_info = nullptr;
bool plain = false;         // SUBARENA_TEST_PLAIN: malloc() and the shared arena
info_fn g_info = nullptr;
arena_info_fn g_arena_info = nullptr;
int failures = 0;
bool on = false;
size_t cap_bytes = 0;
int node0 = 0;
const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));

void fail(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    std::fprintf(stderr, "FAIL: ");
    std::vfprintf(stderr, fmt, ap);
    std::fputc('\n', stderr);
    va_end(ap);
    ++failures;
}

long minorFaults() {
    rusage r{};
    getrusage(RUSAGE_THREAD, &r);
    return r.ru_minflt;
}
long touchFaults(unsigned char* p, size_t bytes, unsigned char v) {
    const long f0 = minorFaults();
    for (size_t o = 0; o < bytes; o += ps) p[o] = v;
    return minorFaults() - f0;
}
void pinTo(int cpu) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    sched_setaffinity(0, sizeof(set), &set);
}
uint64_t dirtyBytes() {
    uint64_t d = 0, p = 0;
    if (g_info) g_info(&d, &p);
    return d;
}
// Pages the shared arena has taken from the region (not counting node
// chunks), and how many of them are free.
bool sharedInfo(size_t* used, size_t* free_pages) {
    *used = *free_pages = 0;
    return g_shared_info && g_shared_info(used, free_pages) == 0;
}
// Grows when the allocator under test carves a new chunk.
size_t carved(int node) {
    size_t c = 0, f = 0;
    if (plain) sharedInfo(&c, &f);
    else if (g_arena_info) g_arena_info(static_cast<uint32_t>(node), &c, &f);
    return c;
}
unsigned char* alloc(size_t n) {
    return static_cast<unsigned char*>(plain ? malloc(n)
                                             : g_numa(static_cast<uint32_t>(node0), n));
}

// CPUs of node0 this process may use, in the kernel's order: with two
// sub-arenas per node, the first maps to sub-arena 0 and the second to 1.
std::vector<int> node0Cpus() {
    char path[96];
    std::snprintf(path, sizeof(path), "/sys/devices/system/node/node%d/cpulist", node0);
    std::vector<int> out;
    cpu_set_t allowed;
    if (sched_getaffinity(0, sizeof(allowed), &allowed) != 0) return out;
    for (int c : numa_test::readList(path))
        if (CPU_ISSET(c, &allowed)) out.push_back(c);
    return out;
}

// ── 1 ──────────────────────────────────────────────────────────────────────
void phaseDifferentSize(int cpu) {
    pinTo(cpu);
    auto* p = alloc(4u << 20);
    if (!p) { fail("allocating 4 MiB failed"); return; }
    touchFaults(p, 4u << 20, 1);
    free(p);
    auto* q = alloc(3u << 20);
    if (!q) { fail("allocating 3 MiB failed"); return; }
    const long faults = touchFaults(q, 3u << 20, 2);
    const size_t pages = (3u << 20) / ps;
    // Reuse means the new block lies (almost) inside the freed one: the freed
    // run can merge with a free neighbour (the one-page probe freed before
    // it), so the new block may start a page or two earlier.
    const auto lo = std::max(reinterpret_cast<uintptr_t>(p), reinterpret_cast<uintptr_t>(q));
    const auto hi = std::min(reinterpret_cast<uintptr_t>(p) + (4u << 20),
                             reinterpret_cast<uintptr_t>(q) + (3u << 20));
    const size_t overlap = hi > lo ? (hi - lo) / ps : 0;
    std::printf("  4 MiB freed, 3 MiB allocated: %zu of %zu pages inside the freed block, "
                "%ld faults\n", overlap, pages, faults);
    if (on) {
        if (overlap + 4 < pages) fail("a smaller block did not reuse the warm run just freed");
        if (faults * 10 > static_cast<long>(pages))
            fail("%ld faults re-touching a reused dirty run of %zu pages", faults, pages);
    } else if (faults * 2 < static_cast<long>(pages)) {
        fail("only %ld faults on %zu pages with sub-arenas off", faults, pages);
    }
    std::vector<void*> ptrs;
    for (size_t o = 0; o < (3u << 20); o += ps) ptrs.push_back(q + o);
    int first = 0;
    if (size_t off = plain ? 0 : numa_test::pointersOffNode(ptrs, node0, &first))
        fail("%zu pages of a reused block are off node %d (first status %d)", off, node0, first);
    free(q);
}

// ── 2 ──────────────────────────────────────────────────────────────────────
void phaseGoesHome(int cpu_a, int cpu_b) {
    pinTo(cpu_a);
    auto* p = alloc(4u << 20);
    if (!p) { fail("allocating 4 MiB failed"); return; }
    touchFaults(p, 4u << 20, 1);
    std::thread([&] { pinTo(cpu_b); free(p); }).join();
    auto* q = alloc(4u << 20);
    if (!q) { fail("allocating 4 MiB failed"); return; }
    const long faults = touchFaults(q, 4u << 20, 2);
    std::printf("  freed on CPU %d, reallocated on CPU %d: %s, %ld faults\n", cpu_b, cpu_a,
                q == p ? "same block" : "different block", faults);
    if (on && (q != p || faults * 10 > static_cast<long>((4u << 20) / ps)))
        fail("a block freed on another sub-arena's CPU did not go home to its own "
             "sub-arena warm");
    free(q);
}

// ── 3 ──────────────────────────────────────────────────────────────────────
void phaseCold(int cpu) {
    pinTo(cpu);
    constexpr size_t kSize = 4u << 20;
    const size_t pages = kSize / ps;
    auto* p = alloc(kSize);
    if (!p) { fail("allocating 4 MiB failed"); return; }
    std::memset(p, 0x5A, kSize);
    std::vector<unsigned char> vec(pages);
    size_t gone = 0;
    for (int i = 0; i < 120 && gone < pages / 2; ++i) {
        usleep(250 * 1000);
        gone = 0;
        if (mincore(p, kSize, vec.data()) == 0)
            for (unsigned char v : vec) gone += !(v & 1);
    }
    if (gone < pages / 2) {
        fail("the block never left RSS (%zu of %zu pages); this phase proves nothing",
             gone, pages);
        free(p);
        return;
    }
    const uint64_t d0 = dirtyBytes();
    free(p);
    if (dirtyBytes() > d0) fail("a block with compressed pages was kept as dirty");
    auto* q = alloc(kSize);
    if (!q) { fail("reallocation failed"); return; }
    std::memset(q, 0xA5, kSize);
    size_t bad = 0;
    for (size_t o = 0; o < kSize; ++o) bad += q[o] != 0xA5;
    std::printf("  cold block: %zu of %zu pages out of RSS; reused and rewritten, %zu bad "
                "bytes\n", gone, pages, bad);
    if (bad) fail("%zu bytes wrong in a reused cold block", bad);
    free(q);
}

// ── 4 ──────────────────────────────────────────────────────────────────────
void phaseDirtyBound(int cpu) {
    pinTo(cpu);
    std::vector<void*> blocks;
    for (int i = 0; i < 16; ++i) {
        auto* p = alloc(8u << 20);
        if (!p) { fail("allocating 8 MiB failed"); break; }
        touchFaults(p, 8u << 20, 3);
        blocks.push_back(p);
    }
    for (void* p : blocks) free(p);
    // The blocks can come from either of node0's two sub-arenas (earlier
    // phases left runs in both, and the sibling scan takes them), and each
    // frees home to its own cap: the bound is the two caps together.
    const size_t bound = 2 * cap_bytes + (8u << 20);
    // Promptly, too: the purger's epoch is 50 ms, and a free that takes a
    // sub-arena over its cap must not wait for it (between epochs a fast
    // churn held 4-6x the cap dirty). 5 ms after each of three rounds, well
    // inside one epoch, so all three passing by luck is unlikely.
    if (on) {
        for (int round = 0; round < 3; ++round) {
            usleep(5 * 1000);
            const uint64_t soon = dirtyBytes();
            std::printf("  5 ms after freeing 128 MiB (round %d): %llu MiB dirty\n", round + 1,
                        static_cast<unsigned long long>(soon >> 20));
            if (soon > bound) {
                fail("%llu MiB dirty 5 ms after the frees, over two sub-arenas' %zu MiB caps: "
                     "the cap waits for the purger's epoch",
                     static_cast<unsigned long long>(soon >> 20), cap_bytes >> 20);
                break;
            }
            if (round == 2) break;
            blocks.clear();
            for (int i = 0; i < 16; ++i) {
                auto* p = alloc(8u << 20);
                if (!p) break;
                touchFaults(p, 8u << 20, 3);
                blocks.push_back(p);
            }
            for (void* p : blocks) free(p);
        }
    }
    usleep(120 * 1000);   // two purge epochs: the cap is enforced by the purger
    const uint64_t held = dirtyBytes();
    std::printf("  after freeing 128 MiB on one CPU: %llu MiB dirty (cap %zu MiB)\n",
                static_cast<unsigned long long>(held >> 20), cap_bytes >> 20);
    if (on && held > bound)
        fail("%llu bytes dirty in node0's two sub-arenas, over their %zu-byte caps",
             static_cast<unsigned long long>(held), cap_bytes);
    if (on && held == 0) fail("freeing 128 MiB of warm blocks left nothing dirty");
    if (!on && held) fail("%llu bytes dirty with sub-arenas off",
                          static_cast<unsigned long long>(held));
    usleep(700 * 1000);   // decay 200 ms + epochs
    const uint64_t left = dirtyBytes();
    std::printf("  700 ms later: %llu bytes dirty\n", static_cast<unsigned long long>(left));
    if (left) fail("%llu bytes still dirty well past the decay time",
                   static_cast<unsigned long long>(left));
}

// ── 5 ──────────────────────────────────────────────────────────────────────
void phaseSibling(int cpu_a, int cpu_b) {
    pinTo(cpu_a);
    auto* p = alloc(8u << 20);
    if (!p) { fail("allocating 8 MiB failed"); return; }
    touchFaults(p, 8u << 20, 4);
    free(p);                               // dirty in cpu_a's sub-arena
    const size_t c0 = carved(node0);
    unsigned char* q = nullptr;
    long faults = 0;
    std::thread([&] {
        pinTo(cpu_b);
        // The other sub-arena has chunks of its own by now (phase 2 freed
        // there); fill whatever it holds so its own free runs and tail miss.
        std::vector<void*> fill;
        for (int i = 0; i < 64; ++i) {
            void* f = alloc(8u << 20);
            if (!f || f == p) { if (f == p) q = static_cast<unsigned char*>(f); else free(f); break; }
            fill.push_back(f);
            if (carved(node0) != c0) break;  // it had to carve: its own space ran out
        }
        for (void* f : fill) free(f);
        if (q) faults = touchFaults(q, 8u << 20, 5);
    }).join();
    std::printf("  sibling scan: %s, %ld faults on %zu pages\n",
                q ? "got the block the other CPU freed" : "did not get it", faults,
                static_cast<size_t>((8u << 20) / ps));
    // Warm, too: today's shared free map hands back the same address after
    // decommitting it (2048 faults on 2048 pages) — that is not the sibling's
    // dirty run, it is the ordinary path.
    if (on && (!q || faults * 10 > static_cast<long>((8u << 20) / ps)))
        fail("a sub-arena ran out and did not take its sibling's warm 8 MiB dirty run "
             "(%s, %ld faults)", q ? "same address" : "carved instead", faults);
    if (q) free(q);
}

// ── 6 ──────────────────────────────────────────────────────────────────────
void phaseChurn(const std::vector<int>& nodes) {
    std::atomic<long> corrupt{0}, off_node{0}, ops{0};
    std::atomic<bool> stop{false};
    std::vector<std::thread> th;
    for (int t = 0; t < 16; ++t) {
        th.emplace_back([&, t] {
            struct Block { uint64_t* p; size_t bytes; uint64_t tag; int node; };
            std::vector<Block> ring(8, Block{nullptr, 0, 0, -1});
            unsigned s = 7 + t;
            uint64_t seq = 0;
            while (!stop.load(std::memory_order_relaxed)) {
                s = s * 1103515245 + 12345;
                Block& b = ring[(s >> 8) % ring.size()];
                if (b.p) {
                    for (size_t o = 0; o < b.bytes / 8; o += ps / 8)
                        if (b.p[o] != b.tag) { ++corrupt; break; }
                    free(b.p);
                }
                b.bytes = ((s >> 4) % 6 + 1) * (1u << 20);
                b.tag = (static_cast<uint64_t>(t) << 40) | ++seq;
                b.node = plain ? -1 : nodes[(s >> 12) % nodes.size()];
                b.p = static_cast<uint64_t*>(
                    plain ? malloc(b.bytes) : g_numa(static_cast<uint32_t>(b.node), b.bytes));
                if (!b.p) continue;
                for (size_t o = 0; o < b.bytes / 8; o += ps / 8) b.p[o] = b.tag;
                if (!plain && (seq & 15) == 0) {
                    int status = -1;
                    void* page = b.p;
                    if (syscall(SYS_move_pages, 0, 1, &page, nullptr, &status, 0) == 0 &&
                        status >= 0 && status != b.node)
                        ++off_node;
                }
                ++ops;
            }
            for (Block& b : ring) free(b.p);
        });
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));
    stop = true;
    for (auto& x : th) x.join();
    std::printf("  churn: %ld blocks, %ld corrupted, %ld sampled pages off their node\n",
                ops.load(), corrupt.load(), off_node.load());
    if (corrupt) fail("%ld blocks overwritten while live — a run handed to two owners",
                      corrupt.load());
    if (off_node) fail("%ld sampled blocks on the wrong node", off_node.load());
}

// ── 7 ──────────────────────────────────────────────────────────────────────
// Nothing lost: blocks larger than a chunk (one aligned so its gap skips a
// whole chunk), then, with everything freed, every node's free pages equal
// what it carved. Frees reach the free lists through the decommit queue, so
// poll.
void phaseAccounting(const std::vector<int>& nodes) {
    using aligned_fn = void* (*)(uint32_t, size_t, size_t);
    auto aligned = reinterpret_cast<aligned_fn>(dlsym(RTLD_DEFAULT, "malloc_numa_aligned"));
    const size_t mib = 1u << 20;
    unsigned char* big = alloc(100 * mib);
    unsigned char* al = aligned && !plain ? static_cast<unsigned char*>(
                                      aligned(static_cast<uint32_t>(node0), 70 * mib, 128 * mib))
                                : nullptr;
    if (!big || (aligned && !plain && !al)) fail("a block larger than a chunk was refused");
    if (big) { touchFaults(big, 100 * mib, 1); free(big); }
    if (al) { touchFaults(al, 70 * mib, 2); free(al); }
    if (plain) {
        const char* lo = std::getenv("SMASH_LARGE_ONLY");
        if (!(lo && lo[0] == '1')) {
            std::printf("  skipped: full mode keeps slab spans in the shared arena\n");
            return;
        }
        size_t u = 0, f = 0;
        bool ok = false;
        for (int i = 0; i < 300; ++i) {
            if (!(ok = sharedInfo(&u, &f)) || u == f) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::printf("  shared arena: %zu pages in use by it, %zu free\n", u, f);
        if (!ok) fail("smash_shared_arena_info is not exported");
        else if (u != f)
            fail("shared arena: %zu pages taken from the region, %zu free with nothing "
                 "allocated — %s", u, f, f > u ? "pages counted twice" : "pages lost");
        return;
    }
    for (int n : nodes) {
        size_t c = 0, f = 0;
        for (int i = 0; i < 300; ++i) {
            if (g_arena_info(static_cast<uint32_t>(n), &c, &f) != 0 || c == f) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::printf("  node %d: carved %zu pages, free %zu\n", n, c, f);
        if (c != f)
            fail("node %d: %zu pages carved, %zu free with nothing allocated — %s", n, c, f,
                 f > c ? "pages counted twice" : "pages lost");
    }
}

}  // namespace

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    g_numa = reinterpret_cast<numa_fn>(dlsym(RTLD_DEFAULT, "malloc_numa"));
    g_info = reinterpret_cast<info_fn>(dlsym(RTLD_DEFAULT, "smash_subarena_info"));
    g_arena_info = reinterpret_cast<arena_info_fn>(dlsym(RTLD_DEFAULT, "smash_numa_arena_info"));
    g_shared_info = reinterpret_cast<shared_info_fn>(dlsym(RTLD_DEFAULT, "smash_shared_arena_info"));
    const char* pl = std::getenv("SUBARENA_TEST_PLAIN");
    plain = pl && pl[0] == '1';
    if (!g_numa || !g_arena_info) { std::printf("SKIP: not running under libsmash\n"); return 0; }
    const char* v = std::getenv("SMASH_SUBARENAS");
    on = v && v[0] == '1';
    const char* cap = std::getenv("SMASH_DIRTY_MAX_MIB");
    cap_bytes = cap ? static_cast<size_t>(std::atol(cap)) << 20 : 256u << 20;
    std::printf("subarena_test: SMASH_SUBARENAS=%s, %s\n", on ? "1" : "(unset)",
                plain ? "plain malloc (shared arena)" : "malloc_numa (node arenas)");
    // A missing export is a failure, but keep going: the phases below measure
    // behaviour (faults, addresses) that does not depend on it.
    if (!g_info) fail("smash_subarena_info is not exported");

    const std::vector<int> nodes = numa_test::memoryNodes();
    if (nodes.empty()) { std::printf("SKIP: no memory nodes\n"); return 0; }
    node0 = nodes[0];
    errno = 0;
    void* probe = g_numa(static_cast<uint32_t>(node0), ps);
    if (!probe && (errno == EPERM || errno == ENOSYS)) {
        std::printf("SKIP: mbind refused\n");
        return 0;
    }
    free(probe);
    const std::vector<int> cpus = node0Cpus();
    if (cpus.empty()) { std::printf("SKIP: no usable CPU on node %d\n", node0); return 0; }
    const bool two = cpus.size() >= 2;

    std::printf("1. reuse at a different size\n");
    phaseDifferentSize(cpus[0]);
    std::printf("2. frees go home\n");
    if (two) phaseGoesHome(cpus[0], cpus[1]); else std::printf("  skipped: one CPU on node\n");
    std::printf("3. cold\n");
    phaseCold(cpus[0]);
    std::printf("4. dirty bound and decay\n");
    phaseDirtyBound(cpus[0]);
    std::printf("5. sibling scan\n");
    if (two) phaseSibling(cpus[0], cpus[1]); else std::printf("  skipped: one CPU on node\n");
    std::printf("6. churn\n");
    {
        cpu_set_t all;
        CPU_ZERO(&all);
        for (int c = 0; c < CPU_SETSIZE; ++c) CPU_SET(c, &all);
        sched_setaffinity(0, sizeof(all), &all);
    }
    phaseChurn(nodes);
    std::printf("7. nothing lost\n");
    phaseAccounting(nodes);

    if (failures) { std::printf("subarena_test: %d FAILURES\n", failures); return 1; }
    std::printf("subarena_test: all checks passed\n");
    return 0;
}
