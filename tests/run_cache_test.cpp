// run_cache_test.cpp - the per-CPU cache of freed large runs
// (SMASH_RUN_CACHE_MIB; see agents/design-run-cache.md).
//
// Without the cache a freed large block goes through the decommit queue —
// mprotect, madvise(MADV_DONTNEED), a free-map insert under the node arena's
// lock — and reusing it takes a zero-fill fault on every page. With the cache,
// a block freed warm and reallocated at the same size on the same CPU comes
// straight back with its pages still resident. That difference is what phase
// 1 measures, in minor page faults, which needs no smash internals.
//
// The expectation follows SMASH_RUN_CACHE_MIB, so one body serves the "on"
// and "off" registrations (off must mean off: faults on every page, no hits).
//
//   1. reuse: same size, same CPU, same node -> no faults, a hit, still bound
//   2. churn: many threads, sizes and nodes; tagged blocks must never be
//      handed to two owners, and every node block stays on its node
//   3. cold: a block compressed before it is freed must not come back through
//      the cache (its pages may be PROT_NONE) and must be fully usable
//   4. bound and drain: cached bytes stay under the cap and go to 0 once
//      the trim age has passed
#include <atomic>
#include <chrono>
#include <cerrno>
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
numa_fn g_numa = nullptr;
info_fn g_info = nullptr;
int failures = 0;
bool cache_on = false;
size_t cap_bytes = 0;
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

void touch(unsigned char* p, size_t bytes, unsigned char v) {
    for (size_t o = 0; o < bytes; o += ps) p[o] = v;
}

void pinToOneCpu() {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(sched_getcpu(), &set);
    sched_setaffinity(0, sizeof(set), &set);
}

uint64_t hits() {
    uint64_t h = 0, b = 0;
    if (g_info) g_info(&h, &b);
    return h;
}
uint64_t cachedBytes() {
    uint64_t h = 0, b = 0;
    if (g_info) g_info(&h, &b);
    return b;
}

// ── 1 ──────────────────────────────────────────────────────────────────────
void phaseReuse(const char* what, void* (*alloc)(size_t), int node) {
    constexpr size_t kSize = 4u << 20;
    const size_t pages = kSize / ps;
    auto* p = static_cast<unsigned char*>(alloc(kSize));
    if (!p) { fail("%s: allocation failed", what); return; }
    touch(p, kSize, 1);
    free(p);
    const uint64_t h0 = hits();
    auto* q = static_cast<unsigned char*>(alloc(kSize));
    if (!q) { fail("%s: reallocation failed", what); return; }
    const long f0 = minorFaults();
    touch(q, kSize, 2);
    const long faults = minorFaults() - f0;
    const uint64_t got = hits() - h0;
    std::printf("  %s: reused %s, %ld faults touching %zu pages, %llu cache hit(s)\n",
                what, q == p ? "the same block" : "a different block", faults, pages,
                static_cast<unsigned long long>(got));
    if (cache_on) {
        if (q != p || got == 0)
            fail("%s: a block freed warm and reallocated at the same size on the same "
                 "CPU did not come back from the run cache", what);
        if (faults * 10 > static_cast<long>(pages))
            fail("%s: %ld faults re-touching a cached block of %zu pages — its pages "
                 "were decommitted, so the cache saved nothing", what, faults, pages);
    } else {
        if (got != 0) fail("%s: %llu run-cache hits with the cache off", what,
                           static_cast<unsigned long long>(got));
        if (faults * 2 < static_cast<long>(pages))
            fail("%s: only %ld faults on %zu reused pages with the cache off — freed "
                 "blocks are not being decommitted", what, faults, pages);
    }
    if (node >= 0) {
        std::vector<void*> ptrs;
        for (size_t o = 0; o < kSize; o += ps) ptrs.push_back(q + o);
        int first = 0;
        if (size_t off = numa_test::pointersOffNode(ptrs, node, &first))
            fail("%s: %zu of %zu pages of the reused block are off node %d (first "
                 "status %d)", what, off, pages, node, first);
    }
    free(q);
}

void* numaOnNode0(size_t n) { return g_numa(static_cast<uint32_t>(numa_test::memoryNodes()[0]), n); }
void* plainMalloc(size_t n) { return malloc(n); }

// ── 2 ──────────────────────────────────────────────────────────────────────
void phaseChurn(const std::vector<int>& nodes) {
    const int kThreads = 16;
    std::atomic<long> corrupt{0}, off_node{0}, ops{0};
    std::atomic<bool> stop{false};
    std::vector<std::thread> th;
    for (int t = 0; t < kThreads; ++t) {
        th.emplace_back([&, t] {
            struct Block { uint64_t* p; size_t bytes; uint64_t tag; int node; };
            std::vector<Block> ring(8, Block{nullptr, 0, 0, -1});
            unsigned s = 99 + t;
            uint64_t seq = 0;
            while (!stop.load(std::memory_order_relaxed)) {
                s = s * 1103515245 + 12345;
                Block& b = ring[(s >> 8) % ring.size()];
                if (b.p) {
                    // Every page's first word still carries this owner's tag.
                    for (size_t o = 0; o < b.bytes / 8; o += ps / 8)
                        if (b.p[o] != b.tag) { ++corrupt; break; }
                    free(b.p);
                }
                b.bytes = ((s >> 4) % 4 + 1) * (1u << 20);
                b.tag = (static_cast<uint64_t>(t) << 40) | ++seq;
                b.node = (g_numa && !nodes.empty() && (s & 1))
                    ? nodes[(s >> 12) % nodes.size()] : -1;
                b.p = static_cast<uint64_t*>(b.node >= 0
                    ? g_numa(static_cast<uint32_t>(b.node), b.bytes) : malloc(b.bytes));
                if (!b.p) continue;
                for (size_t o = 0; o < b.bytes / 8; o += ps / 8) b.p[o] = b.tag;
                if (b.node >= 0 && (seq & 15) == 0) {
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
    std::printf("  churn: %ld blocks, %ld corrupted, %ld sampled pages off their node, "
                "%llu cache hits\n", ops.load(), corrupt.load(), off_node.load(),
                static_cast<unsigned long long>(hits()));
    if (corrupt) fail("%ld blocks were overwritten while live — a run was handed to two "
                      "owners", corrupt.load());
    if (off_node) fail("%ld sampled node blocks were on the wrong node", off_node.load());
    if (cache_on && hits() == 0) fail("churn never hit the run cache");
}

// ── 3 ──────────────────────────────────────────────────────────────────────
void phaseCold(void* (*alloc)(size_t), const char* what) {
    constexpr size_t kSize = 4u << 20;
    const size_t pages = kSize / ps;
    auto* p = static_cast<unsigned char*>(alloc(kSize));
    if (!p) { fail("%s: allocation failed", what); return; }
    std::memset(p, 0x5A, kSize);
    std::vector<unsigned char> vec(pages);
    size_t gone = 0;
    for (int i = 0; i < 120 && gone < pages / 2; ++i) {   // up to 30 s
        usleep(250 * 1000);
        gone = 0;
        if (mincore(p, kSize, vec.data()) == 0)
            for (unsigned char v : vec) gone += !(v & 1);
    }
    if (gone < pages / 2) {
        fail("%s: the block never left RSS (%zu of %zu pages), so this phase proves "
             "nothing", what, gone, pages);
        free(p);
        return;
    }
    const uint64_t b0 = cachedBytes();
    free(p);
    const uint64_t b1 = cachedBytes();
    if (b1 > b0) fail("%s: a block with compressed pages went into the run cache", what);
    auto* q = static_cast<unsigned char*>(alloc(kSize));
    if (!q) { fail("%s: reallocation failed", what); return; }
    std::memset(q, 0xA5, kSize);           // faults here would hang or crash
    size_t bad = 0;
    for (size_t o = 0; o < kSize; ++o) bad += q[o] != 0xA5;
    std::printf("  %s: %zu of %zu pages had left RSS; reallocated and rewritten, %zu bad "
                "bytes\n", what, gone, pages, bad);
    if (bad) fail("%s: %zu bytes wrong after rewriting a reused cold block", what, bad);
    free(q);
}

// ── 4 ──────────────────────────────────────────────────────────────────────
void phaseBound() {
    constexpr size_t kSize = 8u << 20;
    std::vector<void*> blocks;
    for (int i = 0; i < 16; ++i) {
        auto* p = static_cast<unsigned char*>(malloc(kSize));
        if (!p) { fail("malloc(8 MiB) failed"); break; }
        touch(p, kSize, 3);
        blocks.push_back(p);
    }
    for (void* p : blocks) free(p);
    const uint64_t held = cachedBytes();
    std::printf("  bound: after freeing 128 MiB on one CPU, %llu MiB cached (cap %zu MiB)\n",
                static_cast<unsigned long long>(held >> 20), cap_bytes >> 20);
    if (held > cap_bytes) fail("the run cache holds %llu bytes, over its %zu-byte cap",
                               static_cast<unsigned long long>(held), cap_bytes);
    if (cache_on && held == 0) fail("nothing was cached after freeing 16 warm blocks");
    usleep(500 * 1000);                    // several trim ages
    const uint64_t left = cachedBytes();
    std::printf("  drain: %llu bytes cached 500 ms later\n",
                static_cast<unsigned long long>(left));
    if (left) fail("%llu bytes still cached 500 ms after the last free — the trim is not "
                   "returning aged runs", static_cast<unsigned long long>(left));
}

}  // namespace

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) {
        std::printf("SKIP: not running under libsmash\n");
        return 0;
    }
    g_numa = reinterpret_cast<numa_fn>(dlsym(RTLD_DEFAULT, "malloc_numa"));
    g_info = reinterpret_cast<info_fn>(dlsym(RTLD_DEFAULT, "smash_run_cache_info"));
    const char* mib = std::getenv("SMASH_RUN_CACHE_MIB");
    cache_on = mib && std::atol(mib) > 0;
    cap_bytes = cache_on ? static_cast<size_t>(std::atol(mib)) << 20 : 0;
    std::printf("run_cache_test: SMASH_RUN_CACHE_MIB=%s\n", mib ? mib : "(unset)");
    // Missing export is a failure, but keep going: the phases below measure
    // behaviour (page faults, addresses) that does not depend on it.
    if (!g_info) fail("smash_run_cache_info is not exported");
    else {
        uint64_t h = 0, b = 0;
        const int rc = g_info(&h, &b);
        if (cache_on != (rc == 0))
            fail("smash_run_cache_info says the cache is %s", rc == 0 ? "on" : "off");
    }
    pinToOneCpu();   // the cache is per CPU: phases 1, 3 and 4 need one CPU

    errno = 0;
    bool numa_ok = false;
    const std::vector<int> nodes = numa_test::memoryNodes();
    if (g_numa && !nodes.empty()) {
        void* probe = g_numa(static_cast<uint32_t>(nodes[0]), ps);
        numa_ok = probe || !(errno == EPERM || errno == ENOSYS);
        free(probe);
    }

    std::printf("1. reuse\n");
    phaseReuse("malloc 4 MiB", plainMalloc, -1);
    if (numa_ok) phaseReuse("malloc_numa 4 MiB", numaOnNode0, nodes[0]);
    else std::printf("  malloc_numa: skipped (mbind refused)\n");

    std::printf("2. churn\n");
    phaseChurn(numa_ok ? nodes : std::vector<int>{});

    std::printf("3. cold\n");
    phaseCold(plainMalloc, "malloc");
    if (numa_ok) phaseCold(numaOnNode0, "malloc_numa");

    std::printf("4. bound and drain\n");
    phaseBound();

    if (failures) { std::printf("run_cache_test: %d FAILURES\n", failures); return 1; }
    std::printf("run_cache_test: all checks passed\n");
    return 0;
}
