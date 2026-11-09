// idle_direct_snapshot_test.cpp - what the snapshot of a page costs under the
// idle and yt trackers (src/compress/compressor_thread.h compressPage).
//
// Under SMASH_WRITE_TRACK=idle|yt nothing is ever protected before
// compression: a cold page is ACTIVE and PROT_RW. compressPage still read it
// through /proc/self/mem (a pread that takes mmap_lock and pins the page)
// and then read it that way again to verify -- two kernel round trips per
// page where a memcpy would do, since neither stops a concurrent write; the
// shadow window's content compare under PROT_READ (Phase B) is what catches
// one, in every mode. Measured: memread=709400 ms in one tick on the
// 128-core box. A page of a scanning tracker is now copied directly, snapshot
// and verify. The one page a worker cannot read directly is an ACTIVE page
// some earlier reclaim left PROT_NONE and nothing has touched since: its
// read faults, and the handler, seeing the worker's own lock and its
// announced snapshot, makes the page PROT_RW and lets the read run.
//
//   1. 64 MiB of compressible data goes cold; a few pages are written
//      during the shadow window. Once 90 % of it is compressed,
//      smash_tick_cost_info must show 0 /proc/self/mem reads (two per
//      page before), and every byte, the late writes included, must read
//      back.
//   2. 16 pages of a second block are made PROT_NONE by the test itself
//      (what a reclaimed-then-reallocated, never-touched page looks like);
//      they must compress (smash_snapshot_info counts the faults the
//      handler resolved: at least 16), the reads must still be direct, and
//      the data must read back.
//
// Needs CAP_SYS_ADMIN for the idle bitmap: without it the tracker falls
// back and the test skips. Registered with SMASH_WRITE_TRACK=idle,
// SMASH_LARGE_ONLY=1, cold timeout 1 s.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <sys/mman.h>
#include <unistd.h>

namespace {
int failures = 0;
void fail(const char* what) { std::fprintf(stderr, "FAIL: %s\n", what); ++failures; }
using info_fn = int (*)(const void*, int*, int*, int*);
using cost_fn = int (*)(uint64_t*, uint64_t*, uint64_t*);
using snap_fn = int (*)(uint64_t*, uint64_t*, uint64_t*);
using track_fn = const char* (*)();
info_fn g_info;
cost_fn g_cost;
snap_fn g_snap;

uint64_t memreads() { uint64_t s = 0, m = 0, t = 0; g_cost(&s, &m, &t); return m; }

unsigned char pattern(size_t off) { return static_cast<unsigned char>((off >> 12) * 13 + (off >> 6)); }

// Pages of `p` in state COMPRESSED.
size_t compressedPages(const unsigned char* p, size_t bytes, size_t ps) {
    size_t n = 0;
    for (size_t off = 0; off < bytes; off += ps) {
        int st = 0, c = 0, r = 0;
        if (g_info(p + off, &st, &c, &r) == 0 && st == 2) ++n;
    }
    return n;
}
}  // namespace

int main() {
    g_info = reinterpret_cast<info_fn>(dlsym(RTLD_DEFAULT, "smash_page_info"));
    g_cost = reinterpret_cast<cost_fn>(dlsym(RTLD_DEFAULT, "smash_tick_cost_info"));
    g_snap = reinterpret_cast<snap_fn>(dlsym(RTLD_DEFAULT, "smash_snapshot_info"));
    auto track = reinterpret_cast<track_fn>(dlsym(RTLD_DEFAULT, "smash_write_tracker"));
    if (!g_info || !g_cost || !g_snap || !track) { std::fprintf(stderr, "FAIL: diagnostic exports missing\n"); return 1; }
    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));

    // The tracker is only known once compression has started: allocate first.
    const size_t bytes = 64u << 20;
    auto* p = static_cast<unsigned char*>(std::malloc(bytes));
    if (!p) return 1;
    for (size_t off = 0; off < bytes; off += 64) p[off] = pattern(off);
    const char* name = track();
    for (int w = 0; w < 10000 && std::strcmp(name, "unprobed") == 0; w += 100) { usleep(100000); name = track(); }
    if (std::strcmp(name, "idle") != 0 && std::strcmp(name, "yt") != 0) {
        std::printf("idle_direct_snapshot: SKIP (tracker is %s: needs CAP_SYS_ADMIN for idle)\n", name);
        return 77;
    }

    // Phase 1: cold, with writes during the shadow window.
    const uint64_t mr0 = memreads();
    size_t compressed = 0;
    bool wrote_late = false;
    int waited = 0;
    for (; waited < 90000 && compressed < (bytes / ps) * 9 / 10; waited += 250) {
        usleep(250000);
        compressed = compressedPages(p, bytes, ps);
        if (!wrote_late && waited >= 1500) {
            for (size_t off = 0; off < bytes; off += 64 * ps) p[off] = static_cast<unsigned char>(~pattern(off));
            wrote_late = true;
        }
    }
    uint64_t direct = 0, direct_faults = 0, via_mem = 0;
    g_snap(&direct, &direct_faults, &via_mem);
    const uint64_t mr1 = memreads();
    std::printf("idle_direct_snapshot: tracker %s; %zu of %zu pages compressed after %d ms; %llu /proc/self/mem reads (direct %llu, via mem %llu, direct faults %llu)\n",
                name, compressed, bytes / ps, waited, static_cast<unsigned long long>(mr1 - mr0),
                static_cast<unsigned long long>(direct), static_cast<unsigned long long>(via_mem),
                static_cast<unsigned long long>(direct_faults));
    if (compressed < (bytes / ps) * 9 / 10) fail("the block did not compress in 90 s");
    if (mr1 - mr0 != 0) fail("pages of a scanning tracker were read through /proc/self/mem");
    size_t bad = 0;
    for (size_t off = 0; off < bytes; off += 64) {
        const unsigned char want = (off % (64 * ps) == 0) ? static_cast<unsigned char>(~pattern(off)) : pattern(off);
        bad += p[off] != want;
    }
    if (bad) { std::fprintf(stderr, "  %zu bytes wrong\n", bad); fail("data (late writes included) did not read back"); }

    // Phase 2: pages a worker cannot read directly.
    const size_t bytes2 = 4u << 20;
    auto* q = static_cast<unsigned char*>(std::malloc(bytes2));
    if (!q) return 1;
    for (size_t off = 0; off < bytes2; off += 64) q[off] = pattern(off);
    unsigned char* nn = q + 64 * ps;   // 16 pages, page-aligned inside the block
    if (mprotect(nn, 16 * ps, PROT_NONE) != 0) { perror("mprotect"); return 1; }
    const uint64_t mr2 = memreads();
    uint64_t df0 = 0; g_snap(&direct, &df0, &via_mem);
    compressed = 0;
    for (waited = 0; waited < 60000 && compressed < (bytes2 / ps) * 9 / 10; waited += 250) {
        usleep(250000);
        compressed = compressedPages(q, bytes2, ps);
    }
    uint64_t df1 = 0; g_snap(&direct, &df1, &via_mem);
    const uint64_t mr3 = memreads();
    std::printf("idle_direct_snapshot: second block %zu of %zu pages compressed after %d ms; %llu /proc/self/mem reads; %llu direct reads faulted and were resolved\n",
                compressed, bytes2 / ps, waited, static_cast<unsigned long long>(mr3 - mr2),
                static_cast<unsigned long long>(df1 - df0));
    if (compressed < (bytes2 / ps) * 9 / 10) fail("the second block did not compress in 60 s");
    if (mr3 - mr2 != 0) fail("the second block was read through /proc/self/mem");
    if (df1 - df0 < 16) fail("the PROT_NONE pages were not read directly through the handler (fewer than 16 resolved faults)");
    bad = 0;
    for (size_t off = 0; off < bytes2; off += 64) bad += q[off] != pattern(off);
    if (bad) { std::fprintf(stderr, "  %zu bytes wrong\n", bad); fail("second block did not read back"); }

    std::free(q);
    std::free(p);
    if (failures) { std::fprintf(stderr, "idle_direct_snapshot: %d failure(s)\n", failures); return 1; }
    std::printf("idle_direct_snapshot: all tests passed\n");
    return 0;
}
