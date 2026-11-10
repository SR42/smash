// roi_gate_test.cpp - a bucket the ROI gate has declined is compressed again
// once it holds something worth compressing (issue #22), and the gate runs
// the same with the cold timeout set (issue #10).
//
// The gate decides from the bucket's window of compression ratios, and the
// window only changes after a codec run. So a window of pages that saved
// nothing declined every later page of the bucket for good: in large-only
// mode every block of 128 pages or more from one call site's arena shares a
// bucket, and one block of random bytes switched compression off for all the
// blocks that call site allocated afterwards. Every 64th declined page now
// goes to the codec as a probe, and a probe that is stored starts the window
// again.
//
// One worker (SMASH_TEST_WORKERS=1, SMASH_NUMA_COMPRESSOR=0), so there is
// one window. One call site allocates both blocks, so they share its bucket.
//
//   1. 16 MiB of random bytes. The gate must decline most of its pages
//      without running the codec: roi_skip reaches half the block's pages,
//      and the codec has failed on fewer than a quarter of them. With a
//      cold timeout set the gate used to return true for every page, so
//      nothing was declined and every page went to the codec (issue #10;
//      registered both ways).
//   2. The block is freed, so nothing random is left to decline.
//   3. 16 MiB of compressible bytes from the same call site. At least 90 %
//      of its pages must be compressed. Without the probe: none.
//   4. The gate must have declined at most 512 of them on the way: with the
//      probe but the old window kept, it takes about 1600.
//
// The counters are the pipeline line's (SMASH_STATS_SIGNAL=1, SIGUSR2),
// read from a file standing in for stderr.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <dlfcn.h>
#include <string>
#include <unistd.h>

namespace {
using info_fn = int (*)(const void*, int*, int*, int*);
using arena_fn = int (*)(const void*);
info_fn page_info = nullptr;
int g_log_fd = -1, g_saved_err = -1;

constexpr size_t kBlock = size_t{16} << 20;
constexpr size_t kPages = kBlock / 4096;

uint64_t rng = 88172645463325252ull;
inline uint64_t next() { rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17; return rng; }

// The one call site both blocks come from. The store after the call keeps it
// from becoming a tail call, which would make the caller's two lines the
// call sites.
__attribute__((noinline)) unsigned char* allocBlock() {
    auto* p = static_cast<unsigned char*>(std::malloc(kBlock));
    if (p) *static_cast<volatile unsigned char*>(p) = 0;
    return p;
}

struct Counters { unsigned long long tried = 0, stored = 0, gate_fail = 0, roi_skip = 0; bool ok = false; };

Counters readCounters() {
    raise(SIGUSR2);
    usleep(100 * 1000);
    std::string lg;
    char buf[8192];
    lseek(g_log_fd, 0, SEEK_SET);
    for (ssize_t n; (n = read(g_log_fd, buf, sizeof buf)) > 0;) lg.append(buf, static_cast<size_t>(n));
    Counters c;
    const size_t at = lg.rfind("compress tried=");
    if (at != std::string::npos)
        c.ok = std::sscanf(lg.c_str() + at, "compress tried=%llu stored=%llu gate_fail=%llu roi_skip=%llu",
                           &c.tried, &c.stored, &c.gate_fail, &c.roi_skip) == 4;
    return c;
}

size_t compressedPages(const unsigned char* p) {
    size_t c = 0;
    for (size_t off = 0; off < kBlock; off += 4096) {
        int st = 0, cold = 0, rc = 0;
        if (page_info(p + off, &st, &cold, &rc) == 0 && (st == 2 || st == 5)) ++c;
    }
    return c;
}

int fail(const char* fmt, unsigned long long a = 0, unsigned long long b = 0, unsigned long long c = 0) {
    dprintf(g_saved_err, "FAIL: ");
    dprintf(g_saved_err, fmt, a, b, c);
    dprintf(g_saved_err, "\n");
    return 1;
}
}  // namespace

int main() {
    page_info = reinterpret_cast<info_fn>(dlsym(RTLD_DEFAULT, "smash_page_info"));
    auto arena_for = reinterpret_cast<arena_fn>(dlsym(RTLD_DEFAULT, "smash_arena_for"));
    if (!page_info || !arena_for) {
        std::fprintf(stderr, "FAIL: not running under libsmash (diagnostic exports missing)\n");
        return 1;
    }
    char path[] = "/tmp/smash_roi_gate_XXXXXX";
    g_log_fd = mkstemp(path);
    if (g_log_fd < 0) { std::printf("SKIP: mkstemp failed\n"); return 77; }
    unlink(path);
    g_saved_err = dup(2);
    dup2(g_log_fd, 2);

    // 1. Random bytes: declined, mostly without the codec.
    unsigned char* r = allocBlock();
    if (!r) return fail("malloc");
    for (size_t i = 0; i < kBlock / 8; ++i) reinterpret_cast<uint64_t*>(r)[i] = next();
    const int arena_r = arena_for(r);
    Counters c1;
    int waited = 0;
    for (; waited < 60; ++waited) {
        sleep(1);
        c1 = readCounters();
        if (c1.ok && c1.roi_skip >= kPages / 2) break;
    }
    dprintf(g_saved_err, "random block, %d s: tried=%llu stored=%llu gate_fail=%llu roi_skip=%llu of %zu pages\n",
            waited + 1, c1.tried, c1.stored, c1.gate_fail, c1.roi_skip, kPages);
    if (!c1.ok) return fail("no pipeline line on SIGUSR2 (SMASH_STATS_SIGNAL=1 not set?)");
    if (c1.roi_skip < kPages / 2)
        return fail("the ROI gate declined %llu of %llu random pages in 60 s (the codec failed on %llu): "
                    "the gate is not running", c1.roi_skip, kPages, c1.gate_fail);
    if (c1.gate_fail >= kPages / 4)
        return fail("the codec ran and failed on %llu of %llu random pages: the gate let them through",
                    c1.gate_fail, kPages);

    // 2. Nothing random is left.
    std::free(r);
    const Counters c2 = readCounters();

    // 3. Compressible bytes from the same call site.
    unsigned char* g = allocBlock();
    if (!g) return fail("malloc");
    for (size_t i = 0; i < kBlock / 8; ++i) reinterpret_cast<uint64_t*>(g)[i] = next() & 0x0f0f0f0f0f0f0f0full;
    const int arena_g = arena_for(g);
    if (arena_g != arena_r)
        return fail("one call site's two blocks are in arenas %llu and %llu: they do not share a bucket",
                    static_cast<unsigned long long>(arena_r), static_cast<unsigned long long>(arena_g));
    size_t comp = 0;
    for (waited = 0; waited < 90; ++waited) {
        sleep(1);
        comp = compressedPages(g);
        if (comp >= kPages * 9 / 10) break;
    }
    const Counters c3 = readCounters();
    const unsigned long long declined = c3.roi_skip - c2.roi_skip;
    dprintf(g_saved_err, "compressible block from the same call site, %d s: %zu of %zu pages compressed, "
            "%llu declined on the way (stored=%llu)\n", waited + 1, comp, kPages, declined, c3.stored);
    if (comp < kPages * 9 / 10)
        return fail("%llu of %llu compressible pages were compressed in 90 s, %llu were declined: the bucket "
                    "stays switched off after the random block", comp, kPages, declined);
    // 4. It recovered at the first probe.
    if (declined > 512)
        return fail("the gate declined %llu compressible pages before the bucket recovered (limit 512)", declined);

    std::free(g);
    dprintf(g_saved_err, "OK\n");
    return 0;
}
