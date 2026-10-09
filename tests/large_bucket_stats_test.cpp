// large_bucket_stats_test.cpp - pages inside a large block must be counted in
// the block's own (arena, size bucket) statistics.
//
// The compressor finds a page's bucket through the page map, but a large
// block registers only its FIRST page there (O(1) malloc/free), so every page
// inside one found no span: no ratio samples, no compression counts, and the
// ROI model never had data for them (fewer than 8 samples means "compress
// optimistically, fast tier"). In SMASH_LARGE_ONLY=1 that is the whole heap.
//
// 64 MiB held idle until it leaves RSS; the SMASH_BUCKET_STATS=1 dump must
// report compressions for at least 90% of its pages, at least 90% of them in
// large-block buckets (obj_size > 16 KiB). The dump also has to include the
// NUMA node-group workers, which compress most pages on a multi-node host.
//
// With statistics, the ROI model declines buckets whose data does not
// compress, before any codec runs — and a declined page must back off like a
// ratio-gate failure, not be offered again every tick. 32 MiB of random
// bytes held idle 25 s (LBS_PART=decline, its own process and registration,
// default cold timeout): at most 5 compression attempts per page ("compress
// tried="), and some must be declined; 393040 attempts for 32768 pages
// without the backoff. Registered
// with SMASH_USE_LZ4=1: under the default zstd-1 profile the model accepts
// these pages, the codec runs and fails, and the existing ratio-gate backoff
// applies instead — the check then cannot see the decline path.
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <string>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

static size_t resident(const char* p, size_t len) {
    std::vector<unsigned char> v(len / 4096);
    if (mincore(const_cast<char*>(p), len, v.data()) != 0) return len / 4096;
    size_t r = 0;
    for (auto c : v) r += c & 1;
    return r;
}

int main() {
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) { std::printf("SKIP: not running under libsmash\n"); return 0; }
    char path[] = "/tmp/smash_buckets_XXXXXX";
    const int fd = mkstemp(path);
    if (fd < 0) { std::printf("SKIP: mkstemp failed\n"); return 0; }
    unlink(path);
    const int saved = dup(2);
    dup2(fd, 2);

    // LBS_PART=decline (test_large_bucket_decline): random bytes, attempts
    // counted from allocation over 25 s. Registered WITHOUT
    // SMASH_COLD_TIMEOUT_SEC, which makes shouldCompress() return true at
    // once (cold_ticks_overridden) and so hides the decline path.
    if (const char* part = std::getenv("LBS_PART"); part && !std::strcmp(part, "decline")) {
        constexpr size_t kRand = 128u << 20;
        auto* r = static_cast<uint64_t*>(malloc(kRand));
        unsigned long long x = 88172645463325252ull;
        for (size_t i = 0; i < kRand / 8; ++i) { x ^= x << 13; x ^= x >> 7; x ^= x << 17; r[i] = x; }
        sleep(25);
        raise(SIGUSR2);
        usleep(200 * 1000);
        std::string lg;
        char b2[8192];
        lseek(fd, 0, SEEK_SET);
        for (ssize_t n; (n = read(fd, b2, sizeof(b2))) > 0;) lg.append(b2, static_cast<size_t>(n));
        unsigned long long t = 0, st = 0, gate_fail = 0, roi_skip = 0;
        const size_t at = lg.rfind("compress tried=");
        if (at != std::string::npos)
            std::sscanf(lg.c_str() + at, "compress tried=%llu stored=%llu gate_fail=%llu roi_skip=%llu",
                        &t, &st, &gate_fail, &roi_skip);
        const size_t rpages = kRand / 4096;
        dprintf(saved, "128 MiB of random bytes, 25 s: %llu compression attempts for %zu pages "
                "(gate_fail=%llu roi_skip=%llu)\n", t, rpages, gate_fail, roi_skip);
        free(r);
        if (roi_skip == 0) {
            dprintf(saved, "FAIL: the ROI model declined nothing (roi_skip=0): the check cannot "
                    "see the decline path (SMASH_COLD_TIMEOUT_SEC set? SMASH_USE_LZ4 unset?)\n");
            return 1;
        }
        if (t > rpages * 5) {
            dprintf(saved, "FAIL: %.1f attempts per incompressible page: a declined page is "
                    "offered again every tick\n", static_cast<double>(t) / rpages);
            return 1;
        }
        dprintf(saved, "large_bucket_stats_test (decline): all checks passed\n");
        return 0;
    }

    constexpr size_t kLen = 64u << 20;
    auto* p = static_cast<char*>(malloc(kLen));
    for (size_t i = 0; i < kLen; i += 8)
        *reinterpret_cast<uint64_t*>(p + i) = (i / 4096) * 0x9E3779B97F4A7C15ull ^ ((i & 255) * 3);
    for (int s = 0; s < 120 && resident(p, kLen) * 10 > kLen / 4096; ++s) sleep(1);
    raise(SIGUSR2);
    usleep(200 * 1000);

    std::string log;
    char buf[8192];
    lseek(fd, 0, SEEK_SET);
    for (ssize_t n; (n = read(fd, buf, sizeof(buf))) > 0;) log.append(buf, static_cast<size_t>(n));
    dup2(saved, 2);

    unsigned long long large = 0, total = 0;
    int rows = 0;
    for (size_t at = log.find("[smash bucket]"); at != std::string::npos;
         at = log.find("[smash bucket]", at + 1)) {
        int arena = 0, sc = 0;
        unsigned long long obj = 0, samples = 0, compress = 0;
        double ratio = 0;
        if (std::sscanf(log.c_str() + at, "[smash bucket] %d %d %llu %llu %lf%% %llu", &arena, &sc,
                        &obj, &samples, &ratio, &compress) != 6)
            continue;   // the header
        ++rows;
        total += compress;
        if (obj > 16384) large += compress;
    }
    std::printf("64 MiB block compressed (%zu pages resident): %d bucket rows, %llu of %llu "
                "compressions in large-block buckets\n", resident(p, kLen), rows, large, total);
    free(p);

    std::printf("large_bucket_stats_test: all checks passed\n");
    return 0;
}
