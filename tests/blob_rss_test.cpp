// blob_rss_test.cpp - the [smash stats] line's rss= and blob= must report the
// process's resident memory and the memory its compressed blobs hold.
//
// Added for a 128-core arm64 application whose dumps gave the number of
// compressed pages but neither RSS nor the cost of the blobs, so the actual
// saving could not be read from the log.
//
// 64 MiB of pages that each hold 1 KiB of random bytes and 3 KiB of zeros
// (LZ4 or zstd: ~1 KiB per page, so ~16 MiB of blobs) is compressed, then
// freed. Checks:
//   - rss= matches /proc/self/statm read next to the dump (within 16 MiB),
//     and falls by at least 32 MiB once the block is compressed;
//   - blob= live grows by 8..32 MiB with the block compressed, and
//     live <= held;
//   - after free(), live returns to within 4 MiB of where it started.
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

namespace {

constexpr size_t kLen = 64u << 20;
int g_log = -1;

size_t resident(const char* p, size_t len) {
    std::vector<unsigned char> v(len / 4096);
    if (mincore(const_cast<char*>(p), len, v.data()) != 0) return len / 4096;
    size_t r = 0;
    for (auto c : v) r += c & 1;
    return r;
}

size_t statmRssMiB() {
    FILE* f = std::fopen("/proc/self/statm", "r");
    if (!f) return 0;
    unsigned long size = 0, res = 0;
    const int got = std::fscanf(f, "%lu %lu", &size, &res);
    std::fclose(f);
    return got == 2 ? res * static_cast<size_t>(sysconf(_SC_PAGESIZE)) >> 20 : 0;
}

struct Dump { bool ok = false; size_t rss = 0, live = 0, held = 0, regions = 0, statm = 0; };

// Raises SIGUSR2 and parses the newest [smash stats] line.
Dump dump() {
    Dump d;
    raise(SIGUSR2);
    d.statm = statmRssMiB();
    usleep(100 * 1000);
    std::string log;
    char buf[8192];
    lseek(g_log, 0, SEEK_SET);
    for (ssize_t n; (n = read(g_log, buf, sizeof(buf))) > 0;) log.append(buf, static_cast<size_t>(n));
    const size_t at = log.rfind("[smash stats]");
    if (at == std::string::npos) return d;
    const size_t r = log.find(" rss=", at);
    if (r == std::string::npos || r > log.find('\n', at)) return d;
    d.ok = std::sscanf(log.c_str() + r, " rss=%zuMiB  blob=%zu/%zuMiB(%zu regions)", &d.rss, &d.live,
                       &d.held, &d.regions) == 4;
    return d;
}

void show(const char* when, const Dump& d) {
    std::printf("%-10s rss=%zuMiB (statm %zuMiB)  blob=%zu/%zuMiB(%zu regions)\n", when, d.rss, d.statm,
                d.live, d.held, d.regions);
}

}  // namespace

int main() {
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) { std::fprintf(stderr, "FAIL: not running under libsmash\n"); return 1; }
    char path[] = "/tmp/smash_blobrss_XXXXXX";
    g_log = mkstemp(path);
    if (g_log < 0) { std::printf("SKIP: mkstemp failed\n"); return 77; }
    unlink(path);
    const int saved = dup(2);
    dup2(g_log, 2);

    int failures = 0;
    auto fail = [&](const char* what) { std::printf("FAIL: %s\n", what); ++failures; };

    const Dump base = dump();
    auto* a = static_cast<unsigned char*>(malloc(kLen));
    uint64_t x = 0x9E3779B97F4A7C15ull;
    for (size_t pg = 0; pg < kLen; pg += 4096) {
        for (size_t i = 0; i < 1024; i += 8) {
            x ^= x << 13; x ^= x >> 7; x ^= x << 17;
            std::memcpy(a + pg + i, &x, 8);
        }
        std::memset(a + pg + 1024, 0, 4096 - 1024);
    }
    const Dump hot = dump();
    for (int s = 0; s < 240 && resident(reinterpret_cast<char*>(a), kLen) > 0; ++s) usleep(250 * 1000);
    const size_t left = resident(reinterpret_cast<char*>(a), kLen);
    const Dump cold = dump();
    free(a);
    Dump freed = dump();
    for (int s = 0; s < 40 && freed.ok && freed.live > base.live + 4; ++s) {
        usleep(250 * 1000);
        freed = dump();
    }
    dup2(saved, 2);

    show("start", base);
    show("hot", hot);
    show("compressed", cold);
    show("freed", freed);
    if (!base.ok || !hot.ok || !cold.ok || !freed.ok) {
        fail("no rss=/blob= in the [smash stats] line");
        return 1;
    }
    if (left > 0) {
        std::printf("FAIL: %zu of %zu pages still resident after 60 s\n", left, kLen / 4096);
        return 1;
    }
    for (const Dump* d : {&base, &hot, &cold, static_cast<const Dump*>(&freed)}) {
        const size_t lo = d->statm > 16 ? d->statm - 16 : 0;
        if (d->rss < lo || d->rss > d->statm + 16 || d->rss == 0) fail("rss= differs from /proc/self/statm");
        if (d->live > d->held) fail("blob live > held");
    }
    if (hot.rss < cold.rss + 32) fail("rss= did not fall by 32 MiB once the block was compressed");
    const size_t grew = cold.live - base.live;
    if (cold.live < base.live || grew < 8 || grew > 32) fail("blob live did not grow by 8..32 MiB for 64 MiB at ~4:1");
    if (cold.regions < 1) fail("no blob regions with blobs stored");
    if (freed.live > base.live + 4) fail("blob live did not return to its start after free()");
    if (failures) return 1;
    std::printf("blob_rss_test: all checks passed\n");
    return 0;
}
