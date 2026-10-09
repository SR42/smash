// watch_group_test.cpp - coarse watching (SMASH_WATCH_GROUP_PAGES): pages
// that go cold at slightly different times must still change protection
// together, a group at a time.
//
// Under page-protection write tracking every page is armed, escalated,
// compressed and released on its own schedule, set by when it was last
// written. Pages written a few seconds apart reach each stage in different
// ticks, so each tick's protection changes hit a scattered subset: one
// mprotect per page and a mapping split per page. On a 128-core arm64
// application that left 481k mappings, 397k of them smash's own, and every
// protection change and page fault waiting on the mmap_lock behind them.
//
// With a group of G pages idle only when all its pages are (each page takes
// the group's lowest cold count), a group moves through the stages as one.
//
// 64 MiB, every page written once in random order over ~8 s, then idle
// until 90% has left RSS. Reported: the most mappings over the block at any
// 250 ms sample, and phase 2's mprotect calls plus phase 3's arming calls.
// WATCH_TEST_MAX_MAPPINGS / WATCH_TEST_MAX_CALLS bound them (set by the
// registration); the block must leave RSS and keep its bytes.
#include <algorithm>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <random>
#include <string>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

static std::string slurp(int fd) {
    std::string out;
    char buf[8192];
    lseek(fd, 0, SEEK_SET);
    for (ssize_t n; (n = read(fd, buf, sizeof(buf))) > 0;) out.append(buf, static_cast<size_t>(n));
    return out;
}

static int mappingsOver(const char* p, size_t len) {
    FILE* f = fopen("/proc/self/maps", "r");
    if (!f) return -1;
    const auto lo = reinterpret_cast<unsigned long>(p), hi = lo + len;
    char line[512];
    int n = 0;
    while (fgets(line, sizeof(line), f)) {
        unsigned long a = 0, b = 0;
        if (std::sscanf(line, "%lx-%lx", &a, &b) == 2 && a < hi && b > lo) ++n;
    }
    fclose(f);
    return n;
}

static size_t residentPages(const char* p, size_t len, size_t ps) {
    const size_t n = len / ps;
    static unsigned char vec[1 << 15];
    if (n > sizeof(vec) || mincore(const_cast<char*>(p), len, vec) != 0) return n;
    size_t r = 0;
    for (size_t i = 0; i < n; ++i) r += vec[i] & 1;
    return r;
}

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) { std::printf("SKIP: not running under libsmash\n"); return 0; }
    char path[] = "/tmp/smash_watch_XXXXXX";
    const int fd = mkstemp(path);
    if (fd < 0) { std::printf("SKIP: mkstemp failed\n"); return 0; }
    unlink(path);
    const int saved = dup(2);
    dup2(fd, 2);

    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    constexpr size_t kLen = 64u << 20;
    const size_t pages = kLen / ps;
    auto* blk = static_cast<char*>(malloc(kLen));
    std::memset(blk, 0, kLen);
    std::vector<size_t> order(pages);
    for (size_t i = 0; i < pages; ++i) order[i] = i;
    std::shuffle(order.begin(), order.end(), std::mt19937_64(7));

    // Write each page once (its own value) in random order over ~8 s.
    int max_maps = 0;
    const size_t per_step = pages / 32;
    for (size_t k = 0; k < pages; ++k) {
        const size_t pg = order[k];
        std::memset(blk + pg * ps, static_cast<int>(pg % 251) + 1, ps / 2);
        if ((k + 1) % per_step == 0) {
            usleep(250 * 1000);
            max_maps = std::max(max_maps, mappingsOver(blk, kLen));
        }
    }
    size_t res = pages;
    for (int i = 0; i < 240 && res * 10 > pages; ++i) {
        usleep(250 * 1000);
        max_maps = std::max(max_maps, mappingsOver(blk, kLen));
        res = residentPages(blk, kLen, ps);
    }
    raise(SIGUSR2);
    usleep(200 * 1000);
    const std::string log = slurp(fd);
    dup2(saved, 2);
    unsigned long long p3_calls = 0, p2 = 0, armed = 0;
    const size_t at = log.rfind("p3 armed=");
    if (at != std::string::npos)
        std::sscanf(log.c_str() + at, "p3 armed=%llu calls=%llu p2 mprotect=%llu", &armed, &p3_calls, &p2);

    bool bytes_ok = true;
    for (size_t pg = 0; pg < pages && bytes_ok; pg += 7)
        bytes_ok = blk[pg * ps] == static_cast<char>(pg % 251 + 1) && blk[pg * ps + ps - 1] == 0;
    std::printf("64 MiB written in random page order: at most %d mappings over it; phase 2 "
                "mprotect %llu + phase 3 arming %llu calls; %zu of %zu pages resident at the end\n",
                max_maps, p2, p3_calls, res, pages);
    int failures = 0;
    const char* mm = std::getenv("WATCH_TEST_MAX_MAPPINGS");
    const char* mc = std::getenv("WATCH_TEST_MAX_CALLS");
    if (mm && max_maps > std::atoi(mm)) {
        std::printf("FAIL: %d mappings over the block, bound %s\n", max_maps, mm);
        ++failures;
    }
    if (mc && static_cast<long long>(p2 + p3_calls) > std::atoll(mc)) {
        std::printf("FAIL: %llu protection calls, bound %s\n", p2 + p3_calls, mc);
        ++failures;
    }
    if (res * 10 > pages) {
        std::printf("FAIL: %zu of %zu pages still resident\n", res, pages);
        ++failures;
    }
    if (!bytes_ok) {
        std::printf("FAIL: the block's bytes changed\n");
        ++failures;
    }
    free(blk);
    if (failures) return 1;
    std::printf("watch_group_test: all checks passed\n");
    return 0;
}
