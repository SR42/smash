// phase3_batch_test.cpp - Phase 3 must arm contiguous memory with few
// mprotect calls.
//
// Under page-protection write tracking (SMASH_SOFTDIRTY=0) every compressor
// tick makes live pages PROT_READ. That was one mprotect per page: the loop
// grouped pages into runs of 16 but protected each page on its own. On a
// 128-core, 4-node arm64 box with 30M watched pages and 500k mappings that
// was ~30M mprotect calls per pass, each a TLB shootdown to every core, and
// compressor ticks took 68-142 s instead of ~1 s — so compression, which
// counts idle time in ticks, crawled (0.8 GiB in 15 minutes).
//
// 256 MiB of contiguous memory held idle, stderr pointed at a file, the
// SIGUSR2 stats dump prompted until Phase 3 has armed most of it; the
// [smash pipeline] line's "p3 armed=A calls=C" must show at least 64 pages
// per mprotect call.
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <string>
#include <unistd.h>
#include <vector>

static std::string slurp(int fd) {
    std::string out;
    char buf[8192];
    lseek(fd, 0, SEEK_SET);
    for (ssize_t n; (n = read(fd, buf, sizeof(buf))) > 0;) out.append(buf, static_cast<size_t>(n));
    return out;
}

int main() {
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) { std::printf("SKIP: not running under libsmash\n"); return 0; }
    char path[] = "/tmp/smash_p3_XXXXXX";
    const int fd = mkstemp(path);
    if (fd < 0) { std::printf("SKIP: mkstemp failed\n"); return 0; }
    unlink(path);
    const int saved = dup(2);
    dup2(fd, 2);

    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    std::vector<char*> blocks;
    for (int i = 0; i < 64; ++i) {            // 64 x 4 MiB
        auto* p = static_cast<char*>(malloc(4u << 20));
        std::memset(p, i + 1, 4u << 20);
        blocks.push_back(p);
    }
    const unsigned long long want = (256ull << 20) / ps * 3 / 4;
    unsigned long long armed = 0, calls = 0;
    for (int i = 0; i < 120 && armed < want; ++i) {
        raise(SIGUSR2);
        usleep(250 * 1000);
        const std::string log = slurp(fd);
        const size_t at = log.rfind("p3 armed=");
        if (at != std::string::npos)
            std::sscanf(log.c_str() + at, "p3 armed=%llu calls=%llu", &armed, &calls);
    }
    dup2(saved, 2);
    for (char* p : blocks) free(p);

    std::printf("phase 3: %llu pages armed in %llu mprotect calls (%.1f pages per call)\n", armed,
                calls, calls ? static_cast<double>(armed) / static_cast<double>(calls) : 0.0);
    if (armed < want) {
        std::printf("FAIL: phase 3 armed only %llu of the %llu pages expected "
                    "(SMASH_STATS_SIGNAL=1 and SMASH_SOFTDIRTY=0 set?)\n", armed, want);
        return 1;
    }
    if (armed < calls * 64) {
        std::printf("FAIL: phase 3 used an mprotect call per %.1f pages of contiguous idle memory; "
                    "each is a TLB shootdown to every core\n", static_cast<double>(armed) / calls);
        return 1;
    }
    std::printf("phase3_batch_test: all checks passed\n");
    return 0;
}
