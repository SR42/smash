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
//
// Phase 2 had the same shape: escalation to PROT_NONE and the compression
// snapshot's mprotect(PROT_RW), one call per page each — 611 s of a tick on
// the same box. Once 90% of the 256 MiB has been released, "p2 mprotect=M"
// must be at most one call per 32 pages released.
//
// And the compression snapshot: it read each page through /proc/self/mem,
// twice with FixAv (snapshot and verify) — 23.6M reads, 709 s of phase 2 on
// the same box. The [smash phase time] line's "memread=T (N calls)" must
// show at most one read per 8 pages released.
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
    // PHASE3_TEST_MIB (default 256): test_phase3_batch_big uses 2048, more
    // than four nodes' compressor queues (64K pages each) hold, so pages go
    // through flushNodeQueries' local compression too.
    const char* mib_env = std::getenv("PHASE3_TEST_MIB");
    const int mib = mib_env && std::atoi(mib_env) >= 4 ? std::atoi(mib_env) : 256;
    std::vector<char*> blocks;
    for (int i = 0; i < mib / 4; ++i) {       // 4 MiB blocks
        auto* p = static_cast<char*>(malloc(4u << 20));
        std::memset(p, i + 1, 4u << 20);
        blocks.push_back(p);
    }
    const unsigned long long total = (static_cast<unsigned long long>(mib) << 20) / ps;
    const unsigned long long want = total * 3 / 4;
    const unsigned long long want_released = total * 9 / 10;
    unsigned long long armed = 0, calls = 0, p2 = 0, released = 0, mr_ms = 0, mr_calls = 0;
    // PHASE3_HOT_STRIDE_MIB: keep one page per that many MiB hot. A compressor
    // slice then routes a count that is no multiple of the queue's reservation,
    // so its last flush is short. PHASE3_MIN_P2_PER_CALL raises phase 2's check.
    const char* stride_env = std::getenv("PHASE3_HOT_STRIDE_MIB");
    const size_t stride_blocks = stride_env ? static_cast<size_t>(std::atoi(stride_env)) / 4 : 0;
    const char* p2_env = std::getenv("PHASE3_MIN_P2_PER_CALL");
    unsigned long long min_p2 = p2_env ? static_cast<unsigned long long>(std::atoll(p2_env)) : 32;
    // The raised check is about the node queues' batches: one node has none.
    if (access("/sys/devices/system/node/node1", F_OK) != 0) min_p2 = 32;
    for (int i = 0; i < 480 && (armed < want || released < want_released); ++i) {
        if (stride_blocks)
            for (size_t b = 0; b < blocks.size(); b += stride_blocks)
                blocks[b][ps * 7] = static_cast<char>(i);
        raise(SIGUSR2);
        usleep(250 * 1000);
        const std::string log = slurp(fd);
        const size_t at = log.rfind("[smash pipeline]");
        if (at == std::string::npos) continue;
        const std::string line = log.substr(at, log.find('\n', at) - at);
        const size_t a = line.find("p3 armed=");
        if (a != std::string::npos)
            std::sscanf(line.c_str() + a, "p3 armed=%llu calls=%llu p2 mprotect=%llu", &armed,
                        &calls, &p2);
        const size_t r = line.find("released=");
        if (r != std::string::npos) std::sscanf(line.c_str() + r, "released=%llu", &released);
        const size_t m = log.find("memread=", at);
        if (m != std::string::npos)
            std::sscanf(log.c_str() + m, "memread=%llu (%llu calls)", &mr_ms, &mr_calls);
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
    std::printf("phase 2: %llu mprotect calls for %llu pages released (%.1f pages per call)\n",
                p2, released, p2 ? static_cast<double>(released) / static_cast<double>(p2) : 0.0);
    if (released < want_released) {
        std::printf("FAIL: only %llu of %llu pages released within the wait\n", released,
                    want_released);
        return 1;
    }
    if (p2 * min_p2 > released) {
        std::printf("FAIL: phase 2 (escalation and compression snapshot) used an mprotect call "
                    "per %.1f pages\n", static_cast<double>(released) / static_cast<double>(p2));
        return 1;
    }
    std::printf("phase 2 snapshot: %llu /proc/self/mem reads (%llu ms) for %llu pages released\n",
                mr_calls, mr_ms, released);
    if (mr_calls * 8 > released) {
        std::printf("FAIL: the compression snapshot read %.2f pages per page released through "
                    "/proc/self/mem, one system call each\n",
                    static_cast<double>(mr_calls) / static_cast<double>(released));
        return 1;
    }
    std::printf("phase3_batch_test: all checks passed\n");
    return 0;
}
