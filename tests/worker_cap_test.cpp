// worker_cap_test.cpp - idle application threads must not pin the compressor
// to one worker.
//
// The CPU-pressure cap was nproc - (every thread in the process) - (other
// load): a process with nproc threads or more always got one worker, whether
// its threads were busy or asleep. On a 128-thread, 128-core arm64
// application that left phases 1-4 of a 145 GB heap on one thread, and once
// the mmap_lock contention was reduced the compressor's time was mostly
// spent waiting for a CPU (phase 2: 2527 s runq against 236 s user).
//
// 4 x nproc threads that only sleep, next to 1 GiB written once and held
// idle; the [smash pipeline] line's cap= (the CPU-pressure cap the
// compressor last computed) is read after 8 s and must be at least 2. The
// first version sampled workers= instead, which also depends on how much
// work the Little's-law estimate sees: it failed 1 in 3 -j4 suite runs
// under SMASH_SOFTDIRTY=0 with the fixed cap. Old cap: 1 (measured on 8
// CPUs, 32 sleeping threads: workers=1 in every sample).
#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <string>
#include <thread>
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
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) { std::fprintf(stderr, "FAIL: not running under libsmash\n"); return 1; }
    char path[] = "/tmp/smash_workers_XXXXXX";
    const int fd = mkstemp(path);
    if (fd < 0) { std::printf("SKIP: mkstemp failed\n"); return 77; }
    unlink(path);
    const int saved = dup(2);
    dup2(fd, 2);

    const long nproc = sysconf(_SC_NPROCESSORS_ONLN);
    if (nproc < 4) {
        // Below 4 CPUs the floor is 1 by design, and any load gives cap 1
        // with or without the thread count: nothing to tell apart (claudepc,
        // 2 CPUs, under the suite's load: cap 1).
        dup2(saved, 2);
        std::printf("SKIP: %ld CPUs; the cap's floor is 1 below 4\n", nproc);
        return 77;
    }
    std::atomic<bool> stop{false};
    std::vector<std::thread> idle;
    for (long i = 0; i < 4 * nproc; ++i)
        idle.emplace_back([&] { while (!stop.load(std::memory_order_relaxed)) usleep(100 * 1000); });

    std::vector<char*> blocks;
    for (int i = 0; i < 256; ++i) {   // 256 x 4 MiB
        auto* p = static_cast<char*>(malloc(4u << 20));
        for (size_t j = 0; j < (4u << 20); ++j) p[j] = static_cast<char>((j * 131u + static_cast<unsigned>(i)) >> 3);
        blocks.push_back(p);
    }
    unsigned long long cap = 0;
    bool seen = false;
    for (int s = 0; s < 30 && !seen; ++s) {
        sleep(1);
        raise(SIGUSR2);
        usleep(100 * 1000);
        const std::string log = slurp(fd);
        // The cap is first computed at the end of tick 1; read it from tick 4.
        for (size_t at = log.find("[smash pipeline] ticks="); at != std::string::npos;
             at = log.find("[smash pipeline] ticks=", at + 1)) {
            const std::string line = log.substr(at, log.find('\n', at) - at);
            unsigned long long ticks = 0, c = 0;
            const size_t k = line.find(" cap=");
            if (std::sscanf(line.c_str(), "[smash pipeline] ticks=%llu", &ticks) == 1 && ticks >= 4 &&
                k != std::string::npos && std::sscanf(line.c_str() + k, " cap=%llu", &c) == 1) {
                cap = c;
                seen = true;
            }
        }
    }
    stop = true;
    for (auto& t : idle) t.join();
    for (char* p : blocks) free(p);
    dup2(saved, 2);

    std::printf("%ld sleeping threads on %ld CPUs: compressor worker cap %llu\n",
                4 * nproc, nproc, cap);
    if (!seen) {
        std::printf("FAIL: no cap= on the [smash pipeline] line from tick 4 on "
                    "(SMASH_STATS_SIGNAL=1 set?)\n");
        return 1;
    }
    if (cap != 0 && cap < 2) {
        std::printf("FAIL: sleeping threads held the compressor's cap at %llu: the "
                    "CPU-pressure cap counts threads, not load\n", cap);
        return 1;
    }
    std::printf("worker_cap_test: all checks passed\n");
    return 0;
}
