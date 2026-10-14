// helper_park_test.cpp - idle compressor helpers must sleep, and parking them
// must not cost compression or a clean exit.
//
// Added because the 7 default compressor helpers waited for work with
// usleep(100) in a loop: ~6 400 wakeups a second each, all the time, even
// with one worker active (44 800/s for the process; 38 s of kernel time in a
// 180 s run on an 8-CPU VM). They now park on a futex until dispatch() wakes
// the active ones (util/park.h).
//
// Checks:
//   - a 64 MiB block is compressed out of RSS (phases still run with parked
//     helpers) and reads back intact;
//   - once compression has settled, the process's threads other than main
//     switch context fewer than 1 000 times a second (voluntary switches;
//     a polling helper alone makes ~6 400);
//   - 64 children, 8 at a time, each holding 64 MiB and exiting at a random
//     point in its first 2.5 s (before, during or after its first ticks), all
//     exit within 15 s: a helper that never gets its exit wakeup hangs
//     stop()'s join of the helpers.
//
// Not covered: stop() while a tick that uses several workers is in flight.
// stop() clears running_ before joining the coordinator, so a helper leaving
// its loop on running_ (as the polling helpers did) would leave the
// coordinator waiting for it. Helpers now exit only on helpers_exit_, set
// after the coordinator is joined. A child busy enough to keep several workers
// in its ticks was not found cheaply: compression keeps up with one worker.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <dlfcn.h>
#include <map>
#include <signal.h>
#include <string>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

namespace {

constexpr size_t kBlock = 64u << 20, kPage = 4096;

// Voluntary context switches of every thread but this one, by tid.
std::map<int, long> switches() {
    std::map<int, long> m;
    const int self = getpid();
    DIR* d = opendir("/proc/self/task");
    if (!d) return m;
    while (dirent* e = readdir(d)) {
        if (e->d_name[0] == '.' || atoi(e->d_name) == self) continue;
        const std::string path = std::string("/proc/self/task/") + e->d_name + "/status";
        FILE* f = fopen(path.c_str(), "r");
        if (!f) continue;
        char line[256];
        long v = 0;
        while (fgets(line, sizeof line, f)) sscanf(line, "voluntary_ctxt_switches: %ld", &v);
        fclose(f);
        m[atoi(e->d_name)] = v;
    }
    closedir(d);
    return m;
}

size_t resident(const unsigned char* p) {
    static unsigned char v[kBlock / kPage];
    if (mincore(const_cast<unsigned char*>(p), kBlock, v) != 0) return SIZE_MAX;
    size_t r = 0;
    for (unsigned char c : v) r += c & 1;
    return r;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "child") == 0) {
        auto* p = static_cast<unsigned char*>(malloc(64u << 20));
        for (size_t i = 0; i < (64u << 20); i += kPage) p[i] = static_cast<unsigned char>(i >> 12);
        srand(static_cast<unsigned>(getpid()));
        usleep(static_cast<useconds_t>(rand() % 2500) * 1000);
        return 0;
    }
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) { std::printf("SKIP: not running under libsmash\n"); return 0; }
    int failures = 0;

    auto* b = static_cast<unsigned char*>(malloc(kBlock));
    for (size_t i = 0; i < kBlock; ++i) b[i] = static_cast<unsigned char>((i / kPage) * 7 + (i % 61 == 0));
    size_t left = SIZE_MAX;
    for (int s = 0; s < 240 && (left = resident(b)) > 0; ++s) usleep(250 * 1000);
    if (left > 0) {
        std::printf("FAIL: %zu pages of the block still resident after 60 s\n", left);
        ++failures;
    }
    sleep(2);  // let the last compressions and the deferred sweeps finish
    const auto a = switches();
    sleep(3);
    const auto z = switches();
    long total = 0, worst = 0;
    for (const auto& [tid, v] : z) {
        const auto it = a.find(tid);
        const long d = v - (it == a.end() ? 0 : it->second);
        total += d;
        if (d > worst) worst = d;
    }
    std::printf("idle: %ld context switches/s over %zu threads (busiest %ld/s)\n", total / 3, z.size(),
                worst / 3);
    if (total / 3 >= 1000) {
        std::printf("FAIL: idle smash threads switch context %ld times a second (polling?)\n", total / 3);
        ++failures;
    }
    size_t bad = 0;
    for (size_t i = 0; i < kBlock; ++i)
        bad += b[i] != static_cast<unsigned char>((i / kPage) * 7 + (i % 61 == 0));
    if (bad) {
        std::printf("FAIL: %zu bytes of the block read back wrong\n", bad);
        ++failures;
    }
    free(b);

    int hung = 0, crashed = 0;
    for (int batch = 0; batch < 8; ++batch) {
        pid_t pids[8];
        for (pid_t& pid : pids) {
            pid = fork();
            if (pid == 0) {
                execl("/proc/self/exe", argv[0], "child", static_cast<char*>(nullptr));
                _exit(127);
            }
        }
        for (pid_t pid : pids) {
            int st = 0;
            pid_t r = 0;
            for (int w = 0; w < 1500 && (r = waitpid(pid, &st, WNOHANG)) == 0; ++w) usleep(10 * 1000);
            if (r == 0) {
                ++hung;
                kill(pid, SIGKILL);
                waitpid(pid, &st, 0);
            } else if (!WIFEXITED(st) || WEXITSTATUS(st) != 0) {
                ++crashed;
            }
        }
    }
    if (hung || crashed) {
        std::printf("FAIL: of 64 children, %d hung at exit and %d failed\n", hung, crashed);
        ++failures;
    }
    if (failures) return 1;
    std::printf("helper_park_test: all checks passed\n");
    return 0;
}
