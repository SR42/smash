// decommit_park_test.cpp - idle decommit threads must sleep, and parking them
// must not delay giving freed memory back or a clean exit.
//
// Added because each decommit thread (started by the first free of a large
// block on its queue: the shared arena's, and each NUMA node's) waited for work
// with usleep(100) in a loop: ~6 300 wakeups a second per thread, forever,
// in a process that had freed one block. They now park on a futex until a free
// queues work, a sub-arena goes over its dirty cap, or a periodic duty is due.
//
// Checks:
//   - freeing 64 MiB (8 blocks of 8 MiB, written) gives at least 48 MiB of RSS
//     back within 2 s: a parked thread must be woken by the free;
//   - with blocks freed on the shared queue and, under malloc_numa, on every
//     node's queue, the process's threads other than main then switch context
//     fewer than 1 000 times a second (voluntary switches; a polling decommit
//     thread alone makes ~6 300);
//   - 16 children, 4 at a time, each freeing 32 MiB and then ending main with
//     pthread_exit (which makes smash stop and join its threads), all exit
//     within 15 s: a parked thread must be woken to be stopped.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <dlfcn.h>
#include <map>
#include <pthread.h>
#include <signal.h>
#include <string>
#include <sys/wait.h>
#include <unistd.h>

namespace {

constexpr size_t kBlock = 8u << 20;

using MallocNuma = void* (*)(uint32_t, size_t);

long rssMiB() {
    FILE* f = std::fopen("/proc/self/statm", "r");
    if (!f) return -1;
    unsigned long size = 0, res = 0;
    const int got = std::fscanf(f, "%lu %lu", &size, &res);
    std::fclose(f);
    return got == 2 ? static_cast<long>(res * static_cast<unsigned long>(sysconf(_SC_PAGESIZE)) >> 20) : -1;
}

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

void churn(size_t blocks) {
    void* p[64];
    for (size_t i = 0; i < blocks; ++i) {
        p[i] = malloc(kBlock);
        std::memset(p[i], 1, kBlock);
    }
    for (size_t i = 0; i < blocks; ++i) free(p[i]);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "child") == 0) {
        // pthread_exit from main: the last thread's exit is what makes smash
        // stop and join its own threads (util/exit_guard.h); exit() does not.
        churn(4);
        pthread_exit(nullptr);
    }
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) { std::fprintf(stderr, "FAIL: not running under libsmash\n"); return 1; }
    int failures = 0;

    // 1. A free must reach a parked thread. Warm the queue's thread first so
    //    it has started and gone idle.
    churn(1);
    usleep(300 * 1000);
    void* p[8];
    for (void*& b : p) {
        b = malloc(kBlock);
        std::memset(b, 1, kBlock);
    }
    const long before = rssMiB();
    for (void* b : p) free(b);
    long after = before;
    for (int i = 0; i < 40 && before - (after = rssMiB()) < 48; ++i) usleep(50 * 1000);
    std::printf("free: rss %ld -> %ld MiB\n", before, after);
    if (before - after < 48) {
        std::printf("FAIL: freeing 64 MiB gave back %ld MiB within 2 s\n", before - after);
        ++failures;
    }

    // 2. Idle. Free a block on every node's queue too, when there are nodes.
    auto malloc_numa = reinterpret_cast<MallocNuma>(dlsym(RTLD_DEFAULT, "malloc_numa"));
    int nodes = 0;
    if (malloc_numa) {
        for (uint32_t n = 0; n < 8; ++n) {
            const std::string node = "/sys/devices/system/node/node" + std::to_string(n);
            if (access(node.c_str(), F_OK) != 0) continue;
            void* b = malloc_numa(n, kBlock);
            if (!b) continue;
            std::memset(b, 1, kBlock);
            free(b);
            ++nodes;
        }
    }
    sleep(2);
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
    std::printf("idle: %ld context switches/s over %zu threads (busiest %ld/s), %d node queues used\n",
                total / 3, z.size(), worst / 3, nodes);
    if (total / 3 >= 1000) {
        std::printf("FAIL: idle smash threads switch context %ld times a second (polling?)\n", total / 3);
        ++failures;
    }

    // 3. Exit with freshly queued frees.
    int hung = 0, crashed = 0;
    for (int batch = 0; batch < 4; ++batch) {
        pid_t pids[4];
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
        std::printf("FAIL: of 16 children, %d hung at exit and %d failed\n", hung, crashed);
        ++failures;
    }
    if (failures) return 1;
    std::printf("decommit_park_test: all checks passed\n");
    return 0;
}
