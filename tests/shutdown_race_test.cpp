// shutdown_race_test.cpp - every short-lived process under libsmash must exit.
//
// CompressorThread::stop() joins the NUMA node-group workers at process exit.
// stopNodeThreads() stores numa_exit_, bumps numa_work_gen_ and wakes, in that
// order; a worker whose first instructions ran only after all three had
// happened sampled the already-bumped generation into `last`, found
// gen == last, and parked on a futex that nothing would ever bump again. The
// pthread_join in stopNodeThreads() then waited on it forever, under
// __cxa_finalize, and the process never exited.
//
// The window needs startup and teardown to nearly coincide, so it shows up in
// short-lived programs: measured 7 hangs in 300 runs (2.3%) of a 0.02 s test
// before the fix, 0 in 300 after. It first surfaced as an intermittent
// test_arena_routing timeout, which is a poor guard because it only catches
// the race by accident; this test provokes it directly.
//
// Node groups are only created on hosts with >= 2 memory nodes, so on a
// single-node host (CI runners) this still exercises the ordinary compressor
// shutdown path and simply cannot hit this particular race.
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/wait.h>
#include <unistd.h>

namespace {

constexpr int kRuns = 200;
constexpr int kChildTimeoutMs = 10000;

// The child only has to start the compressor (so the node groups exist) and
// then exit immediately — that is the whole race.
int runChild() {
    for (int i = 0; i < 64; ++i) {
        void* p = malloc(4096);
        if (p) { std::memset(p, 1, 64); free(p); }
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    setvbuf(stdout, nullptr, _IONBF, 0);
    if (argc > 1 && std::strcmp(argv[1], "child") == 0) return runChild();

    for (int i = 1; i <= kRuns; ++i) {
        pid_t pid = fork();
        if (pid < 0) { std::fprintf(stderr, "FAIL: fork failed at run %d\n", i); return 1; }
        if (pid == 0) {
            execl("/proc/self/exe", "shutdown_race_test", "child", (char*)nullptr);
            _exit(127);
        }

        int status = 0, waited_ms = 0;
        pid_t r;
        while ((r = waitpid(pid, &status, WNOHANG)) == 0 && waited_ms < kChildTimeoutMs) {
            usleep(2000);
            waited_ms += 2;
        }

        if (r == 0) {
            // Capture what the stuck process looks like before killing it: the
            // signature is main in pthread_join under __cxa_finalize with a
            // node worker asleep in futexWait.
            std::fprintf(stderr, "FAIL: child %d of %d did not exit within %d ms.\n",
                         i, kRuns, kChildTimeoutMs);
            char path[64];
            std::snprintf(path, sizeof path, "/proc/%d/task", (int)pid);
            std::fprintf(stderr, "  surviving threads:\n");
            char cmd[256];
            std::snprintf(cmd, sizeof cmd,
                          "for t in $(ls %s 2>/dev/null); do "
                          "echo \"    tid=$t wchan=$(cat %s/$t/wchan 2>/dev/null)\"; done 1>&2",
                          path, path);
            (void)!system(cmd);
            kill(pid, SIGKILL);
            waitpid(pid, &status, 0);
            std::fprintf(stderr,
                "  A node worker parked on a futex nothing woke, and "
                "stopNodeThreads()'s pthread_join waited on it forever "
                "(src/compress/compressor_thread.h).\n");
            return 1;
        }
        if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
            std::fprintf(stderr, "FAIL: child %d exited abnormally (status %d)\n", i, status);
            return 1;
        }
    }

    std::printf("PASS: %d short-lived processes all exited cleanly\n", kRuns);
    return 0;
}
