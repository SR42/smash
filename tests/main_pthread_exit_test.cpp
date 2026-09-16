// main_pthread_exit_test.cpp - the process must still end when the
// application's last thread exits, after main retired with pthread_exit().
//
// glibc ends a process when its thread count reaches zero: the last thread to
// exit calls exit(0), running atexit handlers. pthread_exit() from main does
// not call exit() — it retires main and waits for the remaining threads. Every
// smash background thread that never exits keeps that count above zero, so the
// process hung forever: measured as a zombie thread-group leader beside 16
// sleeping smash threads, needing SIGKILL, with the application's own atexit
// handler never running. SMASH_NO_COMPRESSOR=1 made the same binary exit 0,
// which is what isolated it to smash's threads.
//
// The body runs in a forked child so the hang is a test failure rather than a
// hung test binary, and so the child's exit status and output can be checked.
// Both markers matter: ATEXIT_RAN is the sharper one, because a "fix" that
// ends the process without glibc's normal last-thread path would still exit 0
// while silently skipping the application's exit handlers.
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <pthread.h>
#include <sys/wait.h>
#include <unistd.h>

namespace {

constexpr int kTimeoutSec = 30;

void atexitMarker() {
    std::printf("ATEXIT_RAN\n");
    std::fflush(stdout);
}

void* worker(void*) {
    usleep(200000);                    // outlive main, so main is not the last
    void* p = malloc(4096);            // touch the allocator from this thread
    if (p) { std::memset(p, 0xab, 4096); free(p); }
    std::printf("WORKER_DONE\n");
    std::fflush(stdout);
    return nullptr;
}

int runChild() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    std::atexit(atexitMarker);
    pthread_t t;
    if (pthread_create(&t, nullptr, worker, nullptr) != 0) {
        std::printf("CREATE_FAILED\n");
        return 1;
    }
    std::printf("MAIN_RETIRED\n");
    pthread_exit(nullptr);             // the process must end when worker does
}

bool fileContains(const char* path, const char* needle) {
    int fd = open(path, O_RDONLY);
    if (fd < 0) return false;
    char buf[4096];
    ssize_t n = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    if (n <= 0) return false;
    buf[n] = '\0';
    return std::strstr(buf, needle) != nullptr;
}

}  // namespace

int main(int argc, char** argv) {
    setvbuf(stdout, nullptr, _IONBF, 0);
    if (argc > 1 && std::strcmp(argv[1], "child") == 0) return runChild();

    char out_path[256];
    std::snprintf(out_path, sizeof out_path, "main_pthread_exit_%d.out", (int)getpid());

    pid_t pid = fork();
    if (pid < 0) { std::fprintf(stderr, "FAIL: fork failed\n"); return 1; }
    if (pid == 0) {
        int fd = open(out_path, O_CREAT | O_TRUNC | O_WRONLY, 0644);
        if (fd < 0) _exit(127);
        dup2(fd, 1); dup2(fd, 2); close(fd);
        execl("/proc/self/exe", "main_pthread_exit_test", "child", (char*)nullptr);
        _exit(127);
    }

    int status = 0, waited_ms = 0;
    pid_t r;
    while ((r = waitpid(pid, &status, WNOHANG)) == 0 && waited_ms < kTimeoutSec * 1000) {
        usleep(20000);
        waited_ms += 20;
    }

    if (r == 0) {
        kill(pid, SIGKILL);
        waitpid(pid, &status, 0);
        std::fprintf(stderr,
            "FAIL: child still alive after %d s. main retired with pthread_exit() "
            "and the process never ended — a smash background thread is keeping "
            "glibc's thread count above zero (see src/util/exit_guard.h).\n",
            kTimeoutSec);
        std::remove(out_path);
        return 1;
    }
    if (!WIFEXITED(status)) {
        std::fprintf(stderr, "FAIL: child did not exit normally (signal %d)\n",
                     WIFSIGNALED(status) ? WTERMSIG(status) : -1);
        std::remove(out_path);
        return 1;
    }
    if (WEXITSTATUS(status) != 0) {
        std::fprintf(stderr, "FAIL: child exit status %d, expected 0 (glibc's "
                     "last-thread path exits 0)\n", WEXITSTATUS(status));
        std::remove(out_path);
        return 1;
    }

    const bool worker_done = fileContains(out_path, "WORKER_DONE");
    const bool atexit_ran = fileContains(out_path, "ATEXIT_RAN");
    std::remove(out_path);

    if (!worker_done) {
        std::fprintf(stderr, "FAIL: worker thread never finished\n");
        return 1;
    }
    if (!atexit_ran) {
        std::fprintf(stderr,
            "FAIL: the process exited but the application's atexit handler never "
            "ran. The last thread must end the process through glibc's own "
            "exit(0) path, not by smash short-circuiting it.\n");
        return 1;
    }

    std::printf("PASS: process ended %d ms after main retired via pthread_exit; "
                "worker finished and atexit handlers ran\n", waited_ms);
    return 0;
}
