// lazy_resolve_race_test.cpp - two threads' first calls to two different
// wrapped functions at the same time (src/linux_syscall_wrappers.cpp,
// SMASH_LAZY_RESOLVE).
//
// Each wrapper dlsym()s its real target on its first call, behind a guard
// against dlsym recursing into the wrapper. The guard was one process-wide
// flag: while thread A's dlsym ran, thread B's first call to any other wrapper
// saw the flag, skipped its own dlsym and ran with no real target -- a raw
// syscall for the I/O wrappers (no EFAULT retry, once), MAP_FAILED from mremap,
// ENOSYS from posix_spawn, nullptr from fgets. 128 threads starting together
// make the overlap routine. The guard is per thread now, and the mmap family
// falls back to the raw syscall rather than failing.
//
// helpers/slowdlsym_shim.c (preloaded after libsmash) makes every dlsym take
// 3 ms so the overlap is certain: thread A makes the process's first pread();
// 500 us into A's dlsym, threads B, C and D make the process's first mremap(),
// posix_spawnp() and fgets(). All four must succeed.
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <spawn.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

extern char** environ;

namespace {
int failures = 0;
void fail(const char* what, int err) {
    std::fprintf(stderr, "FAIL: %s (errno %d: %s)\n", what, err, std::strerror(err));
    ++failures;
}
std::atomic<bool> go{false};
std::atomic<bool> a_started{false};

void waitGo() { while (!go.load()) std::this_thread::yield(); }
void waitA() {
    while (!a_started.load()) std::this_thread::yield();
    std::this_thread::sleep_for(std::chrono::microseconds(500));
}
}  // namespace

int main() {
    // Prepared before the race, with libc calls that are not wrapped (open,
    // write, pipe) or wrapped ones used here already so their resolution is
    // not part of the race (write).
    char path[] = "/tmp/smash_lazy_resolve_XXXXXX";
    int fd = mkstemp(path);
    if (fd < 0) { fail("mkstemp", errno); return 1; }
    unlink(path);
    if (write(fd, "hello\nworld\n", 12) != 12) { fail("write", errno); return 1; }
    int fd2 = dup(fd);   // its own offset for the stream: the write left fd's at EOF
    if (fd2 < 0 || lseek(fd2, 0, SEEK_SET) != 0) { fail("dup/lseek", errno); return 1; }
    FILE* f = fdopen(fd2, "r");
    if (!f) { fail("fdopen", errno); return 1; }
    void* m = mmap(nullptr, 4096, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (m == MAP_FAILED) { fail("mmap", errno); return 1; }

    std::thread a([&] {
        waitGo();
        a_started.store(true);
        char c;
        if (pread(fd, &c, 1, 0) != 1 || c != 'h') fail("pread (thread A)", errno);
    });
    std::thread b([&] {
        waitGo(); waitA();
        errno = 0;
        void* r = mremap(m, 4096, 8192, MREMAP_MAYMOVE);
        if (r == MAP_FAILED) fail("mremap returned MAP_FAILED (thread B)", errno);
        else m = r;
    });
    std::thread c([&] {
        waitGo(); waitA();
        pid_t pid = 0;
        char arg0[] = "true";
        char* argv[] = {arg0, nullptr};
        int rc = posix_spawnp(&pid, "true", nullptr, nullptr, argv, environ);
        if (rc != 0) fail("posix_spawnp (thread C)", rc);
        else {
            int st = 0;
            waitpid(pid, &st, 0);
            if (!WIFEXITED(st) || WEXITSTATUS(st) != 0) fail("spawned child failed (thread C)", 0);
        }
    });
    std::thread d([&] {
        waitGo(); waitA();
        char line[32];
        errno = 0;
        if (!fgets(line, sizeof line, f)) fail("fgets returned nullptr (thread D)", errno);
        else if (std::strcmp(line, "hello\n") != 0) fail("fgets read the wrong line (thread D)", 0);
    });
    go.store(true);
    a.join(); b.join(); c.join(); d.join();
    fclose(f);
    close(fd);

    if (failures) {
        std::fprintf(stderr, "lazy_resolve_race: %d failure(s)\n", failures);
        return 1;
    }
    std::printf("lazy_resolve_race: first calls from 4 threads at once all resolved\n");
    return 0;
}
