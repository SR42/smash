// exit_drain_test.cpp - what exit() costs with a large compressed set
// (src/compress/compressor_thread.h CompressorThread::stop).
//
// stop() decompressed every compressed page back into place, one thread,
// before uninstalling the fault handler: a pwrite through /proc/self/mem and
// an mprotect per page, minutes of exit for tens of millions of pages, with
// nothing printed. The handler now stays installed and the pages stay
// compressed: an access after shutdown faults and is restored on the
// accessing thread, as during the run.
//
//   child: 1.5 GiB of compressible data, waits until it is compressed,
//          registers an atexit handler that reads 1 MiB of it and checks it,
//          prints "exiting", exits
//   parent: the time from "exiting" to the child's end must be under 1 s
//          (the drain took several), and the child must exit 0 (the atexit
//          read, through the handler, saw the right bytes)
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

namespace {
unsigned char* g_block = nullptr;
const size_t kBytes = size_t{1536} << 20;

void atExitRead() {
    // 1 MiB in the middle, through the fault handler.
    size_t bad = 0;
    for (size_t off = kBytes / 2; off < kBytes / 2 + (1u << 20); off += 64)
        bad += g_block[off] != static_cast<unsigned char>((off >> 12) * 7);
    if (bad) { static const char m[] = "FAIL: atexit read wrong bytes\n"; (void)!write(2, m, sizeof m - 1); _exit(7); }
}

int child(int out_fd) {
    using info_fn = int (*)(const void*, int*, int*, int*);
    auto page_info = reinterpret_cast<info_fn>(dlsym(RTLD_DEFAULT, "smash_page_info"));
    if (!page_info) return 2;
    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    // A fork child's compressor restarts on its first thread creation.
    std::thread([] {}).join();
    g_block = static_cast<unsigned char*>(std::malloc(kBytes));
    if (!g_block) return 3;
    for (size_t off = 0; off < kBytes; off += 64) g_block[off] = static_cast<unsigned char>((off >> 12) * 7);
    size_t compressed = 0;
    for (int waited = 0; waited < 120000 && compressed < (kBytes / ps) * 9 / 10; waited += 500) {
        usleep(500000);
        compressed = 0;
        for (size_t off = 0; off < kBytes; off += ps) {
            int st = 0, c = 0, r = 0;
            if (page_info(g_block + off, &st, &c, &r) == 0 && st == 2) ++compressed;
        }
    }
    std::atexit(atExitRead);
    char msg[64];
    int n = std::snprintf(msg, sizeof msg, "exiting %zu\n", compressed);
    (void)!write(out_fd, msg, static_cast<size_t>(n));
    std::exit(0);
}
}  // namespace

int main() {
    int pipefd[2];
    if (pipe(pipefd) != 0) return 1;
    std::fflush(stdout);
    pid_t pid = fork();
    if (pid == 0) { close(pipefd[0]); _exit(child(pipefd[1])); }
    close(pipefd[1]);
    char buf[64] = {};
    ssize_t n = read(pipefd[0], buf, sizeof buf - 1);
    const auto t0 = std::chrono::steady_clock::now();
    int st = 0;
    waitpid(pid, &st, 0);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    size_t compressed = 0;
    if (n > 0) std::sscanf(buf, "exiting %zu", &compressed);
    std::printf("exit_drain: %zu pages compressed; exit() took %.0f ms; status %d\n", compressed, ms,
                WIFEXITED(st) ? WEXITSTATUS(st) : -WTERMSIG(st));
    int failures = 0;
    if (n <= 0 || compressed < 100000) { std::fprintf(stderr, "FAIL: the child did not compress its block\n"); ++failures; }
    if (!WIFEXITED(st) || WEXITSTATUS(st) != 0) { std::fprintf(stderr, "FAIL: child status\n"); ++failures; }
    if (ms > 1000) { std::fprintf(stderr, "FAIL: exit() took %.0f ms (draining the compressed set)\n", ms); ++failures; }
    if (failures) return 1;
    std::printf("exit_drain: all tests passed\n");
    return 0;
}
