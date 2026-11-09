// stdio_chk_test.cpp - fortified buffered reads (__fgets_chk, __fread_chk)
// must recover when the kernel's write lands on memory the compressor owns,
// as fgets and fread do.
//
// Added because _FORTIFY_SOURCE turns fgets/fread into __fgets_chk/__fread_chk
// whenever the compiler knows the destination's size, and smash wrapped only
// the plain names. glibc's own read is internal to libc, so the kernel's
// EFAULT on a watched or compressed page reached the application as a failed
// stream: fgets returned NULL with errno 14 on every one of three popen reads
// of a child's delayed output, while the same program built without fortify
// read all three. Found as test_cold_arena_env's children "not reporting" in
// a UBSan build (instrumentation made the size known).
//
// Stream buffers glibc allocates itself no longer come from smash
// (test_stdio_buffer), so the cases here are the two where the kernel still
// writes smash memory: a buffer the program hands to setvbuf, and a large
// fread, which glibc reads straight into the caller's buffer.
//
// Full mode, cold timeout 1 s. 4 KiB and 12 KiB blocks are written, half of
// each freed, and left 3 s to go cold, so the buffers below reuse cold slots.
// Checks:
//   - two popen reads of a child that sleeps 6 s and prints a line, through
//     __fgets_chk, on a stream given a malloc'd 4 KiB buffer, return the line
//     (warming before the call is not enough: the buffer goes cold again
//     while the read blocks, so this needs the retry);
//   - two __fread_chk of 12000 bytes from a file (more than the stream's
//     buffer, so glibc reads straight into the caller's) into the start of a
//     2 MiB block, written and left until those pages are out of RAM, return
//     all 12000 (glibc's own returned 0, errno 14);
//   - the fortify checks still abort: __fgets_chk with a line longer than
//     the buffer it was told about, and __fread_chk asked for more than it,
//     kill a child with SIGABRT, as glibc's do (the buffer is really 64
//     bytes, so nothing else would).
#define _FORTIFY_SOURCE 2
#include <cerrno>
#include <cstdint>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

namespace {

int failures = 0;
constexpr size_t kBig = 2u << 20;

void fail(const char* what, int round, int err) {
    std::printf("FAIL: %s, round %d (errno %d)\n", what, round, err);
    ++failures;
}

}  // namespace

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) { std::fprintf(stderr, "FAIL: not running under libsmash\n"); return 1; }
#if !(defined(__GLIBC__) && defined(__USE_FORTIFY_LEVEL) && __USE_FORTIFY_LEVEL > 0)
    std::printf("SKIP: _FORTIFY_SOURCE declarations unavailable\n");
    return 77;
#else
    static void* junk4[2048];
    static void* junk12[1024];
    for (void*& j : junk4) {
        j = malloc(4096);
        std::memset(j, 1, 4096);
    }
    for (void*& j : junk12) {
        j = malloc(12288);
        std::memset(j, 1, 12288);
    }
    for (size_t i = 0; i < 2048; i += 2) free(junk4[i]);
    for (size_t i = 0; i < 1024; i += 2) free(junk12[i]);
    sleep(3);

    for (int round = 0; round < 2; ++round) {
        FILE* p = popen("sleep 6; echo hello-from-child", "r");
        if (!p) { fail("popen", round, errno); continue; }
        char* sbuf = static_cast<char*>(malloc(4096));
        setvbuf(p, sbuf, _IOFBF, 4096);
        char line[128];
        errno = 0;
        const char* r = __fgets_chk(line, sizeof line, sizeof line, p);
        const int e = errno;
        std::printf("__fgets_chk round %d: %s (errno %d)\n", round, r ? "line" : "NULL", e);
        if (!r || !std::strstr(line, "hello-from-child")) fail("__fgets_chk lost the child's line", round, e);
        pclose(p);
        free(sbuf);
    }
    for (int round = 0; round < 2; ++round) {
        // Written, then left until it is out of RAM: the kernel's write must
        // find it compressed.
        // A large block: its pages are compressed reliably, unlike a slab
        // slot's (a 12 KiB one never left RAM in 30 s).
        char* buf = static_cast<char*>(malloc(kBig));
        for (size_t i = 0; i < kBig; ++i) buf[i] = static_cast<char>('a' + i % 7);
        bool out = false;
        for (int w = 0; w < 160 && !out; ++w) {
            unsigned char v[3] = {1, 1, 1};
            out = mincore(buf, 12288, v) == 0 && !(v[0] & 1) && !(v[1] & 1) && !(v[2] & 1);
            if (!out) usleep(250 * 1000);
        }
        if (!out) std::printf("note: __fread_chk round %d's buffer did not leave RAM\n", round);
        // A regular file: glibc reads a request larger than the stream's
        // buffer straight into the caller's (on a pipe it only ever filled
        // its own buffer, so the caller's pages were written by a memcpy,
        // which smash's fault handler covers anyway).
        char path[] = "/tmp/smash_stdiochk_XXXXXX";
        const int fd = mkstemp(path);
        if (fd < 0) { fail("mkstemp", round, errno); continue; }
        static char xs[12000];
        std::memset(xs, 'x', sizeof xs);
        if (write(fd, xs, sizeof xs) != static_cast<ssize_t>(sizeof xs)) fail("write", round, errno);
        close(fd);
        FILE* p = fopen(path, "r");
        unlink(path);
        if (!p) { fail("fopen", round, errno); continue; }
        errno = 0;
        const size_t n = __fread_chk(buf, kBig, 1, 12000, p);
        const int e = errno;
        std::printf("__fread_chk round %d: %zu of 12000 bytes (errno %d)\n", round, n, e);
        if (n != 12000 || buf[0] != 'x' || buf[11999] != 'x') fail("__fread_chk came back short", round, e);
        fclose(p);
        free(buf);
    }

    // The overflow checks must survive the wrappers. The buffer really has 64
    // bytes, and the call is told 8: only glibc's check can stop it (on the
    // stack, the canary would abort a missing check too, and hide it).
    auto overflowAborts = [](bool use_fread) {
        const pid_t pid = fork();
        if (pid == 0) {
            FILE* p = popen("echo this-line-is-longer-than-eight-bytes", "r");
            char* big = static_cast<char*>(malloc(64));
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-overflow"
            if (use_fread)
                (void)!__fread_chk(big, 8, 1, 32, p);     // 32 bytes into "8"
            else
                (void)!__fgets_chk(big, 8, 64, p);        // up to 63 into "8"
#pragma GCC diagnostic pop
            _exit(0);
        }
        int st = 0;
        waitpid(pid, &st, 0);
        std::printf("overflowing %s: %s %d\n", use_fread ? "__fread_chk" : "__fgets_chk",
                    WIFSIGNALED(st) ? "signal" : "exit", WIFSIGNALED(st) ? WTERMSIG(st) : WEXITSTATUS(st));
        return WIFSIGNALED(st) && WTERMSIG(st) == SIGABRT;
    };
    if (!overflowAborts(false)) {
        std::printf("FAIL: an overflowing __fgets_chk did not abort\n");
        ++failures;
    }
    if (!overflowAborts(true)) {
        std::printf("FAIL: an overflowing __fread_chk did not abort\n");
        ++failures;
    }
    if (failures) return 1;
    std::printf("stdio_chk_test: all checks passed\n");
    return 0;
#endif
}
