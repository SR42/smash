// stdio_after_compression_test.cpp - buffered reads must keep working once the
// compressor has run.
//
// glibc allocates a FILE's buffer lazily, inside the first read, with malloc —
// so under smash it can land on a page the compressor already owns. The kernel
// then hits a protected page in copy_to_user and returns EFAULT, and because
// glibc's internal read is intra-libc, smash's read() wrapper never sees it:
// the error surfaces to the application as a failed stream. Warming before
// delegating cannot help either, since on the first call there is no buffer to
// warm yet (_IO_buf_base is null).
//
// Measured before the fix, in this exact shape: fgets over /proc/self/maps
// returned 0 lines with ferror set and errno 14 (EFAULT), while raw read(2)
// into a heap buffer of the same size worked — that asymmetry is the tell,
// because raw read goes through smash's EFAULT-retry wrapper and stdio does
// not.
#include <cerrno>
#include <chrono>
#include <dlfcn.h>
#include <thread>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <unistd.h>

namespace {

int failures = 0;
constexpr size_t kBlockMiB = 256;

void fail(const char* what, long got, int err) {
    std::fprintf(stderr, "FAIL: %s (got %ld, errno %d=%s) — a buffered read hit a "
                 "compressed page and the error reached the application\n",
                 what, got, err, std::strerror(err));
    ++failures;
}

// True once the block really has left RSS, so the test is not vacuous.
bool waitCompressed(const unsigned char* p, size_t bytes) {
    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    const size_t pages = bytes / ps;
    auto* vec = static_cast<unsigned char*>(std::malloc(pages));
    if (!vec) return false;
    for (int i = 0; i < 120; ++i) {
        if (mincore(const_cast<unsigned char*>(p), bytes, vec) == 0) {
            size_t gone = 0;
            for (size_t k = 0; k < pages; ++k) if (!(vec[k] & 1)) ++gone;
            if (gone > pages / 10) {           // 10% is plenty to prove the point
                std::fprintf(stderr, "stdio_after_compression: %zu of %zu pages left RSS\n",
                             gone, pages);
                std::free(vec);
                return true;
            }
        }
        usleep(250 * 1000);
    }
    std::free(vec);
    return false;
}

}  // namespace

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    setvbuf(stderr, nullptr, _IONBF, 0);

    // A data file to read back with stdio, written before anything is cold.
    char path[] = "/tmp/smash-stdio-after-compression-XXXXXX";
    int fd = mkstemp(path);
    if (fd < 0) { std::fprintf(stderr, "FAIL: mkstemp\n"); return 1; }
    std::string want;
    for (int i = 0; i < 4096; ++i) want += "the quick brown fox jumps over the lazy dog\n";
    if (write(fd, want.data(), want.size()) != static_cast<ssize_t>(want.size())) {
        std::fprintf(stderr, "FAIL: write temp file\n"); return 1;
    }
    close(fd);

    auto* block = static_cast<unsigned char*>(std::malloc(kBlockMiB << 20));
    if (!block) { std::fprintf(stderr, "FAIL: malloc\n"); return 1; }
    for (size_t i = 0; i < (kBlockMiB << 20); i += 4096)
        block[i] = static_cast<unsigned char>(i >> 12);

    if (!waitCompressed(block, kBlockMiB << 20)) {
        std::fprintf(stderr, "SKIP: nothing compressed within 30s; cannot exercise the path\n");
        unlink(path);
        return 0;
    }

    // Each of these opens a FRESH stream, so glibc allocates its buffer inside
    // the first read — the case that has no buffer to warm up front.
    //
    // Called through dlsym'd pointers. With a buffer of known size,
    // _FORTIFY_SOURCE turns a direct fgets/fread into __fgets_chk/__fread_chk,
    // which smash deliberately does not wrap, and then this test never reaches
    // the code it guards: the UBSan build does exactly that, and failed the
    // fgets check against the pre-fix and fixed libraries alike.
    using fgets_fn = char* (*)(char*, int, FILE*);
    using fread_fn = size_t (*)(void*, size_t, size_t, FILE*);
    auto fgets_p = reinterpret_cast<fgets_fn>(dlsym(RTLD_DEFAULT, "fgets"));
    auto fread_p = reinterpret_cast<fread_fn>(dlsym(RTLD_DEFAULT, "fread"));
    if (!fgets_p || !fread_p) {
        std::fprintf(stderr, "FAIL: dlsym(fgets/fread)\n");
        unlink(path);
        return 1;
    }
    {   // fgets over a procfs file (no size known in advance)
        FILE* f = std::fopen("/proc/self/maps", "r");
        if (!f) { std::fprintf(stderr, "FAIL: fopen /proc/self/maps\n"); ++failures; }
        else {
            char line[1024];
            long lines = 0;
            errno = 0;
            while (fgets_p(line, sizeof(line), f)) ++lines;
            const int e = errno;
            if (lines == 0 || std::ferror(f)) fail("fgets(/proc/self/maps)", lines, e);
            else std::fprintf(stderr, "stdio_after_compression: fgets read %ld lines\n", lines);
            std::fclose(f);
        }
    }
    {   // fread over a regular file, verified byte-exact
        FILE* f = std::fopen(path, "r");
        if (!f) { std::fprintf(stderr, "FAIL: fopen temp file\n"); ++failures; }
        else {
            std::string got;
            char buf[8192];
            size_t n;
            errno = 0;
            while ((n = fread_p(buf, 1, sizeof(buf), f)) > 0) got.append(buf, n);
            const int e = errno;
            if (std::ferror(f) || got.size() != want.size())
                fail("fread(temp file)", static_cast<long>(got.size()), e);
            else if (got != want) fail("fread(temp file) content mismatch", 0, 0);
            else std::fprintf(stderr, "stdio_after_compression: fread returned %zu bytes, byte-exact\n",
                              got.size());
            std::fclose(f);
        }
    }
    {   // fgetc, one byte at a time through a fresh stream
        FILE* f = std::fopen(path, "r");
        if (!f) { std::fprintf(stderr, "FAIL: fopen temp file #2\n"); ++failures; }
        else {
            errno = 0;
            int c = std::fgetc(f);
            const int e = errno;
            if (c == EOF || std::ferror(f)) fail("fgetc(temp file)", c, e);
            std::fclose(f);
        }
    }

    {   // A stream whose buffer already exists, blocked in fgets on a pipe for
        // longer than the cold timeout. The buffer page goes cold while the
        // reader waits, the writer's data then meets a compressed page in the
        // kernel's copy, and glibc reports EFAULT having consumed nothing.
        // Warming before the call cannot help here — the page cools during
        // it — so this is the retry's other job besides the lazy buffer, and
        // the one a retry restricted to bufferless streams broke: 2 of 2 runs
        // failed with errno 14.
        auto fg = fgets_p;
        int fds[2];
        FILE* f = nullptr;
        if (pipe(fds) != 0 || !(f = fdopen(fds[0], "r"))) {
            std::fprintf(stderr, "FAIL: pipe / fdopen\n"); ++failures;
        } else {
            char line[128];
            const char first[] = "first\n";
            if (write(fds[1], first, sizeof(first) - 1) != sizeof(first) - 1 ||
                !fg(line, sizeof(line), f)) {
                std::fprintf(stderr, "FAIL: first line through the pipe\n"); ++failures;
            } else {
                std::thread writer([&] {
                    std::this_thread::sleep_for(std::chrono::seconds(8));
                    const char l[] = "second line, after the reader went cold\n";
                    if (write(fds[1], l, sizeof(l) - 1) != sizeof(l) - 1) std::abort();
                });
                errno = 0;
                char* r = fg(line, sizeof(line), f);
                const int e = errno;
                writer.join();
                if (!r || std::strcmp(line, "second line, after the reader went cold\n") != 0)
                    fail("fgets(pipe) after blocking 8 s", r ? 1 : 0, e);
                else std::fprintf(stderr, "stdio_after_compression: blocked fgets recovered\n");
            }
            std::fclose(f);
            close(fds[1]);
        }
    }

    std::free(block);
    unlink(path);
    if (failures) { std::printf("stdio_after_compression_test: %d FAILURES\n", failures); return 1; }
    std::printf("stdio_after_compression_test: all checks passed\n");
    return 0;
}
