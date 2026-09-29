// short_io_test.cpp - file I/O into / out of a buffer that is only partly
// accessible must transfer everything, not return a short count.
//
// The EFAULT retry only fires when a syscall returns -1/EFAULT. When a
// buffer's first page is accessible and a later one is not (a PROT_READ
// watched page for a read into it, a PROT_NONE compressed or escalated page
// for a write out of it), the kernel copies up to that page and returns a
// SHORT COUNT — success, which the wrappers used to hand straight back. For a
// regular file a short read looks like end of file. Found with
// SMASH_RESTORE_GRANULE_PAGES=16 forced on test_syscall_efault (preadv:
// 16384 of 131072), but under SMASH_SOFTDIRTY=0 every watched page is
// read-only, so any multi-page read into a heap buffer whose first page was
// just written could stop short.
//
// Each case first proves the precondition on the raw kernel entry
// (syscall(2): the first page accessible, a short count), waiting until the
// compressor has made the buffer that way, then checks the libc entry
// transfers the whole buffer with the right bytes. read, pread, readv,
// preadv, preadv2 and __read_chk (fortified read) into the buffer; write,
// pwrite, writev, pwritev out of it. And what must not change: a pipe
// keeps its short count (continuing would block: guarded by alarm()), and a
// file shorter than the buffer still reads as its own length.
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <string>
#include <sys/syscall.h>
#include <sys/uio.h>
#include <unistd.h>

extern "C" ssize_t __read_chk(int fd, void* buf, size_t count, size_t buflen);

namespace {

constexpr size_t kLen = 2u << 20;   // a large block: compressible, watched
size_t g_ps = 4096;
int g_failures = 0;

void fail(const char* what, ssize_t got) {
    std::printf("FAIL: %s returned %zd of %zu (errno %d %s)\n", what, got, kLen, errno,
                std::strerror(errno));
    ++g_failures;
}

unsigned char patternByte(size_t i) { return static_cast<unsigned char>((i * 131u + (i >> 12)) & 0xff); }

bool matchesPattern(const unsigned char* p) {
    for (size_t i = 0; i < kLen; i += 509)
        if (p[i] != patternByte(i)) return false;
    return p[kLen - 1] == patternByte(kLen - 1);
}

int makeFile(bool with_pattern) {
    char path[] = "/tmp/smash_shortio_XXXXXX";
    const int fd = mkstemp(path);
    if (fd < 0) return -1;
    unlink(path);
    if (with_pattern) {
        unsigned char chunk[65536];
        for (size_t off = 0; off < kLen; off += sizeof(chunk)) {
            for (size_t i = 0; i < sizeof(chunk); ++i) chunk[i] = patternByte(off + i);
            if (::syscall(SYS_pwrite64, fd, chunk, sizeof(chunk), off) != static_cast<ssize_t>(sizeof(chunk)))
                return -1;
        }
    }
    return fd;
}

// Waits (up to 60 s) until a raw read of the file into buf stops short: the
// first page writable, a later one not. Touches the first page by writing
// it before each try. False if it never does.
bool waitShortRead(unsigned char* buf, int fd) {
    for (int i = 0; i < 120; ++i) {
        buf[0] = 1;                                   // first page writable
        const ssize_t r = ::syscall(SYS_pread64, fd, buf, kLen, 0);
        if (r > 0 && static_cast<size_t>(r) < kLen) return true;
        usleep(500 * 1000);
    }
    return false;
}

// Waits until a raw write of buf stops short: the first page readable, a
// later one PROT_NONE (escalated or compressed).
bool waitShortWrite(unsigned char* buf, int fd) {
    for (int i = 0; i < 120; ++i) {
        volatile unsigned char c = buf[0];            // first page readable
        (void)c;
        const ssize_t r = ::syscall(SYS_pwrite64, fd, buf, kLen, 0);
        if (r > 0 && static_cast<size_t>(r) < kLen) return true;
        usleep(500 * 1000);
    }
    return false;
}

void fillPattern(unsigned char* buf) {
    for (size_t i = 0; i < kLen; ++i) buf[i] = patternByte(i);
}

bool fileMatches(int fd) {
    static unsigned char check[65536];
    for (size_t off = 0; off < kLen; off += sizeof(check)) {
        if (::syscall(SYS_pread64, fd, check, sizeof(check), off) != static_cast<ssize_t>(sizeof(check)))
            return false;
        for (size_t i = 0; i < sizeof(check); ++i)
            if (check[i] != patternByte(off + i)) return false;
    }
    return true;
}

template <typename Call>
void readCase(const char* name, int src, Call call) {
    auto* buf = static_cast<unsigned char*>(malloc(kLen));
    std::memset(buf, 0x5a, kLen);
    if (!waitShortRead(buf, src)) {
        std::printf("FAIL: %s: a raw read into the buffer never stopped short in 60 s "
                    "(SMASH_SOFTDIRTY=0 set?)\n", name);
        ++g_failures;
        free(buf);
        return;
    }
    buf[0] = 1;
    errno = 0;
    const ssize_t r = call(buf);
    if (r != static_cast<ssize_t>(kLen)) fail(name, r);
    else if (!matchesPattern(buf)) { std::printf("FAIL: %s: wrong bytes\n", name); ++g_failures; }
    else std::printf("%s: %zu of %zu bytes after a raw short read\n", name, static_cast<size_t>(r), kLen);
    free(buf);
}

template <typename Call>
void writeCase(const char* name, Call call) {
    auto* buf = static_cast<unsigned char*>(malloc(kLen));
    fillPattern(buf);
    const int probe = makeFile(false);
    const int dst = makeFile(false);
    if (probe < 0 || dst < 0) { std::printf("SKIP: mkstemp failed\n"); free(buf); return; }
    if (!waitShortWrite(buf, probe)) {
        std::printf("FAIL: %s: a raw write from the buffer never stopped short in 60 s\n", name);
        ++g_failures;
    } else {
        errno = 0;
        const ssize_t r = call(dst, buf);
        if (r != static_cast<ssize_t>(kLen)) fail(name, r);
        else if (!fileMatches(dst)) { std::printf("FAIL: %s: wrong bytes in the file\n", name); ++g_failures; }
        else std::printf("%s: %zu of %zu bytes after a raw short write\n", name, static_cast<size_t>(r), kLen);
    }
    close(probe);
    close(dst);
    free(buf);
}

}  // namespace

int main() {
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) { std::printf("SKIP: not running under libsmash\n"); return 0; }
    g_ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    const int src = makeFile(true);
    if (src < 0) { std::printf("SKIP: could not create the source file\n"); return 0; }

    readCase("read", src, [&](unsigned char* b) {
        lseek(src, 0, SEEK_SET);
        return read(src, b, kLen);
    });
    readCase("pread", src, [&](unsigned char* b) { return pread(src, b, kLen, 0); });
    readCase("__read_chk", src, [&](unsigned char* b) {
        lseek(src, 0, SEEK_SET);
        return __read_chk(src, b, kLen, kLen);
    });
    readCase("readv", src, [&](unsigned char* b) {
        lseek(src, 0, SEEK_SET);
        iovec v[2] = {{b, kLen / 2}, {b + kLen / 2, kLen / 2}};
        return readv(src, v, 2);
    });
    readCase("preadv", src, [&](unsigned char* b) {
        iovec v[2] = {{b, kLen / 2}, {b + kLen / 2, kLen / 2}};
        return preadv(src, v, 2, 0);
    });
    readCase("preadv2", src, [&](unsigned char* b) {
        iovec v[2] = {{b, kLen / 2}, {b + kLen / 2, kLen / 2}};
        return preadv2(src, v, 2, 0, 0);
    });

    writeCase("write", [](int fd, unsigned char* b) { return write(fd, b, kLen); });
    writeCase("pwrite", [](int fd, unsigned char* b) { return pwrite(fd, b, kLen, 0); });
    writeCase("writev", [](int fd, unsigned char* b) {
        iovec v[2] = {{b, kLen / 2}, {b + kLen / 2, kLen / 2}};
        return writev(fd, v, 2);
    });
    writeCase("pwritev", [](int fd, unsigned char* b) {
        iovec v[2] = {{b, kLen / 2}, {b + kLen / 2, kLen / 2}};
        return pwritev(fd, v, 2, 0);
    });

    // A pipe holding 100 bytes: read must return them, not wait for more.
    {
        auto* buf = static_cast<unsigned char*>(malloc(kLen));
        std::memset(buf, 0x5a, kLen);
        int p[2];
        if (pipe(p) == 0) {
            const int probe = makeFile(true);
            if (!waitShortRead(buf, probe)) {
                std::printf("FAIL: pipe: the buffer never became partly writable\n");
                ++g_failures;
            } else {
                unsigned char msg[100];
                std::memset(msg, 7, sizeof(msg));
                if (::write(p[1], msg, sizeof(msg)) != 100) std::printf("SKIP: pipe write\n");
                buf[0] = 1;
                alarm(10);   // a continuation on a pipe would block here
                const ssize_t r = read(p[0], buf, kLen);
                alarm(0);
                if (r != 100) { std::printf("FAIL: pipe read returned %zd, not 100\n", r); ++g_failures; }
                else std::printf("pipe: 100 bytes, no wait for more\n");
            }
            close(probe);
            close(p[0]);
            close(p[1]);
        }
        free(buf);
    }
    // A 1 MiB file into the 2 MiB buffer: 1 MiB, then 0 at end of file.
    {
        auto* buf = static_cast<unsigned char*>(malloc(kLen));
        std::memset(buf, 0x5a, kLen);
        const int half = makeFile(true);
        if (half >= 0 && ftruncate(half, kLen / 2) == 0 && waitShortRead(buf, src)) {
            buf[0] = 1;
            const ssize_t r = pread(half, buf, kLen, 0);
            const ssize_t z = pread(half, buf, kLen, kLen / 2);
            if (r != static_cast<ssize_t>(kLen / 2) || z != 0) {
                std::printf("FAIL: short file: %zd then %zd, want %zu then 0\n", r, z, kLen / 2);
                ++g_failures;
            } else {
                std::printf("short file: %zd bytes, then 0\n", r);
            }
        } else {
            std::printf("FAIL: short file case could not be set up\n");
            ++g_failures;
        }
        if (half >= 0) close(half);
        free(buf);
    }

    close(src);
    if (g_failures) return 1;
    std::printf("short_io_test: all checks passed\n");
    return 0;
}
