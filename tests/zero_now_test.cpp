// zero_now_test.cpp - the [smash stats] line's zero_now= must count the
// compressed pages stored as the all-zero sentinel now, while zero= keeps
// counting every one since start.
//
// Added because zero= is cumulative, so a dump could not say how many of
// `compressed=` were zero pages at that moment.
//
// 32 MiB, every even page written with zeros and every odd page with 1 KiB of
// random bytes (compressible, never all zero), is compressed out of RSS.
// Checks:
//   - zero_now= is within 5% of the 4096 zero pages, and never above zero=;
//   - after every zero page is read back (faulted in, prefetch takes some odd
//     neighbours too), zero_now= falls below 10% of them, and zero= does not
//     fall;
//   - every byte reads back.
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

namespace {

constexpr size_t kLen = 32u << 20, kPage = 4096, kPages = kLen / kPage, kZero = kPages / 2;
int g_log = -1;

size_t resident(const char* p) {
    std::vector<unsigned char> v(kPages);
    if (mincore(const_cast<char*>(p), kLen, v.data()) != 0) return kPages;
    size_t r = 0;
    for (auto c : v) r += c & 1;
    return r;
}

struct Dump { bool ok = false; size_t compressed = 0, zero = 0, zero_now = 0; };

// Raises SIGUSR2 and parses the newest [smash stats] line.
Dump dump() {
    Dump d;
    raise(SIGUSR2);
    usleep(100 * 1000);
    std::string log;
    char buf[8192];
    lseek(g_log, 0, SEEK_SET);
    for (ssize_t n; (n = read(g_log, buf, sizeof(buf))) > 0;) log.append(buf, static_cast<size_t>(n));
    const size_t at = log.rfind("[smash stats]");
    if (at == std::string::npos) return d;
    const std::string line = log.substr(at, log.find('\n', at) - at);
    const size_t c = line.find(" compressed="), z = line.find(" zero=");
    if (c == std::string::npos || z == std::string::npos) return d;
    d.ok = std::sscanf(line.c_str() + c, " compressed=%zu", &d.compressed) == 1 &&
           std::sscanf(line.c_str() + z, " zero=%zu  zero_now=%zu", &d.zero, &d.zero_now) == 2;
    return d;
}

uint8_t dataByte(size_t page, size_t i) { return static_cast<uint8_t>((page * 131 + i * 7) ^ (i >> 3)) | 1; }

}  // namespace

int main() {
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) { std::fprintf(stderr, "FAIL: not running under libsmash\n"); return 1; }
    char path[] = "/tmp/smash_zeronow_XXXXXX";
    g_log = mkstemp(path);
    if (g_log < 0) { std::perror("mkstemp"); return 1; }
    unlink(path);
    dup2(g_log, 2);
    int failures = 0;

    auto* b = static_cast<char*>(malloc(kLen));
    for (size_t p = 0; p < kPages; ++p) {
        char* pg = b + p * kPage;
        if (p % 2 == 0) {
            std::memset(pg, 0, kPage);
        } else {
            for (size_t i = 0; i < 1024; ++i) pg[i] = static_cast<char>(dataByte(p, i));
            std::memset(pg + 1024, 0, kPage - 1024);
        }
    }
    size_t left = kPages;
    for (int s = 0; s < 240 && (left = resident(b)) > 0; ++s) usleep(250 * 1000);
    if (left > 0) {
        std::printf("FAIL: %zu pages still resident after 60 s\n", left);
        return 1;
    }
    const Dump out = dump();
    std::printf("compressed: compressed=%zu zero=%zu zero_now=%zu (%zu zero pages)\n", out.compressed, out.zero,
                out.zero_now, kZero);
    if (!out.ok) {
        std::printf("FAIL: no zero=/zero_now= on the [smash stats] line\n");
        return 1;
    }
    if (out.zero_now < kZero * 95 / 100 || out.zero_now > kZero * 105 / 100) {
        std::printf("FAIL: zero_now=%zu, expected ~%zu\n", out.zero_now, kZero);
        ++failures;
    }
    if (out.zero_now > out.zero) {
        std::printf("FAIL: zero_now=%zu above the cumulative zero=%zu\n", out.zero_now, out.zero);
        ++failures;
    }

    volatile char sink = 0;
    for (size_t p = 0; p < kPages; p += 2) sink = static_cast<char>(sink + b[p * kPage]);
    const Dump back = dump();
    std::printf("read back:  compressed=%zu zero=%zu zero_now=%zu\n", back.compressed, back.zero, back.zero_now);
    if (!back.ok || back.zero_now >= kZero / 10) {
        std::printf("FAIL: zero_now=%zu after every zero page was read back\n", back.zero_now);
        ++failures;
    }
    if (back.zero < out.zero) {
        std::printf("FAIL: cumulative zero= fell from %zu to %zu\n", out.zero, back.zero);
        ++failures;
    }

    size_t bad = 0;
    for (size_t p = 0; p < kPages; ++p)
        for (size_t i = 0; i < kPage; ++i) {
            const uint8_t want = (p % 2 == 1 && i < 1024) ? dataByte(p, i) : 0;
            bad += static_cast<uint8_t>(b[p * kPage + i]) != want;
        }
    if (bad) {
        std::printf("FAIL: %zu bytes read back wrong\n", bad);
        ++failures;
    }
    free(b);
    if (failures) return 1;
    std::printf("zero_now_test: all checks passed\n");
    return 0;
}
