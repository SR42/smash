// prefetch_large_test.cpp - a fault on a compressed page of a large block
// must also restore its compressed neighbours (kPrefetchWindow each side),
// and never pages of another block.
//
// The prefetch only takes neighbours in the same span, and sameSpan()
// looked both pages up in the page map — but a large block registers only
// its FIRST page there (O(1) malloc/free), so every page inside one looked
// up as nothing and the prefetch never ran for large blocks: every
// SMASH_LARGE_ONLY=1 heap. Reading a compressed 2 MiB block in order took
// 512 faults, one per page.
//
//   1. one read in the middle of a compressed 2 MiB block restores its
//      neighbours too (3 to 5 pages resident);
//   2. reading the whole block in order takes at most half as many faults
//      as pages (a third, with a window of 2);
//   3. two adjacent compressed blocks: a read of A's last page restores
//      nothing of B (skipped, with a note, if malloc did not place them
//      next to each other).
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <string>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

static size_t resident(const char* p, size_t len) {
    std::vector<unsigned char> v(len / 4096);
    if (mincore(const_cast<char*>(p), len, v.data()) != 0) return len / 4096;
    size_t r = 0;
    for (auto c : v) r += c & 1;
    return r;
}

static std::string slurp(int fd) {
    std::string out;
    char buf[8192];
    lseek(fd, 0, SEEK_SET);
    for (ssize_t n; (n = read(fd, buf, sizeof(buf))) > 0;) out.append(buf, static_cast<size_t>(n));
    return out;
}

static unsigned long long compressedFaults(int fd) {
    raise(SIGUSR2);
    usleep(100 * 1000);
    const std::string log = slurp(fd);
    const size_t at = log.rfind("faults monitor=");
    unsigned long long m = 0, s = 0, c = 0;
    if (at != std::string::npos)
        std::sscanf(log.c_str() + at, "faults monitor=%llu shadow=%llu compressed=%llu", &m, &s, &c);
    return c;
}

static void fill(char* p, size_t len, unsigned seed) {
    for (size_t i = 0; i < len; i += 8)
        *reinterpret_cast<uint64_t*>(p + i) = (i / 4096 + seed) * 0x9E3779B97F4A7C15ull ^ ((i & 255) * 3);
}

static bool same(const char* p, size_t len, unsigned seed) {
    for (size_t i = 0; i < len; i += 8)
        if (*reinterpret_cast<const uint64_t*>(p + i) !=
            ((i / 4096 + seed) * 0x9E3779B97F4A7C15ull ^ ((i & 255) * 3)))
            return false;
    return true;
}

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) { std::fprintf(stderr, "FAIL: not running under libsmash\n"); return 1; }
    char path[] = "/tmp/smash_prefetch_XXXXXX";
    const int fd = mkstemp(path);
    if (fd < 0) { std::printf("SKIP: mkstemp failed\n"); return 77; }
    unlink(path);
    const int saved = dup(2);
    dup2(fd, 2);

    constexpr size_t kLen = 2u << 20, kPages = kLen / 4096;
    auto* a = static_cast<char*>(malloc(kLen));
    auto* b = static_cast<char*>(malloc(kLen));
    auto* c = static_cast<char*>(malloc(kLen));
    fill(a, kLen, 1);
    fill(b, kLen, 2);
    fill(c, kLen, 3);
    for (int s = 0; s < 120 && resident(a, kLen) + resident(b, kLen) + resident(c, kLen) > 3; ++s)
        sleep(1);
    const size_t cold = resident(a, kLen) + resident(b, kLen) + resident(c, kLen);
    int failures = 0;
#define fail(...) do { dprintf(saved, "FAIL: "); dprintf(saved, __VA_ARGS__); dprintf(saved, "\n"); ++failures; } while (0)
    if (cold > 3) {
        dup2(saved, 2);
        std::printf("FAIL: the blocks never left RSS (%zu pages resident)\n", cold);
        return 1;
    }
    volatile uint64_t sink = 0;

    // 1. One read in the middle of A.
    sink += *reinterpret_cast<volatile uint64_t*>(a + 200 * 4096 + 64);
    const size_t one = resident(a, kLen);
    dprintf(saved, "1. one read in the middle of a compressed block: %zu pages resident\n", one);
    if (one < 3 || one > 5) fail("one read restored %zu pages, want 3-5 (the page and its neighbours)", one);

    // 2. Read all of C in order.
    const unsigned long long f0 = compressedFaults(fd);
    for (size_t pg = 0; pg < kPages; ++pg) sink += *reinterpret_cast<volatile uint64_t*>(c + pg * 4096);
    const unsigned long long f1 = compressedFaults(fd);
    dprintf(saved, "2. reading %zu compressed pages in order: %llu faults\n", kPages, f1 - f0);
    if ((f1 - f0) * 2 > kPages) fail("%llu faults for %zu pages read in order: no prefetch", f1 - f0, kPages);
    if (!same(c, kLen, 3)) fail("block C's bytes changed");

    // 3. The last page of A, next to B.
    if (a + kLen == b) {
        const size_t b0 = resident(b, kLen);
        sink += *reinterpret_cast<volatile uint64_t*>(a + kLen - 64);
        const size_t b1 = resident(b, kLen);
        dprintf(saved, "3. read of A's last page: block B %zu -> %zu pages resident\n", b0, b1);
        if (b1 != b0) fail("a read of block A's last page restored %zu pages of block B", b1 - b0);
    } else if (b + kLen == a) {
        const size_t a0 = resident(a, kLen);
        sink += *reinterpret_cast<volatile uint64_t*>(b + kLen - 64);
        const size_t a1 = resident(a, kLen);
        dprintf(saved, "3. read of B's last page: block A %zu -> %zu pages resident\n", a0, a1);
        if (a1 != a0) fail("a read of block B's last page restored %zu pages of block A", a1 - a0);
    } else {
        dprintf(saved, "3. skipped: the blocks are not adjacent (%p, %p)\n", static_cast<void*>(a),
                static_cast<void*>(b));
    }
    if (!same(a, kLen, 1) || !same(b, kLen, 2)) fail("blocks A/B bytes changed");
    dup2(saved, 2);
    free(a);
    free(b);
    free(c);
    if (failures) return 1;
    std::printf("prefetch_large_test: all checks passed\n");
    return 0;
}
