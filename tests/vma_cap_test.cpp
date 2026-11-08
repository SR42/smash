// vma_cap_test.cpp — regression test for the vm.max_map_count livelock (#94).
//
// Linux only; run under LD_PRELOAD=libsmash.so with a live compressor.
//
// Interpreter teardown walks large compressed runs a few pages at a time.
// Restoring each faulting page alone splits its PROT_NONE VMA (+2 VMAs per
// fault); at vm.max_map_count the single-page mprotect fails with ENOMEM, and
// the old code then either re-faulted forever or marked the inaccessible page
// ACTIVE with its blob freed (SIGSEGV / wrong data).
//
// Needs no root: instead of lowering the sysctl, the test uses up VMAs itself
// (alternating page protections in a private MAP_NORESERVE mapping) until the
// process sits kHeadroom below the cap. Then:
//   1. allocate kNumBufs large blocks (smash VmRegion, compressible), each
//      page tagged with its index;
//   2. wait until most of them are PROT_NONE (compressed and reclaimed);
//   3. walk them every kStride pages, checking each tag (the teardown
//      pattern), then verify every page byte-exact.
// On the old code step 3 adds ~2 * pages / kStride VMAs, well past
// kHeadroom, and hangs or crashes; alarm() turns a hang into a failure.
//
// Exit 0 = pass, 1 = fail, 77 = skipped (not Linux, or the cap is too large
// to approach quickly).

#include <cerrno>
#include <cstdint>  // uint64_t, uintptr_t
#include <cstdio>
#include <cstdlib>
#include <cstring>

#ifndef __linux__
int main() {
    printf("vma_cap_test: SKIP (Linux only)\n");
    return 77;
}
#else

#include <fcntl.h>
#include <signal.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

namespace {

constexpr size_t kPage = 4096;
constexpr size_t kBufBytes = 2u << 20;         // 2 MiB: well above the VmRegion threshold
constexpr size_t kNumBufs = 64;                // 128 MiB, 32768 pages
constexpr size_t kPagesPerBuf = kBufBytes / kPage;
constexpr long kHeadroom = 12000;              // VMAs left below the cap
constexpr size_t kStride = 3;                  // teardown-like fault spacing
constexpr long kMaxCapToTest = 1L << 20;       // larger caps take too long to fill
constexpr int kAlarmSec = 120;

unsigned char* g_bufs[kNumBufs];
uintptr_t g_buf_lo = UINTPTR_MAX, g_buf_hi = 0;  // span of all buffers

double secsSince(const timespec& t0) {
    timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (t.tv_sec - t0.tv_sec) + (t.tv_nsec - t0.tv_nsec) / 1e9;
}

long readLong(const char* path) {
    int fd = open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0) return -1;
    char b[64];
    ssize_t n = read(fd, b, sizeof(b) - 1);
    close(fd);
    if (n <= 0) return -1;
    b[n] = 0;
    return strtol(b, nullptr, 10);
}

// Lines in /proc/self/maps, with raw read() (no stdio buffers to allocate).
long vmaCount() {
    int fd = open("/proc/self/maps", O_RDONLY | O_CLOEXEC);
    if (fd < 0) return -1;
    long n = 0;
    char b[65536];
    ssize_t r;
    while ((r = read(fd, b, sizeof(b))) > 0)
        for (ssize_t k = 0; k < r; ++k) n += b[k] == '\n';
    close(fd);
    return n;
}

// Bytes of the test buffers currently in PROT_NONE mappings.
size_t protNoneBufBytes() {
    int fd = open("/proc/self/maps", O_RDONLY | O_CLOEXEC);
    if (fd < 0) return 0;
    size_t total = 0;
    char b[65536];
    char line[512];
    size_t ll = 0;
    ssize_t r;
    while ((r = read(fd, b, sizeof(b))) > 0) {
        for (ssize_t k = 0; k < r; ++k) {
            if (b[k] != '\n') {
                if (ll + 1 < sizeof(line)) line[ll++] = b[k];
                continue;
            }
            line[ll] = 0;
            ll = 0;
            char* e;
            unsigned long lo = strtoul(line, &e, 16);
            if (*e != '-') continue;
            unsigned long hi = strtoul(e + 1, &e, 16);
            if (*e != ' ' || strncmp(e + 1, "---", 3) != 0) continue;
            if (hi <= g_buf_lo || lo >= g_buf_hi) continue;  // most lines: the filler
            for (size_t i = 0; i < kNumBufs; ++i) {
                uintptr_t blo = reinterpret_cast<uintptr_t>(g_bufs[i]);
                uintptr_t bhi = blo + kBufBytes;
                uintptr_t a = lo > blo ? lo : blo, z = hi < bhi ? hi : bhi;
                if (a < z) total += z - a;
            }
        }
    }
    close(fd);
    return total;
}

uint64_t tagOf(size_t buf, size_t page) {
    return 0x5A5A000000000000ull | (buf << 20) | page;
}

// Compressible page contents unique to (buf, page).
void fillPage(unsigned char* p, size_t buf, size_t page) {
    uint64_t t = tagOf(buf, page);
    memcpy(p, &t, sizeof(t));
    for (size_t j = sizeof(t); j < kPage; ++j)
        p[j] = static_cast<unsigned char>('a' + ((j / 64 + page) % 7));
}

bool checkPage(const unsigned char* p, size_t buf, size_t page) {
    uint64_t t;
    memcpy(&t, p, sizeof(t));
    if (t != tagOf(buf, page)) {
        fprintf(stderr, "FAIL: buf %zu page %zu tag 0x%llx want 0x%llx\n", buf, page,
                (unsigned long long)t, (unsigned long long)tagOf(buf, page));
        return false;
    }
    for (size_t j = sizeof(t); j < kPage; ++j) {
        unsigned char want = static_cast<unsigned char>('a' + ((j / 64 + page) % 7));
        if (p[j] != want) {
            fprintf(stderr, "FAIL: buf %zu page %zu byte %zu got 0x%02x want 0x%02x\n",
                    buf, page, j, p[j], want);
            return false;
        }
    }
    return true;
}

void onAlarm(int) {
    const char msg[] =
        "FAIL: vma_cap_test timed out (livelock at vm.max_map_count?)\n";
    (void)!write(2, msg, sizeof(msg) - 1);
    _exit(1);
}

}  // namespace

int main() {
    const long cap = readLong("/proc/sys/vm/max_map_count");
    if (cap <= 0 || cap > kMaxCapToTest) {
        printf("vma_cap_test: SKIP (vm.max_map_count=%ld)\n", cap);
        return 77;
    }
    setvbuf(stdout, nullptr, _IOLBF, 0);  // progress survives the alarm's _exit
    signal(SIGALRM, onAlarm);
    alarm(kAlarmSec);
    timespec t0;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    // 1. Large, compressible, page-tagged blocks.
    for (size_t i = 0; i < kNumBufs; ++i) {
        g_bufs[i] = static_cast<unsigned char*>(malloc(kBufBytes));
        if (!g_bufs[i]) { fprintf(stderr, "FAIL: malloc\n"); return 1; }
        for (size_t p = 0; p < kPagesPerBuf; ++p) fillPage(g_bufs[i] + p * kPage, i, p);
        uintptr_t a = reinterpret_cast<uintptr_t>(g_bufs[i]);
        if (a < g_buf_lo) g_buf_lo = a;
        if (a + kBufBytes > g_buf_hi) g_buf_hi = a + kBufBytes;
    }

    // Use up VMAs until kHeadroom remain. Each isolated PROT_READ page in a
    // RW mapping adds 2 (the mapping itself adds 1). Counted arithmetically:
    // re-reading /proc/self/maps as it grows toward a 1M cap would be slow.
    // The mapping is never touched, so it costs no RSS.
    const long start = vmaCount();
    const long burn = cap - kHeadroom - start - 1;
    if (burn > 1) {
        const size_t isolated = static_cast<size_t>(burn / 2);
        const size_t pages = 2 * isolated + 1;
        auto* f = static_cast<unsigned char*>(
            mmap(nullptr, pages * kPage, PROT_READ | PROT_WRITE,
                 MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0));
        if (f == MAP_FAILED) { fprintf(stderr, "FAIL: filler mmap errno=%d\n", errno); return 1; }
        for (size_t k = 0; k < isolated; ++k) {
            if (mprotect(f + (2 * k + 1) * kPage, kPage, PROT_READ) != 0) {
                fprintf(stderr, "FAIL: filler mprotect errno=%d at %ld VMAs\n", errno, vmaCount());
                return 1;
            }
        }
    }
    printf("vma_cap_test: cap=%ld start=%ld filled=%ld (%.1f s)\n", cap, start, vmaCount(),
           secsSince(t0));

    // 2. Wait until most of the blocks are compressed and reclaimed.
    const size_t want = kNumBufs * kBufBytes * 3 / 4;
    size_t none = 0;
    for (int s = 0; s < 60 && (none = protNoneBufBytes()) < want; ++s) sleep(1);
    if (none < want) {
        fprintf(stderr, "FAIL: only %zu of %zu bytes compressed (PROT_NONE) after 60 s; "
                        "is the compressor running? VMAs=%ld\n",
                none, kNumBufs * kBufBytes, vmaCount());
        return 1;
    }
    printf("vma_cap_test: %zu MiB PROT_NONE, VMAs=%ld (%.1f s)\n", none >> 20, vmaCount(),
           secsSince(t0));

    // 3. Teardown-like walk: every kStride-th page, low to high. The peak is
    // sampled every kPeakEvery buffers: at a 1M cap each read of
    // /proc/self/maps is ~70 MB.
    constexpr size_t kPeakEvery = 16;
    long peak = 0;
    for (size_t i = 0; i < kNumBufs; ++i) {
        for (size_t p = 0; p < kPagesPerBuf; p += kStride)
            if (!checkPage(g_bufs[i] + p * kPage, i, p)) return 1;
        if ((i + 1) % kPeakEvery == 0) {
            long c = vmaCount();
            if (c > peak) peak = c;
        }
    }
    for (size_t i = 0; i < kNumBufs; ++i)
        for (size_t p = 0; p < kPagesPerBuf; ++p)
            if (!checkPage(g_bufs[i] + p * kPage, i, p)) return 1;
    for (size_t i = 0; i < kNumBufs; ++i) free(g_bufs[i]);

    printf("vma_cap_test: OK, peak VMAs during walk %ld of cap %ld (%.1f s)\n", peak, cap,
           secsSince(t0));
    return 0;
}

#endif  // __linux__
