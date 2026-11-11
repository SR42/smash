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
// When step 2 gives up, the test prints what the process looked like, because
// "is the compressor running?" does not say: the mappings by place and
// protection, smash's state of every page of the blocks, the open files (which
// show the write tracker), the threads' CPU time, how the wait went second by
// second, and smash's own counters (SMASH_STATS_SIGNAL=1, set by the
// registrations). On 2026-11-10 it failed once as root with 0 bytes PROT_NONE
// and 7,433 more mappings than it had made, and 58 later root runs passed:
// the message alone could not tell a compressor that never ran from one that
// compressed everything and could not release it.
//
// Exit 0 = pass, 1 = fail, 77 = skipped (not Linux, or the cap is too large
// to approach quickly).

#include <cerrno>
#include <cstdarg>
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

#include <dlfcn.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/syscall.h>
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
uintptr_t g_fill_lo = 0, g_fill_hi = 0;          // the mapping that uses up VMAs
long g_cap = 0;

// For the report when step 2 gives up.
using page_info_fn = int (*)(const void*, int*, int*, int*);
page_info_fn g_page_info = nullptr;   // smash_page_info, when running under libsmash
bool g_stats_signal = false;          // SMASH_STATS_SIGNAL=1: SIGUSR2 prints smash's counters
struct WaitSample { int tenths; size_t none; long vmas; };
WaitSample g_samples[64];
int g_nsamples = 0;

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

// One pass over /proc/self/maps: every mapping by where it is and how it is
// protected, and the bytes of the test's blocks under each protection.
enum { kPermNone = 0, kPermRead = 1, kPermReadWrite = 2, kPermOther = 3, kPerms = 4 };
enum { kInBlocks = 0, kInFiller = 1, kElsewhere = 2, kPlaces = 3 };
struct MapsView {
    long vmas = 0;
    long count[kPlaces][kPerms] = {};
    size_t block_bytes[kPerms] = {};
};

bool scanMaps(MapsView& v) {
    int fd = open("/proc/self/maps", O_RDONLY | O_CLOEXEC);
    if (fd < 0) return false;
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
            ++v.vmas;
            char* e;
            unsigned long lo = strtoul(line, &e, 16);
            if (*e != '-') continue;
            unsigned long hi = strtoul(e + 1, &e, 16);
            if (*e != ' ') continue;
            const int perm = strncmp(e + 1, "---", 3) == 0 ? kPermNone
                           : strncmp(e + 1, "r--", 3) == 0 ? kPermRead
                           : strncmp(e + 1, "rw-", 3) == 0 ? kPermReadWrite : kPermOther;
            int place = kElsewhere;
            if (lo >= g_fill_lo && hi <= g_fill_hi) {  // most lines: the filler
                place = kInFiller;
            } else if (hi > g_buf_lo && lo < g_buf_hi) {
                for (size_t i = 0; i < kNumBufs; ++i) {
                    uintptr_t blo = reinterpret_cast<uintptr_t>(g_bufs[i]);
                    if (!blo) continue;
                    uintptr_t bhi = blo + kBufBytes;
                    uintptr_t a = lo > blo ? lo : blo, z = hi < bhi ? hi : bhi;
                    if (a < z) {
                        v.block_bytes[perm] += z - a;
                        place = kInBlocks;
                    }
                }
            }
            ++v.count[place][perm];
        }
    }
    close(fd);
    return true;
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

// snprintf and write(2) only: nothing here may need the heap under test.
void say(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void say(const char* fmt, ...) {
    char b[1024];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (static_cast<size_t>(n) >= sizeof(b)) n = sizeof(b) - 1;
    (void)!write(2, b, static_cast<size_t>(n));
}

// Every thread's state and CPU time: whether smash's threads are working,
// sleeping, or were never there.
void reportThreads() {
    int fd = open("/proc/self/task", O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (fd < 0) return;
    say("  threads (id, state, user+system CPU in clock ticks):");
    struct Dirent64 {
        uint64_t d_ino;
        int64_t d_off;
        unsigned short d_reclen;
        unsigned char d_type;
        char d_name[1];
    };
    alignas(8) char buf[2048];
    for (;;) {
        const long n = syscall(SYS_getdents64, fd, buf, sizeof(buf));
        if (n <= 0) break;
        for (long off = 0; off < n;) {
            const auto* d = reinterpret_cast<const Dirent64*>(buf + off);
            off += d->d_reclen;
            if (d->d_name[0] < '0' || d->d_name[0] > '9') continue;
            char path[64], st[512];
            snprintf(path, sizeof(path), "/proc/self/task/%s/stat", d->d_name);
            const int sfd = open(path, O_RDONLY | O_CLOEXEC);
            if (sfd < 0) continue;
            const ssize_t r = read(sfd, st, sizeof(st) - 1);
            close(sfd);
            if (r <= 0) continue;
            st[r] = 0;
            const char* p = strrchr(st, ')');   // the name may hold anything
            if (!p || p[1] != ' ') continue;
            p += 2;
            const char state = *p;
            unsigned long user = 0, sys = 0;    // 11 and 12 fields after the state
            int field = 0;
            for (const char* q = p; *q; ++q) {
                if (*q != ' ') continue;
                ++field;
                if (field == 11) user = strtoul(q + 1, nullptr, 10);
                if (field == 12) { sys = strtoul(q + 1, nullptr, 10); break; }
            }
            say(" %s %c %lu+%lu;", d->d_name, state, user, sys);
        }
    }
    close(fd);
    say("\n");
}

// What the process looks like now that the wait for compression has given up.
void reportState() {
    alarm(60);   // the report has its own time
    say("vma_cap_test: the process when the wait gave up:\n");

    // How the wait went: only the samples where something changed.
    say("  during the wait, at s: MiB of the blocks PROT_NONE / mappings:");
    for (int i = 0; i < g_nsamples; ++i) {
        const WaitSample& a = g_samples[i];
        if (i > 0 && i + 1 < g_nsamples && a.none == g_samples[i - 1].none &&
            a.vmas == g_samples[i - 1].vmas)
            continue;
        say(" %d.%d: %zu / %ld;", a.tenths / 10, a.tenths % 10, a.none >> 20, a.vmas);
    }
    say("\n");

    MapsView v;
    if (scanMaps(v)) {
        say("  mappings now: %ld of the %ld allowed\n", v.vmas, g_cap);
        static const char* const kPlace[kPlaces] = {"the test's blocks", "the filler", "everything else"};
        for (int z = 0; z < kPlaces; ++z)
            say("    in %s: %ld PROT_NONE, %ld read-only, %ld read-write, %ld other\n", kPlace[z],
                v.count[z][kPermNone], v.count[z][kPermRead], v.count[z][kPermReadWrite],
                v.count[z][kPermOther]);
        say("    of the blocks' %zu KiB: %zu PROT_NONE, %zu read-only, %zu read-write\n",
            kNumBufs * kBufBytes >> 10, v.block_bytes[kPermNone] >> 10, v.block_bytes[kPermRead] >> 10,
            v.block_bytes[kPermReadWrite] >> 10);
    }

    // smash's own view of the same pages: a block that is COMPRESSED_SHADOW has
    // been compressed and not released; ACTIVE with a low count never went cold.
    if (g_page_info) {
        long st[7] = {};        // PageState 0-5, then pages smash does not know
        long cold[5] = {};      // 0, 1, 2, 3-9, 10 or more
        int cold_max = 0, rc_max = 0;
        for (size_t i = 0; i < kNumBufs; ++i) {
            for (size_t p = 0; p < kPagesPerBuf; ++p) {
                int s = 0, c = 0, rc = 0;
                if (g_page_info(g_bufs[i] + p * kPage, &s, &c, &rc) != 0 || s < 0 || s > 5) {
                    ++st[6];
                    continue;
                }
                ++st[s];
                if (rc > rc_max) rc_max = rc;
                if (s != 1 && s != 4) continue;
                ++cold[c == 0 ? 0 : c == 1 ? 1 : c == 2 ? 2 : c < 10 ? 3 : 4];
                if (c > cold_max) cold_max = c;
            }
        }
        say("  smash's state of the blocks' %zu pages: %ld ACTIVE, %ld ACTIVE_MONITORING, %ld COMPRESSING, "
            "%ld COMPRESSED_SHADOW, %ld COMPRESSED, %ld EMPTY, %ld unknown to smash\n",
            kNumBufs * kPagesPerBuf, st[1], st[4], st[3], st[5], st[2], st[0], st[6]);
        say("    cold ticks of the ACTIVE and ACTIVE_MONITORING pages: %ld at 0, %ld at 1, %ld at 2, "
            "%ld at 3-9, %ld at 10 or more (highest %d); most recompressions of one page: %d\n",
            cold[0], cold[1], cold[2], cold[3], cold[4], cold_max, rc_max);
    } else {
        say("  smash_page_info is not there: is this running under libsmash?\n");
    }

    // The write tracker shows in what is open: clear_refs (soft-dirty), a
    // userfaultfd, the page_idle bitmap (root: read tracking).
    say("  open files:");
    for (int fd = 0; fd < 128; ++fd) {
        char link[32], target[256];
        snprintf(link, sizeof(link), "/proc/self/fd/%d", fd);
        const ssize_t n = readlink(link, target, sizeof(target) - 1);
        if (n <= 0) continue;
        target[n] = 0;
        say(" %d=%s;", fd, target);
    }
    say("\n");

    reportThreads();

    // Only with smash's handler in place: SIGUSR2 otherwise ends the process.
    struct sigaction usr2 {};
    sigaction(SIGUSR2, nullptr, &usr2);
    if (!g_stats_signal) {
        say("  SMASH_STATS_SIGNAL=1 is not set: no counters from smash\n");
    } else if (usr2.sa_handler == SIG_DFL || usr2.sa_handler == SIG_IGN) {
        say("  SIGUSR2 has no handler: smash's compressor never installed its own\n");
    } else {
        say("  smash's counters, twice, 2 s apart (do ticks= and deferred= still move?):\n");
        raise(SIGUSR2);
        sleep(2);
        raise(SIGUSR2);
    }
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
    g_cap = cap;
    g_page_info = reinterpret_cast<page_info_fn>(dlsym(RTLD_DEFAULT, "smash_page_info"));
    const char* stats_env = getenv("SMASH_STATS_SIGNAL");
    g_stats_signal = stats_env && stats_env[0] == '1';
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
        g_fill_lo = reinterpret_cast<uintptr_t>(f);
        g_fill_hi = g_fill_lo + pages * kPage;
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
    // 60 looks a second apart, and never past 100 s after the start: each
    // look reads a million lines, and the alarm must not cut the report off.
    const size_t want = kNumBufs * kBufBytes * 3 / 4;
    size_t none = 0;
    int waited = 0;
    for (; waited < 60 && secsSince(t0) < 100; ++waited) {
        MapsView v;
        scanMaps(v);
        none = v.block_bytes[kPermNone];
        if (g_nsamples < static_cast<int>(sizeof(g_samples) / sizeof(g_samples[0])))
            g_samples[g_nsamples++] = {static_cast<int>(secsSince(t0) * 10), none, v.vmas};
        if (none >= want) break;
        sleep(1);
    }
    if (none < want) {
        fprintf(stderr, "FAIL: only %zu of %zu bytes compressed (PROT_NONE) after %d s; "
                        "is the compressor running? VMAs=%ld\n",
                none, kNumBufs * kBufBytes, waited, vmaCount());
        reportState();
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
