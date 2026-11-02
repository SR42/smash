// thp_unit_test.cpp - SMASH_THP_UNIT=1: a transparent huge page is compressed
// as one unit and comes back as one, still a huge page.
//
// Without it the compressor protects, verifies and drops a cold huge page in
// runs of 32 pages, which splits it, and a fault restores 4 KiB: hot again,
// the data sits on 4 KiB TLB entries for good. A unit is all 512 pages of one
// cold 2 MiB folio: every protection change and the drop cover the whole 2 MiB,
// which keeps the kernel's huge mapping until the drop frees it, and any fault
// in the unit restores all 512 pages through /proc/self/mem while they are
// still PROT_NONE (the first write allocates a new huge page) before one
// mprotect makes them accessible. A partial restore is the trap: one 4 KiB
// written into a dropped huge range makes the kernel allocate the whole 2 MiB.
//
// Needs SMASH_WRITE_TRACK=idle, so root and the page_idle bitmap (skips
// otherwise), and huge pages in the arena (skips when the kernel gives none).
//
// THP_UNIT_PART:
//   unit     (default) 32 units go cold and leave RSS. A one-byte read in each
//            of 4 units brings back exactly those 4, each a huge page again; a
//            write fault does the same; 16 threads reading 11 units at once
//            restore each unit once and read every byte right; the units go
//            cold and come back once more; a block calloc'd where they were
//            reads zeros.
//            THP_UNIT_EXPECT=split runs the same with SMASH_THP=1 and no
//            SMASH_THP_UNIT: the reads must come back as 4 KiB pages (what
//            the option exists to change).
//   mixed    a unit with one incompressible page still compresses (the ratio
//            is the unit's); a unit of random bytes is rolled back and stays
//            one huge page; a unit with one page read every 200 ms is never
//            compressed and stays one huge page.
//   syscall  write() from, read() and pread() into compressed units (the
//            EFAULT retry faults them in): every byte right, each touched unit
//            a huge page again.
//   fork     a child reads and writes compressed units; then the parent.
//   shrink   realloc shrinks a block with compressed units in place, cutting
//            a unit: the kept bytes read back right, then go cold and come
//            back again.
//   fallback SMASH_THP_UNIT=1 where it cannot work (no idle tracker): the
//            option must say so on stderr and stay off.
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <set>
#include <string>
#include <sys/mman.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

constexpr size_t kPage = 4096, kUnit = 2u << 20, kUnitPages = kUnit / kPage;

using UnitInfo = int (*)(uint64_t*, uint64_t*, uint64_t*, uint64_t*);
UnitInfo g_info;

struct Counts { uint64_t compressed = 0, reclaimed = 0, restored = 0, rolled_back = 0; int active = -1; };
Counts counts() {
    Counts c;
    if (g_info) c.active = g_info(&c.compressed, &c.reclaimed, &c.restored, &c.rolled_back);
    return c;
}
void printCounts(const char* when) {
    const Counts c = counts();
    std::printf("  [%s] units: active=%d compressed=%llu reclaimed=%llu restored=%llu rolled_back=%llu\n", when,
                c.active, (unsigned long long)c.compressed, (unsigned long long)c.reclaimed,
                (unsigned long long)c.restored, (unsigned long long)c.rolled_back);
}

#define CHECK(cond, ...)                                  \
    do {                                                  \
        if (!(cond)) {                                    \
            std::printf("FAIL: " __VA_ARGS__);            \
            std::printf("\n");                            \
            printCounts("at failure");                    \
            return 1;                                     \
        }                                                 \
    } while (0)

uint64_t pattern(size_t page, uint64_t salt) { return (page + 1) * 0x9E3779B97F4A7C15ull ^ salt; }

// Every word of a page is the page's pattern: unique per page, compressible.
void fill(char* p, size_t len, uint64_t salt) {
    for (size_t pg = 0; pg < len / kPage; ++pg) {
        auto* w = reinterpret_cast<uint64_t*>(p + pg * kPage);
        const uint64_t v = pattern(pg, salt);
        for (size_t k = 0; k < kPage / 8; ++k) w[k] = v;
    }
}
// First bad page, or -1.
long bad(const char* p, size_t len, uint64_t salt, size_t first_page = 0) {
    for (size_t pg = 0; pg < len / kPage; ++pg) {
        const auto* w = reinterpret_cast<const uint64_t*>(p + pg * kPage);
        const uint64_t v = pattern(first_page + pg, salt);
        for (size_t k = 0; k < kPage / 8; ++k)
            if (w[k] != v) return static_cast<long>(pg);
    }
    return -1;
}

size_t resident(const void* p, size_t len) {
    std::vector<unsigned char> v(len / kPage);
    if (mincore(const_cast<void*>(p), len, v.data()) != 0) return len / kPage;
    size_t r = 0;
    for (auto c : v) r += c & 1;
    return r;
}

long rollupHugeKib() {
    FILE* f = std::fopen("/proc/self/smaps_rollup", "r");
    if (!f) return -1;
    char line[256];
    long v = -1;
    while (std::fgets(line, sizeof(line), f))
        if (!std::strncmp(line, "AnonHugePages:", 14)) { v = std::atol(line + 14); break; }
    std::fclose(f);
    return v;
}

// Is the 2 MiB at p one transparent huge page right now: every page present,
// physically contiguous from a compound head flagged THP (/proc/kpageflags).
bool unitIsHuge(const char* p) {
    static int pm = open("/proc/self/pagemap", O_RDONLY | O_CLOEXEC);
    static int kf = open("/proc/kpageflags", O_RDONLY | O_CLOEXEC);
    if (pm < 0 || kf < 0) return false;
    uint64_t e[kUnitPages];
    if (pread(pm, e, sizeof(e), static_cast<off_t>(reinterpret_cast<uintptr_t>(p) / kPage * 8)) !=
        static_cast<ssize_t>(sizeof(e)))
        return false;
    const uint64_t pfn0 = e[0] & ((1ull << 55) - 1);
    for (size_t k = 0; k < kUnitPages; ++k)
        if (!(e[k] >> 63) || (e[k] & ((1ull << 55) - 1)) != pfn0 + k) return false;
    uint64_t fl = 0;
    if (pread(kf, &fl, 8, static_cast<off_t>(pfn0 * 8)) != 8) return false;
    return (fl >> 15 & 1) && (fl >> 22 & 1);   // KPF_COMPOUND_HEAD, KPF_THP
}

// The kernel's count of huge-page faults that fell back to 4 KiB pages
// (/proc/vmstat): a unit restored while it was rising may be 4 KiB pages
// without smash having split anything. Every other time every unit touched
// must come back one huge page.
long thpFallbacks() {
    FILE* f = std::fopen("/proc/vmstat", "r");
    if (!f) return 0;
    char line[128];
    long v = 0;
    while (std::fgets(line, sizeof(line), f))
        if (!std::strncmp(line, "thp_fault_fallback ", 19)) { v = std::atol(line + 19); break; }
    std::fclose(f);
    return v;
}

size_t hugeUnits(const char* a, size_t first, size_t n) {
    size_t h = 0;
    for (size_t u = first; u < first + n; ++u) h += unitIsHuge(a + u * kUnit);
    return h;
}

char* alignUnit(void* p) {
    return reinterpret_cast<char*>((reinterpret_cast<uintptr_t>(p) + kUnit - 1) & ~(kUnit - 1));
}

bool expectSplit() {
    const char* e = std::getenv("THP_UNIT_EXPECT");
    return e && std::strcmp(e, "split") == 0;
}

// Wait until at most `max_pages` of [p, p + len) are resident.
bool waitOut(const char* p, size_t len, size_t max_pages, int seconds) {
    for (int t = 0; t < seconds * 10; ++t) {
        if (resident(p, len) <= max_pages) return true;
        usleep(100 * 1000);
    }
    return false;
}

// Root, the bitmap, the idle tracker running, and a block that really got huge
// pages. 0 = go, 1 = fail, 2 = skip (printed).
// `lead`: the 2 MiB before `a` is part of the block too, on 4 KiB pages
// (MADV_NOHUGEPAGE), so it is compressed page by page next to unit 0.
int prepare(char*& a, size_t units, uint64_t salt, void** block = nullptr, bool lead = false) {
    if (geteuid() != 0 || access("/sys/kernel/mm/page_idle/bitmap", R_OK | W_OK) != 0) {
        std::printf("SKIP: needs root and /sys/kernel/mm/page_idle/bitmap\n");
        return 2;
    }
    using TrackFn = const char* (*)();
    auto track = reinterpret_cast<TrackFn>(dlsym(RTLD_DEFAULT, "smash_write_tracker"));
    if (!track) { std::printf("SKIP: not running under libsmash\n"); return 2; }
    void* blk = std::malloc(units * kUnit + (lead ? 2 : 1) * kUnit);
    CHECK(blk, "malloc");
    a = alignUnit(blk) + (lead ? kUnit : 0);
    if (block) *block = blk;
    if (lead) {
        CHECK(madvise(a - kUnit, kUnit, MADV_NOHUGEPAGE) == 0, "madvise: %s", strerror(errno));
        fill(a - kUnit, kUnit, salt ^ 0x5555);
    }
    fill(a, units * kUnit, salt);
    for (int t = 0; t < 50 && std::strcmp(track(), "idle") != 0; ++t) usleep(100 * 1000);
    CHECK(std::strcmp(track(), "idle") == 0, "tracker is %s, not idle", track());
    const size_t huge = hugeUnits(a, 0, units);
    std::printf("  %zu of %zu units are huge pages after the fill (AnonHugePages %ld KiB)\n", huge, units,
                rollupHugeKib());
    // Every check counts on every unit being one huge page at the start.
    if (huge < units) {
        std::printf("SKIP: the kernel gave the arena %zu of %zu huge pages (THP off, or no memory)\n", huge, units);
        return 2;
    }
    if (!expectSplit()) CHECK(g_info, "smash_thp_unit_info is not exported");
    if (!expectSplit()) CHECK(counts().active == 1, "SMASH_THP_UNIT=1 is not active (info says %d)", counts().active);
    return 0;
}

int partUnit() {
    constexpr size_t kUnits = 32, kLen = kUnits * kUnit;
    const bool split = expectSplit();
    char* a = nullptr;
    void* blk = nullptr;
    if (int r = prepare(a, kUnits, 1, &blk, true)) return r == 2 ? 0 : 1;

    CHECK(waitOut(a, kLen, 64, 120), "the block never left RSS: %zu of %zu pages resident",
          resident(a, kLen), kLen / kPage);
    CHECK(waitOut(a - kUnit, kUnit, 16, 60), "the 4 KiB pages before unit 0 never left RSS");
    printCounts("cold");
    Counts c0 = counts();
    if (!split) {
        // A fault on the last 4 KiB page before unit 0 prefetches its
        // neighbours (SMASH_PREFETCH_WINDOW, 2): never into a unit, which
        // would allocate the whole 2 MiB for the one page restored there.
        volatile char s0 = a[-static_cast<long>(kPage) + 3];
        (void)s0;
        CHECK(resident(a, kUnit) == 0, "a fault next to unit 0 brought %zu of its pages back",
              resident(a, kUnit));
        CHECK(counts().restored == c0.restored, "a fault next to unit 0 restored a unit");
        const long bl = bad(a - kUnit, kUnit, 1 ^ 0x5555);
        CHECK(bl < 0, "4 KiB page %ld before unit 0 reads wrong", bl);
        c0 = counts();
    }
    if (!split) CHECK(c0.compressed >= kUnits * 3 / 4, "only %llu units compressed as units",
                      (unsigned long long)c0.compressed);
    const long huge_cold = rollupHugeKib();

    // One byte in each of units 0..3.
    long fb0 = thpFallbacks();
    volatile char sink = 0;
    for (size_t u = 0; u < 4; ++u) sink = sink + a[u * kUnit + 7 * kPage + 5];
    const long huge_read = rollupHugeKib();
    size_t res4 = 0;
    for (size_t u = 0; u < 4; ++u) res4 += resident(a + u * kUnit, kUnit);
    const size_t rest = resident(a + 4 * kUnit, kLen - 4 * kUnit);
    const size_t h4 = hugeUnits(a, 0, 4);
    std::printf("  read 1 byte in 4 units: %zu pages of them resident, %zu huge, AnonHugePages %ld -> %ld KiB, "
                "%zu pages of the other units\n", res4, h4, huge_cold, huge_read, rest);
    if (split) {
        CHECK(h4 == 0 && huge_read - huge_cold < 2048, "a split restore brought back a huge page");
        CHECK(res4 <= 4 * 8, "a split restore brought back %zu pages for 4 reads", res4);
    } else {
        const bool declined = thpFallbacks() != fb0;
        CHECK(h4 == 4 || declined, "only %zu of 4 units came back as huge pages", h4);
        CHECK(huge_read - huge_cold >= 4 * 2048 || declined, "AnonHugePages grew %ld KiB, want >= 8192",
              huge_read - huge_cold);
        CHECK(res4 == 4 * kUnitPages, "the 4 units are not whole: %zu of %zu pages resident", res4, 4 * kUnitPages);
        CHECK(rest <= 64, "%zu pages of units nobody touched came back", rest);
        CHECK(counts().restored - c0.restored == 4, "%llu unit restores for 4 units",
              (unsigned long long)(counts().restored - c0.restored));
    }
    long b = bad(a, 4 * kUnit, 1);
    CHECK(b < 0, "page %ld of units 0..3 reads wrong", b);

    // A write fault restores its unit too (an atomic add of 0 writes).
    {
        const Counts cw = counts();
        __atomic_fetch_add(reinterpret_cast<uint64_t*>(a + 4 * kUnit + 9 * kPage), 0, __ATOMIC_RELAXED);
        if (!split) {
            CHECK(resident(a + 4 * kUnit, kUnit) == kUnitPages, "a write fault did not restore its whole unit");
            CHECK(unitIsHuge(a + 4 * kUnit), "a write fault did not bring its unit back as a huge page");
            CHECK(counts().restored - cw.restored == 1, "the write fault took %llu unit restores",
                  (unsigned long long)(counts().restored - cw.restored));
        }
        b = bad(a + 4 * kUnit, kUnit, 1, 4 * kUnitPages);
        CHECK(b < 0, "page %ld of unit 4 reads wrong after the write fault", b);
    }

    // 16 threads read units 5..15 at once.
    {
        fb0 = thpFallbacks();
        const Counts cr = counts();
        std::atomic<int> go{0}, wrong{0};
        std::set<size_t> touched;
        std::vector<std::thread> ts;
        std::vector<std::vector<size_t>> plan(16);
        for (size_t t = 0; t < 16; ++t)
            for (size_t k = 0; k < 64; ++k) {
                const size_t u = 5 + (t * 7 + k * 13) % 11;
                const size_t pg = u * kUnitPages + (t * 31 + k * 101) % kUnitPages;
                plan[t].push_back(pg);
                touched.insert(u);
            }
        for (size_t t = 0; t < 16; ++t)
            ts.emplace_back([&, t] {
                while (!go.load()) {}
                for (size_t pg : plan[t]) {
                    const auto* w = reinterpret_cast<const uint64_t*>(a + pg * kPage);
                    if (w[(pg * 7) % (kPage / 8)] != pattern(pg, 1)) wrong.fetch_add(1);
                }
            });
        go.store(1);
        for (auto& t : ts) t.join();
        CHECK(wrong.load() == 0, "%d concurrent reads read wrong data", wrong.load());
        b = bad(a + 5 * kUnit, 11 * kUnit, 1, 5 * kUnitPages);
        CHECK(b < 0, "page %ld of units 5..15 reads wrong after the concurrent reads", b);
        if (!split) {
            const uint64_t r = counts().restored - cr.restored;
            CHECK(r == touched.size(), "%llu unit restores for %zu units read by 16 threads", (unsigned long long)r,
                  touched.size());
            const size_t h = hugeUnits(a, 5, 11);
            CHECK(h == 11 || thpFallbacks() != fb0, "only %zu of 11 units read concurrently came back as huge pages",
                  h);
        }
    }

    // Everything cold again, then all of it back.
    fb0 = thpFallbacks();
    CHECK(waitOut(a, kLen, 64, 120), "the block never left RSS a second time: %zu pages resident",
          resident(a, kLen));
    printCounts("cold again");
    if (!split) CHECK(counts().compressed >= c0.compressed + 12, "units restored once were not compressed again");
    b = bad(a, kLen, 1);
    CHECK(b < 0, "page %ld reads wrong after the second round", b);
    if (!split) CHECK(hugeUnits(a, 0, kUnits) == kUnits || thpFallbacks() != fb0,
                      "after reading everything only %zu units are huge", hugeUnits(a, 0, kUnits));

    // Freed while compressed (go cold first), then calloc'd: zeros.
    CHECK(waitOut(a, kLen, 64, 120), "the block never left RSS a third time");
    std::free(blk);
    auto* z = static_cast<uint64_t*>(std::calloc(1, kLen + kUnit));
    CHECK(z, "calloc");
    for (size_t i = 0; i < (kLen + kUnit) / 8; ++i) CHECK(z[i] == 0, "calloc'd word %zu is %llx", i,
                                                          (unsigned long long)z[i]);
    std::free(z);
    printCounts("end");
    std::printf("thp_unit_test unit%s: all checks passed\n", split ? " (split)" : "");
    return 0;
}

int partMixed() {
    // Units 0..3: one random page each. Unit 4: all random. Units 5..6: one
    // page read every 200 ms. Units 7..10: plain.
    constexpr size_t kUnits = 11;
    char* a = nullptr;
    if (int r = prepare(a, kUnits, 2)) return r == 2 ? 0 : 1;
    uint64_t x = 88172645463325252ull;
    auto rnd = [&] { x ^= x << 13; x ^= x >> 7; x ^= x << 17; return x; };
    std::vector<uint64_t> saved;
    for (size_t u = 0; u < 4; ++u) {
        auto* w = reinterpret_cast<uint64_t*>(a + u * kUnit + 100 * kPage);
        for (size_t k = 0; k < kPage / 8; ++k) saved.push_back(w[k] = rnd());
    }
    std::vector<uint64_t> saved4(kUnit / 8);
    for (size_t k = 0; k < kUnit / 8; ++k) reinterpret_cast<uint64_t*>(a + 4 * kUnit)[k] = saved4[k] = rnd();
    CHECK(hugeUnits(a, 0, kUnits) >= kUnits - 2, "the rewrite lost huge pages");
    std::atomic<bool> stop{false};
    std::thread hot([&] {
        while (!stop.load()) {
            volatile char s = a[5 * kUnit + 3 * kPage] + a[6 * kUnit + 300 * kPage];
            (void)s;
            usleep(200 * 1000);
        }
    });
    // Units 0..3 and 7..10 leave RSS; 4, 5 and 6 stay.
    bool out = false;
    for (int t = 0; t < 1200 && !out; ++t) {
        out = resident(a, 4 * kUnit) <= 64 && resident(a + 7 * kUnit, 4 * kUnit) <= 64;
        if (!out) usleep(100 * 1000);
    }
    printCounts("cold");
    CHECK(out, "units with one random page, or plain units, never left RSS (%zu, %zu pages)",
          resident(a, 4 * kUnit), resident(a + 7 * kUnit, 4 * kUnit));
    // A few more ticks for the random unit to have been tried.
    sleep(4);
    const Counts c = counts();
    printCounts("later");
    CHECK(c.rolled_back >= 1, "the unit of random bytes was never tried and rolled back");
    CHECK(resident(a + 4 * kUnit, kUnit) == kUnitPages && unitIsHuge(a + 4 * kUnit),
          "the rolled-back unit is not one resident huge page (%zu pages resident)", resident(a + 4 * kUnit, kUnit));
    CHECK(unitIsHuge(a + 5 * kUnit) && unitIsHuge(a + 6 * kUnit), "a unit with a hot page is not a huge page");
    CHECK(resident(a + 5 * kUnit, 2 * kUnit) == 2 * kUnitPages, "a unit with a hot page lost pages");
    stop.store(true);
    hot.join();
    // Read back: the random pages exactly.
    size_t s = 0;
    for (size_t u = 0; u < 4; ++u) {
        const auto* w = reinterpret_cast<const uint64_t*>(a + u * kUnit + 100 * kPage);
        for (size_t k = 0; k < kPage / 8; ++k) CHECK(w[k] == saved[s++], "random page of unit %zu word %zu", u, k);
        CHECK(unitIsHuge(a + u * kUnit), "unit %zu with a random page came back split", u);
    }
    for (size_t k = 0; k < kUnit / 8; ++k)
        CHECK(reinterpret_cast<const uint64_t*>(a + 4 * kUnit)[k] == saved4[k], "random unit word %zu", k);
    for (size_t u : {0ul, 1ul, 2ul, 3ul, 5ul, 6ul, 7ul, 8ul, 9ul, 10ul})
        for (size_t pg = 0; pg < kUnitPages; ++pg) {
            if (u < 4 && pg == 100) continue;
            const size_t gp = u * kUnitPages + pg;
            CHECK(*reinterpret_cast<const uint64_t*>(a + gp * kPage) == pattern(gp, 2), "unit %zu page %zu", u, pg);
        }
    std::printf("thp_unit_test mixed: all checks passed\n");
    return 0;
}

int partSyscall() {
    constexpr size_t kUnits = 8, kLen = kUnits * kUnit;
    char* a = nullptr;
    if (int r = prepare(a, kUnits, 3)) return r == 2 ? 0 : 1;
    CHECK(waitOut(a, kLen, 64, 120), "the block never left RSS");
    printCounts("cold");
    char path[] = "/tmp/thp_unit_XXXXXX";
    const int fd = mkstemp(path);
    CHECK(fd >= 0, "mkstemp");
    unlink(path);
    // write() of compressed unit 0.
    CHECK(write(fd, a, kUnit) == static_cast<ssize_t>(kUnit), "write from a compressed unit: %s", strerror(errno));
    std::vector<char> back(kUnit);
    CHECK(pread(fd, back.data(), kUnit, 0) == static_cast<ssize_t>(kUnit), "pread back");
    long b = bad(back.data(), kUnit, 3);
    CHECK(b < 0, "page %ld written from a compressed unit is wrong", b);
    CHECK(unitIsHuge(a), "unit 0 did not come back as a huge page after write()");
    // read() into compressed unit 1, 64 KiB at page 3: the file holds unit 0's
    // bytes there, which are unit 1's pattern shifted - so compute.
    std::vector<char> src(64 << 10);
    fill(src.data(), src.size(), 99);
    CHECK(pwrite(fd, src.data(), src.size(), kUnit) == static_cast<ssize_t>(src.size()), "pwrite");
    CHECK(lseek(fd, kUnit, SEEK_SET) == static_cast<off_t>(kUnit), "lseek");
    CHECK(read(fd, a + kUnit + 3 * kPage, src.size()) == static_cast<ssize_t>(src.size()),
          "read into a compressed unit: %s", strerror(errno));
    CHECK(std::memcmp(a + kUnit + 3 * kPage, src.data(), src.size()) == 0, "read() into a compressed unit lost bytes");
    b = bad(a + kUnit, 3 * kPage, 3, kUnitPages);
    CHECK(b < 0, "page %ld of unit 1 before the read() is wrong", b);
    b = bad(a + kUnit + 3 * kPage + src.size(), kUnit - 3 * kPage - src.size(), 3,
            kUnitPages + 3 + src.size() / kPage);
    CHECK(b < 0, "page %ld of unit 1 after the read() is wrong", b);
    CHECK(unitIsHuge(a + kUnit), "unit 1 did not come back as a huge page after read()");
    // pread() across the boundary of compressed units 3 and 4.
    char* at = a + 4 * kUnit - 32 * kPage;
    CHECK(pread(fd, at, kUnit / 4, 0) == static_cast<ssize_t>(kUnit / 4), "pread across units: %s",
          strerror(errno));
    b = bad(at, kUnit / 4, 3);
    CHECK(b < 0, "page %ld of a pread() across two compressed units is wrong", b);
    CHECK(unitIsHuge(a + 3 * kUnit) && unitIsHuge(a + 4 * kUnit), "units 3 and 4 did not come back as huge pages");
    close(fd);
    b = bad(a + 5 * kUnit, 3 * kUnit, 3, 5 * kUnitPages);
    CHECK(b < 0, "page %ld of the untouched units is wrong", b);
    printCounts("end");
    std::printf("thp_unit_test syscall: all checks passed\n");
    return 0;
}

int partFork() {
    constexpr size_t kUnits = 8, kLen = kUnits * kUnit;
    char* a = nullptr;
    if (int r = prepare(a, kUnits, 4)) return r == 2 ? 0 : 1;
    CHECK(waitOut(a, kLen, 64, 120), "the block never left RSS");
    printCounts("cold");
    const long fb0 = thpFallbacks();
    const pid_t pid = fork();
    CHECK(pid >= 0, "fork");
    if (pid == 0) {
        if (bad(a, kLen, 4) >= 0) _exit(3);
        __atomic_fetch_add(reinterpret_cast<uint64_t*>(a + kUnit), 0, __ATOMIC_RELAXED);
        if (bad(a + kUnit, kUnit, 4, kUnitPages) >= 0) _exit(4);
        _exit(0);
    }
    int st = 0;
    CHECK(waitpid(pid, &st, 0) == pid, "waitpid");
    CHECK(WIFEXITED(st) && WEXITSTATUS(st) == 0, "the child read compressed units wrong (status %d)", st);
    CHECK(resident(a, kLen) <= kLen / kPage / 20 + 64, "the child's reads brought the parent's units back");
    const long b = bad(a, kLen, 4);
    CHECK(b < 0, "page %ld reads wrong in the parent after the child", b);
    CHECK(hugeUnits(a, 0, kUnits) == kUnits || thpFallbacks() != fb0, "the parent's units came back split");
    std::printf("thp_unit_test fork: all checks passed\n");
    return 0;
}

int partShrink() {
    constexpr size_t kUnits = 16, kLen = kUnits * kUnit;
    char* a = nullptr;
    void* blk = nullptr;
    if (int r = prepare(a, kUnits, 5, &blk)) return r == 2 ? 0 : 1;
    CHECK(waitOut(a, kLen, 64, 120), "the block never left RSS");
    printCounts("cold");
    // Keep the block up to the middle of unit 4.
    const size_t keep = static_cast<size_t>(a - static_cast<char*>(blk)) + 4 * kUnit + kUnit / 2;
    void* q = std::realloc(blk, keep);
    CHECK(q == blk, "realloc moved a shrinking block");
    long b = bad(a, 4 * kUnit + kUnit / 2, 5);
    CHECK(b < 0, "page %ld kept by the shrink reads wrong", b);
    CHECK(waitOut(a, 4 * kUnit + kUnit / 2, (4 * kUnit + kUnit / 2) / kPage / 10, 120),
          "the kept part never left RSS again");
    b = bad(a, 4 * kUnit + kUnit / 2, 5);
    CHECK(b < 0, "page %ld kept by the shrink reads wrong after another round", b);
    std::free(q);
    printCounts("end");
    std::printf("thp_unit_test shrink: all checks passed\n");
    return 0;
}

int partFallback() {
    if (std::getenv("THP_UNIT_CHILD")) {
        // The decision is made at the first tick with anything in the arena.
        auto* p = static_cast<char*>(std::malloc(8u << 20));
        if (!p) return 6;
        std::memset(p, 1, 8u << 20);
        sleep(4);
        const Counts c = counts();
        std::printf("  child: active=%d\n", c.active);
        return c.active == 0 ? 0 : 5;
    }
    int pfd[2];
    CHECK(pipe(pfd) == 0, "pipe");
    const pid_t pid = fork();
    CHECK(pid >= 0, "fork");
    if (pid == 0) {
        dup2(pfd[1], 2);
        close(pfd[0]);
        setenv("THP_UNIT_CHILD", "1", 1);
        execl("/proc/self/exe", "thp_unit_test", static_cast<char*>(nullptr));
        _exit(127);
    }
    close(pfd[1]);
    std::string err;
    char buf[4096];
    for (ssize_t n; (n = read(pfd[0], buf, sizeof(buf))) > 0;) err.append(buf, static_cast<size_t>(n));
    int st = 0;
    waitpid(pid, &st, 0);
    std::printf("  child stderr: %s", err.c_str());
    CHECK(WIFEXITED(st) && WEXITSTATUS(st) == 0, "the child says the option is active (status %d)", st);
    CHECK(err.find("SMASH_THP_UNIT=1 ignored") != std::string::npos, "no warning that SMASH_THP_UNIT=1 is ignored");
    std::printf("thp_unit_test fallback: all checks passed\n");
    return 0;
}

}  // namespace

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    g_info = reinterpret_cast<UnitInfo>(dlsym(RTLD_DEFAULT, "smash_thp_unit_info"));
    const char* part = std::getenv("THP_UNIT_PART");
    const std::string p = part ? part : "unit";
    if (p == "unit") return partUnit();
    if (p == "mixed") return partMixed();
    if (p == "syscall") return partSyscall();
    if (p == "fork") return partFork();
    if (p == "shrink") return partShrink();
    if (p == "fallback") return partFallback();
    std::printf("FAIL: unknown THP_UNIT_PART=%s\n", part);
    return 1;
}
