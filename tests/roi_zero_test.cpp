// roi_zero_test.cpp - a page that is all zero is compressed whatever the ROI
// gate thinks of its bucket (issue #50).
//
// The gate declines a page from its bucket's window of compression ratios,
// before the page is read. A page that is all zero is stored as a mark: no
// codec runs and there is no blob, and it is never recorded in the window.
// So the window says nothing about it, and a probe (every 64th declined
// page) that turns out to be zero does not restart the window. Zero pages
// were declined with the rest of their bucket: after one block of random
// bytes, the zero pages of later blocks from the same call site were
// compressed at one in 64 per pass, and not at all where they alternated
// with random pages, because every probe landed on a random one. A declined
// page that reads as zero now goes on to the snapshot, which decides.
//
// Reading a page is not free of consequences (it brings a swapped page back,
// sets the accessed bit the kernel and the scanning trackers go by, and can
// take a NUMA hinting fault), so the gate reads a declined page once for
// each time it may have changed: a page that was declined before and has
// not been reported written since is not read again.
//
// ROIZ_PART=blocks (large-only mode), one call site for every block:
//   1. 16 MiB of random bytes, kept: the gate must decline most of it.
//   2. A block whose even pages are zero and whose odd pages are random,
//      and a block that is all zero. At least 95 % of the zero pages of
//      each must be compressed within 90 s, and at most 64 of the random
//      pages. With ROIZ_ONE_WINDOW=1 (one worker) the counter roi_zero must
//      account for them: the gate declined them and they went on.
//   3. Every page of both blocks is read back: a zero page is zero, a
//      random page holds what was written. Both blocks are freed.
//   4. The random block is declined once more, every page of it. With
//      ROIZ_LOOKS=once (a tracker that reports writes) the gate must have
//      read at most 64 pages meanwhile; with ROIZ_LOOKS=always (no write
//      tracking: nothing says a page did not change) at least half of them.
//   5. The even pages of the random block are overwritten with zeros. They
//      were declined and found not zero; the write must bring the gate
//      back to them: at least 95 % compressed within 150 s. Pages still
//      not compressed after 50 s are written again, at most twice: under
//      a scanning tracker one write can be taken for the compressor's own
//      read. The odd pages must still hold their random bytes.
//   6. A block of zeros with a second thread writing a mark into a page the
//      moment it sees it being compressed. The look at the page and its
//      snapshot are not one step, so a page can read as zero and be written
//      before the snapshot; the snapshot must win. No mark may be lost,
//      and no other byte of the block may change. Each page is written
//      once more before its mark is read: a page compressed in standard
//      mode keeps its bytes readable until it is dropped, and only a write
//      makes the stored copy the one that counts. With ROIZ_MIN_STALE=N
//      the counter roi_stale must show at least N pages written between
//      the look and the snapshot (the registration whose snapshot follows
//      a slow mprotect of a writable page).
//
// ROIZ_PART=slots (full mode): 16 MiB of 1024-byte objects with random
// bytes, from one call site; the gate declines their bucket. Then every
// object on an odd page is freed. Those pages hold only free slots, which
// the snapshot zeroes, so they are zero pages to the compressor: at least
// 90 % of them must be compressed within 90 s. The objects kept must hold
// their bytes, and the slots must be usable again.
//
// ROIZ_NEED_TRACK=idle|yt: the registration is for that tracker, which needs
// root (and yt its kernel module). When the stats line names another one,
// the test skips. Under a scanning tracker a compressor thread's read of a
// page is what the page's next scan reports, and step 4 is what shows that
// phase 1 does not let such a read clear the page's back-off. Measured as
// root, reads for declines in step 4: 8192 for 8064 when phase 1 ignores
// the record (every declined page is read again each time); 128-134 for
// about 4100 when only the gate's read is recorded and a probe's snapshot
// is not (issue #52: the pages the passes before had probed); 0 with both.
//
// ROIZ_WANT_HUGE=1 (with SMASH_THP=1): the same on huge pages; the test
// skips unless the random block is on them (AnonHugePages). A huge page has
// one accessed bit for its 512 pages, so there a compressor thread's read
// of one page is reported for all of them, and the record has to cover
// them all: kept for the one page only, every probe took the back-off
// count of the other 511 pages of its huge page.
//
// The counters are the pipeline line's (SMASH_STATS_SIGNAL=1, SIGUSR2),
// read from a file standing in for stderr.
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <dlfcn.h>
#include <pthread.h>
#include <string>
#include <unistd.h>
#include <vector>

namespace {
using info_fn = int (*)(const void*, int*, int*, int*);
using arena_fn = int (*)(const void*);
info_fn page_info = nullptr;
arena_fn arena_for = nullptr;
int g_log_fd = -1, g_saved_err = -1;

constexpr size_t kPage = 4096;
constexpr size_t kBlock = size_t{16} << 20;
constexpr size_t kPages = kBlock / kPage;

struct Rng {
    uint64_t s;
    uint64_t next() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return s; }
};

// The one call site every block comes from. The store after the call keeps
// it from becoming a tail call, which would make the callers' lines the
// call sites.
bool g_want_huge = false;   // ROIZ_WANT_HUGE=1
__attribute__((noinline)) unsigned char* allocBlock(size_t bytes) {
    auto* p = static_cast<unsigned char*>(std::malloc(bytes));
    if (p) *static_cast<volatile unsigned char*>(p) = 0;
    return p;
}

long anonHugeKib() {
    FILE* f = std::fopen("/proc/self/smaps_rollup", "r");
    if (!f) return -1;
    char line[256];
    long v = -1;
    while (std::fgets(line, sizeof line, f))
        if (!std::strncmp(line, "AnonHugePages:", 14)) { v = std::atol(line + 14); break; }
    std::fclose(f);
    return v;
}

// The same for the small objects of the slots part.
constexpr size_t kObj = 1024;
__attribute__((noinline)) unsigned char* allocObject() {
    auto* p = static_cast<unsigned char*>(std::malloc(kObj));
    if (p) *static_cast<volatile unsigned char*>(p) = 0;
    return p;
}

struct Counters {
    unsigned long long tried = 0, stored = 0, gate_fail = 0, roi_skip = 0, roi_zero = 0, roi_look = 0,
                       roi_stale = 0;
    bool ok = false, has_zero = false;
};

Counters readCounters() {
    raise(SIGUSR2);
    usleep(100 * 1000);
    std::string lg;
    char buf[8192];
    lseek(g_log_fd, 0, SEEK_SET);
    for (ssize_t n; (n = read(g_log_fd, buf, sizeof buf)) > 0;) lg.append(buf, static_cast<size_t>(n));
    Counters c;
    const size_t at = lg.rfind("compress tried=");
    if (at == std::string::npos) return c;
    c.ok = std::sscanf(lg.c_str() + at, "compress tried=%llu stored=%llu gate_fail=%llu roi_skip=%llu",
                       &c.tried, &c.stored, &c.gate_fail, &c.roi_skip) == 4;
    const size_t z = lg.find("roi_zero=", at);
    if (z != std::string::npos)
        c.has_zero = std::sscanf(lg.c_str() + z, "roi_zero=%llu roi_look=%llu roi_stale=%llu", &c.roi_zero,
                                 &c.roi_look, &c.roi_stale) == 3;
    return c;
}

// The write tracker the stats line names (track=...), after a SIGUSR2.
std::string trackerInUse() {
    raise(SIGUSR2);
    usleep(100 * 1000);
    std::string lg;
    char buf[8192];
    lseek(g_log_fd, 0, SEEK_SET);
    for (ssize_t n; (n = read(g_log_fd, buf, sizeof buf)) > 0;) lg.append(buf, static_cast<size_t>(n));
    const size_t at = lg.rfind(" track=");
    if (at == std::string::npos) return "";
    const size_t b = at + 7, e = lg.find_first_of(" \n", b);
    return lg.substr(b, e == std::string::npos ? std::string::npos : e - b);
}

bool isCompressed(const unsigned char* page) {
    int st = 0, cold = 0, rc = 0;
    return page_info(page, &st, &cold, &rc) == 0 && (st == 2 || st == 5);
}

// Compressed pages among those of [p, p + bytes) with page number % 2 == parity
// (parity 2: every page).
size_t compressedPages(const unsigned char* p, size_t bytes, int parity) {
    size_t c = 0;
    for (size_t pg = 0; pg < bytes / kPage; ++pg)
        if ((parity == 2 || static_cast<int>(pg & 1) == parity) && isCompressed(p + pg * kPage)) ++c;
    return c;
}

bool pageIsZero(const unsigned char* page) {
    for (size_t k = 0; k < kPage; ++k)
        if (page[k]) return false;
    return true;
}

int fail(const char* fmt, unsigned long long a = 0, unsigned long long b = 0, unsigned long long c = 0) {
    dprintf(g_saved_err, "FAIL: ");
    dprintf(g_saved_err, fmt, a, b, c);
    dprintf(g_saved_err, "\n");
    return 1;
}

unsigned long long envNum(const char* name, unsigned long long dflt) {
    const char* v = std::getenv(name);
    return v && *v ? std::strtoull(v, nullptr, 10) : dflt;
}

// ── Part 4: a writer that strikes a page while it is being compressed ───────
constexpr size_t kStrikePages = 2048;
constexpr size_t kMarkOffset = 1000;   // bytes into the page; 8-byte aligned
constexpr size_t kMark2Offset = 3000;  // the main thread's, written before the check
struct Striker {
    unsigned char* base = nullptr;
    std::atomic<bool> stop{false};
    std::atomic<uint8_t> struck[kStrikePages] = {};
    std::atomic<size_t> strikes{0};
};

uint64_t markOf(size_t pg) { return 0xA5A5000000000000ull | (pg + 1); }

void* strikerMain(void* arg) {
    auto* s = static_cast<Striker*>(arg);
    while (!s->stop.load(std::memory_order_relaxed)) {
        for (size_t pg = 0; pg < kStrikePages; ++pg) {
            if (s->struck[pg].load(std::memory_order_relaxed)) continue;
            int st = 0, cold = 0, rc = 0;
            if (page_info(s->base + pg * kPage, &st, &cold, &rc) != 0 || st != 3) continue;   // 3 = COMPRESSING
            const uint64_t m = markOf(pg);
            std::memcpy(s->base + pg * kPage + kMarkOffset, &m, sizeof m);
            s->struck[pg].store(1, std::memory_order_release);
            s->strikes.fetch_add(1, std::memory_order_relaxed);
        }
    }
    return nullptr;
}

int declineRandomBlock(unsigned char* r, const char* what, size_t pages) {
    Counters c;
    int waited = 0;
    for (; waited < 60; ++waited) {
        sleep(1);
        c = readCounters();
        if (c.ok && c.roi_skip >= pages / 2) break;
    }
    dprintf(g_saved_err, "%s, %d s: tried=%llu stored=%llu gate_fail=%llu roi_skip=%llu of %zu pages\n", what,
            waited + 1, c.tried, c.stored, c.gate_fail, c.roi_skip, pages);
    (void)r;
    if (!c.ok) return fail("no pipeline line on SIGUSR2 (SMASH_STATS_SIGNAL=1 not set?)");
    if (!c.has_zero)
        return fail("the pipeline line has no roi_zero=, roi_look= and roi_stale= counters: this libsmash is older "
                    "than the test");
    if (c.roi_skip < pages / 2)
        return fail("the ROI gate declined %llu of %llu random pages in 60 s (the codec failed on %llu): "
                    "the bucket is not declined, so the test cannot say anything", c.roi_skip, pages, c.gate_fail);
    return 0;
}

int partBlocks() {
    const bool one_window = envNum("ROIZ_ONE_WINDOW", 0) != 0;
    const unsigned long long min_stale = envNum("ROIZ_MIN_STALE", 0);
    const char* looks = std::getenv("ROIZ_LOOKS");
    const bool looks_once = looks && std::strcmp(looks, "once") == 0;
    const bool looks_always = looks && std::strcmp(looks, "always") == 0;

    // 1. Random bytes: the bucket is declined, and stays so (the block is kept).
    Rng rng{88172645463325252ull};
    const Rng random_seed = rng;
    unsigned char* r = allocBlock(kBlock);
    if (!r) return fail("malloc");
    for (size_t i = 0; i < kBlock / 8; ++i) reinterpret_cast<uint64_t*>(r)[i] = rng.next();
    const int arena_r = arena_for(r);
    if (g_want_huge) {
        // Every block of the test is a multiple of 2 MiB, and the arena
        // hands them out back to back from a 2 MiB boundary: no huge page
        // holds pages of two blocks, so nothing the test does to one block
        // is an access to its neighbour's last pages. Checked, not assumed.
        const long huge = anonHugeKib();
        dprintf(g_saved_err, "random block at %p, AnonHugePages %ld KiB\n", static_cast<void*>(r), huge);
        if (reinterpret_cast<uintptr_t>(r) % (size_t{2} << 20) != 0) {
            dprintf(g_saved_err, "SKIP: the block does not start on a 2 MiB boundary\n");
            return 77;
        }
        if (huge < static_cast<long>(kBlock / 1024)) {
            dprintf(g_saved_err, "SKIP: the kernel gave the %zu KiB block %ld KiB of huge pages (THP off, or no "
                    "memory)\n", kBlock / 1024, huge);
            return 77;
        }
    }
    if (int rc = declineRandomBlock(r, "random block", kPages)) return rc;
    const Counters c1 = readCounters();

    // 2. Zero pages from the same call site: alternating with random pages,
    //    and a whole block of them. Written, so every page is resident.
    unsigned char* m = allocBlock(kBlock);
    unsigned char* z = allocBlock(kBlock);
    if (!m || !z) return fail("malloc");
    const Rng mixed_seed = rng;
    for (size_t pg = 0; pg < kPages; ++pg) {
        auto* w = reinterpret_cast<uint64_t*>(m + pg * kPage);
        for (size_t k = 0; k < kPage / 8; ++k) w[k] = (pg & 1) ? rng.next() : 0;
    }
    std::memset(z, 0, kBlock);
    if (arena_for(m) != arena_r || arena_for(z) != arena_r)
        return fail("one call site's blocks are in arenas %llu, %llu and %llu: they do not share a bucket",
                    static_cast<unsigned long long>(arena_r), static_cast<unsigned long long>(arena_for(m)),
                    static_cast<unsigned long long>(arena_for(z)));
    const size_t want_m = kPages / 2 * 95 / 100, want_z = kPages * 95 / 100;
    size_t zm = 0, zz = 0;
    int waited = 0;
    for (; waited < 90; ++waited) {
        sleep(1);
        zm = compressedPages(m, kBlock, 0);
        zz = compressedPages(z, kBlock, 2);
        if (zm >= want_m && zz >= want_z) break;
    }
    const size_t rm = compressedPages(m, kBlock, 1);
    const Counters c2 = readCounters();
    const unsigned long long by_gate = c2.roi_zero - c1.roi_zero;
    dprintf(g_saved_err, "zero pages after %d s: %zu of %zu in the alternating block, %zu of %zu in the zero block; "
            "%zu random pages compressed; roi_zero=%llu roi_skip=%llu\n", waited + 1, zm, kPages / 2, zz, kPages, rm,
            by_gate, c2.roi_skip - c1.roi_skip);
    if (zm < want_m)
        return fail("%llu of %llu zero pages that alternate with random pages were compressed in 90 s: "
                    "the declined bucket keeps them", zm, kPages / 2);
    if (zz < want_z)
        return fail("%llu of %llu pages of the zero block were compressed in 90 s: the declined bucket keeps them",
                    zz, kPages);
    if (rm > 64)
        return fail("%llu of %llu random pages were compressed: the gate is not declining them", rm, kPages / 2);
    if (one_window && by_gate < want_m + want_z)
        return fail("roi_zero says %llu declined pages went on as zero pages, for %llu compressed: they were not "
                    "declined, so the bucket was not the one the random block switched off", by_gate, zm + zz);

    // 3. What was written is what is read.
    Rng chk = mixed_seed;
    for (size_t pg = 0; pg < kPages; ++pg) {
        const unsigned char* page = m + pg * kPage;
        if (!(pg & 1)) {
            if (!pageIsZero(page)) return fail("alternating block: zero page %llu is not zero after its restore", pg);
            continue;
        }
        auto* w = reinterpret_cast<const uint64_t*>(page);
        for (size_t k = 0; k < kPage / 8; ++k)
            if (w[k] != chk.next()) return fail("alternating block: random page %llu changed at word %llu", pg, k);
    }
    for (size_t pg = 0; pg < kPages; ++pg)
        if (!pageIsZero(z + pg * kPage)) return fail("zero block: page %llu is not zero after its restore", pg);
    std::free(m);
    std::free(z);

    // 4. Declined again: read again only where nothing says it is unchanged.
    const Counters c3 = readCounters();
    Counters c4 = c3;
    for (waited = 0; waited < 150; ++waited) {
        sleep(1);
        c4 = readCounters();
        if (c4.roi_skip - c3.roi_skip >= kPages) break;
    }
    const unsigned long long again = c4.roi_skip - c3.roi_skip, reread = c4.roi_look - c3.roi_look;
    dprintf(g_saved_err, "random block declined again, %d s: %llu declines, %llu pages read by the gate\n",
            waited + 1, again, reread);
    if (again < kPages)
        return fail("the random block was declined %llu times in 150 s, fewer than its %llu pages: nothing to "
                    "compare the gate's reads with", again, kPages);
    if (looks_once && reread > 64)
        return fail("the gate read %llu pages while it declined %llu that were not written since it last read "
                    "them (limit 64)", reread, again);
    if (looks_always && reread < kPages / 2)
        return fail("the gate read %llu pages while it declined %llu, with no write tracking to say they are "
                    "unchanged", reread, again);

    // 5. A declined page overwritten with zeros. Written again, up to twice,
    //    where it is still not compressed after 50 s: under a scanning
    //    tracker a write within one scan of a compressor thread's read of
    //    the page (of its huge page, on huge pages) is taken for that read.
    size_t zr = 0;
    int rounds = 0, total_wait = 0;
    for (; rounds < 3 && zr < want_m; ++rounds) {
        for (size_t pg = 0; pg < kPages; pg += 2)
            if (!isCompressed(r + pg * kPage)) std::memset(r + pg * kPage, 0, kPage);
        for (waited = 0; waited < 50; ++waited) {
            sleep(1);
            ++total_wait;
            zr = compressedPages(r, kBlock, 0);
            if (zr >= want_m) break;
        }
    }
    const Counters c5 = readCounters();
    dprintf(g_saved_err, "random block, even pages overwritten with zeros, %d s and %d write(s): %zu of %zu "
            "compressed; roi_zero=%llu roi_look=%llu\n", total_wait, rounds, zr, kPages / 2,
            c5.roi_zero - c4.roi_zero, c5.roi_look - c4.roi_look);
    if (zr < want_m)
        return fail("%llu of %llu declined pages were compressed in 150 s after they were overwritten with "
                    "zeros three times: the gate did not come back to them", zr, kPages / 2);
    Rng rchk = random_seed;
    for (size_t pg = 0; pg < kPages; ++pg) {
        const unsigned char* page = r + pg * kPage;
        auto* w = reinterpret_cast<const uint64_t*>(page);
        for (size_t k = 0; k < kPage / 8; ++k) {
            const uint64_t want = rchk.next();
            if (pg & 1) {
                if (w[k] != want) return fail("random block: page %llu changed at word %llu", pg, k);
            } else if (w[k] != 0) {
                return fail("random block: zeroed page %llu is not zero at word %llu", pg, k);
            }
        }
    }

    // 6. A page written between the gate's look and the snapshot.
    static Striker s;
    s.base = allocBlock(kStrikePages * kPage);
    if (!s.base) return fail("malloc");
    std::memset(s.base, 0, kStrikePages * kPage);
    const Counters c6 = readCounters();
    pthread_t th;
    if (pthread_create(&th, nullptr, strikerMain, &s) != 0) return fail("pthread_create");
    size_t settled = 0;
    for (waited = 0; waited < 60; ++waited) {
        sleep(1);
        settled = 0;
        for (size_t pg = 0; pg < kStrikePages; ++pg)
            if (s.struck[pg].load(std::memory_order_acquire) || isCompressed(s.base + pg * kPage)) ++settled;
        if (settled == kStrikePages) break;
    }
    sleep(3);   // a struck page may be on its way to a second attempt
    s.stop.store(true);
    pthread_join(th, nullptr);
    const Counters c7 = readCounters();
    const size_t strikes = s.strikes.load();
    const unsigned long long stale = c7.roi_stale - c6.roi_stale;
    size_t comp = 0, lost = 0, dirty = 0;
    for (size_t pg = 0; pg < kStrikePages; ++pg)
        if (isCompressed(s.base + pg * kPage)) ++comp;
    for (size_t pg = 0; pg < kStrikePages; ++pg) {
        unsigned char* page = s.base + pg * kPage;
        // A write first: whatever was stored for the page is what it holds
        // from here on.
        const uint64_t m2 = ~markOf(pg);
        std::memcpy(page + kMark2Offset, &m2, sizeof m2);
        uint64_t got, got2;
        std::memcpy(&got, page + kMarkOffset, sizeof got);
        std::memcpy(&got2, page + kMark2Offset, sizeof got2);
        const uint64_t want = s.struck[pg].load() ? markOf(pg) : 0;
        if (got != want || got2 != m2) ++lost;
        std::memset(page + kMarkOffset, 0, sizeof got);
        std::memset(page + kMark2Offset, 0, sizeof got2);
        if (!pageIsZero(page)) ++dirty;
    }
    dprintf(g_saved_err, "writer: %zu of %zu pages struck while being compressed, %llu of them between the look "
            "and the snapshot (roi_stale), %zu compressed at the end; %zu marks lost, %zu pages with other bytes "
            "changed\n", strikes, kStrikePages, stale, comp, lost, dirty);
    if (lost)
        return fail("%llu of %llu marks written while a page was being compressed were lost: a page that read as "
                    "zero was stored as zero after it was written", lost, strikes);
    if (dirty) return fail("%llu pages of the zero block hold bytes nobody wrote", dirty);
    if (settled < kStrikePages)
        return fail("%llu of %llu pages were neither compressed nor struck in 60 s", settled, kStrikePages);
    if (stale < min_stale)
        return fail("%llu pages were written between the gate's look and the snapshot (the writer caught %llu "
                    "being compressed): fewer than the %llu this registration needs to say anything about that "
                    "window", stale, strikes, min_stale);
    std::free(s.base);
    std::free(r);
    return 0;
}

int partSlots() {
    constexpr size_t kObjects = kBlock / kObj;
    std::vector<unsigned char*> o(kObjects);
    std::vector<uint64_t> first(kObjects);
    Rng rng{88172645463325252ull};
    int arena = -1;
    for (size_t i = 0; i < kObjects; ++i) {
        o[i] = allocObject();
        if (!o[i]) return fail("malloc");
        for (size_t k = 0; k < kObj / 8; ++k) reinterpret_cast<uint64_t*>(o[i])[k] = rng.next();
        std::memcpy(&first[i], o[i], 8);
        const int a = arena_for(o[i]);
        if (i == 0) arena = a;
        else if (a != arena) return fail("one call site's objects are in arenas %llu and %llu",
                                         static_cast<unsigned long long>(arena), static_cast<unsigned long long>(a));
    }
    if (int rc = declineRandomBlock(nullptr, "random objects", kPages)) return rc;
    const Counters c1 = readCounters();

    // Free every object that touches an odd page. A page is 4 objects, so an
    // odd page is left with free slots only, and an even one with none.
    std::vector<unsigned char*> freed_pages;
    size_t kept = 0;
    for (size_t i = 0; i < kObjects; ++i) {
        const auto a = reinterpret_cast<uintptr_t>(o[i]);
        if ((a % kPage) + kObj > kPage)
            return fail("object %llu straddles a page: the size class is not what the test assumes", i);
        if (!((a / kPage) & 1)) { ++kept; continue; }
        auto* page = reinterpret_cast<unsigned char*>(a & ~(kPage - 1));
        if (freed_pages.empty() || freed_pages.back() != page) freed_pages.push_back(page);
        std::free(o[i]);
        o[i] = nullptr;
    }
    const size_t n = freed_pages.size();
    if (n < kPages / 4 || kept < kObjects / 4)
        return fail("%llu pages of freed objects and %llu objects kept: not the layout the test assumes", n, kept);
    size_t given = 0;
    int waited = 0;
    for (; waited < 90; ++waited) {
        sleep(1);
        given = 0;
        for (unsigned char* page : freed_pages)
            if (isCompressed(page)) ++given;
        if (given >= n * 9 / 10) break;
    }
    const Counters c2 = readCounters();
    dprintf(g_saved_err, "pages holding only freed objects after %d s: %zu of %zu compressed; roi_zero=%llu "
            "roi_skip=%llu stored=%llu\n", waited + 1, given, n, c2.roi_zero - c1.roi_zero, c2.roi_skip - c1.roi_skip,
            c2.stored - c1.stored);
    if (given < n * 9 / 10)
        return fail("%llu of %llu pages holding only freed objects were compressed in 90 s: the declined bucket "
                    "keeps them resident", given, n);

    // The objects kept hold their bytes, and the freed slots can be used again.
    for (size_t i = 0; i < kObjects; ++i) {
        if (!o[i]) continue;
        uint64_t got;
        std::memcpy(&got, o[i], 8);
        if (got != first[i]) return fail("kept object %llu changed", i);
    }
    Rng again{0x9E3779B97F4A7C15ull};
    std::vector<unsigned char*> fresh;
    for (size_t i = 0; i < kObjects / 4; ++i) {
        unsigned char* p = allocObject();
        if (!p) return fail("malloc");
        for (size_t k = 0; k < kObj / 8; ++k) reinterpret_cast<uint64_t*>(p)[k] = again.next();
        fresh.push_back(p);
    }
    Rng chk{0x9E3779B97F4A7C15ull};
    for (size_t i = 0; i < fresh.size(); ++i)
        for (size_t k = 0; k < kObj / 8; ++k)
            if (reinterpret_cast<uint64_t*>(fresh[i])[k] != chk.next())
                return fail("object %llu allocated after the pages were given back does not hold its bytes", i);
    for (unsigned char* p : fresh) std::free(p);
    for (unsigned char* p : o) std::free(p);
    return 0;
}
}  // namespace

int main() {
    page_info = reinterpret_cast<info_fn>(dlsym(RTLD_DEFAULT, "smash_page_info"));
    arena_for = reinterpret_cast<arena_fn>(dlsym(RTLD_DEFAULT, "smash_arena_for"));
    if (!page_info || !arena_for) {
        std::fprintf(stderr, "FAIL: not running under libsmash (diagnostic exports missing)\n");
        return 1;
    }
    char path[] = "/tmp/smash_roi_zero_XXXXXX";
    g_log_fd = mkstemp(path);
    if (g_log_fd < 0) { std::printf("SKIP: mkstemp failed\n"); return 77; }
    unlink(path);
    g_saved_err = dup(2);
    dup2(g_log_fd, 2);

    g_want_huge = envNum("ROIZ_WANT_HUGE", 0) != 0;
    if (const char* need = std::getenv("ROIZ_NEED_TRACK"); need && *need) {
        // The tracker is chosen at the compressor's first tick with something
        // to track.
        unsigned char* first = allocBlock(kBlock);
        if (!first) return fail("malloc");
        std::memset(first, 1, kBlock);
        std::string got;
        for (int i = 0; i < 40 && (got.empty() || got == "unprobed"); ++i) {
            usleep(250 * 1000);
            got = trackerInUse();
        }
        std::free(first);
        if (got != need) {
            dprintf(g_saved_err, "SKIP: this registration needs SMASH_WRITE_TRACK=%s and the tracker in use is "
                    "'%s' (not root, or the kernel has no such tracker)\n", need, got.c_str());
            return 77;
        }
    }
    const char* part = std::getenv("ROIZ_PART");
    const int rc = (part && std::strcmp(part, "slots") == 0) ? partSlots() : partBlocks();
    if (rc == 0) dprintf(g_saved_err, "OK\n");
    return rc;
}
