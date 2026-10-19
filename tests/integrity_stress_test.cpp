// integrity_stress_test.cpp - many threads keep data in smash memory while
// the compressor compresses, reclaims and restores it; every byte must read
// back as last written.
//
// The suite checked contents one mechanism at a time, from one thread. The
// Phase B race (two workers reclaiming the same chunk; COMPRESSED pages that
// read zeros without a fault) passed all of it and was found by accident,
// when three tests happened to run at once. This test is the load that finds
// that kind of defect: data cycling through every compressor state from many
// threads at once, checked on every access and at the end.
//
// Each thread owns large blocks (1.25-3 MiB unless STRESS_BLOCK_KIB,
// compressible in SMASH_LARGE_ONLY=1) and, outside large-only mode, small
// objects. Every 4 KiB unit of a block holds (tag, version) and a pattern
// derived from them (compressible by LZ4 and zstd); every seventh version is
// all zeros (the zero-page path). The versions the threads expect live in
// mmap'd memory, outside smash. Per round of about 2 ms a thread:
//   - writes or checks units of its one hot block (never compressed);
//   - touches one unit of its other blocks: checks it, rewrites it with the
//     next version, or stores a word's own value back (a write that changes
//     nothing);
//   - checks a unit of the shared blocks, written once and read by every
//     thread (concurrent faults on one compressed page);
//   - full mode: checks or rewrites a small object, and every 3 s checks its
//     survivors -- 256-byte objects left one per page, the rest of each page
//     freed, rebuilt every 9 s (the pages the Phase B race lost);
//   - every STRESS_WAVE_SEC (default 4): rewrites all its cold blocks at once,
//     and checks them all three quarters of a wave later, so every thread's
//     pages reach each compressor phase in the same tick;
//   - now and then: pwrite()s a cold unit to a file and pread()s it back
//     into a private buffer and into the unit (the kernel reading and writing
//     compressed pages), realloc()s a block (the prefix must survive), frees a
//     block and allocates a new one.
// At the end the threads stop, everything is checked, the process idles until
// the data is compressed again, and everything is checked once more.
//
// Fails on any wrong byte, if fewer than STRESS_MIN_RESTORES accesses found
// their page compressed (smash_page_info: COMPRESSED or COMPRESSED_SHADOW),
// and if under half the cold data is compressed after idling -- a run that
// never exercised the compressor proves nothing.
//
// The registered runs use three test-only knobs (see agents/tests.md for what
// each one bought): SMASH_TEST_TICK_MS=50 (a tick every 50 ms instead of 1 s,
// so the whole pipeline runs 20x faster), SMASH_TEST_WORKERS=8 (eight
// compressor workers whatever the load, so worker range boundaries exist on a
// 2-CPU runner) and SMASH_TEST_JITTER_US=200 (random sleeps before smash's
// mprotect and madvise calls).
//
// Environment: STRESS_SEC (default 20), STRESS_THREADS (default 2 x CPUs, at
// most 16), STRESS_MIB per thread (default 16), STRESS_BLOCK_KIB,
// STRESS_WAVE_SEC (0: no waves), STRESS_OBJS and STRESS_OBJ_MAX (small objects
// per thread, default 2048 of up to 4096 bytes), STRESS_SURVIVORS (default
// 8192 allocations, 0 for none), STRESS_MIN_RESTORES (default 2000),
// STRESS_IDLE_SEC (longest wait for the data to be compressed again after the
// stop, default 180; the wait ends as soon as 90% is). Long runs are the soak
// test (agents/tests.md).
//
// An idle phase longer than 5 s prints, every 5 s, the cold units' page
// states, idle counts and restore counts, and with SMASH_STATS_SIGNAL=1 smash's
// stats (stderr) at its start and end. On a 128-core arm64 box with
// SMASH_WRITE_TRACK=idle, after 1800 s, compressor ticks took 5-9 s and every
// page still read idle count 0 at 30 s, the old limit: a timing failure, not
// lost data.
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <new>
#include <pthread.h>
#include <signal.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

namespace {

constexpr size_t kUnit = 4096;
constexpr size_t kMaxUnits = 768;                    // 3 MiB
size_t g_min_units = 320, g_max_units = 768;         // 1.25-3 MiB unless STRESS_BLOCK_KIB
constexpr size_t kMaxBlocks = 64;                    // per thread
size_t g_small_objs = 2048, g_obj_max = 4096;        // per thread, full mode
constexpr size_t kSurvivorSize = 256;
size_t g_survivor_allocs = 8192;                     // STRESS_SURVIVORS
constexpr double kSurvivorCheckSec = 3, kSurvivorRebuildSec = 9;
constexpr size_t kSharedBlocks = 4;
constexpr int kMaxReports = 20;

using PageInfoFn = int (*)(const void*, int*, int*, int*);
PageInfoFn g_page_info = nullptr;
enum : int { kCompressed = 2, kShadow = 5 };

bool g_large_only = false;
std::atomic<bool> g_stop{false};
std::atomic<uint64_t> g_bad_units{0}, g_bad_objs{0}, g_bad_syscalls{0};
std::atomic<uint64_t> g_hit_compressed{0}, g_hit_shadow{0};
std::atomic<uint64_t> g_ops{0}, g_syscall_ops{0}, g_reallocs{0}, g_frees{0};
std::atomic<int> g_reports{0};
std::atomic<uint32_t> g_next_uid{1};
std::atomic<uint64_t> g_waves{0};
double g_t0 = 0, g_wave_sec = 4;

double nowSec() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

// Memory the allocator under test never sees.
template <typename T>
T* privateArray(size_t n) {
    void* p = mmap(nullptr, n * sizeof(T), PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (p == MAP_FAILED) {
        std::fprintf(stderr, "FAIL: mmap of %zu bytes\n", n * sizeof(T));
        std::exit(2);
    }
    return static_cast<T*>(p);
}

uint64_t mix(uint64_t x) {
    x ^= x >> 33;
    x *= 0xff51afd7ed558ccdull;
    x ^= x >> 33;
    x *= 0xc4ceb9fe1a85ec53ull;
    x ^= x >> 33;
    return x;
}

struct Rng {
    uint64_t s;
    uint64_t next() { return s = mix(s + 0x9E3779B97F4A7C15ull); }
    size_t below(size_t n) { return static_cast<size_t>(next() % n); }
};

uint64_t unitTag(uint32_t uid, size_t unit) { return (static_cast<uint64_t>(uid) << 20) | unit; }

uint64_t unitWord(uint64_t tag, uint32_t ver, size_t i) {
    if (ver % 7 == 0) return 0;
    if (i == 0) return tag;
    if (i == 1) return ver;
    return mix(tag * 31 + ver) + (i & 15);   // 16-word period: compresses well
}

void fillUnit(uint8_t* u, uint64_t tag, uint32_t ver) {
    auto* w = reinterpret_cast<uint64_t*>(u);
    for (size_t i = 0; i < kUnit / 8; ++i) w[i] = unitWord(tag, ver, i);
}

// Wrong 8-byte words in the unit.
size_t badWords(const uint8_t* u, uint64_t tag, uint32_t ver) {
    auto* w = reinterpret_cast<const volatile uint64_t*>(u);
    size_t bad = 0;
    for (size_t i = 0; i < kUnit / 8; ++i) bad += w[i] != unitWord(tag, ver, i);
    return bad;
}

int pageState(const void* p) {
    int st = -1;
    if (g_page_info) g_page_info(p, &st, nullptr, nullptr);
    return st;
}

void notePageState(const void* p) {
    const int st = pageState(p);
    if (st == kCompressed) g_hit_compressed.fetch_add(1, std::memory_order_relaxed);
    else if (st == kShadow) g_hit_shadow.fetch_add(1, std::memory_order_relaxed);
}

void report(const char* what, int thread, const void* at, uint64_t tag, uint32_t ver, const uint8_t* u, size_t bad) {
    if (g_reports.fetch_add(1) >= kMaxReports) return;
    auto* w = reinterpret_cast<const volatile uint64_t*>(u);
    size_t zero = 0;
    for (size_t i = 0; i < kUnit / 8; ++i) zero += w[i] == 0;
    std::fprintf(stderr,
                 "MISMATCH %s: thread %d, %p, want tag %#llx version %u; found words 0,1 = %#llx, %llu; %zu of %zu "
                 "words wrong, %zu words zero; page state now %d\n",
                 what, thread, at, static_cast<unsigned long long>(tag), ver, static_cast<unsigned long long>(w[0]),
                 static_cast<unsigned long long>(w[1]), bad, kUnit / 8, zero, pageState(at));
}

struct Block {
    uint8_t* p = nullptr;
    size_t units = 0;
    uint32_t uid = 0;
    uint32_t* ver = nullptr;   // kMaxUnits, private memory
};

struct SmallObj {
    uint8_t* p;
    uint32_t size, uid, ver;
};

Block g_shared[kSharedBlocks];

struct Thread {
    int id = 0;
    pthread_t th{};
    Block* blocks = nullptr;   // private memory
    size_t nblocks = 0;
    SmallObj* objs = nullptr;  // private memory
    SmallObj* surv = nullptr;  // private memory, g_survivor_allocs
    size_t nsurv = 0;
    double surv_built = 0, surv_checked = 0;
    uint8_t* iobuf = nullptr;  // private memory
    int fd = -1;
    Rng rng{0};
};

void newBlock(Block& b, Rng& rng) {
    b.units = g_min_units + rng.below(g_max_units - g_min_units + 1);
    b.uid = g_next_uid.fetch_add(1);
    b.p = static_cast<uint8_t*>(std::malloc(b.units * kUnit));
    if (!b.p) {
        std::fprintf(stderr, "FAIL: malloc of %zu units\n", b.units);
        std::exit(2);
    }
    for (size_t u = 0; u < b.units; ++u) {
        b.ver[u] = 1;
        fillUnit(b.p + u * kUnit, unitTag(b.uid, u), 1);
    }
}

// Checks unit u of b; on a mismatch reports it and rewrites the unit so one
// loss is counted once.
bool checkUnit(Block& b, size_t u, int thread, const char* what) {
    uint8_t* at = b.p + u * kUnit;
    const uint64_t tag = unitTag(b.uid, u);
    const size_t bad = badWords(at, tag, b.ver[u]);
    if (!bad) return true;
    g_bad_units.fetch_add(1);
    report(what, thread, at, tag, b.ver[u], at, bad);
    fillUnit(at, tag, ++b.ver[u]);
    return false;
}

size_t checkBlock(Block& b, int thread, const char* what) {
    size_t bad = 0;
    for (size_t u = 0; u < b.units; ++u) bad += !checkUnit(b, u, thread, what);
    return bad;
}

uint8_t objByte(uint32_t uid, uint32_t ver, size_t j) {
    return static_cast<uint8_t>(mix((static_cast<uint64_t>(uid) << 32) | ver) >> ((j & 7) * 8)) + static_cast<uint8_t>(j >> 6);
}

void fillObj(SmallObj& o) {
    for (size_t j = 0; j < o.size; ++j) o.p[j] = objByte(o.uid, o.ver, j);
}

void newObj(SmallObj& o, Rng& rng) {
    o.size = 16 + static_cast<uint32_t>(rng.below(g_obj_max - 15));
    o.uid = g_next_uid.fetch_add(1);
    o.ver = 1;
    o.p = static_cast<uint8_t*>(std::malloc(o.size));
    if (!o.p) {
        std::fprintf(stderr, "FAIL: malloc of %u bytes\n", o.size);
        std::exit(2);
    }
    fillObj(o);
}

bool checkObj(SmallObj& o, int thread, const char* what) {
    size_t bad = 0;
    for (size_t j = 0; j < o.size; ++j) bad += o.p[j] != objByte(o.uid, o.ver, j);
    if (!bad) return true;
    g_bad_objs.fetch_add(1);
    if (g_reports.fetch_add(1) < kMaxReports)
        std::fprintf(stderr, "MISMATCH %s: thread %d, object %p of %u bytes, version %u: %zu bytes wrong; page state %d\n",
                     what, thread, static_cast<void*>(o.p), o.size, o.ver, bad, pageState(o.p));
    ++o.ver;
    fillObj(o);
    return false;
}

// Survivors (full mode): 2048 objects of 256 bytes allocated at once, all but
// the first on each page freed, the rest left idle -- slab pages that are
// mostly free slots and stay compressed for many ticks. Checked every 5 s,
// rebuilt every 15 s. The Phase B race lost exactly such pages.
void buildSurvivors(Thread& t) {
    t.nsurv = 0;
    if (!g_survivor_allocs) return;
    auto** all = privateArray<uint8_t*>(g_survivor_allocs);
    for (size_t k = 0; k < g_survivor_allocs; ++k) all[k] = static_cast<uint8_t*>(std::malloc(kSurvivorSize));
    t.nsurv = 0;
    uintptr_t last_page = 0;
    for (size_t k = 0; k < g_survivor_allocs; ++k) {
        uint8_t* p = all[k];
        const uintptr_t pg = reinterpret_cast<uintptr_t>(p) / kUnit;
        if (p && pg != last_page && (reinterpret_cast<uintptr_t>(p) + kSurvivorSize - 1) / kUnit == pg) {
            last_page = pg;
            SmallObj& o = t.surv[t.nsurv++];
            o.p = p;
            o.size = kSurvivorSize;
            o.uid = g_next_uid.fetch_add(1);
            o.ver = 1;
            fillObj(o);
        } else {
            std::free(p);
        }
    }
    munmap(all, g_survivor_allocs * sizeof(uint8_t*));
}

void survivorOp(Thread& t) {
    const double now = nowSec();
    if (now - t.surv_checked < kSurvivorCheckSec) return;
    t.surv_checked = now;
    for (size_t k = 0; k < t.nsurv; ++k) {
        notePageState(t.surv[k].p);
        checkObj(t.surv[k], t.id, "survivor read");
    }
    if (now - t.surv_built >= kSurvivorRebuildSec) {
        for (size_t k = 0; k < t.nsurv; ++k) std::free(t.surv[k].p);
        buildSurvivors(t);
        t.surv_built = now;
    }
}

// A cold unit through the kernel: written out from the unit, read back into a
// private buffer and checked, then read back into the unit itself.
void syscallOp(Thread& t, Block& b, size_t u) {
    uint8_t* at = b.p + u * kUnit;
    const uint64_t tag = unitTag(b.uid, u);
    notePageState(at);
    ssize_t n = pwrite(t.fd, at, kUnit, 0);
    if (n != static_cast<ssize_t>(kUnit)) {
        g_bad_syscalls.fetch_add(1);
        if (g_reports.fetch_add(1) < kMaxReports)
            std::fprintf(stderr, "MISMATCH pwrite: thread %d, %p: returned %zd (errno %d)\n", t.id, at, n, errno);
        return;
    }
    n = pread(t.fd, t.iobuf, kUnit, 0);
    const size_t bad = n == static_cast<ssize_t>(kUnit) ? badWords(t.iobuf, tag, b.ver[u]) : kUnit / 8;
    if (bad) {
        g_bad_syscalls.fetch_add(1);
        report("pwrite from a unit", t.id, at, tag, b.ver[u], t.iobuf, bad);
    }
    n = pread(t.fd, at, kUnit, 0);
    if (n != static_cast<ssize_t>(kUnit)) {
        g_bad_syscalls.fetch_add(1);
        if (g_reports.fetch_add(1) < kMaxReports)
            std::fprintf(stderr, "MISMATCH pread into a unit: thread %d, %p: returned %zd (errno %d)\n", t.id, at, n,
                         errno);
        fillUnit(at, tag, b.ver[u]);
        return;
    }
    if (bad) fillUnit(at, tag, b.ver[u]);   // the file held wrong bytes
    checkUnit(b, u, t.id, "pread into a unit");
}

void reallocOp(Thread& t, Block& b) {
    const size_t units = g_min_units + t.rng.below(g_max_units - g_min_units + 1);
    auto* q = static_cast<uint8_t*>(std::realloc(b.p, units * kUnit));
    if (!q) {
        std::fprintf(stderr, "FAIL: realloc to %zu units\n", units);
        std::exit(2);
    }
    b.p = q;
    const size_t keep = std::min(units, b.units);
    for (size_t u = 0; u < keep; ++u) checkUnit(b, u, t.id, "after realloc");
    for (size_t u = keep; u < units; ++u) {
        b.ver[u] = 1;
        fillUnit(b.p + u * kUnit, unitTag(b.uid, u), 1);
    }
    b.units = units;
    g_reallocs.fetch_add(1, std::memory_order_relaxed);
}

void coldOp(Thread& t, Block& b, size_t u) {
    uint8_t* at = b.p + u * kUnit;
    const uint64_t tag = unitTag(b.uid, u);
    notePageState(at);
    const size_t r = t.rng.below(100);
    if (r < 50) {
        checkUnit(b, u, t.id, "cold read");
    } else if (r < 85) {
        if (checkUnit(b, u, t.id, "cold read before a rewrite")) fillUnit(at, tag, ++b.ver[u]);
    } else {
        // A store of the value already there: a write that changes nothing.
        auto* w = reinterpret_cast<volatile uint64_t*>(at) + 2 + t.rng.below(kUnit / 8 - 2);
        *w = *w;
        checkUnit(b, u, t.id, "after a same-value store");
    }
}

// Every thread rewrites all its cold data at the start of each wave and
// reads it all back late in the wave, once it has been compressed: the
// pages of every thread then reach each phase in the same tick, which is
// what puts compressor workers on neighbouring pages at the same time.
void waveOp(Thread& t, long& wrote, long& read) {
    const double at = (nowSec() - g_t0) / g_wave_sec;
    const long w = static_cast<long>(at);
    if (w > wrote) {
        wrote = w;
        for (size_t k = 1; k < t.nblocks; ++k) {
            Block& b = t.blocks[k];
            for (size_t u = 0; u < b.units; ++u)
                if (checkUnit(b, u, t.id, "wave read before a rewrite"))
                    fillUnit(b.p + u * kUnit, unitTag(b.uid, u), ++b.ver[u]);
        }
        if (t.id == 0) g_waves.fetch_add(1, std::memory_order_relaxed);
    } else if (at - static_cast<double>(w) >= 0.75 && w > read) {
        read = w;
        for (size_t k = 1; k < t.nblocks; ++k) {
            Block& b = t.blocks[k];
            for (size_t u = 0; u < b.units; ++u) {
                notePageState(b.p + u * kUnit);
                checkUnit(b, u, t.id, "wave read");
            }
        }
    }
}

void* threadMain(void* arg) {
    Thread& t = *static_cast<Thread*>(arg);
    uint64_t round = 0;
    long wave_wrote = 0, wave_read = 0;
    while (!g_stop.load(std::memory_order_relaxed)) {
        ++round;
        if (g_wave_sec > 0) waveOp(t, wave_wrote, wave_read);
        // Block 0 is hot: never idle long enough to be compressed.
        Block& hot = t.blocks[0];
        for (int k = 0; k < 16; ++k) {
            const size_t u = t.rng.below(hot.units);
            if (t.rng.below(2)) {
                if (checkUnit(hot, u, t.id, "hot read")) fillUnit(hot.p + u * kUnit, unitTag(hot.uid, u), ++hot.ver[u]);
            } else {
                checkUnit(hot, u, t.id, "hot read");
            }
        }
        if (t.nblocks > 1) {
            Block& b = t.blocks[1 + t.rng.below(t.nblocks - 1)];
            const size_t u = t.rng.below(b.units);
            if (round % 64 == 0) {
                syscallOp(t, b, u);
                g_syscall_ops.fetch_add(1, std::memory_order_relaxed);
            } else {
                coldOp(t, b, u);
            }
            if (round % 512 == 0) reallocOp(t, b);
            if (round % 1024 == 0) {
                checkBlock(b, t.id, "before free");
                std::free(b.p);
                newBlock(b, t.rng);
                g_frees.fetch_add(1, std::memory_order_relaxed);
            }
        }
        {
            Block& s = g_shared[t.rng.below(kSharedBlocks)];
            const size_t u = t.rng.below(s.units);
            notePageState(s.p + u * kUnit);
            const uint64_t tag = unitTag(s.uid, u);
            const size_t bad = badWords(s.p + u * kUnit, tag, 1);
            if (bad) {
                g_bad_units.fetch_add(1);
                report("shared read", t.id, s.p + u * kUnit, tag, 1, s.p + u * kUnit, bad);
            }
        }
        if (!g_large_only) survivorOp(t);
        if (!g_large_only) {
            SmallObj& o = t.objs[t.rng.below(g_small_objs)];
            notePageState(o.p);
            const size_t r = t.rng.below(100);
            if (r < 60) {
                checkObj(o, t.id, "small object read");
            } else if (r < 90) {
                if (checkObj(o, t.id, "small object read before a rewrite")) {
                    ++o.ver;
                    fillObj(o);
                }
            } else {
                checkObj(o, t.id, "small object read before free");
                std::free(o.p);
                newObj(o, t.rng);
            }
        }
        g_ops.fetch_add(1, std::memory_order_relaxed);
        usleep(2000);
    }
    return nullptr;
}

size_t envSize(const char* name, size_t dflt) {
    const char* v = std::getenv(name);
    if (!v || !*v) return dflt;
    const long long n = std::atoll(v);
    return n > 0 ? static_cast<size_t>(n) : dflt;
}

size_t checkAll(Thread* threads, size_t nthreads, const char* what) {
    const uint64_t before = g_bad_units.load() + g_bad_objs.load();
    for (size_t i = 0; i < nthreads; ++i) {
        Thread& t = threads[i];
        for (size_t k = 0; k < t.nblocks; ++k) checkBlock(t.blocks[k], t.id, what);
        if (!g_large_only) {
            for (size_t k = 0; k < g_small_objs; ++k) checkObj(t.objs[k], t.id, what);
            for (size_t k = 0; k < t.nsurv; ++k) checkObj(t.surv[k], t.id, what);
        }
    }
    for (Block& s : g_shared)
        for (size_t u = 0; u < s.units; ++u) {
            const uint64_t tag = unitTag(s.uid, u);
            const size_t bad = badWords(s.p + u * kUnit, tag, 1);
            if (bad) {
                g_bad_units.fetch_add(1);
                report(what, -1, s.p + u * kUnit, tag, 1, s.p + u * kUnit, bad);
            }
        }
    return static_cast<size_t>(g_bad_units.load() + g_bad_objs.load() - before);
}

// The cold units' page states, idle counts and restore counts (the back-off
// grows with the count), as smash_page_info reports them.
void idleBreakdown(Thread* threads, size_t nthreads, double t) {
    size_t st_hist[16] = {}, rc_hist[8] = {}, cold_hist[8] = {}, total = 0;
    for (size_t i = 0; i < nthreads; ++i)
        for (size_t k = 1; k < threads[i].nblocks; ++k) {
            const Block& b = threads[i].blocks[k];
            for (size_t u = 0; u < b.units; ++u) {
                int st = -1, cold = -1, rc = -1;
                g_page_info(b.p + u * kUnit, &st, &cold, &rc);
                ++st_hist[st >= 0 && st < 15 ? st : 15];
                ++rc_hist[rc < 0 ? 7 : rc > 6 ? 6 : rc];
                ++cold_hist[cold < 0 ? 7 : cold == 0 ? 0 : cold < 4 ? 1 : cold < 16 ? 2 : cold < 64 ? 3
                            : cold < 128 ? 4 : cold < 255 ? 5 : 6];
                ++total;
            }
        }
    std::printf("  idle %5.1f s: %zu units; state:", t, total);
    for (int x = 0; x < 16; ++x)
        if (st_hist[x]) std::printf(" %d=%zu", x == 15 ? -1 : x, st_hist[x]);
    std::printf("; restore count 0,1,2,3,4,5,6+,?:");
    for (size_t n : rc_hist) std::printf(" %zu", n);
    std::printf("; idle count 0,1-3,4-15,16-63,64-127,128-254,255,?:");
    for (size_t n : cold_hist) std::printf(" %zu", n);
    std::printf("\n");
}

// Fraction of the threads' cold units whose pages the compressor holds.
double heldFraction(Thread* threads, size_t nthreads) {
    size_t held = 0, total = 0;
    for (size_t i = 0; i < nthreads; ++i)
        for (size_t k = 1; k < threads[i].nblocks; ++k) {
            const Block& b = threads[i].blocks[k];
            for (size_t u = 0; u < b.units; ++u) {
                const int st = pageState(b.p + u * kUnit);
                held += st == kCompressed || st == kShadow;
                ++total;
            }
        }
    return total ? static_cast<double>(held) / static_cast<double>(total) : 0;
}

}  // namespace

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    g_page_info = reinterpret_cast<PageInfoFn>(dlsym(RTLD_DEFAULT, "smash_page_info"));
    if (!g_page_info) {
        std::printf("SKIP: not running under libsmash\n");
        return 0;
    }
    const char* lo = std::getenv("SMASH_LARGE_ONLY");
    g_large_only = lo && lo[0] == '1';
    const long cpus = sysconf(_SC_NPROCESSORS_ONLN);
    const double secs = static_cast<double>(envSize("STRESS_SEC", 20));
    const size_t nthreads = envSize("STRESS_THREADS", std::min<size_t>(16, 2 * static_cast<size_t>(cpus > 0 ? cpus : 1)));
    const size_t mib = envSize("STRESS_MIB", 16);
    const size_t min_restores = envSize("STRESS_MIN_RESTORES", 2000);
    const double idle_sec = static_cast<double>(envSize("STRESS_IDLE_SEC", 180));
    const char* sig = std::getenv("SMASH_STATS_SIGNAL");
    const bool stats = sig && sig[0] == '1';  // otherwise SIGUSR2 would kill us
    if (const size_t kib = envSize("STRESS_BLOCK_KIB", 0)) {
        g_max_units = std::min(kMaxUnits, std::max<size_t>(2, kib * 3 / 8));
        g_min_units = std::max<size_t>(1, g_max_units * 5 / 12);
    }
    if (const char* w = std::getenv("STRESS_WAVE_SEC")) g_wave_sec = std::atof(w);
    g_small_objs = envSize("STRESS_OBJS", g_small_objs);
    g_survivor_allocs = envSize("STRESS_SURVIVORS", g_survivor_allocs);
    g_obj_max = std::max<size_t>(16, envSize("STRESS_OBJ_MAX", g_obj_max));

    Rng seed{static_cast<uint64_t>(nowSec() * 1e9)};
    for (Block& s : g_shared) {
        s.ver = privateArray<uint32_t>(kMaxUnits);
        s.units = g_max_units;
        s.uid = g_next_uid.fetch_add(1);
        s.p = static_cast<uint8_t*>(std::malloc(s.units * kUnit));
        if (!s.p) return 2;
        for (size_t u = 0; u < s.units; ++u) fillUnit(s.p + u * kUnit, unitTag(s.uid, u), 1);
    }
    Thread* threads = privateArray<Thread>(nthreads);
    size_t total_units = 0;
    for (size_t i = 0; i < nthreads; ++i) {
        Thread& t = threads[i];
        new (&t) Thread();
        t.id = static_cast<int>(i);
        t.rng.s = seed.next();
        t.blocks = privateArray<Block>(kMaxBlocks);
        t.nblocks = std::min(kMaxBlocks, std::max<size_t>(2, mib * 256 / ((g_min_units + g_max_units) / 2)));
        for (size_t k = 0; k < t.nblocks; ++k) {
            t.blocks[k].ver = privateArray<uint32_t>(kMaxUnits);
            newBlock(t.blocks[k], t.rng);
            total_units += t.blocks[k].units;
        }
        t.objs = privateArray<SmallObj>(g_small_objs);
        t.surv = privateArray<SmallObj>(std::max<size_t>(1, g_survivor_allocs));
        if (!g_large_only) {
            for (size_t k = 0; k < g_small_objs; ++k) newObj(t.objs[k], t.rng);
            buildSurvivors(t);
            t.surv_built = t.surv_checked = nowSec();
        }
        t.iobuf = privateArray<uint8_t>(kUnit);
        char path[] = "/tmp/smash_stress_XXXXXX";
        t.fd = mkstemp(path);
        if (t.fd < 0) {
            std::printf("FAIL: mkstemp\n");
            return 2;
        }
        unlink(path);
    }
    std::printf("integrity stress: %zu threads, %zu blocks of %zu-%zu KiB each (%zu MiB in all), %s, waves every "
                "%.1f s, %.0f s\n", nthreads, threads[0].nblocks, g_min_units * 4, g_max_units * 4,
                total_units * kUnit >> 20, g_large_only ? "large-only" : "full mode with small objects",
                g_wave_sec, secs);

    g_t0 = nowSec();
    for (size_t i = 0; i < nthreads; ++i) pthread_create(&threads[i].th, nullptr, threadMain, &threads[i]);
    const double t0 = g_t0;
    double next_print = t0 + 5;
    while (nowSec() - t0 < secs) {
        usleep(100 * 1000);
        if (nowSec() >= next_print) {
            next_print += 5;
            std::printf("  %4.0f s: %llu rounds, %llu accesses found the page compressed, %llu shadow, %.0f%% of cold "
                        "units held now, %llu bad\n",
                        nowSec() - t0, static_cast<unsigned long long>(g_ops.load()),
                        static_cast<unsigned long long>(g_hit_compressed.load()),
                        static_cast<unsigned long long>(g_hit_shadow.load()), 100 * heldFraction(threads, nthreads),
                        static_cast<unsigned long long>(g_bad_units.load() + g_bad_objs.load() + g_bad_syscalls.load()));
        }
    }
    g_stop.store(true);
    for (size_t i = 0; i < nthreads; ++i) pthread_join(threads[i].th, nullptr);
    const uint64_t during = g_bad_units.load() + g_bad_objs.load() + g_bad_syscalls.load();
    const size_t at_stop = checkAll(threads, nthreads, "final check");

    // Idle until the data is compressed again, then read all of it back.
    const double i0 = nowSec();
    double held = 0, next_line = i0 + 5;
    bool slow = false;
    while (nowSec() - i0 < idle_sec && (held = heldFraction(threads, nthreads)) < 0.9) {
        if (nowSec() >= next_line) {
            if (!slow && stats) raise(SIGUSR2);
            slow = true;
            idleBreakdown(threads, nthreads, nowSec() - i0);
            next_line += 5;
        }
        usleep(250 * 1000);
    }
    const double idled = nowSec() - i0;
    if (slow) {
        idleBreakdown(threads, nthreads, idled);
        if (stats) raise(SIGUSR2);
    }
    const size_t after_idle = checkAll(threads, nthreads, "check after idling");

    std::printf("integrity stress: %llu rounds, %llu accesses found the page compressed and %llu shadow, %llu syscall "
                "round trips, %llu reallocs, %llu blocks freed; wrong: %llu during the run, %zu at the stop, %zu after "
                "idling until %.0f%% was compressed again\n",
                static_cast<unsigned long long>(g_ops.load()), static_cast<unsigned long long>(g_hit_compressed.load()),
                static_cast<unsigned long long>(g_hit_shadow.load()),
                static_cast<unsigned long long>(g_syscall_ops.load()), static_cast<unsigned long long>(g_reallocs.load()),
                static_cast<unsigned long long>(g_frees.load()), static_cast<unsigned long long>(during), at_stop,
                after_idle, 100 * held);

    int failures = 0;
    if (during + at_stop + after_idle) {
        std::printf("FAIL: data read back wrong (MISMATCH lines on stderr; %d reported)\n",
                    std::min(g_reports.load(), kMaxReports));
        ++failures;
    }
    const uint64_t restores = g_hit_compressed.load() + g_hit_shadow.load();
    if (restores < min_restores) {
        std::printf("FAIL: only %llu accesses found their page compressed (want %zu): the run did not exercise the "
                    "compressor\n", static_cast<unsigned long long>(restores), min_restores);
        ++failures;
    }
    if (held < 0.5) {
        std::printf("FAIL: only %.0f%% of the cold units compressed after idling %.0f s\n", 100 * held,
                    idled);
        ++failures;
    }
    if (failures) return 1;
    std::printf("integrity_stress_test: all checks passed\n");
    return 0;
}
