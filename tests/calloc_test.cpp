// calloc must return zeroed memory, and must not write the zeros itself when
// the pages are zero already.
//
// alloc8's calloc wrapper was malloc + memset over the whole block and never
// reached SmashHeap::calloc, so every calloc wrote every page of its block:
// calloc(1 GiB) took 0.7 s and made 960 MiB resident at once, where glibc's (a
// fresh mapping) costs neither. smash now skips the memset for a large block
// whose pages it knows are zero (fresh, or decommitted with MADV_DONTNEED) and
// clears every other one. Both halves are checked:
//
//   fresh   calloc(1, 256 MiB), 64 x calloc(1, 512 KiB) (below the VM
//           threshold: direct mappings), and calloc(1, 256 MiB) again over a
//           freed 256 MiB block once it is decommitted, must each leave RSS
//           within 16 MiB of where it was, and read as zero. Before: +256,
//           +32 and +256 MiB. CALLOC_TEST_RSS=0 skips this, for the
//           registrations whose decommit does not zero (SMASH_MADV_FREE=1,
//           SMASH_NO_DECOMMIT=1), where every arena block must be cleared.
//   reused  a block filled with 0xA5 and freed, then calloc of the same size
//           (2 MiB, 3 MiB + 123 bytes, 20 KiB, 100 MiB), must read as zero.
//           Which free path the memory comes back through is the
//           registration's: a sub-arena's dirty run (SMASH_SUBARENAS=1), the
//           per-CPU run cache (SMASH_RUN_CACHE_MIB), a run decommitted with
//           MADV_FREE or not at all, or a plain decommitted run.
//           CALLOC_TEST_REUSE=1 requires the 2 and 3 MiB callocs to overlap
//           memory that held 0xA5, so the registration cannot pass without
//           taking its path.
//   small   full mode's slab slots, which are never known zero.
//   mixed   (CALLOC_TEST_MIXED=1, SMASH_VM_GIB=1, SMASH_SUBARENAS=1, the
//           recipe of test_subarena_exhaust_mixed) a full region; 8 MiB over a
//           sub-arena's clean run and the dirty run next to it, which held
//           0xA5, must come from those runs and read as zero.
//
// Runs pinned to one CPU: the sub-arena and the run cache are per CPU.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <sched.h>
#include <unistd.h>

namespace {

int failures = 0;
#define CHECK(cond, ...)                                                       \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::printf("FAIL: " __VA_ARGS__);                                 \
            std::printf("\n");                                                 \
            ++failures;                                                        \
        }                                                                      \
    } while (0)

using in_arena_fn = int (*)(const void*);
using shared_info_fn = int (*)(size_t*, size_t*);
using subarena_info_fn = int (*)(uint64_t*, uint64_t*);
in_arena_fn g_in_arena = nullptr;
shared_info_fn g_shared_info = nullptr;
subarena_info_fn g_subarena_info = nullptr;

// Through a volatile pointer: the compiler may not fold calloc + memset.
void* (*volatile p_calloc)(size_t, size_t) = std::calloc;
void* (*volatile p_malloc)(size_t) = std::malloc;

bool envOn(const char* name, bool dflt) {
    const char* v = std::getenv(name);
    return v ? v[0] == '1' : dflt;
}

long rssMiB() {
    long pages = 0, resident = 0;
    FILE* f = std::fopen("/proc/self/statm", "r");
    if (!f) return -1;
    if (std::fscanf(f, "%ld %ld", &pages, &resident) != 2) resident = -1;
    std::fclose(f);
    return resident < 0 ? -1 : resident * sysconf(_SC_PAGESIZE) / (1 << 20);
}

// Wait for frees still being decommitted to leave RSS, so they do not hide
// growth: until it stops falling for 50 ms (at most 5 s).
long settledRssMiB() {
    long prev = rssMiB();
    for (int i = 0; i < 100; ++i) {
        usleep(50 * 1000);
        const long now = rssMiB();
        if (now >= prev) return now;
        prev = now;
    }
    return prev;
}

// First non-zero offset, or SIZE_MAX.
size_t firstNonZero(const void* p, size_t n) {
    const auto* b = static_cast<const unsigned char*>(p);
    for (size_t i = 0; i < n; ++i)
        if (b[i]) return i;
    return SIZE_MAX;
}

void checkZero(const void* p, size_t n, const char* what) {
    const size_t at = firstNonZero(p, n);
    CHECK(at == SIZE_MAX, "%s: byte %zu of %zu is 0x%02x, want a zeroed block", what, at, n,
          at == SIZE_MAX ? 0 : static_cast<const unsigned char*>(p)[at]);
}

void pinToOneCpu() {
    cpu_set_t allowed, one;
    sched_getaffinity(0, sizeof(allowed), &allowed);
    for (int c = 0; c < CPU_SETSIZE; ++c)
        if (CPU_ISSET(c, &allowed)) {
            CPU_ZERO(&one);
            CPU_SET(c, &one);
            sched_setaffinity(0, sizeof(one), &one);
            return;
        }
}

// Large-only mode, nothing live in the shared arena: wait until everything
// freed has been decommitted into its free map.
void drainSharedArena() {
    size_t used = 0, free_pages = 0;
    for (int i = 0; i < 500; ++i) {
        if (g_shared_info(&used, &free_pages) != 0 || used == free_pages) return;
        usleep(10 * 1000);
    }
}

// calloc over memory that held 0xA5. CALLOC_TEST_REUSE=1: the 2 and 3 MiB
// blocks must overlap a freed range, or the registration's path went untested.
void testReused(bool must_reuse) {
    struct Case { size_t size; bool arena_sized; };
    const Case cases[] = {
        {2u << 20, true}, {(3u << 20) + 123, true}, {20u << 10, false}, {100u << 20, false},
    };
    struct Range { uintptr_t lo, hi; };
    Range freed[8];
    int nfreed = 0;
    int k = 0;
    for (const Case& c : cases) {
        for (int round = 0; round < 2; ++round, ++k) {
            auto* p = static_cast<unsigned char*>(p_malloc(c.size));
            CHECK(p, "malloc(%zu) failed", c.size);
            if (!p) continue;
            std::memset(p, 0xA5, c.size);
            if (nfreed < 8) freed[nfreed++] = {reinterpret_cast<uintptr_t>(p),
                                               reinterpret_cast<uintptr_t>(p) + c.size};
            std::free(p);
            if (g_shared_info && envOn("CALLOC_TEST_DRAIN", false)) drainSharedArena();
            // Both forms: one element of the whole size, and many small ones
            // over the same pages (the run cache only serves a run of exactly
            // the freed page count).
            void* q = round == 0 ? p_calloc(1, c.size) : p_calloc(c.size / 16, 16);
            const size_t n = round == 0 ? c.size : c.size / 16 * 16;
            char what[64];
            std::snprintf(what, sizeof(what), "calloc of %zu bytes over a freed block", n);
            CHECK(q, "%s failed", what);
            if (!q) continue;
            checkZero(q, n, what);
            if (must_reuse && c.arena_sized) {
                const auto lo = reinterpret_cast<uintptr_t>(q), hi = lo + n;
                bool overlaps = false;
                for (int i = 0; i < nfreed; ++i) overlaps |= lo < freed[i].hi && freed[i].lo < hi;
                CHECK(overlaps, "%s got %p, not memory that held 0xA5: this registration's free "
                      "path was never taken", what, q);
            }
            std::free(q);
        }
    }
    std::printf("reused: %d callocs over freed 0xA5 blocks read as zero\n", k);
}

// Large callocs whose pages are zero already must not be written. `smash_owns`
// is false where libc's calloc serves them (passthrough, compress-only): there
// the 512 KiB blocks come from glibc's heap once a freed large block has
// raised its mmap threshold (+7 MiB measured), which is glibc's business.
void testFresh(bool smash_owns) {
    constexpr size_t kBig = 256u << 20, kSmall = 512u << 10;
    constexpr long kSlackMiB = 16;
    long r0 = settledRssMiB();
    void* a = p_calloc(1, kBig);
    long r1 = rssMiB();
    CHECK(a, "calloc(1, 256 MiB) failed");
    std::printf("fresh: calloc(1, 256 MiB): RSS +%ld MiB\n", r1 - r0);
    CHECK(r1 - r0 <= kSlackMiB, "calloc(1, 256 MiB) on fresh pages made %ld MiB resident: it "
          "wrote zeros over pages that were zero already", r1 - r0);
    if (a) checkZero(a, kBig, "calloc(1, 256 MiB)");
    std::free(a);

    static void* small[64];   // static: an array of them would come from malloc
    r0 = settledRssMiB();
    for (auto& s : small) s = p_calloc(1, kSmall);
    r1 = rssMiB();
    std::printf("fresh: 64 x calloc(1, 512 KiB): RSS +%ld MiB\n", r1 - r0);
    CHECK(!smash_owns || r1 - r0 <= kSlackMiB / 2, "64 x calloc(1, 512 KiB) made %ld MiB "
          "resident: direct mappings are zero already", r1 - r0);
    for (auto& s : small) {
        CHECK(s, "calloc(1, 512 KiB) failed");
        if (s) checkZero(s, kSmall, "calloc(1, 512 KiB)");
        std::free(s);
    }

    // The same 256 MiB once more, over a freed block decommitted in between:
    // MADV_DONTNEED left its pages zero.
    auto* b = static_cast<unsigned char*>(p_malloc(kBig));
    CHECK(b, "malloc(256 MiB) failed");
    if (!b) return;
    std::memset(b, 0xA5, kBig);
    const long touched = rssMiB();
    std::free(b);
    long now = touched;
    for (int i = 0; i < 500 && now > touched - 200; ++i) {
        usleep(10 * 1000);
        now = rssMiB();
    }
    CHECK(now <= touched - 200, "a freed 256 MiB block was never decommitted (RSS %ld -> %ld MiB)",
          touched, now);
    r0 = settledRssMiB();
    void* c = p_calloc(1, kBig);
    r1 = rssMiB();
    CHECK(c, "calloc(1, 256 MiB) failed");
    const auto lo = reinterpret_cast<uintptr_t>(c), blo = reinterpret_cast<uintptr_t>(b);
    std::printf("decommitted: calloc(1, 256 MiB) %s the freed block: RSS +%ld MiB\n",
                lo < blo + kBig && blo < lo + kBig ? "over" : "not over", r1 - r0);
    CHECK(r1 - r0 <= kSlackMiB, "calloc(1, 256 MiB) over decommitted pages made %ld MiB "
          "resident: it wrote zeros over pages that were zero already", r1 - r0);
    if (c) checkZero(c, kBig, "calloc(1, 256 MiB) over a decommitted block");
    std::free(c);
}

// Full mode's slab slots: a freed slot keeps its bytes. Small blocks go to an
// arena chosen by call site, so a malloc'd slot never came back to a calloc
// from another line: the same calloc call runs twice, the first round writing
// 0xA5 into its 64 blocks and freeing them, and in full mode the second must
// get some of them back, zeroed.
void testSmall(bool full) {
    static const size_t kSizes[] = {24, 64, 1000, 4096, 16000};
    constexpr int kN = 64;
    static void* freed[kN];
    static void* got[kN];
    for (size_t n : kSizes) {
        int reused = 0;
        for (int round = 0; round < 2; ++round) {
            for (int i = 0; i < kN; ++i) {
                void* q = got[i] = p_calloc(1, n);
                CHECK(q, "calloc(1, %zu) failed", n);
                if (!q) continue;
                checkZero(q, n, round ? "small calloc over freed 0xA5 slots" : "small calloc");
                if (round == 0) {
                    std::memset(q, 0xA5, n);
                    freed[i] = q;
                } else {
                    for (void* p : freed) reused += p == q;
                }
            }
            for (void* q : got) std::free(q);
        }
        if (full)
            CHECK(reused > 0, "calloc(1, %zu): none of %d blocks reused a freed 0xA5 slot", n, kN);
    }
    std::printf("small: %zu sizes over freed slots read as zero\n", sizeof(kSizes) / sizeof(kSizes[0]));
}

// test_subarena_exhaust_mixed's recipe: fill a 1 GiB region with 4 MiB
// blocks, make one block's run clean (freed and purged) and its neighbour's
// dirty (freed, holding 0xA5), then ask for 8 MiB, which only the two together
// hold.
int testMixed() {
    constexpr size_t kChunk = 64u << 20, kBlock = 4u << 20;
    constexpr int kMax = 512;
    static void* blocks[kMax];
    if (!g_subarena_info || g_subarena_info(nullptr, nullptr) != 0) {
        std::printf("FAIL: mixed: needs SMASH_SUBARENAS=1 and smash_subarena_info\n");
        return 1;
    }
    int n = 0;
    bool out = false;
    while (n < kMax) {
        auto* p = static_cast<unsigned char*>(p_malloc(kBlock));
        if (!p) break;
        p[0] = 1;
        if (!g_in_arena(p)) { std::free(p); out = true; break; }
        blocks[n++] = p;
    }
    if (!out) { std::printf("FAIL: mixed: the region never ran out (%d blocks)\n", n); return 1; }
    uintptr_t x = 0, base = UINTPTR_MAX;
    for (int i = 0; i < n; ++i) base = std::min(base, reinterpret_cast<uintptr_t>(blocks[i]));
    auto have = [&](uintptr_t a) {
        for (int i = 0; i < n; ++i) if (reinterpret_cast<uintptr_t>(blocks[i]) == a) return true;
        return false;
    };
    for (int i = 0; i < n && !x; ++i) {
        const auto a = reinterpret_cast<uintptr_t>(blocks[i]);
        if ((a - base) % kChunk) continue;
        bool full = true;
        for (size_t k = 1; k < kChunk / kBlock && full; ++k) full = have(a + k * kBlock);
        if (full) x = a;
    }
    if (!x) { std::printf("FAIL: mixed: no chunk filled by 16 of the %d blocks\n", n); return 1; }
    auto release = [&](uintptr_t a) {
        for (int i = 0; i < n; ++i)
            if (reinterpret_cast<uintptr_t>(blocks[i]) == a) { std::free(blocks[i]); blocks[i] = blocks[--n]; return; }
    };
    uint64_t dirty = 0, purged = 0;
    release(x + 5 * kBlock);
    for (int i = 0; i < 500; ++i) {
        g_subarena_info(&dirty, &purged);
        if (dirty == 0) break;
        usleep(10 * 1000);
    }
    if (dirty) { std::printf("FAIL: mixed: the freed block was never purged\n"); return 1; }
    std::memset(reinterpret_cast<void*>(x + 6 * kBlock), 0xA5, kBlock);
    release(x + 6 * kBlock);
    void* m = p_calloc(1, 2 * kBlock);
    const bool from_runs = reinterpret_cast<uintptr_t>(m) == x + 5 * kBlock;
    std::printf("mixed: calloc(1, 8 MiB) over a clean and a dirty 4 MiB run: %s the runs\n",
                from_runs ? "from" : "not from");
    CHECK(from_runs, "mixed: calloc(1, 8 MiB) = %p, not the clean and dirty runs at %p", m,
          reinterpret_cast<void*>(x + 5 * kBlock));
    if (m) checkZero(m, 2 * kBlock, "calloc(1, 8 MiB) over a clean and a dirty run");
    std::free(m);
    while (n) std::free(blocks[--n]);
    return failures ? 1 : 0;
}

}  // namespace

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    g_in_arena = reinterpret_cast<in_arena_fn>(dlsym(RTLD_DEFAULT, "smash_in_arena"));
    if (!g_in_arena) { std::printf("SKIP: not running under libsmash\n"); return 0; }
    g_shared_info = reinterpret_cast<shared_info_fn>(dlsym(RTLD_DEFAULT, "smash_shared_arena_info"));
    g_subarena_info = reinterpret_cast<subarena_info_fn>(dlsym(RTLD_DEFAULT, "smash_subarena_info"));
    pinToOneCpu();
    int rc;
    if (envOn("CALLOC_TEST_MIXED", false)) {
        rc = testMixed();
    } else {
        testReused(envOn("CALLOC_TEST_REUSE", false));
        const char* lo = std::getenv("SMASH_LARGE_ONLY");
        const bool libc = envOn("SMASH_PASSTHROUGH", false) || std::getenv("SMASH_MODE");
        testSmall(!libc && !(lo && lo[0] == '1'));
        if (envOn("CALLOC_TEST_RSS", true)) testFresh(!libc);
        rc = failures ? 1 : 0;
    }
    std::printf("calloc_test: %s\n", rc ? "FAILED" : "all checks passed");
    return rc;
}
