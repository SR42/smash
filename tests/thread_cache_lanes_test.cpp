// thread_cache_lanes_test.cpp - The thread cache must not move a call site's
// objects into another arena.
//
// Every call site's small objects go to its own arena (callsiteArena), so a
// page holds one call site's data and compresses as such. The thread cache
// sits in front of the arenas: a lane of pointers per (arena, size class),
// filled from the arena's slab by refill() and emptied by allocate(). If two
// arenas share a lane, whatever the lane holds goes to either one's call
// site, so a refill for one call site feeds the next allocation of the other.
// Upstream #93 fixed the cache at 16 lanes, which hosts with 17+ CPUs (32+
// arenas by default) or SMASH_COLD_ARENA_FEEDBACK (twice the arenas) share:
// measured at 64 arenas, 57% of 32 interleaved call sites' objects were in
// their own arena and 81% of their pages held objects of several arenas.
// The lanes now follow the arena count (ThreadCache::layout()).
//
// The test runs under LD_PRELOAD in full mode with SMASH_NUM_ARENAS set by
// the ctest (64: depth 4; 128: the depth floor; 4: depth 64). 32 noinline
// call sites of 256-byte objects allocate 2048 objects each, round robin;
// then 200k times a random object is freed and replaced from a random site.
// After each phase every call site's live objects must all be in one arena.
// Same size and stack depth at every site, so the arena is a function of the
// return address alone (SMASH_CPU_ARENA=0, feedback off).

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <dlfcn.h>
#include <vector>

static volatile uintptr_t g_sink;

#define SITE(n)                                                  \
    __attribute__((noinline)) static void* site_##n() {         \
        void* p = malloc(256);                                   \
        g_sink = reinterpret_cast<uintptr_t>(p) + (n);          \
        return p;                                                \
    }
SITE(0)  SITE(1)  SITE(2)  SITE(3)  SITE(4)  SITE(5)  SITE(6)  SITE(7)
SITE(8)  SITE(9)  SITE(10) SITE(11) SITE(12) SITE(13) SITE(14) SITE(15)
SITE(16) SITE(17) SITE(18) SITE(19) SITE(20) SITE(21) SITE(22) SITE(23)
SITE(24) SITE(25) SITE(26) SITE(27) SITE(28) SITE(29) SITE(30) SITE(31)

using site_fn = void* (*)();
static site_fn kSites[] = {
    site_0,  site_1,  site_2,  site_3,  site_4,  site_5,  site_6,  site_7,
    site_8,  site_9,  site_10, site_11, site_12, site_13, site_14, site_15,
    site_16, site_17, site_18, site_19, site_20, site_21, site_22, site_23,
    site_24, site_25, site_26, site_27, site_28, site_29, site_30, site_31,
};
constexpr int kNumSites = 32, kPerSite = 2048;

using arena_fn = int (*)(const void*);
static arena_fn arena_for;

struct Obj { void* p; int site; };

// Returns the number of call sites whose live objects span several arenas.
static int check(const char* phase, const std::vector<Obj>& objs) {
    // first[s]: the arena of site s's first object; -1 none yet
    int first[kNumSites], arenas_used = 0, split = 0;
    size_t stray[kNumSites] = {}, total = 0, strays = 0;
    bool seen[256] = {};
    for (int& f : first) f = -1;
    for (const Obj& o : objs) {
        const int a = arena_for(o.p);
        if (a < 0 || a >= 256) {
            std::fprintf(stderr, "FAIL: %s: %p from site %d is not a smash slab object (arena %d)\n",
                         phase, o.p, o.site, a);
            std::exit(1);
        }
        if (!seen[a]) seen[a] = true, ++arenas_used;
        if (first[o.site] < 0) first[o.site] = a;
        else if (a != first[o.site]) ++stray[o.site];
        ++total;
    }
    for (int s = 0; s < kNumSites; ++s) {
        if (!stray[s]) continue;
        ++split;
        strays += stray[s];
        if (split <= 4)
            std::fprintf(stderr, "  %s: site %d: %zu objects outside arena %d\n", phase, s, stray[s], first[s]);
    }
    std::printf("%s: %zu objects, %d arenas, %d of %d call sites split, %zu objects (%.1f%%) "
                "outside their site's first arena\n",
                phase, total, arenas_used, split, kNumSites, strays, 100.0 * strays / total);
    return split;
}

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    arena_for = reinterpret_cast<arena_fn>(dlsym(RTLD_DEFAULT, "smash_arena_for"));
    if (!arena_for) {
        std::fprintf(stderr, "FAIL: smash_arena_for not found (not running under libsmash)\n");
        return 1;
    }
    const char* na = std::getenv("SMASH_NUM_ARENAS");
    std::printf("SMASH_NUM_ARENAS=%s\n", na ? na : "(default)");

    std::vector<Obj> objs;
    objs.reserve(kNumSites * kPerSite);
    for (int i = 0; i < kPerSite; ++i)
        for (int s = 0; s < kNumSites; ++s) objs.push_back({kSites[s](), s});
    int bad = check("interleaved", objs);

    uint64_t x = 88172645463325252ULL;
    for (int i = 0; i < 200000; ++i) {
        x ^= x << 13; x ^= x >> 7; x ^= x << 17;
        Obj& o = objs[x % objs.size()];
        std::free(o.p);
        o.site = static_cast<int>((x >> 32) % kNumSites);
        o.p = kSites[o.site]();
    }
    bad += check("churn", objs);
    for (Obj& o : objs) std::free(o.p);

    if (bad) {
        std::fprintf(stderr,
            "FAIL: call sites' objects landed in other arenas. A thread-cache lane "
            "shared by two arenas hands one arena's objects to the other's call "
            "site: the lanes must follow the arena count (ThreadCache::layout).\n");
        return 1;
    }
    std::printf("PASS: every call site's objects stayed in one arena\n");
    return 0;
}
