// arena_routing_test.cpp - Regression guard for call-site arena routing.
//
// Smash routes allocations to arenas by hashing the APPLICATION's allocation
// call site (return address), so allocations from the same site share pages
// and compress well. That capture broke once already: under LTO + dynamic
// interposition, the return address seen inside the allocator pointed into
// the wrapper library instead of the app, so every call site collapsed into
// ONE arena (bench_arena_segregation: homogeneity 0%, fully-mixed 100%). The
// fix routes on alloc8_caller_ra, captured at the wrapper entry.
//
// This test asserts the invariant that broke, deterministically and fast:
// allocations from many distinct call sites must NOT all land in one arena.
// It must run UNDER interposition (LD_PRELOAD / DYLD_INSERT_LIBRARIES) — that
// is the only path where the bug manifests; a direct-linked call would use
// __builtin_return_address at the malloc entry and never exercise the wrapper.
//
// With the default routing inputs (SMASH_CPU_ARENA off, thread-hash off), the
// arena is a pure function of (return address, size class, stack depth). Every
// site here allocates the same size from the same depth, so the return address
// is the ONLY variable: buggy capture => 1 distinct arena (FAIL); correct
// capture => several (PASS). The ctest pins SMASH_LARGE_ONLY=0 (slab path) and
// SMASH_CPU_ARENA=0 so nothing else can perturb the routing.

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>

// Query hook exported by libsmash (visibility default + version-script entry).
// Resolved at runtime so the binary links without smash and skips cleanly when
// run with no preload.
using arena_fn = int (*)(const void*);

// 32 distinct allocation sites. noinline + a volatile sink that consumes a
// per-site constant makes each a real call with its own return address (no
// tail-call, no identical-code folding). Same size/depth across all of them,
// so only the return address varies between sites.
static volatile uintptr_t g_sink;

// The distinct per-site constant keeps the bodies non-identical; taking each
// site's address (kSites[] below) additionally makes them ineligible for
// safe identical-code folding. Either alone suffices; both is belt-and-braces.
#define SITE(n)                                                      \
    __attribute__((noinline)) static void* site_##n() {             \
        void* p = malloc(256);                                       \
        g_sink = reinterpret_cast<uintptr_t>(p) + (n);              \
        return p;                                                    \
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
static constexpr int kNumSites = sizeof(kSites) / sizeof(kSites[0]);

int main() {
    auto arena_for = reinterpret_cast<arena_fn>(dlsym(RTLD_DEFAULT, "smash_arena_for"));
    if (!arena_for) {
        // Not running under libsmash — nothing to check. A no-op pass keeps
        // the binary runnable standalone; the ctest always sets the preload.
        fprintf(stderr, "FAIL: smash_arena_for not found (not running under libsmash)\n");
        return 1;
    }

    void* ptrs[kNumSites];
    for (int i = 0; i < kNumSites; ++i) ptrs[i] = kSites[i]();

    // Tally arenas. Bucket is small (kMaxArenas <= 64); use a fixed histogram.
    int hist[256];
    memset(hist, 0, sizeof(hist));
    int owned = 0, foreign = 0, distinct = 0, max_share = 0;
    for (int i = 0; i < kNumSites; ++i) {
        int a = arena_for(ptrs[i]);
        if (a < 0) { ++foreign; continue; }
        if (a >= 256) { fprintf(stderr, "FAIL: arena id %d out of range\n", a); return 1; }
        ++owned;
        if (hist[a]++ == 0) ++distinct;
    }
    for (int a = 0; a < 256; ++a) if (hist[a] > max_share) max_share = hist[a];

    printf("sites=%d owned=%d foreign=%d distinct_arenas=%d max_in_one=%d\n",
           kNumSites, owned, foreign, distinct, max_share);
    for (int a = 0; a < 256; ++a)
        if (hist[a]) printf("  arena %d: %d sites\n", a, hist[a]);

    // A handful of the earliest allocations can predate full smash init and
    // come back foreign; require most sites to be smash-owned so the check is
    // meaningful.
    if (owned < kNumSites * 3 / 4) {
        fprintf(stderr,
            "FAIL: only %d/%d sites were smash-owned — not exercising the slab "
            "arena path (wrong mode? not preloaded?)\n", owned, kNumSites);
        return 1;
    }

    // THE INVARIANT: distinct call sites must not collapse into one arena.
    // Buggy return-address capture makes this exactly 1.
    if (distinct < 2) {
        fprintf(stderr,
            "FAIL: %d smash-owned allocations from %d distinct call sites all "
            "routed to ONE arena — call-site arena routing has regressed "
            "(return address not reaching callsiteArena).\n", owned, kNumSites);
        return 1;
    }

    printf("PASS: %d distinct call sites routed across %d arenas\n",
           kNumSites, distinct);
    return 0;
}
