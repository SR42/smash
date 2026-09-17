// malloc_override_test.cpp - does the malloc override actually take effect,
// and does each mode route allocations where it claims to?
//
// Ported from mimalloc's `test/main-override.c`. The original allocates
// through plain malloc/free/strdup/realloc and asserts every pointer lands in
// mimalloc's own heap (`mi_is_in_heap_region`) — i.e. that interposition
// really happened rather than the calls falling through to libc. The
// allocation sequence here is the original's, unchanged; the ownership
// predicate is smash's exported equivalent, `smash_arena_for()`, which returns
// an arena id for a smash-managed pointer and -1 otherwise.
//
// Dropped from the original, with reasons: `mi_version()` (existed only to
// force the mimalloc link), `_expand()` (MSVC-only), `mi_stats_print()`
// (mimalloc's reporting; smash uses SMASH_STATS=1), and the cross-API section
// that checked mimalloc's API and the overridden malloc were interchangeable —
// smash has no second API, it *is* malloc, so that degenerates to a
// malloc/free pair and is kept only as extra churn.
//
// Stronger than a literal translation in one respect: the original could only
// test a single mode. This asserts opposite truths per mode, so it guards
// large-only *routing* rather than just interposition —
//   full mode:  a small allocation MUST be smash-owned;
//   large-only: a small allocation must NOT be (it passes through to the
//               system allocator), while a >= 1 MiB block MUST be owned.
// A test that only ever asserts "owned" would pass vacuously in large-only.
//
// Verified as a real discriminator: inverting the full-mode sense makes it
// fail (exit 1), and `smash_arena_for()` returns -1 for stack addresses, raw
// mmap results and NULL, while returning a real arena id for malloc'd memory.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>

namespace {

using arena_fn = int (*)(const void*);
arena_fn g_arena_for = nullptr;

bool owned(void* p) { return g_arena_for && g_arena_for(p) >= 0; }

int fail(const char* what) {
    std::fprintf(stderr, "FAIL: %s\n", what);
    return 1;
}

}  // namespace

int main() {
    // Full mode can compress a cold stdout buffer and flush it as NUL at exit,
    // which would eat this test's verdict line while it still exits 0.
    setvbuf(stdout, nullptr, _IONBF, 0);

    g_arena_for = reinterpret_cast<arena_fn>(dlsym(RTLD_DEFAULT, "smash_arena_for"));
    if (!g_arena_for) {
        // Runnable standalone; the ctest registrations always set the preload.
        std::printf("SKIP: smash_arena_for not found (not running under libsmash)\n");
        return 0;
    }

    const char* lo = std::getenv("SMASH_LARGE_ONLY");
    const bool large_only = (lo && lo[0] == '1');
    std::printf("malloc_override_test: mode=%s\n", large_only ? "large-only" : "full");

    // ── the original sequence, unchanged ────────────────────────────────────
    void* p1 = malloc(78);
    if (!p1) return fail("malloc(78) returned null");
    if (!large_only && !owned(p1)) return fail("malloc(78) not smash-owned in full mode");
    if (large_only && owned(p1))
        return fail("malloc(78) smash-owned in large-only mode (should pass through)");

    void* p2 = malloc(24);
    if (!p2) return fail("malloc(24) returned null");
    if (!large_only && !owned(p2)) return fail("malloc(24) not smash-owned in full mode");

    free(p1);
    p1 = malloc(8);
    char* s = strdup("hello\n");
    if (!p1 || !s) return fail("malloc(8) / strdup returned null");
    if (std::strcmp(s, "hello\n") != 0) return fail("strdup content corrupted");

    free(p2);
    p2 = malloc(16);
    void* p3 = realloc(p1, 32);
    if (p3 != nullptr) p1 = p3;
    if (!p1 || !p2) return fail("realloc / malloc(16) returned null");

    std::memset(p1, 0xab, 32);
    std::memset(p2, 0xcd, 16);
    if (static_cast<unsigned char*>(p1)[31] != 0xab) return fail("realloc'd block not writable");
    free(p1);
    free(p2);
    free(s);

    // Cross-API section from the original; degenerates to malloc/free here.
    p1 = malloc(32); free(p1);
    p2 = malloc(32); free(p2);

    // ── mode-independent: a large block is smash-managed in BOTH modes ──────
    // Above kLargeAllocVmThreshold, so it enters the compressible VmRegion
    // whether or not small allocations are passed through.
    const size_t big = 2u << 20;  // 2 MiB
    void* large = malloc(big);
    if (!large) return fail("malloc(2 MiB) returned null");
    if (!owned(large)) return fail("2 MiB block not smash-owned");
    std::memset(large, 0x5a, big);
    if (static_cast<unsigned char*>(large)[big - 1] != 0x5a)
        return fail("2 MiB block not writable to its last byte");
    free(large);

    std::printf("PASS: %s\n",
                large_only ? "large-only — small bypassed, large owned"
                           : "full — small and large owned");
    return 0;
}
