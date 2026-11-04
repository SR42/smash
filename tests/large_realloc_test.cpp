// Regression test: large blocks smash places with direct mmap (above
// kMaxSmallSize but below kLargeAllocVmThreshold, or any large block in
// large-only mode) live outside the contiguous VmRegion arena and are
// registered only in page_map_. xxowns() must still claim them; otherwise
// alloc8 treats them as foreign and forwards realloc / malloc_size / free to
// the system allocator. That made malloc_size() return 0 and realloc() copy
// past the end of the block (SIGSEGV/SIGBUS), which crashed every Rust
// program (Vec growth is a doubling realloc chain) under DYLD_INSERT_LIBRARIES.
//
// Must run under interposition (DYLD_INSERT_LIBRARIES / LD_PRELOAD): the bug
// lives in the alloc8 ownership dispatch, which a direct link never exercises.
// Run in both full and large-only modes.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#if defined(__APPLE__)
#include <malloc/malloc.h>
static size_t usable(void* p) { return malloc_size(p); }
#else
#include <malloc.h>
static size_t usable(void* p) { return malloc_usable_size(p); }
#endif

static int failures = 0;

static void check(bool ok, const char* what, size_t n) {
    if (!ok) {
        std::fprintf(stderr, "FAIL: %s (size %zu)\n", what, n);
        ++failures;
    }
}

static unsigned char pat(size_t i) { return static_cast<unsigned char>(i * 131 + 7); }

int main() {
    // 1. Doubling realloc chain from 8 B to 4 MiB, the shape of Rust's
    //    RawVec growth: contents must survive and usable size must cover
    //    the request at every step (small slab -> mmap'd large -> VmRegion).
    size_t n = 8;
    auto* p = static_cast<unsigned char*>(std::malloc(n));
    for (size_t i = 0; i < n; ++i) p[i] = pat(i);
    while (n < (size_t{4} << 20)) {
        size_t m = n * 2;
        p = static_cast<unsigned char*>(std::realloc(p, m));
        check(p != nullptr, "realloc returned NULL", m);
        if (!p) return 1;
        check(usable(p) >= m, "usable size < requested after realloc", m);
        bool same = true;
        for (size_t i = 0; i < n; ++i) same &= (p[i] == pat(i));
        check(same, "contents lost across realloc", m);
        for (size_t i = n; i < m; ++i) p[i] = pat(i);
        n = m;
    }
    std::free(p);

    // 2. Two live large blocks of each size must not overlap, and each must
    //    report a usable size covering the request.
    const size_t sizes[] = {20000, 32768, 65536, 131072, 500000, 2u << 20};
    for (size_t sz : sizes) {
        auto* a = static_cast<unsigned char*>(std::malloc(sz));
        auto* b = static_cast<unsigned char*>(std::malloc(sz));
        check(a && b, "malloc returned NULL", sz);
        if (!a || !b) return 1;
        check(usable(a) >= sz && usable(b) >= sz, "usable size < requested", sz);
        auto ua = reinterpret_cast<uintptr_t>(a), ub = reinterpret_cast<uintptr_t>(b);
        check(ua + sz <= ub || ub + sz <= ua, "live large blocks overlap", sz);
        std::memset(a, 0xA5, sz);
        std::memset(b, 0x5A, sz);
        check(a[sz - 1] == 0xA5 && b[0] == 0x5A, "blocks clobbered each other", sz);
        // Shrinking realloc keeps the prefix.
        a = static_cast<unsigned char*>(std::realloc(a, sz / 2));
        check(a && a[0] == 0xA5 && a[sz / 2 - 1] == 0xA5, "shrinking realloc lost data", sz);
        std::free(a);
        std::free(b);
    }

    if (failures) {
        std::fprintf(stderr, "large_realloc_test: %d failure(s)\n", failures);
        return 1;
    }
    std::printf("large_realloc_test: OK\n");
    return 0;
}
