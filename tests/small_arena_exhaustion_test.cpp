// small_arena_exhaustion_test.cpp - small allocations must keep working after
// the VmRegion arena is exhausted.
//
// In full mode small objects live in slab spans carved from the VmRegion arena
// (SMASH_VM_GIB, 16 GiB by default). An empty span keeps its pages for its own
// (arena, size class), so a program that moves between size classes uses up a
// fresh share of the arena for each one, even with a small live set. When the
// arena ran out, Slab::allocateNewSpan returned nullptr — unlike LargeAlloc,
// small objects had no fallback — and operator new threw std::bad_alloc. A
// multithreaded application died with "terminate called recursively" (several
// threads throwing at once); SMASH_LARGE_ONLY=1, where small objects go to the
// system allocator, was unaffected. Reproduced here with a 1 GiB arena:
// 800 MiB of 64-byte objects, all freed, then 128-byte objects failed after
// 173 MiB. Spans now fall back to a direct mapping (not compressed).
//
// Registered with SMASH_VM_GIB=1 in full mode. Every object carries a unique
// pattern that is verified before it is freed, so the fallback spans are
// checked for integrity, not just for existing.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <new>
#include <vector>

static int failures = 0;

// Allocate `total` bytes of `sz`-byte objects, fill each with a pattern
// derived from its index, verify all of them, then free everything.
static bool phase(const char* name, size_t sz, size_t total, uint8_t salt) {
    std::vector<unsigned char*> v;
    v.reserve(total / sz);
    try {
        for (size_t got = 0; got < total; got += sz) {
            auto* p = new unsigned char[sz];
            const uint8_t tag = static_cast<uint8_t>(v.size() * 131 + salt);
            std::memset(p, tag, sz);
            v.push_back(p);
        }
    } catch (const std::bad_alloc&) {
        std::fprintf(stderr,
                     "FAIL: %s: std::bad_alloc after %zu MiB of %zu-byte objects. Small "
                     "objects must fall back to a direct mapping when the VmRegion arena "
                     "is exhausted (Slab::allocateNewSpan).\n",
                     name, v.size() * sz >> 20, sz);
        ++failures;
        for (auto* p : v) delete[] p;
        return false;
    }
    size_t bad = 0;
    for (size_t i = 0; i < v.size(); ++i) {
        const uint8_t tag = static_cast<uint8_t>(i * 131 + salt);
        for (size_t k = 0; k < sz; ++k)
            if (v[i][k] != tag) { ++bad; break; }
    }
    if (bad) {
        std::fprintf(stderr, "FAIL: %s: %zu of %zu objects corrupted\n", name, bad, v.size());
        ++failures;
    }
    std::printf("  %s: %zu MiB of %zu-byte objects, %zu corrupted\n",
                name, total >> 20, sz, bad);
    for (auto* p : v) delete[] p;
    return bad == 0;
}

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    const size_t total = 800u << 20;   // 80% of the 1 GiB arena per phase
    phase("phase A", 64, total, 1);
    phase("phase B", 128, total, 2);
    phase("phase C", 256, total, 3);
    if (failures) { std::printf("small_arena_exhaustion_test: %d FAILURES\n", failures); return 1; }
    std::printf("small_arena_exhaustion_test: all checks passed\n");
    return 0;
}
