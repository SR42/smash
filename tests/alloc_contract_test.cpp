// The allocation contract: a satisfiable request must never fail, and an
// unsatisfiable one must fail (testHugeSizes, at the end).
//
// This is the direct regression test for the reported aarch64 symptom — an
// application running under libsmash throwing std::bad_alloc. There is
// exactly one way smash produces that exception: the operator-new wrappers in
// src/linux_syscall_wrappers.cpp are
//
//     void* _Znwm_GLIBCXX_3_4(size_t sz) {
//         void* ptr = xxmalloc(sz);
//         if (!ptr) throw std::bad_alloc();
//         return ptr;
//     }
//
// so std::bad_alloc means smash's malloc returned nullptr. This test sweeps
// every allocation entry point across every size and alignment boundary in
// smash's routing logic and asserts that none of them ever return nullptr
// (or throw) for a request the machine can obviously satisfy, and that what
// comes back is correctly aligned, fully writable, and preserves its bytes.
//
// The boundaries swept are the ones that decide which internal path a request
// takes, i.e. exactly where an arch-dependent geometry bug would hide:
//   * every size class edge, and +/-1 around each (slab vs. thread cache);
//   * kMaxSmallSize = 16 KiB (slab vs. LargeAlloc; also the large-only
//     passthrough threshold);
//   * kLargeAllocVmThreshold = 1 MiB (LargeAlloc direct-mmap vs. VmRegion);
//   * page multiples +/-1 for 4 KiB, 16 KiB and 64 KiB, so the sweep straddles
//     the real page size of every aarch64 kernel configuration regardless of
//     what smash was compiled for;
//   * alignment > kMinAlignment (16), which routes ANY size — including a
//     64-byte object — through LargeAlloc's page-granular mmap path.
//
// Run under LD_PRELOAD / DYLD_INSERT_LIBRARIES in both SMASH_LARGE_ONLY=0
// (full) and SMASH_LARGE_ONLY=1 (production) modes, following the same
// one-body-two-modes pattern as mode_compression_test.cpp: the mode is read
// from the environment, never a compile flag, so the two registrations
// provably execute identical assertions.

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <new>
#include <unistd.h>
#include <utility>
#include <vector>

#if defined(__linux__)
#include <csignal>
#include <sys/resource.h>
#include <sys/wait.h>
#endif

#if defined(__APPLE__)
#include <malloc/malloc.h>
#define USABLE_SIZE(p) malloc_size(p)
#else
#include <malloc.h>
#define USABLE_SIZE(p) malloc_usable_size(p)
#endif

namespace {

int failures = 0;

#define CHECK(cond, msg, ...) do { \
    if (!(cond)) { \
        fprintf(stderr, "FAIL: " msg "\n", ##__VA_ARGS__); \
        ++failures; \
    } \
} while (0)

// Report a bad_alloc with the exact request that produced it. This is the
// message that names the failing size when the aarch64 bug reproduces.
#define REPORT_BAD_ALLOC(what, size, align) do { \
    fprintf(stderr, \
        "FAIL: std::bad_alloc from %s(size=%zu, align=%zu) — smash's malloc " \
        "returned nullptr for a satisfiable request\n", \
        (what), static_cast<size_t>(size), static_cast<size_t>(align)); \
    ++failures; \
} while (0)

unsigned char patternByte(size_t size, size_t i) {
    const unsigned char seed = static_cast<unsigned char>((size * 31 + 7) & 0xff);
    return static_cast<unsigned char>(seed + (i & 0x0f));
}

void fillPattern(void* p, size_t size) {
    auto* b = static_cast<unsigned char*>(p);
    for (size_t i = 0; i < size; ++i) b[i] = patternByte(size, i);
}

bool verifyPattern(const void* p, size_t size) {
    auto* b = static_cast<const unsigned char*>(p);
    for (size_t i = 0; i < size; ++i) {
        if (b[i] != patternByte(size, i)) {
            fprintf(stderr,
                    "FAIL: block of %zu bytes corrupted at offset %zu "
                    "(got 0x%02x want 0x%02x)\n",
                    size, i, b[i], patternByte(size, i));
            ++failures;
            return false;
        }
    }
    return true;
}

// Write the whole block and read it straight back. Touching every byte makes
// a nominally-successful allocation backed by uncommitted (PROT_NONE) pages
// fault here rather than silently corrupt the caller, and catches a usable
// size that overlaps a neighbouring block.
bool fillAndVerify(void* p, size_t size) {
    fillPattern(p, size);
    return verifyPattern(p, size);
}

void checkUsable(void* p, size_t requested, const char* what) {
    size_t usable = USABLE_SIZE(p);
    // A zero answer means "not my pointer" from the system allocator, which
    // is itself a routing bug: the block came from smash, so smash's getSize
    // must recognise it.
    CHECK(usable >= requested,
          "%s(%zu) returned a block whose usable size is %zu (< requested). "
          "getSize() failed to route this pointer",
          what, requested, usable);
}

// ── The size sweep ──────────────────────────────────────────────────────────
// Built at runtime so it covers the boundaries symbolically rather than as a
// frozen list.
std::vector<size_t> buildSizes() {
    std::vector<size_t> s;
    auto add = [&s](size_t v) { if (v > 0) s.push_back(v); };
    auto addAround = [&](size_t v) { add(v - 1); add(v); add(v + 1); };

    // Size-class edges (config.h kClassSizes) and their neighbours.
    static const size_t kClassEdges[] = {
        16,    32,    48,    64,    80,    96,    112,   128,
        160,   192,   224,   256,
        320,   384,   448,   512,
        640,   768,   896,   1024,
        1280,  1536,  1792,  2048,
        2560,  3072,  3584,  4096,
        5120,  6144,  7168,  8192,
        10240, 12288, 14336, 16384,
    };
    for (size_t e : kClassEdges) addAround(e);

    // kMaxSmallSize (16 KiB): slab / LargeAlloc split, and the large-only
    // passthrough threshold.
    addAround(16 * 1024);

    // Page multiples for every plausible kernel page size. A 4 KiB-compiled
    // smash on a 16 KiB or 64 KiB aarch64 kernel mis-rounds exactly here.
    for (size_t page : {size_t{4096}, size_t{16384}, size_t{65536}}) {
        for (size_t n : {size_t{1}, size_t{2}, size_t{3}, size_t{8}, size_t{17}})
            addAround(page * n);
    }

    // kLargeAllocVmThreshold (1 MiB): direct mmap / VmRegion split.
    addAround(1024 * 1024);
    add(1024 * 1024 + 4096);
    add(2 * 1024 * 1024);
    add(5 * 1024 * 1024);
    add(8 * 1024 * 1024);

    return s;
}

// ── malloc / free ───────────────────────────────────────────────────────────
void testMalloc(const std::vector<size_t>& sizes) {
    for (size_t n : sizes) {
        void* p = std::malloc(n);
        if (!p) {
            fprintf(stderr,
                    "FAIL: malloc(%zu) returned nullptr — operator new on this "
                    "size would throw std::bad_alloc\n", n);
            ++failures;
            continue;
        }
        checkUsable(p, n, "malloc");
        fillAndVerify(p, n);
        std::free(p);
    }

    // malloc(0) must return either nullptr or a freeable pointer; it must not
    // hand back something free() rejects.
    void* z = std::malloc(0);
    if (z) std::free(z);
}

// ── operator new / new[] ────────────────────────────────────────────────────
// The path that actually throws. Exercised separately from malloc because the
// GLIBCXX-versioned aliases in linux_syscall_wrappers.cpp are distinct symbols
// from the plain operators.
void testOperatorNew(const std::vector<size_t>& sizes) {
    for (size_t n : sizes) {
        try {
            char* p = new char[n];
            fillAndVerify(p, n);
            delete[] p;
        } catch (const std::bad_alloc&) {
            REPORT_BAD_ALLOC("operator new[]", n, 0);
        }

        try {
            void* p = ::operator new(n);
            fillAndVerify(p, n);
            ::operator delete(p);
        } catch (const std::bad_alloc&) {
            REPORT_BAD_ALLOC("operator new", n, 0);
        }
    }

    // The nothrow forms must return nullptr rather than throw — but for a
    // satisfiable request they must not return nullptr either.
    for (size_t n : sizes) {
        void* p = ::operator new(n, std::nothrow);
        CHECK(p != nullptr, "operator new(%zu, nothrow) returned nullptr", n);
        if (p) { fillAndVerify(p, n); ::operator delete(p, std::nothrow); }
    }
}

// ── over-aligned allocation ─────────────────────────────────────────────────
// SmashHeap::memalign() sends every request with alignment > kMinAlignment
// (16) straight to LargeAlloc, which rounds the alignment up to a full page.
// So `new (align_val_t(64)) T` for a 64-byte T consumes an entire page. This
// sweep asserts correctness (never null, actually aligned, fully writable);
// the footprint cost of that routing is measured by
// aligned_alloc_footprint_test.
void testAlignedNew() {
    static const size_t kAligns[] = {
        16, 32, 64, 128, 256, 512, 1024, 2048,
        4096, 8192, 16384, 32768, 65536,
    };
    static const size_t kSizes[] = {
        1, 8, 64, 100, 1000, 4096, 16384, 65536, 1024 * 1024,
    };

    for (size_t a : kAligns) {
        for (size_t n : kSizes) {
            try {
                void* p = ::operator new(n, std::align_val_t(a));
                if (!p) {
                    fprintf(stderr,
                            "FAIL: operator new(%zu, align %zu) returned "
                            "nullptr\n", n, a);
                    ++failures;
                    continue;
                }
                CHECK((reinterpret_cast<uintptr_t>(p) & (a - 1)) == 0,
                      "operator new(%zu, align %zu) returned %p, misaligned",
                      n, a, p);
                fillAndVerify(p, n);
                ::operator delete(p, std::align_val_t(a));
            } catch (const std::bad_alloc&) {
                REPORT_BAD_ALLOC("operator new(align_val_t)", n, a);
            }
        }
    }
}

// ── posix_memalign / aligned_alloc / memalign ───────────────────────────────
void testPosixAligned() {
    static const size_t kAligns[] = {
        sizeof(void*), 16, 32, 64, 256, 4096, 16384, 65536,
    };
    static const size_t kSizes[] = {1, 64, 1000, 4096, 16384, 1024 * 1024};

    for (size_t a : kAligns) {
        for (size_t n : kSizes) {
            void* p = nullptr;
            int rc = posix_memalign(&p, a, n);
            CHECK(rc == 0 && p != nullptr,
                  "posix_memalign(align=%zu, size=%zu) failed rc=%d (%s)",
                  a, n, rc, std::strerror(rc));
            if (rc != 0 || !p) continue;
            CHECK((reinterpret_cast<uintptr_t>(p) & (a - 1)) == 0,
                  "posix_memalign(align=%zu, size=%zu) returned %p, misaligned",
                  a, n, p);
            checkUsable(p, n, "posix_memalign");
            fillAndVerify(p, n);
            std::free(p);
        }
    }

    // aligned_alloc: any size, a multiple of the alignment or not. C17 (DR 460)
    // dropped the multiple rule and glibc never had it; alloc8's wrapper
    // returned nullptr for aligned_alloc(64, 100), with errno untouched.
    void* (*volatile p_aligned_alloc)(size_t, size_t) = aligned_alloc;
    for (size_t a : kAligns) {
        if (a < sizeof(void*)) continue;
        for (size_t n : {a, 3 * a, 16 * a, size_t{1}, size_t{100}, a + 1, 3 * a / 2 + 7,
                         size_t{3} << 20}) {
            void* p = p_aligned_alloc(a, n);
            CHECK(p != nullptr, "aligned_alloc(align=%zu, size=%zu) returned "
                  "nullptr", a, n);
            if (!p) continue;
            CHECK((reinterpret_cast<uintptr_t>(p) & (a - 1)) == 0,
                  "aligned_alloc(align=%zu, size=%zu) returned %p, misaligned",
                  a, n, p);
            checkUsable(p, n, "aligned_alloc");
            fillAndVerify(p, n);
            std::free(p);
        }
    }
    // An alignment that is not a power of two is EINVAL (glibc 2.38+, C17's
    // "not supported"); the wrapper passed aligned_alloc(24, 48) through and
    // full mode returned a block that is not 24-aligned.
    for (size_t a : {size_t{0}, size_t{3}, size_t{24}, size_t{48}, size_t{4097}}) {
        errno = 0;
        void* p = p_aligned_alloc(a, 48);
        CHECK(!p && errno == EINVAL, "aligned_alloc(align=%zu, size=48) = %p errno=%d, want "
              "nullptr with EINVAL", a, p, errno);
        if (p) std::free(p);
    }

#if !defined(__APPLE__)
    // pvalloc rounds the size up to a whole page.
    const size_t page = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    for (size_t n : {size_t{1}, size_t{100}, page, page + 1, size_t{3} << 20}) {
        void* p = pvalloc(n);
        CHECK(p != nullptr, "pvalloc(%zu) returned nullptr", n);
        if (!p) continue;
        CHECK((reinterpret_cast<uintptr_t>(p) & (page - 1)) == 0,
              "pvalloc(%zu) returned %p, not page-aligned", n, p);
        const size_t rounded = (n + page - 1) & ~(page - 1);
        checkUsable(p, rounded, "pvalloc");
        fillAndVerify(p, rounded);
        std::free(p);
    }
#endif
}

// ── calloc ──────────────────────────────────────────────────────────────────
void testCalloc(const std::vector<size_t>& sizes) {
    for (size_t n : sizes) {
        void* p = std::calloc(1, n);
        CHECK(p != nullptr, "calloc(1, %zu) returned nullptr", n);
        if (!p) continue;
        auto* b = static_cast<unsigned char*>(p);
        bool zeroed = true;
        size_t bad = 0;
        for (size_t i = 0; i < n; ++i) {
            if (b[i] != 0) { zeroed = false; bad = i; break; }
        }
        CHECK(zeroed, "calloc(1, %zu) block is non-zero at offset %zu (0x%02x)",
              n, bad, zeroed ? 0 : b[bad]);
        fillAndVerify(p, n);
        std::free(p);
    }

    // Element-count form, including a case where count*size crosses the
    // small/large boundary.
    for (size_t count : {size_t{1}, size_t{64}, size_t{4096}}) {
        void* p = std::calloc(count, 64);
        CHECK(p != nullptr, "calloc(%zu, 64) returned nullptr", count);
        if (p) { fillAndVerify(p, count * 64); std::free(p); }
    }
}

// ── realloc growth and shrink chains ────────────────────────────────────────
// realloc walks a block across every routing boundary in one lifetime:
// thread cache -> slab -> LargeAlloc direct mmap -> VmRegion, and back down.
// Each step must preserve the prefix bytes.
void testReallocChain() {
    const size_t kMax = 4 * 1024 * 1024;
    size_t n = 8;
    void* p = std::malloc(n);
    CHECK(p != nullptr, "realloc chain: initial malloc(%zu) failed", n);
    if (!p) return;
    std::memset(p, 0xC7, n);

    while (n < kMax) {
        size_t next = n * 2 + 17;   // odd growth so sizes land off boundaries
        void* q = std::realloc(p, next);
        if (!q) {
            fprintf(stderr, "FAIL: realloc(%zu -> %zu) returned nullptr\n",
                    n, next);
            ++failures;
            std::free(p);
            return;
        }
        p = q;
        auto* b = static_cast<unsigned char*>(p);
        for (size_t i = 0; i < n; ++i) {
            if (b[i] != 0xC7) {
                fprintf(stderr,
                        "FAIL: realloc(%zu -> %zu) lost byte %zu (0x%02x)\n",
                        n, next, i, b[i]);
                ++failures;
                std::free(p);
                return;
            }
        }
        std::memset(p, 0xC7, next);
        n = next;
    }

    // Shrink all the way back down.
    while (n > 8) {
        size_t next = n / 3;
        if (next < 8) next = 8;
        void* q = std::realloc(p, next);
        if (!q) {
            fprintf(stderr, "FAIL: realloc(%zu -> %zu) shrink returned "
                    "nullptr\n", n, next);
            ++failures;
            std::free(p);
            return;
        }
        p = q;
        auto* b = static_cast<unsigned char*>(p);
        for (size_t i = 0; i < next; ++i) {
            if (b[i] != 0xC7) {
                fprintf(stderr,
                        "FAIL: realloc shrink(%zu -> %zu) lost byte %zu\n",
                        n, next, i);
                ++failures;
                std::free(p);
                return;
            }
        }
        n = next;
    }

    // realloc(p, 0) frees and returns nullptr.
    void* q = std::realloc(p, 0);
    CHECK(q == nullptr, "realloc(p, 0) returned %p, expected nullptr", q);
    if (q) std::free(q);

    // realloc(nullptr, n) is malloc(n).
    void* r = std::realloc(nullptr, 12345);
    CHECK(r != nullptr, "realloc(nullptr, 12345) returned nullptr");
    if (r) { fillAndVerify(r, 12345); std::free(r); }
}

// ── sustained mixed churn ───────────────────────────────────────────────────
// A single sweep only ever touches a cold allocator. This drives enough
// traffic to refill and drain the thread cache, recycle slab spans, and cycle
// VmRegion pages through the free list — the state where a geometry bug turns
// into an exhausted arena and a nullptr.
void testChurn() {
    const int kRounds = 24;
    const int kLive = 512;
    std::vector<void*> live(kLive, nullptr);
    std::vector<size_t> live_size(kLive, 0);

    uint64_t rng = 0x9E3779B97F4A7C15ull;
    auto next = [&rng]() {
        rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17;
        return rng;
    };

    for (int round = 0; round < kRounds; ++round) {
        for (int i = 0; i < kLive; ++i) {
            if (live[i]) {
                if (!verifyPattern(live[i], live_size[i])) return;
                std::free(live[i]);
                live[i] = nullptr;
            }
            // Mix: mostly small (slab / thread cache), some large enough to
            // reach LargeAlloc and the VmRegion.
            uint64_t r = next();
            size_t n;
            switch (r % 8) {
                case 0:  n = 1 + (r >> 8) % (2 * 1024 * 1024); break;  // large
                case 1:  n = 16 * 1024 + (r >> 8) % 4096; break;       // boundary
                default: n = 1 + (r >> 8) % 16384; break;              // small
            }
            void* p = std::malloc(n);
            if (!p) {
                fprintf(stderr,
                        "FAIL: churn round %d slot %d: malloc(%zu) returned "
                        "nullptr\n", round, i, n);
                ++failures;
                return;
            }
            live[i] = p;
            // Stamp the pattern now; the next round verifies it before
            // freeing, so the block must survive intervening allocator
            // traffic intact. Multi-MiB blocks only get their ends touched —
            // the point is allocator state, not page-fault throughput.
            if (n <= 64 * 1024) {
                live_size[i] = n;
                fillPattern(p, n);
            } else {
                live_size[i] = 0;
                auto* b = static_cast<unsigned char*>(p);
                b[0] = 0x1;
                b[n - 1] = 0x2;
            }
        }
    }

    for (int i = 0; i < kLive; ++i) if (live[i]) std::free(live[i]);
}

// Is libsmash actually interposed? These tests are meaningful standalone (the
// contract holds for any allocator) but only prove something about smash when
// the preload took effect, so say so plainly in the banner. Matches
// arena_routing_test.cpp, which probes the same export.
bool smashInterposed() {
    return dlsym(RTLD_DEFAULT, "smash_arena_for") != nullptr;
}

void printInterposeBanner() {
    if (smashInterposed()) return;
    printf("WARNING: libsmash is not interposed (smash_arena_for not found) — "
           "this run exercises the system allocator and proves nothing about "
           "smash. The ctest registration sets LD_PRELOAD / "
           "DYLD_INSERT_LIBRARIES.\n");
}

const char* modeName() {
    const char* e = std::getenv("SMASH_LARGE_ONLY");
    return (e && e[0] == '1') ? "large-only" : "full";
}

} // namespace

// ── operator new and the new_handler ────────────────────────────────────────
// An operator new that cannot allocate calls the installed std::new_handler
// and tries again; only with no handler left does it throw std::bad_alloc, and
// the nothrow forms return nullptr where the throwing ones throw
// ([new.delete.single]; libstdc++'s do). alloc8's and smash's operators threw
// at once without calling it, so a program whose handler frees a cache got
// bad_alloc instead of its memory. Checked through the operators as this
// binary binds them (smash's GLIBCXX/CXXABI-versioned aliases) and, on Linux,
// looked up by plain name (alloc8's default-version ones).
namespace nh {
int calls = 0;
int uninstall_at = 0;
void* reserve = nullptr;
constexpr size_t kReserveAlign = size_t{2} << 20;
struct Custom : std::bad_alloc {};
void counting() { if (++calls >= uninstall_at) std::set_new_handler(nullptr); }
void throwing() { ++calls; throw Custom(); }
void releasing() {   // frees the reserve once, then gives up
    ++calls;
    if (!reserve) { std::set_new_handler(nullptr); return; }
    ::operator delete(reserve, std::align_val_t(kReserveAlign));
    reserve = nullptr;
}

struct NewSet {
    const char* name;
    void* (*plain)(size_t);
    void* (*array)(size_t);
    void* (*plain_nt)(size_t, const std::nothrow_t&);
    void* (*array_nt)(size_t, const std::nothrow_t&);
    void* (*al)(size_t, std::align_val_t);
    void* (*al_array)(size_t, std::align_val_t);
    void* (*al_nt)(size_t, std::align_val_t, const std::nothrow_t&);
    void* (*al_array_nt)(size_t, std::align_val_t, const std::nothrow_t&);
};

NewSet boundSet() {
    return {"as bound",
            static_cast<void* (*)(size_t)>(::operator new),
            static_cast<void* (*)(size_t)>(::operator new[]),
            static_cast<void* (*)(size_t, const std::nothrow_t&)>(::operator new),
            static_cast<void* (*)(size_t, const std::nothrow_t&)>(::operator new[]),
            static_cast<void* (*)(size_t, std::align_val_t)>(::operator new),
            static_cast<void* (*)(size_t, std::align_val_t)>(::operator new[]),
            static_cast<void* (*)(size_t, std::align_val_t, const std::nothrow_t&)>(::operator new),
            static_cast<void* (*)(size_t, std::align_val_t, const std::nothrow_t&)>(::operator new[])};
}

#if defined(__linux__)
template <typename T> T byName(const char* sym) { return reinterpret_cast<T>(dlsym(RTLD_DEFAULT, sym)); }
NewSet nameSet() {
    NewSet s = boundSet();
    s.name = "by name";
    s.plain = byName<decltype(s.plain)>("_Znwm");
    s.array = byName<decltype(s.array)>("_Znam");
    s.plain_nt = byName<decltype(s.plain_nt)>("_ZnwmRKSt9nothrow_t");
    s.array_nt = byName<decltype(s.array_nt)>("_ZnamRKSt9nothrow_t");
    s.al = byName<decltype(s.al)>("_ZnwmSt11align_val_t");
    s.al_array = byName<decltype(s.al_array)>("_ZnamSt11align_val_t");
    s.al_nt = byName<decltype(s.al_nt)>("_ZnwmSt11align_val_tRKSt9nothrow_t");
    s.al_array_nt = byName<decltype(s.al_array_nt)>("_ZnamSt11align_val_tRKSt9nothrow_t");
    return s;
}
#endif

// One operator over a size nothing can serve: a handler that removes itself on
// its third call is called three times, then the operator fails; a handler
// that throws is passed through by the throwing forms and turned into nullptr
// by the nothrow ones.
template <typename F>
void expectLoop(const char* set, const char* what, bool nothrow, F call) {
    calls = 0;
    uninstall_at = 3;
    std::set_new_handler(counting);
    void* p = nullptr;
    bool threw = false;
    try { p = call(); } catch (const std::bad_alloc&) { threw = true; }
    std::set_new_handler(nullptr);
    CHECK(calls == 3 && !p && threw != nothrow,
          "%s %s: the new_handler was called %d times (want 3, until it removed itself), then %s",
          set, what, calls, p ? "a block" : threw ? "bad_alloc" : "nullptr");

    calls = 0;
    std::set_new_handler(throwing);
    bool custom = false;
    threw = false;
    p = nullptr;
    try { p = call(); } catch (const Custom&) { custom = true; } catch (const std::bad_alloc&) { threw = true; }
    std::set_new_handler(nullptr);
    CHECK(calls == 1 && !p && custom != nothrow && !threw,
          "%s %s with a throwing new_handler: %d call(s), then %s (want 1, then %s)", set, what, calls,
          p ? "a block" : custom ? "its exception" : threw ? "plain bad_alloc" : "nullptr",
          nothrow ? "nullptr" : "its exception");
}

void checkLoop(const NewSet& s) {
    static volatile size_t huge = SIZE_MAX / 2;
    const std::align_val_t al{64};
    expectLoop(s.name, "operator new", false, [&] { return s.plain(huge); });
    expectLoop(s.name, "operator new[]", false, [&] { return s.array(huge); });
    expectLoop(s.name, "operator new(nothrow)", true, [&] { return s.plain_nt(huge, std::nothrow); });
    expectLoop(s.name, "operator new[](nothrow)", true, [&] { return s.array_nt(huge, std::nothrow); });
    expectLoop(s.name, "operator new(align)", false, [&] { return s.al(huge, al); });
    expectLoop(s.name, "operator new[](align)", false, [&] { return s.al_array(huge, al); });
    expectLoop(s.name, "operator new(align, nothrow)", true, [&] { return s.al_nt(huge, al, std::nothrow); });
    expectLoop(s.name, "operator new[](align, nothrow)", true,
               [&] { return s.al_array_nt(huge, al, std::nothrow); });
}

#if defined(__linux__)
long vmSizeKiB() {
    FILE* f = fopen("/proc/self/status", "r");
    char line[256];
    long v = -1;
    while (f && fgets(line, sizeof line, f))
        if (!strncmp(line, "VmSize:", 7)) { v = atol(line + 7); break; }
    if (f) fclose(f);
    return v;
}

// A handler that frees memory lets the retry succeed. In a child (one thread,
// so nothing else maps memory meanwhile) whose address space is capped 32 MiB
// above what it uses, a 64 MiB block at 2 MiB alignment (a direct mapping in
// glibc and smash alike) does not fit until the handler frees a 64 MiB
// reserve: exactly one call, then the block.
int retryChild(const NewSet& s) {
    constexpr size_t kSize = size_t{64} << 20;
    const std::align_val_t al{kReserveAlign};
    struct Form { const char* what; void* (*call)(const NewSet&, size_t, std::align_val_t); };
    const Form forms[] = {
        {"operator new(align)", [](const NewSet& n, size_t z, std::align_val_t a) { return n.al(z, a); }},
        {"operator new[](align)", [](const NewSet& n, size_t z, std::align_val_t a) { return n.al_array(z, a); }},
        {"operator new(align, nothrow)",
         [](const NewSet& n, size_t z, std::align_val_t a) { return n.al_nt(z, a, std::nothrow); }},
        {"operator new[](align, nothrow)",
         [](const NewSet& n, size_t z, std::align_val_t a) { return n.al_array_nt(z, a, std::nothrow); }},
    };
    int bad = 0;
    for (const Form& f : forms) {
        reserve = ::operator new(kSize, al);
        rlimit old{};
        getrlimit(RLIMIT_AS, &old);
        const rlimit cap{static_cast<rlim_t>(vmSizeKiB()) * 1024 + (rlim_t{32} << 20), old.rlim_max};
        setrlimit(RLIMIT_AS, &cap);
        calls = 0;
        std::set_new_handler(releasing);
        void* p = nullptr;
        try { p = f.call(s, kSize, al); } catch (const std::bad_alloc&) {}
        std::set_new_handler(nullptr);
        setrlimit(RLIMIT_AS, &old);
        if (!p || calls != 1) {
            fprintf(stderr, "FAIL: %s %s with a new_handler that frees 64 MiB: %s after %d call(s) "
                    "(want the block after 1)\n", s.name, f.what, p ? "the block" : "no block", calls);
            ++bad;
        }
        if (p) ::operator delete(p, al);
        if (reserve) ::operator delete(reserve, al);
        reserve = nullptr;
    }
    return bad;
}
#endif
}  // namespace nh

void testNewHandler() {
    const nh::NewSet bound = nh::boundSet();
    nh::checkLoop(bound);
#if defined(__linux__)
    const nh::NewSet named = nh::nameSet();
    const bool have_named = named.plain && named.array && named.plain_nt && named.array_nt &&
                            named.al && named.al_array && named.al_nt && named.al_array_nt;
    CHECK(have_named, "operator new by name: dlsym found only some of the eight");
    if (have_named) nh::checkLoop(named);
    const pid_t pid = fork();
    if (pid == 0) _exit(nh::retryChild(bound) + (have_named ? nh::retryChild(named) : 0));
    int status = 0;
    bool done = false;
    for (int i = 0; i < 6000 && !done; ++i) {   // 60 s
        done = waitpid(pid, &status, WNOHANG) == pid;
        if (!done) usleep(10000);
    }
    if (!done) { kill(pid, SIGKILL); waitpid(pid, &status, 0); }
    CHECK(done && WIFEXITED(status) && WEXITSTATUS(status) == 0,
          "new_handler retry child: %s (status 0x%x)", done ? "failed checks" : "hung, killed at 60 s", status);
    printf("new_handler: 8 operators x %d sets loop and pass a throw through; the retry gets the "
           "block once the handler frees memory\n", have_named ? 2 : 1);
#else
    printf("new_handler: 8 operators loop and pass a throw through\n");
#endif
}

// ── Sizes no allocator can serve ────────────────────────────────────────────
// The other half of the contract: an unsatisfiable request fails, with
// ENOMEM, as glibc's does, and never yields a block shorter than asked for.
// LargeAlloc counted pages in 32 bits and rounded without an overflow check,
// so malloc(16 TiB + 4 KiB) returned one page whose malloc_usable_size was
// 16 TiB, malloc(2^62) returned a block too, and posix_memalign(2 MiB,
// SIZE_MAX) returned 0 with one: a caller that writes what it asked for runs
// off the end. The sizes: the rounding wraps (SIZE_MAX), rounds to 0
// (SIZE_MAX - 4094), and the page count is 2^32 (truncates to 0), 2^32 + 1
// (to 1) and 2^50 (2^62 bytes, to 0). Called through volatile pointers so the
// compiler cannot reason about the result. calloc runs last: given a short
// block it zeroes the whole size and faults.
void testHugeSizes() {
    static volatile size_t kHuge[] = {
        SIZE_MAX, SIZE_MAX - 4094, (size_t{1} << 32) * 4096, ((size_t{1} << 32) + 1) * 4096, size_t{1} << 62,
    };
    void* (*volatile p_malloc)(size_t) = std::malloc;
    void* (*volatile p_realloc)(void*, size_t) = std::realloc;
    void* (*volatile p_calloc)(size_t, size_t) = std::calloc;
    void* (*volatile p_aligned_alloc)(size_t, size_t) = aligned_alloc;
    int (*volatile p_posix_memalign)(void**, size_t, size_t) = posix_memalign;
    void* (*volatile p_new)(size_t) = static_cast<void* (*)(size_t)>(::operator new);
    void* (*volatile p_new_arr)(size_t) = static_cast<void* (*)(size_t)>(::operator new[]);
#if !defined(__APPLE__)
    void* (*volatile p_memalign)(size_t, size_t) = memalign;
    void* (*volatile p_pvalloc)(size_t) = pvalloc;
#endif
    constexpr size_t kAlign = 2 * 1024 * 1024;
    for (size_t n : kHuge) {
        errno = 0;
        void* p = p_malloc(n);
        CHECK(!p && errno == ENOMEM, "malloc(%zu) = %p (usable %zu) errno=%d, want nullptr with ENOMEM",
              n, p, p ? USABLE_SIZE(p) : 0, errno);
        if (p) std::free(p);

        // A smash-owned block in both modes, so realloc is smash's own; on
        // failure it must be left as it was.
        auto* old = static_cast<unsigned char*>(std::malloc(kAlign));
        if (old) {
            fillPattern(old, 4096);
            errno = 0;
            void* q = p_realloc(old, n);
            CHECK(!q && errno == ENOMEM, "realloc(2 MiB block, %zu) = %p errno=%d, want nullptr with ENOMEM",
                  n, q, errno);
            if (q) {
                std::free(q);
            } else {
                CHECK(verifyPattern(old, 4096), "realloc(2 MiB block, %zu) failed and changed the old block", n);
                std::free(old);
            }
        }

        void* a = nullptr;
        const int rc = p_posix_memalign(&a, kAlign, n);
        CHECK(rc == ENOMEM && !a, "posix_memalign(2 MiB, %zu) = %d with %p, want ENOMEM", n, rc, a);
        if (rc == 0) std::free(a);
        errno = 0;
        void* b = p_aligned_alloc(kAlign, n);
        CHECK(!b && errno == ENOMEM, "aligned_alloc(2 MiB, %zu) = %p errno=%d, want nullptr with ENOMEM",
              n, b, errno);
        if (b) std::free(b);
#if !defined(__APPLE__)
        errno = 0;
        void* m = p_memalign(4096, n);
        CHECK(!m && errno == ENOMEM, "memalign(4096, %zu) = %p errno=%d, want nullptr with ENOMEM", n, m, errno);
        if (m) std::free(m);
        // alloc8's pvalloc rounded up to a page before smash saw the size, so
        // SIZE_MAX and SIZE_MAX - 4094 wrapped to 0 and got a block.
        errno = 0;
        void* pv = p_pvalloc(n);
        CHECK(!pv && errno == ENOMEM, "pvalloc(%zu) = %p (usable %zu) errno=%d, want nullptr with ENOMEM",
              n, pv, pv ? USABLE_SIZE(pv) : 0, errno);
        if (pv) std::free(pv);
#endif
        for (auto [fn, what] : {std::pair{p_new, "operator new"}, std::pair{p_new_arr, "operator new[]"}}) {
            void* o = nullptr;
            bool threw = false;
            try {
                o = fn(n);
            } catch (const std::bad_alloc&) {
                threw = true;
            }
            CHECK(threw, "%s(%zu) returned %p instead of throwing std::bad_alloc", what, n, o);
            if (o) ::operator delete(o);
        }
        void* nt = ::operator new(n, std::nothrow);
        CHECK(!nt, "operator new(%zu, nothrow) returned %p, want nullptr", n, nt);
        if (nt) ::operator delete(nt);
    }
    for (size_t n : kHuge) {
        errno = 0;
        void* c = p_calloc(1, n);
        CHECK(!c && errno == ENOMEM, "calloc(1, %zu) = %p errno=%d, want nullptr with ENOMEM", n, c, errno);
        if (c) std::free(c);
    }
    // A count * size that wraps: alloc8's calloc returned nullptr and left
    // errno as it was.
    static volatile size_t kWrap[][2] = {
        {SIZE_MAX / 2 + 1, 2}, {size_t{1} << 32, size_t{1} << 32}, {SIZE_MAX, SIZE_MAX}, {3, SIZE_MAX / 2},
    };
    for (auto& w : kWrap) {
        errno = 0;
        void* c = p_calloc(w[0], w[1]);
        CHECK(!c && errno == ENOMEM, "calloc(%zu, %zu) = %p errno=%d, want nullptr with ENOMEM",
              w[0], w[1], c, errno);
        if (c) std::free(c);
    }
    printf("huge sizes: %zu sizes through malloc, realloc, posix_memalign, aligned_alloc, memalign, "
           "pvalloc, operator new, calloc; %zu wrapping calloc products\n",
           sizeof(kHuge) / sizeof(kHuge[0]), sizeof(kWrap) / sizeof(kWrap[0]));
}

int main() {
    // Unbuffered stdout: full mode used to zero the tail of glibc's buffered
    // stdout page at exit (fixed; guarded by test_stdout_buffer_integrity_full),
    // silently eating a test's own verdict lines. Kept as hygiene.
    setvbuf(stdout, nullptr, _IONBF, 0);
    printf("alloc_contract_test: mode=%s\n", modeName());
    printInterposeBanner();

    std::vector<size_t> sizes = buildSizes();
    printf("sweeping %zu distinct sizes\n", sizes.size());

    testMalloc(sizes);
    testOperatorNew(sizes);
    testAlignedNew();
    testPosixAligned();
    testCalloc(sizes);
    testReallocChain();
    testChurn();
    testNewHandler();
    testHugeSizes();

    if (failures) {
        printf("alloc_contract_test (%s): %d FAILURES\n", modeName(), failures);
        return 1;
    }
    printf("alloc_contract_test (%s): all checks passed\n", modeName());
    return 0;
}
