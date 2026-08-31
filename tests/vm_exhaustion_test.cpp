// Exhausting the VmRegion must fall back, not fail.
//
// Large allocations (>= kLargeAllocVmThreshold = 1 MiB) are placed in the
// contiguous VmRegion arena so the compressor can see them. That arena is a
// fixed reservation — SMASH_VM_GIB, 16 GiB by default. Once it is full,
// VmRegion::allocatePages() returns nullptr and LargeAlloc is supposed to
// fall back to a direct mmap:
//
//     if (vm_region_ && ... ) mem = vm_region_->allocatePages(num_pages);
//     if (!mem) { ... mem = vm::mapPages(alloc_size); }
//     if (!mem) return nullptr;
//
// If that fallback does not engage, malloc returns nullptr and the
// operator-new wrappers throw std::bad_alloc — the failure mode config.h
// calls out by name:
//
//     "so workloads with large working sets ... don't exhaust the region and
//      trigger malloc=NULL -> std::bad_alloc / LLVM 'out of memory' errors"
//
// This is a first-class suspect for the aarch64 reports for two reasons.
// First, on a kernel whose pages are larger than smash's compile-time
// kPageSize the arena's usable capacity is a fraction of what smash accounts
// for, so it fills far sooner than the configured size suggests. Second, the
// fallback and arena paths must be distinguished correctly on free(): mixing
// them up is precisely what issue #84 did, poisoning the arena free list with
// out-of-range page indices.
//
// The test drives the arena into overflow with SMASH_VM_GIB=1, then frees
// everything and reallocates — so it covers the fallback path, the mixed
// free routing, and reuse of the recycled arena afterwards.
//
// Only the first and last page of each chunk is touched, so the whole run
// costs a few MiB of RSS despite reserving well over a gigabyte.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <new>
#include <unistd.h>
#include <vector>

namespace {

int failures = 0;

#define CHECK(cond, msg, ...) do { \
    if (!(cond)) { \
        fprintf(stderr, "FAIL: " msg "\n", ##__VA_ARGS__); \
        ++failures; \
    } \
} while (0)

// SMASH_VM_GIB=1 gives a 1 GiB arena (getVmRegionSize() floors at 256 MiB and
// the env value is in whole GiB, so 1 GiB is the smallest reachable arena).
// 1.5 GiB of live 4 MiB chunks overflows it by half.
constexpr size_t kChunkBytes = 4ull * 1024 * 1024;
constexpr size_t kNumChunks = 384;               // 1.5 GiB
constexpr size_t kReallocChunks = 96;            // 384 MiB after the free

// Must match the SMASH_VM_GIB the ctest registration sets.
constexpr size_t kArenaGiB = 1;

size_t g_page = 4096;

// Stamp a sentinel into the first and last page of a chunk. Keeping the
// touched set to two pages is what keeps RSS low; it still forces the kernel
// to genuinely back both ends of every mapping, so a chunk handed back
// uncommitted (PROT_NONE) faults here.
void stamp(void* p, size_t bytes, size_t seed) {
    auto* b = static_cast<unsigned char*>(p);
    std::memset(b, static_cast<int>(seed & 0xff), g_page);
    std::memset(b + bytes - g_page, static_cast<int>((seed >> 8) & 0xff), g_page);
}

bool checkStamp(const void* p, size_t bytes, size_t seed, size_t idx,
                const char* phase) {
    auto* b = static_cast<const unsigned char*>(p);
    unsigned char want_head = static_cast<unsigned char>(seed & 0xff);
    unsigned char want_tail = static_cast<unsigned char>((seed >> 8) & 0xff);
    for (size_t i = 0; i < g_page; ++i) {
        if (b[i] != want_head) {
            fprintf(stderr,
                    "FAIL: %s chunk %zu head byte %zu = 0x%02x, want 0x%02x\n",
                    phase, idx, i, b[i], want_head);
            ++failures;
            return false;
        }
    }
    for (size_t i = 0; i < g_page; ++i) {
        if (b[bytes - g_page + i] != want_tail) {
            fprintf(stderr,
                    "FAIL: %s chunk %zu tail byte %zu = 0x%02x, want 0x%02x\n",
                    phase, idx, i, b[bytes - g_page + i], want_tail);
            ++failures;
            return false;
        }
    }
    return true;
}

// A host configured with strict overcommit (vm.overcommit_memory=2) can
// legitimately refuse 1.5 GiB of anonymous reservation, which would make this
// test report a failure that is not smash's.
bool strictOvercommit() {
#if defined(__linux__)
    FILE* f = fopen("/proc/sys/vm/overcommit_memory", "r");
    if (!f) return false;
    int mode = 0;
    int got = fscanf(f, "%d", &mode);
    fclose(f);
    return got == 1 && mode == 2;
#else
    return false;
#endif
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

const char* vmGib() {
    const char* e = std::getenv("SMASH_VM_GIB");
    return e ? e : "(default)";
}

} // namespace

int main() {
    long ps = sysconf(_SC_PAGESIZE);
    if (ps > 0) g_page = static_cast<size_t>(ps);

    printf("vm_exhaustion_test: SMASH_VM_GIB=%s chunk=%zu MiB count=%zu "
           "(%.2f GiB) page=%zu\n",
           vmGib(), kChunkBytes / (1024 * 1024), kNumChunks,
           (kChunkBytes * kNumChunks) / double(1ull << 30), g_page);
    printInterposeBanner();

    if (strictOvercommit()) {
        printf("SKIP: vm.overcommit_memory=2 (strict) — a 1.5 GiB reservation "
               "can fail for reasons unrelated to smash\n");
        return 0;
    }

    std::vector<void*> chunks;
    chunks.reserve(kNumChunks);

    // ── Phase 1: overflow the arena ─────────────────────────────────────────
    for (size_t i = 0; i < kNumChunks; ++i) {
        void* p = nullptr;
        try {
            p = ::operator new(kChunkBytes);
        } catch (const std::bad_alloc&) {
            fprintf(stderr,
                    "FAIL: std::bad_alloc on chunk %zu of %zu (%zu MiB "
                    "allocated so far). The VmRegion arena filled and "
                    "LargeAlloc's direct-mmap fallback did not engage — "
                    "malloc returned nullptr and operator new threw.\n",
                    i, kNumChunks, (i * kChunkBytes) / (1024 * 1024));
            ++failures;
            break;
        }
        if (!p) {
            fprintf(stderr, "FAIL: operator new(%zu) returned nullptr on "
                    "chunk %zu\n", kChunkBytes, i);
            ++failures;
            break;
        }
        stamp(p, kChunkBytes, i);
        chunks.push_back(p);
    }

    printf("phase 1: allocated %zu / %zu chunks (%.2f GiB)\n",
           chunks.size(), kNumChunks,
           (chunks.size() * kChunkBytes) / double(1ull << 30));
    CHECK(chunks.size() == kNumChunks,
          "only %zu of %zu chunks allocated", chunks.size(), kNumChunks);

    // Prove the fallback actually engaged rather than the arena silently
    // being larger than SMASH_VM_GIB asked for. Phase 1 is pure bump
    // allocation, so every chunk that came from the contiguous arena is
    // adjacent to its neighbour; a chunk from LargeAlloc's direct-mmap
    // fallback lands in a different part of the address space entirely. Sort
    // the chunk addresses and count the discontinuities: all-arena gives
    // zero, an exhausted arena gives at least one.
    // (Without this the test would still pass on a build that ignored
    // SMASH_VM_GIB, and would then be covering nothing.)
    if (chunks.size() > 1) {
        std::vector<uintptr_t> addrs;
        addrs.reserve(chunks.size());
        for (void* p : chunks) addrs.push_back(reinterpret_cast<uintptr_t>(p));
        std::sort(addrs.begin(), addrs.end());
        size_t breaks = 0;
        for (size_t i = 1; i < addrs.size(); ++i)
            if (addrs[i] != addrs[i - 1] + kChunkBytes) ++breaks;
        printf("phase 1: %zu address discontinuities across %zu chunks "
               "(arena is %zu GiB)\n", breaks, chunks.size(), kArenaGiB);
        CHECK(breaks >= 1,
              "all %zu chunks are contiguous, so every one of them came from "
              "the VmRegion arena and LargeAlloc's direct-mmap fallback was "
              "never taken — this run covered nothing. Check that "
              "SMASH_VM_GIB=%s took effect and is smaller than the %.2f GiB "
              "this test allocates",
              chunks.size(), vmGib(),
              (kChunkBytes * kNumChunks) / double(1ull << 30));
    }

    // ── Phase 2: every chunk still holds its data ───────────────────────────
    // Chunks past the arena boundary came from the fallback mmap; if the two
    // populations were confused, one of them reads back wrong here.
    for (size_t i = 0; i < chunks.size(); ++i) {
        if (!checkStamp(chunks[i], kChunkBytes, i, i, "post-alloc")) break;
    }

    // ── Phase 3: free everything, arena and fallback alike ──────────────────
    // LargeAlloc::deallocate routes on inContigArena(): arena pages go back to
    // the VmRegion free list, fallback pages are munmapped. Getting that wrong
    // is issue #84 — an external page index recycled as an arena offset.
    for (void* p : chunks) ::operator delete(p);
    chunks.clear();
    printf("phase 3: freed all chunks\n");

    // ── Phase 4: the arena is usable again ──────────────────────────────────
    // If the free list was poisoned, allocatePages() now mints addresses past
    // the end of the arena and this phase faults or reads back garbage.
    for (size_t i = 0; i < kReallocChunks; ++i) {
        void* p = nullptr;
        try {
            p = ::operator new(kChunkBytes);
        } catch (const std::bad_alloc&) {
            fprintf(stderr,
                    "FAIL: std::bad_alloc on realloc chunk %zu — memory freed "
                    "in phase 3 was not returned to the allocator\n", i);
            ++failures;
            break;
        }
        if (!p) {
            fprintf(stderr, "FAIL: operator new(%zu) returned nullptr on "
                    "realloc chunk %zu\n", kChunkBytes, i);
            ++failures;
            break;
        }
        stamp(p, kChunkBytes, i + 1000);
        chunks.push_back(p);
    }
    printf("phase 4: reallocated %zu / %zu chunks\n",
           chunks.size(), kReallocChunks);
    CHECK(chunks.size() == kReallocChunks,
          "only %zu of %zu chunks reallocated after free",
          chunks.size(), kReallocChunks);

    for (size_t i = 0; i < chunks.size(); ++i) {
        if (!checkStamp(chunks[i], kChunkBytes, i + 1000, i, "post-reuse"))
            break;
    }

    // ── Phase 5: small allocations still work with the arena full ───────────
    // Slab spans come from the same VmRegion. An exhausted arena must not
    // starve the small-object path.
    std::vector<void*> smalls;
    smalls.reserve(4096);
    for (size_t i = 0; i < 4096; ++i) {
        void* p = std::malloc(256);
        if (!p) {
            fprintf(stderr,
                    "FAIL: malloc(256) returned nullptr at iteration %zu with "
                    "the VmRegion arena full — the small-object path was "
                    "starved by large-allocation pressure\n", i);
            ++failures;
            break;
        }
        std::memset(p, 0x5A, 256);
        smalls.push_back(p);
    }
    printf("phase 5: allocated %zu small blocks with the arena full\n",
           smalls.size());
    for (void* p : smalls) std::free(p);

    for (void* p : chunks) ::operator delete(p);

    if (failures) {
        printf("vm_exhaustion_test: %d FAILURES\n", failures);
        return 1;
    }
    printf("vm_exhaustion_test: all checks passed\n");
    return 0;
}
