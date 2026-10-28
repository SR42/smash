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
// VM_EXHAUST_NUMA=1 (test_vm_exhaustion_numa, Linux) does the same through
// malloc_numa(0, ...): a node arena that cannot serve a block falls back to a
// separate mapping bound to the node, never compressed, and must say so once,
// as the plain path does. It used to fall back silently. VM_EXHAUST_NUMA=aligned
// (test_vm_exhaustion_numa_aligned) does it through malloc_numa_aligned, whose
// fallback trims an over-sized mapping to the alignment.
//
// Only the first and last page of each chunk is touched, so the whole run
// costs a few MiB of RSS despite reserving well over a gigabyte.

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <new>
#include <string>
#include <unistd.h>
#include <vector>

#if defined(__linux__)
#include <sched.h>
#include "numa_test_util.h"
#endif

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

// Run `fn` with fd 2 pointed at a temporary file; return what it wrote there,
// after re-emitting it on the real stderr so a FAIL line is never swallowed.
template <typename Fn>
std::string captureStderr(Fn&& fn) {
    char path[] = "/tmp/smash-vm-exhaustion-warn-XXXXXX";
    const int fd = mkstemp(path);
    const int saved = fd >= 0 ? dup(2) : -1;
    if (fd >= 0) unlink(path);
    if (saved >= 0) {
        fflush(stderr);
        dup2(fd, 2);
    }
    fn();
    std::string out;
    if (saved >= 0) {
        fflush(stderr);
        dup2(saved, 2);
        close(saved);
        lseek(fd, 0, SEEK_SET);
        char rb[4096];
        ssize_t n;
        while ((n = read(fd, rb, sizeof(rb))) > 0) out.append(rb, static_cast<size_t>(n));
        if (!out.empty()) fwrite(out.data(), 1, out.size(), stderr);
    }
    if (fd >= 0) close(fd);
    return out;
}

size_t countOf(const std::string& s, const char* needle) {
    size_t n = 0;
    for (size_t at = s.find(needle); at != std::string::npos; at = s.find(needle, at + 1)) ++n;
    return n;
}

#if defined(__linux__)
// The process's address space (VmSize) in bytes, 0 if unreadable.
size_t addressSpaceBytes() {
    FILE* f = fopen("/proc/self/status", "r");
    if (!f) return 0;
    char line[256];
    size_t kb = 0;
    while (fgets(line, sizeof(line), f))
        if (sscanf(line, "VmSize: %zu kB", &kb) == 1) break;
    fclose(f);
    return kb * 1024;
}

// VM_EXHAUST_NUMA=1: overflow the arena through malloc_numa on node 0;
// VM_EXHAUST_NUMA=aligned: through malloc_numa_aligned(0, 4 MiB, 8 MiB), whose
// fallback maps the block plus the alignment, trims both ends and binds what
// is left. The blocks are written from CPUs off node 0 (where there are any),
// so only a binding can put their pages there. Every block, in the arena or
// not, must be bound to node 0, have its written pages there and, aligned,
// sit on an 8 MiB boundary: a plain 4 MiB mmap is often 2 MiB-aligned by the
// kernel, never 8 MiB-aligned on purpose. A block outside the arena must
// cost the address space exactly its own size (an untrimmed fallback keeps
// 8 MiB more each).
int runNumaMode(bool aligned) {
    using MallocNuma = void* (*)(uint32_t, size_t);
    using MallocNumaAligned = void* (*)(uint32_t, size_t, size_t);
    using InArena = int (*)(const void*);
    constexpr size_t kAlign = 8ull * 1024 * 1024;
    auto malloc_numa = reinterpret_cast<MallocNuma>(dlsym(RTLD_DEFAULT, "malloc_numa"));
    auto malloc_numa_aligned =
        reinterpret_cast<MallocNumaAligned>(dlsym(RTLD_DEFAULT, "malloc_numa_aligned"));
    auto in_arena = reinterpret_cast<InArena>(dlsym(RTLD_DEFAULT, "smash_in_arena"));
    if (!malloc_numa || !malloc_numa_aligned || !in_arena) {
        printf("SKIP: malloc_numa / malloc_numa_aligned / smash_in_arena not exported (libsmash not "
               "interposed)\n");
        return 0;
    }
    const char* fn = aligned ? "malloc_numa_aligned" : "malloc_numa";
    cpu_set_t allowed;
    CPU_ZERO(&allowed);
    const bool have_affinity = sched_getaffinity(0, sizeof(allowed), &allowed) == 0;
    const bool pinned = have_affinity && numa_test::pinOutside(0, allowed);
    std::vector<void*> chunks;
    chunks.reserve(kNumChunks);
    std::vector<long> fallback_cost;   // address space each block outside the arena added
    fallback_cost.reserve(kNumChunks);
    size_t outside = 0;
    const std::string err = captureStderr([&] {
        for (size_t i = 0; i < kNumChunks; ++i) {
            const size_t vm_before = addressSpaceBytes();
            void* p = aligned ? malloc_numa_aligned(0, kChunkBytes, kAlign) : malloc_numa(0, kChunkBytes);
            const size_t vm_after = addressSpaceBytes();
            if (!p) {
                fprintf(stderr, "FAIL: %s(0, %zu) returned nullptr on chunk %zu (errno %d)\n",
                        fn, kChunkBytes, i, errno);
                ++failures;
                break;
            }
            if (!in_arena(p)) {
                ++outside;
                fallback_cost.push_back(static_cast<long>(vm_after) - static_cast<long>(vm_before));
            }
            stamp(p, kChunkBytes, i);
            chunks.push_back(p);
        }
    });
    if (pinned) sched_setaffinity(0, sizeof(allowed), &allowed);
    printf("numa: %s allocated %zu / %zu chunks on node 0, %zu outside the arena, writer %s\n", fn,
           chunks.size(), kNumChunks, outside, pinned ? "pinned off node 0" : "on node 0 (no other CPUs)");
    CHECK(chunks.size() == kNumChunks, "only %zu of %zu %s chunks allocated", chunks.size(), kNumChunks, fn);
    // Coverage: without a block outside the arena the fallback never ran.
    CHECK(outside >= 1,
          "every %s chunk is in the arena, so the node arena never ran out and the fallback was not "
          "taken; check that SMASH_VM_GIB=%s took effect", fn, vmGib());
    const size_t warnings = countOf(err, "did not fit the compressible arena");
    CHECK(warnings == 1 && err.find("malloc_numa") != std::string::npos && err.find("node 0") != std::string::npos,
          "%zu %s chunks fell out of the arena and smash printed %zu 'did not fit the compressible "
          "arena' warnings naming malloc_numa and node 0, want exactly 1 (none: the fallback is silent, and "
          "that memory is never compressed; more: the warning is not one-shot)", outside, fn, warnings);

    size_t misaligned = 0, unbound = 0, unbound_outside = 0;
    int first_mode = 0;
    std::vector<void*> written;
    for (void* p : chunks) {
        if (aligned && reinterpret_cast<uintptr_t>(p) % kAlign != 0) ++misaligned;
        numa_test::Policy pol;
        if (!numa_test::isBoundTo(p, 0, &pol)) {
            if (unbound++ == 0) first_mode = pol.mode;
            if (!in_arena(p)) ++unbound_outside;
        }
        written.push_back(p);
        written.push_back(static_cast<char*>(p) + kChunkBytes - g_page);
    }
    int first_status = 0;
    const size_t off_node = numa_test::pointersOffNode(written, 0, &first_status);
    CHECK(misaligned == 0, "%zu of %zu malloc_numa_aligned chunks are not 8 MiB-aligned", misaligned,
          chunks.size());
    CHECK(unbound == 0,
          "%zu of %zu %s chunks (%zu of them outside the arena) are not bound MPOL_BIND|MPOL_F_STATIC_NODES "
          "to {0} (first: mode %#x)", unbound, chunks.size(), fn, unbound_outside, first_mode);
    CHECK(off_node == 0, "%zu of %zu written pages of %s chunks are not on node 0 (first status %d)",
          off_node, written.size(), fn, first_status);

    // Per call, and the median: a thread smash starts meanwhile adds its stack
    // and, in large-only mode, a 64 MiB glibc arena to whichever call it lands
    // in (measured 15-79 MiB over the whole loop).
    if (!fallback_cost.empty()) {
        std::sort(fallback_cost.begin(), fallback_cost.end());
        const long median = fallback_cost[fallback_cost.size() / 2];
        printf("numa: each chunk outside the arena added %ld KiB of address space (median; %ld..%ld)\n",
               median / 1024, fallback_cost.front() / 1024, fallback_cost.back() / 1024);
        CHECK(median == static_cast<long>(kChunkBytes),
              "a %s chunk outside the arena added %ld KiB of address space (median), want %zu: the "
              "fallback mapping keeps more than its block", fn, median / 1024, kChunkBytes / 1024);
    }

    for (size_t i = 0; i < chunks.size(); ++i)
        if (!checkStamp(chunks[i], kChunkBytes, i, i, "numa")) break;
    for (void* p : chunks) std::free(p);
    if (failures) {
        printf("vm_exhaustion_test (numa): %d FAILURES\n", failures);
        return 1;
    }
    printf("vm_exhaustion_test (numa): all checks passed\n");
    return 0;
}
#endif

} // namespace

int main() {
    // Unbuffered stdout: full mode used to zero the tail of glibc's buffered
    // stdout page at exit (fixed; guarded by test_stdout_buffer_integrity_full),
    // silently eating a test's own verdict lines. Kept as hygiene.
    setvbuf(stdout, nullptr, _IONBF, 0);
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
    if (const char* e = std::getenv("VM_EXHAUST_NUMA")) {
#if defined(__linux__)
        if (e[0] == '1') return runNumaMode(false);
        if (strcmp(e, "aligned") == 0) return runNumaMode(true);
#else
        printf("SKIP: VM_EXHAUST_NUMA=%s needs Linux (without NUMA, malloc_numa is plain malloc)\n", e);
        return 0;
#endif
    }

    std::vector<void*> chunks;
    chunks.reserve(kNumChunks);

    // Capture stderr across phase 1. A large allocation that misses the arena
    // comes from a direct mmap and is NEVER compressed, however cold it gets,
    // and nothing else in the process reports that — for a workload of
    // multi-MB or GB blocks it is the difference between smash working and
    // smash doing nothing, so the warning is part of the contract. The
    // redirect is scoped to phase 1 and everything captured is re-emitted, so
    // a FAIL message can never be swallowed.
    char warn_path[] = "/tmp/smash-vm-exhaustion-warn-XXXXXX";
    int warn_fd = mkstemp(warn_path);
    int saved_stderr = -1;
    if (warn_fd >= 0) {
        unlink(warn_path);
        fflush(stderr);
        saved_stderr = dup(2);
        if (saved_stderr >= 0) dup2(warn_fd, 2);
    }

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

    std::string phase1_stderr;
    if (saved_stderr >= 0) {
        fflush(stderr);
        dup2(saved_stderr, 2);
        close(saved_stderr);
        lseek(warn_fd, 0, SEEK_SET);
        char rb[4096];
        ssize_t n;
        while ((n = read(warn_fd, rb, sizeof(rb))) > 0)
            phase1_stderr.append(rb, static_cast<size_t>(n));
        close(warn_fd);
        if (!phase1_stderr.empty())
            fwrite(phase1_stderr.data(), 1, phase1_stderr.size(), stderr);
    }
    CHECK(phase1_stderr.find("did not fit the compressible arena") != std::string::npos,
          "the arena filled and LargeAlloc fell back to a direct mapping, but "
          "smash said nothing — that memory is silently never compressed");

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
