#pragma once
#include <cmath>
#include <chrono>
#include "smash/config.h"
#include "core/bootstrap_alloc.h"
#include "core/size_classes.h"
#include "core/span.h"
#include "core/page_map.h"
#include "core/slab.h"
#include "core/large_alloc.h"
#include "core/thread_cache.h"
#include "vm/vm_region.h"
#include "vm/page_state.h"
#include "vm/fault_handler.h"
#include "compress/compress_store.h"
#include "compress/compress_engine.h"
#include "compress/compressor_thread.h"
#include "util/bitops.h"
#include "util/safe_printf.h"  // allocation-free snprintf for malloc-path diagnostics
#include <alloc8/alloc8.h>     // alloc8_caller_ra: app allocation site captured at wrapper entry
#include <cstddef>
#include <cstdint>
#include <atomic>
#include <dlfcn.h>
#if defined(__linux__)
#include <sched.h>   // sched_getcpu (vDSO) for CPU-indexed arena routing
#include <link.h>    // dl_iterate_phdr: image table for the call-site hash
#endif
#if defined(__APPLE__)
#include <mach-o/dyld.h>
#include <mach-o/getsect.h>
#include <pthread.h>  // pthread_cpu_number_np for CPU-indexed arena routing
#endif

namespace smash {
// Current CPU id, or -1 if unavailable. Used to index arena routing by core
// rather than by thread (see callsiteArena / cpuArenaHash). Both fast paths
// avoid a syscall:
//   Linux : sched_getcpu()        — vDSO, ~3ns.
//   macOS : pthread_cpu_number_np — reads the thread-local commpage, ~2ns
//           (macOS 11.0+; same primitive Hoard uses in threadpoolheap.h).
[[gnu::always_inline]]
inline int currentCpu() {
#if defined(__linux__)
    return sched_getcpu();
#elif defined(__APPLE__)
    size_t cpu = 0;
    if (pthread_cpu_number_np(&cpu) == 0) return static_cast<int>(cpu);
    return -1;
#else
    return -1;
#endif
}
}  // namespace smash

namespace smash {

// Counts threadInit() calls. The first call is the main thread during early
// init (before _objc_init completes). We defer compression start until the
// second threadInit() call, which only happens after full init is complete.
extern std::atomic<int> g_thread_init_count;

// Global pointer to the VmRegion, used by syscall wrappers to warm pages.
// Set after SmashHeap construction when compression is enabled.
extern VmRegion* g_smash_vm_region;

// Global pointer to the PageStateTable, used by the mmap / Mach VM
// interposers to mark newly-tracked external pages as ACTIVE (or EMPTY
// on unmap). Set when compression is initialized; nullptr otherwise.
// Same lifetime contract as g_smash_vm_region.
extern PageStateTable* g_smash_page_states_for_external;

// Global pointer to the PageLockTable, used by the munmap / Mach VM
// DEALLOCATE interposers to serialize external-page untracking against a
// compressor worker mid-snapshot. Without this lock the worker's
// memcpy(page_buf, page_addr, kPageSize) in compressPage can race a concurrent
// munmap that tears down the mapping → SIGSEGV (full-mode external-page race;
// see model/SmashExternalRace.{lean,tla}). Same lifetime as the above.
extern PageLockTable* g_smash_page_locks_for_external;

// Profile-driven skip flag for external page tracking. Set by the
// compressor when loading a profile that marks external pages as hot.
// When true, mmap interposers skip the per-page tracking loop entirely,
// avoiding the O(pages) overhead for each TBB arena allocation.
// (Defined as an inline variable in compress/compressor_thread.h, included
// above — no separate declaration needed here.)

// TLS variables on the malloc fast path are declared extern at namespace
// scope with tls_model("initial-exec"), and defined in smash_heap.cpp.
// libsmash is always loaded via LD_PRELOAD (so its TLS block is part of
// the startup-time TLS reservation), which makes initial-exec safe.
// Without this, the compiler emits __tls_get_addr indirections for
// every static-local thread_local in an inline function, costing one
// dependent indirect call per TLS access on the hot path.
extern __attribute__((tls_model("initial-exec")))
    thread_local ThreadCache* g_thread_cache;

extern __attribute__((tls_model("initial-exec")))
    thread_local int g_full_mode_cached;  // -1 = unknown, 0 = bypass, 1 = full

// Compressor thread bypass flag — defined as inline thread_local in
// compressor_thread.h (included above). Used here on the malloc fast path.

// free()-path last-span cache. The dominant cost of free() at scale is the
// single load from the 128 MB flat page→Span table (a near-random access that
// misses cache once the live set is large). Consecutive frees overwhelmingly
// hit the same span (objects from one slab page are freed together), so a
// one-entry per-thread cache of the most recently resolved span elides that
// load. Validated against g_pagemap_generation (see page_map.h): a span whose
// stamped generation still matches the global one provably still owns its
// range. initial-exec TLS keeps the access to a direct tpidr_el0 + offset load.
//
// The span's [base, end) range is cached INLINE (not read through the Span*)
// so the hit test is a pure TLS integer comparison. This matters on the miss
// path: a workload with no free-locality must not pay a dereference of the
// stale cached span (its own cache miss) before falling through to the table
// load. On a miss we touch only TLS scalars, then resolve via the table once.
extern __attribute__((tls_model("initial-exec")))
    thread_local Span* g_free_cache_span;
extern __attribute__((tls_model("initial-exec")))
    thread_local uintptr_t g_free_cache_base;
extern __attribute__((tls_model("initial-exec")))
    thread_local uintptr_t g_free_cache_end;
extern __attribute__((tls_model("initial-exec")))
    thread_local uint64_t g_free_cache_gen;
// Populate-throttle countdown. On a miss the cache is repopulated only when
// this hits 0, then reset to (interval-1); otherwise it counts down and the
// stale entry is left in place. Hits reset it to 0 so the next miss repopulates
// immediately (fast phase-change adoption). With interval==1 every miss
// repopulates (max locality benefit); larger intervals amortize the populate
// stores away on locality-free workloads where they are pure overhead.
extern __attribute__((tls_model("initial-exec")))
    thread_local uint32_t g_free_cache_skip;

// Stack-base anchor for callsiteArena()'s LLAMA-style depth bucket. Caches the
// thread's TRUE stack base (highest address), computed once. depth_bucket is the
// OFFSET of the current frame below this base, so it is ASLR-invariant and
// history-free. Runs on every malloc; file-scope initial-exec TLS keeps it a
// direct tpidr_el0 + offset load, not the __tls_get_addr call a function-local
// thread_local in an inline function would emit. 0 = not yet computed.
//
// This replaced a drifting reference (the shallowest frame seen so far, reset
// whenever a shallower allocation occurred): that made depth depend on
// allocation history and vary run-to-run — measured as bimodal arena
// assignments / RSS on rocksdb. Anchoring to the real stack base removes the
// drift while preserving the depth signal.
extern __attribute__((tls_model("initial-exec")))
    thread_local uintptr_t g_arena_stack_base;

// Per-thread memo of callsiteArena()'s result, keyed on (ra, size class); see
// callsiteArena(). Namespace scope (not a function-local static) so the
// inlined hit path and the out-of-line miss path share it.
struct ArenaMemo { uintptr_t key; uint8_t base; bool valid; };
inline constexpr size_t kArenaMemoSlots = 256;  // power of 2
extern __attribute__((tls_model("initial-exec")))
    thread_local ArenaMemo g_arena_memo[kArenaMemoSlots];

// Resolve the thread's true stack base once and cache it in g_arena_stack_base.
// Cold: only the first allocation per thread (plus any reentrant allocations the
// query itself makes) reaches here.
[[gnu::cold, gnu::noinline]]
inline uintptr_t computeThreadStackBase(uintptr_t frame) {
    // Reentrancy guard: on glibc, pthread_getattr_np() reads /proc/self/maps for
    // the main thread and calls malloc — which re-enters this via callsiteArena.
    // A reentrant call returns its own frame (depth 0 for that allocation) and
    // does NOT cache; the outer call resolves and caches the real base.
    static __attribute__((tls_model("initial-exec")))
        thread_local bool computing = false;
    if (computing) return frame;
    computing = true;
    uintptr_t base = frame;
#if defined(__APPLE__)
    // Returns the base (highest address); allocation-free.
    if (void* b = pthread_get_stackaddr_np(pthread_self()))
        base = reinterpret_cast<uintptr_t>(b);
#elif defined(__linux__)
    pthread_attr_t attr;
    if (pthread_getattr_np(pthread_self(), &attr) == 0) {
        void* addr = nullptr; size_t size = 0;
        if (pthread_attr_getstack(&attr, &addr, &size) == 0 && addr && size)
            base = reinterpret_cast<uintptr_t>(addr) + size;  // low addr + size = top
        pthread_attr_destroy(&attr);
    }
#endif
    if (base < frame) base = frame;   // sanity: base is the highest (shallowest) addr
    g_arena_stack_base = base;
    computing = false;
    return base;
}

[[gnu::always_inline]]
inline uintptr_t threadStackBase() {
    uintptr_t base = g_arena_stack_base;
    if (base == 0) [[unlikely]]
        base = computeThreadStackBase(
            reinterpret_cast<uintptr_t>(__builtin_frame_address(0)));
    return base;
}

// Repopulate interval for the free() last-span cache (see above). Read once
// from SMASH_FREE_CACHE_REPOP (default kFreeCacheRepopDefault); 1 = always
// repopulate. Exposed as a runtime knob so the policy can be swept without
// rebuilding; the resolved value is cached in a static.
//
// Default 8 chosen by a parameter sweep across the real suite (bench_sqlite,
// bench_rss) plus the locality and no-locality extremes of the churn microbench
// (bench_throughput). The real suite is flat for all intervals (the cache is
// invisible to locality-bearing workloads), the locality case holds 3.2 ns/op
// at every interval, and 8 recovers most of the no-locality populate overhead
// (random-replacement churn 18.9→18.3 ns/op vs 1) while still adopting a new
// hot span within 8 misses on a phase change. Higher intervals buy a sliver
// more on the synthetic no-locality case with no real-suite gain and slower
// phase adoption, so 8 is the Pareto knee.
inline constexpr uint32_t kFreeCacheRepopDefault = 8;
// Hand-rolled lazy init (not a magic static): the guard-variable acquire-load
// measured at 7-10 % of free()'s CPU elsewhere (see config.h getSmashMode()).
// 0 = uninitialised sentinel; valid intervals are >= 1.
// The getenv/parse init bodies below are OUTLINED into cold, noinline
// helpers. The hot readers are always_inline'd into malloc()/free(); with
// the init code written inline in the [[unlikely]] branch, GCC keeps it in
// the hot function body (bigger frame, extra spills across every call
// site) — measured +1.7 ns/op (9.7 → 11.4) on the direct-linked x86-64
// bench_throughput single-alloc/free loop. Outlining restores a minimal
// hot body: one relaxed load, one predicted branch, cold call on first use.
[[gnu::cold, gnu::noinline]]
inline uint32_t freeCacheRepopIntervalInit(std::atomic<uint32_t>& cached) {
    const char* v = std::getenv("SMASH_FREE_CACHE_REPOP");
    long n = v ? std::atol(v) : 0;
    uint32_t interval = (n >= 1 && n <= (1L << 20)) ? static_cast<uint32_t>(n)
                                                    : kFreeCacheRepopDefault;
    cached.store(interval, std::memory_order_relaxed);
    return interval;
}

[[gnu::always_inline]]
inline uint32_t freeCacheRepopInterval() {
    static std::atomic<uint32_t> cached{0};   // constant-init: no guard
    uint32_t interval = cached.load(std::memory_order_relaxed);
    if (interval == 0) [[unlikely]] interval = freeCacheRepopIntervalInit(cached);
    return interval;
}

[[gnu::always_inline]]
inline ThreadCache*& currentThreadCache() { return g_thread_cache; }

// CPU-indexed arena routing (SMASH_CPU_ARENA, default OFF). When set,
// callsiteArena() mixes the running CPU id (sched_getcpu on Linux,
// pthread_cpu_number_np on macOS — same primitive Hoard uses) into the arena
// hash instead of a per-thread monotonic id.
//
// DEFAULT OFF — measured a NET REGRESSION on Hoard Larson: ~22% slower at 32
// threads (125M vs 161M ops/s) and no win at 64t. Root cause: smash mixes the
// lane id into a hash that is then masked to getNumArenas() (64 here) shared
// arenas — so CPU-indexing does NOT give a thread a dedicated arena, it only
// reshuffles which hash collisions occur, and the high-entropy tid hash
// distributes into 64 buckets slightly better. Hoard's win comes from indexing
// a TRUE per-CPU cache array directly (caches[cpu], no shared lock); replicating
// that in smash needs per-CPU/per-core slab lanes, not a hash tweak. Kept as an
// opt-in knob for hosts where CPU-locality routing helps (e.g. arena count ≈
// CPU count). See the deferred-free work for the actual contention fix.
[[gnu::cold, gnu::noinline]]
inline int cpuArenaHashInit(std::atomic<int>& cached) {
    const char* e = std::getenv("SMASH_CPU_ARENA");
    int v = (e && e[0] == '1') ? 1 : 0;  // default OFF; opt in with SMASH_CPU_ARENA=1
    cached.store(v, std::memory_order_relaxed);
    return v;
}

[[gnu::always_inline]]
inline bool cpuArenaHash() {
    // Hand-rolled lazy init (not a magic static) — this runs on every malloc,
    // so the guard-variable acquire-load would sit on the hot path. Init is
    // outlined cold (see freeCacheRepopIntervalInit's comment).
    static std::atomic<int> cached{-1};   // constant-init: no guard
    int v = cached.load(std::memory_order_relaxed);
    if (v < 0) [[unlikely]] v = cpuArenaHashInit(cached);
    return v == 1;
}

// True iff a free()-path diagnostic env var is set (SMASH_COUNT_FREE counts
// every free; SMASH_TRACE_FOREIGN_FREE logs foreign frees). When either is
// active the inlined free() fast path is disabled so every free is routed
// through freeSlow() — keeping the counters/traces complete. One-shot, so the
// steady-state cost is a single predictable branch.
[[gnu::cold, gnu::noinline]]
inline int freeDiagnosticsActiveInit(std::atomic<int>& cached) {
    const char* c = std::getenv("SMASH_COUNT_FREE");
    const char* t = std::getenv("SMASH_TRACE_FOREIGN_FREE");
    int v = ((c && c[0] == '1') || (t && t[0] == '1')) ? 1 : 0;
    cached.store(v, std::memory_order_relaxed);
    return v;
}

[[gnu::always_inline]]
inline bool freeDiagnosticsActive() {
    // Hand-rolled lazy init (not a magic static) — this runs on every free,
    // so the guard-variable acquire-load would sit on the hot path. Init is
    // outlined cold (see freeCacheRepopIntervalInit's comment).
    static std::atomic<int> cached{-1};   // constant-init: no guard
    int v = cached.load(std::memory_order_relaxed);
    if (v < 0) [[unlikely]] v = freeDiagnosticsActiveInit(cached);
    return v == 1;
}

// SMASH_PASSTHROUGH=1: smash passes all malloc/free to system allocator.
// Used to isolate whether issues are caused by smash's allocation logic
// or just by having the library loaded at all.
[[gnu::always_inline]]
inline bool isPassthroughMode() {
    static const bool passthrough = [] {
        const char* v = std::getenv("SMASH_PASSTHROUGH");
        return v && v[0] == '1';
    }();
    return passthrough;
}

// One-shot cache of "full mode" (= not compress-only, not large-only, no
// eager-zero, not passthrough).  If any of those env-var modes is on, the
// malloc fast path has to bypass to the slow path; if none, the fast path
// is safe.
[[gnu::always_inline]]
inline bool fullMallocPath() {
    int s = g_full_mode_cached;
    if (s < 0) [[unlikely]] {
        s = (!isCompressOnlyMode()
          && !isLargeOnlyMode()
          && !isEagerZeroMode()
          && !isPassthroughMode()) ? 1 : 0;
        g_full_mode_cached = s;
    }
    return s == 1;
}

// ── System malloc/free pointers for compress-only mode ──────────────────────
// Resolved early before any malloc interposition is active.
using MallocFn = void*(*)(size_t);
using FreeFn = void(*)(void*);
using CallocFn = void*(*)(size_t, size_t);
using ReallocFn = void*(*)(void*, size_t);
using MemalignFn = int(*)(void**, size_t, size_t);
using MallocSizeFn = size_t(*)(const void*);

struct SystemAllocFns {
    MallocFn malloc = nullptr;
    FreeFn free = nullptr;
    CallocFn calloc = nullptr;
    ReallocFn realloc = nullptr;
    MemalignFn posix_memalign = nullptr;
    MallocSizeFn malloc_size = nullptr;

    // Find the pre-interposition original of a function by scanning the
    // __DATA,__interpose section.  Each entry is {replacement, original}.
    // Returns nullptr if not found.
#if defined(__APPLE__)
    static void* findOriginal(void* replacement) {
        // The interpose section may be in __DATA or __AUTH_CONST (ARM64e).
        static const char* segments[] = {"__AUTH_CONST", "__DATA"};
        uint32_t n = _dyld_image_count();
        for (uint32_t i = 0; i < n; i++) {
            auto* hdr = reinterpret_cast<const struct mach_header_64*>(
                _dyld_get_image_header(i));
            if (!hdr) continue;
            for (auto* seg : segments) {
                unsigned long sz = 0;
                auto* data = getsectiondata(hdr, seg, "__interpose", &sz);
                if (!data || sz == 0) continue;
                size_t count = sz / (2 * sizeof(void*));
                auto* entries = reinterpret_cast<void* const*>(data);
                for (size_t j = 0; j < count; j++) {
                    if (entries[j * 2] == replacement)
                        return entries[j * 2 + 1];
                }
            }
        }
        return nullptr;
    }
#endif

    void resolve() {
#if defined(__APPLE__)
        // On macOS, dlsym returns the interposed wrapper, not the real system
        // function.  Read the original from the __DATA,__interpose section.
        void* h = dlopen("/usr/lib/system/libsystem_malloc.dylib", RTLD_NOLOAD);
        if (h) {
            // Get the current (interposed) function addresses, then find
            // their pre-interposition originals.
            void* cur_malloc = dlsym(h, "malloc");
            void* cur_free = dlsym(h, "free");
            void* cur_calloc = dlsym(h, "calloc");
            void* cur_realloc = dlsym(h, "realloc");
            void* cur_memalign = dlsym(h, "posix_memalign");
            void* cur_msize = dlsym(h, "malloc_size");

            malloc = reinterpret_cast<MallocFn>(findOriginal(cur_malloc));
            free = reinterpret_cast<FreeFn>(findOriginal(cur_free));
            calloc = reinterpret_cast<CallocFn>(findOriginal(cur_calloc));
            realloc = reinterpret_cast<ReallocFn>(findOriginal(cur_realloc));
            posix_memalign = reinterpret_cast<MemalignFn>(findOriginal(cur_memalign));
            malloc_size = reinterpret_cast<MallocSizeFn>(findOriginal(cur_msize));
        }
#elif defined(__linux__)
        malloc = reinterpret_cast<MallocFn>(dlsym(RTLD_NEXT, "malloc"));
        free = reinterpret_cast<FreeFn>(dlsym(RTLD_NEXT, "free"));
        calloc = reinterpret_cast<CallocFn>(dlsym(RTLD_NEXT, "calloc"));
        realloc = reinterpret_cast<ReallocFn>(dlsym(RTLD_NEXT, "realloc"));
        posix_memalign = reinterpret_cast<MemalignFn>(dlsym(RTLD_NEXT, "posix_memalign"));
        malloc_size = reinterpret_cast<MallocSizeFn>(dlsym(RTLD_NEXT, "malloc_usable_size"));

        // SMASH_DEBUG_RESOLVE=1: verify dlsym returned glibc's malloc, not our
        // own. This catches the case where the dynamic linker still resolves
        // RTLD_NEXT to libsmash itself (which would cause infinite recursion or
        // silent passthrough no-ops).
        if (std::getenv("SMASH_DEBUG_RESOLVE")) {
            char buf[256];
            Dl_info info{};
            if (malloc && dladdr(reinterpret_cast<void*>(malloc), &info)) {
                int n = smash::safe_snprintf(buf, sizeof(buf),
                    "[smash debug] system malloc resolved to %s\n",
                    info.dli_fname ? info.dli_fname : "?");
                if (n > 0) (void)!::write(2, buf, (size_t)n);
            } else {
                (void)!::write(2, "[smash debug] system malloc NOT resolved\n", 41);
            }
        }
#endif
    }
};

extern SystemAllocFns g_system_alloc;

// The application's allocation call site, for arena routing. alloc8's
// app-facing wrapper entries (malloc/calloc/realloc/memalign/operator new/...)
// store __builtin_return_address(0) into alloc8_caller_ra before dispatching
// to xxmalloc — the only frame where the RA is unambiguous. Reading
// __builtin_return_address here instead is unreliable: how many frames sit
// between the app and this code depends on inlining/LTO/tail-call decisions
// (measured on macOS Release: every RA seen by callsiteArena pointed into
// libsmash's own wrapper code, collapsing all call sites into one arena).
// The RA(0) fallback covers entries that bypass the wrappers — direct
// SmashHeap calls from tests and internal code — where no app site exists.
[[gnu::always_inline]]
inline uintptr_t appCallerRA() {
    if (void* ra = alloc8_caller_ra) [[likely]]
        return reinterpret_cast<uintptr_t>(ra);
    return reinterpret_cast<uintptr_t>(__builtin_return_address(0));
}

class SmashHeap {
    // Slab array sized to kTotalArenas (= kNumArenas when A3 off, 2*kNumArenas
    // when SMASH_COLD_ARENA_FEEDBACK is on).  Layout:
    //   arenas [0, kNumArenas)          — hot sub-arenas (default routing)
    //   arenas [kNumArenas, 2*kNumArenas) — cold sub-arenas (underfilled)
    Slab slabs_[kTotalArenas * kNumClasses];

    // A3 feedback state (unused/untouched when kColdArenaFeedback is false).
    // One entry per (base_arena, size_class).  Compressor bumps the counter
    // after every successful page compression that originated from the
    // corresponding slab; once it crosses kColdArenaThreshold, the sticky
    // bias flag flips to 1 and callsiteArena() routes subsequent
    // allocations to the cold sub-arena for that (arena, sc).
    std::atomic<uint32_t> compress_count_[kNumArenas * kNumClasses]{};
    std::atomic<uint8_t> cold_bias_[kNumArenas * kNumClasses]{};

    // C1b (adaptive cap) feedback.  Per (base_arena, size_class) counter
    // of re-warm events — a page that was COMPRESSED but got faulted back
    // to ACTIVE.  q̂ = decomp / (comp + decomp) is the online Pareto
    // estimator; cached cap values live in adaptive_cap_ so the slab
    // allocateNewSpan path can read them without recomputing every call.
    std::atomic<uint32_t> decompress_count_[kNumArenas * kNumClasses]{};
    std::atomic<uint32_t> adaptive_cap_[kNumArenas * kNumClasses]{};

#ifdef SMASH_POWER_OF_TWO_CHOICES
    // Power-of-two-choices load counters. Per (arena, size_class) allocation
    // count used to pick the less contended arena. Approximate - wrapping and
    // relaxed atomics are fine since we only need relative comparison.
    std::atomic<uint32_t> arena_alloc_count_[kNumArenas * kNumClasses]{};
#endif

    // Cohort measurement arrays (kMeasureCohorts only).  Per-page tracking
    // of first allocating thread ID and RA hash; a "mixed" flag per axis
    // flips when a second distinct value is seen.  Bootstrap-allocated once
    // compression is initialized.
    struct CohortPage {
        uint32_t first_tid;
        uint32_t first_ra;
        uint8_t  mixed_tid;
        uint8_t  mixed_ra;
    };
    CohortPage* cohort_pages_ = nullptr;
    size_t cohort_pages_len_ = 0;

    LargeAlloc large_alloc_;
    PageMap page_map_;

    // ── ASLR-resilient call-site hash cache ─────────────────────────────────
    // Maps (return_address >> 12) -> stable offset within shared object.
    // Resolving the image base is not free (see imageBaseFor), so we cache. The cache is
    // direct-mapped for minimal overhead on the hot path (~2ns vs 0.5ns for
    // the original hash).
    //
    // Design: 4096 entries, direct-mapped by (ra >> 12) hash. Each entry
    // stores the full ra_page key + precomputed stable hash. On collision,
    // we just recompute (rare - most programs have <1000 unique call sites).
    static constexpr size_t kStableBucketCacheSize = 4096;
    static constexpr size_t kStableBucketCacheMask = kStableBucketCacheSize - 1;
    struct alignas(16) StableBucketEntry {
        uintptr_t ra;           // full return address as cache key
        uint32_t stable_hash;   // precomputed ASLR-stable hash
        uint32_t pad;
    };
    StableBucketEntry stable_bucket_cache_[kStableBucketCacheSize]{};

    // Compute ASLR-resilient hash for a return address.
    // Uses dladdr() to get offset within shared object, caches result.
    // Hot path: single cache lookup with no atomics (ok for single-writer
    // per slot assumption - collisions just cause re-lookup).
    [[gnu::always_inline]]
    uint32_t stableCallsiteHash(uintptr_t ra) {
        size_t slot = (ra * 0x9E3779B97F4A7C15ULL) & kStableBucketCacheMask;

        // Fast path: direct cache hit (no atomics, just plain loads)
        StableBucketEntry& e = stable_bucket_cache_[slot];
        if (e.ra == ra) [[likely]] {
            return e.stable_hash;
        }

        // Slow path: cache miss, call dladdr and populate
        return stableCallsiteHashSlow(ra, slot);
    }

    // ── Image table: return address -> image base, without dladdr ─────────
    // dladdr() searches the image's symbol table for the closest symbol. We
    // only need the image base (dli_fbase), and for a large image (rustc's
    // librustc_driver) that search costs microseconds; with tens of thousands
    // of allocation sites it dominated rustc's run time (5-6x slowdown).
    // Instead keep a sorted table of loaded images' executable ranges and
    // binary-search it. Rebuilt (from BootstrapAlloc, never malloc) only when
    // an address falls outside every known image, e.g. after dlopen; old
    // tables are leaked so concurrent readers never see freed memory.
    struct ImageRange { uintptr_t lo, hi, base; };
    struct ImageTable { size_t n; ImageRange r[1]; };
    std::atomic<ImageTable*> image_table_{nullptr};
    Spinlock image_table_lock_;

    static void sortRanges(ImageRange* r, size_t n) {
        for (size_t i = 1; i < n; ++i) {           // insertion sort; n ~ 1e3, rare
            ImageRange x = r[i];
            size_t j = i;
            while (j > 0 && r[j - 1].lo > x.lo) { r[j] = r[j - 1]; --j; }
            r[j] = x;
        }
    }

#if defined(__linux__)
    struct PhdrCollect { ImageRange* out; size_t cap; size_t n; };
    static int collectPhdr(struct dl_phdr_info* info, size_t, void* arg) {
        auto* c = static_cast<PhdrCollect*>(arg);
        uintptr_t first = UINTPTR_MAX;
        for (int i = 0; i < info->dlpi_phnum; ++i)
            if (info->dlpi_phdr[i].p_type == PT_LOAD && info->dlpi_phdr[i].p_vaddr < first)
                first = info->dlpi_phdr[i].p_vaddr;
        if (first == UINTPTR_MAX) return 0;
        // dladdr's dli_fbase is the map start: load bias + lowest PT_LOAD vaddr.
        uintptr_t base = info->dlpi_addr + (first & ~uintptr_t(4095));
        for (int i = 0; i < info->dlpi_phnum; ++i) {
            const auto& ph = info->dlpi_phdr[i];
            if (ph.p_type != PT_LOAD || !(ph.p_flags & PF_X)) continue;
            if (c->n < c->cap)
                c->out[c->n++] = {info->dlpi_addr + ph.p_vaddr,
                                  info->dlpi_addr + ph.p_vaddr + ph.p_memsz, base};
        }
        return 0;
    }
#endif

    [[gnu::noinline]]
    ImageTable* rebuildImageTable() {
        LockGuard guard(image_table_lock_);
#if defined(__APPLE__)
        size_t cap = _dyld_image_count() + 64;
#else
        size_t cap = 0;
        dl_iterate_phdr([](struct dl_phdr_info* info, size_t, void* arg) {
            for (int i = 0; i < info->dlpi_phnum; ++i)
                if (info->dlpi_phdr[i].p_type == PT_LOAD) ++*static_cast<size_t*>(arg);
            return 0;
        }, &cap);
        cap += 64;
#endif
        auto* t = static_cast<ImageTable*>(BootstrapAlloc::instance().allocate(
            sizeof(ImageTable) + cap * sizeof(ImageRange), alignof(ImageTable)));
        if (!t) return image_table_.load(std::memory_order_acquire);
        size_t n = 0;
#if defined(__APPLE__)
        uint32_t count = _dyld_image_count();
        for (uint32_t i = 0; i < count && n < cap; ++i) {
            auto* hdr = reinterpret_cast<const struct mach_header_64*>(_dyld_get_image_header(i));
            if (!hdr) continue;
            unsigned long size = 0;
            uint8_t* text = getsegmentdata(hdr, "__TEXT", &size);
            if (!text || !size) continue;
            uintptr_t lo = reinterpret_cast<uintptr_t>(text);
            t->r[n++] = {lo, lo + size, reinterpret_cast<uintptr_t>(hdr)};
        }
#else
        PhdrCollect c{t->r, cap, 0};
        dl_iterate_phdr(collectPhdr, &c);
        n = c.n;
#endif
        sortRanges(t->r, n);
        t->n = n;
        image_table_.store(t, std::memory_order_release);
        return t;
    }

    static uintptr_t findImageBase(const ImageTable* t, uintptr_t ra) {
        if (!t) return 0;
        size_t lo = 0, hi = t->n;
        while (lo < hi) {                          // last range with r.lo <= ra
            size_t mid = (lo + hi) / 2;
            if (t->r[mid].lo <= ra) lo = mid + 1; else hi = mid;
        }
        if (lo == 0) return 0;
        const ImageRange& r = t->r[lo - 1];
        return ra < r.hi ? r.base : 0;
    }

    uintptr_t imageBaseFor(uintptr_t ra) {
        ImageTable* t = image_table_.load(std::memory_order_acquire);
        if (uintptr_t b = findImageBase(t, ra)) return b;
        return findImageBase(rebuildImageTable(), ra);   // new image (dlopen)?
    }

    [[gnu::noinline]]
    uint32_t stableCallsiteHashSlow(uintptr_t ra, size_t slot) {
        uint32_t hash = 0;
        uintptr_t base = imageBaseFor(ra);
        if (!base) {
            // Not in any image's code we enumerated: fall back to dladdr.
            Dl_info info{};
            if (dladdr(reinterpret_cast<void*>(ra), &info) && info.dli_fbase)
                base = reinterpret_cast<uintptr_t>(info.dli_fbase);
        }
        if (base) {
            uintptr_t offset = ra - base;
            // Full-avalanche integer hash: even offsets differing by 0x20
            // produce completely different outputs. The old offset^(offset>>16)
            // was a no-op for offsets < 64K (most functions in the same DSO).
            uint32_t x = static_cast<uint32_t>(offset);
            x ^= x >> 16; x *= 0x45d9f3bU;
            x ^= x >> 16; x *= 0x45d9f3bU;
            x ^= x >> 16;
            hash = x;
        } else {
            // dladdr failed - use low 20 bits as fallback
            hash = static_cast<uint32_t>(ra & 0xFFFFF);
        }

        // Store in cache (direct-mapped, overwrites any existing entry)
        stable_bucket_cache_[slot].stable_hash = hash;
        stable_bucket_cache_[slot].ra = ra;  // Write key last
        return hash;
    }

    Slab& slab(uint8_t arena, uint8_t sc) { return slabs_[arena * kNumClasses + sc]; }

    // Hot path: the memo hit, inlined into malloc. Everything else (hashing,
    // stable call-site resolution, tracing) is in callsiteArenaSlow(), so the
    // malloc fast path no longer pays that function's frame setup, callee-saved
    // register spills and stack-protector canary on every call (x86-64 perf:
    // callsiteArena was 13% of malloc+free cycles, almost all prologue/epilogue).
    [[gnu::always_inline]]
    uint8_t callsiteArena(uint8_t sc, uintptr_t ra) {
#if !defined(SMASH_ABLATION_NO_CALLSITE_ARENA) && !defined(SMASH_NO_ARENA_MEMO)
        const uintptr_t memo_key = ra ^ (static_cast<uintptr_t>(sc) << 56);
        const size_t memo_slot =
            (memo_key * 0x9E3779B97F4A7C15ULL >> 56) & (kArenaMemoSlots - 1);
        const ArenaMemo& memo = g_arena_memo[memo_slot];
        if (memo.valid && memo.key == memo_key) [[likely]] {
            uint8_t base = memo.base;
            if constexpr (kColdArenaFeedback) {
                if (cold_bias_[base * kNumClasses + sc].load(std::memory_order_relaxed))
                    return static_cast<uint8_t>(base + getNumArenas());
            }
            return base;
        }
#endif
        return callsiteArenaSlow(sc, ra);
    }

    [[gnu::noinline, gnu::cold]]
    uint8_t callsiteArenaSlow(uint8_t sc, uintptr_t ra) {
#ifdef SMASH_ABLATION_NO_CALLSITE_ARENA
        (void)ra;
        uint8_t base = 0;
#else
#if !defined(SMASH_NO_ARENA_MEMO)
        // Per-op fast path: the arena `base` for a given (ra, sc) is a pure
        // deterministic function of (stable_ra, depth_bucket, sc) — but the
        // murmur3 finalizer that computes it is a chain of dependent 64-bit
        // multiplies that stalls the pipeline on EVERY malloc (measured: 1-thread
        // IPC 1.26 vs glibc 1.66, the whole contention-free deficit — see PLAN).
        // Memoize `base` in a tiny direct-mapped TLS cache keyed on (ra, sc) so the
        // hash runs once per call-site/class per thread. Routing is unchanged: the
        // hash is deterministic in (ra, sc) within a thread (depth_bucket only
        // varies with recursion depth at the same call site, a rare corner we
        // accept — the arena is a compression-homogeneity hint, not correctness).
        // The cold-bias check below is intentionally NOT memoized (must stay live).
        // (Memo hit is handled inline in callsiteArena(); this re-check is
        // the miss path's own lookup so it can fill the same slot below.)
        const uintptr_t memo_key = ra ^ (static_cast<uintptr_t>(sc) << 56);
        const size_t memo_slot =
            (memo_key * 0x9E3779B97F4A7C15ULL >> 56) & (kArenaMemoSlots - 1);
        ArenaMemo& memo = g_arena_memo[memo_slot];
        if (memo.valid && memo.key == memo_key) [[likely]] {
            uint8_t base = memo.base;
            if constexpr (kColdArenaFeedback) {
                if (cold_bias_[base * kNumClasses + sc].load(std::memory_order_relaxed))
                    return static_cast<uint8_t>(base + getNumArenas());
            }
            return base;
        }
#endif
        // LLAMA-style stack hash [Maas et al., ASPLOS 2020]:
        // hash(return_address, stack_depth, object_size).
        //
        // The return address is captured at the alloc8 wrapper entry
        // (alloc8_caller_ra, read via appCallerRA()) — the only frame where
        // __builtin_return_address(0) unambiguously names the application's
        // call site regardless of inlining/LTO/tail-call decisions.
        //
        // ASLR-resilient: stableCallsiteHash() resolves the return address
        // to an offset within its shared object via dladdr(), with caching.
        uint32_t stable_ra = stableCallsiteHash(ra);

        // Compute the stack-depth bucket: the current frame's offset below the
        // thread's TRUE stack base (threadStackBase(), cached in the initial-exec
        // TLS g_arena_stack_base). Anchoring to a fixed base makes depth an
        // ASLR-invariant, history-free function of the call path — see the
        // g_arena_stack_base declaration for why the old drifting reference was
        // replaced.
        //
        // SMASH_ABLATION_NO_DEPTH (off by default): drop the LLAMA stack-depth
        // component so the arena becomes hash(RA, size_class) only. This is a
        // measurement gate for quantifying how much depth segregation buys on
        // sub-page objects; production always builds with depth ON. Do NOT make
        // this the default — see feedback_llama_segregation.
#ifdef SMASH_ABLATION_NO_DEPTH
        uint8_t depth_bucket = 0;
#else
        // Depth bucket: current frame's offset below the thread's true stack
        // base, in 16 KB units, truncated to 8 bits. Anchored to a fixed,
        // ASLR-invariant base (threadStackBase()) so the same call path always
        // yields the same bucket — no history drift.
        uintptr_t frame = reinterpret_cast<uintptr_t>(__builtin_frame_address(0));
        uintptr_t stack_base = threadStackBase();
        uint8_t depth_bucket = (stack_base > frame)
            ? static_cast<uint8_t>(((stack_base - frame) >> 14) & 0xFF)
            : 0;
#endif

        // Mix with murmur3 finalizer to spread closely-spaced addresses
        // across arenas. Without this, adjacent noinline functions (differing
        // by ~32 bytes in .text) hash to the same arena because the low bits
        // of a simple XOR don't change enough.
        uintptr_t h = static_cast<uintptr_t>(stable_ra) ^
                      (static_cast<uintptr_t>(depth_bucket) << 8) ^
                      static_cast<uintptr_t>(sc);
        h ^= h >> 33;
        h *= 0xff51afd7ed558ccdULL;
        h ^= h >> 33;
        h *= 0xc4ceb9fe1a85ec53ULL;
        h ^= h >> 33;
        if (cpuArenaHash()) {
            // CPU-indexed routing. Mix the running CPU (not a per-thread id)
            // into the arena hash so the number of distinct lanes tracks the
            // core count, not the thread count. With per-thread ids, N threads
            // hashing onto M arenas collide by the birthday paradox (measured:
            // Larson@64t variance 17-50M ops/s as threads randomly share an
            // arena's slab lock). Indexing by CPU makes threads that share a
            // core share an arena *structurally* — and they can't run
            // concurrently on that core, so per-arena slab-lock contention is
            // bounded by oversubscription, not luck. sched_getcpu() is vDSO
            // (~3ns) and this runs only on the refill slow path (~1/batch),
            // so the cost is amortized. A migration between read and use only
            // changes routing (a hint); the slab lock still guards correctness.
            const int cpu = currentCpu();
            const uint32_t lane = (cpu >= 0) ? static_cast<uint32_t>(cpu) : 0;
            h ^= static_cast<uintptr_t>(lane) * 0x9E3779B97F4A7C15ULL;
        } else if constexpr (kThreadArenaHash) {
            // Thread-local monotonic id (A2-lite).  Multiplied by a
            // large odd constant so the low bits mix well into h.
            static std::atomic<uint32_t> next_tid{0};
            thread_local uint32_t tid = next_tid.fetch_add(1, std::memory_order_relaxed);
            h ^= static_cast<uintptr_t>(tid) * 0x9E3779B97F4A7C15ULL;
        }
        h ^= h >> 16;

        // Deterministic arena assignment: same call site always maps to same
        // arena. This maximizes compression homogeneity by keeping allocations
        // from the same origin together. We have enough arenas (scaled by CPU
        // count) to avoid contention without load balancing.
        const int mask = getArenaMask();
        uint8_t base = static_cast<uint8_t>(h & mask);

#ifdef SMASH_POWER_OF_TWO_CHOICES
        // Power of two choices (disabled by default): hash to two candidate
        // arenas and pick the less contended one. This reduces max load from
        // O(log n / log log n) to O(log log n) with n threads, but spreads
        // allocations from the same call site across arenas, hurting
        // compression homogeneity. Enable with -DSMASH_POWER_OF_TWO_CHOICES
        // if contention is a bigger concern than compression ratio.
        uint8_t arena1 = base;
        uint8_t arena2 = static_cast<uint8_t>((h >> 8) & mask);
        if (arena1 == arena2) {
            arena2 = static_cast<uint8_t>((arena1 + 1) & mask);
        }
        uint32_t load1 = arena_alloc_count_[arena1 * kNumClasses + sc]
                             .load(std::memory_order_relaxed);
        uint32_t load2 = arena_alloc_count_[arena2 * kNumClasses + sc]
                             .load(std::memory_order_relaxed);
        base = (load1 <= load2) ? arena1 : arena2;
        arena_alloc_count_[base * kNumClasses + sc]
            .fetch_add(1, std::memory_order_relaxed);
#endif
#if !defined(SMASH_NO_ARENA_MEMO) && !defined(SMASH_POWER_OF_TWO_CHOICES)
        // Cache the computed base for this (ra, sc). Not done under
        // power-of-two-choices, where base depends on live per-arena load.
        memo.base = base;
        memo.key = memo_key;
        memo.valid = true;
#endif
#endif
#ifndef SMASH_ABLATION_NO_CALLSITE_ARENA
        // SMASH_ARENA_TRACE=1: print the first 32 distinct return addresses
        // seen by arena routing, with their resolved image + offset and the
        // arena chosen. Diagnostic for verifying that app call sites (not
        // wrapper-internal addresses) reach the router on a given
        // platform/compiler/LTO combination. Zero cost when unset beyond one
        // predicted branch.
        static int trace_arena = []{
            const char* v = std::getenv("SMASH_ARENA_TRACE");
            return (v && v[0] == '1') ? 1 : 0;
        }();
        if (trace_arena) [[unlikely]] {
            static uintptr_t seen[32];
            static std::atomic<int> nseen{0};
            int n = nseen.load(std::memory_order_relaxed);
            bool found = false;
            for (int i = 0; i < n && i < 32; ++i)
                if (seen[i] == ra) { found = true; break; }
            if (!found && n < 32) {
                seen[n] = ra;
                nseen.store(n + 1, std::memory_order_relaxed);
                Dl_info info{};
                const char* fn = "?";
                uintptr_t off = 0;
                if (dladdr(reinterpret_cast<void*>(ra), &info) && info.dli_fbase) {
                    fn = info.dli_fname ? info.dli_fname : "?";
                    off = ra - reinterpret_cast<uintptr_t>(info.dli_fbase);
                }
                char buf[256];
                int len = smash::safe_snprintf(buf, sizeof(buf),
                    "[arena-trace] ra=%p off=0x%lx stable=%x sc=%d depth=%d arena=%d in=%s\n",
                    (void*)ra, (unsigned long)off, stable_ra, (int)sc,
                    (int)depth_bucket, (int)base, fn);
                if (len > 0) (void)!::write(2, buf, (size_t)len);
            }
        }
#endif  // !SMASH_ABLATION_NO_CALLSITE_ARENA
        if constexpr (kColdArenaFeedback) {
            // If compressor has flagged this (arena, sc) as cold-biased,
            // route to the cold sub-arena.
            if (cold_bias_[base * kNumClasses + sc].load(std::memory_order_relaxed))
                return static_cast<uint8_t>(base + getNumArenas());
        }
        return base;
    }

    // Arena routing for large allocations. Uses the ASLR-stable return address
    // hash from the original call site. The return address is resolved via
    // dladdr to get the offset within the shared object, which is stable
    // across process restarts (enabling profile reuse).
    //
    // Takes caller_ra as parameter since this is called from mallocSlow, not
    // directly from the application - we need the address captured at the
    // malloc() entry point.
    uint8_t callsiteArenaForLarge(uintptr_t caller_ra) {
#ifdef SMASH_ABLATION_NO_CALLSITE_ARENA
        (void)caller_ra;
        return 0;
#else
        uint32_t stable_ra = stableCallsiteHash(caller_ra);

        // Simple hash of the stable return address
        uintptr_t h = static_cast<uintptr_t>(stable_ra);
        if constexpr (kThreadArenaHash) {
            static std::atomic<uint32_t> next_tid{0};
            thread_local uint32_t tid = next_tid.fetch_add(1, std::memory_order_relaxed);
            h ^= static_cast<uintptr_t>(tid) * 0x9E3779B97F4A7C15ULL;
        }
        h ^= h >> 16;

        const int mask = getArenaMask();
        return static_cast<uint8_t>(h & mask);
#endif
    }

public:
    // Called by the compressor after a page transitions to COMPRESSED.
    // Updates the (base_arena, size_class) compression counter; once the
    // threshold is crossed, flips the sticky cold-bias flag so subsequent
    // allocations route to the cold sub-arena.
    void onPageCompressed(uint8_t arena_id, uint8_t sc) {
        if constexpr (!kColdArenaFeedback && !kAdaptiveCap) return;
        if (sc >= kNumClasses) return;
        uint8_t base = arena_id & getArenaMask();   // strip cold half
        size_t idx = base * kNumClasses + sc;
        if constexpr (kColdArenaFeedback) {
            uint32_t prev = compress_count_[idx].fetch_add(1, std::memory_order_relaxed);
            if (prev + 1 == kColdArenaThreshold) {
                cold_bias_[idx].store(1, std::memory_order_relaxed);
            }
        } else {
            compress_count_[idx].fetch_add(1, std::memory_order_relaxed);
        }
        if constexpr (kAdaptiveCap) {
            recomputeAdaptiveCap(idx);
        }
    }

    static void compressedEventHook(size_t /*page_idx*/, uint8_t arena_id,
                                    uint8_t sc, void* ctx) {
        static_cast<SmashHeap*>(ctx)->onPageCompressed(arena_id, sc);
    }

    // Called by the compressor when a COMPRESSED page faults back to
    // ACTIVE.  Updates the decompression counter and recomputes the
    // cached adaptive cap for this (base_arena, sc).  The cap is the
    // smallest N with (1 - q̂)^N >= P_target, floored at kAdaptiveCapMin.
    // Hot buckets (q̂ >= q_max) disable the cap outright — under-packing
    // a hot bucket costs RSS without winning compressibility.
    void onPageDecompressed(uint8_t arena_id, uint8_t sc) {
        if constexpr (!kAdaptiveCap) return;
        if (sc >= kNumClasses) return;
        uint8_t base = arena_id & getArenaMask();
        size_t idx = base * kNumClasses + sc;
        decompress_count_[idx].fetch_add(1, std::memory_order_relaxed);
        recomputeAdaptiveCap(idx);
    }

    static void decompressedEventHook(size_t /*page_idx*/, uint8_t arena_id,
                                      uint8_t sc, void* ctx) {
        static_cast<SmashHeap*>(ctx)->onPageDecompressed(arena_id, sc);
    }

    // Recompute and cache the adaptive cap for bucket idx.  Called whenever
    // compress or decompress counters change.  Cheap (one log() call) and
    // races are benign — worst case we publish a slightly stale cap.
    void recomputeAdaptiveCap(size_t idx) {
        uint32_t comp = compress_count_[idx].load(std::memory_order_relaxed);
        uint32_t dec  = decompress_count_[idx].load(std::memory_order_relaxed);
        uint32_t total = comp + dec;
        uint32_t cap = 0;

        // Gate 1 — total-evidence floor.  Early in a bucket's life we know
        // nothing; cap=0 keeps the allocator in its default high-density path.
        if (total < kAdaptiveCapMinSamples) {
            adaptive_cap_[idx].store(0, std::memory_order_relaxed);
            return;
        }

        // Gate 2 — bilateral-evidence gate.  A stream of compressions with
        // zero re-warms looks like a "truly cold" bucket under the Pareto
        // model, but observationally is indistinguishable from "bucket that
        // hasn't had time to re-warm yet".  Workloads like duckdb and redis
        // fill-then-idle fall in the second camp; capping them before any
        // decomp event arrives bakes tiny caps into every span and
        // catastrophically regresses RSS once queries actually touch the
        // data.  Require at least one observed re-warm before applying a
        // cap.  If a bucket is genuinely cold forever, the default path
        // already wins (pages compress, stay compressed) — no under-packing
        // is needed.
        if (dec == 0) {
            adaptive_cap_[idx].store(0, std::memory_order_relaxed);
            return;
        }

        // Gate 3 — hot-bucket rejection.  Under-packing a hot bucket just
        // balloons RSS.  Laplace smoothing α=1 keeps q̂ bounded away from 0
        // for thin samples.
        constexpr double q_max = 0.30;
        double q = double(dec + 1) / double(total + 2);
        if (q >= q_max) {
            adaptive_cap_[idx].store(0, std::memory_order_relaxed);
            return;
        }

        double p = double(kAdaptiveCapTargetPct) / 100.0;
        double n = std::log(p) / std::log(1.0 - q);
        uint32_t cap_u = static_cast<uint32_t>(n);
        cap = cap_u < kAdaptiveCapMin ? kAdaptiveCapMin : cap_u;
        adaptive_cap_[idx].store(cap, std::memory_order_relaxed);
    }

    // Slab-facing cap lookup.  Returns the current cached cap for the
    // (arena, sc) bucket; 0 = no cap.  Called from Slab::allocateNewSpan
    // and Slab::maybeWiden.
    //
    // Clamps the cap against an "underfill floor" equal to
    // natural_slots_per_page / kAdaptiveCapUnderfillFactor — a cap that
    // would leave fewer than (natural / factor) slots per page is
    // rejected.  Without this clamp, a cap of 8 applied to a 64B size
    // class (natural density = 256 slots/page on a 16K page) forces 32×
    // VM underfill.  Compression reclaims zero-filled waste, but for
    // throughput-heavy workloads (duckdb, redis fill) allocations
    // outpace the compressor and RSS balloons.  The floor keeps the
    // underfill factor bounded so the Pareto-motivated cap only fires
    // where the cost is tolerable.
    static uint32_t adaptiveCapQuery(void* ctx, uint8_t arena, uint8_t sc) {
        auto* self = static_cast<SmashHeap*>(ctx);
        if (sc >= kNumClasses) return 0;
        uint8_t base = arena & getArenaMask();
        uint32_t raw = self->adaptive_cap_[base * kNumClasses + sc]
                           .load(std::memory_order_relaxed);
        if (raw == 0) return 0;
        uint32_t osz = kSizeClasses[sc].size;
        if (osz == 0 || osz >= kPageSize) return raw;
        uint32_t natural = static_cast<uint32_t>(kPageSize / osz);
        constexpr uint32_t kUnderfillFactor = 4;
        uint32_t floor_cap = natural / kUnderfillFactor;
        if (floor_cap < kAdaptiveCapMin) floor_cap = kAdaptiveCapMin;
        return raw < floor_cap ? floor_cap : raw;
    }

private:

    // Phase 2-4: compression infrastructure
    VmRegion vm_region_;
    PageStateTable page_states_;
    PageLockTable page_locks_;
    CompressStore compress_store_;
    CompressEngine compress_engine_;
    CompressorThread compressor_;
    vm::FaultHandler fault_handler_;

    bool compression_inited_ = false;
    std::atomic<bool> compression_started_{false};
    std::atomic<int> warmup_count_{0};

    // Fault callback: decompress on access to compressed/monitored pages
    static bool faultCallback(uintptr_t fault_addr, void* ctx) {
        auto* self = static_cast<SmashHeap*>(ctx);
        return self->compressor_.handleFault(fault_addr);
    }

    // Release hook: called by slab when freeing spans that may be compressed
    static void releaseHook(size_t page_idx, size_t page_count, void* ctx) {
        auto* self = static_cast<SmashHeap*>(ctx);
        self->compressor_.releaseCompressedPages(page_idx, page_count);
    }

    void startCompression() {
        bool expected = false;
        if (!compression_started_.compare_exchange_strong(expected, true))
            return;
        // SMASH_NO_COMPRESSOR=1: Skip starting compression threads entirely.
        // Smash still works as a malloc replacement, but no pages get compressed.
        // Useful for isolating whether the compressor threads cause shutdown issues.
        static const bool no_compressor = [] {
            const char* v = std::getenv("SMASH_NO_COMPRESSOR");
            return v && v[0] == '1';
        }();
        if (no_compressor) return;
        fault_handler_.start(faultCallback, this);
        compressor_.start();
    }

public:
    // Called from a pthread_atfork child handler. After fork, the child has
    // inherited compression_started_=true and the parent's now-defunct
    // pthread_t handles, so the next allocation's `if (!started) startCompression()`
    // check short-circuits and the child runs with no compressor — hence the
    // committed=N / compressed=0 we see in postgres backends, Redis daemonized
    // children, etc.
    //
    // Atfork handler trio. The compressor runs on background threads that
    // disappear at fork(); without this the child inherits dead pthread_t
    // handles plus `compression_started_=true`, so its first malloc's check
    // short-circuits and the child runs with no compressor at all (postgres
    // backends, daemonized redis children, etc. all hit this).
    //
    // prepare: ask the coordinator to skip its next tick(); wait briefly
    //   for any in-flight tick to drain. Without this, a fork that lands
    //   mid-tick leaves the child holding page-locks (etc.) acquired by a
    //   thread that no longer exists.
    // parent:  resume normal ticks.
    // child:   reset thread bookkeeping and mark compression not-started so it
    //   respawns LAZILY on the child's next thread creation (threadInit) — NOT
    //   synchronously here. Inherited PageState is left alone: COMPRESSED pages
    //   stay decompressible-on-fault because the compressed bytes live in
    //   BootstrapAlloc memory mapped CoW.
    //
    // Why not respawn the compressor here: startCompression() calls
    // pthread_create(), which alloc8 interposes and routes through xxmalloc ->
    // Slab::allocateBatch. If any slab/store lock was held at fork() time by a
    // thread that no longer exists in the child, that allocation spins forever
    // on the inherited-locked spinlock (the classic multithreaded-fork bug;
    // see issue #61 — the atfork prepare does not yet quiesce the allocator
    // locks). Forcing an allocation in the child handler makes EVERY forked
    // child pay that risk, including children that never allocate on their own
    // — e.g. Redis's startup checkLinuxMadvFreeForkBug(), whose fork child only
    // madvise()s a raw page and writes a byte to a pipe. That child deadlocked
    // here, hanging redis-server before it could serve (regression from
    // 8c11ee79). Deferring respawn to the first real thread creation keeps
    // non-allocating and fork+exec children deadlock-free while still restoring
    // the compressor for children that go on to do threaded work (the
    // multiprocessing/worker case 8c11ee79 targeted). A child that persists
    // single-threaded and only allocates gets no compressor until issue #61's
    // full prepare-side quiesce lands — strictly safer than deadlocking.
    void preparePauseForFork() {
        if (!compression_inited_) return;
        compressor_.pauseForFork();
    }
    void resumeAfterFork() {
        if (!compression_inited_) return;
        compressor_.resumeAfterFork();
    }
    void resetForFork() {
        if (!compression_inited_) return;
        // Reset decommit thread first (it doesn't exist in child after fork).
        // These reset internal bookkeeping only; they must not allocate or take
        // a slab lock (the child stack that motivated this fix reached them
        // safely and only deadlocked at the subsequent startCompression()).
        vm_region_.resetDecommitThreadForFork();
        compressor_.resetForFork();
        // Mark not-started; threadInit() respawns on the child's next thread
        // creation, on a clean stack rather than inside this atfork handler.
        compression_started_.store(false, std::memory_order_release);
    }

    // Stop compressor and drain compressed pages back to PROT_RW. Called
    // from a high-priority destructor in smash_heap.cpp to run before
    // CPython's __cxa_finalize, so any post-shutdown access to a smash
    // page reads ordinary memory instead of faulting on PROT_NONE.
    void shutdownCompressor() {
        if (compression_inited_ && compression_started_.load(std::memory_order_acquire)) {
            compressor_.stop();
        }
    }
private:

    // Track allocation in compress-only mode
    void trackAllocation(void* ptr, size_t size) {
        if (!ptr || size == 0 || !compression_inited_) return;
        uintptr_t start_page = reinterpret_cast<uintptr_t>(ptr) & ~(kPageSize - 1);
        uintptr_t end_page = (reinterpret_cast<uintptr_t>(ptr) + size - 1) & ~(kPageSize - 1);

        for (uintptr_t p = start_page; p <= end_page; p += kPageSize) {
            size_t idx = vm_region_.trackPage(p);
            if (idx > 0) {
                PageState st = page_states_.get(idx);
                if (st == PageState::EMPTY)
                    page_states_.set(idx, PageState::ACTIVE);
            }
        }

        // Start compression after warmup
        if (!compression_started_.load(std::memory_order_relaxed)) {
            if (warmup_count_.fetch_add(1, std::memory_order_relaxed) >= 5000)
                startCompression();
        }
    }

public:
    SmashHeap() {
        // Debug timing
        static const bool time_init = [] {
            const char* v = std::getenv("SMASH_TIME_INIT");
            return v && v[0] == '1';
        }();
        auto t0 = std::chrono::high_resolution_clock::now();
        auto log_time = [&](const char* label) {
            if (!time_init) return;
            auto now = std::chrono::high_resolution_clock::now();
            auto us = std::chrono::duration_cast<std::chrono::microseconds>(now - t0).count();
            fprintf(stderr, "[smash init] %s: %ld us\n", label, us);
            t0 = now;
        };

        bool compress_only = isCompressOnlyMode();
        log_time("mode_check");

        if (!compress_only) {
            page_map_.init();
        }
        log_time("page_map_init");

        // Try to init VmRegion for compression support
        bool vm_ok = vm_region_.init(getVmRegionSize());
        log_time("vm_region_init");

        if (vm_ok) {
            // Wire the radix tree to also publish span pointers into the
            // VmRegion's flat page→Span table. Must happen before any slab
            // initialization so the first allocation's setRange() mirrors.
            if (!compress_only) {
                page_map_.attachVmRegion(&vm_region_);
            }
            log_time("attach_vm");
            page_states_.init(vm_region_.totalPages());
            log_time("page_states_init");
            page_locks_.init(vm_region_.totalPages());
            log_time("page_locks_init");
            compress_store_.init();
            log_time("compress_store_init");
            compress_engine_.init();
            log_time("compress_engine_init");
            compressor_.init(&vm_region_, &page_states_, &page_locks_,
                             &compress_store_, &compress_engine_,
                             compress_only ? nullptr : &page_map_, &fault_handler_);
            log_time("compressor_init");
            // A3: register the cold-arena feedback hook (no-op when
            // SMASH_COLD_ARENA_FEEDBACK is off — onPageCompressed early-returns).
            if (!compress_only && (kColdArenaFeedback || kAdaptiveCap)) {
                compressor_.setCompressedCallback(compressedEventHook, this);
            }
            if (!compress_only && kAdaptiveCap) {
                compressor_.setDecompressedCallback(decompressedEventHook, this);
            }

            if (!compress_only) {
                const int num_arenas = getNumArenas();
                const int total_arenas = getTotalArenas();
                for (int a = 0; a < total_arenas; ++a) {
                    // Hot sub-arenas (a < num_arenas): no per-page cap.
                    // Cold sub-arenas (a >= num_arenas, only when A3 on):
                    // apply kMaxSlotsPerPage to produce sparse pages.
                    // If A3 is off but kMaxSlotsPerPage > 0, the cap
                    // applies globally (ablation mode — measure C1 in
                    // isolation without the feedback loop).
                    uint32_t cap = 0;
                    if constexpr (kColdArenaFeedback) {
                        if (a >= num_arenas) cap = kMaxSlotsPerPage;
                    } else {
                        cap = kMaxSlotsPerPage;
                    }
                    for (int i = 0; i < kNumClasses; ++i) {
                        slabs_[a * kNumClasses + i].init(
                            static_cast<uint8_t>(i), &page_map_,
                            &vm_region_, &page_states_,
                            releaseHook, this,
                            static_cast<uint8_t>(a), cap,
                            compressor_.coldCounts());
                        if constexpr (kAdaptiveCap) {
                            slabs_[a * kNumClasses + i].setCapFn(
                                adaptiveCapQuery, this);
                        }
                    }
                }
            }
            log_time("slabs_init");
            compression_inited_ = true;
            g_smash_vm_region = &vm_region_;
            g_smash_page_states_for_external = &page_states_;
            // Publish the lock table so the munmap / Mach VM deallocate
            // interposers can serialize external-page untracking against a
            // compressor worker's snapshot read (full-mode external-page race).
            g_smash_page_locks_for_external = &page_locks_;

            // When external mmap tracking is active (SMASH_TRACK_EXTERNAL=1),
            // tracked pages live at addresses OUTSIDE the contiguous arena, so
            // the decommit/mprotect bounds guards in platform_mem.h would
            // false-trap when the compressor reclaims a compressed external
            // page. Register VmRegion::contains() so the guards accept a
            // registered external page as in-bounds. Gated: only loosen the
            // guard when tracking is actually on. (lookupIdx is a lock-free
            // hash read → async-signal-safe.)
            if (std::getenv("SMASH_TRACK_EXTERNAL") &&
                std::getenv("SMASH_TRACK_EXTERNAL")[0] == '1') {
                vm::setExternalPageCheck(+[](uintptr_t addr) -> bool {
                    auto* r = g_smash_vm_region;
                    return r && r->contains(addr);
                });
            }

            if constexpr (kMeasureCohorts) {
                cohort_pages_len_ = vm_region_.totalPages();
                cohort_pages_ = bootstrapArray<CohortPage>(cohort_pages_len_);
                __builtin_memset(cohort_pages_, 0,
                                 cohort_pages_len_ * sizeof(CohortPage));
                compressor_.setCohortData(cohort_pages_, cohort_pages_len_);
            }
        } else if (!compress_only) {
            // Fallback: Phase 1 mode (no compression).  No feedback loop
            // possible here, so still respect kUnderfillDenom for ablation.
            const int num_arenas = getNumArenas();
            const int total_arenas = getTotalArenas();
            for (int a = 0; a < total_arenas; ++a) {
                uint32_t cap = 0;
                if constexpr (kColdArenaFeedback) {
                    if (a >= num_arenas) cap = kMaxSlotsPerPage;
                } else {
                    cap = kMaxSlotsPerPage;
                }
                for (int i = 0; i < kNumClasses; ++i)
                    slabs_[a * kNumClasses + i].init(
                        static_cast<uint8_t>(i), &page_map_,
                        nullptr, nullptr, nullptr, nullptr,
                        static_cast<uint8_t>(a), cap);
            }
        }

        if (!compress_only) {
            if (compression_inited_) {
                large_alloc_.init(&page_map_, &vm_region_, &page_states_,
                                  releaseHook, this);
            } else {
                large_alloc_.init(&page_map_);
            }
        }
        log_time("large_alloc_init");
        log_time("TOTAL_CONSTRUCTOR");
    }

    ThreadCache* getOrCreateThreadCache() {
        ThreadCache*& tc = currentThreadCache();
        if (!tc) tc = newThreadCache();
        return tc;
    }

    void stampCohort(void* ptr, uint32_t ra_hash) {
        if constexpr (!kMeasureCohorts) return;
        if (!cohort_pages_) return;
        uintptr_t pa = reinterpret_cast<uintptr_t>(ptr);
        if (!vm_region_.contains(pa)) return;
        size_t idx = vm_region_.pageIndex(pa);
        if (idx >= cohort_pages_len_) return;
        auto& cp = cohort_pages_[idx];
        static std::atomic<uint32_t> next_ctid{1};
        thread_local uint32_t ctid =
            next_ctid.fetch_add(1, std::memory_order_relaxed);
        if (cp.first_tid == 0) cp.first_tid = ctid;
        else if (cp.first_tid != ctid) cp.mixed_tid = 1;
        if (cp.first_ra == 0) cp.first_ra = ra_hash;
        else if (cp.first_ra != ra_hash) cp.mixed_ra = 1;
    }

    // Out-of-line cold path for everything malloc() can't handle on the
    // fast path: compress-only/large-only/eager-zero modes, size > 16 KiB
    // (large alloc), or thread cache miss (refill from slab).  Marked
    // noinline+cold so the fast-path body stays small and the prologue
    // can be a single register save pair.
    [[gnu::noinline, gnu::cold]]
    void* mallocSlow(size_t size, uintptr_t caller_ra = 0) {
        // SMASH_PASSTHROUGH=1: pass ALL allocations to system malloc.
        // Lazy-resolve if needed so we never fall through to smash's heap —
        // a smash-allocated pointer leaking into passthrough mode causes
        // "invalid pointer" aborts when realloc/free reach the system.
        if (isPassthroughMode()) {
            if (!g_system_alloc.malloc) g_system_alloc.resolve();
            if (g_system_alloc.malloc) return g_system_alloc.malloc(size);
            return nullptr;
        }

        // Compressor threads route all allocations to the system allocator.
        // This prevents the compressor from allocating slab pages that it
        // will later compress — which causes SIGSEGV recursion when libc
        // functions on the compressor thread access their own heap data.
        if (g_compressor_thread) {
            if (!g_system_alloc.malloc) g_system_alloc.resolve();
            if (g_system_alloc.malloc) return g_system_alloc.malloc(size);
            return nullptr;
        }

        if (isCompressOnlyMode()) {
            // During early init, g_system_alloc may not be resolved yet
            if (!g_system_alloc.malloc) return nullptr;
            void* ptr = g_system_alloc.malloc(size);
            trackAllocation(ptr, size);
            return ptr;
        }

        if (size == 0) size = 1;

        // Large-only mode: allocations <= largeOnlyThreshold() pass through
        // to system malloc. Default threshold is kMaxSmallSize (16 KB) —
        // overridable via SMASH_LARGE_ONLY_THRESHOLD env var.
        if (isLargeOnlyMode() && size <= largeOnlyThreshold()) {
            // Lazy-resolve system malloc if not done yet (can happen during
            // very early init before our constructor runs).
            if (!g_system_alloc.malloc) g_system_alloc.resolve();
            if (g_system_alloc.malloc) return g_system_alloc.malloc(size);
            // If system malloc couldn't be resolved (during very early init before
            // dlsym is available), fall through to smash's slab allocator.
        }

        uint8_t sc = sizeToClass(size);
        if (sc < kNumClasses) {
            ThreadCache* tc = getOrCreateThreadCache();
            uintptr_t ra_for_arena = caller_ra ? caller_ra : appCallerRA();
            uint8_t arena = callsiteArena(sc, ra_for_arena);
            void* ptr = tc->allocate(sc, arena);
            if (!ptr) ptr = tc->refill(sc, arena, &slab(arena, sc));
            if constexpr (kMeasureCohorts) {
                if (ptr) {
                    uintptr_t ra = ra_for_arena;
                    uint32_t ra32 = static_cast<uint32_t>(ra ^ (ra >> 32));
                    stampCohort(ptr, ra32);
                }
            }
            if (ptr && isEagerZeroMode())
                __builtin_memset(ptr, 0, classSize(sc));
            return ptr;
        }
        // For large allocations, use the caller's return address for arena routing
        uintptr_t ra_for_arena = caller_ra ? caller_ra : appCallerRA();
        uint8_t arena = callsiteArenaForLarge(ra_for_arena);
        void* ptr = large_alloc_.allocate(size, kMinAlignment, arena);
        if (!ptr) {
            char dbg[128];
            int n = smash::safe_snprintf(dbg, sizeof(dbg),
                "[smash debug] large_alloc_.allocate(%zu) returned NULL\n", size);
            if (n > 0) (void)!::write(STDERR_FILENO, dbg, n);
        }
        if (ptr && isEagerZeroMode()) __builtin_memset(ptr, 0, size);
        return ptr;
    }

    // Hot path.  size in [1, kMaxSmallSize] AND thread cache already
    // initialised AND non-empty for this size class → return the cached
    // pointer.  Anything else falls into mallocSlow.  Everything past the
    // first `[[likely]]` returns directly without a function call.
    [[gnu::always_inline]]
    void* malloc(size_t size) {
        // App call site for arena routing, captured by the alloc8 wrapper
        // entry (see appCallerRA()).
        uintptr_t caller_ra = appCallerRA();
        if (fullMallocPath() && size > 0 && size <= kMaxSmallSize) [[likely]] {
            uint8_t sc = sizeToClass(size);
            if (ThreadCache* tc = currentThreadCache()) [[likely]] {
                uint8_t arena = callsiteArena(sc, caller_ra);
                if (void* ptr = tc->allocate(sc, arena)) [[likely]] return ptr;
                if (void* ptr = tc->refill(sc, arena, &slab(arena, sc)))
                    return ptr;
            }
        }
        return mallocSlow(size, caller_ra);
    }

    // Hot path. A non-null, full-mode, smash-owned pointer is resolved to its
    // span via the TLS last-span cache (or the flat table on a miss) and
    // pushed to the thread cache — all inlined into the caller. Everything
    // uncommon (null, bootstrap pointer, passthrough/compress-only modes,
    // foreign pointers, drains, diagnostics) falls through to freeSlow().
    [[gnu::always_inline]]
    void free(void* ptr) {
        if (!ptr) [[unlikely]] return;
        // Anything other than plain full mode (passthrough/compress-only/etc.)
        // or an active diagnostic routes through the cold path, which handles
        // every mode and keeps the diagnostic counters complete.
        if (!fullMallocPath() || freeDiagnosticsActive()) [[unlikely]]
            return freeSlow(ptr);
        if (BootstrapAlloc::instance().owns(ptr)) [[unlikely]] return;

        const auto addr = reinterpret_cast<uintptr_t>(ptr);

        // TLS last-span cache. Read the generation BEFORE resolving the span
        // (the load-before-read ordering that makes the stamp safe — see
        // page_map.h). The hit test is a pure TLS scalar comparison: the
        // pointer falls in the cached span's [base, end) range AND the stamped
        // generation still matches (so that mapping has not been torn down).
        // No span dereference until a hit is confirmed — a locality-free
        // workload pays only TLS compares before falling through to the table.
        const auto gen = g_pagemap_generation.load(std::memory_order_acquire);
        Span* span;
        // No static branch hint: real workloads hit (free-locality), the
        // adversarial random-replacement churn always misses; the hardware
        // predictor adapts to either far better than a fixed [[likely]] that
        // would mispredict every iteration of the no-locality case.
        if (addr >= g_free_cache_base && addr < g_free_cache_end &&
            g_free_cache_gen == gen) {
            span = g_free_cache_span;
            g_free_cache_skip = 0;  // hit → next miss repopulates immediately
        } else {
            if (vm_region_.inContigArena(addr) && vm_region_.hasSpanTable()) [[likely]] {
                span = vm_region_.getSpan(vm_region_.contigPageIndex(addr));
            } else {
                span = page_map_.get(addr);
            }
            if (!span) [[unlikely]] return freeSlow(ptr);
            // Repopulate-throttle: under sustained misses (a locality-free
            // workload) the populate stores are pure overhead, so only
            // repopulate every freeCacheRepopInterval()-th miss. A hit above
            // resets the countdown to 0, so a workload with real locality
            // repopulates on the very next miss (immediate phase adoption).
            if (g_free_cache_skip == 0) {
                // Cache the span and its immutable [base, end) range with the
                // generation read above (never newer than the span we resolved
                // → a future stamp match is sound). A Span* describes the same
                // range for life, so base/end stay valid while the stamp holds.
                g_free_cache_span = span;
                g_free_cache_base = reinterpret_cast<uintptr_t>(span->base);
                g_free_cache_end = g_free_cache_base +
                    static_cast<uintptr_t>(span->page_count) * kPageSize;
                g_free_cache_gen = gen;
                g_free_cache_skip = freeCacheRepopInterval() - 1;
            } else {
                --g_free_cache_skip;
            }
        }
        if (span->is_large) [[unlikely]] { large_alloc_.deallocate(span); return; }
        const auto sc = span->size_class;
        const uint8_t arena = span->arena_id;
        ThreadCache* tc = currentThreadCache();
        if (!tc) [[unlikely]] return freeSlow(ptr);
        if (!tc->deallocate(sc, arena, ptr)) { tc->drain(sc, arena, slabs_, &page_map_); tc->deallocate(sc, arena, ptr); }
    }

    // Cold path: full free() semantics for every mode. Never inlined.
    [[gnu::noinline, gnu::cold]]
    void freeSlow(void* ptr) {
        if (!ptr) return;
        if (BootstrapAlloc::instance().owns(ptr)) return;

        // SMASH_COUNT_FREE=1: count every free entry. If we see the
        // process abort in glibc free with this counter at zero or
        // far below the malloc count, the call is bypassing our
        // interposer entirely.
        static const bool count_free = []{
            const char* v = std::getenv("SMASH_COUNT_FREE");
            return v && v[0] == '1';
        }();
        if (count_free) [[unlikely]] {
            static std::atomic<uint64_t> n{0};
            uint64_t c = n.fetch_add(1, std::memory_order_relaxed) + 1;
            if ((c & 0xFFFFF) == 0) {  // every ~1M
                char buf[64];
                int len = smash::safe_snprintf(buf, sizeof(buf),
                    "[smash free count] %lu\n", (unsigned long)c);
                if (len > 0) (void)!::write(2, buf, (size_t)len);
            }
        }

        // SMASH_PASSTHROUGH=1: pass all frees to system malloc, except for
        // pointers that smash itself allocated during early init (before
        // g_system_alloc was resolvable). Those need smash's free path.
        if (isPassthroughMode()) {
            uintptr_t addr = reinterpret_cast<uintptr_t>(ptr);
            Span* sp = vm_region_.inContigArena(addr) && vm_region_.hasSpanTable()
                ? vm_region_.getSpan(vm_region_.contigPageIndex(addr))
                : page_map_.get(addr);
            if (sp) {
                // Smash-owned pointer slipped in before passthrough was active.
                if (sp->is_large) { large_alloc_.deallocate(sp); return; }
                ThreadCache* tc = getOrCreateThreadCache();
                if (!tc->deallocate(sp->size_class, sp->arena_id, ptr)) {
                    tc->drain(sp->size_class, sp->arena_id, slabs_, &page_map_);
                    tc->deallocate(sp->size_class, sp->arena_id, ptr);
                }
                return;
            }
            if (!g_system_alloc.free) g_system_alloc.resolve();
            if (g_system_alloc.free) g_system_alloc.free(ptr);
            return;
        }

        if (isCompressOnlyMode()) {
            if (g_system_alloc.free) g_system_alloc.free(ptr);
            return;
        }

        // Resolve the span via the flat page→Span table (single load), or the
        // two-level radix walk for addresses that escape the VmRegion. The
        // cold path does not consult or populate the TLS last-span cache —
        // that lives on the inlined fast path.
        const auto addr = reinterpret_cast<uintptr_t>(ptr);
        Span* span;
        if (vm_region_.inContigArena(addr) && vm_region_.hasSpanTable()) [[likely]] {
            span = vm_region_.getSpan(vm_region_.contigPageIndex(addr));
        } else {
            span = page_map_.get(addr);
        }
        if (!span) {
            // Not a Smash-managed pointer. Forward to the system allocator.
            // On Linux this matters in full mode too: alloc8's strong-alias
            // overrides bind malloc/free at link time, so any allocation the
            // dynamic linker / libstdc++ static initializers / glibc helpers
            // make BEFORE SmashHeap's singleton finishes constructing comes
            // back to us as a foreign pointer when freed. Silently dropping
            // it leaks the chunk and (worse) breaks realloc semantics —
            // realloc() falls back to malloc+memcpy(getSize()=0)+free(), so
            // dropping the old block leaves stale data live and corrupts
            // glibc's heap on the next free of the original.
            if (!g_system_alloc.free) g_system_alloc.resolve();
            // Diagnostic: SMASH_TRACE_FOREIGN_FREE=1 prints the first N
            // foreign-pointer frees and where they likely came from. Costs
            // nothing in steady state (one branch).
            static int trace_foreign = []{
                const char* v = std::getenv("SMASH_TRACE_FOREIGN_FREE");
                return (v && v[0] == '1') ? 1 : 0;
            }();
            if (trace_foreign) [[unlikely]] {
                static std::atomic<int> printed{0};
                int n = printed.fetch_add(1, std::memory_order_relaxed);
                if (n < 32) {
                    char buf[160];
                    Dl_info info{};
                    void* ra = __builtin_return_address(0);
                    const char* fname = "?";
                    if (ra && dladdr(ra, &info) && info.dli_fname) fname = info.dli_fname;
                    int len = smash::safe_snprintf(buf, sizeof(buf),
                        "[smash foreign-free] ptr=%p ra=%p in=%s "
                        "vm_lo=%p vm_hi=%p\n",
                        ptr, ra, fname,
                        reinterpret_cast<void*>(vm_region_.base()),
                        reinterpret_cast<void*>(vm_region_.base()
                            + vm_region_.contigPages() * kPageSize));
                    if (len > 0) (void)!::write(2, buf, (size_t)len);
                }
            }
            if (g_system_alloc.free) g_system_alloc.free(ptr);
            return;
        }
        if (span->is_large) { large_alloc_.deallocate(span); return; }
        uint8_t sc = span->size_class;
        uint8_t arena = span->arena_id;
        ThreadCache* tc = getOrCreateThreadCache();
        if (!tc->deallocate(sc, arena, ptr)) { tc->drain(sc, arena, slabs_, &page_map_); tc->deallocate(sc, arena, ptr); }
    }

    void* memalign(size_t alignment, size_t size) {
        // SMASH_PASSTHROUGH=1: pass aligned allocations to system allocator.
        if (isPassthroughMode()) {
            if (!g_system_alloc.posix_memalign) g_system_alloc.resolve();
            if (g_system_alloc.posix_memalign) {
                void* ptr = nullptr;
                if (g_system_alloc.posix_memalign(&ptr, alignment, size) == 0)
                    return ptr;
            }
            return nullptr;
        }

        if (isCompressOnlyMode()) {
            if (!g_system_alloc.posix_memalign) return nullptr;
            void* ptr = nullptr;
            if (g_system_alloc.posix_memalign(&ptr, alignment, size) == 0) {
                trackAllocation(ptr, size);
                return ptr;
            }
            return nullptr;
        }

        if (size == 0) size = 1;
        // Large-only: small aligned allocs go to system allocator
        if (isLargeOnlyMode() && size <= largeOnlyThreshold()) {
            // Lazy-resolve if needed
            if (!g_system_alloc.posix_memalign) g_system_alloc.resolve();
            if (g_system_alloc.posix_memalign) {
                void* ptr = nullptr;
                if (g_system_alloc.posix_memalign(&ptr, alignment, size) == 0)
                    return ptr;
            }
            // If system posix_memalign not available, fall through to smash's allocator
        }
        if (alignment <= kMinAlignment) return this->malloc(size);
        uintptr_t caller_ra = appCallerRA();
        uint8_t arena = callsiteArenaForLarge(caller_ra);
        return large_alloc_.allocate(size, alignment, arena);
    }

    void* calloc(size_t count, size_t size) {
        // SMASH_PASSTHROUGH=1: pass calloc to system allocator.
        if (isPassthroughMode()) {
            if (!g_system_alloc.calloc) g_system_alloc.resolve();
            if (g_system_alloc.calloc) return g_system_alloc.calloc(count, size);
            return nullptr;
        }
        if (isCompressOnlyMode()) {
            if (!g_system_alloc.calloc) return nullptr;
            void* ptr = g_system_alloc.calloc(count, size);
            trackAllocation(ptr, count * size);
            return ptr;
        }
        size_t total = count * size;
        // Large-only: small callocs go to system allocator
        if (isLargeOnlyMode() && total <= largeOnlyThreshold() && g_system_alloc.calloc) {
            return g_system_alloc.calloc(count, size);
        }
        void* ptr = this->malloc(total);
        if (ptr) __builtin_memset(ptr, 0, total);
        return ptr;
    }

    void* realloc(void* old_ptr, size_t size) {
        // SMASH_PASSTHROUGH=1: pass realloc to system allocator, except for
        // pointers smash itself allocated during early init.
        if (isPassthroughMode()) {
            if (old_ptr) {
                uintptr_t addr = reinterpret_cast<uintptr_t>(old_ptr);
                Span* sp = vm_region_.inContigArena(addr) && vm_region_.hasSpanTable()
                    ? vm_region_.getSpan(vm_region_.contigPageIndex(addr))
                    : page_map_.get(addr);
                if (sp) {
                    // Smash-owned: copy to a system-malloc buffer and free old.
                    if (size == 0) { this->free(old_ptr); return nullptr; }
                    if (!g_system_alloc.malloc) g_system_alloc.resolve();
                    if (!g_system_alloc.malloc) return nullptr;
                    void* new_ptr = g_system_alloc.malloc(size);
                    if (new_ptr) {
                        size_t old_size = sp->is_large ? sp->large_size
                                                       : classSize(sp->size_class);
                        __builtin_memcpy(new_ptr, old_ptr,
                                         old_size < size ? old_size : size);
                        this->free(old_ptr);
                    }
                    return new_ptr;
                }
            }
            if (!g_system_alloc.realloc) g_system_alloc.resolve();
            if (g_system_alloc.realloc) return g_system_alloc.realloc(old_ptr, size);
            return nullptr;
        }
        if (isCompressOnlyMode()) {
            if (!g_system_alloc.realloc) return nullptr;
            void* ptr = g_system_alloc.realloc(old_ptr, size);
            trackAllocation(ptr, size);
            return ptr;
        }
        // Full mode: if old_ptr is foreign, forward the whole call to system
        // realloc. We can't do the malloc+memcpy+free dance ourselves because
        // we don't know the old block's size — getSize() returns 0 for
        // foreign pointers and any answer we make up corrupts the data.
        if (!old_ptr) return this->malloc(size);
        if (size == 0) { this->free(old_ptr); return nullptr; }
        uintptr_t addr = reinterpret_cast<uintptr_t>(old_ptr);
        Span* sp = vm_region_.inContigArena(addr) && vm_region_.hasSpanTable()
            ? vm_region_.getSpan(vm_region_.contigPageIndex(addr))
            : page_map_.get(addr);
        if (!sp) {
            if (!g_system_alloc.realloc) g_system_alloc.resolve();
            if (g_system_alloc.realloc) return g_system_alloc.realloc(old_ptr, size);
            return nullptr;
        }
        size_t old_size = sp->is_large ? sp->large_size
                                       : classSize(sp->size_class);
        void* new_ptr = this->malloc(size);
        if (new_ptr) {
            __builtin_memcpy(new_ptr, old_ptr, old_size < size ? old_size : size);
            this->free(old_ptr);
        }
        return new_ptr;
    }

    size_t getSize(void* ptr) {
        if (!ptr) return 0;
        if (BootstrapAlloc::instance().owns(ptr)) return 0;

        // SMASH_PASSTHROUGH=1: ask system allocator for the size.
        if (isPassthroughMode()) {
            if (!g_system_alloc.malloc_size) g_system_alloc.resolve();
            if (g_system_alloc.malloc_size) return g_system_alloc.malloc_size(ptr);
            return 0;
        }

        if (isCompressOnlyMode()) {
            return 0;  // Can't determine size for system allocations
        }

        // Same flat-table fast path as free(): single load when the pointer
        // is inside the contiguous VmRegion arena, falling back to the
        // radix tree for non-VmRegion pointers (e.g. large-only mode's
        // system-malloc passthroughs).
        uintptr_t addr = reinterpret_cast<uintptr_t>(ptr);
        Span* span;
        if (vm_region_.inContigArena(addr) && vm_region_.hasSpanTable()) [[likely]] {
            span = vm_region_.getSpan(vm_region_.contigPageIndex(addr));
        } else {
            span = page_map_.get(addr);
        }
        if (!span) {
            // Query the system allocator for foreign pointers (allocations
            // made before SmashHeap was ready, or via paths we don't see).
            // Returning 0 here lies to alloc8's realloc fallback —
            // memcpy(new, old, 0) leaves the new buffer uninitialized and
            // free(old) silently drops it, the classic recipe for
            // "double free / munmap_chunk: invalid pointer" later.
            if (!g_system_alloc.malloc_size) g_system_alloc.resolve();
            if (g_system_alloc.malloc_size) return g_system_alloc.malloc_size(ptr);
            return 0;
        }
        if (span->is_large) return span->large_size;
        return classSize(span->size_class);
    }

    // Large blocks placed with direct mmap (below kLargeAllocVmThreshold, or
    // in large-only mode) live outside the contiguous arena and are only
    // registered in page_map_; xxowns() must recognise them.
    bool hasPageMapSpan(uintptr_t addr) const { return page_map_.get(addr) != nullptr; }

    // Diagnostic / test hook: the arena a smash-managed pointer was routed to,
    // or -1 for a non-smash pointer (foreign, or large-only passthrough).
    // Backs the exported smash_arena_for(); the arena-routing regression test
    // uses it to assert distinct call sites don't collapse into one arena
    // (the LTO/interposition return-address bug fixed in the RA-capture change).
    // Same span resolution as free()/getSize().
    int arenaFor(const void* ptr) {
        if (!ptr) return -1;
        if (BootstrapAlloc::instance().owns(ptr)) return -1;
        if (isCompressOnlyMode() || isPassthroughMode()) return -1;
        uintptr_t addr = reinterpret_cast<uintptr_t>(ptr);
        Span* span;
        if (vm_region_.inContigArena(addr) && vm_region_.hasSpanTable()) {
            span = vm_region_.getSpan(vm_region_.contigPageIndex(addr));
        } else {
            span = page_map_.get(addr);
        }
        if (!span) return -1;
        return static_cast<int>(span->arena_id);
    }

    // Whole-heap lock/unlock used by the pthread_atfork prepare/parent handlers
    // to quiesce every slab + the large-alloc lock across fork(). Clang Thread
    // Safety Analysis cannot reason about acquiring an *array* of mutexes in a
    // loop (runtime-indexed capabilities are not statically tractable — the
    // "locks in a loop" limitation in the clang docs), so these two functions
    // are exempted. They are exact mirrors (lock all ascending; unlock large
    // then all descending) and both short-circuit identically in compress-only
    // mode, so the pairing is balanced by construction.
    void lock() SMASH_NO_TS_ANALYSIS {
        if (isCompressOnlyMode()) return;
        const int total = getTotalArenas() * kNumClasses;
        for (int i = 0; i < total; ++i) slabs_[i].lockSlab();
        large_alloc_.lockAlloc();
    }
    void unlock() SMASH_NO_TS_ANALYSIS {
        if (isCompressOnlyMode()) return;
        large_alloc_.unlockAlloc();
        const int total = getTotalArenas() * kNumClasses;
        for (int i = total - 1; i >= 0; --i) slabs_[i].unlockSlab();
    }
    // SMASH_NO_THREAD_HOOKS=1: disable all thread lifecycle hooks.
    // Useful for debugging crashes that occur during thread exit.
    static bool noThreadHooks() {
        static const bool no_hooks = [] {
            const char* v = std::getenv("SMASH_NO_THREAD_HOOKS");
            return v && v[0] == '1';
        }();
        return no_hooks;
    }

    void threadInit() {
        if (noThreadHooks()) return;
        if (!isCompressOnlyMode()) {
            getOrCreateThreadCache();
        }
        // Start compression after the second thread init call on macOS.
        // The first call is the main thread during early DYLD_INSERT init
        // (before _objc_init). Subsequent calls happen after init is safe.
        // On Linux, LD_PRELOAD init is complete before threadInit is called,
        // so we can start immediately on the first call.
        if (compression_inited_ &&
            !compression_started_.load(std::memory_order_acquire)) {
#ifdef __APPLE__
            if (g_thread_init_count.fetch_add(1, std::memory_order_acq_rel) >= 1)
#else
            g_thread_init_count.fetch_add(1, std::memory_order_acq_rel);
#endif
            startCompression();
        }
    }
    void threadCleanup() {
        if (noThreadHooks()) return;
        if (isCompressOnlyMode()) return;
        ThreadCache*& tc = currentThreadCache();
        if (tc) { tc->drainAll(slabs_, &page_map_); returnThreadCache(tc); tc = nullptr; }
    }

    CohortPage* cohortPages() const { return cohort_pages_; }
    size_t cohortPagesLen() const { return cohort_pages_len_; }
};
} // namespace smash
