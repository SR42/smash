// smash/config.h - Compile-time tuning knobs
#pragma once

#include <alloc8/platform.h>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

namespace smash {

// ── Page size ────────────────────────────────────────────────────────────────
inline constexpr size_t kPageSize = ALLOC8_PAGE_SIZE;
inline constexpr int kPageShift = (kPageSize == 16384) ? 14 : 12;

// ── Size classes ─────────────────────────────────────────────────────────────
inline constexpr size_t kMaxSmallSize = 16384;
inline constexpr int kNumClasses = 36;
inline constexpr size_t kMinAlignment = 16;

// SMASH_ALIGN_SLAB: serve an aligned small allocation from the slab class
// whose size is a multiple of the alignment, instead of sending every
// alignment above kMinAlignment to LargeAlloc and a whole page. Default ON;
// set to 0 for the old behaviour. See alignedSizeClass().
// SMASH_ALIGN_SLOT_SEARCH: keep an aligned small allocation in its NATURAL
// size class and search that class's bitmap for a slot whose index is a
// multiple of alignment/gcd(class_size, alignment), instead of promoting the
// request to a class that is itself a multiple of the alignment. Off by
// default; read "Aligned small allocations" (agents/allocation-paths.md):
// measured over all 131072 (size, alignment) pairs, it never beats promotion
// when the workload allocates only aligned objects — the slots it steps over
// are stranded — and is up to 7x worse. It wins only when ordinary
// allocations of the same class fill those slots.
// SMASH_NO_DRAIN_WAIT=1: an allocation that finds no free run gives up
// immediately instead of waiting for runs that are freed but still queued for
// decommit. This is the pre-fix behaviour and exists so a test can measure the
// same workload both ways in one run — an absolute fallback rate is not
// portable (measured 34-125 of 320000 on an 8-CPU host against 1537 on a
// 128-CPU one), but the ratio between the two is.
inline bool drainWaitEnabled() {
    static const bool on = [] {
        const char* v = getenv("SMASH_NO_DRAIN_WAIT");
        return !(v && v[0] != '0');
    }();
    return on;
}

inline bool alignSlotSearchEnabled() {
    static const bool on = [] {
        const char* v = getenv("SMASH_ALIGN_SLOT_SEARCH");
        return v && v[0] != '0';
    }();
    return on;
}

inline bool alignSlabEnabled() {
    static const bool on = [] {
        const char* v = getenv("SMASH_ALIGN_SLAB");
        return !v || v[0] != '0';
    }();
    return on;
}

// ── Large-allocation pseudo size-classes (compression-tracking only) ─────────
// Slab pages hold many heterogeneous small objects, so per-bucket compression
// signal is noisy. Large allocations are one slice of one buffer from one
// call site, so the per-bucket signal is much cleaner and is therefore worth
// tracking separately. Buckets are bytewise floor(log2(num_pages)) on the
// span's page count, with the top bucket saturating; eight slots cover [1
// page, 2-3, 4-7, 8-15, 16-31, 32-63, 64-127, >=128 pages]. With
// kLargeAllocVmThreshold ~= 1 MiB, the meaningful slots are the last three;
// the lower five just stay empty for most workloads.
inline constexpr int kNumLargeClasses = 8;
inline constexpr int kTotalBucketsPerArena = kNumClasses + kNumLargeClasses;

// Floor of log2 for 32-bit unsigned.  Returns 0 for n==0 so callers don't
// need a special case (large-alloc spans always have page_count >= 1, but be
// defensive). __builtin_clz is undefined on 0.
[[gnu::always_inline]]
inline int log2FloorU32(uint32_t n) {
    if (n <= 1) return 0;
    return 31 - __builtin_clz(n);
}

// Map a large-allocation span's page count to a tracking bucket index in
// [kNumClasses, kNumClasses + kNumLargeClasses). Saturates at the last slot.
[[gnu::always_inline]]
inline uint8_t largeSizeClass(uint32_t num_pages) {
    int bucket = log2FloorU32(num_pages);
    if (bucket >= kNumLargeClasses) bucket = kNumLargeClasses - 1;
    return static_cast<uint8_t>(kNumClasses + bucket);
}

// ── Arenas ───────────────────────────────────────────────────────────────────
// kMaxArenas is the compile-time upper bound for arena arrays.
// Runtime arena count is determined by getNumArenas() based on CPU count.
#ifndef SMASH_MAX_ARENAS
inline constexpr int kMaxArenas = 128;  // max supported arenas (must be power of 2)
#else
inline constexpr int kMaxArenas = SMASH_MAX_ARENAS;
#endif

// For backward compatibility, kNumArenas is now an alias for getNumArenas()
// in most places. Code that needs the compile-time max should use kMaxArenas.
#ifndef SMASH_NUM_ARENAS
inline constexpr int kNumArenasDefault = 4;  // fallback if CPU detection fails
#else
inline constexpr int kNumArenasDefault = SMASH_NUM_ARENAS;
#endif

// Dynamic arena count based on CPU count using balls-to-bins analysis.
// With n threads and m arenas, max load per arena ~ n/m + sqrt(2n*ln(m)/m).
// For good load balancing, we want m ~ sqrt(n) * c where c ~ 2-4.
// We use m = max(4, min(kMaxArenas, roundUpPow2(sqrt(ncpu) * 4))).
[[gnu::always_inline]]
inline int getNumArenas() {
    static std::atomic<int> cached{-1};
    int v = cached.load(std::memory_order_relaxed);
    if (v < 0) [[unlikely]] {
        // Check for explicit override first
        const char* env = getenv("SMASH_NUM_ARENAS");
        if (env && *env) {
            int parsed = atoi(env);
            if (parsed > 0 && parsed <= kMaxArenas) {
                // Round up to power of 2
                int p = 1;
                while (p < parsed) p <<= 1;
                v = (p <= kMaxArenas) ? p : kMaxArenas;
                cached.store(v, std::memory_order_relaxed);
                return v;
            }
        }
        // Auto-scale based on CPU count
        long ncpu = sysconf(_SC_NPROCESSORS_ONLN);
        if (ncpu <= 0) ncpu = 4;
        // m = sqrt(ncpu) * 4, rounded up to power of 2, clamped to [4, kMaxArenas]
        // For ncpu=4: m=8, ncpu=16: m=16, ncpu=64: m=32, ncpu=192: m=64
        double target = 4.0;
        if (ncpu > 4) {
            // Use sqrt(ncpu) * 4 for larger CPU counts
            double sqrtn = 1.0;
            for (int i = 0; i < 20 && sqrtn * sqrtn < ncpu; ++i) sqrtn += 0.5;
            target = sqrtn * 4.0;
        }
        int m = 4;
        while (m < target && m < kMaxArenas) m <<= 1;
        v = m;
        cached.store(v, std::memory_order_relaxed);
    }
    return v;
}

// Arena mask for fast modulo (arenas must be power of 2)
[[gnu::always_inline]]
inline int getArenaMask() {
    return getNumArenas() - 1;
}

// Legacy alias - compile-time arrays still need a constexpr size
inline constexpr int kNumArenas = kMaxArenas;

// ── Reference-behavior homogeneity knobs (Apr 2026 design memo) ─────────────
//
// A3: Cold-bias feedback arenas.  When on, the slab array is doubled to
// maintain a hot sub-arena (index 0..kNumArenas-1) and a cold sub-arena
// (index kNumArenas..2*kNumArenas-1) for each (callsite, size class).
// A per-(base-arena, size-class) flag flips to cold once the compressor
// records >= kColdArenaThreshold successful compressions for that bucket;
// subsequent allocations route to the cold sub-arena, where they may be
// placed under an underfill policy (C1).
#ifdef SMASH_COLD_ARENA_FEEDBACK
inline constexpr bool kColdArenaFeedback = true;
#else
inline constexpr bool kColdArenaFeedback = false;
#endif
#ifndef SMASH_COLD_ARENA_THRESHOLD
inline constexpr uint32_t kColdArenaThreshold = 8;
#else
inline constexpr uint32_t kColdArenaThreshold = SMASH_COLD_ARENA_THRESHOLD;
#endif
// Compile-time max for array sizing (uses kMaxArenas, not runtime count)
inline constexpr int kTotalArenas = kColdArenaFeedback ? (kMaxArenas * 2) : kMaxArenas;

// Runtime total arena count (hot + cold if feedback enabled)
[[gnu::always_inline]]
inline int getTotalArenas() {
    return kColdArenaFeedback ? (getNumArenas() * 2) : getNumArenas();
}

// C1: Per-page absolute cap on live objects.  The (1-q)^N argument for
// page-cold probability bounds N (objects per page), not the fraction of
// the span used.  A fractional denominator is the wrong knob: with
// denom=4, a 16B size class still puts 256 objects on a page (P(cold)
// ≈ 0 even for tiny q), while a 4KB size class drops to 1 object/page
// (huge underfill cost for the same probability bound).
//
// kMaxSlotsPerPage caps the number of slots whose start address lies on
// any single page.  Spans for cold sub-arenas (A3 on) or all spans (A3
// off) reserve only the first kMaxSlotsPerPage slots per page; the
// remaining bytes stay zero-filled and compress to near-nothing.
// 0 = no cap (default).  Cap is silently ignored when object_size >=
// kPageSize (slot already covers a full page).
#ifndef SMASH_MAX_SLOTS_PER_PAGE
inline constexpr uint32_t kMaxSlotsPerPage = 0;
#else
inline constexpr uint32_t kMaxSlotsPerPage = SMASH_MAX_SLOTS_PER_PAGE;
#endif

// C1b: Adaptive per-page cap driven by compression/decompression feedback.
// When on, the heap tracks per-(base_arena, size_class) counters for both
// successful compression events (cold evidence) and fault decompression
// events (hot evidence, i.e., the page was re-warmed after being cold).
// q̂ = decomp / (comp + decomp) is an online Pareto estimator; N is chosen
// so (1 - q̂)^N >= kAdaptiveCapTargetPct / 100 (i.e., target probability
// that a freshly allocated page stays uniformly cold for one tick window).
// Below kAdaptiveCapMinSamples total events the cap is disabled
// (insufficient evidence).  For hot-dominated buckets (q̂ too high to
// satisfy the target at any N >= kAdaptiveCapMin), the cap is also
// disabled — under-packing a hot bucket just balloons RSS.
#ifdef SMASH_ADAPTIVE_CAP
inline constexpr bool kAdaptiveCap = true;
#else
inline constexpr bool kAdaptiveCap = false;
#endif
#ifndef SMASH_ADAPTIVE_CAP_TARGET_PCT
inline constexpr int kAdaptiveCapTargetPct = 50;   // P(page cold) target
#else
inline constexpr int kAdaptiveCapTargetPct = SMASH_ADAPTIVE_CAP_TARGET_PCT;
#endif
#ifndef SMASH_ADAPTIVE_CAP_MIN
inline constexpr uint32_t kAdaptiveCapMin = 4;     // floor on N
#else
inline constexpr uint32_t kAdaptiveCapMin = SMASH_ADAPTIVE_CAP_MIN;
#endif
#ifndef SMASH_ADAPTIVE_CAP_MIN_SAMPLES
inline constexpr uint32_t kAdaptiveCapMinSamples = 16;
#else
inline constexpr uint32_t kAdaptiveCapMinSamples = SMASH_ADAPTIVE_CAP_MIN_SAMPLES;
#endif

// A2-lite: Thread identity in the arena hash.  When on, callsiteArena()
// XORs a per-thread id into the (RA, frame, sc) hash so allocations from
// different threads are more likely to land in different arenas.  Cheap —
// uses a TLS-cached monotonic id assigned on first call.  Effectiveness
// is bounded by kNumArenas: with 4 arenas, ~4 threads saturate the space
// and the extra term is noise beyond that; consider raising kNumArenas
// to 8 or 16 for heavier multi-threaded workloads.
// Default ON: TBB-heavy workloads (walrus has many parallel workers)
// benefit from per-thread arena routing to spread slab-lock contention.
// Negligible cost when off (one TLS lookup + xor per allocation).
#if defined(SMASH_THREAD_ARENA_HASH) && SMASH_THREAD_ARENA_HASH == 0
inline constexpr bool kThreadArenaHash = false;
#else
inline constexpr bool kThreadArenaHash = true;
#endif

// B1: Page-local batch refill.  When on, Slab::allocateBatch stops once
// the next object would fall on a different page than the batch's first
// object.  Keeps each thread-cache refill confined to one page, so
// objects allocated in the same burst share a lifecycle at page grain.
#ifdef SMASH_PAGE_LOCAL_BATCH
inline constexpr bool kPageLocalBatch = true;
#else
inline constexpr bool kPageLocalBatch = false;
#endif

// Cohort measurement: track per-page thread/call-site mixing and print
// a summary each compressor tick.  Measurement only — no behavioral change.
#ifdef SMASH_MEASURE_COHORTS
inline constexpr bool kMeasureCohorts = true;
#else
inline constexpr bool kMeasureCohorts = false;
#endif

// ── Spans ────────────────────────────────────────────────────────────────────
inline constexpr int kTargetObjectsPerSpan = 64;
inline constexpr int kMaxSpanPages = 8;

// ── Bootstrap allocator ──────────────────────────────────────────────────────
inline constexpr size_t kBootstrapInitialSize = 64 * 1024 * 1024;   // 64 MB
inline constexpr size_t kBootstrapExpandSize = 16 * 1024 * 1024;    // 16 MB
inline constexpr int kBootstrapMaxRegions = 64;
// Regions grow geometrically from kBootstrapExpandSize to this cap, so the
// metadata arena scales with the heap: 64 regions reach ~15 GiB instead of
// the 1.07 GiB that fixed 16 MiB expansions allowed (which a ~169 GB heap
// exhausted, then died dereferencing null).
inline constexpr size_t kBootstrapMaxExpandSize = 256 * 1024 * 1024;

// ── Thread cache ─────────────────────────────────────────────────────────────
// Doubled to 256/128 to halve slab-lock acquisitions on TBB-heavy
// workloads (walrus has 8-12 worker threads compiling in parallel).
// Per-thread memory cost: 64 size classes × 256 ptrs × 8 B = 128 KiB
// per thread. With ≤16 TBB threads that's 2 MiB total — negligible
// vs the 1+ GiB heap. Cool-tail RSS ratio at 99.3 % is the gate.
#ifndef SMASH_THREAD_CACHE_MAX
inline constexpr int kThreadCacheMaxPerClass = 256;
#else
inline constexpr int kThreadCacheMaxPerClass = SMASH_THREAD_CACHE_MAX;
#endif
#ifndef SMASH_THREAD_CACHE_BATCH
inline constexpr int kThreadCacheBatchSize = 128;
#else
inline constexpr int kThreadCacheBatchSize = SMASH_THREAD_CACHE_BATCH;
#endif

// ── Page map ─────────────────────────────────────────────────────────────────
// 48-bit virtual address space assumed
inline constexpr int kAddressBits = 48;
inline constexpr int kPageMapL2Bits = 16;
inline constexpr size_t kPageMapL2Size = 1ULL << kPageMapL2Bits;
inline constexpr int kPageMapL1Bits = kAddressBits - kPageShift - kPageMapL2Bits;
inline constexpr size_t kPageMapL1Size = 1ULL << kPageMapL1Bits;

// ── ROI model defaults (overridable via SMASH_ROI_THRESHOLD env var) ─────────
inline constexpr int kRoiThresholdDefault = 1024;

// ── Compression (Phase 3+) ───────────────────────────────────────────────────
inline constexpr int kCompressIntervalMs = 1000;
// Default 10 ticks (= 10 s at the 1 s tick interval). Lower values compress
// more aggressively but starve continuous workloads of read-only working set
// — PROT_READ-monitoring is blind to reads, so a page being constantly read
// looks identical to an idle one. Span::allocate() resets cold_count on
// alloc activity, but the program may go from "burst-allocating" to
// "read-only execution" with no further allocs, and a 2-tick floor will
// then catch the read-only working set in 2 s flat. 10 ticks gives the
// workload time to establish access patterns through the rc-backoff
// mechanism (one fault → eff_floor doubles) before the wave of fresh
// pages becomes eligible.
//
// Tests pin their own value via SMASH_COLD_TIMEOUT_SEC / SMASH_COLD_TICKS,
// so unit-test latency is unaffected.
#ifndef SMASH_COLD_TICKS
inline constexpr int kColdTicksDefault = 10;
#else
inline constexpr int kColdTicksDefault = SMASH_COLD_TICKS;
#endif
inline constexpr double kMinCompressRatio = 0.75;

// Runtime cold timeout: SMASH_COLD_TIMEOUT_SEC overrides kColdTicks.
// This is the primary time-space tradeoff dial.  Lower values compress
// sooner (more space savings, more decompression faults on re-access).
// Higher values are conservative (less savings, fewer faults).
// Default: kColdTicksDefault * kCompressIntervalMs / 1000.
inline int getColdTicks() {
    static int ticks = []() {
        const char* env = getenv("SMASH_COLD_TIMEOUT_SEC");
        if (env) {
            double sec = atof(env);
            if (sec > 0)
                return static_cast<int>(sec * 1000.0 / kCompressIntervalMs + 0.5);
        }
        return kColdTicksDefault;
    }();
    return ticks;
}

// ── Cooling-page closing ─────────────────────────────────────────────────────
// A free slot whose page has already been idle for this many compressor ticks
// is demoted in Span::allocate()'s preference order: warm-page slots first,
// then cooling-page slots, then COMPRESSED-page slots. Without this, steady
// allocation churn backfills partial pages that hold long-lived cold survivors
// and resets their cold counters, so such a page keeps getting re-heated and
// never reaches compression (its freed slots would otherwise be zeroed and
// compressed out). Preference-only: never forces a new span, so it cannot
// grow the page count — it steers churn toward pages that are warm anyway and
// lets cooling pages finish cooling.
// SMASH_COOLING_CLOSE_TICKS=N overrides; 0 disables. Default: half the cold
// threshold — a page halfway to compression eligibility stops receiving
// backfill.
//
// coolingCloseDefaultTicks() is also the fixed threshold for the reheat=
// telemetry (g_reheat_events): re-heats are counted against the default even
// when closing is disabled, so an A/B of SMASH_COOLING_CLOSE_TICKS=0 vs
// default shows how many re-heats the preference actually prevented.
inline int coolingCloseDefaultTicks() {
    static int ticks = []() {
        int half = getColdTicks() / 2;
        return half < 1 ? 1 : half;
    }();
    return ticks;
}

inline int getCoolingCloseTicks() {
    static int ticks = []() {
        const char* env = getenv("SMASH_COOLING_CLOSE_TICKS");
        if (env) {
            int v = atoi(env);
            return v < 0 ? 0 : v;
        }
        return coolingCloseDefaultTicks();
    }();
    return ticks;
}

// ── Adaptive compression (Phase 5+) ─────────────────────────────────────────
#ifndef SMASH_VERY_COLD_TICKS
inline constexpr int kVeryColdTicks = 60;         // ~1 min → zstd deep
#else
inline constexpr int kVeryColdTicks = SMASH_VERY_COLD_TICKS;
#endif

// ── Time-budget knob (additive over legacy cold-tick gating) ────────────────
//
// SMASH_TIME_BUDGET_PCT=N (0..100). When set, the compressor recomputes a
// marginal-efficiency threshold every N ticks and marks low-efficiency
// (arena, size_class) buckets SKIP so phase 2 short-circuits before any
// locking. Unset / 100 = unlimited (legacy behavior). 0 = no compression
// after exploration completes.
//
// This knob is the future replacement for SMASH_COLD_TIMEOUT_SEC and friends;
// for now it layers on top of them — buckets must satisfy the legacy
// cold-tick floor AND not be SKIP-marked to be compressed. Cached once at
// first read.
//
// Returns -1 when the env var is unset (caller should treat as legacy
// behavior — no SKIP gating). Otherwise clamped to [0, 100].
inline int getTimeBudgetPct() {
    static std::atomic<int> cached{-2};
    int v = cached.load(std::memory_order_relaxed);
    if (v == -2) [[unlikely]] {
        const char* env = getenv("SMASH_TIME_BUDGET_PCT");
        int parsed = -1;
        if (env && *env) {
            char* end = nullptr;
            long n = strtol(env, &end, 10);
            if (end != env) {
                if (n < 0) n = 0;
                if (n > 100) n = 100;
                parsed = static_cast<int>(n);
            }
        }
        cached.store(parsed, std::memory_order_relaxed);
        v = parsed;
    }
    return v;
}

// ── Recompression-thrash back-off ──────────────────────────────────────────
//
// When a page is compressed and then immediately faulted back (compress →
// decompress → recompress loop), per-page `recompress_count_` is bumped on
// each fault. The compressor's phase 2 gate raises the effective cold-tick
// floor by `floor << min(recompress_count + bucket_bias, kMaxBackoffShift)`,
// so a thrashy page must stay idle proportionally longer before being
// eligible for compression again. The penalty decays once the page truly
// cools off — see compressor_thread.h::phase1Range.
#ifndef SMASH_RECOMPRESS_MAX_SHIFT
inline constexpr int kMaxBackoffShift = 6;        // 64× floor max
#else
inline constexpr int kMaxBackoffShift = SMASH_RECOMPRESS_MAX_SHIFT;
#endif

#ifndef SMASH_RECOMPRESS_MAX_FLOOR_TICKS
inline constexpr int kMaxEffectiveFloorTicks = 200;  // sanity cap (~3.3 min)
#else
inline constexpr int kMaxEffectiveFloorTicks = SMASH_RECOMPRESS_MAX_FLOOR_TICKS;
#endif

#ifndef SMASH_RECOMPRESS_DECAY_TICKS
inline constexpr int kRcDecayColdTicks = 8;       // decay edge spacing in cold-ticks
#else
inline constexpr int kRcDecayColdTicks = SMASH_RECOMPRESS_DECAY_TICKS;
#endif

// EMA divisor (×256 fixed-point) for proportional per-bucket bias on the
// back-off shift. `bucket_bias = bucket_ema_x256 / kBucketRcBiasThreshold_x256`
// (capped at kMaxBackoffShift). 256 = "every full unit of average rc in this
// bucket adds +1 to the shift", so a single recompress event in a bucket
// immediately gives every other page in that bucket bias=1 (eff_floor=2×).
// This is load-bearing: without aggressive propagation, fresh pages from a
// thrashy call site each have to individually thrash before any backoff
// applies, which doesn't converge on workloads that allocate hundreds of
// pages per second from the same site.
#ifndef SMASH_RECOMPRESS_BUCKET_BIAS_X256
inline constexpr int kBucketRcBiasThreshold_x256 = 256;
#else
inline constexpr int kBucketRcBiasThreshold_x256 = SMASH_RECOMPRESS_BUCKET_BIAS_X256;
#endif

// Soft-dirty ROI (SMASH_SOFTDIRTY_ROI): number of consecutive WRITE-clean ticks
// after which a page earns FULL relief from its bucket's re-dirty prior — i.e.
// its own demonstrated write-stability fully overrides the bucket average. At
// the 1s tick this is "write-untouched for this many seconds ⇒ trust the page,
// not the bucket." Tunable via SMASH_WRITE_CLEAN_FULL_RELIEF.
#ifndef SMASH_WRITE_CLEAN_FULL_RELIEF
inline constexpr uint32_t kWriteCleanFullRelief = 16;
#else
inline constexpr uint32_t kWriteCleanFullRelief = SMASH_WRITE_CLEAN_FULL_RELIEF;
#endif

// Compile-time default for dictionary training samples per size class.
// Runtime override is SMASH_DICT_TRAIN_SAMPLES (see getDictTrainSamples()
// below), which is the supported way to flip dict training on/off without
// rebuilding. The constexpr stays available for the test_dictionary unit
// test which selects sample sizes at compile time.
#ifndef SMASH_DICT_TRAIN_SAMPLES
inline constexpr int kDictTrainSamples = 0;       // disabled: dicts net-negative (see EXPERIMENTS.md)
#else
inline constexpr int kDictTrainSamples = SMASH_DICT_TRAIN_SAMPLES;
#endif

// Maximum supported dict-train sample buffer per size class. The runtime
// SMASH_DICT_TRAIN_SAMPLES is clamped to this so a single misconfigured
// process can't burn unbounded bootstrap memory (kMaxDictClasses *
// kDictTrainSamplesMax * kPageSize ~ 8 * 256 * 16K = 32 MiB worst case).
inline constexpr int kDictTrainSamplesMax = 256;

// Runtime override for dict-train sample count. Returns 0 when unset OR
// when kDictTrainSamples is compiled to 0 and the env var is unset (cold
// default). When SMASH_DICT_TRAIN_SAMPLES is set, returns clamped value
// in [1, kDictTrainSamplesMax]. Cached on first call.
inline int getDictTrainSamples() {
    static std::atomic<int> cached{-1};
    int v = cached.load(std::memory_order_relaxed);
    if (v < 0) [[unlikely]] {
        const char* env = getenv("SMASH_DICT_TRAIN_SAMPLES");
        int parsed = kDictTrainSamples;
        if (env && *env) {
            char* end = nullptr;
            long n = strtol(env, &end, 10);
            if (end != env) {
                if (n < 0) n = 0;
                if (n > kDictTrainSamplesMax) n = kDictTrainSamplesMax;
                parsed = static_cast<int>(n);
            }
        }
        cached.store(parsed, std::memory_order_relaxed);
        v = parsed;
    }
    return v;
}

// SMASH_PROFILE_FILE_SAVE=1: at process exit, write the merged profile +
// dictionary table to SMASH_PROFILE_FILE. Default off so warm-mode runs
// don't accidentally overwrite the trained file with their (noisier)
// short-run observations.
inline bool getProfileFileSave() {
    static std::atomic<int> cached{-1};
    int v = cached.load(std::memory_order_relaxed);
    if (v < 0) [[unlikely]] {
        const char* env = getenv("SMASH_PROFILE_FILE_SAVE");
        v = (env && env[0] == '1') ? 1 : 0;
        cached.store(v, std::memory_order_relaxed);
    }
    return v == 1;
}

// SMASH_PROFILE_FILE_RW=1: load at start AND save at exit, equivalent to
// setting SMASH_PROFILE_FILE_SAVE=1 alongside an existing profile file.
// Convenience knob for "training" runs that should also re-absorb their
// own previous output.
inline bool getProfileFileRW() {
    static std::atomic<int> cached{-1};
    int v = cached.load(std::memory_order_relaxed);
    if (v < 0) [[unlikely]] {
        const char* env = getenv("SMASH_PROFILE_FILE_RW");
        v = (env && env[0] == '1') ? 1 : 0;
        cached.store(v, std::memory_order_relaxed);
    }
    return v == 1;
}
#ifndef SMASH_PREFETCH_WINDOW
inline constexpr int kPrefetchWindow = 2;         // pages each direction on fault
#else
inline constexpr int kPrefetchWindow = SMASH_PREFETCH_WINDOW;
#endif
inline constexpr int kZstdFastLevel = 1;            // fast tier (replaces LZ4)
inline constexpr int kZstdNormalLevel = 3;
inline constexpr int kZstdDeepLevel = 9;

// Deep tier (zstd-9), compile-time gated. DISABLED by default: measurements
// across the whole benchmark suite (sqlite/rocksdb/leveldb/redis/memcached/
// bench_rss) show the deep tier is never ROI-selected and never wins a
// recompression upgrade, and the fast->deep ratio-gate escalation (which does
// fire, e.g. ~32% of rocksdb pages) yields no RSS benefit — zstd-9's larger
// match window needs input >4KB to help, but smash compresses per-page, so
// z9/z1 blob size is ~1.0 on real heap pages. libsmash therefore ships as
// single-tier zstd-1. The deep-tier code is retained (not deleted) behind this
// flag so two-tier can be restored for future work (e.g. multi-page
// compression, where zstd-9's window could finally pay). Build with
// -DSMASH_ENABLE_DEEP_TIER to re-enable the second tier.
#ifdef SMASH_ENABLE_DEEP_TIER
inline constexpr bool kDeepTierEnabled = true;
#else
inline constexpr bool kDeepTierEnabled = false;
#endif

// Tiered recompression (in development): when true, the compressor scans
// COMPRESSED pages and upgrades fast-tier blobs (zstd-1 by default, LZ4
// when SMASH_USE_LZ4) to deep-tier (zstd-9) once a page has stayed cold
// past kVeryColdTicks.  Without this flag, the initial algorithm chosen
// at compress time is final until the page is decompressed.
//
// Does NOT change the initial tier — see SMASH_USE_LZ4 for that.
// See TIERED_RECOMPRESSION.md for design and measurements.
#ifndef SMASH_NO_TIERED_RECOMPRESSION
inline constexpr bool kTieredRecompression = true;
#else
inline constexpr bool kTieredRecompression = false;
#endif

// When true, use LZ4 as the fast compression tier (opt-in via SMASH_USE_LZ4).
// When false (default), use zstd-1 as the fast tier for better ratios.
//
// We measured LZ4 initial + zstd-9 upgrade on memcached and Redis: peak
// fill_rss is unchanged vs zstd-1 (the bottleneck isn't the algorithm),
// and cool_rss costs ~9-33 MB vs zstd-1 due to whole-region waste from
// "stranger" blobs that prevent draining.  Keep as opt-in; the
// CompressStore freelist (this branch) makes the cost bounded when
// users do opt in.
#ifdef SMASH_USE_LZ4
inline constexpr bool kUseLz4FastTier = true;
#else
inline constexpr bool kUseLz4FastTier = false;
#endif
inline constexpr int kDictLevel = 3;            // CDict built at level 3 (430KB vs 686KB at level 9)
inline constexpr size_t kDictCapacity = 16 * 1024; // 16KB dict (down from 32KB)
inline constexpr int kMaxDictClasses = 8;        // cap total dict memory overhead

// ── Compressor parallelism ───────────────────────────────────────────────────
#ifndef SMASH_COMPRESSOR_WORKERS
inline constexpr int kCompressorWorkers = 2;         // initial compression workers
#else
inline constexpr int kCompressorWorkers = SMASH_COMPRESSOR_WORKERS;
#endif
#ifndef SMASH_MAX_COMPRESSOR_WORKERS
inline constexpr int kMaxCompressorWorkers = 8;      // max workers (pre-allocated pool)
#else
inline constexpr int kMaxCompressorWorkers = SMASH_MAX_COMPRESSOR_WORKERS;
#endif
#ifndef SMASH_COMPRESS_STORE_SHARDS
inline constexpr int kCompressStoreShards = 8;       // CompressStore lock shards
#else
inline constexpr int kCompressStoreShards = SMASH_COMPRESS_STORE_SHARDS;
#endif
inline constexpr int kChunkBits = 6;                 // 64 pages per chunk
inline constexpr int kChunkSize = 1 << kChunkBits;   // pages per chunk

// ── Zero-on-free ────────────────────────────────────────────────────────────
// All zeroing is deferred to the compressor thread (zeroFreeSlots).
// No zeroing occurs in the free() critical path.

// ── Large allocation compression ────────────────────────────────────────────
// Only large allocations >= this size are placed in the VmRegion for
// compression tracking.  Smaller "large" allocs (16KB–256KB) are typically
// internal engine buffers accessed frequently; compressing them causes
// decompression storms when they're needed again.
#ifndef SMASH_LARGE_ALLOC_VM_THRESHOLD
inline constexpr size_t kLargeAllocVmThreshold = 1024 * 1024;  // 1 MB
#else
inline constexpr size_t kLargeAllocVmThreshold = SMASH_LARGE_ALLOC_VM_THRESHOLD;
#endif

// SMASH_MID_BINS=1: bin allocations in the (16 KiB, 1 MiB) band by size and
// serve them from chunks carved out of the compressible arena, instead of one
// direct mmap each. Off by default — it trades ~10% of the requested bytes to
// size-class rounding for compressibility and much cheaper churn, which is
// only a good trade for blocks that go cold. See src/core/mid_alloc.h.
inline bool midBinsEnabled() {
    static std::atomic<int> cached{-1};
    int v = cached.load(std::memory_order_relaxed);
    if (v < 0) [[unlikely]] {
        const char* env = getenv("SMASH_MID_BINS");
        v = (env && env[0] == '1') ? 1 : 0;
        cached.store(v, std::memory_order_relaxed);
    }
    return v == 1;
}

// Runtime override: SMASH_LARGE_ALLOC_VM_THRESHOLD, defaulting to the constant
// above. This decides whether an allocation is compressible at all, and the
// effect is absolute — a block below it goes to a direct mapping the
// compressor never walks, so it stays 100% resident for the life of the
// process no matter how cold it gets. Measured with the default: 320 KB and
// 768 KB blocks sit at 100% resident after a cold wait, 1 MiB blocks at 0%.
//
// The default is deliberately high (see the comment above): blocks in the
// 16 KiB–1 MiB band are often hot engine buffers, and compressing those trades
// RSS for decompression storms. Lower it when a workload's footprint is
// dominated by mid-sized blocks that really do go cold — and measure `reheat=`
// on the SIGUSR2 stats line afterwards, because that is where a storm shows up.
//
// The value is rounded up to a page and floored at one page. Only consulted
// from LargeAlloc::allocate (never the small-object fast path), and cached
// after the first call.
inline size_t largeAllocVmThreshold() {
    static std::atomic<size_t> cached{0};   // 0 = unset
    size_t v = cached.load(std::memory_order_relaxed);
    if (v == 0) [[unlikely]] {
        size_t result = kLargeAllocVmThreshold;
        const char* env = getenv("SMASH_LARGE_ALLOC_VM_THRESHOLD");
        if (env && *env) {
            long long parsed = atoll(env);
            if (parsed > 0) result = static_cast<size_t>(parsed);
        }
        // Round up to a whole page; a sub-page threshold would compare against
        // a page-rounded allocation size and never mean what it says.
        result = (result + kPageSize - 1) & ~(kPageSize - 1);
        if (result < kPageSize) result = kPageSize;
        cached.store(result, std::memory_order_relaxed);
        return result;
    }
    return v;
}

// ── Virtual memory region ────────────────────────────────────────────────────
// Compile-time max. The runtime size is read from SMASH_VM_GIB at startup
// and clamped to [256 MiB, kVmMaxPages*kPageSize]. We bump the compile-time
// max to 64 GiB so workloads with large working sets (neuron-cc compiles
// of multi-GB HLOs) don't exhaust the region and trigger malloc=NULL →
// std::bad_alloc / LLVM "out of memory" errors.
//
// Memory cost per page in metadata (PageStateTable + PageLockTable +
// page_to_span lookup): ~17 bytes. At 16M pages = 64 GiB region, that's
// ~270 MiB of bootstrap/lazy-mapped metadata — negligible vs the heap
// it tracks. The mmap reservation itself is virtual-only (MAP_NORESERVE),
// so the actual RSS cost is committed-pages × kPageSize.
// Ceiling for SMASH_VM_GIB: 1 TiB. It only bounds a request — the arena is a
// PROT_NONE MAP_NORESERVE reservation, and the per-page metadata sized from it
// is demand-paged, so an unused arena costs address space, not memory
// (measured: 16 vs 64 GiB arenas, identical 23.3 MB RSS). The previous 64 GiB
// ceiling was an estimate of metadata cost, not a limit. Page indices are
// 32-bit, so the hard bound is far higher (16 TiB). VmRegion::init halves the
// request if the reservation fails (39-bit arm64 VA, RLIMIT_AS).
inline constexpr size_t kVmMaxBytes = size_t{1} << 40;  // 1 TiB
inline constexpr size_t kVmMaxPages = kVmMaxBytes / kPageSize;
inline constexpr size_t kVmRegionSize = kVmMaxPages * kPageSize;

// Runtime override: SMASH_VM_GIB=N, default 16. Clamped to [0.25, kVmMaxPages*kPageSize/GiB].
inline size_t getVmRegionSize() {
    static std::atomic<size_t> cached{0};
    size_t v = cached.load(std::memory_order_relaxed);
    if (v != 0) return v;
    constexpr size_t kGiB = 1ULL << 30;
    size_t bytes = 16 * kGiB;
    const char* env = getenv("SMASH_VM_GIB");
    if (env) {
        char* end = nullptr;
        long long g = strtoll(env, &end, 10);
        if (end != env && g > 0) bytes = static_cast<size_t>(g) * kGiB;
    }
    if (bytes < (kGiB / 4)) bytes = kGiB / 4;        // 256 MiB floor
    if (bytes > kVmRegionSize) bytes = kVmRegionSize;
    cached.store(bytes, std::memory_order_relaxed);
    return bytes;
}

// ── Runtime mode detection ───────────────────────────────────────────────────
// Set SMASH_MODE=compress_only to enable compress-only mode at runtime.
// In compress-only mode, malloc is forwarded to the system allocator and
// pages are tracked for compression. This works with any allocator (jemalloc,
// tcmalloc, etc.) but doesn't benefit from Smash's allocation optimizations.

enum class SmashMode { Full, CompressOnly };

// Mode helpers used to wrap their cached values in `static T cached = []()...();`,
// which triggers the C++ magic-static guard: an acquire-load + branch on every
// call.  perf-record on a smash-LD_PRELOADed micro-bench showed those guards
// taking ~7-10 % of free()'s CPU.  Switch to a hand-rolled lazy init backed by
// std::atomic<int>: the post-init fast path is a single relaxed `ldr`, no
// acquire fence, no guard check.  -1 = uninitialised, all other values are
// the cached state.
inline SmashMode getSmashMode() {
    static std::atomic<int> cached{-1};
    int v = cached.load(std::memory_order_relaxed);
    if (v < 0) [[unlikely]] {
        const char* env = getenv("SMASH_MODE");
        v = (env && strcmp(env, "compress_only") == 0)
            ? static_cast<int>(SmashMode::CompressOnly)
            : static_cast<int>(SmashMode::Full);
        cached.store(v, std::memory_order_relaxed);
    }
    return static_cast<SmashMode>(v);
}

[[gnu::always_inline, gnu::hot]]
inline bool isCompressOnlyMode() {
#ifdef SMASH_COMPRESS_ONLY
    return true;
#else
    return getSmashMode() == SmashMode::CompressOnly;
#endif
}

// ── Large-only mode ─────────────────────────────────────────────────────────
// Set SMASH_LARGE_ONLY=1 to only manage large allocations through Smash.
// Small allocations (size <= kMaxSmallSize) pass through to the system
// allocator.  This avoids interfering with language runtimes that use their
// own small-object allocator (e.g. Python 3.13+ mimalloc).
[[gnu::always_inline, gnu::hot]]
inline bool isLargeOnlyMode() {
    static std::atomic<int> cached{-1};
    int v = cached.load(std::memory_order_relaxed);
    if (v < 0) [[unlikely]] {
        const char* env = getenv("SMASH_LARGE_ONLY");
        v = (env && env[0] == '1') ? 1 : 0;
        cached.store(v, std::memory_order_relaxed);
    }
    return v == 1;
}

// Threshold for what counts as "small" (passthrough to system) in
// LARGE_ONLY mode. Defaults to kMaxSmallSize (16 KB). Lower values
// route a larger fraction of allocations through smash — useful when
// the workload's byte-volume is dominated by mid-sized allocations
// (e.g. Firefox's 4 KB–7 KB bucket carries ~20 % of allocation volume
// while sitting under the default threshold). Must not exceed
// kMaxSmallSize since smash's slab path tops out there.
[[gnu::always_inline, gnu::hot]]
inline size_t largeOnlyThreshold() {
    static std::atomic<size_t> cached{0};   // 0 = unset
    size_t v = cached.load(std::memory_order_relaxed);
    if (v == 0) [[unlikely]] {
        const char* env = getenv("SMASH_LARGE_ONLY_THRESHOLD");
        size_t result = kMaxSmallSize;
        if (env && *env) {
            size_t parsed = static_cast<size_t>(atoll(env));
            if (parsed > 0 && parsed <= kMaxSmallSize) result = parsed;
        }
        cached.store(result, std::memory_order_relaxed);
        return result;
    }
    return v;
}

// ── Deferred-reclaim mode ───────────────────────────────────────────────────
// Set SMASH_DEFERRED_RECLAIM=1 to split compression into two phases:
//   Phase A: compress a copy, keep page accessible (PROT_RW) → COMPRESSED_SHADOW
//   Phase B: after N ticks of confirmed idleness, reclaim physical memory → COMPRESSED
// This makes Smash compatible with systems that have uncoordinated background
// threads (e.g. WiredTiger's eviction threads) that would crash on PROT_NONE.
[[gnu::always_inline, gnu::hot]]
inline bool isDeferredReclaimMode() {
    static std::atomic<int> cached{-1};
    int v = cached.load(std::memory_order_relaxed);
    if (v < 0) [[unlikely]] {
        const char* env = getenv("SMASH_DEFERRED_RECLAIM");
        // Default ON: the non-deferred path holds the page lock during
        // mprotect(PROT_READ) which deadlocks with concurrent app faults.
        // Deferred mode avoids mprotect during compression (memcpy at PROT_RW),
        // deferring PROT_NONE to the sweeper (which uses tryLock).
        v = (env && env[0] == '0') ? 0 : 1;
        cached.store(v, std::memory_order_relaxed);
    }
    return v == 1;
}

inline int getDeferredReclaimDelay() {
    static int ticks = []{
        const char* env = getenv("SMASH_DEFERRED_RECLAIM_DELAY");
        return (env && atoi(env) > 0) ? atoi(env) : 2;
    }();
    return ticks;
}

// ── Eager-zero mode ─────────────────────────────────────────────────────────
// Set SMASH_EAGER_ZERO=1 to memset newly-allocated buffers to zero on the
// malloc fast path, instead of relying on the compressor thread's deferred
// zero-on-free pass. This trades throughput for correctness with callers
// that assume malloc returns zeroed memory (technically UB, but real-world
// codebases rely on it). Used to A/B test whether deferred-zero is the
// source of data-corruption crashes (e.g. ARM64 PAC failures from stale
// pointer-shaped bytes in reissued slab slots).
[[gnu::always_inline, gnu::hot]]
inline bool isEagerZeroMode() {
    static std::atomic<int> cached{-1};
    int v = cached.load(std::memory_order_relaxed);
    if (v < 0) [[unlikely]] {
        const char* env = getenv("SMASH_EAGER_ZERO");
        v = (env && env[0] == '1') ? 1 : 0;
        cached.store(v, std::memory_order_relaxed);
    }
    return v == 1;
}

} // namespace smash
