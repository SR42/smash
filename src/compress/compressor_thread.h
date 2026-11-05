// smash/src/compress/compressor_thread.h - Background scan-and-compress loop
//
// Integrates cold page detection (access tracking) and multi-algorithm
// compression (LZ4/zstd/zstd+dict). Supports adaptive algorithm selection
// based on cold duration, per-size-class dictionary training, prefetching
// of adjacent pages on fault, and batch decommit.
//
// Parallelism: adaptive worker pool (Little's Law) processes disjoint page ranges.
// Chunk bitmap skips EMPTY pages. Per-fault-slot DCtx avoids decompress races.
// Sharded CompressStore eliminates single lock bottleneck.
#pragma once

#include "smash/config.h"
#include "compress_store.h"
#include "spill_store.h"
#include "compress_engine.h"
#include "compression_roi.h"
#include "../vm/vm_region.h"
#include "../vm/page_state.h"
#include "../vm/platform_mem.h"
#include "../core/bootstrap_alloc.h"
#include "../core/page_map.h"
#include "../util/waker.h"
#include "../util/spinlock.h"
#include "../util/safe_printf.h"  // signal-handler-safe snprintf for SIGUSR2 stats

namespace smash {
// Set true when a loaded profile says external pages were hot, so the mmap/
// mach_vm interposers skip external-page tracking. Defined here as a C++17
// inline variable (single definition across all TUs) so it links in both the
// main libsmash build (smash_heap.cpp) AND the compress-only build
// (co_interpose_*.cpp), which compiles compressor_thread.h but not
// smash_heap.cpp — the latter previously left this symbol undefined on
// macOS's strict linker (Linux happened to tolerate it).
inline std::atomic<bool> g_smash_skip_external_tracking{false};
}

#include <csignal>
#include <cstdio>
#include "../vm/fault_handler.h"
#include "../vm/syscall_compat.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <pthread.h>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <dirent.h>
#include <sys/file.h>         // flock — cross-process profile-merge lock
#if defined(__linux__)
#include <sys/uio.h>          // process_vm_readv
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/mman.h>         // mmap/munmap for the spill mapping
#include <sys/vfs.h>          // statfs — reject tmpfs/ramfs spill dirs
#include <sys/syscall.h>      // SYS_membarrier
#include <linux/membarrier.h> // MEMBARRIER_CMD_*
#ifndef TMPFS_MAGIC
#define TMPFS_MAGIC 0x01021994
#endif
#ifndef RAMFS_MAGIC
#define RAMFS_MAGIC 0x858458f6
#endif
#endif
#if defined(__APPLE__)
#include <mach/mach.h>
#endif

#include <cstdio>
#include <lz4.h>
#define ZSTD_STATIC_LINKING_ONLY
#include <zstd.h>

#if defined(__x86_64__) || defined(_M_X64)
#include <emmintrin.h>  // _mm_stream_si64, _mm_sfence
#endif

namespace smash {

// Compressor thread bypass flag. inline thread_local (C++17) gives a single
// definition across all TUs — both libsmash.so and test executables that
// include this header directly without linking smash_heap.cpp.
inline __attribute__((tls_model("initial-exec")))
    thread_local bool g_compressor_thread = false;

// ── Optional remote-core store-drain barrier ───────────────────────────────
// SMASH_PROT_READ_BARRIER=1 enables a syscall after mprotect(PROT_READ)
// (and before the snapshot memcpy) that forces all other application threads
// to drain their store buffers. Without this, an in-flight store on a remote
// core can retire after the local mprotect IPI ack but before the store buffer
// is drained, leaving the snapshot reading stale bytes — and a later
// decompress-on-fault silently reverts the writer's update.
//
// On Linux ≥ 4.14 we use membarrier(MEMBARRIER_CMD_PRIVATE_EXPEDITED, …),
// which IPIs every other thread of this process and waits for them to issue
// a full barrier before returning. Cost: ~5–20 µs per call; we amortize by
// only calling once per chunked PROT_READ run. On non-Linux it's a no-op.
//
// Registration (MEMBARRIER_CMD_REGISTER_PRIVATE_EXPEDITED) happens lazily on
// first use; if registration or the syscall fails we fall back to a local
// __sync_synchronize and disable further attempts.
[[gnu::always_inline]]
inline bool protReadBarrierEnabled() {
    static const bool enabled = []{
        const char* v = std::getenv("SMASH_PROT_READ_BARRIER");
        return v && v[0] == '1';
    }();
    return enabled;
}

// Drain remote-core store buffers so any in-flight store from a writer
// thread is forced to retire (or fault, if its target page is already
// PROT_NONE). Used between mprotect(PROT_NONE) and madvise(DONTNEED) to
// close the post-snapshot TLB-lag store-loss window. This is correctness,
// not perf — the syscall is mandatory under FixAv. Linux only; macOS
// fallback is a local fence (incomplete; see project notes).
inline void membarrierSyncCore() {
#if defined(__linux__) && defined(SYS_membarrier)
    static std::atomic<int> state{0};  // 0=unknown, 1=ok, -1=disabled
    int s = state.load(std::memory_order_relaxed);
    if (s < 0) { __sync_synchronize(); return; }
    if (s == 0) {
        long r = syscall(SYS_membarrier,
                         MEMBARRIER_CMD_REGISTER_PRIVATE_EXPEDITED, 0, 0);
        if (r != 0) {
            state.store(-1, std::memory_order_relaxed);
            __sync_synchronize();
            return;
        }
        state.store(1, std::memory_order_relaxed);
    }
    long r = syscall(SYS_membarrier,
                     MEMBARRIER_CMD_PRIVATE_EXPEDITED, 0, 0);
    if (r != 0) {
        state.store(-1, std::memory_order_relaxed);
        __sync_synchronize();
    }
#else
    __sync_synchronize();
#endif
}

inline void protReadBarrier() {
    // Env-gated wrapper around the unconditional core. SMASH_PROT_READ_BARRIER
    // controls only whether protect-read sites pay the syscall; FixAv's
    // mprotect(PROT_NONE)→madvise sequence calls membarrierSyncCore directly
    // because that one is correctness-mandatory, not opt-in.
    if (!protReadBarrierEnabled()) return;
    membarrierSyncCore();
}

// FixAv: verify-then-flip ordering — snapshot under PROT_READ → compress →
// snapshot-verify → state=COMPRESSED → mprotect(PROT_NONE) → membarrier →
// madvise(DONTNEED). Setting PROT_NONE before madvise prevents readers from
// observing a DROPPED+RO page (kernel zero-faults). Membarrier drains
// in-flight stores; those that retire after PROT_NONE fault visibly and the
// fault handler decompresses. Snapshot-verify (always-on under FixAv) closes
// the residual writer race where a store retires inside the snapshot window.
// SMASH_FIXAV=1 enables the new ordering and the unconditional verify; off
// keeps legacy behavior so we can validate before flipping the default.
[[gnu::always_inline]]
inline bool fixavEnabled() {
    static const bool enabled = []{
        const char* v = std::getenv("SMASH_FIXAV");
        if (v) return v[0] != '0';
        // Default ON. Soft-dirty prevents compressing recently-written pages
        // but cannot close the micro-race where a write lands DURING the
        // snapshot memcpy (same tick, between the per-page lock acquire and
        // the memcpy completion). The post-snapshot verify is the only
        // defense: re-read the page and abort if it differs from the snapshot.
        // Cost: one extra memcpy + memcmp per compressed page (~2 us/page on
        // modern CPUs — negligible vs zstd compression at ~10 us/page).
        return true;
    }();
    return enabled;
}

// Deferred madvise: decouple mprotect(PROT_NONE) from madvise(DONTNEED) by
// a temporal gap. mprotect runs immediately to give us fault-driven
// decompression; madvise is queued and only runs once the page has stayed
// quiescent (no fault) for kDeferredMadviseTicks ticks. Closes the residual
// SMASH_COLD_TIMEOUT_SEC=1 corruption where in-flight loads from a stale
// TLB observed a (PROT_*, DROPPED) page and saw kernel-zero-faulted bytes.
[[gnu::always_inline]]
inline bool deferMadviseEnabled() {
    // Default ON: 9/9 PASS at SMASH_COLD_TIMEOUT_SEC ∈ {1,5,10} on
    // test7_full closes the residual mprotect→madvise corruption surface
    // (in-flight loads with stale TLB observing DROPPED pages). Set
    // SMASH_DEFER_MADVISE=0 to revert to the legacy immediate-madvise path
    // for diagnostic comparison.
    static const bool enabled = []{
        const char* v = std::getenv("SMASH_DEFER_MADVISE");
        if (v) return v[0] == '1';
        return true;
    }();
    return enabled;
}

// SMASH_PROFILE_THRASH_BACKOFF (default OFF): when a persisted profile records
// that an (arena, size_class) bucket thrashed last run (high compress→
// decompress churn), phase2 multiplies that bucket's cold floor so its pages
// must stay quiescent much longer before becoming eligible again.
//
// MEASURED NET-NEGATIVE on neuron-cc test7_full (2026-06-05): enabling it made
// the compile *slower* (950s vs 721s control, +32%) and slightly *increased*
// churn (23.5% vs 22.9%). Deferring compression keeps thrashy pages ACTIVE
// longer, so the per-tick scan visits more uncompressed pages every tick and a
// large wave compresses at once when the inflated floor is finally crossed —
// the opposite of the intended effect. Kept as an opt-in knob (=1) to document
// the negative result and allow re-testing on other workloads; default OFF.
inline bool profileThrashBackoffEnabled() {
    static const bool enabled = []{
        const char* v = std::getenv("SMASH_PROFILE_THRASH_BACKOFF");
        return v && v[0] == '1';
    }();
    return enabled;
}

inline uint32_t deferMadviseTicks() {
    static const uint32_t ticks = []() -> uint32_t {
        const char* v = std::getenv("SMASH_DEFER_MADVISE_TICKS");
        if (v) {
            int n = std::atoi(v);
            if (n > 0 && n < 100000) return static_cast<uint32_t>(n);
        }
        // REGRESSION FIX (2026-06-06): the old default of 50 was documented as
        // "~500ms at 10ms tick rate" — but the compressor tick interval is
        // kCompressIntervalMs = 1000ms, so 50 ticks = 50 SECONDS. That meant a
        // just-compressed page's physical backing was not dropped for 50s.
        // Any workload whose cooling window is shorter than 50s therefore saw
        // ZERO RSS reduction (pages COMPRESSED but still resident), making
        // smash look strictly worse than the baseline allocators. Bisected to
        // 823515f, which introduced the deferred-madvise sweep with this
        // mismatched constant. The deferral exists to close a TLB-lag
        // corruption window (a few ticks is ample); 2 ticks (~2s) keeps that
        // safety margin while letting reclaim actually happen within a normal
        // cooling phase. Override with SMASH_DEFER_MADVISE_TICKS.
        return 2;  // ~2s at the 1s compressor tick rate
    }();
    return ticks;
}

// Force-initialize all function-local statics that call getenv().
// Must be called before the fault handler is installed (i.e., during
// CompressorThread::init()). Without this, a first-time call on the
// compressor thread that happens to fault → signal handler → the lazy
// static init calls getenv() → getenv traverses environ on a page
// that smash compressed (PROT_NONE) → recursive SIGSEGV → crash.
// Diagnosed from Redis crash: 1M-key fill, compressor thread faults
// during memcpy of a freed page, signal handler chains to default,
// but the lazy static in the handler's callee hadn't initialized yet.
inline void warmupEnvStatics() {
    (void)protReadBarrierEnabled();
    (void)fixavEnabled();
    (void)deferMadviseEnabled();
    (void)profileThrashBackoffEnabled();
    (void)deferMadviseTicks();
    (void)isDeferredReclaimMode();
    (void)getDeferredReclaimDelay();
    (void)getColdTicks();
    (void)getSmashMode();
    (void)isLargeOnlyMode();
    (void)vm::noDecommitEnabled();
    (void)vm::madvMode();
}

class CompressorThread {
    // ── Shared state ──────────────────────────────────────────────────────
    VmRegion* vm_ = nullptr;
    PageStateTable* states_ = nullptr;
    PageLockTable* locks_ = nullptr;
    CompressStore* store_ = nullptr;
    CompressEngine* engine_ = nullptr;      // shared: dict training, dict lookup

    // File-backed spill for very-cold blobs (SMASH_SPILL). When ready, very-cold
    // deep-tier blobs are stored here (clean, kernel-evictable page cache)
    // instead of anonymous RAM. Lazily set up on first very-cold store; all
    // null/!ready means the feature is off and behavior is byte-identical.
    SpillStore spill_;
    int spill_fd_ = -1;                 // unlinked spill file
    void* spill_map_ = nullptr;         // MAP_SHARED mapping base
    size_t spill_map_size_ = 0;         // mapping size
    SpillStore::Region* spill_meta_ = nullptr;  // out-of-band region metadata (bootstrap)
    bool spill_init_done_ = false;      // one-shot setup attempted (success or give-up)
    std::atomic<size_t> spilled_count_{0};  // live kTierSpilled pages (fork fast-path guard)
    PageMap* page_map_ = nullptr;
    vm::FaultHandler* fault_handler_ = nullptr;

    // Optional pre-tick callback (e.g., for VM region scanning in compress-only mode)
    using PreTickFn = void(*)();
    PreTickFn pre_tick_fn_ = nullptr;

    // Optional callback invoked once per successful page compression.
    // Used by SmashHeap to drive the A3 cold-arena feedback loop:
    // arena_id and size_class identify the originating slab.
    using CompressedFn = void(*)(size_t page_idx, uint8_t arena_id,
                                 uint8_t sc, void* ctx);
    CompressedFn compressed_fn_ = nullptr;
    void* compressed_ctx_ = nullptr;

    // Decompressed-page hook (COMPRESSED → ACTIVE on fault).  A "re-warm"
    // is direct evidence the (arena, sc) bucket is hot for this page; the
    // heap uses it to drive adaptive under-packing.
    using DecompressedFn = void(*)(size_t page_idx, uint8_t arena_id,
                                   uint8_t sc, void* ctx);
    DecompressedFn decompressed_fn_ = nullptr;
    void* decompressed_ctx_ = nullptr;

    // Tier-upgrade telemetry: cumulative counts since process start.
    // tier_upgrade_attempts_: every page eligible for upgrade we visited.
    // tier_upgrade_success_:  upgrade actually wrote a smaller blob.
    std::atomic<uint64_t> tier_upgrade_attempts_{0};
    std::atomic<uint64_t> tier_upgrade_success_{0};

    // SMASH_SNAPSHOT_VERIFY telemetry. Counts of page snapshots that were
    // re-read after the snapshot memcpy. *_fails_ increments when the
    // re-read differs from the snapshot — a store retired in our window
    // and we abandoned the attempt. *_passes_ increments when the re-read
    // matches and we proceed.
    std::atomic<uint64_t> snapshot_verify_fails_{0};
    std::atomic<uint64_t> snapshot_verify_passes_{0};

    // Pages found entirely zero at compress time and stored as the no-blob
    // sentinel (data=nullptr, algo=NONE) instead of a CompressStore slot.
    std::atomic<uint64_t> zero_pages_{0};

    // Per-page metadata (allocated from bootstrap, indexed by VmRegion page index)
    CompressedPageInfo* compressed_ = nullptr;
    std::atomic<bool>* accessed_ = nullptr;
    uint8_t* cold_count_ = nullptr;

    // Per-page tier marker for tiered recompression.  0 = no blob (page in
    // ACTIVE/COMPRESSING/etc.), 1 = fast-tier blob (LZ4 or zstd-1, eligible
    // for upgrade), 2 = deep-tier blob (zstd-9 or ZSTD_DICT, terminal).
    // Set when compressPage / recompressPage publishes a blob; checked in
    // phase2 to find upgrade candidates without re-running compression.
    enum TierLevel : uint8_t {
        kTierNone = 0,
        kTierFast = 1,
        kTierDeep = 2,
        // Tried upgrading from fast to deep tier but the result wasn't
        // ≥20% smaller. Phase 2 skips these in subsequent ticks so we
        // don't redo the (decompress-then-recompress) work every tick
        // for zero benefit. On long compiles (test7_full ~6 min, where
        // very_cold_ticks=60 fires multiple times per page) this saves
        // serious wall time.
        kTierFastTried = 3,
        // Blob lives in the file-backed SpillStore (SMASH_SPILL): a very-cold
        // deep-tier blob whose bytes were moved out of anonymous RAM into the
        // spill file so the kernel can evict them as clean page cache.
        // compressed_[i].data still points at the bytes (into the spill mmap),
        // so decompress is unchanged; only the RELEASE path dispatches on this.
        kTierSpilled = 4,
    };
    uint8_t* page_tier_ = nullptr;
    // Per-page recompression count: bumped on every COMPRESSED → ACTIVE
    // fault, decayed in phase1Range when the page settles cold.  phase2Range
    // raises the effective cold-tick floor by 2^min(rc + bucket_bias, kMaxBackoffShift).
    // Saturating uint8_t — the bucket EMA captures values above ~6 anyway.
    uint8_t* recompress_count_ = nullptr;

    // Per-page count of ratio-gate failures (compressPage attempts whose
    // output exceeded min_compress_ratio). Feeds the same exponential
    // effective-floor backoff as recompress_count_, but is reset ONLY by a
    // real access to the page — an idle incompressible page's contents
    // cannot change, so cold-streak decay must not re-arm eager retries.
    uint8_t* compress_fail_count_ = nullptr;

    // ── Soft-dirty ROI per-page state (SMASH_SOFTDIRTY_ROI) ─────────────────
    // write_clean_streak_: consecutive ticks a page's soft-dirty (write) bit
    // stayed UNSET. Distinct from cold_count_ (which also resets on reads under
    // the PROT_READ fallback); this is write-specific evidence the page's data
    // is stable, so its compressed blob would survive. Drives the per-page
    // "streak relief" that lets a long-write-clean page override a thrashy
    // bucket's re-dirty prior. Saturating uint8_t.
    uint8_t* write_clean_streak_ = nullptr;

    // Deferred-reclaim mode: tick at which Phase A completed for each page.
    // Non-zero only for pages in COMPRESSED_SHADOW state.
    uint32_t* shadow_tick_ = nullptr;

    // Deferred-madvise queue (SMASH_DEFER_MADVISE=1). Allocated alongside
    // the other per-page arrays. deferred_pending_[i]=true means page i is
    // COMPRESSED+PROT_NONE+still-backed and waiting for the sweeper to drop
    // backing pages once the TTL expires. deferred_queue_tick_[i] is the
    // tick at which compressPage queued the page; the sweeper compares
    // tick_counter_ - deferred_queue_tick_[i] against kDeferredMadviseTicks.
    // Ordering invariant: state must transition to COMPRESSED BEFORE the
    // pending bit is set; any state transition out of COMPRESSED clears
    // the bit FIRST so the sweeper never madvise's a non-COMPRESSED page.
    std::atomic<bool>* deferred_pending_ = nullptr;
    uint32_t* deferred_queue_tick_ = nullptr;

    uint32_t tick_counter_ = 0;
    int phase3_idle_ticks_ = 0;  // P2.1 selective Phase 3 skip

    // Page index of the C `environ` array within smash's contiguous arena, or
    // kNoEnvironPage if environ is not smash-managed. Refreshed each tick by
    // refreshEnvironExclusion(); phase2Range/phase3Range skip this page so a
    // fork+exec child's getenv() never hits a compressed (PROT_NONE) environ.
    static constexpr size_t kNoEnvironPage = ~size_t{0};
    size_t environ_page_idx_ = kNoEnvironPage;
    long vma_count_cached_ = 0;   // VMA-cap guard: refreshed every N ticks

    // ── Time-budget machinery (SMASH_TIME_BUDGET_PCT) ────────────────────
    // start_steady_time_ is captured at start() and used to compute the
    // available compress budget as elapsed × pct/100. budget_recompute_period_
    // controls how often recomputeMarginalEfficiency runs (every N ticks);
    // 8 keeps the work cheap (~144 buckets sorted) while still tracking
    // workload phase changes.
    std::chrono::steady_clock::time_point start_steady_time_{};
    static constexpr int kBudgetRecomputeEveryTicks = 8;

    // Soft-dirty access tracking (Linux). When SMASH_SOFTDIRTY=1 we replace
    // Phase 3's mprotect(PROT_READ)-based write detection with kernel-side
    // soft-dirty bit tracking via /proc/self/clear_refs + /proc/self/pagemap.
    // Wins:
    //   - No VMA fragmentation (mprotect creates one VMA boundary per
    //     transitioning page; soft-dirty has no VMA cost).
    //   - No SIGSEGV-per-write overhead (kernel just sets a PTE bit).
    //   - One syscall per tick to clear vs N mprotect calls.
    // Cost: ~8 bytes per page in pagemap read at tick start, all-process
    // soft-dirty wipe at tick end (kernel does it lazily by zapping PTEs).
    [[maybe_unused]] int clear_refs_fd_ = -1;
    [[maybe_unused]] int pagemap_fd_ = -1;

    // Idle-page read tracking (Linux, opt-in, privileged). The only fault-free
    // way to learn whether a page was READ since last tick is the PTE Accessed
    // (young) bit, exposed by the kernel's idle-page-tracking sysfs interface
    // (/sys/kernel/mm/page_idle/bitmap). Today read detection costs an
    // mprotect(PROT_NONE) to arm + a SIGSEGV per read (escalateToDeepMonitoring).
    // The idle bitmap removes both: mark a candidate page "idle" this tick, and
    // next tick test the bit — still idle ⇒ not read ⇒ read-cold (compressible);
    // cleared ⇒ was read ⇒ reset cold_count, no fault paid.
    //
    // HARD CONSTRAINT: the bitmap is PFN-indexed and root-only; /proc/self/pagemap
    // redacts the PFN to 0 for unprivileged processes. So this works ONLY when
    // smash runs with CAP_SYS_ADMIN on a CONFIG_IDLE_PAGE_TRACKING=y kernel.
    // ensureIdleReadFds() probes both conditions; on failure idle_read_ok_ stays
    // false and we fall back to the existing PROT_NONE read-arm, byte-identical.
    [[maybe_unused]] int page_idle_fd_ = -1;     // /sys/kernel/mm/page_idle/bitmap
    [[maybe_unused]] int idle_read_ok_ = -1;     // -1 unprobed, 0 unavailable, 1 active

    // /proc/self/mem fd (Linux). Used to populate a page's physical backing
    // *while it is still PROT_NONE*, before flipping it readable — closing the
    // decompress-restore TOCTOU window where a concurrent app thread could read
    // a page that is already PROT_RW but does not yet hold the decompressed
    // data. A pwrite() through this fd uses the kernel's FOLL_FORCE access
    // path, which bypasses the PTE protection bits (the VMA still carries
    // VM_MAYWRITE because our reservation is PROT_RW), so the store lands even
    // though direct user stores to the page would fault. Opened lazily on
    // first restore; reset across fork() like the soft-dirty fds.
    [[maybe_unused]] int self_mem_fd_ = -1;

    // Per-(arena, size_class) recompression-rate signal. Single shared
    // table (not per-worker) so the fault handler can update it without
    // first finding the right worker. Relaxed atomics; tearing on the EMA
    // is acceptable.  Indexed by `arena * kTotalBucketsPerArena + bucket`,
    // where bucket is in [0, kNumClasses) for slab pages or
    // [kNumClasses, kTotalBucketsPerArena) for large-alloc page-count buckets.
    std::atomic<uint16_t>* bucket_rc_ema_x256_ = nullptr;  // ×256 fixed-point
    std::atomic<uint8_t>*  bucket_rc_count_ = nullptr;     // sample count, capped 64
    static constexpr size_t kBucketTableLen = kNumArenas * kTotalBucketsPerArena;

    // Cohort measurement (kMeasureCohorts).  Points to SmashHeap's CohortPage
    // array — opaque here to avoid circular include.  Layout matches
    // SmashHeap::CohortPage: {uint32_t first_tid, uint32_t first_ra,
    //                          uint8_t mixed_tid, uint8_t mixed_ra}.
    struct CohortPage {
        uint32_t first_tid;
        uint32_t first_ra;
        uint8_t  mixed_tid;
        uint8_t  mixed_ra;
    };
    CohortPage* cohort_pages_ = nullptr;
    size_t cohort_pages_len_ = 0;

    // ── Chunk bitmap for fast scanning ────────────────────────────────────
    // One bit per page in kChunkSize-page chunks. Rebuilt at tick start.
    // All three phases iterate only set bits via ctzll + mask clear.
    uint64_t* live_chunks_ = nullptr;
    size_t num_chunks_ = 0;

    // ── Fault handler slots ───────────────────────────────────────────────
    // Each slot has its own buffer and ZSTD DCtx to eliminate data races
    // between concurrent faulting threads and prefetch decompression.
    struct FaultSlot {
        void* buf = nullptr;
        ZSTD_DCtx* dctx = nullptr;
        std::atomic<bool> used{false};
    };
    static constexpr int kFaultSlotCount = 128;  // Enough for many concurrent faults + prefetch
    FaultSlot fault_slots_[kFaultSlotCount]{};

    int acquireFaultSlot() {
        for (int i = 0; i < kFaultSlotCount; ++i) {
            bool expected = false;
            if (fault_slots_[i].used.compare_exchange_strong(
                    expected, true, std::memory_order_acquire))
                return i;
        }
        return -1;
    }

    void releaseFaultSlot(int slot) {
        fault_slots_[slot].used.store(false, std::memory_order_release);
    }

    // ── Per-bucket sliding window stats ───────────────────────────────────
    // A bucket is (arena_id, size_class) per worker.  Each bucket tracks
    // observed compression ratio AND observed compression time per profile
    // (fast/deep), so the ROI model can compute cost/benefit from data the
    // workload has actually produced rather than from synthetic calibration
    // of zeros/random pages that may not resemble real allocator output.
    struct SizeClassStats {
        static constexpr int kWindow = 64;
        static constexpr int kTiers = 2;   // 0 = fast tier, 1 = deep tier
        // Ratio sliding window (0-255, mapping to 0-100% bytes saved).
        uint8_t ratios[kWindow]{};
        uint8_t head = 0, count = 0;
        uint16_t sum = 0;
        // Observed compression time EMA per tier, in microseconds × 16
        // (fixed-point to retain sub-microsecond precision without floats).
        uint32_t cost_ema_x16[kTiers]{};
        uint8_t  cost_count[kTiers]{};

        // UCB1-Tuned (Auer/Cesa-Bianchi/Fischer 2002) per-arm state for
        // tier selection.  Reward is bytes-saved-per-microsecond, clamped
        // to [0, kUcbRewardMaxBytesPerUs] and normalized to [0,1].  Mean
        // and M2 (sum of squared deviations) are updated via Welford.
        uint32_t arm_pulls[kTiers]{};
        double   arm_mean[kTiers]{};
        double   arm_m2[kTiers]{};
        // Per-bucket selection counter.  Used by SMASH_UCB_FORCE_DEEP_EVERY
        // to cycle deep-tier pulls without synchronizing the forcing across
        // buckets (which would create a tick-aligned CPU spike).
        uint32_t selections = 0;

        // ── Soft-dirty ROI: per-bucket re-dirty rate (×256 fixed point) ──────
        // EMA of "a page compressed in this bucket was WRITTEN again shortly
        // after" (soft-dirty set on a recently-decompressed page). High value ⇒
        // compressing this bucket's pages tends to be wasted (blob invalidated
        // by a write), so ROI benefit should be discounted. Observations come
        // from recordRedirty(true on a write-redirty event, false on a clean
        // survival). α = 1/8 (slower than cost EMA: re-dirty is the rarer, more
        // load-bearing signal and we want it stable). count gates trust.
        uint16_t redirty_ema_x256 = 0;
        uint8_t  redirty_count = 0;

        void recordRedirty(bool rewritten) {
            uint16_t sample = rewritten ? 256 : 0;
            if (redirty_count == 0) {
                redirty_ema_x256 = sample;
            } else {
                redirty_ema_x256 = static_cast<uint16_t>(
                    redirty_ema_x256 +
                    (static_cast<int32_t>(sample) - static_cast<int32_t>(redirty_ema_x256)) / 8);
            }
            if (redirty_count < 64) redirty_count++;
        }

        // Survival discount (×256) for computeROI: (256 - P_redirty). Returns
        // 256 (no discount) until enough samples accumulate, so a cold bucket
        // isn't penalized on noise. `streak_relief` (0..256) nudges the discount
        // back toward 1.0 for an individual page with a long write-clean streak
        // (per-page evidence overriding the bucket prior — the "both" design).
        uint32_t redirtyDiscountX256(uint32_t streak_relief_x256) const {
            if (redirty_count < 8) return 256;          // prior: trust nothing yet
            uint32_t p = redirty_ema_x256;              // P_redirty ×256
            if (streak_relief_x256 > 0) {               // relieve by per-page streak
                p = (p > streak_relief_x256) ? (p - streak_relief_x256) : 0;
            }
            return (p >= 256) ? 0 : (256 - p);          // survival = 1 - P_redirty
        }

        void record(size_t comp_size, size_t orig_size) {
            uint8_t r = 0;
            if (orig_size > 0 && comp_size < orig_size)
                r = static_cast<uint8_t>(255 * (orig_size - comp_size) / orig_size);
            if (count == kWindow) {
                sum -= ratios[head];
            } else {
                count++;
            }
            ratios[head] = r;
            sum += r;
            head = static_cast<uint8_t>((head + 1) % kWindow);
        }

        // Record compression time observed for a page compressed with the
        // given tier (0 = fast, 1 = deep).  EMA with α = 1/4.
        void recordCost(int tier, uint32_t elapsed_us) {
            if (tier < 0 || tier >= kTiers) return;
            uint32_t sample_x16 = elapsed_us * 16;
            if (cost_count[tier] == 0)
                cost_ema_x16[tier] = sample_x16;
            else
                cost_ema_x16[tier] += (static_cast<int32_t>(sample_x16) -
                    static_cast<int32_t>(cost_ema_x16[tier])) / 4;
            if (cost_count[tier] < 64) cost_count[tier]++;
        }

        // Observed compression microseconds for the given tier.  Returns 0
        // when fewer than 8 samples exist; callers should fall back to the
        // profile's calibrated throughput in that case.
        uint32_t observedCostUs(int tier) const {
            if (tier < 0 || tier >= kTiers) return 0;
            return cost_count[tier] >= 8
                ? (cost_ema_x16[tier] + 8) / 16 : 0;
        }

        // UCB reward update.  reward must be in [0,1].  Uses Welford so the
        // posterior mean and variance stay numerically stable as pulls grow.
        void recordReward(int tier, double reward) {
            if (tier < 0 || tier >= kTiers) return;
            if (reward < 0.0) reward = 0.0;
            if (reward > 1.0) reward = 1.0;
            uint32_t n = ++arm_pulls[tier];
            double delta  = reward - arm_mean[tier];
            arm_mean[tier] += delta / static_cast<double>(n);
            double delta2 = reward - arm_mean[tier];
            arm_m2[tier] += delta * delta2;
        }

        // ── Profile persistence (P3) ─────────────────────────────────────
        // Compact 16-byte serialized form, suitable for SMASH_PROFILE_FILE.
        // Decision hint: 0 = unknown, 1 = always-skip, 2 = fast-only,
        //                3 = deep-default. Set by load() based on observed
        // history; checked in phase2Range to short-circuit pages from
        // historically-uncompressible buckets.
        struct Persist {
            uint8_t  count;
            uint8_t  decision_hint;
            uint16_t sum;                 // ratio EMA proxy
            uint16_t cost_ema_x16_t0;     // saturated to u16
            uint16_t cost_ema_x16_t1;
            uint8_t  cost_count_t0;
            uint8_t  cost_count_t1;
            // Repurposed first 2 of 6 pad bytes for the budget machinery.
            // Older v1 files set these to zero, which deserialize() treats
            // as "no efficiency carried over" — safe.
            uint16_t efficiency_x256;
            // v6: thrash rate and best tier for profile-driven optimization
            uint8_t  thrash_rate;         // 0-255: recompress events / compress events
            uint8_t  best_tier;           // 0=LZ4, 1=zstd, 255=unknown
            uint8_t  stable_cold;         // 1=pages stay cold, can skip observation
            uint8_t  pad;
        };
        static_assert(sizeof(Persist) == 16, "Persist record must be 16 bytes");

        void serialize(Persist* out) const {
            out->count = count;
            // Decision hint heuristic:
            //  - if mean ratio is very low (<5%): always-skip
            //  - if deep-tier cost ≥ 4× fast-tier cost AND ratio modest: fast-only
            //  - else: unknown
            uint8_t hint = 0;
            if (count >= 8) {
                uint16_t mean_x256 = (count > 0) ? (sum * 256u / count) : 0;
                if (mean_x256 < 13) {  // <5% saved on average
                    hint = 1;
                } else if (cost_count[0] >= 4 && cost_count[1] >= 4 &&
                           cost_ema_x16[1] >= 4 * cost_ema_x16[0] &&
                           mean_x256 < 64) {  // <25% savings + deep is 4×
                    hint = 2;
                }
            }
            out->decision_hint = hint;
            out->sum = sum;
            out->cost_ema_x16_t0 = cost_ema_x16[0] > 65535u ? 65535u
                : static_cast<uint16_t>(cost_ema_x16[0]);
            out->cost_ema_x16_t1 = cost_ema_x16[1] > 65535u ? 65535u
                : static_cast<uint16_t>(cost_ema_x16[1]);
            out->cost_count_t0 = cost_count[0];
            out->cost_count_t1 = cost_count[1];
            out->efficiency_x256 = efficiency_x256;

            // v6: thrash rate and tier selection
            uint32_t cc = compress_count.load(std::memory_order_relaxed);
            uint32_t dc = decompress_count.load(std::memory_order_relaxed);
            // thrash_rate: fraction of compressions that got decompressed (0-255)
            out->thrash_rate = (cc > 0) ? static_cast<uint8_t>(
                std::min<uint32_t>(255, dc * 255 / cc)) : 0;
            // best_tier: pick tier with more pulls (has been explored more)
            // If one tier dominates (>2x pulls), use it; else unknown
            if (arm_pulls[0] > 2 * arm_pulls[1] + 4) {
                out->best_tier = 0;  // LZ4
            } else if (arm_pulls[1] > 2 * arm_pulls[0] + 4) {
                out->best_tier = 1;  // zstd
            } else {
                out->best_tier = 255;  // unknown
            }
            // stable_cold: thrash rate < 10% means pages stay cold
            out->stable_cold = (cc >= 8 && out->thrash_rate < 26) ? 1 : 0;
            out->pad = 0;
        }

        void deserialize(const Persist& in) {
            // Apply with halved count so old data fades fast if workload
            // changed; new observations dominate after ~kWindow/2 records.
            count = in.count > kWindow ? kWindow : (in.count / 2);
            // Reconstruct ratios as a single-bucket histogram using the mean
            // — we don't preserve the per-sample distribution. Sufficient for
            // ROI which only consults sum/count.
            sum = (count > 0) ? (in.sum * count / std::max<uint8_t>(in.count, 1)) : 0;
            head = 0;
            uint8_t mean_r = (count > 0) ? static_cast<uint8_t>(sum / count) : 0;
            for (int i = 0; i < count; ++i) ratios[i] = mean_r;
            cost_ema_x16[0] = in.cost_ema_x16_t0;
            cost_ema_x16[1] = in.cost_ema_x16_t1;
            cost_count[0] = in.cost_count_t0 / 2;
            cost_count[1] = in.cost_count_t1 / 2;
            // Carry budget efficiency across runs as a warm-start hint;
            // recomputeMarginalEfficiency overwrites this once enough fresh
            // data accumulates. Old v1 files store 0 here → treated as
            // EXPLORE on first recompute, which matches cold-start.
            efficiency_x256 = in.efficiency_x256;
            // v6: load stable_cold and best_tier for profile-driven optimization
            profile_stable_cold = in.stable_cold;
            profile_best_tier = in.best_tier;
            // v6+: load thrash_rate so phase2 can back off buckets that
            // churned badly last run (compress→fault→decompress is wasted
            // CPU; deferring those buckets cuts wall time AND reduces churn).
            profile_thrash_rate = in.thrash_rate;
        }

        // Decision hint cached on the live struct after deserialize, so
        // phase2Range can short-circuit without re-running the heuristic.
        uint8_t persist_hint = 0;

        // ── Time-budget machinery (SMASH_TIME_BUDGET_PCT) ────────────────
        // Cumulative bytes saved (orig − compressed) across the bucket, and
        // cumulative microseconds spent compressing+decompressing. Updated
        // from compressPage success path (compress thread) and from the
        // fault decompress path (application thread); std::atomic so the
        // cross-thread updates are well-defined. Relaxed ordering — these
        // are advisory counters consumed by the periodic recompute.
        std::atomic<uint64_t> bytes_saved_total{0};
        std::atomic<uint64_t> time_cost_total_us{0};
        // efficiency = bytes_saved / time_cost_us, in fixed-point ×256.
        // Recomputed once per N ticks by recomputeMarginalEfficiency.
        uint16_t efficiency_x256 = 0;
        // Runtime budget decision: 0=EXPLORE (force-compress while count<8),
        // 1=COMPRESS, 2=SKIP. Distinct from `persist_hint` (legacy persisted
        // always-skip flag) so the two checks compose without aliasing.
        enum BudgetHint : uint8_t {
            kBudgetExplore  = 0,
            kBudgetCompress = 1,
            kBudgetSkip     = 2,
        };
        uint8_t budget_decision_hint = 0;  // == kBudgetExplore

        // v6 profile: thrash tracking for profile-driven cold detection
        std::atomic<uint32_t> compress_count{0};   // successful compressions
        std::atomic<uint32_t> decompress_count{0}; // fault-triggered decompressions
        // Loaded from profile: if stable_cold=1, skip cold observation period
        uint8_t profile_stable_cold = 0;
        uint8_t profile_best_tier = 255;  // 255=unknown, 0=LZ4, 1=zstd
        // Loaded from profile: prior-run thrash rate (0-255 = decompress/compress).
        // High values mark buckets where compression was wasted churn; phase2
        // multiplies the cold floor for these so they must stay quiescent much
        // longer before becoming eligible again. 0 when unknown/cold-start.
        uint8_t profile_thrash_rate = 0;
    };

    // ── Per-arena UCB arm aggregate ───────────────────────────────────────
    // Aggregates (pulls, mean, m2) across all size classes within an arena
    // for each tier.  Used as a warm-start prior when a per-bucket arm
    // hasn't accumulated min_pulls yet — the arena aggregate is a
    // meaningfully better estimate than the uniform prior because arena
    // routing already groups call-sites with similar allocation patterns.
    struct ArenaArmStats {
        static constexpr int kTiers = SizeClassStats::kTiers;
        uint32_t pulls[kTiers]{};
        double   mean[kTiers]{};
        double   m2[kTiers]{};
        void recordReward(int tier, double reward) {
            if (tier < 0 || tier >= kTiers) return;
            if (reward < 0.0) reward = 0.0;
            if (reward > 1.0) reward = 1.0;
            uint32_t n = ++pulls[tier];
            double delta  = reward - mean[tier];
            mean[tier] += delta / static_cast<double>(n);
            double delta2 = reward - mean[tier];
            m2[tier] += delta * delta2;
        }
    };

    // ── Per-worker compression state ──────────────────────────────────────
    // Each worker has its own LZ4 state, ZSTD CCtx, and scratch buffers
    // so Phase 2 (compression) is embarrassingly parallel.
    struct CompressWorker {
        void* lz4_state = nullptr;
        ZSTD_CCtx* zstd_cctx = nullptr;
        void* page_buf = nullptr;
        void* compress_buf = nullptr;
        void* compress_buf2 = nullptr;  // second buffer for dict try-both experiment
        // ROI stats indexed by (arena_id, bucket).  Arena routing produces
        // structurally-homogeneous pages; aggregating stats across arenas
        // would wash out that homogeneity, so each arena gets its own
        // sliding window per bucket.  Bucket is the slab size_class for
        // small allocs or kNumClasses + log2(page_count) for large allocs;
        // index via statsIndex(arena, bucket).
        SizeClassStats sc_stats[kNumArenas * kTotalBucketsPerArena]{};
        // Per-arena UCB aggregate used as warm-start prior under
        // SMASH_UCB_WARMSTART=1.  Indexed by arena_id.
        ArenaArmStats arena_arm[kNumArenas]{};
        size_t range_start = 0, range_end = 0;

        // Pending PROT_NONE batch. compressPage finishes per-page work
        // and stores compressed blob, but defers the
        // (decommit + mprotect(PROT_NONE)) to a chunked flush at the
        // end of phase2Range to slash VMA fragmentation. Pending pages
        // are in state COMPRESSED but still PROT_READ with original
        // contents (writes fault → handleFault → decompress; reads
        // return correct stale data which is fine because content
        // matches the compressed blob).
        static constexpr size_t kPendingProtCap = 256;
        size_t pending_pn_pages[kPendingProtCap];
        size_t pending_pn_count = 0;

        // Compress using worker's own contexts, shared engine's dictionaries
        size_t compress(const void* src, void* dst, size_t src_size,
                       size_t dst_capacity, CompressAlgo algo,
                       uint8_t size_class, int zstd_level,
                       CompressEngine* shared_engine) {
            switch (algo) {
            case CompressAlgo::LZ4: {
                int result = LZ4_compress_fast_extState(
                    lz4_state,
                    static_cast<const char*>(src),
                    static_cast<char*>(dst),
                    static_cast<int>(src_size),
                    static_cast<int>(dst_capacity),
                    1);
                return (result > 0) ? static_cast<size_t>(result) : 0;
            }
            case CompressAlgo::ZSTD: {
                size_t result = ZSTD_compressCCtx(
                    zstd_cctx,
                    dst, dst_capacity,
                    src, src_size,
                    zstd_level);
                return ZSTD_isError(result) ? 0 : result;
            }
            case CompressAlgo::ZSTD_DICT: {
                ZSTD_CDict* cdict = shared_engine->getCDict(size_class);
                if (cdict) {
                    size_t result = ZSTD_compress_usingCDict(
                        zstd_cctx,
                        dst, dst_capacity,
                        src, src_size,
                        cdict);
                    return ZSTD_isError(result) ? 0 : result;
                }
                // Fallback to zstd deep level without dict
                size_t result = ZSTD_compressCCtx(
                    zstd_cctx,
                    dst, dst_capacity,
                    src, src_size,
                    kZstdDeepLevel);
                return ZSTD_isError(result) ? 0 : result;
            }
            default:
                return 0;
            }
        }
    };

    CompressWorker workers_[kMaxCompressorWorkers];
    size_t max_comp_size_ = 0;
    ZSTD_customMem zstd_custom_mem_{};

    static void initWorkerState(CompressWorker& worker, size_t max_comp,
                                ZSTD_customMem custom_mem) {
        worker.page_buf = BootstrapAlloc::instance().allocate(kPageSize, kPageSize);
        worker.compress_buf = BootstrapAlloc::instance().allocate(max_comp, 16);
        worker.compress_buf2 = BootstrapAlloc::instance().allocate(max_comp, 16);
        worker.lz4_state = BootstrapAlloc::instance().allocate(
            static_cast<size_t>(LZ4_sizeofState()), 16);
        worker.zstd_cctx = ZSTD_createCCtx_advanced(custom_mem);
    }

    void ensureWorkerState(int worker_id) {
        auto& w = workers_[worker_id];
        if (w.page_buf) return;
        initWorkerState(w, max_comp_size_, zstd_custom_mem_);
    }

    // ── Dictionary training (coordinator thread) ──────────────────────────
    struct DictTrainState {
        char* sample_data = nullptr;
        size_t* sample_sizes = nullptr;
        std::atomic<uint16_t> num_samples{0};
        bool trained = false;
        bool allocated = false;
        Spinlock alloc_lock;
    };
    DictTrainState dict_train_[kNumClasses]{};

    // ── Dict experiment counters (try-both) ───────────────────────────────
    std::atomic<uint64_t> dict_win_count_{0};   // dict smaller than plain zstd
    std::atomic<uint64_t> dict_loss_count_{0};   // dict larger than plain zstd
    std::atomic<uint64_t> dict_tie_count_{0};    // equal size
    std::atomic<int64_t>  dict_total_delta_{0};  // sum of (plain_size - dict_size), positive = dict wins

    // ── External page profile tracking ────────────────────────────────────
    // Tracks whether ANY external page thrashed (recompress_count > 0) during
    // this process. Set on first external page decompression. When profile is
    // saved, this flag is persisted. On load, if the flag is set, we skip ALL
    // external pages for compression (they were hot in the prior run).
    std::atomic<bool> external_pages_hot_{false};
    bool external_pages_hot_from_profile_{false};

    // ── Thread management ─────────────────────────────────────────────────
    pthread_t coord_thread_{};
    static constexpr int kMaxHelpers = kMaxCompressorWorkers > 1 ? kMaxCompressorWorkers - 1 : 1;
    pthread_t helper_threads_[kMaxHelpers]{};
    std::atomic<bool> running_{false};

    // Fork coordination. paused_ is set by the atfork prepare handler;
    // in_tick_ is set while the coordinator is mid-tick (holding page
    // locks etc.). prepare waits for in_tick_=0 before letting fork()
    // proceed, so the child never inherits a half-done tick.
    std::atomic<bool> paused_{false};
    std::atomic<bool> in_tick_{false};

public:
    // Pause the coordinator (called from a pthread_atfork prepare handler in
    // the parent before fork()). Wait briefly for any in-flight tick to
    // finish. We cap the wait so a stuck tick can never deadlock fork().
    void pauseForFork() {
        // seq_cst store + seq_cst load: the Dekker pairing with coordEntry's
        // in_tick_-then-paused_ sequence (see the comment there). Plain
        // release/acquire does NOT forbid the store-load reordering this
        // handshake relies on.
        paused_.store(true, std::memory_order_seq_cst);
        // Coordinator wakes on its 10 ms tick boundary. Give it a few of
        // those (~50 ms total) to drain in_tick_ before we let fork proceed.
        for (int i = 0; i < 50 && in_tick_.load(std::memory_order_seq_cst); ++i)
            usleep(1000);
    }

    // Resume after fork() in the parent.
    void resumeAfterFork() {
        paused_.store(false, std::memory_order_release);
    }

    // Reset thread-management bookkeeping in a fork()'d child. The actual
    // pthreads themselves are already gone — Linux fork() only clones the
    // calling thread — but the parent's pthread_t handles are still in our
    // arrays and `running_`/`helpers_created_` still claim threads exist.
    // Without this, the singleton's startCompression() CAS fails (because
    // compression_started_ is true) and the child runs with no compressor.
    void resetForFork() {
        running_.store(false, std::memory_order_release);
        paused_.store(false, std::memory_order_release);
        in_tick_.store(false, std::memory_order_release);
        coord_thread_ = pthread_t{};
        for (int i = 0; i < kMaxHelpers; ++i) {
            helper_threads_[i] = pthread_t{};
            helper_done_gen_[i].store(0, std::memory_order_relaxed);
        }
        helpers_created_ = 0;
        active_workers_ = kCompressorWorkers;
        work_gen_.store(0, std::memory_order_release);
        work_waker_.reinitAfterFork();
        current_phase_ = 0;
        // /proc/self/{clear_refs,pagemap} fds inherited from the parent
        // refer to the parent's pid-namespace entries — the kernel
        // resolves /proc/self at open(2) time, not on every read. After
        // fork(), our fds still point at the parent's PTE table. Close
        // them so ensureSoftDirtyFds() re-opens the child's correctly
        // on the first tick post-fork.
        // /proc/self/mem is the same story: the open fd resolves to the
        // parent's address space. Close it so restorePageContents() re-opens
        // the child's on first use. (Writing through a stale parent-mem fd
        // would corrupt the parent or fail — must reset.)
#ifdef __linux__
        if (clear_refs_fd_ >= 0) { close(clear_refs_fd_); clear_refs_fd_ = -1; }
        if (pagemap_fd_ >= 0)    { close(pagemap_fd_);    pagemap_fd_ = -1; }
        if (self_mem_fd_ >= 0)   { close(self_mem_fd_);   self_mem_fd_ = -1; }
        // page_idle bitmap fd is process-global (not /proc/self), but re-probe
        // in the child since PFNs and privilege may differ; reset the cache.
        if (page_idle_fd_ >= 0)  { close(page_idle_fd_);  page_idle_fd_ = -1; }
        idle_read_ok_ = -1;
        // Spill file: the child inherited the parent's MAP_SHARED mapping of the
        // (unlinked) parent spill file plus the in-memory SpillStore offsets/
        // free-list, which are process-local. Continuing to use them would
        // corrupt both processes' bump cursors, and the parent may hole-punch a
        // region out from under the child. So before dropping the inherited
        // mapping, copy every kTierSpilled blob's bytes back into the child's
        // COW-private anonymous CompressStore and re-tag it kTierDeep — a plain
        // byte copy (no decompress needed; comp_size/algo are preserved). Then
        // unmap and reset so the child lazily creates its OWN spill later.
        // Guarded by spilled_count_ so the common (no-spill) fork pays nothing.
        if (spill_.ready() && spilled_count_.load(std::memory_order_relaxed) > 0
            && store_ && compressed_) {
            size_t committed = vm_ ? vm_->committedPages() : 0;
            for (size_t i = 0; i < committed; ++i) {
                if (!page_tier_ || page_tier_[i] != kTierSpilled) continue;
                void* src = compressed_[i].data;
                size_t csz = compressed_[i].compressedSize();
                if (src && spill_.contains(src)) {
                    size_t new_alloc = 0;
                    void* dst = store_->store(src, csz, &new_alloc, i);
                    if (dst) {
                        compressed_[i].set(dst, csz, new_alloc,
                                           compressed_[i].algorithm());
                        page_tier_[i] = kTierDeep;
                    }
                    // If the anonymous store also fails (OOM), leave it tagged
                    // kTierSpilled pointing at the soon-to-be-unmapped blob; an
                    // access would fault — but this is an extreme OOM edge and
                    // far rarer than the wild-store it replaces.
                }
            }
        }
        spilled_count_.store(0, std::memory_order_relaxed);
        if (spill_map_) { munmap(spill_map_, spill_map_size_); spill_map_ = nullptr; }
        if (spill_fd_ >= 0) { close(spill_fd_); spill_fd_ = -1; }
        spill_map_size_ = 0;
        spill_meta_ = nullptr;
        spill_.reset();                // forget inherited parent spill, !ready
        spill_init_done_ = false;      // allow the child to re-init lazily
#endif
    }
private:

    // Work dispatch: coordinator increments work_gen_ to signal helpers.
    // Helpers compare against their last-seen gen to detect new work.
    std::atomic<uint64_t> work_gen_{0};
    Waker work_waker_;       // helpers park here between dispatches
    int current_phase_ = 0;  // 1=access, 2=compress, 3=monitor
    std::atomic<uint64_t> helper_done_gen_[kMaxHelpers]{};

    // ── Adaptive worker scaling (Little's Law) ─────────────────────────
    //
    // By Little's Law, the average number of items in a stable queueing
    // system is L = λ · W, where λ is the arrival rate and W is the mean
    // service time.  For the compression queue:
    //
    //   λ  = pages becoming eligible for compression per tick
    //   μ  = pages one worker compresses per tick  (service rate = 1/W)
    //
    // To keep the queue drained (L → 0), total service rate must meet
    // arrival rate: N · μ ≥ λ.  Solving for the minimum worker count:
    //
    //   N_needed = ⌈λ / μ⌉
    //
    // Both λ and μ are measured each tick and smoothed with EMAs.
    // Workers are pre-allocated up to kMaxCompressorWorkers; helper threads
    // are lazily created on first scale-up and parked when not needed.
    int active_workers_{kCompressorWorkers};  // current active count (runtime)
    int helpers_created_ = 0;  // how many helper threads have been pthread_create'd

    // Per-worker counters for the current tick (reset each tick)
    std::atomic<uint32_t> worker_pages_eligible_[kMaxCompressorWorkers]{};
    std::atomic<uint32_t> worker_pages_compressed_[kMaxCompressorWorkers]{};

    // Exponential moving averages (fixed-point: multiply by 256 to avoid float)
    // EMA update: ema = ema + (sample - ema) / 4   (α = 1/4, ~4-tick response)
    uint32_t lambda_ema_ = 0;  // pages eligible per tick (× 256)
    uint32_t mu_ema_ = 256;    // pages compressed per worker per tick (× 256, init=1 to avoid /0)

    // ── Helper methods ────────────────────────────────────────────────────

    // Soft-dirty ROI gate active? (env-driven, via ROIConfig). Cached read.
    static bool cfgSoftdirtyRoi() {
        return ROIConfig::instance().softdirty_roi;
    }

    uint8_t lookupSizeClass(size_t page_idx) {
        if (!page_map_) return 0;
        void* addr = vm_->pageAddress(page_idx);
        Span* span = page_map_->get(reinterpret_cast<uintptr_t>(addr));
        if (!span) return 0;
        return span->size_class;
    }

    // Read both arena_id and the bucket index in a single page-map lookup.
    // For slab spans, bucket == span->size_class. For large-alloc spans,
    // bucket == largeSizeClass(span->page_count) — slots in the
    // [kNumClasses, kTotalBucketsPerArena) range. Returns false on unmapped
    // pages. Naming kept as `sc` so existing callers compile unchanged; the
    // value is "bucket" semantically, not raw size_class.
    bool lookupSpanInfo(size_t page_idx, uint8_t& arena_id, uint8_t& sc) {
        arena_id = 0; sc = 0;
        if (!page_map_) return false;
        void* addr = vm_->pageAddress(page_idx);
        Span* span = page_map_->get(reinterpret_cast<uintptr_t>(addr));
        if (!span) return false;
        arena_id = span->arena_id;
        if (span->is_large) {
            sc = largeSizeClass(span->page_count);
        } else {
            sc = span->size_class;
        }
        return true;
    }

    // Index into the per-(arena, bucket) ROI stats array. `sc` is the
    // unified bucket index in [0, kTotalBucketsPerArena).
    // Note: arena_id is now ASLR-resilient (computed via stableCallsiteHash
    // in SmashHeap::callsiteArena), so profile data keyed by (arena_id, sc)
    // is stable across runs.
    static inline size_t statsIndex(uint8_t arena_id, uint8_t sc) {
        return static_cast<size_t>(arena_id) * kTotalBucketsPerArena +
               static_cast<size_t>(sc);
    }

    // Soft-dirty ROI: fold one re-dirty observation for `page_idx` into its
    // (arena,bucket) EMA. `rewritten` = a write hit the page after we
    // decompressed it (blob would have been invalidated) vs a read-only touch
    // (blob would have survived). Routed to worker 0's stats — the canonical
    // sink, same convention as decompress_count attribution.
    void recordRedirtyForPage(size_t page_idx, bool rewritten) {
        uint8_t arena_id, sc;
        if (!lookupSpanInfo(page_idx, arena_id, sc)) return;
        if (sc >= kTotalBucketsPerArena) return;
        size_t bidx = statsIndex(arena_id, sc);
        if (bidx < kBucketTableLen)
            workers_[0].sc_stats[bidx].recordRedirty(rewritten);
    }

    bool sameSpan(size_t page_a, size_t page_b) {
        if (!page_map_) return false;
        Span* sa = page_map_->get(reinterpret_cast<uintptr_t>(vm_->pageAddress(page_a)));
        Span* sb = page_map_->get(reinterpret_cast<uintptr_t>(vm_->pageAddress(page_b)));
        return sa && sb && sa == sb;
    }

    // Non-temporal zero: avoids cache pollution when zeroing cold page data.
    // All object slots are 16-byte aligned with sizes that are multiples of 16.
    static void ntZeroMemory(void* dst, size_t size) {
#if defined(__clang__)
        auto* p = static_cast<uint64_t*>(dst);
        size_t n = size / 8;  // always even since size % 16 == 0
        for (size_t i = 0; i < n; i += 2) {
            __builtin_nontemporal_store(static_cast<uint64_t>(0), &p[i]);
            __builtin_nontemporal_store(static_cast<uint64_t>(0), &p[i + 1]);
        }
#elif defined(__x86_64__) || defined(_M_X64)
        auto* p = static_cast<uint64_t*>(dst);
        size_t n = size / 8;
        for (size_t i = 0; i < n; i += 2) {
            _mm_stream_si64(reinterpret_cast<long long*>(&p[i]), 0);
            _mm_stream_si64(reinterpret_cast<long long*>(&p[i + 1]), 0);
        }
        _mm_sfence();
#else
        std::memset(dst, 0, size);
#endif
    }

    // Zero freed slots in the scratch buffer before compression.
    // All freed slots are zeroed here (deferred from free() to avoid critical-path overhead).
    // True iff the (page-aligned) snapshot buffer is entirely zero.
    // Non-zero pages exit at the first non-zero 64-byte group.
    static bool pageIsAllZero(const void* buf) {
        auto* p = static_cast<const uint64_t*>(buf);
        constexpr size_t kWords = kPageSize / sizeof(uint64_t);
        for (size_t i = 0; i < kWords; i += 8) {
            uint64_t acc = p[i] | p[i + 1] | p[i + 2] | p[i + 3] |
                           p[i + 4] | p[i + 5] | p[i + 6] | p[i + 7];
            if (acc != 0) return false;
        }
        return true;
    }

    void zeroFreeSlots(void* page_buf, size_t page_idx) {
        if (!page_map_) return;
        void* page_addr = vm_->pageAddress(page_idx);
        if (!page_addr) return;  // External page untracked
        Span* span = page_map_->get(reinterpret_cast<uintptr_t>(page_addr));
        if (!span || span->is_large || span->object_size == 0) return;

        uintptr_t span_base = reinterpret_cast<uintptr_t>(span->base);
        uintptr_t page_start = reinterpret_cast<uintptr_t>(page_addr);
        size_t obj_size = span->object_size;

        size_t page_off = page_start - span_base;
        size_t page_end_off = page_off + kPageSize;

        // First slot that could overlap this page
        size_t first_slot = page_off / obj_size;

        for (size_t slot = first_slot; slot < span->object_count; ++slot) {
            size_t slot_start = slot * obj_size;
            if (slot_start >= page_end_off) break;
            size_t slot_end = slot_start + obj_size;

            // Check if free (bitmap bit 1 = free)
            if (!(span->bitmap[slot / 64] & (1ULL << (slot % 64)))) continue;

            // Overlap with this page
            size_t zero_start = (slot_start > page_off) ? slot_start : page_off;
            size_t zero_end = (slot_end < page_end_off) ? slot_end : page_end_off;
            if (zero_start >= zero_end) continue;

            size_t buf_offset = zero_start - page_off;
            ntZeroMemory(static_cast<char*>(page_buf) + buf_offset, zero_end - zero_start);
        }
    }

    // ── Chunk bitmap management ───────────────────────────────────────────

    void rebuildChunkBitmap(size_t committed) {
        size_t nchunks = (committed + kChunkSize - 1) / kChunkSize;
        for (size_t c = 0; c < nchunks; ++c) {
            uint64_t mask = 0;
            size_t base = c * kChunkSize;
            size_t end = base + kChunkSize;
            if (end > committed) end = committed;
            for (size_t i = base; i < end; ++i) {
                if (states_->get(i) != PageState::EMPTY)
                    mask |= (1ULL << (i - base));
            }
            live_chunks_[c] = mask;
        }
    }

    // Iterate live pages in [start, end) and call fn(page_idx)
    template<typename Fn>
    void forEachLivePage(size_t start, size_t end, Fn&& fn) {
#ifdef SMASH_ABLATION_NO_CHUNK_BITMAP
        // Linear scan fallback (ablation: measure chunk bitmap contribution)
        for (size_t i = start; i < end; ++i) {
            if (states_->get(i) != PageState::EMPTY)
                fn(i);
        }
#else
        size_t start_chunk = start / kChunkSize;
        size_t end_chunk = (end + kChunkSize - 1) / kChunkSize;
        for (size_t c = start_chunk; c < end_chunk; ++c) {
            uint64_t mask = live_chunks_[c];
            if (mask == 0) continue;
            size_t base = c * kChunkSize;
            while (mask) {
                int bit = __builtin_ctzll(mask);
                size_t idx = base + bit;
                if (idx >= start && idx < end)
                    fn(idx);
                mask &= mask - 1;  // clear lowest set bit
            }
        }
#endif
    }

    // ── Phase implementations ─────────────────────────────────────────────

    // Phase 1: Process access bits and update cold counts
    __attribute__((cold)) void phase1Range(size_t start, size_t end) {
        const bool sd_roi = write_clean_streak_ != nullptr;  // gate on
        forEachLivePage(start, end, [&](size_t i) {
            PageState st = states_->get(i);
            if (st == PageState::ACTIVE || st == PageState::ACTIVE_MONITORING) {
                // Raw WRITE signal (soft-dirty) BEFORE the idle-read augmentation
                // below folds reads in. The soft-dirty ROI machinery needs the
                // write-only bit to track data stability independent of reads.
                const bool written = accessed_[i].load(std::memory_order_relaxed);

                if (sd_roi) {
                    // Per-page write-clean streak: consecutive ticks unwritten.
                    // (Re-dirty classification happens at fault time via the
                    // hardware write-bit, not here — see the decompress path.)
                    if (written) write_clean_streak_[i] = 0;
                    else if (write_clean_streak_[i] < 255) write_clean_streak_[i]++;
                }

                bool accessed = written;
#ifdef __linux__
                // Read detection via idle-page tracking: a page armed idle (in
                // escalateToDeepMonitoring) whose idle bit was CLEARED has been
                // read since — count that as an access, fault-free. soft-dirty
                // (accessed_) only covers writes; this adds the read signal.
                if (!accessed && idle_read_ok_ == 1 &&
                    st == PageState::ACTIVE_MONITORING) {
                    uint64_t pfn = pfnForPage(i);
                    if (pfn && !isPageIdle(pfn)) accessed = true;  // read since armed
                }
#endif
                if (accessed) {
                    cold_count_[i] = 0;
                    // Access can change contents — the incompressibility
                    // verdict behind the ratio-gate backoff is stale.
                    compress_fail_count_[i] = 0;
                    accessed_[i].store(false, std::memory_order_relaxed);
                } else {
                    if (cold_count_[i] < 255) cold_count_[i]++;
                    decayRecompressCount(i);
                }
            } else if (st == PageState::COMPRESSED) {
                // Keep counting for compressed pages so zstd upgrade can trigger
                if (cold_count_[i] < 255) cold_count_[i]++;
                decayRecompressCount(i);
            }
        });
    }

    // Slowly decay the per-page recompression count once a page settles into
    // a long cold streak. We trigger one decrement every kRcDecayColdTicks
    // ticks of inactivity (edge-detect on cold_count_), and a hard reset
    // when cold_count_ saturates — by that point the page has been quiet
    // for ~255 ticks and any prior thrash history is stale.
    inline void decayRecompressCount(size_t i) {
        uint8_t cc = cold_count_[i];
        if (cc == 255) {
            recompress_count_[i] = 0;
        } else if (cc >= kRcDecayColdTicks
                   && cc % kRcDecayColdTicks == 0
                   && recompress_count_[i] > 0) {
            recompress_count_[i]--;
        }
    }

    // Phase 2: Compress cold pages
    //
    // Two-level monitoring: Phase 3 sets pages to PROT_READ, which detects
    // writes but not reads.  A read-hot page (e.g., YCSB Workload B: 95%
    // reads) would appear cold and get compressed, only to be immediately
    // decompressed on the next read — wasteful churn.
    //
    // To avoid this, when a page first reaches the cold threshold we
    // escalate it from PROT_READ to PROT_NONE (deep monitoring) instead
    // of compressing.  Under PROT_NONE any access (read or write) triggers
    // the fault handler, which sets accessed_[]=true.  If the page
    // survives one full tick at PROT_NONE without any access, it is truly
    // cold and Phase 2 compresses it on the next tick.
    __attribute__((cold)) void phase2Range(int worker_id, size_t start, size_t end) {
        const ROIConfig& cfg = ROIConfig::instance();
        uint32_t floor = cfg.cold_ticks_floor;
        bool backoff_enabled = cfg.recompress_backoff;

        // No CPU-pressure multiplier and no per-tick compression budget.
        // The right signal for "don't compress this page yet" isn't
        // CPU saturation — it's "the program is still allocating into
        // this page." Span::allocate() resets cold_count_ for the page
        // it just handed out a chunk on, so a page being filled simply
        // never reaches the floor. Recompression-thrash is handled
        // independently by the per-page rc backoff below.
        forEachLivePage(start, end, [&](size_t i) {
            if (cold_count_[i] < floor) return;
            // Never compress the page(s) backing the C `environ` array. A
            // fork+exec child (Python subprocess.Popen / hlo2penguin) runs
            // getenv() in the child, which walks `environ` — but a raw fork()
            // bypasses pthread_atfork (so smash's child handler never runs) and
            // the child has no fault handler, so if the compressor had put the
            // environ page in COMPRESSED (PROT_NONE) state the child SIGSEGVs
            // in getenv (diagnosed 2026-07-18 from a full-mode core: environ
            // read as a zero/absent page). Excluding this single hot page is
            // negligible for compression and keeps every fork child's getenv
            // valid. environ moves when setenv/putenv reallocs it, so the index
            // is refreshed each tick in refreshEnvironExclusion().
            if (i == environ_page_idx_) return;
            PageState st = states_->get(i);

            // Tiered recompression: COMPRESSED pages at the LZ4 tier
            // (initial fast tier) that have stayed cold long enough get
            // upgraded to zstd-9 for a better ratio.  cold_count_ keeps
            // incrementing for COMPRESSED pages in phase1Range, so this
            // threshold is reliable.
            if constexpr (kTieredRecompression) {
                // Deep (zstd-9) upgrade of an already-compressed fast blob.
                // Gated off by default (kDeepTierEnabled): measured to win
                // 0/111810 upgrades on real heap pages (z9/z1 ~1.0), so it is
                // pure wasted CPU. The spill trigger below is independent and
                // still runs.
                if (kDeepTierEnabled &&
                    st == PageState::COMPRESSED &&
                    cold_count_[i] >= cfg.very_cold_ticks &&
                    page_tier_ && page_tier_[i] == kTierFast) {
                    tier_upgrade_attempts_.fetch_add(1,
                        std::memory_order_relaxed);
                    // Pick ZSTD_DICT if a dict is trained for this size
                    // class, else plain ZSTD at deep level.
                    uint8_t sc = lookupSizeClass(i);
                    CompressAlgo target = CompressAlgo::ZSTD;
                    if (engine_ && sc < kNumClasses &&
                        engine_->hasDictionary(sc)) {
                        target = CompressAlgo::ZSTD_DICT;
                    }
                    if (recompressPage(i, workers_[worker_id], target,
                                       kZstdDeepLevel)) {
                        tier_upgrade_success_.fetch_add(1,
                            std::memory_order_relaxed);
                    }
                    // Regardless of upgrade outcome, this slot was visited
                    // — no further work this tick for this page.
                }
                // File-backed spill (SMASH_SPILL): once a page is very-cold and
                // its tier is resolved (deep, or fast-tried-and-kept), relocate
                // its blob from anonymous RAM into the spill file so the kernel
                // can evict it as clean page cache. Decoupled from the zstd-9
                // upgrade on purpose: the upgrade's 80%-savings gate rejects the
                // large majority of pages (measured: 0/131072 upgrades succeed
                // on a realistic cold set), so gating spill on a successful
                // upgrade would almost never fire. Age + terminal-tier is the
                // right trigger.
                // Terminal tiers: deep-enabled builds spill kTierDeep/
                // kTierFastTried; single-tier (deep disabled) builds never
                // leave kTierFast, so accept it as terminal there.
                if (cfg_spill_ && st == PageState::COMPRESSED &&
                    cold_count_[i] >= cfg.very_cold_ticks && page_tier_ &&
                    (page_tier_[i] == kTierDeep || page_tier_[i] == kTierFastTried ||
                     (!kDeepTierEnabled && page_tier_[i] == kTierFast))) {
                    maybeSpillPage(i);
                }
                if (st == PageState::COMPRESSED || st == PageState::COMPRESSING)
                    return;
            } else if (st != PageState::ACTIVE && st != PageState::ACTIVE_MONITORING) {
                return;
            }
            if (st != PageState::ACTIVE && st != PageState::ACTIVE_MONITORING)
                return;

            // P3: persisted profile fast-path skip. If a prior run marked
            // this (arena, size_class) bucket as ALWAYS_SKIP (mean ratio
            // < 5%), don't even attempt compression — saves the per-page
            // lock acquisition + state CAS + ROI evaluation.
            //
            // SMASH_TIME_BUDGET_PCT: same short-circuit, driven by the
            // marginal-efficiency recompute. budget_decision_hint == SKIP
            // means this bucket fell below the budget cutoff. The check is
            // a single byte load before the per-page lock acquisition, so
            // rejected buckets cost ~zero.
            if (page_map_) {
                void* page_addr_p = vm_->pageAddress(i);
                Span* sp_p = page_map_->get(reinterpret_cast<uintptr_t>(page_addr_p));
                if (sp_p) {
                    // Unified bucket: slab span uses size_class, large span
                    // uses log2(page_count) slot above kNumClasses. Drops the
                    // legacy is_large gate so large allocs are budget-managed
                    // — load-bearing for SMASH_LARGE_ONLY workloads where
                    // every compressed page is large.
                    uint8_t sc_p = sp_p->is_large
                        ? largeSizeClass(sp_p->page_count)
                        : sp_p->size_class;
                    if (sc_p < kTotalBucketsPerArena) {
                        size_t bidx_p = statsIndex(sp_p->arena_id, sc_p);
                        if (bidx_p < kBucketTableLen) {
                            auto& bs = workers_[worker_id].sc_stats[bidx_p];
                            if (bs.budget_decision_hint == SizeClassStats::kBudgetSkip) {
                                return;
                            }
                            if (bs.persist_hint == 1) {
                                return;
                            }
                        }
                    }
                }
            } else {
                // External page (no Span). Check profile first: if a prior run
                // recorded that external pages were hot, skip ALL external pages.
                const size_t contig_pages = vm_->contigPages();
                if (i >= contig_pages) {
                    // Profile-driven skip: external pages were hot in prior run.
                    if (external_pages_hot_from_profile_) return;
                    // Once burned, twice shy: if page thrashed THIS run, skip it.
                    if (recompress_count_[i] > 0) return;
                }
            }

            // v6 profile: adjust the cold floor per the prior run's behavior
            // for this (arena, size_class) bucket:
            //   - stable_cold (thrash < 10% last run): pages stay cold, so we
            //     don't need to re-prove it — compress sooner (floor / 4).
            //   - high thrash (compress→fault→decompress churn last run):
            //     compressing these was wasted CPU that the app immediately
            //     undid. Defer aggressively so they must stay quiescent much
            //     longer before becoming eligible. The multiplier scales with
            //     the recorded thrash rate (0-255): ≥50% thrash → 8× floor,
            //     ≥25% → 4×, ≥12% → 2×. Gated by SMASH_PROFILE_THRASH_BACKOFF
            //     (default ON; =0 restores the prior behavior for A/B testing).
            // stable_cold and high-thrash are mutually exclusive by
            // construction (stable_cold requires thrash < 26/255 ≈ 10%).
            uint32_t profile_floor = floor;
            if (page_map_) {
                void* page_addr_pf = vm_->pageAddress(i);
                Span* sp_pf = page_map_->get(reinterpret_cast<uintptr_t>(page_addr_pf));
                if (sp_pf) {
                    uint8_t sc_pf = sp_pf->is_large
                        ? largeSizeClass(sp_pf->page_count)
                        : sp_pf->size_class;
                    if (sc_pf < kTotalBucketsPerArena) {
                        size_t bidx_pf = statsIndex(sp_pf->arena_id, sc_pf);
                        if (bidx_pf < kBucketTableLen) {
                            const auto& pf = workers_[worker_id].sc_stats[bidx_pf];
                            if (pf.profile_stable_cold) {
                                profile_floor = std::max(1u, floor / 4);
                            } else if (profileThrashBackoffEnabled()) {
                                uint8_t tr = pf.profile_thrash_rate;
                                uint32_t mult = 1;
                                if (tr >= 128) mult = 8;       // ≥50% thrash
                                else if (tr >= 64) mult = 4;   // ≥25%
                                else if (tr >= 31) mult = 2;   // ≥12%
                                if (mult > 1) {
                                    uint64_t wf = static_cast<uint64_t>(floor) * mult;
                                    profile_floor = wf > kMaxEffectiveFloorTicks
                                        ? kMaxEffectiveFloorTicks
                                        : static_cast<uint32_t>(wf);
                                }
                            }
                        }
                    }
                }
            }

            // Recompression-thrash gate. Pages that have been faulted back
            // from COMPRESSED before, or that live in a (arena, size_class)
            // bucket whose pages are doing the same, must stay idle longer
            // before becoming eligible again. Effective floor doubles per
            // accumulated recompress event, capped at kMaxBackoffShift.
            //
            // Looked up via the span so we also pick up the new-page bias
            // from the per-bucket EMA — fresh pages from a known-thrashy
            // call site inherit the wait without needing their own history.
            uint32_t eff_floor = profile_floor;
            if (backoff_enabled) {
                uint8_t rc = recompress_count_[i];
                uint8_t bucket_bias = 0;
                if (page_map_) {
                    void* page_addr = vm_->pageAddress(i);
                    Span* sp = page_map_->get(reinterpret_cast<uintptr_t>(page_addr));
                    if (sp) {
                        uint8_t sc_b = sp->is_large
                            ? largeSizeClass(sp->page_count)
                            : sp->size_class;
                        if (sc_b < kTotalBucketsPerArena) {
                            size_t bidx = statsIndex(sp->arena_id, sc_b);
                            if (bidx < kBucketTableLen) {
                                // Bucket bias: a single recompress event in this
                                // (arena, bucket) gives bias=1 to fresh pages in
                                // the same bucket — they get a longer initial
                                // cold window without needing to thrash themselves
                                // first. Threshold=256 ≈ "one full rc sample"
                                // with α=1/4 EMA.
                                uint16_t ema = bucket_rc_ema_x256_[bidx].load(
                                    std::memory_order_relaxed);
                                if (ema >= kBucketRcBiasThreshold_x256) bucket_bias = 1;
                            }
                        }
                    }
                }
                // Ratio-gate failures back off through the same shift: each
                // failed attempt doubles the idle streak required before the
                // next try (see compressPage's gate-failure path, which also
                // resets cold_count_ so the streak restarts from zero).
                uint32_t fails = compress_fail_count_
                    ? compress_fail_count_[i] : 0;
                uint32_t shift32 = static_cast<uint32_t>(rc) + bucket_bias + fails;
                if (shift32 > kMaxBackoffShift) shift32 = kMaxBackoffShift;
                uint64_t wide_floor = static_cast<uint64_t>(profile_floor) << shift32;
                if (wide_floor > kMaxEffectiveFloorTicks)
                    wide_floor = kMaxEffectiveFloorTicks;
                eff_floor = static_cast<uint32_t>(wide_floor);
            }

            if (cold_count_[i] == profile_floor && st == PageState::ACTIVE_MONITORING) {
                // Always escalate to deep monitoring at the BASE floor —
                // this just changes protection, not compression. It gives
                // the compressor more accurate access info for thrashy
                // pages (which we won't compress anyway until eff_floor).
                escalateToDeepMonitoring(i);
            } else if (cold_count_[i] > profile_floor && cold_count_[i] >= eff_floor) {
                // Page is eligible for compression — count it
                worker_pages_eligible_[worker_id].fetch_add(1, std::memory_order_relaxed);
                if (compressPage(i, workers_[worker_id])) {
                    worker_pages_compressed_[worker_id].fetch_add(1, std::memory_order_relaxed);
                    // Periodic mid-range flush so the pending list
                    // doesn't get too long and so the cap is never hit
                    // inside a hot inner loop.
                    if (workers_[worker_id].pending_pn_count >=
                        CompressWorker::kPendingProtCap / 2) {
                        flushPhase2PendingProt(worker_id);
                    }
                }
            }
        });
        // End-of-range flush.
        flushPhase2PendingProt(worker_id);
    }

    // Apply (mprotect PROT_NONE + decommit) to a worker's accumulated
    // list of just-COMPRESSED pages. Each page is locked individually
    // (lock → check state → mprotect → decommit → unlock) to avoid
    // holding multiple per-page locks simultaneously during mprotect.
    //
    // Why single-page: holding locks across a batch of pages while
    // calling mprotect creates a deadlock with the fault handler.
    // mprotect issues a TLB shootdown IPI; if an app thread on another
    // core faults on a different page in the same batch, its fault
    // handler spins on that page's lock (held by us), while our mprotect
    // waits for the TLB shootdown ack from that core. Circular wait.
    //
    // The per-page approach is safe: lock ensures handleFault cannot
    // transition COMPRESSED → ACTIVE between our state check and the
    // mprotect+decommit. The cost is one mprotect syscall per page
    // instead of one per consecutive run — on Linux, mprotect on a
    // single page within a larger VMA is fast (no VMA split needed
    // when the page is already independently mapped via prior faults).
    void flushPhase2PendingProt(int worker_id) {
        auto& w = workers_[worker_id];
        if (w.pending_pn_count == 0) return;

        for (size_t i = 0; i < w.pending_pn_count; ++i) {
            size_t page_idx = w.pending_pn_pages[i];
            locks_->lock(page_idx);

            if (states_->get(page_idx) != PageState::COMPRESSED) {
                locks_->unlock(page_idx);
                continue;
            }

            void* addr = vm_->pageAddress(page_idx);
            if (!addr || !vm_->contains(reinterpret_cast<uintptr_t>(addr))) {
                locks_->unlock(page_idx);
                continue;
            }

            // Set state to COMPRESSED. The page is still PROT_RW (or
            // PROT_READ from monitoring) — app accesses succeed without
            // faulting. The deferred-madvise sweeper will later (under
            // tryLock + state re-check) apply mprotect(PROT_NONE) and
            // madvise(DONTNEED). This avoids the deadlock where mprotect
            // under lock triggers a fault on an app thread that blocks in
            // handleFault on the same lock, AND the TOCTOU race where
            // unlocking then mprotecting hits a page that handleFault
            // already decompressed.
            deferMadvise(page_idx);
            locks_->unlock(page_idx);
        }
        w.pending_pn_count = 0;
    }

    // Escalate a PROT_READ-monitored page to PROT_NONE (deep monitoring).
    // Any access (read or write) will now trigger the fault handler.
    //
    // Race avoidance: another worker can be inside compressPage between
    // its protectPages(PROT_READ) and its memcpy of page_addr at the
    // moment we get here. If we mprotect PROT_NONE without holding the
    // per-page lock, the compressor thread that holds it SIGSEGVs on its
    // own memcpy. handleFault detects self-recursion via the reentrancy
    // guard and bails, the SIGSEGV chains to SIG_DFL, and the process
    // dies. So acquire the lock and re-verify state before mprotecting.
    void escalateToDeepMonitoring(size_t page_idx) {
#ifdef __linux__
        // Fault-free read detection: if idle-page tracking is usable, mark the
        // page idle instead of arming PROT_NONE. The next READ clears the idle
        // bit in the PTE (no fault); phase2's read-cold check tests it. This
        // avoids both the mprotect arm here and the per-read SIGSEGV. Only taken
        // when the privileged probe passed — otherwise falls through to the
        // legacy PROT_NONE path below, byte-identical to before.
        if (idle_read_ok_ == 1) {
            uint64_t pfn = pfnForPage(page_idx);
            if (pfn) { markPageIdle(pfn); return; }
            // PFN vanished (page swapped/migrated) → fall through to PROT_NONE.
        }
#endif
#if defined(__APPLE__)
        // macOS + deferred-reclaim: skip the PROT_NONE deep-monitoring arm.
        //
        // On macOS, mprotect(PROT_NONE) does NOT reclaim physical pages (only
        // MADV_FREE_REUSABLE does, and that requires PROT_RW) — but task_info's
        // resident_size DROPS anyway, a pure reporting artifact. In the default
        // deferred-reclaim path the very next tick compresses the page, which
        // flips it back to PROT_RW to snapshot it and then HOLDS it PROT_RW for
        // the whole COMPRESSED_SHADOW window before Phase B does the real
        // MADV_FREE_REUSABLE reclaim. Net effect on the RSS graph: a phantom
        // drop at the escalation tick, an immediate re-inflate at the compress
        // tick, then the true drop at Phase B — the "bounce" the user saw.
        //
        // The escalation's only purpose is pre-compression READ detection. In
        // deferred-reclaim mode that value is near-zero: the shadow page is kept
        // PROT_RW, so reads during shadow never fault, and Phase B classifies
        // survival by content-compare (writes), not reads. Dropping the arm here
        // removes the phantom flip and yields a monotonic RSS curve; a genuinely
        // read-hot page is still caught by the fault handler once Phase B has
        // reclaimed it. Non-deferred mode keeps the arm (it reclaims eagerly, so
        // there is no shadow re-inflate to avoid).
        if (isDeferredReclaimMode()) return;
#endif
        locks_->lock(page_idx);
        if (states_->get(page_idx) == PageState::ACTIVE_MONITORING) {
            void* page_addr = vm_->pageAddress(page_idx);
            vm::protectPages(page_addr, kPageSize, false, false);  // PROT_NONE
        }
        locks_->unlock(page_idx);
    }

    // Phase 3: Set up access monitoring for remaining active pages.
    //
    // Chunked: scan the worker's range for maximal runs of contiguous
    // ACTIVE pages, batch the per-page lock acquisitions, do ONE mprotect
    // call for the whole run. This is the principal lever to keep VMA
    // count under /proc/sys/vm/max_map_count on long compiles — per-page
    // mprotect creates one VMA boundary per page, which on multi-GB
    // workloads (e.g. neuron-cc cut_batch-norm-training_33 with ~1M
    // pages compressed) blows past 65530 boundaries and starts failing
    // mprotect() with ENOMEM.
    //
    // Chunk cap (kProtectChunkPages) is the max pages per mprotect call.
    // Larger = fewer VMAs but more lock contention. 64 is a good default:
    // 64 × 4 KiB = 256 KiB per chunk, ~16K chunks for 1M pages, well
    // under the VMA cap.
    //
    // Syscalls touching a transiently PROT_READ page get EFAULT instead of
    // a SIGSEGV; the wrapper's retryWithDecompress loop catches that, walks
    // the buffer pages (which DOES go through the fault handler), and
    // retries. The bounded retry budget (8) outlasts a compressor tick.
    // Maximum pages per chunked mprotect call. Larger = fewer TLB-shootdown
    // IPIs and VMA splits, but longer per-chunk lock-hold (more contention
    // with handleFault). Default 16 (= 64 KiB). Runtime-tunable via
    // SMASH_PROTECT_CHUNK_PAGES so the IPI/lock-hold tradeoff can be swept
    // without a rebuild; clamped to [1, kPendingProtCap].
    static size_t protectChunkPages() {
        static const size_t v = []() -> size_t {
            const char* e = std::getenv("SMASH_PROTECT_CHUNK_PAGES");
            if (e) {
                long n = atol(e);
                if (n >= 1 && n <= static_cast<long>(CompressWorker::kPendingProtCap))
                    return static_cast<size_t>(n);
            }
            return kProtectChunkPagesDefault;
        }();
        return v;
    }
    static constexpr size_t kProtectChunkPagesDefault = 16;
    static constexpr size_t kProtectChunkPages = 16;  // legacy alias (escalate path)

    // Apply mprotect(PROT_READ) to pages that successfully CAS'd to
    // ACTIVE_MONITORING. Unlock before mprotect to avoid deadlock:
    // mprotect(PROT_READ) can trigger a write-fault on an app thread that
    // then blocks in handleFault on the same per-page lock.
    void flushPhase3Run(size_t run_start, size_t run_end) {
        if (run_end <= run_start) return;
        bool any_ok = false;
        for (size_t k = run_start; k < run_end; ++k) {
            locks_->lock(k);
            if (states_->get(k) != PageState::ACTIVE_MONITORING) {
                locks_->unlock(k);
                continue;
            }
            // State is ACTIVE_MONITORING — unlock before mprotect.
            // If mprotect triggers a write-fault on another thread,
            // handleFault sees ACTIVE_MONITORING and restores PROT_RW.
            locks_->unlock(k);

            void* page_addr = vm_->pageAddress(k);
            if (!page_addr || !vm::protectPages(page_addr, kPageSize, true, false)) {
                // mprotect failed — revert state under lock
                locks_->lock(k);
                if (states_->get(k) == PageState::ACTIVE_MONITORING)
                    states_->set(k, PageState::ACTIVE);
                locks_->unlock(k);
            } else {
                any_ok = true;
            }
        }
        if (any_ok) protReadBarrier();
    }

    void phase3Range(size_t start, size_t end) {
        // Use forEachLivePage to skip empty pages. For each live page,
        // CAS to ACTIVE_MONITORING; accumulate a run of consecutive
        // successes and flush via a single mprotect when the run breaks
        // (non-contiguous page index, CAS failure, or chunk size cap).
        size_t run_start = ~size_t{0};
        size_t run_end = 0;
        // Get contig_pages count to identify external pages
        const size_t contig_pages = vm_->contigPages();
        forEachLivePage(start, end, [&](size_t i) {
            // Skip external pages — they're tracked for compression but
            // monitoring them (PROT_READ) interferes with application use.
            // External pages have indices >= contig_pages.
            if (i >= contig_pages) return;

            // Never PROT_READ-arm the environ page (see refreshEnvironExclusion
            // / phase2Range): a fork child's getenv() must always find it
            // readable. Break the run so the flush before/after stays correct.
            if (i == environ_page_idx_) {
                if (run_end > run_start && run_start != ~size_t{0}) {
                    flushPhase3Run(run_start, run_end);
                }
                run_start = ~size_t{0};
                run_end = 0;
                return;
            }

            if (!states_->transition(i, PageState::ACTIVE,
                                        PageState::ACTIVE_MONITORING)) {
                // Run breaks here — flush whatever we accumulated.
                if (run_end > run_start && run_start != ~size_t{0}) {
                    flushPhase3Run(run_start, run_end);
                }
                run_start = ~size_t{0};
                run_end = 0;
                return;
            }
            // CAS succeeded. Extend the run if contiguous and under cap.
            if (run_start == ~size_t{0}) {
                run_start = i;
                run_end = i + 1;
            } else if (i == run_end &&
                       (run_end - run_start) < kProtectChunkPages) {
                run_end = i + 1;
            } else {
                // Discontinuity or cap reached. Flush previous run then
                // start fresh from i.
                flushPhase3Run(run_start, run_end);
                run_start = i;
                run_end = i + 1;
            }
        });
        if (run_end > run_start && run_start != ~size_t{0}) {
            flushPhase3Run(run_start, run_end);
        }
    }

    // ── Compress one page using worker's contexts ─────────────────────────

    __attribute__((cold)) bool compressPage(size_t page_idx, CompressWorker& worker) {
        const bool deferred = isDeferredReclaimMode();

        if (deferred) {
            if (!locks_->tryLock(page_idx)) return false;
        } else {
            locks_->lock(page_idx);
        }

        // Verify still eligible
        PageState st = states_->get(page_idx);
        if (st != PageState::ACTIVE && st != PageState::ACTIVE_MONITORING) {
            locks_->unlock(page_idx);
            return false;
        }

        // Mark as compressing
        states_->set(page_idx, PageState::COMPRESSING);

        // ROI-based compression decision (replaces fixed shouldSkip + selectAlgorithm).
        // Stats are per (arena_id, size_class) so the ROI model sees the
        // per-origin compression ratio, consistent with the arena design.
        uint8_t arena_id = 0, sc = 0;
        bool have_span = lookupSpanInfo(page_idx, arena_id, sc);
        size_t stats_idx = statsIndex(arena_id, sc);
        uint8_t stats_count = (have_span && sc < kTotalBucketsPerArena)
            ? worker.sc_stats[stats_idx].count : 0;
        uint16_t stats_sum = (have_span && sc < kTotalBucketsPerArena)
            ? worker.sc_stats[stats_idx].sum : 0;

        // Soft-dirty ROI: discount the compression benefit by the estimated
        // probability the blob survives un-rewritten. The bucket's learned
        // re-dirty EMA is the prior; a long per-page WRITE-clean streak relieves
        // it (this specific page has proven write-stable even if its bucket is
        // churny on average — the "both" per-page-evidence + per-bucket-prior
        // design). discount==256 (no effect) when the gate is off or the bucket
        // lacks data, so behavior is identical to before in those cases.
        uint32_t redirty_discount_x256 = 256;
        if (cfgSoftdirtyRoi() && have_span && sc < kTotalBucketsPerArena) {
            uint32_t streak_relief = 0;
            if (write_clean_streak_) {
                // Map streak (0..255 ticks) → relief (0..256). A page write-clean
                // for >= kWriteCleanFullRelief ticks gets full relief (discount
                // → 1.0): its own evidence dominates the bucket prior.
                uint8_t s = write_clean_streak_[page_idx];
                streak_relief = (s >= kWriteCleanFullRelief)
                    ? 256u
                    : (static_cast<uint32_t>(s) * 256u / kWriteCleanFullRelief);
            }
            redirty_discount_x256 =
                worker.sc_stats[stats_idx].redirtyDiscountX256(streak_relief);
        }

        if (!CompressionROI::shouldCompress(cold_count_[page_idx],
                                             stats_count, stats_sum,
                                             redirty_discount_x256)) {
            states_->set(page_idx, PageState::ACTIVE);
            locks_->unlock(page_idx);
            return false;
        }

        // Tier selection: either ROI cost/benefit model or UCB1-Tuned bandit.
        const auto& cfg = ROIConfig::instance();
        const AlgoProfile* profile = nullptr;
        int chosen_tier = -1;

        // v6 profile: use persisted best_tier as warm-start hint when we
        // don't have enough data yet. Only applies when UCB is active.
        uint8_t profile_best_tier = 255;
        if (have_span && sc < kTotalBucketsPerArena) {
            profile_best_tier = worker.sc_stats[stats_idx].profile_best_tier;
        }

        if (cfg.use_ucb && cfg.num_profiles > 1) {
            // Build per-arm view from this bucket's stats and the per-profile
            // min_cold_ticks gate.  UCB sees the action space directly; the
            // ROI model's calibrated cost estimates are unused.
            uint32_t pulls[4] = {0,0,0,0};
            double mean[4] = {0,0,0,0};
            double m2[4]   = {0,0,0,0};
            bool eligible[4] = {false,false,false,false};
            auto& s = worker.sc_stats[stats_idx];
            int n = cfg.num_profiles;
            if (n > SizeClassStats::kTiers) n = SizeClassStats::kTiers;
            for (int i = 0; i < n; ++i) {
                pulls[i] = s.arm_pulls[i];
                mean[i]  = s.arm_mean[i];
                m2[i]    = s.arm_m2[i];
                eligible[i] = (cold_count_[page_idx] >= cfg.profiles[i].min_cold_ticks);
            }

            // Force-deep override: every Nth selection in this bucket, if
            // the deep arm is eligible and exists, force-pick it to keep the
            // variance estimate of the deep tier live on heavy-tailed reward
            // distributions.  Cycling per-bucket avoids tick-aligned spikes.
            uint32_t sel_counter = ++s.selections;
            int deep_idx = (n > 1) ? 1 : -1;
            if (cfg.ucb_force_deep_every > 0 && deep_idx >= 0 &&
                eligible[deep_idx] &&
                (sel_counter % cfg.ucb_force_deep_every) == 0) {
                chosen_tier = deep_idx;
            } else if (cfg.ucb_warmstart && have_span) {
                // Warm-start: pass the per-arena aggregate as a prior so
                // under-explored bucket arms borrow strength from the arena
                // rather than starting from a uniform posterior.
                uint32_t pp[4] = {0,0,0,0};
                double   pm[4] = {0,0,0,0};
                double   pmm[4] = {0,0,0,0};
                const auto& a = worker.arena_arm[arena_id];
                int an = n < ArenaArmStats::kTiers ? n : ArenaArmStats::kTiers;
                for (int i = 0; i < an; ++i) {
                    pp[i] = a.pulls[i];
                    pm[i] = a.mean[i];
                    pmm[i] = a.m2[i];
                }
                chosen_tier = CompressionROI::selectTierUCB(
                    pulls, mean, m2, eligible, n,
                    cfg.ucb_min_pulls, cfg.ucb_variant,
                    pp, pm, pmm);
            } else {
                chosen_tier = CompressionROI::selectTierUCB(
                    pulls, mean, m2, eligible, n,
                    cfg.ucb_min_pulls, cfg.ucb_variant);
            }
            // v6 profile: if UCB chose nothing but we have a profiled best tier
            // from a prior run, use it as a warm-start (skip exploration phase).
            if (chosen_tier < 0 && profile_best_tier < static_cast<uint8_t>(n) &&
                eligible[profile_best_tier]) {
                chosen_tier = static_cast<int>(profile_best_tier);
            }
            if (chosen_tier >= 0) profile = &cfg.profiles[chosen_tier];
        } else {
            // Pass the bucket's observed per-tier compression costs into the
            // ROI model so it uses real workload data instead of synthetic
            // calibration when available.
            uint32_t observed_costs_us[2] = {0, 0};
            if (have_span && sc < kTotalBucketsPerArena) {
                observed_costs_us[0] = worker.sc_stats[stats_idx].observedCostUs(0);
                observed_costs_us[1] = worker.sc_stats[stats_idx].observedCostUs(1);
            }
            profile = CompressionROI::selectProfile(
                cold_count_[page_idx], stats_count, stats_sum,
                observed_costs_us);
            if (profile) {
                // Map back to a tier index (0=fast, 1=deep) so reward
                // attribution lines up with the UCB path even when UCB is off.
                chosen_tier = (profile == &cfg.profiles[0]) ? 0 : 1;
            }
        }
        if (!profile) {
            states_->set(page_idx, PageState::ACTIVE);
            locks_->unlock(page_idx);
            return false;
        }
        CompressAlgo algo = profile->algo;
        int zstd_level = profile->zstd_level;
        bool is_fast_tier = (zstd_level != 0 && zstd_level != kZstdDeepLevel)
                          || (algo == CompressAlgo::LZ4);
        // Prefer dictionary for deep-tier zstd if trained. Dictionaries are
        // keyed by real slab size_class only — large-alloc buckets sit above
        // kNumClasses and bypass the dict path.
        if (algo == CompressAlgo::ZSTD && zstd_level == kZstdDeepLevel &&
            engine_ && sc < kNumClasses && engine_->hasDictionary(sc)) {
            algo = CompressAlgo::ZSTD_DICT;
        }

        void* page_addr = vm_->pageAddress(page_idx);
        // External pages may have been untracked between state check and here
        if (!page_addr) {
            states_->set(page_idx, PageState::EMPTY);
            locks_->unlock(page_idx);
            return false;
        }

        if (deferred) {
            // Deferred mode: ensure the page is PROT_RW before the snapshot
            // read below. This mprotect is UNCONDITIONAL (was previously gated
            // on st==ACTIVE_MONITORING) because an ACTIVE page can also be
            // physically PROT_NONE here: escalateToDeepMonitoring arms
            // PROT_NONE for pre-compression read detection, and other
            // deferred-path producers can leave a still-backed page PROT_NONE
            // while its state reads ACTIVE. Reading such a page directly
            // SIGSEGVs the worker (handleFault's reentrancy guard re-raises it
            // fatally). The page is always still backed here (never madvise'd
            // away while ACTIVE), so restoring PROT_RW recovers the live
            // contents; a no-op mprotect on an already-RW page is cheap. This
            // also matches the non-deferred path, which does
            // unlock->mprotect(PROT_READ)->relock before its snapshot. We hold
            // the per-page lock; LOOSENING protection to PROT_RW cannot fault
            // an app thread, so there is no TLB-shootdown-vs-page-lock deadlock
            // (that hazard only applies to TIGHTENING protection).
            vm::protectPages(page_addr, kPageSize, true, true);  // PROT_RW
            __builtin_memcpy(worker.page_buf, page_addr, kPageSize);
            // Snapshot verification for deferred mode. Without PROT_READ
            // protection during the snapshot, a concurrent write can land
            // between our memcpy and the eventual PROT_NONE — the blob
            // would be stale and decompress-on-fault would silently revert
            // the writer's update. Re-read and compare; abort on mismatch.
            if (fixavEnabled()) {
                alignas(64) uint8_t verify_buf[kPageSize];
                __builtin_memcpy(verify_buf, page_addr, kPageSize);
                if (__builtin_memcmp(verify_buf, worker.page_buf, kPageSize) != 0) {
                    snapshot_verify_fails_.fetch_add(1, std::memory_order_relaxed);
                    states_->set(page_idx, PageState::ACTIVE);
                    locks_->unlock(page_idx);
                    return false;
                }
                snapshot_verify_passes_.fetch_add(1, std::memory_order_relaxed);
            }
#ifndef SMASH_ABLATION_NO_ZERO_DEFERRED
            zeroFreeSlots(worker.page_buf, page_idx);
#endif
        } else {
            // Standard mode. Make the page readable: it may be PROT_READ
            // (Phase 3 monitoring) or PROT_NONE (deep monitoring).
            // CRITICAL: unlock before mprotect to prevent deadlock.
            // mprotect issues a TLB-shootdown IPI; if an app thread on
            // another core faults (write to now-PROT_READ page), its fault
            // handler would block on this page's lock — while our mprotect
            // waits for the TLB shootdown ack from that core. Circular wait.
            // Re-check state after re-locking (page may have been freed).
            locks_->unlock(page_idx);
            vm::protectPages(page_addr, kPageSize, true, false);  // PROT_READ
            locks_->lock(page_idx);
            if (states_->get(page_idx) != PageState::COMPRESSING) {
                // State changed while unlocked — abort
                vm::protectPages(page_addr, kPageSize, true, true);  // restore PROT_RW
                locks_->unlock(page_idx);
                return false;
            }
            // Force any in-flight stores on remote cores to retire before
            // we snapshot. mprotect's IPI ack does NOT guarantee the
            // remote store buffers have drained; without this barrier,
            // a writer mid-store on another core can have its write
            // silently rolled back when the page is later
            // decompress-on-fault restored from this snapshot. Gated by
            // SMASH_PROT_READ_BARRIER to keep perf-default unchanged.
            // Under FixAv the barrier is correctness-mandatory; the
            // legacy env-gate becomes a no-op when SMASH_FIXAV is set.
            if (fixavEnabled()) membarrierSyncCore();
            else protReadBarrier();
            // Fast path: user-space memcpy. We hold the per-page lock
            // and just set PROT_READ; concurrent app accesses block in
            // handleFault on the same lock. Only fall back to
            // process_vm_readv (which converts faults to EFAULT instead
            // of SIGSEGV) if the SMASH_SAFE_MEMCPY=1 env knob is set,
            // for diagnostic / paranoid use. The previous default of
            // process_vm_readv added ~3 µs syscall overhead per page,
            // which on 28K pages = ~85 ms of pure syscall time per
            // tick — measurably worse wall time on
            // cut_reshape_8298.
            const bool safe_memcpy = cfg_safe_memcpy_;
#ifdef __linux__
            if (safe_memcpy) {
                struct iovec local_iov{worker.page_buf, kPageSize};
                struct iovec remote_iov{page_addr, kPageSize};
                ssize_t got = process_vm_readv(getpid(), &local_iov, 1,
                                               &remote_iov, 1, 0);
                if (got != static_cast<ssize_t>(kPageSize)) {
                    vm::protectPages(page_addr, kPageSize, true, true);
                    states_->set(page_idx, PageState::ACTIVE);
                    locks_->unlock(page_idx);
                    return false;
                }
            } else {
                __builtin_memcpy(worker.page_buf, page_addr, kPageSize);
            }
#else
            __builtin_memcpy(worker.page_buf, page_addr, kPageSize);
#endif
            // SMASH_SNAPSHOT_VERIFY=1: defends against post-snapshot store
            // retirement. After mprotect(PROT_READ) + (optional) membarrier,
            // a writer's store that was issued under PROT_RW can still
            // retire AFTER our snapshot completes (the store was legal at
            // issue time, the wbuf is just draining late). When such a
            // store retires before mprotect(PROT_NONE) but after the
            // snapshot memcpy, our snapshot is stale: a later
            // decompress-on-fault will silently roll back the writer's
            // update. We re-read the page (still PROT_READ, lock still
            // held) and compare against the snapshot. If they differ, a
            // store retired in our window — abandon this attempt, restore
            // PROT_RW, reset state to ACTIVE; next tick can retry.
            const bool snapshot_verify = cfg_snapshot_verify_;
            // Snapshot-verify is correctness-mandatory under FixAv (it is the
            // sole defense against a writer's store retiring inside our
            // PROT_READ window). The legacy SMASH_SNAPSHOT_VERIFY env var
            // remains as an independent opt-in for non-FixAv runs.
            if (snapshot_verify || fixavEnabled()) {
                alignas(64) uint8_t verify_buf[kPageSize];
                __builtin_memcpy(verify_buf, page_addr, kPageSize);
                if (__builtin_memcmp(verify_buf, worker.page_buf, kPageSize) != 0) {
                    snapshot_verify_fails_.fetch_add(1, std::memory_order_relaxed);
                    vm::protectPages(page_addr, kPageSize, true, true);  // PROT_RW
                    states_->set(page_idx, PageState::ACTIVE);
                    locks_->unlock(page_idx);
                    return false;
                }
                snapshot_verify_passes_.fetch_add(1, std::memory_order_relaxed);
            }
#ifndef SMASH_ABLATION_NO_ZERO_DEFERRED
            zeroFreeSlots(worker.page_buf, page_idx);
#endif
            // P2 chunked: defer both the decommit and the mprotect to
            // the batch flush at end of phase2Range. The page sits at
            // PROT_READ with original contents. App reads return correct
            // data (matches the compressed blob); app writes fault →
            // handleFault → state COMPRESSED → decompress + restore.
            // The end-of-phase batch chunked-mprotects (PROT_NONE) and
            // chunked-decommits runs of contiguous still-COMPRESSED
            // pages, reducing VMA boundaries by ~kProtectChunkPages×.

            if (states_->get(page_idx) != PageState::COMPRESSING) {
                locks_->unlock(page_idx);
                return false;
            }
        }

        // All-zero fast path. zeroFreeSlots() above zeroes every freed slot
        // in the snapshot, so a page whose live objects have all been freed
        // — plus any page the application itself zeroed — is entirely zero
        // here. Compressing it wastes CPU and a CompressStore slot (the
        // blob's live bytes also keep its 16 MiB region from ever draining
        // empty and being decommitted). Record a no-blob sentinel instead:
        // data=nullptr, comp_size=0, algo=NONE. Every decompress path
        // funnels through CompressEngine::decompress / decompressWithDCtx,
        // whose NONE case restores the page with memset(0). Detection is
        // one pass of 64-bit loads over the snapshot; non-zero pages exit
        // at the first non-zero word.
        const bool zero_page = pageIsAllZero(worker.page_buf);

        // Collect sample for dictionary training (page data in worker's buf)
        if (!zero_page) collectDictSample(page_idx, sc, worker.page_buf);

        size_t max_comp = CompressEngine::maxCompressedSizeAny(kPageSize);
        size_t comp_size = 0;
        auto comp_t0 = std::chrono::steady_clock::now();

        // Try-both experiment (opt-in): when dict is selected, also try plain
        // ZSTD and keep the smaller result. Doubles compression CPU for
        // very-cold pages, so only enable for experiments.
        if (zero_page) {
            // Zero page: skip the codec entirely; the sentinel recorded
            // below carries all the information.
        }
#ifdef SMASH_DICT_TRY_BOTH
        else if (algo == CompressAlgo::ZSTD_DICT && worker.compress_buf2) {
            size_t dict_size = worker.compress(worker.page_buf, worker.compress_buf,
                                               kPageSize, max_comp, CompressAlgo::ZSTD_DICT,
                                               sc, zstd_level, engine_);
            size_t plain_size = worker.compress(worker.page_buf, worker.compress_buf2,
                                                kPageSize, max_comp, CompressAlgo::ZSTD,
                                                sc, zstd_level, engine_);
            if (dict_size > 0 && plain_size > 0) {
                int64_t delta = static_cast<int64_t>(plain_size) - static_cast<int64_t>(dict_size);
                dict_total_delta_.fetch_add(delta, std::memory_order_relaxed);
                if (dict_size < plain_size) {
                    dict_win_count_.fetch_add(1, std::memory_order_relaxed);
                    comp_size = dict_size;
                } else if (dict_size > plain_size) {
                    dict_loss_count_.fetch_add(1, std::memory_order_relaxed);
                    algo = CompressAlgo::ZSTD;
                    comp_size = plain_size;
                    void* tmp = worker.compress_buf;
                    worker.compress_buf = worker.compress_buf2;
                    worker.compress_buf2 = tmp;
                } else {
                    dict_tie_count_.fetch_add(1, std::memory_order_relaxed);
                    comp_size = dict_size;
                }
            } else {
                comp_size = dict_size ? dict_size : plain_size;
                if (!dict_size && plain_size) {
                    algo = CompressAlgo::ZSTD;
                    void* tmp = worker.compress_buf;
                    worker.compress_buf = worker.compress_buf2;
                    worker.compress_buf2 = tmp;
                }
            }
        }
#endif
        else {
            comp_size = worker.compress(worker.page_buf, worker.compress_buf,
                                        kPageSize, max_comp, algo, sc,
                                        zstd_level, engine_);
        }

        double min_ratio = ROIConfig::instance().min_compress_ratio;

        // If fast tier failed the ratio gate, fall back to deep compression.
        // zstd-9 often compresses 30-50% better than zstd-1/LZ4 due to
        // better match finding and larger search window.
        // Gated off by default (kDeepTierEnabled): on real heap pages zstd-9
        // ~= zstd-1 (z9/z1 blob ratio ~1.0), so a page that fails the gate at
        // zstd-1 almost always fails it at zstd-9 too — the retry is wasted
        // CPU with no RSS gain (measured: ~32% of rocksdb pages escalate, 0
        // net RSS benefit). Single-tier leaves such pages uncompressed.
        if (!zero_page && kDeepTierEnabled && is_fast_tier &&
            (comp_size == 0 || comp_size > static_cast<size_t>(kPageSize * min_ratio))) {
            algo = CompressAlgo::ZSTD;
            zstd_level = kZstdDeepLevel;
            is_fast_tier = false;
            comp_size = worker.compress(worker.page_buf, worker.compress_buf,
                                        kPageSize, max_comp, algo, sc,
                                        zstd_level, engine_);
        }

        auto comp_t1 = std::chrono::steady_clock::now();
        uint32_t comp_elapsed_us = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                comp_t1 - comp_t0).count());

        if (!zero_page &&
            (comp_size == 0 || comp_size > static_cast<size_t>(kPageSize * min_ratio))) {
            // Not worth compressing; record poor ratio and cost for the
            // tier we actually ran, then restore the page.
            if (have_span && sc < kTotalBucketsPerArena) {
                worker.sc_stats[stats_idx].record(
                    comp_size ? comp_size : kPageSize, kPageSize);
                int tier = is_fast_tier ? 0 : 1;
                worker.sc_stats[stats_idx].recordCost(tier, comp_elapsed_us);
                // Time-budget: failed-compress cost still counts against
                // the budget (we did spend cycles). bytes_saved unchanged.
                worker.sc_stats[stats_idx].time_cost_total_us.fetch_add(
                    comp_elapsed_us, std::memory_order_relaxed);
                if (cfg.use_ucb && chosen_tier >= 0) {
                    worker.sc_stats[stats_idx].recordReward(chosen_tier, 0.0);
                    if (cfg.ucb_warmstart && arena_id < kNumArenas)
                        worker.arena_arm[arena_id].recordReward(chosen_tier, 0.0);
                }
            }
            if (!deferred) {
                if (!vm::protectPages(page_addr, kPageSize, true, true)) {
                    vm::remapPages(page_addr, kPageSize, true, true);
                }
                __builtin_memcpy(page_addr, worker.page_buf, kPageSize);
            }
            // Ratio-gate backoff. Without it, an idle incompressible page
            // stays at cold_count_ >= floor and is re-selected EVERY tick:
            // ~10 µs of mprotect + snapshot + codec + restore per tick,
            // forever. Consume the cold streak (restart from zero) and bump
            // the per-page fail count, which doubles the effective floor
            // for the next attempt (see phase2Range). The count resets only
            // on a real access — an idle page's contents cannot change, so
            // retries stay capped at one per kMaxEffectiveFloorTicks streak.
            if (compress_fail_count_ && compress_fail_count_[page_idx] < 255)
                compress_fail_count_[page_idx]++;
            cold_count_[page_idx] = 0;
            states_->set(page_idx, PageState::ACTIVE);
            locks_->unlock(page_idx);
            return false;
        }

        // Store compressed data (sharded by page_idx). Zero pages store
        // nothing — the sentinel recorded below carries all the information.
        size_t alloc_size = 0;
        void* stored = nullptr;
        if (!zero_page) {
            stored = store_->store(worker.compress_buf, comp_size, &alloc_size, page_idx);
            if (!stored) {
                if (!deferred) {
                    if (!vm::protectPages(page_addr, kPageSize, true, true)) {
                        vm::remapPages(page_addr, kPageSize, true, true);
                    }
                    __builtin_memcpy(page_addr, worker.page_buf, kPageSize);
                }
                states_->set(page_idx, PageState::ACTIVE);
                locks_->unlock(page_idx);
                return false;
            }
        }

        // Record successful compression ratio and observed cost in the
        // per-(arena, bucket) bucket so future ROI decisions use real data.
        // Zero pages are excluded: they say nothing about the codec's
        // behavior on this bucket's live data and would skew the ROI model.
        if (!zero_page && have_span && sc < kTotalBucketsPerArena) {
            worker.sc_stats[stats_idx].record(comp_size, kPageSize);
            int tier = is_fast_tier ? 0 : 1;
            worker.sc_stats[stats_idx].recordCost(tier, comp_elapsed_us);
            // Time-budget: bytes saved this page = (kPageSize - comp_size).
            // Cost is the wall time we spent. Both are cumulative since
            // process start; recomputeMarginalEfficiency reads them and
            // computes per-bucket bytes/µs.
            size_t saved = (comp_size < kPageSize)
                ? (kPageSize - comp_size) : 0;
            worker.sc_stats[stats_idx].bytes_saved_total.fetch_add(
                saved, std::memory_order_relaxed);
            worker.sc_stats[stats_idx].time_cost_total_us.fetch_add(
                comp_elapsed_us, std::memory_order_relaxed);
            // UCB reward: bytes_saved / compress_us, attributed to the
            // originally chosen arm.  comp_elapsed_us covers the (possibly
            // tier-fallback) work that the choice actually triggered, which
            // is the cost the bandit should learn to avoid.
            if (cfg.use_ucb && chosen_tier >= 0) {
                double r = CompressionROI::ucbReward(
                    comp_size, kPageSize, comp_elapsed_us);
                worker.sc_stats[stats_idx].recordReward(chosen_tier, r);
                if (cfg.ucb_warmstart && arena_id < kNumArenas)
                    worker.arena_arm[arena_id].recordReward(chosen_tier, r);
            }
        }

        // Record compressed page info (with algo in top 2 bits). Zero pages
        // record the no-blob sentinel; decompress restores them via memset.
        if (zero_page) {
            compressed_[page_idx].set(nullptr, 0, 0, CompressAlgo::NONE);
            zero_pages_.fetch_add(1, std::memory_order_relaxed);
        } else {
            compressed_[page_idx].set(stored, comp_size, alloc_size, algo);
        }
        if (page_tier_) {
            page_tier_[page_idx] = is_fast_tier ? kTierFast : kTierDeep;
        }
        // v6 profile: track compression count for thrash rate calculation
        if (!zero_page && have_span && sc < kTotalBucketsPerArena) {
            worker.sc_stats[stats_idx].compress_count.fetch_add(1, std::memory_order_relaxed);
        }

        if (deferred) {
            shadow_tick_[page_idx] = tick_counter_;
            states_->set(page_idx, PageState::COMPRESSED_SHADOW);
        } else {
            // P2 chunking: batch the mprotect(PROT_NONE) of just-compressed
            // pages over coalesced contiguous runs (up to kProtectChunkPages
            // per syscall) instead of one mprotect per page. Each mprotect on
            // a mapped page broadcasts a TLB-shootdown IPI to every core
            // running the process; perf showed ~11% of all cycles in
            // asm_sysvec_call_function / smp_call_function_many / flush_tlb
            // plus ~4% kernel mmap_lock contention from this per-page storm.
            // Batching cut neuron-cc full-mode wall time ~9% (640s→581s avg,
            // 2 runs each) at equal RSS. Now DEFAULT ON; the earlier
            // correctness concern (a decompress race on a mid-batch page) is
            // closed by restorePageContents()'s populate-before-readable fix.
            // SMASH_P2_CHUNK=0 reverts to per-page mprotect for comparison.
            static const bool p2_chunk = []{
                const char* v = std::getenv("SMASH_P2_CHUNK");
                return !v || v[0] != '0';
            }();
            if (p2_chunk) {
                states_->set(page_idx, PageState::COMPRESSED);
                if (worker.pending_pn_count < CompressWorker::kPendingProtCap) {
                    worker.pending_pn_pages[worker.pending_pn_count++] = page_idx;
                } else {
                    vm::decommitPages(page_addr, kPageSize);
                    if (!vm::protectPages(page_addr, kPageSize, false, false)) {
                        vm::remapPages(page_addr, kPageSize, false, false);
                    }
                }
            } else if (fixavEnabled()) {
                // FixAv: PROT_NONE BEFORE madvise so readers/writers fault
                // before they can observe a DROPPED+RO page (kernel would
                // otherwise zero-fault, returning zeros for live data).
                // Membarrier drains in-flight stores; any store that
                // retires after mprotect(PROT_NONE) faults visibly and the
                // fault handler decompresses normally.
                if (!vm::protectPages(page_addr, kPageSize, false, false)) {
                    if (!vm::remapPages(page_addr, kPageSize, false, false)) {
                        if (!vm::protectPages(page_addr, kPageSize, true, true)) {
                            vm::remapPages(page_addr, kPageSize, true, true);
                        }
                        __builtin_memcpy(page_addr, worker.page_buf, kPageSize);
                        states_->set(page_idx, PageState::ACTIVE);
                        locks_->unlock(page_idx);
                        return false;
                    }
                }
                states_->set(page_idx, PageState::COMPRESSED);
                membarrierSyncCore();
                vm::decommitPages(page_addr, kPageSize);
            } else if (deferMadviseEnabled()) {
                // Deferred-madvise: mprotect(PROT_NONE) to arm fault-driven
                // decompression, defer madvise(DONTNEED) to per-tick sweeper.
                //
                // Unlock BEFORE mprotect to avoid deadlock: if mprotect's
                // TLB shootdown IPI lands on a core whose thread faults on
                // this page (after waking from sleep), the fault handler
                // needs this lock. Sequence: set COMPRESSED → unlock →
                // mprotect. Between unlock and mprotect the page is
                // COMPRESSED+PROT_RW — app accesses succeed without faulting
                // (no SIGSEGV on RW pages). After mprotect, accesses fault
                // and the handler decompresses normally.
                states_->set(page_idx, PageState::COMPRESSED);
                deferMadvise(page_idx);
                locks_->unlock(page_idx);
                // Re-check: handleFault may have decompressed between unlock
                // and here. Skip mprotect if state changed to avoid infinite
                // SIGSEGV on an ACTIVE page.
                if (states_->get(page_idx) == PageState::COMPRESSED) {
                    if (!vm::protectPages(page_addr, kPageSize, false, false)) {
                        vm::remapPages(page_addr, kPageSize, false, false);
                    }
                }
                if (compressed_fn_ && page_map_) {
                    Span* sp = page_map_->get(reinterpret_cast<uintptr_t>(page_addr));
                    if (sp && !sp->is_large && sp->size_class < kNumClasses) {
                        compressed_fn_(page_idx, sp->arena_id, sp->size_class,
                                       compressed_ctx_);
                    }
                }
                return true;
            } else {
                vm::decommitPages(page_addr, kPageSize);
                if (!vm::protectPages(page_addr, kPageSize, false, false)) {
                    if (!vm::remapPages(page_addr, kPageSize, false, false)) {
                        if (!vm::protectPages(page_addr, kPageSize, true, true)) {
                            vm::remapPages(page_addr, kPageSize, true, true);
                        }
                        __builtin_memcpy(page_addr, worker.page_buf, kPageSize);
                        states_->set(page_idx, PageState::ACTIVE);
                        locks_->unlock(page_idx);
                        return false;
                    }
                }
                states_->set(page_idx, PageState::COMPRESSED);
            }
        }
        locks_->unlock(page_idx);

        // A3 feedback: notify heap that a page from (span->arena_id, sc)
        // successfully compressed. The hook accumulates evidence that the
        // originating slab is cold-biased and eventually flips routing.
        if (compressed_fn_ && page_map_) {
            Span* sp = page_map_->get(reinterpret_cast<uintptr_t>(page_addr));
            if (sp && !sp->is_large && sp->size_class < kNumClasses) {
                compressed_fn_(page_idx, sp->arena_id, sp->size_class,
                               compressed_ctx_);
            }
        }
        return true;
    }

    // Tiered recompression: take a COMPRESSED page and replace its blob
    // with one produced by a (presumably stronger) algorithm.  Used for
    // LZ4 → zstd-9 upgrades once a page has stayed cold long enough that
    // the higher ratio of zstd-9 amortizes against its decomp cost.
    //
    // Locking contract matches compressPage: hold the per-page lock from
    // before state transitions through to after the CompressedPageInfo
    // publish.  Concurrent fault handlers either acquire the lock first
    // (and we abort because state is no longer COMPRESSED) or wait
    // (and see the freshly-published new blob).
    //
    // Returns true on a successful tier upgrade.
    bool recompressPage(size_t page_idx, CompressWorker& worker,
                        CompressAlgo target_algo, int target_zstd_level) {
        locks_->lock(page_idx);

        if (states_->get(page_idx) != PageState::COMPRESSED) {
            // Faulted in by app between phase2 decision and our lock.
            locks_->unlock(page_idx);
            return false;
        }

        // Snapshot current blob info while still under lock.
        CompressedPageInfo old_info = compressed_[page_idx];
        CompressAlgo old_algo = old_info.algorithm();
        size_t old_comp_size = old_info.compressedSize();
        size_t old_alloc_size = old_info.alloc_size;
        void* old_data = old_info.data;

        // Zero-page sentinel (no blob): nothing to recompress — it already
        // occupies zero store bytes. Mark deep so phase 2 stops selecting it.
        if (!old_data) {
            if (page_tier_) page_tier_[page_idx] = kTierDeep;
            locks_->unlock(page_idx);
            return false;
        }

        // No-op if already at target tier.
        if (old_algo == target_algo && target_algo != CompressAlgo::ZSTD_DICT) {
            // Already at deep tier — promote tier marker so phase 2
            // skips this page next tick.
            if (page_tier_) page_tier_[page_idx] = kTierDeep;
            locks_->unlock(page_idx);
            return false;
        }

        uint8_t sc = lookupSizeClass(page_idx);
        uint8_t arena_id = 0;
        bool have_span = lookupSpanInfo(page_idx, arena_id, sc);
        size_t stats_idx = statsIndex(arena_id, sc);

        // We own the blob now; mark state COMPRESSING so concurrent
        // fault paths know the blob may be mid-flight (they wait on
        // the lock we still hold).
        states_->set(page_idx, PageState::COMPRESSING);

        // Acquire a fault slot to borrow a DCtx for decompression.  The
        // shared engine_->zstd_dctx_ isn't thread-safe across workers;
        // fault slots have per-slot DCtx that we can reuse here.
        int slot = -1;
        while ((slot = acquireFaultSlot()) < 0) {
#if defined(__x86_64__)
            __builtin_ia32_pause();
#elif defined(__aarch64__)
            asm volatile("yield");
#endif
        }

        size_t decomp_size = engine_->decompressWithDCtx(
            fault_slots_[slot].dctx,
            old_data, worker.page_buf,
            old_comp_size, kPageSize,
            old_algo, sc);
        releaseFaultSlot(slot);

        if (decomp_size != kPageSize) {
            // Bad blob? Should be unreachable given prior successful compress.
            states_->set(page_idx, PageState::COMPRESSED);
            locks_->unlock(page_idx);
            return false;
        }

        // Recompress with target algo.  Reuse compress_buf as output.
        auto t0 = std::chrono::steady_clock::now();
        size_t max_comp = CompressEngine::maxCompressedSizeAny(kPageSize);
        size_t new_comp_size = worker.compress(
            worker.page_buf, worker.compress_buf,
            kPageSize, max_comp,
            target_algo, sc, target_zstd_level, engine_);
        auto t1 = std::chrono::steady_clock::now();
        uint32_t elapsed_us = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());

        // Reject if compression failed or doesn't deliver meaningful savings
        // over the existing blob.  Threshold: new blob must be ≤ 80% of old.
        // For LZ4 → zstd-9 on real workloads, expected new/old ratio is
        // 0.5 (json) to 0.8 (kv), so the gate fires only on marginal cases.
        size_t savings_gate = (old_comp_size * 4) / 5;
        if (new_comp_size == 0 || new_comp_size > savings_gate) {
            states_->set(page_idx, PageState::COMPRESSED);
            // Mark this page as having tried the upgrade so phase 2
            // doesn't keep retrying every tick (dominant wall-time
            // consumer on neuron-cc cut_batch-norm-training_33: 250K
            // upgrade attempts/tick × ~50µs each = 12.5 s/tick burned
            // on retries that always fail).
            if (page_tier_) page_tier_[page_idx] = kTierFastTried;
            locks_->unlock(page_idx);
            if (have_span && sc < kTotalBucketsPerArena) {
                worker.sc_stats[stats_idx].recordCost(1, elapsed_us);
            }
            return false;
        }

        // File-backed spill (SMASH_SPILL): this is the very-cold (deep-tier)
        // recompression — the natural point to move the blob out of anonymous
        // RAM into the spill file so the kernel can evict it as clean page
        // cache. Try the spill store first; on any miss (feature off, file
        // exhausted, not yet ready) fall back to the anonymous CompressStore.
        size_t new_alloc_size = 0;
        void* new_data = nullptr;
        bool spilled = false;
        if (cfg_spill_ && ensureSpillReady()) {
            new_data = spill_.store(worker.compress_buf, new_comp_size,
                                    &new_alloc_size, page_idx);
            spilled = (new_data != nullptr);
#ifdef __linux__
            // Optional accelerant: push the just-written blob page(s) out to the
            // file now (MADV_PAGEOUT) rather than waiting for memory pressure.
            // Page-align the range. Best-effort; ignore failure.
            if (spilled && cfg_spill_pageout_) {
                auto a = reinterpret_cast<uintptr_t>(new_data);
                uintptr_t start = a & ~(uintptr_t)(kPageSize - 1);
                size_t len = (a + new_comp_size) - start;
                len = (len + kPageSize - 1) & ~(size_t)(kPageSize - 1);
                (void)madvise(reinterpret_cast<void*>(start), len, MADV_PAGEOUT);
            }
#endif
        }
        if (!new_data) {
            new_data = store_->store(worker.compress_buf, new_comp_size,
                                     &new_alloc_size, page_idx);
        }
        if (!new_data) {
            states_->set(page_idx, PageState::COMPRESSED);
            locks_->unlock(page_idx);
            return false;
        }

        // Publish new blob.  Fault handler holds the same lock so this
        // transition is invisible to it; once we release the lock, the
        // fault handler reads compressed_[page_idx] under its own lock
        // acquisition and sees the new blob.
        compressed_[page_idx].set(new_data, new_comp_size, new_alloc_size,
                                  target_algo);
        if (page_tier_) {
            if (spilled) {
                page_tier_[page_idx] = kTierSpilled;
                spilled_count_.fetch_add(1, std::memory_order_relaxed);
            } else {
                page_tier_[page_idx] = kTierDeep;
            }
        }

        states_->set(page_idx, PageState::COMPRESSED);
        locks_->unlock(page_idx);

        // Safe to release old blob now: lock was held while we overwrote
        // the per-page CompressedPageInfo, so no concurrent fault path
        // can still hold a reference to old_data.
        store_->release(old_data, old_alloc_size, page_idx);

        // Stats: target-tier success record.  Tier index 1 = deep in the
        // current 2-tier scheme.
        if (have_span && sc < kTotalBucketsPerArena) {
            worker.sc_stats[stats_idx].record(new_comp_size, kPageSize);
            worker.sc_stats[stats_idx].recordCost(1, elapsed_us);
        }

        return true;
    }


    // ── Dictionary training ───────────────────────────────────────────────

    void collectDictSample(size_t /*page_idx*/, uint8_t sc, void* page_buf) {
        if (sc >= kNumClasses) return;
        const int target_samples = getDictTrainSamples();
        if (target_samples <= 0) return;  // dict training disabled
        auto& dt = dict_train_[sc];
        if (dt.trained) return;
        // hasDictionary already covers warm-loaded dicts — re-collecting
        // samples for those would be wasted work (engine already has a
        // CDict installed and trainDictionary would refuse).
        if (engine_->hasDictionary(sc)) { dt.trained = true; return; }
        if (trainedDictCount() >= kMaxDictClasses) return;

        // Lazy-allocate sample buffers (double-checked locking)
        if (!dt.allocated) {
            LockGuard guard(dt.alloc_lock);
            if (!dt.allocated) {
                size_t buf_size = static_cast<size_t>(target_samples) * kPageSize;
                dt.sample_data = static_cast<char*>(
                    BootstrapAlloc::instance().allocate(buf_size, kPageSize));
                dt.sample_sizes = bootstrapArray<size_t>(target_samples);
                if (!dt.sample_data || !dt.sample_sizes) {
                    dt.trained = true;  // Prevent retries
                    return;
                }
                dt.allocated = true;
            }
        }

        // Atomically claim a sample slot
        uint16_t slot = dt.num_samples.fetch_add(1, std::memory_order_acq_rel);
        if (slot >= static_cast<uint16_t>(target_samples)) return;  // already full

        size_t offset = static_cast<size_t>(slot) * kPageSize;
        __builtin_memcpy(dt.sample_data + offset, page_buf, kPageSize);
        dt.sample_sizes[slot] = kPageSize;
    }

    int trainedDictCount() const {
        int n = 0;
        for (int sc = 0; sc < kNumClasses; ++sc)
            if (engine_->hasDictionary(sc)) ++n;
        return n;
    }

    void trainDictionaries() {
        const int target_samples = getDictTrainSamples();
        if (target_samples <= 0) return;
        if (trainedDictCount() >= kMaxDictClasses) return;

        for (int sc = 0; sc < kNumClasses; ++sc) {
            auto& dt = dict_train_[sc];
            if (dt.trained) continue;
            uint16_t samples = dt.num_samples.load(std::memory_order_acquire);
            if (samples < static_cast<uint16_t>(target_samples)) continue;
            // Cap samples to target_samples to avoid passing bad count to ZDICT
            uint16_t actual_samples = std::min(samples, static_cast<uint16_t>(target_samples));
            dt.trained = engine_->trainDictionary(
                sc, dt.sample_data, dt.sample_sizes, actual_samples);
            if (!dt.trained) dt.trained = true;
            if (trainedDictCount() >= kMaxDictClasses) return;
        }
    }

    // ── Prefetch adjacent compressed pages ────────────────────────────────

    void prefetchAdjacent(size_t page_idx) {
        size_t committed = vm_->committedPages();

        for (int delta = -kPrefetchWindow; delta <= kPrefetchWindow; ++delta) {
            if (delta == 0) continue;

            size_t adj = page_idx + static_cast<size_t>(delta);
            if (delta < 0 && static_cast<size_t>(-delta) > page_idx) continue;
            if (adj >= committed) continue;

            // Only prefetch pages in the same span
            if (!sameSpan(page_idx, adj)) continue;

            // Non-blocking lock to avoid deadlock
            if (!locks_->tryLock(adj)) continue;

            if (states_->get(adj) == PageState::COMPRESSED && compressed_[adj].data) {
                void* adj_addr = vm_->pageAddress(adj);
                CompressAlgo algo = compressed_[adj].algorithm();
                uint8_t sc = lookupSizeClass(adj);

                // Try to acquire a fault slot. If none available, skip this prefetch
                // (it's just an optimization - the page will be decompressed on demand).
                int slot = acquireFaultSlot();
                if (slot < 0) {
                    locks_->unlock(adj);
                    continue;
                }

                engine_->decompressWithDCtx(
                    fault_slots_[slot].dctx,
                    compressed_[adj].data, fault_slots_[slot].buf,
                    compressed_[adj].compressedSize(), kPageSize,
                    algo, sc);
                // Populate backing before the page becomes readable so a
                // concurrent app load never sees a readable-but-empty page.
                restorePageContents(adj_addr, fault_slots_[slot].buf);
                releaseFaultSlot(slot);

                // Release compressed blob (sharded by page index)
                releaseBlob(adj);
                compressed_[adj] = {};

                // Restore to active
                states_->set(adj, PageState::ACTIVE);
                cold_count_[adj] = 0;
            }

            locks_->unlock(adj);
        }
    }

    // ── stdio buffer warming ──────────────────────────────────────────────

    // macOS only: DYLD interposition cannot intercept intra-libSystem calls
    // (getc_unlocked, __srget). Those paths read from FILE._bf without going
    // through our wrapper, so EFAULT-retry can't help. Re-warm stdin/stdout/
    // stderr buffer pages each tick so the kernel never sees a protected page
    // for them. On Linux, LD_PRELOAD intercepts intra-libc calls, so the
    // EFAULT-retry path covers them; this is a no-op there.
    void warmStdioBuffers() {
#ifdef __APPLE__
        FILE* streams[] = { stdin, stdout, stderr };
        for (FILE* f : streams) {
            if (!f || !f->_bf._base || f->_bf._size <= 0) continue;
            smash::vm::warmPages(f->_bf._base, f->_bf._size, vm_);
            smash::vm::warmPages(f, sizeof(FILE), vm_);
        }
#endif
    }

    // Keep the page(s) backing the C `environ` array out of COMPRESSED state.
    // A fork+exec child (Python subprocess.Popen, hlo2penguin) calls getenv()
    // in the child after a raw fork() that bypassed pthread_atfork; if the
    // environ page is PROT_NONE (compressed) the child SIGSEGVs with no fault
    // handler to restore it. We (a) record environ's current page index so
    // phase2Range/phase3Range skip it, and (b) warm it now so if it is already
    // COMPRESSED it gets faulted back to ACTIVE before the next fork. `environ`
    // moves when setenv/putenv reallocs the array, so this is refreshed every
    // tick (one pointer read + one page-index compute — negligible).
    // environ + its NULL-terminated pointer array are page-sized at most for
    // any realistic environment; warmPages covers the array's span.
    void refreshEnvironExclusion() {
#if defined(__linux__)
        // Cheap per-tick bookkeeping ONLY: record which arena page currently
        // holds the `environ` array so phase2Range/phase3Range skip it (never
        // transition it to COMPRESSED/ACTIVE_MONITORING). No page is touched
        // here — exclusion alone keeps environ readable, which is all a fork
        // child's getenv() needs. `environ` moves when setenv/putenv reallocs
        // the array, hence the re-read each tick (one load + one compare).
        char** e = ::environ;  // <unistd.h> declares environ at global scope
        uintptr_t addr = reinterpret_cast<uintptr_t>(e);
        if (!e || !vm_ || !vm_->contains(addr)) {
            // environ not smash-managed (e.g. still on the initial stack /
            // glibc .bss) — smash never compresses it, nothing to exclude.
            environ_page_idx_ = kNoEnvironPage;
            return;
        }
        environ_page_idx_ = vm_->contigPageIndex(addr);
#endif
    }

    // ── Parallel dispatch ─────────────────────────────────────────────────

    void dispatch(int phase) {
        // If helpers are not running (e.g., compressTick() called directly
        // in tests), fall back to single-threaded execution.
        int nw = active_workers_;
        bool helpers_active = running_.load(std::memory_order_relaxed) && nw > 1;
        if (helpers_active) {
            current_phase_ = phase;
            work_gen_.fetch_add(1, std::memory_order_release);
            work_waker_.notifyAll();

            // Coordinator does worker 0's range
            executePhase(phase, 0);

            // Wait for active helpers only (helpers with id >= nw-1 are parked)
            uint64_t target = work_gen_.load(std::memory_order_relaxed);
            for (int i = 0; i < nw - 1; ++i) {
                while (helper_done_gen_[i].load(std::memory_order_acquire) < target) {
                    // spin
                }
            }
        } else {
            // Single-threaded: execute all work on worker 0
            executePhase(phase, 0);
        }
    }

    void executePhase(int phase, int worker_id) {
        auto& w = workers_[worker_id];
        if (phase == 1) phase1Range(w.range_start, w.range_end);
        else if (phase == 2) phase2Range(worker_id, w.range_start, w.range_end);
        else if (phase == 3) phase3Range(w.range_start, w.range_end);
        else if (phase == 4) reclaimShadowRange(w.range_start, w.range_end);
    }

    // ── Phase B: reclaim shadow pages ────────────────────────────────────
    // For pages in COMPRESSED_SHADOW, verify content hasn't changed since
    // Phase A, then reclaim physical memory (decommit + PROT_NONE).

    void reclaimShadowRange(size_t start, size_t end) {
        if (!shadow_tick_) return;
        const int delay = getDeferredReclaimDelay();
        const size_t num_chunks_range = (end - start + kChunkSize - 1) / kChunkSize;
        size_t chunk_base = start / kChunkSize;

        // Mid-loop VMA budget. This is the dominant VMA-creating path in the
        // default deferred-reclaim mode: each reclaimed shadow page gets its
        // own mprotect(PROT_NONE) (lines below), splitting ~1 VMA per page with
        // no coalescing. Phase B runs every tick and is gated only by the
        // COARSE tick-start defer_phases check — so within a single tick it can
        // reclaim thousands of pages and blow past vm.max_map_count before the
        // next tick's guard runs. That exhaustion made mprotect return ENOMEM;
        // the restore fallback then bailed (no crash) but left the page
        // unrestorable, so the faulting access re-faulted forever → LIVELOCK
        // (observed on 3M-row in-memory sqlite: stuck at 2M rows, VMAs pinned
        // at the 65531 cap). Bound VMA creation WITHIN the loop: read the live
        // count once, charge +1 per reclaim, and stop reclaiming new pages once
        // within cfg_vma_margin_ of the cap. Un-reclaimed pages stay
        // COMPRESSED_SHADOW (still accessible, no fault) and are reclaimed a
        // later tick once fault-driven decompression has merged VMAs back.
        long vma_now = readVmaCount();
        const long vma_cap = vmaMaxMapCount() - cfg_vma_margin_;
        long reclaimed_since_resync = 0;
        constexpr long kVmaResyncReclaims = 256;

        for (size_t c = 0; c < num_chunks_range; ++c) {
            if (vma_cap > 0 && vma_now >= vma_cap) break;  // VMA budget exhausted
            size_t ci = chunk_base + c;
            if (ci >= num_chunks_) break;
            uint64_t mask = live_chunks_[ci];
            if (!mask) continue;
            size_t page_base = ci * kChunkSize;

            while (mask) {
                if (vma_cap > 0 && vma_now >= vma_cap) break;  // VMA budget
                int bit = __builtin_ctzll(mask);
                mask &= mask - 1;
                size_t i = page_base + bit;
                if (i >= end) break;

                if (states_->get(i) != PageState::COMPRESSED_SHADOW) continue;
                uint32_t age = tick_counter_ - shadow_tick_[i];
                if (age < static_cast<uint32_t>(delay)) continue;

                if (!locks_->tryLock(i)) continue;
                if (states_->get(i) != PageState::COMPRESSED_SHADOW) {
                    locks_->unlock(i);
                    continue;
                }

                // Set PROT_READ to catch any concurrent writes during
                // verification. A write between now and PROT_NONE would
                // fault through the handler (COMPRESSED_SHADOW case),
                // which discards the blob and restores ACTIVE. This
                // closes the verify-to-reclaim race window.
                void* page_addr = vm_->pageAddress(i);
                if (!page_addr) {
                    states_->set(i, PageState::EMPTY);
                    locks_->unlock(i);
                    continue;
                }
                vm::protectPages(page_addr, kPageSize, true, false);  // PROT_READ

                // If the state was changed by a concurrent fault handler
                // (write came in right as we set PROT_READ), abort.
                if (states_->get(i) != PageState::COMPRESSED_SHADOW) {
                    vm::protectPages(page_addr, kPageSize, true, true);
                    locks_->unlock(i);
                    continue;
                }

                // Verify page content matches the compressed blob.
                int slot = acquireFaultSlot();
                if (slot < 0) {
                    vm::protectPages(page_addr, kPageSize, true, true);
                    locks_->unlock(i);
                    continue;
                }

                CompressAlgo algo = compressed_[i].algorithm();
                uint8_t sc = lookupSizeClass(i);
                engine_->decompressWithDCtx(
                    fault_slots_[slot].dctx,
                    compressed_[i].data, fault_slots_[slot].buf,
                    compressed_[i].compressedSize(), kPageSize,
                    algo, sc);

                bool content_matches =
                    __builtin_memcmp(page_addr, fault_slots_[slot].buf, kPageSize) == 0;
                releaseFaultSlot(slot);

                if (!content_matches) {
                    // Page was modified before we set PROT_READ — discard
                    releaseBlob(i);
                    compressed_[i] = {};
                    shadow_tick_[i] = 0;
                    if (page_tier_) page_tier_[i] = kTierNone;
                    cold_count_[i] = 0;
                    compress_fail_count_[i] = 0;  // contents changed
                    vm::protectPages(page_addr, kPageSize, true, true);
                    states_->set(i, PageState::ACTIVE);
                    locks_->unlock(i);
                    continue;
                }

                // Content verified under PROT_READ — any write between
                // the memcmp and now would have faulted (handler discards
                // blob and sets ACTIVE). Safe to reclaim.
                if (fixavEnabled()) {
                    // FixAv: PROT_NONE → membarrier → madvise so a remote
                    // reader can never observe a DROPPED+RO page (the
                    // mprotect IPI shoots down the TLB; any post-madvise
                    // access faults visibly and the handler decompresses).
                    vm::protectPages(page_addr, kPageSize, false, false);  // PROT_NONE
                    shadow_tick_[i] = 0;
                    states_->set(i, PageState::COMPRESSED);
                    membarrierSyncCore();
                    vm::decommitPages(page_addr, kPageSize);
                } else {
                    vm::decommitPages(page_addr, kPageSize);
                    vm::protectPages(page_addr, kPageSize, false, false);  // PROT_NONE
                    shadow_tick_[i] = 0;
                    states_->set(i, PageState::COMPRESSED);
                }
                locks_->unlock(i);
                // Charge this reclaim against the VMA budget. The PROT_NONE
                // above splits ~1 VMA; resync to the real count periodically
                // to absorb merges from concurrent fault-driven decompression.
                ++vma_now;
                if (++reclaimed_since_resync >= kVmaResyncReclaims) {
                    vma_now = readVmaCount();
                    reclaimed_since_resync = 0;
                }
            }
        }
    }

    // ── Soft-dirty access tracking (Linux only) ───────────────────────────
    //
    // Replaces Phase 3 mprotect-based write monitoring. Pros:
    //  - Kernel sets the soft-dirty PTE bit on every write at no fault cost.
    //  - No VMA fragmentation (mprotect creates one VMA boundary per page;
    //    soft-dirty has no VMA cost). Critical for big workloads that
    //    otherwise hit /proc/sys/vm/max_map_count = 65530.
    //  - One syscall per tick to clear, vs N mprotect calls.
    //
    // Cost:
    //  - Bulk /proc/self/pagemap read at tick start: 8 bytes per page in
    //    our region. For 1M pages that's an 8 MiB sequential read; the
    //    kernel can stream this from the page-table walk fast.
    //  - One write to /proc/self/clear_refs at tick end. The kernel
    //    soft-clears all anonymous PTEs in the process; not just ours.
    //    Side effect: a brief PTE-zap stall, similar in magnitude to the
    //    mprotect-storm we're replacing but bounded to a single syscall.
    //
    // Default ON on Linux. Disabled by `SMASH_SOFTDIRTY=0` for the
    // crossover cases (very sparse writes on small heaps where the
    // pagemap-read fixed cost dominates) or for sandboxes where
    // /proc/self/pagemap isn't openable. Microbenchmark
    // (bench/bench_softdirty_vs_protread.cpp): soft-dirty wins 2-11x
    // on workloads with >0.5% write density, and entirely avoids the
    // VMA-explosion hazard (vm.max_map_count=65530) that the per-page
    // mprotect-on-fault path produces.
    static bool isSoftDirtyEnabled() {
        static const int on = []{
            const char* v = std::getenv("SMASH_SOFTDIRTY");
            // Default: enabled on Linux, opt-out via SMASH_SOFTDIRTY=0.
#ifdef __linux__
            if (!v) return 1;
            return (v[0] == '0') ? 0 : 1;
#else
            (void)v;
            return 0;
#endif
        }();
        return on != 0;
    }

    // Open the soft-dirty fds lazily once. Returns false if the kernel
    // doesn't have soft-dirty support (e.g. not Linux, or pagemap denied).
    bool ensureSoftDirtyFds() {
#ifdef __linux__
        if (clear_refs_fd_ < 0) {
            clear_refs_fd_ = open("/proc/self/clear_refs", O_WRONLY | O_CLOEXEC);
            if (clear_refs_fd_ < 0) return false;
        }
        if (pagemap_fd_ < 0) {
            pagemap_fd_ = open("/proc/self/pagemap", O_RDONLY | O_CLOEXEC);
            if (pagemap_fd_ < 0) return false;
        }
        return true;
#else
        return false;
#endif
    }

    // Probe for usable idle-page read tracking. Requires (1) the sysfs bitmap
    // openable O_RDWR (kernel built with CONFIG_IDLE_PAGE_TRACKING + root), and
    // (2) /proc/self/pagemap exposing a real PFN (non-zero) for a resident page
    // — redacted to 0 without CAP_SYS_ADMIN. Cached: idle_read_ok_ in {0,1}.
    // SMASH_IDLE_READ_TRACK: unset/AUTO ⇒ use iff probe passes; 0 ⇒ force off;
    // 1 ⇒ force on (still requires probe; warns to stderr if it can't).
    bool ensureIdleReadFds() {
#ifdef __linux__
        if (idle_read_ok_ >= 0) return idle_read_ok_ == 1;
        idle_read_ok_ = 0;  // assume unavailable until proven otherwise

        static const int mode = []{
            const char* v = std::getenv("SMASH_IDLE_READ_TRACK");
            if (!v) return 0;                  // AUTO
            if (v[0] == '0') return -1;        // force off
            return 1;                          // force on
        }();
        if (mode == -1) return false;
        if (!ensureSoftDirtyFds()) return false;  // need pagemap for PFNs

        // (2) Can we read a real PFN? Probe our own region's first committed page.
        if (vm_) {
            void* pa = vm_->pageAddress(0);
            if (pa) {
                uint64_t off = (reinterpret_cast<uintptr_t>(pa) / kPageSize) * sizeof(uint64_t);
                uint64_t e = 0;
                if (pread(pagemap_fd_, &e, sizeof(e), off) == (ssize_t)sizeof(e)) {
                    bool present = (e >> 63) & 1;
                    uint64_t pfn = e & ((1ULL << 55) - 1);
                    if (!present || pfn == 0) {
                        if (mode == 1) {
                            const char m[] = "[smash] SMASH_IDLE_READ_TRACK=1 but PFN redacted (need CAP_SYS_ADMIN); falling back\n";
                            (void)!::write(2, m, sizeof(m) - 1);
                        }
                        return false;
                    }
                }
            }
        }
        // (1) Open the idle bitmap O_RDWR.
        page_idle_fd_ = open("/sys/kernel/mm/page_idle/bitmap", O_RDWR | O_CLOEXEC);
        if (page_idle_fd_ < 0) {
            if (mode == 1) {
                const char m[] = "[smash] SMASH_IDLE_READ_TRACK=1 but /sys/kernel/mm/page_idle/bitmap unavailable; falling back\n";
                (void)!::write(2, m, sizeof(m) - 1);
            }
            return false;
        }
        idle_read_ok_ = 1;
        return true;
#else
        return false;
#endif
    }

#ifdef __linux__
    // Read the PFN for a smash page index from pagemap (0 if absent/redacted).
    uint64_t pfnForPage(size_t page_idx) {
        if (pagemap_fd_ < 0 || !vm_) return 0;
        void* pa = vm_->pageAddress(page_idx);
        if (!pa) return 0;
        uint64_t off = (reinterpret_cast<uintptr_t>(pa) / kPageSize) * sizeof(uint64_t);
        uint64_t e = 0;
        if (pread(pagemap_fd_, &e, sizeof(e), off) != (ssize_t)sizeof(e)) return 0;
        if (!((e >> 63) & 1)) return 0;  // not present
        return e & ((1ULL << 55) - 1);
    }

    // Set the idle bit for a PFN (the bitmap is u64-per-64-PFN; bit = pfn%64).
    void markPageIdle(uint64_t pfn) {
        if (page_idle_fd_ < 0 || pfn == 0) return;
        uint64_t word_off = (pfn / 64) * sizeof(uint64_t);
        uint64_t word = 0;
        if (pread(page_idle_fd_, &word, sizeof(word), word_off) == (ssize_t)sizeof(word)) {
            word |= (1ULL << (pfn % 64));
        } else {
            word = (1ULL << (pfn % 64));
        }
        (void)!pwrite(page_idle_fd_, &word, sizeof(word), word_off);
    }

    // Test whether a PFN is still idle (true ⇒ NOT accessed since marked).
    bool isPageIdle(uint64_t pfn) {
        if (page_idle_fd_ < 0 || pfn == 0) return false;
        uint64_t word_off = (pfn / 64) * sizeof(uint64_t);
        uint64_t word = 0;
        if (pread(page_idle_fd_, &word, sizeof(word), word_off) != (ssize_t)sizeof(word))
            return false;
        return (word >> (pfn % 64)) & 1;
    }
#endif

    // ── Atomic page restore (decompress-on-fault correctness) ──────────────
    //
    // Restore `kPageSize` bytes of decompressed data from `src` into the page
    // at `page_addr`, which is currently PROT_NONE (its backing may also have
    // been dropped via madvise(DONTNEED)). The caller holds the page's
    // per-page lock.
    //
    // The hard correctness requirement: a concurrent application thread doing
    // an ordinary load on this page must NEVER observe the page in a readable
    // state that does not yet contain the decompressed bytes. Such a thread
    // does not go through handleFault (it's a hardware load, not our code) and
    // does not take the per-page lock, so the ONLY thing standing between it
    // and stale/zero data is the page protection. Therefore the data MUST be
    // in place before the page becomes readable.
    //
    // The previous implementation did `mprotect(PROT_RW)` THEN `memcpy`,
    // opening a window in which the page was readable but empty. Under
    // multithreaded workloads (e.g. walrus mod_parallel_pass) a remote thread
    // read that window and got zeros / stale bytes, silently corrupting the
    // compiler's data structures. This manifested as nondeterministic internal
    // errors ("overlapping memloc", BIR verification failures, DenseMap
    // assertions) — all downstream symptoms of one page coming back wrong.
    //
    // Linux fast path: write the bytes through /proc/self/mem while the page
    // is still PROT_NONE. The kernel's mem-file access uses FOLL_FORCE, which
    // honors VM_MAYWRITE (our reservation is mapped PROT_RW) rather than the
    // current PTE protection, so the store lands and faults in a fresh backing
    // page even though a direct user store would SIGSEGV. Concurrent readers
    // keep faulting on PROT_NONE and block in handleFault on the per-page lock
    // we hold. Only after the data is in place do we flip to PROT_RW.
    //
    // Fallback (non-Linux, or if /proc/self/mem is unavailable): commit then
    // memcpy. This restores the legacy behavior with its small race window —
    // acceptable because Linux is the production target and the fast path is
    // expected to succeed there.
    void restorePageContents(void* page_addr, const void* src) {
        if (!page_addr || !vm_->contains(reinterpret_cast<uintptr_t>(page_addr))) {
            return;
        }
#ifdef __linux__
        if (self_mem_fd_ < 0) {
            self_mem_fd_ = open("/proc/self/mem", O_RDWR | O_CLOEXEC);
        }
        if (self_mem_fd_ >= 0) {
            // Populate backing while PROT_NONE. pwrite handles partial writes
            // by looping; a page is small enough that one call almost always
            // suffices, but loop to be safe.
            const char* p = static_cast<const char*>(src);
            size_t remaining = kPageSize;
            off_t off = static_cast<off_t>(reinterpret_cast<uintptr_t>(page_addr));
            bool ok = true;
            while (remaining > 0) {
                // Direct syscall: the fault handler cannot use PLT-bound
                // pwrite() because lazy PLT resolution itself accesses
                // the dynamic linker's hash tables, which may live on
                // smash-compressed pages — causing infinite SIGSEGV recursion.
                ssize_t w = syscall(SYS_pwrite64, self_mem_fd_, p, remaining, off);
                if (w <= 0) { ok = false; break; }
                p += w; off += w; remaining -= static_cast<size_t>(w);
            }
            if (ok) {
                // Data is in place; now make it readable. Use mprotect; if the
                // VMA-count cap makes that fail, remapPages can't help here
                // (it would discard the bytes we just wrote), so fall through
                // only on the rare mprotect failure.
                if (vm::protectPages(page_addr, kPageSize, true, true)) {
                    return;
                }
                // mprotect(PROT_RW) failed (ENOMEM at vm.max_map_count). The
                // only recovery is remapPages, which would discard the bytes
                // we just wrote — so fall through to the copy path below, which
                // re-establishes a mapping AND re-writes the data. Rare.
            }
        }
#endif
        // Fallback: legacy commit-then-copy (has a small readable-but-empty
        // window; only reached off the Linux fast path).
        //
        // SAFETY: the memcpy below stores directly into page_addr, so the page
        // MUST be writable first. Under VMA exhaustion (vm.max_map_count hit by
        // per-page mprotect on a heavily hot/cold-interleaved workload, e.g.
        // large in-memory SQLite) BOTH commitPages (mprotect) and remapPages
        // (mmap) fail with ENOMEM. Memcpy-ing into a still-PROT_NONE page is a
        // wild store → SIGSEGV. Verify writability and bail on failure: leave
        // the page PROT_NONE / COMPRESSED so the access simply re-faults (the
        // caller keeps the per-page lock; the data is still recoverable from the
        // blob). Degrade, don't crash.
        if (!vm::commitPages(page_addr, kPageSize) &&
            !vm::remapPages(page_addr, kPageSize, true, true)) {
            static std::atomic<uint64_t> warned{0};
            if (warned.fetch_add(1, std::memory_order_relaxed) == 0) {
                char buf[160];
                int n = smash::safe_snprintf(buf, sizeof(buf),
                    "[smash WARN] restorePageContents: cannot make page %p "
                    "writable (mprotect/mmap ENOMEM — likely vm.max_map_count "
                    "exhausted); skipping restore to avoid wild-store crash\n",
                    page_addr);
                if (n > 0) (void)!::write(2, buf, (size_t)n);
            }
            return;  // page stays PROT_NONE+COMPRESSED → access re-faults; no crash
        }
        __builtin_memcpy(page_addr, src, kPageSize);
    }

    // Read pagemap entries for the smash region [start, end) and OR the
    // soft-dirty bit (PTE bit 55, exposed in pagemap bit 55) into
    // accessed_[i]. Reads in a single bulk pread; no syscall per page.
    void readSoftDirty(size_t start, size_t end) {
#ifdef __linux__
        if (pagemap_fd_ < 0 || end <= start) return;
        if (!vm_) return;
        const size_t contig_pages = vm_->contigPages();
        constexpr size_t kBatchPages = 4096;  // 32 KiB pagemap reads
        uint64_t buf[kBatchPages];

        // Phase 1: Contiguous arena pages (can read in bulk)
        size_t contig_end = (end < contig_pages) ? end : contig_pages;
        size_t i = start;
        while (i < contig_end) {
            size_t batch = contig_end - i;
            if (batch > kBatchPages) batch = kBatchPages;
            void* pa = vm_->pageAddress(i);
            if (!pa) { ++i; continue; }
            uint64_t pfn_off =
                (reinterpret_cast<uintptr_t>(pa) / kPageSize) *
                sizeof(uint64_t);
            ssize_t got = pread(pagemap_fd_, buf,
                                batch * sizeof(uint64_t), pfn_off);
            if (got <= 0) break;
            size_t got_pages = static_cast<size_t>(got) / sizeof(uint64_t);
            for (size_t k = 0; k < got_pages; ++k) {
                if (buf[k] & (1ULL << 55)) {
                    accessed_[i + k].store(true, std::memory_order_relaxed);
                }
            }
            i += got_pages;
            if (got_pages < batch) break;
        }

        // Phase 2: External pages (non-contiguous, must read individually)
        // External pages are at indices [contig_pages, end). Each has its own
        // virtual address from a separate mmap, so we can't batch-read pagemap.
        //
        // Skip if profile says external pages are hot — we won't compress them
        // anyway, so there's no point reading 130K pagemap entries per tick.
        if (external_pages_hot_from_profile_) {
            // Mark all external pages as accessed so phase2 skips them
            for (i = contig_pages; i < end; ++i) {
                accessed_[i].store(true, std::memory_order_relaxed);
            }
        } else {
            if (start < contig_pages) start = contig_pages;
            for (i = start; i < end; ++i) {
                void* pa = vm_->pageAddress(i);
                if (!pa) continue;
                uint64_t pfn_off =
                    (reinterpret_cast<uintptr_t>(pa) / kPageSize) *
                    sizeof(uint64_t);
                uint64_t entry = 0;
                ssize_t got = pread(pagemap_fd_, &entry, sizeof(entry), pfn_off);
                if (got == sizeof(entry) && (entry & (1ULL << 55))) {
                    accessed_[i].store(true, std::memory_order_relaxed);
                }
            }
        }
#else
        (void)start; (void)end;
#endif
    }

    // Clear soft-dirty for the entire process. One syscall.
    void clearSoftDirty() {
#ifdef __linux__
        if (clear_refs_fd_ < 0) return;
        // "4\n" — see Documentation/admin-guide/mm/soft-dirty.rst.
        const char buf[3] = {'4', '\n', 0};
        (void)!write(clear_refs_fd_, buf, 2);
#endif
    }

    // ── Time-budget marginal-efficiency recompute ─────────────────────────
    //
    // Walks every (arena, size_class) bucket aggregated across workers and
    // sets `budget_decision_hint` to one of {EXPLORE, COMPRESS, SKIP} per
    // the SMASH_TIME_BUDGET_PCT design (TIME_BUDGET.md):
    //
    //   - count < 8                          → EXPLORE (always compress)
    //   - SMASH_TIME_BUDGET_PCT unset / 100  → COMPRESS (no skip; legacy)
    //   - otherwise: rank buckets by efficiency desc; admit while
    //     cumulative time_cost ≤ pct × elapsed_wall_us; reject the rest.
    //
    // Cost: O(B log B) where B = kNumArenas * kTotalBucketsPerArena ≤ ~5600
    // (kMaxArenas=128 × (kNumClasses=36 + kNumLargeClasses=8)). With
    // kBudgetRecomputeEveryTicks=8 (one second cadence at 1Hz tick) the
    // amortized overhead is well under 1% even on the slowest machines.
    void recomputeMarginalEfficiency() {
        int budget_pct = getTimeBudgetPct();
        const size_t total = static_cast<size_t>(kNumArenas) * kTotalBucketsPerArena;

        // Aggregate per-bucket counters across workers. Worker 0 is the
        // canonical sink for cross-thread updates (fault decompress path),
        // but worker N's compressPage writes to its own slot — sum them.
        struct BucketAgg {
            uint32_t bidx;
            uint32_t count;
            uint64_t bytes_saved;
            uint64_t time_cost_us;
            uint32_t efficiency_x256;  // bytes_saved/time_cost × 256, saturated
        };
        // Bucket count is bounded; allocate from bootstrap once and reuse.
        // Simplest: stack-allocate up to a reasonable cap. kMaxArenas *
        // kTotalBucketsPerArena = 128 * 44 = 5632 entries × 32 B = 176 KB
        // — too big for stack. Use a static-thread-local heap-bootstrapped
        // buffer since this only runs on the coordinator thread.
        static BucketAgg* aggs = nullptr;
        if (!aggs) {
            aggs = bootstrapArray<BucketAgg>(total);
        }

        for (size_t i = 0; i < total; ++i) {
            uint32_t cnt = 0;
            uint64_t bs = 0, tc = 0;
            for (int w = 0; w < kMaxCompressorWorkers; ++w) {
                auto& s = workers_[w].sc_stats[i];
                cnt += s.count;
                bs += s.bytes_saved_total.load(std::memory_order_relaxed);
                tc += s.time_cost_total_us.load(std::memory_order_relaxed);
            }
            uint32_t eff_x256 = 0;
            // Treat zero time_cost as "no signal" — let exploration keep going.
            if (tc > 0) {
                uint64_t e = (bs * 256ULL) / tc;
                eff_x256 = (e > 0xFFFFu) ? 0xFFFFu : static_cast<uint32_t>(e);
            }
            aggs[i] = BucketAgg{static_cast<uint32_t>(i), cnt, bs, tc, eff_x256};
        }

        // Publish efficiency_x256 to every worker's view of each bucket so
        // saveProfileFile can persist a sane warm-start hint.
        for (size_t i = 0; i < total; ++i) {
            for (int w = 0; w < kMaxCompressorWorkers; ++w) {
                workers_[w].sc_stats[i].efficiency_x256 =
                    static_cast<uint16_t>(aggs[i].efficiency_x256);
            }
        }

        // Default decision when budget knob is unset: COMPRESS for every
        // bucket past exploration. Keeps behavior unchanged from legacy.
        auto setHint = [&](size_t i, uint8_t hint) {
            for (int w = 0; w < kMaxCompressorWorkers; ++w)
                workers_[w].sc_stats[i].budget_decision_hint = hint;
        };

        if (budget_pct < 0 || budget_pct >= 100) {
            for (size_t i = 0; i < total; ++i) {
                uint8_t hint = (aggs[i].count < 8)
                    ? SizeClassStats::kBudgetExplore
                    : SizeClassStats::kBudgetCompress;
                setHint(i, hint);
            }
            return;
        }

        // Available compress budget = elapsed_wall_us × pct / 100.
        auto now = std::chrono::steady_clock::now();
        uint64_t elapsed_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                now - start_steady_time_).count());
        // Avoid degenerate boot-up cases where elapsed is zero.
        if (elapsed_us < 1000) elapsed_us = 1000;
        uint64_t available_us =
            (elapsed_us * static_cast<uint64_t>(budget_pct)) / 100ULL;

        // Rank buckets with count >= 8 by efficiency desc. Buckets with
        // count < 8 are EXPLORE (always-compress) and don't enter the sort.
        // Use a flat in-place insertion sort: B ≤ ~4600 and only the
        // count >= 8 subset participates, so we expect << 1k entries.
        // Re-purpose `aggs` as the sort buffer by partitioning in-place.
        size_t n_rank = 0;
        for (size_t i = 0; i < total; ++i) {
            if (aggs[i].count >= 8) {
                if (n_rank != i) std::swap(aggs[n_rank], aggs[i]);
                ++n_rank;
            }
        }
        // Stable partial sort by efficiency desc on aggs[0..n_rank).
        // std::sort with a comparator is fine — coordinator thread, not
        // a hot path.
        std::sort(aggs, aggs + n_rank, [](const BucketAgg& a, const BucketAgg& b) {
            return a.efficiency_x256 > b.efficiency_x256;
        });

        uint64_t budget_used = 0;
        size_t n_compress = 0, n_skip = 0;
        // Accept while cumulative time fits the budget. Buckets with no
        // observed time_cost (tc=0) are admitted for free — they'll either
        // compress productively (and accumulate cost in future ticks) or
        // turn out hot and fail fast.
        for (size_t k = 0; k < n_rank; ++k) {
            uint64_t inc = aggs[k].time_cost_us;
            if (budget_used + inc <= available_us) {
                budget_used += inc;
                setHint(aggs[k].bidx, SizeClassStats::kBudgetCompress);
                ++n_compress;
            } else {
                setHint(aggs[k].bidx, SizeClassStats::kBudgetSkip);
                ++n_skip;
            }
        }
        // Optional one-line summary to stderr per recompute under SMASH_DEBUG=1.
        // Splits the count into slab vs large-alloc buckets so we can see at
        // a glance whether the gating is firing on large allocs (the case
        // for SMASH_LARGE_ONLY workloads).
        if (s_debug_enabled_) {
            size_t n_compress_lg = 0, n_skip_lg = 0, n_explore_lg = 0;
            size_t n_rank_lg = 0;
            for (size_t i = 0; i < total; ++i) {
                size_t bucket_in_arena = i % kTotalBucketsPerArena;
                if (bucket_in_arena < kNumClasses) continue;  // slab bucket
                uint8_t hint = workers_[0].sc_stats[i].budget_decision_hint;
                if (workers_[0].sc_stats[i].count >= 8) ++n_rank_lg;
                if (hint == SizeClassStats::kBudgetCompress) ++n_compress_lg;
                else if (hint == SizeClassStats::kBudgetSkip) ++n_skip_lg;
                else ++n_explore_lg;
            }
            char dbg[300];
            int dn = smash::safe_snprintf(dbg, sizeof(dbg),
                "[smash budget] tick=%u pct=%d elapsed_us=%llu avail_us=%llu "
                "buckets=%zu (%dx%d) n_rank=%zu compress=%zu skip=%zu explore=%zu "
                "large(rank=%zu compress=%zu skip=%zu explore=%zu)\n",
                tick_counter_, budget_pct,
                (unsigned long long)elapsed_us,
                (unsigned long long)available_us,
                total, kNumArenas, kTotalBucketsPerArena,
                n_rank, n_compress, n_skip, total - n_rank,
                n_rank_lg, n_compress_lg, n_skip_lg, n_explore_lg);
            if (dn > 0) (void)!write(2, dbg, (size_t)dn);
        }
        // Buckets that didn't make the rank cut (count < 8) are EXPLORE.
        // They sit at the tail of `aggs` after the partition.
        for (size_t k = n_rank; k < total; ++k) {
            setHint(aggs[k].bidx, SizeClassStats::kBudgetExplore);
        }
    }

    // ── Deferred madvise helpers ──────────────────────────────────────────

    // Queue a just-COMPRESSED page for deferred madvise. Caller MUST have
    // already transitioned state to COMPRESSED (the bit is consumed under
    // the per-page lock by the sweeper, which double-checks state before
    // dropping backing).
    [[gnu::always_inline]]
    inline void deferMadvise(size_t page_idx) {
        deferred_queue_tick_[page_idx] = tick_counter_;
        deferred_pending_[page_idx].store(true, std::memory_order_release);
    }

    // Sweep the deferred-madvise queue. Called once per tick at the END of
    // tick(). For each pending page whose TTL has elapsed and whose state
    // is still COMPRESSED, drop the backing pages with madvise(DONTNEED).
    // The per-page lock provides ordering against handleFault and
    // releaseCompressedPages, both of which clear the pending bit BEFORE
    // any state transition out of COMPRESSED.
    // Max contiguous pages coalesced into one mprotect/madvise in the sweeper.
    // A single mprotect(PROT_NONE) over a contiguous run produces ONE VMA split
    // instead of one-per-page. Without this, a workload that compresses many
    // scattered cold pages (e.g. large in-memory SQLite) exhausts
    // vm.max_map_count (~65530 default): each per-page mprotect splits a VMA,
    // and once the cap is hit every subsequent mprotect/mmap/pwrite fails with
    // ENOMEM, which used to crash restorePageContents()'s fallback memcpy.
    // 512 pages = 2 MiB bounds the per-batch lock-hold time.
    static constexpr size_t kSweepCoalesceMax = 512;

    // vm.max_map_count, read once. Cap on VMAs per process; each per-page
    // mprotect(PROT_NONE) splits a VMA toward this limit.
    static long vmaMaxMapCount() {
        static const long cap = []{
            FILE* f = fopen("/proc/sys/vm/max_map_count", "r");
            if (!f) return 65530L;
            long n = 65530;
            if (fscanf(f, "%ld", &n) != 1) n = 65530;
            fclose(f);
            return n;
        }();
        return cap;
    }

    // Current VMA count for this process (lines in /proc/self/maps). Linux
    // only; returns 0 elsewhere so callers treat the budget as unbounded.
    static long readVmaCount() {
#ifdef __linux__
        FILE* f = fopen("/proc/self/maps", "r");
        if (!f) return 0;
        long n = 0;
        char buf[4096];
        while (fgets(buf, sizeof(buf), f)) ++n;
        fclose(f);
        return n;
#else
        return 0;
#endif
    }

    // True if `dir` is a real-disk filesystem (NOT tmpfs/ramfs). A spill file on
    // a RAM-backed fs defeats the purpose (it costs RAM/swap, not disk).
    static bool isRealDiskDir(const char* dir) {
#ifdef __linux__
        struct statfs sfs;
        if (statfs(dir, &sfs) != 0) return false;
        auto t = static_cast<unsigned long>(sfs.f_type);
        if (t == TMPFS_MAGIC || t == RAMFS_MAGIC) return false;
        return true;
#else
        (void)dir;
        return false;
#endif
    }

    // One-shot setup of the file-backed spill store. Runs on the compressor
    // thread (never the signal handler), so open/statfs/fallocate/mmap and a
    // BootstrapAlloc metadata array are all fine. On ANY failure it gives up
    // permanently (spill_init_done_ = true, spill_ stays !ready) and the caller
    // falls back to anonymous storage — never crashes. Returns spill_.ready().
    bool ensureSpillReady() {
#ifdef __linux__
        if (spill_.ready()) return true;
        if (spill_init_done_) return false;   // already tried and gave up
        spill_init_done_ = true;
        if (!cfg_spill_) return false;

        // Pick a real-disk directory. SMASH_SPILL_DIR wins if it's real disk;
        // else /var/tmp (FHS persistent, real disk by convention — NOT /tmp,
        // commonly tmpfs), else cwd. Reject tmpfs/ramfs (see isRealDiskDir).
        const char* dir = nullptr;
        if (cfg_spill_dir_ && isRealDiskDir(cfg_spill_dir_)) {
            dir = cfg_spill_dir_;
        } else if (isRealDiskDir("/var/tmp")) {
            dir = "/var/tmp";
        } else if (isRealDiskDir(".")) {
            dir = ".";
        }
        if (!dir) return false;   // no real-disk dir available → disable

        // Create unique file, then unlink immediately: the inode is freed on
        // close/exit/crash, guaranteeing cleanup and no stray files.
        char path[4096];
        int n = smash::safe_snprintf(path, sizeof(path), "%s/smash-spill.%d.XXXXXX",
                                     dir, (int)getpid());
        if (n <= 0 || (size_t)n >= sizeof(path)) return false;
        int fd = mkostemp(path, O_CLOEXEC);
        if (fd < 0) return false;
        unlink(path);

        // Size the file to the cap, rounded down to whole 16 MB regions.
        size_t region = SpillStore::regionSize();
        size_t cap = (size_t)(cfg_spill_max_mb_ > 0 ? cfg_spill_max_mb_ : 1024)
                     * 1024 * 1024;
        size_t map_size = (cap / region) * region;
        if (map_size < region) map_size = region;
        // Pre-fallocate REAL blocks (not sparse) so ENOSPC surfaces here as a
        // failure rather than as SIGBUS on a later write through the mapping.
        if (fallocate(fd, 0, 0, (off_t)map_size) != 0) {
            close(fd);
            return false;   // disk too full to reserve → disable, fall back
        }
        void* map = mmap(nullptr, map_size, PROT_READ | PROT_WRITE,
                         MAP_SHARED, fd, 0);
        if (map == MAP_FAILED) {
            close(fd);
            return false;
        }
        // Out-of-band region metadata from BootstrapAlloc (never freed; fine —
        // one-shot, bounded by map_size/16MB entries, e.g. 64 for a 1 GiB cap).
        size_t meta_bytes = SpillStore::metadataBytesFor(map_size);
        void* meta = BootstrapAlloc::instance().allocate(
            meta_bytes, alignof(SpillStore::Region));
        if (!meta) {
            munmap(map, map_size);
            close(fd);
            return false;
        }
        if (!spill_.init(fd, map, map_size,
                         reinterpret_cast<SpillStore::Region*>(meta),
                         map_size / region)) {
            munmap(map, map_size);
            close(fd);
            return false;
        }
        spill_fd_ = fd;
        spill_map_ = map;
        spill_map_size_ = map_size;
        spill_meta_ = reinterpret_cast<SpillStore::Region*>(meta);
        return true;
#else
        return false;
#endif
    }

    // Release the compressed blob for page_idx to the correct store. A
    // kTierSpilled blob lives in the file-backed SpillStore; everything else is
    // in the anonymous CompressStore. Centralizes the discriminator so no
    // release site has to special-case spill. Does NOT clear compressed_[] or
    // page_tier_ — callers already do that as today.
    // Relocate a very-cold page's compressed blob from the anonymous
    // CompressStore into the file-backed SpillStore (clean, kernel-evictable
    // page cache). Plain byte copy of the existing blob — no recompression, no
    // savings gate. Holds the per-page lock so it's serialized against the
    // fault handler (which reads compressed_[i].data under the same lock).
    // Best-effort: on spill exhaustion / not-ready it leaves the page anonymous.
    void maybeSpillPage(size_t page_idx) {
        if (!ensureSpillReady()) return;
        if (!locks_->tryLock(page_idx)) return;  // contended → try a later tick
        // Re-check under lock: still COMPRESSED, still a terminal anonymous tier.
        if (states_->get(page_idx) != PageState::COMPRESSED ||
            !page_tier_ ||
            (page_tier_[page_idx] != kTierDeep &&
             page_tier_[page_idx] != kTierFastTried)) {
            locks_->unlock(page_idx);
            return;
        }
        void* old_data = compressed_[page_idx].data;
        size_t old_alloc = compressed_[page_idx].alloc_size;
        size_t csz = compressed_[page_idx].compressedSize();
        CompressAlgo algo = compressed_[page_idx].algorithm();
        if (!old_data || spill_.contains(old_data)) {  // nothing to do / already spilled
            locks_->unlock(page_idx);
            return;
        }
        size_t new_alloc = 0;
        void* new_data = spill_.store(old_data, csz, &new_alloc, page_idx);
        if (!new_data) { locks_->unlock(page_idx); return; }  // spill full → keep anon
        compressed_[page_idx].set(new_data, csz, new_alloc, algo);
        page_tier_[page_idx] = kTierSpilled;
        spilled_count_.fetch_add(1, std::memory_order_relaxed);
#ifdef __linux__
        if (cfg_spill_pageout_) {
            auto a = reinterpret_cast<uintptr_t>(new_data);
            uintptr_t s = a & ~(uintptr_t)(kPageSize - 1);
            size_t len = (a + csz) - s;
            len = (len + kPageSize - 1) & ~(size_t)(kPageSize - 1);
            (void)madvise(reinterpret_cast<void*>(s), len, MADV_PAGEOUT);
        }
#endif
        locks_->unlock(page_idx);
        // Release the old anonymous blob outside the page lock is unnecessary
        // (release takes the shard lock, not the page lock); do it here while we
        // still hold nothing that conflicts. Safe: we already repointed
        // compressed_[page_idx], so no fault can reach old_data.
        store_->release(old_data, old_alloc, page_idx);
    }

    void releaseBlob(size_t page_idx) {
        // Zero-page sentinel: no blob was ever stored.
        if (!compressed_[page_idx].data) return;
        if (page_tier_ && page_tier_[page_idx] == kTierSpilled) {
            spill_.release(compressed_[page_idx].data,
                           compressed_[page_idx].alloc_size, page_idx);
            spilled_count_.fetch_sub(1, std::memory_order_relaxed);
        } else {
            store_->release(compressed_[page_idx].data,
                            compressed_[page_idx].alloc_size, page_idx);
        }
    }

    void sweepDeferredMadvise() {
        if (!deferMadviseEnabled() || !deferred_pending_) return;
        const uint32_t now = tick_counter_;
        const uint32_t ttl = deferMadviseTicks();
        const size_t total = vm_->committedPages();

        // Mid-sweep VMA budget. This sweep is where deferred-madvise mode
        // actually issues mprotect(PROT_NONE) — each contiguous run splits
        // ~1 VMA. It runs EVERY tick (not gated by the tick-start defer_phases
        // guard), so on a hot/cold-interleaved workload (large in-memory
        // sqlite) it can split thousands of VMAs within a single tick and blow
        // past vm.max_map_count before the next tick's guard ever runs. When
        // that happened, mprotect started returning ENOMEM; the restore
        // fallback then bailed (no wild-store crash) but left the page
        // unrestorable → the faulting access re-faulted forever → LIVELOCK.
        //
        // Fix: bound VMA creation WITHIN the sweep. Read the live count once,
        // estimate +1 per run we apply, and stop arming new PROT_NONE regions
        // once we're within cfg_vma_margin_ of the cap. Pages left unswept stay
        // COMPRESSED+pending and are picked up a later tick, by which point
        // fault-driven decompression (which mprotect(PROT_RW)s pages and MERGES
        // their VMAs back) has freed headroom. Re-sync the real count every
        // kVmaResyncRuns runs to correct estimate drift (merges/other mmaps).
        long vma_now = readVmaCount();
        const long vma_cap = vmaMaxMapCount() - cfg_vma_margin_;
        long applied_since_resync = 0;
        constexpr long kVmaResyncRuns = 256;

        // Buffer of page indices in the current contiguous run we hold locks on.
        size_t run[kSweepCoalesceMax];
        size_t i = 0;
        while (i < total) {
            // VMA budget: stop arming new PROT_NONE regions when near the cap.
            // Leave remaining pending pages for a later tick (no crash, no
            // livelock — they're still COMPRESSED and fault-decompress fine).
            if (vma_cap > 0 && vma_now >= vma_cap) break;

            // Find the start of an eligible run.
            if (!deferred_pending_[i].load(std::memory_order_acquire) ||
                (now - deferred_queue_tick_[i] < ttl)) { ++i; continue; }

            // Greedily extend a CONTIGUOUS run of eligible+lockable pages.
            // Locks are acquired in strictly ascending page-index order, and
            // handleFault only ever holds a single page lock (blocking), so
            // there is no cross-page deadlock. Pages that fail any check break
            // the run (so the mprotect range stays contiguous AND uniform).
            //
            // ADDRESS contiguity — not just index contiguity — is required: the
            // flush below issues ONE mprotect/madvise over [pageAddress(run[0]),
            // +n*kPageSize). For contiguous-arena pages, index i maps to
            // base_+i*kPageSize, so consecutive indices are consecutive
            // addresses. But SMASH_TRACK_EXTERNAL pages get indices by
            // registration order (contig_pages_+local) and pageAddress() returns
            // their arbitrary registered mmap address, so consecutive external
            // indices are NOT adjacent in memory. Coalescing them would make the
            // single range walk off the first page's mapping into foreign memory
            // (issue #84's secondary bug). Require each page's address to equal
            // the previous page's address + kPageSize; a break here degrades
            // external pages to correct single-page runs while leaving the arena
            // fast path untouched. (protectPages only bounds-checks size<=4096,
            // so a multi-page external run would silently escape its guard.)
            size_t n = 0;
            size_t j = i;
            uintptr_t expect_addr = 0;
            while (j < total && n < kSweepCoalesceMax) {
                if (!deferred_pending_[j].load(std::memory_order_acquire) ||
                    (now - deferred_queue_tick_[j] < ttl))
                    break;                         // not eligible → end run
                // Address-contiguity gate (checked before locking so a
                // non-adjacent page simply ends the run without churn).
                void* aj = vm_->pageAddress(j);
                auto uaj = reinterpret_cast<uintptr_t>(aj);
                if (!aj) break;                    // no mapping → end run
                if (n > 0 && uaj != expect_addr) break;  // address gap → end run
                if (!locks_->tryLock(j)) break;    // contended → end run
                // Re-check under lock; on mismatch drop this page and end run.
                if (!deferred_pending_[j].load(std::memory_order_acquire) ||
                    states_->get(j) != PageState::COMPRESSED) {
                    // Defensively clear a stale pending bit; state moved on.
                    deferred_pending_[j].store(false, std::memory_order_release);
                    locks_->unlock(j);
                    break;
                }
                run[n++] = j;
                expect_addr = uaj + kPageSize;
                ++j;
            }

            if (n == 0) { ++i; continue; }

            // All `n` pages [run[0]..run[n-1]] are contiguous, COMPRESSED, and
            // locked by us. Clear their pending bits, then issue ONE
            // mprotect(PROT_NONE) + ONE madvise over the whole 4 KiB*n range —
            // a single VMA split instead of n.
            for (size_t k = 0; k < n; ++k)
                deferred_pending_[run[k]].store(false, std::memory_order_release);
            void* base = vm_->pageAddress(run[0]);
            if (base) {
                size_t bytes = n * kPageSize;
                if (!vm::protectPages(base, bytes, false, false))
                    vm::remapPages(base, bytes, false, false);
                vm::decommitPages(base, bytes);
                // One contiguous PROT_NONE run splits at most ~2 VMAs (a slice
                // out of the middle of a larger RW region). Charge 2 to stay
                // conservative; resync to the real count periodically.
                vma_now += 2;
                if (++applied_since_resync >= kVmaResyncRuns) {
                    vma_now = readVmaCount();
                    applied_since_resync = 0;
                }
            }
            for (size_t k = 0; k < n; ++k)
                locks_->unlock(run[k]);

            i = j;  // resume after the run (j == run[n-1]+1, or the break point)
        }
    }

    // ── Tick ──────────────────────────────────────────────────────────────

    __attribute__((cold)) void tick() {
        ++tick_counter_;
        if (pre_tick_fn_) pre_tick_fn_();
        if (fault_handler_) fault_handler_->ensureInstalled();
        // Time-budget recompute is cheap (~144-4600 buckets sorted) and only
        // runs every kBudgetRecomputeEveryTicks ticks. When the budget knob
        // is unset this still runs but the inner loop short-circuits to the
        // legacy "all buckets COMPRESS" branch — so cost stays negligible.
        if ((tick_counter_ % kBudgetRecomputeEveryTicks) == 0) {
            recomputeMarginalEfficiency();
        }
        // SMASH_DEFER_PHASES_MS=NNN: skip Phase 2 (compress) and Phase 3
        // (monitor PROT_READ) for the first NNN ms after start(). Useful
        // for workloads that establish IPC channels at startup with
        // buffers in smash-managed pages — once those buffers are no
        // longer hot, normal compressor operation resumes. Phase 1
        // (access tracking bookkeeping) still runs.
        const int defer_ms = cfg_defer_phases_ms_;
        static const auto start_time = std::chrono::steady_clock::now();
        bool defer_phases = false;
        if (defer_ms > 0) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - start_time).count();
            defer_phases = (elapsed_ms < defer_ms);
        }

        // VMA-cap guard. Each compressed page creates at least one VMA
        // boundary in /proc/self/maps; Linux's vm.max_map_count (default
        // 65530) caps total VMAs in a process. When we get within
        // kVmaSafetyMargin of the cap, mprotect starts failing with
        // ENOMEM, which has caused data corruption in walrus on big
        // HLOs (cut_batch-norm-training_33). Skip Phase 2 (compress) and
        // Phase 3 (PROT_READ monitor) entirely when we're close — better
        // to lose RSS savings than corrupt the host application.
        //
        // This tick-start check is the COARSE guard; the sweep itself
        // (sweepDeferredMadvise, where deferred-madvise mode actually issues
        // the mprotect(PROT_NONE) that splits VMAs) has its own mid-loop budget
        // so it can't blow past the cap WITHIN a single tick. See that function.
        //
        // VMA headroom kept below max_map_count. The guard's ONLY job is to
        // leave enough free VMA slots that fault-driven decompression (which
        // calls mprotect(PROT_RW) and tends to MERGE VMAs back, net-reducing
        // count) can always proceed — NOT to prevent every possible overshoot.
        //
        // The margin must NOT clip a workload whose natural cold-page working
        // set fits under the cap. Measured: 500K in-memory sqlite plateaus at
        // ~47.8K VMAs (73% of the 65530 cap) and is perfectly safe — a 20K
        // margin (trip @ 45,530) wrongly clipped it and cut its RSS reduction
        // from +24% to −29% vs glibc. An 8K margin (trip @ 57,530) clears that
        // plateau with room while still tripping 3M sqlite (which needs >65.5K
        // VMAs and would otherwise crash/livelock) before the cap. Read once at
        // startup into cfg_vma_margin_ (env SMASH_VMA_MARGIN); never call getenv
        // on the per-tick path.
        const long max_map_count = vmaMaxMapCount();
        const long kVmaSafetyMargin = cfg_vma_margin_;
        vma_count_cached_ = readVmaCount();
        if (vma_count_cached_ + kVmaSafetyMargin >= max_map_count) {
            // Can't safely fragment more VMAs — defer compression for
            // the remainder of this tick. Phase 1 (access tracking)
            // still runs so when VMA budget recovers (via in-flight
            // fault handlers reclaiming pages) we resume.
            defer_phases = true;
        }
        // Re-claim SIGUSR2 each tick so Firefox / other runtimes that
        // also install a SIGUSR2 handler can't permanently displace ours.
        // Cheap: one sigaction-read + one branch.
        //
        // OFF BY DEFAULT: stealing SIGUSR2 from a host that uses it for its
        // own control plane breaks that app. MariaDB, for example, sends
        // SIGUSR2 (its THR_KILL_SIGNAL) to end its signal thread during
        // shutdown; if smash has overwritten the disposition with its stats
        // dumper, that thread never exits and mysqld_main hangs forever in
        // wait_for_signal_thread_to_end(). The SIGUSR2 stats dump is a
        // debugging convenience, not core function — opt in with
        // SMASH_STATS_SIGNAL=1 (which also gates the initial install).
        if (statsSignalEnabled()) {
            struct sigaction current{};
            sigaction(SIGUSR2, nullptr, &current);
            if (current.sa_handler != &CompressorThread::sigusr1Handler) {
                struct sigaction sa{};
                sa.sa_handler = &CompressorThread::sigusr1Handler;
                sa.sa_flags = SA_RESTART;
                sigemptyset(&sa.sa_mask);
                sigaction(SIGUSR2, &sa, nullptr);
            }
        }

        // Re-warm stdio buffers each tick (macOS-only; no-op on Linux).
        // Without this, intra-libSystem getc_unlocked / __srget paths can
        // hit a protected stdin/stdout/stderr buffer and EFAULT inside libc
        // before our wrapper sees the call.
        warmStdioBuffers();

        // Record which arena page holds `environ` so phase2/phase3 never
        // compress it — a fork+exec child's getenv() must always find it
        // readable. Runs before dispatch(2)/(3) below, and environ is
        // populated + read heavily at startup (long before kColdTicks could
        // make it eligible), so it is excluded before it can ever compress.
        refreshEnvironExclusion();

        // SMASH_DEBUG=1: print a stats line every 5 ticks (~5s at the
        // 1000ms compressor cadence — kCompressIntervalMs). Mirrors the
        // SIGUSR2 stats format so log scrapers can use one parser.
        if (s_debug_enabled_) {
            static int debug_tick_counter = 0;
            if (++debug_tick_counter >= 5) {
                debug_tick_counter = 0;
                sigusr1Handler(0);
            }
        }

        size_t committed = vm_->committedPages();
        if (committed == 0) return;

        // If profile says external pages are hot, limit compressor scope to
        // contiguous pages only. This avoids iterating 130K+ external pages
        // every tick when we know they won't be compressed anyway.
        if (external_pages_hot_from_profile_) {
            size_t contig = vm_->contigPages();
            if (contig < committed) committed = contig;
        }

        // Rebuild chunk bitmap
        rebuildChunkBitmap(committed);

        // Reset per-worker tick counters
        int nw = active_workers_;
        for (int w = 0; w < nw; ++w) {
            worker_pages_eligible_[w].store(0, std::memory_order_relaxed);
            worker_pages_compressed_[w].store(0, std::memory_order_relaxed);
        }

        // Partition page range across active workers.
        // If helpers are not running, worker 0 gets the entire range.
        bool helpers_active = running_.load(std::memory_order_relaxed) && nw > 1;
        if (helpers_active) {
            size_t per_worker = committed / nw;
            for (int w = 0; w < nw; ++w) {
                workers_[w].range_start = w * per_worker;
                workers_[w].range_end = (w == nw - 1)
                    ? committed : (w + 1) * per_worker;
            }
        } else {
            workers_[0].range_start = 0;
            workers_[0].range_end = committed;
        }

        // Soft-dirty: bulk-read kernel-tracked write bits into accessed_
        // BEFORE Phase 1 consumes them, then clear for the next tick.
        bool softdirty = isSoftDirtyEnabled() && ensureSoftDirtyFds();
        if (softdirty) {
            readSoftDirty(0, committed);
        }
#ifdef __linux__
        // Probe idle-page read tracking once (cached). No-op + fallback when the
        // kernel lacks CONFIG_IDLE_PAGE_TRACKING or we lack CAP_SYS_ADMIN (the
        // common unprivileged/LD_PRELOAD case). When active, phase1 reads the
        // idle bit for ACTIVE_MONITORING pages as the fault-free read signal.
        (void)ensureIdleReadFds();
#endif

        dispatch(1);  // Phase 1: access tracking

        if (!defer_phases && isDeferredReclaimMode())
            dispatch(4);  // Phase B: reclaim confirmed-cold shadow pages

        if (!defer_phases) dispatch(2);  // Phase 2: compression
        // Phase 3 mprotects ACTIVE pages to PROT_READ for write-fault tracking.
        // This breaks any syscall / Mach trap that writes into a smash-managed
        // buffer it doesn't know to pin (e.g. mach_msg from CFRunLoop).
        // SMASH_NO_MONITOR=1 disables Phase 3 at runtime, trading off cold
        // detection accuracy for compatibility with such codepaths. Looked
        // up once at startup to keep the tick loop branch-free.
        const bool no_monitor = cfg_no_monitor_;
        // Selective Phase 3 skip: if many recent ticks all reported
        // 0 pages eligible AND 0 compressed, Phase 3 monitoring just
        // creates VMA-fragmenting mprotect storms with no payoff
        // (workload is steady-RW; monitoring won't help). Skip until
        // we hit a re-probe boundary so we don't go permanently blind.
        // Disabled by default (=0); user opt-in via SMASH_PHASE3_SKIP_THRESHOLD.
        static const int phase3_skip_thresh = []{
            const char* v = std::getenv("SMASH_PHASE3_SKIP_THRESHOLD");
            return v ? atoi(v) : 0;
        }();
        bool skip_phase3 = false;
        if (phase3_skip_thresh > 0) {
            uint64_t total_compressed = 0;
            uint64_t total_eligible = 0;
            for (int w = 0; w < nw; ++w) {
                total_compressed += worker_pages_compressed_[w].load(
                    std::memory_order_relaxed);
                total_eligible += worker_pages_eligible_[w].load(
                    std::memory_order_relaxed);
            }
            if (total_compressed == 0 && total_eligible == 0) {
                ++phase3_idle_ticks_;
                if (phase3_idle_ticks_ > phase3_skip_thresh &&
                    (phase3_idle_ticks_ % phase3_skip_thresh) != 0) {
                    skip_phase3 = true;
                }
            } else {
                phase3_idle_ticks_ = 0;
            }
        }
        // Phase 3 is replaced entirely when soft-dirty is enabled — the
        // kernel does the write tracking for us via PTE bits, no
        // mprotect storm needed.
        if (!no_monitor && !defer_phases && !skip_phase3 && !softdirty) {
            dispatch(3);  // Phase 3: monitoring
        }
        if (softdirty) {
            // End-of-tick clear so app writes during the next tick get
            // freshly tracked. One syscall, no per-page cost.
            clearSoftDirty();
        }

        // ── Adaptive worker scaling ──────────────────────────────────────
        // Measure this tick's workload and throughput, update EMAs, and
        // compute the number of workers needed for next tick.
        adaptWorkerCount(nw);

        if constexpr (kMeasureCohorts) tallyCohorts(committed);

        trainDictionaries();

        // Drain any deferred-madvise queue entries whose TTL has elapsed.
        // Runs after all phase work so the sweeper observes the freshly
        // queued pages from this tick at full TTL on the next pass.
        sweepDeferredMadvise();
    }

    // Adapt active worker count using Little's Law (see class comment above).
    // CPU-pressure cap on compressor workers. Returns the maximum number of
    // workers we should run given current system load and our own process's
    // thread count, or 0 to mean "no cap."
    //
    // The compressor is background work: every worker that runs competes
    // with the application for cores. The right answer depends on what the
    // application is doing — a single-threaded app on an 8-core box can
    // afford many compressor workers; a 16-threaded app on a 4-core box
    // cannot afford any.
    //
    // Method: read the live thread count for *our process* (/proc/self/stat
    // num_threads field, or task-dir count on macOS), subtract our known
    // compressor workers to get an estimate of app threads, and reserve
    // enough cores for them. Then add a safety floor based on system-wide
    // 1-minute load average to back off when *other* processes are also
    // saturating the box.
    //
    //   app_threads = max(1, process_threads - (helpers_created_ + 1))
    //   sys_pressure = max(0, ceil(loadavg) - app_threads)  // outside contention
    //   cap = max(1, nproc - app_threads - sys_pressure)
    //
    // Cheap, cached: re-evaluated at most every kCpuPressureSampleTicks
    // ticks. Behind SMASH_CPU_PRESSURE_CAP=0 to allow ablation.
    int cpuPressureWorkerCap() {
        static const bool enabled = []{
            const char* v = std::getenv("SMASH_CPU_PRESSURE_CAP");
            return !(v && v[0] == '0');
        }();
        if (!enabled) return 0;

        static const long nproc_cached = []{
            long n = sysconf(_SC_NPROCESSORS_ONLN);
            return n > 0 ? n : 1;
        }();

        constexpr int kCpuPressureSampleTicks = 4;
        if (cpu_pressure_sample_age_ < kCpuPressureSampleTicks) {
            cpu_pressure_sample_age_++;
            return cpu_pressure_cached_cap_;
        }
        cpu_pressure_sample_age_ = 0;

        // Live thread count of our own process. /proc/self/stat field 20 is
        // num_threads, but stat parsing is finicky (the comm field can
        // contain spaces); easier to count entries in /proc/self/task.
        // On macOS, /proc isn't available → fall back to process-info.
        int proc_threads = countOwnThreads();
        if (proc_threads <= 0) {
            cpu_pressure_cached_cap_ = 0;  // can't read → no cap
            return 0;
        }

        // App threads = total minus our own compressor workers.
        int our_workers = helpers_created_ + 1;  // helpers + coordinator
        int app_threads = proc_threads - our_workers;
        if (app_threads < 1) app_threads = 1;

        // System-wide pressure not attributable to our own app.
        double loads[3] = {0.0, 0.0, 0.0};
        int sys_pressure = 0;
        int load_int = 0;
        if (getloadavg(loads, 1) >= 1) {
            load_int = static_cast<int>(loads[0] + 0.5);
            sys_pressure = load_int - app_threads;
            if (sys_pressure < 0) sys_pressure = 0;
        }

        int cap = static_cast<int>(nproc_cached) - app_threads - sys_pressure;
        if (cap < 1) cap = 1;
        cpu_pressure_cached_cap_ = cap;
        return cap;
    }

    // /proc/self/task/ entry count (Linux) or task_threads() (macOS).
    // Returns -1 on error.
    int countOwnThreads() {
#if defined(__linux__)
        DIR* d = opendir("/proc/self/task");
        if (!d) return -1;
        int count = 0;
        while (struct dirent* e = readdir(d)) {
            if (e->d_name[0] != '.') count++;
        }
        closedir(d);
        return count > 0 ? count : -1;
#elif defined(__APPLE__)
        thread_array_t threads;
        mach_msg_type_number_t n = 0;
        if (task_threads(mach_task_self(), &threads, &n) != KERN_SUCCESS)
            return -1;
        // Free the port rights we just received.
        for (mach_msg_type_number_t i = 0; i < n; ++i)
            mach_port_deallocate(mach_task_self(), threads[i]);
        vm_deallocate(mach_task_self(),
                      reinterpret_cast<vm_address_t>(threads),
                      n * sizeof(thread_t));
        return static_cast<int>(n);
#else
        return -1;
#endif
    }

    int cpu_pressure_cached_cap_ = 0;
    int cpu_pressure_sample_age_ = 1000;  // force resample on first call

    // Measures λ (cold arrival rate) and μ (per-worker service rate) each
    // tick, smooths with EMA (α = 1/4), and sets N = ⌈λ_ema / μ_ema⌉.
    void adaptWorkerCount(int nw) {
        // Sum per-worker counters
        uint32_t total_eligible = 0;
        uint32_t total_compressed = 0;
        for (int w = 0; w < nw; ++w) {
            total_eligible += worker_pages_eligible_[w].load(std::memory_order_relaxed);
            total_compressed += worker_pages_compressed_[w].load(std::memory_order_relaxed);
        }

        // Per-worker throughput (avoid division by zero)
        uint32_t mu_sample = nw > 0 ? (total_compressed + nw - 1) / nw : 0;

        // Update EMAs: ema += (sample - ema) / 4  (in fixed-point ×256)
        uint32_t lambda_sample_fp = total_eligible << 8;
        uint32_t mu_sample_fp = mu_sample << 8;

        lambda_ema_ += (static_cast<int32_t>(lambda_sample_fp) - static_cast<int32_t>(lambda_ema_)) / 4;
        // Only update μ when we have compression work (avoid decaying μ to 0
        // during idle periods, which would cause spurious scale-up)
        if (total_compressed > 0) {
            mu_ema_ += (static_cast<int32_t>(mu_sample_fp) - static_cast<int32_t>(mu_ema_)) / 4;
            if (mu_ema_ == 0) mu_ema_ = 1;  // floor to prevent division by zero
        }

        // N_needed = ⌈λ / μ⌉ (both in same fixed-point scale, so they cancel)
        int n_needed;
        if (lambda_ema_ == 0) {
            n_needed = 1;  // no work → 1 worker is enough
        } else {
            n_needed = static_cast<int>((lambda_ema_ + mu_ema_ - 1) / mu_ema_);
        }

        // Clamp to valid range
        if (n_needed < 1) n_needed = 1;
        if (n_needed > kMaxCompressorWorkers) n_needed = kMaxCompressorWorkers;

        // CPU-pressure cap. The compressor's worker threads compete with
        // application threads for cores; on a saturated machine, scaling up
        // adds context-switch latency to the very fault handlers we depend on
        // for forward progress. Cap workers at (nproc − 1) when 1-minute load
        // average ≥ nproc. This leaves at least one core for the application
        // and prevents the worst-case scenario where every core is running
        // compressor work and the app times out waiting on a fault.
        int cap = cpuPressureWorkerCap();
        if (cap > 0 && n_needed > cap) n_needed = cap;

        // Ensure worker state + helper threads exist for the new count
        // (lazily allocated on first scale-up, never destroyed)
        for (int w = 0; w < n_needed; ++w)
            ensureWorkerState(w);
        while (helpers_created_ < n_needed - 1) {
            int helper_id = helpers_created_;
            auto* ha = static_cast<HelperArg*>(
                BootstrapAlloc::instance().allocate(sizeof(HelperArg), alignof(HelperArg)));
            ha->self = this;
            ha->id = helper_id;
            // EAGAIN under thread exhaustion: stop scaling up rather than
            // counting a helper that does not exist (dispatch would wait
            // forever for its generation ack; stop() would join garbage).
            if (pthread_create(&helper_threads_[helper_id], nullptr, helperEntry, ha) != 0)
                break;
            helpers_created_++;
        }
        // Never claim more active workers than threads that actually exist
        // (coordinator + created helpers).
        if (n_needed > helpers_created_ + 1) n_needed = helpers_created_ + 1;

        active_workers_ = n_needed;
    }

    void tallyCohorts(size_t committed) {
        if (!cohort_pages_ || cohort_pages_len_ == 0) return;
        size_t lim = committed < cohort_pages_len_ ? committed : cohort_pages_len_;
        size_t active_total = 0, active_mixed_tid = 0, active_mixed_ra = 0;
        size_t comp_total = 0, comp_mixed_tid = 0, comp_mixed_ra = 0;
        size_t all_total = 0, all_mixed_tid = 0, all_mixed_ra = 0;
        for (size_t i = 0; i < lim; ++i) {
            auto& cp = cohort_pages_[i];
            if (cp.first_tid == 0) continue;
            all_total++;
            if (cp.mixed_tid) all_mixed_tid++;
            if (cp.mixed_ra) all_mixed_ra++;
            PageState st = states_->get(i);
            if (st == PageState::ACTIVE || st == PageState::ACTIVE_MONITORING) {
                active_total++;
                if (cp.mixed_tid) active_mixed_tid++;
                if (cp.mixed_ra) active_mixed_ra++;
            } else if (st == PageState::COMPRESSED) {
                comp_total++;
                if (cp.mixed_tid) comp_mixed_tid++;
                if (cp.mixed_ra) comp_mixed_ra++;
            }
        }
        auto pct = [](size_t n, size_t d) -> double {
            return d > 0 ? 100.0 * n / d : 0.0;
        };
        fprintf(stderr,
            "[cohort] committed=%zu stamped=%zu  "
            "all=%zu mixed_tid=%zu(%.1f%%) mixed_ra=%zu(%.1f%%)  "
            "active=%zu mixed_tid=%zu(%.1f%%) mixed_ra=%zu(%.1f%%)  "
            "compressed=%zu mixed_tid=%zu(%.1f%%) mixed_ra=%zu(%.1f%%)\n",
            committed, all_total,
            all_total, all_mixed_tid, pct(all_mixed_tid, all_total),
            all_mixed_ra, pct(all_mixed_ra, all_total),
            active_total, active_mixed_tid, pct(active_mixed_tid, active_total),
            active_mixed_ra, pct(active_mixed_ra, active_total),
            comp_total, comp_mixed_tid, pct(comp_mixed_tid, comp_total),
            comp_mixed_ra, pct(comp_mixed_ra, comp_total));
    }

    // ── Thread entry points ───────────────────────────────────────────────

    // Block all asynchronous signals on smash's own compressor/helper threads.
    // A process-directed signal (kill(getpid(), N) / sigqueue) is delivered to
    // an ARBITRARY thread that does not block it. If a smash background thread
    // is chosen, the host application's own handler/waiter never sees it. This
    // breaks apps that use process-directed signals for their control plane —
    // e.g. MariaDB sends SIGUSR2 to wake its signal thread during shutdown; if
    // a smash thread absorbs it, mysqld_main hangs forever in
    // wait_for_signal_thread_to_end(). Blocking here forces every async signal
    // to a host thread. SIGSEGV/SIGBUS/SIGABRT/SIGFPE/SIGILL are left unblocked:
    // they are synchronous (thread-directed at the faulting instruction) and
    // cannot be meaningfully rerouted; the fault handler must still see a
    // genuine fault on this thread, and a real abort/FPE should still fire.
    static void blockAsyncSignalsOnSelf() {
        sigset_t set;
        sigfillset(&set);
        sigdelset(&set, SIGSEGV);
        sigdelset(&set, SIGBUS);
        sigdelset(&set, SIGABRT);
        sigdelset(&set, SIGFPE);
        sigdelset(&set, SIGILL);
        pthread_sigmask(SIG_BLOCK, &set, nullptr);
    }

    __attribute__((cold)) static void* coordEntry(void* arg) {
        g_compressor_thread = true;
        blockAsyncSignalsOnSelf();
        auto* self = static_cast<CompressorThread*>(arg);
        while (self->running_.load(std::memory_order_relaxed)) {
            // Sleep in 10ms intervals, checking running_ flag each time.
            // This keeps the tick interval at kCompressIntervalMs (1 second)
            // while allowing fast shutdown response (~10ms worst case).
            constexpr int kSleepIntervalMs = 10;
            for (int slept = 0;
                 slept < kCompressIntervalMs && self->running_.load(std::memory_order_relaxed);
                 slept += kSleepIntervalMs) {
                usleep(kSleepIntervalMs * 1000);
            }
            if (!self->running_.load(std::memory_order_relaxed)) break;
            // Skip the tick if a fork is in progress. The atfork prepare
            // handler set paused_ and is waiting for in_tick_ to be 0;
            // entering tick() now would race the impending fork().
            //
            // Dekker-style ordering: PUBLISH in_tick_ = true FIRST, then
            // check paused_; pauseForFork() does the mirror image (store
            // paused_, then read in_tick_). With both stores and loads
            // sequentially consistent, at least one side observes the
            // other's store: either we see paused_ and skip (clearing
            // in_tick_), or prepare sees in_tick_ and waits the tick out.
            // The previous check-then-set order had a TOCTOU window: the
            // coordinator reads paused_ == false, prepare then sets
            // paused_ and reads in_tick_ == 0 (not stored yet), fork()
            // proceeds while tick() runs holding page/store locks — and
            // the child inherits those spinlocks locked forever. Modeled
            // in model/SmashForkPause.tla (the check-then-set cfg
            // reproduces the violation; this order passes) and proved in
            // model/SmashForkPause.lean.
            self->in_tick_.store(true, std::memory_order_seq_cst);
            if (self->paused_.load(std::memory_order_seq_cst)) {
                self->in_tick_.store(false, std::memory_order_release);
                continue;
            }
            self->tick();
            self->in_tick_.store(false, std::memory_order_release);
        }
        return nullptr;
    }

    struct HelperArg {
        CompressorThread* self;
        int id;  // 0-based helper index (worker index = id + 1)
    };

    __attribute__((cold)) static void* helperEntry(void* arg) {
        g_compressor_thread = true;
        blockAsyncSignalsOnSelf();
        auto* ha = static_cast<HelperArg*>(arg);
        auto* self = ha->self;
        int helper_id = ha->id;
        int worker_id = helper_id + 1;
        uint64_t last_gen = 0;

        while (self->running_.load(std::memory_order_relaxed)) {
            uint64_t gen = self->work_gen_.load(std::memory_order_acquire);
            if (gen == last_gen) {
                // Park until dispatch() bumps work_gen_ or stop() clears
                // running_ (was: usleep(100) polling; see util/waker.h).
                self->work_waker_.waitUntil([&] {
                    return self->work_gen_.load(std::memory_order_acquire) != last_gen ||
                           !self->running_.load(std::memory_order_relaxed);
                }, 100);
                continue;
            }
            last_gen = gen;

            // Only execute if this worker is active; otherwise just ACK
            // so the coordinator's dispatch() doesn't block on parked helpers.
            if (worker_id < self->active_workers_) {
                self->executePhase(self->current_phase_, worker_id);
            }
            self->helper_done_gen_[helper_id].store(gen, std::memory_order_release);
        }
        return nullptr;
    }

public:
    // Pointer to the per-page cold-tick counter array, valid after init().
    // Slab::allocate exposes this to Span so freshly-allocated chunks reset
    // the cold counter for their containing page — pages still receiving
    // allocations should never be eligible for compression.
    uint8_t* coldCounts() const { return cold_count_; }

    void init(VmRegion* vm, PageStateTable* states, PageLockTable* locks,
              CompressStore* store, CompressEngine* engine,
              PageMap* page_map = nullptr,
              vm::FaultHandler* fault_handler = nullptr) {
        vm_ = vm;
        states_ = states;
        locks_ = locks;
        store_ = store;
        engine_ = engine;
        page_map_ = page_map;
        fault_handler_ = fault_handler;
        pre_tick_fn_ = nullptr;

        size_t max_pages = vm->totalPages();
        compressed_ = bootstrapArray<CompressedPageInfo>(max_pages);
        accessed_ = bootstrapArray<std::atomic<bool>>(max_pages);
        cold_count_ = bootstrapArray<uint8_t>(max_pages);
        recompress_count_ = bootstrapArray<uint8_t>(max_pages);
        compress_fail_count_ = bootstrapArray<uint8_t>(max_pages);
        // Soft-dirty ROI per-page arrays: only allocate when the gate is on
        // (~2 bytes/page = ~32 MiB at full VM otherwise wasted). Read the env
        // directly here — ROIConfig::init() runs later in this function, so the
        // singleton flag isn't set yet at allocation time.
        if (const char* sr = std::getenv("SMASH_SOFTDIRTY_ROI"); sr && std::atoi(sr) != 0) {
            write_clean_streak_ = bootstrapArray<uint8_t>(max_pages);
        }
        if (isDeferredReclaimMode())
            shadow_tick_ = bootstrapArray<uint32_t>(max_pages);
        page_tier_ = bootstrapArray<uint8_t>(max_pages);
        // Always allocate; sweeper short-circuits on the env knob anyway,
        // and unconditional init keeps pointer-null checks out of fault path.
        deferred_pending_ = bootstrapArray<std::atomic<bool>>(max_pages);
        deferred_queue_tick_ = bootstrapArray<uint32_t>(max_pages);

        // Per-bucket recompression EMA table (kNumArenas × kNumClasses entries).
        bucket_rc_ema_x256_ = bootstrapArray<std::atomic<uint16_t>>(kBucketTableLen);
        bucket_rc_count_    = bootstrapArray<std::atomic<uint8_t>>(kBucketTableLen);

        // Chunk bitmap
        num_chunks_ = (max_pages + kChunkSize - 1) / kChunkSize;
        live_chunks_ = bootstrapArray<uint64_t>(num_chunks_);

        // Pre-allocate per-worker state
        size_t max_comp = CompressEngine::maxCompressedSizeAny(kPageSize);
        ZSTD_customMem custom_mem = {
            [](void*, size_t sz) -> void* {
                return BootstrapAlloc::instance().allocate(sz, 16);
            },
            [](void*, void*) {},
            nullptr
        };

        // Pre-allocate state for the initial worker count only. Additional
        // workers get their state allocated lazily on first scale-up via
        // ensureWorkerState(), saving ~300KB per deferred worker at startup.
        for (int w = 0; w < kCompressorWorkers; ++w) {
            initWorkerState(workers_[w], max_comp, custom_mem);
        }
        max_comp_size_ = max_comp;
        zstd_custom_mem_ = custom_mem;

        // Pre-allocate fault handler slots with per-slot DCtx
        for (int i = 0; i < kFaultSlotCount; ++i) {
            fault_slots_[i].buf = BootstrapAlloc::instance().allocate(kPageSize, kPageSize);
            fault_slots_[i].dctx = ZSTD_createDCtx_advanced(custom_mem);
        }

        // Force-initialize all function-local statics that call getenv()
        // BEFORE the fault handler starts. If any of these first-init on the
        // compressor thread while inside a fault context, getenv traverses
        // environ on a smash-compressed page → recursive SIGSEGV → crash.
        warmupEnvStatics();
        warmupClassStatics();

        // Pre-open /proc/self/mem so the fault handler doesn't need to
        // call open() (which may trigger PLT resolution on a compressed page).
#ifdef __linux__
        if (self_mem_fd_ < 0)
            self_mem_fd_ = open("/proc/self/mem", O_RDWR | O_CLOEXEC);
#endif

        // Initialize ROI model (auto-calibrate throughput, read env vars)
        ROIConfig::instance().init(engine_);
    }

    void setPreTickCallback(PreTickFn fn) { pre_tick_fn_ = fn; }

    // A3: feedback hook invoked once per successful page compression
    // with the originating span's arena_id and size_class.
    void setCompressedCallback(CompressedFn fn, void* ctx) {
        compressed_fn_ = fn;
        compressed_ctx_ = ctx;
    }

    void setDecompressedCallback(DecompressedFn fn, void* ctx) {
        decompressed_fn_ = fn;
        decompressed_ctx_ = ctx;
    }

    void setCohortData(void* pages, size_t len) {
        cohort_pages_ = static_cast<CohortPage*>(pages);
        cohort_pages_len_ = len;
    }

    // ── Profile persistence (P3) ─────────────────────────────────────────
    // SMASH_PROFILE_FILE=<path>: path to a binary file used to persist
    // per-bucket compression statistics across runs. On start() we load
    // the file (if it exists and has the right version); on stop() we
    // write a fresh snapshot atomically via rename(2). Buckets marked
    // ALWAYS_SKIP short-circuit phase2 immediately, eliminating ROI cost
    // and per-page lock acquisitions for known-uncompressible buckets.
    static constexpr uint32_t kProfileMagic = 0x53503031u;  // "SP01"
    // v3 added large-allocation pseudo size-classes (kNumLargeClasses entries
    // beyond kNumClasses), so the on-disk record array length grew from
    // kNumArenas*kNumClasses to kNumArenas*kTotalBucketsPerArena.
    // v4 appends a per-size-class dictionary section so warm-mode runs can
    // skip the (~50–200 ms) ZDICT_trainFromBuffer step.
    // v5 adds external_pages_hot flag in header.flags to skip external mmap
    // pages that thrashed in a prior profiling run.
    // Older v1/v2/v3/v4 files are silently rejected by loadProfileFile
    // (header version mismatch), matching the existing pattern for stale
    // persisted profiles.
    static constexpr uint32_t kProfileVersion = 6;

    // Sentinel size_class value marking the end of the dictionary section.
    // 255 was chosen because real size_class values are well below
    // kTotalBucketsPerArena (≈ 44), so 255 is a safe out-of-band marker.
    static constexpr uint8_t kDictSectionEnd = 255;
    // Hard cap on a single dict's serialized size. Matches kDictCapacity
    // (16 KiB), the same upper bound enforced by trainDictionary.
    static constexpr uint32_t kMaxDictBytes = 64 * 1024;

    struct ProfileHeader {
        uint32_t magic;
        uint32_t version;
        uint16_t num_arenas;
        // Total bucket count per arena (kTotalBucketsPerArena). Field name
        // kept as num_classes for ABI compatibility with older headers; the
        // version bump distinguishes v2/v3/v4 layouts.
        uint16_t num_classes;
        // Flags: bit 0 = external_pages_hot (external mmap pages thrashed
        // during the profiling run, so skip them in future runs).
        uint32_t flags;
    };
    static constexpr uint32_t kProfileFlagExternalHot = 1;

    static const char* profileFilePath() {
        static const char* p = []{
            const char* v = getenv("SMASH_PROFILE_FILE");
            return (v && v[0]) ? v : nullptr;
        }();
        return p;
    }

    // Whether to persist the profile + dict at exit. SMASH_PROFILE_FILE_RW
    // implies SAVE; SMASH_PROFILE_FILE_SAVE controls it explicitly. Default
    // off so warm-only runs (the common case in production) don't overwrite
    // a carefully-trained file with their short-run observations.
    static bool shouldSaveProfile() {
        return getProfileFileSave() || getProfileFileRW();
    }

    void loadProfileFile() {
        const char* path = profileFilePath();
        if (!path) return;
        FILE* f = fopen(path, "rb");
        if (!f) return;
        ProfileHeader hdr{};
        if (fread(&hdr, sizeof(hdr), 1, f) != 1 ||
            hdr.magic != kProfileMagic ||
            hdr.version != kProfileVersion ||
            hdr.num_arenas != kNumArenas ||
            hdr.num_classes != kTotalBucketsPerArena) {
            fclose(f);
            return;
        }
        // Load external_pages_hot flag from profile
        if (hdr.flags & kProfileFlagExternalHot) {
            external_pages_hot_from_profile_ = true;
            // Also set the global flag so mmap interposers skip tracking
            g_smash_skip_external_tracking.store(true, std::memory_order_release);
            fprintf(stderr, "[smash] profile loaded: external_pages_hot=true\n");
        }
        size_t total = static_cast<size_t>(kNumArenas) * kTotalBucketsPerArena;
        for (size_t i = 0; i < total; ++i) {
            SizeClassStats::Persist rec{};
            if (fread(&rec, sizeof(rec), 1, f) != 1) {
                // Truncated profile section — accept what we read so far,
                // skip dict section. Cold-start the rest.
                fclose(f);
                return;
            }
            for (int w = 0; w < kMaxCompressorWorkers; ++w) {
                workers_[w].sc_stats[i].deserialize(rec);
                workers_[w].sc_stats[i].persist_hint = rec.decision_hint;
            }
        }

        // Optional dictionary section. Each record is
        //   u8 size_class
        //   u32 dict_size
        //   <dict_size bytes>
        // terminated by size_class == kDictSectionEnd. A truncated tail is
        // not fatal — we treat it the same as "no more dicts" and exit.
        // Stack buffer keeps us off bootstrap on the load-only fast path.
        uint8_t dict_buf[kMaxDictBytes];
        for (;;) {
            uint8_t sc = 0;
            if (fread(&sc, sizeof(sc), 1, f) != 1) break;
            if (sc == kDictSectionEnd) break;
            uint32_t dsize = 0;
            if (fread(&dsize, sizeof(dsize), 1, f) != 1) break;
            if (dsize == 0 || dsize > kMaxDictBytes || sc >= kNumClasses) {
                // Bad record — abort dict-load but leave already-loaded
                // dicts in place. Don't propagate the error: the profile
                // section was usable and we already have a partial load.
                break;
            }
            if (fread(dict_buf, dsize, 1, f) != 1) break;
            if (engine_) {
                engine_->setDictionary(sc, dict_buf, dsize);
                // Mark dict_train_ as already trained so the cold-start
                // sample-collection path is skipped for this size class.
                if (sc < kNumClasses) dict_train_[sc].trained = true;
            }
        }
        fclose(f);
    }

    void saveProfileFile() {
        if (!shouldSaveProfile()) return;
        const char* path = profileFilePath();
        if (!path) return;
        size_t total = static_cast<size_t>(kNumArenas) * kTotalBucketsPerArena;

        // Build THIS process's per-bucket records in memory first (aggregate
        // across workers: highest sample count wins per bucket).
        SizeClassStats::Persist* mine = bootstrapArray<SizeClassStats::Persist>(total);
        for (size_t i = 0; i < total; ++i) {
            int best = 0;
            for (int w = 1; w < kMaxCompressorWorkers; ++w) {
                if (workers_[w].sc_stats[i].count >
                    workers_[best].sc_stats[i].count) best = w;
            }
            workers_[best].sc_stats[i].serialize(&mine[i]);
        }
        uint32_t my_flags = 0;
        if (external_pages_hot_.load(std::memory_order_relaxed))
            my_flags |= kProfileFlagExternalHot;

        // ── Cross-process merge ────────────────────────────────────────────
        // neuron-cc compiles in a ProcessPoolExecutor: the heavy slab churn
        // lives in hlo2penguin / walrus_driver subprocesses, while the parent
        // driver barely touches the managed heap. The old code did a bare
        // rename(tmp.PID -> path), i.e. last-writer-wins — so the parent's
        // EMPTY profile routinely clobbered the subprocesses' real bucket
        // histories, leaving a profile of all-zero buckets that guides
        // nothing. Verified empirically (90 KB profile, 5632 buckets, every
        // count==0). Fix: serialize all saves through an flock on a sidecar
        // lock file and MERGE into the existing profile, keeping whichever
        // record has the higher sample count per bucket. Concurrent
        // subprocess exits now accumulate instead of overwrite, and the
        // empty-heap parent can never wipe real data.
        char lockpath[1100];
        snprintf(lockpath, sizeof(lockpath), "%s.lock", path);
        int lockfd = open(lockpath, O_CREAT | O_RDWR | O_CLOEXEC, 0644);
        if (lockfd >= 0) flock(lockfd, LOCK_EX);

        // Read existing profile records (if any, same version/dims) and merge.
        FILE* ex = fopen(path, "rb");
        if (ex) {
            ProfileHeader ehdr{};
            if (fread(&ehdr, sizeof(ehdr), 1, ex) == 1 &&
                ehdr.magic == kProfileMagic && ehdr.version == kProfileVersion &&
                ehdr.num_arenas == kNumArenas &&
                ehdr.num_classes == kTotalBucketsPerArena) {
                my_flags |= ehdr.flags;  // external-hot is sticky across procs
                for (size_t i = 0; i < total; ++i) {
                    SizeClassStats::Persist disk{};
                    if (fread(&disk, sizeof(disk), 1, ex) != 1) break;
                    // Higher sample count is the more-informed observation.
                    if (disk.count > mine[i].count) mine[i] = disk;
                }
            }
            fclose(ex);
        }

        // Write the merged result to a temp file and atomically rename.
        char tmp[1100];
        snprintf(tmp, sizeof(tmp), "%s.tmp.%d", path, (int)getpid());
        FILE* f = fopen(tmp, "wb");
        if (!f) {
            if (lockfd >= 0) { flock(lockfd, LOCK_UN); close(lockfd); }
            return;
        }
        ProfileHeader hdr{kProfileMagic, kProfileVersion,
                          static_cast<uint16_t>(kNumArenas),
                          static_cast<uint16_t>(kTotalBucketsPerArena), my_flags};
        fwrite(&hdr, sizeof(hdr), 1, f);
        for (size_t i = 0; i < total; ++i)
            fwrite(&mine[i], sizeof(mine[i]), 1, f);

        // Dictionary section (v4). Skipping the section is fine: the
        // sentinel is the only thing v4 readers require to find EOF before
        // returning, and a missing-dict-section is then equivalent to
        // "no dictionaries trained" — which is exactly the case if the
        // process never hit kDictTrainSamples. (Dicts are not merged across
        // processes; whichever save runs last contributes its own. The
        // per-bucket records — which drive persist_hint — are what matter for
        // churn suppression and those ARE merged above.)
        if (engine_) {
            for (int sc = 0; sc < kNumClasses; ++sc) {
                if (!engine_->hasDictionary(static_cast<uint8_t>(sc)))
                    continue;
                const void* db = engine_->dictBytes(static_cast<uint8_t>(sc));
                size_t ds = engine_->dictSize(static_cast<uint8_t>(sc));
                if (!db || ds == 0 || ds > kMaxDictBytes) continue;
                uint8_t sc8 = static_cast<uint8_t>(sc);
                uint32_t ds32 = static_cast<uint32_t>(ds);
                fwrite(&sc8, sizeof(sc8), 1, f);
                fwrite(&ds32, sizeof(ds32), 1, f);
                fwrite(db, ds, 1, f);
            }
        }
        uint8_t end = kDictSectionEnd;
        fwrite(&end, sizeof(end), 1, f);

        fclose(f);
        rename(tmp, path);
        if (lockfd >= 0) { flock(lockfd, LOCK_UN); close(lockfd); }
    }

    // Env-var cache: read once at init time, stored as member variables.
    // Avoids function-local statics with getenv() that could first-init
    // inside the signal handler on the compressor thread → crash.
    bool cfg_safe_memcpy_ = false;
    bool cfg_snapshot_verify_ = false;
    bool cfg_no_monitor_ = false;
    int cfg_phase3_skip_thresh_ = 20;
    int cfg_defer_phases_ms_ = 0;
    long cfg_vma_margin_ = 8192;   // SMASH_VMA_MARGIN; see VMA-cap guard in tick()
    bool cfg_spill_ = false;       // SMASH_SPILL: file-back very-cold blobs
    bool cfg_spill_pageout_ = false; // SMASH_SPILL_PAGEOUT: MADV_PAGEOUT accelerant
    long cfg_spill_max_mb_ = 1024; // SMASH_SPILL_MAX_MB: spill file size cap (default 1 GiB)
    const char* cfg_spill_dir_ = nullptr;  // SMASH_SPILL_DIR (else /var/tmp, cwd)

    void warmupClassStatics() {
        const char* v;
        v = std::getenv("SMASH_SAFE_MEMCPY");
        cfg_safe_memcpy_ = v && v[0] == '1';
        v = std::getenv("SMASH_SNAPSHOT_VERIFY");
        cfg_snapshot_verify_ = v && v[0] == '1';
        v = std::getenv("SMASH_NO_MONITOR");
        cfg_no_monitor_ = v && v[0] == '1';
        v = std::getenv("SMASH_PHASE3_SKIP_THRESHOLD");
        cfg_phase3_skip_thresh_ = v ? std::atoi(v) : 20;
        v = std::getenv("SMASH_DEFER_PHASES_MS");
        cfg_defer_phases_ms_ = v ? std::atoi(v) : 0;
        v = std::getenv("SMASH_VMA_MARGIN");
        cfg_vma_margin_ = v ? std::atol(v) : 8192L;
        v = std::getenv("SMASH_SPILL");
        cfg_spill_ = v && v[0] == '1';
        v = std::getenv("SMASH_SPILL_PAGEOUT");
        cfg_spill_pageout_ = v && v[0] == '1';
        v = std::getenv("SMASH_SPILL_MAX_MB");
        cfg_spill_max_mb_ = v ? std::atol(v) : 1024L;
        cfg_spill_dir_ = std::getenv("SMASH_SPILL_DIR");  // may be null
        // Touch remaining statics to force their initialization:
        (void)std::getenv("SMASH_PROTECT_CHUNK_PAGES");
        (void)std::getenv("SMASH_P2_CHUNK");
        (void)std::getenv("SMASH_SOFTDIRTY");
        (void)std::getenv("SMASH_IDLE_READ_TRACK");
        (void)std::getenv("SMASH_CPU_PRESSURE_CAP");
    }

    void start() {
        running_.store(true, std::memory_order_release);
        start_steady_time_ = std::chrono::steady_clock::now();

        // P3: load persisted bucket stats before any tick fires.
        loadProfileFile();

        // SIGUSR2 stats dump: walk page-state table, write a one-line summary
        // to stderr. Lets us observe whether smash is actually compressing
        // anything during a Firefox run without rebuilding. Use SIGUSR2 (not
        // SIGUSR1) because Firefox installs a SIGUSR1 handler of its own.
        //
        // OFF BY DEFAULT (opt in with SMASH_STATS_SIGNAL=1). Claiming SIGUSR2
        // hijacks it from hosts that use it for their own control plane —
        // e.g. MariaDB sends SIGUSR2 to end its signal thread at shutdown, so
        // stealing it hangs mysqld_main forever. The env-var check is cached
        // once; the tests that drive the SIGUSR2 stats path set it explicitly.
        s_stats_instance_ = this;
        if (statsSignalEnabled()) {
            struct sigaction sa{};
            sa.sa_handler = &CompressorThread::sigusr1Handler;
            sa.sa_flags = SA_RESTART;
            sigemptyset(&sa.sa_mask);
            sigaction(SIGUSR2, &sa, nullptr);
        }

        // SMASH_STATS=1: also emit a stats line on every normal process
        // exit. atexit handlers are inherited across fork(), so each child
        // of a multi-process app (Firefox parent + content + GPU + …)
        // prints its own line as it shuts down. _exit() and SIGKILL skip
        // atexit by design.
        const char* stats_env = std::getenv("SMASH_STATS");
        if (stats_env && stats_env[0] == '1') {
            std::atexit([]() {
                if (s_stats_instance_) sigusr1Handler(0);
            });
        }

        // Register an atexit handler to join the coordinator + helper
        // threads on normal process exit. Without this, the threads spin
        // in their `running_.load()` loops indefinitely after main()
        // returns and the host process can't exit cleanly (observed on
        // walrus_driver post-compile: state=I, no work to do, alive
        // 500+s). stop() is idempotent — guarded by `running_.load()` —
        // so it's safe to call multiple times and safe to inherit across
        // fork. atexit registers in LIFO order, so this runs before the
        // SMASH_STATS handler above; that's fine, stats only walks the
        // page-state table and doesn't depend on the threads being live.
        //
        // SMASH_NO_ATEXIT=1: Skip atexit registration for debugging. The
        // compressor threads will be orphaned on exit, which is fine for
        // testing atexit-ordering hypotheses (process exits via _exit anyway).
        const char* no_atexit = std::getenv("SMASH_NO_ATEXIT");
        if (!no_atexit || no_atexit[0] != '1') {
            s_stop_instance_ = this;
            std::atexit([]() {
                if (s_stop_instance_) s_stop_instance_->stop();
            });
        }

        // SMASH_DEBUG=1: emit a banner now (compressor came up) and a
        // stats line every Nth tick during the run. Distinct from
        // SMASH_STATS=1 (atexit-only) — SMASH_DEBUG is for watching
        // activity live without having to chase PIDs and send SIGUSR2.
        const char* dbg_env = std::getenv("SMASH_DEBUG");
        s_debug_enabled_ = dbg_env && dbg_env[0] == '1';
        if (s_debug_enabled_) {
            char ts[32] = {};
            vm::formatTimestamp(ts, sizeof(ts));
            char buf[200];
            int n = smash::safe_snprintf(buf, sizeof(buf),
                "[smash debug] [%s] compressor start pid=%d workers=%d\n",
                ts, (int)getpid(), kCompressorWorkers);
            if (n > 0) (void)!write(2, buf, (size_t)n);
        }

        // Pre-create ALL helper threads at startup. Calling pthread_create
        // later (during tick) crashes: glibc's _dl_allocate_tls_init walks all
        // loaded modules' link_maps, some of which live on smash-compressed
        // (PROT_NONE) pages. The signal handler handles a few faults but the
        // kernel stops delivering SIGSEGV after ~3 rapid cycles.
        // Use a small stack (128 KiB) since helpers only run compressTick
        // which uses BootstrapAlloc-backed buffers, not stack allocations.
        pthread_attr_t helper_attr;
        pthread_attr_init(&helper_attr);
        pthread_attr_setstacksize(&helper_attr, 128 * 1024);

        for (int w = 0; w < kMaxCompressorWorkers; ++w) ensureWorkerState(w);
        int created = 0;
        for (int i = 0; i < kMaxHelpers; ++i) {
            auto* ha = static_cast<HelperArg*>(
                BootstrapAlloc::instance().allocate(sizeof(HelperArg), alignof(HelperArg)));
            ha->self = this;
            ha->id = i;
            // Under thread/memory exhaustion pthread_create returns EAGAIN.
            // Record only the helpers that actually exist: helpers_created_
            // drives both the adaptive worker cap and stop()'s joins, and
            // joining a never-created pthread_t{} is undefined behavior.
            if (pthread_create(&helper_threads_[i], &helper_attr, helperEntry, ha) != 0)
                break;
            ++created;
        }
        helpers_created_ = created;

        // Start coordinator thread (same small stack). Without it nothing
        // ever ticks: roll back running_ so the helpers (which loop on it)
        // exit, stop() early-returns instead of joining a never-created
        // handle, and the library degrades to a plain allocator instead of
        // crashing at shutdown.
        if (pthread_create(&coord_thread_, &helper_attr, coordEntry, this) != 0) {
            running_.store(false, std::memory_order_release);
            work_waker_.notifyAll();
            char buf[128];
            int n = smash::safe_snprintf(buf, sizeof(buf),
                "[smash WARN] pthread_create(coordinator) failed; "
                "compression disabled\n");
            if (n > 0) (void)!write(2, buf, (size_t)n);
        }
        pthread_attr_destroy(&helper_attr);
    }

    static inline CompressorThread* s_stats_instance_ = nullptr;
    static inline CompressorThread* s_stop_instance_ = nullptr;
    static inline bool s_debug_enabled_ = false;

    // SMASH_STATS_SIGNAL=1 opts into installing the SIGUSR2 stats-dump handler
    // (and the per-tick reclaim). OFF by default so smash never hijacks
    // SIGUSR2 from a host application that uses it for its own control plane
    // (e.g. MariaDB's signal thread → shutdown hang). Cached once.
    static bool statsSignalEnabled() {
        static const bool enabled = []{
            const char* v = std::getenv("SMASH_STATS_SIGNAL");
            return v && v[0] == '1';
        }();
        return enabled;
    }

    static void sigusr1Handler(int) {
        auto* self = s_stats_instance_;
        if (!self || !self->states_ || !self->vm_) return;
        size_t empty = 0, active = 0, monitor = 0, compressing = 0,
               compressed = 0, shadow = 0, total = self->vm_->committedPages();
        size_t raw_bump = self->vm_->rawNextPage();
        for (size_t i = 0; i < total; ++i) {
            switch (self->states_->get(i)) {
            case PageState::EMPTY: ++empty; break;
            case PageState::ACTIVE: ++active; break;
            case PageState::ACTIVE_MONITORING: ++monitor; break;
            case PageState::COMPRESSING: ++compressing; break;
            case PageState::COMPRESSED: ++compressed; break;
            case PageState::COMPRESSED_SHADOW: ++shadow; break;
            }
        }
        char ts[32] = {};
        vm::formatTimestamp(ts, sizeof(ts));
        uint64_t tier_attempts = self->tier_upgrade_attempts_.load(
            std::memory_order_relaxed);
        uint64_t tier_success = self->tier_upgrade_success_.load(
            std::memory_order_relaxed);
        size_t spilled = self->spilled_count_.load(std::memory_order_relaxed);
        char buf[416];
        int n = smash::safe_snprintf(buf, sizeof(buf),
            "[smash stats] [%s] pid=%d committed=%zu  active=%zu  monitor=%zu"
            "  compressing=%zu  compressed=%zu  shadow=%zu  empty=%zu"
            "  tier_up=%llu/%llu  spilled=%zu  zero=%llu  bump=%zu"
            "  reheat=%llu\n",
            ts, (int)getpid(), total, active, monitor, compressing, compressed,
            shadow, empty,
            (unsigned long long)tier_success,
            (unsigned long long)tier_attempts,
            spilled,
            (unsigned long long)self->zero_pages_.load(std::memory_order_relaxed),
            raw_bump,
            (unsigned long long)g_reheat_events.load(std::memory_order_relaxed));
        // Cast to void to silence -Wunused-result on glibc (write is
        // marked __wur there). We're inside a signal handler — there's
        // no useful recovery if write() short-returns.
        if (n > 0) (void)!write(2, buf, (size_t)n);

        // SMASH_BUCKET_STATS=1: dump per-(arena, size_class) stats
        static const bool bucket_stats = []{
            const char* v = std::getenv("SMASH_BUCKET_STATS");
            return v && v[0] == '1';
        }();
        if (bucket_stats) {
            dumpBucketStats();
        }
    }

    // Dump per-(arena, size_class) bucket compressibility and coldness stats.
    // Output format: one line per bucket with samples, sorted by sample count.
    // Columns: arena, sc, obj_size, samples, ratio%, compress, decompress, thrash%
    static void dumpBucketStats() {
        auto* self = s_stats_instance_;
        if (!self) return;

        // Aggregate across workers: take worker with max count per bucket
        struct BucketSummary {
            uint8_t arena;
            uint8_t sc;
            size_t obj_size;
            uint8_t count;
            uint16_t ratio_sum;  // sum of ratios (divide by count for mean)
            uint32_t compress_count;
            uint32_t decompress_count;
        };
        constexpr size_t kMaxBuckets = kNumArenas * kTotalBucketsPerArena;
        BucketSummary summaries[kMaxBuckets];
        size_t num_summaries = 0;

        for (int arena = 0; arena < kNumArenas; ++arena) {
            for (int sc = 0; sc < static_cast<int>(kTotalBucketsPerArena); ++sc) {
                size_t idx = statsIndex(static_cast<uint8_t>(arena),
                                        static_cast<uint8_t>(sc));
                // Find worker with most samples
                int best_w = 0;
                for (int w = 1; w < kMaxCompressorWorkers; ++w) {
                    if (self->workers_[w].sc_stats[idx].count >
                        self->workers_[best_w].sc_stats[idx].count) {
                        best_w = w;
                    }
                }
                const auto& s = self->workers_[best_w].sc_stats[idx];
                if (s.count == 0) continue;  // skip empty buckets

                // Sum compress/decompress across all workers
                uint32_t cc = 0, dc = 0;
                for (int w = 0; w < kMaxCompressorWorkers; ++w) {
                    cc += self->workers_[w].sc_stats[idx].compress_count
                              .load(std::memory_order_relaxed);
                    dc += self->workers_[w].sc_stats[idx].decompress_count
                              .load(std::memory_order_relaxed);
                }

                size_t obj_size = sc < kNumClasses
                    ? kSizeClasses[sc].size
                    : (1ULL << (sc - kNumClasses + 14));
                BucketSummary bs;
                bs.arena = static_cast<uint8_t>(arena);
                bs.sc = static_cast<uint8_t>(sc);
                bs.obj_size = obj_size;
                bs.count = s.count;
                bs.ratio_sum = s.sum;
                bs.compress_count = cc;
                bs.decompress_count = dc;
                summaries[num_summaries++] = bs;
            }
        }

        // Sort by sample count descending (simple insertion sort)
        for (size_t i = 1; i < num_summaries; ++i) {
            auto key = summaries[i];
            size_t j = i;
            while (j > 0 && summaries[j-1].count < key.count) {
                summaries[j] = summaries[j-1];
                --j;
            }
            summaries[j] = key;
        }

        // Header
        const char* hdr = "[smash bucket] arena  sc  obj_size  samples  ratio%  compress  decompress  thrash%\n";
        (void)!write(2, hdr, strlen(hdr));

        // Dump each bucket
        for (size_t i = 0; i < num_summaries; ++i) {
            const auto& b = summaries[i];
            double mean_ratio = b.count > 0 ? (b.ratio_sum * 100.0 / 255.0 / b.count) : 0;
            double thrash_pct = b.compress_count > 0
                ? (b.decompress_count * 100.0 / b.compress_count) : 0;
            char line[160];
            int n = smash::safe_snprintf(line, sizeof(line),
                "[smash bucket] %5d %3d %9zu %8u %6.1f%% %9u %11u %7.1f%%\n",
                b.arena, b.sc, b.obj_size, b.count, mean_ratio,
                b.compress_count, b.decompress_count, thrash_pct);
            if (n > 0) (void)!write(2, line, (size_t)n);
        }
    }

    // Walk every page; for COMPRESSED / COMPRESSED_SHADOW pages,
    // decompress the blob back into the page and restore PROT_RW; for
    // ACTIVE_MONITORING pages, just restore PROT_RW. After this every
    // smash-managed page is plain anonymous RW memory, so any
    // post-shutdown access (CPython's __cxa_finalize, TLS destructors,
    // mimalloc cleanup, etc.) sees ordinary memory and doesn't fault.
    //
    // This must run BEFORE we join compressor threads (we still need
    // the per-page lock infrastructure) and BEFORE we uninstall the
    // fault handler (decompression here doesn't fault, but we want to
    // be defensive — a fault during drain wouldn't be a no-op).
    void drainAllForShutdown() {
        if (!vm_ || !states_ || !locks_ || !engine_ || !store_) return;
        size_t committed = vm_->committedPages();
        for (size_t i = 0; i < committed; ++i) {
            // tryLock to skip pages held by an in-flight tick we just
            // told to stop; the coordinator will exit and we'll re-walk
            // those after the join.
            if (!locks_->tryLock(i)) continue;
            PageState st = states_->get(i);
            void* page_addr = vm_->pageAddress(i);
            if (st == PageState::COMPRESSED ||
                st == PageState::COMPRESSED_SHADOW) {
                CompressAlgo algo = compressed_[i].algorithm();
                uint8_t sc = lookupSizeClass(i);
                int slot = -1;
                while ((slot = acquireFaultSlot()) < 0) {
#if defined(__x86_64__)
                    __builtin_ia32_pause();
#elif defined(__aarch64__)
                    asm volatile("yield");
#endif
                }
                engine_->decompressWithDCtx(
                    fault_slots_[slot].dctx,
                    compressed_[i].data, fault_slots_[slot].buf,
                    compressed_[i].compressedSize(), kPageSize, algo, sc);
                restorePageContents(page_addr, fault_slots_[slot].buf);
                releaseFaultSlot(slot);
                releaseBlob(i);
                compressed_[i] = {};
                if (page_tier_) page_tier_[i] = kTierNone;
                states_->set(i, PageState::ACTIVE);
            } else if (st == PageState::ACTIVE_MONITORING) {
                if (!vm::protectPages(page_addr, kPageSize, true, true)) {
                    vm::remapPages(page_addr, kPageSize, true, true);
                }
                states_->set(i, PageState::ACTIVE);
            }
            locks_->unlock(i);
        }
    }

    void stop() {
        // Shutdown tracing for debugging (SMASH_SHUTDOWN_TRACE=1)
        static const bool trace = []{
            const char* v = std::getenv("SMASH_SHUTDOWN_TRACE");
            return v && v[0] == '1';
        }();
        auto trace_msg = [](const char* msg) {
            if (!trace) return;
            char buf[120];
            char ts[32] = {};
            vm::formatTimestamp(ts, sizeof(ts));
            int n = smash::safe_snprintf(buf, sizeof(buf), "[smash shutdown] [%s] %s pid=%d\n", ts, msg, (int)getpid());
            if (n > 0) (void)!write(2, buf, (size_t)n);
        };

        trace_msg("stop() called");
        if (!running_.load(std::memory_order_relaxed)) {
            trace_msg("not running, returning early");
            return;
        }
        trace_msg("setting running=false");
        running_.store(false, std::memory_order_release);
        work_waker_.notifyAll();
        trace_msg("joining coord_thread...");
        pthread_join(coord_thread_, nullptr);
        trace_msg("coord_thread joined");
        trace_msg("joining helper threads...");
        for (int i = 0; i < helpers_created_; ++i)
            pthread_join(helper_threads_[i], nullptr);
        trace_msg("all helpers joined");
        // After threads are joined, drain compressed pages back to RW so
        // post-shutdown faults (CPython __cxa_finalize, TLS destructors,
        // mimalloc cleanup) hit ordinary memory instead of our about-to-
        // be-uninstalled signal handler.
        trace_msg("draining compressed pages...");
        drainAllForShutdown();
        trace_msg("drain complete");
        // P3: write the profile file so future runs benefit from this
        // run's per-bucket ratio + cost observations.
        trace_msg("saving profile...");
        saveProfileFile();
        trace_msg("profile saved");
        // Uninstall SIGSEGV/SIGBUS handlers; any further faults take the
        // original disposition (SIG_DFL → terminate, the right behaviour
        // for a real heap-use-after-free in the host application).
        trace_msg("stopping fault handler...");
        if (fault_handler_) fault_handler_->stop();
        // Release the spill mapping + fd. drainAllForShutdown() already
        // decompressed every spilled page back to RW and released its blob, so
        // nothing references the mapping now. The file was unlinked at creation,
        // so closing the fd frees its disk blocks. (BootstrapAlloc metadata is
        // never freed — bounded and process is exiting.)
        if (spill_map_) { munmap(spill_map_, spill_map_size_); spill_map_ = nullptr; }
        if (spill_fd_ >= 0) { close(spill_fd_); spill_fd_ = -1; }
        spill_.reset();
        trace_msg("stop() complete");
    }

    // Manually trigger one compression tick (for testing/manual control)
    void compressTick() { tick(); }

    // Dict experiment counters (try-both)
    uint64_t dictWinCount() const { return dict_win_count_.load(std::memory_order_relaxed); }
    uint64_t dictLossCount() const { return dict_loss_count_.load(std::memory_order_relaxed); }
    uint64_t dictTieCount() const { return dict_tie_count_.load(std::memory_order_relaxed); }
    int64_t dictTotalDelta() const { return dict_total_delta_.load(std::memory_order_relaxed); }

    int dictsTrainedCount() const {
        int count = 0;
        for (int sc = 0; sc < kNumClasses; ++sc)
            if (dict_train_[sc].trained) ++count;
        return count;
    }

    // Called by the fault handler when a protected page is accessed.
    // Returns true if the fault was handled (page restored).
    bool handleFault(uintptr_t addr) {
        if (!vm_->contains(addr)) return false;

        size_t page_idx = vm_->pageIndex(addr);

        // Reentrancy guard. compressPage / recompressPage hold the
        // per-page lock while doing memcpy after PROT_READ. If the
        // memcpy SIGSEGVs on the same thread (TLB inconsistency or an
        // underlying smash state-machine bug), the signal handler
        // runs on that same thread; calling locks_->lock(page_idx)
        // would self-deadlock on the non-recursive spinlock.  Detect
        // and bail so the SIGSEGV propagates with a real backtrace.
        if (locks_->heldByThisThread(page_idx)) return false;

        locks_->lock(page_idx);

        PageState st = states_->get(page_idx);

        switch (st) {
        case PageState::COMPRESSED: {
            // Clear deferred-madvise pending BEFORE the state transitions
            // out of COMPRESSED. The per-page lock is held; the sweeper
            // tryLock+recheck will observe the cleared bit (or skip the
            // page entirely because state is no longer COMPRESSED).
            if (deferred_pending_) {
                deferred_pending_[page_idx].store(false, std::memory_order_release);
            }
            void* page_addr = vm_->pageAddress(page_idx);
            CompressAlgo algo = compressed_[page_idx].algorithm();
            uint8_t sc = lookupSizeClass(page_idx);

            // Spin-wait for a fault slot - cannot skip since thread is faulted.
            // With 128 slots this should rarely spin, but under extreme load
            // it ensures we always use thread-safe per-slot DCtx.
            int slot;
            while ((slot = acquireFaultSlot()) < 0) {
#if defined(__x86_64__)
                __builtin_ia32_pause();
#elif defined(__aarch64__)
                asm volatile("yield");
#endif
            }

            // Decompress using per-slot DCtx (no data race). Time it for
            // the budget machinery — refault cost is part of the bucket's
            // amortized time per compressed page.
            auto dec_t0 = std::chrono::steady_clock::now();
            engine_->decompressWithDCtx(
                fault_slots_[slot].dctx,
                compressed_[page_idx].data, fault_slots_[slot].buf,
                compressed_[page_idx].compressedSize(), kPageSize,
                algo, sc);
            auto dec_t1 = std::chrono::steady_clock::now();
            uint32_t dec_us = static_cast<uint32_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    dec_t1 - dec_t0).count());

            // Restore the page. CRITICAL: the decompressed bytes must be in
            // the page's backing BEFORE the page becomes readable — otherwise
            // a concurrent application thread (which does not take the per-page
            // lock and does not go through this handler for an ordinary load)
            // could observe a readable-but-empty page and read stale/zero
            // data. restorePageContents() populates the backing while the page
            // is still PROT_NONE (Linux /proc/self/mem fast path) and only then
            // flips it to PROT_RW. See restorePageContents() for the full
            // rationale and the fallback path.
            restorePageContents(page_addr, fault_slots_[slot].buf);
            releaseFaultSlot(slot);

            // Release compressed blob (sharded store)
            releaseBlob(page_idx);
            compressed_[page_idx] = {};
            if (page_tier_) page_tier_[page_idx] = kTierNone;

            states_->set(page_idx, PageState::ACTIVE);
            cold_count_[page_idx] = 0;
            compress_fail_count_[page_idx] = 0;  // access → verdict stale
            locks_->unlock(page_idx);

            // Recompression-thrash signal. The act of faulting back from
            // COMPRESSED is direct evidence the page was hotter than we
            // thought; bump the per-page count and feed the per-bucket EMA
            // so phase2 backs off on this page (and on fresh pages from the
            // same allocation site) the next round.
            uint8_t prev_rc = recompress_count_[page_idx];
            if (prev_rc < 255) recompress_count_[page_idx] = prev_rc + 1;

            // Soft-dirty ROI: classify this decompress as a WRITE re-dirty (the
            // blob was genuinely invalidated → real churn) vs a READ fault (the
            // blob would have survived → not a re-dirty), using the hardware
            // fault write-bit captured by the signal handler. This is the exact
            // per-fault signal — far better than inferring from soft-dirty a
            // tick later (the workload re-touches the page within the tick, so
            // a next-tick check can't separate the two). When the bit is unknown
            // (-1, e.g. Mach path), fall back to "treat as write" (conservative:
            // a re-dirty defers future compression, the safe direction).
            if (write_clean_streak_) {
                write_clean_streak_[page_idx] = 0;  // freshly live again
                bool was_write = (vm::faultWasWrite() != 0);  // -1 unknown → treat as write
                recordRedirtyForPage(page_idx, was_write);
            }

            // Adaptive cap: decompression = re-warm evidence.  Notify the
            // heap so it can bias cap sizing for this (arena, sc) toward
            // "hot" (no cap / larger cap).
            //
            // Same lookup gives us (arena, size_class) for the bucket EMA
            // update — do both in one Span fetch.
            bool is_external = page_idx >= vm_->contigPages();
            if (is_external) {
                // External page thrashed — mark for profile persistence so
                // future runs skip external pages entirely.
                external_pages_hot_.store(true, std::memory_order_relaxed);
            }
            if (page_map_) {
                Span* sp = page_map_->get(reinterpret_cast<uintptr_t>(page_addr));
                if (sp) {
                    // Slab-only feedback (decompressed_fn_ keys on real
                    // slab size_class for the heap's adaptive-cap tables).
                    if (decompressed_fn_ && !sp->is_large &&
                        sp->size_class < kNumClasses) {
                        decompressed_fn_(page_idx, sp->arena_id, sp->size_class,
                                         decompressed_ctx_);
                    }
                    // Unified bucket for the rc-ema and time-budget tables —
                    // large allocs land in the kNumClasses+log2(pages) slot.
                    uint8_t sc_b = sp->is_large
                        ? largeSizeClass(sp->page_count)
                        : sp->size_class;
                    if (sc_b < kTotalBucketsPerArena) {
                        size_t bidx = statsIndex(sp->arena_id, sc_b);
                        if (bidx < kBucketTableLen) {
                            // EMA α=1/4, ×256 fixed-point. Relaxed atomics; tearing
                            // on the EMA is acceptable.
                            uint16_t old = bucket_rc_ema_x256_[bidx].load(std::memory_order_relaxed);
                            uint16_t sample_x256 = static_cast<uint16_t>(prev_rc + 1) * 256;
                            uint8_t cnt = bucket_rc_count_[bidx].load(std::memory_order_relaxed);
                            uint16_t neu = (cnt == 0)
                                ? sample_x256
                                : static_cast<uint16_t>(static_cast<int32_t>(old)
                                    + (static_cast<int32_t>(sample_x256) - static_cast<int32_t>(old)) / 4);
                            bucket_rc_ema_x256_[bidx].store(neu, std::memory_order_relaxed);
                            if (cnt < 64)
                                bucket_rc_count_[bidx].store(cnt + 1, std::memory_order_relaxed);

                            // Time-budget: charge decompress wall-time against
                            // the bucket. Worker 0 is the canonical sink since
                            // we can't know which worker compressed this page.
                            // recomputeMarginalEfficiency aggregates across
                            // workers, so it doesn't matter where we land.
                            workers_[0].sc_stats[bidx].time_cost_total_us.fetch_add(
                                dec_us, std::memory_order_relaxed);
                            // v6 profile: track decompression for thrash rate
                            workers_[0].sc_stats[bidx].decompress_count.fetch_add(
                                1, std::memory_order_relaxed);
                        }
                    }
                }
            }

            // Prefetch adjacent compressed pages
            prefetchAdjacent(page_idx);

            return true;
        }

        case PageState::COMPRESSING: {
            // TLA+ model proves this is unreachable: fault handler acquires
            // the page lock, but compressor holds it during COMPRESSING.
            // Verified unreachable via TLA+ model checking and
            // 576 benchmark runs with __builtin_trap() (zero crashes).
            // Retained as defensive fallback.
            void* page_addr = vm_->pageAddress(page_idx);
            if (!vm::commitPages(page_addr, kPageSize)) {
                vm::remapPages(page_addr, kPageSize, true, true);
            }
            states_->set(page_idx, PageState::ACTIVE);
            cold_count_[page_idx] = 0;
            compress_fail_count_[page_idx] = 0;
            locks_->unlock(page_idx);
            return true;
        }

        case PageState::ACTIVE_MONITORING: {
            void* page_addr = vm_->pageAddress(page_idx);
            if (!vm::protectPages(page_addr, kPageSize, true, true)) {
                vm::remapPages(page_addr, kPageSize, true, true);
            }
            accessed_[page_idx].store(true, std::memory_order_relaxed);
            states_->set(page_idx, PageState::ACTIVE);
            locks_->unlock(page_idx);
            return true;
        }

        case PageState::ACTIVE: {
            // Race: Phase 3's batched mprotect(PROT_READ) can overwrite a
            // per-page PROT_RW restoration done by a concurrent fault handler.
            void* page_addr = vm_->pageAddress(page_idx);
            if (!vm::protectPages(page_addr, kPageSize, true, true)) {
                vm::remapPages(page_addr, kPageSize, true, true);
            }
            locks_->unlock(page_idx);
            return true;
        }

        case PageState::COMPRESSED_SHADOW: {
            // Page was still accessible (PROT_RW or PROT_READ from Phase 3).
            // Fault means Phase 3 set PROT_READ and a write occurred.
            // Discard the shadow blob, restore to ACTIVE.
            void* page_addr = vm_->pageAddress(page_idx);
            vm::protectPages(page_addr, kPageSize, true, true);
            releaseBlob(page_idx);
            compressed_[page_idx] = {};
            if (shadow_tick_) shadow_tick_[page_idx] = 0;
            if (page_tier_) page_tier_[page_idx] = kTierNone;
            cold_count_[page_idx] = 0;
            compress_fail_count_[page_idx] = 0;  // write → contents changed
            if (recompress_count_[page_idx] < 255)
                recompress_count_[page_idx]++;
            states_->set(page_idx, PageState::ACTIVE);
            locks_->unlock(page_idx);
            return true;
        }

        default:
            locks_->unlock(page_idx);
            return false;
        }
    }

    // Release any compressed data for a range of pages (called during span release)
    void releaseCompressedPages(size_t start_page, size_t num_pages) {
        for (size_t i = start_page; i < start_page + num_pages; ++i) {
            locks_->lock(i);
            // Clear deferred-madvise pending BEFORE the state transitions
            // away from COMPRESSED. Sweeper double-checks state under the
            // per-page lock; this just makes the bit consistent with the
            // observed state.
            if (deferred_pending_) {
                deferred_pending_[i].store(false, std::memory_order_release);
            }
            PageState st = states_->get(i);
            if ((st == PageState::COMPRESSED || st == PageState::COMPRESSED_SHADOW)
                && compressed_[i].data) {
                releaseBlob(i);
                compressed_[i] = {};
            }
            if (shadow_tick_) shadow_tick_[i] = 0;
            states_->set(i, PageState::EMPTY);
            cold_count_[i] = 0;
            recompress_count_[i] = 0;
            compress_fail_count_[i] = 0;
            accessed_[i].store(false, std::memory_order_relaxed);
            locks_->unlock(i);
        }
    }
};

} // namespace smash
