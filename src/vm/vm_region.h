// smash/src/vm/vm_region.h - Single large virtual memory reservation
//
// All slab data pages are allocated from this contiguous region.
// Benefits: O(1) bounds check, page state tracking, reassignable pages.
//
// Supports two modes (selected at runtime via SMASH_MODE env var):
// - Full mode: contiguous VM reservation, pages allocated via bump pointer
// - Compress-only mode: tracks arbitrary page addresses via hash map,
//   allowing compression of pages owned by any allocator
#pragma once

#include <new>
#include <sys/mman.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include "smash/config.h"
#include "platform_mem.h"
#include "page_run_map.h"
#include "page_state.h"
#include "../core/bootstrap_alloc.h"
#include "../util/spinlock.h"
#include "../util/exit_guard.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <thread>

namespace smash {

// Forward declaration so VmRegion can store an atomic<Span*>* without pulling
// in the full Span definition (avoids circular include with core/span.h).
struct Span;

class VmRegion {
    // Mode determined at init time
    bool tracking_mode_ = false;

    // ── Full mode: contiguous allocation ────────────────────────────────────
    char* base_ = nullptr;
    size_t total_pages_ = 0;
    size_t contig_pages_ = 0;  // = region_size / kPageSize in full mode; 0 in tracking mode
    std::atomic<size_t> next_page_{0};  // bump pointer (normal) or next index (tracking)

    // ── Chunked commit watermark ────────────────────────────────────────────
    // The arena is reserved PROT_NONE; pages must be mprotect'd to PROT_RW
    // before use. Committing per span bump means one mprotect per span, and
    // every mprotect takes the kernel's mmap_lock for write — under a workload
    // that bump-allocates a large live set (e.g. Hoard linux-scalability holds
    // ~1.3 GB live) those serialize on osq_lock and dominate CPU. Instead we
    // commit in large chunks ahead of the bump pointer: committed_pages_ is the
    // high-water page index already PROT_RW, advanced under commit_lock_ in
    // kCommitChunkPages steps. A bump only mprotects when it crosses the
    // watermark, turning millions of mprotects into a few hundred. Behavior is
    // preserved: mprotect on an untouched PROT_NONE anonymous range only flips
    // VMA protection; no physical page is backed until first write, so RSS is
    // unchanged (the kernel still faults pages in lazily on touch).
    std::atomic<size_t> committed_pages_{0};
    Spinlock commit_lock_;

    // Pages committed per watermark advance (see committed_pages_).
    //
    // DEFAULT 0 = OFF (commit exactly the requested pages, per-call). Chunked
    // commit was measured to be net-negative: on a single-threaded bump-growth
    // workload (Hoard linux-scalability) it cuts the per-span mprotect/mmap_lock
    // cost ~25%, but it REGRESSES realloc churn (phong +90%) and, worst, holding
    // the global commit_lock_ across a multi-MiB mprotect serializes high
    // thread counts — Larson at 64 threads collapsed ~10x (47M→5M ops/s) with
    // chunking on. The win is narrow and the downside is a scalability cliff, so
    // it ships off. Opt in per-workload via SMASH_COMMIT_CHUNK_KIB=N.
    static constexpr size_t kCommitChunkPages = 0;

    // Free arena pages: a coalescing map of runs (see page_run_map.h). It
    // replaced 64 per-thread lists of FreeRun nodes that never merged — a run
    // could only serve a request no larger than itself, so memory freed at one
    // size could not be reused at a larger one, and a program moving between
    // sizes spilled out of the compressible arena for good. Owner 0 is the
    // shared arena, node + 1 a NUMA node arena; runs of different owners never
    // merge.
    PageRunMap free_map_;
    void (*exhausted_hook_)(void*) = nullptr;
    void* exhausted_ctx_ = nullptr;
    static constexpr uint32_t kSharedOwner = 0;
    static uint32_t ownerOfNode(int node) {
        return node >= 0 ? static_cast<uint32_t>(node) + 1 : kSharedOwner;
    }

    // ── Background decommit thread ──────────────────────────────────────────
    // Two-phase release: releasePages() queues pages for decommit, background
    // thread decommits them and THEN adds to free list. This ensures pages
    // can't be reallocated before decommit completes (fixing the race condition).
    struct DecommitEntry {
        void* addr;
        size_t num_pages;
        size_t page_index;
        int node;  // owning NUMA node arena, or -1 for the shared free lists
        DecommitEntry* next;
    };

    std::atomic<DecommitEntry*> decommit_head_{nullptr};
    DecommitEntry* decommit_pool_ = nullptr;
    Spinlock decommit_pool_lock_;
    pthread_t decommit_thread_{};
    std::atomic<bool> decommit_running_{false};

    DecommitEntry* newDecommitEntry() {
        LockGuard guard(decommit_pool_lock_);
        if (decommit_pool_) {
            DecommitEntry* e = decommit_pool_;
            decommit_pool_ = e->next;
            return e;
        }
        return static_cast<DecommitEntry*>(
            BootstrapAlloc::instance().allocate(sizeof(DecommitEntry), alignof(DecommitEntry)));
    }

    void recycleDecommitEntry(DecommitEntry* e) {
        LockGuard guard(decommit_pool_lock_);
        e->next = decommit_pool_;
        decommit_pool_ = e;
    }

    void queueForDecommit(void* addr, size_t num_pages, size_t page_index, int node) {
        DecommitEntry* e = newDecommitEntry();
        if (!e) {
            // The metadata arena is exhausted (it reports that itself). Do the
            // release synchronously on a stack entry instead of dereferencing
            // null: this path is reached exactly when an allocation has just
            // failed and is giving its pages back.
            DecommitEntry local{};
            local.addr = addr;
            local.num_pages = num_pages;
            local.page_index = page_index;
            local.node = node;
            processDecommitEntry(&local);
            return;
        }
        e->addr = addr;
        e->num_pages = num_pages;
        e->page_index = page_index;
        e->node = node;
        // Lock-free push to MPSC queue
        DecommitEntry* head = decommit_head_.load(std::memory_order_relaxed);
        do {
            e->next = head;
        } while (!decommit_head_.compare_exchange_weak(head, e,
            std::memory_order_release, std::memory_order_relaxed));
    }

    void processDecommitEntry(DecommitEntry* e) {
        // Restore PROT_RW on the range. A freed range may include pages
        // smash compressed before the free (state COMPRESSED →
        // mprotect PROT_NONE). Without this, allocatePages's freelist-pop
        // path hands the still-PROT_NONE range back to the application,
        // and the next access faults. The fault handler sees state=EMPTY
        // (cleared by releaseHook in the free path) and bails — kernel
        // delivers SIGSEGV. Manifests on workloads with many compressed
        // pages followed by reuse of those addresses (e.g. test7_full
        // walrus crash with "DenseMap node count imbalance" or null-deref
        // in nlohmann::json::destroy).
        vm::commitPages(e->addr, e->num_pages * kPageSize);

        // Decommit the pages (release physical backing).
        vm::decommitPages(e->addr, e->num_pages * kPageSize);

        // NOW add to the free map (safe - pages are decommitted and PROT_RW),
        // merging with free neighbours of the same owner. A node arena's pages
        // go back to that arena only (see NodeArena).
        free_map_.insert(e->page_index, e->num_pages, ownerOfNode(e->node));
    }

    static void* decommitThreadEntry(void* arg) {
        // Block asynchronous signals so a process-directed signal (e.g. an
        // app's shutdown SIGUSR2/SIGTERM) is never absorbed by this smash
        // background thread instead of reaching a host application thread.
        // SIGSEGV/SIGBUS/SIGABRT/SIGFPE/SIGILL left unblocked (synchronous /
        // fatal). Mirrors CompressorThread::blockAsyncSignalsOnSelf().
        {
            sigset_t set;
            sigfillset(&set);
            sigdelset(&set, SIGSEGV);
            sigdelset(&set, SIGBUS);
            sigdelset(&set, SIGABRT);
            sigdelset(&set, SIGFPE);
            sigdelset(&set, SIGILL);
            pthread_sigmask(SIG_BLOCK, &set, nullptr);
        }
        // Not an application thread: the process-exit guard must not count
        // this one as a reason to keep the process alive (util/exit_guard.h).
        exit_guard::ThreadRegistration exit_reg;
        auto* self = static_cast<VmRegion*>(arg);
        while (self->decommit_running_.load(std::memory_order_relaxed)) {
            // Drain the queue
            DecommitEntry* batch = self->decommit_head_.exchange(nullptr, std::memory_order_acquire);
            if (batch) {
                while (batch) {
                    DecommitEntry* next = batch->next;
                    self->processDecommitEntry(batch);
                    self->recycleDecommitEntry(batch);
                    batch = next;
                }
            } else {
                usleep(100);  // 100µs - short sleep for responsiveness
            }
        }
        return nullptr;
    }

    void startDecommitThread() {
        if (tracking_mode_) return;
        bool expected = false;
        if (!decommit_running_.compare_exchange_strong(expected, true,
                std::memory_order_acq_rel, std::memory_order_relaxed)) {
            return;
        }
        if (pthread_create(&decommit_thread_, nullptr, decommitThreadEntry, this) != 0) {
            // EAGAIN under thread exhaustion. Without the rollback,
            // decommit_running_ claims a drainer exists while nothing ever
            // pops the MPSC queue: freed runs never reach the free list
            // (allocation starvation) and are never madvise'd (RSS never
            // drops). Roll back so the next release retries creation, and
            // drain whatever is queued right now inline (the exchange-based
            // drain is safe from any thread).
            decommit_running_.store(false, std::memory_order_release);
            DecommitEntry* batch =
                decommit_head_.exchange(nullptr, std::memory_order_acquire);
            while (batch) {
                DecommitEntry* next = batch->next;
                processDecommitEntry(batch);
                recycleDecommitEntry(batch);
                batch = next;
            }
        }
    }

    void ensureDecommitThreadStarted() {
        if (tracking_mode_) return;
        if (decommit_running_.load(std::memory_order_relaxed)) return;
        startDecommitThread();
    }

    void stopDecommitThread() {
        if (!decommit_running_.load(std::memory_order_relaxed)) return;
        decommit_running_.store(false, std::memory_order_release);
        // Never join self — see the same guard in CompressorThread::stop().
        if (!pthread_equal(decommit_thread_, pthread_self()))
            pthread_join(decommit_thread_, nullptr);
        // Drain any remaining entries
        DecommitEntry* batch = decommit_head_.exchange(nullptr, std::memory_order_acquire);
        while (batch) {
            DecommitEntry* next = batch->next;
            processDecommitEntry(batch);
            recycleDecommitEntry(batch);
            batch = next;
        }
    }

    // ── Tracking mode: hash map for arbitrary page addresses ────────────────
    //
    // Used in two situations:
    //   (a) compress-only mode — every malloc and mmap is tracked here, the
    //       contiguous arena is unused.
    //   (b) full mode — the contiguous arena holds all malloc-routed slab
    //       pages, AND this hash holds *external* pages (application-direct
    //       mmap and Mach VM allocations) so the compressor can compress
    //       them too. External pages get indices in
    //       [kVmMaxPages, kVmMaxPages + kTrackMaxPages); contiguous pages
    //       keep their existing low indices.
    static constexpr size_t kTrackMaxPages = 128 * 1024;  // 128K pages (~2GB on 16K pages)
    static constexpr size_t kTrackHashCap  = 256 * 1024;  // 2x headroom
    static constexpr size_t kTrackHashMask = kTrackHashCap - 1;

    struct TrackEntry {
        std::atomic<uintptr_t> key{0};  // page addr >> kPageShift; 0 = empty/freed
        std::atomic<size_t> idx{0};
    };

    TrackEntry* track_hash_ = nullptr;
    uintptr_t* track_reverse_ = nullptr;  // idx → page_addr (page-aligned)

    // Full-mode external-page bookkeeping. The contiguous arena uses
    // next_page_ for its bump pointer (indices 0..kVmMaxPages-1); this
    // counter assigns external indices starting at kVmMaxPages.
    // external_slot_next_: monotonically incremented to reserve a slot
    // external_count_: only incremented after the slot is fully populated
    //                  (address written). committedPages() reads this, so
    //                  the compressor won't iterate to slots-in-progress.
    std::atomic<size_t> external_slot_next_{0};
    std::atomic<size_t> external_count_{0};

    // Flat direct page-index → Span* table. Sized to total_pages_ entries
    // (8 B each) and mmap'd with MAP_NORESERVE so we only pay RSS for the
    // committed slice (~6K pages → ~48 KB resident for a 100 MB heap).
    // Provides a single-load lookup that replaces the two-level radix
    // walk on the free() hot path. Sits outside BootstrapAlloc-managed
    // memory because the radix tree allocates lazily L2 pages from
    // BootstrapAlloc and we want the flat table to escape that.
    std::atomic<Span*>* page_to_span_ = nullptr;
    size_t page_to_span_bytes_ = 0;  // total mapped bytes (for cleanup / sizing)

    size_t lookupIdx(uintptr_t addr) const {
        if (!track_hash_) return 0;
        uintptr_t key = addr >> kPageShift;
        if (key == 0) return 0;
        size_t slot = static_cast<size_t>(key * 0x9E3779B97F4A7C15ULL) >> (64 - 18);
        for (size_t i = 0; i < kTrackHashCap; ++i) {
            size_t s = (slot + i) & kTrackHashMask;
            uintptr_t existing = track_hash_[s].key.load(std::memory_order_relaxed);
            if (existing == key)
                return track_hash_[s].idx.load(std::memory_order_acquire);
            if (existing == 0)
                return 0;
        }
        return 0;
    }

public:
    bool init(size_t region_size) {
        tracking_mode_ = isCompressOnlyMode();

        // Both modes use the tracking hash:
        //   - compress-only mode: it IS the page directory.
        //   - full mode: it tracks application-direct mmap / Mach VM
        //     allocations alongside the contiguous arena.
        // allocateZeroed uses direct mmap for large allocs → demand-paged,
        // zero RSS until first access. TrackEntry's default state is all-zero
        // (key=0 means empty), so no explicit initialization needed.
        track_hash_ = static_cast<TrackEntry*>(
            BootstrapAlloc::instance().allocateZeroed(
                kTrackHashCap * sizeof(TrackEntry), 64));
        track_reverse_ = static_cast<uintptr_t*>(
            BootstrapAlloc::instance().allocateZeroed(
                kTrackMaxPages * sizeof(uintptr_t), 8));
        if (!track_hash_ || !track_reverse_) return false;

        if (!tracking_mode_) {
            // Full mode: reserve contiguous VM region. total_pages_ covers
            // both the contiguous range AND a tail reserved for external
            // pages, so PageStateTable / PageLockTable have room for both.
            // Reserve the arena; if the address space will not take the
            // requested size (39-bit arm64 VA, RLIMIT_AS), halve it down to
            // the 256 MiB floor rather than running with no compression at
            // all, which is what a failed reservation used to mean.
            const size_t requested = region_size;
            constexpr size_t kFloor = size_t{256} << 20;
            for (;;) {
                base_ = static_cast<char*>(vm::reservePages(region_size));
                if (base_ || region_size / 2 < kFloor) break;
                region_size = region_size / 2 / kPageSize * kPageSize;
            }
            if (!base_) return false;
            if (region_size != requested) {
                char buf[192];
                int n = smash::safe_snprintf(buf, sizeof(buf),
                    "[smash] could not reserve a %zu MiB arena; using %zu MiB "
                    "(SMASH_VM_GIB)\n", requested >> 20, region_size >> 20);
                if (n > 0) (void)!::write(2, buf, static_cast<size_t>(n));
            }
            contig_pages_ = region_size / kPageSize;
            total_pages_ = contig_pages_ + kTrackMaxPages;
            vm::setVmBounds(base_, region_size);
            if (!free_map_.init(contig_pages_)) return false;
        } else {
            // Compress-only mode: contiguous arena is unused; tracked pages
            // get indices 1..kTrackMaxPages-1 (index 0 reserved as sentinel).
            (void)region_size;
            total_pages_ = kTrackMaxPages;
            next_page_.store(1, std::memory_order_relaxed);
        }

        // Lazy flat page-index → Span* table. MAP_NORESERVE means the kernel
        // backs only the pages we touch — virtual cost = total_pages_*8B
        // (~32 MiB at kVmMaxPages=1M), resident cost ≈ committed-pages*8B
        // rounded to OS-page granularity.
        page_to_span_bytes_ = total_pages_ * sizeof(std::atomic<Span*>);
        void* m = ::mmap(nullptr, page_to_span_bytes_,
                         PROT_READ | PROT_WRITE,
                         MAP_ANON | MAP_PRIVATE | MAP_NORESERVE,
                         -1, 0);
        if (m == MAP_FAILED) {
            page_to_span_ = nullptr;
            page_to_span_bytes_ = 0;
            // Non-fatal: callers fall back to the radix tree via PageMap.
        } else {
            page_to_span_ = static_cast<std::atomic<Span*>*>(m);
        }

        // Decommit thread is started lazily on first releasePages() call
        // to avoid reentrancy issues during early library initialization.

        return true;
    }

    ~VmRegion() {
        // Stop binding remapped ranges again for a region that is going away.
        VmRegion* self = this;
        if (numa_region_.compare_exchange_strong(self, nullptr))
            vm::g_after_remap_hook = nullptr;
        stopDecommitThread();
    }

    // Stop the lazily started decommit thread from outside the destructor.
    // The process-exit guard (util/exit_guard.h) has to stop every thread
    // smash created, and this one is not the compressor's to stop.
    void stopBackgroundThread() { stopDecommitThread(); }

    bool isTrackingMode() const { return tracking_mode_; }

    // Reset decommit thread state after fork(). The thread doesn't exist in the
    // child (Linux fork only clones the calling thread), so we must reset state
    // so ensureDecommitThreadStarted() will restart it on first releasePages().
    void resetDecommitThreadForFork() {
        // Thread doesn't exist in child - don't try to join it
        decommit_running_.store(false, std::memory_order_relaxed);
        // Drain pending entries synchronously
        DecommitEntry* batch = decommit_head_.exchange(nullptr, std::memory_order_acquire);
        while (batch) {
            DecommitEntry* next = batch->next;
            processDecommitEntry(batch);
            recycleDecommitEntry(batch);
            batch = next;
        }
    }

    // ── Allocation (full mode only) ─────────────────────────────────────────
    void* allocatePages(size_t num_pages) {
        if (tracking_mode_) return nullptr;

        // Reuse freed pages first: the smallest free run that fits, carved
        // from its low end.
        if (const size_t idx = free_map_.take(num_pages, kSharedOwner);
            idx != PageRunMap::kNone) {
            return base_ + idx * kPageSize;
        }

        // Bump allocate from fresh pages (lock-free)
        size_t start = next_page_.fetch_add(num_pages, std::memory_order_release);
        if (start + num_pages > contig_pages_) {
            next_page_.fetch_sub(num_pages, std::memory_order_relaxed);
            // Arena exhausted: let the heap return empty slab spans, which
            // then coalesce in the free map, and try once more.
            if (exhausted_hook_) {
                exhausted_hook_(exhausted_ctx_);
                if (const size_t idx = free_map_.take(num_pages, kSharedOwner);
                    idx != PageRunMap::kNone) {
                    return base_ + idx * kPageSize;
                }
            }
            return nullptr;
        }
        void* addr = base_ + start * kPageSize;
        ensureCommitted(start + num_pages);
        return addr;
    }

    // Ensure pages [0, end_page) are committed (PROT_RW). Fast path: a single
    // relaxed load when the watermark already covers end_page (the common case
    // — one chunk commit serves thousands of bumps). Slow path: take
    // commit_lock_, re-check, and mprotect forward to the next kCommitChunkPages
    // boundary (clamped to contig_pages_). The double-check under the lock means
    // concurrent bumps that all cross the watermark issue exactly one mprotect
    // for the chunk, not one each.
    // Runtime-overridable commit chunk (pages). SMASH_COMMIT_CHUNK_KIB lets the
    // chunk size be swept without rebuilding; 0 disables chunking (commit
    // exactly the requested pages, like the pre-watermark behavior).
    static size_t commitChunkPages() {
        static const size_t pages = [] {
            const char* v = std::getenv("SMASH_COMMIT_CHUNK_KIB");
            if (!v) return kCommitChunkPages;
            long kib = std::atol(v);
            if (kib < 0) return kCommitChunkPages;
            return static_cast<size_t>((static_cast<size_t>(kib) * 1024) / kPageSize);
        }();
        return pages;
    }

    void ensureCommitted(size_t end_page) {
        if (end_page <= committed_pages_.load(std::memory_order_acquire)) [[likely]]
            return;
        LockGuard guard(commit_lock_);
        size_t already = committed_pages_.load(std::memory_order_relaxed);
        if (end_page <= already) return;  // another thread committed it
        const size_t chunk = commitChunkPages();
        // Commit forward to the next chunk boundary so a run of bumps shares
        // this mprotect; round end_page up, clamp to the arena end. chunk==0
        // means "no chunking" — commit exactly what's needed.
        size_t target = (chunk == 0) ? end_page : already + chunk;
        if (target < end_page) {
            target = ((end_page + chunk - 1) / chunk) * chunk;
        }
        if (target > contig_pages_) target = contig_pages_;
        void* addr = base_ + already * kPageSize;
        size_t commit_bytes = (target - already) * kPageSize;
        vm::commitPages(addr, commit_bytes);
        committed_pages_.store(target, std::memory_order_release);
    }

    void releasePages(void* addr, size_t num_pages) {
        if (tracking_mode_) return;

        // Refuse non-arena addresses. The free list stores ARENA page indices;
        // pageIndex() on a tracked-external address returns an external index
        // (>= contig_pages_) which allocatePages would re-mint as
        // base_ + idx*kPageSize — an address past the arena end (#84). No
        // caller should pass one after the LargeAlloc routing fix; if one
        // does, unmap it (the only legitimate producer was a direct-mmap
        // fallback page) and warn once rather than corrupt the free list.
        if (!inContigArena(reinterpret_cast<uintptr_t>(addr))) {
            static std::atomic<bool> warned{false};
            bool expected = false;
            if (warned.compare_exchange_strong(expected, true,
                                               std::memory_order_relaxed)) {
                char buf[128];
                int n = smash::safe_snprintf(buf, sizeof(buf),
                    "[smash WARN] releasePages: non-arena addr=%p npages=%zu"
                    " — unmapping instead of freelisting\n", addr, num_pages);
                if (n > 0) (void)!::write(2, buf, (size_t)n);
            }
            vm::unmapPages(addr, num_pages * kPageSize);
            return;
        }

        // SMASH_LANDMINES=1: synchronously mprotect(PROT_NONE) the freed range
        // and log it. Pages are never returned to the free list, so any
        // dangling pointer in the application surfaces as SIGSEGV at the
        // exact dangling-read site (instead of corrupting later allocations
        // or zero-filling silently). Compares to glibc/jemalloc which keep
        // freed contents intact: this distinguishes "smash decommit causes
        // crash" (landmines pass) from "app UAF" (landmines crash, with
        // si_addr inside a logged range).
        if (landminesEnabled()) {
            (void)::mprotect(addr, num_pages * kPageSize, PROT_NONE);
            char buf[96];
            int n = smash::safe_snprintf(buf, sizeof(buf),
                "[smash landmine] free addr=%p npages=%zu range=[%p,%p)\n",
                addr, num_pages, addr,
                static_cast<char*>(addr) + num_pages * kPageSize);
            if (n > 0) (void)!::write(2, buf, (size_t)n);
            return;  // never reuse — fail loudly on dangling reuse
        }

        // SMASH_ZERO_FREE=1: synchronously zero (memset) the freed range AND
        // allow normal reuse via the freelist. If walrus reads from freed
        // memory between free() and a subsequent allocation, this is exactly
        // what madvise(MADV_DONTNEED) does (zero on next access) but without
        // any kernel involvement: the zero is immediate. If this crashes,
        // walrus has a UAF that reads pre-free data. If it passes, the
        // problem is something else in the madvise/reuse path (e.g. TLB
        // shootdown timing, freelist coherency).
        if (zeroFreeEnabled()) {
            __builtin_memset(addr, 0, num_pages * kPageSize);
            // fall through to normal queue+decommit+reuse path
        }

        // Start decommit thread lazily on first release (avoids reentrancy
        // during early library initialization when pthread_create might malloc).
        ensureDecommitThreadStarted();

        // Two-phase release: queue pages for background decommit. The background
        // thread will decommit them and THEN add to free list. This ensures
        // pages can't be reallocated before decommit completes.
        size_t page_idx = pageIndex(reinterpret_cast<uintptr_t>(addr));
        queueForDecommit(addr, num_pages, page_idx, nodeOfPage(page_idx));
    }

    static bool landminesEnabled() {
        static const bool v = []{
            const char* e = std::getenv("SMASH_LANDMINES");
            return e && e[0] == '1';
        }();
        return v;
    }
    static bool zeroFreeEnabled() {
        static const bool v = []{
            const char* e = std::getenv("SMASH_ZERO_FREE");
            return e && e[0] == '1';
        }();
        return v;
    }

    // releasePages(), but synchronous: decommit now and put the run in the
    // free map before returning, instead of queueing it for the decommit
    // thread. For the arena-exhausted reclaim, whose caller retries the
    // allocation immediately and could not see a queued run. The same order
    // (commit RW, decommit, then free) keeps a run from being reused before it
    // is decommitted. Debug modes and non-arena addresses take the normal path.
    void releasePagesNow(void* addr, size_t num_pages) {
        if (tracking_mode_ || !inContigArena(reinterpret_cast<uintptr_t>(addr)) ||
            landminesEnabled() || zeroFreeEnabled()) {
            releasePages(addr, num_pages);
            return;
        }
        DecommitEntry e{};
        e.addr = addr;
        e.num_pages = num_pages;
        e.page_index = pageIndex(reinterpret_cast<uintptr_t>(addr));
        e.node = nodeOfPage(e.page_index);
        processDecommitEntry(&e);
    }

    // Called by allocatePages() when neither the free map nor the bump pointer
    // can serve a request, before it gives up. The heap uses it to return
    // empty slab spans (Slab::reclaimEmptySpans) so the arena reuses its own
    // memory before anything spills into uncompressed direct mappings. Must
    // not call allocatePages().
    void setExhaustedHook(void (*fn)(void*), void* ctx) {
        exhausted_ctx_ = ctx;
        exhausted_hook_ = fn;
    }

    // ── Page tracking (compress-only mode) ──────────────────────────────────
    // Track a page address, assigning it a compact index.
    // Returns the index (>0) on success, 0 on failure (table full or wrong mode).
    size_t trackPage(uintptr_t page_addr) {
        if (!tracking_mode_ || !track_hash_) return 0;

        uintptr_t key = page_addr >> kPageShift;
        if (key == 0) return 0;
        size_t slot = static_cast<size_t>(key * 0x9E3779B97F4A7C15ULL) >> (64 - 18);
        for (size_t i = 0; i < kTrackHashCap; ++i) {
            size_t s = (slot + i) & kTrackHashMask;
            uintptr_t existing = track_hash_[s].key.load(std::memory_order_relaxed);
            if (existing == key)
                return track_hash_[s].idx.load(std::memory_order_acquire);
            if (existing == 0) {
                uintptr_t expected = 0;
                if (track_hash_[s].key.compare_exchange_strong(
                        expected, key, std::memory_order_acq_rel)) {
                    size_t idx = next_page_.fetch_add(1, std::memory_order_release);
                    if (idx >= total_pages_) return 0;
                    track_reverse_[idx] = page_addr;
                    track_hash_[s].idx.store(idx, std::memory_order_release);
                    return idx;
                }
                // Another thread claimed this slot; recheck
                if (track_hash_[s].key.load(std::memory_order_relaxed) == key)
                    return track_hash_[s].idx.load(std::memory_order_acquire);
            }
        }
        return 0;
    }

    // ── External-page tracking (full mode) ──────────────────────────────────
    // Register a page from an application-direct mmap or Mach VM allocation.
    // Returns the global index assigned (>= contig_pages_), or 0 on failure
    // (table full, wrong mode, or page already inside the contiguous arena).
    //
    // The returned index lives in the same PageStateTable / PageLockTable as
    // contiguous-arena pages, so the compressor's existing tick/dispatch
    // logic processes external pages without modification.
    size_t trackExternalPage(uintptr_t page_addr) {
        if (tracking_mode_ || !track_hash_) return 0;
        // Slot budget exhausted? Bail BEFORE touching the hash. Without this
        // early-out, a mapping larger than the slot budget (e.g. a multi-GiB
        // InnoDB buffer pool) keeps CAS-inserting keys — which succeed (line
        // below) and only THEN discover local >= kTrackMaxPages — permanently
        // polluting the table. Once the table fills, every further page probes
        // the full kTrackHashCap before failing, turning registration into
        // O(pages × cap): a single 6 GiB mmap wedged mariadbd startup for
        // minutes. Reading external_slot_next_ up front caps total tracking
        // work at O(kTrackMaxPages) regardless of how many pages arrive.
        if (external_slot_next_.load(std::memory_order_relaxed) >= kTrackMaxPages)
            return 0;
        // Page already covered by smash's own contiguous arena? Skip.
        auto b = reinterpret_cast<uintptr_t>(base_);
        if (page_addr >= b && page_addr < b + contig_pages_ * kPageSize)
            return 0;

        uintptr_t key = page_addr >> kPageShift;
        if (key == 0) return 0;
        size_t slot = static_cast<size_t>(key * 0x9E3779B97F4A7C15ULL) >> (64 - 18);
        for (size_t i = 0; i < kTrackHashCap; ++i) {
            size_t s = (slot + i) & kTrackHashMask;
            uintptr_t existing = track_hash_[s].key.load(std::memory_order_relaxed);
            if (existing == key)
                return track_hash_[s].idx.load(std::memory_order_acquire);
            if (existing == 0) {
                uintptr_t expected = 0;
                if (track_hash_[s].key.compare_exchange_strong(
                        expected, key, std::memory_order_acq_rel)) {
                    // Reserve a slot first (use relaxed since we control visibility)
                    size_t local = external_slot_next_.fetch_add(1, std::memory_order_relaxed);
                    if (local >= kTrackMaxPages) {
                        // No more slots available
                        return 0;
                    }
                    size_t idx = contig_pages_ + local;
                    // Write address before publishing.
                    track_reverse_[local] = page_addr;
                    track_hash_[s].idx.store(idx, std::memory_order_release);
                    // Only increment external_count_ AFTER everything is set.
                    // This is what committedPages() reads, so the compressor
                    // won't iterate to this index until the address is valid.
                    // Use release so the address write is visible before the
                    // count increment.
                    external_count_.fetch_add(1, std::memory_order_release);
                    return idx;
                }
                if (track_hash_[s].key.load(std::memory_order_relaxed) == key)
                    return track_hash_[s].idx.load(std::memory_order_acquire);
            }
        }
        return 0;
    }

    // Untrack a previously-registered external page (e.g. on munmap /
    // vm_deallocate). Marks the hash slot as freed (key = ~0ULL — a
    // tombstone — so the open-addressing probe skips past it but treats it
    // as occupied for collision purposes). The PageStateTable entry must be
    // cleared by the caller (set to EMPTY) so the compressor stops scanning
    // that index.
    void untrackExternalPage(uintptr_t page_addr) {
        if (tracking_mode_ || !track_hash_) return;
        uintptr_t key = page_addr >> kPageShift;
        if (key == 0) return;
        size_t slot = static_cast<size_t>(key * 0x9E3779B97F4A7C15ULL) >> (64 - 18);
        for (size_t i = 0; i < kTrackHashCap; ++i) {
            size_t s = (slot + i) & kTrackHashMask;
            uintptr_t existing = track_hash_[s].key.load(std::memory_order_relaxed);
            if (existing == 0) return;  // not present
            if (existing == key) {
                // Mark with a tombstone so probe doesn't terminate early.
                track_hash_[s].key.store(~uintptr_t{0}, std::memory_order_release);
                return;
            }
        }
    }

    // Untrack an external page UNDER ITS PER-PAGE LOCK, the serialization the
    // formal model (model/SmashExternalRace.{lean,tla}) proves is required to
    // make external munmap safe against a concurrent compressor worker.
    //
    // The compressor worker compresses an external page by snapshotting it:
    //   tryLock(idx); check ACTIVE; state=COMPRESSING; memcpy(buf, pageAddress(idx), kPageSize)
    // all UNDER lock[idx]. If munmap clears state + tears down the mapping
    // WITHOUT that lock (the pre-fix bug), the worker's memcpy reads an unmapped
    // page → SIGSEGV. By taking lock[idx] here we cannot run while the worker
    // holds it mid-snapshot; and the worker re-loads pageAddress(idx) under the
    // lock and bails when we've cleared track_reverse_ to 0.
    //
    // DEADLOCK SAFETY: this holds only a per-page lock and does NOT perform any
    // mprotect / TLB-shootdown / real_munmap while holding it — the caller does
    // real_munmap AFTER this returns. So it cannot reintroduce the
    // TLB-shootdown-vs-page-lock deadlock documented in compressor_thread.h.
    // (locks/states are passed in to avoid a header dependency cycle.)
    void untrackExternalPageLocked(uintptr_t page_addr,
                                   PageLockTable* locks,
                                   PageStateTable* states) {
        if (tracking_mode_ || !track_hash_) return;
        size_t idx = pageIndex(page_addr);
        if (idx == 0) return;                       // not tracked
        if (locks) locks->lock(idx);
        if (states) states->set(idx, PageState::EMPTY);
        untrackExternalPage(page_addr);             // tombstone the hash slot
        // Clear the reverse map so pageAddress(idx) returns nullptr — a worker
        // that observes the page post-untrack gets null and skips it instead of
        // dereferencing a stale (soon-to-be-unmapped) address.
        if (idx >= contig_pages_) {
            size_t local = idx - contig_pages_;
            if (local < kTrackMaxPages) track_reverse_[local] = 0;
        }
        if (locks) locks->unlock(idx);
    }

    bool contains(uintptr_t addr) const {
        if (tracking_mode_) {
            return lookupIdx(addr) != 0;
        }
        // Full mode: contiguous range OR external hash hit.
        auto b = reinterpret_cast<uintptr_t>(base_);
        if (addr >= b && addr < b + contig_pages_ * kPageSize) return true;
        return lookupIdx(addr) != 0;
    }

    size_t pageIndex(uintptr_t addr) const {
        if (tracking_mode_) return lookupIdx(addr);
        auto b = reinterpret_cast<uintptr_t>(base_);
        if (addr >= b && addr < b + contig_pages_ * kPageSize)
            return (addr - b) / kPageSize;
        return lookupIdx(addr);
    }

    void* pageAddress(size_t index) const {
        if (tracking_mode_) {
            if (index == 0 || index >= total_pages_) return nullptr;
            uintptr_t addr = track_reverse_[index];
            return addr ? reinterpret_cast<void*>(addr) : nullptr;
        }
        // Full mode
        if (index < contig_pages_) return base_ + index * kPageSize;
        size_t local = index - contig_pages_;
        if (local >= kTrackMaxPages) return nullptr;
        // track_reverse_[local] may be 0 if the slot was reserved but the
        // address hasn't been written yet (race with trackExternalPage).
        uintptr_t addr = track_reverse_[local];
        return addr ? reinterpret_cast<void*>(addr) : nullptr;
    }

    // ── Flat page-index → Span* table ───────────────────────────────────────
    // Single-load lookup that replaces the two-level radix walk on the
    // free()/getSize() hot path. Returns nullptr if the table isn't mapped
    // (early init, mmap failure) or if the page isn't currently owned by a
    // span — callers fall back to PageMap::get() in that case.
    [[gnu::always_inline]]
    Span* getSpan(size_t page_idx) const {
        if (!page_to_span_) [[unlikely]] return nullptr;
        if (page_idx >= total_pages_) [[unlikely]] return nullptr;
        return page_to_span_[page_idx].load(std::memory_order_acquire);
    }

    [[gnu::always_inline]]
    void setSpan(size_t page_idx, Span* span) {
        if (!page_to_span_) [[unlikely]] return;
        if (page_idx >= total_pages_) [[unlikely]] return;
        page_to_span_[page_idx].store(span, std::memory_order_release);
    }

    // Set a contiguous run of flat-table slots to the same span. Bounds are
    // checked once for the whole run, then the loop is a tight sequential store
    // — avoids the per-page table-pointer null check + bound compare that
    // calling setSpan() num_pages times would repeat. Used by the span-creation
    // path (PageMap::setRange) where every page of a fresh span maps to the
    // same Span*.
    void setSpanRange(size_t first_page_idx, size_t num_pages, Span* span) {
        if (!page_to_span_) [[unlikely]] return;
        if (first_page_idx + num_pages > total_pages_) [[unlikely]] return;
        for (size_t i = 0; i < num_pages; ++i)
            page_to_span_[first_page_idx + i].store(span, std::memory_order_release);
    }

    // True iff the flat table is mapped (mmap succeeded in init()). Used by
    // PageMap to decide whether to write the flat slot in addition to the
    // radix tree.
    bool hasSpanTable() const { return page_to_span_ != nullptr; }

    // Cheap pointer-in-contiguous-arena check. Used by the free() / getSize()
    // hot path to decide whether the flat span table lookup is sufficient
    // (single load) or whether the caller must fall through to PageMap
    // (which also handles non-VmRegion large allocations). Excludes the
    // external-tracking tail because those pages aren't reachable through a
    // contiguous-base offset.
    [[gnu::always_inline]]
    bool inContigArena(uintptr_t addr) const {
        auto b = reinterpret_cast<uintptr_t>(base_);
        return addr >= b && addr < b + contig_pages_ * kPageSize;
    }

    // Contig-arena-only page index. Faster than pageIndex() which dispatches
    // on tracking-mode and hashes for external pages. Caller must have
    // verified inContigArena(addr).
    [[gnu::always_inline]]
    size_t contigPageIndex(uintptr_t addr) const {
        return (addr - reinterpret_cast<uintptr_t>(base_)) >> kPageShift;
    }

    char* base() const { return base_; }
    size_t totalPages() const { return total_pages_; }
    // Contiguous-arena page count. In full mode, this is the bump-arena
    // capacity (total_pages_ minus the external-page tail). In tracking
    // mode the contiguous arena is unused, so the value is 0.
    size_t contigPages() const { return contig_pages_; }

    // Total external-page slot budget (compile-time constant). Exposed so the
    // mmap interposer can cheaply reject a single mapping that could never fit
    // — see registerLinuxExternalRange().
    static constexpr size_t externalSlotCapacity() { return kTrackMaxPages; }
    // Remaining unclaimed external slots (approximate; races with concurrent
    // tracking but only used as a fast pre-filter, never for correctness).
    size_t externalSlotsRemaining() const {
        size_t used = external_slot_next_.load(std::memory_order_relaxed);
        return used >= kTrackMaxPages ? 0 : kTrackMaxPages - used;
    }
    size_t rawNextPage() const { return next_page_.load(std::memory_order_acquire); }

    // Arena occupancy, for the exhausted warning and the stats line.
    size_t arenaFreePages() { return free_map_.freePages(kSharedOwner); }
    size_t arenaLongestFreeRun() { return free_map_.longestRunAtLeast(kSharedOwner); }

    size_t committedPages() const {
        size_t bump = next_page_.load(std::memory_order_acquire);
        if (tracking_mode_) return bump;
        size_t ext = external_count_.load(std::memory_order_relaxed);
        if (ext == 0) return bump;
        // External pages occupy [contig_pages_, contig_pages_ + ext); the
        // compressor must iterate up to the high end. Pages between bump
        // and contig_pages_ are EMPTY → cheap to skip via the chunk bitmap.
        return contig_pages_ + ext;
    }

    // ── NUMA node arenas (malloc_numa) ──────────────────────────────────────
    // One arena per NUMA node inside the contiguous region. An arena grows by
    // carving whole chunks of kNodeChunkPages off the shared bump pointer,
    // aligned to a fixed grid, and binds each chunk to its node once with
    // mbind. It keeps its own free list, so its pages are only ever reused by
    // the same node. That means no mbind per allocation, no policy reset on
    // free, and no binding leaking into ordinary allocations. The pages share
    // the index space of every other region page, so the compressor and the
    // fault handler treat them like any other page.
    //
    // Why not a fixed slice per node: the compressor walks page indices
    // [0, committedPages()) page by page every tick (chunk-bitmap rebuild,
    // deferred-madvise sweep, soft-dirty pagemap read). Slices above the
    // shared heap would stretch that walk past the whole shared region as
    // soon as one node arena was used. Carving from the bump pointer keeps it
    // compact.
    //
    // A remap of a node page (vm::remapPages) replaces its VMA and drops the
    // policy. rebindAfterRemap(), installed as vm::g_after_remap_hook, binds
    // the page again.
    static constexpr size_t kNodeChunkPages = (64 * 1024 * 1024) / kPageSize;

    // NUMA node owning contiguous page `idx`, or -1. Lock-free and
    // async-signal-safe.
    int nodeOfPage(size_t idx) const {
        const auto* table = node_of_chunk_.load(std::memory_order_acquire);
        if (!table || idx >= contig_pages_) return -1;
        return static_cast<int>(
            table[idx / kNodeChunkPages].load(std::memory_order_acquire)) - 1;
    }

#if defined(__linux__)
    // Allocate `num_pages` from `node`'s arena: its free list first, then the
    // unused tail of its newest chunk, then a newly carved and bound chunk.
    // Returns nullptr with errno EINVAL (bad node, from mbind), EPERM/ENOSYS
    // (mbind refused), or ENOMEM (region exhausted).
    void* allocatePagesOnNode(size_t num_pages, uint32_t node) {
        if (tracking_mode_ || !base_ || num_pages == 0 || num_pages > contig_pages_) {
            errno = ENOMEM;
            return nullptr;
        }
        if (node >= vm::kMaxNumaNodes) {
            errno = EINVAL;
            return nullptr;
        }
        NodeArena* arenas = ensureNodeArenas();
        if (!arenas) {
            errno = ENOMEM;
            return nullptr;
        }
        NodeArena& na = arenas[node];
        LockGuard guard(na.lock);

        if (const size_t idx = free_map_.take(num_pages, ownerOfNode(static_cast<int>(node)));
            idx != PageRunMap::kNone) {
            return base_ + idx * kPageSize;
        }

        if (na.end - na.cur < num_pages) {
            const size_t chunk_pages =
                (num_pages + kNodeChunkPages - 1) / kNodeChunkPages * kNodeChunkPages;
            const size_t start = carveNodeChunk(chunk_pages);
            if (start == kNoChunk) {
                errno = ENOMEM;
                return nullptr;
            }
            void* chunk = base_ + start * kPageSize;
            // Freshly carved, so nothing in the chunk has been touched: no
            // pages to move.
            if (!vm::bindPagesToNode(chunk, chunk_pages * kPageSize, node)) {
                const int err = errno;
                // mbind can fail after binding part of the range.
                vm::unbindPages(chunk, chunk_pages * kPageSize);
                donateToShared(start, chunk_pages);
                errno = err;
                return nullptr;
            }
            auto* table = node_of_chunk_.load(std::memory_order_acquire);
            for (size_t c = start / kNodeChunkPages;
                 c < (start + chunk_pages) / kNodeChunkPages; ++c)
                table[c].store(static_cast<uint16_t>(node + 1), std::memory_order_release);
            if (na.end > na.cur)
                free_map_.insert(na.cur, na.end - na.cur, ownerOfNode(static_cast<int>(node)));
            na.cur = start;
            na.end = start + chunk_pages;
        }
        const size_t idx = na.cur;
        na.cur += num_pages;
        return base_ + idx * kPageSize;
    }
#endif

private:
    struct NodeArena {
        Spinlock lock;   // guards cur/end; freed pages live in free_map_
        size_t cur = 0;  // unused tail of the newest chunk, [cur, end);
        size_t end = 0;  // guarded by lock
    };
    static constexpr size_t kNoChunk = ~size_t{0};

    // Both created on first use, indexed by node id and by grid chunk.
    // node_of_chunk_[c] is node+1, or 0 for a chunk no node arena owns. It is
    // written before any of the chunk's pages are handed out.
    std::atomic<NodeArena*> node_arenas_{nullptr};
    std::atomic<std::atomic<uint16_t>*> node_of_chunk_{nullptr};
    Spinlock numa_init_lock_;
    // Region that vm::g_after_remap_hook binds pages again for.
    static inline std::atomic<VmRegion*> numa_region_{nullptr};

#if defined(__linux__)
    NodeArena* ensureNodeArenas() {
        if (NodeArena* a = node_arenas_.load(std::memory_order_acquire)) [[likely]]
            return a;
        LockGuard guard(numa_init_lock_);
        if (NodeArena* a = node_arenas_.load(std::memory_order_relaxed)) return a;
        auto* table = bootstrapArray<std::atomic<uint16_t>>(
            (contig_pages_ + kNodeChunkPages - 1) / kNodeChunkPages);
        auto* arenas = bootstrapArray<NodeArena>(vm::kMaxNumaNodes);
        if (!table || !arenas) return nullptr;
        node_of_chunk_.store(table, std::memory_order_release);
        // The first region to use node arenas owns the remap hook: the heap's
        // region in production, while tests may create others.
        VmRegion* none = nullptr;
        if (numa_region_.compare_exchange_strong(none, this))
            vm::g_after_remap_hook = &rebindAfterRemap;
        node_arenas_.store(arenas, std::memory_order_release);
        return arenas;
    }

    // Claim `pages` (a multiple of kNodeChunkPages) off the shared bump
    // pointer, starting on the chunk grid. The pages skipped below the grid
    // line are fresh and unbound, so they go to the shared free lists.
    size_t carveNodeChunk(size_t pages) {
        size_t cur = next_page_.load(std::memory_order_acquire);
        size_t start = 0;
        size_t end = 0;
        do {
            // An overflowing allocatePages() bump briefly leaves next_page_
            // past contig_pages_ before undoing it. That value always fails
            // this check, so the CAS never commits on top of it.
            start = (cur + kNodeChunkPages - 1) / kNodeChunkPages * kNodeChunkPages;
            end = start + pages;
            if (end > contig_pages_) return kNoChunk;
        } while (!next_page_.compare_exchange_weak(cur, end, std::memory_order_acq_rel));
        // Commit before donating the gap: allocatePages() hands out free-list
        // pages without committing them.
        ensureCommitted(end);
        if (start > cur) donateToShared(cur, start - cur);
        return start;
    }

    void donateToShared(size_t page_index, size_t page_count) {
        free_map_.insert(page_index, page_count, kSharedOwner);
    }

    // vm::g_after_remap_hook. remapPages() just replaced [addr, addr+size)
    // with a default-policy VMA; bind each run of pages that belongs to a node
    // arena back to its node. MPOL_MF_MOVE because an RW remap is visible to
    // other threads before this runs, and one of them may already have placed
    // a page by touching it. Runs in fault-handler context: lock-free lookups
    // and raw syscalls only, errno preserved.
    static void rebindAfterRemap(void* addr, size_t size) {
        VmRegion* self = numa_region_.load(std::memory_order_acquire);
        if (!self) return;
        const int saved_errno = errno;
        const auto a = reinterpret_cast<uintptr_t>(addr);
        const size_t pages = size / kPageSize;
        auto nodeAt = [&](size_t i) {
            const uintptr_t p = a + i * kPageSize;
            return self->inContigArena(p) ? self->nodeOfPage(self->contigPageIndex(p)) : -1;
        };
        for (size_t i = 0; i < pages;) {
            const int node = nodeAt(i);
            size_t j = i + 1;
            while (j < pages && nodeAt(j) == node) ++j;
            if (node >= 0) {
                (void)vm::bindPagesToNode(reinterpret_cast<void*>(a + i * kPageSize),
                                          (j - i) * kPageSize,
                                          static_cast<uint32_t>(node), MPOL_MF_MOVE);
            }
            i = j;
        }
        errno = saved_errno;
    }
#endif
};

} // namespace smash
