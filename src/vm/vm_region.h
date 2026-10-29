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
#include <time.h>
#include <signal.h>
#include <unistd.h>
#include "smash/config.h"
#include "platform_mem.h"
#include "page_run_map.h"
#include "page_state.h"
#if defined(__linux__)
#include "node_memory.h"
#endif
#include "../core/bootstrap_alloc.h"
#include "../util/spinlock.h"
#include "../util/fork_locks.h"
#include "../util/cpu_shard.h"
#include "../util/exit_guard.h"
#include "../util/park.h"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#if defined(SMASH_SIM)
#include <algorithm>
#include <vector>
#endif

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
        unsigned shard;  // the entry pool shard it came from and returns to
        bool reusable;   // no page was COMPRESSED: none is PROT_NONE (see processDecommitEntry)
        DecommitEntry* next;
    };

    // The pool of spare entries is sharded by CPU (256 shards, see
    // util/cpu_shard.h), one cache line per shard.
    // It used to be one list under one spinlock, taken twice per large free
    // (here, and again when the entry is recycled) — on a 128-thread
    // malloc_numa workload on 128 cores that lock was 32% of all CPU, under
    // releasePages. It also shared a cache line with decommit_head_, which
    // every free CASes, so the line bounced even when the lock was free.
    //
    // An entry goes back to the shard it came from, not the recycler's, so the
    // shards stay balanced even though entries are taken by freeing threads
    // and recycled by whichever thread drains the queue. A thread that finds
    // its shard empty (it migrated, or its CPU's frees are all in flight)
    // allocates a new entry, which then belongs to that shard; the total is
    // bounded by what can be in flight per shard, and entries are 48 bytes.
    // No thread-local state, so nothing is stranded when a thread exits.
    static constexpr unsigned kEntryShards = 256;
    struct alignas(64) EntryShard {
        Spinlock lock;
        DecommitEntry* head = nullptr;
    };
    EntryShard entry_shards_[kEntryShards];

    // One decommit queue per owner — the shared arena and each node arena —
    // each with its own head, in-flight counters and thread. A single global
    // queue made every large free CAS one cache line and every freed run wait
    // for one thread's mprotect + madvise: on smash_sim that thread ran a full
    // core and the 1..7-thread sweep fitted a serial fraction of 9.5%, which
    // caps speedup near 10x however many cores there are. A node's runs only
    // ever go back to that node's free map, so nothing needs to see another
    // node's queue: an allocation that misses drains and waits on its own.
    struct alignas(64) DecommitQueue {
        std::atomic<DecommitEntry*> head{nullptr};
        // Runs that have left their owner and not yet reached its free map:
        // queued, being processed, or still on their way to being queued.
        std::atomic<int> pending{0};
        // Threads inside drainQueueNow() with a batch of this queue in hand.
        std::atomic<int> draining{0};
        std::atomic<bool> running{false};
        // A sub-arena of this queue's set went over its dirty cap: purge on
        // the thread's next pass instead of waiting for the epoch.
        std::atomic<bool> purge_now{false};
        // Parking (util/park.h): the thread sleeps on `wake` while `sleeping`
        // is set; whoever gives it work bumps `wake` and wakes it (kick()).
        std::atomic<uint32_t> wake{0};
        std::atomic<uint32_t> sleeping{0};
        pthread_t thread{};
        VmRegion* region = nullptr;
        int node = -1;              // -1: the shared arena's queue
    };
    DecommitQueue shared_queue_;

    DecommitEntry* newDecommitEntry() {
        const unsigned s = currentCpuShard(kEntryShards);
        EntryShard& sh = entry_shards_[s];
        {
            LockGuard guard(sh.lock);
            if (DecommitEntry* e = sh.head) {
                sh.head = e->next;
                return e;           // e->shard == s already
            }
        }
        auto* e = static_cast<DecommitEntry*>(
            BootstrapAlloc::instance().allocate(sizeof(DecommitEntry), alignof(DecommitEntry)));
        if (e) e->shard = s;
        return e;
    }

    void recycleDecommitEntry(DecommitEntry* e) {
        // Balances the increment in queueForDecommit: an entry is recycled
        // exactly once, after it has been processed and its run is back in the
        // free map. The two paths that process a stack entry instead of a
        // queued one (queueForDecommit's no-entry fallback and
        // releasePagesNow) never reach here, so the count stays balanced.
        queueOf(e->node).pending.fetch_sub(1, std::memory_order_acq_rel);
        EntryShard& sh = entry_shards_[e->shard];
        LockGuard guard(sh.lock);
        e->next = sh.head;
        sh.head = e;
    }

    void queueForDecommit(void* addr, size_t num_pages, size_t page_index, int node,
                          bool reusable) {
        DecommitQueue& q = queueOf(node);
        // Counted from before the entry even exists. A run that has left its
        // owner but has not yet reached the free map is invisible to an
        // allocation looking for it, and newDecommitEntry() can block on the
        // pool lock — a window wide enough to matter: with the count taken
        // only at the push, a 128-thread churn still put 23 of 320000 blocks
        // outside the arena.
        q.pending.fetch_add(1, std::memory_order_acq_rel);
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
            local.reusable = reusable;
            processDecommitEntry(&local);
            q.pending.fetch_sub(1, std::memory_order_acq_rel);
            return;
        }
        e->addr = addr;
        e->num_pages = num_pages;
        e->page_index = page_index;
        e->node = node;
        e->reusable = reusable;
        // Lock-free push to this owner's MPSC queue
        DecommitEntry* head = q.head.load(std::memory_order_relaxed);
        do {
            e->next = head;
        } while (!q.head.compare_exchange_weak(head, e,
            std::memory_order_seq_cst, std::memory_order_relaxed));
        // Onto an empty queue: the thread may be parked. Onto a non-empty one,
        // whoever made it non-empty woke it, or it has not drained yet, and it
        // drains everything before it parks again.
        if (!head) kick(q);
    }

    // Wake the queue's thread if it is parked. Paired with the parking in
    // decommitThreadEntry: either the thread sees the caller's work (the
    // queue, purge_now, running) before it parks, or the caller sees it
    // sleeping here (both sides seq_cst).
    static void kick(DecommitQueue& q) {
        if (q.sleeping.load(std::memory_order_seq_cst) == 0) return;
        q.wake.fetch_add(1, std::memory_order_release);
        unpark(q.wake, 1);
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
        //
        // Only then, though. A run whose release reported no COMPRESSED page
        // (`reusable`) has no PROT_NONE page, and a PROT_READ one left by
        // Phase 3 monitoring is restored by the fault handler once the run's
        // next owner marks it ACTIVE — every taker of free-map runs does (slab
        // spans, mid bins, large blocks), the guarantee the run cache and the
        // sub-arenas' dirty reuse rely on. Skipping the mprotect matters:
        // it takes the process's mmap_lock for writing, and with 32 threads
        // churning 2 MiB blocks it cost 251 us per run (0.8 us alone) and
        // dragged the madvise after it from 6.7 to 123 us, so freed runs came
        // back too slowly and allocations fell out of the arena
        // (test_shared_arena_churn).
        if (!e->reusable) vm::commitPages(e->addr, e->num_pages * kPageSize);

        // Decommit the pages (release physical backing).
        vm::decommitPages(e->addr, e->num_pages * kPageSize);

        // NOW add to the free map (safe - pages are decommitted and PROT_RW),
        // merging with free neighbours of the same owner. A node arena's pages
        // go back to that arena only (see NodeArena).
#if defined(__linux__)
        // Sub-arena chunks: a compressed block's pages go back to their
        // sub-arena as clean; a block larger than a chunk returns its chunks.
        // The shared set's chunks carry no node, so e->node is -1 for them.
        if (subarenasEnabled()) {
            auto* owners = chunk_sub_.load(std::memory_order_acquire);
            const uint8_t o = owners ? owners[chunkOfPage(e->page_index)].load(
                                           std::memory_order_acquire)
                                     : kChunkUnowned;
            NodeArena* set = nullptr;
            if (o != kChunkUnowned)
                set = e->node >= 0 ? &node_arenas_.load(std::memory_order_acquire)[e->node]
                                   : shared_set_.load(std::memory_order_acquire);
            if (set && o == kChunkBig) {
                NodeArena& na = *set;
                const size_t c0 = chunkOfPage(e->page_index);
                const size_t c1 = chunkOfPage(e->page_index + e->num_pages - 1);
                LockGuard guard(na.chunk_lock);
                for (size_t c = c0; c <= c1; ++c) {
                    owners[c].store(kChunkUnowned, std::memory_order_release);
                    na.free_chunks[na.nfree_chunks++] = static_cast<uint32_t>(c);
                }
                return;
            }
            if (set) {
                NodeArena& na = *set;
                const uint32_t sidx = static_cast<uint32_t>(o - 1);
                LockGuard guard(na.subs[sidx].lock);
                na.map.insertUnlocked(e->page_index, e->num_pages, ownerClean(sidx),
                                      kNodeChunkPages, 0);
                maybeReturnChunkLocked(na, sidx, e->page_index);
                return;
            }
        }
#endif
        mapOf(ownerOfNode(e->node)).insert(e->page_index, e->num_pages, ownerOfNode(e->node));
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
        auto* q = static_cast<DecommitQueue*>(arg);
        VmRegion* self = q->region;
        uint64_t last_trim_ms = 0, last_purge_ms = 0;
        // An urgent purge that left a sub-arena over its cap (its lock was
        // held by a free in progress: the purge only tries the lock) is
        // retried 1 ms later rather than at the next epoch.
        bool retry_urgent = false;
        while (q->running.load(std::memory_order_relaxed)) {
            // Return cached runs that have aged out (see cacheFreedRun). The
            // clock is the coarse vDSO one, so this costs a few ns per wake.
            // One thread does it: the shared queue's.
            if (subarenasEnabled()) {
                const uint64_t now = nowMs();
                const bool urgent = (q->purge_now.load(std::memory_order_relaxed) &&
                                     q->purge_now.exchange(false, std::memory_order_acq_rel)) ||
                                    retry_urgent;
                if (urgent || now - last_purge_ms >= purgeEpochMs()) {
                    last_purge_ms = now;
                    if (q->node >= 0)
                        self->purgeNodeSubArenas(static_cast<uint32_t>(q->node), /*all=*/false);
                    else
                        self->purgeSharedSubArenas(/*all=*/false);
                    retry_urgent = urgent && self->overDirtyCap(*q);
                }
            }
            if (q->node < 0 && runCacheCapBytes() != 0) {
                const uint64_t now = nowMs();
                if (now - last_trim_ms >= kRunCacheTrimIntervalMs) {
                    last_trim_ms = now;
                    self->trimRunCache(/*all=*/false);
                }
            }
            // Drain this owner's queue
            DecommitEntry* batch = q->head.exchange(nullptr, std::memory_order_acquire);
            if (batch) {
                while (batch) {
                    DecommitEntry* next = batch->next;
                    self->processDecommitEntry(batch);
                    self->recycleDecommitEntry(batch);
                    batch = next;
                }
            } else {
                // Nothing queued: park until a free queues work, a sub-arena
                // goes over its dirty cap (purge_now), stop(), or the next
                // periodic duty. It used to poll with usleep(100) here: ~6 300
                // wakeups a second per queue thread, forever, once anything
                // had been freed (31 600/s with a node queue on each of 4
                // nodes).
                uint64_t wait_ms = 0;   // no periodic duty: until woken
                const uint64_t now = nowMs();
                auto due = [&](uint64_t last, uint64_t every) {
                    const uint64_t left = now - last >= every ? 1 : every - (now - last);
                    if (wait_ms == 0 || left < wait_ms) wait_ms = left;
                };
                if (subarenasEnabled()) due(last_purge_ms, retry_urgent ? 1 : purgeEpochMs());
                if (q->node < 0 && runCacheCapBytes() != 0) due(last_trim_ms, kRunCacheTrimIntervalMs);
                const uint32_t seq = q->wake.load(std::memory_order_acquire);
                q->sleeping.store(1, std::memory_order_seq_cst);
                if (q->running.load(std::memory_order_seq_cst) &&
                    !q->head.load(std::memory_order_seq_cst) &&
                    !q->purge_now.load(std::memory_order_seq_cst))
                    parkFor(q->wake, seq, wait_ms);
                q->sleeping.store(0, std::memory_order_relaxed);
            }
        }
        return nullptr;
    }

    void startDecommitThread(DecommitQueue& q) {
        if (tracking_mode_) return;
        bool expected = false;
        if (!q.running.compare_exchange_strong(expected, true,
                std::memory_order_acq_rel, std::memory_order_relaxed)) {
            return;
        }
        q.region = this;
        if (pthread_create(&q.thread, nullptr, decommitThreadEntry, &q) != 0) {
            // EAGAIN under thread exhaustion. Without the rollback,
            // decommit_running_ claims a drainer exists while nothing ever
            // pops the MPSC queue: freed runs never reach the free list
            // (allocation starvation) and are never madvise'd (RSS never
            // drops). Roll back so the next release retries creation, and
            // drain whatever is queued right now inline (the exchange-based
            // drain is safe from any thread).
            q.running.store(false, std::memory_order_release);
            DecommitEntry* batch =
                q.head.exchange(nullptr, std::memory_order_acquire);
            while (batch) {
                DecommitEntry* next = batch->next;
                processDecommitEntry(batch);
                recycleDecommitEntry(batch);
                batch = next;
            }
        }
    }

    void ensureDecommitThreadStarted(DecommitQueue& q) {
        if (tracking_mode_) return;
        if (q.running.load(std::memory_order_relaxed)) return;
        startDecommitThread(q);
    }

    void stopDecommitThread(DecommitQueue& q) {
        if (!q.running.load(std::memory_order_relaxed)) return;
        q.running.store(false, std::memory_order_seq_cst);
        kick(q);
        // Never join self — see the same guard in CompressorThread::stop().
        if (!pthread_equal(q.thread, pthread_self()))
            pthread_join(q.thread, nullptr);
        // Drain any remaining entries
        DecommitEntry* batch = q.head.exchange(nullptr, std::memory_order_acquire);
        while (batch) {
            DecommitEntry* next = batch->next;
            processDecommitEntry(batch);
            recycleDecommitEntry(batch);
            batch = next;
        }
    }

    // Every queue: the shared one and each node's that exists.
    template <typename Fn> void forEachQueue(Fn&& fn) {
        fn(shared_queue_);
        if (NodeArena* arenas = node_arenas_.load(std::memory_order_acquire))
            for (uint32_t n = 0; n < vm::kMaxNumaNodes; ++n) fn(arenas[n].queue);
    }
    template <typename Fn> void forEachQueue(Fn&& fn) const {
        fn(shared_queue_);
        if (const NodeArena* arenas = node_arenas_.load(std::memory_order_acquire))
            for (uint32_t n = 0; n < vm::kMaxNumaNodes; ++n) fn(arenas[n].queue);
    }
    void stopDecommitThread() {
        forEachQueue([&](DecommitQueue& q) { stopDecommitThread(q); });
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
#if defined(__linux__) && defined(MADV_HUGEPAGE)
            // SMASH_THP=1: ask for transparent hugepages over the arena.
            // Opt-in, because the trade is sharp in both directions and only
            // the caller knows which side its workload is on. Measured on a
            // THP=madvise host, 256 MiB hot + 256 MiB cold:
            //
            //   hint off  RSS 284 MiB, AnonHugePages   0
            //   hint on   RSS 286 MiB, AnonHugePages 256 MiB
            //
            // i.e. the hot set stays huge-page backed (TLB coverage) while the
            // cold set still compresses, for 2 MiB. But with a sparse touch
            // pattern -- one 4 KiB page per 2 MiB across 512 MiB -- the same
            // hint takes RSS from 24 MiB to 534 MiB, because every touched
            // page drags in a whole huge page. That is the opposite of what
            // this allocator is for, so it stays off unless asked for.
            //
            // Compression itself dismantles them: the compressor protects and
            // reclaims 4 KiB at a time, which splits a huge page, and the
            // restore path (per-page MAP_FIXED remap) never rebuilds one.
            // Measured: a 512 MiB block is 510 MiB huge-backed after touch and
            // 0 after one compress/read-back cycle. The hint is therefore
            // worth something only for pages that stay hot.
            {
                const char* v = std::getenv("SMASH_THP");
                const char* t = std::getenv("SMASH_WRITE_TRACK");
                if (v && v[0] == '1') {
                    (void)::madvise(base_, region_size, MADV_HUGEPAGE);
                } else if (t && std::strcmp(t, "idle") == 0) {
                    // The page_idle bitmap answers for a huge page through
                    // its head PFN only; the idle tracker would see the other
                    // 511 pages as accessed for ever (THP=always hosts).
                    (void)::madvise(base_, region_size, MADV_NOHUGEPAGE);
                }
            }
#endif
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

    // Every lock of the region, for fork() (util/fork_locks.h).
    void forkLocks(ForkLockSet& s) {
        s.add(commit_lock_);
        s.add(numa_init_lock_);
        free_map_.forkLocks(s);
        for (EntryShard& sh : entry_shards_) s.add(sh.lock);
        for (RunCacheShard& sh : run_cache_) s.add(sh.lock);
        auto one = [&](NodeArena& na) {
            s.add(na.lock);
            s.add(na.chunk_lock);
            if (na.map_ready.load(std::memory_order_acquire)) na.map.forkLocks(s);
            if (na.subs_ready.load(std::memory_order_acquire))
                for (uint32_t i = 0; i < na.nsubs; ++i) s.add(na.subs[i].lock);
        };
        if (NodeArena* arenas = node_arenas_.load(std::memory_order_acquire))
            for (uint32_t n = 0; n < vm::kMaxNumaNodes; ++n) one(arenas[n]);
        if (NodeArena* shared = shared_set_.load(std::memory_order_acquire)) one(*shared);
    }

    bool isTrackingMode() const { return tracking_mode_; }

    // Reset decommit thread state after fork(). The thread doesn't exist in the
    // child (Linux fork only clones the calling thread), so we must reset state
    // so ensureDecommitThreadStarted() will restart it on first releasePages().
    void resetDecommitThreadForFork() {
        // Threads don't exist in child - don't try to join them
        forEachQueue([](DecommitQueue& q) {
            q.running.store(false, std::memory_order_relaxed);
            q.sleeping.store(0, std::memory_order_relaxed);
        });
        // A parent thread may have held a pool or run-cache shard lock at the
        // fork; that thread does not exist here. Each critical section is a
        // few pointer moves, so the lists themselves are consistent enough to
        // keep; the locks are not.
        for (EntryShard& sh : entry_shards_) new (&sh.lock) Spinlock();
        for (RunCacheShard& sh : run_cache_) new (&sh.lock) Spinlock();
        if (NodeArena* arenas = node_arenas_.load(std::memory_order_acquire))
            for (uint32_t n = 0; n < vm::kMaxNumaNodes; ++n)
                new (&arenas[n].chunk_lock) Spinlock();
        if (NodeArena* s = shared_set_.load(std::memory_order_acquire))
            new (&s->chunk_lock) Spinlock();
        // A purge the parent's decommit thread had in flight never finishes
        // here: its runs are lost to the child, and its count must not make
        // purgeAllDirtyNow() wait forever.
        forEachSubSet([](NodeArena& na, int) {
            for (uint32_t i = 0; i < na.nsubs; ++i) {
                new (&na.subs[i].lock) Spinlock();
                na.subs[i].purging.store(0, std::memory_order_relaxed);
            }
        });
        drainDecommitQueueNow();
    }

    // Process every queued decommit on the calling thread; returns how many
    // entries were drained. A freed run reaches the free map only when its
    // entry is processed, so an allocation that misses the map can call this
    // to find out whether the pages it wants are merely in flight rather than
    // gone. Does the mprotect/madvise work inline, so it is for paths that
    // would otherwise claim fresh region or fail outright, never a fast path.
    size_t drainQueueNow(DecommitQueue& q) {
        DecommitEntry* batch = q.head.exchange(nullptr, std::memory_order_acquire);
        if (!batch) return 0;
        // Published before the entries are processed and cleared after, so a
        // thread that lost the exchange can tell "nothing queued" from
        // "another thread is holding the queue" — see inFlight().
        q.draining.fetch_add(1, std::memory_order_acq_rel);
        size_t drained = 0;
        while (batch) {
            DecommitEntry* next = batch->next;
            processDecommitEntry(batch);
            recycleDecommitEntry(batch);
            batch = next;
            ++drained;
        }
        q.draining.fetch_sub(1, std::memory_order_acq_rel);
        return drained;
    }

    // Is a run of this queue's owner still on its way back to its free map,
    // either queued or being processed by another thread right now?
    static bool inFlight(const DecommitQueue& q) {
        return q.pending.load(std::memory_order_acquire) != 0 ||
               q.head.load(std::memory_order_acquire) != nullptr ||
               q.draining.load(std::memory_order_acquire) != 0;
    }

    // Every queue, for shutdown, fork and tools: how many entries drained.
    size_t drainDecommitQueueNow() {
        size_t n = 0;
        forEachQueue([&](DecommitQueue& q) { n += drainQueueNow(q); });
        return n;
    }
    bool drainInFlight() const {
        bool any = false;
        forEachQueue([&](const DecommitQueue& q) { any = any || inFlight(q); });
        return any;
    }

    // ── Per-CPU cache of freed large runs (SMASH_RUN_CACHE_MIB) ──────────────
    // A large block freed warm is parked on the freeing CPU's shard instead of
    // going through the decommit queue, and the next allocation of the same
    // page count on that CPU and node takes it back. That cycle then skips the
    // node arena's lock, the free map, the queue, mprotect, madvise and the
    // zero-fill fault on every page — at 128 threads over 4 nodes the node
    // arena's lock alone was 26.6% of all CPU. Design and trade-offs:
    // agents/design-run-cache.md.
    //
    // A cached run's pages stay resident and EMPTY (the compressor ignores
    // EMPTY pages), so this is RSS on loan: bounded per shard by
    // SMASH_RUN_CACHE_MIB and kRunCacheSlots, and returned by the decommit
    // thread once older than SMASH_RUN_CACHE_AGE_MS. Off by default. Only a run
    // the compressor's release reported reusable is offered (no page was
    // COMPRESSED, so none can be PROT_NONE or unbacked); a run is only ever
    // handed back to the same node, so NUMA binding is unchanged.
public:
    bool runCacheEnabled() const {
        // Sub-arenas' dirty bins replace it (and reuse any size).
        return runCacheCapBytes() != 0 && !tracking_mode_ && !landminesEnabled() &&
               !zeroFreeEnabled() && !subarenasEnabled();
    }

    // Park a freed run (its compressed state already released). False when the
    // run is not cacheable; the caller then releases it the ordinary way.
    // `reusable` is the compressor release's verdict; a run it declined is
    // counted (see runCacheStats) and not cached.
    bool cacheFreedRun(void* addr, size_t num_pages, bool reusable) {
        if (!runCacheEnabled()) return false;
        const auto a = reinterpret_cast<uintptr_t>(addr);
        const size_t bytes = num_pages * kPageSize;
        const size_t cap = runCacheCapBytes();
        if (!inContigArena(a)) return false;
        RunCacheShard& sh = run_cache_[currentCpuShard(kRunCacheShards)];
        if (!reusable) { bump(sh.declined_cold); return false; }
        if (bytes > cap || bytes > kRunCacheMaxRunBytes) { bump(sh.declined_big); return false; }
        ensureDecommitThreadStarted(shared_queue_);   // the trim runs there
        const int node = nodeOfPage(pageIndex(a));
        CachedRun evicted[kRunCacheSlots];
        unsigned n_evicted = 0;
        {
            LockGuard guard(sh.lock);
            // Make room, oldest first. Terminates: bytes <= cap was checked.
            while (sh.count == kRunCacheSlots || sh.bytes + bytes > cap) {
                unsigned oldest = 0;
                for (unsigned i = 1; i < sh.count; ++i)
                    if (sh.runs[i].freed_ms < sh.runs[oldest].freed_ms) oldest = i;
                evicted[n_evicted++] = sh.runs[oldest];
                sh.bytes -= sh.runs[oldest].pages * kPageSize;
                sh.runs[oldest] = sh.runs[--sh.count];
            }
            sh.runs[sh.count++] = CachedRun{addr, static_cast<uint32_t>(num_pages),
                                            static_cast<int32_t>(node), nowMs()};
            sh.bytes += bytes;
            bump(sh.puts);
            if (n_evicted) sh.evictions.store(sh.evictions.load(std::memory_order_relaxed) +
                                              n_evicted, std::memory_order_relaxed);
            sh.bytes_published.store(sh.bytes, std::memory_order_relaxed);
        }
        for (unsigned i = 0; i < n_evicted; ++i)   // only reusable runs are cached
            releasePages(evicted[i].addr, evicted[i].pages, /*reusable=*/true);
        return true;
    }

    // A cached run of exactly `num_pages` for `node` (-1: the shared arena)
    // whose address is `align_bytes`-aligned, from this CPU's shard; nullptr
    // on a miss.
    void* takeCachedRun(size_t num_pages, int node, size_t align_bytes) {
        if (!runCacheEnabled()) return nullptr;
        RunCacheShard& sh = run_cache_[currentCpuShard(kRunCacheShards)];
        LockGuard guard(sh.lock);
        bump(sh.takes);
        // Newest first: the most recently freed run is the likeliest to be
        // warm in cache.
        for (unsigned i = sh.count; i-- > 0;) {
            const CachedRun& r = sh.runs[i];
            if (r.pages != num_pages || r.node != node ||
                (reinterpret_cast<uintptr_t>(r.addr) & (align_bytes - 1)) != 0)
                continue;
            void* addr = r.addr;
            sh.bytes -= num_pages * kPageSize;
            sh.runs[i] = sh.runs[--sh.count];
            sh.bytes_published.store(sh.bytes, std::memory_order_relaxed);
            bump(sh.hits);
            return addr;
        }
        return nullptr;
    }

    // Return cached runs to the arena: those older than the age limit, or all
    // of them. Skips a shard whose lock is busy; the next sweep takes it.
    void trimRunCache(bool all) {
        if (!runCacheEnabled()) return;
        const uint64_t now = nowMs();
        const uint64_t age = runCacheAgeMs();
        for (RunCacheShard& sh : run_cache_) {
            if (sh.bytes_published.load(std::memory_order_relaxed) == 0) continue;
            CachedRun out[kRunCacheSlots];
            unsigned n = 0;
            if (!sh.lock.tryLock()) continue;
            for (unsigned i = 0; i < sh.count;) {
                if (all || now - sh.runs[i].freed_ms >= age) {
                    out[n++] = sh.runs[i];
                    sh.bytes -= sh.runs[i].pages * kPageSize;
                    sh.runs[i] = sh.runs[--sh.count];
                } else {
                    ++i;
                }
            }
            sh.bytes_published.store(sh.bytes, std::memory_order_relaxed);
            if (n) sh.trimmed.store(sh.trimmed.load(std::memory_order_relaxed) + n,
                                    std::memory_order_relaxed);
            sh.lock.unlock();
            for (unsigned i = 0; i < n; ++i)
                releasePages(out[i].addr, out[i].pages, /*reusable=*/true);
        }
    }

    // Every counter summed over the shards, for the SIGUSR2 stats line.
    // takes: allocations that looked in the cache; hits: of those, served;
    // puts: frees parked; evictions: parked runs pushed out to make room (a
    // put that evicts still ends in releasePages); trimmed: aged out;
    // declined_cold / declined_big: frees not offered because a page was
    // compressed, or the run is over the per-run or per-shard limit.
    struct RunCacheStats {
        uint64_t takes, hits, puts, evictions, trimmed, declined_cold, declined_big, bytes;
    };
    RunCacheStats runCacheStats() const {
        RunCacheStats t{};
        for (const RunCacheShard& sh : run_cache_) {
            t.takes += sh.takes.load(std::memory_order_relaxed);
            t.hits += sh.hits.load(std::memory_order_relaxed);
            t.puts += sh.puts.load(std::memory_order_relaxed);
            t.evictions += sh.evictions.load(std::memory_order_relaxed);
            t.trimmed += sh.trimmed.load(std::memory_order_relaxed);
            t.declined_cold += sh.declined_cold.load(std::memory_order_relaxed);
            t.declined_big += sh.declined_big.load(std::memory_order_relaxed);
            t.bytes += sh.bytes_published.load(std::memory_order_relaxed);
        }
        return t;
    }

    // Diagnostics behind smash_run_cache_info(): total hits and bytes cached
    // now. Returns whether the cache is enabled.
    bool runCacheInfo(uint64_t* hits, uint64_t* bytes) const {
        uint64_t h = 0, b = 0;
        for (const RunCacheShard& sh : run_cache_) {
            h += sh.hits.load(std::memory_order_relaxed);
            b += sh.bytes_published.load(std::memory_order_relaxed);
        }
        *hits = h;
        *bytes = b;
        return runCacheEnabled();
    }

private:
    struct CachedRun {
        void* addr;
        uint32_t pages;
        int32_t node;       // -1: the shared arena
        uint64_t freed_ms;
    };
    static constexpr unsigned kRunCacheShards = 256;
    static constexpr unsigned kRunCacheSlots = 8;
    static constexpr size_t kRunCacheMaxRunBytes = size_t{64} << 20;
    static constexpr uint64_t kRunCacheTrimIntervalMs = 10;
    struct alignas(64) RunCacheShard {
        Spinlock lock;
        unsigned count = 0;
        size_t bytes = 0;
        // Published copies for the lock-free readers (trim's skip test, info).
        std::atomic<size_t> bytes_published{0};
        // Counters (see runCacheStats). Per shard, so they share the shard's
        // line rather than adding a global one; most are bumped under its lock.
        std::atomic<uint64_t> takes{0}, hits{0}, puts{0}, evictions{0}, trimmed{0};
        std::atomic<uint64_t> declined_cold{0}, declined_big{0};
        CachedRun runs[kRunCacheSlots];
    };
    static void bump(std::atomic<uint64_t>& c) {
        c.fetch_add(1, std::memory_order_relaxed);
    }
    RunCacheShard run_cache_[kRunCacheShards];

    static size_t runCacheCapBytes() {
        static const size_t cap = [] {
            const char* v = std::getenv("SMASH_RUN_CACHE_MIB");
            const long mib = v ? std::atol(v) : 0;
            return mib > 0 ? static_cast<size_t>(mib) << 20 : size_t{0};
        }();
        return cap;
    }
    static uint64_t runCacheAgeMs() {
        static const uint64_t ms = [] {
            const char* v = std::getenv("SMASH_RUN_CACHE_AGE_MS");
            const long n = v ? std::atol(v) : 0;
            return n > 0 ? static_cast<uint64_t>(n) : uint64_t{50};
        }();
        return ms;
    }
    static uint64_t nowMs() {
        timespec t;
#if defined(CLOCK_MONOTONIC_COARSE)
        clock_gettime(CLOCK_MONOTONIC_COARSE, &t);   // vDSO, no syscall
#else
        clock_gettime(CLOCK_MONOTONIC, &t);
#endif
        return static_cast<uint64_t>(t.tv_sec) * 1000 + static_cast<uint64_t>(t.tv_nsec) / 1000000;
    }

public:
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
            // Wholly free chunks pooled by the shared sub-arenas are fresh
            // or purged, and nothing else could ever use them now.
            if (donateSharedPool()) {
                if (const size_t idx = free_map_.take(num_pages, kSharedOwner);
                    idx != PageRunMap::kNone)
                    return base_ + idx * kPageSize;
            }
            // Arena exhausted: let the heap return empty slab spans, which
            // then coalesce in the free map, and try once more.
            if (exhausted_hook_) {
                exhausted_hook_(exhausted_ctx_);
                if (const size_t idx = free_map_.take(num_pages, kSharedOwner);
                    idx != PageRunMap::kNone) {
                    return base_ + idx * kPageSize;
                }
            }
            // Then the same thing the node arenas do: a run that has been
            // freed but is still queued for decommit is not gone, it is in
            // flight, and giving up on it hands the caller a direct mapping
            // that is never compressed. Measured with a 1 GiB arena and 2 MiB
            // blocks before this: 1728 of 40000 blocks fell out at 4 threads,
            // 46263 of 160000 at 8, 93726 at 16 and 144241 at 64 — while a
            // single thread, whose decommit thread keeps up, lost none.
            //
            // drainInFlight() covers both a queued run and one another thread
            // is already processing, so this only stops retrying when there is
            // genuinely nothing left to come back.
            //
            // Runs parked in the per-CPU run cache are free memory too: hand
            // them all back first, or a full cache would push this allocation
            // out of the arena.
            trimRunCache(/*all=*/true);
            uint64_t deadline = 0;
            for (int attempt = 0; drainWaitEnabled() && drainWaitMore(attempt, deadline);
                 ++attempt) {
                drainQueueNow(shared_queue_);
                if (const size_t idx = free_map_.take(num_pages, kSharedOwner);
                    idx != PageRunMap::kNone) {
                    return base_ + idx * kPageSize;
                }
                if (!inFlight(shared_queue_)) break;
                sched_yield();
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

    // `reusable`: the release reported no COMPRESSED page (LargeAlloc and
    // cached runs know it); anything else takes the protection restore.
    void releasePages(void* addr, size_t num_pages, bool reusable = false) {
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

        // Two-phase release: queue pages for background decommit on their
        // owner's queue. Its thread will decommit them and THEN add them to
        // that owner's free map. This ensures pages can't be reallocated
        // before decommit completes. The thread starts lazily on the first
        // release (avoids reentrancy during early library initialization,
        // when pthread_create might malloc).
        size_t page_idx = pageIndex(reinterpret_cast<uintptr_t>(addr));
        const int node = nodeOfPage(page_idx);
        ensureDecommitThreadStarted(queueOf(node));
        queueForDecommit(addr, num_pages, page_idx, node, reusable);
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
    size_t arenaFreePages() {
        size_t f = free_map_.freePages(kSharedOwner);
#if defined(__linux__)
        if (NodeArena* s = shared_set_.load(std::memory_order_acquire)) f += subSetFreePages(*s);
#endif
        return f;
    }

    // Diagnostic: pages `node`'s arena has taken from the region, and how many
    // of them are currently free. The difference is exactly what the node's
    // live blocks cost, which an address span cannot tell you once the arena
    // has any history — blocks legitimately scatter across existing free runs.
    // False if this region has no node arenas (none has been used yet).
    // `free_pages` includes the unused tail of the node's newest chunk
    // ([cur, end)), which is free but not in the free map. Without it,
    // carved - free was live blocks PLUS that tail, and a block served from
    // the tail changed neither number — found by smash_sim's end-of-run check
    // (every carved page free again), which failed on every node with a tail.
    bool nodeArenaInfo(uint32_t node, size_t* carved, size_t* free_pages) {
        if (node >= vm::kMaxNumaNodes) return false;
        NodeArena* arenas = node_arenas_.load(std::memory_order_acquire);
        if (!arenas) return false;
        size_t tail = 0;
        {
            LockGuard guard(arenas[node].lock);
            *carved = arenas[node].carved;
            tail = arenas[node].end - arenas[node].cur;
            // Read under the same lock: a concurrent tail allocation moves a
            // run from the tail to a live block, never into the free map.
            *free_pages = (arenas[node].map_ready.load(std::memory_order_acquire)
                               ? arenas[node].map.freePages(ownerOfNode(static_cast<int>(node)))
                               : 0) + tail;
        }
#if defined(__linux__)
        // Sub-arenas: their dirty and clean runs, their unused tails, and the
        // node's pool of wholly free chunks are all free.
        *free_pages += subSetFreePages(arenas[node]);
#endif
        return true;
    }
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
    // Allocate `num_pages` from `node`'s arena: its own free runs first, then
    // the unused tail of its newest chunk, then a newly carved and bound
    // chunk. `align_pages` (a power of two) constrains the returned ADDRESS,
    // not the page index: the arena base is an mmap result and only page
    // aligned, so the base's own residue is folded in as `phase`.
    // Returns nullptr with errno EINVAL (bad node, from mbind), EPERM/ENOSYS
    // (mbind refused), or ENOMEM (region exhausted).
    void* allocatePagesOnNode(size_t num_pages, uint32_t node) {
        return allocatePagesOnNodeImpl(num_pages, node, 1);
    }

    // Aligned variant: the returned address is a multiple of `alignment`, a
    // power of two. Nothing extra is allocated for it — the free-run search is
    // alignment-aware and the chunk cursor is rounded up, with only the pages
    // actually skipped donated to this node's free runs.
    void* allocatePagesOnNodeAligned(size_t num_pages, uint32_t node, size_t alignment) {
        const size_t align_pages = alignment > kPageSize ? alignment / kPageSize : 1;
        return allocatePagesOnNodeImpl(num_pages, node, align_pages);
    }

private:
    // Pages from index `idx` to the next index whose address is aligned.
    size_t alignGap(size_t idx, size_t align_pages, size_t phase) const {
        if (align_pages <= 1) return 0;
        const size_t m = (idx + phase) & (align_pages - 1);
        return m ? align_pages - m : 0;
    }

    // The arena base's own offset within an alignment window, in pages.
    size_t basePhase(size_t align_pages) const {
        if (align_pages <= 1) return 0;
        return (reinterpret_cast<uintptr_t>(base_) / kPageSize) & (align_pages - 1);
    }

    void* allocatePagesOnNodeImpl(size_t num_pages, uint32_t node, size_t align_pages) {
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
        const uint32_t owner = ownerOfNode(static_cast<int>(node));
        const size_t phase = basePhase(align_pages);
        if (!ensureNodeMap(na)) {
            errno = ENOMEM;
            return nullptr;
        }
        if (subarenasEnabled()) return allocateOnSubArena(na, node, num_pages, align_pages, phase);
        PageRunMap& node_map = na.map;

        // Serve from the unused tail of this node's newest chunk, rounding the
        // cursor up to the alignment and donating only the pages skipped — so
        // an aligned block costs the alignment gap once, not on every block.
        // Caller must hold na.lock.
        auto fromTail = [&]() -> void* {
            const size_t start = na.cur + alignGap(na.cur, align_pages, phase);
            if (start < na.cur || start + num_pages > na.end) return nullptr;
            if (start > na.cur) node_map.insert(na.cur, start - na.cur, owner);
            na.cur = start + num_pages;
            return base_ + start * kPageSize;
        };

        {
            LockGuard guard(na.lock);
            if (const size_t idx = node_map.takeAligned(num_pages, owner, align_pages, phase);
                idx != PageRunMap::kNone)
                return base_ + idx * kPageSize;
            if (void* p = fromTail()) return p;
        }

        // Both missed, so the next step would claim a whole kNodeChunkPages
        // chunk (64 MiB) of fresh region for this one request. Before that:
        // this node's freed pages may simply be in flight. free() queues a run
        // and it joins the free map only when that entry is processed, so a
        // miss does not mean the pages are gone. Claiming fresh region instead
        // is what made a tight allocate/free loop walk the whole arena —
        // measured, 64 KiB blocks with a 1 GiB arena consumed all 1023.9 MiB
        // of it and then served 4.5% of allocations from uncompressible direct
        // mappings, while the same loop with 50 us of slack between iterations
        // reused a 0.8 MiB window and never fell back.
        //
        // Deliberately NOT under na.lock. The drain does an mprotect and an
        // madvise per queued run, and holding the node's lock across that
        // serialises every other thread allocating on this node behind the
        // syscalls: measured at 16 threads over 4 nodes, 15.0 us per
        // allocate+free against 1.9 us with the lock released here. Nothing in
        // the drain touches a NodeArena — processDecommitEntry only takes the
        // free map's own lock — and the state it publishes is exactly what the
        // retry below reads, so dropping the lock costs only a re-check.
        drainQueueNow(na.queue);

        {
        LockGuard guard(na.lock);
        if (const size_t idx = node_map.takeAligned(num_pages, owner, align_pages, phase);
            idx != PageRunMap::kNone)
            return base_ + idx * kPageSize;
        if (void* p = fromTail()) return p;

        // A chunk has to cover the request plus the worst-case alignment gap
        // inside it, since the grid is in page indices and the alignment is on
        // addresses.
        const size_t need = num_pages + (align_pages > 1 ? align_pages - 1 : 0);
        const size_t chunk_pages =
            (need + kNodeChunkPages - 1) / kNodeChunkPages * kNodeChunkPages;
        const size_t start = carveNodeChunk(chunk_pages);
        if (start == kNoChunk) goto exhausted;   // handled below, outside na.lock
        void* chunk = base_ + start * kPageSize;
        // Freshly carved, so nothing in the chunk has been touched: no pages
        // to move.
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
            node_map.insert(na.cur, na.end - na.cur, owner);
        na.cur = start;
        na.end = start + chunk_pages;
        na.carved += chunk_pages;
        if (void* p = fromTail()) return p;
        errno = ENOMEM;   // chunk_pages covers `need`, so this is unreachable
        return nullptr;
        }

exhausted:
        // The region has no fresh chunk left, so this node's own freed pages
        // are the only source there is — and a run that is merely in flight
        // must not be mistaken for one that is gone.
        //
        // drainDecommitQueueNow() takes the WHOLE queue with one exchange, so
        // exactly one thread gets it and every other concurrent caller sees an
        // empty queue. Giving up on that is what let 20 of 320000 blocks fall
        // out of the arena on a 16-thread aarch64 run and, reproduced here by
        // oversubscribing, 203 of 320000 at 64 threads and 6474 at 128. So
        // keep retrying while any run is queued or another thread is draining
        // one, and only conclude "nothing left" when both are false.
        //
        // Not under na.lock: the wait would otherwise block every other thread
        // on this node behind it, and the drain itself does syscalls.
        //
        // The shared arena's exhausted hook is deliberately not called here:
        // it returns empty slab spans, which are shared-owner pages, and runs
        // of different owners never merge. It cannot produce a chunk for this
        // node. Cached runs can (some are this node's), so they go back first.
        trimRunCache(/*all=*/true);
        uint64_t deadline = 0;
        for (int attempt = 0; drainWaitEnabled() && drainWaitMore(attempt, deadline);
             ++attempt) {
            // Drained WITHOUT na.lock, as in the common path: holding it
            // across the drain's mprotect/madvise serialises every other
            // thread on this node and was measured at 18.5 s against 2.8 s for
            // a 128-thread churn, while leaving the residual unchanged — so
            // the fast path taking a recovered run before this one re-acquires
            // is not what limits it either.
            drainQueueNow(na.queue);
            {
                LockGuard guard(na.lock);
                if (const size_t idx =
                        node_map.takeAligned(num_pages, owner, align_pages, phase);
                    idx != PageRunMap::kNone)
                    return base_ + idx * kPageSize;
            }
            if (!inFlight(na.queue)) break;   // genuinely nothing to wait for
            sched_yield();
        }
        errno = ENOMEM;
        return nullptr;
    }

public:
#endif

private:
    // ── Sub-arenas (SMASH_SUBARENAS; agents/design-subarenas.md) ─────────────
    // A node arena split into S sub-arenas (default one per CPU of the node),
    // each owning whole 64 MiB chunks of the node with its own lock, its own
    // dirty and clean free-run bins in the node's map, and its own chunk tail.
    // A free of a block that was never compressed goes straight into its
    // chunk's sub-arena as dirty — still resident, no syscall — and any later
    // allocation that fits is carved from it. The node's decommit thread
    // purges dirty runs older than SMASH_DIRTY_DECAY_MS, or beyond
    // SMASH_DIRTY_MAX_MIB per sub-arena, with the lock dropped around madvise.
    //
    // What makes the shared map safe without its own lock: runs never
    // coalesce across a chunk boundary (insertUnlocked's `boundary`), and a
    // chunk belongs to one sub-arena, so no sub-arena ever reads or writes a
    // tag outside chunks it owns. Blocks larger than a chunk take dedicated
    // chunks and never enter the map; their chunks go back to the node's pool.
    struct NodeArena;   // below; the sub-arena functions take it by reference
    struct alignas(64) SubArena {
        Spinlock lock;
        size_t cur = 0;              // unused tail of its newest chunk, [cur, end)
        size_t end = 0;
        size_t dirty_pages = 0;      // under lock
        std::atomic<size_t> dirty_published{0};   // for the purger's skip test and info
        std::atomic<uint64_t> purged_runs{0};
        std::atomic<uint32_t> purging{0};   // purges with runs out of the map
        std::atomic<size_t> peak_dirty_pages{0};   // most ever dirty at once (info)
        std::atomic<uint64_t> inline_purges{0};    // purges run by a freeing thread
    };
    static constexpr uint8_t kChunkUnowned = 0;   // chunk_sub_: none / node-level
    static constexpr uint8_t kChunkBig = 0xFF;    // a block larger than a chunk
    static constexpr uint32_t kMaxSubArenas = 64;
    static constexpr uint64_t kPurgeEpochMs = 50;   // default of purgeEpochMs()
    std::atomic<std::atomic<uint8_t>*> chunk_sub_{nullptr};   // per chunk: sub + 1, or the above
#if defined(__linux__)
    int16_t cpu_node_[CPU_SETSIZE];
    int16_t cpu_rank_[CPU_SETSIZE];
    uint16_t node_ncpus_[vm::kMaxNumaNodes];
    // The shared set's slice of sub-arenas per node (sharedSubLayout).
    uint8_t shared_node_base_[vm::kMaxNumaNodes];
    uint8_t shared_node_count_[vm::kMaxNumaNodes];
    std::atomic<bool> cpu_tables_ready_{false};

    static bool subarenasEnabled() {
        static const bool on = [] {
            const char* v = std::getenv("SMASH_SUBARENAS");
            return v && v[0] == '1';
        }();
        return on;
    }
    static uint32_t subarenasPerNodeEnv() {
        static const uint32_t n = [] {
            const char* v = std::getenv("SMASH_SUBARENAS_PER_NODE");
            const long x = v ? std::atol(v) : 0;
            return x > 0 ? static_cast<uint32_t>(x < kMaxSubArenas ? x : kMaxSubArenas) : 0u;
        }();
        return n;
    }
    // How often the purger sweeps for aged runs (SMASH_PURGE_EPOCH_MS, default
    // 50). A free over the cap does not wait for it. Tests lengthen it so a
    // purge that did wait would be visible.
    static uint64_t purgeEpochMs() {
        static const uint64_t ms = [] {
            const char* v = std::getenv("SMASH_PURGE_EPOCH_MS");
            const long long x = v ? std::atoll(v) : 0;
            return x > 0 ? static_cast<uint64_t>(x) : kPurgeEpochMs;
        }();
        return ms;
    }
    static uint64_t dirtyDecayMs() {
        static const uint64_t ms = [] {
            const char* v = std::getenv("SMASH_DIRTY_DECAY_MS");
            return v ? static_cast<uint64_t>(std::strtoull(v, nullptr, 10)) : uint64_t{1000};
        }();
        return ms;
    }
    static size_t dirtyMaxPages() {
        static const size_t pages = [] {
            const char* v = std::getenv("SMASH_DIRTY_MAX_MIB");
            const long mib = v ? std::atol(v) : 256;
            return (static_cast<size_t>(mib > 0 ? mib : 0) << 20) / kPageSize;
        }();
        return pages;
    }
    static_assert(2 * kMaxSubArenas <= PageRunMap::kSubOwners);
    static uint32_t ownerDirty(uint32_t s) { return PageRunMap::kFirstSubOwner + 2 * s; }
    static uint32_t ownerClean(uint32_t s) { return PageRunMap::kFirstSubOwner + 2 * s + 1; }
    static uint32_t stampNow() { return static_cast<uint32_t>(nowMs()); }
    static size_t chunkOfPage(size_t idx) { return idx / kNodeChunkPages; }

    // CPU -> (node, rank within the node's CPU list), read once from sysfs
    // with raw syscalls (no allocation).
    void ensureCpuTables() {
        if (cpu_tables_ready_.load(std::memory_order_acquire)) [[likely]] return;
        LockGuard guard(numa_init_lock_);
        if (cpu_tables_ready_.load(std::memory_order_relaxed)) return;
        for (int c = 0; c < CPU_SETSIZE; ++c) { cpu_node_[c] = -1; cpu_rank_[c] = 0; }
        for (uint32_t n = 0; n < vm::kMaxNumaNodes; ++n) node_ncpus_[n] = 0;
        uint64_t nodes[vm::kMaxNumaNodes / 64];
        if (vm::memoryNodes(nodes) > 0) {
            for (uint32_t n = 0; n < vm::kMaxNumaNodes; ++n) {
                if (!((nodes[n / 64] >> (n % 64)) & 1)) continue;
                char path[64];
                smash::safe_snprintf(path, sizeof(path),
                                     "/sys/devices/system/node/node%u/cpulist", n);
                uint64_t bits[CPU_SETSIZE / 64] = {};
                if (!vm::readSysfsIdList(path, bits, CPU_SETSIZE)) continue;
                uint16_t rank = 0;
                for (int c = 0; c < CPU_SETSIZE; ++c) {
                    if (!((bits[c / 64] >> (c % 64)) & 1)) continue;
                    cpu_node_[c] = static_cast<int16_t>(n);
                    cpu_rank_[c] = static_cast<int16_t>(rank++);
                }
                node_ncpus_[n] = rank;
            }
        }
        cpu_tables_ready_.store(true, std::memory_order_release);
    }

    // The shared set (plain malloc's large blocks): one slice of sub-arenas
    // per node with CPUs, SMASH_SUBARENAS_PER_NODE (default: the node's CPUs)
    // each, scaled down to kMaxSubArenas in total. Fills shared_node_base_ /
    // shared_node_count_ and returns the total. Under numa_init_lock_.
    uint32_t sharedSubLayout() {
        uint32_t want[vm::kMaxNumaNodes];
        uint64_t total = 0;
        for (uint32_t n = 0; n < vm::kMaxNumaNodes; ++n) {
            want[n] = node_ncpus_[n] ? (subarenasPerNodeEnv() ? subarenasPerNodeEnv()
                                                             : node_ncpus_[n])
                                     : 0;
            total += want[n];
        }
        if (total == 0) {   // no NUMA topology: one per online CPU
            const long c = sysconf(_SC_NPROCESSORS_ONLN);
            return c > 0 ? static_cast<uint32_t>(c < kMaxSubArenas ? c : kMaxSubArenas) : 1;
        }
        uint32_t base = 0;
        for (uint32_t n = 0; n < vm::kMaxNumaNodes; ++n) {
            if (!want[n]) { shared_node_count_[n] = 0; continue; }
            uint64_t k = total > kMaxSubArenas ? want[n] * kMaxSubArenas / total : want[n];
            if (k == 0) k = 1;
            if (base + k > kMaxSubArenas) k = base < kMaxSubArenas ? kMaxSubArenas - base : 0;
            shared_node_base_[n] = static_cast<uint8_t>(base < kMaxSubArenas ? base : 0);
            shared_node_count_[n] = static_cast<uint8_t>(k ? k : 1);
            base += static_cast<uint32_t>(k);
        }
        return base ? (base < kMaxSubArenas ? base : kMaxSubArenas) : 1;
    }

    // The queue a sub-arena set's compressed frees go through (and whose
    // thread purges it): the node's own, or the shared arena's.
    DecommitQueue& subQueue(NodeArena& na) {
        return &na == shared_set_.load(std::memory_order_relaxed) ? shared_queue_ : na.queue;
    }
    NodeArena* ensureSharedSet() {
        if (NodeArena* s = shared_set_.load(std::memory_order_acquire)) [[likely]] return s;
        LockGuard guard(numa_init_lock_);
        if (NodeArena* s = shared_set_.load(std::memory_order_relaxed)) return s;
        NodeArena* s = bootstrapArray<NodeArena>(1);
        if (!s || !s->map.init(contig_pages_)) return nullptr;
        s->map_ready.store(true, std::memory_order_relaxed);
        shared_set_.store(s, std::memory_order_release);
        return s;
    }
    // Every sub-arena set that exists: fn(na, node), node -1 for the shared one.
    template <typename Fn> void forEachSubSet(Fn&& fn) {
        if (NodeArena* arenas = node_arenas_.load(std::memory_order_acquire))
            for (uint32_t n = 0; n < vm::kMaxNumaNodes; ++n)
                if (arenas[n].subs_ready.load(std::memory_order_acquire))
                    fn(arenas[n], static_cast<int>(n));
        if (NodeArena* s = shared_set_.load(std::memory_order_acquire))
            if (s->subs_ready.load(std::memory_order_acquire)) fn(*s, -1);
    }
    // Move the shared set's pooled chunks to the shared free map, for the
    // non-sub-arena users of the region (slab spans, the fallback) once it
    // is exhausted. A pooled chunk has no runs in the set's map and no owner.
    //
    // First make every chunk that is wholly free poolable: retire each
    // sub-arena's unused tail to its clean runs, so a tail chunk with nothing
    // live is pooled too, and purge the dirty runs of wholly free chunks
    // (their chunks become clean).
    // Without that, 8 threads' sub-arenas held up to 512 MiB of a 1 GiB
    // arena in tails that slab spans could never use.
    bool donateSharedPool() {
        NodeArena* s = shared_set_.load(std::memory_order_acquire);
        if (!s || !s->subs_ready.load(std::memory_order_acquire)) return false;
        for (uint32_t i = 0; i < s->nsubs; ++i) {
            SubArena& sa = s->subs[i];
            LockGuard guard(sa.lock);
            if (sa.cur >= sa.end) continue;
            const size_t idx = sa.cur;
            s->map.insertUnlocked(sa.cur, sa.end - sa.cur, ownerClean(i), kNodeChunkPages, 0);
            sa.cur = sa.end;
            maybeReturnChunkLocked(*s, i, idx);
        }
        // Only the dirty runs of chunks that are then wholly free: a purge
        // elsewhere frees nothing the shared free map can use, and purging
        // every dirty run on each miss made every later block fault its pages
        // in again (smash_sim's 8 threads at 75 % of a 1 GiB region: 1.95M
        // runs purged in 6 s). The tails go first, so a chunk free but for its
        // tail counts as wholly free.
        for (uint32_t i = 0; i < s->nsubs; ++i)
            purgeSubArena(*s, i, /*all=*/true, /*whole_chunks_only=*/true);
        bool any = false;
        for (;;) {
            size_t c;
            {
                LockGuard guard(s->chunk_lock);
                if (!s->nfree_chunks) break;
                c = s->free_chunks[--s->nfree_chunks];
            }
            donateToShared(c * kNodeChunkPages, kNodeChunkPages);
            any = true;
        }
        return any;
    }

    // A set's free pages: dirty and clean runs, unused tails, pooled chunks.
    size_t subSetFreePages(NodeArena& na) {
        if (!na.subs_ready.load(std::memory_order_acquire)) return 0;
        size_t f = 0;
        for (uint32_t i = 0; i < na.nsubs; ++i) {
            LockGuard guard(na.subs[i].lock);
            f += na.map.freePages(ownerDirty(i)) + na.map.freePages(ownerClean(i)) +
                 (na.subs[i].end - na.subs[i].cur);
        }
        LockGuard guard(na.chunk_lock);
        return f + na.nfree_chunks * kNodeChunkPages;
    }
    // The sub-arena set owning page `idx` (its chunk is sub-owned).
    NodeArena* subSetOfPage(size_t idx) {
        const int node = nodeOfPage(idx);
        if (node >= 0) return &node_arenas_.load(std::memory_order_acquire)[node];
        return shared_set_.load(std::memory_order_acquire);
    }

    // Before the node's first sub-arena allocation.
    bool ensureSubArenas(NodeArena& na, int node) {
        if (na.subs_ready.load(std::memory_order_acquire)) [[likely]] return true;
        ensureCpuTables();
        LockGuard guard(numa_init_lock_);
        if (na.subs_ready.load(std::memory_order_relaxed)) return true;
        if (!chunk_sub_.load(std::memory_order_relaxed)) {
            auto* t = bootstrapArray<std::atomic<uint8_t>>(
                (contig_pages_ + kNodeChunkPages - 1) / kNodeChunkPages);
            if (!t) return false;
            chunk_sub_.store(t, std::memory_order_release);
        }
        uint32_t n = node < 0 ? sharedSubLayout() : subarenasPerNodeEnv();
        if (n == 0) n = node >= 0 && node_ncpus_[node] ? node_ncpus_[node] : 1;
        if (n > kMaxSubArenas) n = kMaxSubArenas;
        auto* subs = bootstrapArray<SubArena>(n);
        auto* pool = bootstrapArray<uint32_t>(
            (contig_pages_ + kNodeChunkPages - 1) / kNodeChunkPages);
        if (!subs || !pool) return false;
        na.subs = subs;
        na.nsubs = n;
        na.free_chunks = pool;
        na.subs_ready.store(true, std::memory_order_release);
        return true;
    }

    // This thread's sub-arena of `node`: a CPU of the node maps by its rank,
    // any other CPU (an unpinned thread allocating remotely) by its number.
    uint32_t subIndex(int node, uint32_t nsubs) const {
        const int cpu = sched_getcpu();
        if (cpu < 0 || cpu >= CPU_SETSIZE) return 0;
        if (node < 0) {
            // Shared set: a slice per node, so CPUs of one node share
            // sub-arenas (and the pages they first touched) with each other.
            const int cn = cpu_node_[cpu];
            if (cn >= 0 && shared_node_count_[cn])
                return (shared_node_base_[cn] +
                        static_cast<uint32_t>(cpu_rank_[cpu]) % shared_node_count_[cn]) % nsubs;
            return static_cast<uint32_t>(cpu) % nsubs;
        }
        if (cpu_node_[cpu] == static_cast<int16_t>(node))
            return static_cast<uint32_t>(cpu_rank_[cpu]) % nsubs;
        return static_cast<uint32_t>(cpu) % nsubs;
    }

    // Carve `pages` (a multiple of the chunk size) of fresh region, bind it to
    // `node` and record it in node_of_chunk_. kNoChunk if the region is
    // exhausted or the binding failed (errno says which).
    size_t carveBoundChunks(int node, size_t pages) {
        const size_t start = carveNodeChunk(pages);
        if (start == kNoChunk) { errno = ENOMEM; return kNoChunk; }
        if (node < 0) return start;   // the shared set: no binding, no node
        void* chunk = base_ + start * kPageSize;
        if (!vm::bindPagesToNode(chunk, pages * kPageSize, static_cast<uint32_t>(node))) {
            const int err = errno;
            vm::unbindPages(chunk, pages * kPageSize);
            donateToShared(start, pages);
            errno = err;
            return kNoChunk;
        }
        auto* table = node_of_chunk_.load(std::memory_order_acquire);
        for (size_t c = start / kNodeChunkPages; c < (start + pages) / kNodeChunkPages; ++c)
            table[c].store(static_cast<uint16_t>(node + 1), std::memory_order_release);
        return start;
    }

    // A chunk for sub-arena `s`: from the node's pool of wholly free chunks,
    // else freshly carved. Page index of its start, or kNoChunk.
    size_t chunkForSubArena(NodeArena& na, int node, uint32_t s) {
        size_t start = kNoChunk;
        {
            LockGuard guard(na.chunk_lock);
            if (na.nfree_chunks)
                start = static_cast<size_t>(na.free_chunks[--na.nfree_chunks]) * kNodeChunkPages;
        }
        if (start == kNoChunk) {
            LockGuard guard(na.chunk_lock);   // one carver at a time per node
            // Re-check: a chunk may have come back while we waited.
            if (na.nfree_chunks) {
                start = static_cast<size_t>(na.free_chunks[--na.nfree_chunks]) * kNodeChunkPages;
            } else {
                start = carveBoundChunks(node, kNodeChunkPages);
                if (start == kNoChunk) return kNoChunk;
                LockGuard carved_guard(na.lock);
                na.carved += kNodeChunkPages;
            }
        }
        chunk_sub_.load(std::memory_order_acquire)[chunkOfPage(start)]
            .store(static_cast<uint8_t>(s + 1), std::memory_order_release);
        return start;
    }

    // Under sa.lock: dirty runs, then clean runs, then (own sub-arena only)
    // the chunk tail. A dirty run sets `*took_dirty`: its pages hold what the
    // last block left there, every other source's read as zero.
    void* takeFromSubLocked(NodeArena& na, uint32_t s, size_t num_pages, size_t align_pages,
                            size_t phase, bool tail, bool* took_dirty = nullptr) {
        SubArena& sa = na.subs[s];
        size_t idx = na.map.takeAlignedUnlocked(num_pages, ownerDirty(s), align_pages, phase);
        if (idx != PageRunMap::kNone) {
            sa.dirty_pages -= num_pages;
            sa.dirty_published.store(sa.dirty_pages, std::memory_order_relaxed);
            if (took_dirty) *took_dirty = true;
            return base_ + idx * kPageSize;
        }
        idx = na.map.takeAlignedUnlocked(num_pages, ownerClean(s), align_pages, phase);
        if (idx != PageRunMap::kNone) return base_ + idx * kPageSize;
        if (!tail) return nullptr;
        const size_t start = sa.cur + alignGap(sa.cur, align_pages, phase);
        if (start < sa.cur || start + num_pages > sa.end) return nullptr;
        if (start > sa.cur)
            na.map.insertUnlocked(sa.cur, start - sa.cur, ownerClean(s), kNodeChunkPages, 0);
        sa.cur = start + num_pages;
        return base_ + start * kPageSize;
    }

    // Under sa.lock: where the span of sub-arena `s`'s free runs, dirty and
    // clean, that starts at `start` ends; `start` if no run of `s` starts
    // there. Runs never cross a chunk boundary, and neither does the span.
    size_t mixedSpanEnd(NodeArena& na, uint32_t s, size_t start) const {
        size_t e = start, len;
        uint32_t o, stamp;
        do {
            if (!na.map.runAtUnlocked(e, &len, &o, &stamp) ||
                (o != ownerDirty(s) && o != ownerClean(s)))
                break;
            e += len;
        } while (e % kNodeChunkPages);
        return e;
    }

    // Under sa.lock, once the region has no fresh chunk left: `num_pages`
    // from a dirty run and the clean runs next to it, which never merge (a
    // clean run's pages are purged, a dirty run's are not). A full region's
    // free space ends up split that way, frees landing dirty next to purged
    // runs, and a request neither kind of run holds alone used to leave the
    // sub-arenas — where each one purged every dirty run (donateSharedPool),
    // so the next blocks were all faulted in again: 1-3 MiB churn at 70-75 %
    // of a 1 GiB region fell from 200k to 10k blocks a second for good.
    // The block's dirty pages leave the dirty count (and set `*took_dirty`),
    // and what the request does not use of the runs goes back as it was.
    void* takeMixedFromSubLocked(NodeArena& na, uint32_t s, size_t num_pages,
                                 size_t align_pages, size_t phase, bool* took_dirty = nullptr) {
        SubArena& sa = na.subs[s];
        const uint32_t dirty = ownerDirty(s), clean = ownerClean(s);
        size_t span = PageRunMap::kNone, at = 0;
        auto consider = [&](size_t start, size_t, uint32_t) {
            if (span != PageRunMap::kNone) return;
            if (start % kNodeChunkPages) {   // a span is walked from its first run only
                const size_t l = na.map.runBeforeUnlocked(start);
                size_t len;
                uint32_t o, stamp;
                if (l != PageRunMap::kNone && na.map.runAtUnlocked(l, &len, &o, &stamp) &&
                    (o == dirty || o == clean))
                    return;
            }
            const size_t b = start + alignGap(start, align_pages, phase);
            if (b >= start && b + num_pages <= mixedSpanEnd(na, s, start)) { span = start; at = b; }
        };
        na.map.forEachRunUnlocked(dirty, consider);
        na.map.forEachRunUnlocked(clean, consider);
        if (span == PageRunMap::kNone) return nullptr;
        const size_t blk_end = at + num_pages;
        for (size_t e = span; e < blk_end;) {
            size_t len;
            uint32_t o, stamp;
            if (!na.map.runAtUnlocked(e, &len, &o, &stamp)) break;   // walked above: cannot happen
            const size_t next = e + len;
            if (next > at) {
                na.map.removeRunUnlocked(e, len, o);
                const size_t lo = e > at ? e : at, hi = next < blk_end ? next : blk_end;
                if (o == dirty) {
                    sa.dirty_pages -= hi - lo;
                    if (took_dirty) *took_dirty = true;
                }
                if (e < at) na.map.insertUnlocked(e, at - e, o, kNodeChunkPages, stamp);
                if (next > blk_end)
                    na.map.insertUnlocked(blk_end, next - blk_end, o, kNodeChunkPages, stamp);
            }
            e = next;
        }
        sa.dirty_published.store(sa.dirty_pages, std::memory_order_relaxed);
        return base_ + at * kPageSize;
    }

    void* allocateOnSubArena(NodeArena& na, int node, size_t num_pages,
                             size_t align_pages, size_t phase, bool* took_dirty = nullptr) {
        if (!ensureSubArenas(na, node)) { errno = ENOMEM; return nullptr; }
        const size_t need = num_pages + (align_pages > 1 ? align_pages - 1 : 0);
        if (need > kNodeChunkPages) return allocateBigOnSubArenas(na, node, num_pages, align_pages, phase);
        const uint32_t n = na.nsubs;
        const uint32_t me = subIndex(node, n);
        // Own sub-arena: dirty, clean, tail — then the tail folded into the
        // clean runs, where it merges with a free neighbour (a request can fit
        // the two together and neither alone), and if that still misses, the
        // chunk pooled in case the fold made it wholly free.
        {
            SubArena& sa = na.subs[me];
            LockGuard guard(sa.lock);
            if (void* p = takeFromSubLocked(na, me, num_pages, align_pages, phase, true, took_dirty))
                return p;
            if (sa.cur < sa.end) {
                const size_t idx = sa.cur;
                na.map.insertUnlocked(sa.cur, sa.end - sa.cur, ownerClean(me), kNodeChunkPages, 0);
                sa.cur = sa.end;
                if (void* p = takeFromSubLocked(na, me, num_pages, align_pages, phase, false,
                                                took_dirty))
                    return p;
                maybeReturnChunkLocked(na, me, idx);
            }
        }
        // Siblings, trylock only: a busy sibling is skipped, not waited for.
        for (uint32_t i = 1; i < n; ++i) {
            const uint32_t s = (me + i) % n;
            if (!na.subs[s].lock.tryLock()) continue;
            void* p = takeFromSubLocked(na, s, num_pages, align_pages, phase, false, took_dirty);
            na.subs[s].lock.unlock();
            if (p) return p;
        }
        // A new chunk for this sub-arena.
        const size_t start = chunkForSubArena(na, node, me);
        if (start != kNoChunk) {
            SubArena& sa = na.subs[me];
            LockGuard guard(sa.lock);
            // Another thread of this sub-arena may have set a tail since the
            // fold above. Its unused pages are fresh: clean, and if that makes
            // the chunk wholly free it goes to the pool — this request may be
            // served from a dirty run freed meanwhile, and nothing else would
            // ever pool it (found as a whole free chunk stranded in a
            // sub-arena's clean runs, 8 threads on 2 CPUs).
            const size_t old_cur = sa.cur;
            const bool had_tail = sa.end > sa.cur;
            if (had_tail)
                na.map.insertUnlocked(sa.cur, sa.end - sa.cur, ownerClean(me), kNodeChunkPages, 0);
            sa.cur = start;
            sa.end = start + kNodeChunkPages;
            if (had_tail) maybeReturnChunkLocked(na, me, old_cur);
            if (void* p = takeFromSubLocked(na, me, num_pages, align_pages, phase, true, took_dirty))
                return p;
        }
        // The node is out of region: compressed frees may be in flight to
        // clean runs, and siblings are worth waiting for now — their unused
        // tails too (a block there belongs to the sibling's chunk and goes
        // home to it), or free space parked in them fails the allocation.
        // Then a dirty run with the clean runs next to it, before the request
        // gives up on the sub-arenas (takeMixedFromSubLocked).
        uint64_t deadline = 0;
        for (int attempt = 0; drainWaitMore(attempt, deadline); ++attempt) {
            drainQueueNow(subQueue(na));
            for (uint32_t i = 0; i < n; ++i) {
                const uint32_t s = (me + i) % n;
                LockGuard guard(na.subs[s].lock);
                if (void* p = takeFromSubLocked(na, s, num_pages, align_pages, phase, true,
                                                took_dirty))
                    return p;
            }
            for (uint32_t i = 0; i < n; ++i) {
                const uint32_t s = (me + i) % n;
                LockGuard guard(na.subs[s].lock);
                if (void* p = takeMixedFromSubLocked(na, s, num_pages, align_pages, phase,
                                                     took_dirty))
                    return p;
            }
            if (!inFlight(subQueue(na))) break;
            sched_yield();
        }
        errno = ENOMEM;
        return nullptr;
    }

    // A block larger than a chunk: dedicated fresh chunks, marked kChunkBig,
    // never in the map; freeing it returns the chunks to the node's pool.
    void* allocateBigOnSubArenas(NodeArena& na, int node, size_t num_pages,
                                 size_t align_pages, size_t phase) {
        const size_t need = num_pages + (align_pages > 1 ? align_pages - 1 : 0);
        const size_t pages = (need + kNodeChunkPages - 1) / kNodeChunkPages * kNodeChunkPages;
        auto* owners = chunk_sub_.load(std::memory_order_acquire);
        // Consecutive pooled chunks first: a freed big block returns its
        // chunks to the pool, and carving fresh ones instead walked the whole
        // region (64 GiB in 2 s of 70 MiB churn, then everything fell out).
        auto takeOrCarve = [&]() -> size_t {
            size_t st = takePooledRunLocked(na, pages / kNodeChunkPages);
            if (st != kNoChunk) return st;
            st = carveBoundChunks(node, pages);
            if (st == kNoChunk) return kNoChunk;
            LockGuard carved_guard(na.lock);
            na.carved += pages;
            return st;
        };
        auto takePooled = [&]() {
            LockGuard guard(na.chunk_lock);
            return takePooledRunLocked(na, pages / kNodeChunkPages);
        };
        // A big block just freed may still be in the decommit queue on its
        // way back to the pool. Wait for it (as every other allocation path
        // waits for in-flight frees) BEFORE carving: carving instead grew the
        // set by fresh chunks each time a free lagged — a malloc_numa churn
        // took a whole 1 GiB region into node 0's pool and left plain malloc
        // nothing — and once the region was out, blocks fell out of it (1-7
        // of 100 rounds on the 2-CPU claudepc). The queue's processing takes
        // na.chunk_lock, so the wait runs with it released.
        auto waitForInFlight = [&](size_t st) {
            uint64_t deadline = 0;
            for (int attempt = 0; st == kNoChunk && inFlight(subQueue(na)) &&
                                  drainWaitMore(attempt, deadline); ++attempt) {
                drainQueueNow(subQueue(na));
                st = takePooled();
                if (st == kNoChunk) sched_yield();
            }
            return st;
        };
        size_t start = waitForInFlight(takePooled());
        if (start == kNoChunk) {
            LockGuard guard(na.chunk_lock);
            start = takeOrCarve();
        }
        if (start == kNoChunk) start = waitForInFlight(kNoChunk);   // freed while we carved
        if (start == kNoChunk) return nullptr;
        LockGuard guard(na.chunk_lock);
        // The free returns the chunks the block touches; any the alignment
        // gap (or the rounding) leaves untouched go to the pool now.
        const size_t blk = start + alignGap(start, align_pages, phase);
        const size_t first = chunkOfPage(blk), last = chunkOfPage(blk + num_pages - 1);
        for (size_t c = chunkOfPage(start); c < chunkOfPage(start + pages); ++c) {
            if (c >= first && c <= last) {
                owners[c].store(kChunkBig, std::memory_order_release);
            } else {
                owners[c].store(kChunkUnowned, std::memory_order_release);
                na.free_chunks[na.nfree_chunks++] = static_cast<uint32_t>(c);
            }
        }
        return base_ + blk * kPageSize;
    }

    // Under na.chunk_lock: `k` consecutive chunks from the pool, removed from
    // it; page index of the first, or kNoChunk. Sorts the pool (a stack of
    // chunk indices), so the cost is on the big-block path only.
    size_t takePooledRunLocked(NodeArena& na, size_t k) {
        if (k == 0 || na.nfree_chunks < k) return kNoChunk;
        uint32_t* c = na.free_chunks;
        const size_t n = na.nfree_chunks;
        for (size_t i = 1; i < n; ++i)   // insertion sort: no allocation, n is small
            for (size_t j = i; j > 0 && c[j] < c[j - 1]; --j) {
                const uint32_t t = c[j]; c[j] = c[j - 1]; c[j - 1] = t;
            }
        for (size_t i = 0; i + k <= n; ++i) {
            if (c[i + k - 1] - c[i] != k - 1) continue;   // sorted and distinct
            const size_t first = c[i];
            for (size_t j = i + k; j < n; ++j) c[j - k] = c[j];
            na.nfree_chunks = n - k;
            return first * kNodeChunkPages;
        }
        return kNoChunk;
    }

    // Under sa.lock, after inserting a clean run: if the chunk holding
    // `idx` is now one clean run and is not the sub-arena's tail, give it
    // back to the node's pool so no sub-arena strands a free chunk.
    void maybeReturnChunkLocked(NodeArena& na, uint32_t s, size_t idx) {
        SubArena& sa = na.subs[s];
        const size_t c0 = chunkOfPage(idx) * kNodeChunkPages;
        if (sa.cur >= c0 && sa.cur < c0 + kNodeChunkPages && sa.cur < sa.end) return;
        if (!na.map.removeRunUnlocked(c0, kNodeChunkPages, ownerClean(s))) return;
        chunk_sub_.load(std::memory_order_acquire)[chunkOfPage(c0)]
            .store(kChunkUnowned, std::memory_order_release);
        LockGuard guard(na.chunk_lock);
        na.free_chunks[na.nfree_chunks++] = static_cast<uint32_t>(chunkOfPage(c0));
    }

    // Purge one sub-arena's aged (or, with `all`, every) dirty run, and runs
    // beyond the cap oldest first: remove them under the lock, madvise with
    // it dropped, re-insert them clean. `all` blocks on the lock; the
    // periodic sweep only tries it. `whole_chunks_only` limits the purge to
    // runs in chunks that the sub-arena's runs cover wholly, which it then
    // pools.
    void purgeSubArena(NodeArena& na, uint32_t s, bool all, bool whole_chunks_only = false) {
        SubArena& sa = na.subs[s];
        if (sa.dirty_published.load(std::memory_order_relaxed) == 0) return;
        if (all) sa.lock.lock();
        else if (!sa.lock.tryLock()) return;
        purgeSubArenaHoldingLock(na, s, all, whole_chunks_only);
    }

    // The purge proper, with sa.lock held by the caller; releases it. Also
    // run by a freeing thread (freeRunDirty) once its sub-arena is past twice
    // the cap: one purger thread serves every sub-arena of a set, and with no
    // backpressure a burst of frees that nothing reallocated held the whole
    // burst dirty until that thread had madvised it all (8 threads freeing
    // 256 MiB each: 2 GiB over the caps for as long as that took).
    void purgeSubArenaHoldingLock(NodeArena& na, uint32_t s, bool all, bool whole_chunks_only) {
        SubArena& sa = na.subs[s];
        struct Victim { size_t start, len; uint32_t stamp; };
        constexpr size_t kMaxVictims = 256;
        Victim runs[kMaxVictims];
        size_t n = 0;
        const uint32_t now = stampNow();
        const uint64_t decay = dirtyDecayMs();
        const size_t cap = dirtyMaxPages();
        na.map.forEachRunUnlocked(ownerDirty(s), [&](size_t start, size_t len, uint32_t stamp) {
            if (n >= kMaxVictims) return;
            if (whole_chunks_only) {
                const size_t c0 = chunkOfPage(start) * kNodeChunkPages;
                if (mixedSpanEnd(na, s, c0) != c0 + kNodeChunkPages) return;
            }
            runs[n++] = Victim{start, len, stamp};
        });
        // Oldest first.
        std::sort(runs, runs + n, [now](const Victim& a, const Victim& b) {
            return static_cast<uint32_t>(now - a.stamp) > static_cast<uint32_t>(now - b.stamp);
        });
        // Aged runs go whole; beyond the cap, oldest first, and the run that
        // crosses the cap only in part (its end) — freed blocks coalesce, so
        // purging whole runs would overshoot to nothing (16 x 8 MiB freed in
        // order merge into two 64 MiB runs against a 32 MiB cap).
        size_t keep_pages = sa.dirty_pages;
        size_t removed = 0;
        for (size_t i = 0; i < n; ++i) {
            const bool aged = static_cast<uint32_t>(now - runs[i].stamp) >= decay;
            size_t purge = 0;
            if (all || aged) purge = runs[i].len;
            else if (keep_pages > cap) purge = keep_pages - cap < runs[i].len
                                               ? keep_pages - cap : runs[i].len;
            else break;
            if (!na.map.removeRunUnlocked(runs[i].start, runs[i].len, ownerDirty(s))) continue;
            if (purge < runs[i].len)   // keep the head dirty, with its age
                na.map.insertUnlocked(runs[i].start, runs[i].len - purge, ownerDirty(s),
                                      kNodeChunkPages, runs[i].stamp);
            sa.dirty_pages -= purge;
            keep_pages -= purge;
            runs[removed++] = Victim{runs[i].start + runs[i].len - purge, purge, runs[i].stamp};
        }
        sa.dirty_published.store(sa.dirty_pages, std::memory_order_relaxed);
        if (removed) sa.purging.fetch_add(1, std::memory_order_relaxed);
        sa.lock.unlock();
        if (removed == 0) return;
        for (size_t i = 0; i < removed; ++i)
            vm::decommitPages(base_ + runs[i].start * kPageSize, runs[i].len * kPageSize);
        LockGuard guard(sa.lock);
        for (size_t i = 0; i < removed; ++i) {
            na.map.insertUnlocked(runs[i].start, runs[i].len, ownerClean(s), kNodeChunkPages, 0);
            maybeReturnChunkLocked(na, s, runs[i].start);
        }
        sa.purged_runs.fetch_add(removed, std::memory_order_relaxed);
        sa.purging.fetch_sub(1, std::memory_order_release);
    }

    // Is any sub-arena of the set this queue's thread purges over its cap?
    bool overDirtyCap(const DecommitQueue& q) {
        NodeArena* na = nullptr;
        if (q.node >= 0) {
            if (NodeArena* arenas = node_arenas_.load(std::memory_order_acquire)) na = &arenas[q.node];
        } else {
            na = shared_set_.load(std::memory_order_acquire);
        }
        if (!na || !na->subs_ready.load(std::memory_order_acquire)) return false;
        for (uint32_t s = 0; s < na->nsubs; ++s)
            if (na->subs[s].dirty_published.load(std::memory_order_relaxed) > dirtyMaxPages()) return true;
        return false;
    }

    void purgeNodeSubArenas(uint32_t node, bool all) {
        NodeArena* arenas = node_arenas_.load(std::memory_order_acquire);
        if (!arenas || !arenas[node].subs_ready.load(std::memory_order_acquire)) return;
        NodeArena& na = arenas[node];
        for (uint32_t s = 0; s < na.nsubs; ++s) purgeSubArena(na, s, all);
    }
    void purgeSharedSubArenas(bool all) {
        NodeArena* na = shared_set_.load(std::memory_order_acquire);
        if (!na || !na->subs_ready.load(std::memory_order_acquire)) return;
        for (uint32_t s = 0; s < na->nsubs; ++s) purgeSubArena(*na, s, all);
    }

public:
    // A freed, never-compressed run of a sub-arena chunk goes straight back to
    // that chunk's sub-arena as dirty. False when sub-arenas are off or the
    // run is not in one (the caller then releases it the ordinary way).
    bool freeRunDirty(void* addr, size_t num_pages) {
        if (!subarenasEnabled() || tracking_mode_ || landminesEnabled() || zeroFreeEnabled())
            return false;
        const auto a = reinterpret_cast<uintptr_t>(addr);
        if (!inContigArena(a)) return false;
        const size_t idx = pageIndex(a);
        auto* owners = chunk_sub_.load(std::memory_order_acquire);
        if (!owners) return false;
        const uint8_t o = owners[chunkOfPage(idx)].load(std::memory_order_acquire);
        if (o == kChunkUnowned || o == kChunkBig) return false;
        NodeArena* set = subSetOfPage(idx);   // a sub-owned chunk's set exists
        if (!set) return false;
        NodeArena& na = *set;
        const uint32_t s = static_cast<uint32_t>(o - 1);
        SubArena& sa = na.subs[s];
        bool over = false;
        sa.lock.lock();
        na.map.insertUnlocked(idx, num_pages, ownerDirty(s), kNodeChunkPages, stampNow());
        sa.dirty_pages += num_pages;
        sa.dirty_published.store(sa.dirty_pages, std::memory_order_relaxed);
        if (sa.dirty_pages > sa.peak_dirty_pages.load(std::memory_order_relaxed))
            sa.peak_dirty_pages.store(sa.dirty_pages, std::memory_order_relaxed);
        const size_t cap = dirtyMaxPages();
        if (sa.dirty_pages > 2 * cap) {
            // Past twice the cap the purger thread is behind (or this is a
            // burst nothing reallocates): this thread purges its own excess
            // before anyone sees the lock free, so no sub-arena ever holds
            // more than twice its cap plus one block, however many threads
            // free into it. The madvise runs with the lock dropped.
            sa.inline_purges.fetch_add(1, std::memory_order_relaxed);
            purgeSubArenaHoldingLock(na, s, /*all=*/false, /*whole_chunks_only=*/false);
        } else {
            // Over the cap: the purger takes the oldest now, not at its next
            // 50 ms epoch — between epochs a fast churn otherwise held 4-6x
            // the cap dirty (smash_sim, 8 sub-arenas: 7-13 GiB against 2).
            if (sa.dirty_pages > cap &&
                !subQueue(na).purge_now.load(std::memory_order_relaxed)) {
                subQueue(na).purge_now.store(true, std::memory_order_seq_cst);
                over = true;
            }
            sa.lock.unlock();
        }
        // Woken after the lock is released: woken under it, the purger found
        // the lock held (it only tries it) and slept until the next epoch.
        if (over) kick(subQueue(na));
        ensureDecommitThreadStarted(subQueue(na));   // its thread purges this set
        return true;
    }

    // Purge every dirty run now (tools and tests), and return only once no
    // purge (this one or the purger's) has runs out of the free lists.
    void purgeAllDirtyNow() {
        if (!subarenasEnabled()) return;
        forEachSubSet([&](NodeArena& na, int) {
            for (uint32_t s = 0; s < na.nsubs; ++s) purgeSubArena(na, s, true);
        });
        forEachSubSet([&](NodeArena& na, int) {
            for (uint32_t s = 0; s < na.nsubs; ++s)
                while (na.subs[s].purging.load(std::memory_order_acquire)) sched_yield();
        });
    }

    // Large blocks of plain malloc: from the shared set of sub-arenas when
    // they are on, else (or when it cannot serve) the shared arena as before.
    // `*zeroed` says whether every page of the run reads as zero, which lets
    // calloc skip its memset. Every run in the shared free map, a sub-arena's
    // clean runs and tail, and the chunk pool is fresh or was decommitted
    // before it got there (processDecommitEntry, purgeSubArena); a dirty run
    // is neither, and a decommit may not zero (vm::decommitZeroes).
    void* allocateLargePages(size_t num_pages, bool* zeroed = nullptr) {
        bool took_dirty = false;
        void* p = nullptr;
        if (subarenasEnabled() && !tracking_mode_ && base_ && num_pages &&
            num_pages <= contig_pages_) {
            if (NodeArena* na = ensureSharedSet())
                p = allocateOnSubArena(*na, -1, num_pages, 1, 0, &took_dirty);
        }
        if (!p) p = allocatePages(num_pages);
        if (zeroed) *zeroed = p && !took_dirty && vm::decommitZeroes();
        return p;
    }

    // Behind smash_shared_arena_info(): pages the shared arena has taken from
    // the region (everything walked except node chunks) and how many of them
    // are free — its free map, and its sub-arenas' runs, tails and pool.
    void sharedArenaInfo(size_t* used, size_t* free_pages) {
        size_t walked = next_page_.load(std::memory_order_acquire);
        if (walked > contig_pages_) walked = contig_pages_;
        size_t node_carved = 0;
        if (NodeArena* arenas = node_arenas_.load(std::memory_order_acquire))
            for (uint32_t n = 0; n < vm::kMaxNumaNodes; ++n) {
                LockGuard guard(arenas[n].lock);
                node_carved += arenas[n].carved;
            }
        *used = walked - node_carved;
        *free_pages = arenaFreePages();
    }

#if defined(SMASH_SIM)
    // smash_sim: where a set's free pages sit (node -1: the shared set).
    struct SubSetBreakdown { size_t dirty, clean, tails, pooled, carved, clean_runs, whole_clean; };
    SubSetBreakdown subSetBreakdown(int node) {
        SubSetBreakdown b{};
        NodeArena* set = nullptr;
        if (node < 0) set = shared_set_.load(std::memory_order_acquire);
        else if (NodeArena* arenas = node_arenas_.load(std::memory_order_acquire))
            set = &arenas[node];
        if (!set || !set->subs_ready.load(std::memory_order_acquire)) return b;
        NodeArena& na = *set;
        for (uint32_t i = 0; i < na.nsubs; ++i) {
            LockGuard guard(na.subs[i].lock);
            b.dirty += na.map.freePages(ownerDirty(i));
            b.clean += na.map.freePages(ownerClean(i));
            b.tails += na.subs[i].end - na.subs[i].cur;
            na.map.forEachRunUnlocked(ownerClean(i), [&](size_t, size_t len, uint32_t) {
                ++b.clean_runs;
                b.whole_clean += len == kNodeChunkPages;
            });
        }
        {
            LockGuard guard(na.chunk_lock);
            b.pooled = na.nfree_chunks * kNodeChunkPages;
        }
        LockGuard guard(na.lock);
        b.carved = na.carved;
        return b;
    }

    // smash_sim, with no allocations or frees running (the purger may be):
    // every free page of `node`'s sub-arenas (dirty and clean runs, tails,
    // pooled chunks) listed once, in a chunk its sub-arena owns, each
    // owner's counter equal to its runs' sum. Holds every sub-arena lock and
    // the pool lock, so a purge in flight is only absent, never half-moved.
    // Prints what it finds; returns the number of problems.
    // `node` -1 checks the shared set. `quiet`: nothing is allocated and
    // nothing is dirty (after a full purge), so a clean run covering a whole
    // chunk is one that should have been pooled.
    int checkSubArenas(int node, bool quiet = false) {
        NodeArena* set = nullptr;
        if (node < 0) set = shared_set_.load(std::memory_order_acquire);
        else if (NodeArena* arenas = node_arenas_.load(std::memory_order_acquire))
            set = &arenas[node];
        if (!set || !set->subs_ready.load(std::memory_order_acquire)) return 0;
        NodeArena& na = *set;
        for (uint32_t i = 0; i < na.nsubs; ++i) na.subs[i].lock.lock();
        na.chunk_lock.lock();
        struct Unlock {
            NodeArena& na;
            ~Unlock() {
                na.chunk_lock.unlock();
                for (uint32_t i = na.nsubs; i-- > 0;) na.subs[i].lock.unlock();
            }
        } unlock{na};
        struct Iv { size_t start, len; int kind; uint32_t sub; };
        std::vector<Iv> ivs;
        int bad = 0;
        auto* owners = chunk_sub_.load(std::memory_order_acquire);
        for (uint32_t i = 0; i < na.nsubs; ++i) {
            for (int kind = 0; kind < 2; ++kind) {
                const uint32_t o = kind ? ownerClean(i) : ownerDirty(i);
                size_t sum = 0;
                na.map.forEachRunUnlocked(o, [&](size_t st, size_t len, uint32_t) {
                    ivs.push_back({st, len, kind, i});
                    sum += len;
                    if (quiet && kind == 1 && len == kNodeChunkPages) {
                        std::printf("  node %d sub %u: chunk %zu is wholly free but not pooled\n",
                                    node, i, chunkOfPage(st));
                        ++bad;
                    }
                    if (chunkOfPage(st) != chunkOfPage(st + len - 1)) {
                        std::printf("  node %d sub %u: %s run %zu+%zu crosses a chunk\n",
                                    node, i, kind ? "clean" : "dirty", st, len);
                        ++bad;
                    }
                    if (owners[chunkOfPage(st)].load() != i + 1) {
                        std::printf("  node %d sub %u: %s run %zu+%zu in a chunk owned by %u\n",
                                    node, i, kind ? "clean" : "dirty", st, len,
                                    owners[chunkOfPage(st)].load());
                        ++bad;
                    }
                });
                if (sum != na.map.freePages(o)) {
                    std::printf("  node %d sub %u: %s runs sum to %zu, counter says %zu\n", node,
                                i, kind ? "clean" : "dirty", sum, na.map.freePages(o));
                    ++bad;
                }
            }
            if (na.subs[i].end > na.subs[i].cur)
                ivs.push_back({na.subs[i].cur, na.subs[i].end - na.subs[i].cur, 2, i});
        }
        for (size_t k = 0; k < na.nfree_chunks; ++k)
            ivs.push_back({static_cast<size_t>(na.free_chunks[k]) * kNodeChunkPages,
                           kNodeChunkPages, 3, 0});
        if (node < 0) {
            // Pages of the shared free map (the old path, and the fallback)
            // never lie in a chunk a sub-arena owns.
            LockGuard fm_guard(free_map_.lockForCheck());
            free_map_.forEachRunUnlocked(kSharedOwner, [&](size_t st, size_t len, uint32_t) {
                for (size_t c = chunkOfPage(st); c <= chunkOfPage(st + len - 1); ++c) {
                    if (owners[c].load() == kChunkUnowned) continue;
                    std::printf("  shared free map run %zu+%zu lies in chunk %zu, owned by %u\n",
                                st, len, c, owners[c].load());
                    ++bad;
                    break;
                }
            });
        }
        std::sort(ivs.begin(), ivs.end(), [](const Iv& a, const Iv& b) { return a.start < b.start; });
        static const char* const kKind[] = {"dirty", "clean", "tail", "pool"};
        for (size_t k = 1; k < ivs.size(); ++k) {
            if (ivs[k].start < ivs[k - 1].start + ivs[k - 1].len) {
                std::printf("  node %d: %s %zu+%zu (sub %u) overlaps %s %zu+%zu (sub %u)\n", node,
                            kKind[ivs[k].kind], ivs[k].start, ivs[k].len, ivs[k].sub,
                            kKind[ivs[k - 1].kind], ivs[k - 1].start, ivs[k - 1].len,
                            ivs[k - 1].sub);
                ++bad;
            }
        }
        return bad;
    }
#endif

    // Diagnostics behind smash_subarena_info(): dirty bytes held now, and runs
    // purged so far. Returns whether sub-arenas are on.
    bool subArenaInfo(uint64_t* dirty_bytes, uint64_t* purged_runs) {
        uint64_t d = 0, p = 0;
        forEachSubSet([&](NodeArena& na, int) {
            for (uint32_t s = 0; s < na.nsubs; ++s) {
                d += na.subs[s].dirty_published.load(std::memory_order_relaxed) * kPageSize;
                p += na.subs[s].purged_runs.load(std::memory_order_relaxed);
            }
        });
        *dirty_bytes = d;
        *purged_runs = p;
        return subarenasEnabled();
    }
    // The most any one sub-arena has held dirty at once, and purges run by
    // freeing threads (past twice the cap) so far.
    bool subArenaPressure(uint64_t* peak_dirty_bytes, uint64_t* inline_purges) {
        uint64_t peak = 0, inl = 0;
        forEachSubSet([&](NodeArena& na, int) {
            for (uint32_t s = 0; s < na.nsubs; ++s) {
                peak = std::max<uint64_t>(peak, na.subs[s].peak_dirty_pages.load(std::memory_order_relaxed));
                inl += na.subs[s].inline_purges.load(std::memory_order_relaxed);
            }
        });
        *peak_dirty_bytes = peak * kPageSize;
        *inline_purges = inl;
        return subarenasEnabled();
    }
#else
    static bool subarenasEnabled() { return false; }
    void purgeNodeSubArenas(uint32_t, bool) {}
public:
    bool freeRunDirty(void*, size_t) { return false; }
    void purgeAllDirtyNow() {}
    void* allocateLargePages(size_t num_pages, bool* zeroed = nullptr) {
        if (zeroed) *zeroed = false;
        return allocatePages(num_pages);
    }
    bool donateSharedPool() { return false; }
    void sharedArenaInfo(size_t* used, size_t* free_pages) {
        size_t walked = next_page_.load(std::memory_order_acquire);
        *used = walked > contig_pages_ ? contig_pages_ : walked;
        *free_pages = arenaFreePages();
    }
    bool subArenaInfo(uint64_t* dirty_bytes, uint64_t* purged_runs) {
        *dirty_bytes = *purged_runs = 0;
        return false;
    }
    bool subArenaPressure(uint64_t* peak_dirty_bytes, uint64_t* inline_purges) {
        *peak_dirty_bytes = *inline_purges = 0;
        return false;
    }
#endif

private:
    // Created all-zero by bootstrapArray (no constructors run): an unlocked
    // Spinlock, an empty queue and a PageRunMap awaiting init() are all zero.
    struct NodeArena {
        Spinlock lock;      // guards cur/end/carved; freed pages live in `map`
        size_t cur = 0;     // unused tail of the newest chunk, [cur, end);
        size_t end = 0;     // guarded by lock
        size_t carved = 0;  // total pages this node's arena has taken from the
                            // region, for the diagnostic below
        // This node's own free map and decommit queue, so nothing on this
        // node takes a lock or a cache line that another node uses. The map's
        // storage is demand-paged and a page belongs to one owner for life (a
        // node chunk never returns to the shared arena), so a map per owner
        // costs address space, not memory.
        std::atomic<bool> map_ready{false};
        PageRunMap map;
        DecommitQueue queue;
        // Sub-arenas (SMASH_SUBARENAS): created with the node's first
        // allocation. `chunk_lock` guards carving and the pool of wholly free
        // chunks, never any sub-arena's runs.
        std::atomic<bool> subs_ready{false};
        SubArena* subs = nullptr;
        uint32_t nsubs = 0;
        Spinlock chunk_lock;
        uint32_t* free_chunks = nullptr;   // stack of chunk indices, clean and unowned
        size_t nfree_chunks = 0;
    };

    PageRunMap& mapOf(uint32_t owner) {
        return owner == kSharedOwner
            ? free_map_
            : node_arenas_.load(std::memory_order_acquire)[owner - 1].map;
    }
    DecommitQueue& queueOf(int node) {
        return node < 0 ? shared_queue_
                        : node_arenas_.load(std::memory_order_acquire)[node].queue;
    }
    // Before a node's first page is handed out, so every later insert — here
    // or from its decommit queue — finds the map ready.
    bool ensureNodeMap(NodeArena& na) {
        if (na.map_ready.load(std::memory_order_acquire)) [[likely]] return true;
        LockGuard guard(numa_init_lock_);
        if (na.map_ready.load(std::memory_order_relaxed)) return true;
        if (!na.map.init(contig_pages_)) return false;
        na.map_ready.store(true, std::memory_order_release);
        return true;
    }
    static constexpr size_t kNoChunk = ~size_t{0};

    // How long an allocation that missed keeps re-checking for a run that is
    // still on its way back to its free map: at least kDrainWaitAttempts
    // tries, then until kDrainWaitMs have passed. The loops stop as soon as
    // nothing is in flight, so a genuinely full arena still fails promptly;
    // this only bounds a wait for runs another thread is processing. A count
    // of yields alone was a few hundred microseconds, and with the whole
    // arena in the decommit pipeline (32 threads churning 2 MiB blocks
    // through 1 GiB) it expired while runs were still coming: 150-260 blocks
    // of 160000 fell out here and over 1100 on the 128-core aarch64 box
    // (test_shared_arena_churn). Waiting without a bound gave 0.
    static constexpr int kDrainWaitAttempts = 64;
    static constexpr uint64_t kDrainWaitMs = 50;
    // For those loops: whether to try again. `deadline` starts at 0.
    static bool drainWaitMore(int attempt, uint64_t& deadline) {
        if (attempt < kDrainWaitAttempts) return true;
        const uint64_t now = nowMs();
        if (deadline == 0) deadline = now + kDrainWaitMs;
        return now < deadline;
    }

    // Both created on first use, indexed by node id and by grid chunk.
    // node_of_chunk_[c] is node+1, or 0 for a chunk no node arena owns. It is
    // written before any of the chunk's pages are handed out.

    std::atomic<NodeArena*> node_arenas_{nullptr};
    // Sub-arenas for plain malloc's large blocks (SMASH_SUBARENAS): a
    // NodeArena whose chunks are unbound, created on first use.
    std::atomic<NodeArena*> shared_set_{nullptr};
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
        for (uint32_t n = 0; n < vm::kMaxNumaNodes; ++n) {
            arenas[n].queue.region = this;
            arenas[n].queue.node = static_cast<int>(n);
        }
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
