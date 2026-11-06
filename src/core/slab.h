// smash/src/core/slab.h - Per-size-class span manager
//
// Maintains lists of partial/full/empty spans for one size class.
// Thread cache drains/refills go through here.
#pragma once

#include "smash/config.h"
#include <cstdlib>
#include <unistd.h>
#include "span.h"
#include "page_map.h"
#include "size_classes.h"
#include "../util/spinlock.h"
#include "../util/fork_locks.h"
#include "../util/intrusive_list.h"
#include "../vm/platform_mem.h"
#include "../vm/vm_region.h"
#include "../vm/page_state.h"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <atomic>
#include <unistd.h>

namespace smash {

class Slab {
    uint8_t size_class_;
    uint8_t arena_id_ = 0;
    // Per-slab per-page slot cap (0 = no cap).  Set by SmashHeap during init
    // based on cold vs hot sub-arena identity (C1).
    uint32_t max_slots_per_page_ = 0;
    // Adaptive cap callback.  When set, overrides max_slots_per_page_ per
    // new-span allocation; lets SmashHeap revise the cap as it accumulates
    // compression/decompression feedback for this (arena, sc) bucket.
    using CapFn = uint32_t(*)(void* ctx, uint8_t arena, uint8_t sc);
    CapFn cap_fn_ = nullptr;
    void* cap_ctx_ = nullptr;
    Spinlock lock_;
    IntrusiveList<Span> partial_;
    IntrusiveList<Span> full_;
    IntrusiveList<Span> empty_;
    std::atomic<uint32_t> resident_empty_count_{0};  // resident spans in empty_
    PageMap* page_map_;   // set during init, shared across all slabs
    VmRegion* vm_region_ = nullptr;
    PageStateTable* page_states_ = nullptr;
    // Per-page cold-tick counter array owned by CompressorThread.  Plumbed
    // into Span so the bitmap walk can clear the cold counter on the page
    // it just handed out a chunk on — pages still receiving allocations
    // should not be eligible for compression yet.
    uint8_t* cold_counts_ = nullptr;
    bool (*release_hook_)(size_t, size_t, void*) = nullptr;
    void* release_ctx_ = nullptr;

    // ── Lock-free deferred free (Approach A) ────────────────────────────────
    // When a freeing thread finds lock_ contended, instead of blocking it pushes
    // its already-resolved free batch onto this MPSC Treiber stack and returns.
    // The next thread to acquire lock_ (a malloc refill, or a drain that wins
    // tryLock) drains and applies the whole stack first. Nodes live in bootstrap
    // memory (NOT threaded through user objects — that would put metadata on
    // data pages and break compression) and carry the span resolved AT FREE TIME
    // (page ranges recycle across size classes, so re-resolving later is wrong).
    // Modeled on VmRegion's decommit Treiber stack.
    struct PendingNode {
        void*  ptrs[kThreadCacheMaxPerClass];
        Span*  spans[kThreadCacheMaxPerClass];
        uint32_t count;
        PendingNode* next;
    };
    std::atomic<PendingNode*> pending_head_{nullptr};
    std::atomic<uint32_t> pending_depth_{0};   // node count, for back-pressure
    // Bootstrap node free-pool (recycled; steady-state allocation-free).
    Spinlock pending_pool_lock_;
    PendingNode* pending_pool_ = nullptr;

    // ── Magazine depot (lock-free transfer cache, à la tcmalloc) ─────────────
    // A Treiber stack of FULL magazines (reusing PendingNode as the magazine).
    // On free, a resolved batch is pushed here as a magazine. On refill, a
    // magazine is popped and its pointers handed straight back to the thread
    // cache — WITHOUT taking lock_, touching the bitmap, or updating span lists.
    // Objects circulate thread → depot → thread; the span bitmap keeps them
    // marked "allocated" the whole time (correct: they never logically returned
    // to the span, so the compressor won't zero/compress their slots, and
    // allocated_count stays accurate). This closes the alloc/free churn loop
    // lock-free — the slab lock_ is only hit when the depot is empty (genuine
    // growth) or on the eventual drain to reclaim spans. ABA-safe via a tagged
    // head (counter in the high 16 bits of the pointer word is not portable, so
    // we use a separate generation via a 128-bit-free scheme: the magazine node
    // is never freed back to a shared pool while linked, and pop re-validates).
    // Gated by SMASH_MAGAZINE (default off) for clean A/B.
    //
    // SINGLE global depot, guarded by a dedicated tiny lock whose critical
    // section is an O(1) list swing (NOT the bitmap walk / span-list churn that
    // the main slab lock_ does). Objects circulate thread → depot → thread and
    // never touch lock_/bitmap on the common path.
    //
    // NOTE (2026-07-18): a per-CPU-SHARDED depot was prototyped and measured —
    // it LOST to this single depot (96t: sharded 24.0 vs single 27.9 MOPS).
    // Sharding by running CPU removed the (already tiny) mag_lock_ contention but
    // scattered freed magazines across 64 stacks, dropping the refill hit rate;
    // wider work-stealing (16 probes) made it worse still. The single mag_lock_
    // critical section (~5 instructions) was never the bottleneck, so trading
    // hit-rate for lock-sharding is a net loss here. Single depot kept.
    PendingNode* mag_head_ = nullptr;
    uint32_t mag_count_ = 0;
    Spinlock mag_lock_;
    static constexpr uint32_t kMaxMagazines = 64;   // depot cap: bounds the held
        // (marked-allocated, not-yet-returned) set to ~64·256·8 = 128 KB. An
        // UNCAPPED depot livelocked at 32t — held objects + thread caches held
        // the whole live set, exhausting spans. Cap is load-bearing.

    // Push a resolved batch as a full magazine (free path). false → depot full
    // (caller falls back to the real free path so memory stays reclaimable).
    bool magazinePush(void** ptrs, Span** spans, size_t count) {
        PendingNode* n = acquirePendingNode();
        __builtin_memcpy(n->ptrs, ptrs, count * sizeof(void*));
        __builtin_memcpy(n->spans, spans, count * sizeof(Span*));
        n->count = static_cast<uint32_t>(count);
        {
            LockGuard g(mag_lock_);
            if (mag_count_ >= kMaxMagazines) {
                recyclePendingNode(n);
                return false;
            }
            n->next = mag_head_;
            mag_head_ = n;
            ++mag_count_;
        }
        return true;
    }

    // Pop a full magazine (refill path). Returns its count, or 0 if empty.
    size_t magazinePop(void** out_ptrs) {
        PendingNode* n = nullptr;
        {
            LockGuard g(mag_lock_);
            n = mag_head_;
            if (!n) return 0;
            mag_head_ = n->next;
            --mag_count_;
        }
        size_t count = n->count;
        __builtin_memcpy(out_ptrs, n->ptrs, count * sizeof(void*));
        recyclePendingNode(n);
        return count;
    }

    // Above this many queued nodes, the next freer force-locks and drains rather
    // than pushing — bounds memory if a lock owner stalls.
    static constexpr uint32_t kPendingNodeCap = 64;

    PendingNode* acquirePendingNode() {
        {
            LockGuard g(pending_pool_lock_);
            if (pending_pool_) {
                PendingNode* n = pending_pool_;
                pending_pool_ = n->next;
                return n;
            }
        }
        return static_cast<PendingNode*>(
            BootstrapAlloc::instance().allocate(sizeof(PendingNode), alignof(PendingNode)));
    }

    void recyclePendingNode(PendingNode* n) {
        LockGuard g(pending_pool_lock_);
        n->next = pending_pool_;
        pending_pool_ = n;
    }

    // Push a resolved batch onto the pending stack (lock-free, MP producers).
    // Returns false if no node could be had (the metadata arena is exhausted,
    // which it reports itself); the caller then frees the batch directly
    // instead of writing through a null node.
    bool pushPending(void** ptrs, Span** spans, size_t count) {
        PendingNode* n = acquirePendingNode();
        if (!n) return false;
        __builtin_memcpy(n->ptrs, ptrs, count * sizeof(void*));
        __builtin_memcpy(n->spans, spans, count * sizeof(Span*));
        n->count = static_cast<uint32_t>(count);
        PendingNode* head = pending_head_.load(std::memory_order_relaxed);
        do {
            n->next = head;
        } while (!pending_head_.compare_exchange_weak(
            head, n, std::memory_order_release, std::memory_order_relaxed));
        pending_depth_.fetch_add(1, std::memory_order_relaxed);
        return true;
    }

    // Apply all queued deferred frees. CALLER MUST HOLD lock_ (single consumer).
    // Fast-exits on an empty stack with one relaxed load.
    void drainPending() SMASH_REQUIRES(lock_) {
        if (pending_head_.load(std::memory_order_relaxed) == nullptr) [[likely]] return;
        PendingNode* n = pending_head_.exchange(nullptr, std::memory_order_acquire);
        while (n) {
            PendingNode* next = n->next;
            for (uint32_t i = 0; i < n->count; ++i)
                if (n->spans[i]) deallocateOne(n->ptrs[i], n->spans[i]);
            pending_depth_.fetch_sub(1, std::memory_order_relaxed);
            recyclePendingNode(n);
            n = next;
        }
    }

    // Return one object to its span, updating the partial/full/empty lists.
    // Caller holds lock_. Shared by deallocateBatch (resolves spans itself)
    // and deallocateBatchResolved (spans pre-resolved by the caller).
    void deallocateOne(void* ptr, Span* span) {
        const bool was_full = span->full();
        span->deallocate(ptr);

        if (was_full) {
            full_.remove(span);
            partial_.pushFront(span);
        } else if (span->empty()) {
            partial_.remove(span);
            retireEmptySpan(span);
            empty_.pushFront(span);
        }
    }

    // Says WHICH kind of exhaustion this is, because the fix differs: few free
    // pages means the live set genuinely outgrew the arena (raise
    // SMASH_VM_GIB), while many free pages with a short longest run means
    // fragmentation — a bigger arena only postpones it.
    [[gnu::cold, gnu::noinline]] void warnArenaExhausted(size_t want_pages) {
        static std::atomic<bool> warned{false};
        if (warned.exchange(true, std::memory_order_relaxed)) return;
        const size_t total = vm_region_->contigPages();
        const size_t bump = vm_region_->rawNextPage();
        const size_t free_pages = vm_region_->arenaFreePages();
        const size_t longest = vm_region_->arenaLongestFreeRun();
        char buf[448];
        const int n = smash::safe_snprintf(buf, sizeof(buf),
            "[smash WARN] compressible arena exhausted while asking for %zu pages: "
            "%zu of %zu MiB handed out, %zu MiB free in %zu-page runs or shorter. "
            "New small-object spans now come from direct mmap and are NOT compressed. "
            "%s\n",
            want_pages, bump * kPageSize >> 20, total * kPageSize >> 20,
            free_pages * kPageSize >> 20, longest,
            // Fragmentation is exactly "enough free pages, no long-enough run";
            // a free-to-total ratio cannot distinguish the two cases.
            (free_pages >= want_pages && longest < want_pages)
                ? "A run that long is not available though the space is: the arena is "
                  "fragmented, so a larger SMASH_VM_GIB only postpones this."
                : "The live set outgrew the arena: raise SMASH_VM_GIB (ceiling 1024; "
                  "an unused arena costs address space, not memory).");
        if (n > 0) (void)!::write(2, buf, static_cast<size_t>(n));
    }

    // True if the span's pages are in the compressible VmRegion arena. A span
    // can also come from direct mmap once the arena is exhausted (see
    // allocateNewSpan); those pages have no page-state entries and are never
    // compressed, so every region-only step must be skipped for them.
    bool inRegion(const Span* span) const {
        return vm_region_ &&
               vm_region_->inContigArena(reinterpret_cast<uintptr_t>(span->base));
    }

    // A span just became empty. Its pages leave compressor tracking now
    // (release hook, EMPTY state), but the madvise that drops them is
    // deferred: churn that empties and refills spans (free a batch, allocate
    // a batch) otherwise paid a syscall + TLB shootdown per span plus a zero
    // page fault per page on reuse. A resident empty span is reused as-is, or
    // decommitted by purgeResidentEmpty() once it has sat empty for
    // kResidentEmptyTicks compressor ticks. Process-wide resident empties are
    // capped at residentEmptyMaxBytes(); past that, decommit immediately.
    static constexpr uint32_t kResidentEmptyTicks = 2;
    // Default cap: 1/32 of physical memory, clamped to [64 MiB, 1 GiB]. A
    // batch workload that frees and re-allocates a working set larger than
    // the cap pays decommit + refault on the excess every round (measured on
    // a 260 MB set: 32 MiB cap 2.5 s, 512 MiB 0.75 s, glibc 2.3 s), while
    // the retained spans are reused, so peak RSS is unchanged. The cost is a
    // transient: freed spans of one class stay resident for up to
    // kResidentEmptyTicks ticks while another class grows.
    // SMASH_EMPTY_RESIDENT_MB=N overrides it (0 = decommit at once).
    static size_t residentEmptyMaxBytesInit() {
        if (const char* e = std::getenv("SMASH_EMPTY_RESIDENT_MB"))
            return static_cast<size_t>(std::atol(e)) << 20;
        long pages = sysconf(_SC_PHYS_PAGES);
        long psz = sysconf(_SC_PAGESIZE);
        size_t ram = (pages > 0 && psz > 0) ? static_cast<size_t>(pages) * static_cast<size_t>(psz)
                                            : (size_t{2} << 30);
        size_t cap = ram / 32;
        if (cap < (size_t{64} << 20)) cap = size_t{64} << 20;
        if (cap > (size_t{1} << 30)) cap = size_t{1} << 30;
        return cap;
    }
    static size_t residentEmptyMaxBytes() {
        static std::atomic<size_t> cached{~size_t{0}};
        size_t v = cached.load(std::memory_order_relaxed);
        if (v == ~size_t{0}) [[unlikely]] {
            v = residentEmptyMaxBytesInit();
            cached.store(v, std::memory_order_relaxed);
        }
        return v;
    }
    static inline std::atomic<size_t> resident_empty_bytes_{0};
    static inline std::atomic<uint32_t> slab_tick_{0};

    void retireEmptySpan(Span* span) {
        const size_t bytes = span->page_count * kPageSize;
        if (resident_empty_bytes_.load(std::memory_order_relaxed) + bytes > residentEmptyMaxBytes()) {
            decommitEmptySpan(span);
            return;
        }
        // Same compressor and page-state steps as decommitEmptySpan(), and
        // like it only for arena pages: a span from a direct mapping (arena
        // exhausted) has no page states.
        const bool in_region = inRegion(span);
        if (release_hook_ && in_region) {
            size_t idx = vm_region_->pageIndex(reinterpret_cast<uintptr_t>(span->base));
            release_hook_(idx, span->page_count, release_ctx_);
        }
        if (page_states_ && in_region) {
            size_t idx = vm_region_->pageIndex(reinterpret_cast<uintptr_t>(span->base));
            page_states_->setRange(idx, span->page_count, PageState::EMPTY);
        }
        span->resident_empty = true;
        span->empty_tick = slab_tick_.load(std::memory_order_relaxed);
        resident_empty_bytes_.fetch_add(bytes, std::memory_order_relaxed);
        resident_empty_count_.fetch_add(1, std::memory_order_relaxed);
    }

    void clearResident(Span* span) {
        if (!span->resident_empty) return;
        span->resident_empty = false;
        resident_empty_bytes_.fetch_sub(span->page_count * kPageSize, std::memory_order_relaxed);
        resident_empty_count_.fetch_sub(1, std::memory_order_relaxed);
    }

public:
    static void advanceTick() { slab_tick_.fetch_add(1, std::memory_order_relaxed); }
    static size_t residentEmptyBytes() { return resident_empty_bytes_.load(std::memory_order_relaxed); }

    // Compressor tick: decommit empty spans that have stayed empty for
    // kResidentEmptyTicks ticks. Cheap when nothing is resident.
    void purgeResidentEmpty() {
        if (resident_empty_count_.load(std::memory_order_relaxed) == 0) return;
        const uint32_t now = slab_tick_.load(std::memory_order_relaxed);
        LockGuard guard(lock_);
        for (Span* sp = empty_.front(); sp; sp = sp->list_next) {
            if (sp->resident_empty && now - sp->empty_tick >= kResidentEmptyTicks) {
                clearResident(sp);
                const size_t bytes = sp->page_count * kPageSize;
                if (inRegion(sp)) vm::decommitPages(sp->base, bytes);
                else vm::decommitOwnMapping(sp->base, bytes);
            }
        }
    }
private:

    Span* allocateNewSpan() {
        const auto& info = kSizeClasses[size_class_];
        size_t span_bytes = info.pages * kPageSize;
        void* mem = nullptr;
        bool from_region = false;

        if (vm_region_) {
            mem = vm_region_->allocatePages(info.pages);
            if (mem) {
                from_region = true;
                if (page_states_) {
                    size_t idx = vm_region_->pageIndex(reinterpret_cast<uintptr_t>(mem));
                    page_states_->setRange(idx, info.pages, PageState::ACTIVE);
                }
            }
        }
        if (!mem) {
            // No VmRegion (no compression), or the arena is exhausted. The
            // latter used to return nullptr here, which operator new turns
            // into std::bad_alloc: small objects had no fallback, unlike
            // LargeAlloc. The arena runs out without a large live set because
            // an empty span's pages stay with its (arena, size class) — a
            // program that moves between size classes consumes a fresh share
            // of the arena for each (reproduced: 800 MiB of 64-byte objects,
            // all freed, then 128-byte objects failed after 173 MiB in a
            // 1 GiB arena). A direct mapping keeps the program running; its
            // pages are simply not compressed.
            mem = vm::mapPages(span_bytes);
            if (!mem) return nullptr;
            if (vm_region_) warnArenaExhausted(info.pages);
        }

        Span* span = newSpanDescriptor();
        // Both can fail once the metadata arena is exhausted (it reports that
        // itself). Give the pages back and fail the allocation: mallocSlow
        // reports it, rather than dereferencing null.
        if (!span || !span->init(mem, info.pages, size_class_, arena_id_, currentCap())) {
            if (span) recycleSpanDescriptor(span);
            if (from_region) vm_region_->releasePages(mem, info.pages);
            else vm::unmapPages(mem, span_bytes);
            return nullptr;
        }
        // Plumb the page-state lookup so Span::allocate() can avoid handing
        // out chunks on COMPRESSED pages (would fault on first user access),
        // and the cold-count array so allocate() can reset coldness on the
        // page it just handed a chunk out on.
        if (from_region && page_states_) {
            span->page_states = page_states_;
            span->first_page_vm_idx = static_cast<uint32_t>(
                vm_region_->pageIndex(reinterpret_cast<uintptr_t>(mem)));
            span->cold_counts = cold_counts_;
            // Page ranges recycle across spans: clear any stale cold streak
            // so the fresh span's pages classify as warm, not cooling.
            resetColdCounts(span);
        }
        page_map_->setRange(reinterpret_cast<uintptr_t>(mem), info.pages, span);
        return span;
    }

    // Release every empty span straight back to the arena. Called only when
    // the arena is exhausted (VmRegion's exhausted hook), not on every free:
    // returning spans eagerly made mi-stress ~70% slower (16.9 s -> 28.7 s),
    // because spans oscillating between empty and in use paid a release, a
    // decommit and zero-fill faults on every round trip. Keeping empties is
    // free until the arena runs out; then they are exactly the memory to reuse.
    // tryLock: the caller may hold another slab's lock (allocateNewSpan), so
    // blocking here could deadlock; a busy slab is simply skipped. That skip
    // is only safe because the heap serialises passes (reclaimEmptySpansHook):
    // a slab is almost always busy because another thread's pass is releasing
    // it right now.
    //
    // Adjacent spans go back as one run. Releasing them one at a time cost an
    // mprotect, an madvise and a free-map insert each: 82-92 ms for the ~42000
    // spans a 1 GiB arena of 256-byte objects leaves, which is how long every
    // other allocation that ran out of arena then waited. Spans empty in the
    // order their objects were freed, so the list is nearly address-ordered
    // (those 42000 formed 7 runs), and merging neighbours in list order needs
    // no sort and no buffer.
public:
    size_t reclaimEmptySpans() {
        if (!lock_.tryLock()) return 0;
        drainPending();
        size_t n = 0;
        uintptr_t run_lo = 0, run_hi = 0;           // the pending run, [lo, hi)
        auto flush = [&] {
            if (run_hi == run_lo) return;
            releaseRegionRun(reinterpret_cast<void*>(run_lo), (run_hi - run_lo) / kPageSize);
            run_lo = run_hi = 0;
        };
        while (Span* span = empty_.popFront()) {
            ++n;
            clearResident(span);
            if (!inRegion(span)) {
                releaseSpan(span, /*sync=*/true);
                recycleSpanDescriptor(span);
                continue;
            }
            const uintptr_t lo = reinterpret_cast<uintptr_t>(span->base);
            const uintptr_t hi = lo + span->page_count * kPageSize;
            // Clear the ownership records before the pages go back, as
            // releaseSpan does, so a reallocation cannot resolve to a dead span.
            page_map_->clearRange(lo, span->page_count);
            recycleSpanDescriptor(span);
            if (run_hi != run_lo && lo == run_hi) run_hi = hi;
            else if (run_hi != run_lo && hi == run_lo) run_lo = lo;
            else { flush(); run_lo = lo; run_hi = hi; }
        }
        flush();
        lock_.unlock();
        return n;
    }

private:
    // Return a run of arena pages whose spans are already gone: the
    // compressor's release hook first, then straight to the free map.
    void releaseRegionRun(void* base, size_t pages) {
        if (release_hook_)
            release_hook_(vm_region_->pageIndex(reinterpret_cast<uintptr_t>(base)), pages,
                          release_ctx_);
        vm_region_->releasePagesNow(base, pages);
    }

    void releaseSpan(Span* span, bool sync = false) {
        clearResident(span);
        page_map_->clearRange(reinterpret_cast<uintptr_t>(span->base), span->page_count);
        if (inRegion(span)) {
            if (release_hook_) {
                size_t idx = vm_region_->pageIndex(reinterpret_cast<uintptr_t>(span->base));
                release_hook_(idx, span->page_count, release_ctx_);
            }
            if (sync) vm_region_->releasePagesNow(span->base, span->page_count);
            else vm_region_->releasePages(span->base, span->page_count);
        } else {
            vm::unmapPages(span->base, span->page_count * kPageSize);
        }
    }

    // Decommit pages of an empty span via MADV_DONTNEED/MADV_FREE, releasing
    // physical memory immediately without waiting for the compressor to
    // discover, zero, and compress them.  The span stays in the empty list
    // for reuse; recommitEmptySpan() restores page state on reuse.
    void decommitEmptySpan(Span* span) {
        const bool in_region = inRegion(span);
        if (release_hook_ && in_region) {
            size_t idx = vm_region_->pageIndex(reinterpret_cast<uintptr_t>(span->base));
            release_hook_(idx, span->page_count, release_ctx_);
        }
        size_t bytes = span->page_count * kPageSize;
        if (in_region) vm::decommitPages(span->base, bytes);
        else vm::decommitOwnMapping(span->base, bytes);
        if (page_states_ && in_region) {
            size_t idx = vm_region_->pageIndex(reinterpret_cast<uintptr_t>(span->base));
            page_states_->setRange(idx, span->page_count, PageState::EMPTY);
        }
    }

    // Clear the cold streak on every page of a span so it classifies as
    // warm. Needed when a span's pages (re)enter service — the cold-count
    // array is indexed by VM page and keeps whatever streak the previous
    // occupant left behind.
    void resetColdCounts(Span* span) {
        if (!cold_counts_ || !inRegion(span)) return;
        size_t idx = vm_region_->pageIndex(reinterpret_cast<uintptr_t>(span->base));
        __builtin_memset(cold_counts_ + idx, 0, span->page_count);
    }

    // Restore page state when reusing a decommitted empty span.
    // Physical pages are zero-filled by the kernel on first access
    // (MADV_DONTNEED on Linux, MADV_FREE on macOS).
    void recommitEmptySpan(Span* span) {
        clearResident(span);
        if (page_states_ && inRegion(span)) {
            size_t idx = vm_region_->pageIndex(reinterpret_cast<uintptr_t>(span->base));
            page_states_->setRange(idx, span->page_count, PageState::ACTIVE);
        }
        // The span sat idle in the empty list — its pages carry a stale cold
        // streak that would classify them as cooling and defeat the redirect
        // that chose this span precisely because it has nothing to protect.
        resetColdCounts(span);
    }

public:
    void init(uint8_t sc, PageMap* pm,
              VmRegion* vr = nullptr, PageStateTable* ps = nullptr,
              bool (*hook)(size_t, size_t, void*) = nullptr,
              void* hook_ctx = nullptr,
              uint8_t arena_id = 0,
              uint32_t max_slots_per_page = 0,
              uint8_t* cold_counts = nullptr) {
        size_class_ = sc;
        arena_id_ = arena_id;
        max_slots_per_page_ = max_slots_per_page;
        page_map_ = pm;
        vm_region_ = vr;
        page_states_ = ps;
        cold_counts_ = cold_counts;
        release_hook_ = hook;
        release_ctx_ = hook_ctx;
    }

    // Current per-page cap for new spans (or for widening existing ones).
    // Uses cap_fn_ if installed, else the static max_slots_per_page_.
    uint32_t currentCap() const {
        return cap_fn_ ? cap_fn_(cap_ctx_, arena_id_, size_class_)
                       : max_slots_per_page_;
    }

    // Widen a partial span's bitmap if the bucket's cap has relaxed since
    // this span was initialized.  Lets a bucket mis-classified as cold
    // recover once decomp evidence reveals it's actually hot, without
    // forcing us to discard spans or pre-allocate fresh ones.
    void maybeWiden(Span* span) {
        if (!cap_fn_ || span->current_cap_per_page == 0) return;
        uint32_t cur_cap = currentCap();
        if (cur_cap == 0 || cur_cap > span->current_cap_per_page) {
            span->widenCap(cur_cap);
        }
    }

    // Allocate one object from this size class. Caller must hold no locks.
    // Allocate an object whose slot index is a multiple of `stride`, so its
    // address carries that much extra alignment. nullptr when no span of this
    // class can host one — the caller then falls back to promoting the request
    // to a size class that is itself a multiple of the alignment.
    // SMASH_ALIGN_SLOT_SEARCH only.
    //
    // Only existing spans are searched, and a new one is created just as
    // allocate() would. What this deliberately does NOT do is keep looking
    // past the first partial span: an aligned request that misses there is
    // cheaper to serve by promotion than by walking every span of the class.
    void* allocateAligned(uint32_t stride) {
        LockGuard guard(lock_);
        drainPending();

        if (Span* span = partial_.front()) {
            maybeWiden(span);
            if (void* ptr = span->allocateAligned(stride)) {
                if (span->full()) {
                    partial_.remove(span);
                    full_.pushFront(span);
                }
                return ptr;
            }
        }

        Span* span = empty_.popFront();
        if (!span) {
            span = allocateNewSpan();
            if (!span) return nullptr;
        } else {
            recommitEmptySpan(span);
        }
        void* ptr = span->allocateAligned(stride);
        if (!ptr) {
            // A fresh span always has slot 0 free, so this cannot happen for a
            // stride the span can host; if it ever does, keep the span on the
            // empty list rather than leaking it.
            empty_.pushFront(span);
            return nullptr;
        }
        if (span->full()) full_.pushFront(span);
        else partial_.pushFront(span);
        return ptr;
    }

    void* allocate() {
        LockGuard guard(lock_);
        drainPending();  // apply any deferred cross-thread frees first

        // Try partial spans first
        Span* span = partial_.front();
        if (span) {
            maybeWiden(span);
            Span::AllocFallback fb;
            void* ptr = span->allocateWarm(&fb);
            if (!ptr && fb.any()) {
                // The span's only free slots are on cooling/compressed pages.
                // Prefer a decommitted empty span: committing one of its
                // pages costs the same 4 KB the cooling page would keep
                // resident, and lets the cooling page finish cooling and
                // compress. Never allocate a NEW span for this — if the
                // empty list is dry, consume the fallback (no page-count
                // growth either way).
                if (Span* es = empty_.popFront()) {
                    // Rotate the warmless span to the back so the next
                    // allocation doesn't rescan it while it cools.
                    partial_.remove(span);
                    partial_.pushBack(span);
                    recommitEmptySpan(es);
                    partial_.pushFront(es);
                    span = es;
                    ptr = span->allocate();
                } else {
                    ptr = span->consumeFallback(fb);
                }
            }
            if (span->full()) {
                partial_.remove(span);
                full_.pushFront(span);
            }
            return ptr;
        }

        // Try reusing an empty span (pages were decommitted)
        span = empty_.popFront();
        if (!span) {
            // Allocate a fresh span
            span = allocateNewSpan();
            if (!span) return nullptr;
        } else {
            recommitEmptySpan(span);
        }

        void* ptr = span->allocate();
        if (span->full()) {
            full_.pushFront(span);
        } else {
            partial_.pushFront(span);
        }
        return ptr;
    }

    // Free one object back to its span. Caller must hold no locks.
    void deallocate(Span* span, void* ptr) {
        LockGuard guard(lock_);

        bool was_full = span->full();
        span->deallocate(ptr);

        if (was_full) {
            // Transition: full → partial
            full_.remove(span);
            partial_.pushFront(span);
        } else if (span->empty()) {
            // Transition: partial → empty.  Move to empty list; the
            // compressor's zeroFreeSlots + compress cycle handles
            // reclamation. Immediate madvise here was a syscall per
            // span that destroyed throughput under batch-free workloads.
            partial_.remove(span);
            empty_.pushFront(span);
        }
    }

    // Batch allocate: fill an array with up to `count` pointers.
    // Returns actual number allocated.
    //
    // When kPageLocalBatch is true, the inner loop stops as soon as the
    // next allocated object would lie on a different VM page than the
    // batch's first object.  This keeps each thread-cache refill confined
    // to one page, so objects allocated in a single burst share a page
    // and (under Pareto-skew access) tend to cool together.
    size_t allocateBatch(void** out, size_t count) {
        LockGuard guard(lock_);
        drainPending();  // apply any deferred cross-thread frees first
        size_t allocated = 0;
        // One empty-span redirect per refill: when the front partial span
        // has only cooling/compressed free slots, switch to a decommitted
        // empty span instead of backfilling (same physical cost, and the
        // cooling pages get to compress). Bounded to one redirect so a
        // refill can't churn the empty list; after that, fallbacks are
        // consumed normally. Never allocates a NEW span for this.
        bool redirected = false;

        while (allocated < count) {
            Span* span = partial_.front();
            if (!span) {
                span = empty_.popFront();
                if (!span) {
                    span = allocateNewSpan();
                    if (!span) break;
                } else {
                    recommitEmptySpan(span);
                }
                partial_.pushFront(span);
            }
            maybeWiden(span);

            uintptr_t first_page = 0;
            bool first_in_batch = (allocated == 0);
            while (allocated < count) {
                Span::AllocFallback fb;
                void* ptr = span->allocateWarm(&fb);
                if (!ptr && fb.any()) {
                    if (!redirected && empty_.front()) {
                        // Rotate the warmless span to the back so it cools
                        // undisturbed; the outer loop picks up the empty
                        // span we just promoted.
                        redirected = true;
                        partial_.remove(span);
                        partial_.pushBack(span);
                        Span* es = empty_.popFront();
                        recommitEmptySpan(es);
                        partial_.pushFront(es);
                        break;  // outer loop re-reads partial_.front()
                    }
                    ptr = span->consumeFallback(fb);
                }
                if (!ptr) break;
                if (kPageLocalBatch) {
                    uintptr_t p = reinterpret_cast<uintptr_t>(ptr) & ~(kPageSize - 1);
                    if (first_in_batch) {
                        first_page = p;
                        first_in_batch = false;
                    } else if (p != first_page) {
                        // Crossed a page boundary.  Return the object and stop
                        // — next refill will anchor on a fresh page.
                        span->deallocate(ptr);
                        return allocated;
                    }
                }
                out[allocated++] = ptr;
            }

            if (span->full()) {
                partial_.remove(span);
                full_.pushFront(span);
            }
        }
        return allocated;
    }

    // Batch deallocate: return `count` pointers to their spans.
    void deallocateBatch(void** ptrs, size_t count) {
        LockGuard guard(lock_);
        for (size_t i = 0; i < count; ++i) {
            // Look up the span for each pointer
            Span* span = page_map_->get(reinterpret_cast<uintptr_t>(ptrs[i]));
            if (!span) continue;
            deallocateOne(ptrs[i], span);
        }
    }

    // Same as deallocateBatch but with spans already resolved by the caller
    // (ThreadCache::drain resolves each span once, then groups by arena). Skips
    // the per-pointer PageMap radix walk the plain deallocateBatch would repeat.
    void deallocateBatchResolved(void** ptrs, Span** spans, size_t count) {
        LockGuard guard(lock_);
        drainPending();
        for (size_t i = 0; i < count; ++i) {
            if (!spans[i]) continue;
            deallocateOne(ptrs[i], spans[i]);
        }
    }

    // Lock-free-friendly batch free (Approach A). If the slab lock is
    // uncontended, take it, drain any pending deferred frees, and apply this
    // batch directly. If contended, push the resolved batch onto the pending
    // stack and return without blocking — a future lock holder applies it. To
    // bound memory if a holder stalls, force-lock once the queue is deep.
    void deallocateBatchDeferred(void** ptrs, Span** spans, size_t count)
            SMASH_NO_TS_ANALYSIS {
        if (count == 0) return;
        if (pending_depth_.load(std::memory_order_relaxed) < kPendingNodeCap) {
            if (lock_.tryLock()) {
                drainPending();
                for (size_t i = 0; i < count; ++i)
                    if (spans[i]) deallocateOne(ptrs[i], spans[i]);
                lock_.unlock();
                return;
            }
            // Contended → defer without blocking, unless no node is
            // available; then block and free directly rather than drop it.
            if (pushPending(ptrs, spans, count)) return;
        }
        // Back-pressure: queue too deep, block to drain it and apply directly.
        LockGuard guard(lock_);
        drainPending();
        for (size_t i = 0; i < count; ++i)
            if (spans[i]) deallocateOne(ptrs[i], spans[i]);
    }

    static bool magazineEnabled() {
        static const bool on = [] {
            const char* v = std::getenv("SMASH_MAGAZINE");
            return v && v[0] == '1';
        }();
        return on;
    }

    // ── Magazine (transfer-cache) public entry points ───────────────────────
    // Refill: try to satisfy the whole batch from one popped magazine (no lock_,
    // no bitmap). Returns count handed back, or 0 if the depot is empty (caller
    // falls back to allocateBatch). Only valid when magazineEnabled().
    size_t allocateBatchMag(void** out) {
        return magazinePop(out);
    }

    // Free: push a resolved full-lane batch as a magazine (no lock_, no bitmap).
    // Returns true if pushed; false if the depot is full (caller falls back to
    // the real free path so memory is still reclaimable). Spans are carried so a
    // later depot drain (on shutdown / span reclaim) can return them properly.
    bool deallocateBatchMag(void** ptrs, Span** spans, size_t count) {
        return magazinePush(ptrs, spans, count);
    }

    // Drain the whole depot back through the real free path (bitmap + span
    // lists). Called under lock_ when we need spans reclaimable (scavenge) or at
    // shutdown. Objects held in magazines were never returned to their spans, so
    // this is where their bitmap bits finally clear.
    void drainMagazines() SMASH_REQUIRES(lock_) {
        for (;;) {
            PendingNode* n = nullptr;
            {
                LockGuard g(mag_lock_);
                n = mag_head_;
                if (!n) break;
                mag_head_ = n->next;
                --mag_count_;
            }
            for (uint32_t i = 0; i < n->count; ++i)
                if (n->spans[i]) deallocateOne(n->ptrs[i], n->spans[i]);
            recyclePendingNode(n);
        }
    }

    // Return empty spans' pages to OS
    void scavenge() {
        LockGuard guard(lock_);
        drainPending();
        if (magazineEnabled()) drainMagazines();
        while (Span* span = empty_.popFront()) {
            releaseSpan(span);
        }
    }

    void setCapFn(CapFn fn, void* ctx) {
        cap_fn_ = fn;
        cap_ctx_ = ctx;
    }

    void lockSlab() SMASH_ACQUIRE(lock_) { lock_.lock(); }
    void unlockSlab() SMASH_RELEASE(lock_) { lock_.unlock(); }
    void forkLocks(ForkLockSet& s) { s.add(lock_); s.add(pending_pool_lock_); s.add(mag_lock_); }
};

} // namespace smash
