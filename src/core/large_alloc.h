// smash/src/core/large_alloc.h - Large allocations (> kMaxSmallSize)
//
// When a VmRegion is available, large allocations are placed there so the
// compressor thread can track and compress their pages.  Falls back to
// direct mmap for oversized alignments or when the VmRegion is full.
#pragma once

#if defined(__linux__)
#include <sys/mman.h>   // mremap
#endif

#include "smash/config.h"
#include "span.h"
#include "page_map.h"
#include "../util/spinlock.h"
#include "../util/fork_locks.h"
#include "../util/bitops.h"
#include "../vm/platform_mem.h"
#include "../vm/vm_region.h"
#include "../vm/page_state.h"

#include <cstddef>
#include <cstdint>
#include <cstdlib>

namespace smash {

// ── Direct-mapped block cache ───────────────────────────────────────────────
// From upstream #93. Large blocks below the compressible threshold
// (SMASH_LARGE_ALLOC_VM_THRESHOLD, default 1 MiB) are placed with a direct
// mmap and were munmap'd on free, so a program that churns 16 KiB..1 MiB
// buffers paid an mmap + munmap (+ TLB shootdown + page faults) per
// malloc/free pair: 25-50x slower than glibc, which serves these sizes from
// its heap (measured here: 64 KiB churn 4412 -> 34 ns/op in full mode). Freed
// blocks of those sizes now go to a small cache and are reused by later
// allocations of the same rounded size.
//
// - Page counts are rounded up to 8 steps per power of two (<= 12.5% extra
//   address space; tail pages a program never touches cost no RSS), so a
//   freed block serves any request that rounds to the same bucket.
// - Bounded: kDepth blocks per bucket and maxBytes() in total; a free that
//   would exceed either unmaps as before.
// - Idle blocks are returned to the OS by purgeIdle(), called from the
//   compressor tick, once a bucket has gone kIdleTicks ticks without a free.
// - Only blocks that are direct mappings by design (LargeAlloc's
//   directByDesign) from LargeAlloc::allocate() (Span::block_cacheable),
//   never a malloc_numa block. Upstream also caches arena blocks up to 8 MiB;
//   here a freed arena block takes this branch's release path, where keeping
//   it warm is opt-in (SMASH_RUN_CACHE_MIB, SMASH_SUBARENAS), and a direct
//   mapping of a compressible size (made because the arena was full) is never
//   cached, so it cannot be handed out ahead of reclaimed arena space.
class LargeBlockCache {
public:
    // Blocks up to 8 MiB (the threshold can be raised that far).
    static constexpr size_t kMaxPages = (size_t{8} << 20) / kPageSize;
    static constexpr int kDepth = 32;
    static constexpr size_t kMaxBytesDefault = 32ull << 20;
    // SMASH_LARGE_CACHE_MB=N overrides the byte cap (0 disables the cache).
    static size_t maxBytes() {
        static std::atomic<size_t> cached{~size_t{0}};
        size_t v = cached.load(std::memory_order_relaxed);
        if (v == ~size_t{0}) [[unlikely]] {
            const char* e = std::getenv("SMASH_LARGE_CACHE_MB");
            v = e ? static_cast<size_t>(std::atol(e)) << 20 : kMaxBytesDefault;
            cached.store(v, std::memory_order_relaxed);
        }
        return v;
    }
    static constexpr uint32_t kIdleTicks = 2;  // compressor ticks (kCompressIntervalMs = 1 s)

    // Rounded page count for a request of n pages (n >= 1).
    static size_t roundPages(size_t n) {
        if (n <= 8) return n;
        int b = 63 - __builtin_clzll(n - 1);      // n in (2^b, 2^(b+1)]
        size_t step = size_t{1} << (b - 3);
        return (n + step - 1) & ~(step - 1);
    }
    // The next rounded size above r (r on the rounding grid).
    static size_t nextRounded(size_t r) {
        if (r < 8) return r + 1;
        int b = 63 - __builtin_clzll(r);  // r in [2^b, 2^(b+1)) on the grid
        return r + (size_t{1} << (b - 3));
    }
    static int bucketOf(size_t rounded) {
        if (rounded <= 8) return static_cast<int>(rounded) - 1;
        int b = 63 - __builtin_clzll(rounded - 1);
        size_t step = size_t{1} << (b - 3);
        return 8 + (b - 3) * 8 + static_cast<int>(rounded / step - 9);
    }

    // Pop a cached block of exactly `rounded` pages, or nullptr. Blocks are
    // cached together with their Span descriptor (span->base is the block),
    // which saves a descriptor-stack pop and push per malloc/free pair.
    Span* pop(size_t rounded) {
        Bucket& bk = bucketFor(rounded);
        if (bk.count.load(std::memory_order_relaxed) == 0) return nullptr;
        LockGuard g(bk.lock);
        uint32_t n = bk.count.load(std::memory_order_relaxed);
        if (n == 0) return nullptr;
        Span* p = bk.blocks[n - 1];
        bk.count.store(n - 1, std::memory_order_relaxed);
        bytes_.fetch_sub(rounded * kPageSize, std::memory_order_relaxed);
        return p;
    }

    // Cache a freed block of `rounded` pages with its descriptor. False =>
    // the caller unmaps the block and recycles the descriptor.
    bool push(Span* span, size_t rounded, uint32_t now) {
        size_t sz = rounded * kPageSize;
        if (bytes_.load(std::memory_order_relaxed) + sz > maxBytes()) return false;
        Bucket& bk = bucketFor(rounded);
        LockGuard g(bk.lock);
        uint32_t n = bk.count.load(std::memory_order_relaxed);
        if (n >= kDepth) return false;
        bk.blocks[n] = span;
        bk.count.store(n + 1, std::memory_order_relaxed);
        bk.last_use = now;
        bytes_.fetch_add(sz, std::memory_order_relaxed);
        return true;
    }

    // Room for a block of `rounded` pages under the byte cap?
    bool canAccept(size_t rounded) const {
        return bytes_.load(std::memory_order_relaxed) + rounded * kPageSize <= maxBytes();
    }

    // Return every cached block in buckets untouched for kIdleTicks;
    // release(span, pages) disposes of each block and its descriptor.
    template <class Release>
    void purgeIdle(uint32_t now, Release release) {
        if (bytes_.load(std::memory_order_relaxed) == 0) return;
        for (size_t r = 1; r <= kMaxPages; r = nextRounded(r)) {
            Bucket& bk = bucketFor(r);
            if (bk.count.load(std::memory_order_relaxed) == 0) continue;
            Span* victims[kDepth];
            uint32_t nv = 0;
            {
                LockGuard g(bk.lock);
                if (now - bk.last_use < kIdleTicks) continue;
                nv = bk.count.load(std::memory_order_relaxed);
                for (uint32_t i = 0; i < nv; ++i) victims[i] = bk.blocks[i];
                bk.count.store(0, std::memory_order_relaxed);
            }
            bytes_.fetch_sub(nv * r * kPageSize, std::memory_order_relaxed);
            for (uint32_t i = 0; i < nv; ++i) release(victims[i], r);
        }
    }

    size_t cachedBytes() const { return bytes_.load(std::memory_order_relaxed); }

    void forkLocks(ForkLockSet& s) {
        for (auto& b : buckets_) s.add(b.lock);
    }

private:
    struct alignas(64) Bucket {
        Spinlock lock;
        std::atomic<uint32_t> count{0};
        uint32_t last_use = 0;
        Span* blocks[kDepth];
    };
    static constexpr int kUsedBuckets = []{
        int b = 0;
        for (size_t r = 1; r <= kMaxPages; r = (r <= 8 ? r + 1 : r + (size_t{1} << (63 - __builtin_clzll(r) - 3))))
            ++b;
        return b;
    }();
    Bucket& bucketFor(size_t rounded) { return buckets_[bucketOf(rounded)]; }

    Bucket buckets_[kUsedBuckets + 1];
    std::atomic<size_t> bytes_{0};
};


class LargeAlloc {
    Spinlock lock_;
    PageMap* page_map_;
    VmRegion* vm_region_ = nullptr;
    PageStateTable* page_states_ = nullptr;
    bool (*release_hook_)(size_t, size_t, void*) = nullptr;
    void* release_ctx_ = nullptr;
    LargeBlockCache cache_;
    std::atomic<uint32_t> tick_{0};

    // A cached block leaving the cache for good (always a direct mapping).
    void releaseCached(Span* sp, size_t pages) {
        vm::unmapPages(sp->base, pages * kPageSize);
        recycleSpanDescriptor(sp);
    }

    // A size that never goes to the arena: below the compressible threshold,
    // or no arena at all. Only those blocks are cached. A direct mapping of a
    // compressible size exists only because the arena was full, and caching
    // it handed it back ahead of arena space reclaimed since
    // (test_arena_reclaim_concurrent: 16 of 200 blocks outside the arena, the
    // cache's 32 MiB). Freed arena blocks take this branch's release path,
    // whose warm reuse is opt-in (run cache, sub-arenas).
    bool directByDesign(size_t bytes) const {
        return !vm_region_ || bytes < largeAllocVmThreshold();
    }

public:
    // Compressor tick: advance the cache clock and unmap idle blocks.
    void onTick() {
        const uint32_t now = tick_.fetch_add(1, std::memory_order_relaxed) + 1;
        cache_.purgeIdle(now, [this](Span* sp, size_t pages) { releaseCached(sp, pages); });
    }

    size_t cachedBytes() const { return cache_.cachedBytes(); }

    void init(PageMap* pm,
              VmRegion* vr = nullptr, PageStateTable* ps = nullptr,
              bool (*hook)(size_t, size_t, void*) = nullptr,
              void* hook_ctx = nullptr) {
        page_map_ = pm;
        vm_region_ = vr;
        page_states_ = ps;
        release_hook_ = hook;
        release_ctx_ = hook_ctx;
    }

    // A size in a warning: MiB, or KiB below 1 MiB, so a small block does not
    // read "0 MiB".
    static size_t warnSize(size_t bytes) { return bytes >= (size_t{1} << 20) ? bytes >> 20 : bytes >> 10; }
    static const char* warnUnit(size_t bytes) { return bytes >= (size_t{1} << 20) ? "MiB" : "KiB"; }

    // One-shot: the arena could not take a block that was meant to be
    // compressible, and the block now exists outside it. Called only then:
    // a request that got no block at all spent the warning on a false "it came
    // from a direct mmap", and the overflow that followed went unreported.
    // Mirrors Slab::warnArenaExhausted, including the capacity-vs-fragmentation
    // split, because the fix differs.
    [[gnu::cold, gnu::noinline]] void warnLargeAllocNotCompressible(size_t alloc_size) {
        static std::atomic<bool> warned{false};
        if (warned.exchange(true, std::memory_order_relaxed)) return;
        const size_t total = vm_region_->contigPages();
        const size_t bump = vm_region_->rawNextPage();
        const size_t free_pages = vm_region_->arenaFreePages();
        const size_t longest = vm_region_->arenaLongestFreeRun();
        const size_t want_pages = alloc_size / kPageSize;
        char verdict[224];
        if (want_pages > total)
            // Not "the live set outgrew the arena": nothing else need be live.
            smash::safe_snprintf(verdict, sizeof(verdict),
                "The block alone needs %zu MiB, more than the whole %zu MiB arena: raise "
                "SMASH_VM_GIB above that (ceiling 1024; an unused arena costs address "
                "space, not memory).", alloc_size >> 20, total * kPageSize >> 20);
        else
            smash::safe_snprintf(verdict, sizeof(verdict), "%s",
                // Exact, not a guess: it is fragmentation precisely when enough
                // pages are free but no single run is long enough. A ratio of free
                // to total cannot tell those apart — measured, an arena with 1021
                // of 4096 MiB free in 512 KB runs is 24.9% free and was reported as
                // "the live set outgrew the arena", which is the opposite of the
                // truth and sends the reader to raise SMASH_VM_GIB for nothing.
                (free_pages >= want_pages && longest < want_pages)
                    ? "A run that long is not available though the space is: the arena is "
                      "fragmented, so a larger SMASH_VM_GIB only postpones this. Blocks "
                      "of very different sizes sharing the arena is the usual cause."
                    : "The live set outgrew the arena: raise SMASH_VM_GIB (ceiling 1024; "
                      "an unused arena costs address space, not memory).");
        char buf[640];
        const int n = smash::safe_snprintf(buf, sizeof(buf),
            "[smash WARN] a %zu %s allocation did not fit the compressible arena: "
            "%zu of %zu MiB handed out, %zu MiB free in %zu-page runs or shorter. "
            "It came from a direct mmap instead and will NEVER be compressed, "
            "however cold it gets. %s\n",
            warnSize(alloc_size), warnUnit(alloc_size), bump * kPageSize >> 20, total * kPageSize >> 20,
            free_pages * kPageSize >> 20, longest, verdict);
        if (n > 0) (void)!::write(2, buf, static_cast<size_t>(n));
    }

#if defined(__linux__)
    // One-shot, as warnLargeAllocNotCompressible, and like it called only once
    // the block exists: a malloc_numa block the node's arena could not take.
    // It used to fall back silently.
    [[gnu::cold, gnu::noinline]] void warnNumaAllocNotCompressible(size_t alloc_size, uint32_t node,
                                                                   size_t alignment) {
        static std::atomic<bool> warned{false};
        if (warned.exchange(true, std::memory_order_relaxed)) return;
        const size_t total = vm_region_->contigPages();
        const size_t bump = vm_region_->rawNextPage();
        size_t carved = 0, node_free = 0;
        vm_region_->nodeArenaInfo(node, &carved, &node_free);
        // What the node's arena needs to be sure of placing the block: its
        // pages and, for an alignment past a page, the worst-case gap before
        // an aligned start. Weighing the free pages against the block alone
        // said "has the space" for one page at 2 GiB alignment in a 1 GiB arena.
        const bool aligned = alignment > kPageSize;
        const size_t need_pages = alloc_size / kPageSize + (aligned ? alignment / kPageSize - 1 : 0);
        char align_note[48] = "";
        if (aligned)
            smash::safe_snprintf(align_note, sizeof(align_note), " at %zu %s alignment",
                                 warnSize(alignment), warnUnit(alignment));
        char verdict[224];
        if (need_pages > total)
            smash::safe_snprintf(verdict, sizeof(verdict),
                "%s %zu MiB, more than the whole %zu MiB arena: raise SMASH_VM_GIB above that "
                "(ceiling 1024; an unused arena costs address space, not memory).",
                aligned ? "The block and its alignment need" : "The block needs",
                need_pages * kPageSize >> 20, total * kPageSize >> 20);
        else
            smash::safe_snprintf(verdict, sizeof(verdict), "%s",
                // Exact: the node's arena failed this block with that much free,
                // so the free space is in runs too short, or where this block
                // cannot take it (another sub-arena's, or still being decommitted).
                node_free >= need_pages
                    ? "The node's arena has the space but could not give it to this block: a larger "
                      "SMASH_VM_GIB only postpones this."
                    : "The arena is full: raise SMASH_VM_GIB (ceiling 1024; an unused arena costs "
                      "address space, not memory).");
        char buf[768];
        const int n = smash::safe_snprintf(buf, sizeof(buf),
            "[smash WARN] a %zu %s malloc_numa allocation%s for node %u did not fit the compressible arena: "
            "%zu of %zu MiB handed out, node %u's arena holds %zu MiB of it with %zu MiB free. "
            "It came from a separate mapping bound to node %u instead and will NEVER be compressed, "
            "however cold it gets. %s\n",
            warnSize(alloc_size), warnUnit(alloc_size), align_note, node,
            (bump < total ? bump : total) * kPageSize >> 20, total * kPageSize >> 20,
            node, carved * kPageSize >> 20, node_free * kPageSize >> 20, node, verdict);
        if (n > 0) (void)!::write(2, buf, static_cast<size_t>(n));
    }
#endif

    // `*zeroed`, if given, says whether every page of the block reads as zero
    // already: a fresh mapping, or an arena run that was never used or was
    // decommitted (VmRegion::allocateLargePages). calloc skips its memset then.
    void* allocate(size_t size, size_t alignment, uint8_t arena_id = 0, bool* zeroed = nullptr) {
        if (alignment < kPageSize) alignment = kPageSize;

        // Round size up to page boundary. A size whose rounding wraps or whose
        // page count does not fit the span's 32 bits is ENOMEM, as glibc says
        // for any size it cannot map: unchecked, malloc(16 TiB + 4 KiB)
        // returned one page, malloc(2^62) and malloc(16 TiB) a block of none,
        // and posix_memalign(2 MiB, SIZE_MAX) a block of the alignment alone,
        // each with malloc_usable_size claiming the size asked for.
        size_t alloc_size = roundUp(size, kPageSize);
        if (alloc_size < size || alloc_size / kPageSize > UINT32_MAX) [[unlikely]] {
            errno = ENOMEM;
            return nullptr;
        }
        uint32_t num_pages = static_cast<uint32_t>(alloc_size / kPageSize);

        void* mem = nullptr;
        bool fresh = false;
        Span* span = nullptr;   // a cached block's descriptor, reused

        // Block cache (LargeBlockCache): a size it holds is rounded to its
        // grid, so the block can be cached when freed, and is served from it
        // first: a block of the same rounded size, else one of the next two
        // larger buckets (<= ~25% extra address space). Only sizes that are
        // direct mappings by design; arena sizes are not rounded.
        const bool cacheable = alignment <= kPageSize &&
                               num_pages <= LargeBlockCache::kMaxPages &&
                               LargeBlockCache::maxBytes() != 0 &&
                               directByDesign(alloc_size);
        if (cacheable) {
            size_t rounded = LargeBlockCache::roundPages(num_pages);
            num_pages = static_cast<uint32_t>(rounded);
            span = cache_.pop(rounded);
            for (int up = 0; !span && up < 2; ++up) {
                const size_t bigger = LargeBlockCache::nextRounded(rounded);
                if (bigger > LargeBlockCache::kMaxPages) break;
                rounded = bigger;
                span = cache_.pop(rounded);
            }
            if (span) {
                mem = span->base;
                num_pages = static_cast<uint32_t>(rounded);
            }
        }
        const size_t map_bytes = size_t{num_pages} * kPageSize;

        // Route large-enough allocations through VmRegion for compression
        if (!mem && vm_region_ && alignment <= kPageSize && alloc_size >= largeAllocVmThreshold()) {
            // A run this CPU freed warm, if SMASH_RUN_CACHE_MIB is on: its
            // pages hold what the last block wrote.
            mem = vm_region_->takeCachedRun(num_pages, /*node=*/-1, kPageSize);
            if (!mem) mem = vm_region_->allocateLargePages(num_pages, &fresh);
            if (mem && page_states_) {
                size_t idx = vm_region_->pageIndex(reinterpret_cast<uintptr_t>(mem));
                for (uint32_t p = 0; p < num_pages; ++p)
                    page_states_->set(idx + p, PageState::ACTIVE);
            }
        }

        // Fallback to direct mmap. When the caller asked for a compressible
        // block (at or above the threshold) and the arena could not serve it,
        // say so once the block exists (below): this memory is never walked by
        // the compressor, so it stays resident for the life of the process
        // however cold it gets, and nothing else in the process reports that.
        // It is the failure mode that matters most for a workload of multi-MB
        // or GB blocks, where a few allocations can exhaust the arena outright.
        const bool arena_missed = !mem && vm_region_ && alignment <= kPageSize &&
                                  alloc_size >= largeAllocVmThreshold();
        if (!mem) {
            fresh = true;
            if (alignment <= kPageSize) {
                mem = vm::mapPages(map_bytes);
            } else {
                // Over-allocate by alignment to guarantee aligned region.
                // SmashHeap::memalign keeps alignment a power of two no larger
                // than SIZE_MAX / 2 + 1, so this cannot wrap; checked anyway,
                // as the node path does: a wrapped map_size is a small
                // mapping, and the trim below then unmapped what follows it.
                size_t map_size = alloc_size + alignment;
                if (map_size < alloc_size) { errno = ENOMEM; return nullptr; }
                mem = vm::mapPages(map_size);
                if (!mem) return nullptr;

                auto addr = reinterpret_cast<uintptr_t>(mem);
                auto aligned_addr = roundUp(addr, alignment);
                size_t prefix = aligned_addr - addr;
                size_t suffix = map_size - prefix - alloc_size;

                if (prefix > 0) vm::unmapPages(mem, prefix);
                if (suffix > 0) vm::unmapPages(reinterpret_cast<void*>(aligned_addr + alloc_size), suffix);

                mem = reinterpret_cast<void*>(aligned_addr);
            }
        }

        if (!mem) return nullptr;

        if (!span) span = newSpanDescriptor();
        if (!span) {   // metadata arena exhausted (it reports that itself)
            if (vm_region_ && vm_region_->inContigArena(reinterpret_cast<uintptr_t>(mem)))
                vm_region_->releasePages(mem, num_pages);
            else vm::unmapPages(mem, map_bytes);
            return nullptr;
        }
        span->initLarge(mem, size, num_pages, arena_id);
        span->block_cacheable = cacheable;

        // O(1) registration: only register the first page. The Span contains
        // page_count so we can compute the full range from just the base.
        // PageMap::set uses atomic stores internally - no external lock needed.
        page_map_->set(reinterpret_cast<uintptr_t>(mem), span);
        if (arena_missed) [[unlikely]] warnLargeAllocNotCompressible(alloc_size);
        if (zeroed) *zeroed = fresh;
        return mem;
    }

#if defined(__linux__)
    // NUMA-bound allocation (malloc_numa). Pages come from the VmRegion's
    // arena for `node` (VmRegion::allocatePagesOnNode), so the compressor
    // tracks them like any other large allocation, and deallocate() returns
    // them to that arena. With no VmRegion, or one that is exhausted, the
    // block is a separate bound mapping, which is never compressed.
    // `alignment` is a power of two; anything below a page is raised to one,
    // since node blocks are page-granular in both paths anyway.
    void* allocateOnNode(size_t size, uint32_t node, size_t alignment = kPageSize) {
        if (alignment < kPageSize) alignment = kPageSize;
        size_t alloc_size = roundUp(size, kPageSize);
        if (alloc_size < size || alloc_size / kPageSize > UINT32_MAX) {
            errno = ENOMEM;
            return nullptr;
        }
        const auto num_pages = static_cast<uint32_t>(alloc_size / kPageSize);

        void* mem = nullptr;
        if (vm_region_) {
            // A run this CPU freed warm on this node, if SMASH_RUN_CACHE_MIB is
            // on; it skips the node arena's lock entirely.
            mem = vm_region_->takeCachedRun(num_pages, static_cast<int>(node), alignment);
            if (!mem) {
                mem = vm_region_->allocatePagesOnNodeAligned(num_pages, node, alignment);
                // Any failure but exhaustion (bad node, mbind refused) would
                // fail the same way on a separate mapping.
                if (!mem && errno != ENOMEM) return nullptr;
            }
            if (mem && page_states_) {
                size_t idx = vm_region_->pageIndex(reinterpret_cast<uintptr_t>(mem));
                for (uint32_t p = 0; p < num_pages; ++p)
                    page_states_->set(idx + p, PageState::ACTIVE);
            }
        }
        const bool arena_missed = !mem && vm_region_;   // warned once the block exists, below
        if (!mem) {
            // Separate bound mapping: never compressed. Over-map and trim for
            // an alignment the mapping itself does not guarantee, exactly as
            // the non-NUMA aligned path does, and bind only what is kept —
            // binding before the trim would mbind pages about to be unmapped.
            if (alignment <= kPageSize) {
                mem = vm::mapPages(alloc_size);
                if (!mem) return nullptr;
            } else {
                const size_t map_size = alloc_size + alignment;
                if (map_size < alloc_size) { errno = ENOMEM; return nullptr; }
                void* raw = vm::mapPages(map_size);
                if (!raw) return nullptr;
                const auto addr = reinterpret_cast<uintptr_t>(raw);
                const auto aligned = roundUp(addr, alignment);
                const size_t prefix = aligned - addr;
                const size_t suffix = map_size - prefix - alloc_size;
                if (prefix > 0) vm::unmapPages(raw, prefix);
                if (suffix > 0)
                    vm::unmapPages(reinterpret_cast<void*>(aligned + alloc_size), suffix);
                mem = reinterpret_cast<void*>(aligned);
            }
            if (!vm::bindPagesToNode(mem, alloc_size, node)) {
                int err = errno;
                vm::unmapPages(mem, alloc_size);
                errno = err;
                return nullptr;
            }
        }

        // Arena 0: call-site arenas group slab pages by allocation site, and a
        // whole-page NUMA block has no slab.
        Span* span = newSpanDescriptor();
        if (!span) {   // metadata arena exhausted (it reports that itself)
            // Match deallocate()'s routing: unmapping an arena block would
            // punch a hole in the region's own reservation.
            if (vm_region_ &&
                vm_region_->inContigArena(reinterpret_cast<uintptr_t>(mem)))
                vm_region_->releasePages(mem, num_pages);
            else
                vm::unmapPages(mem, alloc_size);
            return nullptr;
        }
        span->initLarge(mem, size, num_pages);
        page_map_->set(reinterpret_cast<uintptr_t>(mem), span);
        if (arena_missed) [[unlikely]] warnNumaAllocNotCompressible(alloc_size, node, alignment);
        return mem;
    }
#endif

    // realloc: resize `span`'s block where it is. Shrinking by whole pages
    // releases the tail the way a free would (release hook, sub-arena, run
    // cache, decommit queue -- or munmap outside the arena); growing takes
    // the pages behind the block if they are free (VmRegion::tryExtendRun,
    // or an in-place mremap outside the arena); a size that fits the pages
    // the block has just changes its size. False, with nothing changed, when
    // the block cannot be resized where it is: the caller copies, as every
    // realloc of a smash block used to.
    bool resizeInPlace(Span* span, size_t new_size) {
        const size_t new_bytes = roundUp(new_size, kPageSize);
        if (new_bytes < new_size || new_bytes / kPageSize > UINT32_MAX) return false;
        uint32_t new_pages = static_cast<uint32_t>(new_bytes / kPageSize);
        // A cacheable block keeps its size on the cache's grid: within the
        // grid step the block just changes size, and a resize that moves it
        // takes the whole step, so the block can still be cached when freed.
        if (span->block_cacheable && new_pages <= LargeBlockCache::kMaxPages)
            new_pages = static_cast<uint32_t>(LargeBlockCache::roundPages(new_pages));
        const uint32_t old_pages = span->page_count;
        auto* base = static_cast<char*>(span->base);
        const bool arena = vm_region_ && vm_region_->inContigArena(reinterpret_cast<uintptr_t>(base));
        if (new_pages == old_pages) {
            span->large_size = new_size;
            return true;
        }
        if (new_pages < old_pages) {
            if (arena && vm_region_->runIsChunkBig(base)) return false;
            if (arena) releaseArenaRun(base + size_t{new_pages} * kPageSize, old_pages - new_pages);
            else vm::unmapPages(base + size_t{new_pages} * kPageSize, size_t{old_pages - new_pages} * kPageSize);
            span->page_count = new_pages;
            span->large_size = new_size;
            return true;
        }
        const uint32_t extra = new_pages - old_pages;
        if (arena) {
            if (!vm_region_->tryExtendRun(base, old_pages, extra)) return false;
            if (page_states_) {
                const size_t idx = vm_region_->pageIndex(reinterpret_cast<uintptr_t>(base));
                for (uint32_t p = old_pages; p < new_pages; ++p)
                    page_states_->set(idx + p, PageState::ACTIVE);
            }
        } else {
#if defined(__linux__)
            // In place only: a moved mapping would need the page map and
            // the span re-pointed, which the copy path already does.
            void* r = mremap(base, size_t{old_pages} * kPageSize, new_bytes, 0);
            if (r == MAP_FAILED || r != base) return false;
#else
            return false;
#endif
        }
        span->page_count = new_pages;
        span->large_size = new_size;
        return true;
    }

    void deallocate(Span* span) {
        void* base = span->base;
        size_t num_pages = span->page_count;

        // O(1) clear: only clear the first page (matches O(1) registration).
        // Must clear BEFORE releasing pages to prevent race with reallocation.
        page_map_->set(reinterpret_cast<uintptr_t>(base), nullptr);

        const bool cacheable = span->block_cacheable &&
                               num_pages <= LargeBlockCache::kMaxPages &&
                               LargeBlockCache::roundPages(num_pages) == num_pages;

        // Placement test, NOT contains(): under SMASH_TRACK_EXTERNAL,
        // contains() also matches externally-TRACKED pages (including this
        // allocator's own direct-mmap fallback pages if they ever get
        // registered). Feeding a non-arena address to releasePages() poisons
        // the arena free list with an external page index that allocatePages
        // later re-mints as base_+idx*kPageSize — past the arena end (#84).
        // Only pages actually placed in the contiguous arena go back to it.
        if (vm_region_ && vm_region_->inContigArena(reinterpret_cast<uintptr_t>(base))) {
            releaseArenaRun(base, num_pages);
        } else if (cacheable && directByDesign(num_pages * kPageSize) &&
                   cache_.push(span, num_pages, tick_.load(std::memory_order_relaxed))) {
            return;   // cached with its descriptor
        } else {
            vm::unmapPages(base, num_pages * kPageSize);
        }

        recycleSpanDescriptor(span);
    }

    // An arena run of a freed block, or the tail a realloc shrank off one.
    void releaseArenaRun(void* base, size_t num_pages) {
        bool reusable = true;   // no compressor: nothing ever changed protection
        if (release_hook_) {
            size_t idx = vm_region_->pageIndex(reinterpret_cast<uintptr_t>(base));
            reusable = release_hook_(idx, num_pages, release_ctx_);
        }
        // A never-compressed run of a sub-arena chunk goes straight back
        // to its sub-arena, dirty (SMASH_SUBARENAS); otherwise the per-CPU
        // run cache may park it (SMASH_RUN_CACHE_MIB); anything else goes
        // the ordinary way, which restores protection and decommits.
        if (!(reusable && vm_region_->freeRunDirty(base, num_pages)) &&
            !vm_region_->cacheFreedRun(base, num_pages, reusable))
            vm_region_->releasePages(base, num_pages, reusable);
    }

    size_t getSize(Span* span) const {
        return span->large_size;
    }

    void lockAlloc() SMASH_ACQUIRE(lock_) { lock_.lock(); }
    void unlockAlloc() SMASH_RELEASE(lock_) { lock_.unlock(); }
    void forkLocks(ForkLockSet& s) { s.add(lock_); cache_.forkLocks(s); }
};

} // namespace smash
