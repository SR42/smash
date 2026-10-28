// smash/src/core/large_alloc.h - Large allocations (> kMaxSmallSize)
//
// When a VmRegion is available, large allocations are placed there so the
// compressor thread can track and compress their pages.  Falls back to
// direct mmap for oversized alignments or when the VmRegion is full.
#pragma once

#include "smash/config.h"
#include "span.h"
#include "page_map.h"
#include "../util/spinlock.h"
#include "../util/bitops.h"
#include "../vm/platform_mem.h"
#include "../vm/vm_region.h"
#include "../vm/page_state.h"

#include <cstddef>
#include <cstdint>

namespace smash {

class LargeAlloc {
    Spinlock lock_;
    PageMap* page_map_;
    VmRegion* vm_region_ = nullptr;
    PageStateTable* page_states_ = nullptr;
    bool (*release_hook_)(size_t, size_t, void*) = nullptr;
    void* release_ctx_ = nullptr;

public:
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

    void* allocate(size_t size, size_t alignment, uint8_t arena_id = 0) {
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

        // Route large-enough allocations through VmRegion for compression
        if (vm_region_ && alignment <= kPageSize && alloc_size >= largeAllocVmThreshold()) {
            // A run this CPU freed warm, if SMASH_RUN_CACHE_MIB is on.
            mem = vm_region_->takeCachedRun(num_pages, /*node=*/-1, kPageSize);
            if (!mem) mem = vm_region_->allocateLargePages(num_pages);
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
            if (alignment <= kPageSize) {
                mem = vm::mapPages(alloc_size);
            } else {
                // Over-allocate by alignment to guarantee aligned region
                size_t map_size = alloc_size + alignment;
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

        Span* span = newSpanDescriptor();
        if (!span) {   // metadata arena exhausted (it reports that itself)
            if (vm_region_ && vm_region_->inContigArena(reinterpret_cast<uintptr_t>(mem)))
                vm_region_->releasePages(mem, num_pages);
            else vm::unmapPages(mem, alloc_size);
            return nullptr;
        }
        span->initLarge(mem, size, num_pages, arena_id);

        // O(1) registration: only register the first page. The Span contains
        // page_count so we can compute the full range from just the base.
        // PageMap::set uses atomic stores internally - no external lock needed.
        page_map_->set(reinterpret_cast<uintptr_t>(mem), span);
        if (arena_missed) [[unlikely]] warnLargeAllocNotCompressible(alloc_size);
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

    void deallocate(Span* span) {
        void* base = span->base;
        size_t num_pages = span->page_count;

        // O(1) clear: only clear the first page (matches O(1) registration).
        // Must clear BEFORE releasing pages to prevent race with reallocation.
        // No generation bump: large spans never enter free()'s TLS span
        // cache, and the bump was one global cache line every large free
        // wrote (see g_pagemap_generation).
        page_map_->set(reinterpret_cast<uintptr_t>(base), nullptr, /*bump=*/false);

        // Placement test, NOT contains(): under SMASH_TRACK_EXTERNAL,
        // contains() also matches externally-TRACKED pages (including this
        // allocator's own direct-mmap fallback pages if they ever get
        // registered). Feeding a non-arena address to releasePages() poisons
        // the arena free list with an external page index that allocatePages
        // later re-mints as base_+idx*kPageSize — past the arena end (#84).
        // Only pages actually placed in the contiguous arena go back to it.
        if (vm_region_ && vm_region_->inContigArena(reinterpret_cast<uintptr_t>(base))) {
            bool reusable = true;   // no compressor: nothing ever changed protection
            if (release_hook_) {
                size_t idx = vm_region_->pageIndex(reinterpret_cast<uintptr_t>(base));
                reusable = release_hook_(idx, num_pages, release_ctx_);
            }
            // A run with no compressed page may be parked for reuse by this
            // CPU (SMASH_RUN_CACHE_MIB); anything else goes the ordinary way,
            // which restores protection and decommits.
            // A never-compressed run of a sub-arena chunk goes straight back
            // to its sub-arena, dirty (SMASH_SUBARENAS); otherwise the per-CPU
            // run cache may park it (SMASH_RUN_CACHE_MIB); anything else goes
            // the ordinary way, which restores protection and decommits.
            if (!(reusable && vm_region_->freeRunDirty(base, num_pages)) &&
                !vm_region_->cacheFreedRun(base, num_pages, reusable))
                vm_region_->releasePages(base, num_pages, reusable);
        } else {
            vm::unmapPages(base, num_pages * kPageSize);
        }

        recycleSpanDescriptor(span);
    }

    size_t getSize(Span* span) const {
        return span->large_size;
    }

    void lockAlloc() SMASH_ACQUIRE(lock_) { lock_.lock(); }
    void unlockAlloc() SMASH_RELEASE(lock_) { lock_.unlock(); }
};

} // namespace smash
