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
    void (*release_hook_)(size_t, size_t, void*) = nullptr;
    void* release_ctx_ = nullptr;

public:
    void init(PageMap* pm,
              VmRegion* vr = nullptr, PageStateTable* ps = nullptr,
              void (*hook)(size_t, size_t, void*) = nullptr,
              void* hook_ctx = nullptr) {
        page_map_ = pm;
        vm_region_ = vr;
        page_states_ = ps;
        release_hook_ = hook;
        release_ctx_ = hook_ctx;
    }

    void* allocate(size_t size, size_t alignment, uint8_t arena_id = 0) {
        if (alignment < kPageSize) alignment = kPageSize;

        // Round size up to page boundary
        size_t alloc_size = roundUp(size, kPageSize);
        uint32_t num_pages = static_cast<uint32_t>(alloc_size / kPageSize);

        void* mem = nullptr;

        // Route large-enough allocations through VmRegion for compression
        if (vm_region_ && alignment <= kPageSize && alloc_size >= largeAllocVmThreshold()) {
            mem = vm_region_->allocatePages(num_pages);
            if (mem && page_states_) {
                size_t idx = vm_region_->pageIndex(reinterpret_cast<uintptr_t>(mem));
                for (uint32_t p = 0; p < num_pages; ++p)
                    page_states_->set(idx + p, PageState::ACTIVE);
            }
        }

        // Fallback to direct mmap
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
        return mem;
    }

#if defined(__linux__)
    // NUMA-bound allocation (malloc_numa). Pages come from the VmRegion's
    // arena for `node` (VmRegion::allocatePagesOnNode), so the compressor
    // tracks them like any other large allocation, and deallocate() returns
    // them to that arena. With no VmRegion, or one that is exhausted, the
    // block is a separate bound mapping, which is never compressed.
    void* allocateOnNode(size_t size, uint32_t node) {
        size_t alloc_size = roundUp(size, kPageSize);
        if (alloc_size < size || alloc_size / kPageSize > UINT32_MAX) {
            errno = ENOMEM;
            return nullptr;
        }
        const auto num_pages = static_cast<uint32_t>(alloc_size / kPageSize);

        void* mem = nullptr;
        if (vm_region_) {
            mem = vm_region_->allocatePagesOnNode(num_pages, node);
            // Any failure but exhaustion (bad node, mbind refused) would fail
            // the same way on a separate mapping.
            if (!mem && errno != ENOMEM) return nullptr;
            if (mem && page_states_) {
                size_t idx = vm_region_->pageIndex(reinterpret_cast<uintptr_t>(mem));
                for (uint32_t p = 0; p < num_pages; ++p)
                    page_states_->set(idx + p, PageState::ACTIVE);
            }
        }
        if (!mem) {
            mem = vm::mapPages(alloc_size);
            if (!mem) return nullptr;
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
            vm::unmapPages(mem, alloc_size);
            return nullptr;
        }
        span->initLarge(mem, size, num_pages);
        page_map_->set(reinterpret_cast<uintptr_t>(mem), span);
        return mem;
    }
#endif

    void deallocate(Span* span) {
        void* base = span->base;
        size_t num_pages = span->page_count;

        // O(1) clear: only clear the first page (matches O(1) registration).
        // Must clear BEFORE releasing pages to prevent race with reallocation.
        page_map_->set(reinterpret_cast<uintptr_t>(base), nullptr);

        // Placement test, NOT contains(): under SMASH_TRACK_EXTERNAL,
        // contains() also matches externally-TRACKED pages (including this
        // allocator's own direct-mmap fallback pages if they ever get
        // registered). Feeding a non-arena address to releasePages() poisons
        // the arena free list with an external page index that allocatePages
        // later re-mints as base_+idx*kPageSize — past the arena end (#84).
        // Only pages actually placed in the contiguous arena go back to it.
        if (vm_region_ && vm_region_->inContigArena(reinterpret_cast<uintptr_t>(base))) {
            if (release_hook_) {
                size_t idx = vm_region_->pageIndex(reinterpret_cast<uintptr_t>(base));
                release_hook_(idx, num_pages, release_ctx_);
            }
            vm_region_->releasePages(base, num_pages);
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
