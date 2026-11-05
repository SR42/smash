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
#include <cstdlib>
#include <cstdint>

namespace smash {

// ── Direct-mapped block cache ───────────────────────────────────────────────
// Large blocks below kLargeAllocVmThreshold are placed with a direct mmap and
// were munmap'd on free, so a program that churns 16 KiB..1 MiB buffers paid
// an mmap + munmap (+ TLB shootdown + page faults) per malloc/free pair:
// 25-50x slower than glibc, which serves these sizes from its heap. Blocks
// above it (in the VmRegion) were decommitted on free, with the same costs.
// Freed blocks up to 8 MiB now go to a small cache instead and are reused by
// later allocations of the same rounded size.
//
// - Page counts are rounded up to 8 steps per power of two (<= 12.5% extra
//   address space; tail pages a program never touches cost no RSS), so a
//   freed block serves any request that rounds to the same bucket.
// - Bounded: kDepth blocks per bucket and maxBytes() in
//   total; a free that would exceed either unmaps as before.
// - Idle blocks are returned to the OS by purgeIdle(), called from the
//   compressor tick, once a bucket has gone kIdleTicks ticks without a free.
class LargeBlockCache {
public:
    // Blocks up to 8 MiB: direct mappings below kLargeAllocVmThreshold and
    // VmRegion blocks above it.
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

    void lockAll() SMASH_NO_TS_ANALYSIS {
        for (auto& b : buckets_) b.lock.lock();
    }
    void unlockAll() SMASH_NO_TS_ANALYSIS {
        for (auto& b : buckets_) b.lock.unlock();
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
    void (*release_hook_)(size_t, size_t, void*) = nullptr;
    void* release_ctx_ = nullptr;
    LargeBlockCache cache_;
    std::atomic<uint32_t> tick_{0};

public:
    // Compressor tick: advance the cache clock and return idle blocks
    // (VmRegion blocks to the region, direct mappings to the OS).
    void onTick() {
        uint32_t now = tick_.fetch_add(1, std::memory_order_relaxed) + 1;
        cache_.purgeIdle(now, [this](Span* sp, size_t pages) {
            if (vm_region_ && vm_region_->inContigArena(reinterpret_cast<uintptr_t>(sp->base)))
                vm_region_->releasePages(sp->base, pages);
            else
                vm::unmapPages(sp->base, pages * kPageSize);
            recycleSpanDescriptor(sp);
        });
    }

    // Mark a VmRegion block's pages ACTIVE (allocated) for the compressor.
    void markActiveIfVm(void* mem, uint32_t pages) {
        if (!vm_region_ || !page_states_) return;
        if (!vm_region_->inContigArena(reinterpret_cast<uintptr_t>(mem))) return;
        size_t idx = vm_region_->pageIndex(reinterpret_cast<uintptr_t>(mem));
        for (uint32_t p = 0; p < pages; ++p)
            page_states_->set(idx + p, PageState::ACTIVE);
    }
    size_t cachedBytes() const { return cache_.cachedBytes(); }

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

    // *zeroed (optional) is set to true when the block is known to be all
    // zero bytes (a fresh anonymous mapping), so calloc can skip its memset.
    void* allocate(size_t size, size_t alignment, uint8_t arena_id = 0,
                   bool* zeroed = nullptr) {
        if (alignment < kPageSize) alignment = kPageSize;
        if (zeroed) *zeroed = false;

        // Round size up to page boundary
        size_t alloc_size = roundUp(size, kPageSize);
        uint32_t num_pages = static_cast<uint32_t>(alloc_size / kPageSize);

        void* mem = nullptr;
        Span* span = nullptr;   // reused descriptor of a cached block

        // Cacheable sizes (up to LargeBlockCache::kMaxPages, page alignment)
        // are rounded to the cache's size grid so the block can be cached
        // when freed, and are served from the cache first: a block of the
        // same rounded size, else one of the next two larger buckets
        // (<= ~25% extra address space). A cached block may live in the
        // VmRegion (compressible, >= kLargeAllocVmThreshold) or be a direct
        // mapping; its free path checks which.
        const bool cacheable = alignment <= kPageSize && num_pages <= LargeBlockCache::kMaxPages;
        if (cacheable) {
            size_t rounded = LargeBlockCache::roundPages(num_pages);
            span = cache_.pop(rounded);
            for (int up = 0; !span && up < 2; ++up) {
                size_t bigger = LargeBlockCache::nextRounded(rounded);
                if (bigger > LargeBlockCache::kMaxPages) break;
                rounded = bigger;
                span = cache_.pop(rounded);
            }
            if (span) {
                mem = span->base;
                num_pages = static_cast<uint32_t>(rounded);
                markActiveIfVm(mem, num_pages);
            }
        }

        // Route large-enough allocations through VmRegion for compression
        if (!mem && vm_region_ && alignment <= kPageSize && alloc_size >= kLargeAllocVmThreshold) {
            const uint32_t np = cacheable
                ? static_cast<uint32_t>(LargeBlockCache::roundPages(num_pages)) : num_pages;
            mem = vm_region_->allocatePages(np);
            if (mem) {
                num_pages = np;
                markActiveIfVm(mem, num_pages);
            }
        }

        // Direct-mapped cacheable sizes: map the rounded size.
        if (!mem && cacheable) {
            size_t rounded = LargeBlockCache::roundPages(num_pages);
            mem = vm::mapPages(rounded * kPageSize);
            if (mem) {
                num_pages = static_cast<uint32_t>(rounded);
                if (zeroed) *zeroed = true;
            }
        }

        // Fallback to direct mmap
        if (!mem) {
            if (alignment <= kPageSize) {
                mem = vm::mapPages(alloc_size);
                if (mem && zeroed) *zeroed = true;
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

        if (!span) span = newSpanDescriptor();
        span->initLarge(mem, size, num_pages, arena_id);

        // O(1) registration: only register the first page. The Span contains
        // page_count so we can compute the full range from just the base.
        // PageMap::set uses atomic stores internally - no external lock needed.
        page_map_->set(reinterpret_cast<uintptr_t>(mem), span);
        return mem;
    }

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
        const bool cacheable = num_pages <= LargeBlockCache::kMaxPages &&
                               LargeBlockCache::roundPages(num_pages) == num_pages;
        const uint32_t now = tick_.load(std::memory_order_relaxed);
        if (vm_region_ && vm_region_->inContigArena(reinterpret_cast<uintptr_t>(base))) {
            // Leave compressor tracking first (drops any compressed copies,
            // state -> EMPTY), whether the block is cached or released.
            if (release_hook_) {
                size_t idx = vm_region_->pageIndex(reinterpret_cast<uintptr_t>(base));
                release_hook_(idx, num_pages, release_ctx_);
            }
            // Cache instead of releasing: release decommits (madvise + TLB
            // shootdown + zero-fill refault on reuse), which made >= 1 MiB
            // churn ~4x slower than mimalloc on Graviton (__flush_tlb_range
            // 81% of cycles). A page compressed before the free may still be
            // PROT_NONE; restore access as releasePages' drain would.
            if (cacheable && !VmRegion::releaseDebugModes() && cache_.canAccept(num_pages)) {
                if (g_any_page_compressed.load(std::memory_order_relaxed))
                    vm::commitPages(base, num_pages * kPageSize);
                if (cache_.push(span, num_pages, now)) return;
            }
            vm_region_->releasePages(base, num_pages);
        } else if (cacheable && cache_.push(span, num_pages, now)) {
            return;  // cached with its descriptor
        } else {
            vm::unmapPages(base, num_pages * kPageSize);
        }

        recycleSpanDescriptor(span);
    }

    size_t getSize(Span* span) const {
        return span->large_size;
    }

    void lockAlloc() SMASH_ACQUIRE(lock_) { lock_.lock(); cache_.lockAll(); }
    void unlockAlloc() SMASH_RELEASE(lock_) { cache_.unlockAll(); lock_.unlock(); }
};

} // namespace smash
