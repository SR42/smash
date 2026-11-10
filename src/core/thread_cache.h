// smash/src/core/thread_cache.h - Per-thread allocation cache
//
// Stores pointers in arrays (NOT inline freelists) to avoid writing
// metadata into user data pages — critical for compression.
#pragma once

#include "smash/config.h"
#include "bootstrap_alloc.h"
#include "size_classes.h"
#include "../util/spinlock.h"
#include "../util/fork_locks.h"
#include "../util/double_free.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>

namespace smash {

// Forward declarations
class Slab;
class PageMap;

class alignas(64) ThreadCache {
    // Per-(lane, size-class) pointer stacks. One lane per arena: the lane
    // count is the process's arena count (getTotalArenas(): hot and cold
    // arenas both, with SMASH_COLD_ARENA_FEEDBACK), fixed at startup, and the
    // per-class budget kThreadCacheMaxPerClass (256) is split between them,
    // up to a depth of 16:
    //
    //   arenas   4   8  16  32  64  128
    //   depth   16  16  16   8   4    4
    //
    // so a thread holds 256 pointers per class (72 KB) from 16 arenas up,
    // less below (18 KB at 4 arenas, 36 KB at 8), and twice that at 128
    // arenas, where the depth floor of 4 doubles it.
    //
    // A lane must never be shared by two arenas. refill() fills a lane from
    // the asking arena's slab and allocate() hands out whatever the lane
    // holds, so with shared lanes every refill put objects into the other
    // arenas' call sites, not only recycled ones. Measured with 16 fixed
    // lanes (upstream #93) at 32 arenas: 61% of 32 interleaved call sites'
    // objects in their own arena, 81% of pages holding several arenas'
    // objects; at 64 arenas, with half the arenas hot, 40% of the cold
    // objects' pages left RAM instead of 100%.
    //
    // Depth is what buys speed: on random replace (x86-64, 8 CPUs) a
    // malloc+free costs 20 ns at depth 4, 14 ns at 8, 11 ns at 16 and
    // 10 ns at 32 (glibc 17). A large machine, with many arenas, therefore
    // gets a shallow cache.
    //
    // Depth is capped at 16 (kMaxLaneDepthShift) all the same. A lane keeps
    // handing out the slots it holds, and those sit on pages that are going
    // cold: cooling-page closing only keeps the slab from refilling them.
    // test_cooling_close counts the churn allocations that land on such
    // pages: 128 of 38272 at depth 16, 256 at 32, 392 at 64 (its limit is
    // 1%, 382). Without the cap a host with 4 arenas (4 CPUs or fewer) got
    // depth 64 and failed it; 16 is what such a host had with the fixed
    // 16 x 16 layout. The price is the speed of the deeper cache there: at
    // 4 arenas a batch of 64 costs 17.5 ns per malloc+free against 9.0, and
    // random replace 11.7 against 9.3 (agents/performance.md).
    //
    // Layout of one allocation (layout(), the same for every cache of the
    // process): this 64-byte header, then lanes * kNumClasses 2-byte
    // counts (compact, touched on every malloc and free, so they stay in L1;
    // interleaving them with the pointers cost a miss on the count and on
    // the slot), then lanes * kNumClasses stacks of depth pointers.
    //
    // SMASH_CACHE_LANES=N (build option, power of two) caps the lanes, which
    // makes arenas share them again: only for measuring that.
#ifdef SMASH_CACHE_LANES
    static constexpr uint32_t kMaxCacheLanes = SMASH_CACHE_LANES;
#else
    static constexpr uint32_t kMaxCacheLanes = kTotalArenas;
#endif
    static constexpr uint32_t kMinLaneDepthShift = 2;   // depth >= 4
    static constexpr uint32_t kMaxLaneDepthShift = 4;   // depth <= 16
    static_assert((kMaxCacheLanes & (kMaxCacheLanes - 1)) == 0, "SMASH_CACHE_LANES must be a power of 2");
    static_assert(kThreadCacheMaxPerClass >= (1 << kMinLaneDepthShift),
                  "drain buffers are sized kThreadCacheMaxPerClass and must hold a lane");
    static_assert(kThreadCacheMaxPerClass <= 0x8000, "a lane count must fit uint16_t");

public:
    struct Layout {
        uint32_t lanes;         // power of 2
        uint32_t depth_shift;   // depth = 1 << depth_shift
        size_t ptrs_offset;     // from the start of the ThreadCache
        size_t bytes;           // the whole allocation
    };

    static Layout layout() {
        uint32_t lanes = static_cast<uint32_t>(getTotalArenas());
        if (lanes < 1) lanes = 1;
        if (lanes > kMaxCacheLanes) lanes = kMaxCacheLanes;
        // The deepest power of 2 with lanes * depth <= the per-class budget,
        // no deeper than the cap.
        uint32_t shift = kMinLaneDepthShift;
        while (shift < kMaxLaneDepthShift &&
               (size_t{2} << shift) * lanes <= static_cast<size_t>(kThreadCacheMaxPerClass)) ++shift;
        const size_t slots = static_cast<size_t>(lanes) * kNumClasses;
        const size_t ptrs_offset = sizeof(ThreadCache) + ((slots * sizeof(uint16_t) + 63) & ~size_t{63});
        return Layout{lanes, shift, ptrs_offset, ptrs_offset + (slots << shift) * sizeof(void*)};
    }

    ThreadCache* pool_next;

private:
    void** ptrs_;           // stack i is ptrs_[i << depth_shift_ ...]
    uint32_t lane_mask_;    // lanes - 1
    uint32_t depth_;
    uint32_t depth_shift_;
    uint32_t num_stacks_;   // lanes * kNumClasses

    // The counts follow the header (see Layout).
    uint16_t* counts() { return reinterpret_cast<uint16_t*>(this + 1); }
    const uint16_t* counts() const { return reinterpret_cast<const uint16_t*>(this + 1); }

    struct Lane {
        void** ptrs;
        uint16_t& count;
    };
    Lane lane(uint32_t i) { return Lane{ptrs_ + (static_cast<size_t>(i) << depth_shift_), counts()[i]}; }
    uint16_t laneCount(uint32_t i) const { return counts()[i]; }

    uint32_t idx(uint8_t arena, uint8_t sc) const {
        return (arena & lane_mask_) * kNumClasses + sc;
    }

public:
    explicit ThreadCache(const Layout& l) {
        pool_next = nullptr;
        ptrs_ = reinterpret_cast<void**>(reinterpret_cast<char*>(this) + l.ptrs_offset);
        lane_mask_ = l.lanes - 1;
        depth_shift_ = l.depth_shift;
        depth_ = 1u << l.depth_shift;
        num_stacks_ = l.lanes * kNumClasses;
        __builtin_memset(counts(), 0, num_stacks_ * sizeof(uint16_t));
    }

    // Allocate from the arena-specific bucket for this size class.
    void* allocate(uint8_t sc, uint8_t arena) {
        auto c = lane(idx(arena, sc));
        if (c.count > 0) [[likely]] {
            return c.ptrs[--c.count];
        }
        return nullptr;
    }

    // Legacy: allocate without arena (scans all lanes, returns first hit).
    // Used only by paths that don't have arena context.
    void* allocate(uint8_t sc) {
        for (uint32_t ln = 0; ln <= lane_mask_; ++ln) {
            auto c = lane(ln * kNumClasses + sc);
            if (c.count > 0) return c.ptrs[--c.count];
        }
        return nullptr;
    }

    // Free to the correct arena bucket. Returns false if full.
    bool deallocate(uint8_t sc, uint8_t arena, void* ptr) {
        auto c = lane(idx(arena, sc));
        // Opt-in double-free detection (SMASH_DOUBLE_FREE, default off). This
        // is the only place the common `free(p); free(p);` can be caught: both
        // pointers land here and never reach the slab bitmap, so the lane must
        // be scanned. Bounded by the lane depth, but it is the free fast path —
        // hence the cached flag test first, which is all a default build pays.
        if (smash::doubleFreeChecksEnabled()) [[unlikely]] {
            for (uint32_t i = 0; i < c.count; ++i) {
                if (c.ptrs[i] == ptr) {
                    // Swallow it: pushing again would put the same pointer in
                    // the lane twice and hand it out to two callers.
                    if (smash::handleDoubleFree("cache", ptr, i)) return true;
                    break;
                }
            }
        }
        if (c.count < depth_) [[likely]] {
            c.ptrs[c.count++] = ptr;
            return true;
        }
        return false;
    }

    // Legacy deallocate without arena (uses arena 0).
    bool deallocate(uint8_t sc, void* ptr) {
        return deallocate(sc, 0, ptr);
    }

    bool isFull(uint8_t sc, uint8_t arena) const {
        return laneCount(idx(arena, sc)) >= depth_;
    }

    bool isFull(uint8_t sc) const {
        return isFull(sc, 0);
    }

    bool isEmpty(uint8_t sc, uint8_t arena) const {
        return laneCount(idx(arena, sc)) == 0;
    }

    bool isEmpty(uint8_t sc) const {
        for (uint32_t ln = 0; ln <= lane_mask_; ++ln)
            if (laneCount(ln * kNumClasses + sc) > 0) return false;
        return true;
    }

    // Refill this cache for (arena, sc) by batch-allocating from the slab.
    void* refill(uint8_t sc, uint8_t arena, Slab* slab);

    // Legacy refill (for callers that pass the slab directly)
    void* refill(uint8_t sc, Slab* slab) {
        // Determine arena from slab index — not available here, use arena 0
        return refill(sc, 0, slab);
    }

    // Drain the (arena, sc) lane, routing pointers to their arena's slab.
    // The arena is required: pointers live in lane idx(arena, sc), so
    // draining without it (the old lane-0-only form) emptied lane 0 while the
    // caller's full lane was untouched — the retry deallocate then failed and
    // the pointer was silently dropped (leak).
    void drain(uint8_t sc, uint8_t arena, Slab* all_slabs, PageMap* page_map);

    // Drain all classes, routing pointers to their arena's slab.
    void drainAll(Slab* all_slabs, PageMap* page_map);

    // Reset for reuse from pool
    void reset() {
        __builtin_memset(counts(), 0, num_stacks_ * sizeof(uint16_t));
        pool_next = nullptr;
    }

    uint32_t lanes() const { return lane_mask_ + 1; }
    uint32_t laneDepth() const { return depth_; }
};
static_assert(sizeof(ThreadCache) == 64, "the counts start one cache line in");

// Pool of thread caches for reuse across threads
class ThreadCachePool {
    ThreadCache* freelist_ = nullptr;
    Spinlock lock_;

public:
    static ThreadCachePool& instance() {
        alignas(ThreadCachePool) static char buf[sizeof(ThreadCachePool)];
        static ThreadCachePool* inst = new (buf) ThreadCachePool;
        return *inst;
    }

    ThreadCache* acquire() {
        LockGuard guard(lock_);
        ThreadCache* tc = freelist_;
        if (tc) {
            freelist_ = tc->pool_next;
            tc->pool_next = nullptr;
        }
        return tc;
    }

    void release(ThreadCache* tc) {
        LockGuard guard(lock_);
        tc->pool_next = freelist_;
        freelist_ = tc;
    }
    void forkLocks(ForkLockSet& s) { s.add(lock_); }
};

// Allocate a new ThreadCache from bootstrap memory
inline ThreadCache* newThreadCache() {
    // Try pool first
    ThreadCache* tc = ThreadCachePool::instance().acquire();
    if (tc) return tc;

    // Allocate from bootstrap. Returns nullptr when the arena is exhausted;
    // callers must handle that rather than placement-new onto null (which
    // segfaulted at ~12k threads once caches started leaking).
    // Every cache of the process has the same layout (the arena count is
    // fixed at startup), so a pooled one always fits.
    const ThreadCache::Layout l = ThreadCache::layout();
    void* mem = BootstrapAlloc::instance().allocate(l.bytes, alignof(ThreadCache));
    if (!mem) return nullptr;
    return new (mem) ThreadCache(l);
}

// Return ThreadCache to pool for reuse
inline void returnThreadCache(ThreadCache* tc) {
    tc->reset();
    ThreadCachePool::instance().release(tc);
}

} // namespace smash
