// smash/src/core/thread_cache.h - Per-thread allocation cache
//
// Stores pointers in arrays (NOT inline freelists) to avoid writing
// metadata into user data pages — critical for compression.
#pragma once

#include "smash/config.h"
#include "bootstrap_alloc.h"
#include "size_classes.h"
#include "../util/spinlock.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>

namespace smash {

// Forward declarations
class Slab;
class PageMap;

class ThreadCache {
    // Per-(lane, size-class) cache. The cache has kCacheLanes independent lanes;
    // each arena maps to a lane via (arena % kCacheLanes). With kCacheLanes=4
    // and typical runtime arenas=4-8, most arenas get a dedicated lane (or share
    // with at most one other). This enforces page-level segregation by call site
    // while keeping total footprint bounded.
    //
    // Depth per lane = kThreadCacheMaxPerClass / kCacheLanes (= 4 with defaults).
    // Total = kCacheLanes * kNumClasses * depth * 8 = 64 * 36 * 4 * 8 = 72 KB
    // (same as the old flat kNumClasses * 256 * 8 = 72 KB).
    // Shallow depth (4) trades refill frequency for segregation quality.
    // Overridable: fewer lanes → deeper per-lane cache → far fewer slab-lock
    // acquisitions per allocation (glibc's tcache is ~64 deep and refills rarely;
    // smash's default depth-4 hits the slab lock every 4 allocs — 16× more lock
    // traffic, the structural reason smash scales ~2.7× where glibc scales ~12×).
#ifdef SMASH_CACHE_LANES
    static constexpr int kCacheLanes = SMASH_CACHE_LANES;
#else
    static constexpr int kCacheLanes = 64;
#endif
    static constexpr int kPerLaneDepth = kThreadCacheMaxPerClass / kCacheLanes;
    static_assert(kPerLaneDepth >= 4, "per-lane cache too shallow");
    static_assert((kCacheLanes & (kCacheLanes - 1)) == 0, "kCacheLanes must be power of 2");

    struct ClassCache {
        void* ptrs[kPerLaneDepth];
        uint32_t count;
    };

    // Indexed as caches_[lane * kNumClasses + sc]
    ClassCache caches_[kCacheLanes * kNumClasses];

    static int idx(uint8_t arena, uint8_t sc) {
        int lane = arena & (kCacheLanes - 1);
        return lane * kNumClasses + sc;
    }

public:
    ThreadCache* pool_next;

    ThreadCache() {
        __builtin_memset(caches_, 0, sizeof(caches_));
        pool_next = nullptr;
    }

    // Allocate from the arena-specific bucket for this size class.
    void* allocate(uint8_t sc, uint8_t arena) {
        auto& c = caches_[idx(arena, sc)];
        if (c.count > 0) [[likely]] {
            return c.ptrs[--c.count];
        }
        return nullptr;
    }

    // Legacy: allocate without arena (scans all arenas, returns first hit).
    // Used only by paths that don't have arena context.
    void* allocate(uint8_t sc) {
        for (int a = 0; a < kMaxArenas; ++a) {
            auto& c = caches_[idx(a, sc)];
            if (c.count > 0) return c.ptrs[--c.count];
        }
        return nullptr;
    }

    // Free to the correct arena bucket. Returns false if full.
    bool deallocate(uint8_t sc, uint8_t arena, void* ptr) {
        auto& c = caches_[idx(arena, sc)];
        if (c.count < kPerLaneDepth) [[likely]] {
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
        return caches_[idx(arena, sc)].count >= kPerLaneDepth;
    }

    bool isFull(uint8_t sc) const {
        return isFull(sc, 0);
    }

    bool isEmpty(uint8_t sc, uint8_t arena) const {
        return caches_[idx(arena, sc)].count == 0;
    }

    bool isEmpty(uint8_t sc) const {
        for (int a = 0; a < kMaxArenas; ++a)
            if (caches_[idx(a, sc)].count > 0) return false;
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
    // The arena is required: pointers live in caches_[idx(arena, sc)], so
    // draining without it (the old caches_[sc] form) emptied lane 0 while the
    // caller's full lane was untouched — the retry deallocate then failed and
    // the pointer was silently dropped (leak).
    void drain(uint8_t sc, uint8_t arena, Slab* all_slabs, PageMap* page_map);

    // Drain all classes, routing pointers to their arena's slab.
    void drainAll(Slab* all_slabs, PageMap* page_map);

    // Reset for reuse from pool
    void reset() {
        __builtin_memset(caches_, 0, sizeof(caches_));
        pool_next = nullptr;
    }

    // Expose per-arena depth for batch sizing
    static constexpr int perLaneDepth() { return kPerLaneDepth; }
};

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

    // fork(): see SmashHeap::lock().
    void lockForFork() SMASH_NO_TS_ANALYSIS { lock_.lock(); }
    void unlockForFork() SMASH_NO_TS_ANALYSIS { lock_.unlock(); }

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
};

// Allocate a new ThreadCache from bootstrap memory
inline ThreadCache* newThreadCache() {
    // Try pool first
    ThreadCache* tc = ThreadCachePool::instance().acquire();
    if (tc) return tc;

    // Allocate from bootstrap
    void* mem = BootstrapAlloc::instance().allocate(sizeof(ThreadCache), alignof(ThreadCache));
    return new (mem) ThreadCache();
}

// Return ThreadCache to pool for reuse
inline void returnThreadCache(ThreadCache* tc) {
    tc->reset();
    ThreadCachePool::instance().release(tc);
}

} // namespace smash
