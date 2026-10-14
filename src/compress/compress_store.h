// smash/src/compress/compress_store.h - Storage for compressed page blobs
//
// Backed by its own mmap regions (NOT VmRegion — avoids recursive compression).
// Exact-size bump allocator with 16-byte alignment.  No power-of-2 rounding.
// Sharded by page index to reduce lock contention across compressor workers.
//
// Region reclamation: each region tracks live bytes.  When live_bytes drops
// to zero the region's data pages are decommitted (MADV_FREE_REUSABLE on
// macOS, MADV_DONTNEED on Linux), releasing physical memory back to the OS
// while keeping the virtual mapping.  If the region is reused, the OS
// zero-fills on access and the bump allocator is reset.
//
// Blobs are only ever bumped; a freed blob's bytes are never reused in place.
// Compaction gets them back instead: markDraining() marks each region whose
// live bytes have fallen under a share of its used bytes; nothing new is
// stored in a draining region, and the compressor moves its live blobs out
// (storeMoved(): copy into this store, repoint the page under its page lock,
// release the old one) until the region is empty and decommitted.
//
// Until 2026-10 freed blobs went on a per-region free list that every store
// searched first-fit under the shard lock: on a churn workload that was ~500
// slots searched and 14 us held per store (longer than an LZ4 compression),
// with the fault path's release waiting up to 18 ms behind it; bump-only, the
// lock is held 1.4 us. A free list also kept regions a few live blobs among
// holes (1.6 GiB live holding 15.7 GiB on a 128-core application).
//
// Moved blobs go to a region of their own per shard (`survivor`), apart from
// new ones: a blob that survived its region draining is likely long-lived,
// and back among new blobs it was moved again each time their region
// drained.
#pragma once

#include "smash/config.h"
#include "../vm/platform_mem.h"
#include "../util/spinlock.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace smash {

class CompressStore {
    // Pin mode for CompressStore regions — prevents the kernel from evicting
    // compressed blobs under cgroup memory pressure.  Without pinning, a
    // cgroup MemoryHigh cap pushes CompressStore pages into swap, and
    // decompression then requires swap-in + decompress (double-fault latency).
    //
    // Modes:
    //   kOff       — no pinning (default when not under cgroup pressure)
    //   kMlock     — mlock() each region; hard pin, immune to reclaim
    //   kWillNeed  — madvise(MADV_WILLNEED) after each store(); soft hint
    enum class PinMode : uint8_t { kOff = 0, kMlock = 1, kWillNeed = 2 };

    PinMode pin_mode_ = PinMode::kOff;
    int node_ = -1;  // NUMA node every region is bound to, or -1 for none
    unsigned drain_pct_ = 50;  // SMASH_STORE_DRAIN_PCT

    static PinMode detectPinMode() {
        const char* v = std::getenv("SMASH_MLOCK_STORE");
        if (v) {
            if (v[0] == '0') return PinMode::kOff;
            if (v[0] == '2') return PinMode::kWillNeed;
            return PinMode::kMlock;  // "1" or any truthy value
        }
        return PinMode::kOff;
    }

    // Regions for blob storage (own mmap, not from VmRegion or bootstrap)
    struct Region {
        char* base;
        size_t capacity;
        std::atomic<size_t> offset;
        std::atomic<size_t> live_bytes;  // bytes currently in use (not freed)
        Region* next;
        CompressStore* owner;            // the store that mapped this region
        uint32_t shard;                  // the shard whose chain holds it
        // Set by markDraining() under the shard lock, cleared when the region
        // empties; read without it by draining() (a hint, rechecked by the
        // caller under the page lock).
        std::atomic<bool> draining;
    };

    static constexpr size_t kRegionSize = 16 * 1024 * 1024;  // 16MB per region
    // Data starts after the header, rounded up to 16-byte alignment.
    static constexpr size_t kDataStart = (sizeof(Region) + 15) & ~15ULL;

    // Per-shard state: independent lock and region chain
    // `current` heads the chain of every region the shard has mapped (only
    // ever prepended to, so usage() can walk it without the lock); `bump` is
    // the region new blobs are bumped into and `survivor` the one moved blobs
    // are, each nullptr when the next store must pick one: a region that
    // emptied, else a new one. The lock guards both, every region's offset,
    // and resetting a region.
    struct Shard {
        Region* current = nullptr;
        Region* bump = nullptr;
        Region* survivor = nullptr;
        Spinlock lock;
    };

    // A region drains once it has used at least this much and its live bytes
    // are under drain_pct_ of what it has used.
    static constexpr size_t kDrainMinUsed = 1u << 20;

    Shard shards_[kCompressStoreShards];

    // Makes r the head of shard's chain, release-ordered after r's fields and
    // r->next are written: usage() walks the chain without the lock.
    static void publish(Shard& shard, Region* r) {
        if (r) __atomic_store_n(&shard.current, r, __ATOMIC_RELEASE);
    }

    // Allocate a kRegionSize-aligned region so regionOf() can derive
    // the Region* from any interior pointer via address masking.
    Region* newRegion(uint32_t shard) {
        // Over-allocate to guarantee alignment: request 2x and trim.
        void* raw = vm::mapPages(kRegionSize * 2);
        if (!raw) return nullptr;
        auto raw_addr = reinterpret_cast<uintptr_t>(raw);
        auto aligned_addr = (raw_addr + kRegionSize - 1) & ~(kRegionSize - 1);

        // Unmap the prefix and suffix outside the aligned region.
        size_t prefix = aligned_addr - raw_addr;
        if (prefix > 0)
            vm::unmapPages(raw, prefix);
        size_t suffix = (raw_addr + kRegionSize * 2) - (aligned_addr + kRegionSize);
        if (suffix > 0)
            vm::unmapPages(reinterpret_cast<void*>(aligned_addr + kRegionSize), suffix);

#if defined(__linux__)
        // Bind before the header below is written, so every page of a node
        // store lands on its node whichever thread touches it first.
        if (node_ >= 0 &&
            !vm::bindPagesToNode(reinterpret_cast<void*>(aligned_addr), kRegionSize,
                                 static_cast<uint32_t>(node_))) {
            vm::unmapPages(reinterpret_cast<void*>(aligned_addr), kRegionSize);
            return nullptr;
        }
#endif
        auto* r = reinterpret_cast<Region*>(aligned_addr);
        r->base = reinterpret_cast<char*>(aligned_addr);
        r->capacity = kRegionSize;
        r->offset.store(kDataStart, std::memory_order_relaxed);
        r->live_bytes.store(0, std::memory_order_relaxed);
        r->next = nullptr;
        r->owner = this;
        r->shard = shard;
        r->draining.store(false, std::memory_order_relaxed);

        if (pin_mode_ == PinMode::kMlock) {
            if (!vm::lockPages(r->base, kRegionSize)) {
                // mlock failed (RLIMIT_MEMLOCK too low) — degrade to willneed
                pin_mode_ = PinMode::kWillNeed;
            }
        }
        return r;
    }

    // Find which region a pointer belongs to.
    // Works because regions are kRegionSize-aligned.
    static Region* regionOf(void* ptr) {
        auto addr = reinterpret_cast<uintptr_t>(ptr);
        return reinterpret_cast<Region*>(addr & ~(kRegionSize - 1));
    }

    // Reset a fully-empty region: unlock and decommit data pages.
    // Caller must hold the shard lock.
    void resetRegion(Region* r) {
        size_t page_size = static_cast<size_t>(sysconf(_SC_PAGESIZE));
        size_t decommit_start = (kDataStart + page_size - 1) & ~(page_size - 1);
        if (decommit_start < r->capacity) {
            if (pin_mode_ == PinMode::kMlock)
                vm::unlockPages(r->base + decommit_start,
                                r->capacity - decommit_start);
            vm::decommitPages(r->base + decommit_start,
                              r->capacity - decommit_start);
        }
        r->offset.store(kDataStart, std::memory_order_relaxed);
        r->draining.store(false, std::memory_order_relaxed);
        // live_bytes is already 0
    }

    static bool isEmpty(const Region* r) {
        return r->live_bytes.load(std::memory_order_relaxed) == 0 &&
               r->offset.load(std::memory_order_relaxed) == kDataStart;
    }

    static void* bumpInto(Region* r, size_t size, size_t* out_alloc_size) {
        const size_t off = r->offset.load(std::memory_order_relaxed);
        const size_t aligned = (off + 15) & ~15ULL;  // 16-byte alignment
        if (aligned + size > r->capacity) return nullptr;
        r->offset.store(aligned + size, std::memory_order_relaxed);
        r->live_bytes.fetch_add(size, std::memory_order_relaxed);
        *out_alloc_size = size;
        return r->base + aligned;
    }

    // A region for one of the shard's two bump slots: an emptied region the
    // other slot is not using (a draining region is never empty: reset clears
    // the mark), else a new one.
    Region* takeRegion(Shard& shard, uint32_t idx) SMASH_REQUIRES(shard.lock) {
        for (Region* scan = shard.current; scan; scan = scan->next)
            if (scan != shard.bump && scan != shard.survivor && isEmpty(scan)) return scan;
        Region* nr = newRegion(idx);
        if (!nr) return nullptr;
        nr->next = shard.current;
        publish(shard, nr);
        return nr;
    }

    // Bump into `slot` (the shard's bump or survivor region); when it is full
    // or unset, into the region takeRegion() gives it.
    void* bumpAlloc(Shard& shard, Region*& slot, size_t size, size_t* out_alloc_size)
            SMASH_REQUIRES(shard.lock) {
        if (slot)
            if (void* p = bumpInto(slot, size, out_alloc_size)) return p;
        Region* r = takeRegion(shard, static_cast<uint32_t>(&shard - shards_));
        if (!r) return nullptr;
        slot = r;
        return bumpInto(r, size, out_alloc_size);
    }

    void* storeInto(bool moved, const void* data, size_t size, size_t* alloc_size,
                    size_t page_idx) {
        const size_t aligned_size = (size + 15) & ~15ULL;
        Shard& shard = shards_[page_idx % kCompressStoreShards];
        LockGuard guard(shard.lock);
        size_t actual_size = 0;
        void* ptr = bumpAlloc(shard, moved ? shard.survivor : shard.bump, aligned_size,
                              &actual_size);
        if (ptr) {
            __builtin_memcpy(ptr, data, size);
            *alloc_size = actual_size;
            if (pin_mode_ == PinMode::kWillNeed)
                vm::willNeedPages(ptr, actual_size);
        } else {
            *alloc_size = 0;
        }
        return ptr;
    }

public:
    // node >= 0 binds every region of this store to that NUMA node (Linux).
    void init(int node = -1) {
        node_ = node;
        pin_mode_ = detectPinMode();
        if (const char* v = std::getenv("SMASH_STORE_DRAIN_PCT")) {
            const int x = std::atoi(v);
            drain_pct_ = x < 0 ? 0u : (x > 100 ? 100u : static_cast<unsigned>(x));
        }
        for (int s = 0; s < kCompressStoreShards; ++s) {
            Region* r = newRegion(static_cast<uint32_t>(s));
            shards_[s].bump = r;
            publish(shards_[s], r);
        }
    }

    // Memory held by this store's blobs, for the SIGUSR2 stats line. Takes no
    // lock, so it is async-signal-safe: a region is only ever prepended to its
    // shard's chain (publish(), after it is fully initialised) and never
    // unlinked or unmapped, so a walk from an acquire load of the head sees
    // initialised regions only. `live` is the bytes of the blobs stored now;
    // `held` is the bytes below each region's bump pointer, which is what the
    // region keeps resident: live plus the holes freed blobs left (a reset
    // region, decommitted, holds only its header). The counts are racy by the blobs
    // stored or released during the walk, which is fine for a dump.
    void usage(size_t* live, size_t* held, size_t* regions, size_t* draining = nullptr) const {
        for (const Shard& shard : shards_) {
            for (const Region* r = __atomic_load_n(&shard.current, __ATOMIC_ACQUIRE); r;
                 r = r->next) {
                *live += r->live_bytes.load(std::memory_order_relaxed);
                *held += r->offset.load(std::memory_order_relaxed);
                ++*regions;
                if (draining && r->draining.load(std::memory_order_relaxed)) ++*draining;
            }
        }
    }

    // Marks every region that has used at least kDrainMinUsed and holds less
    // than `pct` percent of it live (SMASH_STORE_DRAIN_PCT, default 50; 0 =
    // never). A draining bump or survivor region stops being one. Returns
    // the number of regions draining now. The caller then moves their blobs
    // out: for each page whose blob is in one (draining()), under the page
    // lock, storeMoved() a copy, repoint the page, release the old blob.
    size_t markDraining(unsigned pct) {
        size_t n = 0;
        for (Shard& shard : shards_) {
            LockGuard guard(shard.lock);
            for (Region* r = shard.current; r; r = r->next) {
                if (r->draining.load(std::memory_order_relaxed)) { ++n; continue; }
                const size_t used = r->offset.load(std::memory_order_relaxed) - kDataStart;
                const size_t live = r->live_bytes.load(std::memory_order_relaxed);
                if (pct == 0 || live == 0 || used < kDrainMinUsed || live * 100 >= used * pct)
                    continue;
                r->draining.store(true, std::memory_order_relaxed);
                if (shard.bump == r) shard.bump = nullptr;
                if (shard.survivor == r) shard.survivor = nullptr;
                ++n;
            }
        }
        return n;
    }
    size_t markDraining() { return markDraining(drain_pct_); }

    // Whether the region holding `blob` (a pointer this store type returned)
    // is draining. Lock-free; a hint until rechecked under the page lock.
    static bool draining(const void* blob) {
        return regionOf(const_cast<void*>(blob))->draining.load(std::memory_order_relaxed);
    }
    // The store that mapped the region holding `blob`.
    static CompressStore* ownerOf(const void* blob) {
        return regionOf(const_cast<void*>(blob))->owner;
    }

    // Store a compressed blob. page_idx selects the shard. Returns pointer
    // to stored data and the allocation size (the 16-byte-aligned size), which
    // the caller MUST pass back to release() so live_bytes stays balanced.
    void* store(const void* data, size_t size, size_t* alloc_size,
                size_t page_idx = 0) {
        return storeInto(false, data, size, alloc_size, page_idx);
    }

    // store() for a blob moved out of a draining region: it goes to the
    // shard's survivor region, never among new blobs.
    void* storeMoved(const void* data, size_t size, size_t* alloc_size,
                     size_t page_idx = 0) {
        return storeInto(true, data, size, alloc_size, page_idx);
    }

    // Release a previously stored blob.  `alloc_size` MUST be the value
    // returned by the matching store(). Takes no lock unless the region
    // empties: the fault path releases a blob for every page it restores.
    void release(void* ptr, size_t alloc_size, size_t /*page_idx*/ = 0) {
        if (!ptr) return;

        // Route by the region itself, never by the caller's idea of which store
        // holds the blob. With a store per NUMA node, a page compressed by one
        // node's worker can be released through another store's lookup;
        // resetting a region under the wrong store's shard lock would race
        // that store's bumps into it. The region knows its owner and shard,
        // so a stale lookup costs one extra call and nothing else.
        Region* r = regionOf(ptr);
        if (r->owner && r->owner != this)
            return r->owner->release(ptr, alloc_size);

        if (r->live_bytes.fetch_sub(alloc_size, std::memory_order_acq_rel) > alloc_size)
            return;
        // The region looked empty. Under the lock, which every bump into it
        // holds, check again: a blob stored since keeps it, and of two
        // releases that both saw it empty only the first resets it.
        Shard& shard = shards_[r->shard];
        LockGuard guard(shard.lock);
        if (r->live_bytes.load(std::memory_order_relaxed) == 0 &&
            r->offset.load(std::memory_order_relaxed) != kDataStart)
            resetRegion(r);
    }
};

} // namespace smash
