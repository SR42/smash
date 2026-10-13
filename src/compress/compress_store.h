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
// Compaction: a region keeps its memory while any blob in it is live, and a
// freed blob's hole is reused only by a blob that fits it, so churn leaves
// regions mostly holes (measured on a 128-core application: 1.6 GiB of live
// blobs holding 15.7 GiB). markDraining() marks each region whose live bytes
// have fallen under a share of its used bytes; nothing new is stored in a
// draining region, and the compressor moves its live blobs out (copy into
// this store, repoint the page under its page lock, release the old one)
// until the region is empty and decommitted.
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

    // In-band free-slot header.  When a blob is released we overwrite the
    // first 16 bytes with this header and link the slot into its region's
    // free list.  Future allocations check the free list first before
    // bumping the offset, so re-tier (release-then-allocate within the same
    // shard) reuses the freed slot in place instead of growing the region.
    //
    // Without this, LZ4 → zstd-9 upgrades on memcached produced ~80 MB
    // of stranded LZ4 regions that couldn't drain (a few stranger pages
    // per region kept live_bytes > 0).  See TIERED_RECOMPRESSION.md.
    //
    // Sized to fit within the 16-byte minimum alignment of every blob.
    struct FreeNode {
        uint32_t size;          // slot size in bytes (incl. this header)
        uint32_t next_offset;   // offset of next free slot within region, 0 = end
    };
    static_assert(sizeof(FreeNode) <= 16, "FreeNode must fit minimum-alignment slot");

    // Regions for blob storage (own mmap, not from VmRegion or bootstrap)
    struct Region {
        char* base;
        size_t capacity;
        std::atomic<size_t> offset;
        std::atomic<size_t> live_bytes;  // bytes currently in use (not freed)
        uint32_t free_head;              // offset of first free slot, 0 = none
        Region* next;
        CompressStore* owner;            // the store that mapped this region
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
    // the region new blobs are bumped into, or nullptr when the next store
    // must pick one: a region that emptied, else a new one.
    struct Shard {
        Region* current = nullptr;
        Region* bump = nullptr;
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
    Region* newRegion() {
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
        r->free_head = 0;
        r->next = nullptr;
        r->owner = this;
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
        r->free_head = 0;
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

    // Try to satisfy an allocation from a region's free list (LIFO,
    // first-fit; splits the slot when the remainder is large enough to
    // hold its own free-list header — recovers the unused tail when a
    // smaller blob (e.g. zstd-9, 0.7 KB) is allocated into a slot freed
    // by a larger blob (e.g. LZ4, 1.3 KB).  Caller (bumpAlloc) holds the
    // shard lock.  Returns nullptr if no fitting free slot exists; on
    // success writes the actual returned-slot size into *out_alloc_size.
    static void* tryFreeList(Region* r, size_t requested,
                             size_t* out_alloc_size) {
        if (r->free_head == 0) return nullptr;
        uint32_t prev_off = 0;
        uint32_t cur_off = r->free_head;
        while (cur_off != 0) {
            auto* node = reinterpret_cast<FreeNode*>(r->base + cur_off);
            uint32_t slot_size = node->size;
            if (slot_size >= requested) {
                uint32_t next_off = node->next_offset;
                size_t remainder = slot_size - requested;
                if (remainder >= sizeof(FreeNode)) {
                    // Split: requested bytes go to caller; remainder
                    // stays on the free list at offset cur_off+requested.
                    // requested is 16-byte aligned (enforced in store()),
                    // and so is slot_size (originally allocated 16-byte
                    // aligned, never broken since), so the new node's
                    // offset is also 16-byte aligned.
                    uint32_t rem_off = cur_off +
                        static_cast<uint32_t>(requested);
                    auto* rem_node = reinterpret_cast<FreeNode*>(
                        r->base + rem_off);
                    rem_node->size = static_cast<uint32_t>(remainder);
                    rem_node->next_offset = next_off;
                    if (prev_off == 0) {
                        r->free_head = rem_off;
                    } else {
                        auto* prev = reinterpret_cast<FreeNode*>(
                            r->base + prev_off);
                        prev->next_offset = rem_off;
                    }
                    *out_alloc_size = static_cast<uint32_t>(requested);
                    r->live_bytes.fetch_add(requested,
                        std::memory_order_relaxed);
                    return r->base + cur_off;
                }
                // No-split: hand the whole slot to caller — remainder
                // too small to manage on its own.
                if (prev_off == 0) {
                    r->free_head = next_off;
                } else {
                    auto* prev = reinterpret_cast<FreeNode*>(r->base + prev_off);
                    prev->next_offset = next_off;
                }
                *out_alloc_size = slot_size;
                r->live_bytes.fetch_add(slot_size, std::memory_order_relaxed);
                return r->base + cur_off;
            }
            prev_off = cur_off;
            cur_off = node->next_offset;
        }
        return nullptr;
    }

    // Where a blob goes, in order: a hole in the bump region; the bump
    // region's tail; a hole in any other region not draining; an emptied
    // region, which becomes the bump region (refilled by bump, not handed one
    // blob and passed over again); a new region. Draining regions get nothing.
    void* bumpAlloc(Shard& shard, size_t size, size_t* out_alloc_size)
            SMASH_REQUIRES(shard.lock) {
        Region* b = shard.bump;
        if (b) {
            if (void* p = tryFreeList(b, size, out_alloc_size)) return p;
            if (void* p = bumpInto(b, size, out_alloc_size)) return p;
        }
        Region* empty = nullptr;
        for (Region* scan = shard.current; scan; scan = scan->next) {
            if (scan == b || scan->draining.load(std::memory_order_relaxed)) continue;
            if (scan->free_head != 0) {
                if (void* p = tryFreeList(scan, size, out_alloc_size)) return p;
            } else if (!empty && isEmpty(scan)) {
                empty = scan;
            }
        }
        if (empty) {
            shard.bump = empty;
            return bumpInto(empty, size, out_alloc_size);
        }
        Region* nr = newRegion();
        if (!nr) return nullptr;
        nr->next = shard.current;
        shard.bump = nr;
        void* p = bumpInto(nr, size, out_alloc_size);
        publish(shard, nr);
        return p;
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
            Region* r = newRegion();
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
    // region keeps resident: live plus free-list holes (a reset region,
    // decommitted, holds only its header). The counts are racy by the blobs
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
    // never). A draining bump region stops being one. Returns the number of
    // regions draining now. The caller then moves their blobs out: for each
    // page whose blob is in one (draining()), under the page lock, store a
    // copy here, repoint the page, release the old blob.
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

    // Push a fresh region to the head of each shard's chain.
    // Call before a bulk upgrade to ensure new blobs go into fresh regions
    // while old blob releases drain the old regions (enabling decommit).
    void pushFreshRegions() {
        for (int s = 0; s < kCompressStoreShards; ++s) {
            LockGuard guard(shards_[s].lock);
            Region* nr = newRegion();
            if (nr) {
                nr->next = shards_[s].current;
                shards_[s].bump = nr;
                publish(shards_[s], nr);
            }
        }
    }

    // Store a compressed blob. page_idx selects the shard.
    // Returns pointer to stored data and the actual allocation size — the
    // latter may be larger than the requested size if the allocator
    // returned a free-list slot from a previous release.  Caller MUST pass
    // the exact `alloc_size` back to release() so live_bytes accounting
    // stays balanced.
    void* store(const void* data, size_t size, size_t* alloc_size,
                size_t page_idx = 0) {
        size_t aligned_size = (size + 15) & ~15ULL;
        // Free-list nodes need 8 bytes at the head, and bumped allocations
        // are 16-byte aligned, so any slot we hand out is at least 16
        // bytes.  Enforce here so future free-list pushes always fit.
        if (aligned_size < sizeof(FreeNode)) aligned_size = sizeof(FreeNode);

        int shard_idx = static_cast<int>(page_idx % kCompressStoreShards);
        Shard& shard = shards_[shard_idx];

        LockGuard guard(shard.lock);
        size_t actual_size = 0;
        void* ptr = bumpAlloc(shard, aligned_size, &actual_size);
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

    // Release a previously stored blob.  `alloc_size` MUST be the value
    // returned by the matching store() — the free-list reuses the slot
    // verbatim, so the size must match the slot, not the original
    // requested size.
    void release(void* ptr, size_t alloc_size, size_t page_idx = 0) {
        if (!ptr) return;

        // Route by the region itself, never by the caller's idea of which store
        // holds the blob. With a store per NUMA node, a page compressed by one
        // node's worker can be released through another store's lookup; taking
        // the wrong store's shard lock would leave two stores mutating the same
        // region's free list under different locks — silent corruption that no
        // accounting check would notice. The region knows its owner, so a stale
        // lookup costs one extra call and nothing else.
        Region* r = regionOf(ptr);
        if (r->owner && r->owner != this)
            return r->owner->release(ptr, alloc_size, page_idx);

        int shard_idx = static_cast<int>(page_idx % kCompressStoreShards);
        Shard& shard = shards_[shard_idx];

        LockGuard guard(shard.lock);

        size_t prev = r->live_bytes.fetch_sub(alloc_size, std::memory_order_relaxed);

        if (prev <= alloc_size) {
            // Region is now empty — decommit its data pages.  This also
            // clears free_head, dropping any free-list nodes (their backing
            // memory is about to be decommitted).
            resetRegion(r);
        } else {
            // Region still has live blobs.  Push this slot onto the
            // region's free list so a future allocation can reuse it
            // without bumping the offset.
            uint32_t off = static_cast<uint32_t>(
                static_cast<char*>(ptr) - r->base);
            auto* node = reinterpret_cast<FreeNode*>(ptr);
            node->size = static_cast<uint32_t>(alloc_size);
            node->next_offset = r->free_head;
            r->free_head = off;
        }
    }
};

} // namespace smash
