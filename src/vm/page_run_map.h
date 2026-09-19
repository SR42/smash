#pragma once
// page_run_map.h - coalescing map of free page runs in the VmRegion arena.
//
// Freed arena pages used to go onto 64 per-thread singly-linked lists of
// FreeRun nodes and were never coalesced, so a run could only ever satisfy a
// request no larger than itself: pages freed as 1-page slab spans could not
// hold a 2-page span, and 1 MiB blocks could not hold a 2 MiB one. A program
// moving between sizes kept consuming fresh arena until it spilled past the
// compressible arena for good (test_arena_reuse).
//
// This map coalesces. Every free run is described by boundary tags kept in
// per-page arrays, not in the (decommitted) pages themselves:
//   tag_[start]   length, owner, and the run's links in its size bin
//   end_[last]    start + 1, so the run left of a freed range is found in O(1)
// A freed range merges with a free neighbour on either side if the neighbour
// has the same owner. Owners keep NUMA node arenas apart: 0 is the shared
// arena, n + 1 is node n's arena, and runs of different owners never merge,
// so node-bound pages never leak into shared allocations and vice versa.
//
// Size bins per owner: exact bins for 1..64 pages, then one bin per power of
// two. take() uses a per-owner bitmap of non-empty bins, so it is O(1) apart
// from a first-fit scan inside the one power-of-two bin that may hold runs
// both smaller and larger than the request.
//
// Storage is demand-paged (MAP_NORESERVE) and all-zero means empty: links and
// bin heads store index + 1, so no initialization pass is needed even for a
// 1 TiB arena. Memory cost is 20 bytes per arena page in the worst case of a
// fully fragmented arena, touched only where free runs actually start and end.
//
// One lock guards the whole map. Frees reach it only from VmRegion's decommit
// thread; allocations of arena pages (a new slab span, a large block) are far
// rarer than object allocations.
#include "../util/spinlock.h"
#include "platform_mem.h"

#include <cstddef>
#include <cstdint>
#include <sys/mman.h>

namespace smash {

class PageRunMap {
public:
    static constexpr size_t kNone = ~size_t{0};
    static constexpr uint32_t kOwners = vm::kMaxNumaNodes + 1;

    bool init(size_t num_pages) {
        num_pages_ = num_pages;
        tags_ = static_cast<Tag*>(mapZeroed(num_pages * sizeof(Tag)));
        ends_ = static_cast<uint32_t*>(mapZeroed(num_pages * sizeof(uint32_t)));
        bins_ = static_cast<OwnerBins*>(mapZeroed(kOwners * sizeof(OwnerBins)));
        return tags_ && ends_ && bins_;
    }

    // Add the free run [start, start + len) owned by `owner`, merging with
    // free neighbours of the same owner.
    void insert(size_t start, size_t len, uint32_t owner) {
        if (len == 0) return;
        LockGuard guard(lock_);
        // Left neighbour: a free run of this owner ending at start - 1.
        if (start > 0) {
            const uint32_t t = ends_[start - 1];
            if (t) {
                const size_t s = t - 1;
                if (isRunStart(s) && s + tags_[s].len == start && tags_[s].owner == owner) {
                    const size_t l = tags_[s].len;
                    unlink(s);
                    start = s;
                    len += l;
                }
            }
        }
        // Right neighbour: a free run of this owner starting right after.
        const size_t r = start + len;
        if (r < num_pages_ && isRunStart(r) && tags_[r].owner == owner) {
            const size_t l = tags_[r].len;
            unlink(r);
            len += l;
        }
        link(start, len, owner);
    }

    // Take `len` pages from a free run of `owner`; kNone if none is large
    // enough.
    size_t take(size_t len, uint32_t owner) {
        if (len == 0 || len > 0xFFFFFFFFu) return kNone;
        LockGuard guard(lock_);
        OwnerBins& ob = bins_[owner];
        size_t b = binOf(len);
        size_t s = kNone;
        if (b >= kExactBins) {
            // Power-of-two bin: runs in it may be smaller than len.
            for (uint32_t i = ob.head[b]; i; i = tags_[i - 1].next)
                if (tags_[i - 1].len >= len) { s = i - 1; break; }
            ++b;
        }
        if (s == kNone) {
            const size_t nb = firstNonEmptyBin(ob, b);
            if (nb == kNumBins) return kNone;
            s = ob.head[nb] - 1;       // every run in a higher bin fits
        }
        const size_t have = tags_[s].len;
        unlink(s);
        if (have > len) link(s + len, have - len, owner);  // maximal run: no merge needed
        return s;
    }

    size_t numPages() const { return num_pages_; }

private:
    struct Tag {             // valid only at a free run's first page
        uint32_t len;        // 0 = not the start of a free run
        uint32_t next;       // index + 1 in the bin list, 0 = none
        uint32_t prev;       // index + 1, 0 = none (this is the bin head)
        uint32_t owner;
    };
    static constexpr size_t kExactBins = 64;                 // 1..64 pages
    static constexpr size_t kNumBins = kExactBins + 26;      // then 2^6 .. 2^31
    struct OwnerBins {
        uint32_t head[kNumBins];                             // index + 1
        uint64_t nonempty[2];                                // bit per bin
    };

    static void* mapZeroed(size_t bytes) {
        void* p = ::mmap(nullptr, bytes, PROT_READ | PROT_WRITE,
                         MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
        return p == MAP_FAILED ? nullptr : p;
    }

    static size_t binOf(size_t len) {
        if (len <= kExactBins) return len - 1;
        const size_t log2 = 63 - static_cast<size_t>(__builtin_clzll(len));  // >= 6
        const size_t b = kExactBins + (log2 - 6);
        return b < kNumBins ? b : kNumBins - 1;
    }

    static size_t firstNonEmptyBin(const OwnerBins& ob, size_t from) {
        for (size_t w = from / 64; w < 2; ++w) {
            uint64_t m = ob.nonempty[w];
            if (w == from / 64) m &= ~uint64_t{0} << (from % 64);
            if (m) {
                const size_t b = w * 64 + static_cast<size_t>(__builtin_ctzll(m));
                return b < kNumBins ? b : kNumBins;
            }
        }
        return kNumBins;
    }

    bool isRunStart(size_t s) const { return tags_[s].len != 0; }

    void link(size_t s, size_t len, uint32_t owner) {
        OwnerBins& ob = bins_[owner];
        const size_t b = binOf(len);
        Tag& t = tags_[s];
        t.len = static_cast<uint32_t>(len);
        t.owner = owner;
        t.prev = 0;
        t.next = ob.head[b];
        if (t.next) tags_[t.next - 1].prev = static_cast<uint32_t>(s + 1);
        ob.head[b] = static_cast<uint32_t>(s + 1);
        ob.nonempty[b / 64] |= uint64_t{1} << (b % 64);
        ends_[s + len - 1] = static_cast<uint32_t>(s + 1);
    }

    void unlink(size_t s) {
        Tag& t = tags_[s];
        OwnerBins& ob = bins_[t.owner];
        const size_t b = binOf(t.len);
        if (t.prev) tags_[t.prev - 1].next = t.next;
        else ob.head[b] = t.next;
        if (t.next) tags_[t.next - 1].prev = t.prev;
        if (!ob.head[b]) ob.nonempty[b / 64] &= ~(uint64_t{1} << (b % 64));
        t.len = 0;           // the end tag goes stale; readers re-validate it
        t.next = t.prev = 0;
    }

    Spinlock lock_;
    size_t num_pages_ = 0;
    Tag* tags_ = nullptr;
    uint32_t* ends_ = nullptr;
    OwnerBins* bins_ = nullptr;
};

}  // namespace smash
