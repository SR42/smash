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
// Size bins per owner: exact bins for 1..64 pages, then four bins per power of
// two, so a run in a bin is at most 1.25x the bin's floor. take() uses a
// per-owner bitmap of non-empty bins. Only the request's own bin can hold runs
// shorter than it; take() looks at kTakeScan of those and moves on to the next
// non-empty bin, whose runs all fit, so it is bounded whatever the free lists
// look like. It can miss a fit deep in that bin, which costs fresh region at
// worst; takeAny() does not, for when there is no fresh region left.
//
// A run's pages need not be alike. A sub-arena frees runs without decommitting
// them, so it marks them dirty: one bit per page, and each run counts its dirty
// pages, 0 for a run nothing dirty ever joined (no other user of the map marks
// any). Dirty and clean neighbours merge like any others: kept apart, a purge
// left a dirty head and a clean tail for good, and a chunk with nothing live
// could stay split for ever. Taking from a run reports how many of its pages
// were dirty, which is what calloc needs to know. A word of bits is only ever
// written under the lock of the owner of its pages: a sub-arena owns whole
// chunks, and a chunk is a whole number of words.
//
// Storage is demand-paged (MAP_NORESERVE) and all-zero means empty: links and
// bin heads store index + 1, so no initialization pass is needed even for a
// 1 TiB arena. Memory cost is 24 bytes per arena page in the worst case of a
// fully fragmented arena, touched only where free runs actually start and end.
//
// One lock guards the whole map. Frees reach it only from VmRegion's decommit
// thread; allocations of arena pages (a new slab span, a large block) are far
// rarer than object allocations.
#include "../util/spinlock.h"
#include "../util/fork_locks.h"
#include "platform_mem.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <sys/mman.h>

namespace smash {

class PageRunMap {
public:
    void forkLocks(ForkLockSet& s) { s.add(lock_); }
    Spinlock& lock() { return lock_; }   // for the *Unlocked walkers
    static constexpr size_t kNone = ~size_t{0};
    // Free runs examined per size bin before moving on. A miss only means the
    // caller takes fresh region, so this trades a rare extra chunk for a
    // bounded hold of the lock. kScanAll is for the caller with no fresh
    // region left.
    static constexpr size_t kTakeScan = 8;
    static constexpr size_t kScanAll = ~size_t{0};
    // Owners: 0 shared, 1..kMaxNumaNodes the node arenas, then the sub-arenas
    // (vm_region.h), which must not alias a node's own owner in that node's map.
    static constexpr uint32_t kFirstSubOwner = vm::kMaxNumaNodes + 1;
    static constexpr uint32_t kSubOwners = 128;
    static constexpr uint32_t kOwners = kFirstSubOwner + kSubOwners;

    bool init(size_t num_pages) {
        num_pages_ = num_pages;
        tags_ = static_cast<Tag*>(mapZeroed(num_pages * sizeof(Tag)));
        ends_ = static_cast<uint32_t*>(mapZeroed(num_pages * sizeof(uint32_t)));
        bins_ = static_cast<OwnerBins*>(mapZeroed(kOwners * sizeof(OwnerBins)));
        dirty_bits_ = static_cast<uint64_t*>(mapZeroed((num_pages / 64 + 1) * sizeof(uint64_t)));
        return tags_ && ends_ && bins_ && dirty_bits_;
    }

    // Add the free run [start, start + len) owned by `owner`, merging with
    // free neighbours of the same owner.
    void insert(size_t start, size_t len, uint32_t owner) {
        if (len == 0) return;
        LockGuard guard(lock_);
        insertUnlocked(start, len, owner, 0, 0);
    }

    // ── Caller-locked operations (sub-arenas, see vm_region.h) ─────────────
    // The same operations without the map's own lock, for a caller that holds
    // a lock covering every owner it touches. Safe to mix with the locked
    // ones only if no locked operation can reach the same pages: sub-arenas
    // own whole chunks and pass `boundary` (their chunk size), so a merge
    // never reads a tag outside a chunk they own.
    //
    // `stamp` is the caller's clock at insertion (the purger's age); a merge
    // keeps the newest stamp, so a coalesced run counts as recently freed,
    // and a split keeps the original. `dirty` says how many of the range's
    // pages have their dirty bit set (markDirtyUnlocked), which is the caller's
    // to know: a merge adds the counts and never reads the bits.
    void insertUnlocked(size_t start, size_t len, uint32_t owner, size_t boundary,
                        uint32_t stamp, size_t dirty = 0) {
        if (len == 0) return;
        const size_t end = start + len;
        // Left neighbour: a free run of this owner ending at start - 1.
        if (start > 0 && (boundary == 0 || start % boundary != 0)) {
            const uint32_t t = ends_[start - 1];
            if (t) {
                const size_t s = t - 1;
                if (isRunStart(s) && s + tags_[s].len == start && tags_[s].owner == owner) {
                    const size_t l = tags_[s].len;
                    if (tags_[s].stamp > stamp) stamp = tags_[s].stamp;
                    dirty += tags_[s].dirty;
                    unlink(s);
                    start = s;
                    len += l;
                }
            }
        }
        // Right neighbour: a free run of this owner starting right after.
        const size_t r = end;
        if (r < num_pages_ && (boundary == 0 || r % boundary != 0) && isRunStart(r) &&
            tags_[r].owner == owner) {
            const size_t l = tags_[r].len;
            if (tags_[r].stamp > stamp) stamp = tags_[r].stamp;
            dirty += tags_[r].dirty;
            unlink(r);
            len += l;
        }
        link(start, len, owner, stamp, dirty);
    }

    // Set the dirty bits of [start, start + len); the caller then inserts the
    // range with `dirty` = len. Every page in it must have its bit clear.
    void markDirtyUnlocked(size_t start, size_t len) { setBits(start, len, true); }
    // Dirty pages in [start, start + len) of a run the caller holds.
    size_t countDirtyUnlocked(size_t start, size_t len) const { return countBits(start, len); }
    void clearDirtyUnlocked(size_t start, size_t len) { setBits(start, len, false); }
    // Dirty bits set anywhere below page `end` (smash_sim's checks).
    size_t totalDirtyBits(size_t end) const { return countBits(0, end < num_pages_ ? end : num_pages_); }
    // Where to cut the free run [start, start + len) so the part behind the cut
    // holds exactly `want` dirty pages; the caller knows the run has that many.
    size_t cutForLastDirtyUnlocked(size_t start, size_t len, size_t want) const {
        size_t i = start + len, have = 0;
        while (i > start && have < want) {
            if (!(i % 64) && i - 64 >= start) {   // a whole word
                const size_t c = static_cast<size_t>(__builtin_popcountll(dirty_bits_[i / 64 - 1]));
                if (have + c < want) { have += c; i -= 64; continue; }
            }
            --i;
            if ((dirty_bits_[i / 64] >> (i % 64)) & 1) ++have;
        }
        return i;
    }

    // Every free run of `owner`: fn(start, len, stamp, dirty). The caller must
    // not modify the map inside fn (collect, then act).
    template <typename Fn> void forEachRunUnlocked(uint32_t owner, Fn&& fn) const {
        const OwnerBins& ob = bins_[owner];
        for (size_t b = 0; b < kNumBins; ++b)
            for (uint32_t i = ob.head[b]; i; i = tags_[i - 1].next)
                fn(static_cast<size_t>(i - 1), static_cast<size_t>(tags_[i - 1].len),
                   tags_[i - 1].stamp, static_cast<size_t>(tags_[i - 1].dirty));
    }

    // Remove the free run that starts at `start` if it still is one of
    // `owner` with length `len`; false otherwise (it merged or was taken). Its
    // dirty bits stay as they are: the caller owns them with the pages.
    bool removeRunUnlocked(size_t start, size_t len, uint32_t owner) {
        if (start >= num_pages_ || !isRunStart(start) || tags_[start].owner != owner ||
            tags_[start].len != len)
            return false;
        unlink(start);
        return true;
    }

    // The free run that starts at `start`: false if none does. Caller-locked.
    bool runAtUnlocked(size_t start, size_t* len, uint32_t* owner, uint32_t* stamp,
                       size_t* dirty = nullptr) const {
        if (start >= num_pages_ || !isRunStart(start)) return false;
        *len = tags_[start].len;
        *owner = tags_[start].owner;
        *stamp = tags_[start].stamp;
        if (dirty) *dirty = tags_[start].dirty;
        return true;
    }

    // Take the first `len` pages of the free run of `owner` at `start`, if one
    // starts there and is that long; realloc's in-place growth. `*dirty_taken`
    // as in takeUnlocked.
    bool takeFrontUnlocked(size_t start, size_t len, uint32_t owner, size_t* dirty_taken = nullptr) {
        if (start >= num_pages_ || !isRunStart(start) || tags_[start].owner != owner ||
            tags_[start].len < len)
            return false;
        carve(start, 0, len, owner, dirty_taken);
        return true;
    }

    // Take `len` pages from a free run of `owner`; kNone if the bounded look
    // (see the top of the file) found none. takeAny: kNone only if no run of
    // `owner` is long enough. `*dirty_taken`, if given, is set to how many of
    // the pages were dirty; their bits are cleared.
    size_t take(size_t len, uint32_t owner) {
        if (len == 0 || len > 0xFFFFFFFFu) return kNone;
        LockGuard guard(lock_);
        return takeUnlocked(len, owner);
    }
    size_t takeAny(size_t len, uint32_t owner) {
        if (len == 0 || len > 0xFFFFFFFFu) return kNone;
        LockGuard guard(lock_);
        return takeUnlocked(len, owner, kScanAll);
    }
    size_t takeUnlocked(size_t len, uint32_t owner, size_t scan_limit = kTakeScan,
                        size_t* dirty_taken = nullptr) {
        if (len == 0 || len > 0xFFFFFFFFu) return kNone;
        OwnerBins& ob = bins_[owner];
        size_t b = binOf(len);
        size_t s = kNone;
        if (b >= kExactBins) {
            // A bin above the exact ones: runs in it may be shorter than len.
            size_t scanned = 0;
            for (uint32_t i = ob.head[b]; i && scanned < scan_limit; i = tags_[i - 1].next) {
                ++scanned;
                if (tags_[i - 1].len >= len) { s = i - 1; break; }
            }
            ob.scan_steps += scanned;
            ++b;
        }
        if (s == kNone) {
            const size_t nb = firstNonEmptyBin(ob, b);
            if (nb == kNumBins) return kNone;
            s = ob.head[nb] - 1;       // every run in a higher bin fits
        }
        carve(s, 0, len, owner, dirty_taken);
        return s;
    }

    // Take `len` pages from a free run of `owner` starting at an index with
    // ((index + phase) % align_pages) == 0, where align_pages is a power of
    // two. kNone if no run can host one; the caller then falls back to fresh
    // region, so a bounded search is fine.
    //
    // It carves only what is needed: the part of the chosen run before the
    // aligned start and the part after the request both stay free. The
    // alternative — take len + align_pages - 1 and hand the ends back — costs
    // align_pages - 1 pages per *live* block in a workload that allocates one
    // shape, because the returned fragments are by construction too short to
    // host the next request of that same shape. Measured at 64 KiB alignment:
    // a constant 60 KiB per live block whatever its size, so 1.19x on 320 KiB
    // blocks and 1.94x on 64 KiB ones.
    size_t takeAligned(size_t len, uint32_t owner, size_t align_pages, size_t phase,
                       size_t scan_limit = kTakeScan) {
        if (len == 0 || len > 0xFFFFFFFFu) return kNone;
        LockGuard guard(lock_);
        return takeAlignedUnlocked(len, owner, align_pages, phase, scan_limit);
    }
    size_t takeAlignedUnlocked(size_t len, uint32_t owner, size_t align_pages, size_t phase,
                               size_t scan_limit = kTakeScan, size_t* dirty_taken = nullptr) {
        if (align_pages <= 1) return takeUnlocked(len, owner, scan_limit, dirty_taken);
        if (len == 0 || len > 0xFFFFFFFFu) return kNone;
        OwnerBins& ob = bins_[owner];
        // A run this long hosts the request at any phase, so any bin above the
        // one holding it is an unconditional hit and the scan stays bounded.
        const size_t fits_anywhere = len + align_pages - 1;
        const size_t stop_bin = binOf(fits_anywhere);
        size_t s = kNone, off = 0;
        for (size_t b = binOf(len); b < kNumBins && s == kNone; ++b) {
            if (!((ob.nonempty[b / 64] >> (b % 64)) & 1)) continue;
            if (b > stop_bin) {
                s = ob.head[b] - 1;
                off = alignOffset(s, align_pages, phase);
                break;
            }
            size_t scanned = 0;
            for (uint32_t i = ob.head[b]; i && scanned < scan_limit;
                 i = tags_[i - 1].next, ++scanned) {
                const size_t c = i - 1;
                const size_t o = alignOffset(c, align_pages, phase);
                if (o + len <= tags_[c].len) { s = c; off = o; break; }
            }
            ob.scan_steps += scanned;
        }
        if (s == kNone) return kNone;
        if (off + len > tags_[s].len) return kNone;   // the stop_bin shortcut rules this out
        carve(s, off, len, owner, dirty_taken);
        return s + off;
    }

    size_t numPages() const { return num_pages_; }
#if defined(SMASH_SIM)
    Spinlock& lockForCheck() { return lock_; }   // smash_sim's quiescent checks
#endif

    // Free pages held by `owner`, and a lower bound on its longest run (the
    // start of the highest non-empty size bin). Together they separate "the
    // arena is genuinely full" from "it is fragmented": many free pages but a
    // short longest run means no single allocation fits.
    size_t freePages(uint32_t owner) {
        // Relaxed, no lock: caller-locked owners change it under their own
        // lock, so this is a diagnostic snapshot either way.
        return bins_[owner].free_pages.load(std::memory_order_relaxed);
    }
    size_t longestRunAtLeast(uint32_t owner) {
        LockGuard guard(lock_);
        const OwnerBins& ob = bins_[owner];
        for (size_t b = kNumBins; b-- > 0;)
            if (ob.head[b]) return binFloor(b);
        return 0;
    }
    // Free runs of `owner` looked at by the take functions so far.
    uint64_t scanSteps(uint32_t owner) const { return bins_[owner].scan_steps; }

private:
    struct Tag {             // valid only at a free run's first page
        uint32_t len;        // 0 = not the start of a free run
        uint32_t next;       // index + 1 in the bin list, 0 = none
        uint32_t prev;       // index + 1, 0 = none (this is the bin head)
        uint32_t owner;
        uint32_t stamp;      // caller's clock when freed (sub-arena purge age)
        uint32_t dirty;      // pages of the run with their dirty bit set
    };
    static constexpr size_t kExactBins = 64;                 // 1..64 pages
    static constexpr size_t kBinsPerDoubling = 4;            // then 2^6 .. 2^31
    static constexpr size_t kNumBins = kExactBins + 26 * kBinsPerDoubling;
    static constexpr size_t kBinWords = (kNumBins + 63) / 64;
    struct OwnerBins {
        uint32_t head[kNumBins];                             // index + 1
        uint64_t nonempty[kBinWords];                        // bit per bin
        std::atomic<size_t> free_pages;                      // for diagnostics
        uint64_t scan_steps;                                 // under the owner's lock
    };

    static void* mapZeroed(size_t bytes) {
        void* p = ::mmap(nullptr, bytes, PROT_READ | PROT_WRITE,
                         MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
        return p == MAP_FAILED ? nullptr : p;
    }

    // Pages from `s` to the next index satisfying the alignment. align_pages
    // is a power of two, so the modulo is a mask.
    static size_t alignOffset(size_t s, size_t align_pages, size_t phase) {
        const size_t m = (s + phase) & (align_pages - 1);
        return m ? align_pages - m : 0;
    }

    static size_t binOf(size_t len) {
        if (len <= kExactBins) return len - 1;
        const size_t log2 = 63 - static_cast<size_t>(__builtin_clzll(len));  // >= 6
        const size_t sub = (len >> (log2 - 2)) & (kBinsPerDoubling - 1);
        const size_t b = kExactBins + (log2 - 6) * kBinsPerDoubling + sub;
        return b < kNumBins ? b : kNumBins - 1;
    }
    // The shortest length binOf() files in bin `b`, or less.
    static size_t binFloor(size_t b) {
        if (b < kExactBins) return b + 1;
        const size_t k = b - kExactBins;
        return (kBinsPerDoubling + k % kBinsPerDoubling) << (4 + k / kBinsPerDoubling);
    }

    static size_t firstNonEmptyBin(const OwnerBins& ob, size_t from) {
        for (size_t w = from / 64; w < kBinWords; ++w) {
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

    // Take [s + off, s + off + len) out of the free run at `s`. What is left
    // on either side stays free, with its share of the dirty count; a maximal
    // run's pieces have no free neighbour to merge with.
    void carve(size_t s, size_t off, size_t len, uint32_t owner, size_t* dirty_taken) {
        const size_t have = tags_[s].len;
        const size_t dirty = tags_[s].dirty;
        const uint32_t stamp = tags_[s].stamp;
        unlink(s);
        size_t taken = 0, head = 0;
        if (dirty) {
            taken = countBits(s + off, len);
            if (taken) setBits(s + off, len, false);
            if (off) head = countBits(s, off);
        }
        if (off) link(s, off, owner, stamp, head);
        if (const size_t tail = have - off - len; tail)
            link(s + off + len, tail, owner, stamp, dirty - taken - head);
        if (dirty_taken) *dirty_taken = taken;
    }

    void setBits(size_t start, size_t len, bool on) {
        size_t i = start;
        const size_t end = start + len;
        while (i < end) {
            const size_t w = i / 64, b = i % 64;
            const size_t n = end - i < 64 - b ? end - i : 64 - b;
            const uint64_t mask = (n == 64 ? ~uint64_t{0} : (uint64_t{1} << n) - 1) << b;
            if (on) dirty_bits_[w] |= mask;
            else dirty_bits_[w] &= ~mask;
            i += n;
        }
    }
    size_t countBits(size_t start, size_t len) const {
        size_t i = start, c = 0;
        const size_t end = start + len;
        while (i < end) {
            const size_t w = i / 64, b = i % 64;
            const size_t n = end - i < 64 - b ? end - i : 64 - b;
            const uint64_t mask = (n == 64 ? ~uint64_t{0} : (uint64_t{1} << n) - 1) << b;
            c += static_cast<size_t>(__builtin_popcountll(dirty_bits_[w] & mask));
            i += n;
        }
        return c;
    }

    void link(size_t s, size_t len, uint32_t owner, uint32_t stamp = 0, size_t dirty = 0) {
        OwnerBins& ob = bins_[owner];
        const size_t b = binOf(len);
        Tag& t = tags_[s];
        t.len = static_cast<uint32_t>(len);
        t.owner = owner;
        t.stamp = stamp;
        t.dirty = static_cast<uint32_t>(dirty);
        t.prev = 0;
        t.next = ob.head[b];
        if (t.next) tags_[t.next - 1].prev = static_cast<uint32_t>(s + 1);
        ob.head[b] = static_cast<uint32_t>(s + 1);
        ob.nonempty[b / 64] |= uint64_t{1} << (b % 64);
        ob.free_pages.store(ob.free_pages.load(std::memory_order_relaxed) + len,
                            std::memory_order_relaxed);
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
        ob.free_pages.store(ob.free_pages.load(std::memory_order_relaxed) - t.len,
                            std::memory_order_relaxed);
        t.len = 0;           // the end tag goes stale; readers re-validate it
        t.next = t.prev = t.dirty = 0;
    }

    Spinlock lock_;
    size_t num_pages_ = 0;
    Tag* tags_ = nullptr;
    uint32_t* ends_ = nullptr;
    OwnerBins* bins_ = nullptr;
    uint64_t* dirty_bits_ = nullptr;
};

}  // namespace smash
