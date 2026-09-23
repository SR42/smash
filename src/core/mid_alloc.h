#pragma once
// mid_alloc.h - size-binned allocator for the 16 KiB .. 1 MiB band.
//
// Without it, an allocation in this band takes LargeAlloc's direct-mmap path:
// never placed in the compressible arena, so never compressed however cold it
// gets, and costing one mmap on allocation and one munmap on free (~10 µs per
// malloc/free cycle, against ~17 ns for glibc).
//
// Putting those blocks in the arena as individual page runs is NOT the fix,
// and that is what `kLargeAllocVmThreshold` has been protecting against: each
// free then leaves a hole the size of that block, interleaved with live
// neighbours, and a later large request finds no run long enough. Measured, a
// 4 GiB arena filled with 512 KB blocks and half of them freed has 1919 MiB
// free and a longest run of 512 KB — a 1 GiB allocation misses the arena
// entirely and stops being compressible. Trading a small block's
// compressibility for a large block's is a bad trade.
//
// So bin them. Each size class owns chunks carved from the arena and slices
// each chunk into equal blocks tracked by the same bitmap Span already uses
// for small objects. A freed block is reused by its own class, so holes are
// refilled by same-size allocations instead of accumulating. The arena only
// ever sees whole chunks — uniform and page-aligned — which coalesce cleanly,
// so large contiguous requests stay possible.
//
// The cost is rounding: classes are spaced 1.25x apart, which wastes 10% of
// the requested bytes on average and 25% at worst (against 37% average / 98%
// worst for powers of two). That waste is uncompressed resident memory, so it
// only pays off for blocks that actually go cold. Off by default: SMASH_MID_BINS=1.
//
// Not covered in this first version, by choice: aligned allocations (they keep
// LargeAlloc's path), NUMA node arenas (malloc_numa is unaffected), and
// returning an emptied chunk to the arena — see the note on reclaim below.
#include "smash/config.h"
#include "../util/spinlock.h"
#include "../vm/vm_region.h"
#include "../vm/page_state.h"
#include "page_map.h"
#include "span.h"

#include <cstddef>
#include <cstdint>

namespace smash {

// 1.25x spacing, every entry a whole number of pages. Covers (16 KiB, 1 MiB].
inline constexpr uint32_t kMidClassBytes[] = {
     20u << 10,  28u << 10,  36u << 10,  48u << 10,  60u << 10,  76u << 10,
     96u << 10, 120u << 10, 152u << 10, 192u << 10, 240u << 10, 300u << 10,
    376u << 10, 472u << 10, 592u << 10, 740u << 10, 928u << 10, 1024u << 10,
};
inline constexpr int kNumMidClasses =
    static_cast<int>(sizeof(kMidClassBytes) / sizeof(kMidClassBytes[0]));

class MidAlloc {
public:
    void init(VmRegion* vm, PageStateTable* states, PageMap* page_map,
              bool (*release_hook)(size_t, size_t, void*) = nullptr,
              void* release_ctx = nullptr) {
        vm_ = vm;
        states_ = states;
        page_map_ = page_map;
        release_hook_ = release_hook;
        release_ctx_ = release_ctx;
    }

    // Smallest class that fits, or -1 if the size is outside the band. The
    // band's top follows the runtime threshold: a size that LargeAlloc would
    // put in the arena anyway must not be binned, or big blocks would start
    // fragmenting the very chunks this exists to keep uniform.
    static int classOf(size_t size) {
        if (size <= kMaxSmallSize || size >= largeAllocVmThreshold()) return -1;
        for (int i = 0; i < kNumMidClasses; ++i)
            if (size <= kMidClassBytes[i]) return i;
        return -1;
    }

    static uint32_t classSizeBytes(int cls) { return kMidClassBytes[cls]; }

    // Class index of an existing chunk, from the object size it was built
    // with. Keeps the class out of Span, which has no spare field.
    static int classOfSpan(const Span* s) {
        for (int i = 0; i < kNumMidClasses; ++i)
            if (kMidClassBytes[i] == s->object_size) return i;
        return -1;
    }

    void* allocate(size_t size, uint8_t arena) {
        const int cls = classOf(size);
        if (cls < 0 || !vm_ || !page_map_) return nullptr;
        MidClass& mc = shard(arena, cls);
        {
            LockGuard guard(mc.lock);
            while (Span* s = mc.partial_head) {
                if (void* p = s->allocate()) {
                    if (s->full()) unlink(mc, s);
                    return p;
                }
                unlink(mc, s);       // full after all; drop it from the list
            }
        }
        // Build the chunk OUTSIDE the lock. It takes pages from the arena,
        // sets a page state per page and registers the whole run in the page
        // map — hundreds of stores for a 2 MiB chunk. Doing that while holding
        // the class lock serialises every other thread in the class behind it,
        // and measured at 64 threads that made the binned path 5.6x SLOWER
        // than the direct-mmap path it replaces (29.3 s against 5.3 s), while
        // still being 6x faster at 4 threads. Two threads may now build a
        // chunk for the same class at once; both are usable, so the loser
        // simply contributes its chunk to the list.
        Span* s = newChunk(cls, arena);
        if (!s) return nullptr;
        LockGuard guard(mc.lock);
        void* p = s->allocate();
        if (!p) return nullptr;      // a fresh chunk always has room; be safe
        if (!s->full()) link(mc, s);
        return p;
    }

    void deallocate(Span* span, void* ptr) {
        const int cls = classOfSpan(span);
        if (cls < 0) return;         // not ours; caller checked is_mid, so unreachable
        MidClass& mc = shard(span->arena_id, cls);
        LockGuard guard(mc.lock);
        const bool was_full = span->full();
        span->deallocate(ptr);
        // An emptied chunk is deliberately kept, not returned to the arena.
        // Releasing spans eagerly cost ~70% on mi-stress when the slab did it
        // (see "Returning empty slab spans is a trap" in CLAUDE.md): a chunk
        // that oscillates between empty and in use pays a release, a decommit
        // and zero-fill faults every round trip. Its pages stay compressible
        // while idle, which is the point of putting them here.
        if (was_full) link(mc, span);
    }

    // Called from VmRegion's exhausted hook, i.e. only when the arena has run
    // out and the alternative is an uncompressible direct mapping. Keeping
    // empty chunks until then is deliberate (see deallocate); handing them
    // back on every free is the trap that cost ~70% on mi-stress when the slab
    // did it. Uses tryLock throughout: the hook can fire with a class lock
    // already held further up the stack.
    void reclaimEmptyChunks() {
        if (!vm_ || !page_map_) return;
        for (int i = 0; i < kMaxArenas * kNumMidClasses; ++i) {
            MidClass& mc = classes_[i];
            if (!mc.lock.tryLock()) continue;
            Span* s = mc.partial_head;
            while (s) {
                Span* next = s->list_next;
                if (s->allocated_count == 0) {
                    unlink(mc, s);
                    void* base = s->base;
                    const uint32_t pages = s->page_count;
                    // Clear the ownership records BEFORE the pages go back, so
                    // a reallocation of the same range cannot resolve to this
                    // dead chunk.
                    page_map_->setRange(reinterpret_cast<uintptr_t>(base), pages, nullptr);
                    // An idle chunk is exactly what the compressor takes, so
                    // its pages usually hold blobs. Release them and reset the
                    // page states first, as LargeAlloc and Slab do; otherwise
                    // every blob leaks and the pages reach their next owner
                    // still marked COMPRESSED (test_mid_bins_reclaim: 3200 of
                    // 3200 pages kept their blobs).
                    if (release_hook_)
                        release_hook_(vm_->pageIndex(reinterpret_cast<uintptr_t>(base)),
                                      pages, release_ctx_);
                    vm_->releasePagesNow(base, pages);
                    recycleSpanDescriptor(s);
                }
                s = next;
            }
            mc.lock.unlock();
        }
    }

private:
    struct MidClass {
        Spinlock lock;
        Span* partial_head = nullptr;
    };

    // Enough blocks per chunk to amortise the arena allocation, and at least
    // 2 MiB so the arena sees runs it can coalesce, capped at 16 MiB so a
    // single first-touch of a big class cannot reserve an unreasonable amount.
    static uint32_t chunkPagesFor(int cls) {
        const uint32_t obj_pages = kMidClassBytes[cls] / kPageSize;
        uint32_t pages = obj_pages * 4;
        if (pages < 512) pages = 512;
        if (pages > 4096) pages = 4096;
        return pages;
    }

    Span* newChunk(int cls, uint8_t arena) {
        const uint32_t pages = chunkPagesFor(cls);
        void* mem = vm_->allocatePages(pages);
        if (!mem) return nullptr;    // arena exhausted: caller falls back
        Span* s = newSpanDescriptor();
        if (!s) { vm_->releasePages(mem, pages); return nullptr; }
        if (!s->init(mem, pages, largeSizeClass(pages), arena, 0,
                     kMidClassBytes[cls])) {
            vm_->releasePages(mem, pages);
            return nullptr;
        }
        // Mark the pages live so the compressor tracks them, exactly as
        // LargeAlloc does for an arena-backed block.
        if (states_) {
            const size_t idx = vm_->pageIndex(reinterpret_cast<uintptr_t>(mem));
            for (uint32_t p = 0; p < pages; ++p)
                states_->set(idx + p, PageState::ACTIVE);
        }
        // Register every page so free() resolves any pointer in the chunk.
        page_map_->setRange(reinterpret_cast<uintptr_t>(mem), pages, s);
        return s;
    }

    static void link(MidClass& mc, Span* s) {
        s->list_prev = nullptr;
        s->list_next = mc.partial_head;
        if (mc.partial_head) mc.partial_head->list_prev = s;
        mc.partial_head = s;
    }
    static void unlink(MidClass& mc, Span* s) {
        if (s->list_prev) s->list_prev->list_next = s->list_next;
        else if (mc.partial_head == s) mc.partial_head = s->list_next;
        if (s->list_next) s->list_next->list_prev = s->list_prev;
        s->list_prev = s->list_next = nullptr;
    }

    // Sharded by arena, like every other allocation structure here: the class
    // alone gives 18 locks, and 64 threads on 18 locks is what the measurement
    // above was about. A block's arena is recorded in its Span, so free()
    // finds the same shard.
    MidClass& shard(uint8_t arena, int cls) {
        const int a = (arena < kMaxArenas) ? arena : 0;
        return classes_[a * kNumMidClasses + cls];
    }
    MidClass classes_[kMaxArenas * kNumMidClasses];
    VmRegion* vm_ = nullptr;
    PageStateTable* states_ = nullptr;
    PageMap* page_map_ = nullptr;
    bool (*release_hook_)(size_t, size_t, void*) = nullptr;
    void* release_ctx_ = nullptr;
};

}  // namespace smash
