// smash/src/core/span.h - Span descriptor with bitmap-based free tracking
//
// Key invariant: span metadata (this struct + bitmap) lives in bootstrap memory,
// NEVER mixed with user data pages. This keeps data pages pure for compression.
#pragma once

#include "smash/config.h"
#include "bootstrap_alloc.h"
#include "size_classes.h"
#include "../util/bitops.h"
#include "../util/double_free.h"
#include "../util/spinlock.h"
#include "../vm/page_state.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace smash {

// Allocator-induced re-heat events: a slot was handed out on a page that had
// already cooled past the closing threshold, resetting its cold streak (see
// getCoolingCloseTicks()). With cooling-page closing active this only happens
// when a span has no warmer free slot, so a high rate flags workloads where
// span-level (not just slot-level) placement would pay off. Reported as
// reheat= in the SIGUSR2 stats line.
inline std::atomic<uint64_t> g_reheat_events{0};

// Words in the largest slab-span bitmap: kMaxSpanPages pages of the smallest
// (16-byte) class.
inline constexpr size_t kMaxSpanBitmapWords =
    (static_cast<size_t>(kMaxSpanPages) * kPageSize / kMinAlignment + 63) / 64;

struct Span {
    void* base;                 // start of data pages
    uint64_t* bitmap;           // 1 = free, 0 = allocated; from BootstrapAlloc
    uint32_t page_count;
    uint32_t object_size;
    // Slots that *could* exist in the span's pages (pages*kPageSize/object_size).
    // Bitmap is always sized for this.  Under a per-page cap, most of these bits
    // are held at 0 and never handed out.  allocate() must scan the full bitmap
    // — not just (object_count+63)/64 words — because the live bits are spread
    // across every page, not packed at the front.
    uint32_t full_capacity;
    uint16_t object_count;      // currently usable slots (grows with widenCap)
    uint16_t allocated_count;
    uint8_t size_class;
    bool is_large;              // true for mmap-backed large allocations
    bool is_mid;                // true for a MidAlloc chunk (see mid_alloc.h)
    uint8_t arena_id;           // which arena this span belongs to
    uint16_t free_hint_word;    // word index to start bitmap scan from
    // Per-page cap currently encoded in the bitmap.  0 = no cap (dense).
    // Slab::widenCapIfNeeded() compares this to the current adaptive cap and
    // unlocks additional slots when the cap has relaxed since span init.
    uint16_t current_cap_per_page;

    // Intrusive list pointers (used by Slab's partial/full/empty lists)
    Span* list_prev;
    Span* list_next;

    // For large allocations: actual requested size
    size_t large_size;

    // For thread cache pool recycling
    Span* next_free;

    // Optional: page-state lookup so allocate() can skip slots whose page
    // is currently COMPRESSED. Set by Slab::allocateNewSpan when a
    // VmRegion is in use; null otherwise (large allocs, no-vm fallback).
    // The slab pre-computes first_page_vm_idx (= vm_region_->pageIndex(base))
    // so the inner allocate loop doesn't have to call back into VmRegion.
    PageStateTable* page_states;
    uint32_t        first_page_vm_idx;
    // Optional: per-page cold-tick counter array owned by CompressorThread.
    // allocate() resets the counter for the page that just received a chunk,
    // so phase2 doesn't compress pages that are still being filled.
    uint8_t*        cold_counts;

    // Bitmap storage that stays with the descriptor across recycling: empty
    // slab spans go back to the arena and their descriptors are reused, so a
    // fresh bitmap per init would leak metadata on every span churn. Sized to
    // the class that first used this descriptor and grown only when a later
    // class needs more — sizing every bitmap for the largest span instead
    // wasted 248 bytes on a 16 KiB class and cut the heap the metadata arena
    // can cover by 3x (329 GB -> 102 GB).
    uint64_t*       bitmap_store;
    uint32_t        bitmap_store_words;

    // Initialize a slab span for the given size class.
    //
    // When max_slots_per_page > 0 and object_size < kPageSize, only the first
    // `cap` slots that start on each page are made available, where
    // cap = min(max_slots_per_page, slots-starting-on-that-page).  This
    // bounds per-page live count uniformly across size classes, which is
    // what the (1-q)^N page-cold probability argument actually requires.
    // Disallowed slots stay zero-filled and compress to near-zero.
    //
    // The bitmap remains sized for the full capacity (1 bit per potential
    // slot); disallowed slots are simply marked allocated (0) so the
    // allocator skips them.
    // `explicit_object_size` is for MidAlloc, whose classes are not in
    // kSizeClasses: it carves a chunk into equal blocks of its own size and
    // reuses everything else here unchanged (bitmap, capacity, free hint).
    // Zero means "take the size from the size class", the slab's behaviour.
    bool init(void* base_, uint32_t pages, uint8_t sc, uint8_t arena = 0,
              uint32_t max_slots_per_page = 0, uint32_t explicit_object_size = 0) {
        base = base_;
        page_count = pages;
        size_class = sc;
        is_large = false;
        is_mid = explicit_object_size != 0;
        arena_id = arena;
        object_size = explicit_object_size ? explicit_object_size : kSizeClasses[sc].size;
        full_capacity = (pages * kPageSize) / object_size;

        allocated_count = 0;
        free_hint_word = 0;
        list_prev = nullptr;
        list_next = nullptr;
        large_size = 0;
        next_free = nullptr;
        page_states = nullptr;
        first_page_vm_idx = 0;
        cold_counts = nullptr;

        size_t num_words = (full_capacity + 63) / 64;
        if (bitmap_store_words < num_words) {
            bitmap_store = static_cast<uint64_t*>(BootstrapAlloc::instance().allocate(
                num_words * sizeof(uint64_t), alignof(uint64_t)));
            bitmap_store_words = bitmap_store ? static_cast<uint32_t>(num_words) : 0;
        }
        bitmap = bitmap_store;
        if (!bitmap) return false;   // metadata arena exhausted; caller bails

        const bool cap_active = max_slots_per_page > 0 && object_size < kPageSize;

        if (!cap_active) {
            object_count = static_cast<uint16_t>(full_capacity);
            current_cap_per_page = 0;
            if (num_words > 0) {
                __builtin_memset(bitmap, 0xFF, num_words * sizeof(uint64_t));
                size_t valid_bits = full_capacity % 64;
                if (valid_bits != 0) {
                    bitmap[num_words - 1] = (1ULL << valid_bits) - 1;
                }
            }
            return true;
        }

        // Per-page cap path: build sparse bitmap.
        if (num_words > 0) __builtin_memset(bitmap, 0, num_words * sizeof(uint64_t));
        current_cap_per_page = static_cast<uint16_t>(max_slots_per_page);
        uint32_t live = 0;
        for (uint32_t p = 0; p < pages; ++p) {
            uint64_t first = (static_cast<uint64_t>(p) * kPageSize + object_size - 1) / object_size;
            uint64_t next  = (static_cast<uint64_t>(p + 1) * kPageSize + object_size - 1) / object_size;
            if (next > full_capacity) next = full_capacity;
            uint64_t avail_on_page = (next > first) ? (next - first) : 0;
            uint64_t use = avail_on_page < max_slots_per_page ? avail_on_page : max_slots_per_page;
            for (uint64_t i = 0; i < use; ++i) {
                uint64_t slot = first + i;
                bitmap[slot / 64] |= (1ULL << (slot % 64));
                ++live;
            }
        }
        if (live == 0) {
            bitmap[0] |= 1ULL;
            live = 1;
        }
        object_count = static_cast<uint16_t>(live);
        return true;
    }

    // Widen (or remove) the per-page cap on an already-initialized span.
    // Unlocks additional slots on each page for future allocations.  Existing
    // live/free bookkeeping for the currently-usable slots is preserved.
    // Caller must hold the slab lock.  new_cap_per_page == 0 removes the cap
    // entirely (dense bitmap).  Safe to call with new_cap <= current_cap
    // (no-op).
    void widenCap(uint32_t new_cap_per_page) {
        if (is_large || object_size == 0 || object_size >= kPageSize) return;
        const uint32_t cur = current_cap_per_page;
        // 0 means "no cap / dense" — cannot widen further.
        if (cur == 0) return;
        const uint32_t pages = page_count;
        const uint32_t osz = object_size;
        uint32_t added = 0;
        if (new_cap_per_page == 0) {
            // Dense: flip every remaining bit on.  Counts slots beyond cur.
            for (uint32_t p = 0; p < pages; ++p) {
                uint64_t first = (static_cast<uint64_t>(p) * kPageSize + osz - 1) / osz;
                uint64_t next  = (static_cast<uint64_t>(p + 1) * kPageSize + osz - 1) / osz;
                if (next > full_capacity) next = full_capacity;
                uint64_t avail = (next > first) ? (next - first) : 0;
                uint64_t start_offset = (avail < cur) ? avail : cur;
                for (uint64_t i = start_offset; i < avail; ++i) {
                    uint64_t slot = first + i;
                    size_t w = slot / 64, b = slot % 64;
                    if (!(bitmap[w] & (1ULL << b))) {
                        bitmap[w] |= (1ULL << b);
                        ++added;
                    }
                }
            }
            current_cap_per_page = 0;
        } else {
            if (new_cap_per_page <= cur) return;
            for (uint32_t p = 0; p < pages; ++p) {
                uint64_t first = (static_cast<uint64_t>(p) * kPageSize + osz - 1) / osz;
                uint64_t next  = (static_cast<uint64_t>(p + 1) * kPageSize + osz - 1) / osz;
                if (next > full_capacity) next = full_capacity;
                uint64_t avail = (next > first) ? (next - first) : 0;
                uint64_t old_end = (avail < cur) ? avail : cur;
                uint64_t new_end = (avail < new_cap_per_page) ? avail : new_cap_per_page;
                for (uint64_t i = old_end; i < new_end; ++i) {
                    uint64_t slot = first + i;
                    size_t w = slot / 64, b = slot % 64;
                    if (!(bitmap[w] & (1ULL << b))) {
                        bitmap[w] |= (1ULL << b);
                        ++added;
                    }
                }
            }
            current_cap_per_page = static_cast<uint16_t>(new_cap_per_page);
        }
        if (added) {
            uint32_t nc = static_cast<uint32_t>(object_count) + added;
            if (nc > full_capacity) nc = full_capacity;
            if (nc > 0xFFFFu) nc = 0xFFFFu;
            object_count = static_cast<uint16_t>(nc);
            free_hint_word = 0;
        }
    }

    // Initialize a large allocation span
    void initLarge(void* base_, size_t size, uint32_t pages, uint8_t arena = 0) {
        base = base_;
        page_count = pages;
        size_class = kNumClasses;
        is_large = true;
        is_mid = false;
        arena_id = arena;
        object_size = 0;
        full_capacity = 0;
        current_cap_per_page = 0;
        object_count = 0;
        allocated_count = 1;
        free_hint_word = 0;
        bitmap = nullptr;
        list_prev = nullptr;
        list_next = nullptr;
        large_size = size;
        next_free = nullptr;
        page_states = nullptr;
        first_page_vm_idx = 0;
        cold_counts = nullptr;
    }

    // Reset the cold-tick counter for the page containing the slot we just
    // handed out, so phase2 won't see this page as "idle" while it's still
    // receiving fresh allocations.  Plain (non-atomic) write — cold_count_
    // is a uint8_t snapshot read by phase1/phase2 with no ordering needs.
    // telemetry_ticks classifies the overwrite: clobbering a streak that had
    // already crossed coolingCloseDefaultTicks() is an allocator-induced
    // re-heat, counted in g_reheat_events. The telemetry threshold is fixed
    // at the default (not the env-overridable closing threshold) so runs
    // with closing disabled still count the re-heats closing would prevent.
    inline void resetColdCountForSlot(size_t slot_byte_offset,
                                      int telemetry_ticks) {
        if (!cold_counts) return;
        size_t page_off = slot_byte_offset / kPageSize;
        size_t page_idx = static_cast<size_t>(first_page_vm_idx) + page_off;
        if (telemetry_ticks > 0 && cold_counts[page_idx] >= telemetry_ticks)
            g_reheat_events.fetch_add(1, std::memory_order_relaxed);
        cold_counts[page_idx] = 0;
    }

    // Allocation preference class of the page holding a free slot.
    enum class SlotPageClass : uint8_t {
        WARM = 0,       // recently touched, or EMPTY (nothing live to protect)
        COOLING = 1,    // live data partway to compression — backfilling here
                        // resets the cold streak and delays/void's the
                        // compressor's investment in this page
        COMPRESSED_PAGE = 2,  // slot write would fault + decompress
    };

    // Classify the page for the preference cascade. All reads here are racy
    // snapshots of compressor-owned state (page-state byte, cold-tick byte);
    // a stale value only shifts a placement preference, never correctness —
    // the bitmap under the slab lock is the sole allocation truth.
    inline SlotPageClass classifySlotPage(size_t slot_byte_offset,
                                          int close_ticks) const {
        if (!page_states) return SlotPageClass::WARM;
        // Until any page has ever been compressed (always, when compression
        // is disabled), no placement decision matters — skip the per-slot
        // loads. Same latch rationale as the old slotPageCompressed().
        if (!g_any_page_compressed.load(std::memory_order_relaxed))
            return SlotPageClass::WARM;
        size_t page_idx = static_cast<size_t>(first_page_vm_idx)
                        + slot_byte_offset / kPageSize;
        PageState st = page_states->get(page_idx);
        if (st == PageState::COMPRESSED)
            return SlotPageClass::COMPRESSED_PAGE;
        // EMPTY = decommitted, nothing live: as good as warm — backfilling
        // protects nothing and costs only a kernel zero-fill on first touch.
        if (st == PageState::EMPTY)
            return SlotPageClass::WARM;
        // COMPRESSING is mid-snapshot and about to become COMPRESSED —
        // treat as cooling (avoid if a warm slot exists, allowed as
        // fallback, same as before this classification existed).
        if (st == PageState::COMPRESSING)
            return SlotPageClass::COOLING;
        if (close_ticks > 0 && cold_counts &&
            cold_counts[page_idx] >= close_ticks)
            return SlotPageClass::COOLING;
        return SlotPageClass::WARM;
    }

    // Walk one bitmap word, picking the first free slot on a WARM page.
    // Returns the chosen bit (0..63), or -1 if this word has no warm free
    // slot. As a side effect, records the first COOLING-page slot and the
    // first COMPRESSED-page slot seen (only if the caller hasn't already
    // recorded one elsewhere) — allocate() falls back to those, in that
    // order, when the whole span has no warm slot.
    inline int pickActiveBit(size_t word_idx, uint64_t word, int close_ticks,
                             int* cool_word, int* cool_bit,
                             int* comp_word, int* comp_bit) const {
        while (word) {
            int bit = ctz(word);
            size_t slot = word_idx * 64 + bit;
            size_t off  = slot * object_size;
            switch (classifySlotPage(off, close_ticks)) {
            case SlotPageClass::WARM:
                return bit;
            case SlotPageClass::COOLING:
                if (*cool_word < 0) {
                    *cool_word = static_cast<int>(word_idx);
                    *cool_bit  = bit;
                }
                break;
            case SlotPageClass::COMPRESSED_PAGE:
                if (*comp_word < 0) {
                    *comp_word = static_cast<int>(word_idx);
                    *comp_bit  = bit;
                }
                break;
            }
            word &= word - 1;  // clear this bit, try next
        }
        return -1;
    }

    // Fallback slots recorded by a warm scan: the first free slot seen on a
    // COOLING page and on a COMPRESSED page. Lets the caller (Slab) decide
    // whether to consume a fallback or redirect the allocation to another
    // span, without paying a second bitmap scan.
    struct AllocFallback {
        int cool_word = -1, cool_bit = 0;
        int comp_word = -1, comp_bit = 0;
        bool any() const { return cool_word >= 0 || comp_word >= 0; }
    };

    // Take the slot at (word, bit) out of the bitmap and return its address.
    inline void* takeSlot(int word, int bit, bool reset_cold, int telem_ticks) {
        size_t slot = static_cast<size_t>(word) * 64 + static_cast<size_t>(bit);
        bitmap[word] &= ~(1ULL << bit);
        ++allocated_count;
        free_hint_word = static_cast<uint16_t>(word);
        size_t off = slot * object_size;
        if (reset_cold) resetColdCountForSlot(off, telem_ticks);
        return static_cast<char*>(base) + off;
    }

    // Allocate one object from a WARM page, or return nullptr with *fb
    // recording the best cooling/compressed fallback slots seen. One scan
    // serves both outcomes.
    //
    // Scans over `full_capacity` words, not `object_count`: under a per-page
    // cap the live bits are spread across every page (bit positions 0..cap-1
    // on each page), so a truncated scan would miss every live slot beyond
    // the first page and the span would appear "full" at ~1/page_count
    // utilization.
    void* allocateWarm(AllocFallback* fb) {
        size_t num_words = (full_capacity + 63) / 64;
        const int close_ticks = cold_counts ? getCoolingCloseTicks() : 0;
        const int telem_ticks = cold_counts ? coolingCloseDefaultTicks() : 0;

        for (size_t i = free_hint_word; i < num_words; ++i) {
            uint64_t w = bitmap[i];
            if (!w) continue;
            int bit = pickActiveBit(i, w, close_ticks,
                                    &fb->cool_word, &fb->cool_bit,
                                    &fb->comp_word, &fb->comp_bit);
            if (bit < 0) continue;
            return takeSlot(static_cast<int>(i), bit, true, telem_ticks);
        }
        for (size_t i = 0; i < free_hint_word; ++i) {
            uint64_t w = bitmap[i];
            if (!w) continue;
            int bit = pickActiveBit(i, w, close_ticks,
                                    &fb->cool_word, &fb->cool_bit,
                                    &fb->comp_word, &fb->comp_bit);
            if (bit < 0) continue;
            return takeSlot(static_cast<int>(i), bit, true, telem_ticks);
        }
        return nullptr;  // no warm slot; fb holds any fallbacks
    }

    // Consume a fallback recorded by allocateWarm(): cooling first (streak
    // reset counts as a re-heat, but no fault), then compressed (the user's
    // first access will fault and decompress; cold_counts untouched — the
    // fault handler resets it). Returns nullptr if fb is empty (span full).
    void* consumeFallback(const AllocFallback& fb) {
        const int telem_ticks = cold_counts ? coolingCloseDefaultTicks() : 0;
        if (fb.cool_word >= 0)
            return takeSlot(fb.cool_word, fb.cool_bit, true, telem_ticks);
        if (fb.comp_word >= 0)
            return takeSlot(fb.comp_word, fb.comp_bit, false, 0);
        return nullptr;  // full
    }

    // Allocate one object. Returns nullptr if span is full.
    // Full preference cascade in one call: warm page > cooling page >
    // COMPRESSED page. Warm avoids both a decompress fault and re-heating a
    // page partway to compression; cooling costs a cold-streak reset but no
    // fault; compressed costs a fault on the user's first access. Any
    // fallback beats nullptr, which would force a whole new span.
    void* allocate() {
        AllocFallback fb;
        if (void* p = allocateWarm(&fb)) return p;
        return consumeFallback(fb);
    }

    // Free one object back to this span.
    void deallocate(void* ptr) {
        size_t offset = static_cast<size_t>(static_cast<char*>(ptr) - static_cast<char*>(base));
        size_t slot = offset / object_size;
        size_t word = slot / 64;
        size_t bit = slot % 64;

        // The bitmap is the sole allocation truth (1 = free), and this function
        // already loads and stores bitmap[word] — so testing the bit first is a
        // single `test` on a value already in a register, not extra memory
        // traffic. A set bit means the slot is ALREADY free: a double free, or
        // a free of an interior/bogus pointer that maps onto a free slot.
        //
        // Returning early is the correct recovery, not merely the polite one:
        // falling through would also run `--allocated_count`, underflowing it
        // and corrupting the full/empty transitions in Slab::deallocateOne.
        // Measured before this check existed: full mode absorbed a double free
        // silently (glibc aborts with "double free detected in tcache 2"),
        // leaving the slab uncorrupted but the error undetected.
        //
        // Coverage is real but partial, and deliberately so:
        //   - ThreadCache::deallocate pushes onto a pointer array in front of
        //     the slab, so a repeat free that stays in the cache is not seen
        //     until that lane drains;
        //   - LargeAlloc::deallocate clears the page map first, so a second
        //     free of a large block finds no span and takes the "not ours"
        //     path entirely.
        // Catching the slab case is cheap; chasing the rest would cost an O(n)
        // scan on the hot path.
        if (bitmap[word] & (1ULL << bit)) [[unlikely]] {
            // handleDoubleFree consults the policy; off returns false and we
            // fall through to the historical behaviour.
            if (smash::handleDoubleFree("slab", ptr, slot)) return;
        }

        bitmap[word] |= (1ULL << bit);   // mark free
        --allocated_count;
        if (word < free_hint_word) {
            free_hint_word = static_cast<uint16_t>(word);
        }
    }

    bool full() const { return allocated_count == object_count; }
    bool empty() const { return allocated_count == 0; }

    // Check if a pointer belongs to this span
    bool contains(const void* ptr) const {
        auto p = reinterpret_cast<uintptr_t>(ptr);
        auto b = reinterpret_cast<uintptr_t>(base);
        return p >= b && p < b + static_cast<size_t>(page_count) * kPageSize;
    }
};

// Every slab span, mid-size chunk and large block costs one of these in the
// bootstrap arena, which is never freed — so this size is what bounds the heap
// smash can cover (see "The metadata arena bounds how large a heap smash can
// manage"). Pin it so a new field is a decision, not an accident.
static_assert(sizeof(Span) <= 112,
              "Span grew: that shrinks the coverable heap proportionally");

// Span descriptor cache: recycled descriptors, reused before BootstrapAlloc.
// Both large spans and (since empty slab spans are returned to the arena)
// slab spans pass through it. It was a lock-free Treiber stack whose pop read
// s->next_free and then CASed the head — the textbook ABA race: between the
// read and the CAS another thread can pop `s`, pop its successor, and push `s`
// back, and the CAS then installs a successor that is in use. A spinlock costs
// one uncontended atomic per span creation, which is rare next to object
// allocation.
struct SpanFreeList {
    Spinlock lock;
    Span* head = nullptr;
};
inline SpanFreeList& spanFreeList() {
    static SpanFreeList list;
    return list;
}

inline Span* newSpanDescriptor() {
    SpanFreeList& fl = spanFreeList();
    Span* s = nullptr;
    {
        LockGuard guard(fl.lock);
        s = fl.head;
        if (s) fl.head = s->next_free;
    }
    if (!s) s = static_cast<Span*>(
        BootstrapAlloc::instance().allocate(sizeof(Span), alignof(Span)));
    if (!s) return nullptr;   // metadata arena exhausted (it reports that)
    // Keep the bitmap storage across the reset (see Span::bitmap_store).
    uint64_t* store = s->bitmap_store;
    const uint32_t words = s->bitmap_store_words;
    __builtin_memset(s, 0, sizeof(Span));
    s->bitmap_store = store;
    s->bitmap_store_words = words;
    return s;
}

inline void recycleSpanDescriptor(Span* s) {
    SpanFreeList& fl = spanFreeList();
    LockGuard guard(fl.lock);
    s->next_free = fl.head;
    fl.head = s;
}

} // namespace smash
