// smash/src/core/page_map.h - Two-level radix tree: virtual address → Span*
//
// Provides O(1) lookup from any address to the Span that manages it.
// L1 table allocated eagerly from bootstrap; L2 tables allocated lazily.
//
// Optionally mirrors writes into a VmRegion's flat page-index → Span* table
// when the address is inside the VmRegion. The flat table gives the fast
// path a single-load lookup; the radix tree remains the authoritative source
// for addresses that escape the VmRegion (large allocs that bypass it).
#pragma once

#include "smash/config.h"
#include "bootstrap_alloc.h"
#include "span.h"
#include "../vm/vm_region.h"
#include "../util/spinlock.h"

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace smash {

// Global page→span mapping generation. Bumped every time a page's span
// mapping is torn down (PageMap::set(addr, nullptr) — the universal teardown
// signal for both Slab::releaseSpan and LargeAlloc::deallocate). Address-keyed
// span caches (e.g. the free() TLS last-span cache) stamp this value at
// populate time and discard the cache on mismatch.
//
// Correctness rests on the load-before-read ordering at the cache site: read
// the generation BEFORE reading the span mapping. A teardown that races the
// cache fill bumps the counter, so a cached entry whose stamped generation
// still equals the current one provably reflects a mapping that has not been
// torn down since. A stale-low stamp only forces an extra cache miss; it can
// never yield a span for a range that has since been reassigned to a different
// size class. Teardowns are rare, so this counter stays read-mostly and
// cache-resident — that is the whole point versus re-reading the 128 MB flat
// page→span table on every free.
inline std::atomic<uint64_t> g_pagemap_generation{0};

class PageMap {
    using L2Entry = std::atomic<Span*>;
    using L2Table = L2Entry*;

    std::atomic<L2Table>* level1_;   // kPageMapL1Size entries, from bootstrap
    Spinlock l2_lock_;
    // Optional VmRegion whose flat page→Span table mirrors radix writes.
    // Set via attachVmRegion(); nullptr falls back to radix-only behavior.
    VmRegion* vm_region_ = nullptr;

    L2Table ensureL2(size_t l1_idx) {
        L2Table tbl = level1_[l1_idx].load(std::memory_order_acquire);
        if (tbl) [[likely]] return tbl;
        LockGuard guard(l2_lock_);
        tbl = level1_[l1_idx].load(std::memory_order_relaxed);
        if (tbl) return tbl;
        tbl = static_cast<L2Table>(
            BootstrapAlloc::instance().allocateZeroed(
                kPageMapL2Size * sizeof(L2Entry), alignof(L2Entry)));
        level1_[l1_idx].store(tbl, std::memory_order_release);
        return tbl;
    }

    // If the address falls inside the attached VmRegion, mirror the write
    // into its flat table. Always also update the radix tree so external
    // (non-VmRegion) callers continue to resolve via PageMap::get().
    [[gnu::always_inline]]
    void mirrorFlat(uintptr_t addr, Span* span) {
        if (!vm_region_) return;
        if (!vm_region_->contains(addr)) return;
        size_t idx = vm_region_->pageIndex(addr);
        // pageIndex() returns 0 for "not present in tracking mode"; in full
        // mode index 0 is a legitimate contig-arena page, but contains()
        // gates entry — so a 0 here is fine because we only land here when
        // the address is inside the region.
        vm_region_->setSpan(idx, span);
    }

public:
    // fork(): l2_lock_ nests inside slab locks and around BootstrapAlloc.
    void lockForFork() SMASH_NO_TS_ANALYSIS { l2_lock_.lock(); }
    void unlockForFork() SMASH_NO_TS_ANALYSIS { l2_lock_.unlock(); }

public:
    void init() {
        level1_ = static_cast<std::atomic<L2Table>*>(
            BootstrapAlloc::instance().allocateZeroed(
                kPageMapL1Size * sizeof(std::atomic<L2Table>),
                alignof(std::atomic<L2Table>)));
    }

    // Attach a VmRegion so subsequent set/setRange/clearRange operations
    // also update its flat page→Span table. Must be called before any
    // allocation activity routes through this PageMap if the flat table
    // is to be consistent with the radix tree.
    void attachVmRegion(VmRegion* vr) { vm_region_ = vr; }

    void set(uintptr_t addr, Span* span) {
        const auto page_idx = addr >> kPageShift;
        const auto l1 = page_idx >> kPageMapL2Bits;
        const auto l2 = page_idx & (kPageMapL2Size - 1);
        L2Table tbl = ensureL2(l1);
        tbl[l2].store(span, std::memory_order_release);
        mirrorFlat(addr, span);
        // Teardown of a page→span mapping invalidates any address-keyed span
        // cache. Bump after the store so observers that reload the generation
        // and re-resolve see the cleared mapping. release ordering pairs with
        // the acquire load at the cache site.
        if (span == nullptr) {
            g_pagemap_generation.fetch_add(1, std::memory_order_release);
        }
    }

    Span* get(uintptr_t addr) const {
        const auto page_idx = addr >> kPageShift;
        const auto l1 = page_idx >> kPageMapL2Bits;
        const auto l2 = page_idx & (kPageMapL2Size - 1);
        L2Table tbl = level1_[l1].load(std::memory_order_acquire);
        if (!tbl) [[unlikely]] return nullptr;
        return tbl[l2].load(std::memory_order_acquire);
    }

    // Set all pages in a range to the same span. Called per span create/teardown
    // with all pages contiguous and mapping to one Span*. The naive form
    // (num_pages × set()) redundantly re-derives the L2 table and re-runs the
    // VmRegion dispatch for every page; here we resolve the radix L2 table(s)
    // once and mirror the whole run into the flat table in a single batched
    // store loop. Correctness is identical to looping set().
    void setRange(uintptr_t addr, size_t num_pages, Span* span) {
        if (num_pages == 0) return;
        const auto first_page = addr >> kPageShift;
        const auto last_page  = first_page + num_pages - 1;

        // Radix tree: the L2 index is page_idx & (kPageMapL2Size-1); a run
        // crosses an L2 table only if it straddles a kPageMapL2Size boundary
        // (262144 pages = 1 GiB). Spans are at most kMaxSpanPages, so this is
        // effectively always the single-L2 fast path; the multi-L2 case is
        // handled correctly by walking table by table.
        size_t page = first_page;
        while (page <= last_page) {
            const auto l1 = page >> kPageMapL2Bits;
            const auto l2 = page & (kPageMapL2Size - 1);
            L2Table tbl = ensureL2(l1);
            // Fill to the end of this L2 table or the end of the run.
            const size_t l2_end = (l2 + (last_page - page) < kPageMapL2Size - 1)
                ? l2 + (last_page - page)
                : kPageMapL2Size - 1;
            for (size_t s = l2; s <= l2_end; ++s)
                tbl[s].store(span, std::memory_order_release);
            page += (l2_end - l2) + 1;
        }

        // Flat table mirror: one bounds-checked batched store run.
        if (vm_region_ && vm_region_->contains(addr)) {
            vm_region_->setSpanRange(vm_region_->pageIndex(addr), num_pages, span);
        }

        // A teardown (span == nullptr) invalidates address-keyed span caches.
        // Bump once for the whole run, after the stores. Matches the per-page
        // set() semantics (which bumps once per cleared page) closely enough —
        // the cache only needs to observe *a* change, and a single monotonic
        // bump per teardown is sufficient and cheaper.
        if (span == nullptr) {
            g_pagemap_generation.fetch_add(1, std::memory_order_release);
        }
    }

    // Clear all pages in a range
    void clearRange(uintptr_t addr, size_t num_pages) {
        setRange(addr, num_pages, nullptr);
    }
};

} // namespace smash
