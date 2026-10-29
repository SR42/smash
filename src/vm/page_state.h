// smash/src/vm/page_state.h - Per-page state machine
//
// Tracks the lifecycle of every page in the VmRegion:
//   EMPTY → ACTIVE → ACTIVE_MONITORING → COMPRESSING → COMPRESSED → ACTIVE
//   Deferred-reclaim mode adds: COMPRESSING → COMPRESSED_SHADOW → COMPRESSED
#pragma once

#include "smash/config.h"
#include "../core/bootstrap_alloc.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <pthread.h>

namespace smash {

enum class PageState : uint8_t {
    EMPTY             = 0,  // Virtual reservation only, not committed
    ACTIVE            = 1,  // Committed, PROT_READ|PROT_WRITE
    COMPRESSED        = 2,  // Data compressed, page is PROT_NONE
    COMPRESSING       = 3,  // Compressor thread is working on this page
    ACTIVE_MONITORING = 4,  // PROT_READ for write-fault access tracking
    COMPRESSED_SHADOW = 5,  // Compressed copy stored; page still PROT_RW (deferred reclaim)
};

// Set the first time any page is driven to COMPRESSED. The span bitmap walk
// (Span::pickActiveBit / classifySlotPage) reads a per-slot page-state byte
// to avoid handing out chunks on COMPRESSED pages — but until the compressor
// has actually compressed something (and always, when compression is disabled
// via SMASH_NO_COMPRESSOR) no page can be COMPRESSED, so that per-slot atomic
// load is pure overhead on the allocation hot path. This flag lets the walk
// short-circuit it: false → no page is COMPRESSED → skip the check entirely.
// One monotonic 0→1 transition, only ever set on the (comparatively rare)
// compression path, only ever read (relaxed) on the alloc path.
inline std::atomic<bool> g_any_page_compressed{false};

class PageStateTable {
    std::atomic<uint8_t>* states_;
    // One bit per page: the allocator handed out a block on the page while
    // the compressor held it (see markAllocReopened).
    std::atomic<uint64_t>* alloc_reopened_ = nullptr;
    // One bit per page: its state is not EMPTY. Kept here, by every store,
    // so the compressor's tick can walk the pages that exist instead of
    // reading every state byte up to the high-water mark first
    // (rebuildChunkBitmap: 134M bytes a tick on a 512 GiB arena, on the
    // coordinator alone). Advisory: a phase re-reads the state of each page
    // the bitmap names, as it always had to (pages free mid-tick).
    std::atomic<uint64_t>* live_ = nullptr;
    size_t num_pages_;

    void noteState(size_t i, PageState st) {
        auto& w = live_[i >> 6];
        const uint64_t bit = uint64_t{1} << (i & 63);
        const bool set = w.load(std::memory_order_relaxed) & bit;
        if (st == PageState::EMPTY) { if (set) w.fetch_and(~bit, std::memory_order_relaxed); }
        else if (!set) w.fetch_or(bit, std::memory_order_relaxed);
    }

public:
    void init(size_t num_pages) {
        num_pages_ = num_pages;
        states_ = bootstrapArray<std::atomic<uint8_t>>(num_pages);
        alloc_reopened_ = bootstrapArray<std::atomic<uint64_t>>((num_pages + 63) / 64);
        live_ = bootstrapArray<std::atomic<uint64_t>>((num_pages + 63) / 64);
    }

    // 64 pages per word, bit b of word w: page 64w + b is not EMPTY.
    const std::atomic<uint64_t>* liveWords() const { return live_; }

    PageState get(size_t page_idx) const {
        return static_cast<PageState>(states_[page_idx].load(std::memory_order_acquire));
    }

    // For the allocation path, where a stale value is harmless (see
    // SmashHeap::noteBlockOnHeldPages).
    PageState getRelaxed(size_t page_idx) const {
        return static_cast<PageState>(states_[page_idx].load(std::memory_order_relaxed));
    }

    // The allocator may hand out a block on a page the compressor holds
    // (COMPRESSED, COMPRESSED_SHADOW or COMPRESSING): the thread cache
    // returns a slot freed into it before the page went cold, and the slab
    // falls back to one when a span has no warmer free slot. The program's
    // first write to the block then restores the page, and that restore says
    // nothing about whether the page was compressed too early: the
    // recompression back-off must not count it, or the page waited 4x the
    // cold timeout before it was compressed again and every fresh page of its
    // call site 2x (test_alloc_reopen). The allocator marks the page; the
    // fault handler takes the mark. A mark left by a block never written
    // makes the page's next restore go uncounted -- once.
    void markAllocReopened(size_t page_idx) {
        alloc_reopened_[page_idx >> 6].fetch_or(uint64_t{1} << (page_idx & 63), std::memory_order_relaxed);
    }
    bool takeAllocReopened(size_t page_idx) {
        const uint64_t bit = uint64_t{1} << (page_idx & 63);
        auto& w = alloc_reopened_[page_idx >> 6];
        if (!(w.load(std::memory_order_relaxed) & bit)) return false;
        return (w.fetch_and(~bit, std::memory_order_relaxed) & bit) != 0;
    }

    void set(size_t page_idx, PageState state) {
        states_[page_idx].store(static_cast<uint8_t>(state), std::memory_order_release);
        noteState(page_idx, state);
        // Latch the global "some page is compressed" flag so the alloc-path
        // bitmap walk can skip its per-slot COMPRESSED check until this fires.
        if (state == PageState::COMPRESSED) [[unlikely]]
            g_any_page_compressed.store(true, std::memory_order_relaxed);
    }

    // Set a contiguous run of pages to the same state in one pass. Used by the
    // span-create path (every page of a fresh span → ACTIVE) so the per-page
    // store loop isn't re-entered through set()'s COMPRESSED-latch branch each
    // iteration. Semantically identical to looping set().
    void setRange(size_t first_page_idx, size_t num_pages, PageState state) {
        const auto v = static_cast<uint8_t>(state);
        for (size_t i = 0; i < num_pages; ++i)
            states_[first_page_idx + i].store(v, std::memory_order_release);
        // The live bits, a word at a time.
        for (size_t i = first_page_idx, end = first_page_idx + num_pages; i < end;) {
            const size_t word_end = (i | 63) + 1;
            const size_t stop = word_end < end ? word_end : end;
            uint64_t mask = ~uint64_t{0};
            if (stop - i < 64) mask = ((uint64_t{1} << (stop - i)) - 1) << (i & 63);
            auto& w = live_[i >> 6];
            if (state == PageState::EMPTY) {
                if (w.load(std::memory_order_relaxed) & mask) w.fetch_and(~mask, std::memory_order_relaxed);
            } else if ((w.load(std::memory_order_relaxed) & mask) != mask) {
                w.fetch_or(mask, std::memory_order_relaxed);
            }
            i = stop;
        }
        if (state == PageState::COMPRESSED) [[unlikely]]
            g_any_page_compressed.store(true, std::memory_order_relaxed);
    }

    // CAS transition. Returns true on success.
    bool transition(size_t page_idx, PageState expected, PageState desired) {
        auto exp = static_cast<uint8_t>(expected);
        const bool ok = states_[page_idx].compare_exchange_strong(
            exp, static_cast<uint8_t>(desired),
            std::memory_order_acq_rel, std::memory_order_acquire);
        if (ok) noteState(page_idx, desired);
        return ok;
    }

    size_t numPages() const { return num_pages_; }
};

// Per-page spinlock table for compression synchronization.
// Fine-grained: one atomic_flag per page, plus an owner-TID slot so the
// fault handler can detect when the current thread already holds the
// lock (and bail out instead of self-deadlocking on the spinlock).
//
// Portability: pthread_t is available on both Linux and macOS, compares
// with pthread_equal. The owner slot is a plain (non-atomic) pthread_t —
// only the lock owner reads/writes it, so no atomicity needed beyond the
// release fence implied by atomic_flag.clear().
class PageLockTable {
    std::atomic_flag* locks_;
    pthread_t* owners_;
    size_t num_pages_;

public:
    void init(size_t num_pages) {
        num_pages_ = num_pages;
        // atomic_flag is 1 byte; bootstrap returns zeroed memory (= clear state)
        locks_ = static_cast<std::atomic_flag*>(
            BootstrapAlloc::instance().allocateZeroed(
                num_pages * sizeof(std::atomic_flag),
                alignof(std::atomic_flag)));
        owners_ = static_cast<pthread_t*>(
            BootstrapAlloc::instance().allocateZeroed(
                num_pages * sizeof(pthread_t),
                alignof(pthread_t)));
    }

    // A fork child: every page lock held at the clone -- by a worker mid-tick,
    // or by a thread inside the fault handler (lock, decompress, write,
    // mprotect, unlock: microseconds, but 2 of 200 forks under 8 faulting
    // threads landed inside) -- belongs to a thread that does not exist here.
    // Only the forking thread does, and it holds none, so every set flag is
    // cleared, owner too (a thread the child creates can reuse a parent
    // thread's pthread_t, and heldByThisThread would then see itself).
    // Reads a word per 8 pages over [0, n) and writes only the held ones, so
    // the lock table's untouched pages stay shared with the parent. Returns
    // how many were held.
    size_t resetForFork(size_t n) {
        static_assert(sizeof(std::atomic_flag) == 1, "the word scan reads one page lock per byte");
        if (n > num_pages_) n = num_pages_;
        size_t reset = 0;
        const unsigned char* raw = reinterpret_cast<const unsigned char*>(locks_);
        size_t i = 0;
        for (; i + 8 <= n; i += 8) {
            uint64_t w;
            std::memcpy(&w, raw + i, sizeof w);
            if (w == 0) continue;
            for (size_t k = i; k < i + 8; ++k) reset += resetOneForFork(k);
        }
        for (; i < n; ++i) reset += resetOneForFork(i);
        return reset;
    }

    void lock(size_t page_idx) {
        while (locks_[page_idx].test_and_set(std::memory_order_acquire)) {
#if defined(__x86_64__)
            __builtin_ia32_pause();
#elif defined(__aarch64__)
            asm volatile("yield");
#endif
        }
        owners_[page_idx] = pthread_self();
    }

    // Non-blocking lock attempt. Returns true if lock was acquired.
    bool tryLock(size_t page_idx) {
        if (locks_[page_idx].test_and_set(std::memory_order_acquire)) return false;
        owners_[page_idx] = pthread_self();
        return true;
    }

    size_t resetOneForFork(size_t i) {
        if (!locks_[i].test(std::memory_order_relaxed)) return 0;
        owners_[i] = pthread_t{};
        locks_[i].clear(std::memory_order_relaxed);
        return 1;
    }

    void unlock(size_t page_idx) {
        // Clear owner before releasing the flag — otherwise another thread
        // can grab the lock, write a new owner, and then we'd clobber it.
        owners_[page_idx] = pthread_t{};
        locks_[page_idx].clear(std::memory_order_release);
    }

    // Used by the fault handler to detect self-recursion. compressPage
    // holds the lock while doing memcpy after PROT_READ; if the memcpy
    // SIGSEGVs (TLB inconsistency or smash state-machine bug), the
    // signal is delivered to the same thread, and handleFault must NOT
    // call lock() again — that would self-deadlock on the non-recursive
    // spinlock. Returns true if THIS thread is the current lock owner.
    bool heldByThisThread(size_t page_idx) const {
        // Read order: check the flag is set, then read owner. If flag is
        // clear, no one holds the lock; owner read is racy but irrelevant.
        // If flag is set and owner equals pthread_self(), it's us.
        // (We assume pthread_t reads are atomic enough on the architectures
        // we care about — pointer-sized on Linux glibc, opaque struct on
        // some platforms but in practice 8-byte aligned. The test_and_set
        // earlier means the lock holder already wrote owners_[page_idx];
        // we're just reading the value back.)
        if (!locks_[page_idx].test(std::memory_order_acquire)) return false;
        return pthread_equal(owners_[page_idx], pthread_self()) != 0;
    }
};

} // namespace smash
