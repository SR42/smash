// smash/src/vm/page_state.h - Per-page state machine
//
// Tracks the lifecycle of every page in the VmRegion:
//   EMPTY → ACTIVE → ACTIVE_MONITORING → COMPRESSING → COMPRESSED → ACTIVE
//   Deferred-reclaim mode adds: COMPRESSING → COMPRESSED_SHADOW → COMPRESSED
#pragma once

#include "smash/config.h"
#include "../core/bootstrap_alloc.h"

#include <atomic>
#include "../util/pressure_stats.h"
#include <cstddef>
#include <cstdint>
#include <cstring>
#include "../util/park.h"
#include "../util/spinlock.h"
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
        if (pstats::g_stamp) [[unlikely]] pstatsHook(page_idx, state);
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
        if (pstats::g_stamp) [[unlikely]] {
            for (size_t i = 0; i < num_pages; ++i) pstatsHook(first_page_idx + i, state);
        }
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

private:
    // SMASH_PRESSURE_LOG accounting (util/pressure_stats.h). Runs before the
    // store so a page's stamp is set before any reader can see it compressed.
    static void pstatsHook(size_t page_idx, PageState state) {
        pstats::onTransition(page_idx,
            state == PageState::COMPRESSED || state == PageState::COMPRESSED_SHADOW,
            state == PageState::COMPRESSING, state == PageState::EMPTY);
    }
};

// Per-page lock table for compression synchronization.
// Fine-grained: one byte per page (0 free, 1 held, 2 held with a waiter
// parked), plus an owner-TID slot so the fault handler can detect when the
// current thread already holds the lock (and bail out instead of
// self-deadlocking).
//
// A waiter spins for about the microseconds a normal hold lasts (a restore:
// decompress, a write through /proc/self/mem, an mprotect) and then parks on
// a futex: the wait words are a small hashed table (util/park.h), the unlock
// of a lock whose byte says 2 bumps the page's word and wakes everyone on it.
// Before, the byte was a test-and-set spun on for ever, and it is faulting
// application threads that wait on it: a thread waiting a second for a
// stalled holder burnt a CPU for that second, 128 of them 128 CPUs. The
// protocol is the three-state mutex with the futex word kept apart: a waiter
// reads the word, then exchanges the byte to 2; if that found the lock free
// it owns it, otherwise it sleeps while the word is unchanged. An unlock
// that ran in between saw the 2, so it changed the word, and the sleep
// returns at once. Waiters parked on a colliding word wake spuriously and
// re-check.
//
// Portability: pthread_t is available on both Linux and macOS, compares
// with pthread_equal. The owner slot is a plain (non-atomic) pthread_t --
// only the lock owner reads/writes it, so no atomicity needed beyond the
// release fence implied by the byte's release store.
class PageLockTable {
    std::atomic<uint8_t>* locks_;
    pthread_t* owners_;
    size_t num_pages_;
    static constexpr size_t kWaitWords = 1024;
    std::atomic<uint32_t>* wait_words_;
    // About 20 us of read-spinning before parking (pause/yield is tens of
    // nanoseconds): longer than a restore, shorter than anything worth a
    // CPU.
    static constexpr unsigned kSpinBeforePark = 1000;

    std::atomic<uint32_t>& waitWord(size_t page_idx) {
        return wait_words_[(page_idx * 0x9E3779B97F4A7C15ull >> 54) & (kWaitWords - 1)];
    }

public:
    void init(size_t num_pages) {
        num_pages_ = num_pages;
        // bootstrap returns zeroed memory (= free state)
        locks_ = static_cast<std::atomic<uint8_t>*>(
            BootstrapAlloc::instance().allocateZeroed(
                num_pages * sizeof(std::atomic<uint8_t>),
                alignof(std::atomic<uint8_t>)));
        owners_ = static_cast<pthread_t*>(
            BootstrapAlloc::instance().allocateZeroed(
                num_pages * sizeof(pthread_t),
                alignof(pthread_t)));
        wait_words_ = static_cast<std::atomic<uint32_t>*>(
            BootstrapAlloc::instance().allocateZeroed(
                kWaitWords * sizeof(std::atomic<uint32_t>),
                alignof(std::atomic<uint32_t>)));
    }

    // A fork child: every page lock held at the clone -- by a worker mid-tick,
    // or by a thread inside the fault handler (lock, decompress, write,
    // mprotect, unlock: microseconds, but 2 of 200 forks under 8 faulting
    // threads landed inside) -- belongs to a thread that does not exist here.
    // Only the forking thread does, and it holds none, so every held byte is
    // cleared, owner too (a thread the child creates can reuse a parent
    // thread's pthread_t, and heldByThisThread would then see itself).
    // Reads a word per 8 pages over [0, n) and writes only the held ones, so
    // the lock table's untouched pages stay shared with the parent. Returns
    // how many were held. No thread is parked in the child, so the wait
    // words need nothing.
    size_t resetForFork(size_t n) {
        static_assert(sizeof(std::atomic<uint8_t>) == 1, "the word scan reads one page lock per byte");
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
        std::atomic<uint8_t>& b = locks_[page_idx];
        if (tryAcquire(b)) { owners_[page_idx] = pthread_self(); return; }
        for (;;) {
            // Read-spin (no RMW: the line stays shared while it is held).
            for (unsigned i = 0; i < kSpinBeforePark; ++i) {
                if (b.load(std::memory_order_relaxed) == 0 && tryAcquire(b)) {
                    owners_[page_idx] = pthread_self();
                    return;
                }
                spinRelax();
            }
            // Park. The word is read before the byte is marked: an unlock
            // between the two sees the mark and changes the word.
            std::atomic<uint32_t>& w = waitWord(page_idx);
            const uint32_t seq = w.load(std::memory_order_acquire);
            if (b.exchange(2, std::memory_order_acquire) == 0) {
                owners_[page_idx] = pthread_self();
                return;
            }
            park(w, seq);
        }
    }

    // Non-blocking lock attempt. Returns true if lock was acquired.
    bool tryLock(size_t page_idx) {
        if (!tryAcquire(locks_[page_idx])) return false;
        owners_[page_idx] = pthread_self();
        return true;
    }

    size_t resetOneForFork(size_t i) {
        if (locks_[i].load(std::memory_order_relaxed) == 0) return 0;
        owners_[i] = pthread_t{};
        locks_[i].store(0, std::memory_order_relaxed);
        return 1;
    }

    void unlock(size_t page_idx) {
        // Clear owner before releasing the byte -- otherwise another thread
        // can grab the lock, write a new owner, and then we'd clobber it.
        owners_[page_idx] = pthread_t{};
        if (locks_[page_idx].exchange(0, std::memory_order_release) == 2) {
            std::atomic<uint32_t>& w = waitWord(page_idx);
            w.fetch_add(1, std::memory_order_release);
            unpark(w, 0x7fffffff);
        }
    }

    // Used by the fault handler to detect self-recursion. compressPage
    // holds the lock while doing memcpy after PROT_READ; if the memcpy
    // SIGSEGVs (TLB inconsistency or smash state-machine bug), the
    // signal is delivered to the same thread, and handleFault must NOT
    // call lock() again -- that would self-deadlock on the non-recursive
    // lock. Returns true if THIS thread is the current lock owner.
    bool heldByThisThread(size_t page_idx) const {
        // Read order: check the byte is set, then read owner. If the byte is
        // clear, no one holds the lock; owner read is racy but irrelevant.
        // If it is set and owner equals pthread_self(), it's us.
        if (locks_[page_idx].load(std::memory_order_acquire) == 0) return false;
        return pthread_equal(owners_[page_idx], pthread_self()) != 0;
    }

private:
    static bool tryAcquire(std::atomic<uint8_t>& b) {
        uint8_t expected = 0;
        return b.compare_exchange_strong(expected, 1, std::memory_order_acquire, std::memory_order_relaxed);
    }
};

} // namespace smash
