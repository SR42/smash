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
    size_t num_pages_;

public:
    void init(size_t num_pages) {
        num_pages_ = num_pages;
        states_ = bootstrapArray<std::atomic<uint8_t>>(num_pages);
    }

    PageState get(size_t page_idx) const {
        return static_cast<PageState>(states_[page_idx].load(std::memory_order_acquire));
    }

    void set(size_t page_idx, PageState state) {
        if (pstats::g_stamp) [[unlikely]] pstatsHook(page_idx, state);
        states_[page_idx].store(static_cast<uint8_t>(state), std::memory_order_release);
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
        if (state == PageState::COMPRESSED) [[unlikely]]
            g_any_page_compressed.store(true, std::memory_order_relaxed);
    }

    // CAS transition. Returns true on success.
    bool transition(size_t page_idx, PageState expected, PageState desired) {
        auto exp = static_cast<uint8_t>(expected);
        return states_[page_idx].compare_exchange_strong(
            exp, static_cast<uint8_t>(desired),
            std::memory_order_acq_rel, std::memory_order_acquire);
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
        return pthread_equal(owners_[page_idx], pthread_self()) != 0;
    }
};

} // namespace smash
