// smash/src/core/bootstrap_alloc.h - Bump allocator for internal metadata
//
// All smash metadata (span descriptors, bitmaps, page map tables, thread caches)
// is allocated from here. This avoids any dependency on the managed heap.
// Memory is never freed — just bump-allocated from mmap'd regions.
#pragma once

#include "smash/config.h"
#include "../vm/platform_mem.h"
#include "../util/spinlock.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>
#ifdef __linux__
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace smash {

class BootstrapAlloc {
    struct Region {
        char* base;
        size_t capacity;
        std::atomic<size_t> offset;
        std::atomic<size_t> committed;  // high-water of committed bytes (page-aligned)
    };

    Region regions_[kBootstrapMaxRegions];
    std::atomic<size_t> num_regions_{0};
    Spinlock expand_lock_;

    // Address bounds covering all regions added so far.  Lets owns()
    // reject the common case (a non-bootstrap pointer) with a single
    // pair of relaxed atomic loads, instead of iterating regions_.
    // Region ranges may have gaps inside [lo, hi] so a hit still falls
    // through to the precise loop below — but every free() in the
    // postgres workload reaches owns() with a slab pointer outside
    // bootstrap memory, and the bounds check rejects in 4 instructions.
    std::atomic<uintptr_t> bounds_lo_{UINTPTR_MAX};
    std::atomic<uintptr_t> bounds_hi_{0};

    static size_t pageSize() {
        static const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
        return ps;
    }

    static size_t roundUpToPage(size_t n) {
        size_t ps = pageSize();
        return (n + ps - 1) & ~(ps - 1);
    }

    // Commit [committed, roundUp(end)) of r. Caller must hold expand_lock_.
    void commitLocked(Region& r, size_t end) SMASH_NO_TS_ANALYSIS {
        size_t target = roundUpToPage(end);
        if (target > r.capacity) target = r.capacity;
        size_t cur = r.committed.load(std::memory_order_relaxed);
        if (cur >= target) return;
        vm::commitPages(r.base + cur, target - cur);
        r.committed.store(target, std::memory_order_release);
    }

    void ensureCommitted(Region& r, size_t end, bool lock_held) {
        if (end <= r.committed.load(std::memory_order_acquire)) return;
        // Serialize commits: mprotect must complete before any thread
        // may write to the newly-committed range. The lock is only
        // contended when multiple threads cross a page boundary
        // simultaneously (rare — bump allocator is fast).
        //
        // expandAndAllocate() already holds expand_lock_ (Spinlock is not
        // recursive). Taking it again there self-deadlocked the thread that
        // grew the bootstrap allocator -- a brand-new region always needs a
        // commit -- and then every other thread needing a commit (a new
        // thread's thread cache, span descriptors, TLV setup ...).
        if (lock_held) { commitLocked(r, end); return; }
        LockGuard guard(expand_lock_);
        commitLocked(r, end);
    }

    void* tryAllocFrom(Region& r, size_t size, size_t align, bool lock_held = false) {
        size_t off = r.offset.load(std::memory_order_relaxed);
        for (;;) {
            size_t aligned = (off + align - 1) & ~(align - 1);
            size_t end = aligned + size;
            if (end > r.capacity) return nullptr;
            if (r.offset.compare_exchange_weak(off, end, std::memory_order_relaxed)) {
                ensureCommitted(r, end, lock_held);
                return r.base + aligned;
            }
        }
    }

    bool addRegion(size_t size) {
        void* mem = vm::reservePages(size);
        if (!mem) return false;
        size_t idx = num_regions_.load(std::memory_order_relaxed);
        if (idx >= kBootstrapMaxRegions) {
            vm::unmapPages(mem, size);
            return false;
        }
        regions_[idx].base = static_cast<char*>(mem);
        regions_[idx].capacity = size;
        regions_[idx].offset.store(0, std::memory_order_relaxed);
        regions_[idx].committed.store(0, std::memory_order_relaxed);
        num_regions_.store(idx + 1, std::memory_order_release);

        // Extend [lo, hi] envelope so owns() can fast-reject pointers
        // that don't fall in any bootstrap region.  Atomicity matters
        // only for monotonicity; a stale bound just causes an extra
        // (correct) loop iteration in owns().
        uintptr_t base = reinterpret_cast<uintptr_t>(mem);
        uintptr_t end = base + size;
        for (uintptr_t lo = bounds_lo_.load(std::memory_order_relaxed); base < lo; ) {
            if (bounds_lo_.compare_exchange_weak(lo, base, std::memory_order_relaxed))
                break;
        }
        for (uintptr_t hi = bounds_hi_.load(std::memory_order_relaxed); end > hi; ) {
            if (bounds_hi_.compare_exchange_weak(hi, end, std::memory_order_relaxed))
                break;
        }
        return true;
    }

    void* expandAndAllocate(size_t size, size_t align) {
        LockGuard guard(expand_lock_);
        // Double-check: another thread may have expanded while we waited
        size_t n = num_regions_.load(std::memory_order_acquire);
        if (n > 0) {
            void* p = tryAllocFrom(regions_[n - 1], size, align, /*lock_held=*/true);
            if (p) return p;
        }
        size_t region_size = kBootstrapExpandSize;
        if (size + align > region_size) region_size = size + align;
        if (!addRegion(region_size)) return nullptr;
        n = num_regions_.load(std::memory_order_acquire);
        return tryAllocFrom(regions_[n - 1], size, align, /*lock_held=*/true);
    }

    BootstrapAlloc() {
        addRegion(kBootstrapInitialSize);
    }

public:
    // fork() safety: expand_lock_ serializes lazy page commits (and region
    // growth). If another thread holds it when a thread calls fork(), the
    // child inherits it locked with no owner, and the child's next commit --
    // e.g. the compressor restarting on its first new thread -- spins forever.
    // Called from smash's pthread_atfork handlers: prepare takes the lock LAST
    // (after everything that might allocate from here), parent/child release it.
    void atforkPrepare() SMASH_NO_TS_ANALYSIS { expand_lock_.lock(); }
    void atforkParent() SMASH_NO_TS_ANALYSIS { expand_lock_.unlock(); }
    void atforkChild() SMASH_NO_TS_ANALYSIS { expand_lock_.unlock(); }

public:
    // Singleton instance, accessed on every free() via owns().  Profiling
    // showed this as 6.5% of free()'s CPU when not inlined; mark hot+inline
    // so the post-init fast path becomes a single relaxed-load + branch.
    [[gnu::always_inline, gnu::hot]]
    static BootstrapAlloc& instance() {
        // Manual singleton to avoid __cxa_guard (which might call malloc on some platforms)
        alignas(BootstrapAlloc) static char buf[sizeof(BootstrapAlloc)];
        static std::atomic<int> state{0};  // 0=uninit, 1=initializing, 2=ready
        int s = state.load(std::memory_order_acquire);
        if (s == 2) [[likely]]
            return *reinterpret_cast<BootstrapAlloc*>(buf);
        if (s == 0 && state.compare_exchange_strong(s, 1, std::memory_order_acq_rel)) {
            new (buf) BootstrapAlloc();
            state.store(2, std::memory_order_release);
            return *reinterpret_cast<BootstrapAlloc*>(buf);
        }
        // Another thread initializing — spin
        while (state.load(std::memory_order_acquire) != 2) {
#if defined(__x86_64__)
            __builtin_ia32_pause();
#elif defined(__aarch64__)
            asm volatile("yield");
#endif
        }
        return *reinterpret_cast<BootstrapAlloc*>(buf);
    }

    void* allocate(size_t size, size_t align = 16) {
        if (size == 0) size = 1;
        // Try most recent region first (most likely to have space)
        size_t n = num_regions_.load(std::memory_order_acquire);
        for (size_t i = n; i > 0; --i) {
            void* p = tryAllocFrom(regions_[i - 1], size, align);
            if (p) return p;
        }
        return expandAndAllocate(size, align);
    }

    // Zero-initialized allocation. For small sizes, bump-allocate and memset.
    // For large sizes (>= 64 KB), use direct mmap which provides zero pages
    // on-demand without touching memory upfront.
    void* allocateZeroed(size_t size, size_t align = 16) {
        // Threshold: direct mmap for large allocations to avoid memset cost
        constexpr size_t kDirectMmapThreshold = 64 * 1024;  // 64 KB
        if (size >= kDirectMmapThreshold) {
            // Round up to page size for mmap
            size_t page_size = static_cast<size_t>(sysconf(_SC_PAGESIZE));
            size_t rounded = (size + page_size - 1) & ~(page_size - 1);
            // Use syscall directly to bypass our mmap interposer. This is
            // internal smash metadata that must NOT be tracked as external
            // pages (would cause infinite recursion or corruption).
#ifdef __linux__
            void* p = reinterpret_cast<void*>(
                syscall(SYS_mmap, nullptr, rounded, PROT_READ | PROT_WRITE,
                        MAP_PRIVATE | MAP_ANON, -1, 0));
            return (p == MAP_FAILED) ? nullptr : p;
#else
            // macOS: use vm::mapPages which doesn't go through our interposer
            return vm::mapPages(rounded);
#endif
        }
        // Small allocation: bump-allocate and memset
        void* p = allocate(size, align);
        if (p) __builtin_memset(p, 0, size);
        return p;
    }

    [[gnu::always_inline, gnu::hot]]
    bool owns(const void* ptr) const {
        auto p = reinterpret_cast<uintptr_t>(ptr);
        // Fast-reject: ptr outside any bootstrap region's envelope.
        // The two relaxed loads are 2 instructions on aarch64; the
        // alternative (the loop below) is 5+ per region.
        if (p < bounds_lo_.load(std::memory_order_relaxed)) return false;
        if (p >= bounds_hi_.load(std::memory_order_relaxed)) return false;
        // Inside envelope but possibly in a gap between regions — fall
        // through to the precise scan.
        size_t n = num_regions_.load(std::memory_order_acquire);
        for (size_t i = 0; i < n; ++i) {
            auto base = reinterpret_cast<uintptr_t>(regions_[i].base);
            if (p >= base && p < base + regions_[i].capacity) return true;
        }
        return false;
    }
};

// Convenience: allocate a zeroed array of T from bootstrap
template<typename T>
T* bootstrapArray(size_t count) {
    size_t bytes = count * sizeof(T);
    return static_cast<T*>(BootstrapAlloc::instance().allocateZeroed(bytes, alignof(T)));
}

} // namespace smash
