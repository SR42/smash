// smash/src/vm/platform_mem.h - Raw OS memory primitives (never calls malloc)
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cerrno>
#include "../util/safe_printf.h"  // allocation-free snprintf

#if defined(_WIN32)
#include <windows.h>
#else
#include <sys/mman.h>
#include <unistd.h>
#if defined(__linux__)
#include <sys/syscall.h>
#include <linux/mempolicy.h>
// MADV_COLD / MADV_PAGEOUT are stable kernel ABI constants (Linux 5.4+) but
// only appear in <sys/mman.h> from glibc 2.31. We build release artifacts on
// an older glibc (manylinux, glibc 2.28) for drop-in portability, so define
// the fallbacks here. The values are fixed by the kernel ABI; the call is a
// best-effort hint and no-ops on kernels that don't support it.
#ifndef MADV_COLD
#define MADV_COLD 20
#endif
#ifndef MADV_PAGEOUT
#define MADV_PAGEOUT 21
#endif
#endif
#endif

namespace smash::vm {

// Global bounds for smash's VmRegion. Set during init; protectPages/remapPages/
// decommitPages assert all calls target addresses within this range.
inline void* g_vm_bounds_base = nullptr;
inline size_t g_vm_bounds_size = 0;
inline void setVmBounds(void* base, size_t size) {
    g_vm_bounds_base = base;
    g_vm_bounds_size = size;
}

// Optional predicate for externally-tracked pages (SMASH_TRACK_EXTERNAL=1).
// Such pages are app-direct mmap regions registered with the VmRegion; their
// addresses are legitimately OUTSIDE the contiguous [g_vm_bounds_base, +size)
// arena, so the bounds guards below would false-trap on them. When external
// tracking is active VmRegion sets this to its contains() check; the guards
// treat an address as in-bounds if it is either inside the contiguous arena OR
// a registered external page. Null (default) → contiguous-only, no behavior
// change for the common case. Must be async-signal-safe (lock-free hash read).
inline bool (*g_external_page_check)(uintptr_t addr) = nullptr;
inline void setExternalPageCheck(bool (*fn)(uintptr_t)) {
    g_external_page_check = fn;
}

// True if [addr, addr+size) is a legitimate smash decommit/mprotect target:
// inside the contiguous arena, or (when tracking) a registered external page.
inline bool vmAddrAllowed(uintptr_t a, size_t size) {
    auto base = reinterpret_cast<uintptr_t>(g_vm_bounds_base);
    if (a >= base && a + size <= base + g_vm_bounds_size) return true;
    // External page: per-page granular ops (size<=kPageSize); check the page.
    if (g_external_page_check && g_external_page_check(a)) return true;
    return false;
}

// Raw mmap/munmap that BYPASS libsmash's own exported mmap/munmap wrappers.
//
// libsmash exports mmap/munmap (SMASH_TRACK_EXTERNAL interposition), and a
// preloaded library's internal PLT calls to those symbols resolve to its OWN
// wrappers. So every internal mapPages() was re-entering the interposer and —
// under SMASH_TRACK_EXTERNAL=1 — registering smash's own PROT_RW mappings
// (e.g. LargeAlloc's direct-mmap fallback) as "external app pages". That
// poisoned VmRegion::contains(), which LargeAlloc::deallocate used to route
// frees: tracked fallback pages went to releasePages() with an EXTERNAL page
// index, the arena free list recycled that index as an ARENA OFFSET, and
// allocatePages minted pointers past the arena end → app-memory corruption
// (issue #84, FFmpeg av_expr_free SIGSEGV / decommitPages OOB trap).
// Direct syscalls also make these safe from PLT lazy-resolution in fault paths
// (same rationale as SYS_mprotect in protectPages below).
#if defined(__linux__)
inline void* rawMmap(void* addr, size_t size, int prot, int flags) {
    long r = syscall(SYS_mmap, addr, size, prot, flags, -1, 0);
    return (r == -1) ? MAP_FAILED : reinterpret_cast<void*>(r);
}
inline int rawMunmap(void* addr, size_t size) {
    return static_cast<int>(syscall(SYS_munmap, addr, size));
}
#elif !defined(_WIN32)
// macOS: DYLD __interpose rewrites callers OUTSIDE the interposing image only;
// libsmash's own calls to mmap/munmap are not self-interposed, so the libc
// symbols are already "raw" here.
inline void* rawMmap(void* addr, size_t size, int prot, int flags) {
    return mmap(addr, size, prot, flags, -1, 0);
}
inline int rawMunmap(void* addr, size_t size) { return munmap(addr, size); }
#endif

// Map anonymous private pages. Returns nullptr on failure.
inline void* mapPages(size_t size) {
#if defined(_WIN32)
    return VirtualAlloc(nullptr, size, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE);
#else
    void* p = rawMmap(nullptr, size, PROT_READ | PROT_WRITE,
                      MAP_PRIVATE | MAP_ANON);
    return (p == MAP_FAILED) ? nullptr : p;
#endif
}

// Reserve virtual address space without committing physical pages.
inline void* reservePages(size_t size) {
#if defined(_WIN32)
    return VirtualAlloc(nullptr, size, MEM_RESERVE, PAGE_NOACCESS);
#elif defined(__linux__)
    void* p = rawMmap(nullptr, size, PROT_NONE,
                      MAP_PRIVATE | MAP_ANON | MAP_NORESERVE);
    return (p == MAP_FAILED) ? nullptr : p;
#else
    // macOS: pages are lazily backed, plain mmap suffices
    void* p = rawMmap(nullptr, size, PROT_NONE,
                      MAP_PRIVATE | MAP_ANON);
    return (p == MAP_FAILED) ? nullptr : p;
#endif
}

// Commit pages within a reserved region (make them accessible).
inline bool commitPages(void* addr, size_t size) {
#if defined(_WIN32)
    return VirtualAlloc(addr, size, MEM_COMMIT, PAGE_READWRITE) != nullptr;
#else
    return mprotect(addr, size, PROT_READ | PROT_WRITE) == 0;
#endif
}

// Decommit pages (release physical backing but keep virtual reservation).
// Cached at first call (safe — decommitPages is first called during init
// before the fault handler is active). NOT safe if first call is from the
// compressor thread inside a signal context; warmupEnvStatics() must
// trigger this before the compressor starts.
inline int madvMode() {
    static const int m = []{
        const char* v = std::getenv("SMASH_MADV_FREE");
        return (v && v[0] == '1') ? MADV_FREE : MADV_DONTNEED;
    }();
    return m;
}

inline bool noDecommitEnabled() {
    static const bool v = []{
        const char* e = std::getenv("SMASH_NO_DECOMMIT");
        return e && e[0] == '1';
    }();
    return v;
}

inline void decommitPages(void* addr, size_t size) {
    if (noDecommitEnabled()) return;
    // Bounds check: madvise(DONTNEED) on wrong address drops backing for
    // libc/ld.so pages, causing mysterious SIGSEGV later.
    if (g_vm_bounds_base && size <= 4096 * 64) {
        auto a = reinterpret_cast<uintptr_t>(addr);
        if (!vmAddrAllowed(a, size)) {
            char buf[160];
            int n = smash::safe_snprintf(buf, sizeof(buf),
                "[smash FATAL] decommitPages OOB: addr=%p size=%zu "
                "vm=[%p, +%zu]\n",
                addr, size, g_vm_bounds_base, g_vm_bounds_size);
            if (n > 0) (void)!::write(2, buf, (size_t)n);
            __builtin_trap();
        }
    }
#if defined(_WIN32)
    VirtualFree(addr, size, MEM_DECOMMIT);
#elif defined(__linux__)
    // MADV_DONTNEED immediately frees the physical page AND shoots down the
    // TLB on every core mapping it — the dominant smash-induced IPI cost
    // (microbench: MADV_DONTNEED added ~45K of 102K TLB IPIs over 100K ops).
    // MADV_FREE is lazy: it marks the page reclaimable but doesn't drop it or
    // shoot down TLBs until the kernel hits memory pressure, cutting both the
    // IPI count (~11%) and wall time (~17%) in the microbench. The tradeoff is
    // that MADV_FREE pages remain in RSS until reclaimed, so on a
    // low-memory-pressure host the RSS win shrinks. Correctness is unaffected:
    // smash only decommits pages that are already PROT_NONE, so any access
    // faults into the handler and is overwritten with decompressed data before
    // the (possibly stale, possibly zero) underlying page is observed.
    // SMASH_MADV_FREE=1 selects the lazy path; default DONTNEED preserves the
    // current eager-reclaim RSS behavior.
    static const int madv = madvMode();
    madvise(addr, size, madv);
#elif defined(__APPLE__)
    // MADV_FREE_REUSABLE tells the kernel to reclaim physical backing immediately.
    // Must be called while pages are still accessible (PROT_READ or PROT_RW);
    // fails with EPERM on PROT_NONE pages.
    //
    // GOTCHA (macOS RSS reporting): MADV_FREE_REUSABLE is the ONLY way to
    // actually drop physical pages here. mprotect(PROT_NONE) does NOT reclaim —
    // but task_info's resident_size (what `ps`/Activity Monitor/getCurrentRSSBytes
    // report) DROPS anyway when a page goes PROT_NONE, then JUMPS back when it
    // returns to PROT_RW. That phantom drop-and-rebound is pure reporting noise,
    // not real memory movement. Do not read a PROT_NONE-induced RSS dip as a
    // reclaim win, and do not "fix" RSS by mprotecting instead of madvising.
    // This artifact caused a visible RSS "bounce" in bench_rss (a PROT_NONE
    // deep-monitoring arm one tick before compression); see the __APPLE__ block
    // in CompressorThread::escalateToDeepMonitoring. On Linux MADV_DONTNEED both
    // reclaims and updates VmRSS truthfully, so none of this applies there.
    madvise(addr, size, MADV_FREE_REUSABLE);
#endif
}

// Unmap pages entirely (release virtual + physical).
inline void unmapPages(void* addr, size_t size) {
#if defined(_WIN32)
    VirtualFree(addr, 0, MEM_RELEASE);
#else
    // Note: small unmaps outside VmRegion are legitimate — LargeAlloc
    // falls back to direct mmap for oversized allocations, and deallocate
    // unmaps them here. Do NOT trap on these.
    // rawMunmap: never re-enter libsmash's own munmap interposer (its
    // deregistration loop takes per-page locks — unsafe from fault paths,
    // and pointless for smash-internal pages).
    rawMunmap(addr, size);
#endif
}

// Protect pages. Returns true on success, false on failure.
//
// Linux mprotect can fail with ENOMEM when the kernel's VMA split would
// take the process over `/proc/sys/vm/max_map_count` (default 65530).
// Smash creates one VMA boundary per compressed page since each page is
// PROT_NONE'd individually within a larger PROT_RW reservation. On long
// compiles with many cold pages, this hits the cap and subsequent
// mprotect() calls silently fail. Callers should check the return.
inline bool protectPages(void* addr, size_t size, bool read, bool write) {
#if defined(_WIN32)
    DWORD prot = PAGE_NOACCESS;
    if (read && write) prot = PAGE_READWRITE;
    else if (read) prot = PAGE_READONLY;
    DWORD old;
    return VirtualProtect(addr, size, prot, &old) != 0;
#else
    int prot = PROT_NONE;
    if (read) prot |= PROT_READ;
    if (write) prot |= PROT_WRITE;

    // Bounds check: if VmRegion bounds are set, verify we're not accidentally
    // mprotecting a page outside our arena (which would corrupt libc/ld.so).
    if (g_vm_bounds_base && size <= 4096) {
        auto a = reinterpret_cast<uintptr_t>(addr);
        if (!vmAddrAllowed(a, size)) {
            // Out of bounds! This is the bug. Trap with diagnostic.
            char buf[160];
            int n = smash::safe_snprintf(buf, sizeof(buf),
                "[smash FATAL] protectPages OOB: addr=%p size=%zu prot=%d "
                "vm=[%p, +%zu]\n",
                addr, size, prot, g_vm_bounds_base, g_vm_bounds_size);
            if (n > 0) (void)!::write(2, buf, (size_t)n);
            __builtin_trap();
        }
    }

#if defined(__linux__)
    // Direct syscall: the libc mprotect wrapper may access TLS (for errno)
    // which can live on a smash-compressed page — causing infinite SIGSEGV
    // recursion when called from the fault handler.
    if (syscall(SYS_mprotect, addr, size, prot) == 0) return true;
#else
    if (mprotect(addr, size, prot) == 0) return true;
#endif
    int err = errno;
    static const bool trace = []{
        const char* v = getenv("SMASH_TRACE_MPROTECT_FAIL");
        return v && v[0] == '1';
    }();
    if (trace) {
        char buf[160];
        int n = smash::safe_snprintf(buf, sizeof(buf),
            "[smash mprotect-fail] addr=%p size=%zu prot=%d errno=%d\n",
            addr, size, prot, err);
        if (n > 0) (void)!::write(2, buf, (size_t)n);
    }
    errno = err;
    return false;
#endif
}

// Called by remapPages() after it replaces a range. A mempolicy lives on the
// VMA, and MAP_FIXED installs a fresh default-policy VMA, so any NUMA binding
// on the range is gone. VmRegion sets this when its NUMA node arenas are
// created, to bind the node pieces again. Must be async-signal-safe, because
// remapPages runs in fault-handler paths. Set once, before any node page
// exists, like g_external_page_check.
inline void (*g_after_remap_hook)(void* addr, size_t size) = nullptr;

// Replace pages with a fresh anonymous mapping at the same address.
// Returns true on success. Used to collapse the per-page VMA fragmentation
// that mprotect-PROT_NONE creates so we don't hit vm.max_map_count.
//
// MAP_FIXED replaces the existing mapping atomically; the kernel will
// re-merge with adjacent identically-protected VMAs after the call.
inline bool remapPages(void* addr, size_t size, bool read, bool write) {
#if defined(_WIN32)
    // No analogous primitive on Windows that's strictly safer than
    // VirtualProtect — fall back to a plain protect call.
    return protectPages(addr, size, read, write);
#else
    // Bounds check: MAP_FIXED destroys any existing mapping at the target.
    // If addr is outside VmRegion, this would corrupt libc/ld.so mappings.
    if (g_vm_bounds_base) {
        auto a = reinterpret_cast<uintptr_t>(addr);
        if (!vmAddrAllowed(a, size)) {
            char buf[160];
            int n = smash::safe_snprintf(buf, sizeof(buf),
                "[smash FATAL] remapPages OOB: addr=%p size=%zu "
                "vm=[%p, +%zu]\n",
                addr, size, g_vm_bounds_base, g_vm_bounds_size);
            if (n > 0) (void)!::write(2, buf, (size_t)n);
            __builtin_trap();
        }
    }

    int prot = PROT_NONE;
    if (read) prot |= PROT_READ;
    if (write) prot |= PROT_WRITE;
    // rawMmap: remapPages runs in fault-handler paths; the PLT-bound mmap
    // would both risk lazy-resolution and re-enter smash's own interposer.
    void* p = rawMmap(addr, size, prot,
                      MAP_PRIVATE | MAP_ANON | MAP_FIXED);
    if (p != addr) return false;
    if (g_after_remap_hook) g_after_remap_hook(addr, size);
    return true;
#endif
}

// Pin pages in physical memory (prevent swap/reclaim under cgroup pressure).
// Returns true on success.  mlock counts against RLIMIT_MEMLOCK; most Linux
// distros default to 64 KiB per-user which is too low — CAP_IPC_LOCK or
// raising the limit is required for large regions.  Failures are silent.
inline bool lockPages(void* addr, size_t size) {
#if defined(_WIN32)
    return VirtualLock(addr, size) != 0;
#else
    return mlock(addr, size) == 0;
#endif
}

inline void unlockPages(void* addr, size_t size) {
#if defined(_WIN32)
    VirtualUnlock(addr, size);
#else
    munlock(addr, size);
#endif
}

// Hint to the kernel that pages will be needed soon (raises LRU priority,
// faults in pages that were swapped out).  Lighter than mlock — no hard pin,
// just a strong keep-in-RAM signal.
inline void willNeedPages(void* addr, size_t size) {
#if !defined(_WIN32)
    madvise(addr, size, MADV_WILLNEED);
#endif
}

#if defined(__linux__)
// Largest node id + 1 that bindPagesToNode accepts: the kernel's MAX_NUMNODES
// ceiling (CONFIG_NODES_SHIFT tops out at 10 on x86-64 and arm64).
inline constexpr uint32_t kMaxNumaNodes = 1024;

// Bind [addr, addr+size) to NUMA node `node` (backs malloc_numa).
//
// MPOL_BIND, not MPOL_PREFERRED: the caller named a node, and PREFERRED
// silently spills onto other nodes under memory pressure, so the locality the
// caller asked for disappears with no error. MPOL_F_STATIC_NODES keeps `node`
// a physical id; without it a later cpuset change rebinds the policy
// *relatively* and places new pages on a different node even while `node` is
// still allowed. `flags` is 0 for a range with no pages yet, or MPOL_MF_MOVE
// to also migrate pages that were already placed elsewhere.
//
// Raw syscall: glibc has no mbind wrapper (libnuma does), and smash does not
// link libnuma. Fails with EINVAL if the node is out of range, offline,
// memoryless, or outside the process's cpuset.
inline bool bindPagesToNode(void* addr, size_t size, uint32_t node,
                            unsigned flags = 0) {
    if (node >= kMaxNumaNodes) { errno = EINVAL; return false; }
    constexpr size_t kBitsPerWord = 8 * sizeof(unsigned long);
    unsigned long mask[kMaxNumaNodes / kBitsPerWord] = {};
    mask[node / kBitsPerWord] = 1UL << (node % kBitsPerWord);
    // maxnode is one past the mask width: the kernel's get_nodes() decrements
    // it before reading (the same +1 libnuma passes).
    return syscall(SYS_mbind, addr, size, MPOL_BIND | MPOL_F_STATIC_NODES,
                   mask, kMaxNumaNodes + 1, flags) == 0;
}

// Reset the range to MPOL_DEFAULT. Best effort.
inline void unbindPages(void* addr, size_t size) {
    (void)syscall(SYS_mbind, addr, size, MPOL_DEFAULT, nullptr, 0, 0);
}
#endif

} // namespace smash::vm
