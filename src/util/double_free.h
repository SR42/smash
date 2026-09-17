#pragma once
// double_free.h - opt-in double-free detection policy, shared by the two
// places that can see a repeated free.
//
// A double free is undefined behaviour in C and C++ (`free` is specified as UB
// if the space has already been deallocated), so detecting it is a
// quality-of-implementation aid for debugging, not a conformance requirement.
// glibc chooses to abort ("free(): double free detected in tcache 2"); smash
// historically absorbed it silently. That is why this is **off by default**:
// no correct program can trigger it, so no correct program should pay for it.
//
// SMASH_DOUBLE_FREE=off|warn|abort   (default: off)
//   off    historical behaviour — the second free is applied as before
//   warn   report to stderr and DROP the second free
//   abort  report, then trap
//
// Two detection points, because a repeated free can be caught at either:
//   - ThreadCache::deallocate — scans the lane. This catches the common
//     `free(p); free(p);` case, which never reaches the slab because both
//     pointers sit in the per-thread cache. The scan is bounded by
//     kPerLaneDepth but runs on the free fast path, so it is gated on the
//     policy being enabled and costs one predictable branch when off.
//   - Span::deallocate — tests the bitmap bit (1 = free) before setting it.
//     Essentially free: the function already loads and stores that word, so
//     the check is a `test` on a value in a register. Catches a repeat free
//     that survives a lane drain, and a free of an interior/bogus pointer
//     that maps onto an already-free slot.
//
// Reporting uses safe_snprintf + write(2), never fprintf: this runs inside
// free(), where glibc's stdio can call malloc and reenter the allocator.

#include "safe_printf.h"
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <unistd.h>

namespace smash {

enum class DoubleFreePolicy : uint8_t { kOff, kWarn, kAbort };

inline DoubleFreePolicy doubleFreePolicy() {
    static const DoubleFreePolicy p = [] {
        const char* v = std::getenv("SMASH_DOUBLE_FREE");
        if (!v) return DoubleFreePolicy::kOff;
        if (v[0] == 'a') return DoubleFreePolicy::kAbort;   // abort
        if (v[0] == 'w') return DoubleFreePolicy::kWarn;    // warn
        return DoubleFreePolicy::kOff;                      // off / anything else
    }();
    return p;
}

// True when any detection is enabled. Hot callers test this BEFORE doing any
// scanning work, so the disabled path is a single cached load plus a branch.
[[gnu::always_inline]]
inline bool doubleFreeChecksEnabled() {
    return doubleFreePolicy() != DoubleFreePolicy::kOff;
}

// Rate-limited: a loop double-freeing would otherwise flood stderr.
[[gnu::cold, gnu::noinline]]
inline void reportDoubleFree(const char* where, const void* ptr, size_t detail) {
    static std::atomic<uint64_t> seen{0};
    const uint64_t n = seen.fetch_add(1, std::memory_order_relaxed) + 1;
    if (n <= 8 || (n & 0xFFF) == 0) {
        char buf[160];
        const int len = smash::safe_snprintf(buf, sizeof(buf),
            "[smash double-free] %s ptr=%p detail=%zu (occurrence %lu; "
            "second free dropped)\n",
            where, ptr, detail, static_cast<unsigned long>(n));
        if (len > 0) (void)!::write(2, buf, static_cast<size_t>(len));
    }
}

// Report and decide. Returns true if the caller should DROP the second free.
[[gnu::cold, gnu::noinline]]
inline bool handleDoubleFree(const char* where, const void* ptr, size_t detail) {
    const DoubleFreePolicy p = doubleFreePolicy();
    if (p == DoubleFreePolicy::kOff) return false;
    reportDoubleFree(where, ptr, detail);
    if (p == DoubleFreePolicy::kAbort) __builtin_trap();
    return true;
}

}  // namespace smash
