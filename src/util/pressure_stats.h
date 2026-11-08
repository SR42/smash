// smash/src/util/pressure_stats.h - time/space accounting for the compressor
//
// Measurement only (no policy). Enabled by SMASH_PRESSURE_LOG=<path> (or
// "stderr"); when unset, the only cost is one predicted-not-taken branch on
// a null pointer in PageStateTable::set and the fault path.
//
// What it answers: is compression paying for itself, and how scarce is CPU?
//
//   * Residence time W of each compression: a per-page stamp is written when
//     the page enters COMPRESSED / COMPRESSED_SHADOW and read when it leaves.
//     Each exit is classified:
//       refault-read / refault-write  app touched it (decompress on the app's
//                                     critical path)
//       shadow-discard                compressed copy invalidated before its
//                                     RSS was ever reclaimed (pure waste)
//       prefetch                      decompressed as a neighbor of a fault
//                                     (prefetchAdjacent)
//       restored                      moved back to ACTIVE by some other
//                                     compressor path, without a fault
//       freed                         page released (EMPTY) while still
//                                     compressed (the best outcome)
//     Ages go into log2(ms) histograms. Pages still compressed at exit are
//     reported as censored ages.
//   * Little's Law cross-check: per tick, the compressed population L and the
//     entry rate lambda; mean(L)/lambda should match the measured mean W.
//   * Cost: compressor CPU (coordinator thread + helper phases), fault-path
//     time and decompress time (charged to app threads).
//   * Scarcity: wall-vs-CPU stretch of compressor phases (portable proxy for
//     run-queue delay), Linux PSI cpu some/full, loadavg, process CPU.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <ctime>

namespace smash::pstats {

// Per-page entry stamp: (ms since g_base_ns) + 1; 0 = not compressed. Null
// unless the log is enabled. Written and read under the per-page lock on the
// compress and fault paths; other readers are statistics-only.
inline uint32_t* g_stamp = nullptr;
inline uint64_t g_base_ns = 0;

inline uint64_t monoNs() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull +
           static_cast<uint64_t>(ts.tv_nsec);
}

inline uint64_t threadCpuNs() {
    timespec ts;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull +
           static_cast<uint64_t>(ts.tv_nsec);
}

inline uint32_t stampNow() {
    return static_cast<uint32_t>((monoNs() - g_base_ns) / 1000000ull) + 1;
}

// Bucket b >= 1 holds ages in [2^(b-1), 2^b) ms; bucket 0 holds < 1 ms.
// 24 buckets reach 2^23 ms (~2.3 h); older ages saturate in the last one.
inline constexpr int kHistBuckets = 24;
enum Exit : int { kRefaultRead = 0, kRefaultWrite, kShadowDiscard, kPrefetch,
                  kRestored, kFreed, kNumExits };

inline int ageBucket(uint32_t ms) {
    if (ms == 0) return 0;
    int b = 32 - __builtin_clz(ms);
    return b < kHistBuckets ? b : kHistBuckets - 1;
}

struct Counters {
    std::atomic<uint64_t> entries{0};          // pages entering compressed state
    std::atomic<uint64_t> exits[kNumExits]{};  // by exit kind
    std::atomic<uint64_t> exit_age_ms[kNumExits]{};
    std::atomic<uint64_t> hist[kNumExits][kHistBuckets]{};
    std::atomic<uint64_t> fault_ns{0};         // whole COMPRESSED fault handling
    std::atomic<uint64_t> decomp_ns{0};        // decompress part of it
    std::atomic<uint64_t> phase_cpu_ns{0};     // executePhase CPU, all workers
    std::atomic<uint64_t> phase_wall_ns{0};    // executePhase wall, all workers
    std::atomic<uint64_t> helper_cpu_ns{0};    // executePhase CPU, helpers only
};
inline Counters g;

inline bool enabled() { return g_stamp != nullptr; }

inline void recordExit(Exit kind, uint32_t stamp) {
    uint32_t now = stampNow();
    uint32_t age = now > stamp ? now - stamp : 0;
    g.exits[kind].fetch_add(1, std::memory_order_relaxed);
    g.exit_age_ms[kind].fetch_add(age, std::memory_order_relaxed);
    g.hist[kind][ageBucket(age)].fetch_add(1, std::memory_order_relaxed);
}

// Called by the fault / discard paths BEFORE they move the page out of the
// compressed state, so the generic transition hook below sees stamp 0.
inline void onExit(size_t page_idx, Exit kind) {
    if (!g_stamp) return;
    uint32_t s = g_stamp[page_idx];
    if (!s) return;
    g_stamp[page_idx] = 0;
    recordExit(kind, s);
}

// Generic transition hook (PageStateTable). `compressed_family` covers
// COMPRESSED and COMPRESSED_SHADOW; `compressing` is the transient state a
// recompression passes through, which keeps the original stamp. Any other
// destination with a live stamp means the page left through an untagged
// path: EMPTY = freed while compressed, otherwise restored without a fault.
inline void onTransition(size_t page_idx, bool compressed_family, bool compressing,
                         bool to_empty) {
    uint32_t s = g_stamp[page_idx];
    if (compressed_family) {
        if (!s) {
            g_stamp[page_idx] = stampNow();
            g.entries.fetch_add(1, std::memory_order_relaxed);
        }
    } else if (!compressing && s) {
        g_stamp[page_idx] = 0;
        recordExit(to_empty ? kFreed : kRestored, s);
    }
}

}  // namespace smash::pstats
