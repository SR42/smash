#pragma once
// cpu_shard.h - pick a shard of a per-CPU pool.
//
// Global free lists taken on every large allocation or free (the decommit
// entry pool, the span descriptor cache) collapse at high core counts: on a
// 128-core box one such spinlock was a third of all CPU. Sharding them by the
// current CPU keeps each shard's lock on one core's cache line most of the
// time.
#include <cstdint>
#include <pthread.h>
#if defined(__linux__)
#include <sched.h>
#endif

namespace smash {

// Shard in [0, shards) for the calling thread; `shards` is a power of two.
// Pools use 256 shards so that no two CPUs share one on hosts up to 256 CPUs:
// with 64, CPU c and c + 64 shared a shard, which on a 128-core, 4-node box
// paired node 0 with node 2 and node 1 with node 3 — every shard's lock and
// cache line crossed sockets, and pinning threads to their node made that
// systematic (32 pinned threads ran at 1.5M allocate+free/s, unpinned 2.0M).
inline unsigned currentCpuShard(unsigned shards) {
#if defined(__linux__)
    // glibc reads this from rseq where the kernel supports it: no syscall.
    const int cpu = sched_getcpu();
    if (cpu >= 0) return static_cast<unsigned>(cpu) & (shards - 1);
#endif
    // No CPU number: spread by thread instead.
    const auto t = reinterpret_cast<uintptr_t>(pthread_self());
    return static_cast<unsigned>((t >> 12) ^ (t >> 20)) & (shards - 1);
}

}  // namespace smash
