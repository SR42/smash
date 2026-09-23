# Design: a per-CPU cache of freed large runs

Status: **implemented** (2026-09-23), off by default (`SMASH_RUN_CACHE_MIB`). Measurements at the end.

## Problem

A 128-thread application doing `malloc_numa` of multi-MB blocks on a 128-core,
4-node box spends 26.6% of all CPU in `Spinlock::lock()`, almost all under
`LargeAlloc::allocateOnNode`. Two global locks on the same path were already
sharded (`491c6fe` entry pool, `5da814b` span descriptors: 36.6% → 30% →
26.6%). What remains is structural:

- every node-arena allocation takes that node's `na.lock` and searches the free
  map under it (`vm_region.h` fast path and post-drain retry: 800 + 589 of the
  lock samples in the repro) — 32 threads per lock on that box;
- every free runs the compressor's per-page release (`releaseHook`, 30–40% of
  CPU on an 8-CPU repro), an `mprotect` (process-wide mmap write lock, TLB
  shootdown) and an `madvise`, and the reused run then takes a zero-fill fault
  per page.

Sharding cannot fix the first: `na.lock` guards the node's free memory itself.
What fixes both is not reaching the arena at all on the common
allocate → free → allocate cycle.

## Design

**Per-CPU, keyed by node.** 64 shards (as `util/cpu_shard.h`), each one cache
line of header plus a small array of cached runs `{addr, pages, node, freed_at}`.
Per-CPU rather than per-thread: no TLS, nothing to flush at thread exit, and
bounded by CPUs rather than threads. A run is reused only for the same node
(node arena pages are `mbind`-bound; the shared arena is node -1), so NUMA
placement is unchanged.

**Free** (`LargeAlloc::deallocate`, arena blocks only):
1. Clear the page map entry, as today.
2. Release compressed state through the existing hook, which is extended to
   report whether any page was `COMPRESSED`/`COMPRESSING` (i.e. may be
   `PROT_NONE` or have no backing).
3. If none was, and the run fits the shard's caps, push it to the current
   CPU's shard and return — no queue entry, no `mprotect`, no `madvise`, pages
   stay resident and `EMPTY` (the compressor skips `EMPTY` pages).
4. Otherwise (a cold block being freed, or the shard is full), today's path.
   A full shard evicts its oldest run to today's path instead.

**Allocate** (`allocateOnNode` / `allocate`, before touching the arena): pop
an exact page-count match for this node from the current CPU's shard (and,
for an aligned request, only a run that already satisfies the alignment); mark
its pages `ACTIVE` as today. On a miss, today's path.

**Trim.** The decommit thread, which already wakes every 100 µs, sweeps the
shards every ~10 ms with `tryLock` and sends runs older than a threshold
(default ~50 ms) to today's path. So a burst leaves RSS behind for at most
that long, and a workload that stops churning returns everything.

**Caps and defaults.** `SMASH_RUN_CACHE_MIB` = bytes per shard (0 disables);
at most 8 runs per shard; runs above a size limit (e.g. 64 MiB) never cached.
Worst-case extra RSS is `shards × cap` (64 × 32 MiB = 2 GiB) — bounded by
shards, not by threads. **Off by default** until measured on the 128-core box;
then decide.

**Bypassed** under `SMASH_LANDMINES`, `SMASH_ZERO_FREE`, tracking mode and
compress-only mode, which all depend on today's free path.

## What it changes, and what it does not

- Removes from the cached cycle: `na.lock`, the free-map lock, the decommit
  queue, `mprotect`, `madvise`, zero-fill faults, entry and descriptor churn.
- Does not remove the per-page `releaseHook` walk on free. That is a separate,
  smaller change (a per-span "ever compressed" flag set by the compressor, so a
  never-compressed block skips the walk) — measure after this one.
- Reuse returns old contents, not zeros, exactly as glibc/jemalloc do;
  `calloc` already zeroes explicitly.

## Risks to test for

- A run handed to two owners (double pop, or pop racing trim).
- A reused run with stale protection: `PROT_READ` from Phase 3 monitoring under
  `SMASH_SOFTDIRTY=0`, or `PROT_NONE` if the "was compressed" report is wrong.
- Node binding: a cached node run must only ever serve that node.
- RSS: cached bytes must stay under the cap and drain to 0 after the trim age.
- `fork()` with a shard locked (reset shard locks in the child, as the reclaim
  token is).

## Tests (written first, failing against today's code)

1. **Reuse stays off the arena path**: a single thread doing `malloc_numa` /
   free / `malloc_numa` of one size must get the same address back with an
   exported counter showing a cache hit and no `na.lock` acquisition (new
   diagnostic export). Fails today: the second allocation goes through the
   free map (after the queue drains) or fresh region.
2. **Correctness under churn**: 8–64 threads, several sizes and nodes, each
   block tagged and verified before free; every block's pages on its node
   (`move_pages`); no two live blocks overlap.
3. **Cold blocks are not cached**: a block left to compress, then freed and
   reallocated at the same size, must be readable/writable without a fault
   loop, with or without `SMASH_SOFTDIRTY=0`.
4. **RSS bound and drain**: after a burst, cached bytes ≤ cap; after the trim
   age, 0.
5. Existing suite, UBSan, and root on claudepc.

## Measurement

- 8-CPU repro (`numachurn`, `malloc_numa` 1–8 MiB, node = thread % 4):
  ops/s at 8 / 16 / 64 threads and the `Spinlock::lock` share, cache off vs on.
- 128-core box: the same `perf report --sort sym` as before (26.6%
  `Spinlock::lock` under `allocateOnNode`), plus RSS against the current 86 GB.

## Scope

Roughly 300 lines in `large_alloc.h` / `vm_region.h` / the compressor's
release hook, plus one test file and a diagnostic export.

## Measured (implementation, 8-CPU 4-node host)

`numachurn` (`malloc_numa` 1-8 MiB, node = thread % 4), interleaved, 3 rounds:

| threads | cache off | `SMASH_RUN_CACHE_MIB=32` |
|---|---|---|
| 8 (1:1) | 73-75k ops/s | 150-161k (~2.1x) |
| 16 | 70-72k | 131-135k (~1.9x) |
| 64 (8x oversubscribed) | 31-36k | 29-31k; 40.6k at 128 MiB |

At 64 threads eight threads share each CPU's shard, and with random sizes an
exact match in 8 slots is less likely (122k hits at 32 MiB, 175k at 128 MiB):
the cap, not the design, limits it there. One thread per core — the 128-core
box's shape — is the 8- and 16-thread case.

As implemented, one deviation from the plan above: the export is
`smash_run_cache_info(hits, cached_bytes)`, and the reuse test discriminates
on minor faults rather than on a lock-acquisition counter.
