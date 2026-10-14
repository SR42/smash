# Design: per-CPU sub-arenas with immediate dirty reuse

Status: **phases 1 and 3a implemented** (2026-09-24) behind
`SMASH_SUBARENAS=1` — node arenas, then the shared arena; phase 2 (measuring
on the 128-core box) pending, and with it making them the default and
retiring the run cache (3b). Open questions resolved at the
end; what the implementation changed is under "Phase 1 as built".

## Problem

After sharding the global pools (`491c6fe`, `5da814b`, `4c28970`) and splitting
the free map and decommit queue per node (`fcfaca0`), `smash_sim` on the
128-core, 4-node aarch64 box puts one site on top at 128 threads:
`allocatePagesOnNodeImpl`'s `na.lock`, **47% of all thread time waiting pinned,
79% unpinned**, everything else under 3%. Moving the free-map search out of
`na.lock` (option A) only moved the wait to the node's `PageRunMap` lock
(measured, reverted): the node's free map itself is the meeting point of every
cache miss and every free of 32 threads.

Two more costs sit on the same path:

- **Every free is a syscall pipeline before reuse.** A freed run is decommitted
  (`mprotect` + `madvise`) before it reaches the free map, and its next owner
  takes a zero-fill fault per page.
- **The run cache only reuses exact sizes** (8 slots per CPU): 62% hits on 8
  sizes, 32% on 16 (`smash_sim`, one thread per CPU).

## What jemalloc does (verified in /home/claude/jemalloc, 5.3.1-dev)

- **Many small page allocators:** 4 arenas per CPU by default
  (`jemalloc.c:187`, `jemalloc_init.c:377-390`) — 512 on a 128-core box — each
  with its own dirty / muzzy / retained extent caches and locks. Threads stick to
  the least-loaded arena. No NUMA awareness anywhere.
- **Free makes pages reusable immediately:** `pac_dalloc` records the extent in
  the dirty cache, no syscall (`pac.c:476-480`); a later allocation of any size
  can be carved from it (coalescing free map with per-size-class heaps and a
  non-empty bitmap, `eset.c`).
- **Purging is time-based and off the hot path:** `dirty_decay_ms` = 10 s,
  `muzzy_decay_ms` = 0 (`arena.h:31-32`), 200 smoothstep epochs, triggered by a
  1/1000 ticker with trylock-and-skip or by background threads; `madvise` runs
  with the extent-cache lock dropped (`pac.c:635-674`, `790-808`). No `mprotect`.
- **No per-thread cache for multi-MiB blocks** (tcache stops at 32 KiB,
  `tcache.h:39-40`) and **no large size classes** by default
  (`opt_disable_large_size_classes = true`, `jemalloc.c:135`); instead growth
  over-reserves to the class size (capped at 2 MiB) and banks the tail in the
  dirty cache (`pac.c:139-152, 200-240`).
- **Caveat relevant to us:** allocations >= 8 MiB go to one shared "huge" arena
  (`arena_inlines.h:154-173`) and, without background threads, are purged on
  free (`extent.c:1090-1099`). jemalloc treats that size as rare.
- Smaller ideas: adaptive mutexes (spin ~600, then block, `mutex.c`); a growth
  lock separate from the free-list lock; `lg_max_fit` (never split a free run
  more than 64x the request); byte-capped caches with cold-tail eviction
  (`sec.c:251-273`).

## Design

### Structure

Each node arena (and, in phase 3, the shared arena) is split into **S
sub-arenas**: `SMASH_SUBARENAS_PER_NODE=N` sets S directly (capped at 64);
unset or 0 means one per CPU of the node (read from
`/sys/devices/system/node/nodeN/cpulist`; 32 on the target box).
A thread picks the sub-arena for `(node, current CPU)`: a CPU of the node maps
to its own sub-arena; a CPU of another node (an unpinned thread) maps by
`cpu % S`.

A sub-arena owns:

- **whole 64 MiB node chunks**, carved from the node as today (so `mbind`
  binding and `node_of_chunk_` are unchanged); a per-chunk record of which
  sub-arena owns it;
- **two sets of free-run bins — dirty and clean** — in the node's `PageRunMap`;
- a **chunk tail** (`cur`, `end`) like today's `NodeArena`;
- **one lock**, covering its bins, its tail and its dirty accounting;
- dirty-byte counters and an **LRU of dirty runs** (for decay).

The node keeps: the chunk grow lock (separate from every sub-arena lock), the
pool of wholly free chunks, and its decommit thread (now the purger).

### The free map: one tag array per node, bins and lock per sub-arena

A `PageRunMap` per sub-arena would reserve 20 bytes x every arena page x 128
maps — ~320 GB of address space on a 512 GiB arena. Instead the node keeps
its one tag/end array (each page is only ever described by the sub-arena that
owns its chunk) and the map gains **per-owner bins and a per-owner lock**, where
an owner is `(sub-arena, dirty|clean)`. What makes this safe is a rule the
current map does not need: **runs never coalesce across a chunk boundary.**
Then no operation reads or writes a tag outside chunks its sub-arena owns,
and the sub-arena lock is the only lock it needs. (Requests larger than a chunk
keep today's node-level path.)

### Free

1. The compressor's release hook runs as today and reports whether any page
   was compressed.
2. **Not compressed** (the churn case): insert the run into the owning
   sub-arena's **dirty** bins, coalescing with dirty neighbours in the chunk;
   append to its dirty LRU; no syscall. Pages stay resident and `EMPTY` (the
   compressor ignores `EMPTY` pages).
3. **Compressed**: today's decommit path (`mprotect` RW + `madvise`), then
   insert into the **clean** bins.
4. The owning sub-arena is the chunk's, not the freeing CPU's: a block freed
   on another CPU (`FreeDynMem`) still goes home — one lock, no cross-sub-arena
   state.

### Allocate

In the thread's sub-arena, under its lock: dirty bins (best fit, bounded by an
`lg_max_fit`-style cap), then clean bins, then the chunk tail. On a miss:
**trylock-scan sibling sub-arenas** of the node for a dirty or clean run (as
jemalloc's SEC scans shards, `sec.c:164-203`), then take a wholly free chunk
from the node pool, then carve a new chunk under the grow lock (re-checking
the pool after taking it). A dirty run needs no work before reuse; a clean
run is already committed.

### Purge (decay)

The node's decommit thread becomes its purger. Every epoch (default 50 ms) it
visits each sub-arena with **trylock** (skip if busy), takes dirty runs older
than `SMASH_DIRTY_DECAY_MS` (environment variable, default **1000**, not
jemalloc's 10 s — RSS is the point of smash; 0 purges on the next epoch) or
beyond `SMASH_DIRTY_MAX_MIB` per sub-arena (oldest first), marks them in-flight, **drops the lock**, `madvise`s them (batched), and
re-inserts them as clean. Whole-chunk clean runs go back to the node pool, so
memory freed by one sub-arena is not stranded there.

### What it replaces

- The **run cache**: dirty bins are a per-CPU cache for any size, with
  coalescing. Disabled when sub-arenas are on; removed once they are default.
- The **decommit queue on the free path** for uncompressed runs: only
  compressed runs and purges still pay syscalls, and never under a free-map
  lock.

## Costs and risks

- **RSS**: freed memory stays resident up to the decay time — bounded by
  `SMASH_DIRTY_DECAY_MS` and `SMASH_DIRTY_MAX_MIB` per sub-arena, reported on
  the stats line. Decay 0 means today's behaviour (purge at once, but still
  off the free path).
- **Fragmentation / address space**: 32 sub-arenas per node each hold partly
  used chunks. Mitigated by sibling scans, the free-chunk pool and chunk-bounded
  runs; measured with `smash_sim` (arena walked, per-node carved vs free).
- **PageRunMap refactor** (per-owner locks, chunk-bounded coalescing, LRU of
  dirty runs) is the core engineering risk; it is also what the simulator's
  ownership shadow and drain accounting check most directly.
- **8 MiB blocks** fit 8 per chunk; blocks over a chunk keep the node path.
- **Locks**: sub-arena locks should be nearly uncontended; if not, an
  adaptive spin-then-futex lock (jemalloc's choice) is the follow-up, not a
  global `Spinlock` change.

## Phases

1. Sub-arenas + dirty/clean bins + purge for **node arenas** (`malloc_numa`,
   the target workload), behind `SMASH_SUBARENAS=1`; run cache off when on.
2. Measure: `smash_sim` invariants and 1..4-thread comparison here; 8..128
   threads pinned/unpinned on the 128-core box against `fcfaca0`/`4c28970`;
   then the application (lock profile, RSS vs 86 GB).
3. (a) Shared arena, behind the same flag — done. (b) Make default; retire the
   run cache and the free-path decommit queue — after phase 2.

## Tests (written first, failing against today's code)

- Reuse of a freed warm block of a **different** size from the same chunk
  takes no page faults (the run cache cannot do this; faults are the
  discriminator, as in `test_run_cache`).
- A block freed on another CPU returns to its chunk's sub-arena.
- Compressed blocks go to clean bins and are fully usable after reuse (also
  under `SMASH_SOFTDIRTY=0`).
- Dirty bytes drain to 0 within decay + one epoch; stay under the cap.
- Sibling scan: a sub-arena with nothing free reuses a sibling's dirty run
  before carving a chunk.
- `smash_sim`: ownership, page map and full-drain accounting (now including
  dirty and clean bins and the chunk pool), across seeds; planted-bug checks
  for a missing purge and a cross-sub-arena double hand-out.
- Suite, UBSan, root on claudepc.

## Configuration

| Variable | Default | Meaning |
|---|---|---|
| `SMASH_SUBARENAS` | 0 (off) | 1 enables sub-arenas (phase 1 opt-in) |
| `SMASH_SUBARENAS_PER_NODE` | one per CPU of the node | sub-arenas per node, 1..64 |
| `SMASH_DIRTY_DECAY_MS` | 1000 | age at which the purger returns a dirty run to the OS; 0 = next epoch |
| `SMASH_DIRTY_MAX_MIB` | 256 | dirty bytes per sub-arena before the oldest are purged early |

## Resolved (2026-09-24)

1. Sub-arenas per node: configurable (`SMASH_SUBARENAS_PER_NODE`), default one
   per CPU; `smash_sim` sweeps it to choose the default for real.
2. Decay: configurable (`SMASH_DIRTY_DECAY_MS`), default 1 s.
3. Sibling scans never cross nodes: a node's pages stay bound to it, as today.

## Phase 1 as built (2026-09-24)

- **Where**: `VmRegion` (`src/vm/vm_region.h`, section "Sub-arenas"); the free
  map gained caller-locked operations with a coalescing `boundary` (the chunk
  size) and a per-run `stamp` (`insertUnlocked`, `takeAlignedUnlocked`,
  `forEachRunUnlocked`, `removeRunUnlocked` in `page_run_map.h`). Sub-arena
  owners are numbered after every node's (`PageRunMap::kFirstSubOwner`), so
  they never alias a node's own owner in its map.
- **Dirty LRU**: no list; the purger collects a sub-arena's dirty runs (up to
  256) with their stamps and sorts them oldest first. A merge keeps the newest
  stamp. Beyond the cap it purges the oldest runs and **splits** the run that
  crosses it (freed blocks coalesce into chunk-sized runs, so whole-run purging
  overshot to 0 dirty).
- **Blocks larger than a chunk** get dedicated fresh chunks marked `kChunkBig`;
  their free (always through the decommit queue) returns the chunks to the
  node's pool. Chunks an alignment gap leaves untouched go to the pool at once.
- **`free()`**: `LargeAlloc::deallocate` offers a reusable run to
  `freeRunDirty` first; it declines outside sub-arena chunks and whenever
  tracking mode, landmines or zero-on-free is on, and the run then takes the
  ordinary path.
- **Purger**: the node's decommit thread, every 50 ms, trylock per sub-arena;
  it `madvise`s with the lock dropped and counts itself in `purging` so
  `purgeAllDirtyNow()` can wait for it.
- **Exposure**: `smash_subarena_info(&dirty_bytes, &purged_runs)`;
  `smash_numa_arena_info` counts dirty, clean, tails and pooled chunks as free.
- **Tests**: `test_subarena_*` (7 phases) and `test_smash_sim_smoke_subarenas`;
  see [tests.md](tests.md) and [simulator.md](simulator.md) for measurements.

## Phase 3a as built: the shared arena (2026-09-24)

- **One more set**: `shared_set_`, a `NodeArena` created on first use with its
  own free map, whose chunks are carved off the same bump pointer but never
  bound (`carveBoundChunks(-1, …)` skips `mbind` and `node_of_chunk_`). The
  sub-arena functions take `node` -1 for it; its compressed frees and its
  purger use the shared decommit queue (`subQueue`).
- **Which sub-arena**: a slice per node with CPUs (`sharedSubLayout`),
  `SMASH_SUBARENAS_PER_NODE` each (default the node's CPUs), scaled to 64 in
  total — so on the 128-CPU box, 16 per node, two CPUs of the same node each.
  Dirty memory is first-touched on the freeing CPU's node, so keeping reuse
  within a node keeps it local.
- **Only `LargeAlloc`** goes through it (`allocateLargePages`); slab spans and
  mid bins keep `allocatePages`. When the set cannot serve (region
  exhausted), the block falls back to the shared free map as before.
- **Accounting**: `arenaFreePages()` counts the set; `smash_shared_arena_info`
  reports pages in use by the shared arena vs free.
- **Region exhaustion**: when the shared free map runs the region out
  (`allocatePages`), the shared set purges its dirty runs, retires every
  sub-arena's unused tail into its clean runs, and donates every wholly free
  chunk to the shared free map (`donateSharedPool`). Without it 8 threads'
  sub-arenas stranded up to half of a 1 GiB region (448-704 MiB of small
  objects fit instead of 960; 978 with sub-arenas off). For any set, the
  last-resort retry after the region is out also takes siblings' tails, not
  only the caller's own. Both are `test_subarena_exhaust_*`.
- **A miss folds the tail into the clean runs** before looking at siblings
  (the tail and its free neighbour can serve what neither can alone), and
  both that fold and a tail change pool the chunk if it is now wholly free.
  Without the second, a chunk stranded: a miss drops the lock, another
  thread frees into the sub-arena's dirty runs, the tail change leaves the
  old chunk wholly free but only in this sub-arena's clean runs, and the
  request is served from the fresh dirty run instead (`smash_sim` now checks
  that no wholly free chunk is left unpooled after a full purge).
- **Blocks larger than a chunk wait for freed chunks in flight** (2026-09-28)
  before carving new ones: a lagging decommit queue otherwise made every
  free-then-allocate carve fresh chunks until a node's pool held the whole
  region (`test_subarena_exhaust_big`, now 300 rounds on one CPU).
- **Blocks larger than a chunk reuse pooled chunks** (`takePooledRunLocked`:
  consecutive indices in the sorted pool) before carving fresh ones. Phase 1
  always carved, so freed big blocks only fed the pool and 70 MiB churn walked
  a 64 GiB arena in 2 s (`test_subarena_exhaust_big`).
- **The cap is enforced at once**: a free that takes a sub-arena over
  `SMASH_DIRTY_MAX_MIB` sets its queue's `purge_now`, and the purger (parked; woken by
  the free once it drops the sub-arena's lock, and retried every 1 ms while a
  sub-arena stays over its cap) purges at once instead of at the 50 ms sweep.
  Before, a fast churn held 4-6x the cap dirty between sweeps (`smash_sim`,
  8 sub-arenas: 7-13 GiB against 2), and because wholly free chunks only
  reach the pool when purged, 63 MiB blocks found it empty and carved fresh
  chunks: 40 of 64 GiB walked for a 0.8 GiB live set, now 7.6 (3.9 with
  sub-arenas off).
- **Address space**: sub-arenas walk more of the arena than the single free
  map (`smash_sim`, 6 threads, 1-70 MiB blocks: 16.6 / 18.1 GiB vs 11.3 / 4.2
  GiB, shared / node): each keeps partly used chunks and up to its dirty cap.
  It is address space, not RSS; size `SMASH_VM_GIB` with that margin.
