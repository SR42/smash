# smash_sim: stress the large-block allocator without memory

`tools/smash_sim.cpp`, built as `tests/smash_sim` in the build directory.

## What it is

The real large-block code — `LargeAlloc`, `VmRegion` (shared arena, per-node
arenas, the free map, the decommit queue and its thread, the per-CPU run
cache), the span-descriptor pools, the page map — driven by worker threads,
compiled with `-DSMASH_SIM`, which does two things and nothing in the shipped
library:

- **The page-level OS calls become no-ops** (`commitPages`, `decommitPages`,
  `protectPages`, `remapPages`, `bindPagesToNode`, `unbindPages` in
  `platform_mem.h`), with an optional busy-wait standing in for their cost
  (`--os-call-ns`). The arena is still a real `PROT_NONE` reservation, so
  addresses are real, but no block page is ever committed or touched: 128
  threads churning 8 MiB blocks through a 512 GiB arena run in 320 MB RSS.
- **`Spinlock::lock` records contention per call site** (return address,
  contended acquisitions, time waited), symbolized with `addr2line` at the end,
  so it answers "which lock" without perf.

The compressor is replaced by a release hook that resets page states as the
compressor would and reports a chosen fraction of frees as having compressed
pages (`--cold`), which is what decides whether a run may enter the run cache.

## What it checks

- **No page handed to two live blocks**: a shadow ownership map claims every
  page of every block with a CAS, and releases it on free.
- **The page map resolves every freed block to its own span.**
- **Nothing leaks**: after every block is freed, the run cache flushed and the
  decommit queue drained, every page a node arena carved is free again, and the
  walked arena equals shared free + node carved.
- **No hang**: no progress for 3 s, or workers not stopping within 10 s, is
  reported with the contended sites so far and exit status 3.

Verified by planting bugs: a run cache that hands a run out without removing it
(the double hand-out corrupts the free map into a cycle — caught as a hang);
a trim that drops runs instead of returning them (caught as a leak on every
node). The real code holds on every seed tried.

**First finding**: `nodeArenaInfo` (behind `smash_numa_arena_info`) did not
count the unused tail of a node's newest chunk as free, so `carved - free` was
live blocks plus that tail — the end-of-run check failed on every node with a
tail. `test_malloc_numa_aligned_*`'s footprint check used those numbers, and a
block served from the tail changed neither, so it was blind on that path. Fixed;
the footprint tests still pass on the corrected numbers.

## Usage

```
smash_sim [--threads N] [--seconds S] [--nodes K] [--arena-gib G]
          [--sizes 1M,2M:3,...] [--live K] [--numa F] [--cold F]
          [--cross-free F] [--run-cache-mib M] [--os-call-ns NS]
          [--no-check] [--seed X]
```

- `--sizes` is a list with optional weights (`1M:5,8M:1`); each thread picks
  one per step. Default: 1..8 MiB, equal weights.
- `--live K` blocks per thread; each step frees a random one and allocates.
- `--numa F` fraction through `malloc_numa`'s node path (node = thread %
  nodes); the rest take the shared arena. Shared-path sizes under
  `SMASH_LARGE_ALLOC_VM_THRESHOLD` (1 MiB) fall back to a real, untouched
  mmap and are reported as outside the arena.
- `--cross-free F` fraction of frees handed to another thread (the
  `FreeDynMem` pattern: allocated on one thread, freed on another).
- `--no-check` skips the ownership shadow for maximum rate.

Output: ops/s per second, throughput, OS calls per op, placement, how far the
arena was walked, run-cache counters, the ten most contended lock sites (share
of thread time spent waiting), and the invariants.

## What it can and cannot tell you

- **Policy, exactly**: run-cache hit rates for a size mix, arena walk,
  fragmentation, fallbacks. For per-CPU policy run one thread per CPU: that is
  what each shard sees on a box with one thread per core, whatever its size.
- **Correctness under stress**: the invariants above, at any thread count.
- **Contention, qualitatively**: which sites wait, on this machine. It cannot
  make 8 cores contend like 128 — oversubscribed threads are preempted while
  holding spinlocks, which exaggerates everything — so use it to rank sites
  and compare variants, not to predict a 128-core number.
- **Not** page faults, TLB shootdowns, the kernel's mmap lock or the
  compressor's own cost. `--os-call-ns` is a flat stand-in for the syscalls.

## Results so far (8-CPU, 4-node host)

Run cache hit rate by size mix, one thread per CPU (`--threads 8`):

| sizes per thread | 32 MiB/CPU | 128 MiB/CPU |
|---|---|---|
| 1 (4 MiB) | 100% | 100% |
| 8 (1..8 MiB) | 62% | 70% |
| 16 (1..16 MiB) | 32% | 44% |

A bigger cap barely helps: the limit is 8 exact-match slots per CPU. That
matches the 128-core box, where `SMASH_RUN_CACHE_MIB=128` still left 20% of CPU
in `releasePages`, and points at rounding cached runs to size classes.

128 threads, 512 GiB arena, `--os-call-ns 3000`, cache off: the node arena's
lock (`allocatePagesOnNodeImpl`, fast path and post-drain retry) is 71% of
thread time waiting — the same site the 128-core profile put at 26.6% of CPU.
