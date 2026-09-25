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
- **Sub-arena free lists** (with `--subarenas`), before and after a full
  purge, holding every sub-arena lock: every dirty or clean run lies in one
  chunk its sub-arena owns, no page is listed twice across dirty runs, clean
  runs, tails and the chunk pool, each owner's counter equals its runs, and
  (for the shared arena's set) no run of the shared free map lies in a chunk
  a sub-arena owns; after the purge, no clean run covers a whole chunk (it
  should be in the pool); and
  nothing is dirty after the purge (which also waits for the purger's
  in-flight runs, so the leak check cannot race it).
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
          [--subarenas S] [--decay-ms MS] [--dirty-max-mib M]
          [--no-check] [--pin] [--seed X]
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
- `--subarenas S` turns on sub-arenas for the node arenas (`SMASH_SUBARENAS=1`),
  S per node; 0 = one per CPU of the real node. `--decay-ms` and
  `--dirty-max-mib` set `SMASH_DIRTY_DECAY_MS` / `SMASH_DIRTY_MAX_MIB`.
- `--no-check` skips the ownership shadow for maximum rate.
- `--pin` runs thread t on the CPUs of node t % nodes, so every allocation is
  node-local. Without it, threads allocate on node t % nodes from wherever the
  scheduler puts them, and the locks and queue heads they touch may live on
  another socket.

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

## Scaling sweep and the per-node prototype

1..8 threads, one per core, `--os-call-ns 3000`: efficiency fell steadily
(53% at 8 threads cache off, 40% cache on) with recorded lock waits of only
~7-10% of thread time; the single decommit thread ran a full core (8 workers
plus it oversubscribe the box, hence 8 < 7). A USL fit on 1..7 gave a pure
serial fraction (9.5% / 17.5%, coherency 0). Splitting the free map and the
decommit queue per node took it to ~0 / 2.2% on 1..4 threads — see "One free
map and one decommit queue per node" in [performance.md](performance.md).
Fitting a curve past the point where background threads exceed the cores is
meaningless: the plateau is the CPU budget, and the fit's error jumps from
~0.01 to 0.24-0.49 when it is included.

## On the 128-core, 4-node aarch64 box (2026-09-24)

`--nodes 4 --arena-gib 512 --seconds 5 --no-check --os-call-ns 3000
--run-cache-mib 128`, allocate+free per second:

| threads | before (2f0398b) | per-node (fcfaca0) | ratio |
|---|---|---|---|
| 8 | 621k | 800k | 1.29x |
| 16 | 1160k | 1395k | 1.20x |
| 32 | 1198k | 2417k | 2.02x |
| 64 | 675k | 1579k | 2.34x |
| 128 | 414k | 1083k | 2.62x |

Both curves are **retrograde** past ~32 threads — the thing an 8-core box
cannot show. A USL fit with a free per-thread rate puts all of it in the
coherency term (cross-node cache-line traffic): kappa 0.0018 before, 0.00079
per-node, peak moving from ~24 to ~36 threads and doubling (1.2M -> 2.4M), fit
error 8-12% on five points. Per-node is still 55% below its own peak at 128.
Those runs were unpinned: thread t allocated on node t % 4 from any CPU, so
locks and queue heads crossed sockets; `--pin` was added to separate that
from contention within a node.

## Sub-arenas, node and plain malloc (2026-09-24, same host)

`--nodes 4 --arena-gib 64 --no-check --os-call-ns 3000 --pin`, allocate+free
per second, on the code with the shared set, the exhaustion fixes and the
immediate purge:

| threads | node: off | node: run cache 128 MiB | node: sub-arenas | malloc: off | malloc: run cache | malloc: sub-arenas |
|---|---|---|---|---|---|---|
| 1 | 198k | 616k | 1163k | 189k | 550k | 989k |
| 2 | 389k | 1180k | 2254k | 293k | 747k | 1894k |
| 4 | 769k | 2252k | 4296k | 417k | 1093k | 3761k |
| 8 | 924k | 2334k | 7245k | 407k | 911k | 6418k |

8 threads, 30% of frees on another thread: 6.09M vs 1.30M (node), 5.56M vs
0.70M (malloc); 30% cold: 2.09M vs 1.94M and 1.72M vs 0.84M — cold frees
still go through one decommit thread per queue.

With `--subarenas` the run also prints where each set's free pages sit when
the workers stop (dirty, clean, tails, pooled). That line found the next
problem: a 64K..63 MiB mix walked 40 of 64 GiB for a 0.8 GiB live set, with
37 GiB of it in the pools at the end. Traced every 100 ms, dirty memory sat at
7-13 GiB against a 2 GiB total cap: the purger enforced the cap only at its
50 ms sweep, and wholly free chunks only reach the pool when purged, so
between sweeps 63 MiB blocks found the pool empty and carved. A free over the
cap now wakes the purger at once: dirty 0.6-1.4 GiB, 7.6 GiB walked (3.9 with
sub-arenas off), same throughput.

## Sub-arenas (2026-09-24, 8-CPU 4-node host)

`--nodes 4 --arena-gib 64 --numa 1 --no-check --os-call-ns 3000 --pin`,
allocate+free per second (two runs each, within 2%):

| threads | cache off | run cache 128 MiB | sub-arenas (one per CPU) |
|---|---|---|---|
| 1 | 199k | 575k | 896k |
| 2 | 389k | 1096k | 1756k |
| 4 | 766k | 2212k | 3318k |
| 8 | 904k | 2170k | 5844k |

Sub-arenas scale at 82% efficiency to 8 threads with **0 OS calls per op**
(warm frees never leave the sub-arena, and reuse is within decay); the run
cache is flat past 4. Unpinned 8 threads: 5.77M vs 2.12M; with 30% of frees
handed to another thread: 4.97M vs 1.21M. With 30% of frees cold the gap
closes (1.99M vs 1.84M): cold frees still go through the node's one decommit
thread, which saturates at 3 µs per OS call — the next bottleneck if the
application frees many compressed blocks.

**Finding with the shared set**: blocks larger than a chunk always carved
fresh chunks and never reused the ones their frees pooled, so 70 MiB churn
walked the whole 64 GiB arena in 2 s and node blocks then fell out of it
(170717 in 2 s) — visible only in the placement line, since every page was
still accounted for. Fixed; the invariant checks stay silent on placement by
design, so read that line when a size mix includes blocks over 64 MiB.

**Findings on first run**: the sub-arenas' free-map owner ids aliased the node
arenas' own (node n uses owner n+1, which was sub-arena 0's dirty or clean
owner), so node 2's free pages were counted twice (free > carved); fixed by
numbering sub-arena owners after every node's. The pre-purge check was added
after a planted misrouted free passed the post-purge one.
