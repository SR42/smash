# Performance: measured costs and limits

Measured costs of the hot paths, compression throughput and its limits, codec and unit-size trade-offs, and the open performance defects. Moved from CLAUDE.md.

## The base small-object path: the thread cache is 4 deep, and that is the cost

An alloc+free pair of 64-byte objects costs **35 ns against glibc's 9 ns** with
no drains involved, and the reason is `kPerLaneDepth`:

```
kCacheLanes = 64, kThreadCacheMaxPerClass = 256
kPerLaneDepth = 256 / 64 = 4
```

Each (lane, size-class) cache holds **four pointers**, so roughly every fourth
allocation goes to the slab through `ThreadCache::refill` and every fourth free
drains through `drainRangeToSlabs`. A `perf` profile of a pure cache-hit
workload (batch 64, far below the nominal 256 capacity) confirms it:
`drainRangeToSlabs` 22.3%, `refill` 14.4% — a third of the time in paths that
should not run at all on a cache hit.

The lanes exist to keep arenas separate, so the depth cannot simply be bought
by reducing them — that caps segregation. Measured, all on this 8-CPU host
(16 arenas, 32 with cold-bias doubling):

| lanes | depth | memory/thread | alloc+free | distinct arenas over 32 call sites |
|---|---|---|---|---|
| **64 (stock)** | **4** | **72 KB** | **35.5–36.7 ns** | **14** |
| 32 | 8 | 72 KB | 29.7–30.2 ns | 14 |
| 8 | 32 | 72 KB | 21.0 ns | **8 — collapsed** |
| 64 | 16 | 288 KB | 23.4–23.9 ns | 14 |
| 64 | 64 | 1152 KB | 15.3–17.0 ns | 14 |
| glibc | | | 9.0 ns | |

Two things follow. **Depth is worth a lot** — 4 → 64 takes the path from 35 ns
to 15 ns, within 70% of glibc — but buying it with `kThreadCacheMaxPerClass`
costs 1.15 MB per thread, and per-thread memory is exactly what
`test_thread_cache_reclaim_*` exists to police. **Cutting lanes is not free
either**, even though it looks it: at the same 72 KB, 8 lanes give 21 ns but
collapse 32 call sites from 14 distinct arenas to 8, because the lane count
caps how many arenas a thread can hold objects for. `bench_arena_segregation`
does *not* see this (it reports ratios for homogeneous pages only, not how many
pages stay homogeneous), so the routing test is the instrument that matters.

**All of that is measured at low thread counts, and it does not extrapolate.**
The same alloc/free workload across threads (8 physical cores, so 128 threads
is oversubscribed), with one free in eight handed to a peer:

| threads | stock (depth 4) | depth 64 | glibc |
|---|---|---|---|
| 1 | 35.3 ns | **16.0** | 12.0 |
| 8 | 48.8 ns | **31.5** | 20.9 |
| 32 | 43.3 ns | **35.5** | 21.5 |
| 128 | **53.5 ns** | 75.3 | 25.6 |

A deeper cache wins by 2x at one thread, by a third at eight, barely at 32, and
**loses at 128** — 1.15 MB per thread is 147 MB of thread caches across 128
threads, and that stops fitting anywhere. Whether that crossover is the same on
a machine with 128 *real* cores is not something an 8-core host can answer;
treat the last row as a warning that depth is not free at scale, not as a
measurement of a 128-core box.

**Arena and lane counts scale with CPUs, and the interesting case is 128
cores**: `getNumArenas()` gives 64 there, so 128 arenas with cold-bias against
64 lanes — 2:1 collisions. Those collisions are benign by construction:
`lane = arena & 63` and the cold-bias arena is `base + 64`, so each arena
collides with exactly its own twin, never with an unrelated call-site group.
It also means the lane-sizing fix below buys such a machine nothing — its lanes
are already saturated, and matching lanes to 128 arenas within a fixed budget
would *halve* the depth to 2.

**The principled fix is to size the lanes to the runtime arena count**, which
is known at thread-cache creation: `getTotalArenas()` is 32 here and 8 on a
4-CPU host, against a fixed 64 lanes. That wastes half the cache on this host
and 87% of it on a small one, and pays for it in depth. Matching them is worth
**17% here at identical memory with routing untouched** (35.5 → 29.7 ns,
distinct arenas 14 either way) and considerably more on smaller hosts, where 8
arenas would give depth 32. It needs ThreadCache to use a runtime stride and a
runtime per-lane depth instead of the compile-time `void* ptrs[kPerLaneDepth]`,
so it is a refactor of the hottest path, not a constant change.

## The thread-cache drain path, and what actually costs in it

`drainRangeToSlabs` (`smash_heap.cpp`) returns a lane's worth of pointers to
their arenas' slabs: resolve each pointer's span, counting-sort by arena into
stack buffers, then one batch per non-empty arena. It runs whenever a free
finds its lane full (every ~128 frees, draining half a 256-entry lane) and on
every thread teardown, so it is the cross-thread-free path.

**It is expensive, and worth knowing by how much.** A batch allocate/free loop
of 64-byte objects, varying the batch across the 256-entry lane capacity so
drains go from never to constant:

| batch | smash | glibc |
|---|---|---|
| 64 (no drain) | 41.5 ns | 8.7 ns |
| 256 | 84.1 ns | 8.8 ns |
| 4096 | 117.4 ns | 9.4 ns |
| 32768 | 118.7 ns | 9.5 ns |

So draining roughly triples the per-operation cost of this pattern.

Two things in it were wasteful, and **the one that mattered was not the one
that looked like it would**:

- `uint32_t counts[kTotalArenas]{}` zeroed **1 KiB on every drain**
  (`kTotalArenas` was 256 — `kMaxArenas` 128, doubled for cold-arena
  feedback; it is 128 since feedback became a runtime switch)
  while `getTotalArenas()` is typically 8. Zeroing only the live entries:
  **119.0 → 110.5 ns, −7.1%** on the hot workload above.
- The span lookup used `PageMap::get()`, the two-level radix walk, where
  `free()`'s own fast path uses one load from the flat per-page table. Adding
  `PageMap::getFast()` and using it here gains **nothing measurable when the
  tables are hot** (110.5 → 111.4 ns, inside noise) and **−1.1% on a large
  shuffled working set** where they are not (779.4 ± 5.0 → 770.6 ± 4.5 ns over
  six runs). Keep both, but do not expect the second one to show up on a
  microbenchmark that reuses the same pages.

The frame is also ~9.5 KiB of stack buffers per call (`spans`, `arenas`,
`counts`, `offsets`, `cursor`, `out_ptrs`, `out_spans`, all sized for the
compile-time maxima). Suite 66/66.

## Per-page metadata costs ~1% of the arena, and it is sized for the whole arena

The compressor keeps fifteen per-page arrays (`compressed_`, `cold_count_`,
`page_tier_`, `shadow_tick_`, the page-state table, the per-page `Span*` table,
…). Summed, they are **43 bytes per 4 KiB page — 1.05% of whatever the arena
covers** — and they come from BootstrapAlloc sized for `vm->totalPages()`, not
for the pages actually used. `MAP_NORESERVE` keeps the untouched part off RSS,
but the bootstrap *capacity* must cover the whole arena:

| `SMASH_VM_GIB` | metadata capacity needed |
|---|---|
| 16 (default) | 0.18 GB |
| 128 | 1.44 GB |
| 512 | 5.77 GB |
| 1024 (ceiling) | 11.54 GB |

Against a capacity of 15.4 GB after the geometric-growth fix, and **1.07 GB
before it** — which is exactly why a run with `SMASH_VM_GIB=512` died at
~169 GB RSS with a write to address 0. Raising `SMASH_VM_GIB` is not free: it
costs ~11 MB of metadata capacity per GiB of arena even if the arena is empty.

For a workload's *resident* cost the same 1% applies to pages actually used, on
top of the compressed blobs. A 50 GB cold set that compresses 16.7x (measured
on a real application under zram, LZ4, which compresses page-at-a-time exactly
as smash does) lands at roughly 3.0 GB of blobs plus 0.56 GB of metadata —
about **14x effective**, against the codec's 16.7x.

## The ratio you measure depends entirely on the data you measure it with

`test_compression_ratio` and `bench_compression` both build their page with
`fillCompressiblePage()`: a fixed record header, a zeroed middle and a tail of
`(off + i) & 0xFF`. The comment rightly rejects an all-one-byte page as
degenerate, but the replacement is still **periodic** — the pattern repeats
with a period of at most 256 bytes — and periodicity is exactly what a
compressor is best at. Measured consequences, same codecs, same machine:

| data | zstd-1 at 4 KiB | zstd-1 at 2 MiB |
|---|---|---|
| smash's generator | 25–26x | **6043x** |
| synthetic heap-shaped (pointers, strings, ints, padding) | 3.13x | 3.52x |
| 64 MiB of real system binaries | 2.01x | 2.37x |

A 6000x ratio is not a compression result, it is a statement that the input
repeats. Two things follow. The **25–26x the in-tree test reports is an upper
bound**, not what an application should expect — the paper's own RQ2 figures
(LZ4 4.7–12.3x, zstd-1 8.8–20.4x) sit well below it. And **that data must
never be used to reason about unit size**: it says a 2 MiB unit is 240x better,
where realistic data says 8–20%. The test is still a good regression guard for
"did the codec break", which is what it claims to be; it is not a model of a
heap.

The same caveat applies to the codec choice. On smash's generator LZ4 gives up
only 4% of zstd's ratio (25.3x against 26.3x), which would make LZ4 look nearly
free; on realistic data it gives up 25–40%. Which of those an application sees
depends on its own bytes.

## The compression unit is one page, and bigger is only worth it up to ~64 KiB

A recurring question, and the reason THP degrades under smash (the compressor
protects and reclaims 4 KiB at a time, which splits a huge page). Measured with
zstd level 1, the tier smash actually uses, over 64 MiB of two data sets —
synthetic heap-shaped bytes (pointers, short strings, small ints, zero
padding) and 64 MiB of real system binaries — compressing the same bytes as
independent units of each size:

| unit | ratio (heap / real) | decompress per unit | compress per unit |
|---|---|---|---|
| **4 KiB** (today) | 3.13x / 2.01x | 7.9 / 7.4 µs | 15.5 / 16.4 µs |
| 16 KiB | 3.37x / 2.25x | 19.3 / 16.6 µs | 41.5 / 46.7 µs |
| 64 KiB | 3.36x / 2.32x | 58.4 / 48.0 µs | 145 / 151 µs |
| 256 KiB | 3.37x / 2.41x | 219 / 175 µs | 551 / 555 µs |
| 1 MiB | 3.52x / 2.36x | 853 / 675 µs | 2007 / 1953 µs |
| 2 MiB (THP-sized) | 3.52x / 2.37x | **1684 / 1338 µs** | 3947 / 3852 µs |

Two things follow. **The ratio gain is front-loaded and small**: +8% (heap) to
+12% (real) by 16 KiB, +20% at best anywhere, and it plateaus — real data is
*worse* at 2 MiB than at 256 KiB. **The latency is linear and unforgiving**: a
THP-sized unit makes every first touch cost 1.3–1.7 ms, against 8 µs today.
That is the argument against ever matching the compression unit to a huge page,
independent of how convenient it would be for THP.

(Numbers below are with compression contexts reused, as smash does —
`ZSTD_compressCCtx` / `ZSTD_decompressDCtx` and `LZ4_compress_fast_extState`.
Using the one-shot APIs instead inflates the 4 KiB rows by ~25% and leaves the
large ones unchanged, which flatters big units.)

**LZ4 shifts the trade but does not change the conclusion.** Same data, same
unit sizes, `LZ4_compress_fast` at acceleration 1 against zstd-1:

| unit | zstd-1 ratio | zstd-1 decomp | LZ4 ratio | LZ4 decomp |
|---|---|---|---|---|
| 4 KiB | 3.13x / 2.01x | 8.4 / 7.8 µs | 1.86x / 1.59x | **1.8 / 1.2 µs** |
| 16 KiB | 3.37x / 2.25x | 19.8 / 17.0 µs | 1.91x / 1.74x | 7.0 / 4.1 µs |
| 64 KiB | 3.36x / 2.32x | 58.8 / 48.8 µs | 1.93x / 1.84x | 27.3 / 15.9 µs |
| 2 MiB | 3.52x / 2.37x | 1644 / 1394 µs | 1.93x / 1.81x | 857 / 460 µs |

LZ4 decompresses **4–6x faster** and compresses **25–40% worse**, and it gains
even less from a larger unit than zstd does (1.86x → 1.93x across the whole
range on heap-shaped data) because it finds no long-range matches to exploit.
So the unit-size conclusion holds for both codecs: 2 MiB still costs half a
millisecond per fault under LZ4, which is 60x worse than the 8 µs a zstd-1 page
costs today.

The choice is therefore between axes, not sizes: **zstd-1 at 16–64 KiB** for
ratio, **LZ4 at 4 KiB** for fault latency (1.2–1.8 µs). smash exposes
the codec at startup (`SMASH_CODEC=lz4` picks LZ4 as the fast tier), and the ROI model chooses fast versus deep tier per page, so the
tiering to build on is in place.

The interesting range is **16–64 KiB**, and there is a second reason to look
there: the fault path already decompresses `SMASH_PREFETCH_WINDOW` = 2 pages in each
direction, so a fault that finds its neighbours compressed already pays up to
5 x 7.9 = 40 µs. One 16 KiB unit covers 4 pages for 19 µs *and* compresses 8–12%
better — cheaper than what the prefetch window already does, for a better
ratio. Nothing is implemented; the compressor is page-indexed in ~120 places,
`PageState` is per page and the store is keyed by page index, so this is a
refactor rather than a constant.

## Coalescing the reclaim syscalls (+41% compression throughput)

`reclaimShadowRange()` — the dominant path in the default deferred-reclaim
mode — issued **one madvise and one mprotect per reclaimed page**, and under
FixAv one membarrier per page as well. Its own comment said so ("each reclaimed
shadow page gets its own mprotect(PROT_NONE) … with no coalescing"), and an
interposer that counts calls by caller confirmed it: **422,151 madvise calls
over a 2 GiB cold set, every one of them 4096 bytes**. (The deferred-madvise
sweeper already coalesced; this path did not. Note that `protectPages` reaches
the kernel through `syscall(SYS_mprotect, …)`, so an `LD_PRELOAD` shim that
interposes libc's `mprotect` sees none of them — use `strace` for those and the
shim for `madvise`.)

Pages are visited in ascending index order within a chunk, and chunks ascend,
so reclaimable pages are usually adjacent. Accumulating a contiguous run and
issuing one syscall per run gives, measured on the same workload:

| | per page | coalesced |
|---|---|---|
| madvise calls | 422,151 × 4 KiB | **13,386 × 128 KiB** |
| compression throughput | 142–152 MB/s | **193–222 MB/s** |

**The run cap is 32 pages on purpose.** A page stays locked from the moment it
joins a run until the flush, so an application thread faulting on it waits for
the rest of the run's verification work (~10 µs per page). 32 bounds that at a
few hundred µs while still removing 97% of the syscalls; the sweeper can use
512 because it holds no page across decompression. Measured cold re-access
latency is unchanged at 0.2 µs/object.

Correctness rests on the locks being held across the flush: state and
`shadow_tick_` are set there too, so a fault on any page of a pending run still
sees `COMPRESSED_SHADOW` and blocks on the lock exactly as before. Every exit
from the loop flushes, including the VMA-budget breaks and the function's
return — a missed flush would leave pages locked forever. Suite 66/66 in both
the default and `SMASH_FIXAV=1` configurations, integrity checks clean in both.

## Batching the PROT_READ arm (+60% on top of that)

Coalescing the reclaim tail left **one** per-page mprotect in
`reclaimShadowRange()`: the `PROT_READ` arm that precedes the content check.
It was invisible to the counting interposer for the reason noted above —
`protectPages` goes through `syscall(SYS_mprotect, …)` — so it was attributed
with a temporary return-address counter compiled into `protectPages` itself.
Over a 1 GiB cold set it was **254,803 of the 794,668 remaining calls**, and
`compressPage`'s defensive `PROT_RW` was the other 539,865.

A look-ahead now arms a whole contiguous run in one call and remembers the
window (`armed_hi`), so the per-page path re-arms nothing. The run stops at the
chunk edge, at 32 pages, at the first page that is not `COMPRESSED_SHADOW` or
not yet past the delay, and at any address discontinuity.

**The look-ahead takes no locks on the pages it arms**, and that is safe for a
specific reason: arming only *tightens* protection, which this path already has
to tolerate racing with. Every page re-checks its state under its own lock
immediately afterwards, and a write that lands first faults, the handler drops
the blob and restores `PROT_RW`, and the loop then skips the page. A page armed
but never reached — the chunk ends, the VMA budget trips — costs one spurious
fault and heals the same way. Locking the look-ahead window instead would
reintroduce exactly the hold time the 32-page cap exists to bound.

Measured on a 1 GiB cold set (`strace -f -c`), all three stages:

| | per page | reclaim tail | + arm |
|---|---|---|---|
| mprotect | 1,048,620 | 794,668 | **540,716** |
| madvise | 262,168 | 8,216 | **8,216** |
| membarrier | 262,145 | 8,193 | **8,193** |
| MB/s (4 GiB, no strace) | 145–153 | ~208 | **237–260** |

The ~2.1 mprotect per page that remain are `compressPage`'s defensive
`PROT_RW`, one per page. Batching those needs per-page protection tracking —
the call exists to repair a page whose protection may have been changed by
another path — so it is left alone.

## Compression throughput is ~250 MB/s and does not scale with cores

**The numbers in this section were measured before the syscall batching above**
and are kept because the *shape* of the limit is unchanged — only the constant
moved. Re-measured on the identical 16 GiB workload after batching:
**254 MB/s** (was 162), CPU 122%, system 66.8 s against user 49.7 s. It is
still system-heavy, still nowhere near using the eight available cores, and
still a full pass over 200 GB in **~13 minutes** rather than ~21.

No counter silently caps compression — `SMASH_TIME_BUDGET_PCT`, the one knob
that marks buckets SKIP, is unset by default and returns -1 (no gating). What
limits it is the per-page syscall cost, and it gets worse with thread count
rather than better.

Measured, 16 GiB of compressible data left cold on an 8-core host:
**162 MB/s**, and 8 GiB gave 161 MB/s — flat. The telling number is the CPU:
**117%**, with system time (50 s) nearly double user time (28 s). Eight workers
are available and the machine has eight cores; compression uses barely one of
them, so it is not CPU-bound and more workers cannot help.

`strace -c -f` over ~485k compressed pages shows why (pre-batching counts;
mprotect is now ~2.1 per page and madvise and membarrier ~0.03):

| syscall | calls | per page |
|---|---|---|
| mprotect | 3,068,894 | **~6.3** |
| madvise | 485,843 | 1.0 |
| pwrite64 (clear_refs) | 485,843 | 1.0 |
| **membarrier** | 485,844 | **1.0** |

Both of the expensive ones scale badly with the application's thread count, not
with smash's: `mprotect` takes the process-wide `mmap_lock` in write mode, so
it serialises against every application thread's faults and mappings, and
`membarrier(MEMBARRIER_CMD_PRIVATE_EXPEDITED)` IPIs **every core running a
thread of the process** — once per page. The membarrier is load-bearing (it
drains other threads' store buffers before the page snapshot; without it a
racing store is lost), so it cannot simply be removed.

Measured effect of the application using its own cores (6 GiB cold, spinner
threads on an 8-core box): **164 MB/s idle, 93–98 MB/s with 4–7 busy threads**.
On this host that is confounded with plain CPU competition, so read it as a
direction, not a coefficient.

**At 200 GB this is still the binding constraint**: a full pass takes ~13
minutes at 254 MB/s (~21 at the pre-batching 160), and new cold pages queue
behind it. Measured directly — with a live
cold set, the time for a *fresh* cold block to compress was 8.0 s at 1 GiB
live, 42.6 s at 8 GiB, and **never within 60 s at 24 GiB**.

Two other fixed sizes matter at that scale, though neither is a silent cap:
`kFaultSlotCount = 128` bounds concurrent decompressions (a 128-thread app
faulting in parallel exhausts it exactly, and `acquireFaultSlot()` spins), and
`kMaxCompressorWorkers = 8` bounds the worker pool — irrelevant while the work
is syscall-bound, but it would become the next limit if the syscalls were
fixed.

## The arena's page allocator has one lock, and it collapses past ~8 threads

`VmRegion::allocatePages()` calls `free_map_.take()` **first on every large
allocation**, and `PageRunMap` is guarded by a single spinlock ("One lock
guards the whole map"). Frees reach the same lock through
`processDecommitEntry` → `insert()`. So every large allocation and every large
free serialises on one lock, whatever the arena count.

Measured with 2 MiB blocks churned across threads (8 physical cores):

| threads | smash total ops/sec | µs per alloc+free | glibc |
|---|---|---|---|
| 1 | 57k | 17.5 | 0.1 µs |
| 2 | **96k** | 10.4 | 0.2 µs |
| 4 | 70k | 14.3 | 0.1 µs |
| 8 | 32k | 31.0 | 0.3 µs |
| 16 | 22k | 45.7 | 0.3 µs |

Throughput peaks at **two threads** and then goes backwards. The cause is
smash's lock, not the kernel — forcing the same workload onto the direct-mmap
path (`SMASH_LARGE_ALLOC_VM_THRESHOLD` above the block size), which skips
`PageRunMap` entirely but still pays mmap/munmap, stays flat:

| threads | arena path | direct mmap |
|---|---|---|
| 1 | 17.5 µs | 19.0 µs |
| 8 | 20.7 µs | 20.3 µs |
| 16 | **49.3 µs** | 21.9 µs |

So beyond ~8 concurrent large allocations the compressible path is *slower than
falling back to uncompressed mappings*. For a workload whose bytes are large
blocks and which runs a thread per core on a large machine, this is the
scalability wall, and it is a design property rather than a tuning parameter:
the slab has per-(arena, class) locks and the mid-size bins are sharded by
arena, but the page allocator underneath them all has one.

Two directions, neither implemented: shard `PageRunMap` by owner/arena the way
everything above it is sharded, and cache freed runs per thread so the common
allocate-free-allocate cycle never reaches the shared map (which would also
address the ~10 µs churn cost below, since a cached run needs no mprotect and
no madvise).

## Every malloc/free cycle over 16 KiB costs ~10 µs (open defect)

Measured 2026-09-20 while chasing a report of ~1.6% CPU in `xxfree`. A block
larger than `kMaxSmallSize` that is freed and reallocated goes to the kernel
**every time**, in both modes. One block, allocated and freed in a loop
(`-fno-builtin-malloc`, so nothing is elided):

| size | glibc | smash large-only | ratio |
|---|---|---|---|
| 32 KiB | 17.2 ns | 9,664 ns | **562×** |
| 256 KiB | 17.6 ns | 9,684 ns | **550×** |
| 2 MiB | 16.1 ns | 12,773 ns | **793×** |

The cliff is at 16 KiB, not at `kLargeAllocVmThreshold`: 16 KiB costs 26 ns
(system passthrough), 17 KiB costs 11,125 ns. Two mechanisms, same effect —
`strace -c` over 20 k pairs shows **20,115 munmap + 20,198 mmap** for 32 KiB
blocks (below the 1 MiB threshold, so `LargeAlloc` maps each one directly), and
**19,063 mprotect + 10,836 madvise** for 2 MiB blocks (arena path: commit on
allocate, decommit on free). glibc avoids both by caching freed blocks and by
raising its mmap threshold dynamically once it sees mmap'd blocks being freed.

Nothing in the current configuration space mitigates it: `SMASH_NO_DECOMMIT=1`
(11.9 → 11.3 µs, and *worse* at 2 MiB), `SMASH_NO_COMPRESSOR=1` (9.7 → 8.2 µs),
`SMASH_VM_GIB=1` and full mode (10.0 µs) all land in the same range. The fix is
a **warm cache of freed large blocks** — keep a bounded set mapped and committed
for reuse instead of returning them to the kernel on every free, the way
jemalloc's decay-based purging and glibc's dynamic mmap threshold do. The
`PageRunMap` already recycles the *pages*; what costs is the syscall and fault
work around them.

**What this does NOT explain is small-allocation overhead**, which is modest and
was measured separately: a 64-byte malloc+free pair costs 6.7–7.2 ns under
glibc, 18.6 ns in large-only mode and 15.1 ns in full mode. Note the ordering —
large-only is *slower* than full mode for small blocks, because it pays smash's
dispatch and then the system allocator's work.

**Two micro-optimisations of the free path were tried and measured worse**;
neither is in the tree:
- Moving the `SMASH_COUNT_FREE` / `SMASH_TRACE_FOREIGN_FREE` diagnostics out of
  `freeSlow` into cold helpers. Their `char[64]`/`char[160]`+`Dl_info` locals
  give `freeSlow` a 208-byte frame and a `-fstack-protector-strong` canary, and
  `perf annotate` attributed ~18% of the function to the canary instructions.
  Removing it changed nothing: interleaved A/B gave 18.6 ns before, 19.0 ns
  after. The perf attribution was skid — samples landing on the instruction
  after a stalled load.
- A `largeOnlyPassthroughFree()` fast path in `free()`, rejecting non-smash
  pointers with a range compare so they never enter the cold `freeSlow`, gated
  on a counter of live non-arena `LargeAlloc` blocks. Measured **3.5 ns slower**
  (18.6 → 22.2 ns), consistently, interleaved.

The lesson for the next attempt: on this path, `perf` attribution and A/B timing
disagree, and only the timing is load-bearing.

## Two review findings that measured as noise or near it (2026-09-22)

- **An empty `dispatch(5)` every tick.** On Linux the coordinator calls `dispatch(5)` (the NUMA node-queue phase) every tick even when no node group has work, and `dispatch()` bumps `work_gen_` and busy-waits for every active helper. It looks expensive; it is not, because the coordinator ticks once per **second** (`kCompressIntervalMs = 1000`), and at idle the adaptive worker count is 1, so there is no barrier at all. Instrumented with `SMASH_NUMA_COMPRESSOR=0` (the single-node case) under 2 GiB re-dirtied continuously: 162 µs spent in empty phase-5 dispatches over 33 s, with the smash threads using 108% of a CPU. Left as is. Note that helpers poll `work_gen_` with `usleep(100)`, so one barrier with helpers active costs ~100–160 µs of coordinator spin; that bound is what a faster tick rate would have to revisit.
- **`[[gnu::always_inline, gnu::hot]]` on the wrong function.** Inserting `BootstrapAlloc::usage()` above `owns()` moved the attribute onto `usage()`, and `owns()` became an out-of-line `.constprop` clone called from `xxfree`, `xxrealloc` (4 sites), `xxmalloc_usable_size` and the sized frees. Interleaved, pinned, 30M malloc/free pairs: **no difference in full mode** (13.5–13.9 ns either way), but in large-only mode, where every small free goes through `freeSlow()`, the out-of-line call cost 0.6 ns per pair (22.7–22.9 → 22.2–22.4 ns, non-overlapping in 5 of 5 rounds) and ~0.5 ns per batched free. Fixed by moving the attribute back.

## The decommit-entry pool was one lock for every large free (2026-09-23)

A 128-thread `malloc_numa` workload on a 128-core, 4-node box profiled at
**36.6% of all CPU in `smash::Spinlock::lock()`, 32.3% of it under
`VmRegion::releasePages` ← `LargeAlloc::deallocate`**, and 10.2% in
`drainDecommitQueueNow` under `LargeAlloc::allocateOnNode`. `releasePages` takes
exactly one lock: `queueForDecommit` → `newDecommitEntry()` popped a 48-byte
queue record from one global list under one spinlock, and `recycleDecommitEntry`
pushed it back under the same lock — twice per large free, from every thread.
The lock also sat on the same cache line as `decommit_head_`, which every free
CASes, so the line bounced even while the lock was free.

The pool is now 256 shards (64 at first; see below), one cache line each, indexed by `sched_getcpu()`
(rseq-backed in glibc, so no syscall). An entry records its shard and is
recycled to it, not to the recycler's, so shards stay balanced although
entries are taken by freeing threads and returned by whichever thread drains.
No thread-local state, so nothing is stranded at thread exit.

**Measured here it is at best a small win, and that is expected:** an 8-CPU
box cannot reproduce 128 cores contending on one line. `malloc_numa` churn
(1–8 MiB blocks, node = thread % 4), interleaved: 8 threads (1:1) 73.7–75.3k →
75.6–76.5k ops/s, 3 of 3 rounds; at 64 and 128 threads (8x and 16x
oversubscribed) the runs overlap (128: 3.8–4.9k vs 4.2–4.9k over 8 rounds) and
are dominated by the node-allocation path's locks, not this one. The verdict
belongs to the 128-core box.

Profiling traps on the way, both cost a round trip: `perf report` sorts by
`comm,dso,sym`, so with named threads each thread is its own row and a 36%
lock shows as ~0.67% per thread — use `--sort sym`; and `-S` matches the
demangled name, so `-S _ZN5smash8Spinlock4lockEv` silently matches nothing —
use `--symbol-filter="Spinlock::lock"`. libsmash has no frame pointers in
RelWithDebInfo, so use `--call-graph dwarf` (or `lbr` on Intel).

**On the 128-core box it took `Spinlock::lock` from 36.6% to 30%, and the
remainder moved to `LargeAlloc::deallocate` (24%).** Its one direct lock is
the span-descriptor cache (`recycleSpanDescriptor`, and `newSpanDescriptor`
on every large allocation): the same pattern, one global list, justified in
its comment by "span creation is rare next to object allocation", which is
false for churned large blocks. It is sharded the same way now, 64 per-CPU
shards, and a descriptor returns to the shard it was taken from
(`Span::free_shard`, a byte of existing padding — `sizeof(Span)` stays 112).
Returning it to the freeing thread's shard instead would let a
producer/consumer pattern drain one shard into another and make the empty
side mint descriptors from BootstrapAlloc, which never frees. Neutral at 8
threads here (74.2-75.1k vs 74.2-75.6k); the span-descriptor race test
passes. `util/cpu_shard.h` holds the shard pick both pools use.

What an 8-CPU box does show at 1:1 is the next cost: `releaseHook` —
`releaseCompressedPages` walking every page of every freed block under its
page lock (40% at 8 threads, 30% in the large-only repro) whether or not any
page was ever compressed.

## One free map and one decommit queue per node (2026-09-23)

`smash_sim` swept 1..7 threads (one per core, `--os-call-ns 3000`) and the
Universal Scalability Law fitted the large-block path with a **serial fraction
of 9.1% with the run cache off and 19.5% with it on**, and no coherency term:
Amdahl-shaped, capped near 10x and 5x however many cores. The serialized work
was the one decommit queue and thread every large free goes through (that
thread ran a full core) and the one `PageRunMap` lock every owner's runs sit
behind — although runs of different owners never merge.

Now each node arena has its own `PageRunMap` and its own `DecommitQueue`
(head, in-flight counters, thread), kept in its `NodeArena` slot and created on
the node's first allocation; the shared arena keeps `free_map_` and
`shared_queue_`. A page belongs to one owner for life (a node chunk never
returns to the shared arena), so a map per owner costs address space, not
memory: the maps are demand-paged and each only touches its own pages. An
allocation that misses drains and waits only on its own owner's queue. The run
cache's trim stays on the shared queue's thread.

Interleaved, same sweep, fitted on 1..4 threads (past 4, the four node
threads plus the workers exceed this box's 8 cores):

| | 4-thread speedup | 4-thread ops/s | serial fraction |
|---|---|---|---|
| before, cache off | 3.18x | 631k | 9.1% |
| after, cache off | 3.82x | 740k (+17%) | ~0 |
| before, cache 128 MiB | 2.53x | 1.57M | 19.5% |
| after, cache 128 MiB | 3.55x | 2.13M (+35%) | 2.2% |

Contended free-map acquisitions at 4 threads fell from 101k to 27k, and the
drain waits vanished. No 128-thread projection is claimed: with four points
and little loss left the fit cannot separate serial work from coherency, and it
attributes the remainder to a quadratic term that predicts a spurious collapse.
The 128-core curve should be measured there with `smash_sim` (both builds, 8 to
128 threads), which needs no memory.

Cost: one more decommit thread per NUMA node in use. They polled every 100 us
when idle until 2026-10-14 (31 600 wakeups/s with four nodes); they park now —
see defect-history.md, "Idle decommit threads polled".

## 64 shards paired CPUs across sockets (2026-09-24)

The three per-CPU pools (decommit entries, span descriptors, run cache)
started with 64 shards, picked as CPU mod 64. On the 128-core, 4-node box CPU
c and c + 64 shared a shard, so node 0 shared every shard with node 2 and node
1 with node 3: each shard's lock and cache line crossed sockets, and each
run-cache shard's 8 slots were split between two CPUs. Pinning threads to
their node made the pairing systematic, and `smash_sim` showed the symptom as
pinned threads being *slower* at 32 threads (1.5M against 2.0M
allocate+free/s unpinned), while pinning still helped at 128 (1.3M against
1.0M). All three pools are 256 shards now: one per CPU on hosts up to 256.
The run cache's cap is per CPU, so its worst case is CPUs x cap.

## Every large free bumped one global counter (2026-09-25)

With sub-arenas the node and shared paths no longer meet at a lock, and
`smash_sim` on the 128-core, 4-node aarch64 box still barely scaled with
threads (plain malloc, pinned: ~1.5M allocate+free/s at 128 threads) while the
busiest lock site waited 1.8% of thread time. `perf` put **49% of all time in
`PageMap::set`**: `LargeAlloc::deallocate` clears the block's page-map entry,
and every teardown did `g_pagemap_generation.fetch_add` — one cache line
written by every large free on every core. The counter only invalidates
free()'s per-thread last-span cache, and that cache must then never hold a
large span: descriptors are recycled, so a stale entry would name whatever
block its descriptor describes next (`test_large_free_span_cache` frees a
live 1 MiB block that way when large spans are cached without the bump).
Large spans are no longer cached (the fast path dispatches them before the
fill) and a large free clears its entry with `bump = false`; slab and mid
teardowns still bump.

On the 8-CPU host the bump was worth 1-4% (plain malloc with sub-arenas, 8
threads: 7.21M -> 7.62M). The simulator had its own global counter on the
same path (`g_next_block_id`, a block id taken even with `--no-check`,
removed in `2d4852a`); removing it alone did not help the 128-core box, which
is how the page-map counter was found.

## Freed runs came back too slowly, and the wait for them was too short (2026-09-26)

`test_shared_arena_churn` phase A (32 threads churning 2 MiB blocks through a
1 GiB arena) failed on the 128-core aarch64 box with 1192/1173 of 160000
blocks outside the arena. With that much churn nearly the whole arena is in
the decommit pipeline at any instant, so an allocation that misses has to wait
for a run to be processed. Two things made that wait fail:

- **The drain was serialised by a redundant `mprotect`.** Every entry did
  `mprotect(RW)` before `madvise`, to undo compression's PROT_NONE — even when
  the release had reported no COMPRESSED page. `mprotect` takes `mmap_lock` for
  writing: 0.8 us per call alone, 251 us at 32 threads, and it dragged the
  `madvise` from 6.7 to 123 us. A reusable run now skips it (the entry carries
  `reusable`); a PROT_READ page left by monitoring is restored by the fault
  handler once the next owner marks the page ACTIVE, which every taker does.
- **The wait was 64 `sched_yield`s**, a few hundred microseconds. It is now
  at least that, then up to 50 ms while runs are still in flight, in the
  shared, node and sub-arena retry loops.

Phase A: 0 of 160000 out, idle and with three copies at once (was 188-264 and
466-548), and the phase takes half the time (1.1-1.3 s vs 2.3-2.5 s). The
timed wait alone also reaches 0; the `mprotect` skip alone 23-85.

## Counting mappings cost more than the compression (2026-09-28)

An application on the 128-core arm64 box under `SMASH_SOFTDIRTY=0` had 503842
mappings, 115 GiB watched, nothing compressed and RSS at 194 GB. Page
protection splits mappings — a write to one page of a read-only run makes that
page writable on its own — and smash counted them by reading
`/proc/self/maps`: once per tick, at the start of every reclaim and sweep range
(every worker, every node group) and again every 256 pages reclaimed. The read
is linear in the mappings (193 ms at 600k, 341 ms at 1M on the 8-CPU host) and
holds the process's `mmap_lock` while the kernel formats it. Reproduced with an
idle 1 GiB next to 1M mappings: 28% compressed after 120 s, against 100% after
16 s without them, a rate matching one read per 256 pages. A shared estimate
(last count plus the runs smash split since, erring high) now serves every
thread; the real count is re-read at most every 20x the last read's cost (at
least 1 s), or every second within a quarter of the cap. 600k mappings: 100%
after 17 s; 900k: 92.8% after 20 s. `test_idle_compress_protread_many_vmas`.

## Phase 3 protected every page with its own mprotect (2026-09-28)

After the `/proc/self/maps` fix the arm64 application still compressed at
~1 MB/s. The new `[smash pipeline]` line showed why: 11 ticks in 9 minutes,
the last 68 s and the slowest 142 s — and everything in the pipeline counts
idle time in ticks. Phase 3 (arm live pages PROT_READ) grouped pages into runs
of 16 but called `mprotect` once per page inside the run: ~30M calls per full
pass there, each a TLB shootdown to every core of a 128-core box in a process
with 500k mappings. Locally (8 GiB, 8 threads writing 16k scattered pages/s)
phase 3 was 2732 of a 2752 ms tick, arming 1.8M pages in 113k runs.
`flushPhase3Run` now checks each page under its lock and arms each contiguous
stretch with one `mprotect`; the run cap is 256 pages (the
`SMASH_PROTECT_CHUNK_PAGES` knob, which nothing had read). Same workload:
2M pages armed in 7708 calls and 26 ms; steady-state phase 3 66-75 ms (the
single pages re-armed after scattered writes — one call each, inherent).

## Phase 2 also protected every page on its own (2026-09-28)

With Phase 3 batched, the arm64 application's ticks were still 80-170 s, now
`phase_ms=1694/611220/16184/150`: phase 2. Per page it made two `mprotect`
calls — escalation to PROT_NONE (read detection before compressing) and,
in `compressPage`'s default deferred path, an unconditional PROT_RW so the
snapshot could read the page. Now:

- escalations are collected per range and armed by `escalateBatch`: every
  page locked and checked, one `mprotect` per contiguous stretch (the idle-bit
  path under root stays per page — it makes no `mprotect`);
- the snapshot reads the page through `/proc/self/mem` (already open for
  `restorePageContents`), which works whatever the page's protection, and
  falls back to the old `mprotect` + `memcpy`;
- the pages left in COMPRESSED_SHADOW that way are made PROT_READ in batches
  (`flushShadowReadable`, one call per contiguous stretch) — shadow pages stay
  readable, as `test_prefetch` requires, and a write now faults and drops the
  blob at once instead of being caught by Phase B's content compare;
- node-group threads take contiguous slices of their queue, not every n-th
  entry, so their pages are adjacent and batch too.

256 MiB idle: phase-2 `mprotect` calls 131072 -> 768. 8 GiB with 16k scattered
writes/s: ~37k calls for ~2.07M pages, all released by tick 32. Shadow-write
stress (writes landing in the shadow window, every value read back): 0 stale
of ~85k checks.

## Hot pages were re-armed every tick (2026-09-29)

With phases 2 and 3 batched, the arm64 application still had 140 s ticks and
per-phase maxima of minutes (`max=184400/304810/18509/20927`). The
`[smash phase time]` split (17 ticks, one worker) showed the compressor was
not computing:

| phase | usr | sys | runq | blocked (s) |
|---|---|---|---|---|
| 1 scan | 2.1 | 42.6 | 13.4 | 66.7 |
| 2 select/compress | 11.0 | 111.4 | 27.6 | 96.0 |
| 3 arm | 2.1 | 49.0 | 22.0 | 147.4 |

Phase 1 makes no system calls, yet spent 109 s in the kernel or blocked
across 39.6k minor faults (~3 ms each): waiting on the `mmap_lock`. The
writers were the application's own threads: `faults monitor=8161544` in 17
ticks, each a SIGSEGV plus an `mprotect(PROT_RW)` of one page (splitting a
mapping), against the compressor's ~1.7M calls. Phase 3 re-armed every ACTIVE
page every tick, so a page the application kept writing cost two `mprotect`
calls and a fault per tick, forever. Raising the worker count
(`SMASH_CPU_PRESSURE_CAP=0`, 4 workers) did not change the maxima.

Now a write fault on a watched page within a tick of its arming
(`cold_count_` <= 1) raises the page's exponent and makes it wait 2^exp ticks
unarmed (`rearm_wait_`, counted down by Phase 3; cap `SMASH_REARM_BACKOFF_MAX`,
default 6 = 64 ticks); a fault after >= 8 quiet ticks clears it. While it waits
Phase 1 holds its cold count at 0: it is unwatched, and the next fault must see
how long it stayed unwritten *after* re-arming. 8 MiB written continuously:
1.00 -> 0.16 write faults per hot page per tick over 19 ticks (and ~1/64 at the
default cap in steady state); idle memory beside it and the hot block once its
writer stops still leave RSS. Counters: `rearm hot= waits=` on the pipeline
line.

First run on the box (tick 17, same workload): write faults 8.16M -> 4.37M,
compressed + shadow 1.55M -> 2.87M, released 684k -> 1.65M; ticks still
97-135 s. Only 46% of faults counted hot: "rewritten within a tick" is a
100 s window when ticks are that long. `/proc/<pid>/maps`: 481k lines, 181k
`---p` and 216k `r--p` anonymous — smash's own protection runs, the process
itself would have a few hundred. Next: `SMASH_RESTORE_GRANULE_PAGES`
(config.md), opt-in until `group nbr_active=` shows the writes cluster.

Second run (2026-09-30, tick 26, 16-page group restore on): compression at
scale at last — 24.4M pages stored, 4.58M compressed and dropped, 7.05M zero
pages, 19.45M still in shadow (released two ticks after compression,
`SMASH_DEFERRED_RECLAIM_DELAY`). But ticks of 724 s: phases 316/386 s, 99% of
tick time in phases (the "75 s outside the phases" earlier was a dump mixing
two ticks). The split had moved: p2 usr/sys/runq/blocked = 236/1228/2527/3307
s, p1 11/508/663/1484 s — waiting for a CPU was now a third of it, with one
worker. Phase 1's 74k minor faults cost ~20 ms each in `blocked`.

The user then pointed out that compression did work at scale, but only
after at least 30 minutes. Every stage counted ticks — cold after 10, one
PROT_NONE read-watch tick, two ticks in shadow: ~14 ticks from arming to
release, 14 s at 1 s ticks and 30 min to hours at 2-12 min ones. Idle time now
advances by the tick's length in nominal seconds (`tick_inc_`,
`nominal_clock_`), stopping at the cold floor; `test_slow_tick` with ~4 s
ticks: idle 64 MiB out of RSS after 21 s instead of still resident after 40.
Under ~700 s ticks the same pipeline takes ~4-5 ticks.

Third run (2026-09-30, `cc8bf50`, `SMASH_CPU_PRESSURE_CAP=0`): compression
from the first minutes (tick 9: 11.8M pages stored, 4.9M released), ticks
166-207 s. Phase 2 thread time 54/273/652/200 s (usr/sys/runq/blocked), and
the new syscall timing named the biggest item: `memread=709400 ms (23584400
calls)` — the snapshot read every page through `/proc/self/mem`, twice with
FixAv (snapshot + verify), ~30 us each under that load. Phase 2's `mprotect`s
were 84 s, phase 3's 90 s, phase 4's 105 s. Now compression candidates are
batched (`makeReadableForSnapshot`): the still-watched ones (PROT_READ or
escalated PROT_NONE — not writable, so nothing can land during the copy)
get one `mprotect(PROT_READ)` per contiguous stretch under their locks and a
plain `memcpy`, no verify; anything else takes the old path. 256 MiB idle:
131072 reads -> 0, phase-2 `mprotect` calls 768 -> 512 (the shadow arming is
no longer needed for them). Writes racing compression (16 threads, compressible
pages, 3 x 40 s): 0 stale of 509952 checks.

Fourth run (`6d96149`, tick 10): 26M pages compressed and released plus 9.5M
zero pages — but one tick of 1448 s, phase 2 1359 s of it, with 2474 s of
*user* CPU for 28.2M compressions (88 us/page; LZ4 is ~5). Nearly every page
took the deep tier (zstd-9): it is eligible at `kVeryColdTicks` (60) cold
ticks, and with idle time in nominal seconds a page reaches that before it is
compressed whenever ticks are long — before, it was compressed a tick or two
after the floor of 10 and got the fast tier. `SMASH_VERY_COLD_TICKS=9999`
removes the deep tier. Also still 16.9M snapshot reads: pages compressed
locally by `flushNodeQueries` (node unknown, own, or its 64K-page queue full
during the backlog) bypassed the readable batches; fixed. Phase 4's releases
took 928 s of syscall time for 26.5M pages (runs capped at 32).

Coarse watching (`SMASH_WATCH_GROUP_PAGES`, 2026-10-09): the other half of
the mapping problem. Pages that go cold seconds apart were escalated,
compressed and released in different ticks, so every tick's protection
changes hit a scattered subset — 64 MiB written in random page order split
into 9445 mappings (one boundary every 1.7 pages) and took 40880 protection
calls. With 16-page groups that become idle together: 678 and 11141. Phase 2
still averages ~10 calls per group (likely node-queue routing and batch
edges splitting groups); not yet looked into.

The CPU-pressure cap (`cpuPressureWorkerCap`) subtracted **every**
thread of the process from `nproc`, busy or not, so a process with >= nproc
threads always gets one worker. Fixed now (load average, floor
`clamp(nproc/16, 2, 8)`; see config.md) — earlier more workers did not help
because everything was lock-bound, but with a third of phase time in the run
queue they now should.
