# Allocation paths: which sizes go where

How allocations are routed (arena vs direct mapping, slab vs LargeAlloc, mid-size bins, THP) and what each route costs. Moved from CLAUDE.md.

## Hugepages and THP

**Explicit hugepages (`MAP_HUGETLB`): supported by exclusion.** The mmap
interposer's filter skips them (`isTrackableMapping()`), so an application's
hugetlb mappings keep working and are never tracked or compressed. That skip is
load-bearing, not an omission — a hugetlb VMA only accepts huge-page-aligned
operations, so every `mprotect`/`madvise`/`MAP_FIXED` smash would issue on one
is rejected with EINVAL, and before the filter existed a 16 MiB mapping produced
8192 failing page operations in six seconds (`test_hugepage_no_mprotect_storm`).
smash never allocates hugetlb memory itself.

**THP: available, opt-in via `SMASH_THP=1`, and destroyed by compression.**
By default smash issues no `MADV_HUGEPAGE`, so on a `THP=madvise` host (the
Ubuntu default) the arena gets no huge pages, exactly like glibc's malloc —
measured 0% for both. `SMASH_THP=1` madvises the arena and it becomes
huge-page backed (510 MiB of a 512 MiB block). Whether that is worth having
depends entirely on the access pattern, and the two directions were measured:

| workload | no hint | `SMASH_THP=1` |
|---|---|---|
| 256 MiB hot (rewritten) + 256 MiB cold | RSS 284 MiB, AHP 0 | RSS 286 MiB, **AHP 256 MiB** |
| sparse: one 4 KiB page per 2 MiB over 512 MiB | RSS 24 MiB | RSS **534 MiB** |

So a dense hot set keeps huge-page coverage while the cold set still compresses
(2 MiB of extra RSS), and a sparse one inflates RSS 22× because every touched
page drags in a whole huge page. That is the opposite of what this allocator is
for, which is why it is off by default. `test_thp_arena_{off,on}` guard it and
skip unless the host policy is `[madvise]`, the only setting where the two
cases can be told apart; verified as a real discriminator (the on-case fails
against a library without the knob).

**Compression dismantles huge pages either way.** The compressor protects and
reclaims 4 KiB at a time, which splits a huge page, and the restore path is a
per-page `MAP_FIXED` remap that never rebuilds one. On a `THP=always` host,
touching 512 MiB of malloc'd memory:

| | glibc | smash (full mode, compressor active) |
|---|---|---|
| after touch | RSS 513 MiB, AnonHugePages 512 (100%), 24 VMAs | RSS 592 MiB, AnonHugePages 584 (**99%**), 101 VMAs |
| after cold wait | unchanged | RSS **99 MiB**, AnonHugePages 92, 110 VMAs |
| after reading it all back | unchanged | RSS 594 MiB, AnonHugePages **74 (12%)**, 114 VMAs |

So compression works on THP-backed memory (592 → 99 MiB here) and the VMA count
stays flat — no splitting storm. What is lost is the huge-page backing: a
workload that faults its whole live set back ends up at ~12% AnonHugePages
instead of ~99%, and only khugepaged can re-collapse it. RSS before compression
is also higher under smash (592 vs 513 MiB) because the arena's huge pages
round up. **hugetlb is a different matter and is not possible here at all**: a
hugetlb VMA rejects every sub-2 MiB `mprotect`, `madvise` and `MAP_FIXED`
smash's page management depends on (measured directly — EINVAL, EINVAL,
ENOMEM), so backing the arena with `MAP_HUGETLB` would require the compressor
to work in 2 MiB units end to end.

## Which allocation sizes are actually compressed

`kLargeAllocVmThreshold` (1 MiB) decides this, and the effect is absolute:
**an allocation below it never enters the compressible arena and is never
compressed**, in either mode. Measured with the exported `smash_in_arena()`
plus `mincore` after a cold wait, 8 blocks per size, filled with highly
compressible data:

| size | in arena | resident after cold wait |
|---|---|---|
| 320 KB / 512 KB / 768 KB | 0/8 | **100%** — untouched by the compressor |
| 1 MiB and above | 8/8 | 0% |

Same result in full mode and large-only. So a workload whose memory sits in
16 KiB–1 MiB blocks pays all of smash's overhead — including the ~10 µs
malloc/free cycle in [performance.md](performance.md) — and gets **zero** RSS reduction on
that memory. The comment on the constant explains the policy ("typically
internal engine buffers accessed frequently; compressing them causes
decompression storms"), which is a real risk, but it is a blanket rule rather
than a cold/hot decision, and smash's own cold detection already exists to make
that call.

**`malloc_numa` and `malloc_numa_aligned` are the exception: they ignore the
threshold at every size.**
`LargeAlloc::allocateOnNode` has no size test — it always tries
`VmRegion::allocatePagesOnNode` first, and only falls back to a separate bound
mapping (never compressed) when there is no VmRegion or it is exhausted.
Measured on a 4-node host, node 0, after a cold wait: `malloc_numa` blocks of
4 KB, 64 KB, 320 KB, 768 KB, 1 MB and 4 MB all report `smash_in_arena() == 1`
and 0% resident, while plain `malloc` of 64 KB / 320 KB / 768 KB is 100%
resident. So a caller that can use `malloc_numa` gets sub-threshold blocks
compressed today with no rebuild — at the cost of binding that memory to one
node (`MPOL_BIND`), page granularity, and `realloc` returning an unbound block.
`malloc_numa_aligned(node, size, align)` is the same deal with an alignment
guarantee, which is otherwise unavailable on a compressible block: ordinary
`posix_memalign`/`aligned_alloc` above `kMinAlignment` go to `LargeAlloc`'s
over-map-and-trim path, which is a direct mapping and is never compressed.

**Above the threshold, size is not a limit.** 2 MiB, 32 MiB, 256 MiB and 1 GiB
blocks all land in the arena and compress to 0% resident. What bounds them is
arena *capacity*, and missing it is silent by default — which for a workload of
multi-MB or GB blocks is the difference between smash working and smash doing
nothing. With a 2 GiB arena and four live 1 GiB blocks, the first two compress
to 0% and the last two sit at 100% resident forever, because `LargeAlloc` fell
back to a direct mapping the compressor never walks. `LargeAlloc` now says so
once, with the same capacity-vs-fragmentation split as
`Slab::warnArenaExhausted`:

```
[smash WARN] a 1024 MiB allocation did not fit the compressible arena: 2048 of
2048 MiB handed out, 0 MiB free in 0-page runs or shorter. It came from a
direct mmap instead and will NEVER be compressed, however cold it gets. …
```

`test_vm_exhaustion` asserts the warning appears (it already forces the
fallback); **verified as a real discriminator** — against a library without it
the test fails with "smash said nothing". So size the arena above the live set:
`SMASH_VM_GIB` costs address space, not memory.

**`malloc_numa` warns too (2026-10-27).** A node arena that cannot take a block
falls back to a separate mapping bound to the node, never compressed, and until
then said nothing (`LargeAlloc::allocateOnNode`). It now prints its own one-shot
line, with the same phrase so one grep finds both, the node, and how much of
the arena the node holds and has free:

```
[smash WARN] a 4 MiB malloc_numa allocation for node 0 did not fit the
compressible arena: 1024 of 1024 MiB handed out, node 0's arena holds 1024 MiB
of it with 0 MiB free. It came from a separate mapping bound to node 0 instead
and will NEVER be compressed, however cold it gets. The arena is full: …
```

When the node's free pages cover the block, the line says the node's arena had
the space but could not give it to this block (runs too short, another
sub-arena's, or still being decommitted) instead. `test_vm_exhaustion_numa`
overflows a 1 GiB arena through `malloc_numa(0, …)` (128 of 384 4 MiB blocks
fall out) and requires exactly one such line: the library before it printed 0,
and a mutant without the one-shot guard 128.

**Both warnings print only once the block exists (2026-10-28).** They used to
print before the fallback mapping was tried, so a request no mapping could serve
(a 16 TiB `malloc`, a `malloc_numa_aligned` at 2^63 alignment) got NULL, said
"it came from a direct mmap" (or "a separate mapping") falsely, and spent the
one-shot: the real overflow that followed printed nothing. Each also weighs the
block against the whole arena first: a block larger than it (or, for
`malloc_numa_aligned`, its pages plus the worst-case alignment gap) gets "The
block needs N MiB, more than the whole M MiB arena: raise SMASH_VM_GIB above
that", where the plain line blamed "the live set" and the node line said its
arena "has the space" (it weighed the free pages against the block alone). A
block under 1 MiB is named in KiB, not "0 MiB", and an aligned node block gets
"at N MiB alignment". Guarded by `test_vm_exhaustion{,_numa,_numa_aligned}`
(a refused request first, which must print nothing) and
`test_vm_exhaustion_too_big` ([tests.md](tests.md)).

Still silent: `posix_memalign`/`aligned_alloc` above `kMinAlignment` on a large
block always take a direct mapping (above), from the first allocation, so there
is no overflow to report.

The threshold is a **runtime** setting, `SMASH_LARGE_ALLOC_VM_THRESHOLD`
(bytes, rounded up to a page, floored at one page), defaulting to the
compile-time `kLargeAllocVmThreshold`. The compile-time define still works as
the default (`cmake .. -DSMASH_LARGE_ALLOC_VM_THRESHOLD=131072`), but no rebuild
is needed to change it:

```bash
SMASH_LARGE_ALLOC_VM_THRESHOLD=262144 LD_PRELOAD=libsmash.so ./app   # 256 KiB
```

Verified: with the default, 320/512/768 KB blocks are 0/8 in the arena and 100%
resident after a cold wait; at 262144 they are 8/8 in the arena and 0%
resident. At a 128 KiB threshold the 320 KB churn cost also drops from 9.7 µs
to 6.5 µs per malloc/free cycle, since the arena path beats mmap/munmap.
`test_large_alloc_threshold_{default,lowered}` guard it — one body, with the
expectation derived from the env var so the two runs assert the same invariant.
**Verified as a real discriminator:** the lowered registration fails 3 checks
against a library without the runtime knob, while the default registration
passes against both (so the default behaviour is unchanged).

**The real reason for the gap is not the one the comment gives: it keeps the
arena able to serve large contiguous requests.** A big allocation needs an
unbroken run of pages, and mid-size blocks scattered through the arena are what
makes such a run impossible. Measured with a 4 GiB arena filled to 3.75 GiB
with 512 KB blocks, every other one then freed, followed by a 1 GiB request:

| | mid-size blocks in the arena | the later 1 GiB allocation |
|---|---|---|
| default threshold | 0 of 7680 | **lands in the arena** (compressible) |
| threshold 128 KiB | 7680 of 7680 | **falls back to a direct mapping** (never compressed) |

So for a workload that mixes mid-size blocks with multi-hundred-MB or GB
allocations, lowering the threshold can cost more compression than it gains:
the mid-size blocks become compressible, and the big ones stop being. The
arena is also a fixed budget (`SMASH_VM_GIB`), and mid-size blocks consume it.
Neither point is in the comment on the constant, and both are stronger than the
one that is.

**Small objects are not exempt, they fail differently.** The same shape with
64-byte objects also denies the 1 GiB request, but for the opposite reason:
freeing every other object of 31 million returned **0 MiB**, because objects
share pages and a page comes back only when every object on it is free. So
small objects produce *occupancy* (raise `SMASH_VM_GIB` and it helps) while
mid-size blocks produce *holes* (a larger arena only postpones it). Telling the
two apart is what the warning is for, and getting that test right matters:
"free-to-total ratio" is not the distinction — an arena 24.9% free in 512 KB
runs was reported as "the live set outgrew the arena" when the truth was
fragmentation. The test is now exact: enough free pages but no run long enough.

**Freed arena memory is not immediately reusable.** Frees go through the
deferred decommit queue before returning to the free map, so a burst of frees
followed straight away by a large allocation sees only part of it: measured
1004 MiB free immediately after freeing 1919 MiB worth, and the full 1919 MiB
three seconds later. A program that frees a lot and then asks for one big block
can therefore miss the arena and land on an uncompressible direct mapping,
purely on timing.

**The storm the constant's comment warns about did not appear when tested.**
With the threshold lowered so 512 KB blocks are eligible, 64 continuously
rewritten buffers stayed **100% resident** while 64 untouched ones went to
**0%** — cold detection made exactly the distinction the size gate is a proxy
for. That is the argument against the gate as a *compressibility* rule: smash
already answers "is this hot?" per page, from observed access, and the size
gate answers it by proxy while also excluding genuinely cold mid-size memory.

**But the gate also decides arena placement, and that part cuts differently in
the two modes.** Below the threshold a block is a direct mmap (a syscall pair
per allocation); above it, it comes from the arena. Measured with mi-stress
(32 threads, default args) and a 320 KB churn loop:

| | large-only | full mode |
|---|---|---|
| mi-stress, default 1 MiB | 7.10 / 7.23 s | 16.24 / 17.26 s |
| mi-stress, threshold 16 KiB | **4.64 / 4.59 s** | 18.19 / 18.87 s |
| 320 KB malloc/free cycle | 9.85 µs → **7.59 µs** | — |

In large-only the arena is much cheaper than the syscalls it replaces (35%
faster); in full mode the same change is ~12% *slower*, because small objects
already come from the arena there and mid-size blocks add contention rather
than removing syscalls. `bench_sqlite --quick` is unchanged either way (54.1 /
53.8% reduction at the default, 52.0 / 54.1% at 16 KiB; ops/sec within noise).

So the knob is genuinely workload- and mode-dependent, which is the case for
having it at runtime rather than picking a new constant. A workload in
large-only mode with cold mid-size blocks wants it lowered on both counts;
full mode wants measurement first.

## calloc clears only what is not zero already

`calloc` goes to `SmashHeap::calloc` (through `SmashRedirect`, smash_heap.cpp;
alloc8's own `calloc` was `malloc` + `memset` of the whole block). Passthrough,
compress-only, compressor threads and, in large-only mode, sizes up to the
threshold get libc's `calloc`. A slab or mid-bin block is always cleared. A
block past `kMaxSmallSize` is cleared only when `LargeAlloc::allocate` cannot
vouch that its pages read as zero: a fresh mapping (below
`SMASH_LARGE_ALLOC_VM_THRESHOLD`, or an arena miss) and an arena run that was
never used or was decommitted are not written, so `calloc(1, 256 MiB)` costs no
RSS until the pages are touched, as with glibc. A sub-arena's dirty run, a
span that includes one and a run-cache hit are cleared, and so is every arena
run under `SMASH_MADV_FREE=1` or `SMASH_NO_DECOMMIT=1`, whose decommit keeps
the bytes. The invariant this rests on is in
[conventions.md](conventions.md); the history in
[defect-history.md](defect-history.md).

## Aligned small allocations come from the slab

`memalign()` sent every alignment above `kMinAlignment` (16 B) to `LargeAlloc`,
so an `alignas(64)` 64-byte object cost a whole page **and** an mmap/munmap
pair. No new "aligned slab" is needed to fix that, because the existing classes
already provide the alignment: an object's address is
`span->base + slot * object_size`, and a span's base is always page-aligned, so
for a power-of-two alignment no larger than a page **every object of a class
whose size is a multiple of that alignment is aligned**. `alignedSizeClass()`
picks the smallest such class and `memalign()` then requests exactly
`classSize()` bytes — `sizeToClass(classSize(c)) == c`, so this reuses the
ordinary small path and produces an ordinary object that `free()`, `getSize()`
and the thread cache already handle, with no new state anywhere.

The table's 16-byte steps are what make this cheap: for 64-byte alignment only
the 16/32/48/80/96/112/160/224 classes are not multiples, so a request rounds
up to the next class rather than to a page.

Measured under interposition, 200 k live objects, full mode:

| request | before | after |
|---|---|---|
| 64 B @ 64 B align | 4226 B/object | **76 B/object** |
| 100 B @ 64 B align | 4226 | 140 |
| 128 B @ 128 B align | 4225 | 140 |
| 200 B @ 256 B align | 4226 | 269 |
| 4096 B @ 4096 B align | 4225 | 4139 (the object *is* a page) |
| alloc+free latency | **9755 ns** | **20 ns** (glibc: 10 ns) |

`test_aligned_alloc_footprint_full` goes from 66× to **1.2×**. Correctness swept
over `posix_memalign`, `aligned_alloc` and `operator new(align_val_t)` for
alignments 32 B–8 KiB against 21 sizes from 1 B to 64 KiB: 4536 allocations per
configuration, checked for alignment, usable size and byte stability, 0
failures in all four of {full, large-only} × {routing on, off}.

**The other approach — searching a normal slab for aligned slots — is built
but off by default (`SMASH_ALIGN_SLOT_SEARCH=1`), because it usually loses.**
It keeps the request in its *natural* class and takes a slot whose index is a
multiple of `alignment / gcd(class_size, alignment)`. The slots it steps over
are stranded unless something else allocates them, so its cost is
`class_size x stride` = `lcm(class_size, alignment)`. Computed over all 131072
(size, alignment) pairs, that **never beats promotion** when the workload
allocates only aligned objects: equal in 110928 cases, worse in 20144, worst
**7.0x** (97 B at 128-byte alignment — natural class 112 B x stride 8 = 896 B
against a promoted 128 B). Measured, 200 k live 48-byte objects at 64-byte
alignment: 76 B/object promoted, **202 B/object** searched.

It wins only when ordinary allocations of the same class fill those slots, and
**call-site arena routing normally prevents exactly that**: `callsiteArena()`
sends a `posix_memalign` call site and a `malloc` call site to different
arenas, hence different slabs, so the plain allocations never reach the skipped
slots. Measured with 200 k aligned 48-byte objects interleaved with 600 k plain
ones — 62 B/object promoted, **94 B/object searched** with default arenas, i.e.
worse, and 61 B/object once `SMASH_NUM_ARENAS=1 SMASH_CPU_ARENA=0` forces them
to share.

Where it does pay is a large alignment on a small object, with traffic to fill
the gaps. 50 k objects at 4096-byte alignment and 64 bytes each, interleaved
with 3.2 M plain 64-byte allocations, single arena: **137 -> 74 bytes per
object, 444.5 MiB -> 241.3 MiB**. Promotion has to round those to the 4096-byte
class; the search leaves them in the 64-byte class.

So enable it only with evidence that the aligned and plain allocations of a
class share an arena — which in practice means collapsing arenas — and that the
alignment is far larger than the object. The suite passes 70/70 with it forced
on, and the aligned-API sweep (4536 allocations per configuration) is clean in
both modes with it on.

`SMASH_ALIGN_SLAB=0` restores the old routing. Alignments above a page, and
sizes above `kMaxSmallSize`, still go to `LargeAlloc` — as does everything in
large-only mode, where small aligned requests pass through to the system
allocator (2.1×).

## Mid-size bins (`SMASH_MID_BINS=1`, opt-in)

Serves the (16 KiB, 1 MiB) band from size-binned chunks carved out of the arena
instead of one direct `mmap` each, which is what makes that band compressible
at all (`src/core/mid_alloc.h`). 18 classes spaced 1.25x apart, each owning
chunks of at least 2 MiB sliced into equal blocks tracked by the same `Span`
bitmap the slab uses. A freed block is reused by its own class, so the arena
only ever sees whole uniform chunks — which is the point, because individual
mid-size blocks in the arena are exactly what denies a later large contiguous
request (see the section above).

| | bins off | bins on |
|---|---|---|
| 320 KB block in the arena | no | **yes** |
| cold 320 KB block after a cold wait | 100% resident | **0% resident** |
| 320 KB malloc/free cycle | 9,641 ns | **34.7 ns** (glibc: 19.6) |
| mi-stress, large-only | 7.18 s | **2.85 s** |
| mi-stress, full mode | 16.19 s | **10.32 s** |
| 64-thread mid-size churn (bigaba) | 5.59 s | 8.72 s |

Suite 65/65 with bins off and with `SMASH_MID_BINS=1` forced across every test.

**The in-tree benchmarks cannot see this feature, which is a gap in them, not
a null result about it.** Measured with bins off and on, interleaved, on a
freshly built library: `bench_rss` 71–72% either way (it allocates 1 MiB
blocks, already above the band), `bench_sqlite --quick` identical in both modes
(peak 199.3 vs 199.9 MiB large-only, reduction 27.7 vs 27.7, ops within noise),
and `bench_json`, `bench_kv_store`, `bench_coldhotmix`, `bench_model_serving`
all unchanged. None of them allocates a meaningful share of its footprint in
(16 KiB, 1 MiB). That makes them a decent no-regression check and no kind of
validation — the evidence for the feature is the targeted measurements above
plus mimalloc's `test-stress`, which is third-party and does span the band:
7.2 s → 2.8 s in large-only mode and 16.7 s → 10.4 s in full mode, four runs
interleaved.

**Contention is the thing to watch, and two fixes were needed to get here.**
The first version kept one lock per class and built new chunks while holding
it — setting a page state and a page-map entry for every page of a 2 MiB chunk
inside the critical section. At 64 threads that made the binned path **5.6x
slower** than the direct-mmap path it replaces (29.3 s against 5.3 s) while
still being 6x faster at 4 threads. Sharding the classes by arena and building
the chunk outside the lock took 64 threads to 8.7 s. The remaining gap is real:
this is an opt-in feature that wins 2.5-5x on ordinary workloads and still
loses ~1.6x on a benchmark where every allocation is mid-size across 64
threads.

**Empty chunks return to the arena only under pressure**, through the same
`VmRegion` exhausted hook the slab uses — handing them back on every free is
the trap that cost ~70% on mi-stress. Verified: 3 GiB of 512 KB blocks, all
freed, then a 1 GiB allocation lands in the arena. Without the hook wired the
chunks would hold the arena forever.

**Not covered in this first version:** aligned allocations (they keep
LargeAlloc's path), `malloc_numa` (unaffected), and sizes between 1 MiB and a
raised `SMASH_LARGE_ALLOC_VM_THRESHOLD` (no class covers them, so they take the
direct mapping as before).
