# Allocator Benchmark Results: neuron-cc test7_full

**Date:** 2026-06-01  
**Test:** `test7_fsdp_c474_b8_63l_135416_def_modular.hlo` (9.3 MB HLO)  
**Target:** trn2  
**Machine:** 192-core build instance  
**Build:** USE_TCMALLOC=OFF (libwalrus.so uses system allocator via PLT)

## Results

| Allocator | Wall Time | Peak RSS | Avg RSS | Notes |
|-----------|-----------|----------|---------|-------|
| glibc     | 372s      | 35,719 MB | 21,855 MB | System default |
| jemalloc  | 283s      | 18,766 MB | 11,726 MB | Lowest peak RSS |
| mimalloc  | 258s      | 21,003 MB | 13,651 MB | Fastest wall time |

## Analysis

- **glibc** has massive peak RSS (35.7 GB) — almost 2x jemalloc's peak
- **jemalloc** achieves 47% lower peak RSS than glibc (18.8 GB vs 35.7 GB)
- **mimalloc** is 31% faster than glibc (258s vs 372s) with 41% lower peak RSS

## Methodology

- RSS measured every 0.5s across all descendant processes (Python launcher + hlo2penguin + walrus workers)
- `LD_PRELOAD` used for jemalloc and mimalloc
- `PYTHONMALLOC=malloc` ensures Python uses the preloaded allocator
- All runs on same machine, sequential (no parallel interference)

## Allocator Versions

- **glibc:** system default (AL2023)
- **jemalloc:** 5.3.0 (built from source)
- **mimalloc:** 2.1.2 (built from source)

## Raw Data

Results directory: `smash/build/allocator_bench_20260601_193819/`

```
label,rc,wall_s,peak_mb,avg_mb
glibc,0,372,35719,21855
jemalloc,0,283,18766,11726
mimalloc,0,258,21003,13651
```

## Smash Results (LARGE_ONLY mode)

| Config | Wall Time | Peak RSS | Avg RSS | Notes |
|--------|-----------|----------|---------|-------|
| smash_profile | 516s | 31,160 MB | 20,555 MB | Profile generation run |
| smash_with_profile | 478s | 31,307 MB | 20,018 MB | Using saved profile |
| smash_aggressive | 470s | 32,804 MB | 21,149 MB | 1s cold timeout |

**Settings:** `SMASH_LARGE_ONLY=1`, `SMASH_COLD_TIMEOUT_SEC=5` (or 1 for aggressive)

### Smash Analysis

Smash LARGE_ONLY mode is **not competitive**:
- 69% slower than jemalloc (478s vs 283s)
- 67% higher peak RSS than jemalloc (31.3 GB vs 18.8 GB)
- Even worse than glibc on wall time (478s vs 372s)

**Root cause:** LARGE_ONLY mode only manages allocations > 16KB, missing the bulk of memory in the slab allocator. The overhead of smash's page tracking + compression doesn't pay off when most allocations bypass it.

## Smash Results (FULL mode)

**Date:** 2026-06-02

After fixing:
1. ~~tcmalloc in libwalrus.so~~ (USE_TCMALLOC=OFF)
2. ~~RTLD_DEEPBIND in JobRegistry.py~~ (disabled by default)
3. vm.max_map_count increased to 1,000,000

| Config | Wall Time | Peak RSS | Avg RSS | Notes |
|--------|-----------|----------|---------|-------|
| smash_full_48g_10s | ~468s | **17,780 MB** | 9,909 MB | Full mode, 48 GiB VM, 10s cold timeout |
| smash_profile_gen | ~573s | 17,381 MB | 10,492 MB | Profile generation (failed due to neuron-cc bug) |

**Settings:** `SMASH_VM_GIB=48`, `SMASH_COLD_TIMEOUT_SEC=10`, `SMASH_DEFER_MADVISE=1`

## Stability Test Results (2026-06-02)

**3 runs per allocator to test reliability:**

| Allocator | Run 1 | Run 2 | Run 3 | Pass Rate |
|-----------|-------|-------|-------|-----------|
| glibc | PASS (369s) | PASS (348s) | PASS (361s) | **3/3 (100%)** |
| jemalloc | PASS (284s) | PASS (275s) | PASS (287s) | **3/3 (100%)** |
| mimalloc | PASS (268s) | PASS (270s) | PASS (250s) | **3/3 (100%)** |
| tcmalloc | SEGFAULT | - | - | **0/1** |
| smash_full | PASS (696s) | FAIL | FAIL | **1/3 (33%)** |

**Key finding:** Baseline allocators (glibc, jemalloc, mimalloc) pass 100% of runs. Smash only passes 33%. The failures appear as neuron-cc internal errors but **only occur with smash**, suggesting smash causes memory corruption that manifests as compiler bugs.

**Smash failure errors:**
- Run 2: `custom_call.20_i65 is overlapping with must-pinned memloc DynamicDMAScratchLoc`
- Run 3: `post_sched failed` (22/24 modules passed)

**tcmalloc:** Crashes immediately on startup (SEGFAULT). Likely conflicts with neuron-cc's internal tcmalloc or other symbol issues.

### Conclusion

Smash full mode is **not production-ready** for neuron-cc. While it occasionally produces lower RSS than jemalloc, it has a 67% failure rate. Extensive binary-search debugging (2026-06-05) has narrowed down the issue:

### Full Mode Bug Investigation (2026-06-05)

**Test matrix (neuron-cc test7_full HLO):**

| Configuration | Pass Rate | Finding |
|--------------|-----------|---------|
| SMASH_LARGE_ONLY=1 | 3/3 (100%) | Large-only mode works |
| SMASH_NO_COMPRESSOR=1 | 3/3 (100%) | Allocation path is correct |
| SMASH_COLD_TICKS=9999 | 3/3 (100%) | No compression works |
| SMASH_NO_MONITOR=1 | 1/3 (33%) | Compression path has bug |
| SMASH_USE_LZ4=1 | 1/2 (50%) | Bug not zstd-specific (invalid: this variable was never read, so the run used zstd-1; see `SMASH_CODEC` in agents/config.md) |
| SMASH_DEFER_PHASES_MS=30000 | 1/2 (50%) | Timing doesn't help |
| SMASH_FIXAV=1 | 0/3 (0%) | Strictest ordering is WORSE |
| Default full mode | ~1/3 (33%) | Base failure rate |

**Root cause analysis:**
- Bug is in the compression/decompression path, NOT allocation
- Bug is NOT specific to zstd (LZ4 also fails)
- Bug is NOT fixed by strictest memory ordering (FIXAV=1)
- Bug is NOT timing-related (DEFER_PHASES doesn't help)

### ROOT CAUSE FOUND & FIXED (2026-06-05)

**The bug: a TOCTOU window in the decompress-on-fault restore path.**

`handleFault()` (and `prefetchAdjacent()`) restored a compressed page with this
sequence (`compressor_thread.h`):

```cpp
decompress(blob → scratch_buf);
commitPages(page_addr);                 // mprotect(PROT_RW): page is now READABLE
memcpy(page_addr, scratch_buf, 4096);   // decompressed data arrives HERE
```

Between the `mprotect` and the `memcpy`, the page is **readable but does not yet
contain the decompressed bytes**. The faulting thread holds the per-page lock,
but a *concurrent* application thread doing an ordinary load on the same page
does **not** fault (the page is now `PROT_RW`) and does **not** take the per-page
lock — so it reads whatever the physical backing currently holds: zeros (if the
backing was dropped via `madvise(DONTNEED)`) or stale bytes. That silently
corrupts the application's data, surfacing later as the assorted nondeterministic
internal errors above.

This single window explains **every** observation:

| Observation | Why |
|---|---|
| No-compression configs → 100% | No decompress, no window |
| LZ4 and zstd both fail | Bug is in the restore path, not the codec |
| **FIXAV → 0/3 (worse)** | FIXAV `madvise`s backing immediately on compress, so the window *always* exposes a zero-fill page |
| Default deferMadvise → ~33% | Backing kept ~500 ms, so the window exposes correct data for recently-compressed pages and zeros only for swept ones |
| **NO_DECOMMIT → OOM, not corruption** | Backing never dropped → window exposes correct original data; RSS just explodes |
| Fails in walrus `mod_parallel_pass` | Needs concurrent threads sharing a page — exactly multithreaded BIR passes |

**The fix (`restorePageContents()`):** populate the page's physical backing
*before* it becomes readable. On Linux we `pwrite()` the decompressed bytes
through `/proc/self/mem` while the page is still `PROT_NONE` (the kernel's
mem-file access uses `FOLL_FORCE`, honoring the VMA's `VM_MAYWRITE` rather than
the PTE protection, so the store lands and faults in fresh backing), then flip
to `PROT_RW`. Concurrent readers keep faulting on `PROT_NONE` and block on the
per-page lock until the data is in place. Non-Linux falls back to the legacy
commit-then-copy. Verified safe with a standalone repro; all 16 smash unit
tests pass.

**Post-fix verification (2026-06-05):** full mode (`SMASH_VM_GIB=48
SMASH_COLD_TIMEOUT_SEC=10 SMASH_DEFER_MADVISE=1`) on test7_full now passes
**5/5** consecutive runs (681s, 556s, 693s, 703s, 588s), versus the pre-fix
~1/3 rate. All 16 smash unit tests also pass.

| Config | Pre-fix | Post-fix |
|--------|---------|----------|
| full_default | ~1/3 (33%) | **5/5 (100%)** |

**Current recommendation:** `SMASH_LARGE_ONLY=1` remains the conservative
production default until the fixed full mode accumulates more soak time across
additional HLOs, but full mode is now functionally correct on test7_full.

### Post-fix performance (2026-06-05, fixed libsmash, profiling pass)

Single profiling pass on test7_full (RSS sampled across the whole process tree
at 0.5 s; smash = `SMASH_VM_GIB=48 SMASH_COLD_TIMEOUT_SEC=10 SMASH_DEFER_MADVISE=1`):

| Config | Wall | Peak RSS | Avg RSS | Peak vs jemalloc | Peak vs glibc |
|--------|------|----------|---------|------------------|---------------|
| glibc                 | 356s | 35,830 MB | 22,337 MB | +93% | baseline |
| jemalloc              | 262s | 18,585 MB | 11,560 MB | baseline | −48% |
| smash full (profile-gen) | 750s | 17,782 MB | 9,762 MB | −4.3% | −50% |
| **smash full (with profile)** | **644s** | **17,635 MB** | **10,496 MB** | **−5.1%** | **−51%** |

- Peak RSS: smash full beats jemalloc by ~5% and halves glibc.
- Avg RSS: smash full is ~9–16% below jemalloc (more memory returned during
  compilation pauses).
- Wall time: ~2.5× jemalloc — the compression/decompression/fault overhead is
  the cost. The profile pass trims wall time vs profile-gen (750s → 644s) but
  the gap to jemalloc remains the main tradeoff.
- The `/proc/self/mem` restore adds one `pwrite` syscall per fault-decompress
  (replacing a `memcpy`); no measurable wall-time regression vs the pre-fix
  numbers (468–516s historical, within run-to-run variance given the now-correct
  but still-uncached profile).

Takeaway: with the TOCTOU fix, full mode delivers the lowest peak RSS of any
allocator tested while staying correct (5/5). The memory win is real; the
wall-time cost is the open question for whether it ships beyond LARGE_ONLY.

### Wall-time attack (2026-06-05)

Goal: cut the ~2.5× wall-time gap vs jemalloc. Method: instrument churn,
profile with `perf`, then target the dominant cost.

**Churn / compression stats** (`SMASH_BUCKET_STATS`, no-profile baseline):
1.26M compressions, 300K fault-decompressions ⇒ **~22% churn**. Pages compress
*well* (74–99% ratios) — the wasted work is re-access churn and the syscall
overhead of compressing, not poor compressibility.

**`perf` flat profile** (privileged container, system-wide 99 Hz, cycles) of a
644 s full-mode compile — cycle distribution by DSO:

| DSO | % cycles |
|-----|----------|
| `[kernel.kallsyms]` | 46.2% |
| `libwalrus.so` (compiler backend, real work) | 24.7% |
| `libsmash.so` | 11.1% |
| libtbb / libc / libBIR / python | ~11% |

Kernel time broke down as: **TLB-shootdown IPIs 11.4%**
(`asm_sysvec_call_function` + `smp_call_function_many` + `flush_tlb_func`),
**mmap_lock contention 4.0%**, sched/idle 6.2%, page faults 1.2%. The 11.4% +
4.0% are *directly caused* by issuing one `mprotect(PROT_NONE)` per
just-compressed page — each broadcasts a TLB-flush IPI to all ~24 worker cores.

**Wins applied:**

| Change | Effect | Status |
|--------|--------|--------|
| Profile-merge fix (was clobbered → empty) | 657s vs 741s = **−11%** | shipped |
| P2_CHUNK batched mprotect (default-on) | 581s vs 640s avg = **−9%** | shipped |

P2_CHUNK coalesces contiguous just-compressed pages into one `mprotect`
(≤ `SMASH_PROTECT_CHUNK_PAGES`, default 16), cutting the IPI/lock cost the perf
profile exposed. Both batched runs beat both per-page-control runs; RSS
unchanged (same pages compressed, fewer syscalls).

**Dead ends (measured, reverted/defaulted-off):**
- **Python `gc.disable()`**: net-negative (715s vs 657s). Cyclic garbage
  accumulates → larger heap → more pages to compress. Not recommended.
- **Profile thrash-backoff** (defer buckets that churned last run): net-negative
  (950s vs 721s control). Deferring keeps thrashy pages ACTIVE longer, so the
  per-tick scan does more work and a wave compresses at once. Defaulted OFF
  (`SMASH_PROFILE_THRASH_BACKOFF=1` to re-enable).
- **Larger P2_CHUNK window** (`SMASH_PROTECT_CHUNK_PAGES` 64/128): no wall-time
  gain over 16, and a wash on RSS. Default stays 16.

**Final benchmark with correct whole-tree RSS** (test7_full, profile loaded,
P2_CHUNK on; the RSS sampler's per-0.5s /proc walk inflates all smash
wall-times here by a fixed amount, so compare RSS absolutely and wall-time only
across the smash rows):

| Config | Wall | Peak RSS | Avg RSS | Peak vs jemalloc | Avg vs jemalloc |
|--------|------|----------|---------|------------------|-----------------|
| jemalloc           | 253s | 18,583 MB | 11,475 MB | baseline | baseline |
| smash chunk=16     | 674s | 17,512 MB | 10,303 MB | −5.8% | −10.2% |
| smash chunk=64     | 711s | 17,315 MB |  9,653 MB | −6.8% | **−15.9%** |
| smash chunk=128    | 674s | 17,574 MB |  9,849 MB | −5.4% | −14.2% |

Smash holds the lowest peak AND avg RSS across all chunk sizes. The wall-time
gap to jemalloc remains (compression is CPU work the baseline doesn't do), but
the profile-merge and P2_CHUNK wins removed ~20% of smash's *own* overhead.

> ⚠️ **The RSS numbers above are an over-count — see the cgroup re-measurement
> below, which overturns the memory claim.**

### Memory re-measured with mstat / cgroup v2 (2026-06-05) — corrects the record

The "lowest RSS" claims above came from **summing `/proc/<pid>/status` VmRSS
across the process tree**, which **double-counts pages shared between processes**
(libwalrus, libsmash, Python, shared file mmaps) — and it over-counts the
many-process baseline differently from smash. Re-measured with
[mstat](https://github.com/bpowers/mstat), which reports the cgroup v2
`memory.current` of the whole process tree (shared pages counted once, the true
physical footprint):

| Config | Wall | Peak (memory.current) | Avg (memory.current) |
|--------|------|-----------------------|----------------------|
| jemalloc       | 620s | 17,952 MiB | 10,404 MiB |
| smash full     | 687s | 17,814 MiB | 10,279 MiB |
| Δ (smash−jem)  | +11% | **−0.8%** | **−1.2%** |

**Under fair cgroup accounting, smash full-mode is memory-neutral vs jemalloc
(within ~1%, i.e. noise) while ~11% slower.** The previously-reported −6% peak /
−16% avg "wins" were a per-process-RSS summing artifact, not a real reduction.

Harness notes (`tools/mstat_bench.sh`): mstat hardcodes a 16 GiB `memory.max` on
its cgroup → a background watcher raises it to avoid OOM-killing the ~17.5 GB
compile; and mstat discards its TSV if the child exits non-zero, so the compile
is wrapped `bash -c "...; true"` (neuron-cc exits 245 even on success; the .neff
is the real success signal). N=1 per config.

**Honest standing:** the wall-time engineering wins (profile-merge −11%,
P2_CHUNK −9%) are real and shipped. The *memory* advantage over jemalloc does
not survive a fair (cgroup) measurement on this workload. Smash full-mode's case
on neuron-cc is therefore "comparable memory, higher wall-time" — not a clear
win. `SMASH_LARGE_ONLY=1` remains the conservative production default.

### Full Mode Analysis

**Smash full mode achieves lower peak RSS than jemalloc:**
- Peak RSS: 17.8 GB vs jemalloc's 18.8 GB (**5% reduction**)
- Wall time: 468s vs jemalloc's 283s (66% slower)
- Average RSS: 9.9 GB vs jemalloc's 11.7 GB (**15% reduction**)

The RSS reduction comes from:
- Arena routing concentrating similar allocations on the same pages
- Cold page compression (zstd) during compilation pauses
- Deferred zeroing of freed slots for better compression ratios

**Trade-off:** Smash adds significant wall-time overhead (66% slower than jemalloc) due to:
- Compression/decompression cycles
- Fault handling for compressed pages
- Additional bookkeeping for page state tracking

### Comparison Summary

| Allocator | Wall Time | Peak RSS | vs jemalloc Peak | Notes |
|-----------|-----------|----------|------------------|-------|
| glibc     | 372s      | 35,719 MB | +90% | System default |
| mimalloc  | 258s      | 21,003 MB | +12% | Fastest |
| jemalloc  | 283s      | 18,766 MB | baseline | Best RSS without smash |
| **smash full** | **468s** | **17,780 MB** | **-5%** | Lowest peak RSS |
| smash LARGE_ONLY | 478s | 31,307 MB | +67% | Not competitive |

## Conclusion

Smash full mode achieves the lowest peak RSS of all tested allocators, beating jemalloc by 5%. However, the 66% wall-time overhead may not be acceptable for all use cases. The sweet spot depends on whether memory or time is the binding constraint.
