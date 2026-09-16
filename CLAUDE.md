# CLAUDE.md - Project Guide for Claude Code

## What is smash?

A compression-aware memory allocator that transparently compresses cold pages to reduce RSS. It interposes on malloc/free via alloc8 and uses signal-based fault handling to decompress on access.

## Build

```bash
# alloc8 is cloned automatically via FetchContent on first configure.
# To use a local checkout instead: cmake .. -DALLOC8_DIR=/path/to/alloc8
mkdir build && cd build
cmake .. && make -j$(nproc)

# With benchmarks
cmake .. -DSMASH_BUILD_BENCH=ON && make -j$(nproc)
```

## Test

```bash
cd build && ctest --output-on-failure
```

All 35 enabled tests must pass (3 more are registered `DISABLED` with the `known_issue` label — see below). CI (`.github/workflows/ci.yml`) runs the full suite on `ubuntu-latest` and `macos-latest` on every push and PR. Eighteen of the tests are end-to-end under `DYLD_INSERT_LIBRARIES` / `LD_PRELOAD`:

- `test_arena_routing` guards call-site arena routing — the invariant behind the paper's compression-homogeneity claim. Runs a program with 32 distinct noinline allocation sites under interposition and asserts, via the exported `smash_arena_for(ptr)` query, that they do **not** all collapse into one arena. This is the regression guard for the LTO/wrapper return-address bug: under dynamic interposition the return address seen inside the allocator can point into the wrapper library instead of the app, collapsing every call site into one arena (homogeneity 0%). Fast and deterministic (no compressor): full mode routes 256 B allocations through the slab, and `SMASH_CPU_ARENA=0` + default thread-hash-off make the arena a pure function of the return address, so a buggy capture yields exactly 1 distinct arena. Verified as a real discriminator (injecting a constant RA makes it fail with `distinct_arenas=1`). Must run under interposition — a direct-linked call would never exercise the wrapper path where the bug lives.

- `test_malloc_numa_{full,large_only}` and `test_numa_arena` guard `malloc_numa(node, size)` (`include/smash/smash.h`). It serves page-granular blocks from per-node arenas inside the VmRegion (see Key Conventions).
  - **`malloc_numa_test.cpp`** runs under preload in both modes. For every online node it asserts that `get_mempolicy(MPOL_F_ADDR)` reports exactly `MPOL_BIND | MPOL_F_STATIC_NODES` on `{node}`, and that `move_pages` reports every written page on that node. The writer is pinned to off-node CPUs first, so first-touch locality can't put the pages there by accident. It then leaves a 16 MiB block per node cold (`SMASH_COLD_TIMEOUT_SEC=1`) and waits for its pages to leave RSS (`mincore`). It reads the blocks back from off-node CPUs and checks they are byte-exact, with every page on the node and the policy intact. It also checks heap-block semantics, `EINVAL` for bad nodes, freed blocks coming back at the same address, and that ordinary `malloc` memory carries no policy.
  - **`test_numa_arena.cpp`** is direct-linked and deterministic. It covers grid-aligned chunks, the gap skipped below a chunk and a bad node's chunk becoming unbound shared pages, free-list reuse within a node, and `remapPages()` binding a replaced node page again, including a remap that straddles a shared page and a node page.
  - **Verified as a real discriminator** on a 4-node host:
    - not installing the remap hook fails the arena test's three remap checks;
    - routing freed node pages to the shared free lists fails the arena test's shared-list and reuse checks on all 4 nodes. End to end, 0 of 16 frees come back at the same address, and the binding leaks into a later `malloc(256)`;
    - never marking node blocks `ACTIVE` fails the compression check: 0 of 4096 pages leave RSS in 30 s on every node.
  - Skips on `ENOSYS`/`EPERM` from `mbind`. Single-node CI runners cover node 0 only, with no off-node pin.

- `test_numa_compressor_{full,large_only}` guard the per-node compressor groups (`numa_compressor_test.cpp`, both modes, `SMASH_COLD_TIMEOUT_SEC=1`). It leaves a 16 MiB block cold on every node, written from off-node CPUs. Once most of each block has left RSS, it uses the diagnostic exports `smash_numa_compressor_info` and `smash_compressed_blob_for` to check, per node, that:
  - the group's threads have CPU affinity inside the node, and none of their compressions ran on another CPU;
  - the group counted at least as many compressions as the block has evicted pages;
  - worker state, page and compress buffers, and the zstd context are bound (`MPOL_BIND`) to the node, and on it wherever already touched;
  - every compressed page's blob is on the node, and reading the block back from off-node CPUs is byte-exact with every page on the node.

  Nothing after the cold wait writes to malloc'd memory, because in full mode a write into a heap page that went cold can be lost (the `test_stdout_buffer_integrity_full` defect). An earlier version that read the node CPU lists after the wait saw an empty list in full mode.
  - **Ordinary `malloc` memory** is covered by the same test: a block per node, written from that node's own CPUs (with no binding, first touch is what places it), must end up with its blobs on that node. Measured 4096/4096 per node in both modes.
  - **Verified as a real discriminator** on a 4-node host:
    - disabling `routeByNode` sends every blob of the ordinary blocks to the shared store's node: 0 of 4096 on-node for three of the four blocks, and a pass only for the block that happens to share that node;
    - capping the node-query batch per range instead of rolling it does the same — the bug this actually caught;
    - not pinning the group threads fails the affinity check on all 4 nodes, and the off-node-compression check on 3 of them;
    - storing node blobs in the shared store leaves most blobs off the node on every node, from 2234 of 2901 to 3221 of 4096;
    - a node allocator that doesn't bind its memory fails the working-memory binding check;
    - letting the default workers compress node chunks failed the compression-count check in 3 of 3 runs, but on only one or two nodes per run, so that check is probabilistic;
    - pointing `storeOfBlob` at the shared store is **not** detectable, and should not be: `CompressStore::release` forwards by region owner, so the stale lookup costs one extra call and corrupts nothing. Before that routing existed this mutation passed the test while silently letting two stores mutate one region's free list under different locks.
  - Skips on single-node hosts (CI runners), when `mbind` is refused, and with `SMASH_NUMA_COMPRESSOR=0`.

- `test_external_mapping` (uses a live compressor) exercises the `SMASH_TRACK_EXTERNAL=1` registration path. The ctest invocation sets the env var; without it the test would silently no-op (registration path gated).
- `test_malloc_compression` and `test_large_only_compression` are the **same executable** (`mode_compression_test.cpp`) run twice, differing only in `SMASH_LARGE_ONLY` (`0` = full, `1` = large-only — the production-supported config). The mode is read from the env at runtime, never a compile flag, so the two modes execute byte-identical assertions and cannot drift apart in coverage. The body allocates large chunks (≥ 1 MiB, so they clear `kLargeAllocVmThreshold` and enter the compressible VmRegion in **both** modes) interleaved with small chunks (≤ 16 KiB — smash slab in full mode, system passthrough in large-only). It sleeps past `SMASH_COLD_TIMEOUT_SEC`, sends `SIGUSR2`, parses `compressed=N` from the stats line, asserts `N > 0`, then verifies a byte-exact read-back of **both** classes — proving full-mode slab integrity / large-only passthrough is never corrupted, and that fault-decompress restores the compressed large chunks. (Registered via the `add_mode_compression_test(name, large_only)` helper in `tests/CMakeLists.txt`.)
- `test_compression_ratio` (in-process, direct-linked, no compressor thread) compresses a realistically-compressible page with LZ4 and zstd and asserts the achieved ratio against the paper's RQ2 figures (evaluation.tex: LZ4 4.7–12.3×, zstd-1 8.8–20.4×). Two-tier: a hard floor that fails the test (2.5× LZ4 / 4× zstd — catches "stored uncompressed" regressions) plus a WARN if it falls short of the paper's best-case number. Also asserts byte-exact roundtrip. Unlike `test_malloc_compression` (which only checks compression *happened*), this pins the codec *ratio*.

### aarch64 / page-geometry guards

Five tests cover the class of bug that makes an application under libsmash throw `std::bad_alloc` on aarch64. There is exactly one way smash produces that exception — the operator-new wrappers in `src/linux_syscall_wrappers.cpp` do `void* p = xxmalloc(sz); if (!p) throw std::bad_alloc();` — so the exception always means **smash's malloc returned nullptr**. These tests attack the arch-dependent geometry that makes it do so.

The root risk is that `kPageSize` is a **compile-time** constant (`ALLOC8_PAGE_SIZE`), and alloc8 hard-codes `4096` for *every* Linux target including aarch64 — while aarch64 kernels commonly use 16 KiB or 64 KiB base pages (RHEL/Rocky/Oracle aarch64 ship 64 KiB). Every `mprotect`/`madvise` smash issues is then sub-page-aligned and rejected with `EINVAL`, and `VmRegion::ensureCommitted()` ignores `commitPages()`' return value and advances its watermark anyway, so callers get PROT_NONE memory back.

- `test_page_size` (unit, no preload) — the constants. Asserts `1 << kPageShift == kPageSize` (config.h derives the shift with a ternary that only knows 4 KiB and 16 KiB, so a 64 KiB build silently gets shift 12), `kPageSize == sysconf(_SC_PAGESIZE)`, that this machine's addresses fit the hard-coded `kAddressBits = 48` `PageMap` L1 table (which is indexed with no bounds check), and that every size class still has a valid span geometry — a class whose span holds zero objects makes `Slab::allocate()` return nullptr forever. Verified as a real discriminator: rebuilding it against a patched `ALLOC8_PAGE_SIZE` of 65536 fails on both the shift and the kernel mismatch, 16384 fails on the mismatch alone.
- `test_vm_granularity` (unit, no preload) — the same mismatch at the syscall level. Issues the exact reserve → commit → protect → decommit sequence `VmRegion` and the compressor depend on, at `kPageSize` offsets 0/1/2/3/7/15/31/63 into a reservation. Verified as a real discriminator: built with `ALLOC8_PAGE_SIZE=1024` on this 4 KiB-page host (the same smaller-than-kernel relationship as a 4 KiB build on a 64 KiB kernel) every non-zero offset fails with `errno=22 EINVAL`.
- `test_alloc_contract_{full,large_only}` — the direct regression test for the reported symptom. One executable (`alloc_contract_test.cpp`) run twice, differing only in `SMASH_LARGE_ONLY`, same one-body-two-modes contract as `mode_compression_test`. Sweeps 163 sizes across every routing boundary (size-class edges ±1, `kMaxSmallSize`, `kLargeAllocVmThreshold`, page multiples for 4/16/64 KiB pages) through `malloc`, `operator new`/`new[]`/nothrow/`align_val_t`, `posix_memalign`, `aligned_alloc`, `calloc`, `realloc` grow/shrink chains, and sustained mixed churn — asserting none ever yields nullptr or throws, and that every block is correctly aligned, fully writable, and byte-stable.
- `test_aligned_alloc_footprint_{full,large_only}` — footprint, not correctness. `SmashHeap::memalign()` routes **every** allocation with alignment > `kMinAlignment` (16 B) to `LargeAlloc`, which rounds up to a whole page, so an ordinary `alignas(64)` 64-byte object costs a full kernel page. Two-tier like `test_compression_ratio`: a hard 384 MiB RSS budget that 4 KiB (47 MiB measured) and 16 KiB pages clear with ~2× headroom but a 64 KiB-page kernel exceeds ~2×, plus a WARN documenting the amplification (65× in full mode here; large-only measures 2.1×, since it passes small aligned requests to the system allocator).
- `test_vm_exhaustion` — `SMASH_VM_GIB=1` caps the contiguous arena at 1 GiB and the test allocates 1.5 GiB of live 4 MiB chunks, forcing `LargeAlloc`'s direct-mmap fallback; config.h names this failure by name ("exhaust the region and trigger malloc=NULL → std::bad_alloc"). Also covers the mixed arena/fallback free routing (issue #84's class) by freeing everything and reallocating. Self-evidencing: phase 1 sorts the chunk addresses and requires at least one discontinuity, since arena chunks are bump-allocated adjacent — with `SMASH_VM_GIB=16` it reports 0 discontinuities and correctly fails rather than passing on coverage it never exercised.

Only the first and last page of each chunk is touched in `test_vm_exhaustion`, so reserving 1.5 GiB costs a few MiB of RSS.

A sixth test, `test_esr_write_bit`, covers a different aarch64 gap found while reading the same code — not a `bad_alloc` route. `FaultHandler::signalHandler` classifies each fault as a read or a write for the compressor's soft-dirty ROI re-dirty signal, but the decode was x86-only (`gregs[REG_ERR]` bit 1), so on aarch64 `faultWasWrite()` stayed `-1` for every fault and the sole reader (`recordRedirtyForPage`) counted every decompress as a write. Safe — an over-reported re-dirty defers compression — but blind: a read fault would have kept its blob valid and should not have been scored as churn. `aarch64EsrWriteBit()` decodes ESR_EL1's WnR bit (bit 6) out of the `esr_context` record the kernel puts in `sigcontext::__reserved`, walking the `_aarch64_ctx` chain by hand rather than including `<asm/sigcontext.h>` (which conflicts with the signal includes, per the note on `faultWasWrite`). It reports unknown rather than guessing when WnR carries no information — a cache-maintenance fault (CM bit set), or an exception class that is not a data or instruction abort. The decoder is a pure function over bytes, so the test feeds it synthetic signal frames and runs on any host; there is no aarch64 CI runner, and the aarch64 branch of the handler itself is therefore unbuilt by CI.

### BootstrapAlloc growth (self-deadlock guard)

`test_bootstrap`'s `testGrowPastFirstRegion` allocates 1 MiB at a time until it passes the 64 MiB initial region, forcing `addRegion()`, and touches both ends of every chunk so the new region is proven writable. It is single-threaded and takes milliseconds.

It guards the deadlock described in Key Conventions: `expandAndAllocate()` holds `expand_lock_` and calls `tryAllocFrom()`, which used to call `ensureCommitted()`, which took the same non-recursive spinlock to `mprotect` the next pages. The first allocation that had to grow the arena spun forever holding the lock, and every other thread piled up behind it — 25 threads in `R` state at ~730% CPU, 23 of them innermost in `spinRelax`.

Found with mimalloc's `test-stress` built with `USE_STD_MALLOC=1` (the only test in that suite that exercises smash rather than mimalloc's own `mi_*` API), run under a plain `LD_PRELOAD` with no smash environment — i.e. the default, experimental full mode. The trigger is total allocation, not concurrency: a `ThreadCache` is 92 KiB and goes through `allocate()`, so ~728 of them exhaust the initial region. 16 threads × 20 iterations passed; 4 × 250, 8 × 120, 16 × 60 and 32 × 50 all hung, and all four complete in 1–16 s after the fix.

**Verified as a real discriminator:** reverted to the pre-fix `bootstrap_alloc.h`, this test hangs (killed at 45 s); restored, it passes. The RSS-budget tests (`test_aligned_alloc_footprint_{full,large_only}`, `test_vm_exhaustion`, `test_vma_pressure`) all still pass, which is the check that mapping regions read/write up front did not cost footprint — a budget check, not a direct before/after RSS measurement.

### Hugepages, VMA pressure, and buffered-stdio integrity

Follow-ups for the case where the application allocates hugepages with `mmap`. All Linux-only; they skip cleanly elsewhere.

- `test_hugepage_mmap` — the mmap interposer's filter in `src/linux_syscall_wrappers.cpp` is `anon && writable` with **no `MAP_HUGETLB` carve-out**, and `registerLinuxExternalRange()` walks the mapping in 4 KiB strides. So under `SMASH_TRACK_EXTERNAL=1` every 4 KiB slice of an application hugepage mapping is registered as an external page and marked ACTIVE. hugetlb VMAs require huge-page-aligned addresses and lengths, so *every* page operation smash then performs on them is rejected — verified directly on a host with hugepages reserved: `mprotect` → EINVAL, `madvise(MADV_DONTNEED)` → EINVAL, `mmap(MAP_FIXED|MAP_ANON)` → ENOMEM. The test asserts what must hold today: the app's `mmap(MAP_HUGETLB)` still succeeds, its data stays byte-exact (smash's restore path is `remapPages()`, i.e. `MAP_FIXED|MAP_ANON`, which would shred a hugetlb VMA if the kernel allowed it), and allocation keeps working while the mapping is registered. Skips when the host has no hugepages reserved (CI runners have none; `sysctl -w vm.nr_hugepages=N` to exercise it).
- `test_hugepage_no_mprotect_storm` (**DISABLED, `known_issue`**) — same binary with `SMASH_TEST_STRICT_HUGEPAGE=1`, which promotes the wasted work from a WARN to a failure. Nothing records that a page is unmanageable, so the compressor retries every tick forever: a 16 MiB hugetlb mapping produces **8192 failing `mprotect` calls (errno 22) in six seconds** and keeps going, while the slices permanently occupy VmRegion's `kTrackMaxPages` = 128 Ki-page external budget (512 MiB at 4 KiB pages) despite being permanently uncompressible. Fix is a `MAP_HUGETLB` skip in the interposer's filter; enable the test then.
- `test_vma_pressure` — crossing `vm.max_map_count` makes `mmap` return ENOMEM, which makes `malloc` return nullptr, which makes `operator new` throw `std::bad_alloc`; `compressor_thread.h` also records the resulting restore livelock ("VMAs pinned at the 65531 cap"). smash mitigates this with soft-dirty tracking (default ON), chunked Phase-3 `mprotect`, and a mid-loop VMA budget in `reclaimShadowRange()`, but nothing pinned them in place. Drives a 576 MiB compress-and-refault workload including a THP-hinted region (a partial `mprotect` inside a transparent hugepage splits it, up to 512 VMAs per 2 MiB) and asserts the VMA count stays far below the cap. Measured peak on x86-64 with defaults: 218 VMAs, against a budget of 8192 and a cap of 1048576 — a naive one-VMA-per-page implementation would produce ~131072.
- `test_stdout_buffer_integrity_large_only` / `test_stdout_buffer_integrity_full` (**the `_full` one is DISABLED, `known_issue`**) — glibc allocates stdout's 4 KiB buffer with `malloc`, so in full mode it is an ordinary slab object. An application writing in bursts separated by quiet stretches lets that page go cold; the compressor compresses it while it still holds unflushed bytes, and **everything written afterwards is flushed as NUL at exit**. Measured: 228 of 513 bytes zeroed and 4 of 9 marker lines lost, *identically on every run*. Byte-perfect with no libsmash, in large-only mode, and in full mode with compression deferred; unaffected by `SMASH_SOFTDIRTY`, `SMASH_DEFER_MADVISE`, `SMASH_NO_DECOMMIT`, `SMASH_MADV_FREE` and `SMASH_NO_MONITOR`. This is silent application-data corruption, and a plausible route to a `std::bad_alloc` report — a zeroed length or capacity field turns the next allocation into an absurd request. It re-execs itself rather than plain-forking (fork clones only the calling thread, so a forked child has no compressor thread and never compresses anything) and inspects the child's output file after it exits, because the damage only appears at the exit-time flush.

Because of that last defect, every end-to-end test in this group calls `setvbuf(stdout, nullptr, _IONBF, 0)` on entry — otherwise full mode can zero the tail of a test's own buffered output and eat its verdict lines while it still exits 0.

After ctest, CI also runs `bench/run_quick_ci.py` which drives `bench_rss` (in-process: 64 MiB compressible alloc → ≥30 % peak-RSS reduction at t=10 s) and `bench_sqlite --quick` under the preloaded libsmash (≥5 % cooling-phase RSS reduction). Local baselines are ~46 % and ~13 % respectively, so the thresholds are well below noise; a real regression in the compressor or the malloc-interposed path will trip them. Configured with `-DSMASH_BUILD_BENCH=ON -DSMASH_BUILD_BENCH_DEPS=OFF -DSMASH_BUILD_BENCH_ALLOCATORS=OFF` so the CI build skips Redis/memcached/RocksDB/tcmalloc/jemalloc/hoard/mesh/etc. — only the smash-internal benches are needed.

### Verifying paper claims end-to-end (`bench/verify_paper_claims.py`)

`run_quick_ci.py` is a regression tripwire; `verify_paper_claims.py` is the explicit "do we still match the paper?" harness. It computes serve/cool-phase RSS reduction (`1 − min_rss/peak_rss`) and checks each workload against the paper's per-app figure with a two-tier scheme: a conservative hard floor fails the run; a shortfall vs the published number only WARNs (a shortfall usually means the bench profile undershoots the full paper workload — the WARN points to `run_paper_experiments.py` for the full-dataset re-check — not a hardware gap). Prints `✓ BEATS paper` when measured ≥ the claim.

- **In-process apps** (`rss`, `sqlite`, `rocksdb`) run their bench binary directly, in **both** full and large-only mode. Large-only on these is a *no-regression* check, not a paper-reduction check: their allocations are < 1 MiB so large-only passes them through to the system allocator (the large-only compression mechanism is proven by `test_large_only_compression`). This is the default app set.
- **External-service apps** (`memcached`, `redis`, `redis_ext`, `redis_patched`) are full-mode only and slower; they're driven through `run_paper_experiments.py` (which owns the server + protocol-client lifecycle) and read back from the JSON it writes. Opt in via `--apps`.

Needs `-DSMASH_BUILD_BENCH=ON`. The build dir is autodetected from `./build*` (newest libsmash with a `bench/` tree); override with `--build-dir` or `$BUILD_DIR`. Examples:
```
python3 bench/verify_paper_claims.py                      # in-process apps, both modes
python3 bench/verify_paper_claims.py --apps redis,memcached   # external services
```
Measured on the EPYC 9R14 (full datasets): memcached 86%, rocksdb 76%, sqlite 64%, redis 54%, redis-ext 64%, redis-patched 55%, redis-ext-patched 77% — five of seven beat the paper, two match.

## Project Structure

```
include/smash/          Public API headers (config.h, smash.h)
src/
  core/                 Allocator core: bootstrap_alloc, size_classes, span, page_map, slab, large_alloc, thread_cache
  vm/                   Virtual memory: platform_mem, vm_region, page_state, fault_handler
  compress/             Compression: compress_engine (LZ4/zstd/zstd+dict), compress_store, compressor_thread
  util/                 Utilities: bitops, spinlock, intrusive_list
  smash_heap.h/.cpp     Main allocator singleton + alloc8 integration
tests/                  Unit + integration tests
bench/                  Benchmarks (throughput, compression ratio, RSS, latency)
```

## Architecture

- **Header-only internals**: Most code lives in headers under `src/`. Only `smash_heap.cpp` is compiled.
- **BootstrapAlloc**: All internal metadata allocated from a bump allocator (never calls malloc). Critical for avoiding reentrancy in the fault handler.
- **Arena-based allocation**: 4 arenas (`kNumArenas`). `callsiteArena()` hashes the application's allocation call site to route allocations from the same call site to the same arena. The return address comes from `alloc8_caller_ra` — an initial-exec TLS hint stored by alloc8's wrapper entries (`replace_malloc`, `custommalloc`, `operator new`, ...), read via `appCallerRA()`. Never use `__builtin_return_address(N)` inside smash for routing: the frame count between the app and smash depends on inlining/LTO/tail-call decisions and silently collapses all call sites into one (debug with `SMASH_ARENA_TRACE=1`). Pages within an arena contain similar data → better compression ratios. `Slab slabs_[kNumArenas * kNumClasses]` flat 2D array.
- **Zero-on-free (deferred)**: All zeroing is deferred to the compressor thread's `zeroFreeSlots()` — no zeroing occurs in the `free()` critical path. Uses non-temporal stores (`ntZeroMemory`) to avoid cache pollution. This makes partial pages compress well (zero runs → high compression ratios).
- **PageState machine**: EMPTY → ACTIVE → ACTIVE_MONITORING → COMPRESSING → COMPRESSED → ACTIVE. CAS transitions ensure safe coordination between compressor thread and fault handler.
- **CompressEngine**: Supports LZ4, zstd, and zstd+dictionary. Algorithm packed in top 2 bits of `CompressedPageInfo::comp_size`. All zstd contexts pre-allocated via `ZSTD_customMem` routing to BootstrapAlloc.
- **Algorithm tiering (single-shot, ROI-driven)**: ROI model picks fast tier (zstd-1 by default; LZ4 if `SMASH_USE_LZ4`) or deep tier (zstd-9) per page **at initial compression time**. There is no later upgrade path; once a page is compressed, the chosen blob stays as-is until the page is decompressed by access. `AlgoProfile` carries `algo + zstd_level` so the calibrated profile reflects what's actually run; calibration in `compression_roi.h::calibrate()` benchmarks the actual fast-tier algorithm.
- **Adaptive per-bucket cost**: `SizeClassStats` tracks observed compression microseconds per tier (EMA, fixed-point). After 8 samples per (bucket, tier), the ROI model substitutes observed cost for the calibrated estimate via `selectProfile(..., observed_costs_us)`.
- **Parallel compressor**: Coordinator thread + worker threads. Chunk bitmap (`live_chunks_[]`) skips EMPTY pages. Sharded `CompressStore` (8 shards) eliminates lock contention. Per-worker compression contexts (LZ4 state, ZSTD CCtx, scratch buffers, SizeClassStats).
- **Adaptive worker count via Little's Law**: each tick the compressor sets active workers to `N = ⌈λ/μ⌉` where `λ` is pages-eligible/tick and `μ` is per-worker pages-compressed/tick (both EMA-smoothed). Workers pre-allocated up to `kMaxCompressorWorkers`; helpers lazily `pthread_create`'d on first scale-up.
- **Per-fault-slot DCtx**: Each of 32 fault slots has its own `ZSTD_DCtx*`, fixing data race between concurrent decompressions from app threads and prefetch.
- **Per-origin sliding-window stats**: `SizeClassStats` keyed by `(arena, size_class, worker)` — `sc_stats[kNumArenas * kNumClasses]` per worker, indexed via `statsIndex(arena, sc)`. Each bucket holds a 64-entry ratio window (0–255) plus per-tier compression-time EMAs. Aggregating across arenas would wash out the homogeneity arena routing produces.
- **Signal handler path**: No malloc allowed. Decompression uses pre-allocated per-slot contexts only.
- **NUMA compressor groups** (Linux, hosts with ≥2 memory nodes): every page — a `malloc_numa` node arena page or an ordinary heap page — is compressed by a group of threads dedicated to its own node, not by the default workers.
  - **Threads:** pinned to the node's CPUs; they park on a futex between phases.
  - **Memory:** worker state, scratch buffers, zstd/LZ4 contexts and stacks come from memory bound to the node (`vm::NodeBumpAlloc`, `src/vm/node_memory.h`). Blobs go to a per-node `CompressStore` whose regions are bound to the node.
  - **Scheduling (arena pages):** each tick `assignNodeChunks()` lists each node's chunks. Every group thread takes an equal address slice of each chunk and runs phases 1–4 on it, while the default workers skip those chunks (`ownedByRunningNodeGroup`).
  - **Ordinary heap pages** carry no binding, so their node has to be discovered. `page_node_` holds a per-page hint, learned free in the fault handler (`notePageNode`: a restored page lands on the faulting thread's node) and otherwise resolved in one `move_pages` batch per Phase 2 range (`flushNodeQueries`). Phase 2 hands each eligible page to its node's queue (`routeByNode` → `enqueueForNode`) instead of compressing it, and the group drains that queue in **phase 5**, on its own threads and into its own store. A wrong hint costs locality, never correctness.
    - The batch must be **rolling**: flush it when full and keep deferring. A fixed cap per range routed only the first batch and sent everything after it to the local worker and the shared store — measured as three of four blocks' blobs landing on one node.
  - **Lifetime:** groups are created once in `start()` and never freed. Never create them mid-tick: `pthread_create` is unsafe once pages may be compressed.
  - **Still shared:** the coordinator, the deferred-madvise sweep and fault-time decompression. Decompression runs on the faulting thread; the arena binding puts the restored page back on the node.

## Syscall & Buffered I/O Compatibility

See [agents/syscall-compat.md](agents/syscall-compat.md) for details on EFAULT-driven decompress-and-retry, DYLD interposition limitations, buffered-I/O carve-outs, and the full list of interposed functions per platform. It also records two things worth knowing before adding a wrapper: a wrapper exported only at the base glibc version never runs when glibc versions that symbol (the caller binds the versioned one), and a measured audit of which syscalls still return `EFAULT` when their buffer is on a compressed page.

## External-Mapping Tracking (`SMASH_TRACK_EXTERNAL=1`)

See [agents/external-tracking.md](agents/external-tracking.md) for details on the VmRegion tracking hybrid, filter rules, and opt-in rationale.

## Production Configuration

For applications with concurrent threads and significant slab/small-object traffic (e.g., neuron-cc, walrus C++ backend), the production-supported configuration is:

```
LD_PRELOAD=/path/to/libsmash.so PYTHONMALLOC=malloc SMASH_LARGE_ONLY=1
```

Full mode (without `SMASH_LARGE_ONLY=1`) is **experimental**.

```
SMASH_DEFER_MADVISE=1   # default ON; do not disable in production
```

Defers `madvise(MADV_DONTNEED)` to a per-tick sweeper that only runs after a page has been quiescent for `SMASH_DEFER_MADVISE_TICKS=N` ticks (default 50, ≈500 ms). Closes a separate corruption race where in-flight loads/stores from a writer's stale TLB observed a recently-DROPPED page and saw zeros.

See [agents/neuron-cc-investigation.md](agents/neuron-cc-investigation.md) for the full investigation into TOCTOU race fix, async-signal-safety bugs, DEEPBIND/islpy failures, and libwalrus tcmalloc conflicts.

## Large-Only Mode (`SMASH_LARGE_ONLY=1`)

For applications with their own small-object allocator (e.g., Python 3.13+ uses mimalloc), set `SMASH_LARGE_ONLY=1` to only manage large allocations (> `kMaxSmallSize` = 16KB):

- `malloc(size <= 16KB)` → system malloc passthrough
- `malloc(size > 16KB)` → Smash's `LargeAlloc` → VmRegion (compressible)
- `free(ptr)` checks `page_map_`; non-Smash pointers forwarded to system free
- `getSize(ptr)` checks `page_map_` first (returns Smash size class or `span->large_size`); falls back to system `malloc_size`/`malloc_usable_size` only for non-Smash pointers

### Resolving original system malloc on macOS

On macOS, `dlsym(RTLD_NEXT, "malloc")` returns the interposed wrapper — DYLD interposition is truly global and cannot be bypassed via dlsym (even `dlopen("libsystem_malloc.dylib") + dlsym` returns the wrapper). To get the real system malloc, `SystemAllocFns::findOriginal()` scans the `__interpose` Mach-O section at runtime. On ARM64e (Apple Silicon), this section lives in `__AUTH_CONST` (not `__DATA`) due to pointer authentication. Each entry is `{replacement, original}` — match by replacement address, read the original.

## macOS Page Reclamation

`mprotect(PROT_NONE)` does NOT release physical memory on macOS — the RSS drop visible in `task_info` is a reporting artifact. To actually reclaim physical pages, use `MADV_FREE_REUSABLE` (madvise hint 7), which is what jemalloc and WebKit use. `MADV_FREE_REUSABLE` requires pages to be accessible (PROT_READ or PROT_RW); it fails with EPERM on PROT_NONE pages.

The compressor flow calls `decommitPages()` (MADV_FREE_REUSABLE) **before** `mprotect(PROT_NONE)`, after the page data has been copied to the scratch buffer. On Linux, `MADV_DONTNEED` works regardless of protection.

## macOS arm64 / arm64e

libsmash is built **fat** (arm64 + arm64e) so it can interpose into both ordinary and PAC (arm64e) processes; the benchmark binaries are built **arm64-only** because they are the app under test and must load arm64-only preloads. See [agents/macos-arm64e.md](agents/macos-arm64e.md) for the full mechanism, the exec-vs-load trap (macOS ≤ 15 SIGKILLs third-party arm64e; macOS 26 runs it), and the debugging pitfalls.

## Key Conventions

- Never allocate from the managed heap inside smash internals — use BootstrapAlloc
- `BootstrapAlloc`'s allocation path is lock-free by design: regions are mapped read/write up front (`vm::mapPagesNoReserve`, `MAP_NORESERVE` so untouched pages still cost nothing) and `tryAllocFrom()` is a pure CAS bump. `expand_lock_` guards `addRegion()` alone. **Never take that lock from the allocation path**: `expandAndAllocate()` already holds it when it calls `tryAllocFrom()`, so re-entering it self-deadlocks on a non-recursive spinlock and wedges every other thread behind the holder. That is exactly what the old `ensureCommitted()` did — it `mprotect`-ed the next pages under `expand_lock_` on every allocation that crossed a page — and it hung the process the first time the arena had to grow. Trigger is total allocation, not concurrency: a `ThreadCache` is 92 KiB and goes through `allocate()`, so ~728 of them exhaust the 64 MiB initial region, and 4 threads were enough to reach it.
- Data pages never contain metadata (bitmap-based free tracking, pointer arrays in thread cache)
- Fine-grained locking: per-slab spinlocks, per-page spinlocks. No global heap lock. 4 arenas reduce slab lock contention.
- `PageLockTable::tryLock()` used for prefetch to avoid deadlock
- NUMA-bound blocks (`malloc_numa` → `LargeAlloc::allocateOnNode` → `VmRegion::allocatePagesOnNode`) live in per-node arenas inside the VmRegion and are compressed like any other page.
  - **Growth:** an arena grows in 64 MiB grid-aligned chunks carved off the shared bump pointer, and each chunk is `mbind`-ed once.
  - **No fixed per-node slices:** the compressor walks `[0, committedPages())` page by page every tick (chunk-bitmap rebuild, deferred-madvise sweep, soft-dirty pagemap read). Slices above the shared heap would stretch that walk across the whole region.
  - **Freeing:** freed node pages must go back to their own arena, never the shared free lists, or the binding leaks into ordinary allocations.
  - **Remaps:** any `MAP_FIXED` over a node page drops its policy. `remapPages()` calls `vm::g_after_remap_hook` to bind it again; a new `MAP_FIXED` site must do the same.
- Compressed blobs: a worker stores through `workerStore(worker)` (its node's store, or the shared one) and releases through `storeOfBlob(page_idx)`, which reads the `blob_node_` byte recorded at store time. That lookup is a **performance** hint only — `CompressStore::release` routes by the region's recorded `owner`, so a release arriving at the wrong store is forwarded instead of pushing onto that region's free list under the wrong shard lock. Keep it that way: an ordinary heap page migrates between nodes, so the owning store can never be derived from where the page lives now, and a wrong guess must stay harmless. A blob in a node store never spills to the file-backed store (`blobIsNodeLocal`).
- Compressor startup: constructor at priority 201 (after alloc8 pthread hooks at 200) calls `xxthread_init()` twice on macOS, once on Linux, ensuring the compressor starts even for non-ObjC programs (e.g., Python) and single-threaded programs
- Fault handler handles ACTIVE state (not just ACTIVE_MONITORING) to cover the race where Phase 3's batched mprotect overwrites a per-page PROT_RW restoration
- ThreadCache `drain()`/`drainAll()` bucket pointers by `span->arena_id` before returning to correct arena's slab

## Dependencies

- **alloc8**: Interposition framework (cloned via FetchContent; override with `-DALLOC8_DIR=`)
- **LZ4 v1.9.4**: Fast compression (fetched via CMake FetchContent)
- **Zstandard v1.5.6**: Dictionary compression (fetched via CMake FetchContent)

Benchmark-only deps (built by `-DSMASH_BUILD_BENCH_DEPS=ON`, all vendored via
ExternalProject into `build/bench/deps/`): Redis 8.0.2, Valkey 9.1.0,
memcached 1.6.34, RocksDB v9.8.4, LevelDB 1.23, and **Snappy 1.2.1** — vendored
so LevelDB builds with SST compression enabled and `bench_leveldb` links a
static libsnappy.a (no reliance on a system libsnappy).

## Benchmarks

See [agents/benchmarks.md](agents/benchmarks.md) for the full benchmark procedure, paper experiments, application-specific configuration, and result provenance.

Most common commands:

```bash
cd build
ctest --output-on-failure                                   # unit + integration tests
python3 ../bench/run_quick_ci.py                            # CI regression (bench_rss + bench_sqlite)
python3 bench/verify_paper_claims.py                        # verify paper figures
python3 ../bench/run_paper_experiments.py --runs 3          # full paper experiments
```

## Releases

Drop-in `libsmash` binaries are published to GitHub Releases by `.github/workflows/release.yml`. Publishing a Release (not a bare tag, not `workflow_dispatch`) is what triggers the build and attaches the five assets (Linux x86-64/aarch64 `.so`, macOS arm64 `.dylib`, macOS fat `.zip`, `SHA256SUMS.txt`). See [agents/releases.md](agents/releases.md) for the full procedure, the glibc-floor rationale, and the macOS arm64/arm64e split.

```bash
gh release create vX.Y.Z-alpha --target master --prerelease \
  --title "libsmash vX.Y.Z-alpha" --notes "…"     # publishing triggers the build
```

## Config Tuning

Key constants in `include/smash/config.h`:
- `kColdTicks = 2`: Ticks without access before fast-tier compression considered
- `kVeryColdTicks = 60`: Cold-tick threshold for the deep-tier (zstd-9) profile in the ROI model
- `kMinCompressRatio = 0.75`: Only store if compressed < 75% of original
- `kPrefetchWindow = 2`: Pages prefetched in each direction on fault
- `kDictTrainSamples = 0`: Pages before dictionary training (disabled by default; dicts net-negative)
- `kNumArenas = 4`: Call-site arena count (must be power of 2)
- `kCompressorWorkers = 2`: Initial compression worker count
- `kMaxCompressorWorkers = 8`: Cap for adaptive worker scaling (Little's Law)
- `kCompressStoreShards = 8`: CompressStore lock shards
- `kChunkSize = 64`: Pages per chunk for scan bitmap
- `kLargeAllocVmThreshold = 1MB`: Only large allocs above this go in VmRegion

Runtime environment variables:
- `SMASH_LARGE_ONLY=1`: Large-only mode — small allocations (≤16KB) pass through to system malloc. Production-supported config for concurrent workloads
- `SMASH_DEFER_MADVISE=1`: Default ON. Defers `madvise(MADV_DONTNEED)` to a per-tick sweeper after `SMASH_DEFER_MADVISE_TICKS` quiescent ticks (default 50). Load-bearing for correctness; do not disable in production
- `SMASH_DEFER_MADVISE_TICKS=N`: Number of ticks (default 50, ≈500 ms) a page must be quiescent before sweeper madvises it
- `SMASH_MODE=compress_only`: Compress-only mode — track pages without replacing malloc
- `SMASH_TRACK_EXTERNAL=1`: Register application-direct `mmap` / `mach_vm_allocate` results so the compressor sees them. Opt-in (see "External-Mapping Tracking" above)
- `SMASH_DEFER_PHASES_MS=N`: Skip Phase 2 (compress) + Phase 3 (monitor) for the first N ms after start. Useful for workloads that establish IPC channels at startup with buffers in smash-managed pages (Firefox sweet spot is 30000)
- `SMASH_NO_MONITOR=1`: Disable Phase 3 (PROT_READ access tracking) entirely. Trades cold-detection accuracy for compatibility with code paths that synchronously check page protection
- `SMASH_COLD_TIMEOUT_SEC=N`: Override cold timeout at runtime
- `SMASH_COOLING_CLOSE_TICKS=N`: Cooling-page closing threshold (default `getColdTicks()/2`; `0` disables). Slab allocation prefers warm/EMPTY pages over pages whose cold streak ≥ N, and redirects to a decommitted empty span (never a new one) when the front span has only cooling/compressed free slots. Allocator-induced re-heats are counted in the `reheat=` field of the SIGUSR2 stats line (`SMASH_STATS_SIGNAL=1`); the telemetry threshold stays at the default even when closing is disabled, so A/B runs can measure prevented re-heats. Note: objects recycled through the thread cache bypass slab placement entirely — the app's writes into recycled objects re-heat their pages regardless of this policy
- `SMASH_VERY_COLD_TICKS=N`: Override deep-tier cold-tick threshold (ROI model). `9999` disables the deep tier entirely (fast tier only).
- `SMASH_ROI_THRESHOLD=N`: Override ROI cutoff (default 1024)
- `SMASH_FAST_COMP_MBS_HI/LO`, `SMASH_FAST_DECOMP_MBS_HI/LO`: Override fast-tier calibration
- `SMASH_DEEP_COMP_MBS_HI/LO`, `SMASH_DEEP_DECOMP_MBS_HI/LO`: Override deep-tier calibration
- `SMASH_CALIBRATE=always|never`: Force/skip startup calibration
- `SMASH_CALIBRATION_FILE=path`: Cache and reload calibration
- `SMASH_NUMA_COMPRESSOR=0`: Disable the per-node compressor groups. `malloc_numa` pages are then compressed by the default workers into the shared store
- `SMASH_NUMA_WORKERS=N`: Threads per node compressor group (default `kCompressorWorkers`, capped by the node's CPU count and 8)
