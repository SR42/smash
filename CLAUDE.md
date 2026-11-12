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

### Sanitizers

`-DSMASH_SANITIZE=undefined` builds smash and its tests with UBSan. Always use a
separate build directory — the sanitized library is neither what you benchmark
nor what you ship.

```bash
cmake -S . -B build-ubsan -DSMASH_SANITIZE=undefined \
      -DALLOC8_DIR=$PWD/build/_deps/alloc8-src
cmake --build build-ubsan -j$(nproc)
cd build-ubsan && UBSAN_OPTIONS=print_stacktrace=1:log_path=/tmp/ubsan ctest
```

The suite, the benches and a stress round are clean (0 findings). Before
believing a clean run, confirm reporting works (a deliberate-UB program must
produce a `log_path` file), and run it as root somewhere, or the idle-bitmap
path is never instrumented. `address` is accepted but useless for the preloaded
library (ASan replaces malloc). Full history, including the GCC `constinit`
codegen bug: [agents/sanitizers.md](agents/sanitizers.md).

## Test

```bash
cd build && ctest --output-on-failure
```

All 280 tests must pass or skip; none are registered `DISABLED`. A skip exits 77 and ctest lists it as skipped: unprivileged on x86-64, 34 skip (idle tracker, THP unit, youngtrace). `test_libsmash_loads` is the fixture every test requires: when libsmash is missing or not this build's, nothing else runs. Run the suite **as root** too when touching the compressor: idle-page read tracking only exists with root (and `SMASH_WRITE_TRACK=yt` only with the youngtrace module loaded), and CI (`.github/workflows/ci.yml`, `ubuntu-latest` + `macos-latest`, every push and PR) runs unprivileged. After ctest, CI runs `bench/run_quick_ci.py` (`bench_rss` ≥30 %, `bench_sqlite --quick` ≥5 % RSS reduction).

**Before changing a test or its threshold, read its entry in [agents/tests.md](agents/tests.md).** Thresholds encode measurements, several tests are probabilistic (one run is not a verdict), and some harness shapes (sharing a process between phases, `fork()` with smash threads running) reproduce the very symptom under test.

Rules that the investigations behind this suite keep relearning:

- **A new test must be verified as a real discriminator**: break the code it guards (source mutation, or the pre-fix library built with identical flags) and watch it fail, then restore and watch it pass. An env-var toggle is not a valid control when the test overrides that variable itself.
- **Byte-identical results across supposedly different configs mean the flag is inert**, not that the suspect is innocent. `SMASH_DEFERRED_RECLAIM` (default ON) makes many compressor knobs inert.
- **On the free path, `perf` attribution and interleaved A/B timing disagree, and only the timing counts.**
- **`bench_sqlite` tells you whether a compressor change did anything; `bench_rss` saturates at ~72 %.**
- **A churn-only stress proves nothing about the compressor** (blocks are freed before they go cold); hold a live set idle.
- **Exit 77 only when the host cannot run the test or its scenario did not arise; never skip on the symptom the test guards.** A preloaded test that finds no libsmash fails, and so does a wait for compression that times out: a note and a pass on an uncompressed page is a skip too. Until 2026-11-09 skips returned 0 and passed: a block outside the arena, a dead double-free child and an inert test all read as green ([agents/tests.md](agents/tests.md), "Skips and the preload fixture").

## Further documentation

Read the relevant file before working in that area; none of it is loaded automatically.

| File | Contents |
|---|---|
| [agents/tests.md](agents/tests.md) | What every test guards, how it was verified, harness traps; coverage gaps |
| [agents/defect-history.md](agents/defect-history.md) | Fixed defects: bootstrap deadlock, thread-cache leak, shutdown race, `pthread_exit` hang, post-shutdown faults, idle-page tracking, arena exhaustion, metadata arena, write tracking (soft-dirty/uffd) |
| [agents/allocation-paths.md](agents/allocation-paths.md) | Which sizes are compressed, `SMASH_LARGE_ALLOC_VM_THRESHOLD`, THP/hugetlb, aligned allocations from the slab, mid-size bins |
| [agents/performance.md](agents/performance.md) | Thread-cache depth, drain path, metadata cost, codec ratio vs data, compression unit size, syscall batching, throughput limits, the `PageRunMap` lock, the ~10 µs >16 KiB churn defect |
| [agents/conventions.md](agents/conventions.md) | Long form of every Key Convention below, with the incident behind it |
| [agents/config.md](agents/config.md) | All `config.h` constants and `SMASH_*` environment variables |
| [agents/sanitizers.md](agents/sanitizers.md) | UBSan rounds, the `__*_chk` gap, the GCC `constinit` bug |
| [agents/syscall-compat.md](agents/syscall-compat.md) | EFAULT retry, interposed functions, `_FORTIFY_SOURCE` aliases |
| [agents/simulator.md](agents/simulator.md) | `smash_sim`: drive the large-block allocator (arenas, free map, decommit queue, run cache) at full rate with stubbed OS calls, per-site lock contention, invariant checks |
| [agents/benchmarks.md](agents/benchmarks.md) | Benchmark procedures, paper experiments, `verify_paper_claims.py` |
| [agents/external-tracking.md](agents/external-tracking.md), [agents/macos-arm64e.md](agents/macos-arm64e.md), [agents/releases.md](agents/releases.md), [agents/neuron-cc-investigation.md](agents/neuron-cc-investigation.md) | As named |

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
- **Arena-based allocation**: 4 arenas (`kNumArenas`). `callsiteArena()` hashes the application's allocation call site to route allocations from the same call site to the same arena. The return address comes from `alloc8_caller_ra` — an initial-exec TLS hint stored by alloc8's wrapper entries (`replace_malloc`, `custommalloc`, `operator new`, ...), read via `appCallerRA()`. Never use `__builtin_return_address(N)` inside smash for routing: the frame count between the app and smash depends on inlining/LTO/tail-call decisions and silently collapses all call sites into one (debug with `SMASH_ARENA_TRACE=1`). **Every allocating entry smash defines itself must call `ALLOC8_SET_CALLER_RA()`** before `xxmalloc`/`xxmemalign`: the versioned `operator new` aliases (`_Znwm@GLIBCXX_3.4`, ... in `linux_syscall_wrappers.cpp`, the ones libstdc++-linked code binds) did not, so they routed by whatever the previous allocation left there — and once that was glibc's stdio buffer allocation, every following `new` went to the system allocator, uncompressed. Pages within an arena contain similar data → better compression ratios. `Slab slabs_[kNumArenas * kNumClasses]` flat 2D array.
- **Zero-on-free (deferred)**: All zeroing is deferred to the compressor thread's `zeroFreeSlots()` — no zeroing occurs in the `free()` critical path. Uses non-temporal stores (`ntZeroMemory`) to avoid cache pollution. This makes partial pages compress well (zero runs → high compression ratios).
- **PageState machine**: EMPTY → ACTIVE → ACTIVE_MONITORING → COMPRESSING → COMPRESSED → ACTIVE. CAS transitions ensure safe coordination between compressor thread and fault handler.
- **CompressEngine**: Supports LZ4, zstd, and zstd+dictionary. Algorithm packed in top 2 bits of `CompressedPageInfo::comp_size`. All zstd contexts pre-allocated via `ZSTD_customMem` routing to BootstrapAlloc.
- **Algorithm tiering (single-shot, ROI-driven)**: ROI model picks fast tier (zstd-1 by default; LZ4 with `SMASH_CODEC=lz4`) or deep tier (zstd-9) per page **at initial compression time**. There is no later upgrade path; once a page is compressed, the chosen blob stays as-is until the page is decompressed by access. `AlgoProfile` carries `algo + zstd_level` so the calibrated profile reflects what's actually run; calibration in `compression_roi.h::calibrate()` benchmarks the actual fast-tier algorithm.
- **Adaptive per-bucket cost**: `SizeClassStats` tracks observed compression microseconds per tier (EMA, fixed-point). After 8 samples per (bucket, tier), the ROI model substitutes observed cost for the calibrated estimate via `selectProfile(..., observed_costs_us)`.
- **ROI gate and its probe**: before any codec runs, `CompressionROI::shouldCompress` declines a page whose bucket's ratio window says compressing it is not worth its cold time; the page backs off like a ratio-gate failure. The window only changes after a codec run, so every 64th page a bucket declines goes to the codec anyway (`kRoiProbeEvery`), and a probe that is stored restarts the window. Without the probe, one block of incompressible bytes switched compression off for every later block of its bucket, for good (issue #22, `test_roi_gate_recovers`). The cold timeout sets the gate's floor and nothing else; `SMASH_ROI_THRESHOLD=0` switches the gate off. A page that is all zero is stored as a mark, with no codec run and no blob, and is never recorded in the window, so the window says nothing about it: a declined page that reads as zero goes on to the snapshot, which decides (`readsAsZero`; issue #50, `test_roi_gate_zero_pages*`). Such pages were compressed at one in 64 per pass, and not at all where they alternated with incompressible ones. The gate reads a declined page once for each time it may have changed (a page declined before and not reported written since is not read again); a page holding only free slots is recognised from the span's bitmap, without a read. Under `SMASH_THP_UNIT` the ratio that counts is the unit's, so a unit's attempt starts the bucket's window again when the window would decline its first page, and the unit's own first 8 pages are the sample: declined unseen, units that compress well waited for a probe behind one unit of random bytes (issue #52, `test_thp_unit_window`).
- **Parallel compressor**: Coordinator thread + worker threads. Chunk bitmap (`live_chunks_[]`) skips EMPTY pages. Sharded `CompressStore` (8 shards) eliminates lock contention. Per-worker compression contexts (LZ4 state, ZSTD CCtx, scratch buffers, SizeClassStats).
- **Adaptive worker count**: each tick sets the next tick's default workers to `N = ⌈2 × load⌉`, where the load is the CPU time the workers spent in the tick's phases divided by the wall time since the previous tick started, smoothed (up at once, down a quarter of the way per tick; `src/compress/worker_scaling.h`). Twice the load, so that a tick takes no longer than the sleep between two. It was `⌈eligible / (compressed per worker)⌉`, which is workers × eligible / compressed: one worker at 100 % for ticks of 3-9 s under 512 MiB/s of cold data, and the cap of 8 for 20 ms of work a tick whenever some eligible pages were not compressed by the default workers (declined, or handed to a NUMA node's group) (issue #8, `test_worker_scaling`, `test_worker_count_*`). The pool is `kMaxCompressorWorkers` (32), allocated lazily (a worker's state is ~190 KiB); the runtime cap is `SMASH_MAX_WORKERS`, default `clamp(nproc/4, 8, 32)`, and every worker and helper thread up to the cap exists from `start()` (a `pthread_create` mid-tick is unsafe). The pool was a compile-time 8, which a 128-core host hit every tick. Each phase hands the committed range out in **dynamic slices** (`slice_pages_`: 16 MiB or a sixteenth of a worker's share, chunk- and watch-group-aligned, and under `idle`/`yt` a multiple of the scan slice so no request is cut) to whichever worker asks next, never a static per-worker partition (one worker carried a phase whose live pages sat in its eighth while seven idled). `SMASH_TICK_PAGE_BUDGET=N` caps the pages one tick compresses (slot reserved before the work; the rest stay cold for the next tick). On a NUMA host every compressor thread, default worker or node group thread, claims from every pool of a phase (shared-range slices, node chunk units, node queue batches) once its own is spent (`executePhase`, `executeNodePhase`); whichever side owned the heap used to carry the phase while the other idled at the barrier.
- **Per-fault-slot DCtx**: Each of 32 fault slots has its own `ZSTD_DCtx*`, fixing data race between concurrent decompressions from app threads and prefetch.
- **Per-origin sliding-window stats**: `SizeClassStats` keyed by `(arena, size_class, worker)` — `sc_stats[kNumArenas * kNumClasses]` per worker, indexed via `statsIndex(arena, sc)`. Each bucket holds a 64-entry ratio window (0–255) plus per-tier compression-time EMAs. Aggregating across arenas would wash out the homogeneity arena routing produces.
- **Signal handler path**: No malloc allowed. Decompression uses pre-allocated per-slot contexts only.
- **NUMA compressor groups** (Linux, hosts with ≥2 memory nodes): every page — a `malloc_numa` node arena page or an ordinary heap page — is compressed by a group of threads dedicated to its own node, not by the default workers.
  - **Threads:** pinned to the node's CPUs; they park on a futex between phases.
  - **Memory:** worker state, scratch buffers, zstd/LZ4 contexts and stacks come from memory bound to the node (`vm::NodeBumpAlloc`, `src/vm/node_memory.h`). Blobs go to a per-node `CompressStore` whose regions are bound to the node.
  - **Scheduling (arena pages):** each tick `assignNodeChunks()` lists each node's chunks, handed out in 8 MiB units (`takeNodeUnits`) to the group's threads first and then to any thread with nothing left of its own. A thread on the shared range skips those chunks (`ownedByRunningNodeGroup`); a thread on a unit does not (`tl_on_node_unit_`). A node arena page met by another node's thread is routed to its node's queue like any other, so its blob still lands in its node's store.
  - **Ordinary heap pages** carry no binding, so their node has to be discovered. `page_node_` holds a per-page hint, learned free in the fault handler (`notePageNode`: a restored page lands on the faulting thread's node), from a `yt` scan's items with `SMASH_YT_NODE_HINTS=1` (exact for an item and a run's first member; the rest of a run is sampled one page in 64 with `move_pages` and filled only between agreeing samples, never guessed: the guess put a whole block's blobs off its node), and otherwise resolved in one `move_pages` batch per Phase 2 range (`flushNodeQueries`). Phase 2 hands each eligible page to its node's queue (`routeByNode` → `enqueueForNode`) instead of compressing it — staged per routing worker and written as one 256-slot reservation at a time (`flushRouteStage`, unused tail `kQueueEmpty`), because a slot taken per page let concurrent routers interleave the queue page by page and cost an mprotect per 3 pages in phase 5 — and the queue is drained in **phase 5** in batches of one reservation (`drainQueueBatches`): by the group's threads first, then by the default workers (the node a worker's CPU is on first) and the other groups, always into the node's store (`route_store`). The default workers used to idle through phase 5 while the groups compressed the whole cold set, and the groups through phases 1–4 of an ordinary heap; `smash_work_share_info` and the stats line's `help=` count the shared work. A wrong hint costs locality, never correctness.
    - A page whose node's queue is full (64K entries per node per tick) is compressed by the routing worker **into that node's store** (`compressPageTo`, `route_store`): blob placement never depends on queue capacity. It used to go to the shared store, so a first cold wave past 256 MiB per node per tick put most blobs off-node for good.
    - The batch must be **rolling**: flush it when full and keep deferring. A fixed cap per range routed only the first batch and sent everything after it to the local worker and the shared store — measured as three of four blocks' blobs landing on one node.
  - **Lifetime:** groups are created once in `start()` and never freed. Never create them mid-tick: `pthread_create` is unsafe once pages may be compressed.
  - **Still shared:** the coordinator, the deferred-madvise sweep and fault-time decompression. Decompression runs on the faulting thread; the arena binding puts the restored page back on the node.

## Allocation Fast Path

From upstream (#90, #93). Small `malloc`/`free`/`calloc`/`realloc` are served by lean entry points (`SmashFastRedirect` in `smash_heap.cpp`) that alloc8's `xx*` functions call directly; everything else tail-calls the full `SmashHeap` path, which handles every mode.

- **One gate:** the per-thread `g_fast_tc` is published only when the lean path is valid (`leanEntriesAllowed()`: full mode, no free diagnostics, no `SMASH_COLD_ARENA_FEEDBACK`, heap constructed) and cleared before a thread's cache goes back to the pool, by `threadCleanup()` and by the TLS destructor (`reclaimThreadCache`) alike.
- **malloc:** the `callsiteArena()` memo hit only. A memo naming `kStdioArena` takes the full path (stdio buffers go to the system allocator), and once anything has been compressed the block is marked on held pages like the full path does (`noteHeld`, a tail call; without it a reused slot counted as thrash, `test_alloc_reopen`).
- **free() without touching the Span:** the VmRegion keeps a 2-byte page tag (small flag, arena, size class) beside the flat page→Span table, so free picks the thread-cache lane with one load. Large and **mid-size** spans are tagged 0: a `SMASH_MID_BINS` block must go back to `MidAlloc`, never to a thread cache.
- **Thread cache:** one lane per arena (`getTotalArenas()`, fixed at startup; `ThreadCache::layout()`), the 256-pointer budget per class split between them and the depth capped at 16: depth 16 up to 16 arenas, 8 at 32, 4 at 64, and 4 at 128 (the floor, twice the memory). **Keep the cap**: a lane hands its slots out again whatever page they are on, so a deeper cache puts more churn on pages going cold (depth 64 at 4 arenas failed `test_cooling_close`; `test_cooling_close_4_arenas` shows it on any host). Counts in a compact array separate from the pointers. **Never let two arenas share a lane**: a refill fills it from the asking arena's slab and the other arena's call site takes from it next, so placement mixes on every refill, not only for recycled objects. Upstream #93's fixed 16 lanes did that above 16 arenas (17+ CPUs, or cold-arena feedback): at 64 arenas 57% of objects stayed in their call site's arena (`test_thread_cache_lanes_*`; this branch only). `SMASH_CACHE_LANES=N` (build option) caps the lanes, for measuring exactly that. The runtime stride costs 1–5% on the pure fast path where the split is unchanged (16 arenas; [agents/performance.md](agents/performance.md)).
- **Span::slotOf** divides by multiply and shift with a per-class table (`kSpanDivMagic`), not a Span field: `Span` stays at its pinned 112 bytes. Mid-size spans divide (their objects are past the bound the reciprocal is exact for).
- **Linking:** `LINKER:-Bsymbolic` binds the wrappers' calls to `xxmalloc`/`xxfree` locally. The old `"-Bsymbolic"` went to the compiler driver as `-B symbolic` and never reached ld.
- **Direct-mapped large blocks** (below `SMASH_LARGE_ALLOC_VM_THRESHOLD`, so 16 KiB–1 MiB in full mode) are cached on free with their Span descriptors (`LargeBlockCache` in `large_alloc.h`) instead of being unmapped: 64 KiB churn 4412 → 34 ns/op. Sizes round to 8 steps per power of two (`resizeInPlace` keeps a cacheable block on that grid); idle buckets are unmapped after 2 compressor ticks. Never a `malloc_numa` block (`Span::block_cacheable`). **Upstream also caches arena blocks up to 8 MiB by default; this branch does not**: a freed arena block takes the release path, where keeping it warm stays opt-in (`SMASH_RUN_CACHE_MIB`, `SMASH_SUBARENAS`), and a direct mapping of a compressible size (one made because the arena was full) is never cached, which would hand it out ahead of reclaimed arena space (`test_arena_reclaim_concurrent`). Caching arena blocks measured only 5–8% on 2 MiB churn here: ~30 µs of each 2 MiB free is `releaseCompressedPages`' per-page walk.
- **Empty slab spans** stay resident for 2 compressor ticks before `madvise` (they leave compressor tracking at once), under a process-wide cap; spans outside the arena decommit as their own mapping. They wait on their own list (`resident_`, newest first; decommitted ones are on `empty_`), and the tick's purge takes the old ones off its back 64 at a time and drops their pages **with the slab lock released**, neighbours in one call. It used to hold the lock across one `madvise` per span: after a 768 MiB free every `malloc` of that class and arena waited 270-300 ms (issue #49, `test_slab_purge_*`).
- **Background threads** park on futexes (`util/park.h`: compressor helpers on `helper_go_`, decommit queues on their `wake` word, page locks), `__ulock_wait` on macOS. Upstream's `Waker` (a condition variable) for the same polling loops is superseded: a condition variable cannot serve the page locks, which park from the fault handler. A `Spinlock` waiter sleeps on the lock word after about 20 µs of spinning and `unlock()` wakes one sleeper; it used to spin and `sched_yield()` for as long as the hold lasted (`test_spinlock_park`).

Measured upstream on x86-64 (Xeon 8175M) and Graviton3, glibc 2.35, against upstream's own base: faster than glibc on every microbenchmark (malloc+free pairs 0.78×/0.86×, 16 KiB–1 MiB churn 0.22×/0.09×), g++ -O2 at parity, Python churn 0.67×/0.64×.

## Syscall & Buffered I/O Compatibility

See [agents/syscall-compat.md](agents/syscall-compat.md) for details on EFAULT-driven decompress-and-retry, DYLD interposition limitations, buffered-I/O carve-outs, and the full list of interposed functions per platform. It also records two things worth knowing before adding a wrapper: a wrapper exported only at the base glibc version never runs when glibc versions that symbol (the caller binds the versioned one: `test_symbol_versions` compares every function libsmash defines with the default version the loaded system libraries give it, so a new wrapper at the wrong version fails it), and a measured audit of which syscalls still return `EFAULT` when their buffer is on a compressed page.

The EFAULT retry is only safe when the kernel did nothing. An out-parameter the kernel writes **after** acting (`wait4`/`waitid` status, `accept`/`recvfrom` address, `recvmsg`'s `msghdr` and control data, `sendfile` offset, `sendmmsg` `msg_len`, old mask/limit, `nanosleep` `rem`) goes to the kernel as a **stack copy** when it is on a smash page (`withOut`, `withAddrOut`, `recvmsgOut`), never retried: the retry reaped, accepted, received, sent or slept a second time (`test_syscall_side_effect`). `read` on an `eventfd`/`signalfd`/`timerfd`/`inotify`/tty can still lose what it consumed (its data buffer cannot be bounced).

A buffer that is only partly accessible (first page writable, a later one watched or compressed) makes file I/O return a **short count** instead of `EFAULT`; `fileIoWith1Buf`/`fileIoWithIovec` continue the transfer for regular files and block devices only (never sockets/pipes, where continuing would block). Before that fix a 2 MiB `read` under `SMASH_SOFTDIRTY=0` could return 4 KiB. Stdio stream buffers come from the system allocator, not smash (`StdioBufferCaller`: allocations made from glibc's `_IO_file_doallocate`), because glibc fills them with an internal `read` that never reaches the EFAULT retry: in full mode `getline`, `fscanf` and every `*_unlocked` reader lost data with errno 14, and so did fortified `fgets`/`fread` (`__fgets_chk`/`__fread_chk`, now wrapped). On a message socket (UDP, AF_UNIX dgram/seqpacket) a faulting receive makes the kernel **drop the datagram**, so the EFAULT retry returned the next one: receives into smash memory on such sockets now peek first (`peekedRecv`), and `recvmmsg`/`sendmmsg` are exported at `GLIBC_2.12`/`2.14` too — at the base version alone they never ran.

That audit found 21 exposed syscalls; all of them are wrapped now except `futex`, which has no libc symbol to interpose (and is the least exposed of the set: every pthread primitive touches its futex word from userspace first, which faults through smash's handler). Measured on the fixed library with the same probe: **22 of 22 raw kernel entries return EFAULT on a compressed page, and 22 of 22 libc entries recover** — `getgroups` was the one that did not, until `__getgroups_chk` was wrapped too, because `gcc -O2` rewrites `getgroups` into the fortified alias for a `malloc`'d list. `test_syscall_efault`'s phase 19 is the in-tree guard: one compression wait, then one probe per syscall on its own page, every buffer and descriptor prepared *before* the wait, since filling a buffer is a write and a write is what keeps a page out of the compressor's reach. Interposition costs ~15 ns per syscall and the retry logic ~5 ns on top (~1%), paid only by these syscalls.

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
- the band above that and below `SMASH_LARGE_ALLOC_VM_THRESHOLD` (default 1 MiB), which is never compressed → system malloc too (`largeOnlySystemSize`). It used to be a mapping per malloc: 80 page faults per churned 320 KiB block against 0, a syscall pair and a TLB shootdown. `SMASH_MID_BINS=1` keeps the band in the arena instead; lowering the threshold shrinks it
- `malloc(size >= threshold)` → Smash's `LargeAlloc` → VmRegion (compressible)
- `realloc` of a system block to a compressible size moves it to the arena, once; left with glibc, a buffer grown by realloc was never compressed
- `free(ptr)` checks `page_map_`; non-Smash pointers forwarded to system free
- `getSize(ptr)` checks `page_map_` first (returns Smash size class or `span->large_size`); falls back to system `malloc_size`/`malloc_usable_size` only for non-Smash pointers

### Resolving original system malloc on macOS

On macOS, `dlsym(RTLD_NEXT, "malloc")` returns the interposed wrapper — DYLD interposition is truly global and cannot be bypassed via dlsym (even `dlopen("libsystem_malloc.dylib") + dlsym` returns the wrapper). To get the real system malloc, `SystemAllocFns::findOriginal()` scans the `__interpose` Mach-O section at runtime. On ARM64e (Apple Silicon), this section lives in `__AUTH_CONST` (not `__DATA`) due to pointer authentication. Each entry is `{replacement, original}` — match by replacement address, read the original.

## macOS Page Reclamation

`mprotect(PROT_NONE)` does NOT release physical memory on macOS — the RSS drop visible in `task_info` is a reporting artifact. To actually reclaim physical pages, use `MADV_FREE_REUSABLE` (madvise hint 7), which is what jemalloc and WebKit use. `MADV_FREE_REUSABLE` requires pages to be accessible (PROT_READ or PROT_RW); it fails with EPERM on PROT_NONE pages.

The compressor flow calls `decommitPages()` (MADV_FREE_REUSABLE) **before** `mprotect(PROT_NONE)`, after the page data has been copied to the scratch buffer. On Linux, `MADV_DONTNEED` works regardless of protection.

## macOS arm64 / arm64e

libsmash is built **fat** (arm64 + arm64e) so it can interpose into both ordinary and PAC (arm64e) processes; the benchmark binaries are built **arm64-only** because they are the app under test and must load arm64-only preloads. See [agents/macos-arm64e.md](agents/macos-arm64e.md) for the full mechanism, the exec-vs-load trap (macOS ≤ 15 SIGKILLs third-party arm64e; macOS 26 runs it), and the debugging pitfalls.

## macOS Per-Thread State (TSD slot, `SMASH_TSD_SLOT`)

From upstream (#89, #91). On Mach-O every `thread_local` access from a dylib is a `_tlv_get_addr` call (`tls_model("initial-exec")` has no effect), one per distinct variable, and it was the hottest non-waiting frame on the malloc/free path. On Apple silicon the hot per-thread fields therefore live in one `ThreadState` (`smash_heap.h`) whose pointer sits in pthread TSD slot `SMASH_TSD_SLOT_INDEX` (default **89**, `__PTK_FRAMEWORK_OLDGC_KEY9`, mimalloc's: reserved by libpthread, never handed out by `pthread_key_create`); a read is `mrs tpidrro_el0` + load (`src/util/tsd_slot.h`). Linux and x86-64 macOS keep the `thread_local`s.

- `g_thread_cache`, `g_fast_tc`, `g_full_mode_cached`, `g_arena_stack_base` and `g_arena_memo` are macros for `threadState()->field` there, so call sites are unchanged.
- **Lifecycle:** a null slot means not set up; `threadStateSlow()` reuses a state from a free list or carves one from `BootstrapAlloc`, and arms a pthread-key destructor that drains a live thread cache at exit (GCD workers never get `xxthread_cleanup`) and recycles the state. The free list's lock is in the fork lock set (`collectForkLocks`).
- **Caller hint:** `tsd_slot.h` is force-included into every libsmash TU and defines `ALLOC8_CALLER_RA_HOOK`, so alloc8's wrappers store the return address in `ThreadState::caller_ra`. It needs an alloc8 with `ALLOC8_HAS_CALLER_RA_HOOK` (the pin is 882ff34); the build `#error`s otherwise, since call-site arenas would silently collapse.
- **Conflicts:** if the main thread's slot already holds something that is not a `ThreadState` when smash starts, smash aborts with a message. No runtime fallback: pick another slot with `-DSMASH_TSD_SLOT_INDEX=N` or disable with `-DSMASH_TSD_SLOT=OFF`.
- **Test:** `test_tsd_thread` (macOS only). Measured upstream (M1 Max): malloc+free 41.4 → 24.7 ns/op, jaq 24.3 → 18.0 s.


## Key Conventions

One line each; the incident and measurements behind each are in [agents/conventions.md](agents/conventions.md).

- Never allocate from the managed heap inside smash internals — use BootstrapAlloc.
- `BootstrapAlloc`'s allocation path is a lock-free CAS bump; `expand_lock_` guards `addRegion()` alone. **Never take that lock from the allocation path** — `expandAndAllocate()` already holds it, so re-entry self-deadlocks.
- Data pages never contain metadata (bitmap-based free tracking, pointer arrays in thread cache).
- Fine-grained locking: per-slab and per-page spinlocks, no global heap lock. `PageLockTable::tryLock()` for prefetch, to avoid deadlock.
- `Slab` is laid out by cache line (256 bytes, aligned): `lock_` with the fields only `init()` writes, the span lists on the next two lines, what freeing threads write without the lock on the last. Keep anything written during a hold, or by a freeing thread, off `lock_`'s line: waiters read that line in a loop (four threads on one slab: 76 M objects/s, 55-66 with the alternatives; [agents/performance.md](agents/performance.md), "The slab's cache lines").
- Never make a system call per list entry under a `Spinlock`: take the entries off their list in bounded batches, release the lock, do the work, put them back (`Slab::purgeResidentEmpty`, `LargeBlockCache::purgeIdle`). Every thread that needs the lock waits for the whole walk.
- NUMA node arenas (`malloc_numa` → `VmRegion::allocatePagesOnNode`) grow in 64 MiB grid-aligned `mbind`-ed chunks off the shared bump pointer (no fixed per-node slices). Freed node pages go back to **their own** arena, never the shared free lists. Each node arena has its own free map and its own decommit queue and thread; nothing on one node touches another node's lock, queue or thread. Any `MAP_FIXED` over a node page must call `vm::g_after_remap_hook` to rebind it.
- Blobs are stored via `workerStore(worker)` and released via `storeOfBlob(page_idx)`, which is only a performance hint: `CompressStore::release` forwards by region owner. Keep a wrong guess harmless — heap pages migrate between nodes.
- Read or change `compressed_[i]` only under page `i`'s lock: store compaction moves blobs (`moveBlobOutOfDraining`), and the lock is all that keeps a reader off the old copy.
- Double-free detection needs **both** the thread-cache lane scan (the common case) and the `Span::deallocate` bitmap check, and must **drop** the second free, or `--allocated_count` underflows.
- Never export both `name@@VER` and `name@VER` for one interposed function (the forwarder jumps to itself). List only the `_NN` alias in the version script.
- The generated version script must be a `LINK_DEPENDS` of `smash` and `smash_noopt`, or editing it relinks nothing.
- Interposed thread hooks must be exported at the version callers bind: `pthread_create`/`pthread_exit` at the base version **and** `GLIBC_2.34`.
- A thread cache must be reclaimable without the interposer (`pthread_key_t` destructor); keep both double-return guards. `newThreadCache()` can return nullptr and every caller handles it.
- Every file-scope `extern thread_local` is `constinit`, declared and defined.
- A thread parking on a futex checks its exit flag **before** parking, not only after a generation change.
- Never trust an idle-bitmap mark you have not read back, and never read the idle bit of a page you did not mark.
- Never give a smash thread a fixed stack size — use `createSmashThread()` / `smashThreadStackSize()` (`util/thread_stack.h`).
- A thread smash creates never allocates from the heap smash manages, and has no thread cache: start it with `startSmashThread()` (or `createSmashThread()`), never plain `pthread_create`, and call `enterSmashThread()` first in its entry function. The mark (`g_smash_thread`) has to be there before anything on the thread can allocate; it used to be set after `threadInit` had given the thread a cache, and every small `malloc` on a compressor thread came from the arena (issue #7, `test_smash_thread_alloc_*`). A new allocating entry point must send a marked thread to the system allocator, as `mallocSlow`, `calloc` and `memalign` do.
- Size the compressor's worker pool by load (CPU time of the phases per second of wall time), which does not depend on how many workers did the work; never by a quotient of what the workers achieved, which has the current count as a fixed point (issue #8, `worker_scaling.h`).
- A gate that decides from statistics must keep feeding them: let a share of what it declines through (`kRoiProbeEvery`), or the first bad window is final (issue #22).
- A gate must not decline, on its bucket's statistics, a page those statistics do not describe: an all-zero page needs no codec and is never in the ratio window (issue #50).
- A compressor thread reads a page it is not about to compress at most once for each time the page may have changed (`readsAsZero`'s `seen`). A read brings a swapped page back, sets the accessed bit the kernel and the `idle`/`yt` trackers go by, and can take a NUMA hinting fault.
- Under a scanning tracker (`idle`, `yt`) every read by a compressor thread that leaves the page uncompressed is recorded (`noteOwnRead`: the gate's look, the snapshot of a compression that fails, a unit that is rolled back), and phase 1 keeps the page's back-off count through the scan that reports the read. The scan cannot tell that read from the application's: a failed compression cleared its own back-off and was tried again one cold floor later, for ever (issue #52, `test_fail_backoff_*`, `test_thp_unit_backoff*`). A large folio has one accessed bit for all its pages, so the scan carries the record to every page that shares the bit (`idleScan`, `ytMarkItem`).
- A variable whose only reader is a signal handler is `volatile` (`t_snapshot_page_plus1`): the compiler removed the store that announced a direct page read as dead, and the worker's fault was fatal.
- An environment variable sets one thing. `SMASH_COLD_TIMEOUT_SEC` also switched the ROI gate off, and since every test and benchmark sets it, none of them ran the policy the default runs (issue #10). An ablation gets its own switch.
- Never make a shared free list lock-free unless it is ABA-safe (the span-descriptor Treiber stack crashed 64-thread workloads).
- Never return empty slab spans to the arena on every free (~70 % on mi-stress); reclaim in the VmRegion exhausted hook, which must take other slab locks with `tryLock` only, never call `allocatePages()`, and run one pass at a time (a skipped slab is usually one another pass is releasing).
- A smash thread must never keep a process alive: register with `exit_guard::ThreadRegistration`, be stopped by `SmashHeap::stopAllSmashThreads()`, and never join the calling thread.
- Compressor startup: constructor priority 201 (after alloc8's pthread hooks at 200) calls `xxthread_init()` — twice on macOS, once on Linux.
- An application's `sigaction()`/`signal()` for SIGSEGV/SIGBUS must never displace smash's handler: they are interposed and chained (`FaultHandler::interposedSigaction`); smash's own installs go through `FaultHandler::sysSigaction`.
- The fault handler handles ACTIVE (not just ACTIVE_MONITORING) pages, covering Phase 3's batched mprotect overwriting a per-page PROT_RW restore. A page's state is not proof of its protection.
- ThreadCache `drain()`/`drainAll()` bucket pointers by `span->arena_id` before returning them to the right arena's slab.
- `realloc` of a smash block resizes in place when it can (`LargeAlloc::resizeInPlace`, `VmRegion::tryExtendRun`: the tail is released like a free, growth takes the free pages behind the block, a small block stays put when the new size fits and uses more than half); it copies only when it must. Never let an in-place path hand back pages a release would treat as part of a whole chunk (`runIsChunkBig`), and pages taken past the bump pointer must be committed like `allocatePages` does (`ensureCommitted`): a tail left above the commit watermark is swept `PROT_RW` by the next commit, compressed or not (zeros read, writes lost, old blob restored).
- Never put a page into the shared free map, a sub-arena's free runs with its dirty bit clear, or the chunk pool unless it is fresh or was decommitted: `calloc` skips its memset for a block from there (`VmRegion::allocateLargePages`'s `zeroed`). A dirty source must report itself (`took_dirty`). A freed page that is not decommitted is marked (`PageRunMap::markDirtyUnlocked`) before it is inserted.
- Fresh region goes last: a miss drains what is on its way back before it carves (`allocatePages`, `allocateOnSubArena`). The bump pointer never comes down, and carving instead turned a lagging purger into address space (all of a 64 GiB arena in 8 s with 512 MiB live).
- A bounded `take` may miss a run that fits (`PageRunMap::kTakeScan`). Once there is no fresh region left, use `takeAny` / `kScanAll`.
- `SmashHeap()` and `SystemAllocFns::resolve()` must not reach the heap: anything they allocate, directly or through a libc call that does (glibc < 2.34's `dlsym` callocs), comes from `core/early_alloc.h`, and alloc8's `getHeap()` re-entered on the constructing thread aborts (patch 0003). Keep both windows (`early::Window`) around any new code there.
- Every `Spinlock` the allocator owns must be in `SmashHeap::collectForkLocks()` (`util/fork_locks.h`): `lock()`/`unlock()` are what alloc8's atfork handlers call, and a lock left out is inherited held by a fork child whenever another thread held it at the clone. `ForkLockSet` drops what does not fit, so it must never fill (`smash_fork_lock_info`). Page locks are the exception: never waited for (a fault handler holds one for microseconds), always reset in the child (`PageLockTable::resetForFork`), whether or not a tick was in flight.
- Under `SMASH_THP_UNIT` a page of a unit compressed whole (`kUnitWhole`) comes back with the rest of its unit or not at all (`restoreUnit`; prefetch skips it): one 4 KiB written into a dropped huge range makes the kernel allocate the whole 2 MiB. Anything that protects or drops a unit covers all 2 MiB, or it splits the huge page. Several page locks are taken in ascending order or with `tryLock`.
- A thread waiting for a page lock parks after ~20 µs of spinning (`PageLockTable`: three-state byte, hashed futex words in `util/park.h`); never make it spin for ever again, and never hold a page lock across anything that can block.
- At `vm.max_map_count` a restore takes the page's run or its whole VMA (`restoreNearCap`, `restoreWholeVma`: no split needed). A page that still cannot be made accessible keeps its state and its blob, and the access retries with no page lock held (`capStall`, FATAL after 2 s). Never `remapPages` a page whose backing holds live data: it maps zeros over it.
- Under a scanning tracker (`idle`, `yt`) a page is snapshotted with a direct memcpy, never through `/proc/self/mem`; the worker announces the page in `t_snapshot_page_plus1` so the handler can make a PROT_NONE one readable instead of treating the worker's fault as fatal. Phase B's content compare under PROT_READ is what catches a write, in every mode.
- A tick must not read anything proportional to the arena's high-water mark: the live bitmap is kept by `PageStateTable` on every state store, the deferred sweep returns when its pending count is 0. `stop()` never drains the compressed set: the handler stays installed through exit.
- A wrapper that warms a buffer touches it with `touchByteWritable` (an atomic +0), never a read-then-write: the latter lost another thread's store to the byte.
- smash's SIGSEGV/SIGBUS handler runs on the alternate stack smash gives each thread (`SA_ONSTACK`, `ensureAltStackForThread`); the application's `sigaltstack()` is recorded, not installed. `FaultHandler::start()` must never record its own handler as the chain target (a fork child calls it again).

## Dependencies

- **alloc8**: Interposition framework (cloned via FetchContent at a pinned commit, `SMASH_ALLOC8_COMMIT` in CMakeLists.txt, with `patches/alloc8/*.patch` applied on top — 0001 `aligned_alloc`/`pvalloc`, 0002 the `new_handler`, 0003 `getHeap()` re-entry aborts; override with `-DALLOC8_DIR=`, which warns for each patch that tree lacks). Fix alloc8 bugs with a patch there, never by editing `_deps/alloc8-src`; moving the pin means checking each patch still applies and is still needed.
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

Full list of constants and environment variables: [agents/config.md](agents/config.md). The ones that matter most:

- `SMASH_LARGE_ONLY=1`: production-supported mode; allocations ≤ 16 KiB pass through to system malloc.
- `SMASH_DEFER_MADVISE=1` (default ON): load-bearing for correctness; do not disable in production.
- `SMASH_VM_GIB=N` (default 16): compressible arena size. Costs address space, not memory; size it above the live set or large blocks silently fall back to uncompressed mappings.
- `SMASH_LARGE_ALLOC_VM_THRESHOLD=N` (default 1 MiB): below it, a block is never compressed. Under `SMASH_LARGE_ONLY=1` it is glibc's, otherwise a direct mapping.
- `SMASH_COLD_TIMEOUT_SEC=N`: cold timeout override (tests use 1–2). Idle time is counted in nominal seconds, so it stays seconds when ticks run long. At most 254: larger values are clamped with a warning (a page's idle count is one byte; 255 or more used to compress nothing). Use `SMASH_NO_COMPRESSOR=1` to switch compression off. It sets the floor and nothing else: until 2026-11-10 a timeout that was set also switched the ROI gate off (issue #10). `SMASH_ROI_THRESHOLD=0` is the switch for that.
- `SMASH_WRITE_TRACK=auto|softdirty|uffd|none|idle|yt`, `SMASH_SOFTDIRTY=0`: write detection. `idle` (opt-in, needs `CAP_SYS_ADMIN` and the page_idle bitmap read-write) tracks reads and writes with the kernel's accessed bit and protects nothing: zero monitor faults on the arm64 6.6 path where `SMASH_SOFTDIRTY=0` is the only alternative. It keeps the host's THP setting (`SMASH_THP=1` hints the arena, `SMASH_THP=0` forbids it huge pages) and reads a large folio at its head. `yt` (opt-in, needs the youngtrace module and `CAP_SYS_ADMIN`) reads the same bit through the module's page-table walk: on the same workload 0.50 s of scanning against the bitmap's 2.2 s (K=4), 0.84 s against 9.2 s (K=1).
- `SMASH_THP_UNIT=1` (opt-in, `idle` tracking only): a cold transparent huge page is compressed and restored as one 2 MiB unit, so it comes back a huge page; without it compression splits it and a fault brings back 4 KiB. The price is per fault: one byte read restores 2 MiB (512 decompressions on the faulting thread: 630 us against 26 us per page, measured on highly compressible data). A unit's compression ratio is judged as a whole. Hints the arena huge like `SMASH_THP=1`; where it cannot work it says why on stderr and stays off. See "Hugepages and THP" in [agents/allocation-paths.md](agents/allocation-paths.md).
- `SMASH_LARGE_CACHE_MB=N` (default 32, `0` = off): byte cap of the cache of freed direct-mapped large blocks (`LargeBlockCache`; sizes below `SMASH_LARGE_ALLOC_VM_THRESHOLD`). Idle buckets are unmapped after 2 ticks.
- `SMASH_EMPTY_RESIDENT_MB=N`: cap on empty slab spans kept resident for 2 ticks before their `madvise`. Default 1/32 of RAM, clamped to [64 MiB, 1 GiB]; `0` decommits at once.
- `SMASH_RUN_CACHE_MIB=N` (default off): per-CPU cache of freed large runs — about 2x large-block churn throughput one thread per core, at up to (CPUs) x N MiB of extra RSS. See [agents/design-run-cache.md](agents/design-run-cache.md).
- `SMASH_CODEC=lz4|zstd`: fast-tier codec (default zstd-1). `SMASH_USE_LZ4` was never an environment variable; the build option of that name is removed.
- `SMASH_STATS_SIGNAL=1`: enable the SIGUSR2 stats line (`compressed=`, `zero_now=`, `bump=`, `arena_free=`, `track=`, `codec=`, `cold_arenas=`, `boot=`, `reheat=`, `rss=`, `blob=`; `bump=` rising while `arena_free=` is large means the arena is walking past its own free memory).
- `SMASH_PRESSURE_LOG=<path>|stderr` (measurement only, upstream #95): a `tick` line per compressor tick (compressed population, entries, exits by kind, compressor and fault CPU, PSI, RSS) and residence-time histograms at exit; summarize with `scripts/pressure_report.py`. Details in [agents/config.md](agents/config.md).
- `SMASH_PREFETCH_WINDOW=N` (default 2, 0 = off): compressed neighbours a fault restores on each side. Costs the faulting thread a decompress and two syscalls per neighbour; random access measured 53 µs per fault at 2, 15 µs at 0.
- `SMASH_COLD_ARENA_FEEDBACK=1` (full mode, default off): a call site whose pages keep getting compressed allocates from a separate cold slab.
