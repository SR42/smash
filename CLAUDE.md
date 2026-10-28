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

All 165 tests must pass; none are registered `DISABLED`. Run the suite **as root** too when touching the compressor: idle-page read tracking only exists with root (and `SMASH_WRITE_TRACK=yt` only with the youngtrace module loaded), and CI (`.github/workflows/ci.yml`, `ubuntu-latest` + `macos-latest`, every push and PR) runs unprivileged. After ctest, CI runs `bench/run_quick_ci.py` (`bench_rss` ≥30 %, `bench_sqlite --quick` ≥5 % RSS reduction).

**Before changing a test or its threshold, read its entry in [agents/tests.md](agents/tests.md).** Thresholds encode measurements, several tests are probabilistic (one run is not a verdict), and some harness shapes (sharing a process between phases, `fork()` with smash threads running) reproduce the very symptom under test.

Rules that the investigations behind this suite keep relearning:

- **A new test must be verified as a real discriminator**: break the code it guards (source mutation, or the pre-fix library built with identical flags) and watch it fail, then restore and watch it pass. An env-var toggle is not a valid control when the test overrides that variable itself.
- **Byte-identical results across supposedly different configs mean the flag is inert**, not that the suspect is innocent. `SMASH_DEFERRED_RECLAIM` (default ON) makes many compressor knobs inert.
- **On the free path, `perf` attribution and interleaved A/B timing disagree, and only the timing counts.**
- **`bench_sqlite` tells you whether a compressor change did anything; `bench_rss` saturates at ~72 %.**
- **A churn-only stress proves nothing about the compressor** (blocks are freed before they go cold); hold a live set idle.

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

One line each; the incident and measurements behind each are in [agents/conventions.md](agents/conventions.md).

- Never allocate from the managed heap inside smash internals — use BootstrapAlloc.
- `BootstrapAlloc`'s allocation path is a lock-free CAS bump; `expand_lock_` guards `addRegion()` alone. **Never take that lock from the allocation path** — `expandAndAllocate()` already holds it, so re-entry self-deadlocks.
- Data pages never contain metadata (bitmap-based free tracking, pointer arrays in thread cache).
- Fine-grained locking: per-slab and per-page spinlocks, no global heap lock. `PageLockTable::tryLock()` for prefetch, to avoid deadlock.
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
- Never make a shared free list lock-free unless it is ABA-safe (the span-descriptor Treiber stack crashed 64-thread workloads).
- Never return empty slab spans to the arena on every free (~70 % on mi-stress); reclaim in the VmRegion exhausted hook, which must take other slab locks with `tryLock` only, never call `allocatePages()`, and run one pass at a time (a skipped slab is usually one another pass is releasing).
- A smash thread must never keep a process alive: register with `exit_guard::ThreadRegistration`, be stopped by `SmashHeap::stopAllSmashThreads()`, and never join the calling thread.
- Compressor startup: constructor priority 201 (after alloc8's pthread hooks at 200) calls `xxthread_init()` — twice on macOS, once on Linux.
- An application's `sigaction()`/`signal()` for SIGSEGV/SIGBUS must never displace smash's handler: they are interposed and chained (`FaultHandler::interposedSigaction`); smash's own installs go through `FaultHandler::sysSigaction`.
- The fault handler handles ACTIVE (not just ACTIVE_MONITORING) pages, covering Phase 3's batched mprotect overwriting a per-page PROT_RW restore. A page's state is not proof of its protection.
- ThreadCache `drain()`/`drainAll()` bucket pointers by `span->arena_id` before returning them to the right arena's slab.

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

Full list of constants and environment variables: [agents/config.md](agents/config.md). The ones that matter most:

- `SMASH_LARGE_ONLY=1`: production-supported mode; allocations ≤ 16 KiB pass through to system malloc.
- `SMASH_DEFER_MADVISE=1` (default ON): load-bearing for correctness; do not disable in production.
- `SMASH_VM_GIB=N` (default 16): compressible arena size. Costs address space, not memory; size it above the live set or large blocks silently fall back to uncompressed mappings.
- `SMASH_LARGE_ALLOC_VM_THRESHOLD=N` (default 1 MiB): below it, a block is never compressed.
- `SMASH_COLD_TIMEOUT_SEC=N`: cold timeout override (tests use 1–2). Idle time is counted in nominal seconds, so it stays seconds when ticks run long. At most 254: larger values are clamped with a warning (a page's idle count is one byte; 255 or more used to compress nothing). Use `SMASH_NO_COMPRESSOR=1` to switch compression off.
- `SMASH_WRITE_TRACK=auto|softdirty|uffd|none|idle|yt`, `SMASH_SOFTDIRTY=0`: write detection. `idle` (opt-in, needs `CAP_SYS_ADMIN` and the page_idle bitmap read-write) tracks reads and writes with the kernel's accessed bit and protects nothing: zero monitor faults on the arm64 6.6 path where `SMASH_SOFTDIRTY=0` is the only alternative. `yt` (opt-in, needs the youngtrace module and `CAP_SYS_ADMIN`) reads the same bit through the module's page-table walk: on the same workload 0.50 s of scanning against the bitmap's 2.2 s (K=4), 0.84 s against 9.2 s (K=1).
- `SMASH_RUN_CACHE_MIB=N` (default off): per-CPU cache of freed large runs — about 2x large-block churn throughput one thread per core, at up to (CPUs) x N MiB of extra RSS. See [agents/design-run-cache.md](agents/design-run-cache.md).
- `SMASH_CODEC=lz4|zstd`: fast-tier codec (default zstd-1). `SMASH_USE_LZ4` was never an environment variable; the build option of that name is removed.
- `SMASH_STATS_SIGNAL=1`: enable the SIGUSR2 stats line (`compressed=`, `zero_now=`, `track=`, `codec=`, `cold_arenas=`, `boot=`, `reheat=`, `rss=`, `blob=`).
- `SMASH_PREFETCH_WINDOW=N` (default 2, 0 = off): compressed neighbours a fault restores on each side. Costs the faulting thread a decompress and two syscalls per neighbour; random access measured 53 µs per fault at 2, 15 µs at 0.
- `SMASH_COLD_ARENA_FEEDBACK=1` (full mode, default off): a call site whose pages keep getting compressed allocates from a separate cold slab.
