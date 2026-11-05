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

All tests must pass (`ctest` lists them; some are macOS-only). CI (`.github/workflows/ci.yml`) runs the full suite on `ubuntu-latest` and `macos-latest` on every push and PR. Five of the tests are end-to-end under `DYLD_INSERT_LIBRARIES` / `LD_PRELOAD`:

- `test_arena_routing` guards call-site arena routing — the invariant behind the paper's compression-homogeneity claim. Runs a program with 32 distinct noinline allocation sites under interposition and asserts, via the exported `smash_arena_for(ptr)` query, that they do **not** all collapse into one arena. This is the regression guard for the LTO/wrapper return-address bug: under dynamic interposition the return address seen inside the allocator can point into the wrapper library instead of the app, collapsing every call site into one arena (homogeneity 0%). Fast and deterministic (no compressor): full mode routes 256 B allocations through the slab, and `SMASH_CPU_ARENA=0` + default thread-hash-off make the arena a pure function of the return address, so a buggy capture yields exactly 1 distinct arena. Verified as a real discriminator (injecting a constant RA makes it fail with `distinct_arenas=1`). Must run under interposition — a direct-linked call would never exercise the wrapper path where the bug lives.

- `test_external_mapping` (uses a live compressor) exercises the `SMASH_TRACK_EXTERNAL=1` registration path. The ctest invocation sets the env var; without it the test would silently no-op (registration path gated).
- `test_malloc_compression` and `test_large_only_compression` are the **same executable** (`mode_compression_test.cpp`) run twice, differing only in `SMASH_LARGE_ONLY` (`0` = full, `1` = large-only — the production-supported config). The mode is read from the env at runtime, never a compile flag, so the two modes execute byte-identical assertions and cannot drift apart in coverage. The body allocates large chunks (≥ 1 MiB, so they clear `kLargeAllocVmThreshold` and enter the compressible VmRegion in **both** modes) interleaved with small chunks (≤ 16 KiB — smash slab in full mode, system passthrough in large-only). It sleeps past `SMASH_COLD_TIMEOUT_SEC`, sends `SIGUSR2`, parses `compressed=N` from the stats line, asserts `N > 0`, then verifies a byte-exact read-back of **both** classes — proving full-mode slab integrity / large-only passthrough is never corrupted, and that fault-decompress restores the compressed large chunks. (Registered via the `add_mode_compression_test(name, large_only)` helper in `tests/CMakeLists.txt`.)
- `test_compression_ratio` (in-process, direct-linked, no compressor thread) compresses a realistically-compressible page with LZ4 and zstd and asserts the achieved ratio against the paper's RQ2 figures (evaluation.tex: LZ4 4.7–12.3×, zstd-1 8.8–20.4×). Two-tier: a hard floor that fails the test (2.5× LZ4 / 4× zstd — catches "stored uncompressed" regressions) plus a WARN if it falls short of the paper's best-case number. Also asserts byte-exact roundtrip. Unlike `test_malloc_compression` (which only checks compression *happened*), this pins the codec *ratio*.

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

## Syscall & Buffered I/O Compatibility

See [agents/syscall-compat.md](agents/syscall-compat.md) for details on EFAULT-driven decompress-and-retry, DYLD interposition limitations, buffered-I/O carve-outs, and the full list of interposed functions per platform.

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

## macOS Per-Thread State (TSD slot, `SMASH_TSD_SLOT`)

On Mach-O every `thread_local` access from a dylib calls `_tlv_get_addr` — `tls_model("initial-exec")` has no effect (there is no static TLS model) — and each distinct variable a function touches is its own call. Smash's malloc/free fast path touched eight of them plus alloc8's caller-RA hint, which made `_tlv_get_addr` the hottest non-waiting frame in allocation-heavy programs.

On Apple silicon (arm64/arm64e) the hot per-thread fields therefore live in one `ThreadState` (`smash_heap.h`) whose pointer sits in pthread TSD slot `SMASH_TSD_SLOT_INDEX` (default **89**, `__PTK_FRAMEWORK_OLDGC_KEY9`, the slot mimalloc uses: reserved by libpthread for the long-gone Objective-C GC, never handed out by `pthread_key_create`). A read is `mrs tpidrro_el0` + load; see `src/util/tsd_slot.h`. x86-64 macOS keeps the `thread_local`s: the `%gs` equivalent could not be validated (smash is unstable on x86-64 under Rosetta with or without it, and no Intel Mac was available).

- The old names (`g_thread_cache`, `g_free_cache_*`, `g_arena_stack_base`, `g_full_mode_cached`) are macros for `threadState()->field`, so call sites are unchanged. Linux and x86-64 macOS keep the `thread_local`s.
- **Lifecycle:** a null slot means "not set up": `threadStateSlow()` pops a recycled `ThreadState` or carves one from `BootstrapAlloc` (never malloc), installs it, and arms a pthread-key destructor. At thread exit the destructor drains a still-live thread cache (GCD workers never pass through alloc8's `pthread_create` trampoline, so they get no `xxthread_cleanup`), clears the slot, and pushes the state on a free list. A later destructor that allocates simply gets a fresh state (pthread re-runs destructors). The free-list lock is handled in the atfork handlers.
- **Caller hint:** `tsd_slot.h` is force-included into every libsmash TU, including alloc8's wrappers, and defines `ALLOC8_CALLER_RA_HOOK`, so the wrappers store the return address in `ThreadState::caller_ra` (first field) instead of alloc8's `thread_local`. Requires an alloc8 with `ALLOC8_HAS_CALLER_RA_HOOK` — the build `#error`s otherwise, because the hint would silently never be written and call-site arenas would collapse.
- **Conflicts:** the slot is unofficial. If the main thread's slot already holds something that is not a `ThreadState` (magic check) when smash's constructor runs — e.g. a second malloc replacement using the same slot — smash aborts with a message rather than corrupt both. There is deliberately no runtime fallback switch: the fast path reads the slot unconditionally. Pick another slot with `-DSMASH_TSD_SLOT_INDEX=N` or disable with `-DSMASH_TSD_SLOT=OFF`. A `thread_local` fallback state exists only for `BootstrapAlloc` exhaustion.
- **Test:** `test_tsd_thread` (1024 short-lived pthreads, GCD workers, an app destructor that allocates during exit, fork with live threads) checks reuse via the exported `smash_tsd_stats()`.
- Measured (M1 Max): malloc+free microbench 41.4 → 24.7 ns/op; jaq 24.3 → 18.0 s; minijinja, stylua, rhai 10–13% faster.

## Key Conventions

- Never allocate from the managed heap inside smash internals — use BootstrapAlloc
- Data pages never contain metadata (bitmap-based free tracking, pointer arrays in thread cache)
- Fine-grained locking: per-slab spinlocks, per-page spinlocks. No global heap lock. 4 arenas reduce slab lock contention.
- `PageLockTable::tryLock()` used for prefetch to avoid deadlock
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
