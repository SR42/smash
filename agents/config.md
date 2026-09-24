# Configuration: constants and environment variables

Full reference, moved from CLAUDE.md.

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
- `SMASH_TRACK_EXTERNAL=1`: Register application-direct `mmap` / `mach_vm_allocate` results so the compressor sees them. Opt-in (see [external-tracking.md](external-tracking.md))
- `SMASH_DEFER_PHASES_MS=N`: Skip Phase 2 (compress) + Phase 3 (monitor) for the first N ms after start. Useful for workloads that establish IPC channels at startup with buffers in smash-managed pages (Firefox sweet spot is 30000)
- `SMASH_WRITE_TRACK=auto|softdirty|uffd|none`: How writes are detected (default `auto`, chosen by testing each mechanism; see "Write tracking" in [defect-history.md](defect-history.md)). `SMASH_SOFTDIRTY=0` still selects Phase 3 PROT_READ monitoring
- `SMASH_NO_MONITOR=1`: Disable Phase 3 (PROT_READ access tracking) entirely. Trades cold-detection accuracy for compatibility with code paths that synchronously check page protection
- `SMASH_VM_GIB=N`: Size of the compressible arena, default 16, ceiling 1 TiB (`kVmMaxBytes`). The arena is a PROT_NONE `MAP_NORESERVE` reservation and its per-page metadata is demand-paged, so an unused arena costs address space, not memory (measured: 16, 64 and 1024 GiB all start at 23.3 MB RSS). If the reservation fails — a 39-bit-VA arm64 kernel, `ulimit -v` — `VmRegion::init` halves the request down to 256 MiB and says so, instead of silently running without compression as it used to (measured: with a 48 GiB address-space limit, `SMASH_VM_GIB=1024` gets a 32 GiB arena and compresses; before, compression was off). The old 64 GiB ceiling was an estimate of metadata cost, not a limit; page indices are 32-bit, so the hard bound is 16 TiB
- `SMASH_NO_DRAIN_WAIT=1`: an allocation that finds no free run gives up immediately instead of waiting for runs that are freed but still queued for decommit (the pre-fix behaviour). For measuring what that wait is worth on a given machine — an absolute fallback rate is not portable across core counts
- `SMASH_NO_RECLAIM_WAIT=1`: every thread that runs the arena out runs its own empty-span reclaim pass concurrently and treats a slab another pass holds as empty, instead of waiting for that pass (the pre-fix behaviour). For measuring the wait; see `test_arena_reclaim_concurrent` in [tests.md](tests.md).
- `SMASH_RUN_CACHE_MIB=N`: per-CPU cache of freed large runs, N MiB per CPU (256 shards, one per CPU up to 256 CPUs; 8 runs each; runs over 64 MiB never cached); 0/unset = off (default). A block freed warm (no page compressed) is parked and reused by the next same-size allocation on that CPU and node, skipping the node arena's lock, the free map, the decommit queue, mprotect/madvise and zero-fill faults. Costs up to (CPUs in use) x N MiB of resident memory while runs are parked — 128 CPUs at N=32 is 4 GiB. See [design-run-cache.md](design-run-cache.md)
- `SMASH_RUN_CACHE_AGE_MS=N`: age after which the decommit thread returns a cached run to the arena (default 50)
- `SMASH_ALIGN_SLOT_SEARCH=1`: Keep an aligned small allocation in its natural size class and search that class's bitmap for an aligned slot, instead of promoting it to a class that is a multiple of the alignment. **Off by default and usually a loss** — the stranded slots make it up to 7× worse, and call-site arena routing keeps plain allocations from filling them. See "Aligned small allocations come from the slab" in [allocation-paths.md](allocation-paths.md)
- `SMASH_ALIGN_SLAB=0`: Restore the old aligned-allocation routing, where any alignment above 16 B goes to `LargeAlloc` and costs a whole page per object. Default ON (slab-served); see "Aligned small allocations come from the slab" in [allocation-paths.md](allocation-paths.md)
- `SMASH_MID_BINS=1`: Serve the (16 KiB, 1 MiB) band from size-binned chunks in the compressible arena instead of one direct `mmap` each. Off by default: it trades ~10% of the requested bytes to size-class rounding for compressibility and much cheaper churn, which only pays for blocks that go cold. See "Mid-size bins" in [allocation-paths.md](allocation-paths.md)
- `SMASH_THP=1`: `madvise(MADV_HUGEPAGE)` the arena. Off by default: it keeps a dense hot set on huge pages (TLB coverage) while the cold set still compresses, but inflates RSS 22× on a sparse touch pattern. See "Hugepages and THP" in [allocation-paths.md](allocation-paths.md)
- `SMASH_LARGE_ALLOC_VM_THRESHOLD=N`: Bytes at or above which a large allocation is placed in the compressible VmRegion instead of a direct mapping (default `kLargeAllocVmThreshold` = 1 MiB; rounded up to a page). **This decides what can be compressed at all** — a block below it is never walked by the compressor and stays resident however cold it gets. Lower it for a workload whose footprint is mid-sized blocks; see "Which allocation sizes are actually compressed" in [allocation-paths.md](allocation-paths.md)
- `SMASH_COLD_TIMEOUT_SEC=N`: Override cold timeout at runtime
- `SMASH_DOUBLE_FREE=off|warn|abort`: Double-free detection, **default `off`**. `warn` reports to stderr (rate-limited, via `safe_snprintf` + `write(2)` — never `fprintf`, which can call malloc and reenter) and drops the second free; `abort` reports then traps. A double free is UB, so this is a debugging aid rather than a safety guarantee: it is not a promise that a detected double free was harmless, only that this one was dropped. No measurable cost when enabled (see `test_double_free_full`)
- `SMASH_NO_EXIT_GUARD=1`: Disable the process-exit guard, so smash's background threads keep the process alive after the application's last thread exits (the pre-fix behaviour — see "Process exit after `pthread_exit()` from main" in [defect-history.md](defect-history.md)). Escape hatch for an application that genuinely relies on a library's threads outliving its own
- `SMASH_COOLING_CLOSE_TICKS=N`: Cooling-page closing threshold (default `getColdTicks()/2`; `0` disables). Slab allocation prefers warm/EMPTY pages over pages whose cold streak ≥ N, and redirects to a decommitted empty span (never a new one) when the front span has only cooling/compressed free slots. Allocator-induced re-heats are counted in the `reheat=` field of the SIGUSR2 stats line (`SMASH_STATS_SIGNAL=1`); the telemetry threshold stays at the default even when closing is disabled, so A/B runs can measure prevented re-heats. Note: objects recycled through the thread cache bypass slab placement entirely — the app's writes into recycled objects re-heat their pages regardless of this policy
- `SMASH_VERY_COLD_TICKS=N`: Override deep-tier cold-tick threshold (ROI model). `9999` disables the deep tier entirely (fast tier only).
- `SMASH_ROI_THRESHOLD=N`: Override ROI cutoff (default 1024)
- `SMASH_FAST_COMP_MBS_HI/LO`, `SMASH_FAST_DECOMP_MBS_HI/LO`: Override fast-tier calibration
- `SMASH_DEEP_COMP_MBS_HI/LO`, `SMASH_DEEP_DECOMP_MBS_HI/LO`: Override deep-tier calibration
- `SMASH_CALIBRATE=always|never`: Force/skip startup calibration
- `SMASH_CALIBRATION_FILE=path`: Cache and reload calibration
- `SMASH_NUMA_COMPRESSOR=0`: Disable the per-node compressor groups. `malloc_numa` pages are then compressed by the default workers into the shared store
- `SMASH_NUMA_WORKERS=N`: Threads per node compressor group (default `kCompressorWorkers`, capped by the node's CPU count and 8)
