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
separate build directory: the sanitized library is slower and links libubsan, so
it is neither what you benchmark nor what you ship, and reusing the normal build
directory is how you end up testing a library you did not mean to test.

```bash
cmake -S . -B build-ubsan -DSMASH_SANITIZE=undefined \
      -DALLOC8_DIR=$PWD/build/_deps/alloc8-src
cmake --build build-ubsan -j$(nproc)
cd build-ubsan && UBSAN_OPTIONS=print_stacktrace=1:log_path=/tmp/ubsan ctest
```

`log_path` is worth setting: it gives every test process its own report file, so
a finding inside a test that still *passes* is not lost — ctest only shows the
output of tests that fail. Findings are recoverable by default so the first does
not mask the rest; `UBSAN_OPTIONS=halt_on_error=1` stops at the first instead.

The option also accepts `address`, but ASan is not useful for the preloaded
library: ASan replaces malloc, which is exactly what smash exists to do, so the
two fight over the same job. Use it to instrument a single direct-linked test
binary, not libsmash.

**The suite is clean under UBSan: 42/42 with 0 findings.** Getting there took two
real fixes, both found *because* UBSan changed what the compiler could prove:

- `test_syscall_efault` failed under UBSan while passing in the normal build.
  That was **a genuine coverage gap, not a sanitizer artifact** — see
  "_FORTIFY_SOURCE and the `__*_chk` aliases" in
  [agents/syscall-compat.md](agents/syscall-compat.md). Instrumentation made the
  destination object size knowable, which flipped the test's `read` to
  `__read_chk`, a symbol libsmash did not export, so the EFAULT retry was
  bypassed. Plain `gcc -O2` with no sanitizer does the same to any read into a
  fixed-size buffer. An earlier note here claimed this was not a smash defect;
  that was wrong.
- "load of null pointer of type 'int'" at `smash_heap.h:315` in
  `fullMallocPath()`, plus the matching "store to null pointer" at line 321,
  reported only from direct-linked test binaries — which still **pass**. It was a
  **GCC codegen bug**, not a smash defect, and the fix is `constinit` on
  `g_full_mode_cached`.

  Without `constinit` the compiler treats an `extern thread_local` as possibly
  dynamically initialized and guards each read with its init wrapper. The
  generated sequence is:

  ```
  cmpq $0x0, _ZTHN5smash18g_full_mode_cachedE   # is the INIT WRAPPER null?
  je   .Lskip
  call _ZTHN5smash18g_full_mode_cachedE@plt
  .Lskip:
  mov  %fs:0x0,%rax                            # thread pointer   (no flag write)
  lea  -0x1088(%rax),%rax                      # &g_full_mode_cached (no flag write)
  je   .Lreport                                # branches on the STALE flags above!
  mov  %fs:...,%eax                            # the real, correct load
  ```

  `mov` and `lea` do not touch flags, so the "is this address null?" branch reuses
  ZF from the comparison of the **wrapper**, which is weak-undefined (nothing to
  initialize) and therefore always zero. The handler is called every time with a
  literal `0` (`xor %esi,%esi`) and control then rejoins and performs the correct
  `%fs:` load — a spurious diagnostic over working code.

  Needs `-O2` or above; gcc 13.3 and 14.2 both reproduce, clang 18 is clean.
  `constinit` keeps the variable out of the dynamic-init path, which deletes the
  wrapper guard and the false report together: **14 findings → 0**, and the whole
  sanitized suite now reports **0 findings**. It also removes a
  `cmpq`/`je`/`call` from the fast path of direct-linked binaries.

  All eight extern `thread_local`s in `smash_heap.h` are `constinit` for this
  reason — `g_full_mode_cached`, `g_thread_cache`, the five `g_free_cache_*` and
  `g_arena_stack_base` — which took the init-wrapper references in
  `test_integration` from **204 to 0**, with no wrapper symbols left. Only these
  file-scope `extern` declarations generate wrappers; `inline thread_local` and
  function-local `static thread_local` (e.g. `g_compressor_thread`,
  `tl_numa_worker_`) generate none, so they need nothing.

  `libsmash.so` itself never had the guard — inside the library GCC sees the
  definition and the use in one LTO unit and proves no dynamic init — so this
  cost only ever fell on the direct-linked test binaries.

## Test

```bash
cd build && ctest --output-on-failure
```

All 50 tests must pass; none are registered `DISABLED`. Run the suite **as root** too when touching the compressor: some paths only exist with root (see "Idle-page read tracking"), and an unprivileged run — which is all CI does — never executes them. CI (`.github/workflows/ci.yml`) runs the full suite on `ubuntu-latest` and `macos-latest` on every push and PR. Twenty-eight of the tests are end-to-end under `DYLD_INSERT_LIBRARIES` / `LD_PRELOAD`:

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

  Nothing after the cold wait writes to malloc'd memory. That was originally required because a write into a cold full-mode heap page could be lost (the `test_stdout_buffer_integrity_full` defect, since fixed — see that entry); it is kept because the test should not depend on that path, and an earlier version that read the node CPU lists after the wait saw an empty list in full mode.
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

- `test_malloc_override_{full,large_only}` guard that interposition actually takes effect, and that each mode routes where it claims to. Ported from mimalloc's `test/main-override.c` (`malloc_override_test.cpp`): the original allocates through plain `malloc`/`free`/`strdup`/`realloc` and asserts every pointer lands in mimalloc's own heap via `mi_is_in_heap_region` — i.e. that the calls did not fall through to libc. The allocation sequence is kept unchanged; the ownership predicate becomes smash's exported `smash_arena_for()`. Dropped from the original: `mi_version()` (forced the mimalloc link), `_expand()` (MSVC-only) and `mi_stats_print()`; the cross-API section degenerates to a malloc/free pair because smash has no second API — it *is* malloc.

  Stronger than the original in one way: it asserts **opposite truths per mode**, so it guards large-only *routing* and not just interposition — a small allocation must be smash-owned in full mode and must **not** be under `SMASH_LARGE_ONLY=1` (it passes through to the system allocator), while a 2 MiB block must be owned in both. A test that only ever asserted "owned" would pass vacuously in large-only. **Verified as a real discriminator:** inverting the full-mode sense fails with exit 1, and `smash_arena_for()` returns −1 for stack addresses, raw `mmap` results and `NULL` while returning a real arena id for malloc'd memory. Needs no mimalloc checkout — the port removed every mimalloc dependency.

- `test_double_free_full` guards the opt-in double-free detection (`double_free_test.cpp`, registered with `SMASH_DOUBLE_FREE=warn`). A double free is **undefined behaviour** in C and C++, so detecting it is a debugging aid, not a conformance requirement: glibc aborts (`free(): double free detected in tcache 2`), smash historically absorbed it silently. Detection is therefore **off by default** — no correct program can trigger it, so no correct program should pay for it.

  There are two detection points, and which one fires matters. A plain `free(p); free(p);` never reaches the slab: both pointers sit in the per-thread cache lane, so `ThreadCache::deallocate` scans the lane (bounded by `kPerLaneDepth`) when the policy is enabled. `Span::deallocate` tests the bitmap bit (1 = free) before setting it, catching a repeat free that survives a lane drain and a free of an interior pointer that maps onto an already-free slot; that check is essentially free because the function already loads and stores that word. On detection the second free is **dropped** — not politeness but correctness, since falling through would also run `--allocated_count`, underflowing it and corrupting the full/empty transitions in `Slab::deallocateOne`.

  The test forks the offending child (so `abort` cannot take the test process down) and asserts all three policies: `warn` reports, drops, and leaves the allocator consistent (4096 same-class allocations, no duplicate pointers); unset is silent; `abort` dies by signal. **Verified as a real discriminator by source mutation** — neutering the lane scan makes it fail with the intended message, restoring it makes it pass. An env-var-based discriminator does *not* work here: the test sets `SMASH_DOUBLE_FREE` per child, overriding anything passed from outside.

  **Measured cost of enabling it** (20M alloc/free pairs of one size class, so nearly every free hits the lane, runs interleaved to expose drift): `off` 15.15 / 15.24 ns/pair, `warn` 14.76 / 14.25 ns/pair — no measurable cost, with the two `off` runs agreeing to 0.6%. mi-stress is unchanged within noise (17 s → 18 s full, 7 s → 7 s large-only).

- `test_external_mapping` (uses a live compressor) exercises the `SMASH_TRACK_EXTERNAL=1` registration path. The ctest invocation sets the env var; without it the test would silently no-op (registration path gated).
- `test_malloc_compression` and `test_large_only_compression` are the **same executable** (`mode_compression_test.cpp`) run twice, differing only in `SMASH_LARGE_ONLY` (`0` = full, `1` = large-only — the production-supported config). The mode is read from the env at runtime, never a compile flag, so the two modes execute byte-identical assertions and cannot drift apart in coverage. The body allocates large chunks (≥ 1 MiB, so they clear `kLargeAllocVmThreshold` and enter the compressible VmRegion in **both** modes) interleaved with small chunks (≤ 16 KiB — smash slab in full mode, system passthrough in large-only). It sleeps past `SMASH_COLD_TIMEOUT_SEC`, sends `SIGUSR2`, parses `compressed=N` from the stats line, asserts `N > 0`, then verifies a byte-exact read-back of **both** classes — proving full-mode slab integrity / large-only passthrough is never corrupted, and that fault-decompress restores the compressed large chunks. (Registered via the `add_mode_compression_test(name, large_only)` helper in `tests/CMakeLists.txt`.)
- `test_compression_ratio` (in-process, direct-linked, no compressor thread) compresses a realistically-compressible page with LZ4 and zstd and asserts the achieved ratio against the paper's RQ2 figures (evaluation.tex: LZ4 4.7–12.3×, zstd-1 8.8–20.4×). Two-tier: a hard floor that fails the test (2.5× LZ4 / 4× zstd — catches "stored uncompressed" regressions) plus a WARN if it falls short of the paper's best-case number. Also asserts byte-exact roundtrip. Unlike `test_malloc_compression` (which only checks compression *happened*), this pins the codec *ratio*.
- `test_fault_cycle` / `test_fault_cycle_eager_reclaim` — the real fault path, in-process: compress a page, read it, and require a genuine SIGSEGV/SIGBUS that the handler decompresses. One binary, run with default (deferred) reclaim and with `SMASH_DEFERRED_RECLAIM=0`. Each cycle ticks until the page is **really inaccessible** (no read permission in the process's own mapping table: `/proc/self/maps` on Linux, `mach_vm_region` on macOS), then asserts the read went through the fault handler (a fault counter), the data is byte-exact, and the page is live again.

  It was disabled from June to September 2026 as "signal-delivery timing varies across CI environments". There was no timing involved. It assumed three ticks leave a page protected, and neither mode does: under eager reclaim a COMPRESSED page stays readable until the deferred-madvise sweeper protects it, so the read succeeded without a fault, the page stayed COMPRESSED, and the old "state after read" check failed **20/20** — reproduced by rebuilding the commit just before the disable. The data was always correct, and a write in that window does fault and is kept. Under deferred reclaim the page is COMPRESSED_SHADOW (still mapped read/write) after three ticks, so the old test's read never faulted and **the test passed without exercising the fault handler at all**. **Verified as a real discriminator:** with the fault handler made to restore one corrupted byte, the old test still passed in the default mode; the new one fails in both modes (`data mismatch at offset 7`). Measured passing 130/130 before re-enabling, idle, 2× CPU-oversubscribed, pinned to one busy core, and on a 2-CPU host as user and root.

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

### Thread-cache reclaim (symbol-version guard)

`test_thread_cache_reclaim_{full,tls_only,large_only}` — one body (`thread_cache_reclaim_test.cpp`) creating and joining 1500 threads one at a time, asserting RSS grows less than 8 KiB per thread. A `ThreadCache` is 92 KiB, so a leaked one is an order of magnitude over that line.

Reclaiming the cache used to depend entirely on alloc8's `pthread_create` interposer calling `xxthread_cleanup`, and on **glibc ≥ 2.34 that interposer never ran for application threads**: glibc 2.34 merged libpthread into libc and moved the default version of `pthread_create`/`pthread_exit` to `GLIBC_2.34`, while libsmash exported only the base `GLIBC_2.2.5` definition, so every caller bound straight to libc. Measured with atomic counters: of 200 application threads **16 were wrapped, and all 16 were smash's own** compressor threads (whose calls resolve inside libsmash under `-Bsymbolic`). RSS grew **91.2 KiB per thread**, linear over 2000 threads, and 14000 threads segfaulted once `BootstrapAlloc` was exhausted and `newThreadCache()` placement-new'd onto null. Forcing a test binary to request `pthread_create@GLIBC_2.2.5` flipped it to 216 wrapped / 199 pool hits / 0.3 KiB per thread, which is what identified the version as the cause rather than the pool.

Two independent mechanisms now cover it, and the three registrations isolate them: `_full` exercises the versioned alias path, `_tls_only` sets `SMASH_NO_THREAD_HOOKS=1` so `threadInit`/`threadCleanup` are no-ops and only the `pthread_key_t` destructor can reclaim, and `_large_only` is the control where small allocations never create a cache. Measured after the fix: 0.3, 0.2 and 0.2 KiB per thread respectively, with RSS flat from thread 500 to thread 14000.

A separate hang on the same idiom — an application whose *main* thread retires with `pthread_exit(NULL)` — was found while testing this and is fixed by the process-exit guard described in the next section.

### Compressor shutdown race (node-worker park loop)

`test_shutdown_race_{full,large_only}` spawn 200 very short-lived preloaded
children and assert every one exits. `CompressorThread::stop()` joins the NUMA
node-group workers at process exit, and a worker could be asleep on a futex that
nothing would ever wake again, so the `pthread_join` inside `__cxa_finalize`
waited forever and the process never exited.

`stopNodeThreads()` stores `numa_exit_`, bumps `numa_work_gen_` and wakes, in
that order. The worker's park loop sampled the generation once at thread start
and only consulted `numa_exit_` *after* observing a generation **change**. A
worker whose first instructions ran after all three stop operations had already
happened therefore sampled the already-bumped generation into `last`, found
`gen == last`, and parked forever. The window between the load and the
`futexWait` is already safe — the value changed, so `FUTEX_WAIT` returns EAGAIN
— it is the thread-start-after-stop ordering that was not. The fix is to check
`numa_exit_` **before** parking.

It needs startup and teardown to nearly coincide, so it hit short-lived
programs. Measured on `arena_routing_test` (0.02 s): **7 hangs in 300 runs
(2.3%) before the fix, 0 in 300 after**. It first surfaced as an intermittent
`test_arena_routing` timeout — a poor guard, since that test only catches the
race by accident, which is why `test_shutdown_race_*` provokes it directly. A
gdb backtrace of a caught hang showed main in `pthread_join` under
`__cxa_finalize` → `stop()` → `stopNodeThreads()`, with a node worker asleep in
`futexWait`. Node groups only exist on hosts with ≥ 2 memory nodes, which is why
single-node CI runners never saw it.

**Verified as a real discriminator:** rebuilt against the pre-fix park loop,
`test_shutdown_race_full` and `test_shutdown_race_large_only` both fail (0% of 2
passed, ~11 s each as they hit a hung child); with the fix restored both pass in
~3.5 s.

### Process exit after `pthread_exit()` from main

`test_main_pthread_exit_{full,large_only}` — a forked child creates a worker thread and then main retires with `pthread_exit(NULL)`; the parent asserts the child exits **0 within 30 s** *and* that the child's own `atexit` handler ran. The handler is the sharper assertion: a "fix" that ended the process without going through glibc's normal last-thread path would still exit 0 while silently skipping the application's exit handlers.

glibc ends a process when its thread count reaches zero — the last thread to exit calls `exit(0)`, which is why the no-preload control runs `atexit` handlers and exits 0. `pthread_exit()` from main does **not** call `exit()`: it retires main and leaves the process alive until that count drops. Every smash background thread that never exits kept it above zero, so the process hung forever. Measured: a zombie thread-group leader beside 16 sleeping smash threads, needing SIGKILL, with the application's `atexit` handler never running. Neither of smash's existing shutdown hooks helps, because both hang off `exit()` — the `atexit` handler in `compressor_thread.h` and `destructor(101)` in `smash_heap.cpp`.

`src/util/exit_guard.h` fixes it by **removing smash's threads once the application has none left and letting glibc's own last-thread logic run**, so status, `atexit` handlers and static destructors are stock rather than reproduced here. Calling `exit()` from smash would mean choosing a status and running the application's handlers on a compressor thread.

- **Detection is a `/proc/self/task` scan**, not a counter: any task that is not the caller, not a registered smash thread, and not a zombie (the retired main thread lingers as one, and is counted in `Threads:`) is a live application thread. A counter of wrapped threads would inherit the failure mode fixed in the previous section, where interposition silently stopped wrapping application threads.
- **Triggered from a second `pthread_key_t`** armed for every thread whether or not it allocates, so the trigger works in large-only mode too. Before main retires it costs one relaxed atomic load per thread exit and scans nothing; the compressor re-checks once per tick only after main has retired, covering an application thread smash never saw start.
- **Every smash-created thread registers** (`exit_guard::ThreadRegistration`, RAII so each return path releases the slot): the compressor coordinator, helpers and node workers, plus VmRegion's lazily started decommit thread. The registry is cleared in the `atfork` child handler, since fork clones only the calling thread.
- **Shutdown stops threads only** — the compress store stays intact, so `atexit` handlers and static destructors running afterwards still fault in compressed pages normally (decompression runs on the faulting thread and needs no compressor thread). `CompressorThread::stop()` and `stopDecommitThread()` skip joining the calling thread, because the coordinator's fallback path can run the shutdown from inside the coordinator itself.

Measured after the fix, worker outliving main: full mode 218 ms and large-only 219 ms, against a 205 ms no-preload control — both exit 0 with handlers run, where both hung before. The main-retires-alone shape (500 threads, then `pthread_exit` from main) goes from a hang to 59 ms.

**Verified as a real discriminator:** with `SMASH_NO_EXIT_GUARD=1` the repro hangs again and has to be SIGKILLed, and the test itself fails on its 30 s child timeout; with the guard on, the same test passes in 220 ms. The isolation run that first identified the cause is the same switch in reverse — `SMASH_NO_COMPRESSOR=1` made the pre-fix binary exit 0, which pinned the hang on smash's threads rather than on anything else the library does.

### Idle-page read tracking (root only)

`test_idle_read_tracking` (`idle_read_tracking_test.cpp`, in-process, `SMASH_COLD_TICKS=2 SMASH_SOFTDIRTY=0`) guards the one read-detection path that only exists as root. With `CAP_SYS_ADMIN`, `/proc/self/pagemap` exposes real PFNs, so `ensureIdleReadFds()` turns on idle-page tracking: at escalation a monitored page is marked idle through `/sys/kernel/mm/page_idle/bitmap` instead of being armed PROT_NONE, and phase 1 treats "no longer idle" as "read", with no fault paid. Unprivileged, the PFN is redacted and the path never activates — so the whole suite, and CI, had never run it.

First reported as three aarch64 failures — `test_compression` and `test_prefetch` (`page X should be COMPRESSED, got 4`, i.e. stuck in `ACTIVE_MONITORING`) and `test_syscall_efault` (`the buffer page never left RSS within 30s`). **It is not arch-specific: it is root.** On x86-64 the same three failed with the same messages as root, passed as a normal user, and passed as root with `SMASH_IDLE_READ_TRACK=0`. They are exactly the three tests that pin `SMASH_SOFTDIRTY=0`; with soft-dirty on, pages never enter `ACTIVE_MONITORING`, where the idle check lives. Two defects:

- **The idle bit was trusted for pages that were never marked.** The check was gated on `idle_read_ok_ == 1 && st == ACTIVE_MONITORING`, not on whether this page was armed. An unmarked page reads "not idle", so every monitored page counted as read at its first check, reset its cold count, and never reached escalation — nothing compressed. This was the actual cause of the three failures: even pages already on the LRU never compressed before the fix.
- **A failed mark was trusted.** The kernel silently ignores an idle mark on a page that is not on the LRU. The shared zero page never is (a page the application has only read), and freshly written pages are not until the per-CPU LRU-add batch drains. Measured on 6.8 as root: 0/8 freshly written pages take the mark, 8/8 after 256 MiB of other faults. Such a page reads "not idle" forever. A read that *was* counted had the same effect: the bit stays clear until the page is armed again, and it was only re-armed at `cold_count == floor`, which it could no longer reach.

The fix is a per-page `idle_armed_` flag, allocated only when the probe succeeds (unprivileged runs pay nothing): escalation marks the page, **reads the bit back**, and sets the flag only if the mark took — otherwise it falls back to the PROT_NONE arm. Phase 1 consults the idle bit only for flagged pages and clears the flag once it reports a read; entering monitoring clears it too.

The test asserts three scenarios: written pages use the idle arm (still PROT_READ, never PROT_NONE) and compress at tick 3; a read after the arm is detected without a fault (nothing compressed at tick 3) and the pages still compress once quiet (tick 6); zero-page-backed pages cannot take the mark, fall back to PROT_NONE, and compress at tick 3. **Verified as a real discriminator:** built with identical flags against the pre-fix compressor it fails 4 checks as root — no page compresses within 16 ticks in any scenario, and the zero page never falls back. It skips when not root or when the kernel lacks `CONFIG_IDLE_PAGE_TRACKING`, so it only guards anything when the suite is run as root. Measured as root after the fix: full suite 47/47.

### Hugepages, VMA pressure, and buffered-stdio integrity

Follow-ups for the case where the application allocates hugepages with `mmap`. All Linux-only; they skip cleanly elsewhere.

- `test_hugepage_mmap` — application hugepage mappings under the mmap interposer. The interposer's filter (`isTrackableMapping()` in `src/linux_syscall_wrappers.cpp`) takes anonymous writable mappings and **skips `MAP_HUGETLB`**. It did not always: `registerLinuxExternalRange()` walks a mapping in 4 KiB strides, so under `SMASH_TRACK_EXTERNAL=1` every 4 KiB slice of an application hugepage mapping used to be registered as an external page and marked ACTIVE. hugetlb VMAs require huge-page-aligned addresses and lengths, so *every* page operation smash then performed on them was rejected — verified directly on a host with hugepages reserved: `mprotect` → EINVAL, `madvise(MADV_DONTNEED)` → EINVAL, `mmap(MAP_FIXED|MAP_ANON)` → ENOMEM. The test asserts that the app's `mmap(MAP_HUGETLB)` still succeeds, that its data stays byte-exact (smash's restore path is `remapPages()`, i.e. `MAP_FIXED|MAP_ANON`, which would shred a hugetlb VMA if the kernel allowed it), and that allocation keeps working. It also moves one mapping with a same-size `mremap` to a huge-aligned address (growing a hugetlb mapping is EINVAL; kernels that refuse the move too get a NOTE and skip it). Skips when the host has fewer than 8 hugepages free (CI runners have none; `sysctl -w vm.nr_hugepages=N` to exercise it).
- `test_hugepage_no_mprotect_storm` — same binary with `SMASH_TEST_STRICT_HUGEPAGE=1`, which turns wasted page operations on hugetlb pages from a WARN into a failure; this is the regression guard for the skip. Nothing records that a page is unmanageable, so a registered hugetlb slice is retried every tick forever: before the fix a 16 MiB mapping produced **8192 failing page operations in six seconds** and kept going, while the slices held VmRegion's `kTrackMaxPages` = 128 Ki-page external budget (512 MiB at 4 KiB pages) despite being permanently uncompressible. Now it produces 0.

  The skip needs a second half. `mremap`'s flags cannot say `MAP_HUGETLB`, and the `mremap` interposer used to register the destination of any move or growth unconditionally, so moving a hugetlb mapping re-registered it. It now tracks a moved or grown range only if the source was tracked (`vm->contains(old_addr)`), which carries the `mmap` filter's verdict across the remap. **Verified as a real discriminator for each half:** with the `MAP_HUGETLB` skip removed the strict test sees 8192 failing operations and fails; with only the `mremap` gate removed it sees 1024 — from the single moved 2 MiB mapping — and fails; with both it sees 0. Ordinary anonymous mappings grown by `mremap` are still tracked and compressed (both halves leave RSS, as before). A file-backed mapping grown by `mremap` stayed resident *before* the gate as well, so the gate is not claimed to fix a demonstrated file-mapping problem — only to keep the filter's verdict.
- `test_vma_pressure` — crossing `vm.max_map_count` makes `mmap` return ENOMEM, which makes `malloc` return nullptr, which makes `operator new` throw `std::bad_alloc`; `compressor_thread.h` also records the resulting restore livelock ("VMAs pinned at the 65531 cap"). smash mitigates this with soft-dirty tracking (default ON), chunked Phase-3 `mprotect`, and a mid-loop VMA budget in `reclaimShadowRange()`, but nothing pinned them in place. Drives a 576 MiB compress-and-refault workload including a THP-hinted region (a partial `mprotect` inside a transparent hugepage splits it, up to 512 VMAs per 2 MiB) and asserts the VMA count stays far below the cap. Measured peak on x86-64 with defaults: 218 VMAs, against a budget of 8192 and a cap of 1048576 — a naive one-VMA-per-page implementation would produce ~131072.
- `test_stdout_buffer_integrity_large_only` / `test_stdout_buffer_integrity_full` — glibc allocates stdout's 4 KiB buffer with `malloc`, so in full mode it is an ordinary slab object. An application writing in bursts separated by quiet stretches lets that page go cold and be compressed while it still holds unflushed bytes. **Fixed**; both registrations are enabled and this is the regression guard.

  The cause was not compression losing writes — it was **shutdown reinstating a stale snapshot over live data**. `drainAllForShutdown()` handled `COMPRESSED` and `COMPRESSED_SHADOW` identically: decompress the blob, `restorePageContents()` over the page. But a shadow page is still mapped PROT_RW and holds the application's *live* bytes; nothing faults on a write to it, so its blob goes stale silently and only `reclaimShadowRange()`'s `content_matches` check ever notices — and a page that is never reclaimed is never checked. At exit the drain therefore reverted the buffer to a snapshot taken five marker lines earlier: 228 of 513 bytes flushed as NUL, 4 of 9 marker lines lost, *identically on every run*. The drain now mirrors `handleFault`'s `COMPRESSED_SHADOW` case and `reclaimShadowRange()`'s mismatch branch — keep the page, drop the blob — and deliberately has no `remapPages()` fallback there, which would discard the very bytes being preserved.

  **The knobs that look like they should matter do not, and that is a trap worth remembering**: `SMASH_SOFTDIRTY`, `SMASH_DEFER_MADVISE`, `SMASH_DEFER_MADVISE_TICKS`, `SMASH_NO_DECOMMIT`, `SMASH_MADV_FREE`, `SMASH_NO_MONITOR`, `SMASH_FIXAV` and `SMASH_P2_CHUNK` all produce *byte-identical* output, because every one of them steers an arm of the non-deferred `else` in `compressPage()` while `SMASH_DEFERRED_RECLAIM` (default ON, defined in `include/smash/config.h`) takes the `if`. Byte-identical results across supposedly different configs mean the flag is inert, not that the suspect is innocent. The flags that do discriminate are `SMASH_DEFERRED_RECLAIM=0` and `SMASH_DEFERRED_RECLAIM_DELAY`; `DELAY=100000` (never reclaim, so the page keeps its *oldest* blob) made it predictably worse — 399 NUL with the boundary at offset 114 rather than 228 at 285.

  **Verified as a real discriminator:** reverting only the drain's shadow branch, rebuilt in a scratch tree with identical cmake flags alongside a fixed control built the same way, reproduces 228 NUL at offset 285 on every run and fails the test with 2 failures, while the control is byte-perfect.

  **Independently reproduced on unmodified third-party code**, which is the stronger evidence: mimalloc's `test-stress` (the `USE_STD_MALLOC=1` build) with stdout redirected to a file, run under the pre-fix library with `SMASH_DEFERRED_RECLAIM_DELAY=100000`, emits 177 bytes of which **53 are NUL** from offset 124 to EOF, identically on every run; the post-fix library is byte-identical to a no-libsmash control. The surviving prefix is the tell — it ends with mi-stress's one-line banner **printed twice**. The blob snapshotted the buffer while it held the banner; glibc then flushed, rewound `_IO_write_ptr` and wrote the progress lines; the drain reinstated the old page contents under the newer write pointer, so the banner's bytes reappear where the progress lines belong and the file length (which lives in the `FILE` struct, never reverted) stays correct. Ordinary mi-stress at the default reclaim delay is **not** affected — the page has to still be `COMPRESSED_SHADOW` at exit — which is why this went unnoticed while the suite was green. The corruption is deterministic, so a byte-identical reproduction is the right bar here; note that `strace` perturbs it away entirely. It re-execs itself rather than plain-forking (fork clones only the calling thread, so a forked child has no compressor thread and never compresses anything) and inspects the child's output file after it exits, because the damage only appears at the exit-time flush.

Every end-to-end test in this group calls `setvbuf(stdout, nullptr, _IONBF, 0)` on entry. That was originally forced by the defect above — full mode would zero the tail of a test's own buffered output and eat its verdict lines while it still exited 0 — and is kept as hygiene now that it is fixed, so a regression on this path can never silently swallow a failure message.

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
- A repeated free is only visible in two places, and the obvious one is the wrong one. `free(p); free(p);` never reaches the slab bitmap — both pointers land in the `ThreadCache` lane — so lane-scanning is what catches the common case, and the bitmap check (`Span::deallocate`, 1 = free) catches only what survives a drain. Any future hardening that inspects just one of the two will silently miss half the cases. Whatever detects it must **drop** the second free rather than apply it, or `--allocated_count` underflows and corrupts `Slab::deallocateOne`'s list transitions.
- Never export both `name@@VER` and `name@VER` from libsmash for the same interposed function. A versioned reference resolves to the exact-version match — the `@` alias — and if that alias is a thin `jmp name@plt` forwarder, the GOT slot points back at it: a jump to itself that spins at 100% CPU. List only the `_NN` alias in the version script and let the bare name fall under `local: *`. See "_FORTIFY_SOURCE and the `__*_chk` aliases" in [agents/syscall-compat.md](agents/syscall-compat.md).
- The generated version script must be a declared `LINK_DEPENDS` of the `smash` and `smash_noopt` targets. CMake infers no link dependency from a `--version-script=` linker flag, so without it a change to `smash_version_script.map.in` regenerates the map and relinks **nothing** — and the next test run silently measures the previous library. (`configure_file` only rewrites the generated map when its content changes, so `touch`ing the `.in` correctly does nothing.)
- Thread hooks must be exported at the version the caller binds. `pthread_create`/`pthread_exit` are exported **twice** — at the base version and at `GLIBC_2.34` (`.symver` aliases in `linux_syscall_wrappers.cpp`, both names listed in the version script). glibc 2.34 merged libpthread into libc and made `GLIBC_2.34` the default for these, so a base-only export means alloc8's interposer never sees an application thread and every thread leaks its 92 KiB `ThreadCache`. The same rule as for versioned syscall wrappers (see [agents/syscall-compat.md](agents/syscall-compat.md)); it applies to thread hooks too, and it failed silently for both.
- A thread cache must be reclaimable without the interposer. `getOrCreateThreadCache()` arms a `pthread_key_t` whose destructor drains and pools the cache on any thread exit, however the thread was created; `threadCleanup()` clears that key's value so a wrapped thread cannot return the same cache twice, and the destructor re-checks `currentThreadCache()` before acting. Keep both guards if you touch either path.
- `newThreadCache()` returns nullptr when BootstrapAlloc is exhausted, and every `getOrCreateThreadCache()` caller handles it: `mallocSlow` falls back to the system allocator, and both `freeSlow` paths return the object straight to its slab via `Slab::deallocate(span, ptr)` rather than dropping it.
- Every file-scope `extern thread_local` must be declared **and** defined `constinit`. Without it the compiler treats the variable as possibly dynamically initialized and guards each read with a `cmpq`/`je`/`call` on its `_ZTH...` init wrapper — on the malloc and free fast paths — and that guard also feeds a GCC `-fsanitize=null` bug that reports a spurious null load over correct code (see "Sanitizers"). `constinit` is a compile-time assertion, so a non-constant initializer fails the build rather than silently reinstating the guard.
- A thread that parks on a futex must check its exit flag **before** parking, not only after it observes a work-generation change. The publisher sets the flag, bumps the generation and wakes, in that order; a thread whose first instructions run after all three has nothing left to observe and sleeps forever, taking the joiner down with it. Re-checking after a generation change is not enough, because a thread that starts late never sees one. This cost a 2.3% hang of short-lived processes (see "Compressor shutdown race").
- Never trust an idle-bitmap mark you have not read back, and never read the idle bit of a page you did not mark. The kernel silently drops marks on pages not on the LRU (the zero page, freshly faulted pages), and an unmarked page reads "not idle", which the read-detection code interprets as "read". Either mistake keeps pages hot forever, and only as root — see "Idle-page read tracking".
- A smash thread must never be what keeps a process alive. glibc ends a process when its thread count hits zero, so any thread smash creates has to register with `exit_guard::ThreadRegistration` (`util/exit_guard.h`) and be stopped by `SmashHeap::stopAllSmashThreads()`. A new background thread that skips either one re-introduces the `pthread_exit()`-from-main hang, and does so silently — the process just never exits. Any stop path must also skip joining the calling thread.
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
- `SMASH_DOUBLE_FREE=off|warn|abort`: Double-free detection, **default `off`**. `warn` reports to stderr (rate-limited, via `safe_snprintf` + `write(2)` — never `fprintf`, which can call malloc and reenter) and drops the second free; `abort` reports then traps. A double free is UB, so this is a debugging aid rather than a safety guarantee: it is not a promise that a detected double free was harmless, only that this one was dropped. No measurable cost when enabled (see `test_double_free_full`)
- `SMASH_NO_EXIT_GUARD=1`: Disable the process-exit guard, so smash's background threads keep the process alive after the application's last thread exits (the pre-fix behaviour — see "Process exit after `pthread_exit()` from main"). Escape hatch for an application that genuinely relies on a library's threads outliving its own
- `SMASH_COOLING_CLOSE_TICKS=N`: Cooling-page closing threshold (default `getColdTicks()/2`; `0` disables). Slab allocation prefers warm/EMPTY pages over pages whose cold streak ≥ N, and redirects to a decommitted empty span (never a new one) when the front span has only cooling/compressed free slots. Allocator-induced re-heats are counted in the `reheat=` field of the SIGUSR2 stats line (`SMASH_STATS_SIGNAL=1`); the telemetry threshold stays at the default even when closing is disabled, so A/B runs can measure prevented re-heats. Note: objects recycled through the thread cache bypass slab placement entirely — the app's writes into recycled objects re-heat their pages regardless of this policy
- `SMASH_VERY_COLD_TICKS=N`: Override deep-tier cold-tick threshold (ROI model). `9999` disables the deep tier entirely (fast tier only).
- `SMASH_ROI_THRESHOLD=N`: Override ROI cutoff (default 1024)
- `SMASH_FAST_COMP_MBS_HI/LO`, `SMASH_FAST_DECOMP_MBS_HI/LO`: Override fast-tier calibration
- `SMASH_DEEP_COMP_MBS_HI/LO`, `SMASH_DEEP_DECOMP_MBS_HI/LO`: Override deep-tier calibration
- `SMASH_CALIBRATE=always|never`: Force/skip startup calibration
- `SMASH_CALIBRATION_FILE=path`: Cache and reload calibration
- `SMASH_NUMA_COMPRESSOR=0`: Disable the per-node compressor groups. `malloc_numa` pages are then compressed by the default workers into the shared store
- `SMASH_NUMA_WORKERS=N`: Threads per node compressor group (default `kCompressorWorkers`, capped by the node's CPU count and 8)
