# Sanitizers (UBSan)

Moved from CLAUDE.md, which keeps only the build commands and the two rules worth repeating.


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

**The suite is clean under UBSan: 0 findings**, unprivileged (75/75) *and* as root
on claudepc (idle-page tracking exercised), 2026-09-23, after the mid-bin /
NUMA-hint / stdio-retry fixes: 74/75 there, the failure being the pre-existing
`test_arena_reclaim_concurrent` phase B, then 75/75 in both suites once that was
fixed. That round turned up two things that were not UB:

- `test_stdio_after_compression` was **vacuous in this build**: the sanitized
  test binary binds `__fgets_chk`/`__fread_chk` (fortified, which smash does not
  wrap) where the normal build binds `fgets`/`fread`, so its fgets check failed
  against the pre-fix library too. It now calls them through `dlsym`. Check
  `objdump -d … | grep _chk` on any stdio test that must reach a wrapper.
- `test_arena_reclaim_concurrent` phase B failed once in the full run and 3 of
  18 reruns, then 0 of 80. It reproduced on every build under CPU contention,
  the pre-change one included — a real defect in the exhausted hook, since
  fixed; see its entry in [tests.md](tests.md).

Earlier: Two things are worth
repeating on any future sanitizer round:

- **Confirm the reporting mechanism before believing the silence.** "No report
  files" and "reports were never enabled" look identical. A deliberate-UB
  program built and run with the same flags and `UBSAN_OPTIONS` must produce a
  `log_path` file; it does.
- **Run it as root somewhere.** Unprivileged, `test_idle_read_tracking` prints
  `SKIP: idle-page read tracking needs root` and the whole idle-bitmap path
  stays uninstrumented — it was the one test body the local sanitized run never
  executed. As root it runs (`all checks passed`), as do the three
  `SMASH_SOFTDIRTY=0` tests that historically broke only as root. The remaining
  skip anywhere in the suite is dictionary *training*, which is off by design
  (`kDictTrainSamples = 0`).

The **benchmarks** are sanitized too, and were not before: `-DSMASH_BUILD_BENCH=ON`
in the UBSan build dir instruments every bench that links smash internals.
`run_quick_ci.py` passes (bench_rss 67%, bench_sqlite 56.2%) and all 16 built
benches exit 0, with **0 findings** — including the compression-internals ones
(`bench_compression`, `bench_algo_compare`, `bench_dict_experiment`,
`bench_heap_compress` across its four app profiles), which is where codec UB
would surface. Do **not** read those RSS percentages as performance: UBSan slows
everything down, so pages have longer to go cold and the reduction comes out
above the ~46%/~13% unsanitized baselines. The run is a UB check, not a
measurement.

A sanitized **stress** round found nothing either, over mi-stress (both modes),
64-thread aligned-`new` churn, 64 threads of 17–81 KiB blocks with cross-thread
frees (both modes — the 64-byte version is vacuous in large-only, where small
aligned requests pass through to the system allocator), a 1 GiB arena driven
into the exhausted hook, and a cold-set workload that compresses and faults
back in under concurrent allocation: 30–32 k pages compressed per mode,
262144 byte checks, 0 mismatches, 0 nulls, 0 findings. Note that a
churn-only stress proves little about the compressor — `compressed=0` and
`shadow=0` while blocks are freed too fast to go cold — so a stress meant to
cover the compressor has to hold a live set idle. Getting there took two
real fixes, both found *because* UBSan changed what the compiler could prove:

- `test_syscall_efault` failed under UBSan while passing in the normal build.
  That was **a genuine coverage gap, not a sanitizer artifact** — see
  "_FORTIFY_SOURCE and the `__*_chk` aliases" in
  [agents/syscall-compat.md](syscall-compat.md). Instrumentation made the
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
