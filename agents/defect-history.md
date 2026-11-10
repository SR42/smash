# Fixed defects and their guards

Investigations behind fixed defects: symptom, cause, fix, measurements, and the test that guards each. Moved from CLAUDE.md; the rule each one taught is in Key Conventions there, with the long form in [conventions.md](conventions.md).

## BootstrapAlloc growth (self-deadlock guard)

`test_bootstrap`'s `testGrowPastFirstRegion` allocates 1 MiB at a time until it passes the 64 MiB initial region, forcing `addRegion()`, and touches both ends of every chunk so the new region is proven writable. It is single-threaded and takes milliseconds.

It guards the deadlock described in Key Conventions ([conventions.md](conventions.md)): `expandAndAllocate()` holds `expand_lock_` and calls `tryAllocFrom()`, which used to call `ensureCommitted()`, which took the same non-recursive spinlock to `mprotect` the next pages. The first allocation that had to grow the arena spun forever holding the lock, and every other thread piled up behind it — 25 threads in `R` state at ~730% CPU, 23 of them innermost in `spinRelax`.

Found with mimalloc's `test-stress` built with `USE_STD_MALLOC=1` (the only test in that suite that exercises smash rather than mimalloc's own `mi_*` API), run under a plain `LD_PRELOAD` with no smash environment — i.e. the default, experimental full mode. The trigger is total allocation, not concurrency: a `ThreadCache` is 92 KiB and goes through `allocate()`, so ~728 of them exhaust the initial region. 16 threads × 20 iterations passed; 4 × 250, 8 × 120, 16 × 60 and 32 × 50 all hung, and all four complete in 1–16 s after the fix.

**Verified as a real discriminator:** reverted to the pre-fix `bootstrap_alloc.h`, this test hangs (killed at 45 s); restored, it passes. The RSS-budget tests (`test_aligned_alloc_footprint_{full,large_only}`, `test_vm_exhaustion`, `test_vma_pressure`) all still pass, which is the check that mapping regions read/write up front did not cost footprint — a budget check, not a direct before/after RSS measurement.

## Thread-cache reclaim (symbol-version guard)

`test_thread_cache_reclaim_{full,tls_only,large_only}` — one body (`thread_cache_reclaim_test.cpp`) creating and joining 1500 threads one at a time, asserting RSS grows less than 8 KiB per thread. A `ThreadCache` is 92 KiB, so a leaked one is an order of magnitude over that line.

Reclaiming the cache used to depend entirely on alloc8's `pthread_create` interposer calling `xxthread_cleanup`, and on **glibc ≥ 2.34 that interposer never ran for application threads**: glibc 2.34 merged libpthread into libc and moved the default version of `pthread_create`/`pthread_exit` to `GLIBC_2.34`, while libsmash exported only the base `GLIBC_2.2.5` definition, so every caller bound straight to libc. Measured with atomic counters: of 200 application threads **16 were wrapped, and all 16 were smash's own** compressor threads (whose calls resolve inside libsmash under `-Bsymbolic`). RSS grew **91.2 KiB per thread**, linear over 2000 threads, and 14000 threads segfaulted once `BootstrapAlloc` was exhausted and `newThreadCache()` placement-new'd onto null. Forcing a test binary to request `pthread_create@GLIBC_2.2.5` flipped it to 216 wrapped / 199 pool hits / 0.3 KiB per thread, which is what identified the version as the cause rather than the pool.

Two independent mechanisms now cover it, and the three registrations isolate them: `_full` exercises the versioned alias path, `_tls_only` sets `SMASH_NO_THREAD_HOOKS=1` so `threadInit`/`threadCleanup` are no-ops and only the `pthread_key_t` destructor can reclaim, and `_large_only` is the control where small allocations never create a cache. Measured after the fix: 0.3, 0.2 and 0.2 KiB per thread respectively, with RSS flat from thread 500 to thread 14000.

A separate hang on the same idiom — an application whose *main* thread retires with `pthread_exit(NULL)` — was found while testing this and is fixed by the process-exit guard described in "Process exit after `pthread_exit()` from main" below.

## Compressor shutdown race (node-worker park loop)

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

## Process exit after `pthread_exit()` from main

`test_main_pthread_exit_{full,large_only}` — a forked child creates a worker thread and then main retires with `pthread_exit(NULL)`; the parent asserts the child exits **0 within 30 s** *and* that the child's own `atexit` handler ran. The handler is the sharper assertion: a "fix" that ended the process without going through glibc's normal last-thread path would still exit 0 while silently skipping the application's exit handlers.

glibc ends a process when its thread count reaches zero — the last thread to exit calls `exit(0)`, which is why the no-preload control runs `atexit` handlers and exits 0. `pthread_exit()` from main does **not** call `exit()`: it retires main and leaves the process alive until that count drops. Every smash background thread that never exits kept it above zero, so the process hung forever. Measured: a zombie thread-group leader beside 16 sleeping smash threads, needing SIGKILL, with the application's `atexit` handler never running. Neither of smash's existing shutdown hooks helps, because both hang off `exit()` — the `atexit` handler in `compressor_thread.h` and `destructor(101)` in `smash_heap.cpp`.

`src/util/exit_guard.h` fixes it by **removing smash's threads once the application has none left and letting glibc's own last-thread logic run**, so status, `atexit` handlers and static destructors are stock rather than reproduced here. Calling `exit()` from smash would mean choosing a status and running the application's handlers on a compressor thread.

- **Detection is a `/proc/self/task` scan**, not a counter: any task that is not the caller, not a registered smash thread, and not a zombie (the retired main thread lingers as one, and is counted in `Threads:`) is a live application thread. A counter of wrapped threads would inherit the failure mode fixed in the previous section, where interposition silently stopped wrapping application threads.
- **Triggered from a second `pthread_key_t`** armed for every thread whether or not it allocates, so the trigger works in large-only mode too. Before main retires it costs one relaxed atomic load per thread exit and scans nothing; the compressor re-checks once per tick only after main has retired, covering an application thread smash never saw start.
- **Every smash-created thread registers** (`exit_guard::ThreadRegistration`, RAII so each return path releases the slot): the compressor coordinator, helpers and node workers, plus VmRegion's lazily started decommit thread. The registry is cleared in the `atfork` child handler, since fork clones only the calling thread.
- **Shutdown stops threads only** — the compress store stays intact, so `atexit` handlers and static destructors running afterwards still fault in compressed pages normally (decompression runs on the faulting thread and needs no compressor thread). `CompressorThread::stop()` and `stopDecommitThread()` skip joining the calling thread, because the coordinator's fallback path can run the shutdown from inside the coordinator itself.

Measured after the fix, worker outliving main: full mode 218 ms and large-only 219 ms, against a 205 ms no-preload control — both exit 0 with handlers run, where both hung before. The main-retires-alone shape (500 threads, then `pthread_exit` from main) goes from a hang to 59 ms.

**Verified as a real discriminator:** with `SMASH_NO_EXIT_GUARD=1` the repro hangs again and has to be SIGKILLed, and the test itself fails on its 30 s child timeout; with the guard on, the same test passes in 220 ms. The isolation run that first identified the cause is the same switch in reverse — `SMASH_NO_COMPRESSOR=1` made the pre-fix binary exit 0, which pinned the hang on smash's threads rather than on anything else the library does.

## Memory must still work after the compressor shuts down

`CompressorThread::stop()` drains compressed pages and then **uninstalls the
SIGSEGV/SIGBUS handler**, deliberately, so a real application heap bug surfaces
normally. Everything that runs after it — atexit handlers, static destructors,
anything that allocates during exit — still uses smash's heap with no handler
behind it.

The drain restores `COMPRESSED`, `COMPRESSED_SHADOW` and `ACTIVE_MONITORING`,
and **trusts `ACTIVE`**. That was the hole: *a page's state is not proof of its
protection*. The fault handler repairs ACTIVE-but-still-protected pages on
purpose (Phase 3's batched mprotect can overwrite a per-page PROT_RW
restoration) — a race smash tolerates precisely because the handler is there.
Once it is gone, the same page kills the process on the next touch:
**SEGV_ACCERR, no message, exit 139, long after the workload finished**. One
mprotect over the contiguous arena at the end of the drain closes it; it only
widens permissions, so a page mid-restore cannot lose.

Found by running the suite against a **coverage-instrumented build**, where
gcov's exit-time dump is the late allocator: `test_span_descriptor_race_full`
segfaulted after printing PASS. Worth keeping as a technique — an instrumented
build is a cheap way to get a realistic "something allocates during exit" load
that the suite otherwise never applies. The evidence chain, in order: the
`SMASH_SHUTDOWN_TRACE=1` output ran to `stop() complete` *before* the crash; a
SIGSEGV handler preloaded **ahead of** libsmash (so smash's teardown restores
it) caught `si_code=2` at an address `smash_in_arena()` confirmed was in the
arena; the page index was inside the range the drain had just walked;
instrumenting `handleFault` showed the post-drain fault arriving on a page in
state ACTIVE; and leaving the handler installed through exit made it disappear
2/2. Verified, same test binary: pre-fix library 3/3 segfault, fixed 5/5 clean,
and the coverage suite went 57/58 → 58/58.

**No regression test, deliberately.** A synthetic workload with a late atexit
handler allocating and writing 8 MiB — including one registered from a library
preloaded ahead of libsmash, so it runs after libsmash's destructor — did not
reproduce the skew in 11 runs across two pre-fix libraries. Registering it would
have added a test that passes with and without the fix.

## Idle-page read tracking (root only)

`test_idle_read_tracking` (`idle_read_tracking_test.cpp`, in-process, `SMASH_COLD_TICKS=2 SMASH_SOFTDIRTY=0`) guards the one read-detection path that only exists as root. With `CAP_SYS_ADMIN`, `/proc/self/pagemap` exposes real PFNs, so `ensureIdleReadFds()` turns on idle-page tracking: at escalation a monitored page is marked idle through `/sys/kernel/mm/page_idle/bitmap` instead of being armed PROT_NONE, and phase 1 treats "no longer idle" as "read", with no fault paid. Unprivileged, the PFN is redacted and the path never activates — so the whole suite, and CI, had never run it.

First reported as three aarch64 failures — `test_compression` and `test_prefetch` (`page X should be COMPRESSED, got 4`, i.e. stuck in `ACTIVE_MONITORING`) and `test_syscall_efault` (`the buffer page never left RSS within 30s`). **It is not arch-specific: it is root.** On x86-64 the same three failed with the same messages as root, passed as a normal user, and passed as root with `SMASH_IDLE_READ_TRACK=0`. They are exactly the three tests that pin `SMASH_SOFTDIRTY=0`; with soft-dirty on, pages never enter `ACTIVE_MONITORING`, where the idle check lives. Two defects:

- **The idle bit was trusted for pages that were never marked.** The check was gated on `idle_read_ok_ == 1 && st == ACTIVE_MONITORING`, not on whether this page was armed. An unmarked page reads "not idle", so every monitored page counted as read at its first check, reset its cold count, and never reached escalation — nothing compressed. This was the actual cause of the three failures: even pages already on the LRU never compressed before the fix.
- **A failed mark was trusted.** The kernel silently ignores an idle mark on a page that is not on the LRU. The shared zero page never is (a page the application has only read), and freshly written pages are not until the per-CPU LRU-add batch drains. Measured on 6.8 as root: 0/8 freshly written pages take the mark, 8/8 after 256 MiB of other faults. Such a page reads "not idle" forever. A read that *was* counted had the same effect: the bit stays clear until the page is armed again, and it was only re-armed at `cold_count == floor`, which it could no longer reach.

The fix is a per-page `idle_armed_` flag, allocated only when the probe succeeds (unprivileged runs pay nothing): escalation marks the page, **reads the bit back**, and sets the flag only if the mark took — otherwise it falls back to the PROT_NONE arm. Phase 1 consults the idle bit only for flagged pages and clears the flag once it reports a read; entering monitoring clears it too.

The test asserts three scenarios: written pages use the idle arm (still PROT_READ, never PROT_NONE) and compress at tick 3; a read after the arm is detected without a fault (nothing compressed at tick 3) and the pages still compress once quiet (tick 6); zero-page-backed pages cannot take the mark, fall back to PROT_NONE, and compress at tick 3. **Verified as a real discriminator:** built with identical flags against the pre-fix compressor it fails 4 checks as root — no page compresses within 16 ticks in any scenario, and the zero page never falls back. It skips when not root or when the kernel lacks `CONFIG_IDLE_PAGE_TRACKING`, so it only guards anything when the suite is run as root. Measured as root after the fix: full suite 47/47.

## Small allocations after the arena is exhausted

`test_small_arena_exhaustion` (full mode, `SMASH_VM_GIB=1`). Reported on aarch64 (4 KiB pages) as `std::bad_alloc` and "terminate called recursively" in full mode only; `SMASH_LARGE_ONLY=1` worked. "terminate called recursively" is libstdc++ reporting a second thread reaching `std::terminate` while the first is in it — several threads getting `bad_alloc` at once.

In full mode small objects live in slab spans carved from the VmRegion arena, and **when the arena ran out, `Slab::allocateNewSpan()` returned nullptr**: large allocations fall back to a direct mapping, small ones had no fallback. The arena runs out without a large live set because an empty span's pages stay with its own (arena, size class) — `Slab::scavenge()` exists but has no callers — so a program that moves between size classes consumes a fresh share for each. Reproduced with a 1 GiB arena: 800 MiB of 64-byte objects, all freed, then 128-byte objects failed after **173 MiB** with the arena at 262143/262144 pages.

Fix: a span falls back to a direct mapping (`vm::mapPages`), and every region-only step in `Slab` — page states, the release hook, cold counts, `releasePages` — is skipped for it (`Slab::inRegion()`). Its pages are not compressed; a one-time warning says so. Emptying such a span uses `vm::decommitOwnMapping()`, because `vm::decommitPages()` deliberately traps on any address outside the arena. **Verified:** three phases of 800 MiB each (64, 128, 256-byte objects) complete with every object's pattern intact; the previous library fails phase B at 173 MiB; 64 threads × 10 rounds with the arena exhausted made 2.56 M allocations with 0 nulls.

**Freed memory is now reused by the arena** (`test_arena_reuse`, `test_page_run_map`). The fallback keeps a long-running program alive, but memory past the arena is never compressed, and two things made such a program drift out of the arena permanently:

- **Empty slab spans were never returned** (`Slab::scavenge()` had no callers), so every page a size class had ever used stayed with it.
- **Freed page runs were never coalesced.** They sat on 64 per-thread singly-linked lists of `FreeRun` nodes, so a run could only serve a request no larger than itself: 1-page runs freed by the 64-byte class could not hold a 2-page 128-byte span, 1 MiB runs could not hold a 2 MiB block.

Both are fixed. `PageRunMap` (`src/vm/page_run_map.h`) replaces the lists: boundary tags in demand-paged per-page arrays (length/owner/links at a run's first page, start at its last), size bins with a non-empty bitmap, merge with both neighbours on free. Owners keep NUMA node arenas apart (0 shared, n + 1 node n) — runs of different owners never merge. When neither the map nor the bump pointer can serve a request, VmRegion calls an **exhausted hook**: the heap returns every slab's empty spans synchronously (`Slab::reclaimEmptySpans`, `VmRegion::releasePagesNow`), they coalesce, and the allocation is retried before anything falls back to a direct mapping. Span descriptors are recycled with their bitmap (`Span::bitmap_store`), so span churn no longer leaks BootstrapAlloc.

Measured with a 1 GiB arena, phases of ~70 % each, everything freed between phases:

| phase | before | after |
|---|---|---|
| small 64 B | 100 % | 100 % |
| small 128 B / 256 B / 1 KiB | 40 / 6 / 0 % | 100 % |
| large 1 / 2 / 3 MiB | 0 % | 100 % |

**Verified as real discriminators:** with merging disabled, or with the hook not registered, `test_arena_reuse` fails the same six phases as before the fix; `test_page_run_map` checks the map against a per-page model through 284 k operations (4,919 large takes that only merged runs can serve) and fails 21 checks with merging disabled.

**Returning spans eagerly is a trap.** The first version released empty spans beyond two per slab on every free; mi-stress went from 16.9 s to 28.7 s, because spans oscillating between empty and in use paid a release, a decommit and zero-fill faults each round trip. Isolated by building the map without trimming (16.3–16.9 s) and trimming without merging (27–30 s). Reclaiming only when the arena is exhausted costs nothing in steady state: mi-stress 16.7–16.9 s against 16.2–16.9 s before; thread churn 83–86 s against 86 s; and 16 threads of 1–4 MiB blocks got faster, 10.2 s against 16.6 s, since the old path walked singly-linked lists across shards. 64 threads with the hook firing constantly (1 GiB arena): 2.56 M allocations, 0 nulls, no deadlock — the hook takes other slabs' locks with `tryLock` only.

## The metadata arena bounds how large a heap smash can manage

Every slab span costs a `Span` descriptor (112 B, pinned by a static_assert) plus its bitmap in BootstrapAlloc, which is never freed, so that arena's capacity bounds the heap. Fixed 16 MiB expansions capped it at 64 MiB + 63 × 16 MiB = **1.07 GiB** ≈ a 100 GB heap of large classes — and when it ran out, `allocate()` returned null into code that did not check it: **a write to address 0, with no message**. Reported as a crash at ~169 GB RSS.

- **Regions now grow geometrically** (16 → 32 → … → `kBootstrapMaxExpandSize` = 256 MiB), so 64 regions reach **15.4 GiB**, covering a multi-TB heap. `test_bootstrap`'s `testCapacityScales` fails if that drops below 8 GiB. Measured with the cap cut to 6 regions: flat growth exhausts at 144 MiB and fails the workload, geometric gives 560 MiB and completes.
- **Bitmaps are sized per class again.** Sizing every bitmap for the largest span (an earlier version of the span-recycling change) wasted 248 B on a 16 KiB class and cut the coverable heap 3× (329 GB → 102 GB). `Span::bitmap_store_words` tracks what the descriptor already owns and grows it only when a larger class needs more.
- **Exhaustion is loud and survivable.** `BootstrapAlloc::reportExhausted` prints regions, MiB and the failed request once; `newSpanDescriptor`, `Span::init`, `Slab::allocateNewSpan`, `LargeAlloc::allocate`, `PageMap::ensureL2` and `Slab::pushPending` all handle null, and `VmRegion::queueForDecommit` falls back to a synchronous release (its queue entry comes from the same arena). Verified with the arena cut to one 64 MiB region: 4/4 runs fail with `std::bad_alloc` and a message, where the same build crashed at 0x0 before. Each of those null paths was found by a crash, not by reading.
- **Watch it** on the SIGUSR2 stats line: `boot=<used>/<mapped>MiB(<regions>)`.

A fault smash does not handle, with the default disposition behind it, now prints `[smash FATAL] unhandled sig=… at addr=… — the process is about to die.` plus what smash thinks that address is: its own metadata, a page of the compressible arena (with the page's state — a fault there that smash did not handle is a smash bug), a tracked external page, or memory smash never managed. Silence used to be the only symptom, which left no way to tell a smash bug from an application one. Nothing is printed when the application has its own handler (it may well survive), and a `SIGKILL` — the OOM killer — still prints nothing by definition; check `dmesg`.

When the arena is exhausted the warning says **which kind** of exhaustion it is, because the fix differs: it reports pages handed out, free pages still in the map, and the longest reusable run. Few free pages means the live set genuinely outgrew the arena (raise `SMASH_VM_GIB`); many free pages with a short longest run means fragmentation, which a larger arena only postpones.

**Every full-mode allocation that is about to return nullptr now says why** (`util/alloc_failure.h`, rate-limited, allocation-free): `[smash alloc-fail] <path>: size=… align=… errno=… arena=used/total pages vmas=count/max_map_count`. Before, smash returned null silently and the only symptom was the application's `bad_alloc`.

## Write tracking: soft-dirty, userfaultfd, or neither

The compressor has to know which pages were written since the last tick. `selectWriteTracker()` picks the mechanism once per process **by testing it**, and the choice is on the SIGUSR2 stats line as `track=`:

- `softdirty` — `/proc/self/clear_refs` + `pagemap` bit 55, the default wherever it works.
- `uffd` — async userfaultfd write-protect (`UFFD_FEATURE_WP_ASYNC` + `WP_UNPOPULATED`) registered over the arena, read and re-armed each tick with the `PAGEMAP_SCAN` ioctl. Linux 6.7+. Opened with `UFFD_USER_MODE_ONLY`, which unprivileged processes may use under the usual `vm.unprivileged_userfaultfd=0`. A scratch page is checked end to end before the arena is registered.
- `none` — the files open but neither mechanism works: no write signal, only Phase B's content check.
- `protread` — Phase 3 `mprotect(PROT_READ)` monitoring: `SMASH_SOFTDIRTY=0`, or `/proc` unavailable (both as before).

`SMASH_WRITE_TRACK=auto|softdirty|uffd|none` (default `auto`: soft-dirty if it works, else userfaultfd, else `none`).

- `idle` (2026-10-13, opt-in, `SMASH_WRITE_TRACK=idle`) — the page_idle bitmap for reads and writes alike, no page protection at all. Built for an arm64 6.6 host, where `protread` was the only working tracker and its arming, write faults and `mprotect` calls under the `mmap_lock` were the suspected cause of the application's periodic slowdowns. The read-check path ("Idle-page read tracking" above) used to require `/proc/self/clear_refs` as well as `pagemap`; a binary given `CAP_SYS_ADMIN` by `setcap` is non-dumpable, its `/proc/self` files are root-owned, and the clear_refs open failed silently even when pagemap was readable. Both bitmap paths now need `pagemap` only.
- `yt` (2026-10-13, opt-in, `SMASH_WRITE_TRACK=yt`) — the same accessed-bit signal as `idle`, read through the youngtrace module's page-table walk (`/dev/youngtrace/scanner`, `mode=1`) instead of the bitmap: no PFNs, no rmap walk, 8-28 ns per page against the bitmap's 380-440. The module zeroes its whole output buffer after every request (~110 us at the default 8 MiB), so requests are as large as the buffer can always hold, and so are yt slices (64 MiB until 2026-10-23, which made every request at K > 1 a quarter of that: see "yt: every request covered a quarter of the module's buffer"). A cut request could not be resumed the way yt's own tools resume: in `mode=1` the pages the module cleared but could not report were lost, so they count as accessed instead. (Reported from here to youngtrace as #86 and fixed there in `c56f1b6`: the module now stops before a page table it has no room to report, so a resumed pass loses nothing. Verified 2026-10-26 on claudepc and the arm VM at `scan_buf_cap=65536`: the pre-fix module lost 16-2081 pages per pass in 22 of 25 layouts, including 2 MiB leaves and runs closed at a VMA's end, the fixed one none.) Requests are sized so none can be cut. As first written, a reported item marked only the pages inside its own request; a huge page split by a request edge lost its accessed bit to the first request and its other part read idle, and hot pages were compressed (`test_yt_tracker_thp`: 128 shadow and 86 decompression faults). Every page of the item is marked now.

**Why the test matters: arm64 has no soft-dirty** — `CONFIG_MEM_SOFT_DIRTY` does not exist there. smash used to decide soft-dirty worked because the two `/proc` files opened; the kernel accepted the clear as a no-op and reported bit 55 as 0 forever, so on arm64 every page looked never written, Phase 3 was skipped (soft-dirty replaces it), and each tick paid a whole-process page-table walk for nothing. Hot pages were compressed anyway and only Phase B's content check discarded them. Measured on x86 with bit 55 masked out to simulate it (hot 32 MiB rewritten every 20 ms, plus cold 32 MiB): real soft-dirty kept the hot set live; the simulated arm64 kernel compressed all 8192 hot pages; with the fix the same simulated kernel selects `uffd` and matches soft-dirty — hot set live, and for a 3-second writer 60 MiB RSS / 10.5 ms per write pass / 8.5 s CPU against soft-dirty's 60 MiB / 10.3 ms / 8.6 s. `SMASH_SOFTDIRTY=0` is **not** a good workaround there: `protread` was the slowest on both workloads (≈45 % more CPU, 7 % fewer write passes at 20 ms).

Two things the userfaultfd path has to handle, both verified with a standalone prototype before wiring it in: a `MAP_FIXED` remap creates a new VMA the userfaultfd no longer covers (writes become invisible), so `remapPages()` re-registers through `vm::g_after_remap_track_hook`; and a forked child inherits the fd but not the registration, so the post-fork reset closes it and the child selects afresh (verified: a forked child that starts a thread reports `track=uffd` with its hot set tracked). External pages (`SMASH_TRACK_EXTERNAL=1`) are the application's own mappings and are deliberately not registered, so they get no write signal under `uffd`.

`test_uffd_write_tracking` (in-process, forced to `uffd`, skips before Linux 6.7) drives `compressTick()` with a page written before every tick, a cold page, and a written page that went through `remapPages()`, and requires that neither written page is compressed at **any** tick. **Verified as a real discriminator:** with `SMASH_WRITE_TRACK=none` both written pages compress at tick 1; with only the remap hook removed, just the remapped page compresses (tick 2). Checking only the final state is not enough — an untracked written page cycles between SHADOW and ACTIVE as Phase B discards its stale blobs, and the last tick can land on ACTIVE. Suite 53/53 in default and forced-`uffd` modes, unprivileged and as root.

## An application's SIGSEGV handler displaced smash's (2026-09-28)

An application on the 128-core aarch64 box (kernel 6.6) crashed with SIGSEGV
under `SMASH_SOFTDIRTY=0`, on builds before and after the last free-path
changes, and nothing from smash was printed. Signal handlers are per
process: an application's `sigaction(SIGSEGV)` replaced smash's handler, and
smash only put its own back on its next compressor tick (`ensureInstalled`,
0.3-3 s measured). In between, every fault smash had to resolve — a compressed
page, a monitored PROT_READ page, a page a batched mprotect had left
read-only — went to the application's handler, which treats it as a crash.
With `track=none` those faults are rare; with `SMASH_SOFTDIRTY=0` the first
write to every monitored page is one. smash's own `[smash FATAL]` line never
printed because the application's handler, not SIG_DFL, was next in line.

(The cause of that application's crash is not confirmed — it had not been
re-run with the fix when this was written — but the window is a defect on its
own, and `test_signal_chain`'s phase A reproduces it deterministically.)

Fix: smash interposes `sigaction`, `__sigaction`, `signal`, `bsd_signal`,
`ssignal`, `sysv_signal`, `__sysv_signal` and `sigset`. For SIGSEGV and SIGBUS
the application's handler becomes the one smash chains to; queries return it;
the kernel keeps smash's. Found while writing it: dispatch treated a handler
as SIG_DFL only without `SA_SIGINFO`, but the kernel resets a one-shot
`SA_SIGINFO` handler to SIG_DFL with the flag still set, so a reset handler
would have been called at address 0; it now decides by the handler value, as
the kernel does. `SMASH_NO_SIGACTION_WRAP=1` restores the old behaviour.
`SA_ONSTACK` is deliberately not set on smash's handler: alternate signal
stacks are often small (~8 KiB per Rust thread) and the handler decompresses.

Measured: `test_signal_chain` phase F, 4 threads swapping SIGSEGV handlers
against 8 threads taking smash faults and their own for 1.5 s — 0 misrouted
(~46000 of each); with the old behaviour all 165228 smash faults went to the
application's handler.

## Fault prefetch never ran inside large blocks (fixed 2026-10-09)

`prefetchAdjacent` restores up to `kPrefetchWindow` (2) compressed neighbours
on each side of a faulting page, but only those `sameSpan()` says belong to the
same allocation, and `sameSpan()` looked both pages up in the page map and
required the same span. A large block registers only its **first** page there
(O(1) malloc/free, `LargeAlloc`), so every page inside one looked up as
nothing and the prefetch never ran — in every `SMASH_LARGE_ONLY=1` heap, the
production mode. Found while measuring restore cost for whole huge pages:
reading a compressed 2 MiB block in order took 512 faults, one per page, ~18 us
each on an 8-vCPU x86 VM. Now two pages are in the same large block exactly
when no span starts in between (no registered page in `(lo, hi]`); both
registered still means the same slab span. After: 171 faults for the same
read. `test_prefetch_large`.

The same page-map blind spot was in `lookupSpanInfo()` — fixed next.

## Large blocks had no bucket statistics (fixed 2026-10-09)

`lookupSpanInfo()` (the compressor's (arena, bucket) lookup) had the same
blind spot: a page inside a large block found no span, so `compressPage`
recorded **no** statistics for it — no ratio samples, no compression counts
(`SMASH_BUCKET_STATS=1` printed no rows for a compressed 64 MiB block), the
per-bucket profile/budget/thrash hints never applied, and with fewer than 8
samples the ROI gate always took the optimistic "compress" branch. In
`SMASH_LARGE_ONLY=1` that was the whole heap. Now `spanOfPage()` resolves a
page the page map does not know through a 4-entry per-worker cache, or by
walking back to the nearest registered page and checking its large block
covers the page; compressor threads only (the fault handler stays O(1)).
`lookupSizeClass()` — the dictionary choice at decompression — is unchanged on
purpose: a large block's bucket is never a dictionary class, so its pages are
compressed without one either way. `SMASH_LARGE_STATS=0` restores the old
behaviour.

Two things came with it. The bucket dump read only the default workers, but on
a multi-node host the NUMA node-group workers compress most pages: it now
includes them (without that, still no rows). And with statistics the ROI gate
started declining incompressible buckets — and a declined page was set ACTIVE
without backoff, so it was offered (and re-armed by Phase 3) every tick:
128 MiB of random bytes took 458544 attempts in 25 s. It now backs off like a
ratio-gate failure (cold streak restarted, effective floor doubled): one
attempt per page, 32752 of 32768 declined before any codec ran
(`roi_skip=`). Note `SMASH_COLD_TIMEOUT_SEC` sets `cold_ticks_overridden`,
which makes `shouldCompress()` return true at once: tests that set it never
reach the decline path. (It no longer does: see "The cold timeout switched
the ROI gate off" below.)


## The blob store kept regions of holes (fixed 2026-10-13)

A 128-core application's `[smash stats]` showed `blob=1735/5208MiB(338 regions)`
at one dump and `blob=1627/15717MiB(1078 regions)` three hours later: live
blob bytes flat, the memory holding them tripled, 10 GiB of a 46 GiB RSS
growth. The store packs blobs into 16 MiB regions with a bump pointer. A freed
blob became a hole on its region's free list, reused only by a blob that fit
it (no coalescing), and a region was decommitted only when its last blob went.
Under compress/decompress churn every region ended up a few live blobs among
holes. A second defect hid under it: a region that did empty was found by the
next store once the shard's head region was full, took that one blob, and was
then never bumped into again, so the next store mapped a new region.

Now each shard has a bump region separate from its chain head (an emptied
region becomes it and fills by bump), and `markDraining()` (each tick, before
Phase 1) marks every region with at least 1 MiB used and less than
`SMASH_STORE_DRAIN_PCT` (50) % of it live, the bump region included. Draining
regions get no new blobs; Phase 1 moves the blob of each compressed or shadow
page it finds in one into the same store under the page lock, and the region
empties and is decommitted. Measurements and tests: `SMASH_STORE_DRAIN_PCT` in
[config.md](config.md), `compress_store_test` and `test_blob_compact` in
[tests.md](tests.md).

### Then the free list itself (2026-10-14)

The free list the holes sat on was searched first-fit, one slot at a time,
under the shard spinlock, on every store; freed neighbours never merged, and
splits left small leftovers. Instrumented on the 1 GiB churn workload: ~500
slots searched and 14 us held per store (an LZ4 compression is 5-10 us),
15 000 slots at worst. The fault path releases a blob for every page it
restores under the same lock, so with 8 reader threads a release waited up to
26 ms. With compaction in place the holes were not worth reusing, so stores
now only bump, and `release()` takes the lock only when a region empties (it
locks the shard the region records, rechecks live bytes, and only then
resets). Lock held 1.3 us, slowest release 2 ms, user CPU 90 -> 29 s, held
about 10 % higher.

Moved blobs now go to a survivor region per shard, apart from new blobs, on
the theory that a blob which survived its region draining is long-lived and,
put back among new blobs, gets moved again. On both workloads measured
(uniform reads, and 90 % of reads to 5 % of pages) it changed neither the
bytes moved (598 vs 587 MiB) nor CPU: most blobs are moved once, while their
region is still filling with short-lived blobs, and repeat moves were rare.
It costs one more partly filled region per shard. Kept because it is cheap and
an application with long-lived blobs mixed among fast-churning ones is the case
it exists for; measure before relying on it.


## Idle compressor helpers polled (fixed 2026-10-14)

The default compressor pool starts all 7 helpers (`kMaxCompressorWorkers` 8)
at startup, and each waited for work with `usleep(100)` in a loop: ~6 400
wakeups a second per helper, whether or not the tick used it (a 128-core
application's dumps showed `workers=1`, so 7 helpers polled for nothing). The
coordinator meanwhile spun on the helpers' acks at full speed during every
phase. Found profiling store changes: 38 s of a 180 s run's kernel time was
helpers in `nanosleep`, the largest single kernel cost of the process.

Each helper now parks on its own futex word (`helper_go_`, `util/park.h`);
`dispatch()` bumps and wakes the words of the active helpers only, and parks
the coordinator on `helpers_pending_` until the last of them is done. Helpers
exit on `helpers_exit_`, set by `releaseHelpers()` after `stop()` has joined
the coordinator, never on `running_`: `stop()` clears `running_` while a tick
may still dispatch, and a helper that left on it (as the polling helpers
could) leaves the coordinator waiting forever. A helper's starting word value
is recorded by its creator, so one first scheduled after the exit signal still
sees its word changed. `start()` also caps the first tick's workers at the
helpers that exist (a failed `pthread_create` used to leave dispatch waiting
on a helper that was never made). macOS has no futex here and keeps the 100 us
poll.

Measured on the 8-CPU x86 VM against the previous commit, same flags:
idle context switches 44 200-44 600/s -> 106/s; churn workload (1 GiB, 2000
random reads every 0.5 s, 150 s) total CPU 114.7/113.9 -> 67.4/67.1 s; with 8
reader threads 137.7/131.7 -> 87.2/88.6 s, fault latency unchanged (p99
0.61-0.90 ms both); 90 % of reads to 5 % of pages, 180 s, 78.6 -> 27.6 s. Bytes
moved and held unchanged. Guarded by `test_helper_park`. The coordinator still
sleeps in 10 ms steps between ticks (~100 wakeups a second).

## Idle decommit threads polled (fixed 2026-10-14)

Each decommit queue's thread (the shared arena's, and one per NUMA node whose
arena has freed a block; started by the first free on that queue) waited for
work with `usleep(100)` in a loop: ~6 300 wakeups a second per thread, for the
rest of the process's life once anything had been freed. With a node queue on
each of four nodes, 31 600 wakeups a second. Found while counting smash's
threads for a report, after the helper fix above.

The thread now parks on its queue's `wake` word (`util/park.h`, `parkFor`) when
its queue is empty and nothing is urgent, for at most the time to its next
periodic duty (the sub-arena purge epoch, the run-cache trim), or without limit
when it has none. It is woken (`VmRegion::kick`) by a push onto an empty queue,
by a free that takes a sub-arena over `SMASH_DIRTY_MAX_MIB`, and by
`stopDecommitThread`. The handshake is Dekker-style: the thread sets
`sleeping`, then rechecks the queue, `purge_now` and `running`; a waker
publishes its work, then reads `sleeping` (all seq_cst).

Two things came out of the change:

- **Waking under the sub-arena lock lost the purge.** The first version woke
  the purger from inside `freeRunDirty`'s critical section; the purger, now
  woken at once instead of 100 us later, found the lock held, and its purge
  only tries the lock. `purge_now` was consumed, so it slept until the epoch:
  `test_subarena_plain_on` left 132 MiB dirty against a 32 MiB cap, 8 of 8
  runs. The wake now comes after the lock is released.
- **An urgent purge that leaves a sub-arena over its cap is retried 1 ms
  later** instead of at the next epoch. The polling code lost such a purge the
  same way whenever a free held the lock at that moment, which fits
  `test_subarena_on_softdirty0`'s one failure earlier the same day (one run
  purged, then nothing until the sweep). Not covered by a test: that needs a
  free to hold the lock exactly when the purge tries it.

Measured on the 8-CPU x86 VM (4 nodes), a process that freed blocks on every
queue and then sat idle for 10 s: 1.71 s of CPU (17 % of a core) -> 0.04 s;
with sub-arenas (`SMASH_DIRTY_MAX_MIB=64`) 2.06 s -> 0.06 s; idle context
switches 31 600/s -> 126/s. 2 MiB churn on 8 threads unchanged (default
3 833-4 088 -> 4 226-4 390 blocks/s; sub-arenas 332-359k -> 364-369k; run cache
348-365k -> 338-354k). Guarded by `test_decommit_park{,_subarenas}`. With
sub-arenas on, each queue's thread still wakes once per purge epoch (50 ms by
default: 20/s per queue); with the run cache on, the shared queue's every 10 ms.
macOS has no futex here and keeps the 100 us poll.

## SMASH_USE_LZ4 was never an environment variable; two codec-setting defects (fixed 2026-10-15)

The fast-tier codec was chosen at build time only (`-DSMASH_USE_LZ4=ON` defines
`kUseLz4FastTier`). Nothing read an environment variable of that name, yet one
application ran for months with `SMASH_USE_LZ4=1` in its environment,
believing it selected LZ4, and `test_large_bucket_decline` was registered with
it on the same belief (its claim that zstd-1 would accept its pages was never
true: both codecs decline them identically). Found while checking every
tunable against the source for a summary. `SMASH_CODEC=lz4|zstd` now picks
the codec in `ROIConfig::init()`, before calibration, and the stats line
reports `codec=`. The build option is removed (CMake warns if it is passed),
so the codec has one switch. The README's settings table listed
`SMASH_USE_LZ4=1` as an environment variable, and `tools/debug_compression.sh`
set it the same way; both are corrected. Blobs already carry their codec in `comp_size`'s top bits,
so nothing else changed. Measured on 64 MiB of text-like pages: 32 MiB of
blobs with LZ4, 18 MiB with zstd-1.

Two defects in neighbouring settings, found on the way:

- **`SMASH_VERY_COLD_TICKS=9999` compressed nothing.** Documented as "fast tier
  only", it removed every profile whose codec was zstd. With zstd-1 as the
  fast tier (the default) that was every profile,
  and `selectProfile()` then returns nothing for every page. It now removes
  the deep level only.
- **`SMASH_CALIBRATION_FILE` hung every process at startup**, whether the file
  existed or not: `loadCalibrationFile()` ran `fopen()` inside the heap's
  constructor, `fopen` allocates its `FILE` with malloc, and malloc waits for
  the heap being constructed (one thread spinning in
  `HeapRedirect::getHeap()`). Load and save now use `open`/`read`/`write`
  with a stack buffer. The file also records the codec its fast-tier figures
  were measured with, and a file for another codec is re-measured and
  rewritten instead of feeding LZ4's speeds to a zstd profile.

Guarded by `test_codec_env`.

## Stdio readers lost data in full mode (fixed 2026-10-15)

In full mode every stdio stream's buffer was smash memory (glibc mallocs it
in `_IO_file_doallocate`), and glibc fills it with an internal `read` that
never reaches smash's EFAULT retry. Only the wrapped `fgets`/`fread`/`fgetc`
recovered; `getline`, `getdelim`, `fscanf` and the `*_unlocked` readers lost
a `popen` child's delayed line with errno 14 every time, and so did `fgets`
and `fread` in any program built with `_FORTIFY_SOURCE` where the buffer
size is known, because they become the unwrapped `__fgets_chk` /
`__fread_chk`. Found when `test_cold_arena_env` (full-mode parent, `fgets`
on `popen`) lost all its children's output in the root UBSan suite. The
first diagnosis — root only, a hole in the buffer warming — was wrong: the
same program built without fortify read every line as root and not, and the
UBSan-built one failed unprivileged too; what UBSan changed was the symbol
the call bound.

Fixed by sending `_IO_file_doallocate`'s allocations to the system allocator
(decided in `callsiteArena()`, so `malloc`'s fast path pays one compare of a
value it already has) and by wrapping `__fgets_chk` / `__fread_chk`. That routing exposed an
older bug: smash's versioned `operator new` aliases never stored their
caller, so C++ allocations routed by the previous allocation's call site —
after a stdio buffer, to the system allocator (`test_vm_exhaustion`: 384 of
384 blocks outside the arena). They store it now. Details
and measurements: "Stdio stream buffers came from smash" in
[syscall-compat.md](syscall-compat.md). Not affected: `SMASH_LARGE_ONLY=1`,
where stream buffers already came from glibc (except a block size above
16 KiB, now covered too). Guarded by `test_stdio_buffer`, `test_stdio_chk`
and `test_chk_interposition`.

## Pages with dirty freed slots were never reclaimed in full mode (fixed 2026-10-15)

The compressor zeroes a page's free slots in its snapshot so they compress
well (`zeroFreeSlots`), but leaves the live page alone: the freed objects'
bytes stay there until the page is decommitted. In deferred-reclaim mode (the
default) Phase B checks that a `COMPRESSED_SHADOW` page is unchanged before
reclaiming it, and compared the live page byte for byte against the
decompressed, zeroed blob. Every page with a dirty free slot failed: it went
back to ACTIVE with its cold count reset, was compressed again a few ticks
later, failed again, forever. Measured with one live object and 15 freed
256-byte objects per page: **0 of 256 pages ever reclaimed, 1280 of 1370
compressions discarded** (`release changed=` in the `[smash pipeline]` line),
the pages cycling every ~6 s; with the freed objects cleared before `free`,
256 of 256 reclaimed and 0 discarded. Besides the RSS, every cycle cost a
compression and a decompression, and each discard made the page look freshly
written, so cooling-page closing (which steers allocations away from pages
going cold) saw all of them turn warm at once and sent churn onto them.

Found by `test_cooling_close`, whose survivor pages all went back to ACTIVE
in the same tick although nothing wrote to them. Fixed in
`shadowMatchesBlob`: a plain memcmp first, and on a mismatch a slot-by-slot
compare that skips the slots free now (their bytes are garbage either way, and
the restore gives them the blob's zeros); a slot allocated since the snapshot
is compared, so a write to it still discards the page. The bitmap is read
without the slab lock after the page is armed PROT_READ — as `zeroFreeSlots`
reads it when the snapshot is taken. Not affected: `SMASH_LARGE_ONLY=1` (large
blocks have no slots) and `SMASH_DEFERRED_RECLAIM=0`. Guarded by
`test_shadow_free_slots`, which also rewrites live objects on shadow pages and
checks the rewrites survive.

## Thrash back-off: the count decayed during the wait it imposed (fixed 2026-10-15)

A page faulted back from COMPRESSED has its recompress count raised, and Phase
2 then wants it idle for `floor << count` ticks before compressing it again.
The count was meant to decay "once a page truly settles cold", one step per
`kRcDecayColdTicks` (8) idle ticks — but the decay ran for idle ACTIVE pages
too, i.e. during the very wait the back-off imposed. So every wait was cut to
about 8 ticks: at the floor of 2 the waits were 4, 8, 8, 8 ticks instead of 4,
8, 16, 32 (the count never got past 2), and at the default floor of 10 a page
faulted back once waited no longer than a page never compressed (the count
dropped to 0 at 8 idle ticks, before the 20-tick wait ended). The design
commit (May 2026, floor then 2) expected "after 6 thrash events ... effectively
exempt until decay brings rc back down". Now the count decays only while the
page is COMPRESSED (and is still reset when an idle page's cold count
saturates). Measured end to end (`test_thrash_backoff`, floor 3 s): restored
pages compressed again after 13.1 s, against 5.0 s with the back-off off.

This applies in `SMASH_LARGE_ONLY=1` too. With the default 10 s cold timeout,
a page faulted back once (count 1, plus the bucket bias of 1 that its own
restore set) waited 20 s before and waits 40 s now; faulted back again before
its count decays (one step per 8 ticks spent compressed), 80 s, and so on up
to the 200-tick cap. Before the fix the wait was 20 s after one restore and
grew by only 8 s per further one (24 s at count 3, 32 s at count 4). `SMASH_RECOMPRESS_BACKOFF=0` turns the back-off off.

Two properties of the per-bucket half of the back-off, found while writing
the test, fixed the next day (below):
- Any restore counted, including one the allocator caused.
- The bucket bias never decayed.

## Thrash back-off: allocator restores counted, and the bucket bias never wore off (fixed 2026-10-16)

**Allocator restores.** The allocator hands out a block on a page the
compressor holds in two ways: the thread cache returns a slot freed into it
before the page went cold, and the slab falls back to a free slot on a
cooling or compressed page when a span has no warmer one and no empty span is
left (`consumeFallback`). The program's first write to the block faults the
page back, and that restore raised the page's recompress count and fed the
bucket EMA like a page compressed too early. Measured (`test_alloc_reopen`,
floor 3 s): 128 of 128 reopened pages got a count and were compressed again
after 13.1 s (4x the floor), the call site's fresh pages after 7.1 s (2x),
another call site's after 5.1 s. Both ways happen: a probe found the thread
cache handing back a freed slot on a compressed page as the very next
allocation, and in 64 allocations from a call site whose free slots were all
on compressed pages, 6 landed on a page still compressed. Now the heap checks
the page state of every small block it hands out once any page has been
compressed (`SmashHeap::noteBlockOnHeldPages`, one relaxed byte load per page
of the block) and marks a page that is COMPRESSED, COMPRESSED_SHADOW or
COMPRESSING in a per-page bitmap (`PageStateTable::markAllocReopened`); the
fault handler takes the mark and counts neither the page nor the bucket.
`alloc_reopen=` in the `[smash pipeline]` line counts such restores. After:
0 of 128 counted, all three groups at 5.1 s.

The first version recorded the thread's last small allocation in two TLS words
and had the fault handler compare the faulting page with it. It caught 3 of
132 such restores: the test, like any program whose constructors allocate a
member before writing their first field, allocated something else (a 48-byte
`std::set` node) between `malloc` and the first write. Don't go back to it.

Left: a mark set for a block that is never written, or whose page is restored
first by another access, makes that page's next restore go uncounted, once;
and a block handed out on a page just before it turns COMPRESSING is not
marked.

**The bias never wore off.** Phase 2 gives a page bias 1 (a doubled floor)
while its bucket's EMA of restore counts is >= 256 (x256 fixed point). The EMA
was fed only on restores, and every sample was a count >= 1, so it could never
fall below 256 again: one restore in a bucket doubled the floor of every fresh
page of it for the rest of the process. In `SMASH_LARGE_ONLY=1`, where a bucket
is (arena, log2 pages), that is every new block of that size range from that
call site's arena. Measured (`test_thrash_backoff`, step 4, 11 s after
everything was compressed again): the restored site's later pages at 6.2 s,
the other site's at 4.1 s (large mode: 6.2 against 5.2; 4.2 with the back-off
off). The first fix fed the EMA a 0 for each page that stayed COMPRESSED for
`kRcDecayColdTicks` (8) ticks. That was wrong too (see the next section): the
bias is now two counts per bucket, compressed pages that came back and that
stayed (`settle_ticks_`, once per compression), halved together once they sum
to `kBucketBiasWindow` (1024), with bias 1 while more came back than stayed.
After: 4.1 against 4.1 in both modes; right after the restores the fresh
pages still wait 2x (7.1 s).

## Thrash back-off: large blocks counted by their first page, and the EMA fix turned the bias off for good (fixed 2026-10-16)

**First pages only.** The restore path found a page's bucket through the
page map, which registers a large block's first page only, so in
`SMASH_LARGE_ONLY=1` the restore of any other page never reached the bucket
bias (Phase 2 applies the bias to every page, resolving interior pages through
its span cache; the restore path cannot afford that walk). Measured
(`test_thrash_backoff_large_interior`: step 1 reads every page of 64 blocks of
256 KiB but the first): no bias at all, T fresh 5.1 s against C fresh 5.1 s,
restored pages 7.1 s instead of 13. Now `compressPage`, which already resolves
interior pages for its statistics, records each page's bucket
(`page_bucket_`, 2 bytes per page, written under the page lock), and the
restore and the "stayed" count use it. The decompression time charged to the
bucket and the thrash rate kept for profiles went through the page map until
the next fix below.

**The EMA with zeros.** Phase 2's bias was on while the bucket EMA was >= 256,
and a page restored once fed exactly 256. With a 0 for each page that stayed,
one 0 turned the bias off, and in integer steps of 1/4 the EMA then climbs to
253 and stops: restores of pages compressed once could never turn it back on.
In the interior test the 64 untouched first pages, compressed throughout,
cancelled the restores of the other 4032 (restored pages at 8.1 s instead of
13). Replaced by the two counts above (back > kept); the threshold constant
`kBucketRcBiasThreshold_x256` is gone (`kBucketBiasWindow`,
`-DSMASH_RECOMPRESS_BUCKET_WINDOW=N`). After: restored 13.2 s, T fresh 7.1 s,
C fresh 5.1 s; later 4.1 against 4.1.

## Time budget and thrash rate counted a large block's first page; profiles saved one worker's rate (fixed 2026-10-17)

**First pages only, again.** The restore path charged decompression time
(`time_cost_total_us`, the time budget's input) and counted decompressions
(`decompress_count`, the thrash rate saved to `SMASH_PROFILE_FILE` and shown by
`SMASH_BUCKET_STATS=1`) in the bucket it found through the page map -- a large
block's first page only -- while `compressPage` counted every page's
compression. A call site whose blocks came back whole showed one
decompression per block: in `test_compression`'s `testInteriorBucketStats`
(one 64-page block, every page compressed and restored) a saved thrash rate of
**4/255 and `stable_cold=1`**, which makes the next run compress that call
site's pages after a quarter of the cold timeout -- the opposite of what a
100 % thrash rate should do. Both now use `page_bucket_`, read under the page
lock. Two smaller corrections with it: a zero page's restore counts neither
(its compression counted neither, so the rate could exceed 100 %), and an
allocator restore (`markAllocReopened`) is charged time but not counted as a
decompression for the thrash rate.

**One worker's rate.** `saveProfileFile` serialized the worker with the most
samples in each bucket, thrash rate included: its own compressions over its
own decompressions -- but the fault handler charges every decompression to
worker 0. Another worker holding the most samples saved 0 % (`stable_cold`);
worker 0 holding them saved every worker's decompressions over its own
compressions. It also left out the NUMA node groups, which compress nearly
every page on a multi-node host. The rate now comes from the counts of every
worker, node groups included (in the test: 80 of 100 split over two workers
saved 0/255 `stable_cold=1` before, 204/255 after).

Effect before the fix: none unless `SMASH_PROFILE_FILE` with
`SMASH_PROFILE_FILE_SAVE`/`_RW`, `SMASH_TIME_BUDGET_PCT` below 100, or
`SMASH_BUCKET_STATS=1` is set. **Not fixed:** `loadProfileFile` deserializes
into the default workers only, so on a multi-node host the node groups, which
run Phase 2 for nearly every page, never see a loaded profile.

## Phase B reclaimed other workers' pages: COMPRESSED pages read zeros without a fault (fixed 2026-10-16)

**Silent data corruption under load, in the default deferred-reclaim mode.**
Worker ranges are cut at `committed / workers` and node-group slices at
`16384 * slice / threads` pages, neither on 64-page chunk boundaries (only
under coarse watching). `forEachLivePage` filters pages to [start, end), but
`reclaimShadowRange` walked whole chunks from `start / 64` with only an upper
bound, so a worker whose range began mid-chunk also reclaimed the previous
worker's pages of that chunk, concurrently with it. Its per-page work rechecks
state under the page lock, but its look-ahead arms a run of COMPRESSED_SHADOW
pages PROT_READ **without** their locks. When the other worker reclaimed
those pages (decommit, PROT_NONE, COMPRESSED) between the look-ahead's state
check and its `mprotect`, the PROT_READ landed last: pages marked COMPRESSED,
their memory dropped, and readable -- every read returned zeros and no fault
ever restored them. It also counted chunks from `end - start`, one short when
`start` was mid-chunk, so the chunk holding a range's last pages was walked
only through the overlap.

Found by `test_thrash_backoff` reading back wrong bytes when its three modes
ran at once (`ctest -j3`): whole runs of up to 32 consecutive pages of a set
read as zeros, `smash_page_info` state 2 right after the read. Reproduced on
the library from before the back-off changes (`f8930c6`) too -- 31628 bad
bytes in 1 of 5 rounds of six concurrent runs -- so it predates them; the
current tree hit it in 2 of 5 and 4 of about 19 other rounds. Unprivileged
and root alike; more workers and a busier machine widen the window, so a
128-core host with node groups is the most exposed. Fixed: chunks
[start / 64, ceil(end / 64)), pages outside [start, end) skipped.
`test_compression`'s `testReclaimRange` (`test_compression_reclaim_range`,
`SMASH_DEFERRED_RECLAIM_DELAY=1000` so ticks reclaim nothing): Phase B over
[+10, +74) of two shadow chunks must reclaim exactly those 64 pages -- before
the fix 10 below the range and 54 of 64 inside.

## Idle scan: every worker read and wrote the page_idle bitmap through one open file (fixed 2026-10-19)

`SMASH_WRITE_TRACK=idle` opened `/sys/kernel/mm/page_idle/bitmap` once (`page_idle_fd_`), and every compressor worker's Phase 1 scan read and re-marked it through that file. sysfs runs one read or write at a time per open file (kernfs `of->mutex`), so the workers' scans ran one at a time and the waiting ones spun on the mutex.

Found on the 128-core arm64 box (kernel 6.6) with `integrity_stress_test` at 128 threads x 64 MiB for 1800 s under `idle`: ticks of 4-13 s that were nearly all Phase 1, 10.3M ms of Phase 1 system time against 92 s blocked, and the idle scan at 41.5 us per page over the run, 98 us during the idle wait. A kernel-only profile (`perf record -a -g -e cycles:k`, during the idle wait) put `mutex_spin_on_owner` (7 %) and `osq_lock` (1 %) under `idleScan`'s `pread`/`pwrite`, `page_idle_get_page` at 3 %, and nothing else above 0.6 % but idle and task switching.

Fix: each scanning thread opens its own (`idleBitmapFd()`, thread-local, kept until exit; the shared file if the open fails). `ensureIdleReadFds()` also opened a second shared file over the tracker probe's and leaked the first; it now reuses it. Measured as root on claudepc (2-CPU x86, 2 nodes), the stress workload under `idle` at 2 threads x 256 MiB with 8 workers, 60 s, two runs of each: scan cost **10.1 and 11.3 us per page before, 6.0 and 6.3 after**; Phase 1 system time 13.1/15.0 s to 9.0/9.0 s, blocked 10.1/12.2 s to 4.9/5.2 s. Two CPUs cap the gain there; on the arm64 box eight workers can run at once. Guarded by `test_idle_tracker_workers`.

Not fixed: a bitmap read makes the kernel check all 64 pages of each word and walk the mappings of every one marked idle, asked about or not. The new `O other idle` count on `idle_scan=` measured that at 13.6 per page asked during the arm64 box's idle wait (1.3 during the run, a lower bound), and 7-10 per page on claudepc with a dense 512 MiB heap. Scanning in physical-address order over a larger batch would read each word once per pass.


## yt: every request covered a quarter of the module's buffer (fixed 2026-10-23)

`SMASH_WRITE_TRACK=yt` sizes a request to the most the module's output buffer can always hold, `(scan_buf_cap - 8) / 128` pages: 65535 (256 MiB) at the default 8 MiB. But slices were a fixed 64 MiB, and at K > 1 the slices due in one tick are never adjacent (`idleSliceDue`: slice s is due when (s + tick) % K == 0), so a run of due slices was always one slice and every request covered 64 MiB. Every request costs the module a zeroing of its whole buffer (`memset(buf + len, 0, cap - len)` at the end of `scan_target()`, whatever the scan wrote), so the default K=4 paid that four times as often as the buffer allowed. Found by reading the module while checking how smash uses it: the earlier x86 measurement had 1176 requests for 19267584 pages at K=4, exactly 16384 each, and 26 ns per page against 11 at K=1, where due slices are adjacent (58121 pages a request).

Fix: a yt slice is the largest power of two of pages that fits one request (`yt_slice_shift_`, set by the probe from `scan_buf_cap`): 32768 pages, 128 MiB, at the default buffer, whose 65535-page limit misses 256 MiB by one page (8 bytes of output at worst). `SMASH_YT_REQ_PAGES` still caps requests without changing the slices, so `test_yt_tracker_thp` keeps its request edges inside huge pages. Measured as root, the idle/warm/hot workload (2 GiB: 256 MiB read every 1.5 s, 256 MiB written every 2 s, the rest idle), K=4, 150 s, two interleaved runs of each:

| | requests | yt scan | read pass avg / p50 / p90 |
|---|---|---|---|
| claudepc (2-CPU x86), old | 1168 / 1168 | 473 / 471 ms | 3.5-3.9 / 2.9-3.3 / 4.6-5.0 ms |
| claudepc, new | 584 / 588 | 414 / 340 ms | 3.4-3.9 / 4.2-4.5 / 7.6-8.3 ms |
| arm VM (1 CPU, QEMU-emulated), old | 857 / 832 | 5990 / 6131 ms | 20.8-23.7 / 19.0-20.7 / 28.5-37.1 ms |
| arm VM, new | 430 / 434 | 4151 / 4124 ms | 22.0-24.1 / 19.6-20.0 / 30.1-38.3 ms |

The arm VM's figures fit about 4.7 ms per request plus 150 ns per page (emulation: about 10x x86), so after the fix half of its scan time is still the per-request zeroing. On claudepc the read pass's median and p90 rose in both runs while its average did not; on the arm VM they did not move. Not explained, and not repeated beyond the two runs. Guarded by `test_yt_tracker_slices`.

Not fixed: a worker range edge inside a due slice still splits it into two requests (default worker ranges are `committed / workers`, not slice-aligned), and node-group threads each scan an equal part of every 64 MiB node chunk, one request per part. The zeroing itself could cover only what the previous scan wrote; that is a youngtrace change. Done there in `992e189` (one zero word ends the stream; the bytes after it may be an earlier scan's, which the reader never reads): on claudepc the same 150 s K=4 run scanned in 213 / 245 ms against 340 / 404 ms with the module before it, same requests and pages, cut=0 (2026-10-26).

## Sub-arenas: a full region purged every dirty run on each miss (fixed 2026-10-27)

With `SMASH_SUBARENAS=1`, once the region had no fresh chunk left, large-block throughput could fall about 20x and never recover. 8 threads churning 1-3 MiB blocks at 70-75 % of a 1 GiB region (`SMASH_VM_GIB=1`, the compressor off, the user's other settings) ran at about 200k blocks a second for 2-13 s, then at 9-11k for the rest of the run. Page faults rose from 0.2M to 1.5-1.8M a second and the CPU time moved from user to system. The `[smash WARN]` fallback line printed in the same second, because a few blocks (0.1-1 %) left the region, but those mappings were not the cost. Every `madvise` in that state (516 of 516 stacks) came from `allocatePages` -> `donateSharedPool` -> `purgeSubArena`.

Cause, measured with an instrumented `smash_sim` at each miss: the requests that missed were 3 MiB, and the largest dirty run and the largest clean run were both mostly 2 MiB. Adjacent dirty and clean runs of the same sub-arena added up to 5-9 MiB, but the two never merge: a clean run's pages are purged and a dirty run's are not. So the request left the sub-arenas for the shared free map, and `donateSharedPool` purged every dirty run of every sub-arena on each such miss, even when no chunk became wholly free and nothing could be donated. The next blocks were then faulted in from scratch, frees put new dirty runs next to the now-clean ones, and the next miss came sooner.

Fix: (1) the last-resort retry in `allocateOnSubArena`, reached only when no fresh chunk is left, takes a block across a dirty run and the clean runs next to it (`takeMixedFromSubLocked`). Only the block's dirty pages leave the dirty count, and the rest of each run goes back as it was. That is safe because `calloc` always clears and nothing assumes arena pages are zero. (2) `donateSharedPool` retires the tails first and then purges only the dirty runs of chunks that the sub-arena's runs cover wholly, which are the only ones it can donate. Interleaved old/new on a loaded 8-CPU host, live 45 and 48 per thread: old 9-11k blocks/s in the second half with 2.4-3.3k blocks out; new 137-198k with none out. `smash_sim` (stubbed OS calls, 8 threads, 1 GiB, 8 s): at live 52, 327742 blocks out and 6.1M runs purged became 1 and 222. In four other configurations (cold pages, cross-thread frees, `malloc_numa` on 2 and 4 nodes) new put no more blocks out and purged up to 25 000x fewer runs. No invariant failures. `test_subarena_exhaust_mixed` guards both parts.

Not fixed: a chunk donated to the shared free map never returns to the sub-arenas, and a block freed there goes through the decommit queue (`madvise`) rather than staying dirty. At 81 % and above (live 52-60 here) blocks really do not fit. They leave the region with the warning, and each one is a fresh mapping faulted in. Throughput there went from 5-7k to 15-160k, but not to 200k.

## Arena-overflow warnings spent on requests that got no block (fixed 2026-10-28)

`LargeAlloc`'s one-shot "did not fit the compressible arena" warnings (plain
large blocks and `malloc_numa`) printed as soon as the arena missed, before the
fallback mapping was tried. A request no mapping could serve — a 16 TiB
`malloc`, one page of `malloc_numa_aligned` at 2^63 alignment — returned NULL
after saying "It came from a direct mmap instead" (or "a separate mapping"),
which was false, and spent the one-shot: the real overflow later in the process
printed nothing, the very case the warning exists for. Found by
`test_malloc_numa_aligned`'s 2^63 check, whose run printed the warning with
"a 0 MiB malloc_numa allocation … The node's arena has the space".

Fixed: both warn after the block is registered. Two verdicts were wrong as well.
The node line weighed the node's free pages against the block alone, so one page
at 2 GiB alignment in a 1 GiB arena "has the space"; it now counts the worst-case
alignment gap, and a need larger than the whole arena says so ("The block and
its alignment need N MiB, more than the whole M MiB arena: raise SMASH_VM_GIB
above that"). The plain line said "the live set outgrew the arena" for a single
block larger than the arena with nothing else live; it gets the same verdict.
Sizes under 1 MiB print in KiB. Tests: a refused request at the start of
`test_vm_exhaustion{,_numa,_numa_aligned}`, and `test_vm_exhaustion_too_big`;
the library before failed all four, and five mutants (each part of the fix
undone) each failed only their own check.

## Large blocks: sizes past 2^32 pages returned short blocks (fixed 2026-10-28)

`LargeAlloc::allocate` rounded the size up to a page without an overflow check
and cast the page count to the span's 32 bits. In both modes, where glibc
returns NULL with `ENOMEM`:

| request | before |
|---|---|
| `malloc(16 TiB + 4 KiB)` (2^32 + 1 pages) | a one-page block, `malloc_usable_size` 16 TiB |
| `malloc(16 TiB)`, `malloc(2^62)` (2^32, 2^50 pages) | a block of 0 pages |
| `posix_memalign(2 MiB, SIZE_MAX)` | 0, with a 2 MiB block (the size rounded to 0, the over-map was the alignment alone) |
| `malloc(SIZE_MAX)` | NULL, but `errno` `EINVAL` (mmap of 0 bytes) |

A caller that writes what it asked for runs off the end: `calloc` of such a
size zeroed the whole length and faulted, and `realloc` to it copied the old
block into nothing. Any caller passing an unchecked length (a size read from a
file or the network, `n * size` computed by hand) got a short block instead of
NULL. Found while testing `malloc_numa`'s oversized requests, whose path had
the guard (`allocateOnNode`) and so behaved.

Fixed: the same guard in `allocate`, `ENOMEM` for a wrapping size or a page
count past 32 bits. Every entry point reaches it: `malloc`, `realloc` and
`calloc` through `SmashHeap::malloc` (`calloc`'s own multiplication was already
checked), the aligned calls through `memalign`. The mid-size bins and the
aligned slab classes return before any arithmetic for sizes this large. Guarded
by `test_alloc_contract_{full,large_only}`'s `testHugeSizes`.

Not smash's: alloc8's `pvalloc` rounds the size up to a page itself
(`(sz + pagesize - 1) & ~(pagesize - 1)`), so `pvalloc(SIZE_MAX)` asked smash
for 0 bytes: measured with the fix, a 4 KiB block in full mode and a 24-byte
system block in large-only mode (a 0-byte request is small, so it passes
through), where glibc's own `pvalloc` returns NULL with `ENOMEM`. Sizes that
do not wrap there, such as 2^32 + 1 pages, failed correctly. Fixed the same day
by smash's patch to alloc8 (next section).

## calloc wrote every page of its block; alloc8's aligned_alloc refused most sizes (fixed 2026-10-28)

Found by reading the alloc8 code smash links, then measured against glibc 2.39
(probe, `SMASH_NO_COMPRESSOR=1`; full and large-only alike):

| call | glibc | smash before |
|---|---|---|
| `calloc(1, 64 MiB)` | 0 ms, RSS +0 | 45 ms, RSS +64 MiB |
| `calloc(1, 1 GiB)` | 0 ms, RSS +0 | 660-725 ms, RSS +960 MiB |
| `aligned_alloc(64, 100)`, `(4096, 100)`, `(2 MiB, 3 MiB)` | a block | NULL, `errno` untouched |
| `aligned_alloc(24, 48)` | NULL, `EINVAL` | full mode: a block not 24-aligned |
| `calloc(SIZE_MAX/2 + 1, 2)` | NULL, `ENOMEM` | NULL, `errno` untouched |

**calloc.** alloc8's `HeapRedirect::calloc` is `malloc` + `memset` of the
whole block; it never asks the heap, so `SmashHeap::calloc` was dead code (its
comment said so). The `memset` faults in every page of a block whose pages are
zero already, so a large sparse table allocated with `calloc` became resident
at once, the opposite of what smash is for. It also bypassed libc's `calloc`
in passthrough and compress-only modes (+256 MiB for a 256 MiB `calloc`).

Fixed: `SmashRedirect` (smash_heap.cpp) derives from `HeapRedirect` and sends
`calloc` to `SmashHeap::calloc`, which routes compressor threads,
passthrough and compress-only to libc's `calloc` (as `mallocSlow` routes their
`malloc`), checks the product (`ENOMEM`), and for a block past
`kMaxSmallSize` asks `mallocSlow` whether its pages read as zero. They do when
the block is a fresh mapping (below `SMASH_LARGE_ALLOC_VM_THRESHOLD`, or an
arena miss) or an arena run that was never used or was decommitted: every run
in the shared free map, a sub-arena's clean runs and tail, and the chunk pool
is fresh or went through `decommitPages` before it got there. Not when it is a
sub-arena's dirty run (`takeFromSubLocked`), a span that includes one
(`takeMixedFromSubLocked`), a run-cache hit, or when the decommit does not
zero (`vm::decommitZeroes()`: `SMASH_MADV_FREE=1`, `SMASH_NO_DECOMMIT=1`,
macOS). Slab slots and mid bins are always cleared. After: +0 MiB for 256 MiB
fresh, 64 x 512 KiB, and 256 MiB over a decommitted block, in every mode.

**alloc8.** Its `aligned_alloc` kept C11's rule that the size be a multiple of
the alignment, which C17 (DR 460) dropped and glibc never enforced, so a
program that works on glibc got NULL (and a "Success" error message); it did
not check that the alignment is a power of two. Its `pvalloc` wrapped (previous
section). Neither is smash's code, so the fix is a patch, not an edit:
alloc8 is now pinned to the commit smash is tested against (`GIT_TAG main`
had pulled whatever upstream was at each fresh configure, release builds
included) and `patches/alloc8/*.patch` is applied by FetchContent's patch step
(`cmake/apply_patches.cmake`): any size, `EINVAL` for an alignment that is not
a power of two (glibc 2.38+), `ENOMEM` for a `pvalloc` within a page of
`SIZE_MAX`, on Linux and in the macOS wrapper. A build against `-DALLOC8_DIR`
warns for each patch that tree lacks. The release workflow did exactly that,
with a checkout of alloc8's default branch, and CI checked one out unused;
both now take the pinned fetch. The calloc overflow `errno` needed no
patch: smash's own `calloc` now sets it.

Tests, written first: `test_calloc_*` (nine registrations) and
`test_alloc_contract_{full,large_only}`; [tests.md](tests.md). The library
before failed both contract tests and six calloc registrations; fourteen
mutants, each undoing one part, were each caught.

## operator new never called the new_handler (fixed 2026-10-28)

An `operator new` that cannot allocate must call the installed
`std::new_handler` and try again, throwing `std::bad_alloc` only once there is
none; the nothrow forms do the same and return nullptr where the throwing ones
throw ([new.delete.single]). libstdc++'s do. Both sets smash exports threw at
once: alloc8's operators (`new_delete.inc`, the default-version `_Znwm` and
friends) and smash's own GLIBCXX/CXXABI-versioned aliases in
linux_syscall_wrappers.cpp (what code linked against libstdc++ binds). Measured
with a handler installed and a request nothing can serve: glibc's runtime
called it once before throwing (and before nothrow returned nullptr), smash 0
times, in full and large-only modes. A program whose handler frees a cache
and retries got `bad_alloc` instead of its memory.

Fixed in both: on a null result each allocating operator calls a cold
`newRetry` / `alloc8_new_retry` (alloc8 patch 0002, also its macOS
`new_delete.cpp`) that loops on `std::get_new_handler()`, retrying the same
allocation and restoring `alloc8_caller_ra` after the handler (which may
allocate, and would otherwise move the retry to another arena); the nothrow
forms wrap it in `catch (...)` and return nullptr, as libstdc++'s
`operator new(size_t, nothrow_t)` does. The fast path is unchanged: one
null test.

Guarded by `test_alloc_contract_{full,large_only}`'s `testNewHandler`
([tests.md](tests.md)); the library before failed 33 of its checks in each
mode.


## The release hook walked every page of every freed block with the compressor off (fixed 2026-10-29)

`LargeAlloc::deallocate` hands every arena block to
`CompressorThread::releaseCompressedPages`, which took each page's lock and
reset some fifteen per-page arrays — 16 384 lock/unlock pairs and ~250 000
scattered stores for a 64 MiB block — whether or not a compressor thread had
ever existed. `SMASH_NO_COMPRESSOR=1` skips `start()` only, so the control an
operator reaches for to measure the allocator alone paid the full walk; so did
every free before `start()`. Now a free made before any compressor thread has
started (`ever_started_`, latched by `start()` and kept by a fork child, whose
`g_any_page_compressed` guards pages compressed in the parent) resets the
page-state bytes in bulk and skips the walk: 7 µs against 137 µs per 8 MiB
block. The walk with the compressor running is unchanged; the ideas for
shrinking it (a span-level "ever compressed" bit set by the tick; CAS
transitions so the free path need not take each page's lock) are open.
`test_release_hook_{off,off_full,on}` ([tests.md](tests.md)); a mutant that
always walks and one that always takes the bulk path each fail exactly the
registrations they should.

## One purger thread and no backpressure: a burst of frees held gigabytes dirty (fixed 2026-10-29)

Each set's dirty runs are purged by one thread (the node's or the shared set's
decommit thread), which a free past `SMASH_DIRTY_MAX_MIB` wakes. Churn that
reallocates what it frees never needs it (1.2M frees/s on 8 threads, zero
purges: each thread takes its own dirty run back), but a burst nothing takes
— a working set shrinking, blocks freed on CPUs that never allocate — handed
that one thread the whole burst with nothing bounding the dirty bytes but its
`madvise` throughput: 8 threads each freeing 128 MiB against an 8 MiB cap put
128 MiB dirty into one sub-arena, and would have put as much as they freed.
A free that takes its sub-arena past twice the cap now purges the excess on
the freeing thread, under the lock it already holds (`purgeSubArenaHoldingLock`,
the madvise with the lock dropped), so no sub-arena is ever more than twice
its cap plus one block over whatever the thread's backlog; the thread keeps
the time decay and the under-2x case. The purge's victim sort is `std::sort`
(it was an insertion sort under the lock). The 128 MiB bursts now peak at
exactly the bound, 20 MiB, with 404 inline purges over 6 rounds; the slowest
burst of 32 frees took 18 ms (its own madvise). `test_subarena_pressure{,_full}`;
a mutant without the inline purge holds 128 MiB.

## The constructor allocated and hung; glibc < 2.34's dlsym does (fixed 2026-10-29)

alloc8's `HeapRedirect::getHeap()` parks every thread but the one
constructing the heap until construction is done, and had no provision for
the constructing thread coming back: an allocation from inside `SmashHeap()`
spun for ever with nothing printed. glibc before 2.34 does exactly that —
its `dlsym` callocs a per-thread dlerror record on a thread's first call, and
the constructor's first `mmap` goes through smash's wrapper, which `dlsym`s
its real target; RHEL 8 (2.28, the release floor), Ubuntu 20.04 and Debian 11
(2.31) all hang at startup. The same calloc inside `SystemAllocFns::resolve`
(no system calloc yet) resolved again, recursively, until the stack ran out.
Reproduced with `tests/helpers/olddlsym_shim.c`, which gives this glibc's
`dlsym` the old behaviour. Two parts: `core/early_alloc.h`, a 256 KiB static
arena that serves malloc/calloc/memalign to a thread inside either window
(`early::Window` in the constructor and in `resolve()`; one relaxed load on
the malloc path when no thread is in one), with free a no-op and realloc and
`malloc_usable_size` handled on the heap's foreign-pointer paths; and alloc8
patch 0003, which makes `getHeap()` re-entered on the constructing thread
abort with a message instead of spinning, for whatever the arena does not
cover. On this glibc the arena serves three loader allocations made inside
the fortified resolver's `dlopen(libc, RTLD_NOLOAD)`, 2.4 KiB.
`test_early_alloc_old_dlsym{,_full}`: the library before hangs both; with the
arena disabled the run aborts with alloc8's message.

## The lazy-resolve guard was process-wide (fixed 2026-10-29)

Every wrapper `dlsym`s its real target on first use behind a guard against
dlsym recursing into the wrapper. The guard was one `std::atomic<bool>` for
the process: while thread A's dlsym ran, thread B's first call to any other
wrapper saw the flag, skipped its own dlsym and ran unresolved — a raw syscall
without the EFAULT retry for the I/O wrappers, `MAP_FAILED` from `mmap`,
`mmap64` and `mremap` (with a stale errno), -1 from `execvp`, `ENOSYS` from
`posix_spawn`, nullptr from `fgets`. 128 threads starting together make the
overlap routine. The guard is a `thread_local` now, the resolved pointer an
atomic (every thread raced to write it), and the mmap family falls back to the
raw syscall instead of failing. `test_lazy_resolve_race`, with
`tests/helpers/slowdlsym_shim.c` (3 ms per dlsym) so the overlap is certain:
the library before fails it every run (`mremap` MAP_FAILED, `fgets` nullptr).

## The blob store had no cap, and a refused page was retried every tick (fixed 2026-10-29)

`CompressStore` grew until `mmap` (or, for a node store, `mbind`) failed, and
a page whose blob the store could not take went back to ACTIVE with its cold
streak intact and no failure counted: selected, snapshotted, compressed and
refused again on every tick, for ever, silently. `SMASH_BLOB_MAX_MIB` caps the
bytes every store keeps resident (`g_blob_held_bytes`: below each region's
bump pointer; a moved blob is compaction and never refused), a refusal backs
off exactly like a ratio-gate failure (the cold streak restarts, the effective
floor doubles) and the first one warns. `test_blob_cap`: 64 MiB of half-
compressible data against an 8 MiB cap holds 8 MiB, and a window four times
as long as the first sees fewer refusals than it (17 235 then 12 859, against
42 194 then 102 872 for a mutant without the back-off).

## A restore that could not get a VMA spun for ever (fixed 2026-10-29)

A restore writes the page through `/proc/self/mem` and then `mprotect`s it
PROT_RW, which splits a VMA; at `vm.max_map_count` that is ENOMEM, as were
both fallbacks, and `restorePageContents` returned with the page still
PROT_NONE + COMPRESSED — the faulting instruction re-executed at once and the
thread spun at full speed, silently, until some other thread's unmap freed a
VMA, or for ever. Worse, the fault handler then marked the page ACTIVE anyway,
so once a VMA did come free the page came back without its data (the test's
read-back of the pre-fix library returns the wrong byte). The restore now
waits for a VMA in 5 ms steps for up to 2 s and then re-establishes the
mapping and rewrites the data; if none comes it prints a FATAL line and
aborts, since the blob cannot be put back. `test_vma_exhaust` fills the
process to `vm.max_map_count` with one-page mappings of alternating
protection and touches the middle of a fully compressed block: with a thread
unmapping half the fillers 300 ms later the touch completes with 21 ms of CPU
for the waiting thread (385 ms of wall time); with nothing unmapped the child
aborts with the message inside 8 s where the library before spun on.

## At vm.max_map_count a fault could only wait for a VMA, then abort (fixed 2026-11-08, upstream #94 ported)

The fix above waited for a VMA to come free and aborted after 2 s. Nothing in
a process frees VMAs on a schedule, so at the cap that was an abort: upstream's
`test_vma_cap` (an interpreter-teardown walk, every 3rd page of 128 MiB of
compressed blocks, 12K VMAs below the cap) aborted on every run once the
prefetch window was off (`SMASH_PREFETCH_WINDOW=0`) or near-cap handling was
(a negative `SMASH_VMA_MARGIN`); with the default window it passed only because
prefetch restored the walk contiguously. Upstream had the earlier, spinning
variant and fixed it in #94; the port, adapted to this branch:

- **Whole-VMA restore at the cap** (`restoreWholeVma`): changing the protection
  of an entire VMA needs no split, so it succeeds at the cap and merges with
  the RW neighbours. Every compressed page in the range is decompressed into
  its backing through `/proc/self/mem` first, then one `mprotect`; any other
  page's lock busy, or a page mid-compression, free, or in a unit compressed
  whole, aborts it with nothing changed. Upstream finds the VMA in
  `/proc/self/maps`; at this host's (and CI's) 1 048 576-map cap one read cost
  295 ms of CPU for a single restore, so the run of inaccessible pages around
  the fault, from page states, goes first (4-5 ms), and the maps file is read
  only when that `mprotect` needed a split.
- **Near the cap** (`restoreNearCap`, within `SMASH_VMA_MARGIN` by the shared
  estimate, which now also counts the splits faults make): the faulting page's
  run up to its nearer end comes back instead of the page alone, so a
  teardown walk stops adding two VMAs per fault.
- **A restore that cannot complete leaves the page as it was** — COMPRESSED
  with its blob, its deferred-madvise bit back — and the access re-faults:
  `capStall` yields, then sleeps 1 ms per retry, with no page lock held (the
  old wait slept under it), and aborts with a FATAL line after 2 s without
  progress. Upstream retries for ever.
- **`remapPages` no longer backs up a failed `mprotect` of a live page**
  (`makeResidentPageWritable`, ACTIVE and watched pages): at the cap it fails
  too, and anywhere it succeeds it replaces the live page with zeros. On ENOMEM
  the whole VMA is restored instead.
- **`restoreUnit`**: a page of a whole unit that could not be made accessible
  was released with the rest; it now keeps its blob and its state.
- **A VMA budget for Phases 2 and 3 within a tick** (`chargeVmaBudget`), armed
  from the shared estimate after Phase B: before, only the tick-start check
  bounded them.
- **`readVmaCount` uses raw syscalls**: upstream found the stdio version read 0
  at high counts on Graviton (stdio buffers on watched pages), which turned
  every VMA guard off. Not measured here, where stdio buffers come from the
  system allocator, but `fopen`'s `FILE` still came from the managed heap.

`test_vma_cap{,_no_prefetch,_at_cap}` and a rewritten `test_vma_exhaust`
([tests.md](tests.md)) guard it; the library before the port fails the
variants and the at-cap and stall cases.

## A stack overflow never reached an application's alternate-stack handler (fixed 2026-10-29)

smash's handler was installed without `SA_ONSTACK` (deliberately: alternate
stacks are often small, and the handler decompresses). So when a thread ran
off the end of its stack the kernel had no stack to push smash's frame on and
killed the process before any handler ran — the application's `SA_ONSTACK`
handler, the one JVMs, Go, Rust and crash reporters rely on for exactly this,
never saw the fault. The handler now has `SA_ONSTACK` and runs on a 256 KiB
alternate stack smash gives every thread (`ensureAltStackForThread`, a raw
`mmap` so `SMASH_TRACK_EXTERNAL` can never make a signal stack compressible;
released on thread exit), and the application's chained handler runs on it
too. The application's own `sigaltstack()` is interposed: recorded and
reported back, never installed. Found on the way: `FaultHandler::start()` run
a second time (a fork child restarts the compressor on its first thread
creation) recorded the kernel's current handler — smash's own, inherited from
the parent — as the one to chain foreign faults to, and every such fault in a
child recursed into itself; the missing alternate stack had hidden it behind
the crash. `start()` keeps the chain when the previous handler is its own.
`test_altstack_{large_only,full}`: both overflow children died of SIGSEGV
before, in the main thread with the application's stack registered and in a
thread with none.

## The warming touch lost concurrent stores (fixed 2026-10-29)

Wrappers with no retry surface (`pthread_sigmask`, the affinity calls, stdio,
`getcwd`) touch the first byte of each page of the buffer the kernel will
read or write, so a compressed page is restored and a PROT_READ page made
writable before the real call. The touch was `c = *p; *p = c`: a thread
storing to that byte between the load and the store had its store undone,
and the byte is the first of a page, as likely live data as any other in a
shared large buffer. It is an atomic `+0` now (`touchByteWritable`), which
faults exactly as a plain store does and is indivisible. `test_warm_pages_race`:
a writer verifying its own stores to the first byte of a page while another
thread `pthread_sigmask`s with a set on it lost 7 stores in 2 s before, none
after.

## fork() quiesced only the slabs and LargeAlloc (fixed 2026-10-29)

alloc8 registers atfork handlers that call `SmashHeap::lock()` before the
fork and `unlock()` in the parent and the child. `lock()` took the slab locks
and LargeAlloc's; the child's `resetDecommitThreadForFork` re-initialised the
sub-arena, pool, run-cache and chunk locks. Everything else — the span
descriptor shards, the free maps, the node arenas' locks, the commit and NUMA
init locks, the mid classes, the thread cache pool, the metadata arena's
expand lock, the blob stores' shards, and the page locks — a child inherited
as it was at the clone, and a lock another parent thread held then was held
in the child by a thread that did not exist. The compressor's pause gave up
after 50 ms with a tick in flight, with nothing reset for it. Stress did not
show it (200 forks under 8 churning threads: 0 hung; the holds are tens of
nanoseconds); a thread holding the span shards for 400 ms at the fork hangs
every child. `lock()` now pauses the tick and takes every allocator lock
(`util/fork_locks.h`: in rounds of trylock, so it cannot deadlock against the
threads it waits out, bounded at 3 s), `unlock()` releases them, and
`forkChild()` re-initialises them all plus every held page lock of the
committed range. A tick stops at the next chunk once a fork is pending
(`forEachLivePage`, the reclaim loop).
`test_fork_{large_only,full,no_compressor}`: with the locks the old `lock()`
left out held for 400 ms (`smash_debug_hold_locks`), `fork()` now takes
350 ms and the child runs; a mutant with the old `lock()` forks in 1 ms and
the child hangs, which is the compressor-off registration's job to catch (with
a compressor, the holder also blocks the tick and the pause waits it out).

**Page locks** are not in the set the prepare handler takes: a fault handler
holds one for the microseconds of a decompress, a write through
`/proc/self/mem` and an `mprotect`, and waiting for them would make every
`fork()` wait on every faulting thread. The first version of this fix reset
them in the child only when the tick pause had timed out (`fork_mid_tick_`),
as if workers were the only holders. A clone inside a fault handler's window
left the child's page `PROT_NONE`, still `COMPRESSED` with its blob, and its
lock held by a thread that did not exist: the child's first read of it
faulted and spun in `PageLockTable::lock()` for ever. A `SMASH_SOFTDIRTY=0`
run of the suite hung 2 of 200 children this way (1200 standalone forks did
not; the holds are microseconds). `forkChild()` now resets every held page
lock unconditionally (`PageLockTable::resetForFork`: one word read per 8
pages over the committed range, writes only to the held ones, owner slot
too, so a thread the child creates that reuses a parent thread's `pthread_t`
is not taken for the holder; `heldByThisThread` checks the flag before the
owner). A clone after the `mprotect` and before the state store leaves a
readable page with the right bytes marked `COMPRESSED` and its blob kept:
nothing faults on it or sweeps it (the handler clears its deferred bit
first), so it costs one blob. `test_fork_*`'s page-lock phase holds one
compressed page's lock for 400 ms (`smash_debug_hold_page_lock`) while main
forks and the child reads the page: the mid-tick-only mutant hangs all three
children, the fix runs them.

## Every tick read a byte per page of the arena's high-water mark, twice (fixed 2026-10-29)

Each tick began with `rebuildChunkBitmap`: the state byte of every page from
0 to the committed high-water mark, to learn which 64-page chunks hold a
live page, and ended with `sweepDeferredMadvise` reading every page's
deferred-madvise bit over the same range -- on the coordinator alone, 134 MB
of reads per tick each on a 512 GiB arena that was once full, whatever is
live now, and the sweep for nothing in the default deferred-reclaim mode,
where the bit is never set. `PageStateTable` now keeps the live bitmap itself
on every state store (`noteState` in `set`, `setRange`, `transition`), the
compressor reads it in place (`liveWords()`; `rebuildChunkBitmap` is gone),
and a count of pending pages (`deferred_pending_n_`) lets the sweep return
at once when none is. `smash_tick_cost_info` exports the sweep's scanned
pages, the memread calls and the tick count; `test_tick_scan` leaves a 1 GiB
high-water mark behind 8 MiB live and requires three ticks to scan 0 page
bits (262 144 per tick before), then checks the kept block still compresses
and reads back, which is the bitmap being right.

## exit() decompressed every compressed page before the handler came down (fixed 2026-10-29)

`CompressorThread::stop()` called `drainAllForShutdown()`: every compressed
page restored in place, one thread, a `pwrite` through `/proc/self/mem` and an
`mprotect` each, before the fault handler was uninstalled and the spill
mapping unmapped -- minutes of silent exit for tens of millions of pages, and
every bit of it work the process was about to throw away. `stop()` now stops
the threads and leaves the handler installed, the stores allocated and the
pages compressed: an access after shutdown (an `atexit` handler, a static
destructor) faults and is restored on the accessing thread, as during the
run. `test_exit_drain`: a child compresses 1.5 GiB (364 314 pages), registers
an `atexit` handler that reads 1 MiB of it back, and exits; the time from its
"exiting" line to its end must be under 1 s and the handler must have seen
the right bytes. 19 ms now; 2281 ms with the drain.

## A thread waiting for a page lock spun for as long as the holder held it (fixed 2026-10-29)

`PageLockTable` was a test-and-set byte per page spun on without bound, and
the threads that wait on it are faulting application threads: whoever reads a
page while another thread restores it, every thread when a worker holds the
lock in the non-deferred mode, 128 readers of one hot page as it comes back.
A holder stalled for a second (a `pwrite` through `/proc/self/mem` waiting
on reclaim, a `move_pages`, a debugger) cost a CPU per waiter for that
second. The byte is now three-state (free, held, held with a parked waiter)
and a waiter read-spins for about 20 µs -- longer than a restore -- then
parks on a futex: 1024 hashed wait words (`util/park.h`), the word read
before the byte is marked 2, so an unlock in between (which sees the 2,
bumps the word and wakes everyone on it) makes the sleep return at once;
colliding waiters wake spuriously and re-check. The fault-slot scan also
starts at a rotating offset instead of slot 0, where every concurrent fault
used to collide. `test_page_lock_park`: 16 threads waiting out a 1 s hold
cost 0.01 s of CPU and run 1 ms after the release; the library before,
5.21 s of CPU on 8 CPUs.

## Under the idle and yt trackers every snapshot was two reads through /proc/self/mem (fixed 2026-10-29)

`compressPage` read a candidate page through `/proc/self/mem` so that a
watched page (PROT_READ, or PROT_NONE after escalation) could be copied
without an `mprotect`, and read it that way again to verify the copy. Under
a scanning tracker (`SMASH_WRITE_TRACK=idle|yt`) nothing is ever protected
before compression: the page is ACTIVE and PROT_RW, the pread stopped no
write a memcpy would not have (the shadow window's content compare under
PROT_READ, Phase B, is what catches one in every mode), and each pread took
`mmap_lock` and pinned the page -- `memread=709400 ms` in one tick on the
128-core box, two per page. The batch that makes protread candidates
readable for a direct copy never applied, since it keys on
ACTIVE_MONITORING, which only Phase 3 sets and Phase 3 is skipped under a
scanning tracker. A scanning tracker's ACTIVE page is now copied directly,
snapshot and verify, and the page is still queued for the shadow window's
PROT_READ. The one page a worker cannot read directly is an ACTIVE page an
earlier reclaim left PROT_NONE and nothing touched since (idle, so a
candidate): the worker announces the page it is copying
(`t_snapshot_page_plus1`), and the handler, which used to treat a fault from
the thread holding the page's own lock as a state-machine error, makes that
page PROT_RW and lets the read run. `test_idle_direct_snapshot` (root): 0
reads for 16 384 pages, 16 PROT_NONE pages resolved through the handler;
the keep-pread mutant reads twice per page.

## An ordinary page whose node queue was full had its blob stored off-node (fixed 2026-10-29)

Phase 2 hands each eligible ordinary-heap page to its node's compressor group
through a per-node queue of 64K entries (256 MiB) per tick. When the queue
was full `enqueueForNode` returned false, indistinguishable from "no group",
and the routing worker compressed the page itself into the SHARED store: the
blob landed on whatever node that worker ran on and stayed there for as long
as the page stayed compressed, and nothing counted the overflow. The first
cold wave of a large process (512 GiB on the 128-core box) overflows every
node's queue every tick, so most of its blobs were off-node. The routing
worker still compresses an overflowed page itself, but into the page's own
node's store (`compressPageTo` sets `route_store`/`route_node` around the
call; `workerStore` and `noteBlobStore` honour them), so blob placement no
longer depends on queue capacity; `smash_numa_route_info` counts queued
pages and queue-full events. `test_numa_queue_overflow` (two nodes, workers
pinned to node 0, 512 MiB first-touched on node 1): 0 of 131072 blobs
off-node; the shared-store mutant 65536, exactly the overflow.

## A yt run's node was guessed for every member (fixed 2026-10-29)

With `SMASH_YT_NODE_HINTS=1` a youngtrace scan sets `page_node_` from each
item's node bits. The module keys a run (consecutive present pages of one
size) by size and class only and reports the first page's node, so past the
first member the node was a guess, and `ytMarkItem` filled every unhinted
page of the run with it. A hinted page is never queried, so the guess stuck:
two 16 MiB blocks first touched on nodes 0 and 1, adjacent in the arena, came
back as one 8192-page run, every page of the second was hinted node 0, and 0
of its 4096 blobs landed on its node ("16384 pages hinted, 0 move_pages
queries"; `test_numa_compressor_yt_hints` failed on claudepc from 2026-10-26,
passing once when the blocks were not one run). Fix: the rest of a run is
checked with one `move_pages` call per run (at most 256 pages a call) over
one page per 64 (`kYtSampleStride`, or per member when members are larger)
and the run's last page; each sample is exact, the pages between two samples
take their node only when both agree, and the pages between two that
disagree stay unknown for Phase 2's batch query. Cost 1/64 of that batch;
samples count under `node_query=`. Verified as root on claudepc, three runs
each: fixed 319 pages queried of 8192 and every blob on its node; the
`yt-run-guess` mutant fails all three as before. Resolution 64 pages: a node
change and back inside 64 pages still mishints, costing locality only.

## Static per-worker ranges and no tick budget (fixed 2026-10-29)

Each phase split the committed range into one static slice per active
worker (`committed / nw`). A heap whose live pages sat in one worker's slice
-- the usual shape after a large free, or any live set at the bottom of a
512 GiB arena -- was scanned, compressed, monitored and reclaimed by that one
thread while the others idled at the barrier; `performance.md` records ticks
of 724 s and 1448 s on the 128-core box. `dispatch()` now resets a slice
cursor and `executePhase` takes slices of `slice_pages_` (16 MiB, or a
sixteenth of a worker's share, chunk- and watch-group-aligned, and under a
scanning tracker a multiple of its scan slice too) until the
range is spent, so the imbalance is bounded at one slice; the per-phase
range functions were already self-contained per call (their flushes run at
the end of each). `test_worker_balance`: four workers, 128 MiB live at the
bottom of 1 GiB: 25 % each, against 100 % on worker 0 with one slice per
worker. The first version aligned slices to the chunk only, and a slice edge
cut every yt request (Phase 1 asks the module about the due scan slices
inside its range): `test_yt_tracker_slices` as root on claudepc, one worker,
147456 pages committed, got 9216-page slices and 7399-page requests of a
32768-page scan slice. A phase slice is now also a multiple of
`1 << sliceShift()` under `idle` and `yt`. And a tick compressed everything
eligible, however much:
`SMASH_TICK_PAGE_BUDGET=N` reserves a slot per page before the work
(`tick_reserved_`; a check-then-compress overshot by one per concurrent
worker) and turns the rest away cold for the next tick; `test_tick_budget`:
at most 1000 of 2048 per tick, against 2048 without. The ownership split
between the node groups and the default workers, and the 8-worker pool, are
the next section.

## The node groups and the default workers idled through each other's phases, and the pool was 8 (fixed 2026-10-30)

On a NUMA host the compressor's threads were split by ownership: node arena
chunks to the node's group (each thread a static slice of each chunk),
everything else to the default workers, and phase 5 -- the pages phase 2
routed to each node's queue -- to the group alone, each thread a static
slice of the queue. An ordinary heap carries no binding, so the default
workers alone walked phases 1-4 while every group idled at the barrier, and
in phase 5 the default workers idled while the groups compressed the whole
cold set: on the 128-core box, 8 threads working and 8 waiting in every
phase. The pool of default workers was a compile-time 8
(`kMaxCompressorWorkers`), which the Little's-law estimate hit every tick
there (`performance.md`: 724 s and 1448 s ticks).

Now a phase's work is pools claimed with atomic cursors by whichever
compressor thread asks next: the shared range in slices (as before), each
group's chunks in 8 MiB units (`takeNodeUnits`, aligned like the slices)
and each group's queue in `kCompressBatch` batches (`drainQueueBatches`).
A default worker takes the shared range, then helps the groups, the node
its CPU is on first; a group thread takes its own node's pool, then the
shared range (skipping every group's chunks as a default worker does:
`ownedByRunningNodeGroup` now keys on `tl_on_node_unit_`, not on which
thread asks), then the other groups'. The groups are woken for phases 1-4
whenever the shared range has more slices than the default workers can
start at once. Placement never depends on who compresses: a thread draining
another node's queue sets `route_store` to that node's store for the whole
drain, and a node arena page met on a unit by another node's thread is
routed to its node's queue. The spill path (`SMASH_SPILL`) honoured only
`worker.store`, so a default worker's overflow-routed blob could have
spilled off its node; it checks `route_store` too. `test_numa_work_sharing`
(two default workers pinned to node 0, one thread per group, 256 MiB on
node 1 under a 1 GiB high-water mark): 83 % of node 1's compressions by
threads outside its group, 220 shared-range slices taken by group threads,
0 of 65536 blobs off-node, against 0 and 0 with the split restored.
`test_numa_compressor` counts `helped_compressions` with the group's own
(its blob-placement check is what guards where helped blobs went).

Sharing phase 2 broke the node queues' adjacency. `enqueueForNode` took one
queue slot per page (`fetch_add(1)`), so threads routing at the same time
interleaved a queue page by page; with the default workers alone routing
that was rare, with every group thread routing too it was the rule, and
phase 5's 256-page drain batches became runs of two or three pages:
`test_phase3_batch` measured 3.4 pages per phase-2 `mprotect` (19018 calls
for 65536 pages; 128 per call with the sharing mutated out or without
groups). A routing worker now stages its pages per node
(`CompressWorker::route_stage`) and `flushRouteStage` moves 256 at a time
into one reservation of `kCompressBatch` slots, the unused tail of a short
one (a range's end) filled with `NodeGroup::kQueueEmpty`, which
`drainQueueBatches` skips: every drain batch starts at a reservation and is
one worker's adjacent pages. The pages past the queue's capacity are met at
the flush and compressed by the router into the node's store
(`compressLocalBatch`, shared with `flushNodeQueries`), as the full-queue
page was before. 128 pages per call again (127.8 at 2 GiB); the mutant that reserves a
slot per page measures 4.1 and fails the test.

The pool is 32, allocated lazily (`ensureWorkerState`: a `CompressWorker`
is ~190 KiB, mostly bucket stats, so the array of 8 objects became
pointers). A worker created later starts from worker 0's buckets, as a
group's workers do -- `seedBucketStats`: the ratio window, the cost EMAs and
the profile fields, never the four totals (`compress_count`,
`decompress_count`, `bytes_saved_total`, `time_cost_total_us`). Those are
summed over every worker by the saved profile's thrash rate, the budget's
efficiency and the stats dump, and the first version copied them whole:
`test_compression_bucket_stats`, whose sum walks all 32 worker slots,
counted 63 compressions as 1953 (31 copies) and saved thrash rate 8/255
`stable_cold=1` for a block restored whole. The group workers' copy had the
same flaw, hidden because they are created in `start()` before any tick.
The runtime cap is `SMASH_MAX_WORKERS`, default `clamp(nproc / 4, 8,
32)`, so hosts of up to 32 CPUs are unchanged; every worker and helper
thread up to the cap is created in `start()` (a `pthread_create` mid-tick
is unsafe), and `SMASH_TEST_WORKERS` raises the cap with it.
`test_worker_pool`: 12 workers asked for, 12 reported, 12 % on the busiest;
8 reported with the old constant.

## realloc always copied (fixed 2026-10-29)

`SmashHeap::realloc` of any smash-owned block was malloc + memcpy + free,
whatever the sizes: shrinking a 1 GiB block to 512 MiB copied 512 MiB and
held 1.5 GiB until the free, growing a buffer copied it however much free
room followed it, and realloc to the same size copied too. In place now
(strictly less memory than before: the copy's second block never exists). A
large block: a new size within its pages just changes `large_size`; fewer
pages release the tail exactly as a free would (`LargeAlloc::releaseArenaRun`,
factored out of `deallocate`: release hook, sub-arena dirty run, run cache,
decommit queue; `munmap` outside the arena), except for a block larger than
a sub-arena chunk, whose release returns whole chunks (`runIsChunkBig`:
copy path); more pages are taken from behind the block when they are free
(`VmRegion::tryExtendRun`: the shared arena's bump pointer by CAS, a free
run of the shared map, a sub-arena chunk's unused tail or one of its dirty
or clean runs under the sub-arena's lock, never across a chunk boundary or
in a node arena; `mremap` in place outside the arena), and set ACTIVE. A
small or mid block stays where it is when the new size fits and uses more
than half of it. `test_realloc_{large_only,subarenas,full}`: same pointer
for the in-place cases, the data intact, the tail's pages back in the arena;
the library before moved every one.

The first version of `tryExtendRun`'s bump-pointer path moved `next_page_`
and nothing else; `allocatePages` also calls `ensureCommitted`, and
`kCommitChunkPages` is 0, so the commit watermark is exactly the bump
pointer. The grown tail sat above it, PROT_NONE: every first touch faulted
and the handler's ACTIVE path made the page PROT_RW, so nothing showed --
until the next fresh allocation's `ensureCommitted` swept `mprotect(PROT_RW)`
from the old watermark over the tail, whose pages were by then compressed and
dropped. Reads returned zeros without a fault (state still COMPRESSED), writes
landed on memory the compressor owned and were lost at the next drop, and a
later restore brought the old blob back. Found by
`test_integrity_stress_small_heap` as root on claudepc, 1 run of 13: 21 wrong
units, all in 51-87 of one block, grown at round 18432 (the test reallocs
every 512 rounds), zeros first, then versions behind. The path now commits
the tail like `allocatePages`; `test_realloc_commit` reads the compressed,
dropped tail back after a 1 MiB malloc: intact, against 1024 zero pages with
the line removed.

## The 16 KiB-1 MiB band was a mapping per malloc under large-only (fixed 2026-10-31)

The band sits between the slab's top size and the compressible threshold.
`SMASH_LARGE_ONLY=1` handed everything below 16 KiB to glibc and everything above
it to `LargeAlloc`, which sends sizes below the threshold to `mmap`: never
compressed, and every block a syscall pair, a fault per page and a TLB shootdown,
serialized on `mmap_lock`. Nothing argued for that; it was what was left between
two allocators. The band is glibc's now (`largeOnlySystemSize`), whose dynamic mmap
threshold and bins serve a churned 320 KiB block with 0 faults against 80. A system
block that `realloc` grows to a compressible size moves to the arena once, or a
buffer grown from small would have stayed resident for ever (it did, from 16 KiB,
before this too). Full mode has no glibc path for the band and keeps its mappings;
`SMASH_MID_BINS=1` stays opt-in there, 64-thread churn having measured slower with
it. Whether this explains the slowdown on the 128-core application is not known.

## Fresh region was taken before freed pages on their way back (fixed 2026-10-31)

`allocateOnSubArena` carved a chunk once the runs it could see missed, and waited
for frees in flight only after the region was out; `allocatePages` bumped likewise.
A run in the decommit queue or out of the map for a purge's madvise is invisible
to that check, so a purger behind the frees became address space, and the bump
pointer never retreats. `smash_sim` with 512 MiB live, half the frees cold and a
200 us OS call walked **all of a 64 GiB arena in 8 s**, on the shared path, and
on the sub-arena paths once the drain was removed from them. A miss now drains the
queue itself first, and sub-arenas wait up to 2 ms for purges in flight: 1.4-2.9 GiB
walked, and throughput did not suffer (shared 13.7k to 25.7k operations per
second; the sub-arena cases 29k to 35k and 70k to 75k). This is the
mechanism that fits "the arena grows while RSS stays the same" reported from the
128-core box; it has not been measured there. `test_smash_sim_walk_*`.

## Free-run search: an unbounded walk, and dirty and clean runs that never merged (fixed 2026-10-31)

Bins above 64 pages were one per power of two, so a request met runs shorter than
itself and walked them first-fit under the lock: 10000 runs of 65 pages and a request
for 66 walked all 10000 on each miss, on each sibling. Now four bins per power of two
and at most `kTakeScan` runs of the own bin, then the next non-empty bin, whose runs all
fit. A bounded miss costs fresh region at worst; `takeAny` / `kScanAll` is for when
there is none.

Each sub-arena had a dirty and a clean owner, whose runs could not merge. A purge split
a run into a dirty head and a clean tail for good, and a chunk of nothing but free
pages served nothing longer than its longest run of one kind. Runs hold both now, with
a bit per page and a count per run. The measurements, and what became of
`takeMixedFromSubLocked`, are in [design-subarenas.md](design-subarenas.md); the short
form is twice the throughput at a third of the faults on mixed sizes and 35 % less
address space walked, with no loss where blocks reuse exactly. No preference for dirty
pages remains but the bins' LIFO order, which was the risk the measurement was for.

## The idle tracker forbade huge pages, and scanned freed address space (fixed 2026-10-31)

`SMASH_WRITE_TRACK=idle` put `MADV_NOHUGEPAGE` on the arena because the bitmap answers
for a large folio through its head alone: `page_idle_get_folio` returns nothing for a
tail, so 511 pages of each huge page read accessed for ever. So a heap under this tracker
faulted 4 KiB at a time. The scan now reads `/proc/kpageflags` and takes each page of
a large folio to its head, which also covers a folio that is mapped in part, the state a
partial purge or compression leaves it in until memory pressure splits it. On huge pages
one bit answers for 512 pages: 70 ns per page against 1.2 us on claudepc. The flags read
adds 45 % to a scan of 4 KiB pages, so it happens only where the host's THP or mTHP
settings let the arena hold large folios. A host on `always` now gets huge pages under
this tracker, with what that costs a sparse workload (22x RSS, see allocation-paths.md);
`SMASH_THP=0` forbids them. Not measured on the arm64 6.6 kernel the tracker was written
for: x86 6.8 only.

The scan also read pagemap for every 1024 pages of the range whether or not anything
lived there. Freed address space stays below the high-water mark for good, so it cost
2048 reads per tick per 8 GiB ever walked. Batches with nothing live are now skipped.

## A fork left the shared sub-arenas', compressor's and metadata arena's locks out (fixed 2026-10-31)

`ForkLockSet` held 4096 locks and `add()` dropped the rest without a word. The fork
fix of 2026-10-29 (304da9f) listed 2 x `kMaxNumaNodes` = 2048 node-arena locks ahead of
the shared set's sub-arenas, the compressor's and `BootstrapAlloc`'s, so with the slabs and
shards the set was full before it got to them: the prepare handler never took them, and
the child inherited whatever they were protecting. It was found when the free-run changes
of the next batch made a child crash often enough to see: `test_fork_no_compressor`,
SIGSEGV in about 1 run in 14 at `tags_[0xFFFFFFFF]`, a size bin whose head had been cleared
and whose bitmap bit had not. A check run under the prepare handler found that state with
every lock in the set held and `lockAll` successful, which is what pointed at the set.
The committed library has the same full set and did not crash in 40 runs, so the window
was there before; why the later code hits it more often was not established.
`smash_debug_hold_locks` holds the same truncated set, so the test's rounds with the locks
held could not have noticed. The limit is 32768, against 16953 at 128 arenas; 0 of 60
runs after the fix, and `fork_test` fails if the set ever fills.


## memalign used its alignment unchecked (fixed 2026-11-05)

alloc8 validates the alignment of `posix_memalign` (a power of two, a multiple
of `sizeof(void*)`) and `aligned_alloc` (a power of two). Its `memalign`,
`__libc_memalign`, `valloc` and `pvalloc` call `xxmemalign` with whatever they
were given, and `SmashHeap::memalign` used it as it came. glibc's `memalign`
accepts any alignment: above `SIZE_MAX / 2 + 1` is `EINVAL`, one that is not a
power of two is rounded up to the next, one below 16 is `malloc`'s. Measured on
the library before, full and large-only alike:

| call | glibc | smash before |
|---|---|---|
| `memalign(SIZE_MAX, 1)` | NULL, `EINVAL` | `mmap` of 4095 bytes, `munmap(0x1000, 122 TiB)`, SIGSEGV |
| `memalign(2^63 + 1, 1)`, `(2^63 + 4096, 1)` | NULL, `EINVAL` | not `EINVAL` |
| `memalign(12288, 100)`, `(20480, n)`, `(3 MiB, 100)` | 16 KiB / 32 KiB / 4 MiB aligned | 4 KiB / 8 KiB / 1 MiB aligned |
| `memalign(24 / 48 / 96 / 3 MiB, n)` under `SMASH_PASSTHROUGH=1` | a block | NULL, `errno` 0 |

The crash: `LargeAlloc::allocate`'s over-map size `alloc_size + alignment`
wrapped to 4095, and the trim after it unmapped a "prefix" of most of the
address space. The misalignment: `roundUp` assumes a power of two. Passthrough
and compress-only forward to the system's `posix_memalign`, which rejects what
glibc's `memalign` accepts.

Fixed in `SmashHeap::memalign`, first, before any mode's path: glibc's rules
(`EINVAL` above `SIZE_MAX / 2 + 1`, `std::bit_ceil` for a non-power of two,
`kMinAlignment` as the floor). `LargeAlloc::allocate`'s over-map now has the
same wrap check as its node path, which cannot fire with the alignment kept at
most `2^63` (a block is at most 2^44 bytes) but keeps the trim from unmapping
what follows a wrapped mapping. Guarded by `test_alloc_contract_*`'s
`testMemalignAlignments`: the library before failed 26 checks in full mode and
22 in large-only (the `SIZE_MAX` calls crash in their own child), and the
passthrough registration (`SMASH_PASSTHROUGH=1`, the same binary) fails 58.

## A slab lock held across the purge, and waiters that spun (fixed 2026-11-10, issue #49)

Full mode only. Two defects that multiplied each other, and a third thing
that is not fixed.

**The purge held the slab's lock across one `madvise` per span.**
`Slab::purgeResidentEmpty()` runs on every compressor tick. It took `lock_`
and walked the whole empty list, calling `vm::decommitPages` for each span
that had been empty for two ticks. After a large free that is tens of
thousands of system calls in one hold, and every `malloc` that needs a refill
from that slab (same arena, same size class) waits for all of it. Measured on
the unmodified library, one thread, batches of 64 `malloc` + 64 `free` after
freeing N MiB of the same size:

| Object size | Freed | Slowest batch before | after |
|---|---|---|---|
| 256 B | 256 MiB | 96-98 ms | 1.0-1.3 ms |
| 256 B | 768 MiB | 285-296 ms | 7.1-7.8 ms |
| 256 B | 1024 MiB | 388-392 ms | 6.6-7.9 ms |
| 64 B | 768 MiB | 585-813 ms | 5.9-6.6 ms |
| 4096 B | 768 MiB | 169-188 ms | 7.1-8.6 ms |

(Three runs each, 8 CPUs. `strace -c`: 49,159 `madvise` calls for the 768 MiB
of 256-byte objects, 777 now. On the 2-CPU host, where the cap on resident
empty spans is 122 MiB: 32-33 ms before, 0.6 ms after.)

Resident empty spans now have their own list (`resident_`, newest at the
front, so the old ones are at its back and the purge needs no scan). The
purge takes 64 of them off the list under the lock, releases it, drops their
pages (neighbours in one call: they empty in the order their objects were
freed, so the list is nearly address-ordered), takes the lock again and puts
them on `empty_`. While they are off the lists no allocation can be handed
one. An allocation that finds both lists empty during that window carves a
new span: at most 64 spans per slab, and only after every other empty span is
used up. The purge also stops at its next batch when a `fork()` is waiting
for the tick (`CompressorThread::forkPauseFlag`): a fork during a purge
waited for all of it.

The 6-8 ms that remain after the larger frees are not the purge: that batch
is at the first tick after the free (0.8-1.4 s), the purge comes a tick
later, and the library before has it as its second slowest batch.
`SMASH_WRITE_TRACK=none` halves it (7.2 to 3.6 ms), and a run with the
compressor off showed a 6.8 ms batch of its own, so part of it is the write
tracker's pass over the process (issue #23) and part is the host. Not chased.

**A waiter spun for as long as the hold lasted.** `Spinlock` read the lock
word in a loop and called `sched_yield()` every 1024 pauses; its comment said
1024 pauses outlast any critical section. Behind the purge every thread on
the slab burnt a CPU for the whole hold (four threads on one arena: 344 ms
each, 1.4 s of CPU for one purge; now their slowest batch is 3.8-4.9 ms with
0.1 ms of CPU in it). A waiter now spins for about 20 µs, bounded by the clock and not by a count of pause instructions (one takes
16 ns on the Zen 3 here, about a cycle on some arm64 cores), then marks the
word (0 free, 1 held, 2 held with a sleeper) and sleeps on it as a futex;
`unlock()` is an atomic exchange and wakes one sleeper when it finds the
mark. A thread that has slept takes the lock marked, so its own unlock wakes
the next. `lock()` saves and restores `errno` around the sleep (`free()` must
not change it, and an interrupted futex wait sets `EINTR`).

**Not fixed: the wait behind a holder the scheduler has descheduled.** With
more threads than CPUs a thread is regularly taken off its CPU while it holds
a slab lock, and whoever needs that slab then waits until the scheduler runs
the holder again. Sleeping waiters stop competing with the holder for the
CPU, which recovers throughput, but the wait itself is the scheduler's:

| 64-byte objects, each thread its own FIFO of 1024, 3 s | Throughput against one thread per CPU | Lock waits over 1 ms | Longest | All lock waits together |
|---|---|---|---|---|
| 16 threads on the 2-CPU host (4 arenas), before | 0.84 | 1236-1292 | 88-128 ms | 17.9-19.1 s |
| after | 0.90 | 1240-1296 | 60-86 ms | 17.1-17.8 s |
| 16 threads pinned to 2 of 8 CPUs, 4 arenas, before | 0.90 | 849-876 | 98-120 ms | 18.2-19.1 s |
| after | 1.00 | 820-915 | 81-145 ms | 14.8-16.2 s |
| 4 threads pinned to 1 CPU, 1 arena, before | 0.97 | 406-422 | 27-48 ms | 3.4-3.8 s |
| after | 0.98 | 324-413 | 10 ms | 1.4-1.7 s |

Nothing a lock does while waiting can shorten that: a futex has no way to
hand the waiter's time slice to the holder, and priority inheritance does
nothing between threads of equal priority. It needs the holder not to be
preempted in the middle of a critical section, which a thread on these
kernels (6.8) cannot ask for, or the slab not to be shared. With every CPU
busy the time is not lost to the process, as the throughput column shows.
glibc does not show these waits: its threads do not share an arena here.

**The slab's layout.** The fix added one list to `Slab` (24 bytes) and the
benchmarks moved by more than the lock change explains: four threads on one
slab +13%, producer/consumer pairs -4%, and with 80-byte objects four threads
on one slab -27%. A slab was 200 unaligned bytes in an array, so which fields shared a cache line depended on the size class and the
arena; one more list shifted all of it. What matters is the line `lock_` is
on: waiters read it in a loop, so whatever the holder writes there is fetched
back by every waiter and has to be taken from them again, and whatever
freeing threads write there does the same to the holder. A slab is now 256
bytes and cache-line aligned: `lock_` with the fields only `init()` writes,
the span lists on the next two lines, the deferred-free fields on the last.
Numbers and the layouts tried: [performance.md](performance.md), "The slab's
cache lines".

Guarded by `test_slab_purge_unlocked`, `test_slab_purge_fork` and
`test_spinlock_park` ([tests.md](tests.md)).

## smash's own threads allocated from the arena (fixed 2026-11-10, issue #7)

A thread smash creates must not keep its data on pages smash compresses. The
compressor is what takes those pages away: its own data would be compressed
like any other, and it would fault on it when the fault cannot be served.
While it holds that page's lock the handler refuses (the lock is not
recursive) and the SIGSEGV is fatal; while it holds another page's lock, two
workers doing that to each other deadlock; inside the fault handler the fault
re-enters it. So a compressor thread's allocations are meant to go to the
system allocator, and `mallocSlow` and `calloc` sent them there.

Only those two did, and only when reached:

- Every thread started through the thread hook got a thread cache in
  `threadInit`, smash's own included, before its entry function set the mark.
  `malloc`'s fast path, and since the upstream merge the lean `malloc` and
  `calloc` entries, look at nothing but the cache.
- `memalign` (and so `posix_memalign`, `aligned_alloc`, `valloc`) never looked
  at the mark.
- VmRegion's decommit thread was not marked at all.

Measured with a test program that defines `usleep()`, `madvise()` and
`sched_getcpu()`, which libsmash calls on its own threads, and allocates from
inside them: in full mode 17 of smash's threads were reached and on every one
`malloc(100)`, `calloc`, `realloc(NULL, 300)`, `posix_memalign(64, 256)`,
`new char[100]`, a `realloc` to 5000 bytes and `strdup` returned arena
memory; in large-only mode, the production mode, `aligned_alloc(4096, 2 MiB)`
on the coordinator did. In gdb all 16 threads with the mark had a thread
cache. No crash was reproduced: these threads allocate little (a `FILE` for
`/proc`, the profile file), so it stayed latent.

Fixed:

- `startSmashThread()` (`util/thread_stack.h`) starts every thread smash
  creates. It calls libc's `pthread_create` (inside libsmash the plain name is
  alloc8's interposer, whose trampoline runs the application's thread hook
  first) with a trampoline of its own that sets the mark, `g_smash_thread`,
  before anything else runs on the thread, then calls `threadInit` for what
  such a thread still needs from it (the exit watch, the alternate signal
  stack), then the thread's function. The function and its argument travel in
  a fixed table: the new thread must not allocate, and a block from the
  creating thread would be the arena's.
- `getOrCreateThreadCache()` returns nullptr on a marked thread, so every
  allocation on it takes the slow path, where `malloc` and `calloc` already
  checked. `memalign` checks now. `realloc` needs nothing: it allocates
  through `malloc`.
- Each entry function calls `enterSmashThread()` first. On a thread the
  trampoline started it does nothing. On one that came through the
  application's hook after all (libc's `pthread_create` not found; macOS,
  where the trampoline is not used) it sets the mark and hands the thread
  cache back.

All 16 marked threads now have no thread cache (gdb), which is also about
1 MiB less metadata (the stats line's `boot=` 21 to 20 MiB at 16 arenas).

Not done: the Mach exception thread on macOS is started as before and not
marked; nothing here was built or run on macOS.

Guarded by `test_smash_thread_alloc_full` and
`test_smash_thread_alloc_large_only` ([tests.md](tests.md)).


## The cold timeout switched the ROI gate off, and a declined bucket stayed declined (fixed 2026-11-10, issues #10 and #22)

Two defects in `CompressionROI::shouldCompress`, the gate that decides before
any codec runs whether a page is worth compressing. The first hid the second
from every test.

**The timeout was also a policy switch (#10).** The gate returned true for
every page past the floor whenever `SMASH_COLD_TIMEOUT_SEC` or
`SMASH_COLD_TICKS` had been given a valid value ("ablation: if cold_ticks was
explicitly overridden, use fixed threshold only"). The timeout is the
documented time-space dial and what every test and benchmark sets, so all of
them ran a policy the default configuration never runs. One worker, a 32 MiB
block of random bytes and then two of compressible bytes from the same call
site, 36 s:

| Setting | tried | stored | gate_fail | roi_skip |
|---|---|---|---|---|
| none (the default, 10 s) | 49152 | 0 | 8 | 49144 |
| `SMASH_COLD_TIMEOUT_SEC=10` | 32768 | 16384 | 16384 | 0 |
| `SMASH_COLD_TICKS=10` | 32768 | 16384 | 16384 | 0 |

The same timeout, and a different outcome for every page: with it set, the
codec ran on all 8192 random pages, twice; without it, nothing was compressed
at all. `test_large_bucket_decline` had to be registered without the timeout
for this reason, and a comment in its test said so.

**A declined bucket could not recover (#22).** The first row is the second
defect. The gate decides from the bucket's window of compression ratios
(`SizeClassStats`, 64 samples per worker, arena and size class), and a sample
is recorded only after a codec run. Once the window holds nothing but pages
that saved nothing, its mean is 0, the benefit is 0 at any cold time, every
page is declined, and nothing can change the window again. With fewer than 8
samples the gate passes everything, so 8 incompressible pages are enough on a
fresh bucket; on a used one, 64 in a row. In large-only mode every block of
128 pages or more from one call site's arena is one bucket, so one buffer of
random, encrypted or already compressed bytes switched compression off for
every later large block of that call site, on that worker.

Fixing the first alone would have put the second into every run that sets
the timeout: such runs had been compressing everything past the floor, and
would have started losing whole buckets. So they are fixed together.

- The timeout sets the floor and nothing else. `ROIConfig::cold_ticks_overridden`
  is gone. `SMASH_ROI_THRESHOLD=0` switches the gate off (every ROI is at
  least 0), for a run that must compress every page past the floor.
- Every 64th page a bucket's statistics decline goes to the codec anyway
  (`kRoiProbeEvery`, `SizeClassStats::declined_run`; the hard floor is never
  probed). A probe that fails the ratio gate is recorded like any failure. A
  probe that is stored restarts the window (`restartRatios`), so the pages
  after it are judged by what the bucket holds now: with fewer than 8 samples
  the gate passes them, and their ratios fill the window. Without the restart
  the old samples outvote the probes; at a 2 s timeout 1582 compressible
  pages were declined before the bucket recovered.
- A declined page backs off as before (cold streak restarted, effective floor
  doubled), so a bucket of incompressible pages costs one codec run per 64
  declines: 128 MiB of random bytes over 25 s, 32768 attempts, about 32200
  declined, about 570 codec runs (16 before the probe).

The same program after the fix, in all three settings of the table:
`tried=32775 stored=16377 gate_fail=321 roi_skip=16077 roi_probe=255`. With
`SMASH_ROI_THRESHOLD=0`: `tried=32768 stored=16384 gate_fail=16384
roi_skip=0`, what the timeout used to give.

Not done:

- All-zero pages in a declined bucket are still declined: the gate runs
  before the snapshot that would recognise them (the last sentence of #22).
- `kMaxEffectiveFloorTicks` (200) is below `kMaxColdTicks` (254), so the
  back-offs cannot lengthen a timeout above 200 s and at most double one
  above 100 s (the "Related" note in #10). A page's idle count is one byte,
  so no cap can give a 254 s timeout a longer back-off; raising 200 to 254
  would change how long thrashing pages wait at the default timeout, which
  nothing here measured.
- The benchmarks that set the timeout now run the default policy. Their
  published numbers were taken with the gate off and have not been taken
  again.

Guarded by `test_roi_gate_recovers`, `test_roi_gate_recovers_timeout_set`
and `test_large_bucket_decline_timeout_set` ([tests.md](tests.md)).
