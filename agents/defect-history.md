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
- `yt` (2026-10-13, opt-in, `SMASH_WRITE_TRACK=yt`) — the same accessed-bit signal as `idle`, read through the youngtrace module's page-table walk (`/dev/youngtrace/scanner`, `mode=1`) instead of the bitmap: no PFNs, no rmap walk, 8-28 ns per page against the bitmap's 380-440. The module zeroes its whole output buffer after every request (~110 us at the default 8 MiB), so yt slices are 64 MiB and requests as large as the buffer can always hold. A cut request cannot be resumed the way yt's own tools resume: in `mode=1` the pages the module cleared but could not report are lost, so they count as accessed instead. Requests are sized so none can be cut. As first written, a reported item marked only the pages inside its own request; a huge page split by a request edge lost its accessed bit to the first request and its other part read idle, and hot pages were compressed (`test_yt_tracker_thp`: 128 shadow and 86 decompression faults). Every page of the item is marked now.

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
reach the decline path.


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
