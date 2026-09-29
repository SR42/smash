# Syscall & Buffered I/O Compatibility

Smash's mprotect-based monitoring (PROT_READ) and compression (PROT_NONE) can conflict with kernel syscalls that access userspace buffers. **The kernel does not raise a signal for syscall-side faults**: when `copy_from_user`/`copy_to_user` (Linux) or `copyin`/`copyout` (macOS) hits a protected page during a syscall, the kernel uses the page-fault fixup path to convert the fault into `-EFAULT` and discard the address. The SIGSEGV/SIGBUS handler in `fault_handler.h` only fires for direct user-code accesses; for syscalls we have to detect EFAULT ourselves and trigger a userspace touch (which *does* go through the handler) so the page can be decompressed.

## EFAULT-driven decompress-and-retry (`syscall_compat.h::retryWithDecompress`)

Every buffer-taking syscall wrapper follows the same shape: call the real syscall, on `errno == EFAULT` walk the buffer pages (one byte per page; the SIGSEGV handler decompresses), then retry. Bounded to 8 attempts with us-scale exponential backoff (1, 2, 4, ..., 128 us); the compressor tick fires at ~10 ms intervals so 8 attempts at 128 us is well inside one tick. Bounded so unmapped-pointer bugs surface as EFAULT instead of livelocking. `retryMachOnInvalidData` is the same shape but keyed on `MACH_RCV_INVALID_DATA` / `MACH_SEND_INVALID_DATA` for the mach_msg trio.

The PageState CAS + per-page lock are sufficient for correctness — there is no separate pin counter. The fault handler takes the same per-page lock as the compressor's `transition(ACTIVE -> COMPRESSING)`, so syscall-retry -> walk -> fault -> handler -> decompress -> retry is correct without a pin counter.

## DYLD interposition limitations (macOS) — the buffered-I/O carve-out

`__DATA,__interpose` only intercepts **cross-dylib** GOT calls. Intra-libSystem calls are invisible:
- `fread()`/`fgetc()` -> internal `read()`: NOT intercepted (same dylib)
- `getc_unlocked` is a macro (`__sgetc`) inlined into callers, calls `__srget` for refills
- C++ iostream (`std::cin`) -> `getc_unlocked` -> `__srget` -> `read()` (all intra-libSystem)
- Direct `read()` from application code: IS intercepted (cross-dylib)

For these paths the EFAULT-retry model does not help — those syscalls happen inside libc and never enter our wrapper. Two carve-outs exist:

1. **Buffered I/O wrappers** (`fread`/`fgets`/`fgetc`/`getc`/`fwrite`/`fflush` on both platforms): proactively `warmPages` the user buffer + the FILE's internal buffer (macOS `stream->_bf._base`, Linux `stream->_IO_buf_base`) before delegating to libc. There is no retry path because libc's internal `__read` is intra-dylib.
2. **stdio buffer warming** (`compressor_thread.h::warmStdioBuffers`, macOS-only): re-warms `stdin`/`stdout`/`stderr` FILE struct + buffer each compressor tick so the kernel never finds them protected. Without this, intra-libSystem `getc_unlocked` / `__srget` would EFAULT inside libc with no retry surface.

## Interposed functions

- macOS (`smash_heap.cpp`): `read`, `write`, `pread`, `pwrite`, `readv`, `writev`, `recv`, `send`, `recvfrom`, `sendto`, `recvmsg`, `sendmsg`, `poll`, `kevent`, `kevent64`, `mach_msg`, `mach_msg_overwrite`, `mach_msg2_internal`, plus the buffered-I/O carve-out above.
- Linux (`linux_syscall_wrappers.cpp`): everything macOS has minus the Mach trio plus `ppoll`, `select`, `pselect`, `accept`, `accept4`, `recvmmsg`, `sendmmsg`, `getsockopt`, `getsockname`, `getpeername`, `getrandom`, `epoll_wait`, `epoll_pwait`, `sched_setaffinity`, `sched_getaffinity`, `pthread_setaffinity_np`, `pthread_getaffinity_np`, plus the `stat`/`statx`, xattr, `statvfs`/`getrusage`/`prlimit64`, `getdents64`/`readlink` and `preadv`/`pwritev` families and `getcwd` (see the file for the full set).
  - Plus the fortified `__*_chk` aliases of the EFAULT-retry family (`__read_chk`, `__pread_chk`, `__pread64_chk`, `__recv_chk`, `__recvfrom_chk`, `__poll_chk`, `__ppoll_chk`, `__readlink_chk`, `__readlinkat_chk`) — see "_FORTIFY_SOURCE and the `__*_chk` aliases" below for why the plain names alone are not enough.
  - Plus the Round 6 set below: `epoll_ctl`, `ioctl`, `bind`, `setsockopt`, `sendfile`/`sendfile64`, `process_vm_readv`/`process_vm_writev`, `mincore`, `uname`, `sysinfo`, `statfs`/`statfs64`, `getrlimit`/`setrlimit`, `getgroups` (and `__getgroups_chk`), `getcpu`, `prctl`, `sigprocmask`, `pthread_sigmask`, `nanosleep`, `clock_nanosleep`, `utimensat`, `pipe`/`pipe2`, `socketpair`.
  - Plus the signal-handler installers, which are not EFAULT wrappers: `sigaction`, `__sigaction`, `signal`, `bsd_signal`, `ssignal`, `sysv_signal`, `__sysv_signal`, `sigset`. For SIGSEGV and SIGBUS they hand the application's handler to `FaultHandler::interposedSigaction`, which makes it the handler smash chains to and keeps smash's own in the kernel; every other signal goes to libc (see "An application's SIGSEGV handler displaced smash's" in [defect-history.md](defect-history.md)). All are at the base glibc version, so the base export is what callers bind.
  - The affinity four are wrapped because a thread pool keeps one `cpu_set_t` per worker on the heap. `pthread_{set,get}affinity_np` need wrappers of their own: glibc reaches the kernel through an internal `__sched_setaffinity`, so interposing `sched_setaffinity` does not cover them, and they report failure by return value rather than through `errno`, so they warm the mask up front instead of relying only on the EFAULT retry.

## Platform-specific interposition patterns

**macOS**: Do NOT use `dlsym(RTLD_NEXT)` — it returns the wrapper itself. Instead read the `.original` field from the interpose struct:
```cpp
extern "C" ssize_t smash_read(int fd, void* buf, size_t count);
SMASH_INTERPOSE(smash_read, read);
extern "C" ssize_t smash_read(int fd, void* buf, size_t count) {
    auto* vm = smash::g_smash_vm_region;
    return smash::vm::retryWithDecompress(
        [&] { return reinterpret_cast<read_fn>(smash_interpose_smash_read.original)(fd, buf, count); },
        [&] { if (vm && buf && count) smash::vm::walkPagesForFault(buf, count, vm); });
}
```

**Linux**: Use `dlsym(RTLD_NEXT)` with lazy resolution in `linux_syscall_wrappers.cpp`. For versioned glibc symbols (e.g., `epoll_wait@GLIBC_2.3.2` used by libevent), create aliased wrapper functions with `.symver` directives and export both versions in `smash_version_script.map`:
```cpp
// Wrapper for GLIBC_2.3.2 version
SMASH_VISIBLE int epoll_wait_232(...) { return epoll_wait(...); }
__asm__(".symver epoll_wait_232,epoll_wait@GLIBC_2.3.2");
```

**A wrapper only wins if it is exported at the version the caller binds.** This is not a corner case: glibc versions `sched_setaffinity@GLIBC_2.3.4`, `pthread_setaffinity_np@GLIBC_2.34` and `pthread_getaffinity_np@GLIBC_2.32`, every caller binary references those exact versions, and the base-version (`GLIBC_2.2.5`) definitions added first never ran at all — the syscalls kept returning EFAULT with the wrappers apparently in place.

The same trap applies beyond the EFAULT wrappers: alloc8 exports `pthread_create`/`pthread_exit` to wrap every thread in its init/cleanup trampoline, glibc 2.34 merged libpthread into libc and made `GLIBC_2.34` their default version, and the base-only export meant the interposer **never ran for a single application thread** on any modern glibc — measured 16 wrapped out of 200, all 16 smash's own threads, each unwrapped thread leaking a 92 KiB `ThreadCache`. Both are now aliased at `GLIBC_2.34` as well. When adding any interposed function, check both sides:

```bash
objdump -T <caller-binary> | grep <symbol>          # the version a caller asks for
nm -D --with-symbol-versions libsmash.so | grep <symbol>   # must list that version too
```

## _FORTIFY_SOURCE and the `__*_chk` aliases

glibc's fortify headers rewrite a call whose destination object size is known at compile time into the `__*_chk` alias: `read(fd, buf, n)` becomes `__read_chk(fd, buf, n, sizeof buf)`. The plain symbol is then never referenced, so interposing `read` alone misses the call and the kernel's EFAULT goes straight back to the application. `_FORTIFY_SOURCE` is on by default on Ubuntu, Fedora and RHEL, and any read into a fixed-size buffer qualifies — measured with plain `gcc -O2` and no sanitizer, a read/pread/recv into a `char buf[4096]` emits `__read_chk`/`__pread_chk`/`__recv_chk`, and only `-U_FORTIFY_SOURCE` brings the plain names back. It surfaced as `test_syscall_efault` failing only under UBSan, because instrumentation made the object size knowable.

Wrapped: `__read_chk`, `__pread_chk`, `__pread64_chk`, `__recv_chk`, `__recvfrom_chk`, `__poll_chk`, `__ppoll_chk`, `__readlink_chk`, `__readlinkat_chk` — the EFAULT-retry family only. `__fgets_chk` / `__fread_chk` / `__getcwd_chk` are deliberately **not** wrapped: they return a pointer or a count, so the retry predicate (`ret == -1 && errno == EFAULT`) cannot apply, and warming the buffer on every call would land on smash's own hot paths (the compressor reads `/proc/self/maps` with `fgets` in a loop). Note the argument-order traps: `__recv_chk` puts `buflen` **before** `flags`, and `__fread_chk` takes `ptrlen` second.

**The versions differ per architecture, and that silently disabled the whole carve-out on aarch64.** glibc versions these at GLIBC_2.4 / 2.5 / 2.16 on x86-64, and the map exported smash's wrappers at exactly those versions — but the aarch64 port starts at GLIBC_2.17, so every one of them is `__*_chk@@GLIBC_2.17` there (verified against an aarch64 libc: all ten). No export matched, so on aarch64 every fortified read into a heap buffer went straight to libc and the EFAULT decompress-and-retry never ran. Reported from an aarch64 run as `test_syscall_efault` failing with `__getgroups_chk returned -1 errno=14`; that test is simply the first to call a `__*_chk` by name, and the other nine were equally unbound. CMake now emits the bare names in the base version node when the base *is* `GLIBC_2.17`, and nowhere else — on x86-64 they must stay local, for the self-jump reason below.

`test_chk_interposition` guards it on any architecture and takes 0.02 s: taking each wrapper's address emits the same versioned reference a call would, so `dladdr` reports which library actually won. **Verified as a real discriminator** — against a stub exporting `smash_in_arena` but no wrappers (the exact shape of the aarch64 bug), all nine fail and name the cause, instead of surfacing 52 seconds later as an EFAULT with no hint about symbol versions.

**Export only the `_NN` alias, never the bare `__*_chk` name.** If libsmash defines both `__recv_chk@@GLIBC_2.4` (the wrapper) and `__recv_chk@GLIBC_2.4` (the alias), a versioned reference resolves to the exact-version match — the alias — whose body is `jmp __recv_chk@plt`, and whose GOT slot then points back at the alias. That is a jump to itself: 100% CPU, no stack growth, `rip` pinned on a two-instruction stub. With the bare name unlisted it falls under `local: *`, so the alias's internal call binds directly with no PLT, while callers still bind `__recv_chk@GLIBC_2.4` as intended.

That failure is hard to see with the usual tools, so use the right ones: `.rela.plt` looks **correct** (it names the `@@` wrapper) because the dynamic linker overrides it at load time, and gdb backtraces hide the loop because tail calls leave no frames. What identifies it is the **runtime GOT value** (`x/gx` on the slot, which held the forwarder's address) and `perf record`, which put 84% of cycles in `__recv_chk@GLIBC_2.4`.

Resolve the real `__*_chk` functions **eagerly**, in a constructor, not with a call-site `dlsym`: smash reads `/proc/self/pagemap` with `read()`, fortify turns that into `__read_chk`, and `-Bsymbolic` binds it to our own wrapper — so a first-call `dlsym` lands on the compressor's tick path. An earlier lazy version stopped the instrumented build compressing at all.

## Audit: syscalls that break on a compressed buffer (2026-09-16, Linux)

`retryWithDecompress` only helps for the syscalls smash interposes; everything
else hands the application the kernel's EFAULT. Measured with a scratch probe
(not in the tree): a 16 MiB heap block, each syscall called twice — once warm,
once with `mincore` confirming its page had left RSS — in full mode with
`SMASH_COLD_TIMEOUT_SEC=1`. **21 of 31 probes failed.**

Not exposed:
- interposed syscalls (`read`, `writev`, `fstat` controls — all recovered);
- vDSO calls such as `clock_gettime`: they run in userspace, so a compressed page faults normally and the handler decompresses it;
- libc functions that bounce through a stack buffer and copy in userspace: `gethostname`, `sigaction`'s `oldact`, `times`.

The 21 that failed, in the audit's risk order — how often the buffer is a heap
object:

| Risk | Syscalls |
| --- | --- |
| Heap buffers are idiomatic | `epoll_ctl` (event structs in heap arrays), `ioctl` (arbitrary app structs), `futex` (a `pthread_mutex_t`/`pthread_cond_t` embedded in a heap object), `bind`, `setsockopt`, `sendfile` (offset), `process_vm_readv`, `mincore` |
| Usually stack, occasionally heap | `uname`, `sysinfo`, `statfs`, `getrlimit`, `getgroups`, `getcpu`, `prctl(PR_GET_NAME)`, `sigprocmask` (oldset), `nanosleep`, `utimensat`, `pipe2`, `socketpair` |

A syscall only breaks when the application passes a *smash-managed heap*
buffer; stack and static buffers are never compressed, which is why this class
rarely surfaces. Affinity surfaced because thread pools keep their masks on the
heap.

### Round 6 (2026-09-20): all but `futex` are now wrapped

Every entry in both tiers now has a wrapper, plus `sendfile64`,
`process_vm_writev`, `statfs64`, `setrlimit`, `pthread_sigmask`,
`clock_nanosleep` and `pipe`, which are the same shape. They follow the
existing pattern exactly — `retryWith1Buf` / `retryWith2Bufs` / a hand-written
walk — and all honour `SMASH_NO_SYSCALL_WRAP=1`.

**`futex` is the one that is not wrapped, and cannot be:** glibc has no
`futex` symbol to interpose, so there is nothing for `LD_PRELOAD` to replace.
It is also the least exposed item on the list in practice — every pthread
primitive reads its futex word from userspace before it makes the syscall, and
that read faults through smash's handler and decompresses the page. Only a
program issuing a raw `syscall(SYS_futex, ...)` against a heap word it has not
touched is at risk.

Three things the wrappers had to get right that the existing ones did not:

- **An unsized buffer.** `ioctl`'s argument has no length in its signature.
  `_IOC_DIR(request) != _IOC_NONE` means the size is encoded in the request
  number (`_IOC_SIZE`); the legacy numbers (`FIONREAD`, `TIOCGWINSZ`) encode
  nothing, so the walk falls back to the single page the argument points into,
  through `walkUnsizedForFault`, which stops at the arena's commit watermark.
  **Walking past a buffer's end inside the arena is not safe**: above the
  watermark the arena is a PROT_NONE reservation, and a fault there is not one
  smash can resolve. The same reasoning makes `walkPathForFault` read a path
  string byte by byte and stop at the terminator instead of walking a fixed
  `PATH_MAX`.
- **A return value that is not `-1`/`errno`.** `clock_nanosleep` returns the
  error code directly and leaves `errno` alone, so it carries its own retry
  loop keyed on `ret == EFAULT`. `pthread_sigmask` reports the same way and
  cannot be retried usefully at all, so it warms up front, like
  `pthread_setaffinity_np`.
- **`_FORTIFY_SOURCE`, again.** `getgroups` is the one syscall in this set
  glibc fortifies, and the audit probe caught it: plain `gcc -O2` rewrote
  `getgroups(64, list)` into `__getgroups_chk(64, list, 256)` for a `malloc`'d
  list — `__builtin_object_size` knows how much of the block is left — so the
  plain wrapper never ran and the call still returned EFAULT. `__getgroups_chk`
  is now wrapped too, resolved eagerly in the constructor with the rest, and
  exported only as the `_24` alias.

**Verified as a real discriminator.** The audit probe calls each syscall twice
on two separate compressed pages: once through the raw kernel entry
(`syscall(SYS_x, ...)`, which no wrapper can see) and once through the libc
entry point. Under the fixed library: **22 of 22 raw calls return EFAULT, and
22 of 22 libc calls succeed.** `getgroups` was 21/22 until `__getgroups_chk`
was wrapped, which is how that gap was found. `test_syscall_efault`'s phase 19
is the in-tree version: one compression wait, then one probe per syscall
against its own page, each checked non-resident with `mincore` immediately
before the call. Every buffer and descriptor is prepared *before* the wait —
filling a buffer is a write, and a write is exactly what keeps a page out of
the compressor's reach, so preparing afterwards would make every probe pass
vacuously.

**Cost** (300 k iterations of `ioctl` + `uname`, which are as close to a pure
syscall loop as this gets): 419–438 ns per syscall with no preload, 453 ns
passing through (`SMASH_NO_SYSCALL_WRAP=1`) and 458–462 ns wrapped. So
interposition itself costs ~15 ns per syscall and the retry logic ~5 ns on top
— about 1% of a syscall, paid only by the syscalls listed above.

## Short transfers when a buffer is only partly compressed (open gap, 2026-09-29)

The EFAULT retry (`retryWithDecompress`) only fires when the syscall returns
-1/`EFAULT`. When a buffer's first pages are accessible and a later one is
compressed, the kernel copies up to that page and returns a **short count**
instead — a successful return the wrappers pass straight to the application.
For a regular file (`read`, `pread`, `preadv`, ...) callers commonly treat a
short count as EOF. Found with `SMASH_RESTORE_GRANULE_PAGES=16` forced on the
suite: `test_syscall_efault`'s `preadv` returned 16384 of 131072 bytes — group
restore compresses a freshly written buffer unevenly (neighbours' cold counts
are frozen for a tick), but a buffer straddling an ACTIVE page and a
COMPRESSED one hits the same path with default settings. Not fixed: a fix
would pre-fault (walk) a buffer's compressed pages before the call, or retry
the remainder after a short count when the page after it is compressed.

