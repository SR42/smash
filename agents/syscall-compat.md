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

## Audit: syscalls that still break on a compressed buffer (2026-09-16, Linux)

`retryWithDecompress` only helps for the syscalls smash interposes; everything else hands the application the kernel's EFAULT. Measured with a scratch probe (not in the tree): a 16 MiB heap block, each syscall called twice — once warm, once with `mincore` confirming its page had left RSS — in full mode with `SMASH_COLD_TIMEOUT_SEC=1`. **21 of 31 probes still fail.**

Not exposed:
- interposed syscalls (`read`, `writev`, `fstat` controls — all recovered);
- vDSO calls such as `clock_gettime`: they run in userspace, so a compressed page faults normally and the handler decompresses it;
- libc functions that bounce through a stack buffer and copy in userspace: `gethostname`, `sigaction`'s `oldact`, `times`.

Still exposed, ordered by how often the buffer is a heap object:

| Risk | Syscalls |
| --- | --- |
| Heap buffers are idiomatic | `epoll_ctl` (event structs in heap arrays), `ioctl` (arbitrary app structs), `futex` (a `pthread_mutex_t`/`pthread_cond_t` embedded in a heap object), `bind`, `setsockopt`, `sendfile` (offset), `process_vm_readv`, `mincore` |
| Usually stack, occasionally heap | `uname`, `sysinfo`, `statfs`, `getrlimit`, `getgroups`, `getcpu`, `prctl(PR_GET_NAME)`, `sigprocmask` (oldset), `nanosleep`, `utimensat`, `pipe2`, `socketpair` |

A syscall only breaks when the application passes a *smash-managed heap* buffer; stack and static buffers are never compressed, which is why this class rarely surfaces. Affinity surfaced because thread pools keep their masks on the heap.
