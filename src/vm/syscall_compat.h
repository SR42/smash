// smash/src/vm/syscall_compat.h - Page warming for kernel syscall compatibility
//
// Kernel syscalls (kevent, read, recv, etc.) access userspace buffers directly
// without raising SIGSEGV: when copy_to_user / copy_from_user (Linux) or
// copyin / copyout (macOS) hits a protected page during a syscall, the kernel
// uses the page-fault fixup path to convert the fault into -EFAULT and discard
// the address. So our SEGV/SIGBUS handler never fires for syscall-side faults
// — we have to detect EFAULT in userspace and trigger a touch ourselves
// (which DOES go through the fault handler) so the page can be decompressed.
//
// retryWithDecompress is the canonical pattern: call the syscall, on EFAULT
// walk the buffer pages (one byte per page; SEGV → handler → decompress → RW)
// then retry. Bounded to 8 attempts with µs-scale backoff so unmapped-pointer
// bugs surface as EFAULT instead of livelocking. retryMachOnInvalidData is
// the same shape but keyed on MACH_RCV_INVALID_DATA / MACH_SEND_INVALID_DATA
// since mach_msg uses Mach error codes rather than errno.
//
// warmPages is a proactive read-then-write-back over each page in a buffer.
// Used by buffered-I/O wrappers (fread/fgets/fwrite, …) where libc's internal
// __read happens intra-dylib and never enters our wrapper, so EFAULT can't be
// observed/retried; proactive warming is the only defense.
#pragma once

#include "smash/config.h"
#include "vm_region.h"
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/uio.h>

#ifdef __APPLE__
#include <mach/message.h>
#endif

namespace smash::vm {

// Fill buf with "YYYY-MM-DD HH:MM:SS" from local time. Returns the
// number of chars written (excluding the null), or 0 on failure.
// Used by every "[smash …] …" line so log scrapers can correlate
// across processes. localtime_r/strftime are not strictly POSIX
// async-signal-safe but glibc/Apple implementations work in practice
// inside our signal handlers.
inline int formatTimestamp(char* buf, size_t cap) {
    if (cap < 20) return 0;
    time_t now = time(nullptr);
    struct tm tm_buf;
    if (!localtime_r(&now, &tm_buf)) return 0;
    return static_cast<int>(strftime(buf, cap, "%Y-%m-%d %H:%M:%S", &tm_buf));
}

// One write access to the byte at `p` that leaves its value alone: an atomic
// read-modify-write of +0. A PROT_NONE page faults (the handler decompresses)
// and a PROT_READ monitoring page faults back to PROT_RW, exactly as a plain
// store would -- but a plain `c = *p; *p = c` lost another thread's store to
// that byte in between (the first byte of a page of a shared buffer, in a
// wrapper that was only warming it). The RMW is indivisible, so a concurrent
// store lands before or after it, never under it.
inline void touchByteWritable(uintptr_t p) {
    __atomic_fetch_add(reinterpret_cast<unsigned char*>(p), 0, __ATOMIC_RELAXED);
}

// Proactive write touch of one byte per page in [buf, buf+len).
// Used by wrappers that have no retry surface (intra-libSystem buffered I/O,
// getcwd hooks). PROT_NONE pages fault → handler decompresses; the write
// transitions PROT_READ monitoring pages back to PROT_RW.
inline void warmPages(const void* buf, size_t len, VmRegion* vm) {
    if (!buf || !len || !vm) return;
    auto base = reinterpret_cast<uintptr_t>(buf);
    auto end = base + len;
    for (auto p = base & ~(static_cast<uintptr_t>(kPageSize) - 1); p < end; p += kPageSize) {
        if (!vm->contains(p)) continue;
        touchByteWritable(p);
    }
}

// Same touch pattern as warmPages but used from the EFAULT-retry callback —
// kept as a separate name so call sites are self-documenting.
inline void walkPagesForFault(const void* buf, size_t len, VmRegion* vm) {
    warmPages(buf, len, vm);
}

// Warm a buffer whose length the caller does not know (ioctl's legacy request
// numbers). Walks at most `max_pages` pages and stops at the first page that
// is not a committed arena page: above the commit watermark the arena is a
// PROT_NONE reservation, and a fault there is not one smash can resolve.
inline void walkUnsizedForFault(const void* buf, size_t max_pages, VmRegion* vm) {
    if (!buf || !vm) return;
    auto p = reinterpret_cast<uintptr_t>(buf) & ~(static_cast<uintptr_t>(kPageSize) - 1);
    for (size_t i = 0; i < max_pages; ++i, p += kPageSize) {
        if (!vm->contains(p) || vm->pageIndex(p) >= vm->committedPages()) return;
        touchByteWritable(p);
    }
}

inline void walkIovecForFault(const struct iovec* iov, int iovcnt, VmRegion* vm) {
    for (int i = 0; i < iovcnt; ++i) {
        if (iov[i].iov_base && iov[i].iov_len)
            walkPagesForFault(iov[i].iov_base, iov[i].iov_len, vm);
    }
}

// Up to 8 retries with µs-scale exponential backoff (1, 2, 4, ..., 128µs).
// The compressor tick runs at ~10ms; 8 attempts at 128µs is well inside
// one tick — outlasts any compressor race that could re-compress between
// walk and retry. Bounded so a genuine bug (caller passed an unmapped
// pointer) still surfaces as EFAULT instead of livelocking.
template <typename Syscall, typename Walk>
inline auto retryWithDecompress(Syscall syscall, Walk walk)
    -> decltype(syscall()) {
    auto ret = syscall();
    long backoff_ns = 1000;  // 1µs
    for (int attempt = 0;
         ret == -1 && errno == EFAULT && attempt < 8;
         ++attempt) {
        walk();
        if (attempt > 0) {
            struct timespec ts = {0, backoff_ns};
            nanosleep(&ts, nullptr);
            backoff_ns *= 2;
        }
        ret = syscall();
    }
    return ret;
}

#ifdef __APPLE__
// Mach equivalent: mach_msg signals buffer-touch failure via
// MACH_RCV_INVALID_DATA / MACH_SEND_INVALID_DATA rather than EFAULT.
template <typename Syscall, typename Walk>
inline mach_msg_return_t retryMachOnInvalidData(Syscall syscall, Walk walk) {
    auto ret = syscall();
    long backoff_ns = 1000;
    for (int attempt = 0;
         (ret == MACH_RCV_INVALID_DATA || ret == MACH_SEND_INVALID_DATA) && attempt < 8;
         ++attempt) {
        walk();
        if (attempt > 0) {
            struct timespec ts = {0, backoff_ns};
            nanosleep(&ts, nullptr);
            backoff_ns *= 2;
        }
        ret = syscall();
    }
    return ret;
}
#endif

} // namespace smash::vm

// Forward declaration of the global VmRegion pointer (defined in
// smash_heap.cpp). Declared in the smash:: namespace to match the
// definition. The wrapper templates below use it without pulling in
// smash_heap.h (which would create circular include dependencies).
namespace smash { extern VmRegion* g_smash_vm_region; }

namespace smash::vm {

// ── High-level wrapper templates ──────────────────────────────────────────
// Most syscall wrappers touch one or two userspace buffers and want the
// "if EFAULT, walk + retry" loop. These templates collapse the boilerplate
// to a single line per wrapper.

// Single-buffer wrapper (read/write/recv/send/poll/epoll_wait/fstat/...).
template <typename Syscall>
inline auto retryWith1Buf(Syscall syscall, const void* buf, size_t len)
    -> decltype(syscall()) {
    auto* vm = ::smash::g_smash_vm_region;
    return retryWithDecompress(
        syscall,
        [&] { if (vm && buf && len) walkPagesForFault(buf, len, vm); });
}

// Two-buffer wrapper (kevent/kevent64 changelist + eventlist).
template <typename Syscall>
inline auto retryWith2Bufs(Syscall syscall,
                           const void* a, size_t a_len,
                           const void* b, size_t b_len)
    -> decltype(syscall()) {
    auto* vm = ::smash::g_smash_vm_region;
    return retryWithDecompress(
        syscall,
        [&] {
            if (vm) {
                if (a && a_len) walkPagesForFault(a, a_len, vm);
                if (b && b_len) walkPagesForFault(b, b_len, vm);
            }
        });
}

// Iovec wrapper (readv/writev). Walks every iovec entry on retry.
template <typename Syscall>
inline auto retryWithIovec(Syscall syscall,
                           const struct iovec* iov, int iovcnt)
    -> decltype(syscall()) {
    auto* vm = ::smash::g_smash_vm_region;
    return retryWithDecompress(
        syscall,
        [&] { if (vm && iov && iovcnt > 0) walkIovecForFault(iov, iovcnt, vm); });
}

// ── Short transfers on files ─────────────────────────────────────────────
// When a buffer's first page is accessible and a later one is not (a
// PROT_READ watched page for a read into it, a PROT_NONE compressed or
// escalated page for a write out of it), the kernel copies up to that page
// and returns a SHORT COUNT, not -1/EFAULT, so retryWithDecompress never
// fires. For a regular file or block device a short read looks like end of
// file: under SMASH_SOFTDIRTY=0 a 2 MiB read into a heap buffer whose first
// page had just been written returned 4096 bytes (short_io_test). For those
// descriptors the wrappers below carry on with the rest of the buffer
// (continuing on a file never blocks; at a real end of file the next call
// returns 0). Sockets and pipes keep the kernel's short count: short
// transfers are normal there, callers loop, and nothing is lost. Only a
// short count whose next byte is on a smash page costs the fstat.
inline bool fdIsFile(int fd) {
    struct stat st;
    return ::fstat(fd, &st) == 0 && (S_ISREG(st.st_mode) || S_ISBLK(st.st_mode));
}

// Upper bound on continuations: each makes progress.
inline constexpr int kMaxShortContinuations = 1 << 16;

// One buffer. call(done) issues the syscall for [done, total) of it.
template <typename Call>
inline ssize_t fileIoWith1Buf(int fd, const void* buf, size_t total, Call call) {
    auto* vm = ::smash::g_smash_vm_region;
    const auto base = reinterpret_cast<uintptr_t>(buf);
    auto attempt = [&](size_t done) {
        return retryWithDecompress(
            [&] { return call(done); },
            [&] {
                if (vm && buf && total > done)
                    walkPagesForFault(reinterpret_cast<const void*>(base + done), total - done, vm);
            });
    };
    const ssize_t r = attempt(0);
    if (r <= 0 || static_cast<size_t>(r) >= total || !vm || !buf ||
        !vm->contains(base + static_cast<size_t>(r)))
        return r;
    const int saved_errno = errno;
    if (!fdIsFile(fd)) { errno = saved_errno; return r; }
    size_t done = static_cast<size_t>(r);
    for (int i = 0; i < kMaxShortContinuations && done < total; ++i) {
        const ssize_t n = attempt(done);
        if (n <= 0) break;
        done += static_cast<size_t>(n);
    }
    errno = saved_errno;
    return static_cast<ssize_t>(done);
}

// An iovec array. call(iov, iovcnt, done) issues the syscall for the part
// of the vector from byte `done` on; past the first call it gets a copy
// trimmed to the remainder (at most kIovRemainder entries at a time).
template <typename Call>
inline ssize_t fileIoWithIovec(int fd, const struct iovec* iov, int iovcnt, Call call) {
    auto* vm = ::smash::g_smash_vm_region;
    constexpr int kIovRemainder = 64;
    struct iovec rem[kIovRemainder];
    // The remainder of the vector from byte `done`; nullptr past its end.
    auto remainder = [&](size_t done, int* cnt) -> const struct iovec* {
        int k = 0;
        while (k < iovcnt && done >= iov[k].iov_len) done -= iov[k++].iov_len;
        if (k >= iovcnt) return nullptr;
        int n = 0;
        for (; k < iovcnt && n < kIovRemainder; ++k, ++n) {
            rem[n] = iov[k];
            if (n == 0) {
                rem[0].iov_base = static_cast<char*>(iov[k].iov_base) + done;
                rem[0].iov_len -= done;
            }
        }
        *cnt = n;
        return rem;
    };
    auto attempt = [&](const struct iovec* v, int cnt, size_t done) {
        return retryWithDecompress(
            [&] { return call(v, cnt, done); },
            [&] { if (vm && v && cnt > 0) walkIovecForFault(v, cnt, vm); });
    };
    const ssize_t r = attempt(iov, iovcnt, 0);
    if (r <= 0 || !vm || !iov || iovcnt <= 0) return r;
    int cnt = 0;
    const struct iovec* next = remainder(static_cast<size_t>(r), &cnt);
    if (!next || !vm->contains(reinterpret_cast<uintptr_t>(next[0].iov_base))) return r;
    const int saved_errno = errno;
    if (!fdIsFile(fd)) { errno = saved_errno; return r; }
    size_t done = static_cast<size_t>(r);
    for (int i = 0; i < kMaxShortContinuations; ++i) {
        next = remainder(done, &cnt);
        if (!next) break;
        const ssize_t n = attempt(next, cnt, done);
        if (n <= 0) break;
        done += static_cast<size_t>(n);
    }
    errno = saved_errno;
    return static_cast<ssize_t>(done);
}

// ── Datagram receives ─────────────────────────────────────────────────────
// On a message socket (UDP, AF_UNIX SOCK_DGRAM / SOCK_SEQPACKET, raw) the
// kernel takes the datagram off the queue before copying it out, and frees
// it if the copy faults: the EFAULT retry then received the NEXT datagram
// and the first was lost, silently (dgram_recv_test: every libc receive
// into a watched page under SMASH_SOFTDIRTY=0). Touching the buffer first is
// not enough: a blocking receive can wait longer than a tick, and Phase 3
// re-arms the pages meanwhile. So when a data buffer is on a smash page and
// the socket carries messages, the receive is first issued with MSG_PEEK:
// it waits exactly as the real call would (blocking, timeouts, signals) and
// leaves the datagram queued, so the usual EFAULT retry can make the buffer
// writable; the real receive follows at once, into a buffer the kernel just
// wrote. The peek asks for no address and no control data (a peeked
// SCM_RIGHTS message must not install descriptors); those small buffers are
// touched just before the real call instead. Not covered: SO_PEEK_OFF
// sockets (a peek moves their peek offset), and recvmmsg, which gets its
// buffers touched first and can still lose a datagram when its wait outlasts
// a compressor tick.
inline bool fdIsMessageSocket(int fd) {
    const int saved_errno = errno;
    int type = 0;
    socklen_t len = sizeof(type);
    const bool ok = ::getsockopt(fd, SOL_SOCKET, SO_TYPE, &type, &len) == 0;
    errno = saved_errno;
    return ok && (type == SOCK_DGRAM || type == SOCK_SEQPACKET || type == SOCK_RAW ||
                  type == SOCK_RDM);
}

// True when any byte of [p, p + len) is in smash's region. Always inlined:
// it is on every wrapped read, and once the out-parameter bounces gave it
// enough callers for GCC to emit it out of line, passing __read_chk's
// write_only buffer to it read as an access (-Wmaybe-uninitialized).
[[gnu::always_inline]] inline bool inSmashRegion(const void* p, size_t len) {
    auto* vm = ::smash::g_smash_vm_region;
    if (!vm || !p || !len) return false;
    const auto a = reinterpret_cast<uintptr_t>(p);
    return vm->contains(a) || vm->contains(a + len - 1);
}
inline bool iovecInSmashRegion(const struct iovec* iov, size_t n) {
    for (size_t i = 0; iov && i < n; ++i)
        if (inSmashRegion(iov[i].iov_base, iov[i].iov_len)) return true;
    return false;
}

// Makes [p, p + len) writable, touching only bytes inside it (one per
// page): unlike warmPages, never a byte of a neighbouring object.
inline void touchRangeWritable(void* p, size_t len) {
    auto* vm = ::smash::g_smash_vm_region;
    if (!vm || !p || !len) return;
    const auto a = reinterpret_cast<uintptr_t>(p), end = a + len;
    for (uintptr_t q = a; q < end; q = (q & ~(uintptr_t{kPageSize} - 1)) + kPageSize) {
        if (!vm->contains(q)) continue;
        touchByteWritable(q);
    }
}

// Whether a receive of `flags` on fd with its data buffer in smash's region
// (in_region) takes the peek path. A caller's own MSG_PEEK (nothing is
// consumed) and MSG_ERRQUEUE keep the plain retry.
inline bool wantsPeekedRecv(int fd, int flags, bool in_region) {
    return in_region && !(flags & (MSG_PEEK | MSG_ERRQUEUE)) && fdIsMessageSocket(fd);
}

// peek(): the receive with MSG_PEEK and only the data buffers; walk():
// make them writable after an EFAULT; prep(): make the address / control
// buffers writable; real(): the receive as asked, not retried (an EFAULT
// there means the kernel dropped the datagram; retrying would hand the
// caller the next one as if it were this one).
template <typename Peek, typename Walk, typename Prep, typename Real>
inline auto peekedRecv(Peek peek, Walk walk, Prep prep, Real real) -> decltype(real()) {
    const auto p = retryWithDecompress(peek, walk);
    if (p < 0) return p;
    prep();
    return real();
}

// ── Out-parameters the kernel writes after the work is done ───────────────
// Some syscalls act first and write their results last: wait4/waitid reap
// the child, accept dequeues the connection, a receive consumes the data,
// sendfile transfers it, sendmmsg sends the datagram, rt_sigprocmask and
// prlimit64 apply the new mask or limit, an interrupted nanosleep has slept.
// When that last write faults, the kernel returns EFAULT with the work done,
// and the EFAULT retry did it again: the child's status lost (the retry reaps
// another, or gets ECHILD), the next connection, the next chunk of a stream,
// the file sent twice, the new mask or limit reported as the old one, the
// whole sleep again. Measured on Linux 6.8 with each out-parameter on a page
// the kernel can read but not write (what a watched smash page is to it):
// all eleven of those calls. A control buffer the kernel cannot write is not
// even an EFAULT: the message arrives with MSG_CTRUNC and a passed
// descriptor is closed. Touching the out-parameter first is not enough: a
// blocking call can outlast the cold timeout.
//
// So when an out-parameter is on a smash page, the kernel gets a stack copy,
// which it can always write, and the result is copied out in user space
// afterwards, where a compressed or watched page faults through the handler
// like any other store. Only what the kernel would have written is written,
// and only when it would have: the copy is seeded from the caller's object,
// so bytes the kernel leaves alone keep their value. An out-parameter that is
// not on a smash page goes to the kernel untouched, so an EFAULT there is the
// caller's own and is not retried.

// call(p) with *p replaced by a stack copy when *p is on a smash page;
// copied back when written(result).
template <typename T, typename Call, typename Written>
inline auto withOut(T* p, Call call, Written written) -> decltype(call(p)) {
    if (!p || !inSmashRegion(p, sizeof(T))) return call(p);
    T k;
    __builtin_memcpy(&k, p, sizeof(T));
    const auto r = call(&k);
    if (written(r)) __builtin_memcpy(p, &k, sizeof(T));
    return r;
}

// call(addr, addrlen) for an accept or a receive, with the address bounced
// when it or its length is on a smash page. The kernel's move_addr_to_user,
// in user space: after a non-negative result, at most the caller's length of
// the address is copied and the address's full length is reported. A
// negative length goes to the kernel as it is, for its EINVAL.
template <typename Call>
inline auto withAddrOut(struct sockaddr* addr, socklen_t* addrlen, Call call)
    -> decltype(call(addr, addrlen)) {
    if (!addr || !addrlen) return call(addr, addrlen);
    if (!inSmashRegion(addrlen, sizeof(*addrlen)) && !inSmashRegion(addr, 1))
        return call(addr, addrlen);
    const socklen_t want = *addrlen;
    if (static_cast<int>(want) < 0) return call(addr, addrlen);
    struct sockaddr_storage ss;
    socklen_t len = sizeof(ss);
    const auto r = call(reinterpret_cast<struct sockaddr*>(&ss), &len);
    if (r >= 0) {
        __builtin_memcpy(addr, &ss, want < len ? want : len);
        *addrlen = len;
    }
    return r;
}

// recvmsg(msg) with every field the kernel writes after consuming the data
// bounced when the msghdr, its address or its control buffer is on a smash
// page: msg_name and msg_namelen (only when msg_name is set, as the kernel
// does), msg_flags, msg_controllen, and the control data itself up to
// kCtlBounce bytes. A larger control buffer is made writable instead, which
// leaves a call that blocks past the cold timeout exposed. A negative
// msg_namelen goes to the kernel as it is: it fails before consuming.
inline constexpr size_t kCtlBounce = 1024;

template <typename Recv>
inline ssize_t recvmsgOut(struct msghdr* msg, Recv recv) {
    if (!msg) return recv(msg);
    if (!inSmashRegion(msg, sizeof(*msg)) &&
        !inSmashRegion(msg->msg_name, msg->msg_namelen) &&
        !inSmashRegion(msg->msg_control, msg->msg_controllen))
        return recv(msg);
    struct msghdr k = *msg;
    if (static_cast<int>(k.msg_namelen) < 0) return recv(msg);
    void* const name = k.msg_name;
    const socklen_t namelen = k.msg_namelen;
    void* const control = k.msg_control;
    struct sockaddr_storage ss;
    if (name) {
        k.msg_name = &ss;
        if (k.msg_namelen > sizeof(ss)) k.msg_namelen = sizeof(ss);
    }
    // Seeded from the caller's buffer: the kernel does not write a cmsg's
    // alignment padding, which msg_controllen counts.
    alignas(struct cmsghdr) unsigned char ctl[kCtlBounce];
    const bool ctl_bounced = control && k.msg_controllen && k.msg_controllen <= sizeof(ctl);
    if (ctl_bounced) {
        __builtin_memcpy(ctl, control, k.msg_controllen);
        k.msg_control = ctl;
    } else if (control) {
        touchRangeWritable(control, k.msg_controllen);
    }
    const ssize_t r = recv(&k);
    if (r < 0) return r;
    if (name) {
        __builtin_memcpy(name, &ss, namelen < k.msg_namelen ? namelen : k.msg_namelen);
        msg->msg_namelen = k.msg_namelen;
    }
    if (ctl_bounced) __builtin_memcpy(control, ctl, k.msg_controllen);
    msg->msg_controllen = k.msg_controllen;
    msg->msg_flags = k.msg_flags;
    return r;
}

} // namespace smash::vm
