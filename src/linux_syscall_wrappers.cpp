// smash/src/linux_syscall_wrappers.cpp - Linux LD_PRELOAD syscall interposition
//
// On Linux, kernel syscalls access userspace buffers directly without
// triggering SIGSEGV. When Smash marks pages PROT_NONE (compressed) or
// PROT_READ (monitoring), kernel syscalls on those buffers return EFAULT.
//
// We interpose on syscalls that read/write userspace buffers and warm+pin
// each Smash-managed page before calling the real syscall.
//
// Real function pointers are resolved lazily via dlsym(RTLD_NEXT) on
// first call. This avoids the early-init crash where glibc calls
// read/write/poll before constructors run. A recursion guard falls
// back to raw syscall() if dlsym itself triggers our wrappers — safe
// because Smash hasn't started yet during early init.

#ifdef __linux__

// _GNU_SOURCE is needed for `struct statx` in <sys/stat.h>.
#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif

#include "smash_heap.h"
#include "vm/syscall_compat.h"

#include <atomic>
#include <dlfcn.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <sys/syscall.h>
#include <sys/epoll.h>
#include <sys/select.h>
#include <sys/random.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/vfs.h>
#include <sys/statvfs.h>
#include <sys/wait.h>
#include <sys/resource.h>
#include <sys/xattr.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <limits.h>
#include <malloc.h>

// Recursion guard: if dlsym triggers one of our wrappers, skip the
// dlsym attempt and let the wrapper fall through to raw syscall().
// This is safe because during early init no pages are compressed.
static std::atomic<bool> g_resolving_syscalls{false};

// Global disable: SMASH_NO_SYSCALL_WRAP=1 makes all wrappers pass-through
static const bool g_syscall_wrap_disabled = [] {
    const char* v = std::getenv("SMASH_NO_SYSCALL_WRAP");
    return v && v[0] == '1';
}();

// Lazy-resolve a real function pointer via dlsym(RTLD_NEXT).
// If we're in a dlsym recursion, leaves the pointer as nullptr
// so the caller falls through to its raw syscall() path.
#define SMASH_LAZY_RESOLVE(fn_type, name) \
    static fn_type real_##name = nullptr; \
    if (__builtin_expect(!real_##name, 0)) { \
        if (!g_resolving_syscalls.load(std::memory_order_relaxed)) { \
            g_resolving_syscalls.store(true, std::memory_order_relaxed); \
            real_##name = reinterpret_cast<fn_type>(dlsym(RTLD_NEXT, #name)); \
            g_resolving_syscalls.store(false, std::memory_order_relaxed); \
        } \
    }

// Passthrough macro: if syscall wrapping disabled or real not resolved, call through
#define SMASH_PASSTHROUGH_IF_DISABLED(name, ...) \
    if (g_syscall_wrap_disabled && real_##name) return real_##name(__VA_ARGS__)

// Check if ANY part of a buffer range is in Smash-managed memory.
// Only return true if the buffer actually needs warming.
static inline bool bufferInHeap(const void* buf, size_t len, smash::VmRegion* vm) {
    if (!vm || !buf || !len) return false;
    auto base = reinterpret_cast<uintptr_t>(buf);
    // Check first and last page of buffer
    return vm->contains(base) || vm->contains(base + len - 1);
}

// Helper: check if any iovec buffer is in Smash-managed memory
static inline bool iovecInHeap(const struct iovec* iov, int iovcnt, smash::VmRegion* vm) {
    if (!vm || !iov || iovcnt <= 0) return false;
    for (int i = 0; i < iovcnt; ++i) {
        if (bufferInHeap(iov[i].iov_base, iov[i].iov_len, vm))
            return true;
    }
    return false;
}

// Use explicit visibility attribute - pragma doesn't work with -fvisibility=hidden
#define SMASH_VISIBLE __attribute__((visibility("default")))

extern "C" {

SMASH_VISIBLE ssize_t read(int fd, void* buf, size_t count) {
    using fn_t = ssize_t(*)(int, void*, size_t);
    SMASH_LAZY_RESOLVE(fn_t, read);
    if (!real_read) return syscall(SYS_read, fd, buf, count);
    SMASH_PASSTHROUGH_IF_DISABLED(read, fd, buf, count);
    return smash::vm::retryWith1Buf(
        [&] { return real_read(fd, buf, count); },
        buf, count);
}

// ── _FORTIFY_SOURCE variants ────────────────────────────────────────────────
// glibc's fortify headers rewrite a call whose destination object size is known
// at compile time into the `__*_chk` alias: `read(fd, buf, n)` becomes
// `__read_chk(fd, buf, n, sizeof buf)`. The plain symbol is then never
// referenced, so interposing `read` alone misses the call and the EFAULT
// decompress-and-retry never runs — the kernel's EFAULT goes straight back to
// the application, which sees a spurious "Bad address".
//
// Not a corner case: _FORTIFY_SOURCE is on by default on Ubuntu, Fedora and
// RHEL, and any read into a fixed-size buffer qualifies. Measured with plain
// `gcc -O2` and no sanitizer, a read/pread/recv into a `char buf[4096]` emits
// __read_chk/__pread_chk/__recv_chk; only -U_FORTIFY_SOURCE brings the plain
// names back. Same lesson as the versioned aliases further down: a wrapper only
// wins if it is exported under the exact name the caller binds.
//
// TWO DELIBERATE LIMITS, both learned the hard way:
//
//  1. Only the EFAULT-retry family is wrapped. retryWith1Buf costs nothing
//     unless the syscall actually returns EFAULT. The proactive-warm functions
//     (fgets/fread/getcwd) are NOT wrapped: warming walks and faults the whole
//     buffer on every call, and smash's own code calls fgets in a loop over
//     /proc/self/maps (readVmaCount), so wrapping them put that cost on the
//     compressor's own path.
//
//  2. Resolved EAGERLY below, never with dlsym at the call site. smash reads
//     /proc/self/pagemap with read(), which fortify turns into __read_chk, and
//     -Bsymbolic binds that to this wrapper — so a first-call dlsym lands on
//     the compressor's tick path. An earlier version that resolved lazily
//     stopped the instrumented build compressing at all (compressed=0), and
//     SMASH_NO_SYSCALL_WRAP=1 did not help because the dlsym ran before the
//     passthrough check.
static ssize_t (*g_real_read_chk)(int, void*, size_t, size_t) = nullptr;
static ssize_t (*g_real_pread_chk)(int, void*, size_t, off_t, size_t) = nullptr;
static ssize_t (*g_real_pread64_chk)(int, void*, size_t, off64_t, size_t) = nullptr;
static ssize_t (*g_real_recv_chk)(int, void*, size_t, size_t, int) = nullptr;
static ssize_t (*g_real_recvfrom_chk)(int, void*, size_t, size_t, int,
                                      struct sockaddr*, socklen_t*) = nullptr;
static int (*g_real_poll_chk)(struct pollfd*, nfds_t, int, size_t) = nullptr;
static int (*g_real_ppoll_chk)(struct pollfd*, nfds_t, const struct timespec*,
                               const sigset_t*, size_t) = nullptr;
static ssize_t (*g_real_readlink_chk)(const char*, char*, size_t, size_t) = nullptr;
static ssize_t (*g_real_readlinkat_chk)(int, const char*, char*, size_t, size_t) = nullptr;

// Resolve one fortified alias, refusing any pointer that lands back inside
// libsmash. RTLD_NEXT is NOT trustworthy here: resolved from a constructor it
// returned our OWN versioned alias (__recv_chk@GLIBC_2.4, i.e. __recv_chk_24),
// so the wrapper tail-called itself. At -O3 that is a jump, not a call, so it
// span forever with no stack growth — two gdb backtraces caught the main thread
// in state R inside __recv_chk_24 and __readlink_chk_24. Take the pointers from
// an explicit libc handle instead, and keep the self-reference check as a
// belt: a null pointer just means the wrapper falls back to the plain syscall,
// which is correct, only without the fortified EFAULT retry.
static void* resolveFortified(void* libc, const char* name, const void* self_base) {
    void* p = libc ? dlsym(libc, name) : nullptr;
    if (!p) p = dlsym(RTLD_NEXT, name);
    if (!p) return nullptr;
    Dl_info info;
    if (dladdr(p, &info) && info.dli_fbase == self_base) return nullptr;
    return p;
}

// Priority 102: after alloc8 installs its wrappers (101), well before the
// compressor starts (201). The recursion guard mirrors SMASH_LAZY_RESOLVE's —
// dlsym allocates, and that allocation lands in smash's own malloc.
__attribute__((constructor(102)))
static void smash_resolve_fortified_syscalls() {
    g_resolving_syscalls.store(true, std::memory_order_relaxed);
    Dl_info self{};
    const void* self_base =
        dladdr(reinterpret_cast<void*>(&smash_resolve_fortified_syscalls), &self)
            ? self.dli_fbase : nullptr;
    void* libc = dlopen("libc.so.6", RTLD_LAZY | RTLD_NOLOAD);  // already mapped
    g_real_read_chk = reinterpret_cast<decltype(g_real_read_chk)>(resolveFortified(libc, "__read_chk", self_base));
    g_real_pread_chk = reinterpret_cast<decltype(g_real_pread_chk)>(resolveFortified(libc, "__pread_chk", self_base));
    g_real_pread64_chk = reinterpret_cast<decltype(g_real_pread64_chk)>(resolveFortified(libc, "__pread64_chk", self_base));
    g_real_recv_chk = reinterpret_cast<decltype(g_real_recv_chk)>(resolveFortified(libc, "__recv_chk", self_base));
    g_real_recvfrom_chk = reinterpret_cast<decltype(g_real_recvfrom_chk)>(resolveFortified(libc, "__recvfrom_chk", self_base));
    g_real_poll_chk = reinterpret_cast<decltype(g_real_poll_chk)>(resolveFortified(libc, "__poll_chk", self_base));
    g_real_ppoll_chk = reinterpret_cast<decltype(g_real_ppoll_chk)>(resolveFortified(libc, "__ppoll_chk", self_base));
    g_real_readlink_chk = reinterpret_cast<decltype(g_real_readlink_chk)>(resolveFortified(libc, "__readlink_chk", self_base));
    g_real_readlinkat_chk = reinterpret_cast<decltype(g_real_readlinkat_chk)>(resolveFortified(libc, "__readlinkat_chk", self_base));
    // Intentionally no dlclose: the handle is RTLD_NOLOAD on a library that is
    // never unloaded, and dropping the refcount buys nothing.
    g_resolving_syscalls.store(false, std::memory_order_relaxed);
}

SMASH_VISIBLE ssize_t __read_chk(int fd, void* buf, size_t count, size_t buflen) {
    if (!g_real_read_chk) return read(fd, buf, count);
    if (g_syscall_wrap_disabled) return g_real_read_chk(fd, buf, count, buflen);
    return smash::vm::retryWith1Buf(
        [&] { return g_real_read_chk(fd, buf, count, buflen); },
        buf, count);
}
SMASH_VISIBLE ssize_t __read_chk_24(int fd, void* buf, size_t count, size_t buflen) {
    return __read_chk(fd, buf, count, buflen);
}
__asm__(".symver __read_chk_24,__read_chk@GLIBC_2.4");

SMASH_VISIBLE ssize_t __pread_chk(int fd, void* buf, size_t count, off_t offset,
                                  size_t buflen) {
    if (!g_real_pread_chk) return pread(fd, buf, count, offset);
    if (g_syscall_wrap_disabled) return g_real_pread_chk(fd, buf, count, offset, buflen);
    return smash::vm::retryWith1Buf(
        [&] { return g_real_pread_chk(fd, buf, count, offset, buflen); },
        buf, count);
}
SMASH_VISIBLE ssize_t __pread_chk_24(int fd, void* buf, size_t count, off_t offset,
                                     size_t buflen) {
    return __pread_chk(fd, buf, count, offset, buflen);
}
__asm__(".symver __pread_chk_24,__pread_chk@GLIBC_2.4");

SMASH_VISIBLE ssize_t __pread64_chk(int fd, void* buf, size_t count, off64_t offset,
                                    size_t buflen) {
    if (!g_real_pread64_chk) return pread(fd, buf, count, static_cast<off_t>(offset));
    if (g_syscall_wrap_disabled) return g_real_pread64_chk(fd, buf, count, offset, buflen);
    return smash::vm::retryWith1Buf(
        [&] { return g_real_pread64_chk(fd, buf, count, offset, buflen); },
        buf, count);
}
SMASH_VISIBLE ssize_t __pread64_chk_24(int fd, void* buf, size_t count,
                                       off64_t offset, size_t buflen) {
    return __pread64_chk(fd, buf, count, offset, buflen);
}
__asm__(".symver __pread64_chk_24,__pread64_chk@GLIBC_2.4");

// buflen comes BEFORE flags here — not the base recv() argument order.
SMASH_VISIBLE ssize_t __recv_chk(int s, void* buf, size_t len, size_t buflen, int flags) {
    if (!g_real_recv_chk) return recv(s, buf, len, flags);
    if (g_syscall_wrap_disabled) return g_real_recv_chk(s, buf, len, buflen, flags);
    return smash::vm::retryWith1Buf(
        [&] { return g_real_recv_chk(s, buf, len, buflen, flags); },
        buf, len);
}
SMASH_VISIBLE ssize_t __recv_chk_24(int s, void* buf, size_t len, size_t buflen,
                                    int flags) {
    return __recv_chk(s, buf, len, buflen, flags);
}
__asm__(".symver __recv_chk_24,__recv_chk@GLIBC_2.4");

SMASH_VISIBLE ssize_t __recvfrom_chk(int s, void* buf, size_t len, size_t buflen,
                                     int flags, struct sockaddr* addr,
                                     socklen_t* addr_len) {
    if (!g_real_recvfrom_chk) return recvfrom(s, buf, len, flags, addr, addr_len);
    if (g_syscall_wrap_disabled)
        return g_real_recvfrom_chk(s, buf, len, buflen, flags, addr, addr_len);
    return smash::vm::retryWith1Buf(
        [&] { return g_real_recvfrom_chk(s, buf, len, buflen, flags, addr, addr_len); },
        buf, len);
}
SMASH_VISIBLE ssize_t __recvfrom_chk_24(int s, void* buf, size_t len, size_t buflen,
                                        int flags, struct sockaddr* addr,
                                        socklen_t* addr_len) {
    return __recvfrom_chk(s, buf, len, buflen, flags, addr, addr_len);
}
__asm__(".symver __recvfrom_chk_24,__recvfrom_chk@GLIBC_2.4");

SMASH_VISIBLE int __poll_chk(struct pollfd* fds, nfds_t nfds, int timeout, size_t fdslen) {
    if (!g_real_poll_chk) return poll(fds, nfds, timeout);
    if (g_syscall_wrap_disabled) return g_real_poll_chk(fds, nfds, timeout, fdslen);
    return smash::vm::retryWith1Buf(
        [&] { return g_real_poll_chk(fds, nfds, timeout, fdslen); },
        fds, nfds * sizeof(struct pollfd));
}
SMASH_VISIBLE int __poll_chk_216(struct pollfd* fds, nfds_t nfds, int timeout,
                                 size_t fdslen) {
    return __poll_chk(fds, nfds, timeout, fdslen);
}
__asm__(".symver __poll_chk_216,__poll_chk@GLIBC_2.16");

SMASH_VISIBLE int __ppoll_chk(struct pollfd* fds, nfds_t nfds,
                              const struct timespec* timeout,
                              const sigset_t* ss, size_t fdslen) {
    if (!g_real_ppoll_chk) return ppoll(fds, nfds, timeout, ss);
    if (g_syscall_wrap_disabled) return g_real_ppoll_chk(fds, nfds, timeout, ss, fdslen);
    return smash::vm::retryWith1Buf(
        [&] { return g_real_ppoll_chk(fds, nfds, timeout, ss, fdslen); },
        fds, nfds * sizeof(struct pollfd));
}
SMASH_VISIBLE int __ppoll_chk_216(struct pollfd* fds, nfds_t nfds,
                                  const struct timespec* timeout,
                                  const sigset_t* ss, size_t fdslen) {
    return __ppoll_chk(fds, nfds, timeout, ss, fdslen);
}
__asm__(".symver __ppoll_chk_216,__ppoll_chk@GLIBC_2.16");

SMASH_VISIBLE ssize_t __readlink_chk(const char* path, char* buf, size_t len,
                                     size_t buflen) {
    if (!g_real_readlink_chk) return readlink(path, buf, len);
    if (g_syscall_wrap_disabled) return g_real_readlink_chk(path, buf, len, buflen);
    return smash::vm::retryWith1Buf(
        [&] { return g_real_readlink_chk(path, buf, len, buflen); },
        buf, len);
}
SMASH_VISIBLE ssize_t __readlink_chk_24(const char* path, char* buf, size_t len,
                                        size_t buflen) {
    return __readlink_chk(path, buf, len, buflen);
}
__asm__(".symver __readlink_chk_24,__readlink_chk@GLIBC_2.4");

// readlinkat's fortified alias is versioned GLIBC_2.5, not 2.4.
SMASH_VISIBLE ssize_t __readlinkat_chk(int fd, const char* path, char* buf,
                                       size_t len, size_t buflen) {
    if (!g_real_readlinkat_chk) return readlinkat(fd, path, buf, len);
    if (g_syscall_wrap_disabled) return g_real_readlinkat_chk(fd, path, buf, len, buflen);
    return smash::vm::retryWith1Buf(
        [&] { return g_real_readlinkat_chk(fd, path, buf, len, buflen); },
        buf, len);
}
SMASH_VISIBLE ssize_t __readlinkat_chk_25(int fd, const char* path, char* buf,
                                          size_t len, size_t buflen) {
    return __readlinkat_chk(fd, path, buf, len, buflen);
}
__asm__(".symver __readlinkat_chk_25,__readlinkat_chk@GLIBC_2.5");

SMASH_VISIBLE ssize_t write(int fd, const void* buf, size_t count) {
    using fn_t = ssize_t(*)(int, const void*, size_t);
    SMASH_LAZY_RESOLVE(fn_t, write);
    if (!real_write) return syscall(SYS_write, fd, buf, count);
    SMASH_PASSTHROUGH_IF_DISABLED(write, fd, buf, count);
    return smash::vm::retryWith1Buf(
        [&] { return real_write(fd, buf, count); },
        buf, count);
}

SMASH_VISIBLE ssize_t pread(int fd, void* buf, size_t count, off_t offset) {
    using fn_t = ssize_t(*)(int, void*, size_t, off_t);
    SMASH_LAZY_RESOLVE(fn_t, pread);
    if (!real_pread) return syscall(SYS_pread64, fd, buf, count, offset);
    return smash::vm::retryWith1Buf(
        [&] { return real_pread(fd, buf, count, offset); },
        buf, count);
}

SMASH_VISIBLE ssize_t pwrite(int fd, const void* buf, size_t count, off_t offset) {
    using fn_t = ssize_t(*)(int, const void*, size_t, off_t);
    SMASH_LAZY_RESOLVE(fn_t, pwrite);
    if (!real_pwrite) return syscall(SYS_pwrite64, fd, buf, count, offset);
    return smash::vm::retryWith1Buf(
        [&] { return real_pwrite(fd, buf, count, offset); },
        buf, count);
}

SMASH_VISIBLE ssize_t readv(int fd, const struct iovec* iov, int iovcnt) {
    using fn_t = ssize_t(*)(int, const struct iovec*, int);
    SMASH_LAZY_RESOLVE(fn_t, readv);
    if (!real_readv) return syscall(SYS_readv, fd, iov, iovcnt);
    return smash::vm::retryWithIovec(
        [&] { return real_readv(fd, iov, iovcnt); },
        iov, iovcnt);
}

SMASH_VISIBLE ssize_t writev(int fd, const struct iovec* iov, int iovcnt) {
    using fn_t = ssize_t(*)(int, const struct iovec*, int);
    SMASH_LAZY_RESOLVE(fn_t, writev);
    if (!real_writev) return syscall(SYS_writev, fd, iov, iovcnt);
    return smash::vm::retryWithIovec(
        [&] { return real_writev(fd, iov, iovcnt); },
        iov, iovcnt);
}

SMASH_VISIBLE ssize_t recv(int s, void* buf, size_t len, int flags) {
    using fn_t = ssize_t(*)(int, void*, size_t, int);
    SMASH_LAZY_RESOLVE(fn_t, recv);
    if (!real_recv) return syscall(SYS_recvfrom, s, buf, len, flags, nullptr, nullptr);
    return smash::vm::retryWith1Buf(
        [&] { return real_recv(s, buf, len, flags); },
        buf, len);
}

SMASH_VISIBLE ssize_t send(int s, const void* buf, size_t len, int flags) {
    using fn_t = ssize_t(*)(int, const void*, size_t, int);
    SMASH_LAZY_RESOLVE(fn_t, send);
    if (!real_send) return syscall(SYS_sendto, s, buf, len, flags, nullptr, 0);
    return smash::vm::retryWith1Buf(
        [&] { return real_send(s, buf, len, flags); },
        buf, len);
}

SMASH_VISIBLE ssize_t recvfrom(int s, void* buf, size_t len, int flags,
                 struct sockaddr* from, socklen_t* fromlen) {
    using fn_t = ssize_t(*)(int, void*, size_t, int, struct sockaddr*, socklen_t*);
    SMASH_LAZY_RESOLVE(fn_t, recvfrom);
    if (!real_recvfrom) return syscall(SYS_recvfrom, s, buf, len, flags, from, fromlen);
    return smash::vm::retryWith1Buf(
        [&] { return real_recvfrom(s, buf, len, flags, from, fromlen); },
        buf, len);
}

SMASH_VISIBLE ssize_t sendto(int s, const void* buf, size_t len, int flags,
               const struct sockaddr* to, socklen_t tolen) {
    using fn_t = ssize_t(*)(int, const void*, size_t, int, const struct sockaddr*, socklen_t);
    SMASH_LAZY_RESOLVE(fn_t, sendto);
    if (!real_sendto) return syscall(SYS_sendto, s, buf, len, flags, to, tolen);
    return smash::vm::retryWith1Buf(
        [&] { return real_sendto(s, buf, len, flags, to, tolen); },
        buf, len);
}

SMASH_VISIBLE ssize_t recvmsg(int s, struct msghdr* msg, int flags) {
    using fn_t = ssize_t(*)(int, struct msghdr*, int);
    SMASH_LAZY_RESOLVE(fn_t, recvmsg);
    if (!real_recvmsg) return syscall(SYS_recvmsg, s, msg, flags);
    auto* vm = smash::g_smash_vm_region;
    bool in_heap = msg && iovecInHeap(msg->msg_iov, static_cast<int>(msg->msg_iovlen), vm);
    ssize_t ret = smash::vm::retryWithDecompress(
        [&] { return real_recvmsg(s, msg, flags); },
        [&] { if (in_heap) smash::vm::walkIovecForFault(msg->msg_iov, static_cast<int>(msg->msg_iovlen), vm); });
    return ret;
}

SMASH_VISIBLE ssize_t sendmsg(int s, const struct msghdr* msg, int flags) {
    using fn_t = ssize_t(*)(int, const struct msghdr*, int);
    SMASH_LAZY_RESOLVE(fn_t, sendmsg);
    if (!real_sendmsg) return syscall(SYS_sendmsg, s, msg, flags);
    auto* vm = smash::g_smash_vm_region;
    bool in_heap = msg && iovecInHeap(msg->msg_iov, static_cast<int>(msg->msg_iovlen), vm);
    ssize_t ret = smash::vm::retryWithDecompress(
        [&] { return real_sendmsg(s, msg, flags); },
        [&] { if (in_heap) smash::vm::walkIovecForFault(msg->msg_iov, static_cast<int>(msg->msg_iovlen), vm); });
    return ret;
}

// poll/ppoll - GLib (used by GTK/Firefox) heap-allocates pollfd arrays in its
// main loop. The kernel writes revents back into the array, so if those pages
// are PROT_NONE or PROT_READ, the kernel returns EFAULT.
SMASH_VISIBLE int poll(struct pollfd* fds, nfds_t nfds, int timeout) {
    using fn_t = int(*)(struct pollfd*, nfds_t, int);
    SMASH_LAZY_RESOLVE(fn_t, poll);
    if (!real_poll) {
#ifdef SYS_poll
        return syscall(SYS_poll, fds, nfds, timeout);
#else
        // arm64 / riscv have no SYS_poll; route via ppoll.
        struct timespec ts, *tsp = nullptr;
        if (timeout >= 0) {
            ts.tv_sec  = timeout / 1000;
            ts.tv_nsec = static_cast<long>(timeout % 1000) * 1000000;
            tsp = &ts;
        }
        return syscall(SYS_ppoll, fds, nfds, tsp, nullptr, 0);
#endif
    }
    auto* vm = smash::g_smash_vm_region;
    size_t size = static_cast<size_t>(nfds) * sizeof(struct pollfd);
    bool in_heap = bufferInHeap(fds, size, vm);
    int ret = smash::vm::retryWithDecompress(
        [&] { return real_poll(fds, nfds, timeout); },
        [&] { if (in_heap) smash::vm::walkPagesForFault(fds, size, vm); });
    return ret;
}

SMASH_VISIBLE int ppoll(struct pollfd* fds, nfds_t nfds,
                        const struct timespec* tmo_p, const sigset_t* sigmask) {
    using fn_t = int(*)(struct pollfd*, nfds_t, const struct timespec*, const sigset_t*);
    SMASH_LAZY_RESOLVE(fn_t, ppoll);
    if (!real_ppoll) return syscall(SYS_ppoll, fds, nfds, tmo_p, sigmask);
    auto* vm = smash::g_smash_vm_region;
    size_t size = static_cast<size_t>(nfds) * sizeof(struct pollfd);
    bool in_heap = bufferInHeap(fds, size, vm);
    int ret = smash::vm::retryWithDecompress(
        [&] { return real_ppoll(fds, nfds, tmo_p, sigmask); },
        [&] { if (in_heap) smash::vm::walkPagesForFault(fds, size, vm); });
    return ret;
}

// ── select/pselect6 ─────────────────────────────────────────────────────────
// Firefox/GLib and many event loops use select(). The kernel reads/writes
// fd_set buffers; if those pages are protected we get EFAULT.
// fd_set is typically stack-allocated but may be heap-allocated for large sets.
SMASH_VISIBLE int select(int nfds, fd_set* readfds, fd_set* writefds,
                         fd_set* exceptfds, struct timeval* timeout) {
    using fn_t = int(*)(int, fd_set*, fd_set*, fd_set*, struct timeval*);
    SMASH_LAZY_RESOLVE(fn_t, select);
    if (!real_select) {
#ifdef SYS_select
        return syscall(SYS_select, nfds, readfds, writefds, exceptfds, timeout);
#else
        // arm64 / riscv have no SYS_select; route via pselect6.
        struct timespec ts, *tsp = nullptr;
        if (timeout) {
            ts.tv_sec  = timeout->tv_sec;
            ts.tv_nsec = timeout->tv_usec * 1000;
            tsp = &ts;
        }
        return syscall(SYS_pselect6, nfds, readfds, writefds, exceptfds, tsp, nullptr);
#endif
    }
    auto* vm = smash::g_smash_vm_region;
    // fd_set size depends on nfds but is bounded by sizeof(fd_set)
    size_t set_size = sizeof(fd_set);
    bool pin_r = readfds && bufferInHeap(readfds, set_size, vm);
    bool pin_w = writefds && bufferInHeap(writefds, set_size, vm);
    bool pin_e = exceptfds && bufferInHeap(exceptfds, set_size, vm);
    int ret = smash::vm::retryWithDecompress(
        [&] { return real_select(nfds, readfds, writefds, exceptfds, timeout); },
        [&] {
            if (pin_r) smash::vm::walkPagesForFault(readfds, set_size, vm);
            if (pin_w) smash::vm::walkPagesForFault(writefds, set_size, vm);
            if (pin_e) smash::vm::walkPagesForFault(exceptfds, set_size, vm);
        });
    return ret;
}

SMASH_VISIBLE int pselect(int nfds, fd_set* readfds, fd_set* writefds,
                          fd_set* exceptfds, const struct timespec* timeout,
                          const sigset_t* sigmask) {
    using fn_t = int(*)(int, fd_set*, fd_set*, fd_set*, const struct timespec*, const sigset_t*);
    SMASH_LAZY_RESOLVE(fn_t, pselect);
    if (!real_pselect) return syscall(SYS_pselect6, nfds, readfds, writefds, exceptfds, timeout, sigmask);
    auto* vm = smash::g_smash_vm_region;
    size_t set_size = sizeof(fd_set);
    bool pin_r = readfds && bufferInHeap(readfds, set_size, vm);
    bool pin_w = writefds && bufferInHeap(writefds, set_size, vm);
    bool pin_e = exceptfds && bufferInHeap(exceptfds, set_size, vm);
    int ret = smash::vm::retryWithDecompress(
        [&] { return real_pselect(nfds, readfds, writefds, exceptfds, timeout, sigmask); },
        [&] {
            if (pin_r) smash::vm::walkPagesForFault(readfds, set_size, vm);
            if (pin_w) smash::vm::walkPagesForFault(writefds, set_size, vm);
            if (pin_e) smash::vm::walkPagesForFault(exceptfds, set_size, vm);
        });
    return ret;
}

// ── accept/accept4 ──────────────────────────────────────────────────────────
// The kernel writes the peer address into a heap-allocated sockaddr buffer.
SMASH_VISIBLE int accept(int sockfd, struct sockaddr* addr, socklen_t* addrlen) {
    using fn_t = int(*)(int, struct sockaddr*, socklen_t*);
    SMASH_LAZY_RESOLVE(fn_t, accept);
    if (!real_accept) return syscall(SYS_accept, sockfd, addr, addrlen);
    auto* vm = smash::g_smash_vm_region;
    size_t addr_size = (addr && addrlen) ? static_cast<size_t>(*addrlen) : 0;
    bool in_heap = addr_size && bufferInHeap(addr, addr_size, vm);
    int ret = smash::vm::retryWithDecompress(
        [&] { return real_accept(sockfd, addr, addrlen); },
        [&] { if (in_heap) smash::vm::walkPagesForFault(addr, addr_size, vm); });
    return ret;
}

SMASH_VISIBLE int accept4(int sockfd, struct sockaddr* addr, socklen_t* addrlen, int flags) {
    using fn_t = int(*)(int, struct sockaddr*, socklen_t*, int);
    SMASH_LAZY_RESOLVE(fn_t, accept4);
    if (!real_accept4) return syscall(SYS_accept4, sockfd, addr, addrlen, flags);
    auto* vm = smash::g_smash_vm_region;
    size_t addr_size = (addr && addrlen) ? static_cast<size_t>(*addrlen) : 0;
    bool in_heap = addr_size && bufferInHeap(addr, addr_size, vm);
    int ret = smash::vm::retryWithDecompress(
        [&] { return real_accept4(sockfd, addr, addrlen, flags); },
        [&] { if (in_heap) smash::vm::walkPagesForFault(addr, addr_size, vm); });
    return ret;
}

// ── recvmmsg/sendmmsg ───────────────────────────────────────────────────────
// Batch message I/O. Each mmsghdr contains a msghdr with iovecs.
SMASH_VISIBLE int recvmmsg(int sockfd, struct mmsghdr* msgvec, unsigned int vlen,
                           int flags, struct timespec* timeout) {
    using fn_t = int(*)(int, struct mmsghdr*, unsigned int, int, struct timespec*);
    SMASH_LAZY_RESOLVE(fn_t, recvmmsg);
    if (!real_recvmmsg) return syscall(SYS_recvmmsg, sockfd, msgvec, vlen, flags, timeout);
    auto* vm = smash::g_smash_vm_region;
    int ret = smash::vm::retryWithDecompress(
        [&] { return real_recvmmsg(sockfd, msgvec, vlen, flags, timeout); },
        [&] {
            for (unsigned i = 0; i < vlen; ++i) {
                struct msghdr* msg = &msgvec[i].msg_hdr;
                if (msg->msg_iov && msg->msg_iovlen > 0)
                    smash::vm::walkIovecForFault(msg->msg_iov, static_cast<int>(msg->msg_iovlen), vm);
            }
        });
    return ret;
}

SMASH_VISIBLE int sendmmsg(int sockfd, struct mmsghdr* msgvec, unsigned int vlen,
                           int flags) {
    using fn_t = int(*)(int, struct mmsghdr*, unsigned int, int);
    SMASH_LAZY_RESOLVE(fn_t, sendmmsg);
    if (!real_sendmmsg) return syscall(SYS_sendmmsg, sockfd, msgvec, vlen, flags);
    auto* vm = smash::g_smash_vm_region;
    int ret = smash::vm::retryWithDecompress(
        [&] { return real_sendmmsg(sockfd, msgvec, vlen, flags); },
        [&] {
            for (unsigned i = 0; i < vlen; ++i) {
                struct msghdr* msg = &msgvec[i].msg_hdr;
                if (msg->msg_iov && msg->msg_iovlen > 0)
                    smash::vm::walkIovecForFault(msg->msg_iov, static_cast<int>(msg->msg_iovlen), vm);
            }
        });
    return ret;
}

// ── Socket address query syscalls ───────────────────────────────────────────
// getsockopt writes into a heap-allocated buffer; getsockname/getpeername
// write into a sockaddr buffer. All can EFAULT on protected pages.
SMASH_VISIBLE int getsockopt(int sockfd, int level, int optname,
                             void* optval, socklen_t* optlen) {
    using fn_t = int(*)(int, int, int, void*, socklen_t*);
    SMASH_LAZY_RESOLVE(fn_t, getsockopt);
    if (!real_getsockopt) return syscall(SYS_getsockopt, sockfd, level, optname, optval, optlen);
    auto* vm = smash::g_smash_vm_region;
    size_t val_size = (optval && optlen) ? static_cast<size_t>(*optlen) : 0;
    bool in_heap = val_size && bufferInHeap(optval, val_size, vm);
    int ret = smash::vm::retryWithDecompress(
        [&] { return real_getsockopt(sockfd, level, optname, optval, optlen); },
        [&] { if (in_heap) smash::vm::walkPagesForFault(optval, val_size, vm); });
    return ret;
}

SMASH_VISIBLE int getsockname(int sockfd, struct sockaddr* addr, socklen_t* addrlen) {
    using fn_t = int(*)(int, struct sockaddr*, socklen_t*);
    SMASH_LAZY_RESOLVE(fn_t, getsockname);
    if (!real_getsockname) return syscall(SYS_getsockname, sockfd, addr, addrlen);
    auto* vm = smash::g_smash_vm_region;
    size_t addr_size = (addr && addrlen) ? static_cast<size_t>(*addrlen) : 0;
    bool in_heap = addr_size && bufferInHeap(addr, addr_size, vm);
    int ret = smash::vm::retryWithDecompress(
        [&] { return real_getsockname(sockfd, addr, addrlen); },
        [&] { if (in_heap) smash::vm::walkPagesForFault(addr, addr_size, vm); });
    return ret;
}

SMASH_VISIBLE int getpeername(int sockfd, struct sockaddr* addr, socklen_t* addrlen) {
    using fn_t = int(*)(int, struct sockaddr*, socklen_t*);
    SMASH_LAZY_RESOLVE(fn_t, getpeername);
    if (!real_getpeername) return syscall(SYS_getpeername, sockfd, addr, addrlen);
    auto* vm = smash::g_smash_vm_region;
    size_t addr_size = (addr && addrlen) ? static_cast<size_t>(*addrlen) : 0;
    bool in_heap = addr_size && bufferInHeap(addr, addr_size, vm);
    int ret = smash::vm::retryWithDecompress(
        [&] { return real_getpeername(sockfd, addr, addrlen); },
        [&] { if (in_heap) smash::vm::walkPagesForFault(addr, addr_size, vm); });
    return ret;
}

// ── fstat family / fstatfs ──────────────────────────────────────────────────
// glibc's fstat ABI is a moving target:
//   - glibc < 2.33: <sys/stat.h> declares `fstat` as a static inline that
//     calls `__fxstat(_STAT_VER, fd, buf)`. Linker only sees __fxstat.
//   - glibc ≥ 2.33: `fstat` is a real exported symbol, no inline stub.
//   - With _FILE_OFFSET_BITS=64 (Ubuntu's default for C++ programs):
//       <sys/stat.h> does `#define fstat fstat64`, so callers actually
//       link against `fstat64` (or `__fxstat64` on old glibc).
// To intercept reliably across all combinations, wrap every variant. Each
// is a one-line shell over retryWith1Buf with the appropriate struct size.

SMASH_VISIBLE int fstat(int fd, struct stat* st) {
    using fn_t = int(*)(int, struct stat*);
    SMASH_LAZY_RESOLVE(fn_t, fstat);
    if (!real_fstat) return syscall(SYS_fstat, fd, st);
    return smash::vm::retryWith1Buf(
        [&] { return real_fstat(fd, st); },
        st, sizeof(struct stat));
}

// fstat64 path: with _FILE_OFFSET_BITS=64 + LFS, glibc's preprocessor
// rewrites fstat→fstat64 at the call site. The signature must match
// glibc's declaration in <sys/stat.h>; struct stat64 has the same layout
// as struct stat on x86_64 modern glibc.
SMASH_VISIBLE int fstat64(int fd, struct stat64* st) {
    using fn_t = int(*)(int, struct stat64*);
    SMASH_LAZY_RESOLVE(fn_t, fstat64);
    if (!real_fstat64) return syscall(SYS_fstat, fd, st);
    return smash::vm::retryWith1Buf(
        [&] { return real_fstat64(fd, st); },
        st, sizeof(struct stat64));
}

// __fxstat(version, fd, buf) — glibc < 2.33 internal symbol that the old
// inline `fstat` calls into. Modern glibc still exports a deprecated stub
// for ABI compatibility.
SMASH_VISIBLE int __fxstat(int ver, int fd, struct stat* st) {
    using fn_t = int(*)(int, int, struct stat*);
    SMASH_LAZY_RESOLVE(fn_t, __fxstat);
    if (!real___fxstat) return syscall(SYS_fstat, fd, st);
    return smash::vm::retryWith1Buf(
        [&] { return real___fxstat(ver, fd, st); },
        st, sizeof(struct stat));
}

SMASH_VISIBLE int __fxstat64(int ver, int fd, struct stat64* st) {
    using fn_t = int(*)(int, int, struct stat64*);
    SMASH_LAZY_RESOLVE(fn_t, __fxstat64);
    if (!real___fxstat64) return syscall(SYS_fstat, fd, st);
    return smash::vm::retryWith1Buf(
        [&] { return real___fxstat64(ver, fd, st); },
        st, sizeof(struct stat64));
}

SMASH_VISIBLE int fstatfs(int fd, struct statfs* st) {
    using fn_t = int(*)(int, struct statfs*);
    SMASH_LAZY_RESOLVE(fn_t, fstatfs);
    if (!real_fstatfs) return syscall(SYS_fstatfs, fd, st);
    return smash::vm::retryWith1Buf(
        [&] { return real_fstatfs(fd, st); },
        st, sizeof(struct statfs));
}

// ── stat / lstat / fstatat / statx ──────────────────────────────────────────
// Same hazard as fstat: kernel writes a struct (stat / statx) into a
// userspace buffer that may be heap-allocated and compressed. Cover the
// same ABI surface — modern symbol, _64 LFS variant, and the legacy
// __xstat / __lxstat / __fxstatat pre-2.33 entry points.

// Fall back via SYS_newfstatat (works on every modern arch including
// arm64/riscv where SYS_stat doesn't exist).
SMASH_VISIBLE int stat(const char* path, struct stat* st) {
    using fn_t = int(*)(const char*, struct stat*);
    SMASH_LAZY_RESOLVE(fn_t, stat);
    if (!real_stat) return syscall(SYS_newfstatat, AT_FDCWD, path, st, 0);
    return smash::vm::retryWith1Buf(
        [&] { return real_stat(path, st); },
        st, sizeof(struct stat));
}

SMASH_VISIBLE int lstat(const char* path, struct stat* st) {
    using fn_t = int(*)(const char*, struct stat*);
    SMASH_LAZY_RESOLVE(fn_t, lstat);
    if (!real_lstat) return syscall(SYS_newfstatat, AT_FDCWD, path, st, AT_SYMLINK_NOFOLLOW);
    return smash::vm::retryWith1Buf(
        [&] { return real_lstat(path, st); },
        st, sizeof(struct stat));
}

SMASH_VISIBLE int fstatat(int dirfd, const char* path, struct stat* st, int flags) {
    using fn_t = int(*)(int, const char*, struct stat*, int);
    SMASH_LAZY_RESOLVE(fn_t, fstatat);
    if (!real_fstatat) return syscall(SYS_newfstatat, dirfd, path, st, flags);
    return smash::vm::retryWith1Buf(
        [&] { return real_fstatat(dirfd, path, st, flags); },
        st, sizeof(struct stat));
}

SMASH_VISIBLE int stat64(const char* path, struct stat64* st) {
    using fn_t = int(*)(const char*, struct stat64*);
    SMASH_LAZY_RESOLVE(fn_t, stat64);
    if (!real_stat64) return syscall(SYS_newfstatat, AT_FDCWD, path, st, 0);
    return smash::vm::retryWith1Buf(
        [&] { return real_stat64(path, st); },
        st, sizeof(struct stat64));
}

SMASH_VISIBLE int lstat64(const char* path, struct stat64* st) {
    using fn_t = int(*)(const char*, struct stat64*);
    SMASH_LAZY_RESOLVE(fn_t, lstat64);
    if (!real_lstat64) return syscall(SYS_newfstatat, AT_FDCWD, path, st, AT_SYMLINK_NOFOLLOW);
    return smash::vm::retryWith1Buf(
        [&] { return real_lstat64(path, st); },
        st, sizeof(struct stat64));
}

SMASH_VISIBLE int fstatat64(int dirfd, const char* path, struct stat64* st, int flags) {
    using fn_t = int(*)(int, const char*, struct stat64*, int);
    SMASH_LAZY_RESOLVE(fn_t, fstatat64);
    if (!real_fstatat64) return syscall(SYS_newfstatat, dirfd, path, st, flags);
    return smash::vm::retryWith1Buf(
        [&] { return real_fstatat64(dirfd, path, st, flags); },
        st, sizeof(struct stat64));
}

// __xstat / __lxstat / __fxstatat (and _64 variants): glibc < 2.33 internal
// symbols that the old stat / lstat / fstatat inlines forwarded to.
SMASH_VISIBLE int __xstat(int ver, const char* path, struct stat* st) {
    using fn_t = int(*)(int, const char*, struct stat*);
    SMASH_LAZY_RESOLVE(fn_t, __xstat);
    if (!real___xstat) return syscall(SYS_newfstatat, AT_FDCWD, path, st, 0);
    return smash::vm::retryWith1Buf(
        [&] { return real___xstat(ver, path, st); },
        st, sizeof(struct stat));
}

SMASH_VISIBLE int __lxstat(int ver, const char* path, struct stat* st) {
    using fn_t = int(*)(int, const char*, struct stat*);
    SMASH_LAZY_RESOLVE(fn_t, __lxstat);
    if (!real___lxstat) return syscall(SYS_newfstatat, AT_FDCWD, path, st, AT_SYMLINK_NOFOLLOW);
    return smash::vm::retryWith1Buf(
        [&] { return real___lxstat(ver, path, st); },
        st, sizeof(struct stat));
}

SMASH_VISIBLE int __fxstatat(int ver, int dirfd, const char* path, struct stat* st, int flags) {
    using fn_t = int(*)(int, int, const char*, struct stat*, int);
    SMASH_LAZY_RESOLVE(fn_t, __fxstatat);
    if (!real___fxstatat) return syscall(SYS_newfstatat, dirfd, path, st, flags);
    return smash::vm::retryWith1Buf(
        [&] { return real___fxstatat(ver, dirfd, path, st, flags); },
        st, sizeof(struct stat));
}

SMASH_VISIBLE int __xstat64(int ver, const char* path, struct stat64* st) {
    using fn_t = int(*)(int, const char*, struct stat64*);
    SMASH_LAZY_RESOLVE(fn_t, __xstat64);
    if (!real___xstat64) return syscall(SYS_newfstatat, AT_FDCWD, path, st, 0);
    return smash::vm::retryWith1Buf(
        [&] { return real___xstat64(ver, path, st); },
        st, sizeof(struct stat64));
}

SMASH_VISIBLE int __lxstat64(int ver, const char* path, struct stat64* st) {
    using fn_t = int(*)(int, const char*, struct stat64*);
    SMASH_LAZY_RESOLVE(fn_t, __lxstat64);
    if (!real___lxstat64) return syscall(SYS_newfstatat, AT_FDCWD, path, st, AT_SYMLINK_NOFOLLOW);
    return smash::vm::retryWith1Buf(
        [&] { return real___lxstat64(ver, path, st); },
        st, sizeof(struct stat64));
}

SMASH_VISIBLE int __fxstatat64(int ver, int dirfd, const char* path, struct stat64* st, int flags) {
    using fn_t = int(*)(int, int, const char*, struct stat64*, int);
    SMASH_LAZY_RESOLVE(fn_t, __fxstatat64);
    if (!real___fxstatat64) return syscall(SYS_newfstatat, dirfd, path, st, flags);
    return smash::vm::retryWith1Buf(
        [&] { return real___fxstatat64(ver, dirfd, path, st, flags); },
        st, sizeof(struct stat64));
}

// statx — modern stat replacement (glibc 2.28+). Single-version symbol,
// no GLIBC_2.33 aliasing dance needed.
SMASH_VISIBLE int statx(int dirfd, const char* path, int flags,
                         unsigned int mask, struct statx* buf) {
    using fn_t = int(*)(int, const char*, int, unsigned int, struct statx*);
    SMASH_LAZY_RESOLVE(fn_t, statx);
    if (!real_statx) return syscall(SYS_statx, dirfd, path, flags, mask, buf);
    return smash::vm::retryWith1Buf(
        [&] { return real_statx(dirfd, path, flags, mask, buf); },
        buf, sizeof(struct statx));
}

// ── getdents64 / readlink / readlinkat ──────────────────────────────────────
// getdents64: kernel writes a sequence of struct linux_dirent64 entries into
//   a userspace buffer (typically 32 KiB). readdir() callers in glibc go
//   through __getdents64 internally (not interposable), but direct callers
//   exist. Wrap the public symbol.
// readlink / readlinkat: kernel writes the symlink target string into the
//   user buffer.

SMASH_VISIBLE ssize_t getdents64(int fd, void* dirp, size_t count) {
    using fn_t = ssize_t(*)(int, void*, size_t);
    SMASH_LAZY_RESOLVE(fn_t, getdents64);
    if (!real_getdents64) return syscall(SYS_getdents64, fd, dirp, count);
    return smash::vm::retryWith1Buf(
        [&] { return real_getdents64(fd, dirp, count); },
        dirp, count);
}

SMASH_VISIBLE ssize_t readlink(const char* path, char* buf, size_t bufsize) {
    using fn_t = ssize_t(*)(const char*, char*, size_t);
    SMASH_LAZY_RESOLVE(fn_t, readlink);
    if (!real_readlink) return syscall(SYS_readlinkat, AT_FDCWD, path, buf, bufsize);
    return smash::vm::retryWith1Buf(
        [&] { return real_readlink(path, buf, bufsize); },
        buf, bufsize);
}

SMASH_VISIBLE ssize_t readlinkat(int dirfd, const char* path,
                                  char* buf, size_t bufsize) {
    using fn_t = ssize_t(*)(int, const char*, char*, size_t);
    SMASH_LAZY_RESOLVE(fn_t, readlinkat);
    if (!real_readlinkat) return syscall(SYS_readlinkat, dirfd, path, buf, bufsize);
    return smash::vm::retryWith1Buf(
        [&] { return real_readlinkat(dirfd, path, buf, bufsize); },
        buf, bufsize);
}

// ── preadv / pwritev / preadv2 / pwritev2 ───────────────────────────────────
// Positioned vectored I/O. preadv / pwritev added in glibc 2.10 (2009);
// preadv2 / pwritev2 added in glibc 2.26 (2017). The kernel reads from /
// writes to each iovec entry; on a compressed page the syscall returns
// EFAULT exactly like read/writev.

SMASH_VISIBLE ssize_t preadv(int fd, const struct iovec* iov, int iovcnt, off_t offset) {
    using fn_t = ssize_t(*)(int, const struct iovec*, int, off_t);
    SMASH_LAZY_RESOLVE(fn_t, preadv);
    if (!real_preadv) return syscall(SYS_preadv, fd, iov, iovcnt, offset);
    return smash::vm::retryWithIovec(
        [&] { return real_preadv(fd, iov, iovcnt, offset); },
        iov, iovcnt);
}

SMASH_VISIBLE ssize_t pwritev(int fd, const struct iovec* iov, int iovcnt, off_t offset) {
    using fn_t = ssize_t(*)(int, const struct iovec*, int, off_t);
    SMASH_LAZY_RESOLVE(fn_t, pwritev);
    if (!real_pwritev) return syscall(SYS_pwritev, fd, iov, iovcnt, offset);
    return smash::vm::retryWithIovec(
        [&] { return real_pwritev(fd, iov, iovcnt, offset); },
        iov, iovcnt);
}

SMASH_VISIBLE ssize_t preadv2(int fd, const struct iovec* iov, int iovcnt,
                               off_t offset, int flags) {
    using fn_t = ssize_t(*)(int, const struct iovec*, int, off_t, int);
    SMASH_LAZY_RESOLVE(fn_t, preadv2);
    if (!real_preadv2) return syscall(SYS_preadv2, fd, iov, iovcnt, offset, flags);
    return smash::vm::retryWithIovec(
        [&] { return real_preadv2(fd, iov, iovcnt, offset, flags); },
        iov, iovcnt);
}

SMASH_VISIBLE ssize_t pwritev2(int fd, const struct iovec* iov, int iovcnt,
                                off_t offset, int flags) {
    using fn_t = ssize_t(*)(int, const struct iovec*, int, off_t, int);
    SMASH_LAZY_RESOLVE(fn_t, pwritev2);
    if (!real_pwritev2) return syscall(SYS_pwritev2, fd, iov, iovcnt, offset, flags);
    return smash::vm::retryWithIovec(
        [&] { return real_pwritev2(fd, iov, iovcnt, offset, flags); },
        iov, iovcnt);
}

// ── waitid / wait4 (process management) ─────────────────────────────────────
// waitid writes a 128-byte siginfo_t into *infop; wait4 writes int wstatus
// AND struct rusage. wstatus is usually stack (4 bytes); rusage may be heap
// for long-lived accumulating perf counters.

SMASH_VISIBLE int waitid(idtype_t idtype, id_t id, siginfo_t* info, int options) {
    using fn_t = int(*)(idtype_t, id_t, siginfo_t*, int);
    SMASH_LAZY_RESOLVE(fn_t, waitid);
    if (!real_waitid) return syscall(SYS_waitid, idtype, id, info, options, nullptr);
    return smash::vm::retryWith1Buf(
        [&] { return real_waitid(idtype, id, info, options); },
        info, sizeof(siginfo_t));
}

SMASH_VISIBLE pid_t wait4(pid_t pid, int* wstatus, int options, struct rusage* rusage) {
    using fn_t = pid_t(*)(pid_t, int*, int, struct rusage*);
    SMASH_LAZY_RESOLVE(fn_t, wait4);
    if (!real_wait4) return syscall(SYS_wait4, pid, wstatus, options, rusage);
    auto* vm = smash::g_smash_vm_region;
    return smash::vm::retryWithDecompress(
        [&] { return real_wait4(pid, wstatus, options, rusage); },
        [&] {
            if (vm) {
                if (wstatus) smash::vm::walkPagesForFault(wstatus, sizeof(int), vm);
                if (rusage) smash::vm::walkPagesForFault(rusage, sizeof(struct rusage), vm);
            }
        });
}

// ── statvfs / fstatvfs ──────────────────────────────────────────────────────
// POSIX path-form statfs. struct statvfs is similar in size to statfs (~112B);
// heap-allocated when callers track multiple filesystems.
SMASH_VISIBLE int statvfs(const char* path, struct statvfs* st) {
    using fn_t = int(*)(const char*, struct statvfs*);
    SMASH_LAZY_RESOLVE(fn_t, statvfs);
    if (!real_statvfs) {
        errno = ENOSYS;
        return -1;
    }
    return smash::vm::retryWith1Buf(
        [&] { return real_statvfs(path, st); },
        st, sizeof(struct statvfs));
}

SMASH_VISIBLE int fstatvfs(int fd, struct statvfs* st) {
    using fn_t = int(*)(int, struct statvfs*);
    SMASH_LAZY_RESOLVE(fn_t, fstatvfs);
    if (!real_fstatvfs) {
        errno = ENOSYS;
        return -1;
    }
    return smash::vm::retryWith1Buf(
        [&] { return real_fstatvfs(fd, st); },
        st, sizeof(struct statvfs));
}

// ── xattr family ────────────────────────────────────────────────────────────
// getxattr / lgetxattr / fgetxattr write attribute value into user buffer.
// listxattr / llistxattr / flistxattr write a NUL-separated name list.
// Used by SELinux/AppArmor labels, cap_net_raw, mac sandbox metadata, etc.
SMASH_VISIBLE ssize_t getxattr(const char* path, const char* name,
                                void* value, size_t size) {
    using fn_t = ssize_t(*)(const char*, const char*, void*, size_t);
    SMASH_LAZY_RESOLVE(fn_t, getxattr);
    if (!real_getxattr) return syscall(SYS_getxattr, path, name, value, size);
    return smash::vm::retryWith1Buf(
        [&] { return real_getxattr(path, name, value, size); },
        value, size);
}

SMASH_VISIBLE ssize_t lgetxattr(const char* path, const char* name,
                                 void* value, size_t size) {
    using fn_t = ssize_t(*)(const char*, const char*, void*, size_t);
    SMASH_LAZY_RESOLVE(fn_t, lgetxattr);
    if (!real_lgetxattr) return syscall(SYS_lgetxattr, path, name, value, size);
    return smash::vm::retryWith1Buf(
        [&] { return real_lgetxattr(path, name, value, size); },
        value, size);
}

SMASH_VISIBLE ssize_t fgetxattr(int fd, const char* name,
                                 void* value, size_t size) {
    using fn_t = ssize_t(*)(int, const char*, void*, size_t);
    SMASH_LAZY_RESOLVE(fn_t, fgetxattr);
    if (!real_fgetxattr) return syscall(SYS_fgetxattr, fd, name, value, size);
    return smash::vm::retryWith1Buf(
        [&] { return real_fgetxattr(fd, name, value, size); },
        value, size);
}

SMASH_VISIBLE ssize_t listxattr(const char* path, char* list, size_t size) {
    using fn_t = ssize_t(*)(const char*, char*, size_t);
    SMASH_LAZY_RESOLVE(fn_t, listxattr);
    if (!real_listxattr) return syscall(SYS_listxattr, path, list, size);
    return smash::vm::retryWith1Buf(
        [&] { return real_listxattr(path, list, size); },
        list, size);
}

SMASH_VISIBLE ssize_t llistxattr(const char* path, char* list, size_t size) {
    using fn_t = ssize_t(*)(const char*, char*, size_t);
    SMASH_LAZY_RESOLVE(fn_t, llistxattr);
    if (!real_llistxattr) return syscall(SYS_llistxattr, path, list, size);
    return smash::vm::retryWith1Buf(
        [&] { return real_llistxattr(path, list, size); },
        list, size);
}

SMASH_VISIBLE ssize_t flistxattr(int fd, char* list, size_t size) {
    using fn_t = ssize_t(*)(int, char*, size_t);
    SMASH_LAZY_RESOLVE(fn_t, flistxattr);
    if (!real_flistxattr) return syscall(SYS_flistxattr, fd, list, size);
    return smash::vm::retryWith1Buf(
        [&] { return real_flistxattr(fd, list, size); },
        list, size);
}

// ── getrusage / prlimit64 ───────────────────────────────────────────────────
// getrusage writes struct rusage; prlimit64 writes (and optionally reads)
// struct rlimit. Both can be heap-allocated when callers persist them.
SMASH_VISIBLE int getrusage(int who, struct rusage* usage) {
    using fn_t = int(*)(int, struct rusage*);
    SMASH_LAZY_RESOLVE(fn_t, getrusage);
    if (!real_getrusage) return syscall(SYS_getrusage, who, usage);
    return smash::vm::retryWith1Buf(
        [&] { return real_getrusage(who, usage); },
        usage, sizeof(struct rusage));
}

// glibc declares prlimit64's resource arg as `enum __rlimit_resource`,
// not plain `int`, so the C-linkage signature must match exactly.
SMASH_VISIBLE int prlimit64(pid_t pid, enum __rlimit_resource resource,
                             const struct rlimit64* new_limit,
                             struct rlimit64* old_limit) {
    using fn_t = int(*)(pid_t, enum __rlimit_resource,
                         const struct rlimit64*, struct rlimit64*);
    SMASH_LAZY_RESOLVE(fn_t, prlimit64);
    if (!real_prlimit64) return syscall(SYS_prlimit64, pid, resource, new_limit, old_limit);
    auto* vm = smash::g_smash_vm_region;
    return smash::vm::retryWithDecompress(
        [&] { return real_prlimit64(pid, resource, new_limit, old_limit); },
        [&] {
            if (vm) {
                if (new_limit)
                    smash::vm::walkPagesForFault(new_limit, sizeof(struct rlimit64), vm);
                if (old_limit)
                    smash::vm::walkPagesForFault(old_limit, sizeof(struct rlimit64), vm);
            }
        });
}

// ── getrandom ───────────────────────────────────────────────────────────────
// The kernel fills a userspace buffer with random bytes. NSS/OpenSSL/Firefox
// use this for cryptographic initialization.
SMASH_VISIBLE ssize_t getrandom(void* buf, size_t buflen, unsigned int flags) {
    using fn_t = ssize_t(*)(void*, size_t, unsigned int);
    SMASH_LAZY_RESOLVE(fn_t, getrandom);
    if (!real_getrandom) return syscall(SYS_getrandom, buf, buflen, flags);
    return smash::vm::retryWith1Buf(
        [&] { return real_getrandom(buf, buflen, flags); },
        buf, buflen);
}

// ── CPU affinity ────────────────────────────────────────────────────────────
// The kernel reads (set) or writes (get) a cpu_set_t, and thread pools keep one
// per worker on the heap. Measured: with the mask on a compressed page,
// sched_setaffinity returns EFAULT and the pin silently does not happen.
//
// pthread_{set,get}affinity_np need their own wrappers. glibc reaches the
// kernel through an internal __sched_setaffinity, so interposing
// sched_setaffinity does not cover them. They also report failure by return
// value rather than through errno, so they warm the mask up front rather than
// relying only on the EFAULT retry.
SMASH_VISIBLE int sched_setaffinity(pid_t pid, size_t cpusetsize,
                                    const cpu_set_t* mask) {
    using fn_t = int(*)(pid_t, size_t, const cpu_set_t*);
    SMASH_LAZY_RESOLVE(fn_t, sched_setaffinity);
    if (!real_sched_setaffinity)
        return static_cast<int>(syscall(SYS_sched_setaffinity, pid, cpusetsize, mask));
    return smash::vm::retryWith1Buf(
        [&] { return real_sched_setaffinity(pid, cpusetsize, mask); },
        mask, cpusetsize);
}

SMASH_VISIBLE int sched_getaffinity(pid_t pid, size_t cpusetsize, cpu_set_t* mask) {
    using fn_t = int(*)(pid_t, size_t, cpu_set_t*);
    SMASH_LAZY_RESOLVE(fn_t, sched_getaffinity);
    if (!real_sched_getaffinity)
        return static_cast<int>(syscall(SYS_sched_getaffinity, pid, cpusetsize, mask));
    return smash::vm::retryWith1Buf(
        [&] { return real_sched_getaffinity(pid, cpusetsize, mask); },
        mask, cpusetsize);
}

SMASH_VISIBLE int pthread_setaffinity_np(pthread_t thread, size_t cpusetsize,
                                         const cpu_set_t* mask) {
    using fn_t = int(*)(pthread_t, size_t, const cpu_set_t*);
    SMASH_LAZY_RESOLVE(fn_t, pthread_setaffinity_np);
    auto* vm = smash::g_smash_vm_region;
    // Through a void* so the nonnull-declared parameter still gets checked:
    // an interposer sees whatever the application actually passes.
    const void* mask_mem = mask;
    if (vm && mask_mem && cpusetsize) smash::vm::warmPages(mask_mem, cpusetsize, vm);
    if (!real_pthread_setaffinity_np) {
        // dlsym recursion during early init. The raw syscall covers the common
        // self-pinning case; anything else has to wait for a later call.
        if (pthread_equal(thread, pthread_self()))
            return syscall(SYS_sched_setaffinity, 0, cpusetsize, mask) == 0 ? 0 : errno;
        return ENOSYS;
    }
    int rc = real_pthread_setaffinity_np(thread, cpusetsize, mask);
    for (int attempt = 0; rc == EFAULT && attempt < 8; ++attempt) {
        if (vm && mask_mem && cpusetsize) smash::vm::warmPages(mask_mem, cpusetsize, vm);
        rc = real_pthread_setaffinity_np(thread, cpusetsize, mask);
    }
    return rc;
}

SMASH_VISIBLE int pthread_getaffinity_np(pthread_t thread, size_t cpusetsize,
                                         cpu_set_t* mask) {
    using fn_t = int(*)(pthread_t, size_t, cpu_set_t*);
    SMASH_LAZY_RESOLVE(fn_t, pthread_getaffinity_np);
    auto* vm = smash::g_smash_vm_region;
    const void* mask_mem = mask;
    if (vm && mask_mem && cpusetsize) smash::vm::warmPages(mask_mem, cpusetsize, vm);
    if (!real_pthread_getaffinity_np) {
        if (pthread_equal(thread, pthread_self()))
            return syscall(SYS_sched_getaffinity, 0, cpusetsize, mask) >= 0 ? 0 : errno;
        return ENOSYS;
    }
    int rc = real_pthread_getaffinity_np(thread, cpusetsize, mask);
    for (int attempt = 0; rc == EFAULT && attempt < 8; ++attempt) {
        if (vm && mask_mem && cpusetsize) smash::vm::warmPages(mask_mem, cpusetsize, vm);
        rc = real_pthread_getaffinity_np(thread, cpusetsize, mask);
    }
    return rc;
}

// Callers bind the versioned symbols glibc actually exports —
// sched_*@GLIBC_2.3.4, pthread_setaffinity_np@GLIBC_2.34,
// pthread_getaffinity_np@GLIBC_2.32 — and a versioned reference does not match
// the base-version definitions above, so libc keeps winning. (Same trap as
// fstat@GLIBC_2.33; see smash_version_script.map.in.) Export aliases at those
// versions too, following the epoll_wait@GLIBC_2.3.2 pattern.
SMASH_VISIBLE int sched_setaffinity_234(pid_t pid, size_t cpusetsize,
                                        const cpu_set_t* mask) {
    return sched_setaffinity(pid, cpusetsize, mask);
}
__asm__(".symver sched_setaffinity_234,sched_setaffinity@GLIBC_2.3.4");

SMASH_VISIBLE int sched_getaffinity_234(pid_t pid, size_t cpusetsize, cpu_set_t* mask) {
    return sched_getaffinity(pid, cpusetsize, mask);
}
__asm__(".symver sched_getaffinity_234,sched_getaffinity@GLIBC_2.3.4");

SMASH_VISIBLE int pthread_setaffinity_np_234(pthread_t thread, size_t cpusetsize,
                                             const cpu_set_t* mask) {
    return pthread_setaffinity_np(thread, cpusetsize, mask);
}
__asm__(".symver pthread_setaffinity_np_234,pthread_setaffinity_np@GLIBC_2.34");

SMASH_VISIBLE int pthread_getaffinity_np_232(pthread_t thread, size_t cpusetsize,
                                             cpu_set_t* mask) {
    return pthread_getaffinity_np(thread, cpusetsize, mask);
}
__asm__(".symver pthread_getaffinity_np_232,pthread_getaffinity_np@GLIBC_2.32");

// ── pthread_create / pthread_exit at the version modern glibc uses ──────────
// alloc8 exports pthread_create/pthread_exit (linux_threads.cpp) to wrap every
// thread in a trampoline that calls xxthread_init / xxthread_cleanup. glibc
// 2.34 merged libpthread into libc and moved the default version to
// GLIBC_2.34, so a caller built against it binds libc directly and alloc8's
// interposer never runs for application threads. Measured on this tree before
// these aliases existed: of 200 application threads, 16 were wrapped — and all
// 16 were smash's own compressor threads, whose internal calls bind inside
// libsmash. Every unwrapped thread leaked its ~92 KiB ThreadCache.
//
// Forcing a caller to request pthread_create@GLIBC_2.2.5 flipped it: 216
// wrapped, 199 cache reuses, RSS growth 91.2 -> 0.3 KiB per thread. These
// aliases make that the default. The calls below resolve inside libsmash
// (-Bsymbolic), i.e. to alloc8's interposer.
SMASH_VISIBLE int pthread_create_234(pthread_t* thread, const pthread_attr_t* attr,
                                     void* (*start_routine)(void*), void* arg) {
    return pthread_create(thread, attr, start_routine, arg);
}
__asm__(".symver pthread_create_234,pthread_create@GLIBC_2.34");

SMASH_VISIBLE void pthread_exit_234(void* value_ptr) {
    pthread_exit(value_ptr);
    __builtin_unreachable();
}
__asm__(".symver pthread_exit_234,pthread_exit@GLIBC_2.34");

// ── getcwd wrapper ──────────────────────────────────────────────────────────
// getcwd writes the current working directory path into a user-provided buffer.
// Uses the standard EFAULT-retry pattern like other syscall wrappers.

SMASH_VISIBLE char* getcwd(char* buf, size_t size) {
    using fn_t = char*(*)(char*, size_t);
    SMASH_LAZY_RESOLVE(fn_t, getcwd);

    // For the buf==NULL case, glibc allocates a buffer internally that may
    // land in a smash-managed page. The kernel can't take a SIGSEGV when
    // copy_to_user hits a protected page during the syscall — it returns
    // EFAULT instead. Without a buf to walk, we can't proactively warm.
    //
    // Workaround: use a stack buffer ourselves, then strdup the result so
    // the C++ filesystem layer's getcwd(NULL,0) idiom keeps working. The
    // stack page is already PROT_RW and unmanaged by smash. Apply the
    // standard EFAULT-retry pattern in case the stack itself is somehow
    // protected (shouldn't happen, but harmless).
    auto* vm = smash::g_smash_vm_region;
    if (!buf) {
        char tmp[PATH_MAX];
        auto do_getcwd_tmp = [&]() -> char* {
            if (!real_getcwd) {
                return (syscall(SYS_getcwd, tmp, sizeof(tmp)) < 0) ? nullptr : tmp;
            }
            return real_getcwd(tmp, sizeof(tmp));
        };
        char* ret = do_getcwd_tmp();
        long backoff_ns = 1000;
        for (int attempt = 0; ret == nullptr && errno == EFAULT && attempt < 8; ++attempt) {
            if (vm) smash::vm::walkPagesForFault(tmp, sizeof(tmp), vm);
            if (attempt > 0) {
                struct timespec ts = {0, backoff_ns};
                nanosleep(&ts, nullptr);
                backoff_ns *= 2;
            }
            ret = do_getcwd_tmp();
        }
        if (!ret) return nullptr;
        return strdup(ret);
    }

    // Standard retry pattern when caller supplied a buffer.
    auto do_getcwd = [&]() -> char* {
        if (!real_getcwd) {
            return (syscall(SYS_getcwd, buf, size) < 0) ? nullptr : buf;
        }
        return real_getcwd(buf, size);
    };

    char* ret = do_getcwd();
    long backoff_ns = 1000;
    for (int attempt = 0; ret == nullptr && errno == EFAULT && attempt < 8; ++attempt) {
        if (vm && size) smash::vm::walkPagesForFault(buf, size, vm);
        if (attempt > 0) {
            struct timespec ts = {0, backoff_ns};
            nanosleep(&ts, nullptr);
            backoff_ns *= 2;
        }
        ret = do_getcwd();
    }
    return ret;
}

// Legacy getcwd hooks removed - the direct wrapper above handles everything.

// ── Buffered I/O wrappers ───────────────────────────────────────────────────
// glibc's fread/fgets/fgetc call __read() internally without going through
// the PLT, so our read() wrapper never sees those calls.  We interpose the
// higher-level functions to warm+pin the FILE's internal buffer (where the
// kernel writes via copyin/copyout) before passing through to the real impl.
//
// glibc FILE struct: _IO_buf_base .. _IO_buf_end is the reserve area.

static inline void warmGlibcFileBuffer(FILE* stream, smash::VmRegion* vm) {
    if (!stream || !vm) return;
    char* base = stream->_IO_buf_base;
    char* end  = stream->_IO_buf_end;
    if (base && end > base) {
        size_t sz = static_cast<size_t>(end - base);
        if (bufferInHeap(base, sz, vm)) {
            smash::vm::warmPages(base, sz, vm);
        }
    }
}

SMASH_VISIBLE size_t fread(void* ptr, size_t size, size_t nmemb, FILE* stream) {
    using fn_t = size_t(*)(void*, size_t, size_t, FILE*);
    SMASH_LAZY_RESOLVE(fn_t, fread);
    if (!real_fread) return 0;
    auto* vm = smash::g_smash_vm_region;
    size_t total = size * nmemb;
    bool pin_buf = total && bufferInHeap(ptr, total, vm);
    if (pin_buf) smash::vm::warmPages(ptr, total, vm);
    warmGlibcFileBuffer(stream, vm);
    size_t ret = real_fread(ptr, size, nmemb, stream);
    return ret;
}

SMASH_VISIBLE char* fgets(char* s, int size, FILE* stream) {
    using fn_t = char*(*)(char*, int, FILE*);
    SMASH_LAZY_RESOLVE(fn_t, fgets);
    if (!real_fgets) return nullptr;
    auto* vm = smash::g_smash_vm_region;
    bool pin_s = (s && size > 0) && bufferInHeap(s, static_cast<size_t>(size), vm);
    if (pin_s) smash::vm::warmPages(s, size, vm);
    warmGlibcFileBuffer(stream, vm);
    char* ret = real_fgets(s, size, stream);
    return ret;
}

SMASH_VISIBLE int fgetc(FILE* stream) {
    using fn_t = int(*)(FILE*);
    SMASH_LAZY_RESOLVE(fn_t, fgetc);
    if (!real_fgetc) return EOF;
    auto* vm = smash::g_smash_vm_region;
    warmGlibcFileBuffer(stream, vm);
    int ret = real_fgetc(stream);
    return ret;
}

SMASH_VISIBLE int getc(FILE* stream) {
    // getc is semantically identical to fgetc
    return fgetc(stream);
}

SMASH_VISIBLE size_t fwrite(const void* ptr, size_t size, size_t nmemb, FILE* stream) {
    using fn_t = size_t(*)(const void*, size_t, size_t, FILE*);
    SMASH_LAZY_RESOLVE(fn_t, fwrite);
    if (!real_fwrite) return 0;
    auto* vm = smash::g_smash_vm_region;
    size_t total = size * nmemb;
    if (total && bufferInHeap(ptr, total, vm))
        smash::vm::warmPages(const_cast<void*>(ptr), total, vm);
    warmGlibcFileBuffer(stream, vm);
    size_t ret = real_fwrite(ptr, size, nmemb, stream);
    return ret;
}

SMASH_VISIBLE int fflush(FILE* stream) {
    using fn_t = int(*)(FILE*);
    SMASH_LAZY_RESOLVE(fn_t, fflush);
    if (!real_fflush) return EOF;
    auto* vm = smash::g_smash_vm_region;
    warmGlibcFileBuffer(stream, vm);
    int ret = real_fflush(stream);
    return ret;
}

// NOTE: ioctl is NOT interposed. The variadic signature makes it impossible
// to know whether the third argument is a pointer or integer, and guessing
// wrong corrupts warmPages/pinPages calls. Most ioctls use stack buffers
// or non-heap memory anyway.

// epoll_wait/epoll_pwait - libevent allocates event buffers via malloc,
// so they ARE Smash-managed and need warming before kernel access.
// NOTE: Some apps (libevent) may mmap their own buffers, so we warm
// unconditionally if events is non-null to avoid EFAULT from kernel.
SMASH_VISIBLE int epoll_wait(int epfd, struct epoll_event* events, int maxevents, int timeout) {
    using fn_t = int(*)(int, struct epoll_event*, int, int);
    SMASH_LAZY_RESOLVE(fn_t, epoll_wait);
    if (!real_epoll_wait) {
#ifdef SYS_epoll_wait
        return syscall(SYS_epoll_wait, epfd, events, maxevents, timeout);
#else
        // arm64 / riscv have no SYS_epoll_wait; route via epoll_pwait
        // with NULL sigmask. _NSIG/8 = 8 (kernel sigset_t size).
        return syscall(SYS_epoll_pwait, epfd, events, maxevents, timeout, nullptr, 8);
#endif
    }
    auto* vm = smash::g_smash_vm_region;
    size_t size = static_cast<size_t>(maxevents) * sizeof(struct epoll_event);
    bool in_heap = bufferInHeap(events, size, vm);
    int ret = smash::vm::retryWithDecompress(
        [&] { return real_epoll_wait(epfd, events, maxevents, timeout); },
        [&] { if (in_heap) smash::vm::walkPagesForFault(events, size, vm); });
    return ret;
}

SMASH_VISIBLE int epoll_pwait(int epfd, struct epoll_event* events, int maxevents, int timeout, const sigset_t* sigmask) {
    using fn_t = int(*)(int, struct epoll_event*, int, int, const sigset_t*);
    SMASH_LAZY_RESOLVE(fn_t, epoll_pwait);
    if (!real_epoll_pwait) return syscall(SYS_epoll_pwait, epfd, events, maxevents, timeout, sigmask);
    auto* vm = smash::g_smash_vm_region;
    size_t size = static_cast<size_t>(maxevents) * sizeof(struct epoll_event);
    bool in_heap = bufferInHeap(events, size, vm);
    int ret = smash::vm::retryWithDecompress(
        [&] { return real_epoll_pwait(epfd, events, maxevents, timeout, sigmask); },
        [&] { if (in_heap) smash::vm::walkPagesForFault(events, size, vm); });
    return ret;
}

// Versioned symbol aliases for epoll_wait/epoll_pwait
// libevent requests epoll_wait@GLIBC_2.3.2, glibc uses 2.2.5 internally
// We export both versions pointing to the same implementation
SMASH_VISIBLE int epoll_wait_232(int epfd, struct epoll_event* events, int maxevents, int timeout) {
    // Forward to the main implementation
    return epoll_wait(epfd, events, maxevents, timeout);
}

SMASH_VISIBLE int epoll_pwait_232(int epfd, struct epoll_event* events, int maxevents, int timeout, const sigset_t* sigmask) {
    return epoll_pwait(epfd, events, maxevents, timeout, sigmask);
}

// ── fstat versioned aliases (GLIBC_2.33) ────────────────────────────────────
// glibc 2.34 re-versioned fstat/fstat64 from GLIBC_2.2.5 to GLIBC_2.33 to
// carry y2038-safe struct stat. C++ binaries on Ubuntu 22.04+ reference
// fstat@GLIBC_2.33 specifically; without these aliases the dynamic linker
// won't match our LD_PRELOAD'd fstat (which lives at GLIBC_2.2.5) and
// goes to glibc directly.
SMASH_VISIBLE int fstat_233(int fd, struct stat* st) {
    return fstat(fd, st);
}
SMASH_VISIBLE int fstat64_233(int fd, struct stat64* st) {
    return fstat64(fd, st);
}
SMASH_VISIBLE int stat_233(const char* path, struct stat* st) {
    return stat(path, st);
}
SMASH_VISIBLE int stat64_233(const char* path, struct stat64* st) {
    return stat64(path, st);
}
SMASH_VISIBLE int lstat_233(const char* path, struct stat* st) {
    return lstat(path, st);
}
SMASH_VISIBLE int lstat64_233(const char* path, struct stat64* st) {
    return lstat64(path, st);
}
SMASH_VISIBLE int fstatat_233(int dirfd, const char* path, struct stat* st, int flags) {
    return fstatat(dirfd, path, st, flags);
}
SMASH_VISIBLE int fstatat64_233(int dirfd, const char* path, struct stat64* st, int flags) {
    return fstatat64(dirfd, path, st, flags);
}
// statx was introduced in glibc 2.28 — binaries reference statx@GLIBC_2.28.
SMASH_VISIBLE int statx_228(int dirfd, const char* path, int flags,
                             unsigned int mask, struct statx* buf) {
    return statx(dirfd, path, flags, mask, buf);
}
// getdents64 userspace function added in glibc 2.30 — binaries built
// against glibc 2.30+ reference getdents64@GLIBC_2.30.
SMASH_VISIBLE ssize_t getdents64_230(int fd, void* dirp, size_t count) {
    return getdents64(fd, dirp, count);
}
// preadv / pwritev introduced in glibc 2.10.
SMASH_VISIBLE ssize_t preadv_210(int fd, const struct iovec* iov, int iovcnt, off_t offset) {
    return preadv(fd, iov, iovcnt, offset);
}
SMASH_VISIBLE ssize_t pwritev_210(int fd, const struct iovec* iov, int iovcnt, off_t offset) {
    return pwritev(fd, iov, iovcnt, offset);
}
// preadv2 / pwritev2 introduced in glibc 2.26.
SMASH_VISIBLE ssize_t preadv2_226(int fd, const struct iovec* iov, int iovcnt, off_t offset, int flags) {
    return preadv2(fd, iov, iovcnt, offset, flags);
}
SMASH_VISIBLE ssize_t pwritev2_226(int fd, const struct iovec* iov, int iovcnt, off_t offset, int flags) {
    return pwritev2(fd, iov, iovcnt, offset, flags);
}

// ── External-mapping interposers (mmap / munmap) ────────────────────────────
//
// Application-direct mmap calls (e.g., SpiderMonkey GC arenas, jemalloc
// internal slabs in projects that bundle it) bypass smash's malloc and so
// escape compression. Track MAP_ANONYMOUS + PROT_WRITE mappings via
// VmRegion's external-page hash so the compressor's tick can see them.
// File-backed and read-only mappings are explicitly skipped (the former
// would break msync semantics under compression; the latter never need it).
//
// External tracking is OFF by default; set SMASH_TRACK_EXTERNAL=1 to
// enable. Same opt-in polarity as the macOS path — see smash_heap.cpp
// for the rationale (Firefox-on-macOS regression with the registration
// path active).

namespace {

inline bool externalTrackingEnabledLinux() {
    static const bool enabled = []{
        const char* v = std::getenv("SMASH_TRACK_EXTERNAL");
        return v && v[0] == '1';
    }();
    return enabled;
}

inline void registerLinuxExternalRange(smash::VmRegion* vm, void* base, size_t len) {
    if (!vm || !base || !len) return;
    if (!externalTrackingEnabledLinux()) return;
    // Profile-driven skip: if we already know external pages are hot from a
    // prior run, skip the O(pages) registration loop entirely.
    if (smash::g_smash_skip_external_tracking.load(std::memory_order_acquire)) return;
    // Safety check: page_states must be initialized
    if (!smash::g_smash_page_states_for_external) return;

    size_t npages = (len + smash::kPageSize - 1) / smash::kPageSize;

    // Oversized single mapping guard. A mapping larger than the entire
    // external-page slot budget can never be fully tracked, and such mappings
    // are invariably app-managed arenas (a DB buffer pool, a JVM/GC heap) that
    // the application keeps warm — so compressing them is counterproductive
    // AND the registration loop would burn O(slot-budget) work inside the
    // caller's mmap() (observed wedging mariadbd startup for minutes on a
    // 6 GiB InnoDB buffer pool). Skip the whole mapping rather than tracking a
    // useless prefix. Logged once so the coverage gap is never silent.
    if (npages > vm->externalSlotCapacity()) {
        static std::atomic<bool> warned{false};
        bool expected = false;
        if (warned.compare_exchange_strong(expected, true,
                                           std::memory_order_relaxed)) {
            char buf[192];
            int n = smash::safe_snprintf(buf, sizeof(buf),
                "[smash] SMASH_TRACK_EXTERNAL: skipping mmap of %zu pages "
                "(> %zu slot budget) at %p — likely an app-managed arena kept "
                "warm; not compressible. Further such skips are silent.\n",
                npages, vm->externalSlotCapacity(), base);
            if (n > 0) (void)!::write(2, buf, (size_t)n);
        }
        return;
    }

    auto start = reinterpret_cast<uintptr_t>(base) & ~(uintptr_t{smash::kPageSize} - 1);
    auto end = (reinterpret_cast<uintptr_t>(base) + len + smash::kPageSize - 1)
               & ~(uintptr_t{smash::kPageSize} - 1);
    for (uintptr_t p = start; p < end; p += smash::kPageSize) {
        size_t idx = vm->trackExternalPage(p);
        if (idx == 0) continue;
        smash::g_smash_page_states_for_external->set(idx, smash::PageState::ACTIVE);
    }
}

inline void deregisterLinuxExternalRange(smash::VmRegion* vm, void* base, size_t len) {
    if (!vm || !base || !len) return;
    if (!externalTrackingEnabledLinux()) return;
    // Safety check: page_states must be initialized
    if (!smash::g_smash_page_states_for_external) return;

    auto start = reinterpret_cast<uintptr_t>(base) & ~(uintptr_t{smash::kPageSize} - 1);
    auto end = (reinterpret_cast<uintptr_t>(base) + len + smash::kPageSize - 1)
               & ~(uintptr_t{smash::kPageSize} - 1);
    for (uintptr_t p = start; p < end; p += smash::kPageSize) {
        // Untrack UNDER the per-page lock so a compressor worker mid-snapshot on
        // this external page can't race the real munmap (in the caller, AFTER
        // this returns) and read an unmapped page. Proven necessary +
        // deadlock-free by model/SmashExternalRace.{lean,tla}: no
        // mprotect/munmap is performed while the per-page lock is held.
        vm->untrackExternalPageLocked(p,
            smash::g_smash_page_locks_for_external,
            smash::g_smash_page_states_for_external);
    }
}

// Which application mmaps are compression candidates. Anonymous and writable
// only (see above), and never MAP_HUGETLB: a hugetlb VMA only accepts
// huge-page-aligned addresses and lengths, so every 4 KiB mprotect / madvise /
// MAP_FIXED smash would issue on a registered slice is rejected with EINVAL or
// ENOMEM. Nothing records that a page is unmanageable, so the compressor used
// to retry every tick forever — 8192 failing page operations in 6 s for one
// 16 MiB mapping — while the slices permanently held external slots
// (test_hugepage_no_mprotect_storm).
inline bool isTrackableMapping(int prot, int flags, size_t len) {
    if (len == 0) return false;
    if (!(flags & MAP_ANONYMOUS)) return false;
    if (!(prot & PROT_WRITE)) return false;
#ifdef MAP_HUGETLB
    if (flags & MAP_HUGETLB) return false;
#endif
    return true;
}

}  // namespace

SMASH_VISIBLE void* mmap(void* addr, size_t len, int prot, int flags, int fd, off_t offset) {
    using fn_t = void*(*)(void*, size_t, int, int, int, off_t);
    SMASH_LAZY_RESOLVE(fn_t, mmap);
    if (!real_mmap) return MAP_FAILED;
    void* ret = real_mmap(addr, len, prot, flags, fd, offset);
    if (ret == MAP_FAILED) return ret;
    if (!isTrackableMapping(prot, flags, len)) return ret;
    auto* vm = smash::g_smash_vm_region;
    if (vm) registerLinuxExternalRange(vm, ret, len);
    return ret;
}

SMASH_VISIBLE int munmap(void* addr, size_t len) {
    using fn_t = int(*)(void*, size_t);
    SMASH_LAZY_RESOLVE(fn_t, munmap);
    if (!real_munmap) return -1;
    auto* vm = smash::g_smash_vm_region;
    if (vm && addr && len) deregisterLinuxExternalRange(vm, addr, len);
    return real_munmap(addr, len);
}

// mmap64 is the LFS (Large File Support) version used internally by glibc
SMASH_VISIBLE void* mmap64(void* addr, size_t len, int prot, int flags, int fd, off64_t offset) {
    using fn_t = void*(*)(void*, size_t, int, int, int, off64_t);
    SMASH_LAZY_RESOLVE(fn_t, mmap64);
    if (!real_mmap64) return MAP_FAILED;
    void* ret = real_mmap64(addr, len, prot, flags, fd, offset);
    if (ret == MAP_FAILED) return ret;
    if (!isTrackableMapping(prot, flags, len)) return ret;
    auto* vm = smash::g_smash_vm_region;
    if (vm) registerLinuxExternalRange(vm, ret, len);
    return ret;
}

// mremap can grow/move mappings — track new range, untrack old if moved
SMASH_VISIBLE void* mremap(void* old_addr, size_t old_size, size_t new_size, int flags, ...) {
    using fn_t = void*(*)(void*, size_t, size_t, int, ...);
    SMASH_LAZY_RESOLVE(fn_t, mremap);
    if (!real_mremap) return MAP_FAILED;

    void* new_addr = nullptr;
    if (flags & MREMAP_FIXED) {
        va_list ap;
        va_start(ap, flags);
        new_addr = va_arg(ap, void*);
        va_end(ap);
    }

    void* ret;
    if (flags & MREMAP_FIXED) {
        ret = real_mremap(old_addr, old_size, new_size, flags, new_addr);
    } else {
        ret = real_mremap(old_addr, old_size, new_size, flags);
    }

    if (ret == MAP_FAILED) return ret;

    auto* vm = smash::g_smash_vm_region;
    // mremap's flags say nothing about the mapping's type, so a moved or grown
    // range is tracked only if its source was: that keeps the mmap filter's
    // verdict (MAP_HUGETLB, file-backed, read-only are never tracked) instead
    // of registering whatever the application remaps.
    if (vm && externalTrackingEnabledLinux() &&
        vm->contains(reinterpret_cast<uintptr_t>(old_addr))) {
        // If mapping moved (ret != old_addr), untrack old and track new
        if (ret != old_addr) {
            deregisterLinuxExternalRange(vm, old_addr, old_size);
            registerLinuxExternalRange(vm, ret, new_size);
        } else if (new_size > old_size) {
            // Grew in place — track additional pages
            registerLinuxExternalRange(vm,
                reinterpret_cast<char*>(ret) + old_size,
                new_size - old_size);
        } else if (new_size < old_size) {
            // Shrunk — untrack removed pages
            deregisterLinuxExternalRange(vm,
                reinterpret_cast<char*>(ret) + new_size,
                old_size - new_size);
        }
    }
    return ret;
}

} // extern "C"

// Create version aliases: epoll_wait_232 -> epoll_wait@GLIBC_2.3.2
__asm__(".symver epoll_wait_232,epoll_wait@GLIBC_2.3.2");
__asm__(".symver epoll_pwait_232,epoll_pwait@GLIBC_2.3.2");
__asm__(".symver fstat_233,fstat@GLIBC_2.33");
__asm__(".symver fstat64_233,fstat64@GLIBC_2.33");
__asm__(".symver stat_233,stat@GLIBC_2.33");
__asm__(".symver stat64_233,stat64@GLIBC_2.33");
__asm__(".symver lstat_233,lstat@GLIBC_2.33");
__asm__(".symver lstat64_233,lstat64@GLIBC_2.33");
__asm__(".symver fstatat_233,fstatat@GLIBC_2.33");
__asm__(".symver fstatat64_233,fstatat64@GLIBC_2.33");
__asm__(".symver statx_228,statx@GLIBC_2.28");
__asm__(".symver getdents64_230,getdents64@GLIBC_2.30");
__asm__(".symver preadv_210,preadv@GLIBC_2.10");
__asm__(".symver pwritev_210,pwritev@GLIBC_2.10");
__asm__(".symver preadv2_226,preadv2@GLIBC_2.26");
__asm__(".symver pwritev2_226,pwritev2@GLIBC_2.26");

// ══════════════════════════════════════════════════════════════════════════════
// TBB scalable_allocator interposition
// ══════════════════════════════════════════════════════════════════════════════
//
// Intel TBB (Thread Building Blocks) uses its own memory allocator that bypasses
// malloc and allocates 64MB arenas directly via mmap. By interposing TBB's
// scalable_malloc family, we redirect those allocations through smash's malloc,
// making them eligible for compression.
//
// This is critical for walrus (neuronx-cc backend) which uses TBB and can have
// 15-20GB of TBB-managed memory that would otherwise escape compression.
//
// Note: These symbols are weak to allow linking even when TBB is not present.
// When TBB IS present and linked dynamically, LD_PRELOAD ensures our wrappers
// take precedence.

// TBB interposition call counters (for debugging)
static std::atomic<uint64_t> g_scalable_malloc_count{0};
static std::atomic<uint64_t> g_scalable_malloc_bytes{0};

extern "C" {

// Core allocation functions
SMASH_VISIBLE void* scalable_malloc(size_t size) {
    g_scalable_malloc_count.fetch_add(1, std::memory_order_relaxed);
    g_scalable_malloc_bytes.fetch_add(size, std::memory_order_relaxed);
    return malloc(size);
}

SMASH_VISIBLE void scalable_free(void* ptr) {
    free(ptr);
}

SMASH_VISIBLE void* scalable_realloc(void* ptr, size_t size) {
    return realloc(ptr, size);
}

SMASH_VISIBLE void* scalable_calloc(size_t nobj, size_t size) {
    return calloc(nobj, size);
}

// Aligned allocation functions
SMASH_VISIBLE int scalable_posix_memalign(void** memptr, size_t alignment, size_t size) {
    return posix_memalign(memptr, alignment, size);
}

SMASH_VISIBLE void* scalable_aligned_malloc(size_t size, size_t alignment) {
    void* ptr = nullptr;
    if (posix_memalign(&ptr, alignment, size) != 0) {
        return nullptr;
    }
    return ptr;
}

SMASH_VISIBLE void scalable_aligned_free(void* ptr) {
    free(ptr);
}

SMASH_VISIBLE void* scalable_aligned_realloc(void* ptr, size_t size, size_t alignment) {
    // TBB's aligned_realloc: allocate new, copy, free old
    if (!ptr) {
        return scalable_aligned_malloc(size, alignment);
    }
    if (size == 0) {
        free(ptr);
        return nullptr;
    }
    void* new_ptr = nullptr;
    if (posix_memalign(&new_ptr, alignment, size) != 0) {
        return nullptr;
    }
    // We don't know the old size, so we copy `size` bytes (safe if growing)
    // For shrinking, this may read past end but won't write past new allocation
    size_t old_size = malloc_usable_size(ptr);
    size_t copy_size = (old_size < size) ? old_size : size;
    memcpy(new_ptr, ptr, copy_size);
    free(ptr);
    return new_ptr;
}

// Size query
SMASH_VISIBLE size_t scalable_msize(void* ptr) {
    if (!ptr) return 0;
    return malloc_usable_size(ptr);
}

// TBB memory pool allocation (route to regular malloc)
// These are used by TBB's memory_pool<scalable_allocator>
SMASH_VISIBLE void* pool_malloc(void* /*pool*/, size_t size) {
    return malloc(size);
}

SMASH_VISIBLE void pool_free(void* /*pool*/, void* ptr) {
    free(ptr);
}

SMASH_VISIBLE void* pool_realloc(void* /*pool*/, void* ptr, size_t size) {
    return realloc(ptr, size);
}

SMASH_VISIBLE void* pool_aligned_malloc(void* /*pool*/, size_t size, size_t alignment) {
    return scalable_aligned_malloc(size, alignment);
}

SMASH_VISIBLE void pool_aligned_free(void* /*pool*/, void* ptr) {
    free(ptr);
}

}  // extern "C"

// Report TBB interposition stats at process exit
namespace {
void reportTbbStatsAtExit() {
    uint64_t sc_count = g_scalable_malloc_count.load(std::memory_order_relaxed);
    uint64_t sc_bytes = g_scalable_malloc_bytes.load(std::memory_order_relaxed);
    if (sc_count > 0 && std::getenv("SMASH_STATS")) {
        // Use safe_snprintf + write(2): glibc's fprintf can call malloc
        // for stream lock acquisition, and atexit can fire after the
        // allocator is in a quiescing state where that recurses.
        char buf[160];
        int n = smash::safe_snprintf(buf, sizeof(buf),
            "[smash tbb] scalable_malloc: %lu calls, %.1f MB\n",
            (unsigned long)sc_count, sc_bytes / (1024.0 * 1024.0));
        if (n > 0) (void)!::write(2, buf, (size_t)n);
    }
}

struct TbbStatsRegistrar {
    TbbStatsRegistrar() { std::atexit(reportTbbStatsAtExit); }
} g_tbb_stats_registrar;
}  // namespace

// TBB cache_aligned_{allocate,deallocate} interposition: REMOVED.
// libtbb dispatches its allocate/deallocate through internal function-pointer
// handlers (`cache_aligned_allocate_handler`, `cache_aligned_deallocate_handler`).
// LD_PRELOAD interposes the public symbols only, which catches cross-DSO calls
// (libwalrus -> libtbb) but NOT libtbb-internal calls (which jump directly
// through the handler pointers in libtbb's data segment). The resulting
// allocate/deallocate-via-different-allocator mismatch corrupts the heap with
// "double free or corruption (out)" / "munmap_chunk(): invalid pointer" inside
// neuron-cc's Tensorizer subprocesses. Letting TBB manage its own cache-aligned
// allocations is the only safe option — we lose visibility into a few hundred
// MB of TBB-internal memory but gain correctness.

extern "C" {

// ── execve / execvp ─────────────────────────────────────────────────────────
//
// execve() doesn't return on success, so we can't use the EFAULT-retry
// pattern. Instead we proactively warm every smash-managed page that
// could appear in argv/envp strings, then call the real syscall.
//
// Production trigger: subprocess.run(['/path/to/binary', ...]) from
// Python under full smash, where the path string ended up in a
// smash-managed page that got compressed before subprocess fork+exec.
// Without this wrapper, execve returns EFAULT and Python surfaces it
// as `[Errno 14] Bad address: '<path>'`.

static inline void warm_str(const char* s) {
    auto* vm = smash::g_smash_vm_region;
    if (!vm || !s) return;
    if (!vm->contains(reinterpret_cast<uintptr_t>(s))) return;
    // Walk the string until NUL, touching each page.
    smash::vm::warmPages(s, strlen(s) + 1, vm);
}

static inline void warm_argv(char* const* arr) {
    if (!arr) return;
    auto* vm = smash::g_smash_vm_region;
    if (!vm) return;
    // Touch the array itself (pointers).
    size_t n = 0;
    while (arr[n]) ++n;
    if (n > 0) {
        smash::vm::warmPages(arr, (n + 1) * sizeof(char*), vm);
    }
    for (size_t i = 0; i < n; ++i) warm_str(arr[i]);
}

SMASH_VISIBLE int execve(const char* pathname, char* const argv[], char* const envp[]) {
    using fn_t = int(*)(const char*, char* const[], char* const[]);
    SMASH_LAZY_RESOLVE(fn_t, execve);
    warm_str(pathname);
    warm_argv(argv);
    warm_argv(envp);
    if (std::getenv("SMASH_TRACE_EXEC")) {
        const char* msg = "[smash exec] execve called\n";
        (void)!write(2, msg, strlen(msg));
    }
    if (!real_execve) return syscall(SYS_execve, pathname, argv, envp);
    return real_execve(pathname, argv, envp);
}

SMASH_VISIBLE int execv(const char* pathname, char* const argv[]) {
    using fn_t = int(*)(const char*, char* const[]);
    SMASH_LAZY_RESOLVE(fn_t, execv);
    if (std::getenv("SMASH_TRACE_EXEC")) {
        const char* msg = "[smash exec] execv called\n";
        (void)!write(2, msg, strlen(msg));
    }
    warm_str(pathname);
    warm_argv(argv);
    if (!real_execv) {
        extern char **environ;
        return syscall(SYS_execve, pathname, argv, environ);
    }
    return real_execv(pathname, argv);
}

SMASH_VISIBLE int execvp(const char* file, char* const argv[]) {
    using fn_t = int(*)(const char*, char* const[]);
    SMASH_LAZY_RESOLVE(fn_t, execvp);
    warm_str(file);
    warm_argv(argv);
    if (!real_execvp) return -1;  // PATH search needs libc
    return real_execvp(file, argv);
}

SMASH_VISIBLE int execvpe(const char* file, char* const argv[], char* const envp[]) {
    using fn_t = int(*)(const char*, char* const[], char* const[]);
    SMASH_LAZY_RESOLVE(fn_t, execvpe);
    warm_str(file);
    warm_argv(argv);
    warm_argv(envp);
    if (!real_execvpe) return -1;
    return real_execvpe(file, argv, envp);
}

// posix_spawn / posix_spawnp — Python's subprocess uses these by default
// since 3.8 when close_fds is True. Same warming pattern as execve.
struct posix_spawn_file_actions_t_opaque;
struct posix_spawnattr_t_opaque;

SMASH_VISIBLE int posix_spawn(pid_t* pid, const char* path,
                               const void* file_actions,
                               const void* attrp,
                               char* const argv[], char* const envp[]) {
    using fn_t = int(*)(pid_t*, const char*, const void*, const void*,
                        char* const[], char* const[]);
    SMASH_LAZY_RESOLVE(fn_t, posix_spawn);
    warm_str(path);
    warm_argv(argv);
    warm_argv(envp);
    if (!real_posix_spawn) return ENOSYS;
    return real_posix_spawn(pid, path, file_actions, attrp, argv, envp);
}

SMASH_VISIBLE int posix_spawnp(pid_t* pid, const char* file,
                                const void* file_actions,
                                const void* attrp,
                                char* const argv[], char* const envp[]) {
    using fn_t = int(*)(pid_t*, const char*, const void*, const void*,
                        char* const[], char* const[]);
    SMASH_LAZY_RESOLVE(fn_t, posix_spawnp);
    warm_str(file);
    warm_argv(argv);
    warm_argv(envp);
    if (!real_posix_spawnp) return ENOSYS;
    return real_posix_spawnp(pid, file, file_actions, attrp, argv, envp);
}

} // extern "C"

// ─── C++ OPERATOR NEW/DELETE VERSIONED ALIASES ─────────────────────────────────
// libBIR.so and other walrus libraries request operator new/delete with
// GLIBCXX_3.4 and CXXABI_1.3.x symbol versions, not GLIBC_2.2.5.
// Without these versioned aliases, LD_PRELOAD fails to capture C++ allocations,
// and walrus's statically-linked tcmalloc wins the symbol resolution.
// Result: allocations through smash, deallocations through tcmalloc → crash.
//
// We create aliases for the GLIBCXX/CXXABI-versioned symbols that call
// alloc8's xxmalloc/xxfree directly (NOT through the operator symbols, which
// would create infinite recursion via PLT).

// External declarations for alloc8's core allocation functions
extern "C" {
extern void* xxmalloc(size_t);
extern void  xxfree(void*);
extern void  xxfree_sized(void*, size_t);
extern void  xxfree_aligned_sized(void*, size_t, size_t);
extern void* xxmemalign(size_t, size_t);
}

// Aliased wrapper functions for GLIBCXX_3.4 versions
// These directly call xxmalloc/xxfree, matching alloc8's operator new/delete
extern "C" {
SMASH_VISIBLE void* _Znwm_GLIBCXX_3_4(size_t sz) {
    void* ptr = xxmalloc(sz);
    if (!ptr) throw std::bad_alloc();
    return ptr;
}
SMASH_VISIBLE void* _Znam_GLIBCXX_3_4(size_t sz) {
    void* ptr = xxmalloc(sz);
    if (!ptr) throw std::bad_alloc();
    return ptr;
}
SMASH_VISIBLE void  _ZdlPv_GLIBCXX_3_4(void* p)  { if (p) xxfree(p); }
SMASH_VISIBLE void  _ZdaPv_GLIBCXX_3_4(void* p)  { if (p) xxfree(p); }
SMASH_VISIBLE void* _ZnwmRKSt9nothrow_t_GLIBCXX_3_4(size_t sz, const void*)
    { return xxmalloc(sz); }
SMASH_VISIBLE void* _ZnamRKSt9nothrow_t_GLIBCXX_3_4(size_t sz, const void*)
    { return xxmalloc(sz); }
SMASH_VISIBLE void  _ZdlPvRKSt9nothrow_t_GLIBCXX_3_4(void* p, const void*)
    { if (p) xxfree(p); }
SMASH_VISIBLE void  _ZdaPvRKSt9nothrow_t_GLIBCXX_3_4(void* p, const void*)
    { if (p) xxfree(p); }
}

// CXXABI_1.3.9 versions (sized delete)
extern "C" {
SMASH_VISIBLE void _ZdlPvm_CXXABI_1_3_9(void* p, size_t sz)  { if (p) xxfree_sized(p, sz); }
SMASH_VISIBLE void _ZdaPvm_CXXABI_1_3_9(void* p, size_t sz)  { if (p) xxfree_sized(p, sz); }
}

// CXXABI_1.3.11 versions (aligned)
extern "C" {
SMASH_VISIBLE void* _ZnwmSt11align_val_t_CXXABI_1_3_11(size_t sz, size_t al) {
    void* ptr = xxmemalign(al, sz);
    if (!ptr) throw std::bad_alloc();
    return ptr;
}
SMASH_VISIBLE void* _ZnamSt11align_val_t_CXXABI_1_3_11(size_t sz, size_t al) {
    void* ptr = xxmemalign(al, sz);
    if (!ptr) throw std::bad_alloc();
    return ptr;
}
SMASH_VISIBLE void  _ZdlPvSt11align_val_t_CXXABI_1_3_11(void* p, size_t)
    { if (p) xxfree(p); }
SMASH_VISIBLE void  _ZdaPvSt11align_val_t_CXXABI_1_3_11(void* p, size_t)
    { if (p) xxfree(p); }
SMASH_VISIBLE void  _ZdlPvmSt11align_val_t_CXXABI_1_3_11(void* p, size_t sz, size_t al)
    { if (p) xxfree_aligned_sized(p, al, sz); }
SMASH_VISIBLE void  _ZdaPvmSt11align_val_t_CXXABI_1_3_11(void* p, size_t sz, size_t al)
    { if (p) xxfree_aligned_sized(p, al, sz); }
SMASH_VISIBLE void* _ZnwmSt11align_val_tRKSt9nothrow_t_CXXABI_1_3_11(
        size_t sz, size_t al, const void*)
    { return xxmemalign(al, sz); }
SMASH_VISIBLE void* _ZnamSt11align_val_tRKSt9nothrow_t_CXXABI_1_3_11(
        size_t sz, size_t al, const void*)
    { return xxmemalign(al, sz); }
}

// .symver directives create versioned symbol aliases.
// Use single @ for non-default versions (so we don't conflict with alloc8's
// GLIBC_2.2.5 versions). The dynamic linker will still match these when
// a caller requests the specific version (e.g., _Znwm@GLIBCXX_3.4).
__asm__(".symver _Znwm_GLIBCXX_3_4,_Znwm@GLIBCXX_3.4");
__asm__(".symver _Znam_GLIBCXX_3_4,_Znam@GLIBCXX_3.4");
__asm__(".symver _ZdlPv_GLIBCXX_3_4,_ZdlPv@GLIBCXX_3.4");
__asm__(".symver _ZdaPv_GLIBCXX_3_4,_ZdaPv@GLIBCXX_3.4");
__asm__(".symver _ZnwmRKSt9nothrow_t_GLIBCXX_3_4,_ZnwmRKSt9nothrow_t@GLIBCXX_3.4");
__asm__(".symver _ZnamRKSt9nothrow_t_GLIBCXX_3_4,_ZnamRKSt9nothrow_t@GLIBCXX_3.4");
__asm__(".symver _ZdlPvRKSt9nothrow_t_GLIBCXX_3_4,_ZdlPvRKSt9nothrow_t@GLIBCXX_3.4");
__asm__(".symver _ZdaPvRKSt9nothrow_t_GLIBCXX_3_4,_ZdaPvRKSt9nothrow_t@GLIBCXX_3.4");

__asm__(".symver _ZdlPvm_CXXABI_1_3_9,_ZdlPvm@CXXABI_1.3.9");
__asm__(".symver _ZdaPvm_CXXABI_1_3_9,_ZdaPvm@CXXABI_1.3.9");

__asm__(".symver _ZnwmSt11align_val_t_CXXABI_1_3_11,_ZnwmSt11align_val_t@CXXABI_1.3.11");
__asm__(".symver _ZnamSt11align_val_t_CXXABI_1_3_11,_ZnamSt11align_val_t@CXXABI_1.3.11");
__asm__(".symver _ZdlPvSt11align_val_t_CXXABI_1_3_11,_ZdlPvSt11align_val_t@CXXABI_1.3.11");
__asm__(".symver _ZdaPvSt11align_val_t_CXXABI_1_3_11,_ZdaPvSt11align_val_t@CXXABI_1.3.11");
__asm__(".symver _ZdlPvmSt11align_val_t_CXXABI_1_3_11,_ZdlPvmSt11align_val_t@CXXABI_1.3.11");
__asm__(".symver _ZdaPvmSt11align_val_t_CXXABI_1_3_11,_ZdaPvmSt11align_val_t@CXXABI_1.3.11");
__asm__(".symver _ZnwmSt11align_val_tRKSt9nothrow_t_CXXABI_1_3_11,_ZnwmSt11align_val_tRKSt9nothrow_t@CXXABI_1.3.11");
__asm__(".symver _ZnamSt11align_val_tRKSt9nothrow_t_CXXABI_1_3_11,_ZnamSt11align_val_tRKSt9nothrow_t@CXXABI_1.3.11");

#endif // __linux__
