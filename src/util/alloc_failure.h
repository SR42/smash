#pragma once
// alloc_failure.h - say why a full-mode allocation is about to return nullptr.
//
// smash's operator new turns a nullptr from xxmalloc into std::bad_alloc, and
// a multithreaded application then typically dies with
//   terminate called after throwing an instance of 'std::bad_alloc'
//   terminate called recursively
// with nothing saying which allocation failed or why. This prints one line
// per failure (rate-limited) naming the path that failed, the request, errno,
// how full the arena is, and the process's VMA count against
// vm.max_map_count — enough to tell arena exhaustion from an mmap/VMA limit
// from something else.
//
// It runs inside a failing malloc, so it must not allocate: raw syscalls, a
// static buffer, safe_snprintf, write(2). (The compressor's own VMA helpers
// use fopen, which mallocs.)
#include "safe_printf.h"
#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <unistd.h>
#if defined(__linux__)
#include <sys/syscall.h>
#endif

namespace smash {

// Lines in /proc/self/maps, or -1. No allocation.
inline long vmaCountNoAlloc() {
#if defined(__linux__)
    static char buf[1 << 16];  // shared static: only ever read by the reporter
    const int fd = static_cast<int>(syscall(SYS_openat, AT_FDCWD, "/proc/self/maps",
                                            O_RDONLY | O_CLOEXEC));
    if (fd < 0) return -1;
    long n = 0, r;
    while ((r = syscall(SYS_read, fd, buf, sizeof(buf))) > 0)
        for (long i = 0; i < r; ++i) n += buf[i] == '\n';
    syscall(SYS_close, fd);
    return n;
#else
    return -1;
#endif
}

// vm.max_map_count, or -1. No allocation.
inline long vmaCapNoAlloc() {
#if defined(__linux__)
    char b[32];
    const int fd = static_cast<int>(syscall(SYS_openat, AT_FDCWD, "/proc/sys/vm/max_map_count",
                                            O_RDONLY | O_CLOEXEC));
    if (fd < 0) return -1;
    const long r = syscall(SYS_read, fd, b, sizeof(b) - 1);
    syscall(SYS_close, fd);
    if (r <= 0) return -1;
    long v = 0;
    for (long i = 0; i < r && b[i] >= '0' && b[i] <= '9'; ++i) v = v * 10 + (b[i] - '0');
    return v;
#else
    return -1;
#endif
}

// `where` names the failing path; arena_used/arena_total are pages of the
// contiguous VmRegion. errno is preserved for the caller.
[[gnu::cold, gnu::noinline]]
inline void reportAllocFailure(const char* where, size_t size, size_t alignment,
                               size_t arena_used, size_t arena_total) {
    const int err = errno;
    static std::atomic<uint64_t> seen{0};
    const uint64_t n = seen.fetch_add(1, std::memory_order_relaxed) + 1;
    if (n <= 8 || (n & 0xFFF) == 0) {
        char line[320];
        const int len = smash::safe_snprintf(line, sizeof(line),
            "[smash alloc-fail] %s: size=%zu align=%zu errno=%d arena=%zu/%zu pages "
            "vmas=%ld/%ld (failure %lu; the caller gets nullptr, operator new throws "
            "std::bad_alloc)\n",
            where, size, alignment, err, arena_used, arena_total,
            vmaCountNoAlloc(), vmaCapNoAlloc(), static_cast<unsigned long>(n));
        if (len > 0) (void)!::write(2, line, static_cast<size_t>(len));
    }
    errno = err;
}

}  // namespace smash
