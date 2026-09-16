// smash/src/vm/node_memory.h - Memory bound to one NUMA node, and the node
// topology that the per-node compressor groups are built from. Linux only.
#pragma once

#if defined(__linux__)

#include "smash/config.h"
#include "platform_mem.h"
#include "../util/spinlock.h"

#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <sched.h>
#include <sys/syscall.h>
#include <unistd.h>

namespace smash::vm {

// Never-freeing bump allocator whose pages are bound (MPOL_BIND) to one NUMA
// node before first touch, so they land on that node whichever thread writes
// them. Backs a node compressor group's worker state, scratch buffers, codec
// contexts and CompressStore object.
class NodeBumpAlloc {
    static constexpr size_t kRegionBytes = 4 * 1024 * 1024;
    uint32_t node_;
    Spinlock lock_;
    char* cur_ = nullptr;
    size_t left_ = 0;

public:
    explicit NodeBumpAlloc(uint32_t node) : node_(node) {}

    // `align` must be a power of two no larger than kPageSize.
    void* allocate(size_t size, size_t align = 16) {
        LockGuard guard(lock_);
        size_t pad = (align - (reinterpret_cast<uintptr_t>(cur_) & (align - 1))) & (align - 1);
        if (!cur_ || pad + size > left_) {
            size_t bytes = (size + kPageSize - 1) & ~(kPageSize - 1);
            if (bytes < kRegionBytes) bytes = kRegionBytes;
            void* m = mapPages(bytes);
            if (!m) return nullptr;
            if (!bindPagesToNode(m, bytes, node_)) {
                unmapPages(m, bytes);
                return nullptr;
            }
            cur_ = static_cast<char*>(m);
            left_ = bytes;
            pad = 0;  // a fresh mapping is page-aligned
        }
        char* p = cur_ + pad;
        cur_ = p + size;
        left_ -= pad + size;
        return p;
    }

    // ZSTD_customMem hooks (opaque = NodeBumpAlloc*). Codec contexts live for
    // the life of the process, so nothing is freed.
    static void* zstdAlloc(void* opaque, size_t size) {
        return static_cast<NodeBumpAlloc*>(opaque)->allocate(size, 16);
    }
    static void zstdFree(void*, void*) {}
};

// Reads a sysfs id list such as "0-3,8" into a bitmap of `nbits` bits.
// Raw syscalls and no allocation. False if the file cannot be read.
inline bool readSysfsIdList(const char* path, uint64_t* bits, size_t nbits) {
    long fd = syscall(SYS_openat, AT_FDCWD, path, O_RDONLY | O_CLOEXEC);
    if (fd < 0) return false;
    char buf[4096];
    long n = syscall(SYS_read, fd, buf, sizeof(buf) - 1);
    syscall(SYS_close, fd);
    if (n < 0) return false;
    buf[n] = '\0';
    for (size_t w = 0; w < (nbits + 63) / 64; ++w) bits[w] = 0;
    const char* s = buf;
    while (*s >= '0' && *s <= '9') {
        unsigned long lo = 0;
        while (*s >= '0' && *s <= '9') lo = lo * 10 + static_cast<unsigned long>(*s++ - '0');
        unsigned long hi = lo;
        if (*s == '-') {
            ++s;
            hi = 0;
            while (*s >= '0' && *s <= '9') hi = hi * 10 + static_cast<unsigned long>(*s++ - '0');
        }
        for (unsigned long v = lo; v <= hi && v < nbits; ++v) bits[v / 64] |= 1ULL << (v % 64);
        if (*s == ',') ++s;
    }
    return true;
}

// NUMA nodes with memory, as a kMaxNumaNodes-bit bitmap. Returns the count.
inline size_t memoryNodes(uint64_t* bits) {
    if (!readSysfsIdList("/sys/devices/system/node/has_memory", bits, kMaxNumaNodes) &&
        !readSysfsIdList("/sys/devices/system/node/online", bits, kMaxNumaNodes))
        return 0;
    size_t count = 0;
    for (size_t w = 0; w < kMaxNumaNodes / 64; ++w) count += __builtin_popcountll(bits[w]);
    return count;
}

// CPUs of `node` that this thread may run on. False if there are none
// (a memory-only node, or a cpuset that excludes all of them).
inline bool nodeCpus(uint32_t node, cpu_set_t* out) {
    char path[64];
    smash::safe_snprintf(path, sizeof(path), "/sys/devices/system/node/node%u/cpulist", node);
    uint64_t bits[CPU_SETSIZE / 64] = {};
    CPU_ZERO(out);
    if (!readSysfsIdList(path, bits, CPU_SETSIZE)) return false;
    cpu_set_t allowed;
    const bool have_allowed = sched_getaffinity(0, sizeof(allowed), &allowed) == 0;
    for (int c = 0; c < CPU_SETSIZE; ++c) {
        if (((bits[c / 64] >> (c % 64)) & 1) && (!have_allowed || CPU_ISSET(c, &allowed)))
            CPU_SET(c, out);
    }
    return CPU_COUNT(out) > 0;
}

// Node of each CPU, so a thread can name its node without a syscall. -1 until
// initCpuNodeTable() runs, and for CPUs no node claims.
inline int16_t g_cpu_node_table[CPU_SETSIZE];

// Build the table once. Call it from startup, never from a fault: the
// function-local static below takes a guard the first time through.
inline void initCpuNodeTable() {
    static const bool built = [] {
        for (int c = 0; c < CPU_SETSIZE; ++c) g_cpu_node_table[c] = -1;
        uint64_t nodes[kMaxNumaNodes / 64];
        if (memoryNodes(nodes) == 0) return true;
        for (uint32_t n = 0; n < kMaxNumaNodes; ++n) {
            if (!((nodes[n / 64] >> (n % 64)) & 1)) continue;
            char path[64];
            smash::safe_snprintf(path, sizeof(path),
                                 "/sys/devices/system/node/node%u/cpulist", n);
            uint64_t bits[CPU_SETSIZE / 64] = {};
            if (!readSysfsIdList(path, bits, CPU_SETSIZE)) continue;
            for (int c = 0; c < CPU_SETSIZE; ++c)
                if ((bits[c / 64] >> (c % 64)) & 1)
                    g_cpu_node_table[c] = static_cast<int16_t>(n);
        }
        return true;
    }();
    (void)built;
    }

// NUMA node this thread is running on, or -1. sched_getcpu() is a vDSO read and
// the table is immutable after init, so this is safe from the fault handler.
inline int nodeOfCurrentCpu() {
    const int cpu = sched_getcpu();
    if (cpu < 0 || cpu >= CPU_SETSIZE) return -1;
    return g_cpu_node_table[cpu];
}

}  // namespace smash::vm

#endif
