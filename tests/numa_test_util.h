// numa_test_util.h - NUMA helpers shared by malloc_numa_test and
// test_numa_arena: node lists, pinning a thread off a node, and reading a
// range's policy and physical placement back from the kernel.
#pragma once

#if defined(__linux__)

#include <cerrno>
#include <climits>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <vector>

#include <linux/mempolicy.h>
#include <sched.h>
#include <sys/syscall.h>
#include <unistd.h>

namespace numa_test {

// Parses a sysfs list such as "0-3,6" ("" for a CPU-less node).
inline std::vector<int> readList(const char* path) {
    std::vector<int> out;
    FILE* f = fopen(path, "r");
    if (!f) return out;
    char buf[4096];
    if (fgets(buf, sizeof(buf), f)) {
        for (char* s = buf; *s && *s != '\n';) {
            char* end;
            long lo = strtol(s, &end, 10);
            if (end == s) break;
            long hi = lo;
            if (*end == '-') hi = strtol(end + 1, &end, 10);
            for (long v = lo; v <= hi; ++v) out.push_back(static_cast<int>(v));
            s = (*end == ',') ? end + 1 : end;
        }
    }
    fclose(f);
    return out;
}

// Online NUMA nodes; {0} when the kernel exposes no node tree.
inline std::vector<int> onlineNodes() {
    std::vector<int> nodes = readList("/sys/devices/system/node/online");
    if (nodes.empty()) nodes.push_back(0);
    return nodes;
}

// NUMA nodes with memory; the online nodes when has_memory is missing.
inline std::vector<int> memoryNodes() {
    std::vector<int> nodes = readList("/sys/devices/system/node/has_memory");
    return nodes.empty() ? onlineNodes() : nodes;
}

// Pointers in `ptrs` whose page is not on `node`. *first_status gets the first
// offending move_pages status (a node id, or -errno for an unplaced page).
inline size_t pointersOffNode(std::vector<void*> ptrs, int node, int* first_status) {
    if (ptrs.empty()) return 0;
    std::vector<int> status(ptrs.size(), INT_MIN);
    if (syscall(SYS_move_pages, 0, ptrs.size(), ptrs.data(), nullptr, status.data(), 0) != 0) {
        *first_status = -errno;
        return ptrs.size();
    }
    size_t off = 0;
    for (int s : status)
        if (s != node && off++ == 0) *first_status = s;
    return off;
}

// Pins the thread to every allowed CPU not on `node`. False (and no pin) when
// all of them are on `node`, e.g. a single-node host.
inline bool pinOutside(int node, const cpu_set_t& allowed) {
    char path[128];
    snprintf(path, sizeof(path), "/sys/devices/system/node/node%d/cpulist", node);
    cpu_set_t set = allowed;
    for (int cpu : readList(path)) CPU_CLR(cpu, &set);
    if (CPU_COUNT(&set) == 0) return false;
    return sched_setaffinity(0, sizeof(set), &set) == 0;
}

inline constexpr unsigned long kMaskBits = 1024;

struct Policy {
    int mode;
    unsigned long mask[kMaskBits / (8 * sizeof(unsigned long))];
};

inline bool queryPolicy(void* addr, Policy* p) {
    memset(p, 0, sizeof(*p));
    return syscall(SYS_get_mempolicy, &p->mode, p->mask, kMaskBits + 1,
                   addr, MPOL_F_ADDR) == 0;
}

// True if addr's policy is exactly MPOL_BIND | MPOL_F_STATIC_NODES on {node}.
// *p holds what the kernel reported either way.
inline bool isBoundTo(void* addr, int node, Policy* p) {
    if (!queryPolicy(addr, p)) return false;
    if (p->mode != (MPOL_BIND | MPOL_F_STATIC_NODES)) return false;
    constexpr int kBits = 8 * sizeof(unsigned long);
    for (size_t w = 0; w < std::size(p->mask); ++w) {
        unsigned long want = (static_cast<size_t>(node / kBits) == w)
                                 ? (1UL << (node % kBits)) : 0;
        if (p->mask[w] != want) return false;
    }
    return true;
}

// Pages of [p, p+bytes) not on `node`. *first_status gets the first offending
// move_pages status (a node id, or -errno for a page with no placement).
inline size_t pagesOffNode(void* p, size_t bytes, int node, int* first_status) {
    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    const size_t n = (bytes + ps - 1) / ps;
    std::vector<void*> pages(n);
    std::vector<int> status(n, INT_MIN);
    for (size_t i = 0; i < n; ++i) pages[i] = static_cast<char*>(p) + i * ps;
    if (syscall(SYS_move_pages, 0, n, pages.data(), nullptr, status.data(), 0) != 0) {
        *first_status = -errno;
        return n;
    }
    size_t off = 0;
    for (int s : status)
        if (s != node && off++ == 0) *first_status = s;
    return off;
}

}  // namespace numa_test

#endif
