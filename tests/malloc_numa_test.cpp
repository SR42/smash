// malloc_numa_test.cpp - Contract for the exported malloc_numa(node, size).
//
// malloc_numa serves page-granular blocks from a per-node arena inside smash's
// heap region, whose chunks are bound MPOL_BIND | MPOL_F_STATIC_NODES to the
// node. Under LD_PRELOAD, in both heap modes, for every online node this
// checks that a block is:
//   - bound by that policy: get_mempolicy(MPOL_F_ADDR) reports MPOL_BIND with
//     the static-nodes flag and a nodemask of exactly {node};
//   - physically on that node once written: move_pages(nodes=NULL) reports the
//     node for every page. The writer is first pinned to CPUs outside the node
//     (when there are any), so first-touch locality cannot put the pages there
//     by accident; only the binding can;
//   - an ordinary heap block: smash-owned, page-aligned, malloc_usable_size
//     >= size, realloc keeps its bytes, free releases it;
//   - compressed when cold, and restored onto its node. Its pages leave RSS
//     (smash drops a page only after storing its compressed copy), then read
//     back byte-exact from off-node CPUs with every page on the node. Needs
//     SMASH_COLD_TIMEOUT_SEC=1, which the ctest sets.
// It also checks that an offline or out-of-range node fails with EINVAL, that
// freed blocks go back to their node's arena, and that ordinary malloc memory
// carries no policy, so a binding never leaks into the heap.
//
// Skips when the kernel or a seccomp filter refuses mbind (ENOSYS/EPERM).

#include <cerrno>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>

#if defined(__linux__)
#include <malloc.h>
#include <sys/mman.h>
#include "numa_test_util.h"
#endif

using numa_fn = void* (*)(uint32_t, size_t);
using arena_fn = int (*)(const void*);

static constexpr size_t kMiB = 1024 * 1024;
static int g_failures = 0;

__attribute__((format(printf, 1, 2)))
static void fail(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    fputs("FAIL: ", stderr);
    vfprintf(stderr, fmt, ap);
    fputc('\n', stderr);
    va_end(ap);
    ++g_failures;
}

#if defined(__linux__)

using numa_test::isBoundTo;
using numa_test::pagesOffNode;
using numa_test::pinOutside;
using numa_test::Policy;

static unsigned char pattern(int node, size_t size, size_t i) {
    return static_cast<unsigned char>(i * 31 + size * 7 + static_cast<size_t>(node) + 1);
}

static void checkBlock(numa_fn numa, arena_fn arena_for, int node, size_t size) {
    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    errno = 0;
    auto* p = static_cast<unsigned char*>(numa(static_cast<uint32_t>(node), size));
    if (!p) {
        fail("malloc_numa(%d, %zu) returned NULL (errno=%d %s)",
             node, size, errno, strerror(errno));
        return;
    }
    if (arena_for(p) < 0)
        fail("malloc_numa(%d, %zu)=%p is not smash-owned, so free() would hand "
             "it to the system allocator", node, size, static_cast<void*>(p));
    if (reinterpret_cast<uintptr_t>(p) % ps != 0)
        fail("malloc_numa(%d, %zu)=%p is not page-aligned",
             node, size, static_cast<void*>(p));
    if (malloc_usable_size(p) < size)
        fail("malloc_numa(%d, %zu): malloc_usable_size=%zu",
             node, size, malloc_usable_size(p));

    Policy pol;
    if (!isBoundTo(p, node, &pol))
        fail("malloc_numa(%d, %zu): policy mode=%#x mask[0]=%#lx, want "
             "MPOL_BIND|MPOL_F_STATIC_NODES (%#x) on {%d}",
             node, size, pol.mode, pol.mask[0], MPOL_BIND | MPOL_F_STATIC_NODES, node);

    for (size_t i = 0; i < size; ++i) p[i] = pattern(node, size, i);
    int first_status = 0;
    if (size_t off = pagesOffNode(p, size, node, &first_status))
        fail("malloc_numa(%d, %zu): %zu of %zu pages not on node %d "
             "(first status=%d)", node, size, off, (size + ps - 1) / ps,
             node, first_status);
    for (size_t i = 0; i < size; ++i) {
        if (p[i] != pattern(node, size, i)) {
            fail("malloc_numa(%d, %zu): byte %zu changed", node, size, i);
            break;
        }
    }

    // realloc copies into an unbound block; the bytes must survive, and the
    // realloc frees the NUMA block through smash's normal free path.
    auto* q = static_cast<unsigned char*>(realloc(p, size * 2));
    if (!q) {
        fail("realloc(malloc_numa(%d, %zu), %zu) returned NULL", node, size, size * 2);
        free(p);
        return;
    }
    for (size_t i = 0; i < size; ++i) {
        if (q[i] != pattern(node, size, i)) {
            fail("realloc(malloc_numa(%d, %zu)): byte %zu lost", node, size, i);
            break;
        }
    }
    free(q);
}

static void checkNoPolicy(const char* what, void* p) {
    Policy pol;
    if (!numa_test::queryPolicy(p, &pol))
        fail("get_mempolicy(%s %p) failed: errno=%d", what, p, errno);
    else if (pol.mode != MPOL_DEFAULT)
        fail("%s %p carries mempolicy mode=%#x: a NUMA binding leaked into "
             "ordinary heap memory", what, p, pol.mode);
}

// The reason node arenas live in smash's heap region: cold malloc_numa pages
// are compressed, and a page decompressed on access comes back on its node.
static void checkCompressedAndRestored(numa_fn numa, const std::vector<int>& nodes,
                                       const cpu_set_t* allowed) {
    constexpr size_t kBlock = 16 * kMiB;
    const size_t pages = kBlock / static_cast<size_t>(sysconf(_SC_PAGESIZE));

    std::vector<unsigned char*> blocks;
    for (int node : nodes) {
        auto* p = static_cast<unsigned char*>(numa(static_cast<uint32_t>(node), kBlock));
        if (!p) fail("malloc_numa(%d, 16 MiB) returned NULL (errno=%d)", node, errno);
        if (p) {
            const bool pinned = allowed && pinOutside(node, *allowed);
            for (size_t i = 0; i < kBlock; ++i) p[i] = pattern(node, kBlock, i);
            if (pinned) sched_setaffinity(0, sizeof(*allowed), allowed);
        }
        blocks.push_back(p);
    }

    // Wait for at least half of every block to leave RSS.
    std::vector<unsigned char> vec(pages);
    std::vector<size_t> evicted(nodes.size(), 0);
    int waited_ms = 0;
    for (; waited_ms <= 30000; waited_ms += 250) {
        bool all = true;
        for (size_t k = 0; k < blocks.size(); ++k) {
            if (!blocks[k]) continue;
            evicted[k] = 0;
            if (mincore(blocks[k], kBlock, vec.data()) == 0)
                for (unsigned char v : vec) evicted[k] += !(v & 1);
            if (evicted[k] < pages / 2) all = false;
        }
        if (all) break;
        usleep(250 * 1000);
    }

    for (size_t k = 0; k < blocks.size(); ++k) {
        unsigned char* p = blocks[k];
        if (!p) continue;
        const int node = nodes[k];
        if (evicted[k] < pages / 2)
            fail("node %d: only %zu of %zu pages of a cold malloc_numa block left "
                 "RSS within 30 s, so the block is not being compressed",
                 node, evicted[k], pages);

        // Reading faults each compressed page back in. With the reader off the
        // node, only the binding can place the restored pages on it.
        const bool pinned = allowed && pinOutside(node, *allowed);
        size_t differ = 0;
        for (size_t i = 0; i < kBlock; ++i) differ += p[i] != pattern(node, kBlock, i);
        if (pinned) sched_setaffinity(0, sizeof(*allowed), allowed);
        if (differ)
            fail("node %d: %zu bytes differ after decompression", node, differ);

        int first_status = 0;
        if (size_t off = pagesOffNode(p, kBlock, node, &first_status))
            fail("node %d: %zu of %zu pages restored off-node (first status=%d)",
                 node, off, pages, first_status);
        Policy pol;
        if (!isBoundTo(p, node, &pol))
            fail("node %d: policy after decompression mode=%#x mask[0]=%#lx",
                 node, pol.mode, pol.mask[0]);
        printf("node %d: %zu/%zu pages compressed out of RSS, restored on node "
               "(reader %s)\n", node, evicted[k], pages,
               pinned ? "pinned off-node" : "on-node");
        free(p);
    }
    printf("compression wait: %d ms\n", waited_ms);
}

static void runLinux(numa_fn numa, arena_fn arena_for) {
    const std::vector<int> nodes = numa_test::onlineNodes();

    errno = 0;
    void* probe = numa(static_cast<uint32_t>(nodes.front()), 1);
    if (!probe && (errno == ENOSYS || errno == EPERM)) {
        printf("SKIP: mbind unavailable (errno=%d %s)\n", errno, strerror(errno));
        exit(0);
    }
    free(probe);

    cpu_set_t allowed;
    CPU_ZERO(&allowed);
    const cpu_set_t* pin =
        sched_getaffinity(0, sizeof(allowed), &allowed) == 0 ? &allowed : nullptr;

    const size_t kSizes[] = {1, 4096, 16 * 1024 + 1, 3 * kMiB + 123, 64 * kMiB};
    for (int node : nodes) {
        const bool pinned = pin && pinOutside(node, allowed);
        for (size_t size : kSizes) checkBlock(numa, arena_for, node, size);
        if (pinned) sched_setaffinity(0, sizeof(allowed), &allowed);
        printf("node %d: %zu sizes checked, writer %s\n", node, std::size(kSizes),
               pinned ? "pinned off-node" : "on-node (no off-node CPUs)");
    }

    // Offline and out-of-range nodes.
    std::vector<uint32_t> bad = {1024, UINT32_MAX};
    const uint32_t past = static_cast<uint32_t>(nodes.back()) + 1;
    if (past < 1024) bad.push_back(past);
    for (uint32_t n : bad) {
        errno = 0;
        void* p = numa(n, 4096);
        if (p || errno != EINVAL) {
            fail("malloc_numa(%u, 4096) = %p errno=%d, want NULL with EINVAL",
                 n, p, errno);
            free(p);
        }
    }

    // free() returns a block's pages to its node arena: once the decommit
    // thread has run, the next same-size request on that node gets them back.
    for (int node : nodes) {
        void* a = numa(static_cast<uint32_t>(node), kMiB);
        if (!a) {
            fail("malloc_numa(%d, 1 MiB) returned NULL", node);
            continue;
        }
        free(a);
        int reused = 0;
        for (int i = 0; i < 16; ++i) {
            usleep(20 * 1000);
            void* b = numa(static_cast<uint32_t>(node), kMiB);
            reused += (b == a);
            free(b);
        }
        if (reused < 16)
            fail("node %d: only %d of 16 frees came back at the same address, so "
                 "freed pages are not returning to the node arena", node, reused);
    }

    // Ordinary allocations, including one of a just-freed NUMA block's size,
    // must carry no policy.
    free(numa(static_cast<uint32_t>(nodes.back()), 8 * kMiB));
    void* large = malloc(8 * kMiB);
    void* small = malloc(256);
    if (!large || !small) {
        fail("plain malloc returned NULL");
    } else {
        memset(large, 0x5a, 8 * kMiB);
        memset(small, 0x5a, 256);
        checkNoPolicy("malloc(8 MiB)", large);
        checkNoPolicy("malloc(256)", small);
    }
    free(large);
    free(small);

    checkCompressedAndRestored(numa, nodes, pin);

    printf("checked %zu node(s) and %zu bad node ids\n", nodes.size(), bad.size());
}

#else  // !__linux__

// One memory node: node 0 is a plain heap block, any other node is EINVAL.
static void runSingleNode(numa_fn numa, arena_fn arena_for) {
    auto* p = static_cast<unsigned char*>(numa(0, kMiB));
    if (!p) {
        fail("malloc_numa(0, 1 MiB) returned NULL (errno=%d)", errno);
    } else {
        memset(p, 0x5a, kMiB);
        if (arena_for(p) < 0) fail("malloc_numa(0, 1 MiB) is not smash-owned");
        free(p);
    }
    errno = 0;
    void* q = numa(1, 4096);
    if (q || errno != EINVAL) {
        fail("malloc_numa(1, 4096) = %p errno=%d, want NULL with EINVAL", q, errno);
        free(q);
    }
}

#endif

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);

    auto numa = reinterpret_cast<numa_fn>(dlsym(RTLD_DEFAULT, "malloc_numa"));
    auto arena_for = reinterpret_cast<arena_fn>(dlsym(RTLD_DEFAULT, "smash_arena_for"));
    if (!numa || !arena_for) {
        printf("SKIP: malloc_numa not found (not running under libsmash)\n");
        return 0;
    }

#if defined(__linux__)
    runLinux(numa, arena_for);
#else
    runSingleNode(numa, arena_for);
#endif

    if (g_failures) {
        fprintf(stderr, "FAIL: %d check(s) failed\n", g_failures);
        return 1;
    }
    printf("PASS: malloc_numa contract holds\n");
    return 0;
}
