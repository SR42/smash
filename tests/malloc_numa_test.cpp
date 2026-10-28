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
// size 0 is a one-page block, that a size whose page count does not fit fails
// with ENOMEM, that freed blocks go back to their node's arena, and that
// ordinary malloc memory carries no policy, so a binding never leaks into the
// heap. With MALLOC_NUMA_EXPECT_ENOTSUP=1 it only checks that both calls fail
// with ENOTSUP (registered under SMASH_PASSTHROUGH=1 and compress-only).
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

    // Size 0 is a block of one page, as malloc(0) is a block: not NULL, a new
    // one each call, bound to the node, freeable.
    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    for (int node : nodes) {
        errno = 0;
        void* a = numa(static_cast<uint32_t>(node), 0);
        void* b = numa(static_cast<uint32_t>(node), 0);
        if (!a || !b) {
            fail("malloc_numa(%d, 0) returned NULL (errno=%d %s)", node, errno, strerror(errno));
        } else {
            if (a == b) fail("malloc_numa(%d, 0) returned %p twice", node, a);
            for (void* p : {a, b}) {
                Policy pol;
                if (arena_for(p) < 0) fail("malloc_numa(%d, 0)=%p is not smash-owned", node, p);
                if (reinterpret_cast<uintptr_t>(p) % ps != 0)
                    fail("malloc_numa(%d, 0)=%p is not page-aligned", node, p);
                if (!isBoundTo(p, node, &pol))
                    fail("malloc_numa(%d, 0): policy mode=%#x mask[0]=%#lx, want "
                         "MPOL_BIND|MPOL_F_STATIC_NODES on {%d}", node, pol.mode, pol.mask[0], node);
                *static_cast<volatile unsigned char*>(p) = 1;
            }
        }
        free(a);
        free(b);
    }

    // A size whose page count does not fit is NULL with ENOMEM, never a
    // shorter block: SIZE_MAX overflows the rounding, SIZE_MAX - ps + 2 rounds
    // to 0, and 2^32 + 1 pages is one page in a 32-bit page count.
    for (size_t size : {SIZE_MAX, SIZE_MAX - ps + 2, ((size_t{1} << 32) + 1) * ps}) {
        errno = 0;
        void* p = numa(static_cast<uint32_t>(nodes.front()), size);
        if (p || errno != ENOMEM) {
            fail("malloc_numa(%d, %zu) = %p errno=%d %s, want NULL with ENOMEM",
                 nodes.front(), size, p, errno, strerror(errno));
            free(p);
        }
    }

    // free() returns a block's pages to their node's own arena. Checked as
    // what that promises, not as "the same address comes back": frees reach
    // the arena through the node's decommit queue asynchronously, and while
    // one is still queued the allocator correctly takes the next free run of
    // the same arena. The address check failed that way 4 times in 100 root
    // runs on a 2-CPU host (the queue lagged the 20 ms sleep by several
    // rounds: each miss one block further on, the node's free pages one
    // block short per round, every block still smash-owned and bound to the
    // node, nothing carved). So, over 16 allocate/free rounds:
    //   - every block is smash-owned and bound to the node;
    //   - the node's arena carves nothing new (freed pages are reused);
    //   - once the queue has drained, the node's free pages are back at the
    //     count they had before the rounds (nothing leaked, nothing handed
    //     to the shared lists).
    using arena_info_fn = int (*)(uint32_t, size_t*, size_t*);
    auto arena_info =
        reinterpret_cast<arena_info_fn>(dlsym(RTLD_DEFAULT, "smash_numa_arena_info"));
    auto info = [&](int node, long* carved, long* free_pages) {
        size_t c = 0, f = 0;
        const bool ok = arena_info && arena_info(static_cast<uint32_t>(node), &c, &f) == 0;
        *carved = ok ? static_cast<long>(c) : -1;
        *free_pages = ok ? static_cast<long>(f) : -1;
        return ok;
    };
    // The node's free pages once its decommit queue has gone quiet: two equal
    // readings 50 ms apart, within 5 s.
    auto settledFree = [&](int node) {
        long c = 0, f = -1, prev = -2;
        for (int i = 0; i < 100 && f != prev; ++i) {
            prev = f;
            usleep(50 * 1000);
            info(node, &c, &f);
        }
        return f;
    };
    for (int node : nodes) {
        long carved0 = 0, dummy = 0;
        if (!info(node, &carved0, &dummy)) {
            fail("smash_numa_arena_info(%d) failed", node);
            continue;
        }
        const long free0 = settledFree(node);
        int bad_blocks = 0;
        for (int i = 0; i < 16; ++i) {
            usleep(20 * 1000);
            void* b = numa(static_cast<uint32_t>(node), kMiB);
            if (!b) {
                fail("malloc_numa(%d, 1 MiB) returned NULL in round %d", node, i);
                continue;
            }
            Policy bp;
            if (arena_for(b) < 0 || !isBoundTo(b, node, &bp)) ++bad_blocks;
            free(b);
        }
        long carved1 = 0;
        info(node, &carved1, &dummy);
        const long free1 = settledFree(node);
        printf("node %d: 16 rounds of 1 MiB: carved %+ld pages, free pages %ld -> %ld after "
               "the queue drained\n", node, carved1 - carved0, free0, free1);
        if (bad_blocks)
            fail("node %d: %d of 16 blocks were not smash-owned and bound to the node",
                 node, bad_blocks);
        if (carved1 != carved0)
            fail("node %d: the arena carved %ld new pages for 16 rounds of a 1 MiB block: "
                 "freed pages are not being reused", node, carved1 - carved0);
        if (free1 != free0)
            fail("node %d: %ld free pages before the rounds, %ld after the queue drained: "
                 "freed pages are not returning to the node arena", node, free0, free1);
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

// MALLOC_NUMA_EXPECT_ENOTSUP=1, run with SMASH_PASSTHROUGH=1 or
// SMASH_MODE=compress_only: neither heap can own a node block (passthrough
// answers malloc_usable_size from the system allocator, compress-only also
// frees through it), so both calls must fail with ENOTSUP instead of handing
// libc a mapping it never made. A block returned anyway is not freed: that
// free is the hazard.
static int runNotSupported(numa_fn numa) {
    using aligned_fn = void* (*)(uint32_t, size_t, size_t);
    auto aligned = reinterpret_cast<aligned_fn>(dlsym(RTLD_DEFAULT, "malloc_numa_aligned"));
    const char* pt = getenv("SMASH_PASSTHROUGH");
    const char* mode = getenv("SMASH_MODE");
    printf("SMASH_PASSTHROUGH=%s SMASH_MODE=%s\n", pt ? pt : "(unset)", mode ? mode : "(unset)");
    errno = 0;
    if (void* p = numa(0, kMiB); p || errno != ENOTSUP)
        fail("malloc_numa(0, 1 MiB) = %p errno=%d %s, want NULL with ENOTSUP",
             p, errno, strerror(errno));
    if (!aligned) {
        fail("malloc_numa_aligned is not exported");
    } else {
        errno = 0;
        if (void* p = aligned(0, kMiB, 2 * kMiB); p || errno != ENOTSUP)
            fail("malloc_numa_aligned(0, 1 MiB, 2 MiB) = %p errno=%d %s, want NULL with ENOTSUP",
                 p, errno, strerror(errno));
    }
    if (g_failures) {
        fprintf(stderr, "FAIL: %d check(s) failed\n", g_failures);
        return 1;
    }
    printf("PASS: malloc_numa and malloc_numa_aligned refuse with ENOTSUP\n");
    return 0;
}

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);

    auto numa = reinterpret_cast<numa_fn>(dlsym(RTLD_DEFAULT, "malloc_numa"));
    auto arena_for = reinterpret_cast<arena_fn>(dlsym(RTLD_DEFAULT, "smash_arena_for"));
    if (!numa || !arena_for) {
        printf("SKIP: malloc_numa not found (not running under libsmash)\n");
        return 0;
    }
    if (const char* e = getenv("MALLOC_NUMA_EXPECT_ENOTSUP"); e && e[0] == '1')
        return runNotSupported(numa);

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
