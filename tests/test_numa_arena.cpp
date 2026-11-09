// test_numa_arena.cpp - VmRegion's NUMA node arenas (backing malloc_numa).
//
// Direct-linked, with no preload and no compressor, so every step is
// deterministic. On each online node it checks that:
//   - a node allocation is inside the region, starts on the 64 MiB chunk
//     grid, and is bound MPOL_BIND | MPOL_F_STATIC_NODES to exactly {node};
//   - its pages land on the node even when written from off-node CPUs;
//   - freed pages are reused by the same node and never handed out by the
//     shared allocatePages();
//   - vm::remapPages() on a node page (the compressor's fallback when mprotect
//     fails) leaves the page bound, where MAP_FIXED alone would reset it to
//     the default policy. A remap spanning a shared page and a node page binds
//     only the node page.
// Plus: the gap skipped below a chunk and a chunk carved for a bad node both
// become unbound shared pages, a bad node fails with EINVAL, and an oversized
// request fails with ENOMEM.
#include "vm/vm_region.h"
#include "smash/config.h"
#include "numa_test_util.h"

#include <cerrno>
#include <cstdio>
#include <cstring>

using namespace smash;

static int failures = 0;

#define CHECK(cond, msg, ...) do { \
    if (!(cond)) { \
        fprintf(stderr, "FAIL: " msg "\n", ##__VA_ARGS__); \
        ++failures; \
    } \
} while (0)

#if defined(__linux__)

static int pageNode(const VmRegion& vm, void* p) {
    return vm.nodeOfPage(vm.contigPageIndex(reinterpret_cast<uintptr_t>(p)));
}

// Writes [p, p+bytes) from CPUs off `node` (when there are any), then checks
// every page is on `node`.
static void checkPlacedOnNode(void* p, size_t bytes, int node, const cpu_set_t* allowed,
                              const char* what) {
    const bool pinned = allowed && numa_test::pinOutside(node, *allowed);
    memset(p, 0x40 + node, bytes);
    if (pinned) sched_setaffinity(0, sizeof(*allowed), allowed);
    int status = 0;
    size_t off = numa_test::pagesOffNode(p, bytes, node, &status);
    CHECK(off == 0, "%s on node %d: %zu of %zu pages off-node (first status=%d)",
          what, node, off, bytes / kPageSize, status);
}

static void checkUnbound(void* p, const char* what) {
    numa_test::Policy pol;
    static_cast<char*>(p)[0] = 1;
    CHECK(numa_test::queryPolicy(p, &pol) && pol.mode == MPOL_DEFAULT,
          "%s %p carries policy mode=%#x: a node binding leaked into shared pages",
          what, p, pol.mode);
}

static bool testNodeArenas() {
    const std::vector<int> nodes = numa_test::onlineNodes();
    const size_t chunk_bytes = VmRegion::kNodeChunkPages * kPageSize;
    const size_t region_pages = 16 * VmRegion::kNodeChunkPages;

    // Static: the remap hook points at the first region that creates node
    // arenas, so it must outlive every remapPages() call below.
    static VmRegion vm;
    if (!vm.init(region_pages * kPageSize)) {
        CHECK(false, "VmRegion init failed");
        return true;
    }
    const auto base = reinterpret_cast<uintptr_t>(vm.base());

    // A shared page first, so the first node chunk has to skip to the grid.
    void* shared0 = vm.allocatePages(1);
    CHECK(shared0 == vm.base(), "first shared page %p, want region base %p",
          shared0, static_cast<void*>(vm.base()));

    errno = 0;
    void* probe = vm.allocatePagesOnNode(3, static_cast<uint32_t>(nodes.front()));
    if (!probe && (errno == EPERM || errno == ENOSYS)) {
        fprintf(stderr, "numa_arena: SKIP (mbind unavailable: errno=%d %s)\n",
                errno, strerror(errno));
        return false;
    }

    cpu_set_t allowed;
    CPU_ZERO(&allowed);
    const cpu_set_t* pin =
        sched_getaffinity(0, sizeof(allowed), &allowed) == 0 ? &allowed : nullptr;

    // One 3-page allocation per node: each carves its node's first chunk.
    std::vector<void*> first(nodes.size(), nullptr);
    for (size_t k = 0; k < nodes.size(); ++k) {
        const int node = nodes[k];
        void* p = (k == 0) ? probe : vm.allocatePagesOnNode(3, static_cast<uint32_t>(node));
        if (!p) {
            CHECK(false, "allocatePagesOnNode(3, %d) failed: errno=%d %s",
                  node, errno, strerror(errno));
            continue;
        }
        const auto a = reinterpret_cast<uintptr_t>(p);
        CHECK(vm.inContigArena(a), "node %d: %p is outside the region", node, p);
        CHECK((a - base) % chunk_bytes == 0 && a > base,
              "node %d: first allocation %p is not on the chunk grid", node, p);
        CHECK(pageNode(vm, p) == node, "node %d: nodeOfPage(%p) = %d",
              node, p, pageNode(vm, p));
        numa_test::Policy pol;
        CHECK(numa_test::isBoundTo(p, node, &pol),
              "node %d: policy mode=%#x mask[0]=%#lx, want MPOL_BIND|MPOL_F_STATIC_NODES "
              "on {%d}", node, pol.mode, pol.mask[0], node);
        checkPlacedOnNode(p, 3 * kPageSize, node, pin, "first allocation");
        first[k] = p;
    }

    // The gap below the first chunk went to the shared free lists, unbound.
    for (int i = 0; i < 8; ++i) {
        void* s = vm.allocatePages(2);
        if (!s) {
            CHECK(false, "shared allocatePages(2) failed");
            break;
        }
        CHECK(pageNode(vm, s) == -1, "shared allocatePages returned node-arena page %p", s);
        checkUnbound(s, "shared page from the gap");
    }

    // Freed node pages return to their own arena, never the shared lists.
    for (size_t k = 0; k < nodes.size(); ++k)
        if (first[k]) vm.releasePages(first[k], 3);
    usleep(200 * 1000);  // the decommit thread moves runs onto free lists
    for (size_t k = 0; k < nodes.size(); ++k) {
        void* s = vm.allocatePages(3);
        CHECK(s && pageNode(vm, s) == -1,
              "shared allocatePages(3) after freeing node pages returned %p (node %d)",
              s, s ? pageNode(vm, s) : -2);
    }
    for (size_t k = 0; k < nodes.size(); ++k) {
        if (!first[k]) continue;
        const int node = nodes[k];
        void* again = vm.allocatePagesOnNode(3, static_cast<uint32_t>(node));
        CHECK(again == first[k], "node %d: freed pages not reused (got %p, freed %p)",
              node, again, first[k]);
        if (again) checkPlacedOnNode(again, 3 * kPageSize, node, pin, "reused pages");
    }

    // remapPages() replaces the VMA; the hook must bind the page again.
    {
        const int node = nodes.back();
        void* p = vm.allocatePagesOnNode(1, static_cast<uint32_t>(node));
        CHECK(p != nullptr, "allocatePagesOnNode(1, %d) failed", node);
        if (p) {
            memset(p, 7, kPageSize);
            CHECK(vm::remapPages(p, kPageSize, true, true), "remapPages(%p) failed", p);
            numa_test::Policy pol;
            CHECK(numa_test::isBoundTo(p, node, &pol),
                  "remapPages dropped the node %d binding on %p: mode=%#x", node, p, pol.mode);
            checkPlacedOnNode(p, kPageSize, node, pin, "remapped page");
        }
    }

    // A remap spanning the last gap page and the first node chunk's first page
    // binds only the node page. (Discards that page's bytes; it is not reused.)
    {
        void* below = reinterpret_cast<void*>(base + chunk_bytes - kPageSize);
        void* grid = reinterpret_cast<void*>(base + chunk_bytes);
        CHECK(pageNode(vm, below) == -1 && pageNode(vm, grid) == nodes.front(),
              "unexpected layout around the first chunk: %d / %d",
              pageNode(vm, below), pageNode(vm, grid));
        CHECK(vm::remapPages(below, 2 * kPageSize, true, true), "straddling remap failed");
        checkUnbound(below, "shared page in a straddling remap");
        numa_test::Policy pol;
        CHECK(numa_test::isBoundTo(grid, nodes.front(), &pol),
              "straddling remap left node page %p with mode=%#x", grid, pol.mode);
    }

    // Bad nodes. The chunk carved for the offline node becomes shared pages.
    const uint32_t past = static_cast<uint32_t>(nodes.back()) + 1;
    if (past < vm::kMaxNumaNodes) {
        errno = 0;
        void* bad = vm.allocatePagesOnNode(1, past);
        CHECK(!bad && errno == EINVAL, "allocatePagesOnNode(1, %u) = %p errno=%d, want EINVAL",
              past, bad, errno);
        void* s = vm.allocatePages(VmRegion::kNodeChunkPages);
        CHECK(s && pageNode(vm, s) == -1,
              "the chunk carved for bad node %u did not become a shared run (got %p)", past, s);
        if (s) checkUnbound(s, "chunk carved for a bad node");
    }
    errno = 0;
    CHECK(!vm.allocatePagesOnNode(1, vm::kMaxNumaNodes) && errno == EINVAL,
          "allocatePagesOnNode(1, kMaxNumaNodes) errno=%d, want EINVAL", errno);

    errno = 0;
    CHECK(!vm.allocatePagesOnNode(region_pages, static_cast<uint32_t>(nodes.front())) &&
              errno == ENOMEM,
          "allocatePagesOnNode(whole region) errno=%d, want ENOMEM", errno);

    fprintf(stderr, "numa_arena: %zu node(s) checked\n", nodes.size());
    return true;
}

#endif

int main() {
#if defined(__linux__)
    if (!testNodeArenas()) return 77;  // skipped
#else
    fprintf(stderr, "numa_arena: SKIP (Linux only)\n");
    return 77;
#endif
    if (failures == 0) {
        fprintf(stderr, "numa_arena: all tests passed\n");
        return 0;
    }
    fprintf(stderr, "numa_arena: %d failures\n", failures);
    return 1;
}
