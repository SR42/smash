// numa_compressor_test.cpp - Per-node compression of malloc_numa memory.
//
// On a host with two or more memory nodes, each node's malloc_numa pages are
// compressed by a compressor group dedicated to that node. Under LD_PRELOAD,
// in both heap modes, this leaves a 16 MiB compressible block cold on every
// node, written from off-node CPUs. Once most of each block has left RSS, it
// checks for every node that:
//   - the node has a group: running threads whose CPU affinity stays inside
//     the node's CPUs, and every compression they counted ran on one of
//     those CPUs (off_node_compressions == 0);
//   - the group counted at least as many compressions as the block has
//     evicted pages;
//   - the group's working memory (worker state, page and compress buffers,
//     zstd context) is bound to the node, and every page of it already in use
//     is on the node;
//   - the blob of every compressed page of the block is on the node;
//   - reading the block back from off-node CPUs restores every byte, with
//     every page on the node;
//   - once that off-node read-back has gone cold again, the recompressed
//     block's blobs are still on the node: the read-back leaves a node hint
//     pointing elsewhere, and the binding must win over it.
// Then the same for ordinary malloc memory, which carries no binding: the
// compressor has to discover each page's node, and the blobs must still land
// on the node the page was first touched from.
// Uses the diagnostic exports smash_numa_compressor_info and
// smash_compressed_blob_for. The ctest sets SMASH_COLD_TIMEOUT_SEC=1. Skips on
// single-node hosts, when mbind is refused, and with SMASH_NUMA_COMPRESSOR=0.
//
// NUMA_YT_HINTS=1 (test_numa_compressor_yt_hints: SMASH_WRITE_TRACK=yt,
// SMASH_YT_NODE_HINTS=1, root and the youngtrace module, skips without them):
// the ordinary blocks' nodes must come from the yt scan's node bits, not from
// move_pages. Over the ordinary blocks' phase the stats line's node_query
// must have asked about fewer pages than a quarter of the blocks' compressed
// pages (each needs one query without the hints), and node_hints must have
// set some; their blobs must still be on their node, as checked above.
//
// Nothing after the cold wait writes to malloc'd memory. CPU lists and pin
// sets are read before the wait, and later scratch arrays are mmap'd. This was
// originally required because a write into a cold full-mode heap page could be
// lost (the defect behind test_stdout_buffer_integrity_full, since fixed: the
// shutdown drain reverted live COMPRESSED_SHADOW pages to their stale blobs).
// The structure is kept so this test cannot fail for a reason unrelated to
// NUMA if that path ever regresses.

#include <cerrno>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>

#if defined(__linux__)
#include <csignal>
#include <sys/mman.h>
#include "numa_test_util.h"
#endif

// Same layout as the struct in src/smash_heap.cpp; `size` guards it.
struct SmashNumaCompressorInfo {
    uint32_t size;
    uint32_t workers;
    int32_t  tids[8];
    uint64_t pages_compressed;
    uint64_t off_node_compressions;
    void*    memory[8][4];
    uint64_t helped_compressions;   // the node's queued pages compressed by other threads
};

using numa_fn = void* (*)(uint32_t, size_t);
using info_fn = int (*)(uint32_t, SmashNumaCompressorInfo*);
using blob_fn = void* (*)(const void*);

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

static constexpr size_t kBlock = 16 * kMiB;

// NUMA_YT_HINTS=1: the stats line, dumped into a file of its own (fd 2 is
// pointed at it only for the dump; fail() prints to stderr) and read into a
// static buffer (nothing after the cold wait allocates).
static int g_stats_fd = -1;
static char g_stats[1 << 16];

static void dumpStats() {
    g_stats[0] = 0;
    // Truncating keeps the offset: rewind too, or the dump lands past a hole.
    if (g_stats_fd < 0 || ftruncate(g_stats_fd, 0) != 0 || lseek(g_stats_fd, 0, SEEK_SET) != 0) return;
    const int saved = dup(2);
    dup2(g_stats_fd, 2);
    raise(SIGUSR2);
    dup2(saved, 2);
    close(saved);
    const ssize_t n = pread(g_stats_fd, g_stats, sizeof(g_stats) - 1, 0);
    g_stats[n > 0 ? n : 0] = 0;
}

// The number after `key` in the last dump; with `after_calls`, the one after
// the " calls, " that follows it ("key=ms (C calls, P pages)").
static unsigned long long statNum(const char* key, bool after_calls = false) {
    const char* at = strstr(g_stats, key);
    if (!at) return ~0ull;
    at += strlen(key);
    if (after_calls) {
        at = strstr(at, " calls, ");
        if (!at) return ~0ull;
        at += 8;
    }
    return strtoull(at, nullptr, 10);
}

static unsigned char pattern(int node, size_t i) {
    return static_cast<unsigned char>(i * 131 + static_cast<size_t>(node) * 7 + 1);
}

// Pointers whose page is not on `node`, using caller-provided arrays (no
// heap). With `unplaced_ok`, a page never touched (-EFAULT/-ENOENT) counts as
// fine. *first_status gets the first offending status.
static size_t offNode(void** ptrs, int* status, size_t n, int node, bool unplaced_ok,
                      int* first_status) {
    if (n == 0) return 0;
    for (size_t i = 0; i < n; ++i) status[i] = INT_MIN;
    if (syscall(SYS_move_pages, 0, n, ptrs, nullptr, status, 0) != 0) {
        *first_status = -errno;
        return n;
    }
    size_t off = 0;
    for (size_t i = 0; i < n; ++i) {
        const int s = status[i];
        if (s == node || (unplaced_ok && (s == -EFAULT || s == -ENOENT))) continue;
        if (off++ == 0) *first_status = s;
    }
    return off;
}

static void checkGroup(info_fn info, int node, size_t evicted, const cpu_set_t& node_cpus) {
    SmashNumaCompressorInfo in{};
    in.size = sizeof(in);
    const int rc = info(static_cast<uint32_t>(node), &in);
    if (rc != 0) {
        fail("node %d has no compressor group (smash_numa_compressor_info rc=%d)", node, rc);
        return;
    }
    const uint32_t workers = in.workers < 8 ? in.workers : 8;
    if (workers == 0) fail("node %d: compressor group has no running threads", node);

    for (uint32_t w = 0; w < workers; ++w) {
        cpu_set_t aff;
        if (in.tids[w] <= 0 || sched_getaffinity(in.tids[w], sizeof(aff), &aff) != 0) {
            fail("node %d: cannot read affinity of compressor thread %d", node, in.tids[w]);
            continue;
        }
        for (int c = 0; c < CPU_SETSIZE; ++c) {
            if (CPU_ISSET(c, &aff) && !CPU_ISSET(c, &node_cpus)) {
                fail("node %d: compressor thread %d may run on CPU %d, outside the node",
                     node, in.tids[w], c);
                break;
            }
        }
    }
    if (in.off_node_compressions != 0)
        fail("node %d: %llu of %llu compressions ran on a CPU outside the node", node,
             static_cast<unsigned long long>(in.off_node_compressions),
             static_cast<unsigned long long>(in.pages_compressed));
    // Threads outside the group may drain the node's queue into the node's
    // store (helped_compressions); those pages are counted here too. The
    // blob placement check below is what guards where they went.
    if (in.pages_compressed + in.helped_compressions < evicted)
        fail("node %d: the group counted %llu compressions (+%llu by other threads), but %zu "
             "of the block's pages left RSS, so another compressor took them", node,
             static_cast<unsigned long long>(in.pages_compressed),
             static_cast<unsigned long long>(in.helped_compressions), evicted);

    // Working memory: bound to the node, and on it wherever already touched.
    static const char* const kNames[4] = {"state", "page buffer", "compress buffer",
                                          "zstd context"};
    void* ptrs[8 * 4];
    int status[8 * 4];
    size_t n = 0;
    for (uint32_t w = 0; w < workers; ++w) {
        for (int m = 0; m < 4; ++m) {
            void* p = in.memory[w][m];
            numa_test::Policy pol;
            if (!p) {
                fail("node %d: worker %u has no %s", node, w, kNames[m]);
                continue;
            }
            if (!numa_test::isBoundTo(p, node, &pol))
                fail("node %d: worker %u %s at %p has policy mode=%#x mask[0]=%#lx, not "
                     "bound to the node", node, w, kNames[m], p, pol.mode, pol.mask[0]);
            ptrs[n++] = p;
        }
    }
    int first = 0;
    if (size_t off = offNode(ptrs, status, n, node, true, &first))
        fail("node %d: %zu working-memory pages are on another node (first status=%d)",
             node, off, first);
    size_t used_page_bufs = 0;
    for (size_t i = 1; i < n; i += 4) used_page_bufs += status[i] == node;
    if (in.pages_compressed > 0 && used_page_bufs == 0)
        fail("node %d: compressions were counted, but no worker's page buffer was used", node);

    printf("node %d: %u compressor threads, %llu pages compressed, %zu of %u page "
           "buffers in use\n", node, workers,
           static_cast<unsigned long long>(in.pages_compressed), used_page_bufs, workers);
}

// Returns true if the test was skipped.
static bool runLinux(numa_fn numa, info_fn info, blob_fn blob) {
    const std::vector<int> nodes = numa_test::memoryNodes();
    if (nodes.size() < 2) {
        printf("SKIP: %zu memory node(s); node compressor groups need two or more\n",
               nodes.size());
        return true;
    }
    if (const char* v = getenv("SMASH_NUMA_COMPRESSOR"); v && v[0] == '0') {
        printf("SKIP: SMASH_NUMA_COMPRESSOR=0\n");
        return true;
    }
    errno = 0;
    void* probe = numa(static_cast<uint32_t>(nodes.front()), 1);
    if (!probe && (errno == EPERM || errno == ENOSYS)) {
        printf("SKIP: mbind unavailable (errno=%d %s)\n", errno, strerror(errno));
        return true;
    }
    free(probe);

    const bool yt_hints = getenv("NUMA_YT_HINTS") && getenv("NUMA_YT_HINTS")[0] == '1';
    if (yt_hints) {
        char path[] = "/tmp/smash_numa_yt_XXXXXX";
        g_stats_fd = mkstemp(path);
        if (g_stats_fd >= 0) unlink(path);
        // The tracker is chosen when the compressor starts (track=unprobed
        // until then).
        for (int i = 0; i < 100; ++i) {
            dumpStats();
            if (g_stats[0] && !strstr(g_stats, "track=unprobed")) break;
            usleep(100 * 1000);
        }
        if (!strstr(g_stats, " track=yt ")) {
            const char* t = strstr(g_stats, "track=");
            printf("SKIP: NUMA_YT_HINTS needs track=yt (root and the youngtrace module); %.*s\n",
                   t ? 20 : 40, t ? t : (g_stats[0] ? g_stats : "no stats line"));
            return true;
        }
        char meta[4] = {};
        if (FILE* f = fopen("/sys/module/youngtrace/parameters/page_meta", "r")) {
            if (!fgets(meta, sizeof(meta), f)) meta[0] = 0;
            fclose(f);
        }
        if (meta[0] != 'Y') {
            printf("SKIP: the youngtrace module reports no page nodes (page_meta is not Y)\n");
            return true;
        }
    }

    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    const size_t pages = kBlock / ps;
    const size_t count = nodes.size();

    // Everything read after the wait is prepared now.
    cpu_set_t allowed;
    if (sched_getaffinity(0, sizeof(allowed), &allowed) != 0) CPU_ZERO(&allowed);
    std::vector<cpu_set_t> node_cpus(count), off_node(count);
    std::vector<bool> can_pin(count);     // CPUs outside the node
    std::vector<bool> can_pin_on(count);  // CPUs on the node
    for (size_t k = 0; k < count; ++k) {
        char path[128];
        snprintf(path, sizeof(path), "/sys/devices/system/node/node%d/cpulist", nodes[k]);
        CPU_ZERO(&node_cpus[k]);
        for (int c : numa_test::readList(path)) CPU_SET(c, &node_cpus[k]);
        off_node[k] = allowed;
        for (int c = 0; c < CPU_SETSIZE; ++c)
            if (CPU_ISSET(c, &node_cpus[k])) CPU_CLR(c, &off_node[k]);
        can_pin[k] = CPU_COUNT(&off_node[k]) > 0;
        can_pin_on[k] = CPU_COUNT(&node_cpus[k]) > 0;
    }
    std::vector<unsigned char*> blocks(count, nullptr);
    std::vector<size_t> evicted(count, 0);
    std::vector<unsigned char*> plain(count, nullptr);
    std::vector<size_t> plain_evicted(count, 0);
    const size_t scratch_bytes = pages * (sizeof(void*) + sizeof(int) + 1);
    void* scratch = mmap(nullptr, scratch_bytes, PROT_READ | PROT_WRITE,
                         MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (scratch == MAP_FAILED) {
        fail("mmap scratch failed");
        return false;
    }
    auto* ptrs = static_cast<void**>(scratch);
    auto* status = reinterpret_cast<int*>(ptrs + pages);
    auto* vec = reinterpret_cast<unsigned char*>(status + pages);

    for (size_t k = 0; k < count; ++k) {
        const int node = nodes[k];
        auto* p = static_cast<unsigned char*>(numa(static_cast<uint32_t>(node), kBlock));
        if (!p) {
            fail("malloc_numa(%d, 16 MiB) returned NULL (errno=%d)", node, errno);
            continue;
        }
        if (can_pin[k]) sched_setaffinity(0, sizeof(off_node[k]), &off_node[k]);
        for (size_t i = 0; i < kBlock; ++i) p[i] = pattern(node, i);
        if (can_pin[k]) sched_setaffinity(0, sizeof(allowed), &allowed);
        blocks[k] = p;
    }

    // Ordinary malloc: unbound, so first touch decides the node and the
    // compressor has to discover it.
    dumpStats();
    const unsigned long long hints0 = statNum(" node_hints="), query0 = statNum("node_query=", true);
    for (size_t k = 0; k < count; ++k) {
        const int node = nodes[k];
        auto* p = static_cast<unsigned char*>(malloc(kBlock));
        if (!p) {
            fail("malloc(16 MiB) returned NULL");
            continue;
        }
        // Pinned ON the node, unlike the bound blocks above: with no binding,
        // first touch is what places these pages, and the compressor then has
        // to discover where they are.
        if (can_pin_on[k]) sched_setaffinity(0, sizeof(node_cpus[k]), &node_cpus[k]);
        for (size_t i = 0; i < kBlock; ++i) p[i] = pattern(node, i);
        if (can_pin_on[k]) sched_setaffinity(0, sizeof(allowed), &allowed);
        plain[k] = p;
    }

    // Wait for at least half of every block to leave RSS.
    int waited_ms = 0;
    for (; waited_ms <= 30000; waited_ms += 250) {
        bool all = true;
        for (size_t k = 0; k < count; ++k) {
            if (blocks[k]) {
                evicted[k] = 0;
                if (mincore(blocks[k], kBlock, vec) == 0)
                    for (size_t i = 0; i < pages; ++i) evicted[k] += !(vec[i] & 1);
                if (evicted[k] < pages / 2) all = false;
            }
            if (plain[k]) {
                plain_evicted[k] = 0;
                if (mincore(plain[k], kBlock, vec) == 0)
                    for (size_t i = 0; i < pages; ++i) plain_evicted[k] += !(vec[i] & 1);
                if (plain_evicted[k] < pages / 2) all = false;
            }
        }
        if (all) break;
        usleep(250 * 1000);
    }
    printf("compression wait: %d ms\n", waited_ms);

    for (size_t k = 0; k < count; ++k) {
        unsigned char* p = blocks[k];
        if (!p) continue;
        const int node = nodes[k];
        if (evicted[k] < pages / 2)
            fail("node %d: only %zu of %zu pages of a cold block left RSS within 30 s",
                 node, evicted[k], pages);
        checkGroup(info, node, evicted[k], node_cpus[k]);

        // Every evicted page must have a blob, and every blob must be on the node.
        size_t blobs = 0;
        size_t no_blob = 0;
        if (mincore(p, kBlock, vec) == 0) {
            for (size_t i = 0; i < pages; ++i) {
                if (vec[i] & 1) continue;
                if (void* b = blob(p + i * ps)) ptrs[blobs++] = b;
                else ++no_blob;
            }
        }
        if (no_blob) fail("node %d: %zu evicted pages have no compressed blob", node, no_blob);
        int first = 0;
        if (size_t off = offNode(ptrs, status, blobs, node, false, &first))
            fail("node %d: %zu of %zu compressed blobs are not on the node (first status=%d)",
                 node, off, blobs, first);

        // Read back from off-node CPUs: restored pages must return to the node.
        if (can_pin[k]) sched_setaffinity(0, sizeof(off_node[k]), &off_node[k]);
        size_t differ = 0;
        for (size_t i = 0; i < kBlock; ++i) differ += p[i] != pattern(node, i);
        if (can_pin[k]) sched_setaffinity(0, sizeof(allowed), &allowed);
        if (differ) fail("node %d: %zu bytes differ after decompression", node, differ);
        for (size_t i = 0; i < pages; ++i) ptrs[i] = p + i * ps;
        size_t off = offNode(ptrs, status, pages, node, false, &first);
        if (off)
            fail("node %d: %zu of %zu pages restored off the node (first status=%d)",
                 node, off, pages, first);
        printf("node %d: %zu/%zu pages compressed, %zu blobs on the node, %zu/%zu pages "
               "restored onto it\n", node, evicted[k], pages, blobs, pages - off, pages);
        // Kept for the second round below, not freed.
    }
    // Ordinary malloc memory: unbound, node discovered per page. Its blobs
    // must still follow the page's own node. Before the routing batch was made
    // rolling, three of four blocks had every blob on one node instead.
    for (size_t k = 0; k < count; ++k) {
        unsigned char* p = plain[k];
        if (!p) continue;
        const int node = nodes[k];
        if (plain_evicted[k] < pages / 2) {
            fail("malloc block on node %d: only %zu of %zu pages left RSS within 30 s",
                 node, plain_evicted[k], pages);
            continue;
        }
        size_t blobs = 0;
        size_t no_blob = 0;
        if (mincore(p, kBlock, vec) == 0) {
            for (size_t i = 0; i < pages; ++i) {
                if (vec[i] & 1) continue;
                if (void* b = blob(p + i * ps)) ptrs[blobs++] = b;
                else ++no_blob;
            }
        }
        if (no_blob)
            fail("malloc block on node %d: %zu evicted pages have no blob", node, no_blob);
        int first = 0;
        const size_t off = offNode(ptrs, status, blobs, node, false, &first);
        // A few strays are fine: a page whose node was learned late, or one
        // compressed before its node was known. Broken routing sends nearly
        // all of them to whichever node the shared store sits on.
        if (blobs == 0 || off * 10 > blobs)
            fail("malloc block first touched on node %d: %zu of %zu blobs are off the "
                 "node (first status=%d) — compression is not following the page's node",
                 node, off, blobs, first);
        size_t differ = 0;
        for (size_t i = 0; i < kBlock; ++i) differ += p[i] != pattern(node, i);
        if (differ)
            fail("malloc block on node %d: %zu bytes differ after decompression",
                 node, differ);
        printf("malloc block on node %d: %zu/%zu pages compressed, %zu/%zu blobs on "
               "the node\n", node, plain_evicted[k], pages, blobs - off, blobs);
        free(p);
    }

    if (yt_hints) {
        dumpStats();
        const unsigned long long hints = statNum(" node_hints=") - hints0,
                                 queried = statNum("node_query=", true) - query0;
        size_t compressed = 0;
        for (size_t k = 0; k < count; ++k) compressed += plain_evicted[k];
        printf("yt node hints: %llu pages hinted, %llu move_pages queries, for %zu compressed pages "
               "of the ordinary blocks\n", hints, queried, compressed);
        if (hints == 0 || hints > (1ull << 40))
            fail("yt node hints: the scan set no page's node (node_hints +%llu)", hints);
        if (queried > (1ull << 40) || queried * 4 >= compressed)
            fail("yt node hints: %llu move_pages queries for %zu compressed pages - the nodes "
                 "did not come from the scan", queried, compressed);
    }

    // Second round for the bound blocks. The off-node read-back above faulted
    // every page in on a CPU outside the node, and the fault handler records
    // the faulting CPU's node as the page's node hint (notePageNode) — right
    // for ordinary memory, wrong for a malloc_numa page, whose node is fixed
    // by its binding. Once the blocks go cold again, their blobs must still be
    // on the block's own node: a group that trusted the hint over the binding
    // handed every page to the other node's group and its store.
    for (int round_ms = 0; round_ms <= 30000; round_ms += 250) {
        bool all = true;
        for (size_t k = 0; k < count; ++k) {
            if (!blocks[k]) continue;
            evicted[k] = 0;
            if (mincore(blocks[k], kBlock, vec) == 0)
                for (size_t i = 0; i < pages; ++i) evicted[k] += !(vec[i] & 1);
            if (evicted[k] < pages / 2) all = false;
        }
        if (all) {
            printf("second compression wait: %d ms\n", round_ms);
            break;
        }
        usleep(250 * 1000);
    }
    for (size_t k = 0; k < count; ++k) {
        unsigned char* p = blocks[k];
        if (!p) continue;
        const int node = nodes[k];
        if (evicted[k] < pages / 2) {
            fail("node %d: only %zu of %zu pages left RSS again within 30 s of the "
                 "read-back", node, evicted[k], pages);
            free(p);
            continue;
        }
        size_t blobs = 0;
        if (mincore(p, kBlock, vec) == 0) {
            for (size_t i = 0; i < pages; ++i) {
                if (vec[i] & 1) continue;
                if (void* b = blob(p + i * ps)) ptrs[blobs++] = b;
            }
        }
        int first = 0;
        const size_t off = offNode(ptrs, status, blobs, node, false, &first);
        if (blobs == 0 || off)
            fail("node %d: after an off-node read-back, %zu of %zu blobs of the "
                 "recompressed block are not on the node (first status=%d) — the page's "
                 "node hint overrode its binding", node, off, blobs, first);
        printf("node %d: recompressed after off-node reads, %zu/%zu blobs on the node\n",
               node, blobs - off, blobs);
        free(p);
    }

    munmap(scratch, scratch_bytes);
    return false;
}

#endif

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);

    auto numa = reinterpret_cast<numa_fn>(dlsym(RTLD_DEFAULT, "malloc_numa"));
    auto info = reinterpret_cast<info_fn>(dlsym(RTLD_DEFAULT, "smash_numa_compressor_info"));
    auto blob = reinterpret_cast<blob_fn>(dlsym(RTLD_DEFAULT, "smash_compressed_blob_for"));
    if (!numa || !info || !blob) {
        fprintf(stderr, "FAIL: NUMA exports not found (not running under libsmash)\n");
        return 1;
    }

#if defined(__linux__)
    if (runLinux(numa, info, blob)) return 77;
#else
    printf("SKIP: NUMA compressor groups are Linux only\n");
    return 77;
#endif

    if (g_failures) {
        fprintf(stderr, "FAIL: %d check(s) failed\n", g_failures);
        return 1;
    }
    printf("PASS: every node's pages are compressed and stored on that node\n");
    return 0;
}
