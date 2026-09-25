// smash_sim.cpp - drive smash's large-block memory management at full rate
// without touching memory and without the compressor.
//
// Built with -DSMASH_SIM, which turns the page-level OS calls (mprotect,
// madvise, mbind) into no-ops with an optional simulated cost (--os-call-ns)
// and makes Spinlock record contention per call site. Everything else is the
// real code: LargeAlloc, VmRegion (shared arena, per-node arenas, the free
// map, the decommit queue and thread, the run cache), the span-descriptor
// pools and the page map. The compressor is replaced by a release hook that
// reports a chosen fraction of frees as "had compressed pages" (--cold).
//
// Block memory is never written: the arena is a PROT_NONE reservation and no
// commit reaches the kernel, so a million 8 MiB blocks cost no RSS. What is
// real is every lock, atomic and list the allocator uses, so the lock report
// shows where threads wait on this machine. It cannot make 8 cores contend
// like 128 — oversubscription is not parallelism — but it measures policy
// (cache hit rates for a size mix, arena walk, fragmentation) exactly, and
// checks invariants under stress:
//   - no page is ever handed to two live blocks (a shadow ownership map);
//   - the page map resolves every live block to its own span;
//   - after everything is freed and drained, every carved page is free.
//
// Usage: smash_sim [--threads N] [--seconds S] [--nodes K] [--arena-gib G]
//                  [--sizes 1M,2M:3,...] [--live K] [--numa F] [--cold F]
//                  [--cross-free F] [--run-cache-mib M] [--os-call-ns NS]
//                  [--subarenas S] [--decay-ms MS] [--dirty-max-mib M]
//                  [--no-check] [--pin] [--seed X]
// Exit status is 1 if an invariant failed.
#include "smash/config.h"
#include "core/large_alloc.h"
#include "core/page_map.h"
#include "vm/page_state.h"
#include "vm/vm_region.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <link.h>
#include <cxxabi.h>
#include <mutex>
#include <sched.h>
#include <string>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace smash;

// The embedded printf library (safe_printf) expects its user to supply this;
// smash_heap.cpp does in the library. With -rdynamic every printf entry point
// is exported and kept, so the reference has to resolve here too.
extern "C" void _putchar(char c) { std::fputc(c, stderr); }

namespace {

struct Options {
    int threads = 8;
    int seconds = 5;
    int nodes = 4;
    size_t arena_gib = 16;
    std::vector<std::pair<size_t, unsigned>> sizes;   // bytes, weight
    int live = 8;
    double numa = 1.0;        // fraction of allocations through the node path
    double cold = 0.0;        // fraction of frees reported as compressed
    double cross_free = 0.0;  // fraction of frees handed to another thread
    long run_cache_mib = 0;
    int subarenas = -1;         // -1 off; 0 one per CPU of the node; else per node
    long decay_ms = -1;
    long dirty_max_mib = -1;
    uint64_t os_call_ns = 0;
    bool check = true;
    bool pin = false;         // run thread t on the CPUs of node t % nodes
    uint64_t seed = 1;
};
Options opt;

size_t parseSize(const char* s) {
    char* end = nullptr;
    double v = std::strtod(s, &end);
    switch (end && *end ? *end : 'B') {
        case 'K': case 'k': v *= 1024; break;
        case 'M': case 'm': v *= 1024 * 1024; break;
        case 'G': case 'g': v *= 1024.0 * 1024 * 1024; break;
        default: break;
    }
    return static_cast<size_t>(v);
}

void parseSizes(const char* list) {
    opt.sizes.clear();
    std::string s(list);
    size_t pos = 0;
    while (pos <= s.size()) {
        size_t comma = s.find(',', pos);
        std::string item = s.substr(pos, comma == std::string::npos ? std::string::npos : comma - pos);
        if (!item.empty()) {
            unsigned w = 1;
            if (size_t colon = item.find(':'); colon != std::string::npos) {
                w = static_cast<unsigned>(std::atoi(item.c_str() + colon + 1));
                item.resize(colon);
            }
            opt.sizes.emplace_back(parseSize(item.c_str()), w ? w : 1);
        }
        if (comma == std::string::npos) break;
        pos = comma + 1;
    }
}

void usage() {
    std::fprintf(stderr,
        "usage: smash_sim [--threads N] [--seconds S] [--nodes K] [--arena-gib G]\n"
        "                 [--sizes 1M,2M:3,...] [--live K] [--numa F] [--cold F]\n"
        "                 [--cross-free F] [--run-cache-mib M] [--os-call-ns NS]\n"
        "                 [--subarenas S] [--decay-ms MS] [--dirty-max-mib M]\n"
        "                 [--no-check] [--pin] [--seed X]\n");
    std::exit(2);
}

void parseArgs(int argc, char** argv) {
    parseSizes("1M,2M,3M,4M,5M,6M,7M,8M");
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> const char* { if (i + 1 >= argc) usage(); return argv[++i]; };
        if (a == "--threads") opt.threads = std::atoi(next());
        else if (a == "--seconds") opt.seconds = std::atoi(next());
        else if (a == "--nodes") opt.nodes = std::atoi(next());
        else if (a == "--arena-gib") opt.arena_gib = std::strtoull(next(), nullptr, 10);
        else if (a == "--sizes") parseSizes(next());
        else if (a == "--live") opt.live = std::atoi(next());
        else if (a == "--numa") opt.numa = std::atof(next());
        else if (a == "--cold") opt.cold = std::atof(next());
        else if (a == "--cross-free") opt.cross_free = std::atof(next());
        else if (a == "--run-cache-mib") opt.run_cache_mib = std::atol(next());
        else if (a == "--subarenas") opt.subarenas = std::atoi(next());
        else if (a == "--decay-ms") opt.decay_ms = std::atol(next());
        else if (a == "--dirty-max-mib") opt.dirty_max_mib = std::atol(next());
        else if (a == "--os-call-ns") opt.os_call_ns = std::strtoull(next(), nullptr, 10);
        else if (a == "--no-check") opt.check = false;
        else if (a == "--pin") opt.pin = true;
        else if (a == "--seed") opt.seed = std::strtoull(next(), nullptr, 10);
        else usage();
    }
    if (opt.threads < 1 || opt.seconds < 1 || opt.nodes < 1 || opt.live < 1 ||
        opt.sizes.empty() || opt.nodes > static_cast<int>(vm::kMaxNumaNodes))
        usage();
}

// ── The allocator under test ────────────────────────────────────────────────
VmRegion g_vm;
PageMap g_page_map;
PageStateTable g_states;
LargeAlloc g_large;

// Stands in for the compressor's release: resets the pages as it would, and
// reports a run as "had compressed pages" with probability --cold.
thread_local uint64_t tl_rng = 0;
uint64_t rnd() {
    tl_rng ^= tl_rng << 13; tl_rng ^= tl_rng >> 7; tl_rng ^= tl_rng << 17;
    return tl_rng;
}
double rnd01() { return static_cast<double>(rnd() >> 11) * (1.0 / 9007199254740992.0); }

bool simRelease(size_t first, size_t count, void*) {
    for (size_t i = first; i < first + count; ++i) g_states.set(i, PageState::EMPTY);
    return !(opt.cold > 0 && rnd01() < opt.cold);
}

// ── Invariant checking without touching block memory ────────────────────────
std::atomic<uint32_t>* g_owner = nullptr;   // per arena page: 0 or the owning block id
std::atomic<uint64_t> g_double_handout{0}, g_owner_mismatch{0}, g_span_mismatch{0};
std::atomic<uint32_t> g_next_block_id{1};

void claim(void* p, size_t bytes, uint32_t id) {
    if (!opt.check || !g_vm.inContigArena(reinterpret_cast<uintptr_t>(p))) return;
    const size_t first = g_vm.contigPageIndex(reinterpret_cast<uintptr_t>(p));
    for (size_t i = 0; i < (bytes + kPageSize - 1) / kPageSize; ++i) {
        uint32_t expected = 0;
        if (!g_owner[first + i].compare_exchange_strong(expected, id)) ++g_double_handout;
    }
}
void unclaim(void* p, size_t bytes, uint32_t id) {
    if (!opt.check || !g_vm.inContigArena(reinterpret_cast<uintptr_t>(p))) return;
    const size_t first = g_vm.contigPageIndex(reinterpret_cast<uintptr_t>(p));
    for (size_t i = 0; i < (bytes + kPageSize - 1) / kPageSize; ++i)
        if (g_owner[first + i].exchange(0) != id) ++g_owner_mismatch;
}

struct Block { void* p = nullptr; size_t bytes = 0; uint32_t id = 0; };

void freeBlock(const Block& b) {
    unclaim(b.p, b.bytes, b.id);
    Span* span = g_page_map.get(reinterpret_cast<uintptr_t>(b.p));
    if (!span || span->base != b.p || !span->is_large) { ++g_span_mismatch; return; }
    g_large.deallocate(span);
}

// ── Pinning ─────────────────────────────────────────────────────────────────
// --pin: thread t runs on the CPUs of node t % nodes, so every allocation is
// node-local — the NUMA-aware pattern. Without it a thread allocates on node
// t % nodes from wherever the scheduler put it, and every lock and queue head
// it touches may live on another socket. Simulated node n maps to real node
// n % (real nodes); a host whose node lists cannot be read is not pinned.
std::vector<cpu_set_t> g_node_cpus;
void loadNodeCpus() {
    for (int n = 0;; ++n) {
        char path[96];
        std::snprintf(path, sizeof(path), "/sys/devices/system/node/node%d/cpulist", n);
        FILE* f = std::fopen(path, "r");
        if (!f) break;
        cpu_set_t set;
        CPU_ZERO(&set);
        char buf[4096] = {};
        if (std::fgets(buf, sizeof(buf), f)) {
            for (char* tok = std::strtok(buf, ",\n"); tok; tok = std::strtok(nullptr, ",\n")) {
                int lo = 0, hi = 0;
                if (std::sscanf(tok, "%d-%d", &lo, &hi) == 2) {
                    for (int c = lo; c <= hi && c < CPU_SETSIZE; ++c) CPU_SET(c, &set);
                } else if (std::sscanf(tok, "%d", &lo) == 1 && lo < CPU_SETSIZE) {
                    CPU_SET(lo, &set);
                }
            }
        }
        std::fclose(f);
        if (CPU_COUNT(&set) > 0) g_node_cpus.push_back(set);
    }
}

// ── Workers ─────────────────────────────────────────────────────────────────
struct alignas(64) Inbox {
    std::mutex m;
    std::vector<Block> blocks;
};
std::vector<Inbox> g_inbox;
std::atomic<bool> g_stop{false};
std::atomic<uint64_t> g_ops{0}, g_outside{0}, g_nulls{0};
uint64_t g_weight_total = 0;

size_t pickSize() {
    uint64_t r = rnd() % g_weight_total;
    for (const auto& [bytes, w] : opt.sizes) {
        if (r < w) return bytes;
        r -= w;
    }
    return opt.sizes.back().first;
}

struct WorkerResult { uint64_t os_calls = 0; };

void worker(int t, WorkerResult* out) {
    tl_rng = opt.seed * 0x9E3779B97F4A7C15ull + static_cast<uint64_t>(t) + 1;
    if (opt.pin && !g_node_cpus.empty()) {
        const cpu_set_t& set = g_node_cpus[static_cast<size_t>(t % opt.nodes) % g_node_cpus.size()];
        sched_setaffinity(0, sizeof(set), &set);
    }
    std::vector<Block> ring(static_cast<size_t>(opt.live));
    std::vector<Block> incoming;
    uint64_t ops = 0;
    while (!g_stop.load(std::memory_order_relaxed)) {
        Block& slot = ring[rnd() % ring.size()];
        if (slot.p) {
            if (opt.cross_free > 0 && opt.threads > 1 && rnd01() < opt.cross_free) {
                Inbox& in = g_inbox[(t + 1 + rnd() % (opt.threads - 1)) % opt.threads];
                std::lock_guard<std::mutex> g(in.m);
                in.blocks.push_back(slot);
            } else {
                freeBlock(slot);
            }
            slot = Block{};
        }
        {
            std::lock_guard<std::mutex> g(g_inbox[t].m);
            incoming.swap(g_inbox[t].blocks);
        }
        for (const Block& b : incoming) freeBlock(b);
        incoming.clear();

        const size_t bytes = pickSize();
        void* p;
        if (rnd01() < opt.numa)
            p = g_large.allocateOnNode(bytes, static_cast<uint32_t>(t % opt.nodes));
        else
            p = g_large.allocate(bytes, kMinAlignment, 0);
        if (!p) { ++g_nulls; continue; }
        if (!g_vm.inContigArena(reinterpret_cast<uintptr_t>(p))) ++g_outside;
        slot = Block{p, bytes, g_next_block_id.fetch_add(1, std::memory_order_relaxed)};
        claim(slot.p, slot.bytes, slot.id);
        if ((++ops & 255) == 0) g_ops.fetch_add(256, std::memory_order_relaxed);
    }
    g_ops.fetch_add(ops & 255, std::memory_order_relaxed);
    for (const Block& b : ring) if (b.p) freeBlock(b);
    out->os_calls = vm::tl_sim_os_calls;
}

// ── Reporting ───────────────────────────────────────────────────────────────
// The link-time address of `ra` in the object that contains it, and that
// object's path. dl_iterate_phdr gives the load bias, which is what addr2line
// needs subtracted: dladdr's dli_fbase is the mapping base, which equals the
// bias for a PIE but is 0x400000 (or wherever) for a non-PIE executable.
struct ObjAddr { uintptr_t link_addr = 0; std::string path; bool found = false; };
ObjAddr objectAddress(uintptr_t ra) {
    struct Ctx { uintptr_t ra; ObjAddr out; } ctx{ra, {}};
    dl_iterate_phdr([](dl_phdr_info* info, size_t, void* data) -> int {
        auto* c = static_cast<Ctx*>(data);
        for (int i = 0; i < info->dlpi_phnum; ++i) {
            const ElfW(Phdr)& ph = info->dlpi_phdr[i];
            if (ph.p_type != PT_LOAD) continue;
            const uintptr_t lo = info->dlpi_addr + ph.p_vaddr;
            if (c->ra >= lo && c->ra < lo + ph.p_memsz) {
                c->out.link_addr = c->ra - info->dlpi_addr;
                c->out.path = info->dlpi_name ? info->dlpi_name : "";
                c->out.found = true;
                return 1;
            }
        }
        return 0;
    }, &ctx);
    // The main program's entry has an empty name; resolve our own executable
    // here (not "/proc/self/exe" itself, which in addr2line's process names
    // addr2line).
    if (ctx.out.found && (ctx.out.path.empty() || ctx.out.path[0] != '/')) {
        char self[4096];
        const ssize_t n = readlink("/proc/self/exe", self, sizeof(self) - 1);
        if (n > 0) { self[n] = 0; ctx.out.path = self; }
    }
    return ctx.out;
}

// Name a lock call site. Two sources, because either can fail on a given host:
//   - dladdr + demangling, from the executable's own dynamic symbol table
//     (smash_sim links with -rdynamic): the enclosing function, needs nothing
//     external, but no line and no inlined frames;
//   - addr2line on the debug info: the innermost inlined frame with file:line,
//     but it needs binutils that can read this compiler's DWARF (GCC 11+
//     writes DWARF 5, which older binutils — e.g. RHEL 8's — print as ??).
// The addr2line answer is used when it has one; otherwise the dladdr name.
std::string symbolize(uintptr_t ra) {
    const uintptr_t call = ra - 1;   // inside the call instruction, not after it
    std::string name;
    Dl_info info{};
    if (dladdr(reinterpret_cast<void*>(call), &info) && info.dli_sname) {
        int status = 0;
        char* dem = abi::__cxa_demangle(info.dli_sname, nullptr, nullptr, &status);
        name = (status == 0 && dem) ? dem : info.dli_sname;
        std::free(dem);
    }
    const ObjAddr oa = objectAddress(call);
    std::string line;
    if (oa.found && !oa.path.empty()) {
        char cmd[4300];
        std::snprintf(cmd, sizeof(cmd), "addr2line -f -C -i -e '%s' 0x%" PRIxPTR " 2>/dev/null",
                      oa.path.c_str(), oa.link_addr);
        if (FILE* f = popen(cmd, "r")) {
            char buf[512];
            std::vector<std::string> lines;
            while (std::fgets(buf, sizeof(buf), f)) {
                buf[std::strcspn(buf, "\n")] = 0;
                lines.push_back(buf);
            }
            pclose(f);
            // addr2line -i prints innermost first: function, file:line, then
            // its inlined-into callers. Take the innermost frame that is not
            // the lock guard and has a name.
            for (size_t i = 0; i + 1 < lines.size(); i += 2) {
                if (lines[i] == "??" || lines[i].find("LockGuard") != std::string::npos) continue;
                std::string loc = lines[i + 1];
                if (size_t slash = loc.rfind('/'); slash != std::string::npos) loc = loc.substr(slash + 1);
                line = lines[i] + " @ " + loc;
                break;
            }
        }
    }
    if (!line.empty()) return line;
    char off[48];
    std::snprintf(off, sizeof(off), " (+0x%" PRIxPTR ")", oa.found ? oa.link_addr : call);
    if (!name.empty()) return name + off + " [no line: addr2line gave none]";
    return std::string("unresolved") + off;
}

void reportLocks(double seconds) {
    struct Row { uint64_t wait_ns, contended; uintptr_t ra; };
    std::vector<Row> rows;
    for (const auto& s : sim::g_lock_sites) {
        const uintptr_t ra = s.ra.load();
        if (ra) rows.push_back({s.wait_ns.load(), s.contended.load(), ra});
    }
    std::sort(rows.begin(), rows.end(), [](const Row& a, const Row& b) { return a.wait_ns > b.wait_ns; });
    std::printf("lock contention (threads x seconds waiting, contended acquisitions, site):\n");
    if (rows.empty()) std::printf("  none\n");
    for (size_t i = 0; i < rows.size() && i < 10; ++i)
        std::printf("  %6.1f%%  %10" PRIu64 "  %s\n",
                    100.0 * static_cast<double>(rows[i].wait_ns) / 1e9 /
                        (seconds * opt.threads),
                    rows[i].contended, symbolize(rows[i].ra).c_str());
}

}  // namespace

int main(int argc, char** argv) {
    parseArgs(argc, argv);
    for (const auto& s : opt.sizes) g_weight_total += s.second;
    // The run cache reads its settings from the environment on first use.
    if (opt.run_cache_mib > 0) {
        char v[32];
        std::snprintf(v, sizeof(v), "%ld", opt.run_cache_mib);
        setenv("SMASH_RUN_CACHE_MIB", v, 1);
    }
    // Sub-arenas too (SMASH_SUBARENAS*, SMASH_DIRTY_*), read on first use.
    auto setNum = [](const char* name, long v) {
        char b[32];
        std::snprintf(b, sizeof(b), "%ld", v);
        setenv(name, b, 1);
    };
    if (opt.subarenas >= 0) {
        setenv("SMASH_SUBARENAS", "1", 1);
        setNum("SMASH_SUBARENAS_PER_NODE", opt.subarenas);
    }
    if (opt.decay_ms >= 0) setNum("SMASH_DIRTY_DECAY_MS", opt.decay_ms);
    if (opt.dirty_max_mib >= 0) setNum("SMASH_DIRTY_MAX_MIB", opt.dirty_max_mib);
    vm::g_sim_os_call_ns = opt.os_call_ns;

    const size_t arena_bytes = opt.arena_gib << 30;
    g_page_map.init();
    if (!g_vm.init(arena_bytes)) { std::fprintf(stderr, "VmRegion init failed\n"); return 2; }
    g_page_map.attachVmRegion(&g_vm);
    g_states.init(g_vm.totalPages());
    g_large.init(&g_page_map, &g_vm, &g_states, simRelease, nullptr);
    if (opt.check) {
        const size_t bytes = g_vm.contigPages() * sizeof(std::atomic<uint32_t>);
        void* m = mmap(nullptr, bytes, PROT_READ | PROT_WRITE,
                       MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
        if (m == MAP_FAILED) { std::fprintf(stderr, "shadow map failed\n"); return 2; }
        g_owner = static_cast<std::atomic<uint32_t>*>(m);
    }

    std::printf("smash_sim: %d threads, %d s, %d node(s), %zu GiB arena, run cache %ld MiB/CPU, "
                "OS call cost %" PRIu64 " ns, numa %.2f, cold %.2f, cross-free %.2f, live %d/thread, "
                "%zu size(s)%s\n",
                opt.threads, opt.seconds, opt.nodes, opt.arena_gib, opt.run_cache_mib,
                opt.os_call_ns, opt.numa, opt.cold, opt.cross_free, opt.live, opt.sizes.size(),
                opt.check ? ", checking" : "");
    if (opt.subarenas >= 0)
        std::printf("sub-arenas: %s per node, decay %s ms, dirty cap %s MiB each\n",
                    opt.subarenas ? std::to_string(opt.subarenas).c_str() : "one per CPU",
                    opt.decay_ms >= 0 ? std::to_string(opt.decay_ms).c_str() : "1000",
                    opt.dirty_max_mib >= 0 ? std::to_string(opt.dirty_max_mib).c_str() : "256");

    if (opt.pin) {
        loadNodeCpus();
        std::printf("pinning: thread t on node t %% %d's CPUs, %zu real node(s)%s\n", opt.nodes,
                    g_node_cpus.size(), g_node_cpus.empty() ? " — none readable, not pinned" : "");
    }
    g_inbox = std::vector<Inbox>(static_cast<size_t>(opt.threads));
    std::vector<WorkerResult> results(static_cast<size_t>(opt.threads));
    std::vector<std::thread> th;
    const auto t0 = std::chrono::steady_clock::now();
    for (int t = 0; t < opt.threads; ++t) th.emplace_back(worker, t, &results[t]);
    // A corrupted free map or a lock never released shows up as no progress:
    // report it and exit rather than hang (a planted double hand-out loops
    // PageRunMap::take forever under its lock, and every other thread spins).
    auto hang = [&](const char* when) {
        std::printf("\nFAIL: no progress %s — a hang (a corrupted free list or a lock "
                    "never released). Contended sites so far:\n", when);
        reportLocks(std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
        std::fflush(stdout);
        std::_Exit(3);
    };
    uint64_t last = 0;
    int stalled = 0;
    std::printf("ops/s:");
    for (int s = 0; s < opt.seconds; ++s) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        const uint64_t o = g_ops.load();
        std::printf(" %" PRIu64, o - last);
        std::fflush(stdout);
        stalled = (o == last) ? stalled + 1 : 0;
        if (stalled >= 3) hang("for 3 s during the run");
        last = o;
    }
    std::printf("\n");
    g_stop = true;
    std::atomic<bool> joined{false};
    std::thread watchdog([&] {
        for (int i = 0; i < 100 && !joined.load(); ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!joined.load()) hang("for 10 s after the workers were told to stop");
    });
    for (auto& x : th) x.join();
    joined = true;
    watchdog.join();
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    // Frees parked in inboxes whose owner already stopped.
    for (Inbox& in : g_inbox) for (const Block& b : in.blocks) freeBlock(b);

    uint64_t os_calls = 0;
    for (const auto& r : results) os_calls += r.os_calls;
    const uint64_t ops = g_ops.load();
    std::printf("throughput: %.0f allocate+free per second, %.2f us per op per thread; "
                "%.2f OS calls per op\n",
                static_cast<double>(ops) / secs,
                secs * opt.threads * 1e6 / static_cast<double>(ops ? ops : 1),
                static_cast<double>(os_calls) / static_cast<double>(ops ? ops : 1));
    std::printf("placement: %" PRIu64 " blocks outside the arena, %" PRIu64 " NULL\n",
                g_outside.load(), g_nulls.load());
    const size_t bump = g_vm.rawNextPage();
    std::printf("arena: %.2f of %zu GiB walked (bump), %.2f GiB free in the shared arena "
                "(longest run %zu pages)\n",
                static_cast<double>(bump * kPageSize) / (1ull << 30), opt.arena_gib,
                static_cast<double>(g_vm.arenaFreePages() * kPageSize) / (1ull << 30),
                g_vm.arenaLongestFreeRun());
    if (g_vm.runCacheEnabled()) {
        const auto rc = g_vm.runCacheStats();
        std::printf("run cache: takes=%" PRIu64 " hits=%" PRIu64 " (%.1f%%) puts=%" PRIu64
                    " evict=%" PRIu64 " trim=%" PRIu64 " declined_cold=%" PRIu64
                    " declined_big=%" PRIu64 "\n",
                    rc.takes, rc.hits, rc.takes ? 100.0 * rc.hits / rc.takes : 0.0, rc.puts,
                    rc.evictions, rc.trimmed, rc.declined_cold, rc.declined_big);
    }
    uint64_t dirty_bytes = 0, purged_runs = 0;
    if (g_vm.subArenaInfo(&dirty_bytes, &purged_runs))
        std::printf("sub-arenas: %.1f MiB dirty now, %" PRIu64 " runs purged\n",
                    static_cast<double>(dirty_bytes) / (1 << 20), purged_runs);
    reportLocks(secs);

    // Drain: everything is freed; return cached runs, process the queue, and
    // every carved page must be free again.
    g_vm.trimRunCache(/*all=*/true);
    for (int i = 0; i < 100 && g_vm.drainInFlight(); ++i) {
        g_vm.drainDecommitQueueNow();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    int failures = 0;
    // Sub-arenas: where their free pages sit as the run left them, before
    // anything is purged (every block is freed by now).
    if (opt.subarenas >= 0) {
        for (int n = -1; n < opt.nodes; ++n) {
            const auto b = g_vm.subSetBreakdown(n);
            if (!b.dirty && !b.clean && !b.tails && !b.pooled) continue;
            auto gib = [](size_t pages) { return static_cast<double>(pages * kPageSize) / (1ull << 30); };
            std::printf("%s %d free: dirty %.2f GiB, clean %.2f GiB (%zu runs, %zu whole chunks), "
                        "tails %.2f GiB, pooled %.2f GiB\n", n < 0 ? "shared set" : "node", n,
                        gib(b.dirty), gib(b.clean), b.clean_runs, b.whole_clean, gib(b.tails),
                        gib(b.pooled));
        }
    }
    // Sub-arenas: their free lists are consistent as the run left them (a
    // misrouted free is visible only now: a full purge turns whole chunks
    // clean and pools them, whoever held the runs), and a full purge leaves
    // nothing dirty (dirty runs count as free below, so a purge that drops
    // runs shows up there, one that skips them here).
    for (int n = -1; n < opt.nodes; ++n) {   // -1: the shared arena's set
        if (const int bad = g_vm.checkSubArenas(n)) {
            std::printf("FAIL: %s %d: %d sub-arena free-list problem(s) before the purge (above)\n",
                        n < 0 ? "shared set" : "node", n, bad);
            ++failures;
        }
    }
    g_vm.purgeAllDirtyNow();
    if (g_vm.subArenaInfo(&dirty_bytes, &purged_runs) && dirty_bytes != 0) {
        std::printf("FAIL: %.1f MiB still dirty after purging everything\n",
                    static_cast<double>(dirty_bytes) / (1 << 20));
        ++failures;
    }
    for (int n = -1; n < opt.nodes; ++n) {   // -1: the shared arena's set
        if (const int bad = g_vm.checkSubArenas(n, /*quiet=*/true)) {
            std::printf("FAIL: %s %d: %d sub-arena free-list problem(s) after the purge (above)\n",
                        n < 0 ? "shared set" : "node", n, bad);
            ++failures;
        }
    }
    size_t carved_total = 0;
    for (int n = 0; n < opt.nodes; ++n) {
        size_t carved = 0, free_pages = 0;
        if (!g_vm.nodeArenaInfo(static_cast<uint32_t>(n), &carved, &free_pages)) continue;
        carved_total += carved;
        std::printf("node %d: carved %zu pages, free %zu\n", n, carved, free_pages);
        if (carved != free_pages) {
            std::printf("FAIL: node %d: %zu pages carved but only %zu free after everything "
                        "was freed and drained — pages leaked\n", n, carved, free_pages);
            ++failures;
        }
    }
    const size_t shared_free = g_vm.arenaFreePages();
    if (bump != shared_free + carved_total) {
        std::printf("FAIL: %zu pages walked, but %zu free in the shared arena + %zu carved by "
                    "nodes after everything was freed — %zd pages unaccounted for\n",
                    bump, shared_free, carved_total,
                    static_cast<ssize_t>(bump) - static_cast<ssize_t>(shared_free + carved_total));
        ++failures;
    }
    if (g_double_handout) { std::printf("FAIL: %" PRIu64 " pages handed to two live blocks\n", g_double_handout.load()); ++failures; }
    if (g_owner_mismatch) { std::printf("FAIL: %" PRIu64 " freed pages owned by another block\n", g_owner_mismatch.load()); ++failures; }
    if (g_span_mismatch) { std::printf("FAIL: %" PRIu64 " frees whose page map entry did not resolve to the block's span\n", g_span_mismatch.load()); ++failures; }
    std::printf("invariants: %s%s\n", failures ? "FAILED" : "hold",
                opt.check ? "" : " (ownership not checked: --no-check)");
    return failures ? 1 : 0;
}
