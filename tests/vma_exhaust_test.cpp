#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
// vma_exhaust_test.cpp - restoring a compressed page when the kernel has no
// VMA to give (src/compress/compressor_thread.h: restorePageContents,
// restoreWholeVma, capStall).
//
// A restore writes the bytes through /proc/self/mem and then mprotect()s the
// page PROT_RW, which splits a VMA; at vm.max_map_count that fails with
// ENOMEM, as do the fallbacks. History:
//   - the handler returned with the page still PROT_NONE + COMPRESSED and the
//     thread spun at full speed, silently, until an unmap elsewhere freed a
//     VMA, or for ever (and marked the page ACTIVE, losing its data);
//   - then the restore waited for a VMA under the page lock, 5 ms steps, and
//     aborted after 2 s. Nothing frees VMAs on a schedule, so at the cap that
//     was an abort;
//   - now (upstream #94, ported) the handler restores the page's whole VMA,
//     which needs no split. Only if that cannot be done (a page of the VMA
//     busy) does the access re-fault and retry, sleeping, for up to 2 s, and
//     then the thread says so and aborts.
//
// The parent compresses a 4 MiB block (one PROT_NONE VMA); each case runs in
// a forked child, which inherits the compressed pages and exhausts the
// process's VMAs with one-page mappings of alternating protection (so none
// merge), then touches a page in the middle of the block (a single-page
// restore must split its VMA: two new VMAs).
//
//   1. at the cap: the touch must complete at once with the right byte (the
//      whole block comes back as one VMA) and cost under 150 ms of CPU. The
//      library before aborted after 2 s. Reading /proc/self/maps to find the
//      VMA cost ~300 ms at a 1M-map cap; the run from page states costs ~4.
//   2. stall: another thread holds the lock of a page in the same VMA for
//      300 ms (smash_debug_hold_page_lock), so the whole-VMA restore cannot
//      take it; the touch must complete once it is released, and the waiting
//      thread must not have burnt CPU meanwhile (under 150 ms for a 300 ms
//      wait: a spin burns all of it).
//   3. no recovery: the lock is held for 10 s; the child must abort with the
//      FATAL message within 8 s rather than spin for ever.
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {
int failures = 0;
void fail(const char* what) { std::fprintf(stderr, "FAIL: %s\n", what); ++failures; }
using info_fn = int (*)(const void*, int*, int*, int*);
using hold_fn = int (*)(const void*, unsigned);
info_fn g_page_info = nullptr;
hold_fn g_hold_page_lock = nullptr;
const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
constexpr size_t kBlock = 4u << 20;
constexpr size_t kTouch = (2u << 20) + 1;   // a byte in a page in the middle

double cpuSeconds() {
    rusage r{};
    getrusage(RUSAGE_THREAD, &r);   // this thread alone: the child has compressor threads of its own
    return r.ru_utime.tv_sec + r.ru_stime.tv_sec + (r.ru_utime.tv_usec + r.ru_stime.tv_usec) / 1e6;
}

// A block, compressed through and through (every page PROT_NONE, one VMA).
unsigned char* compressedBlock(size_t bytes) {
    auto* p = static_cast<unsigned char*>(std::malloc(bytes));
    for (size_t i = 0; i < bytes; i += 64) p[i] = static_cast<unsigned char>(i >> 12), p[i + 1] = 0x5A;
    for (int waited = 0; waited < 30000; waited += 200) {
        usleep(200000);
        size_t compressed = 0, pages = bytes / ps;
        for (size_t i = 0; i < pages; ++i) {
            int st = 0, c = 0, r = 0;
            if (g_page_info(p + i * ps, &st, &c, &r) == 0 && st == 2) ++compressed;
        }
        if (compressed == pages) return p;
    }
    std::fprintf(stderr, "block did not compress fully in 30 s\n");
    return nullptr;
}

// One-page mappings of alternating protection until mmap says ENOMEM
// (vm.max_map_count: 65530 by default, 1048576 on some distributions).
std::vector<void*> exhaustVmas() {
    size_t limit = 65530;
    if (FILE* f = std::fopen("/proc/sys/vm/max_map_count", "r")) {
        unsigned long v = 0;
        if (std::fscanf(f, "%lu", &v) == 1 && v) limit = v;
        std::fclose(f);
    }
    std::vector<void*> fillers;
    fillers.reserve(limit + 16);
    for (;;) {
        const int prot = (fillers.size() & 1) ? PROT_READ : PROT_NONE;
        void* m = mmap(nullptr, ps, prot, MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
        if (m == MAP_FAILED) break;
        fillers.push_back(m);
        if (fillers.size() > limit + 1000) break;   // the limit is not being enforced
    }
    return fillers;
}

// True if a fresh one-page split of a mapping fails now: the precondition of
// every case (otherwise the restore never meets the cap).
bool atCap() {
    void* m = mmap(nullptr, 3 * ps, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
    if (m == MAP_FAILED) return true;
    const bool split_failed = mprotect(static_cast<char*>(m) + ps, ps, PROT_READ) != 0;
    munmap(m, 3 * ps);
    return split_failed;
}

unsigned char* g_block = nullptr;   // compressed by the parent, inherited by each child

// Exit codes: 0 pass, 1 fail, 3 not exercised (the cap was not reached).
int childAtCap() {
    unsigned char* p = g_block;
    auto fillers = exhaustVmas();
    std::fprintf(stderr, "  at-cap child: %zu filler mappings\n", fillers.size());
    if (!atCap()) { std::fprintf(stderr, "  (a split still succeeds: not at the cap)\n"); return 3; }
    const double cpu0 = cpuSeconds();
    const auto t0 = std::chrono::steady_clock::now();
    volatile unsigned char v = p[kTouch];
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    const double cpu_ms = (cpuSeconds() - cpu0) * 1000;
    std::fprintf(stderr, "  at-cap child: the touch took %.0f ms of wall time and %.0f ms of CPU\n", ms, cpu_ms);
    if (v != 0x5A) { std::fprintf(stderr, "FAIL: restored byte wrong\n"); return 1; }
    for (size_t i = 0; i < kBlock; i += 64) {
        if (p[i] != static_cast<unsigned char>(i >> 12) || p[i + 1] != 0x5A) {
            std::fprintf(stderr, "FAIL: byte at offset %zu wrong after the restore\n", i);
            return 1;
        }
    }
    if (cpu_ms > 150) { std::fprintf(stderr, "FAIL: %.0f ms of CPU for one restore\n", cpu_ms); return 1; }
    return 0;
}

// A thread made before the VMAs run out (a thread stack is a mapping) holds
// the lock of the block's first page for `hold_ms` once told to.
int childHeld(unsigned hold_ms) {
    unsigned char* p = g_block;
    std::atomic<int> phase{0};   // 1: VMAs exhausted, 2: lock taken (approximately)
    std::thread holder([&] {
        while (phase.load() == 0) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        phase.store(2);
        if (g_hold_page_lock(p, hold_ms) != 0) std::fprintf(stderr, "  (could not hold the page lock)\n");
    });
    auto fillers = exhaustVmas();
    std::fprintf(stderr, "  held-lock child (%u ms): %zu filler mappings\n", hold_ms, fillers.size());
    const bool cap = atCap();
    phase.store(1);
    while (phase.load() != 2) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));   // the holder is inside the call
    if (!cap) { holder.join(); std::fprintf(stderr, "  (a split still succeeds: not at the cap)\n"); return 3; }
    const double cpu0 = cpuSeconds();
    const auto t0 = std::chrono::steady_clock::now();
    volatile unsigned char v = p[kTouch];
    const double waited_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    const double cpu_ms = (cpuSeconds() - cpu0) * 1000;
    holder.join();
    std::fprintf(stderr, "  held-lock child: the touch took %.0f ms of wall time and %.0f ms of CPU\n",
                 waited_ms, cpu_ms);
    if (v != 0x5A) { std::fprintf(stderr, "FAIL: restored byte wrong\n"); return 1; }
    if (waited_ms < 150) { std::fprintf(stderr, "  (the touch did not wait for the lock: not exercised)\n"); return 3; }
    if (cpu_ms > 150) { std::fprintf(stderr, "FAIL: %.0f ms of CPU burnt waiting (a spin, not a wait)\n", cpu_ms); return 1; }
    return 0;
}
int childStall() { return childHeld(300); }
int childNoRecovery() { return childHeld(10000); }

// Runs `body` in a child; returns the wait status, or -1 after `timeout_s`.
int runChild(int (*body)(), int timeout_s) {
    std::fflush(stdout);
    pid_t pid = fork();
    if (pid < 0) { fail("fork"); return -1; }
    if (pid == 0) _exit(body());
    for (int waited = 0; waited < timeout_s * 100; ++waited) {
        int st = 0;
        if (waitpid(pid, &st, WNOHANG) == pid) return st;
        usleep(10000);
    }
    kill(pid, SIGKILL);
    waitpid(pid, nullptr, 0);
    return -1;
}

void expectPass(const char* name, int st) {
    char msg[160];
    if (st == -1) { std::snprintf(msg, sizeof msg, "%s child hung", name); fail(msg); }
    else if (WIFSIGNALED(st)) { std::snprintf(msg, sizeof msg, "%s child died of signal %d", name, WTERMSIG(st)); fail(msg); }
    else if (WEXITSTATUS(st) == 3) { std::snprintf(msg, sizeof msg, "%s case not exercised", name); fail(msg); }
    else if (WEXITSTATUS(st) != 0) { std::snprintf(msg, sizeof msg, "%s child failed", name); fail(msg); }
    else std::printf("  %s: passed\n", name);
}
}  // namespace

int main() {
    g_page_info = reinterpret_cast<info_fn>(dlsym(RTLD_DEFAULT, "smash_page_info"));
    g_hold_page_lock = reinterpret_cast<hold_fn>(dlsym(RTLD_DEFAULT, "smash_debug_hold_page_lock"));
    if (!g_page_info || !g_hold_page_lock) {
        std::fprintf(stderr, "FAIL: smash_page_info / smash_debug_hold_page_lock not exported\n");
        return 1;
    }
    std::printf("vma_exhaust:\n");
    // Compressed here, where the compressor runs; a fork child inherits the
    // PROT_NONE pages, the blobs and the fault handler (the compressor itself
    // only restarts in a child once it creates a thread).
    g_block = compressedBlock(kBlock);
    if (!g_block) { fail("the block did not compress"); return 1; }

    // A case that cannot reach the cap fails rather than passing unexercised:
    // a skip here is how this test's previous version stopped guarding
    // anything without anyone noticing.
    expectPass("at-cap", runChild(childAtCap, 60));
    expectPass("stall", runChild(childStall, 60));

    const int st = runChild(childNoRecovery, 8 + 4);
    if (st == -1) fail("no-recovery child spun instead of aborting");
    else if (WIFSIGNALED(st) && WTERMSIG(st) == SIGABRT) std::printf("  no recovery: the child aborted with the FATAL message\n");
    else if (WIFEXITED(st) && WEXITSTATUS(st) == 3) fail("no-recovery case not exercised");
    else fail("no-recovery child ended some other way");

    if (failures) { std::fprintf(stderr, "vma_exhaust: %d failure(s)\n", failures); return 1; }
    std::printf("vma_exhaust: all tests passed\n");
    return 0;
}
