#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
// vma_exhaust_test.cpp - restoring a compressed page when the kernel has no
// VMA to give (src/compress/compressor_thread.h restorePageContents).
//
// A restore writes the bytes through /proc/self/mem and then mprotect()s the
// page PROT_RW, which splits a VMA; at vm.max_map_count that fails with
// ENOMEM, as do the fallbacks. The handler then returned with the page still
// PROT_NONE + COMPRESSED, the faulting instruction re-executed, and the
// thread spun at full speed -- silently, until an unmap elsewhere freed a
// VMA, or for ever. The restore now waits for a VMA in 5 ms steps for up to
// 2 s, and if none comes it says so and aborts.
//
// The parent compresses a block; each case runs in a forked child, which
// inherits the compressed pages and exhausts the process's VMAs with one-page
// mappings of alternating protection (so none merge), then touches a page in
// the middle of the compressed block (its VMA must split: two new VMAs).
//
//   1. recovery: a thread unmaps the fillers 300 ms later; the touch must
//      complete, and the waiting thread must not have burnt CPU meanwhile
//      (under 150 ms of CPU for a 300 ms wait; the spin burnt all of it)
//   2. no recovery: nothing is unmapped; the child must abort with the
//      FATAL message within 5 s rather than spin for ever
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
info_fn g_page_info = nullptr;
const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));

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

unsigned char* g_block = nullptr;   // compressed by the parent, inherited by each child

int childRecovery() {
    unsigned char* p = g_block;
    // The freer exists before the VMAs run out: a thread's stack is a mapping.
    std::vector<void*> fillers;
    std::atomic<bool> exhausted{false};
    std::thread freer([&] {
        while (!exhausted.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        for (size_t i = 0; i < fillers.size(); i += 2) munmap(fillers[i], ps);
    });
    fillers = exhaustVmas();
    std::fprintf(stderr, "  recovery child: %zu filler mappings\n", fillers.size());
    exhausted.store(true);
    const double cpu0 = cpuSeconds();
    const auto t0 = std::chrono::steady_clock::now();
    volatile unsigned char v = p[(2u << 20) + 1];   // a page in the middle
    const double waited_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    const double cpu_ms = (cpuSeconds() - cpu0) * 1000;
    freer.join();
    std::fprintf(stderr, "  recovery child: the touch took %.0f ms of wall time and %.0f ms of CPU\n", waited_ms, cpu_ms);
    if (v != 0x5A) { std::fprintf(stderr, "FAIL: restored byte wrong\n"); return 1; }
    if (waited_ms < 200) std::fprintf(stderr, "  (the touch did not stall; the VMA split did not fail -- case not exercised)\n");
    else if (cpu_ms > 150) { std::fprintf(stderr, "FAIL: %.0f ms of CPU burnt waiting for a VMA (a spin, not a wait)\n", cpu_ms); return 1; }
    return 0;
}

int childNoRecovery() {
    unsigned char* p = g_block;
    auto fillers = exhaustVmas();
    std::fprintf(stderr, "  no-recovery child: %zu filler mappings; touching\n", fillers.size());
    volatile unsigned char v = p[(2u << 20) + 1];
    (void)v;
    // Reached only if the split did not fail (not exercised) -- exit 3 so the
    // parent can tell.
    return 3;
}

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
}  // namespace

int main() {
    g_page_info = reinterpret_cast<info_fn>(dlsym(RTLD_DEFAULT, "smash_page_info"));
    if (!g_page_info) { std::fprintf(stderr, "FAIL: smash_page_info not exported\n"); return 1; }
    std::printf("vma_exhaust:\n");
    // Compressed here, where the compressor runs; a fork child inherits the
    // PROT_NONE pages, the blobs and the fault handler (the compressor itself
    // only restarts in a child once it creates a thread).
    g_block = compressedBlock(4u << 20);
    if (!g_block) { fail("the block did not compress"); return 1; }

    int st = runChild(childRecovery, 60);
    if (st == -1) fail("recovery child hung");
    else if (WIFSIGNALED(st)) fail("recovery child died of a signal");
    else if (WEXITSTATUS(st) != 0) fail("recovery child failed");
    else std::printf("  recovery: the touch completed once a VMA came free\n");

    st = runChild(childNoRecovery, 8);
    if (st == -1) fail("no-recovery child spun for 8 s instead of aborting");
    else if (WIFSIGNALED(st) && WTERMSIG(st) == SIGABRT) std::printf("  no recovery: the child aborted with the FATAL message\n");
    else if (WIFEXITED(st) && WEXITSTATUS(st) == 3) std::printf("  (no-recovery case not exercised: the split did not fail)\n");
    else fail("no-recovery child ended some other way");

    if (failures) { std::fprintf(stderr, "vma_exhaust: %d failure(s)\n", failures); return 1; }
    std::printf("vma_exhaust: all tests passed\n");
    return 0;
}
