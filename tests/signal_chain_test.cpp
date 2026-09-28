// signal_chain_test.cpp - an application's own SIGSEGV handler must never
// displace smash's.
//
// Signal handlers are per process: an application's sigaction(SIGSEGV)
// replaced smash's handler, and smash only took it back on its next
// compressor tick (0.3-3 s measured). In between, every fault smash has to
// resolve — a compressed page, a monitored (PROT_READ) page — went to the
// application's handler, which treats it as a crash. SMASH_SOFTDIRTY=0 makes
// such faults frequent, and an application on the 128-core aarch64 box
// crashed there. smash now interposes sigaction() and friends: for SIGSEGV
// and SIGBUS it records the application's handler as the one to pass
// foreign faults to, reports it back on queries, and leaves its own handler
// in the kernel.
//
//   A. install a handler, then at once write to a page of a live smash block
//      the test made read-only itself (an ACTIVE page: smash restores it) —
//      smash must resolve it; the application's handler must not see it
//   B. a fault on the test's own PROT_NONE page reaches the application's
//      handler, with its address
//   C. sigaction() queries return the application's handler; signal()
//      returns the previous one
//   D. the kernel's handler (read with the raw rt_sigaction syscall) is
//      still smash's
//   E. SA_RESETHAND: after one foreign fault the handler reads back SIG_DFL;
//      SIGUSR1 (not smash's) is installed and delivered as usual
//   F. 4 threads keep swapping SIGSEGV handlers while 8 threads take smash
//      faults and faults of their own for 1.5 s: every smash fault resolved,
//      every own fault delivered to an application handler (the handler
//      smash chains to is rewritten while faults read it)
#include <atomic>
#include <csetjmp>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

int failures = 0;
void fail(const char* what) {
    std::printf("FAIL: %s\n", what);
    ++failures;
}

sigjmp_buf g_jmp;
volatile sig_atomic_t g_app_faults = 0;
void* volatile g_app_addr = nullptr;

void appHandler(int, siginfo_t* si, void*) {
    g_app_faults = g_app_faults + 1;
    g_app_addr = si->si_addr;
    siglongjmp(g_jmp, 1);
}
void appHandler2(int) { siglongjmp(g_jmp, 2); }

volatile sig_atomic_t g_usr1 = 0;
void onUsr1(int) { g_usr1 = 1; }

thread_local sigjmp_buf tl_jmp;
std::atomic<long> g_h1{0}, g_h2{0};
void stressA(int, siginfo_t*, void*) { g_h1.fetch_add(1, std::memory_order_relaxed); siglongjmp(tl_jmp, 1); }
void stressB(int, siginfo_t*, void*) { g_h2.fetch_add(1, std::memory_order_relaxed); siglongjmp(tl_jmp, 1); }

// The kernel's current handler for `sig`, bypassing any interposed wrapper.
void* kernelHandler(int sig) {
    struct KSigaction { void* handler; unsigned long flags; void* restorer; unsigned long mask; } k{};
    if (syscall(SYS_rt_sigaction, sig, nullptr, &k, sizeof(k.mask)) != 0) return nullptr;
    return k.handler;
}

}  // namespace

int main() {
    auto in_arena = reinterpret_cast<int (*)(const void*)>(dlsym(RTLD_DEFAULT, "smash_in_arena"));
    if (!in_arena) { std::printf("SKIP: not running under libsmash\n"); return 0; }
    const long ps = sysconf(_SC_PAGESIZE);

    auto* block = static_cast<char*>(malloc(8u << 20));
    if (!block || !in_arena(block)) { std::printf("SKIP: the block is not in the arena\n"); return 0; }
    std::memset(block, 1, 8u << 20);
    void* const smash_handler = kernelHandler(SIGSEGV);

    // A.
    struct sigaction sa{};
    sa.sa_sigaction = appHandler;
    sa.sa_flags = SA_SIGINFO;
    sigemptyset(&sa.sa_mask);
    struct sigaction before{};
    if (sigaction(SIGSEGV, &sa, &before) != 0) fail("sigaction(SIGSEGV) failed");
    mprotect(block, static_cast<size_t>(ps), PROT_READ);
    if (sigsetjmp(g_jmp, 1) == 0) {
        block[0] = 2;   // smash's page, smash's fault
        std::printf("A. write to a smash page right after installing a handler: resolved by smash\n");
    } else {
        fail("A. the application's handler received a fault on a smash page");
    }
    if (block[0] != 2) fail("A. the write did not land");

    // B.
    auto* own = static_cast<char*>(mmap(nullptr, static_cast<size_t>(ps), PROT_NONE,
                                        MAP_PRIVATE | MAP_ANONYMOUS, -1, 0));
    const int faults0 = g_app_faults;
    if (sigsetjmp(g_jmp, 1) == 0) {
        *reinterpret_cast<volatile char*>(own) = 1;
        fail("B. writing to a PROT_NONE page did not fault");
    }
    std::printf("B. own fault: application handler ran %d time(s), address %s\n",
                g_app_faults - faults0, g_app_addr == own ? "correct" : "WRONG");
    if (g_app_faults - faults0 != 1 || g_app_addr != own)
        fail("B. the application's own fault did not reach its handler with its address");

    // C.
    struct sigaction cur{};
    sigaction(SIGSEGV, nullptr, &cur);
    if (!(cur.sa_flags & SA_SIGINFO) || cur.sa_sigaction != appHandler)
        fail("C. sigaction() query did not return the application's handler");
    auto prev = signal(SIGSEGV, appHandler2);
    if (reinterpret_cast<void*>(prev) != reinterpret_cast<void*>(appHandler))
        fail("C. signal() did not return the previous handler");
    if (sigsetjmp(g_jmp, 1) == 0) {
        *reinterpret_cast<volatile char*>(own) = 1;
        fail("C. fault did not reach the handler installed with signal()");
    } else {
        std::printf("C. queries and signal() see the application's handlers\n");
    }

    // D.
    void* now = kernelHandler(SIGSEGV);
    std::printf("D. kernel handler %s smash's\n", now == smash_handler ? "is still" : "is NOT");
    if (now != smash_handler || now == reinterpret_cast<void*>(appHandler) ||
        now == reinterpret_cast<void*>(appHandler2))
        fail("D. the application's handler replaced smash's in the kernel");

    // E.
    struct sigaction once{};
    once.sa_sigaction = appHandler;
    once.sa_flags = SA_SIGINFO | SA_RESETHAND;
    sigemptyset(&once.sa_mask);
    sigaction(SIGSEGV, &once, nullptr);
    if (sigsetjmp(g_jmp, 1) == 0) *reinterpret_cast<volatile char*>(own) = 1;
    // The kernel resets only the handler (the flags, SA_SIGINFO included,
    // stay), and SIG_DFL is decided by the handler value.
    sigaction(SIGSEGV, nullptr, &cur);
    if (cur.sa_handler != SIG_DFL)
        fail("E. an SA_RESETHAND handler did not reset to SIG_DFL after one fault");
    signal(SIGUSR1, onUsr1);
    raise(SIGUSR1);
    if (!g_usr1) fail("E. SIGUSR1 was not delivered to its handler");
    std::printf("E. SA_RESETHAND reset, SIGUSR1 delivered: %s\n",
                failures ? "(see above)" : "yes");

    // F.
    {
        std::atomic<bool> stop{false};
        std::atomic<long> resolved{0}, delivered{0}, misrouted{0};
        std::vector<std::thread> th;
        for (int t = 0; t < 4; ++t) {
            th.emplace_back([&, t] {
                struct sigaction a{}, b{};
                a.sa_sigaction = stressA;
                a.sa_flags = SA_SIGINFO;
                sigemptyset(&a.sa_mask);
                b = a;
                b.sa_sigaction = stressB;
                for (long i = 0; !stop.load(std::memory_order_relaxed); ++i)
                    sigaction(SIGSEGV, (i + t) & 1 ? &a : &b, nullptr);
            });
        }
        for (int t = 0; t < 8; ++t) {
            th.emplace_back([&] {
                auto* blk = static_cast<char*>(malloc(4u << 20));
                std::memset(blk, 1, 4u << 20);
                auto* mine = static_cast<char*>(mmap(nullptr, static_cast<size_t>(ps), PROT_NONE,
                                                     MAP_PRIVATE | MAP_ANONYMOUS, -1, 0));
                while (!stop.load(std::memory_order_relaxed)) {
                    mprotect(blk, static_cast<size_t>(ps), PROT_READ);
                    if (sigsetjmp(tl_jmp, 1) == 0) { blk[0] = 3; ++resolved; }
                    else ++misrouted;   // smash's fault reached an application handler
                    if (sigsetjmp(tl_jmp, 1) == 0) { *reinterpret_cast<volatile char*>(mine) = 1; ++misrouted; }
                    else ++delivered;
                }
                munmap(mine, static_cast<size_t>(ps));
                free(blk);
            });
        }
        usleep(1500 * 1000);
        stop = true;
        for (auto& x : th) x.join();
        std::printf("F. handlers swapped by 4 threads: %ld smash faults resolved, %ld own faults "
                    "delivered, %ld misrouted\n", resolved.load(), delivered.load(), misrouted.load());
        if (misrouted || !resolved || !delivered)
            fail("F. a fault went to the wrong handler while handlers were being swapped");
    }

    sigaction(SIGSEGV, &before, nullptr);
    mprotect(block, static_cast<size_t>(ps), PROT_READ | PROT_WRITE);
    free(block);
    munmap(own, static_cast<size_t>(ps));
    if (failures) { std::printf("signal_chain_test: %d FAILURES\n", failures); return 1; }
    std::printf("signal_chain_test: all checks passed\n");
    return 0;
}
