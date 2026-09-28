// sigtrace_classify_test.cpp - SMASH_SIGTRACE=1 must say what smash thinks a
// fault's address is, on the path where the application's own handler takes
// the fault next.
//
// When smash does not resolve a fault it passes it to the handler that was
// installed before it re-claimed SIGSEGV. If that handler is the
// application's, smash's "[smash FATAL]" line (which classifies the address)
// never prints, and the one trace left is the SMASH_SIGTRACE line. An
// application crashing under SMASH_SOFTDIRTY=0 on the 128-core aarch64 box
// showed nothing smash-related for exactly that reason, so the trace line now
// carries the same classification.
//
// The test installs its own SIGSEGV handler after smash, waits until smash's
// compressor re-claims SIGSEGV (it checks on every tick), then writes to a
// freed smash block it has made read-only itself — a fault smash knows the
// page of but does not resolve. Registered with PASS_REGULAR_EXPRESSION on the
// classified trace line.
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <sys/mman.h>
#include <unistd.h>

static void onSegv(int, siginfo_t* si, void*) {
    char b[96];
    const int n = snprintf(b, sizeof(b), "application handler: SIGSEGV at %p\n", si->si_addr);
    (void)!write(1, b, static_cast<size_t>(n));
    _exit(0);
}

int main() {
    auto in_arena = reinterpret_cast<int (*)(const void*)>(dlsym(RTLD_DEFAULT, "smash_in_arena"));
    if (!in_arena) { std::printf("SKIP: not running under libsmash\n"); return 0; }
    auto* p = static_cast<char*>(malloc(8u << 20));
    if (!p || !in_arena(p)) { std::printf("SKIP: the block is not in the arena\n"); return 0; }
    std::memset(p, 1, 8u << 20);
    free(p);
    usleep(200 * 1000);
    mprotect(p, 4096, PROT_READ);   // a freed smash page, read-only: not smash's to fix

    struct sigaction sa{};
    sa.sa_sigaction = onSegv;
    sa.sa_flags = SA_SIGINFO;
    sigaction(SIGSEGV, &sa, nullptr);
    // Until smash re-claims SIGSEGV, a fault would go straight to onSegv.
    bool reclaimed = false;
    for (int i = 0; i < 200 && !reclaimed; ++i) {
        usleep(50 * 1000);
        struct sigaction cur{};
        sigaction(SIGSEGV, nullptr, &cur);
        reclaimed = cur.sa_sigaction != onSegv;
    }
    if (!reclaimed) { std::printf("SKIP: smash did not re-claim SIGSEGV within 10 s\n"); return 0; }
    std::fflush(stdout);
    *reinterpret_cast<volatile char*>(p) = 2;
    std::printf("FAIL: the write to a read-only page did not fault\n");
    return 1;
}
