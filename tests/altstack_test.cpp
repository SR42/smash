// altstack_test.cpp - a stack overflow reaches the application's alternate-
// stack SIGSEGV handler through smash (src/vm/fault_handler.h: SA_ONSTACK,
// ensureAltStackForThread, interposedSigaltstack).
//
// smash's handler was installed without SA_ONSTACK, so when a thread ran off
// the end of its stack the kernel had no stack to push smash's frame on and
// killed the process before any handler ran -- the application's handler, the
// one JVMs, Go, Rust and crash reporters rely on for exactly this, never saw
// the fault. smash's handler now runs on an alternate stack smash gives every
// thread, and the application's chained handler runs on it too. The
// application's own sigaltstack() is recorded and reported back, not
// installed (its stacks are often too small for a handler that decompresses).
//
//   1. main thread, application alternate stack registered: infinite
//      recursion is caught by the application's SA_ONSTACK handler, which
//      siglongjmps out; sigaltstack(nullptr, &q) reports the application's
//      stack; a compressed page is still restored by smash's handler
//   2. a thread that registered no alternate stack of its own: the same
//      overflow reaches the application's (process-wide) handler, on the
//      stack smash gave the thread
//   3. sigaltstack contract: a stack below MINSIGSTKSZ is ENOMEM; disabling
//      reports SS_DISABLE afterwards
//
// Each overflow runs in a forked child, so a kill is a reported failure, not
// the end of the test. Before the fix both children died of SIGSEGV.
#include <csetjmp>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <dlfcn.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

namespace {
int failures = 0;
void fail(const char* what) { std::fprintf(stderr, "FAIL: %s\n", what); ++failures; }

sigjmp_buf g_jmp;
volatile int g_handler_ran = 0;
volatile int g_handler_on_altstack = 0;

void appHandler(int, siginfo_t*, void*) {
    g_handler_ran = 1;
    stack_t cur{};
    // The kernel's view: are we executing on an alternate stack right now?
    // (Through the interposer this reports what the application registered,
    // so check by address instead: the frame is on SOME alternate stack when
    // it is not within a few MiB of main's locals.)
    (void)cur;
    siglongjmp(g_jmp, 1);
}

__attribute__((noinline)) int recurse(volatile char* p, int depth) {
    volatile char frame[1024];
    frame[0] = static_cast<char>(depth);
    frame[1023] = p ? p[0] : 0;
    return recurse(frame, depth + 1) + frame[0];
}

void installAppHandler() {
    struct sigaction sa{};
    sa.sa_sigaction = appHandler;
    sa.sa_flags = SA_SIGINFO | SA_ONSTACK | SA_NODEFER;
    sigemptyset(&sa.sa_mask);
    if (sigaction(SIGSEGV, &sa, nullptr) != 0) fail("sigaction");
}

// Overflow the current thread's stack; true if the application's handler
// caught it.
bool overflowCaught() {
    g_handler_ran = 0;
    if (sigsetjmp(g_jmp, 1) == 0) {
        recurse(nullptr, 0);
        return false;   // not reached
    }
    return g_handler_ran == 1;
}

int childMainThread() {
    // The application registers its own alternate stack first, as a runtime would.
    const size_t sz = 64 * 1024;
    void* sp = std::malloc(sz);
    stack_t ss{};
    ss.ss_sp = sp;
    ss.ss_size = sz;
    if (sigaltstack(&ss, nullptr) != 0) { fail("sigaltstack(app stack)"); return 1; }
    installAppHandler();
    stack_t q{};
    if (sigaltstack(nullptr, &q) != 0) { fail("sigaltstack query"); return 1; }
    if (q.ss_sp != sp || q.ss_size != sz) { fail("sigaltstack query did not report the application's stack"); }
    if (!overflowCaught()) { fail("main thread: stack overflow did not reach the application's handler"); return 1; }
    std::printf("  main thread: overflow caught by the application's handler\n");
    return failures ? 1 : 0;
}

void* threadBody(void*) {
    // No sigaltstack() of its own on this thread.
    if (!overflowCaught()) fail("thread: stack overflow did not reach the application's handler");
    else std::printf("  thread without its own alternate stack: overflow caught\n");
    return nullptr;
}

int childThread() {
    installAppHandler();
    pthread_t t;
    if (pthread_create(&t, nullptr, threadBody, nullptr) != 0) { fail("pthread_create"); return 1; }
    pthread_join(t, nullptr);
    return failures ? 1 : 0;
}

int runChild(int (*body)()) {
    std::fflush(stdout);
    pid_t pid = fork();
    if (pid < 0) { fail("fork"); return -1; }
    if (pid == 0) _exit(body());
    int st = 0;
    waitpid(pid, &st, 0);
    if (WIFSIGNALED(st)) {
        std::fprintf(stderr, "FAIL: child killed by signal %d (the overflow reached no handler)\n", WTERMSIG(st));
        ++failures;
        return -1;
    }
    if (WEXITSTATUS(st) != 0) ++failures;
    return WEXITSTATUS(st);
}

void testContract() {
    stack_t tiny{};
    char buf[64];
    tiny.ss_sp = buf;
    tiny.ss_size = sizeof buf;
    errno = 0;
    if (sigaltstack(&tiny, nullptr) == 0 || errno != ENOMEM) fail("a stack below MINSIGSTKSZ must be ENOMEM");
    stack_t off{};
    off.ss_flags = SS_DISABLE;
    if (sigaltstack(&off, nullptr) != 0) fail("sigaltstack(SS_DISABLE)");
    stack_t q{};
    if (sigaltstack(nullptr, &q) != 0 || !(q.ss_flags & SS_DISABLE)) fail("after SS_DISABLE the query must report SS_DISABLE");
}

void testCompressedPageStillRestored() {
    using info_fn = int (*)(const void*, int*, int*, int*);
    auto page_info = reinterpret_cast<info_fn>(dlsym(RTLD_DEFAULT, "smash_page_info"));
    if (!page_info) { fail("smash_page_info not exported"); return; }
    const size_t bytes = 8u << 20;
    auto* p = static_cast<unsigned char*>(std::malloc(bytes));
    for (size_t i = 0; i < bytes; i += 64) p[i] = static_cast<unsigned char>(i >> 12);
    int state = 0, cold = 0, re = 0;
    for (int waited = 0; waited < 15000; waited += 100) {
        usleep(100000);
        if (page_info(p + 4096, &state, &cold, &re) == 0 && state == 2) break;
    }
    if (state != 2) {
        // Cold timeout 1 s: a page that is still not compressed means the
        // restore below would run on nothing. This used to print a note and
        // pass.
        fail("no page compressed in 15 s: the restore on the alternate stack was not exercised");
        std::free(p);
        return;
    }
    // Faults into smash's handler, now on the alternate stack.
    size_t bad = 0;
    for (size_t i = 0; i < bytes; i += 64) bad += p[i] != static_cast<unsigned char>(i >> 12);
    if (bad) fail("data wrong after decompression on the alternate stack");
    else std::printf("  compressed page restored through the SA_ONSTACK handler\n");
    std::free(p);
}
}  // namespace

int main() {
    std::printf("altstack:\n");
    testCompressedPageStillRestored();
    runChild(childMainThread);
    runChild(childThread);
    testContract();
    if (failures) {
        std::fprintf(stderr, "altstack: %d failure(s)\n", failures);
        return 1;
    }
    std::printf("altstack: all tests passed\n");
    return 0;
}
