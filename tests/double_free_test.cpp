// double_free_test.cpp - opt-in double-free detection.
//
// A double free is undefined behaviour in C and C++, so detecting it is a
// debugging aid, not a conformance requirement: glibc aborts ("free(): double
// free detected in tcache 2"), smash historically absorbed it silently. The
// check is therefore OFF by default — no correct program can trigger it, so no
// correct program should pay for it — and enabled with
// SMASH_DOUBLE_FREE=warn|abort.
//
// Where the detection lives matters. A plain `free(p); free(p);` never reaches
// the slab: both pointers land in the per-thread cache lane, which is why
// ThreadCache::deallocate scans the lane when the policy is enabled. The slab
// bitmap check (Span::deallocate, 1 = free) catches the rarer case that
// survives a lane drain, and a free of an interior pointer that maps onto an
// already-free slot.
//
// Asserted here:
//   1. warn  - the repeated free is reported, dropped, and the allocator stays
//              consistent afterwards (dropping rather than applying it is what
//              keeps the lane from holding the same pointer twice);
//   2. unset - silent, i.e. the default really is off;
//   3. abort - the process dies by signal.
//
// The offending free happens in a forked child so the abort policy cannot take
// the test process down, and so stderr can be inspected.
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

namespace {

constexpr int kProbe = 4096;   // same-size-class allocations after the double free

int runChild() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    char* c = static_cast<char*>(malloc(3));
    if (!c) return 2;
    free(c);
    free(c);                                    // deliberate double free
    std::printf("SURVIVED\n");

    // If the second free was dropped, the lane holds this pointer once and the
    // same address is never handed to two callers. If it was applied (policy
    // off), duplicates are possible — which is why only the warn case asserts
    // consistency.
    static void* v[kProbe];
    for (int i = 0; i < kProbe; i++) {
        v[i] = malloc(3);
        if (!v[i]) { std::printf("ALLOC_NULL\n"); return 3; }
    }
    for (int i = 0; i < kProbe; i++) {
        for (int j = i + 1; j < kProbe && j < i + 64; j++) {
            if (v[i] == v[j]) { std::printf("DUPLICATE\n"); return 4; }
        }
    }
    for (int i = 0; i < kProbe; i++) { std::memset(v[i], 0xab, 3); free(v[i]); }
    std::printf("CONSISTENT\n");
    return 0;
}

// Run the child with SMASH_DOUBLE_FREE=policy (nullptr = unset); capture its
// combined stdout+stderr and wait status.
bool runCase(const char* policy, char* out, size_t out_sz, int* status) {
    char tmpl[] = "/tmp/smash_dfree_XXXXXX";
    const int fd = mkstemp(tmpl);
    if (fd < 0) return false;
    const pid_t pid = fork();
    if (pid < 0) { close(fd); unlink(tmpl); return false; }
    if (pid == 0) {
        dup2(fd, 1); dup2(fd, 2); close(fd);
        if (policy) setenv("SMASH_DOUBLE_FREE", policy, 1);
        else        unsetenv("SMASH_DOUBLE_FREE");
        execl("/proc/self/exe", "double_free_test", "child", (char*)nullptr);
        _exit(127);
    }
    waitpid(pid, status, 0);
    lseek(fd, 0, SEEK_SET);
    const ssize_t n = read(fd, out, out_sz - 1);
    out[n > 0 ? n : 0] = '\0';
    close(fd);
    unlink(tmpl);
    return true;
}

bool has(const char* hay, const char* needle) { return std::strstr(hay, needle) != nullptr; }

}  // namespace

int main(int argc, char** argv) {
    setvbuf(stdout, nullptr, _IONBF, 0);
    if (argc > 1 && std::strcmp(argv[1], "child") == 0) return runChild();

    char out[8192];
    int status = 0;

    // ── 1: warn — detected, dropped, allocator still consistent ─────────────
    if (!runCase("warn", out, sizeof out, &status)) {
        std::fprintf(stderr, "FAIL: could not fork child\n"); return 1;
    }
    if (!has(out, "SURVIVED")) {
        // Under glibc (no preload) the process aborts instead; nothing to guard.
        std::printf("SKIP: child did not survive the double free (not under libsmash?)\n");
        return 0;
    }
    if (!has(out, "[smash double-free]")) {
        std::fprintf(stderr,
            "FAIL: SMASH_DOUBLE_FREE=warn did not report the repeated free. The "
            "common free(p);free(p); case is caught by the ThreadCache lane "
            "scan — both pointers stay in the cache and never reach the slab.\n");
        return 1;
    }
    if (has(out, "DUPLICATE") || !has(out, "CONSISTENT")) {
        std::fprintf(stderr,
            "FAIL: allocator inconsistent after a dropped double free "
            "(duplicate=%d consistent=%d)\n",
            (int)has(out, "DUPLICATE"), (int)has(out, "CONSISTENT"));
        return 1;
    }
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        std::fprintf(stderr, "FAIL: warn must not kill the process (status %d)\n", status);
        return 1;
    }

    // ── 2: unset — the default really is off ────────────────────────────────
    if (!runCase(nullptr, out, sizeof out, &status)) {
        std::fprintf(stderr, "FAIL: could not fork child (unset)\n"); return 1;
    }
    if (has(out, "[smash double-free]")) {
        std::fprintf(stderr, "FAIL: detection fired with SMASH_DOUBLE_FREE unset; "
                             "the default must be off\n");
        return 1;
    }
    // Deliberately no consistency assertion here: with the policy off the second
    // free is applied, and a duplicate hand-out is permitted (the program has
    // already invoked UB). Asserting otherwise would be asserting a guarantee
    // smash does not make.

    // ── 3: abort — dies by signal ───────────────────────────────────────────
    if (!runCase("abort", out, sizeof out, &status)) {
        std::fprintf(stderr, "FAIL: could not fork child (abort)\n"); return 1;
    }
    if (!WIFSIGNALED(status)) {
        std::fprintf(stderr, "FAIL: SMASH_DOUBLE_FREE=abort should trap "
                             "(status %d, exited=%d)\n", status, WIFEXITED(status));
        return 1;
    }

    std::printf("PASS: warn reports and drops (allocator consistent), "
                "unset is silent, abort traps (signal %d)\n", WTERMSIG(status));
    return 0;
}
