// chk_interposition_test.cpp - every __*_chk wrapper must actually win the
// binding, on whatever architecture this runs.
//
// _FORTIFY_SOURCE rewrites a call whose destination size is known into the
// __*_chk alias, so those are the symbols a hardened program really calls.
// smash wraps them to keep the EFAULT decompress-and-retry working — but a
// wrapper only wins if it is exported at the version the caller binds, and
// that version is not the same everywhere: glibc versions them GLIBC_2.4 /
// 2.5 / 2.16 on x86-64 and GLIBC_2.17 on aarch64, where the whole port starts
// at 2.17. Exported only at the x86-64 versions, every one of these silently
// resolved to libc on aarch64 and the retry never ran. It surfaced as
// test_syscall_efault failing with "__getgroups_chk returned -1 errno=14",
// 52 seconds into a test whose message says nothing about symbol versions.
//
// Taking a function's address makes the linker emit the same versioned
// reference a real call would, so dladdr on it reports which library actually
// won. That is the whole check, and it is instant.
#define _FORTIFY_SOURCE 2
#include <dlfcn.h>
#include <poll.h>
#include <cstdio>
#include <cstring>
#include <grp.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {

int failures = 0;

void expectOwnedBySmash(const char* name, void* addr) {
    Dl_info info{};
    if (!addr) { std::fprintf(stderr, "FAIL: %s has no address\n", name); ++failures; return; }
    if (!dladdr(addr, &info) || !info.dli_fname) {
        std::fprintf(stderr, "FAIL: dladdr(%s) failed\n", name);
        ++failures;
        return;
    }
    const bool ours = std::strstr(info.dli_fname, "libsmash") != nullptr;
    std::printf("  %-20s -> %s%s\n", name, info.dli_fname, ours ? "" : "   <-- NOT smash");
    if (!ours) {
        std::fprintf(stderr,
            "FAIL: %s resolves to %s, not libsmash — the wrapper is not exported at the "
            "version this architecture's callers bind, so the EFAULT decompress-and-retry "
            "never runs for fortified calls\n", name, info.dli_fname);
        ++failures;
    }
}

}  // namespace

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    // Is smash even loaded? Without it, every symbol legitimately belongs to libc.
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) {
        std::fprintf(stderr, "FAIL: not running under libsmash\n");
        return 1;
    }
#if defined(__GLIBC__) && defined(__USE_FORTIFY_LEVEL) && __USE_FORTIFY_LEVEL > 0
    expectOwnedBySmash("__read_chk",       reinterpret_cast<void*>(&__read_chk));
    expectOwnedBySmash("__pread64_chk",    reinterpret_cast<void*>(&__pread64_chk));
    expectOwnedBySmash("__recv_chk",       reinterpret_cast<void*>(&__recv_chk));
    expectOwnedBySmash("__recvfrom_chk",   reinterpret_cast<void*>(&__recvfrom_chk));
    expectOwnedBySmash("__poll_chk",       reinterpret_cast<void*>(&__poll_chk));
    expectOwnedBySmash("__ppoll_chk",      reinterpret_cast<void*>(&__ppoll_chk));
    expectOwnedBySmash("__readlink_chk",   reinterpret_cast<void*>(&__readlink_chk));
    expectOwnedBySmash("__readlinkat_chk", reinterpret_cast<void*>(&__readlinkat_chk));
    expectOwnedBySmash("__getgroups_chk",  reinterpret_cast<void*>(&__getgroups_chk));
    // Buffered reads: glibc's internal read into the stream's buffer never
    // reaches smash's read(), so these route to the fgets/fread wrappers and
    // their retry (stdio_chk_test).
    expectOwnedBySmash("__fgets_chk",      reinterpret_cast<void*>(&__fgets_chk));
    expectOwnedBySmash("__fread_chk",      reinterpret_cast<void*>(&__fread_chk));
#else
    std::printf("SKIP: _FORTIFY_SOURCE declarations unavailable; nothing to check\n");
    return 77;
#endif
    if (failures) {
        std::printf("chk_interposition_test: %d FAILURES\n", failures);
        return 1;
    }
    std::printf("chk_interposition_test: all fortified wrappers are smash's\n");
    return 0;
}
