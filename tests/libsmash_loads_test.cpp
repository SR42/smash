// libsmash_loads_test.cpp - the fixture every test requires: the preloaded
// libsmash must exist and be the one this build made.
//
// A missing preload is only a warning to the dynamic loader
// (`LD_PRELOAD=/nonexistent /bin/true` exits 0), and add_test does not build
// the smash target, so a test-only or partial build ran every test under
// glibc. Measured on 2026-11-09 with libsmash.so missing: 201 of 247 tests
// passed, 136 of them by printing SKIP and returning 0, and 29 because
// glibc satisfies what they check. Now this test fails, and ctest does not
// run the tests that require it (FIXTURES_REQUIRED libsmash, set for every
// test at the end of tests/CMakeLists.txt).
//
// SMASH_TEST_EXPECT_LIB is the smash target's file. The library that defines
// smash_in_arena must be that file, not another libsmash the process picked
// up (an installed one, /etc/ld.so.preload).
#include <cerrno>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <unistd.h>

int main() {
    const char* want = std::getenv("SMASH_TEST_EXPECT_LIB");
    if (!want || !*want) {
        std::fprintf(stderr, "FAIL: SMASH_TEST_EXPECT_LIB is not set (see tests/CMakeLists.txt)\n");
        return 1;
    }
    if (access(want, R_OK) != 0) {
        std::fprintf(stderr, "FAIL: %s: %s. Build the smash target: every test preloads it.\n", want,
                     std::strerror(errno));
        return 1;
    }
    void* sym = dlsym(RTLD_DEFAULT, "smash_in_arena");
    if (!sym) {
        std::fprintf(stderr, "FAIL: %s exists but is not loaded (smash_in_arena not found); "
                             "the loader's warning above says why\n", want);
        return 1;
    }
    Dl_info info{};
    if (!dladdr(sym, &info) || !info.dli_fname) {
        std::fprintf(stderr, "FAIL: dladdr cannot name the library that defines smash_in_arena\n");
        return 1;
    }
    char got_real[PATH_MAX], want_real[PATH_MAX];
    if (!realpath(info.dli_fname, got_real) || !realpath(want, want_real)) {
        std::fprintf(stderr, "FAIL: realpath(%s or %s): %s\n", info.dli_fname, want, std::strerror(errno));
        return 1;
    }
    if (std::strcmp(got_real, want_real) != 0) {
        std::fprintf(stderr, "FAIL: smash_in_arena comes from %s, not from this build's %s\n", got_real,
                     want_real);
        return 1;
    }
    std::printf("libsmash loaded: %s\n", got_real);
    return 0;
}
