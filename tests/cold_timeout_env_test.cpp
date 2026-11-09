// cold_timeout_env_test.cpp - SMASH_COLD_TIMEOUT_SEC / SMASH_COLD_TICKS
// outside what smash can honour must say so, once, at startup.
//
// A page's idle count is one byte that stops at 255, and a page is compressed
// only once its count is above the timeout: SMASH_COLD_TIMEOUT_SEC=255 or more
// compressed nothing, with no warning, and atof() turned a value like 1e12
// into an out-of-range conversion. Such values are now clamped to 254 s with a
// [smash WARN] line naming SMASH_NO_COMPRESSOR=1, and text that is not a
// positive number keeps the default with a warning. test_compression's
// testColdTimeoutParse covers the parser case by case, and
// test_compression_cold_max that a clamped floor still compresses; this
// checks the warning reaches stderr of a preloaded process exactly once, in
// both modes, and that the process runs normally.
#include "placement_test_util.h"

#include <vector>

using namespace placement;

namespace {

int child() {
    // Allocate and touch a block in each size range, and give the compressor
    // a tick to start.
    std::vector<char*> blocks;
    for (size_t sz : {size_t{256}, size_t{64} << 10, size_t{4} << 20}) {
        char* p = static_cast<char*>(malloc(sz));
        if (!p) return 2;
        std::memset(p, 0x41, sz);
        blocks.push_back(p);
    }
    usleep(1500 * 1000);
    for (char* p : blocks) free(p);
    std::printf("child ok\n");
    return 0;
}

size_t count(const std::string& s, const std::string& what) {
    size_t n = 0;
    for (size_t at = s.find(what); at != std::string::npos; at = s.find(what, at + 1)) ++n;
    return n;
}

}  // namespace

int main(int argc, char** argv) {
    setvbuf(stdout, nullptr, _IONBF, 0);
    if (argc > 1 && std::strcmp(argv[1], "child") == 0) return child();
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) {
        std::fprintf(stderr, "FAIL: not running under libsmash\n");
        return 1;
    }
    char self[PATH_MAX];
    if (!selfPath(self)) return 1;

    struct Case { const char* env; const char* warning; };
    const Case cases[] = {
        {"SMASH_COLD_TIMEOUT_SEC=100", nullptr},
        {"SMASH_COLD_TIMEOUT_SEC=254", nullptr},
        {"SMASH_COLD_TIMEOUT_SEC=255", "SMASH_COLD_TIMEOUT_SEC=255 is above the maximum of 254 s"},
        {"SMASH_COLD_TIMEOUT_SEC=99999", "SMASH_COLD_TIMEOUT_SEC=99999 is above the maximum of 254 s"},
        {"SMASH_COLD_TIMEOUT_SEC=1e12", "SMASH_COLD_TIMEOUT_SEC=1e12 is above the maximum of 254 s"},
        {"SMASH_COLD_TIMEOUT_SEC=abc", "SMASH_COLD_TIMEOUT_SEC=abc is not a positive number"},
        {"SMASH_COLD_TIMEOUT_SEC=0", "SMASH_COLD_TIMEOUT_SEC=0 is not a positive number"},
        {"SMASH_COLD_TICKS=999", "SMASH_COLD_TICKS=999 is above the maximum of 254 ticks"},
    };
    int failures = 0;
    for (const char* mode : {"SMASH_LARGE_ONLY=1", "SMASH_LARGE_ONLY=0"}) {
        for (const Case& c : cases) {
            const std::string cmd = std::string("timeout 60 env -u SMASH_COLD_TIMEOUT_SEC -u SMASH_COLD_TICKS ") +
                                    mode + " " + c.env + " '" + self + "' child 2>&1";
            int st = 0;
            const std::string out = runChild(cmd, &st);
            const size_t warns = count(out, "[smash WARN] SMASH_COLD_");
            const size_t want = c.warning ? 1 : 0;
            const bool matched = !c.warning || out.find(c.warning) != std::string::npos;
            const bool ran = st == 0 && out.find("child ok") != std::string::npos;
            std::printf("%-20s %-30s status %d, %zu cold-timeout warning(s)%s\n", mode, c.env, st, warns,
                        matched ? "" : " (not the expected text)");
            if (!ran || warns != want || !matched) {
                std::printf("FAIL: %s %s: want %zu warning(s)%s%s and a normal run\n", mode, c.env, want,
                            c.warning ? " containing \"" : "", c.warning ? (std::string(c.warning) + "\"").c_str() : "");
                showTail(out);
                ++failures;
            }
        }
    }
    if (failures) return 1;
    std::printf("cold_timeout_env_test: all checks passed\n");
    return 0;
}
