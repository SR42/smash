// write_track_fallback_test.cpp - when auto-selection finds no write tracking,
// smash must say so.
//
// SMASH_WRITE_TRACK=auto tries soft-dirty, then async userfaultfd
// write-protect (Linux 6.7+). arm64 has no soft-dirty, so on a 6.6 arm64
// kernel both fail and the tracker is `none`: no write signal, cold memory
// found only by compressing and re-checking contents. That cost an
// application most of its compression (103 of 144 GiB of its arena left
// uncompressed) with nothing printed. The rule checked here, on any kernel:
// the stats line reports track=none exactly when the warning was printed,
// unless SMASH_WRITE_TRACK=none asked for it (then no warning).
//
// Registered three ways: SMASH_WRITE_TRACK_SIMULATE_OLD_KERNEL=1 (must end at
// none, with the warning), the same plus SMASH_WRITE_TRACK=none (none, no
// warning), and the host as it is (whatever it selects, consistently).
//
// The warning and the stats line go to fd 2, so the test points fd 2 at a
// file of its own, prompts the stats line with SIGUSR2 until the tracker has
// been chosen (the compressor picks it on its first tick), and reads it back.
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <string>
#include <unistd.h>

static std::string slurp(int fd) {
    std::string out;
    char buf[4096];
    lseek(fd, 0, SEEK_SET);
    for (ssize_t n; (n = read(fd, buf, sizeof(buf))) > 0;) out.append(buf, static_cast<size_t>(n));
    return out;
}

int main() {
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) { std::printf("SKIP: not running under libsmash\n"); return 0; }
    char path[] = "/tmp/smash_wt_XXXXXX";
    const int fd = mkstemp(path);
    if (fd < 0) { std::printf("SKIP: mkstemp failed\n"); return 0; }
    unlink(path);
    const int saved = dup(2);
    dup2(fd, 2);

    // Something in the arena, so the compressor has work and ticks.
    auto* p = static_cast<char*>(malloc(8u << 20));
    std::memset(p, 7, 8u << 20);

    std::string log;
    std::string track;
    for (int i = 0; i < 100 && (track.empty() || track == "unprobed"); ++i) {
        raise(SIGUSR2);
        usleep(100 * 1000);
        log = slurp(fd);
        const size_t at = log.rfind("track=");
        if (at != std::string::npos) {
            const size_t end = log.find_first_of(" \n", at);
            track = log.substr(at + 6, end == std::string::npos ? std::string::npos : end - at - 6);
        }
    }
    dup2(saved, 2);
    free(p);

    const bool warned = log.find("[smash WARN] no write tracking on this kernel") != std::string::npos;
    const char* wt = std::getenv("SMASH_WRITE_TRACK");
    const bool asked_none = wt && std::strcmp(wt, "none") == 0;
    const char* sim = std::getenv("SMASH_WRITE_TRACK_SIMULATE_OLD_KERNEL");
    const bool simulated = sim && sim[0] == '1';
    std::printf("write tracker: %s, warning %s%s%s\n", track.empty() ? "(no stats line)" : track.c_str(),
                warned ? "printed" : "not printed", simulated ? ", old kernel simulated" : "",
                asked_none ? ", SMASH_WRITE_TRACK=none" : "");

    if (track.empty() || track == "unprobed") {
        std::printf("FAIL: the stats line never reported a chosen write tracker "
                    "(is SMASH_STATS_SIGNAL=1 set?)\n");
        return 1;
    }
    if (simulated && track != "none" && track != "protread") {
        std::printf("FAIL: with soft-dirty and userfaultfd simulated away the tracker is %s\n",
                    track.c_str());
        return 1;
    }
    const bool expect_warn = track == "none" && !asked_none;
    if (warned != expect_warn) {
        std::printf("FAIL: track=%s but the warning was %s\n", track.c_str(),
                    warned ? "printed" : "not printed");
        return 1;
    }
    std::printf("write_track_fallback_test: all checks passed\n");
    return 0;
}
