// seed_profile_test.cpp - a loaded SMASH_PROFILE_FILE reaches every
// compressor worker (compressor_thread.h seedBucketStats).
//
// The profile is read at start() into the workers that exist then: workers 0
// and 1 (kCompressorWorkers). The rest of the pool, up to SMASH_MAX_WORKERS,
// and every NUMA node group's workers are allocated after it and copy worker
// 0's buckets. Each worker decides from its own copy, so a worker that did not
// get it ignores what the profile says about the pages it handles.
//
// A profile in which every bucket says "always skip" (decision_hint 1, what a
// run whose pages never compressed saves) must keep every page of a cold,
// compressible 256 MiB block resident, whichever worker handles it:
//   SEED_GROUPS unset: SMASH_TEST_WORKERS=8, SMASH_NUMA_COMPRESSOR=0 -- six of
//     the eight workers come from the pool after the load;
//   SEED_GROUPS=1: SMASH_TEST_WORKERS=1 and the node groups, whose threads take
//     slices of the shared range (smash_work_share_info must show some);
//     needs two memory nodes with CPUs, skips otherwise.
// The same block without the profile compresses (the control), and the run
// with it is held twice as long as the control took.
//
// The profile itself is written by a first child (SMASH_PROFILE_FILE_SAVE=1),
// so its header matches this build, and then every record's hint is set.
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <dlfcn.h>
#include <string>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

static size_t resident(const char* p, size_t len) {
    std::vector<unsigned char> v(len / 4096);
    if (mincore(const_cast<char*>(p), len, v.data()) != 0) return len / 4096;
    size_t r = 0;
    for (auto c : v) r += c & 1;
    return r;
}

static void fill(char* p, size_t len) {
    for (size_t i = 0; i < len; i += 8)
        *reinterpret_cast<uint64_t*>(p + i) = (i / 4096) * 0x9E3779B97F4A7C15ull ^ ((i & 255) * 3);
}

// Memory nodes that have CPUs.
static int nodesWithCpus() {
    int n = 0;
    DIR* d = opendir("/sys/devices/system/node");
    if (!d) return 0;
    while (dirent* e = readdir(d)) {
        if (std::strncmp(e->d_name, "node", 4) != 0 || e->d_name[4] < '0' || e->d_name[4] > '9') continue;
        std::string path = std::string("/sys/devices/system/node/") + e->d_name + "/cpulist";
        FILE* f = std::fopen(path.c_str(), "r");
        if (!f) continue;
        char buf[256] = {};
        if (std::fgets(buf, sizeof buf, f) && buf[0] >= '0' && buf[0] <= '9') ++n;
        std::fclose(f);
    }
    closedir(d);
    return n;
}

// Run this binary again as SEED_PART=part; returns its exit status.
static int child(const char* part, const char* profile, bool save, const char* hold_s) {
    std::fflush(stdout);
    const pid_t pid = fork();
    if (pid < 0) return -1;
    if (pid == 0) {
        setenv("SEED_PART", part, 1);
        if (profile) setenv("SMASH_PROFILE_FILE", profile, 1);
        else unsetenv("SMASH_PROFILE_FILE");
        if (save) setenv("SMASH_PROFILE_FILE_SAVE", "1", 1);
        else unsetenv("SMASH_PROFILE_FILE_SAVE");
        if (hold_s) setenv("SEED_HOLD_S", hold_s, 1);
        char* argv[] = {const_cast<char*>("seed_profile_test"), nullptr};
        execv("/proc/self/exe", argv);
        _exit(127);
    }
    int st = 0;
    if (waitpid(pid, &st, 0) != pid) return -1;
    return WIFEXITED(st) ? WEXITSTATUS(st) : 128 + WTERMSIG(st);
}

static int runPart(const char* part) {
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) { std::printf("SKIP: not running under libsmash\n"); return 0; }
    if (!std::strcmp(part, "save")) {
        // Something compressed, so the save has run through a tick.
        constexpr size_t kLen = 4u << 20;
        char* p = static_cast<char*>(std::malloc(kLen));
        fill(p, kLen);
        for (int s = 0; s < 40 && resident(p, kLen) * 2 > kLen / 4096; ++s) usleep(250000);
        return 0;   // the profile is written at exit
    }
    constexpr size_t kLen = 256u << 20;
    const size_t pages = kLen / 4096;
    char* p = static_cast<char*>(std::malloc(kLen));
    if (!p) { std::printf("FAIL: malloc\n"); return 1; }
    fill(p, kLen);
    const auto t0 = std::chrono::steady_clock::now();
    if (!std::strcmp(part, "control")) {
        for (int s = 0; s < 120 && resident(p, kLen) * 10 > pages; ++s) usleep(250000);
        const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        const size_t r = resident(p, kLen);
        std::printf("control (no profile): %zu of %zu pages resident after %.1f s\n", r, pages, secs);
        if (r * 10 > pages) { std::printf("FAIL: the block never compressed without the profile\n"); return 1; }
        std::printf("CONTROL_SECS=%.1f\n", secs);
        return 0;
    }
    // "run": the all-skip profile is loaded.
    const char* h = std::getenv("SEED_HOLD_S");
    const double hold = h ? std::atof(h) : 10.0;
    size_t lowest = pages;
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < hold) {
        const size_t r = resident(p, kLen);
        if (r < lowest) lowest = r;
        usleep(250000);
    }
    int failures = 0;
    std::printf("run (all-skip profile): at least %zu of %zu pages resident over %.1f s\n", lowest, pages, hold);
    if (lowest + 64 < pages) {
        std::printf("FAIL: %zu pages left RSS: a worker ignored the profile\n", pages - lowest);
        ++failures;
    }
    if (std::getenv("SEED_GROUPS")) {
        using share_fn = int (*)(uint64_t*, uint64_t*, uint64_t*);
        auto share = reinterpret_cast<share_fn>(dlsym(RTLD_DEFAULT, "smash_work_share_info"));
        uint64_t q = 0, s = 0, u = 0;
        if (!share || share(&q, &s, &u) != 0) {
            std::printf("FAIL: smash_work_share_info unavailable\n");
            ++failures;
        } else {
            std::printf("node group threads took %llu slices of the shared range\n",
                        static_cast<unsigned long long>(s));
            if (s == 0) {
                std::printf("FAIL: no node group thread took a slice: the groups' copy was not exercised\n");
                ++failures;
            }
        }
    }
    return failures ? 1 : 0;
}

int main() {
    if (const char* part = std::getenv("SEED_PART")) return runPart(part);
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) { std::printf("SKIP: not running under libsmash\n"); return 0; }
    if (std::getenv("SEED_GROUPS") && nodesWithCpus() < 2) {
        std::printf("SKIP: fewer than two memory nodes with CPUs, no node groups\n");
        return 0;
    }

    char path[] = "/tmp/smash_seed_profile_XXXXXX";
    const int fd = mkstemp(path);
    if (fd < 0) { std::printf("FAIL: mkstemp: %s\n", std::strerror(errno)); return 1; }
    close(fd);
    const std::string lock = std::string(path) + ".lock";
    int failures = 0;
    auto done = [&](int rc) {
        unlink(path);
        unlink(lock.c_str());
        return rc;
    };

    if (child("save", path, true, nullptr) != 0) { std::printf("FAIL: the save child failed\n"); return done(1); }
    // Header: u32 magic, u32 version, u16 arenas, u16 buckets, u32 flags;
    // then a 16-byte record per (arena, bucket), decision_hint at byte 1.
    std::vector<unsigned char> buf;
    if (FILE* f = std::fopen(path, "rb")) {
        unsigned char b[65536];
        for (size_t n; (n = std::fread(b, 1, sizeof b, f)) > 0;) buf.insert(buf.end(), b, b + n);
        std::fclose(f);
    }
    if (buf.size() < 16) { std::printf("FAIL: no profile saved (%zu bytes)\n", buf.size()); return done(1); }
    uint16_t arenas = 0, buckets = 0;
    std::memcpy(&arenas, &buf[8], 2);
    std::memcpy(&buckets, &buf[10], 2);
    const size_t records = static_cast<size_t>(arenas) * buckets;
    if (records == 0 || buf.size() < 16 + records * 16) {
        std::printf("FAIL: profile of %zu bytes holds fewer than %zu records\n", buf.size(), records);
        return done(1);
    }
    for (size_t i = 0; i < records; ++i) buf[16 + i * 16 + 1] = 1;   // always skip
    if (FILE* f = std::fopen(path, "wb")) {
        const bool ok = std::fwrite(buf.data(), 1, buf.size(), f) == buf.size();
        if (std::fclose(f) != 0 || !ok) { std::printf("FAIL: rewriting the profile\n"); return done(1); }
    }
    std::printf("profile: %zu records, every one set to always-skip\n", records);

    // The control, timed: its stdout carries CONTROL_SECS.
    int pipefd[2];
    if (pipe(pipefd) != 0) return done(1);
    const int saved = dup(1);
    dup2(pipefd[1], 1);
    close(pipefd[1]);
    const int crc = child("control", nullptr, false, nullptr);
    std::fflush(stdout);
    dup2(saved, 1);
    close(saved);
    std::string out;
    char b[4096];
    for (ssize_t n; (n = read(pipefd[0], b, sizeof b)) > 0;) out.append(b, static_cast<size_t>(n));
    close(pipefd[0]);
    std::fputs(out.c_str(), stdout);
    if (crc != 0) return done(1);
    double secs = 5;
    if (const size_t at = out.find("CONTROL_SECS="); at != std::string::npos) secs = std::atof(out.c_str() + at + 13);
    char hold[32];
    std::snprintf(hold, sizeof hold, "%.1f", secs * 2 > 8 ? secs * 2 : 8.0);

    if (child("run", path, false, hold) != 0) ++failures;
    if (failures) { std::printf("seed_profile_test: FAILED\n"); return done(1); }
    std::printf("seed_profile_test: all checks passed\n");
    return done(0);
}
