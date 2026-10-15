// codec_env_test.cpp - SMASH_CODEC must pick the codec pages are compressed
// with, and the stats line must say which one is in use.
//
// Added because the codec was a build option only (-DSMASH_USE_LZ4, now
// removed): an application that set SMASH_USE_LZ4=1 in its environment, as
// if it were a setting, got zstd-1 with nothing to tell it so.
// SMASH_CODEC=lz4|zstd now picks it at startup, and the stats line reports
// codec=lz4|zstd1.
//
// Also guards SMASH_VERY_COLD_TICKS=9999 ("fast tier only"), which removed
// every zstd profile, the fast one included, so nothing was ever compressed,
// and SMASH_CALIBRATION_FILE, which hung every process at startup (fopen()
// inside the heap's constructor waited for the heap) and now records the
// codec its figures were measured with.
//
// Each case runs in a child process (the variables are read at startup). The
// child fills 64 MiB with text-like pages (words from a small vocabulary:
// LZ4 finds the repeats, zstd also entropy-codes them), waits for the block to
// leave RSS, prints the stats line's codec= and blob= live MiB, and checks
// every byte reads back. Checks:
//   - SMASH_CODEC=lz4 reports codec=lz4, SMASH_CODEC=zstd reports codec=zstd1;
//   - lz4's blobs are at least 15% larger than zstd's (the codec really
//     changed, not just the label);
//   - unset, and an unrecognised value (which must print a [smash WARN]),
//     report the default (zstd1) and blobs within 10% of zstd's;
//   - SMASH_VERY_COLD_TICKS=9999 still compresses the block;
//   - with SMASH_CALIBRATION_FILE, a child starts (within 90 s), the file is
//     written with "fast_codec": "lz4", and a zstd child then rewrites it
//     with "zstd1" (a file measured with another codec is not reused);
//   - every child reads its data back intact.
#include <climits>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <signal.h>
#include <string>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

namespace {

constexpr size_t kLen = 64u << 20, kPage = 4096, kPages = kLen / kPage;

const char* const kWords[] = {
    "alpha", "bravo", "charlie", "delta", "echo", "foxtrot", "golf", "hotel",
    "india", "juliet", "kilo", "lima", "mike", "november", "oscar", "papa",
    "quebec", "romeo", "sierra", "tango", "uniform", "victor", "whiskey", "xray",
    "yankee", "zulu", "red", "green", "blue", "cyan", "magenta", "yellow",
};

// Deterministic text-like content: the same for every child, so the codecs
// are compared on identical data.
void fill(char* b) {
    uint64_t x = 0x9E3779B97F4A7C15ull;
    size_t i = 0;
    while (i < kLen) {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        const char* w = kWords[x % 32];
        const size_t n = std::strlen(w);
        for (size_t k = 0; k < n && i < kLen; ++k) b[i++] = w[k];
        if (i < kLen) b[i++] = (x >> 8) % 7 == 0 ? '\n' : ' ';
    }
}

size_t resident(const char* p) {
    std::vector<unsigned char> v(kPages);
    if (mincore(const_cast<char*>(p), kLen, v.data()) != 0) return kPages;
    size_t r = 0;
    for (auto c : v) r += c & 1;
    return r;
}

int child() {
    char path[] = "/tmp/smash_codec_XXXXXX";
    const int log = mkstemp(path);
    if (log < 0) return 2;
    unlink(path);
    const int saved = dup(2);
    dup2(log, 2);

    auto* b = static_cast<char*>(malloc(kLen));
    fill(b);
    size_t left = kPages;
    for (int s = 0; s < 240 && (left = resident(b)) > kPages / 100; ++s) usleep(250 * 1000);
    raise(SIGUSR2);
    usleep(100 * 1000);
    dup2(saved, 2);

    std::string text;
    char buf[8192];
    lseek(log, 0, SEEK_SET);
    for (ssize_t n; (n = read(log, buf, sizeof(buf))) > 0;) text.append(buf, static_cast<size_t>(n));
    std::string codec = "?";
    size_t live = 0;
    const size_t at = text.rfind("[smash stats]");
    if (at != std::string::npos) {
        const std::string line = text.substr(at, text.find('\n', at) - at);
        const size_t c = line.find(" codec="), bl = line.find(" blob=");
        if (c != std::string::npos) codec = line.substr(c + 7, line.find(' ', c + 7) - c - 7);
        if (bl != std::string::npos) std::sscanf(line.c_str() + bl, " blob=%zu/", &live);
    }

    std::vector<char> want(kLen);
    fill(want.data());
    const bool intact = std::memcmp(b, want.data(), kLen) == 0;
    std::printf("codec=%s live=%zu resident=%zu intact=%d\n", codec.c_str(), live, left, intact ? 1 : 0);
    free(b);
    return 0;
}

struct Result { bool ok = false; std::string codec; size_t live = 0, resident = 0; int warned = 0, intact = 0; };

Result run(const char* self, const char* env) {
    Result r;
    // stderr too: smash reads SMASH_CODEC, and warns, before the child runs.
    std::string cmd = std::string("timeout 90 env -u SMASH_CODEC -u SMASH_VERY_COLD_TICKS -u SMASH_CALIBRATION_FILE ") +
                      env + " '" + self + "' child 2>&1";
    FILE* p = popen(cmd.c_str(), "r");
    if (!p) return r;
    char line[512] = {0}, codec[64] = {0};
    while (fgets(line, sizeof(line), p)) {
        if (std::strstr(line, "[smash WARN] SMASH_CODEC")) r.warned = 1;
        if (std::sscanf(line, "codec=%63s live=%zu resident=%zu intact=%d", codec, &r.live, &r.resident,
                        &r.intact) == 4) {
            r.ok = true;
            r.codec = codec;
        }
    }
    const int st = pclose(p);
    r.ok = r.ok && st == 0;
    std::printf("%-28s -> codec=%s blob live=%zu MiB, %zu pages resident, warned=%d, intact=%d\n", env,
                r.codec.c_str(), r.live, r.resident, r.warned, r.intact);
    return r;
}

std::string codecInFile(const char* path) {
    char buf[2048] = {0};
    const int fd = open(path, O_RDONLY);
    if (fd < 0) return "(no file)";
    const ssize_t n = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    const char* t = n > 0 ? std::strstr(buf, "\"fast_codec\": \"") : nullptr;
    if (!t) return "(no tag)";
    t += std::strlen("\"fast_codec\": \"");
    return std::string(t, std::strcspn(t, "\""));
}

bool near(size_t a, size_t b) { return a * 10 <= b * 11 && b * 10 <= a * 11; }

}  // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "child") == 0) return child();
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) { std::printf("SKIP: not running under libsmash\n"); return 0; }
    char self[PATH_MAX];
    const ssize_t n = readlink("/proc/self/exe", self, sizeof(self) - 1);
    if (n <= 0) return 1;
    self[n] = '\0';
    int failures = 0;
    auto fail = [&](const std::string& m) { std::printf("FAIL: %s\n", m.c_str()); ++failures; };

    const Result lz4 = run(self, "SMASH_CODEC=lz4");
    const Result zstd = run(self, "SMASH_CODEC=zstd");
    const Result dflt = run(self, "");
    const Result bogus = run(self, "SMASH_CODEC=snappy");
    const Result fast_only = run(self, "SMASH_VERY_COLD_TICKS=9999");
    char cal[] = "/tmp/smash_cal_XXXXXX";
    const int cfd = mkstemp(cal);
    if (cfd < 0) return 1;
    close(cfd);
    unlink(cal);
    const Result cal_lz4 = run(self, (std::string("SMASH_CODEC=lz4 SMASH_CALIBRATION_FILE=") + cal).c_str());
    const std::string tag_lz4 = codecInFile(cal);
    const Result cal_zstd = run(self, (std::string("SMASH_CODEC=zstd SMASH_CALIBRATION_FILE=") + cal).c_str());
    const std::string tag_zstd = codecInFile(cal);
    unlink(cal);
    std::printf("calibration file: \"%s\" after lz4, \"%s\" after zstd\n", tag_lz4.c_str(), tag_zstd.c_str());
    for (const Result* r : {&lz4, &zstd, &dflt, &bogus, &fast_only, &cal_lz4, &cal_zstd}) {
        if (!r->ok) fail("a child did not report");
        else if (!r->intact) fail("a child read its data back wrong (codec=" + r->codec + ")");
        else if (r->resident > kPages / 100) fail("a child's block stayed resident (codec=" + r->codec + ")");
    }
    if (failures) return 1;

    if (lz4.codec != "lz4") fail("SMASH_CODEC=lz4 reports codec=" + lz4.codec);
    if (zstd.codec != "zstd1") fail("SMASH_CODEC=zstd reports codec=" + zstd.codec);
    if (lz4.live * 100 < zstd.live * 115)
        fail("lz4 blobs " + std::to_string(lz4.live) + " MiB, zstd " + std::to_string(zstd.live) +
             " MiB: the codec did not change");
    if (dflt.codec != "zstd1") fail("unset reports codec=" + dflt.codec + ", want the default zstd1");
    else if (!near(dflt.live, zstd.live)) fail("unset: blobs do not match zstd's");
    if (bogus.codec != dflt.codec || !near(bogus.live, dflt.live)) fail("an unrecognised SMASH_CODEC changed the codec");
    if (!bogus.warned) fail("an unrecognised SMASH_CODEC printed no [smash WARN]");
    if (lz4.warned || zstd.warned || dflt.warned) fail("a valid SMASH_CODEC printed a warning");
    if (cal_lz4.codec != "lz4" || !near(cal_lz4.live, lz4.live)) fail("lz4 with a calibration file: not lz4");
    if (cal_zstd.codec != "zstd1" || !near(cal_zstd.live, zstd.live)) fail("zstd with a calibration file: not zstd");
    if (tag_lz4 != "lz4") fail("calibration file after lz4 says " + tag_lz4);
    if (tag_zstd != "zstd1") fail("calibration file after zstd says " + tag_zstd + " (lz4 figures reused?)");
    if (failures) return 1;
    std::printf("codec_env_test: all checks passed\n");
    return 0;
}
