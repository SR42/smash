// fault_time_test.cpp - the [smash faults] line must report how long the
// fault handler took, per kind, for the faults the pipeline line counts.
//
// Added to measure fault cost on a 128-core arm64 application: the counters
// said how many faults (514k decompressions, 1.58M shadow, 592k monitor in
// ~30 min) but not how long each took, and under mmap_lock contention a
// single fault there could be far slower than the ~18 us measured on an x86
// VM.
//
// 8 MiB compressed and read back (decompression faults); 8 MiB written
// every 250 ms meanwhile (monitor faults, SMASH_SOFTDIRTY=0). For both kinds the line's n=
// must equal the pipeline line's fault count, with avg > 0 (printed to 0.1 us:
// a monitor fault on claudepc takes well under 1 us in the handler, and the
// first version's integer avg=0 failed there), max >= 1 us, avg <= max and
// p50 <= p99 <= p99.9.
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <string>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

static size_t resident(const char* p, size_t len) {
    std::vector<unsigned char> v(len / 4096);
    if (mincore(const_cast<char*>(p), len, v.data()) != 0) return len / 4096;
    size_t r = 0;
    for (auto c : v) r += c & 1;
    return r;
}

struct Kind { unsigned long long n = 0, p50 = 0, p99 = 0, p999 = 0, max = 0; double avg = 0; bool ok = false; };

static Kind parseKind(const std::string& line, const char* name) {
    Kind k;
    const std::string key = std::string(" ") + name + " n=";
    const size_t at = line.find(key);
    if (at == std::string::npos) return k;
    k.ok = std::sscanf(line.c_str() + at + 1 + std::strlen(name),
                       " n=%llu avg=%lf p50<%llu p99<%llu p99.9<%llu max=%llu", &k.n, &k.avg,
                       &k.p50, &k.p99, &k.p999, &k.max) == 6;
    return k;
}

int main() {
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) { std::printf("SKIP: not running under libsmash\n"); return 0; }
    char path[] = "/tmp/smash_faulttime_XXXXXX";
    const int fd = mkstemp(path);
    if (fd < 0) { std::printf("SKIP: mkstemp failed\n"); return 0; }
    unlink(path);
    const int saved = dup(2);
    dup2(fd, 2);

    constexpr size_t kLen = 8u << 20;
    auto* a = static_cast<char*>(malloc(kLen));
    auto* b = static_cast<char*>(malloc(kLen));
    for (size_t i = 0; i < kLen; i += 8)
        *reinterpret_cast<uint64_t*>(a + i) = (i / 4096) * 0x9E3779B97F4A7C15ull ^ ((i & 255) * 3);
    std::memset(b, 1, kLen);
    // a goes cold and is compressed; b is written every 250 ms, so it is
    // watched again each tick and each write after that takes a monitor
    // fault, but it never goes cold.
    for (int s = 0; s < 480 && resident(a, kLen) > 0; ++s) {
        for (size_t off = 0; off < kLen; off += 4096) b[off] = static_cast<char>(s);
        usleep(250 * 1000);
    }
    volatile uint64_t sink = 0;
    for (size_t off = 0; off < kLen; off += 4096) sink += *reinterpret_cast<volatile uint64_t*>(a + off);
    raise(SIGUSR2);
    usleep(200 * 1000);

    std::string log;
    char buf[8192];
    lseek(fd, 0, SEEK_SET);
    for (ssize_t n; (n = read(fd, buf, sizeof(buf))) > 0;) log.append(buf, static_cast<size_t>(n));
    dup2(saved, 2);
    unsigned long long fm = 0, fs = 0, fc = 0;
    const size_t pl = log.rfind("faults monitor=");
    if (pl != std::string::npos)
        std::sscanf(log.c_str() + pl, "faults monitor=%llu shadow=%llu compressed=%llu", &fm, &fs, &fc);
    const size_t fl = log.rfind("[smash faults]");
    const std::string line = fl == std::string::npos ? "" : log.substr(fl, log.find('\n', fl) - fl);
    std::printf("%s\n", line.empty() ? "(no [smash faults] line)" : line.c_str());
    int failures = 0;
    auto check = [&](const char* name, unsigned long long want) {
        const Kind k = parseKind(line, name);
        if (!k.ok) { std::printf("FAIL: no %s entry\n", name); ++failures; return; }
        if (k.n != want || k.n < 100) {
            std::printf("FAIL: %s n=%llu, the pipeline line counted %llu (want equal, >= 100)\n", name, k.n, want);
            ++failures;
        }
        if (k.avg <= 0 || k.max < 1 || k.p50 > k.p99 || k.p99 > k.p999 || k.avg > k.max + 1) {
            std::printf("FAIL: %s timings inconsistent: avg=%.1f p50<%llu p99<%llu p99.9<%llu max=%llu\n",
                        name, k.avg, k.p50, k.p99, k.p999, k.max);
            ++failures;
        }
    };
    check("compressed", fc);
    check("monitor", fm);
    free(a);
    free(b);
    if (failures) return 1;
    std::printf("fault_time_test: all checks passed\n");
    return 0;
}
