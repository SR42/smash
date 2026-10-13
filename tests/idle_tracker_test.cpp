// idle_tracker_test.cpp - SMASH_WRITE_TRACK=idle: the page_idle bitmap as the
// only access signal, with nothing protected to watch a page.
//
// Added for a 128-core arm64 application on a 6.6 kernel, where the only
// write tracking was page protection (SMASH_SOFTDIRTY=0): every first write
// to an idle page faulted and cost an mprotect under the process's mmap_lock,
// and a read was seen only by making the page PROT_NONE for a tick. With
// CAP_SYS_ADMIN (real PFNs in /proc/self/pagemap) and the bitmap read-write,
// the kernel's accessed bit answers both.
//
// IDLE_PART=tracker (root, skips otherwise): 32 MiB left idle, 16 MiB
// written and 16 MiB read every 300 ms. The idle block must leave RSS within
// 60 s and the stats line must say track=idle; then, over 6 more seconds of
// the same traffic, the two hot blocks must take no fault at all (no
// decompression, no shadow, no monitor fault: nothing watched, nothing hot
// compressed), Phase 3 must arm nothing, the scan must have covered pages,
// the arena must be MADV_NOHUGEPAGE (VmFlags nh), and every byte must read
// back. Run with the default scan period (a slice every 4 ticks) and with
// SMASH_IDLE_SCAN_TICKS=1 (every tick).
//
// IDLE_PART=fallback (any user): with SMASH_THP=1 the tracker must refuse,
// say why on stderr, and fall back to page protection, which still
// compresses the idle block.
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

constexpr size_t kCold = 32u << 20, kHot = 16u << 20;
int g_log = -1;

size_t resident(const void* p, size_t len) {
    std::vector<unsigned char> v(len / 4096);
    if (mincore(const_cast<void*>(p), len, v.data()) != 0) return len / 4096;
    size_t r = 0;
    for (auto c : v) r += c & 1;
    return r;
}

uint64_t pattern(size_t i, uint64_t salt) { return (i / 4096) * 0x9E3779B97F4A7C15ull ^ salt; }

void fill(char* p, size_t len, uint64_t salt) {
    for (size_t i = 0; i < len; i += 4096) {
        std::memset(p + i, 0, 4096);
        *reinterpret_cast<uint64_t*>(p + i) = pattern(i, salt);
    }
}

bool check(const char* p, size_t len, uint64_t salt) {
    for (size_t i = 0; i < len; i += 4096)
        if (*reinterpret_cast<const uint64_t*>(p + i) != pattern(i, salt)) return false;
    return true;
}

std::string dumpLog() {
    raise(SIGUSR2);
    usleep(200 * 1000);
    std::string log;
    char buf[8192];
    lseek(g_log, 0, SEEK_SET);
    for (ssize_t n; (n = read(g_log, buf, sizeof(buf))) > 0;) log.append(buf, static_cast<size_t>(n));
    return log;
}

unsigned long long field(const std::string& log, const char* key) {
    const size_t at = log.rfind(key);
    if (at == std::string::npos) return ~0ull;
    return std::strtoull(log.c_str() + at + std::strlen(key), nullptr, 10);
}

std::string trackOf(const std::string& log) {
    const size_t at = log.rfind("track=");
    if (at == std::string::npos) return "";
    const size_t e = log.find_first_of(" \n", at);
    return log.substr(at + 6, e - at - 6);
}

// VmFlags of the mapping holding p, from /proc/self/smaps.
std::string vmFlags(const void* p) {
    FILE* f = std::fopen("/proc/self/smaps", "r");
    if (!f) return "?";
    char line[512];
    bool in = false;
    const auto a = reinterpret_cast<uintptr_t>(p);
    std::string out = "?";
    while (std::fgets(line, sizeof(line), f)) {
        unsigned long lo, hi;
        if (std::sscanf(line, "%lx-%lx ", &lo, &hi) == 2 && std::strchr(line, '-') < std::strchr(line, ' ')) {
            in = a >= lo && a < hi;
            continue;
        }
        if (in && std::strncmp(line, "VmFlags:", 8) == 0) { out = line + 8; break; }
    }
    std::fclose(f);
    return out;
}

bool idleUsable() {
    if (geteuid() != 0) return false;
    if (access("/sys/kernel/mm/page_idle/bitmap", R_OK | W_OK) != 0) return false;
    return true;
}

}  // namespace

int main() {
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) { std::printf("SKIP: not running under libsmash\n"); return 0; }
    const char* part_env = std::getenv("IDLE_PART");
    const std::string part = part_env ? part_env : "tracker";
    if (part == "tracker" && !idleUsable()) {
        std::printf("SKIP: needs root and /sys/kernel/mm/page_idle/bitmap\n");
        return 0;
    }
    char path[] = "/tmp/smash_idletrack_XXXXXX";
    g_log = mkstemp(path);
    if (g_log < 0) { std::printf("SKIP: mkstemp failed\n"); return 0; }
    unlink(path);
    const int saved = dup(2);
    dup2(g_log, 2);

    auto* cold = static_cast<char*>(malloc(kCold));
    auto* wr = static_cast<char*>(malloc(kHot));
    auto* rd = static_cast<char*>(malloc(kHot));
    fill(cold, kCold, 1);
    fill(wr, kHot, 2);
    fill(rd, kHot, 3);

    std::atomic<bool> stop{false};
    std::thread hot([&] {
        volatile uint64_t sink = 0;
        while (!stop) {
            for (size_t i = 0; i < kHot; i += 4096) {
                *reinterpret_cast<volatile uint64_t*>(wr + i) = pattern(i, 2);
                sink += *reinterpret_cast<volatile uint64_t*>(rd + i);
            }
            usleep(300 * 1000);
        }
    });
    const auto t0 = std::chrono::steady_clock::now();
    while (resident(cold, kCold) > 0 && std::chrono::steady_clock::now() - t0 < std::chrono::seconds(60))
        usleep(250 * 1000);
    const double cold_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    const size_t cold_left = resident(cold, kCold);
    const std::string before = dumpLog();
    sleep(6);
    const std::string after = dumpLog();
    stop = true;
    hot.join();
    const bool data_ok = check(cold, kCold, 1) && check(wr, kHot, 2) && check(rd, kHot, 3);
    const std::string flags = vmFlags(cold);
    dup2(saved, 2);

    int failures = 0;
    auto fail = [&](const std::string& m) { std::printf("FAIL: %s\n", m.c_str()); ++failures; };
    const std::string track = trackOf(after);
    std::printf("track=%s cold block left RSS in %.1f s (%zu pages still resident)\n", track.c_str(), cold_s,
                cold_left);
    if (cold_left > 0) fail("the idle block never left RSS");
    if (!data_ok) fail("data differs after compression");

    if (part == "fallback") {
        if (track != "protread") fail("track=" + track + ", want protread after the refusal");
        if (after.find("SMASH_WRITE_TRACK=idle unavailable: SMASH_THP=1") == std::string::npos)
            fail("no warning naming SMASH_THP=1");
        if (failures) return 1;
        std::printf("idle_tracker_test (fallback): all checks passed\n");
        return 0;
    }

    // The fault counters on the pipeline line ("faults monitor= shadow=
    // compressed="): the stats line has shadow=/compressed= of its own.
    auto faultField = [&](const std::string& log, const char* k) {
        const size_t at = log.rfind("faults monitor=");
        if (at == std::string::npos) return ~0ull;
        const size_t f = log.find(k, at);
        return f == std::string::npos ? ~0ull : std::strtoull(log.c_str() + f + std::strlen(k), nullptr, 10);
    };
    const unsigned long long f_mon = faultField(after, "monitor="), armed = field(after, "p3 armed="),
                             pages = field(after, " calls, ");
    const unsigned long long d_shadow = faultField(after, " shadow=") - faultField(before, " shadow="),
                             d_comp = faultField(after, " compressed=") - faultField(before, " compressed=");
    std::printf("over 6 s of hot traffic: faults shadow=+%llu compressed=+%llu; total monitor=%llu, "
                "p3 armed=%llu, idle-scanned pages=%llu; arena VmFlags:%s",
                d_shadow, d_comp, f_mon, armed, pages, flags.c_str());
    if (track != "idle") fail("track=" + track + ", want idle");
    if (d_shadow != 0 || d_comp != 0) fail("hot pages were compressed (faults on shadow/compressed pages)");
    if (f_mon != 0) fail("monitor faults: pages were watched");
    if (armed != 0) fail("Phase 3 armed pages");
    if (pages == 0 || pages == ~0ull) fail("the idle scan covered no pages");
    if (flags.find(" nh") == std::string::npos) fail("the arena is not MADV_NOHUGEPAGE");
    if (failures) return 1;
    std::printf("idle_tracker_test: all checks passed\n");
    return 0;
}
