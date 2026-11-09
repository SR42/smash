// idle_tracker_test.cpp - SMASH_WRITE_TRACK=idle|yt: the page_idle bitmap, or
// the youngtrace module's page-table walk, as the only access signal, with
// nothing protected to watch a page. The tracker comes from the environment.
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
// and every byte must read back. Run with the default scan period (a slice
// every 4 ticks), where the scan must cover no more than its 1/4 share of the
// arena per tick, and with SMASH_IDLE_SCAN_TICKS=1 (every tick).
//
// idle with SMASH_THP=1 (IDLE_WANT_HUGE=1): the same on huge pages, which the
// arena must really hold before the blocks go cold (AnonHugePages; skips where
// the host has THP off). The bitmap answers for a folio through its head alone,
// so a tracker reading each page's own bit took the 511 others for accessed,
// and the idle block never left RSS. And with 8 GiB allocated and freed first
// (IDLE_FREED_MIB): address space with nothing live costs the scan no system
// call, over the window it may make no more than IDLE_MAX_CALLS_PER_TICK.
//
// yt (the module loaded, root): the same, plus no cut request: every request must fit the
// module's buffer. And once with SMASH_THP=1 and 640-page requests
// (SMASH_YT_REQ_PAGES), so request edges split huge pages: a huge page has
// one accessed bit, which the first request covering it clears, so the part
// the next request covers must have been marked by the first. And once with
// a 512 MiB span allocated and never touched (IDLE_SPAN_MIB) and one worker,
// so the arena holds several slices: at K > 1 a request must cover a whole
// slice, the largest power of two of pages the module's buffer can always
// hold (scan_buf_cap). Every request pays the module a zeroing of that whole
// buffer, and 64 MiB slices, never adjacent when due, made every request
// 64 MiB - a quarter of what the default 8 MiB buffer holds. And once with
// SMASH_YT_MMAP=1: the scans are read from the module's buffer mapped into
// the process (/proc/self/maps must show it), and every check above must
// still hold; without the variable nothing may be mapped. And once with
// SMASH_YT_WHOLE=1 and the 512 MiB span, default workers: one request per due
// tick covers the whole arena, so over the window there are at most ticks/K + 2
// scans (requests resumed after a cut do not count), each over every
// committed page.
//
// IDLE_PART=fallback (any user): the tracker must refuse - idle with
// SMASH_KPAGEFLAGS naming no file, yt with SMASH_YT_DEVICE naming no device - say why on stderr,
// and fall back to page protection, which still compresses the idle block.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
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

// "idle_scan=T (C calls, P pages, W words, O other idle, R pagemap)": R.
unsigned long long idlePagemapReads(const std::string& log) {
    const size_t at = log.rfind("idle_scan=");
    if (at == std::string::npos) return ~0ull;
    const size_t o = log.find(" other idle, ", at);
    if (o == std::string::npos) return ~0ull;
    return std::strtoull(log.c_str() + o + 13, nullptr, 10);
}

std::string trackOf(const std::string& log) {
    const size_t at = log.rfind("track=");
    if (at == std::string::npos) return "";
    const size_t e = log.find_first_of(" \n", at);
    return log.substr(at + 6, e - at - 6);
}

// AnonHugePages of the mapping holding p, from /proc/self/smaps, in KiB.
size_t anonHugeKib(const void* p) {
    FILE* f = std::fopen("/proc/self/smaps", "r");
    if (!f) return 0;
    char line[512];
    bool in = false;
    const auto a = reinterpret_cast<uintptr_t>(p);
    size_t out = 0;
    while (std::fgets(line, sizeof(line), f)) {
        unsigned long lo, hi;
        if (std::sscanf(line, "%lx-%lx ", &lo, &hi) == 2 && std::strchr(line, '-') < std::strchr(line, ' ')) {
            in = a >= lo && a < hi;
            continue;
        }
        if (in && std::strncmp(line, "AnonHugePages:", 14) == 0) { out = std::strtoull(line + 14, nullptr, 10); break; }
    }
    std::fclose(f);
    return out;
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

bool yt() {
    const char* t = std::getenv("SMASH_WRITE_TRACK");
    return t && std::strcmp(t, "yt") == 0;
}

bool trackerUsable() {
    if (geteuid() != 0) return false;
    return access(yt() ? "/dev/youngtrace/scanner" : "/sys/kernel/mm/page_idle/bitmap", R_OK | W_OK) == 0;
}

// The "<key>N (C calls, P pages" counters of the syscall line.
// Open files of this process on the page_idle bitmap.
int bitmapFiles() {
    int n = 0;
    if (DIR* d = opendir("/proc/self/fd")) {
        while (dirent* e = readdir(d)) {
            char path[64], target[128];
            std::snprintf(path, sizeof(path), "/proc/self/fd/%s", e->d_name);
            const ssize_t len = readlink(path, target, sizeof(target) - 1);
            if (len <= 0) continue;
            target[len] = '\0';
            n += std::strcmp(target, "/sys/kernel/mm/page_idle/bitmap") == 0;
        }
        closedir(d);
    }
    return n;
}

unsigned long long scanPages(const std::string& log, const char* key) {
    const size_t at = log.rfind(key);
    if (at == std::string::npos) return ~0ull;
    const size_t c = log.find(" calls, ", at);
    return c == std::string::npos ? ~0ull : std::strtoull(log.c_str() + c + 8, nullptr, 10);
}

// Mappings of the youngtrace device in this process.
int scannerMaps() {
    int n = 0;
    if (FILE* f = std::fopen("/proc/self/maps", "r")) {
        char line[512];
        while (std::fgets(line, sizeof(line), f)) n += std::strstr(line, "youngtrace/scanner") != nullptr;
        std::fclose(f);
    }
    return n;
}

unsigned long long scanCalls(const std::string& log, const char* key) {
    const size_t at = log.rfind(key);
    if (at == std::string::npos) return ~0ull;
    const size_t c = log.find(" (", at);
    return c == std::string::npos ? ~0ull : std::strtoull(log.c_str() + c + 2, nullptr, 10);
}

// Pages in one yt slice: the largest power of two not above the pages a
// request can cover without being cut, (scan_buf_cap - 8) / 128 (a record and
// an item per page at worst), at most 65536.
unsigned long long ytSlicePages() {
    unsigned long long cap = 0;
    if (FILE* f = std::fopen("/sys/module/youngtrace/parameters/scan_buf_cap", "r")) {
        if (std::fscanf(f, "%llu", &cap) != 1) cap = 0;
        std::fclose(f);
    }
    if (cap < 64 * 1024) return 0;
    unsigned long long fit = std::min((cap - 8) / 128, 65536ull), s = 1;
    while (s * 2 <= fit) s *= 2;
    return s;
}

}  // namespace

int main() {
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) { std::fprintf(stderr, "FAIL: not running under libsmash\n"); return 1; }
    const char* part_env = std::getenv("IDLE_PART");
    const std::string part = part_env ? part_env : "tracker";
    if (part == "tracker" && !trackerUsable()) {
        std::printf("SKIP: needs root and %s\n",
                    yt() ? "the youngtrace module (/dev/youngtrace/scanner)" : "/sys/kernel/mm/page_idle/bitmap");
        return 77;
    }
    char path[] = "/tmp/smash_idletrack_XXXXXX";
    g_log = mkstemp(path);
    if (g_log < 0) { std::printf("SKIP: mkstemp failed\n"); return 77; }
    unlink(path);
    const int saved = dup(2);
    dup2(g_log, 2);

    // High-water mark: address space walked once and free for good after that.
    const char* freed_env = std::getenv("IDLE_FREED_MIB");
    if (freed_env) free(malloc(static_cast<size_t>(std::atoi(freed_env)) << 20));

    auto* cold = static_cast<char*>(malloc(kCold));
    auto* wr = static_cast<char*>(malloc(kHot));
    auto* rd = static_cast<char*>(malloc(kHot));
    fill(cold, kCold, 1);
    fill(wr, kHot, 2);
    fill(rd, kHot, 3);
    // Never touched: it only makes the arena span several yt slices.
    const char* span_env = std::getenv("IDLE_SPAN_MIB");
    const size_t span = span_env ? static_cast<size_t>(std::atoi(span_env)) << 20 : 0;
    void* untouched = span ? malloc(span) : nullptr;

    // Huge pages must be what the blocks are on, or this case tests nothing.
    if (std::getenv("IDLE_WANT_HUGE")) {
        const size_t huge = anonHugeKib(cold);
        std::printf("arena AnonHugePages after filling the blocks: %zu KiB\n", huge);
        if (huge < 16 * 1024) {
            dup2(saved, 2);
            std::printf("SKIP: the arena holds no huge pages (THP off on this host?)\n");
            return 77;
        }
    }

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
        const char* why = yt() ? "SMASH_WRITE_TRACK=yt unavailable: /nonexistent/youngtrace/scanner cannot be opened"
                               : "SMASH_WRITE_TRACK=idle unavailable: ";
        if (after.find(why) == std::string::npos) fail(std::string("no warning: ") + why);
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
                             pages = scanPages(after, yt() ? "yt_scan=" : "idle_scan="),
                             cut = yt() ? field(after, " cut=") : 0;
    const unsigned long long d_shadow = faultField(after, " shadow=") - faultField(before, " shadow="),
                             d_comp = faultField(after, " compressed=") - faultField(before, " compressed=");
    std::printf("over 6 s of hot traffic: faults shadow=+%llu compressed=+%llu; total monitor=%llu, "
                "p3 armed=%llu, scanned pages=%llu; arena VmFlags:%s",
                d_shadow, d_comp, f_mon, armed, pages, flags.c_str());
    const std::string want = yt() ? "yt" : "idle";
    if (track != want) fail("track=" + track + ", want " + want);
    if (d_shadow != 0 || d_comp != 0) fail("hot pages were compressed (faults on shadow/compressed pages)");
    if (f_mon != 0) fail("monitor faults: pages were watched");
    if (armed != 0) fail("Phase 3 armed pages");
    if (pages == 0 || pages == ~0ull) fail("the idle scan covered no pages");
    if (flags.find(" nh") != std::string::npos) fail("the arena is MADV_NOHUGEPAGE");
    if (cut != 0) fail("youngtrace requests failed or were cut: cut=" + std::to_string(cut));
    if (yt()) {
        const char* m = std::getenv("SMASH_YT_MMAP");
        const bool want = m && m[0] == '1';
        const int maps = scannerMaps();
        std::printf("youngtrace buffer mapped %d times (SMASH_YT_MMAP=%s)\n", maps, m ? m : "unset");
        if (want && maps == 0) fail("SMASH_YT_MMAP=1 but no youngtrace buffer is mapped");
        if (!want && maps != 0) fail("a youngtrace buffer is mapped without SMASH_YT_MMAP=1");
    }
    // SMASH_IDLE_SCAN_TICKS=K scans 1/K of the arena per tick: over the
    // window, at most ticks/K + 2 arenas' worth (the slices fall due in
    // whole slices, so a tick can take more than its share).
    const char* k_env = std::getenv("SMASH_IDLE_SCAN_TICKS");
    const unsigned long long k = k_env ? std::strtoull(k_env, nullptr, 10) : 4,
                             d_pages = pages - scanPages(before, yt() ? "yt_scan=" : "idle_scan="),
                             d_ticks = field(after, "ticks=") - field(before, "ticks="),
                             committed = field(after, "committed=");
    std::printf("scan over the window: %llu pages in %llu ticks, %llu pages committed, K=%llu\n", d_pages,
                d_ticks, committed, k);
    if (k > 1 && d_pages > committed * (d_ticks / k + 2))
        fail("the scan covered more than 1/K of the arena per tick");
    if (!yt() && freed_env) {
        const unsigned long long d_reads = idlePagemapReads(after) - idlePagemapReads(before);
        const unsigned long long per_tick = d_ticks ? d_reads / d_ticks : d_reads;
        std::printf("pagemap reads over the window: %llu in %llu ticks (%llu per tick), %llu pages committed\n",
                    d_reads, d_ticks, per_tick, committed);
        // The 64 MiB live is 16 reads per tick; the 8 GiB nothing lives on
        // would be 2048 more.
        if (idlePagemapReads(after) == ~0ull) fail("no pagemap reads on the stats line");
        else if (per_tick > 100) fail("the idle scan reads address space with nothing live on it");
    }
    // A request covers a whole due slice (IDLE_SPAN_MIB, one worker, so no
    // range edge cuts one): on average more than half a slice, where 64 MiB
    // slices gave at most 16384 pages a request.
    const char* whole_env = std::getenv("SMASH_YT_WHOLE");
    const bool whole = yt() && whole_env && whole_env[0] == '1';
    if (yt() && span && k > 1 && !whole) {
        const unsigned long long d_calls = scanCalls(after, "yt_scan=") - scanCalls(before, "yt_scan="),
                                 slice = ytSlicePages();
        std::printf("yt requests over the window: %llu for %llu pages, %llu per request; slice %llu pages\n",
                    d_calls, d_pages, d_calls ? d_pages / d_calls : 0, slice);
        if (slice == 0) fail("cannot read /sys/module/youngtrace/parameters/scan_buf_cap");
        else if (d_calls == 0 || d_pages * 2 <= d_calls * slice)
            fail("yt requests cover less than half a slice: " + std::to_string(d_calls ? d_pages / d_calls : 0) +
                 " pages each, slice " + std::to_string(slice));
    }
    // SMASH_YT_WHOLE=1: the coordinator scans the whole arena, one request
    // resumed where the module's cap cuts it, every K ticks.
    if (whole) {
        const unsigned long long d_calls = scanCalls(after, "yt_scan=") - scanCalls(before, "yt_scan="),
                                 d_resumed = field(after, " resumed=") - field(before, " resumed="),
                                 scans = d_calls - d_resumed;
        std::printf("whole-arena scans over the window: %llu (%llu requests, %llu resumed) for %llu pages in "
                    "%llu ticks, %llu pages committed, longest request %llu us\n",
                    scans, d_calls, d_resumed, d_pages, d_ticks, committed, field(after, "max_req_us="));
        if (scans == 0 || scans > d_ticks / k + 2)
            fail("SMASH_YT_WHOLE=1: " + std::to_string(scans) + " scans in " + std::to_string(d_ticks) +
                 " ticks, want one per due tick");
        else if (d_pages < scans * (committed / 2))
            fail("SMASH_YT_WHOLE=1: a scan covered " + std::to_string(d_pages / scans) + " pages of " +
                 std::to_string(committed) + " committed");
    }
    // The bitmap is read in 64-page words, and a read makes the kernel check
    // every idle-marked page of the word, not only the ones asked about:
    // "N words, M other idle" counts both (diagnostic for the arm64 box,
    // where the scan cost 46-86 us per page). At least one word per 64 pages.
    if (!yt()) {
        const size_t at = after.rfind("idle_scan=");
        unsigned long long ms = 0, calls = 0, pg = 0, words = 0, other = 0;
        if (at == std::string::npos ||
            std::sscanf(after.c_str() + at, "idle_scan=%llu (%llu calls, %llu pages, %llu words, %llu other idle)",
                        &ms, &calls, &pg, &words, &other) != 5)
            fail("no \"idle_scan=ms (N calls, P pages, W words, O other idle)\" on the stats line");
        else if (words * 64 < pg && !std::getenv("IDLE_WANT_HUGE"))   // on huge pages one bit answers for 512
            fail("bitmap words read: " + std::to_string(words) + " for " + std::to_string(pg) + " pages");
        else
            std::printf("bitmap: %llu words read for %llu pages, %llu other idle pages in them\n", words, pg, other);
    }
    // Every scanning thread has its own open bitmap: sysfs runs one read or
    // write at a time per open file (kernfs of->mutex), and 8 workers sharing
    // one spent the arm64 box's scan spinning on it. With SMASH_TEST_WORKERS
    // >= 2: the probe's file plus at least two threads' own.
    const char* tw = std::getenv("SMASH_TEST_WORKERS");
    if (!yt() && tw && std::atoi(tw) >= 2) {
        const int files = bitmapFiles();
        std::printf("page_idle bitmap open %d times with %s workers\n", files, tw);
        if (files < 3) fail("the scanning threads share one open bitmap: " + std::to_string(files) + " open");
    }
    free(untouched);
    if (failures) return 1;
    std::printf("idle_tracker_test: all checks passed\n");
    return 0;
}
