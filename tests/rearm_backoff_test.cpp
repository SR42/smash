// rearm_backoff_test.cpp - under page-protection write tracking, a page the
// application keeps writing must not be re-armed every tick.
//
// With SMASH_SOFTDIRTY=0 (no kernel write tracking), Phase 3 makes every
// ACTIVE page PROT_READ each tick and a write faults it back to PROT_RW:
// two mprotect calls and a SIGSEGV per hot page per tick, all serialised on
// the process's mmap_lock. On a 128-core, 4-node arm64 box that was 8.2M
// write faults in 17 ticks, and the compressor's own phases spent their
// time blocked or in the kernel behind them (p3: 2 s user, 49 s sys, 147 s
// blocked) with ticks of 140 s instead of 1 s.
//
// A page written within a tick of being armed now waits 2, 4, 8, ...
// ticks (SMASH_REARM_BACKOFF_MAX caps the exponent) before it is armed
// again, and is not counted idle while it waits.
//
//   A. 8 MiB written continuously next to 32 MiB held idle: write faults on
//      watched pages must stay under 0.4 per hot page per tick (1.0 without
//      the backoff); the hot pages must not be compressed while unwatched
//      (shadow + compressed faults under 5% of them); the idle block must
//      still leave RSS.
//   B. The writer stops: the formerly hot block must leave RSS too.
#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <string>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>

namespace {

struct Snap { unsigned long long ticks = 0, monitor = 0, shadow = 0, compressed = 0; bool ok = false; };

std::string slurp(int fd) {
    std::string out;
    char buf[8192];
    lseek(fd, 0, SEEK_SET);
    for (ssize_t n; (n = read(fd, buf, sizeof(buf))) > 0;) out.append(buf, static_cast<size_t>(n));
    return out;
}

// Dump the stats and parse the newest [smash pipeline] line.
Snap snap(int fd) {
    const size_t before = slurp(fd).size();
    raise(SIGUSR2);
    Snap s;
    for (int i = 0; i < 50; ++i) {
        const std::string log = slurp(fd);
        const size_t at = log.rfind("[smash pipeline]");
        if (at != std::string::npos && at >= before) {
            const std::string line = log.substr(at, log.find('\n', at) - at);
            const size_t t = line.find("ticks=");
            const size_t f = line.find("faults monitor=");
            if (t != std::string::npos && f != std::string::npos &&
                std::sscanf(line.c_str() + t, "ticks=%llu", &s.ticks) == 1 &&
                std::sscanf(line.c_str() + f, "faults monitor=%llu shadow=%llu compressed=%llu",
                            &s.monitor, &s.shadow, &s.compressed) == 3)
                s.ok = true;
            return s;
        }
        usleep(10 * 1000);
    }
    return s;
}

size_t residentPages(const char* p, size_t len, size_t ps) {
    const size_t n = len / ps;
    unsigned char vec[16384];
    if (n > sizeof(vec) || mincore(const_cast<char*>(p), len, vec) != 0) return n;
    size_t r = 0;
    for (size_t i = 0; i < n; ++i) r += vec[i] & 1;
    return r;
}

}  // namespace

int main() {
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) { std::printf("SKIP: not running under libsmash\n"); return 0; }
    char path[] = "/tmp/smash_rearm_XXXXXX";
    const int fd = mkstemp(path);
    if (fd < 0) { std::printf("SKIP: mkstemp failed\n"); return 0; }
    unlink(path);
    const int saved = dup(2);
    dup2(fd, 2);

    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    constexpr size_t kHot = 8u << 20, kCold = 32u << 20;
    auto* hot = static_cast<char*>(malloc(kHot));
    auto* cold = static_cast<char*>(malloc(kCold));
    for (size_t i = 0; i < kCold; ++i) cold[i] = static_cast<char>((i / ps) * 7 + (i % 64));
    std::memset(hot, 1, kHot);
    const size_t hot_pages = kHot / ps;

    std::atomic<bool> stop{false};
    std::thread writer([&] {
        for (unsigned v = 2; !stop.load(std::memory_order_relaxed); ++v) {
            for (size_t off = 0; off < kHot; off += ps) hot[off] = static_cast<char>(v);
            usleep(20 * 1000);
        }
    });

    int failures = 0;
    auto fail = [&](const char* fmt, auto... a) {
        dprintf(saved, "FAIL: ");
        dprintf(saved, fmt, a...);
        dprintf(saved, "\n");
        ++failures;
    };

    // A: hot writer running.
    sleep(3);
    const Snap a0 = snap(fd);
    sleep(20);
    const Snap a1 = snap(fd);
    const size_t cold_resident = residentPages(cold, kCold, ps);
    stop = true;
    writer.join();
    if (!a0.ok || !a1.ok || a1.ticks <= a0.ticks) {
        dup2(saved, 2);
        std::printf("FAIL: no [smash pipeline] counters (SMASH_STATS_SIGNAL=1 and "
                    "SMASH_SOFTDIRTY=0 set?)\n");
        return 1;
    }
    const double ticks = static_cast<double>(a1.ticks - a0.ticks);
    const double per = static_cast<double>(a1.monitor - a0.monitor) / (ticks * hot_pages);
    const unsigned long long unwatched = (a1.shadow - a0.shadow) + (a1.compressed - a0.compressed);
    dprintf(saved, "A: %.0f ticks, %llu write faults on watched pages = %.2f per hot page per tick; "
            "%llu shadow/compressed faults; idle block %zu of %zu pages resident\n",
            ticks, a1.monitor - a0.monitor, per, unwatched, cold_resident, kCold / ps);
    if (per > 0.4)
        fail("hot pages are re-armed nearly every tick (%.2f write faults per page per tick): "
             "each is two mprotect calls under the mmap_lock", per);
    if (unwatched * 20 > hot_pages)
        fail("%llu faults on shadow/compressed hot pages: pages waiting to be re-armed were "
             "counted idle and compressed while nobody watched them", unwatched);
    if (cold_resident * 10 > kCold / ps)
        fail("the idle block kept %zu of %zu pages resident next to the hot one",
             cold_resident, kCold / ps);

    // B: the writer has stopped; the hot block must go cold and leave RSS.
    size_t hot_resident = hot_pages;
    for (int i = 0; i < 90 && hot_resident * 10 > hot_pages; ++i) {
        sleep(1);
        hot_resident = residentPages(hot, kHot, ps);
    }
    dprintf(saved, "B: formerly hot block %zu of %zu pages resident\n", hot_resident, hot_pages);
    if (hot_resident * 10 > hot_pages)
        fail("the formerly hot block kept %zu of %zu pages resident 90 s after its writer "
             "stopped", hot_resident, hot_pages);

    dup2(saved, 2);
    for (size_t off = 0; off < kHot; off += ps)
        if (hot[off] == 0) { std::printf("FAIL: hot block lost its contents\n"); return 1; }
    free(hot);
    free(cold);
    if (failures) return 1;
    std::printf("rearm_backoff_test: all checks passed\n");
    return 0;
}
