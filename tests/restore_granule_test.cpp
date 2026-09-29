// restore_granule_test.cpp - with SMASH_RESTORE_GRANULE_PAGES=16 under
// page-protection write tracking, a write fault makes the page's whole
// aligned 16-page group writable, so writes that cluster cost one fault
// (and one mprotect under the mmap_lock) per group instead of per page.
//
// From a 128-core arm64 application under SMASH_SOFTDIRTY=0: 481k mappings,
// 397k of them smash's own PROT_READ / PROT_NONE runs, and ~2.6k
// application-thread mprotect calls per second from write faults.
//
// 8 MiB written page by page every 20 ms (the re-arm backoff off, to isolate
// the group restore) next to 16 MiB held idle: at most 0.15 write faults per
// hot page per tick (1.05 without it), and the idle block must still leave
// RSS. (A mapping-count check was dropped: a sequential writer never
// fragments the block, 3 mappings at most even without the group restore.)
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

struct Snap { unsigned long long ticks = 0, monitor = 0; bool ok = false; };

std::string slurp(int fd) {
    std::string out;
    char buf[8192];
    lseek(fd, 0, SEEK_SET);
    for (ssize_t n; (n = read(fd, buf, sizeof(buf))) > 0;) out.append(buf, static_cast<size_t>(n));
    return out;
}

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
            s.ok = t != std::string::npos && f != std::string::npos &&
                   std::sscanf(line.c_str() + t, "ticks=%llu", &s.ticks) == 1 &&
                   std::sscanf(line.c_str() + f, "faults monitor=%llu", &s.monitor) == 1;
            return s;
        }
        usleep(10 * 1000);
    }
    return s;
}

size_t residentPages(const char* p, size_t len, size_t ps) {
    const size_t n = len / ps;
    unsigned char vec[8192];
    if (n > sizeof(vec) || mincore(const_cast<char*>(p), len, vec) != 0) return n;
    size_t r = 0;
    for (size_t i = 0; i < n; ++i) r += vec[i] & 1;
    return r;
}

}  // namespace

int main() {
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) { std::printf("SKIP: not running under libsmash\n"); return 0; }
    char path[] = "/tmp/smash_granule_XXXXXX";
    const int fd = mkstemp(path);
    if (fd < 0) { std::printf("SKIP: mkstemp failed\n"); return 0; }
    unlink(path);
    const int saved = dup(2);
    dup2(fd, 2);

    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    constexpr size_t kHot = 8u << 20, kCold = 16u << 20;
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

    sleep(3);
    const Snap a0 = snap(fd);
    sleep(16);
    const Snap a1 = snap(fd);
    const size_t cold_resident = residentPages(cold, kCold, ps);
    stop = true;
    writer.join();
    dup2(saved, 2);

    if (!a0.ok || !a1.ok || a1.ticks <= a0.ticks) {
        std::printf("FAIL: no [smash pipeline] counters (SMASH_STATS_SIGNAL=1 and "
                    "SMASH_SOFTDIRTY=0 set?)\n");
        return 1;
    }
    const double ticks = static_cast<double>(a1.ticks - a0.ticks);
    const double per = static_cast<double>(a1.monitor - a0.monitor) / (ticks * hot_pages);
    std::printf("%.0f ticks, %llu write faults on watched pages = %.3f per hot page per tick; "
                "idle block %zu of %zu pages resident\n",
                ticks, a1.monitor - a0.monitor, per, cold_resident, kCold / ps);
    int failures = 0;
    if (per > 0.15) {
        std::printf("FAIL: %.3f write faults per hot page per tick: a fault restores one page, "
                    "not its group\n", per);
        ++failures;
    }
    if (cold_resident * 10 > kCold / ps) {
        std::printf("FAIL: the idle block kept %zu of %zu pages resident\n", cold_resident,
                    kCold / ps);
        ++failures;
    }
    for (size_t off = 0; off < kHot; off += ps)
        if (hot[off] == 0) { std::printf("FAIL: hot block lost its contents\n"); return 1; }
    free(hot);
    free(cold);
    if (failures) return 1;
    std::printf("restore_granule_test: all checks passed\n");
    return 0;
}
