// fail_backoff_test.cpp - a page whose compression failed backs off, under
// every write tracker (issue #52).
//
// A page that fails the ratio gate, or whose blob the store refuses, has its
// cold streak restarted and its back-off count raised: the next attempt
// needs twice the idle time, up to kMaxEffectiveFloorTicks. An access clears
// the count, because the content may have changed.
//
// Under the scanning trackers (SMASH_WRITE_TRACK=idle, yt) an access is
// whatever set the page's accessed bit since its last scan, and the
// compressor's own snapshot of the page does that. So a failed attempt
// cleared its own back-off at the page's next scan, and the page went to the
// codec again one cold floor later, for as long as it stayed cold. The
// snapshot's read is now recorded, and phase 1 keeps the count through the
// scan that reports it.
//
// FB_MODE=ratio: 16 MiB of random bytes with the ROI gate off
//   (SMASH_ROI_THRESHOLD=0), so every attempt runs the codec and fails the
//   ratio gate. Counted: gate_fail of the pipeline line.
// FB_MODE=store: 16 MiB of half-compressible bytes against a blob store
//   capped at 2 MiB (SMASH_BLOB_MAX_MIB=2). Once the store is full every
//   attempt is refused. Counted: the refusals (smash_resilience_info), for
//   the pages that are not compressed at the end.
//
// In both, the count is read 60 s after every page has been tried once, and
// may be at most FB_MAX_PER_PAGE attempts per page (8). With a 2 s timeout
// and a scan every tick the attempts come about 3, 8, 17, 34 and 67 s after
// the page went cold: five in the window. Without the back-off there is one
// every 4 s: sixteen.
//
// FB_NEED_TRACK=idle|yt: the registration is for that tracker (root; yt
// with its module) and skips when another one is in use. The registrations
// without it run the default tracker, where the count has always held: they
// pin the limit where every host runs it.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <dlfcn.h>
#include <string>
#include <unistd.h>

namespace {
using info_fn = int (*)(const void*, int*, int*, int*);
using res_fn = int (*)(uint64_t*, uint64_t*);
using track_fn = const char* (*)();
info_fn page_info = nullptr;
res_fn resilience = nullptr;
track_fn tracker = nullptr;
int g_log_fd = -1, g_saved_err = -1;

constexpr size_t kPage = 4096;
constexpr size_t kBlock = size_t{16} << 20;
constexpr size_t kPages = kBlock / kPage;

struct Rng {
    uint64_t s;
    uint64_t next() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return s; }
};

struct Counters { unsigned long long tried = 0, stored = 0, gate_fail = 0, roi_skip = 0; bool ok = false; };

Counters readCounters() {
    raise(SIGUSR2);
    usleep(100 * 1000);
    std::string lg;
    char buf[8192];
    lseek(g_log_fd, 0, SEEK_SET);
    for (ssize_t n; (n = read(g_log_fd, buf, sizeof buf)) > 0;) lg.append(buf, static_cast<size_t>(n));
    Counters c;
    const size_t at = lg.rfind("compress tried=");
    if (at != std::string::npos)
        c.ok = std::sscanf(lg.c_str() + at, "compress tried=%llu stored=%llu gate_fail=%llu roi_skip=%llu",
                           &c.tried, &c.stored, &c.gate_fail, &c.roi_skip) == 4;
    return c;
}

unsigned long long storeFails() {
    uint64_t f = 0, r = 0;
    resilience(&f, &r);
    return f;
}

size_t compressedPages(const unsigned char* p) {
    size_t c = 0;
    for (size_t off = 0; off < kBlock; off += kPage) {
        int st = 0, cold = 0, rc = 0;
        if (page_info(p + off, &st, &cold, &rc) == 0 && (st == 2 || st == 5)) ++c;
    }
    return c;
}

// 32 random bytes and 32 zeros per 64: about half its size compressed, well
// under the ratio gate.
void fillHalf(unsigned char* p, Rng& rng) {
    for (size_t i = 0; i < kBlock; i += 64) {
        for (int k = 0; k < 4; ++k) {
            const uint64_t v = rng.next();
            std::memcpy(p + i + 8 * k, &v, 8);
        }
        std::memset(p + i + 32, 0, 32);
    }
}

int fail(const char* fmt, unsigned long long a = 0, unsigned long long b = 0, unsigned long long c = 0) {
    dprintf(g_saved_err, "FAIL: ");
    dprintf(g_saved_err, fmt, a, b, c);
    dprintf(g_saved_err, "\n");
    return 1;
}

unsigned long long envNum(const char* name, unsigned long long dflt) {
    const char* v = std::getenv(name);
    return v && *v ? std::strtoull(v, nullptr, 10) : dflt;
}
}  // namespace

int main() {
    page_info = reinterpret_cast<info_fn>(dlsym(RTLD_DEFAULT, "smash_page_info"));
    resilience = reinterpret_cast<res_fn>(dlsym(RTLD_DEFAULT, "smash_resilience_info"));
    tracker = reinterpret_cast<track_fn>(dlsym(RTLD_DEFAULT, "smash_write_tracker"));
    if (!page_info || !resilience || !tracker) {
        std::fprintf(stderr, "FAIL: not running under libsmash (diagnostic exports missing)\n");
        return 1;
    }
    char path[] = "/tmp/smash_fail_backoff_XXXXXX";
    g_log_fd = mkstemp(path);
    if (g_log_fd < 0) { std::printf("SKIP: mkstemp failed\n"); return 77; }
    unlink(path);
    g_saved_err = dup(2);
    dup2(g_log_fd, 2);

    const char* mode_env = std::getenv("FB_MODE");
    const bool store = mode_env && std::strcmp(mode_env, "store") == 0;
    const unsigned long long max_per_page = envNum("FB_MAX_PER_PAGE", 8);

    Rng rng{88172645463325252ull};
    const Rng seed = rng;
    auto* p = static_cast<unsigned char*>(std::malloc(kBlock));
    if (!p) return fail("malloc");
    if (store) fillHalf(p, rng);
    else for (size_t i = 0; i < kBlock / 8; ++i) reinterpret_cast<uint64_t*>(p)[i] = rng.next();

    // The tracker is chosen at the compressor's first tick with something to
    // track.
    std::string track = tracker();
    for (int i = 0; i < 40 && track == "unprobed"; ++i) {
        usleep(250 * 1000);
        track = tracker();
    }
    if (const char* need = std::getenv("FB_NEED_TRACK"); need && *need && track != need) {
        dprintf(g_saved_err, "SKIP: this registration needs SMASH_WRITE_TRACK=%s and the tracker in use is '%s' "
                "(not root, or the kernel has no such tracker)\n", need, track.c_str());
        return 77;
    }

    // Every page tried once.
    Counters c;
    unsigned long long first = 0;
    int waited = 0;
    for (; waited < 60; ++waited) {
        sleep(1);
        c = readCounters();
        first = store ? c.stored + storeFails() : c.gate_fail;
        if (c.ok && first >= kPages * 95 / 100) break;
    }
    if (!c.ok) return fail("no pipeline line on SIGUSR2 (SMASH_STATS_SIGNAL=1 not set?)");
    if (first < kPages * 95 / 100)
        return fail("%llu of %llu pages were tried in 60 s: nothing to count", first, kPages);
    dprintf(g_saved_err, "track=%s %s: every page tried after %d s (tried=%llu stored=%llu gate_fail=%llu "
            "roi_skip=%llu refused=%llu)\n", track.c_str(), store ? "store" : "ratio", waited + 1, c.tried, c.stored,
            c.gate_fail, c.roi_skip, storeFails());

    sleep(60);
    c = readCounters();
    const unsigned long long refused = storeFails();
    unsigned long long attempts, pages;
    if (store) {
        pages = kPages - compressedPages(p);
        attempts = refused;
        if (pages < kPages / 4)
            return fail("only %llu of %llu pages are still refused: the store is not capped (SMASH_BLOB_MAX_MIB)",
                        pages, kPages);
    } else {
        pages = kPages;
        attempts = c.gate_fail;
        if (c.roi_skip > kPages / 8)
            return fail("the ROI gate declined %llu times: it is not off (SMASH_ROI_THRESHOLD=0), so the codec "
                        "attempts are not what limits the count", c.roi_skip);
    }
    dprintf(g_saved_err, "60 s later: %llu attempts on %llu pages, %llu.%02llu per page (limit %llu)\n", attempts,
            pages, attempts / pages, attempts * 100 / pages % 100, max_per_page);
    if (attempts < 2 * pages)
        return fail("%llu attempts on %llu pages in 60 s: fewer than two each, the compressor stopped trying",
                    attempts, pages);
    if (attempts > max_per_page * pages)
        return fail("%llu attempts on %llu pages whose compression fails every time (limit %llu per page): "
                    "they do not back off", attempts, pages, max_per_page);

    // Nothing lost on the way.
    Rng chk = seed;
    if (store) {
        for (size_t i = 0; i < kBlock; i += 64) {
            for (int k = 0; k < 4; ++k) {
                uint64_t got;
                std::memcpy(&got, p + i + 8 * k, 8);
                if (got != chk.next()) return fail("byte %llu changed", i + 8 * static_cast<unsigned>(k));
            }
            for (int k = 32; k < 64; ++k)
                if (p[i + k]) return fail("byte %llu changed", i + static_cast<unsigned>(k));
        }
    } else {
        for (size_t i = 0; i < kBlock / 8; ++i)
            if (reinterpret_cast<uint64_t*>(p)[i] != chk.next()) return fail("word %llu changed", i);
    }
    std::free(p);
    dprintf(g_saved_err, "OK\n");
    return 0;
}
