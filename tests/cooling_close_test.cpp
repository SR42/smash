// cooling_close_test.cpp - in full mode, allocation churn must not refill the
// free slots of pages that are going cold (SMASH_COOLING_CLOSE_TICKS, default
// half the cold timeout), so those pages can finish cooling and be compressed.
//
// The case the mechanism exists for: a call site leaves one long-lived object
// on each of its pages and frees the rest, then keeps allocating and freeing
// short-lived objects. Each refill that lands in one of the long-lived
// objects' pages writes to it and restarts its cold count, so with steady
// churn those pages never get compressed.
//
// Each case runs in its own child process, everything on one thread (the
// arena hash includes a per-thread id):
//   0. write 64 KiB of 1 KiB objects from another call site and wait until one
//      of their pages is compressed: until any page has been, the allocator
//      skips the cooling check altogether (g_any_page_compressed);
//   1. allocate 4096 + 4096 objects of 256 bytes from one call site; keep the
//      first object of each page of the first 4096 (the survivors, 256
//      pages), free every other object (the second 4096 leave empty spans);
//   2. wait until every survivor page has been idle for the closing threshold
//      (smash_page_info's cold count >= 2 at SMASH_COLD_TIMEOUT_SEC=4);
//   3. churn from the same call site for 12 s: grow a live set to 2048
//      objects, 64 every 20 ms, each written, then free them all, and again;
//      count the allocations that land on a survivor page;
//   4. read the survivor pages' states, the SIGUSR2 stats line's reheat=, and
//      the survivors' bytes.
// A churn that frees each burst at once is no test: it reuses the same few
// slots and leaves the other survivor pages alone either way (measured: 238 of
// 256 compressed with closing on and off). Growing a live set needs fresh
// slots every cycle, and the partial spans' holes are the first offered.
// Checks:
//   - default: <= 1% of the churn's allocations land on survivor pages and
//     >= 90% of the survivor pages are compressed;
//   - SMASH_COOLING_CLOSE_TICKS=0 (closing off), the control: >= 1000 land
//     there and <= 70% get compressed, and reheat= counts the refills
//     (>= 100); the default's reheat= is under a tenth of that;
//   - every survivor reads back intact.
#include "placement_test_util.h"

#include <fcntl.h>
#include <set>
#include <signal.h>
#include <vector>

using namespace placement;

namespace {

constexpr size_t kObj = 256, kCount = 4096, kStep = 64, kLive = 2048;

static volatile uintptr_t g_sink;

__attribute__((noinline)) void* site() {
    void* p = malloc(kObj);
    g_sink = reinterpret_cast<uintptr_t>(p);
    return p;
}

__attribute__((noinline)) void* otherSite() {
    void* p = malloc(1024);
    g_sink = reinterpret_cast<uintptr_t>(p) + 1;
    return p;
}

unsigned char want(size_t obj, size_t i) { return i < 64 ? static_cast<unsigned char>(obj * 5 + i) : 0; }

int child() {
    const PageInfoFn page_info = pageInfoFn();
    if (!page_info) {
        std::printf("child: smash_page_info not found\n");
        return 2;
    }
    char path[] = "/tmp/smash_coolclose_XXXXXX";
    const int log = mkstemp(path);
    if (log < 0) {
        std::printf("child: mkstemp failed: %s\n", std::strerror(errno));
        return 2;
    }
    unlink(path);

    // Step 0: get some page compressed.
    std::vector<unsigned char*> other(64);
    for (auto*& o : other) {
        o = static_cast<unsigned char*>(otherSite());
        std::memset(o, 0x11, 64);
        std::memset(o + 64, 0, 1024 - 64);
    }
    bool latched = false;
    for (int s = 0; s < 600 && !latched; ++s) {
        for (auto* o : other) {
            int st = -1;
            if (page_info(o, &st, nullptr, nullptr) == 0 && compressedState(st)) latched = true;
        }
        if (!latched) usleep(50 * 1000);
    }

    std::vector<unsigned char*> objs(2 * kCount);
    for (size_t k = 0; k < objs.size(); ++k) {
        objs[k] = static_cast<unsigned char*>(site());
        for (size_t i = 0; i < kObj; ++i) objs[k][i] = want(k, i);
    }
    std::vector<size_t> survivors;
    std::set<uintptr_t> pages;
    for (size_t k = 0; k < kCount; ++k) {
        // One survivor per page, wholly inside it.
        const uintptr_t pg = pageOf(objs[k]);
        if (pageOf(objs[k] + kObj - 1) == pg && !pages.count(pg)) {
            pages.insert(pg);
            survivors.push_back(k);
        }
    }
    std::vector<bool> keep(objs.size());
    for (size_t k : survivors) keep[k] = true;
    for (size_t k = 0; k < objs.size(); ++k)
        if (!keep[k]) free(objs[k]);

    auto pageStates = [&](size_t* compressed, int* min_cold) {
        *compressed = 0;
        *min_cold = 255;
        for (uintptr_t pg : pages) {
            int st = -1, cold = 0;
            if (page_info(reinterpret_cast<void*>(pg), &st, &cold, nullptr) != 0) continue;
            if (compressedState(st)) ++*compressed;
            else if (cold < *min_cold) *min_cold = cold;
        }
    };

    // Wait for the survivor pages to cool past the closing threshold.
    size_t compressed = 0;
    int min_cold = 0;
    const double t0 = nowSec();
    while (nowSec() - t0 < 30.0) {
        pageStates(&compressed, &min_cold);
        if (min_cold >= 2) break;
        usleep(50 * 1000);
    }
    const double cooled_at = nowSec() - t0;
    const size_t compressed_before = compressed;
    const int min_cold_before = min_cold;

    // Churn from the same call site.
    size_t allocs = 0, landed = 0;
    std::vector<unsigned char*> live;
    live.reserve(kLive);
    const double t1 = nowSec();
    while (nowSec() - t1 < 12.0) {
        for (size_t j = 0; j < kStep; ++j) {
            auto* b = static_cast<unsigned char*>(site());
            std::memset(b, 0x77, kObj);
            ++allocs;
            landed += pages.count(pageOf(b)) || pages.count(pageOf(b + kObj - 1));
            live.push_back(b);
        }
        if (live.size() >= kLive) {
            for (auto* b : live) free(b);
            live.clear();
        }
        usleep(20 * 1000);
    }
    for (auto* b : live) free(b);
    pageStates(&compressed, &min_cold);

    const int saved = dup(2);
    dup2(log, 2);
    raise(SIGUSR2);
    usleep(100 * 1000);
    dup2(saved, 2);
    std::string text;
    char buf[8192];
    lseek(log, 0, SEEK_SET);
    for (ssize_t n; (n = read(log, buf, sizeof(buf))) > 0;) text.append(buf, static_cast<size_t>(n));
    long reheat = -1;
    const size_t at = text.rfind(" reheat=");
    if (at != std::string::npos) reheat = std::atol(text.c_str() + at + 8);

    size_t bad = 0;
    for (size_t k : survivors)
        for (size_t i = 0; i < kObj; ++i) bad += objs[k][i] != want(k, i);
    for (auto* o : other) free(o);
    std::printf("result latched=%d pages=%zu cooled_s=%.1f min_cold=%d compressed_before=%zu allocs=%zu landed=%zu "
                "compressed=%zu reheat=%ld bad=%zu\n",
                latched ? 1 : 0, pages.size(), cooled_at, min_cold_before, compressed_before, allocs, landed, compressed, reheat, bad);
    for (size_t k : survivors) free(objs[k]);
    return 0;
}

struct Result {
    bool ok = false;
    int latched = 0;
    size_t pages = 0, compressed_before = 0, allocs = 0, landed = 0, compressed = 0, bad = 0;
    double cooled = 0;
    int min_cold = 0;
    long reheat = -1;
};

Result run(const char* self, const char* env) {
    Result r;
    const std::string cmd = std::string("timeout 120 env -u SMASH_COOLING_CLOSE_TICKS ") + env + " '" + self +
                            "' child 2>&1";
    int st = 0;
    const std::string out = runChild(cmd, &st);
    const std::string line = taggedLine(out, "result ");
    r.ok = st == 0 &&
           std::sscanf(line.c_str(),
                       "result latched=%d pages=%zu cooled_s=%lf min_cold=%d compressed_before=%zu allocs=%zu landed=%zu "
                       "compressed=%zu reheat=%ld bad=%zu",
                       &r.latched, &r.pages, &r.cooled, &r.min_cold, &r.compressed_before, &r.allocs, &r.landed,
                       &r.compressed, &r.reheat, &r.bad) == 10;
    std::printf("%-28s %zu survivor pages cooled to %d in %.1f s (%zu already compressed); churn: %zu of %zu "
                "allocations on them; then %zu of %zu compressed; reheat=%ld; %zu bad bytes (status %d)\n",
                *env ? env : "(default)", r.pages, r.min_cold, r.cooled, r.compressed_before, r.landed, r.allocs,
                r.compressed, r.pages, r.reheat, r.bad, st);
    if (!r.ok) showTail(out);
    return r;
}

}  // namespace

int main(int argc, char** argv) {
    setvbuf(stdout, nullptr, _IONBF, 0);
    if (argc > 1 && std::strcmp(argv[1], "child") == 0) return child();
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) {
        std::printf("SKIP: not running under libsmash\n");
        return 0;
    }
    char self[PATH_MAX];
    if (!selfPath(self)) return 1;
    int failures = 0;
    auto fail = [&](const char* c, const std::string& why) {
        std::printf("FAIL: %s: %s\n", c, why.c_str());
        ++failures;
    };

    const Result d = run(self, "");
    const Result c = run(self, "SMASH_COOLING_CLOSE_TICKS=0");
    for (const Result* r : {&d, &c}) {
        const char* name = r == &d ? "default" : "closing off";
        if (!r->ok) { fail(name, "the child did not report"); continue; }
        if (!r->latched) fail(name, "no page of the other call site was ever compressed");
        if (r->pages < 200) fail(name, "only " + std::to_string(r->pages) + " survivor pages");
        if (r->min_cold < 2) fail(name, "survivor pages never cooled to the closing threshold");
        if (r->bad) fail(name, std::to_string(r->bad) + " survivor bytes read back wrong");
    }
    if (d.ok) {
        if (d.landed * 100 > d.allocs)
            fail("default", std::to_string(d.landed) + " of " + std::to_string(d.allocs) +
                                " churn allocations refilled pages going cold");
        if (d.compressed * 10 < d.pages * 9)
            fail("default", "only " + std::to_string(d.compressed) + " of " + std::to_string(d.pages) +
                                " survivor pages compressed under churn");
    }
    if (c.ok) {
        if (c.landed < 1000)
            fail("closing off", "control: only " + std::to_string(c.landed) +
                                    " churn allocations on survivor pages; the case does not refill them");
        if (c.compressed * 10 > c.pages * 7)
            fail("closing off", "control: " + std::to_string(c.compressed) + " of " + std::to_string(c.pages) +
                                    " survivor pages compressed though churn refilled them");
        if (c.reheat < 100) fail("closing off", "control: reheat=" + std::to_string(c.reheat) + ", want >= 100");
    }
    if (d.ok && c.ok && d.reheat * 10 >= c.reheat)
        fail("default", "reheat=" + std::to_string(d.reheat) + ", not under a tenth of closing off's " +
                            std::to_string(c.reheat));
    if (failures) return 1;
    std::printf("cooling_close_test: all checks passed\n");
    return 0;
}
