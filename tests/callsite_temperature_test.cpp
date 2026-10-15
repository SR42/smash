// callsite_temperature_test.cpp - in full mode, objects from a call site whose
// data stays hot must not share pages with objects from a call site whose
// data goes cold, so the cold ones can be compressed while the hot ones are
// in use.
//
// test_arena_routing only checks that distinct call sites reach distinct
// arenas. This checks what that routing is for: two sites allocate 256-byte
// objects alternately (hot, cold, hot, cold, ... 4096 each, one thread); the
// hot site's objects are then written every 100 ms while the cold site's are
// left alone. Each case runs in its own child process.
//
// Checks, default arenas (the cold site is the first of 8 whose arena differs
// from the hot site's):
//   - no page holds objects of both sites;
//   - >= 90% of the cold site's pages are compressed (smash_page_info) within
//     20 s, and <= 5% of the hot site's;
//   - every object reads back intact.
// Control, SMASH_NUM_ARENAS=1 (every site in one arena, as with the
// call-site hash broken): >= 90% of the cold site's pages also hold hot
// objects and <= 10% of them get compressed -- which is what makes the first
// case a test of the routing rather than of the compressor.
#include "placement_test_util.h"

#include <set>
#include <vector>

using namespace placement;

namespace {

constexpr size_t kObj = 256, kCount = 4096;

static volatile uintptr_t g_sink;

#define SITE(n)                                                \
    __attribute__((noinline)) static void* site_##n() {       \
        void* p = malloc(kObj);                                \
        g_sink = reinterpret_cast<uintptr_t>(p) + (n);        \
        return p;                                              \
    }
SITE(0) SITE(1) SITE(2) SITE(3) SITE(4) SITE(5) SITE(6) SITE(7)
using SiteFn = void* (*)();
SiteFn kSites[] = {site_0, site_1, site_2, site_3, site_4, site_5, site_6, site_7};

unsigned char want(size_t obj, size_t i, bool hot) {
    return i < 64 ? static_cast<unsigned char>(obj * 3 + i + (hot ? 1 : 0)) : 0;
}

int child() {
    const ArenaForFn arena_for = arenaForFn();
    const PageInfoFn page_info = pageInfoFn();
    if (!arena_for || !page_info) {
        std::printf("child: smash_arena_for/smash_page_info not found\n");
        return 2;
    }
    // The hot site is site 0; the cold one the first whose arena differs.
    void* probe = kSites[0]();
    const int hot_arena = arena_for(probe);
    free(probe);
    SiteFn cold_site = kSites[1];
    int cold_arena = -1;
    for (SiteFn s : kSites) {
        void* p = s();
        const int a = arena_for(p);
        free(p);
        if (a != hot_arena) {
            cold_site = s;
            cold_arena = a;
            break;
        }
    }
    if (cold_arena < 0) cold_arena = hot_arena;

    std::vector<unsigned char*> hot(kCount), cold(kCount);
    for (size_t k = 0; k < kCount; ++k) {
        hot[k] = static_cast<unsigned char*>(kSites[0]());
        cold[k] = static_cast<unsigned char*>(cold_site());
        for (size_t i = 0; i < kObj; ++i) {
            hot[k][i] = want(k, i, true);
            cold[k][i] = want(k, i, false);
        }
    }
    std::set<uintptr_t> hot_pages, cold_pages;
    for (size_t k = 0; k < kCount; ++k) {
        hot_pages.insert(pageOf(hot[k]));
        hot_pages.insert(pageOf(hot[k] + kObj - 1));
        cold_pages.insert(pageOf(cold[k]));
        cold_pages.insert(pageOf(cold[k] + kObj - 1));
    }
    size_t mixed = 0;
    for (uintptr_t pg : cold_pages) mixed += hot_pages.count(pg);

    auto countCompressed = [&](const std::set<uintptr_t>& pages) {
        size_t n = 0;
        for (uintptr_t pg : pages) {
            int st = -1;
            if (page_info(reinterpret_cast<void*>(pg), &st, nullptr, nullptr) == 0 && compressedState(st)) ++n;
        }
        return n;
    };

    // Keep the hot objects hot: one write each every 100 ms.
    const double t0 = nowSec();
    size_t cold_done = 0, hot_done = 0;
    unsigned round = 0;
    while (nowSec() - t0 < 20.0) {
        ++round;
        for (size_t k = 0; k < kCount; ++k) hot[k][kObj - 1] = static_cast<unsigned char>(round);
        usleep(100 * 1000);
        if (round % 5 == 0) {
            cold_done = countCompressed(cold_pages);
            if (cold_done * 10 >= cold_pages.size() * 9 && nowSec() - t0 > 4.0) break;
        }
    }
    const double secs = nowSec() - t0;
    cold_done = countCompressed(cold_pages);
    hot_done = countCompressed(hot_pages);

    size_t bad = 0;
    for (size_t k = 0; k < kCount; ++k)
        for (size_t i = 0; i + 1 < kObj; ++i) bad += (hot[k][i] != want(k, i, true)) + (cold[k][i] != want(k, i, false));
    for (size_t k = 0; k < kCount; ++k) bad += cold[k][kObj - 1] != want(k, kObj - 1, false);
    std::printf("result hot_arena=%d cold_arena=%d hot_pages=%zu cold_pages=%zu mixed=%zu cold_compressed=%zu "
                "hot_compressed=%zu secs=%.1f bad=%zu\n",
                hot_arena, cold_arena, hot_pages.size(), cold_pages.size(), mixed, cold_done, hot_done, secs, bad);
    for (size_t k = 0; k < kCount; ++k) {
        free(hot[k]);
        free(cold[k]);
    }
    return 0;
}

struct Result {
    bool ok = false;
    int hot_arena = 0, cold_arena = 0;
    size_t hot_pages = 0, cold_pages = 0, mixed = 0, cold_done = 0, hot_done = 0, bad = 0;
    double secs = 0;
};

Result run(const char* self, const char* env) {
    Result r;
    const std::string cmd = std::string("timeout 120 env -u SMASH_NUM_ARENAS ") + env + " '" + self + "' child 2>&1";
    int st = 0;
    const std::string out = runChild(cmd, &st);
    const std::string line = taggedLine(out, "result ");
    r.ok = st == 0 && std::sscanf(line.c_str(),
                                  "result hot_arena=%d cold_arena=%d hot_pages=%zu cold_pages=%zu mixed=%zu "
                                  "cold_compressed=%zu hot_compressed=%zu secs=%lf bad=%zu",
                                  &r.hot_arena, &r.cold_arena, &r.hot_pages, &r.cold_pages, &r.mixed, &r.cold_done,
                                  &r.hot_done, &r.secs, &r.bad) == 9;
    std::printf("%-20s arenas hot=%d cold=%d; %zu of %zu cold pages shared with hot objects; compressed: %zu of "
                "%zu cold pages, %zu of %zu hot pages after %.1f s; %zu bad bytes (status %d)\n",
                *env ? env : "(default)", r.hot_arena, r.cold_arena, r.mixed, r.cold_pages, r.cold_done,
                r.cold_pages, r.hot_done, r.hot_pages, r.secs, r.bad, st);
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
    if (!d.ok) fail("default", "the child did not report");
    else {
        if (d.hot_arena == d.cold_arena) fail("default", "no site of 8 routed away from the hot site's arena");
        if (d.mixed) fail("default", std::to_string(d.mixed) + " cold pages also hold hot objects");
        if (d.cold_done * 10 < d.cold_pages * 9)
            fail("default", "only " + std::to_string(d.cold_done) + " of " + std::to_string(d.cold_pages) +
                                " cold pages compressed");
        if (d.hot_done * 20 > d.hot_pages)
            fail("default", std::to_string(d.hot_done) + " hot pages compressed while being written");
        if (d.bad) fail("default", std::to_string(d.bad) + " bytes read back wrong");
    }

    const Result c = run(self, "SMASH_NUM_ARENAS=1");
    if (!c.ok) fail("one arena", "the child did not report");
    else {
        if (c.mixed * 10 < c.cold_pages * 9)
            fail("one arena", "control: only " + std::to_string(c.mixed) + " of " + std::to_string(c.cold_pages) +
                                  " cold pages hold hot objects; the case does not mix sites without routing");
        if (c.cold_done * 10 > c.cold_pages)
            fail("one arena", "control: " + std::to_string(c.cold_done) +
                                  " cold pages compressed though they hold hot objects");
        if (c.bad) fail("one arena", std::to_string(c.bad) + " bytes read back wrong");
    }
    if (failures) return 1;
    std::printf("callsite_temperature_test: all checks passed\n");
    return 0;
}
