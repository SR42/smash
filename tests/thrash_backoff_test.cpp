// thrash_backoff_test.cpp - in full mode, a page faulted back from compression
// must wait longer before it is compressed again, and so must fresh pages of
// the same call site (bucket = arena x size class), while another call site's
// fresh pages keep the plain cold timeout.
//
// The recompression-thrash back-off: a restored page's effective floor is
// floor << (recompress count + bucket bias), the bias 1 once any page of its
// bucket has been restored. test_compression's testRecompressBackoff pins the
// per-page schedule tick by tick, but has no spans, so it cannot see the
// bucket bias; this runs the whole allocator.
//
// Each case runs in its own child process, everything on one thread (the
// arena hash includes a per-thread id). SMASH_COLD_TIMEOUT_SEC=3 (floor 3
// ticks of 1 s), SMASH_PREFETCH_WINDOW=0 (a prefetched neighbour is restored
// without a fault, so without its own count):
//   1. 2048 objects of 256 bytes from call site T, written; wait until 90% of
//      their pages are reclaimed (COMPRESSED), then read every object: each
//      page faults back with recompress count 1 ("restored");
//   2. at once, 2048 fresh objects from T ("T fresh") and 2048 from call site
//      C, routed to another arena ("C fresh"), written;
//   3. poll smash_page_info every 100 ms and note when each page is first
//      compressed again.
// Checks (medians, seconds from step 2):
//   - default: T fresh >= C fresh + 1.5 (2x the floor against 1x: 6 vs 4
//     ticks), restored >= C fresh + 5.5 (4x: 12 ticks);
//   - SMASH_RECOMPRESS_BACKOFF=0: all three within 1.5 s of each other;
//   - every object reads back intact.
// Before the decay fix, the restored pages' count decayed during the wait
// (after kRcDecayColdTicks = 8 idle ticks), so they compressed at 8, not 12.
#include "placement_test_util.h"

#include <algorithm>
#include <set>
#include <vector>

using namespace placement;

namespace {

constexpr size_t kObj = 256, kCount = 2048;

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

unsigned char want(size_t set, size_t k, size_t i) {
    return i < 64 ? static_cast<unsigned char>(set * 31 + k * 3 + i) : 0;
}

std::vector<unsigned char*> allocSet(SiteFn s, size_t set) {
    std::vector<unsigned char*> v(kCount);
    for (size_t k = 0; k < kCount; ++k) {
        v[k] = static_cast<unsigned char*>(s());
        for (size_t i = 0; i < kObj; ++i) v[k][i] = want(set, k, i);
    }
    return v;
}

std::vector<uintptr_t> pagesOf(const std::vector<unsigned char*>& v) {
    std::set<uintptr_t> s;
    for (auto* p : v) {
        s.insert(pageOf(p));
        s.insert(pageOf(p + kObj - 1));
    }
    return {s.begin(), s.end()};
}

double median(std::vector<double> v) {
    if (v.empty()) return -1;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

int child() {
    const ArenaForFn arena_for = arenaForFn();
    const PageInfoFn page_info = pageInfoFn();
    if (!arena_for || !page_info) {
        std::printf("child: smash_arena_for/smash_page_info not found\n");
        return 2;
    }
    auto state = [&](uintptr_t pg, int* rc = nullptr) {
        int st = -1;
        page_info(reinterpret_cast<void*>(pg), &st, nullptr, rc);
        return st;
    };

    // Step 1.
    auto t1 = allocSet(kSites[0], 1);
    const auto t1_pages = pagesOf(t1);
    size_t reclaimed = 0;
    const double s0 = nowSec();
    while (nowSec() - s0 < 30.0) {
        reclaimed = 0;
        for (uintptr_t pg : t1_pages) reclaimed += state(pg) == kCompressed;
        if (reclaimed * 10 >= t1_pages.size() * 9) break;
        usleep(100 * 1000);
    }
    size_t bad = 0;
    for (size_t k = 0; k < kCount; ++k)
        for (size_t i = 0; i < kObj; ++i) bad += t1[k][i] != want(1, k, i);
    std::vector<uintptr_t> restored;
    for (uintptr_t pg : t1_pages) {
        int rc = 0;
        if (!compressedState(state(pg, &rc)) && rc >= 1) restored.push_back(pg);
    }

    // Step 2. Find site C only now and allocate from it at once: a probe
    // object freed early leaves a slot on a page that goes cold and is
    // compressed, the allocator hands that slot out again rather than open a
    // new span, and the first write to it faults the page back -- a restore
    // the bucket bias counts like a thrash (measured: C's fresh pages then
    // waited 2x the floor too).
    const int t_arena = arena_for(t1[0]);
    SiteFn c_site = nullptr;
    int c_arena = -1;
    for (SiteFn s : kSites) {
        void* p = s();
        const int a = arena_for(p);
        free(p);
        if (a != t_arena) {
            c_site = s;
            c_arena = a;
            break;
        }
    }
    if (!c_site) {
        std::printf("child: no call site routed away from arena %d\n", t_arena);
        return 2;
    }
    const double start = nowSec();
    auto t2 = allocSet(kSites[0], 2);
    auto c2 = allocSet(c_site, 3);
    const auto t2_pages = pagesOf(t2), c2_pages = pagesOf(c2);

    // Step 3.
    struct Group { const std::vector<uintptr_t>* pages; std::vector<double> when; };
    Group groups[3] = {{&restored, {}}, {&t2_pages, {}}, {&c2_pages, {}}};
    for (Group& g : groups) g.when.assign(g.pages->size(), -1);
    while (nowSec() - start < 60.0) {
        bool all = true;
        for (Group& g : groups)
            for (size_t j = 0; j < g.pages->size(); ++j) {
                if (g.when[j] >= 0) continue;
                if (compressedState(state((*g.pages)[j]))) g.when[j] = nowSec() - start;
                else all = false;
            }
        if (all) break;
        usleep(100 * 1000);
    }
    double med[3];
    size_t never[3];
    for (int g = 0; g < 3; ++g) {
        std::vector<double> got;
        for (double w : groups[g].when)
            if (w >= 0) got.push_back(w);
        never[g] = groups[g].when.size() - got.size();
        med[g] = median(got);
    }
    for (size_t k = 0; k < kCount; ++k)
        for (size_t i = 0; i < kObj; ++i)
            bad += (t1[k][i] != want(1, k, i)) + (t2[k][i] != want(2, k, i)) + (c2[k][i] != want(3, k, i));
    std::printf("result t_arena=%d c_arena=%d t1_pages=%zu reclaimed=%zu restored=%zu restored_s=%.1f t_fresh_s=%.1f "
                "c_fresh_s=%.1f never=%zu/%zu/%zu bad=%zu\n",
                t_arena, c_arena, t1_pages.size(), reclaimed, restored.size(), med[0], med[1], med[2], never[0],
                never[1], never[2], bad);
    for (auto* v : {&t1, &t2, &c2})
        for (auto* p : *v) free(p);
    return 0;
}

struct Result {
    bool ok = false;
    int t_arena = 0, c_arena = 0;
    size_t t1_pages = 0, reclaimed = 0, restored = 0, never[3] = {}, bad = 0;
    double restored_s = -1, t_fresh_s = -1, c_fresh_s = -1;
};

Result run(const char* self, const char* env) {
    Result r;
    const std::string cmd = std::string("timeout 150 env -u SMASH_RECOMPRESS_BACKOFF ") + env + " '" + self +
                            "' child 2>&1";
    int st = 0;
    const std::string out = runChild(cmd, &st);
    const std::string line = taggedLine(out, "result ");
    r.ok = st == 0 &&
           std::sscanf(line.c_str(),
                       "result t_arena=%d c_arena=%d t1_pages=%zu reclaimed=%zu restored=%zu restored_s=%lf "
                       "t_fresh_s=%lf c_fresh_s=%lf never=%zu/%zu/%zu bad=%zu",
                       &r.t_arena, &r.c_arena, &r.t1_pages, &r.reclaimed, &r.restored, &r.restored_s, &r.t_fresh_s,
                       &r.c_fresh_s, &r.never[0], &r.never[1], &r.never[2], &r.bad) == 12;
    std::printf("%-28s %zu of %zu pages restored with a count; compressed again after (median): restored %.1f s, "
                "T fresh %.1f s, C fresh %.1f s; never: %zu/%zu/%zu; %zu bad bytes (status %d)\n",
                *env ? env : "(default)", r.restored, r.t1_pages, r.restored_s, r.t_fresh_s, r.c_fresh_s, r.never[0],
                r.never[1], r.never[2], r.bad, st);
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
    const Result o = run(self, "SMASH_RECOMPRESS_BACKOFF=0");
    for (const Result* r : {&d, &o}) {
        const char* name = r == &d ? "default" : "back-off off";
        if (!r->ok) { fail(name, "the child did not report"); continue; }
        if (r->restored * 10 < r->t1_pages * 8)
            fail(name, "only " + std::to_string(r->restored) + " of " + std::to_string(r->t1_pages) +
                           " pages restored with a recompress count");
        if (r->never[0] + r->never[1] + r->never[2]) fail(name, "some pages never compressed again within 60 s");
        if (r->bad) fail(name, std::to_string(r->bad) + " bytes read back wrong");
    }
    char buf[160];
    if (d.ok) {
        if (d.t_fresh_s < d.c_fresh_s + 1.5) {
            std::snprintf(buf, sizeof buf, "fresh pages of the restored call site compressed after %.1f s, "
                          "the other site's after %.1f s: no bucket bias", d.t_fresh_s, d.c_fresh_s);
            fail("default", buf);
        }
        if (d.restored_s < d.c_fresh_s + 5.5) {
            std::snprintf(buf, sizeof buf, "restored pages compressed again after %.1f s, fresh ones after %.1f s: "
                          "want >= 5.5 s more (4x the floor)", d.restored_s, d.c_fresh_s);
            fail("default", buf);
        }
    }
    if (o.ok) {
        const double lo = std::min({o.restored_s, o.t_fresh_s, o.c_fresh_s});
        const double hi = std::max({o.restored_s, o.t_fresh_s, o.c_fresh_s});
        if (hi - lo > 1.5) {
            std::snprintf(buf, sizeof buf, "control: medians %.1f/%.1f/%.1f s differ by more than 1.5 s",
                          o.restored_s, o.t_fresh_s, o.c_fresh_s);
            fail("back-off off", buf);
        }
    }
    if (failures) return 1;
    std::printf("thrash_backoff_test: all checks passed\n");
    return 0;
}
