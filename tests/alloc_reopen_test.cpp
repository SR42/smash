// alloc_reopen_test.cpp - in full mode, a compressed page restored because
// the allocator handed out a free slot on it must not count as recompression
// thrash: neither for the page (its recompress count) nor for its call site's
// fresh pages (the bucket bias).
//
// The allocator reopens a compressed page two ways: the thread cache returns a
// slot freed into it before the page went cold, and the slab, when a span's
// only free slots are on cooling or compressed pages and no empty span is
// left, hands out one of those (consumeFallback). Either way the program's
// first write to the new object faults the page back. That restore used to
// bump the page's recompress count and count for the bucket bias like a page that
// was compressed too early: the page then waited 4x the cold timeout before it
// was compressed again, and every fresh page of the call site 2x.
//
// One child process, one thread (the arena hash includes a per-thread id),
// SMASH_COLD_TIMEOUT_SEC=3, SMASH_PREFETCH_WINDOW=0:
//   1. 2048 objects of 256 bytes from call site C, written; the first object
//      of each page stays live, the rest are freed. Wait until 90% of those
//      pages are compressed.
//   2. Allocate 2048 objects from C and write each. The first ~1900 land on
//      the compressed pages ("reopened": compressed when malloc returned the
//      slot), the rest on new pages. Then at once 2048 "C fresh" objects from
//      C (new pages) and 2048 "R fresh" from call site R, routed to another
//      arena.
//   3. Poll smash_page_info every 100 ms and note when each page is first
//      compressed again.
// Checks:
//   - at least 64 pages reopened, and none of them has a recompress count;
//   - medians: reopened and C fresh within 1.5 s of R fresh (before the fix:
//     13.1 and 7.1 s against 5.1 s);
//   - every object reads back intact.
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
    return i < 64 ? static_cast<unsigned char>(set * 31 + k * 3 + i + 1) : 0;
}

void fill(unsigned char* p, size_t set, size_t k) {
    for (size_t i = 0; i < kObj; ++i) p[i] = want(set, k, i);
}

size_t badBytes(const std::vector<unsigned char*>& v, size_t set) {
    size_t bad = 0;
    for (size_t k = 0; k < v.size(); ++k)
        if (v[k])
            for (size_t i = 0; i < kObj; ++i) bad += v[k][i] != want(set, k, i);
    return bad;
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

}  // namespace

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    const PageInfoFn page_info = pageInfoFn();
    const ArenaForFn arena_for = arenaForFn();
    if (!page_info || !arena_for) {
        std::printf("SKIP: not running under libsmash\n");
        return 0;
    }
    auto state = [&](uintptr_t pg, int* rc = nullptr) {
        int st = -1;
        page_info(reinterpret_cast<void*>(pg), &st, nullptr, rc);
        return st;
    };

    // Step 1.
    std::vector<unsigned char*> c1(kCount);
    for (size_t k = 0; k < kCount; ++k) {
        c1[k] = static_cast<unsigned char*>(kSites[0]());
        fill(c1[k], 1, k);
    }
    std::set<uintptr_t> seen;
    std::vector<uintptr_t> survivor_pages;
    for (size_t k = 0; k < kCount; ++k) {
        const uintptr_t pg = pageOf(c1[k]);
        if (pageOf(c1[k] + kObj - 1) == pg && !seen.count(pg)) {
            seen.insert(pg);
            survivor_pages.push_back(pg);
        } else {
            free(c1[k]);
            c1[k] = nullptr;
        }
    }
    size_t compressed = 0;
    const double s0 = nowSec();
    while (nowSec() - s0 < 30.0) {
        compressed = 0;
        for (uintptr_t pg : survivor_pages) compressed += state(pg) == kCompressed;
        if (compressed * 10 >= survivor_pages.size() * 9) break;
        usleep(100 * 1000);
    }

    // Step 2. Site R is found by allocating one object only now, for the
    // reason test_thrash_backoff gives: a probe freed early leaves a slot that
    // would itself be reopened.
    const int c_arena = arena_for(survivor_pages.empty() ? nullptr : reinterpret_cast<void*>(survivor_pages[0]));
    std::set<uintptr_t> reopened_set;
    std::vector<unsigned char*> c2(kCount);
    for (size_t k = 0; k < kCount; ++k) {
        c2[k] = static_cast<unsigned char*>(kSites[0]());
        const uintptr_t pg = pageOf(c2[k]);
        if (state(pg) == kCompressed) reopened_set.insert(pg);
        fill(c2[k], 2, k);
    }
    SiteFn r_site = nullptr;
    int r_arena = -1;
    for (SiteFn s : kSites) {
        void* p = s();
        const int a = arena_for(p);
        free(p);
        if (a != c_arena) {
            r_site = s;
            r_arena = a;
            break;
        }
    }
    if (!r_site) {
        std::printf("FAIL: no call site routed away from arena %d\n", c_arena);
        return 1;
    }
    const double start = nowSec();
    std::vector<unsigned char*> c3(kCount), r3(kCount);
    for (size_t k = 0; k < kCount; ++k) {
        c3[k] = static_cast<unsigned char*>(kSites[0]());
        fill(c3[k], 3, k);
    }
    for (size_t k = 0; k < kCount; ++k) {
        r3[k] = static_cast<unsigned char*>(r_site());
        fill(r3[k], 4, k);
    }
    const std::vector<uintptr_t> reopened(reopened_set.begin(), reopened_set.end());
    size_t with_rc = 0;
    for (uintptr_t pg : reopened) {
        int rc = 0;
        state(pg, &rc);
        with_rc += rc > 0;
    }
    // Fresh pages: only those holding no object of an earlier set.
    std::set<uintptr_t> older(survivor_pages.begin(), survivor_pages.end());
    for (uintptr_t pg : pagesOf(c2)) older.insert(pg);
    std::vector<uintptr_t> c_fresh;
    for (uintptr_t pg : pagesOf(c3))
        if (!older.count(pg)) c_fresh.push_back(pg);
    const std::vector<uintptr_t> r_fresh = pagesOf(r3);

    // Step 3.
    struct Group { const std::vector<uintptr_t>* pages; std::vector<double> when; };
    Group groups[3] = {{&reopened, {}}, {&c_fresh, {}}, {&r_fresh, {}}};
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
    const size_t bad = badBytes(c1, 1) + badBytes(c2, 2) + badBytes(c3, 3) + badBytes(r3, 4);
    std::printf("arenas C=%d R=%d; %zu of %zu survivor pages compressed; %zu pages reopened by the allocator, %zu "
                "of them with a recompress count; compressed again after (median): reopened %.1f s, C fresh %.1f s "
                "(%zu pages), R fresh %.1f s (%zu pages); never: %zu/%zu/%zu; %zu bad bytes\n",
                c_arena, r_arena, compressed, survivor_pages.size(), reopened.size(), with_rc, med[0], med[1],
                c_fresh.size(), med[2], r_fresh.size(), never[0], never[1], never[2], bad);

    int failures = 0;
    char buf[200];
    auto fail = [&](const char* why) {
        std::printf("FAIL: %s\n", why);
        ++failures;
    };
    if (compressed * 10 < survivor_pages.size() * 9) fail("the survivor pages were not compressed within 30 s");
    if (reopened.size() < 64) {
        std::snprintf(buf, sizeof buf, "only %zu compressed pages reopened by the allocator", reopened.size());
        fail(buf);
    }
    if (with_rc) {
        std::snprintf(buf, sizeof buf, "%zu of %zu pages reopened by the allocator got a recompress count", with_rc,
                      reopened.size());
        fail(buf);
    }
    if (c_fresh.size() < 64) fail("too few fresh pages from call site C");
    if (never[0] + never[1] + never[2]) fail("some pages never compressed again within 60 s");
    if (med[0] > med[2] + 1.5) {
        std::snprintf(buf, sizeof buf, "reopened pages compressed again after %.1f s, R's fresh pages after %.1f s",
                      med[0], med[2]);
        fail(buf);
    }
    if (med[1] > med[2] + 1.5) {
        std::snprintf(buf, sizeof buf, "C's fresh pages compressed after %.1f s, R's after %.1f s: bucket bias from "
                      "allocator restores", med[1], med[2]);
        fail(buf);
    }
    if (bad) fail("objects read back wrong");
    for (auto* v : {&c1, &c2, &c3, &r3})
        for (auto* p : *v) free(p);
    if (failures) return 1;
    std::printf("alloc_reopen_test: all checks passed\n");
    return 0;
}
