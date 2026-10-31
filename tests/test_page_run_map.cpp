// test_page_run_map.cpp - PageRunMap against a trivially correct model.
//
// The model is one owner-or-busy entry per page. Random frees (insert) and
// allocations (take) across several owners, checking after every operation:
//   - take() returns only pages that are free and belong to that owner;
//   - take() fails ONLY when the model has no contiguous free run of that
//     length for that owner. This is the coalescing check: without merging,
//     adjacent freed ranges would stay separate and take() would fail while
//     the model can satisfy the request. That is takeAny(): take() stops
//     looking after a bounded walk (checked at the end, with the walk's bound).
// Owners never mix: a run of one owner must not absorb its neighbour of
// another (node-bound pages must not leak into shared allocations).
#include "vm/page_run_map.h"

#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <random>
#include <vector>

using namespace smash;

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { fprintf(stderr, "FAIL: " __VA_ARGS__); fputc('\n', stderr); ++failures; } } while (0)

static constexpr int kBusy = -1;

// Longest contiguous free run of `owner` in the model.
static size_t longestRun(const std::vector<int>& m, int owner) {
    size_t best = 0, cur = 0;
    for (int v : m) { cur = (v == owner) ? cur + 1 : 0; if (cur > best) best = cur; }
    return best;
}

int main() {
    constexpr size_t kPages = 4096;
    constexpr int kOwnersUsed = 3;   // shared + two node arenas
    PageRunMap map;
    CHECK(map.init(kPages), "init failed");

    std::vector<int> model(kPages, kBusy);
    std::mt19937_64 rng(12345);
    // Pages are assigned to owners in fixed 256-page chunks, like node arenas.
    auto ownerOf = [](size_t page) { return static_cast<int>((page / 256) % kOwnersUsed); };

    long takes = 0, fails_ok = 0, merged_takes = 0;

    // take() with the model checks; returns the start or kNone.
    auto checkedTake = [&](size_t len, int owner, int cycle) -> size_t {
        const size_t got = map.takeAny(len, static_cast<uint32_t>(owner));
        ++takes;
        if (got == PageRunMap::kNone) {
            const size_t best = longestRun(model, owner);
            CHECK(best < len, "cycle %d: take(%zu, owner %d) failed but the model has a free "
                  "run of %zu pages (runs were not coalesced)", cycle, len, owner, best);
            ++fails_ok;
            return got;
        }
        for (size_t p = got; p < got + len; ++p) {
            if (p >= kPages || model[p] != owner) {
                CHECK(false, "cycle %d: take(%zu, owner %d) returned page %zu, which the model "
                      "has as %s", cycle, len, owner, p,
                      p >= kPages ? "out of range" : model[p] == kBusy ? "busy" : "another owner's");
                return PageRunMap::kNone;
            }
        }
        for (size_t p = got; p < got + len; ++p) model[p] = kBusy;
        return got;
    };

    // Start with every chunk free, one run per 256-page chunk.
    for (size_t c = 0; c < kPages; c += 256) {
        map.insert(c, 256, static_cast<uint32_t>(ownerOf(c)));
        for (size_t p = c; p < c + 256; ++p) model[p] = ownerOf(c);
    }

    for (int cycle = 0; cycle < 300 && failures == 0; ++cycle) {
        // 1. Allocate everything in small pieces (1..8 pages).
        std::vector<std::pair<size_t, size_t>> pieces;   // start, len
        for (int owner = 0; owner < kOwnersUsed; ++owner) {
            for (;;) {
                size_t len = 1 + rng() % 8;
                size_t got = checkedTake(len, owner, cycle);
                if (got == PageRunMap::kNone) { len = 1; got = checkedTake(1, owner, cycle); }
                if (got == PageRunMap::kNone) break;
                pieces.push_back({got, len});
            }
        }
        // 2. Free all pieces in random order: neighbours must merge back.
        std::shuffle(pieces.begin(), pieces.end(), rng);
        for (auto [st, len] : pieces) {
            const int owner = ownerOf(st);
            map.insert(st, len, static_cast<uint32_t>(owner));
            for (size_t p = st; p < st + len; ++p) model[p] = owner;
        }
        // 3. Request large runs, up to a whole chunk; then free them again.
        std::vector<std::pair<size_t, size_t>> big;
        for (int k = 0; k < 20; ++k) {
            const int owner = static_cast<int>(rng() % kOwnersUsed);
            const size_t len = 100 + rng() % 157;             // 100..256 pages
            const size_t got = checkedTake(len, owner, cycle);
            if (got != PageRunMap::kNone) { big.push_back({got, len}); ++merged_takes; }
        }
        for (auto [st, len] : big) {
            map.insert(st, len, static_cast<uint32_t>(ownerOf(st)));
            for (size_t p = st; p < st + len; ++p) model[p] = ownerOf(st);
        }
    }
    printf("  %ld takes, %ld correctly refused, %ld large (100..256-page) takes served from merged runs\n",
           takes, fails_ok, merged_takes);
    CHECK(merged_takes > 1000, "only %ld takes needed merged runs: the test did not exercise "
          "coalescing", merged_takes);

    // The walk's bound. A size bin above 64 pages holds runs of up to 1.25x its
    // floor, so a request can meet shorter runs in its own bin; it looks at
    // kTakeScan of them and moves on. 10000 runs of 65 pages and a request for
    // 66 used to walk all 10000 under the lock, on every miss, on every sibling
    // sub-arena (first-fit in one bin per power of two).
    {
        constexpr size_t kRuns = 10000;
        constexpr uint32_t kOwner = 1;
        PageRunMap m;
        CHECK(m.init((kRuns + 2) * 70), "init failed");
        for (size_t i = 0; i < kRuns; ++i) m.insert(i * 70, 65, kOwner);
        uint64_t before = m.scanSteps(kOwner);
        CHECK(m.take(66, kOwner) == PageRunMap::kNone, "take(66) found a run among runs of 65");
        uint64_t steps = m.scanSteps(kOwner) - before;
        printf("  a miss among %zu shorter runs walked %llu of them\n", kRuns, (unsigned long long)steps);
        CHECK(steps <= PageRunMap::kTakeScan, "a miss walked %llu runs", (unsigned long long)steps);

        // A longer run in the same power of two is one step away, however many
        // shorter runs were freed after it.
        PageRunMap m2;
        CHECK(m2.init((kRuns + 2) * 150), "init failed");
        m2.insert(0, 100, kOwner);
        for (size_t i = 1; i <= kRuns; ++i) m2.insert(i * 150, 65, kOwner);
        before = m2.scanSteps(kOwner);
        CHECK(m2.take(66, kOwner) == 0, "take(66) did not find the longer run");
        steps = m2.scanSteps(kOwner) - before;
        printf("  a hit behind %zu shorter runs walked %llu of them\n", kRuns, (unsigned long long)steps);
        CHECK(steps <= PageRunMap::kTakeScan, "a hit walked %llu runs", (unsigned long long)steps);
        // The remainder of 34 pages stays free and whole.
        CHECK(m2.take(34, kOwner) == 66, "the remainder is gone");

        // takeAny finds what the bound skips: the region's last resort must
        // not fail with the pages free. The one fit sits behind every short run.
        PageRunMap m3;
        CHECK(m3.init(70 * 1002), "init failed");
        m3.insert(0, 66, kOwner);
        for (size_t i = 1; i <= 1000; ++i) m3.insert(i * 70, 65, kOwner);
        CHECK(m3.take(66, kOwner) == PageRunMap::kNone, "the bound did not bound");
        CHECK(m3.takeAny(66, kOwner) == 0, "takeAny missed a run that fits");
    }

    // Every pair of run length and request around every bin edge: with one run
    // in the map, take() succeeds exactly when the run is long enough, and the
    // remainder is one run again. Catches a bin whose floor is wrong in either
    // direction: a "bin above is sure to fit" that is not, or a run filed too low.
    {
        constexpr uint32_t kOwner = 2;
        constexpr size_t kSpan = 1u << 20;
        PageRunMap m;
        CHECK(m.init(kSpan), "init failed");
        std::vector<size_t> lens;
        for (size_t v = 1; v < kSpan / 2; v = v * 5 / 4 + 1)
            for (long d = -2; d <= 2; ++d)
                if (static_cast<long>(v) + d >= 1) lens.push_back(static_cast<size_t>(static_cast<long>(v) + d));
        std::sort(lens.begin(), lens.end());
        lens.erase(std::unique(lens.begin(), lens.end()), lens.end());
        long pairs = 0;
        for (size_t have : lens) {
            for (size_t want : lens) {
                if (want > have * 2 || have > want * 2) continue;
                m.insert(0, have, kOwner);
                const size_t got = m.take(want, kOwner);
                CHECK((got != PageRunMap::kNone) == (want <= have),
                      "run %zu, request %zu: got %zu", have, want, got);
                if (got != PageRunMap::kNone) {
                    CHECK(got == 0, "run %zu, request %zu at %zu", have, want, got);
                    CHECK(m.freePages(kOwner) == have - want, "run %zu, request %zu: %zu left", have,
                          want, m.freePages(kOwner));
                    if (have > want) CHECK(m.take(have - want, kOwner) == want, "remainder of %zu, %zu", have, want);
                } else {
                    CHECK(m.take(have, kOwner) == 0, "run %zu lost", have);
                }
                CHECK(m.freePages(kOwner) == 0, "run %zu, request %zu left pages behind", have, want);
                ++pairs;
                if (failures > 10) break;
            }
        }
        printf("  %ld run/request pairs around the bin edges\n", pairs);
        // The longest-run diagnostic is a lower bound, and within a bin of it.
        for (size_t have : lens) {
            m.insert(0, have, kOwner);
            const size_t floor = m.longestRunAtLeast(kOwner);
            CHECK(floor <= have && floor * 5 / 4 + 1 >= have, "run %zu reported as at least %zu", have, floor);
            m.take(have, kOwner);
        }
    }

    // Dirty pages against a model of one state per page. Dirty and clean
    // neighbours merge, so a run holds both; a take must report exactly the
    // dirty pages it took and leave the rest of the run counted right.
    {
        constexpr uint32_t kOwner = 3;
        constexpr size_t kN = 4096;
        enum : char { kUsed, kClean, kDirty };
        std::vector<char> st(kN, kUsed);
        PageRunMap m;
        CHECK(m.init(kN), "init failed");
        std::mt19937_64 r(77);
        auto modelDirty = [&](size_t a, size_t n) {
            size_t c = 0;
            for (size_t i = a; i < a + n; ++i) c += st[i] == kDirty;
            return c;
        };
        auto checkRuns = [&](const char* when) {
            size_t runs = 0, pages = 0, dirty = 0;
            m.forEachRunUnlocked(kOwner, [&](size_t start, size_t len, uint32_t, size_t d) {
                ++runs; pages += len; dirty += d;
                CHECK(d == modelDirty(start, len), "%s: run %zu+%zu counts %zu dirty, model %zu", when,
                      start, len, d, modelDirty(start, len));
                CHECK(m.countDirtyUnlocked(start, len) == d, "%s: run %zu+%zu counts %zu, bits %zu", when,
                      start, len, d, m.countDirtyUnlocked(start, len));
                // Maximal: nothing free on either side, whatever its state.
                CHECK(start == 0 || st[start - 1] == kUsed, "%s: free page before run %zu", when, start);
                CHECK(start + len == kN || st[start + len] == kUsed, "%s: free page after run %zu+%zu",
                      when, start, len);
            });
            size_t mf = 0, md = 0;
            for (char c : st) { mf += c != kUsed; md += c == kDirty; }
            CHECK(pages == mf && dirty == md, "%s: %zu free %zu dirty, model %zu %zu", when, pages, dirty, mf, md);
            CHECK(m.totalDirtyBits(kN) == md, "%s: %zu bits, model %zu dirty", when, m.totalDirtyBits(kN), md);
            return runs;
        };
        size_t mixed = 0;
        for (int op = 0; op < 60000 && failures < 10; ++op) {
            const unsigned what = static_cast<unsigned>(r() % 8);
            const size_t len = 1 + r() % 200;
            if (what < 3) {          // free a used range, dirty or clean
                const size_t a = r() % (kN - len);
                bool used = true;
                for (size_t i = a; i < a + len; ++i) used = used && st[i] == kUsed;
                if (!used) continue;
                const bool d = r() & 1;
                if (d) m.markDirtyUnlocked(a, len);
                m.insertUnlocked(a, len, kOwner, 0, 0, d ? len : 0);
                for (size_t i = a; i < a + len; ++i) st[i] = d ? kDirty : kClean;
            } else if (what < 6) {   // take, plain or aligned
                size_t got_dirty = 99999;
                const size_t align = what == 5 ? size_t{1} << (r() % 6) : 1;
                const size_t got = m.takeAlignedUnlocked(len, kOwner, align, 0, PageRunMap::kScanAll, &got_dirty);
                if (got == PageRunMap::kNone) continue;
                CHECK(got % align == 0, "take aligned %zu at %zu", align, got);
                const size_t want = modelDirty(got, len);
                CHECK(got_dirty == want, "take %zu at %zu: says %zu dirty, model %zu", len, got, got_dirty, want);
                mixed += want != 0 && want != len;
                for (size_t i = got; i < got + len; ++i) {
                    CHECK(st[i] != kUsed, "page %zu taken twice", i);
                    st[i] = kUsed;
                }
            } else if (what == 6) {  // realloc's grow: the front of the run at a page
                const size_t a = r() % (kN - len);
                size_t d = 99999;
                bool free_run = st[a] != kUsed && (a == 0 || st[a - 1] == kUsed);
                for (size_t i = a; free_run && i < a + len; ++i) free_run = st[i] != kUsed;
                const bool got = m.takeFrontUnlocked(a, len, kOwner, &d);
                CHECK(got == free_run, "takeFront %zu+%zu: %d, model %d", a, len, got, free_run);
                if (!got) continue;
                CHECK(d == modelDirty(a, len), "takeFront %zu+%zu: says %zu dirty, model %zu", a, len, d, modelDirty(a, len));
                for (size_t i = a; i < a + len; ++i) st[i] = kUsed;
            } else {                 // a purge's cut: the run's end holds `want` dirty pages
                size_t pick = PageRunMap::kNone, plen = 0, pdirty = 0;
                m.forEachRunUnlocked(kOwner, [&](size_t start, size_t l, uint32_t, size_t d) {
                    if (d >= 2 && pick == PageRunMap::kNone) { pick = start; plen = l; pdirty = d; }
                });
                if (pick == PageRunMap::kNone) continue;
                const size_t want = 1 + r() % (pdirty - 1);
                const size_t cut = m.cutForLastDirtyUnlocked(pick, plen, want);
                CHECK(cut >= pick && cut < pick + plen, "cut %zu outside %zu+%zu", cut, pick, plen);
                CHECK(modelDirty(cut, pick + plen - cut) == want, "cut of %zu+%zu at %zu holds %zu dirty, wanted %zu",
                      pick, plen, cut, modelDirty(cut, pick + plen - cut), want);
                CHECK(st[cut] == kDirty, "cut %zu is not at a dirty page", cut);
                CHECK(m.removeRunUnlocked(pick, plen, kOwner), "run %zu+%zu not removable", pick, plen);
                if (cut > pick) m.insertUnlocked(pick, cut - pick, kOwner, 0, 0, pdirty - want);
                m.clearDirtyUnlocked(cut, pick + plen - cut);
                m.insertUnlocked(cut, pick + plen - cut, kOwner, 0, 0, 0);
                for (size_t i = cut; i < pick + plen; ++i) st[i] = kClean;
            }
            if (op % 64 == 0) checkRuns("random");
        }
        checkRuns("end");
        printf("  dirty bits: %zu takes straddled dirty and clean pages\n", mixed);
        CHECK(mixed > 200, "only %zu takes held both kinds of page: the merge was not exercised", mixed);
    }

    if (failures) { printf("test_page_run_map: %d FAILURES\n", failures); return 1; }
    printf("test_page_run_map: all checks passed\n");
    return 0;
}
