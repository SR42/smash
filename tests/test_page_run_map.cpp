// test_page_run_map.cpp - PageRunMap against a trivially correct model.
//
// The model is one owner-or-busy entry per page. Random frees (insert) and
// allocations (take) across several owners, checking after every operation:
//   - take() returns only pages that are free and belong to that owner;
//   - take() fails ONLY when the model has no contiguous free run of that
//     length for that owner. This is the coalescing check: without merging,
//     adjacent freed ranges would stay separate and take() would fail while
//     the model can satisfy the request.
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
        const size_t got = map.take(len, static_cast<uint32_t>(owner));
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
    if (failures) { printf("test_page_run_map: %d FAILURES\n", failures); return 1; }
    printf("test_page_run_map: all checks passed\n");
    return 0;
}
