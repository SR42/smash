// shadow_free_slots_test.cpp - in full mode with deferred reclaim (the
// default), a compressed page whose freed slots still hold the freed objects'
// bytes must be reclaimed, and a write to a live object on a page awaiting
// reclaim must survive.
//
// The compressor zeroes free slots in its snapshot so they compress well
// (zeroFreeSlots), but the live page keeps the old bytes. Phase B checks that
// a COMPRESSED_SHADOW page is unchanged before reclaiming it, and compared it
// byte for byte against the zeroed blob: every page with a dirty free slot
// failed, went back to ACTIVE with its cold count reset, and was compressed
// again, forever -- 1280 of 1370 compressions discarded and 0 of 256 pages
// reclaimed in this shape; with the slots zeroed before free(), 256 of 256.
// The check now skips slots that are free (shadowMatchesBlob).
//
// 4096 objects of 256 bytes from one call site; the first object of each page
// stays live, the other 15 are freed without being cleared. While pages sit
// in COMPRESSED_SHADOW, the live object on every fourth one is rewritten.
// Checks, after 45 s at most:
//   - >= 90% of the untouched pages are COMPRESSED (reclaimed), none ACTIVE;
//   - every rewritten object reads back its new bytes, 3 s after its page is
//     reclaimed again and at the end, and every other its old ones: a check
//     that skipped live slots would reclaim a rewritten page with the old
//     blob and restore the old bytes;
//   - at least 32 pages were rewritten while in COMPRESSED_SHADOW.
#include "placement_test_util.h"

#include <set>
#include <vector>

using namespace placement;

namespace {

constexpr size_t kObj = 256, kCount = 4096;

unsigned char oldByte(size_t k, size_t i) { return i < 64 ? static_cast<unsigned char>(k * 7 + i + 1) : 0; }
unsigned char newByte(size_t k, size_t i) { return i < 64 ? static_cast<unsigned char>(k * 11 + i + 3) : 0x5e; }

}  // namespace

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    const PageInfoFn page_info = pageInfoFn();
    if (!page_info) {
        std::fprintf(stderr, "FAIL: not running under libsmash\n");
        return 1;
    }
    std::vector<unsigned char*> objs(kCount);
    for (size_t k = 0; k < kCount; ++k) {
        objs[k] = static_cast<unsigned char*>(malloc(kObj));
        for (size_t i = 0; i < kObj; ++i) objs[k][i] = oldByte(k, i);
    }
    std::vector<size_t> live;
    std::set<uintptr_t> seen;
    for (size_t k = 0; k < kCount; ++k) {
        const uintptr_t pg = pageOf(objs[k]);
        if (pageOf(objs[k] + kObj - 1) == pg && !seen.count(pg)) {
            seen.insert(pg);
            live.push_back(k);
        } else {
            free(objs[k]);  // not cleared: the freed slot keeps these bytes
        }
    }
    // Every fourth page's live object is rewritten while the page is a shadow,
    // then read back 3 s after the page is next seen reclaimed (COMPRESSED):
    // by then its physical page has been dropped (deferred madvise, 2 ticks),
    // so the read restores the blob. Read only at the end, a lost rewrite went
    // unseen in 7 of 8 runs of a check that accepted any page.
    std::vector<int> role(live.size());  // 0 untouched, 1 to rewrite, 2 rewritten, 3 read back
    std::vector<double> reclaimed_at(live.size(), -1);
    for (size_t j = 0; j < live.size(); j += 4) role[j] = 1;

    size_t rewritten = 0, read_back = 0, bad_new = 0, compressed = 0, active = 0, untouched = 0;
    const double t0 = nowSec();
    while (nowSec() - t0 < 45.0) {
        compressed = active = untouched = 0;
        bool pending = false;
        for (size_t j = 0; j < live.size(); ++j) {
            int st = -1;
            if (page_info(objs[live[j]], &st, nullptr, nullptr) != 0) continue;
            if (role[j] == 1) {
                if (st == kShadow) {
                    for (size_t i = 0; i < kObj; ++i) objs[live[j]][i] = newByte(live[j], i);
                    role[j] = 2;
                    ++rewritten;
                }
                pending = true;
            } else if (role[j] == 2) {
                pending = true;
                if (st == kCompressed && reclaimed_at[j] < 0) reclaimed_at[j] = nowSec();
                if (reclaimed_at[j] >= 0 && nowSec() - reclaimed_at[j] > 3.0) {
                    for (size_t i = 0; i < kObj; ++i) bad_new += objs[live[j]][i] != newByte(live[j], i);
                    role[j] = 3;
                    ++read_back;
                }
            } else if (role[j] == 0) {
                ++untouched;
                compressed += st == kCompressed;
                active += st == kActive || st == kMonitoring;
            }
        }
        if (!pending && compressed == untouched && nowSec() - t0 > 8.0) break;
        usleep(20 * 1000);
    }
    const double secs = nowSec() - t0;

    size_t bad_old = 0;
    for (size_t j = 0; j < live.size(); ++j) {
        const size_t k = live[j];
        for (size_t i = 0; i < kObj; ++i) {
            if (role[j] >= 2) bad_new += objs[k][i] != newByte(k, i);
            else bad_old += objs[k][i] != oldByte(k, i);
        }
    }
    std::printf("%zu pages; untouched: %zu of %zu reclaimed (COMPRESSED), %zu active, after %.1f s; %zu rewritten "
                "while shadow, %zu read back 3 s after reclaim; bad bytes: %zu rewritten, %zu untouched\n",
                live.size(), compressed, untouched, active, secs, rewritten, read_back, bad_new, bad_old);
    int failures = 0;
    auto fail = [&](const std::string& why) {
        std::printf("FAIL: %s\n", why.c_str());
        ++failures;
    };
    if (live.size() < 200) fail("only " + std::to_string(live.size()) + " pages");
    if (compressed * 10 < untouched * 9)
        fail("only " + std::to_string(compressed) + " of " + std::to_string(untouched) +
             " pages with dirty free slots reclaimed");
    if (active) fail(std::to_string(active) + " untouched pages thrown back to ACTIVE");
    if (rewritten < 32) fail("only " + std::to_string(rewritten) + " pages rewritten while shadow");
    if (read_back < rewritten) fail("only " + std::to_string(read_back) + " rewritten pages reclaimed again and read back");
    if (bad_new) fail(std::to_string(bad_new) + " bytes of rewritten objects lost (the rewrite was reclaimed)");
    if (bad_old) fail(std::to_string(bad_old) + " bytes of untouched objects read back wrong");
    for (size_t k : live) free(objs[k]);
    if (failures) return 1;
    std::printf("shadow_free_slots_test: all checks passed\n");
    return 0;
}
