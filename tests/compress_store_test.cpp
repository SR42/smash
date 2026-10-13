// compress_store_test.cpp - how the compressed-blob store reuses and returns
// its 16 MiB regions (src/compress/compress_store.h).
//
// Added for a 128-core arm64 application whose store held 15.7 GiB for 1.6
// GiB of live blobs after hours of compress/decompress churn: a freed blob
// left a hole only a blob of the same size or smaller could reuse, and a
// region went back to the OS only once every blob in it was gone.
//
// Checks, all in shard 0 (page index 0):
//   - a region emptied by releases is refilled by bump, not handed one blob
//     and abandoned for a new region;
//   - markDraining(50) marks a full region under half live, never the region
//     the shard bumps into, and no new blob lands in a draining region's
//     holes;
//   - moving a draining region's blobs out (store a copy, release the old)
//     empties it: it is reset, decommitted, no longer draining;
//   - markDraining(0) marks nothing.
#include "compress/compress_store.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace smash;

namespace {

int failures = 0;
void check(bool ok, const char* what) {
    if (!ok) { std::printf("FAIL: %s\n", what); ++failures; }
}

constexpr size_t kRegion = 16u << 20;
constexpr size_t kBlob = 4000;           // 16-aligned: 4000 bytes per slot
uintptr_t regionOf(const void* p) { return reinterpret_cast<uintptr_t>(p) & ~(kRegion - 1); }

struct Blob { void* p; size_t alloc; unsigned char tag; };

Blob put(CompressStore& s, unsigned char tag) {
    unsigned char buf[kBlob];
    std::memset(buf, tag, sizeof(buf));
    Blob b{nullptr, 0, tag};
    b.p = s.store(buf, sizeof(buf), &b.alloc, 0);
    return b;
}

bool intact(const Blob& b) {
    const auto* c = static_cast<const unsigned char*>(b.p);
    for (size_t i = 0; i < kBlob; ++i)
        if (c[i] != b.tag) return false;
    return true;
}

size_t regions(const CompressStore& s, size_t* held = nullptr) {
    size_t live = 0, h = 0, n = 0;
    s.usage(&live, &h, &n);
    if (held) *held = h;
    return n;
}

// Stores blobs until one lands outside region `r`; returns those inside it.
std::vector<Blob> fillRegion(CompressStore& s, uintptr_t r, Blob* spill, unsigned char tag) {
    std::vector<Blob> in;
    for (;;) {
        Blob b = put(s, tag);
        if (!b.p) { check(false, "store failed"); return in; }
        if (regionOf(b.p) != r) { *spill = b; return in; }
        in.push_back(b);
    }
}

void testReuseOfEmptiedRegion() {
    CompressStore s;
    s.init();
    // R0 is shard 0's first region: fill it, then the next blob opens R1.
    Blob first = put(s, 1);
    const uintptr_t r0 = regionOf(first.p);
    Blob spill{};
    std::vector<Blob> a = fillRegion(s, r0, &spill, 1);
    a.push_back(first);
    const uintptr_t r1 = regionOf(spill.p);
    // Empty R0 completely: it is reset and decommitted.
    for (const Blob& b : a) s.release(b.p, b.alloc, 0);
    size_t held_before = 0;
    regions(s, &held_before);
    // Fill R1, then keep storing one more region's worth: R0 must take it.
    Blob next{};
    std::vector<Blob> b1 = fillRegion(s, r1, &next, 2);
    size_t in_r0 = regionOf(next.p) == r0;
    for (size_t i = 0; i < kRegion / kBlob; ++i) {
        Blob b = put(s, 3);
        in_r0 += regionOf(b.p) == r0;
    }
    char what[128];
    std::snprintf(what, sizeof(what), "an emptied region is refilled by bump (%zu of %zu blobs landed in it)",
                  in_r0, kRegion / kBlob);
    check(in_r0 > kRegion / kBlob / 2, what);
}

void testDraining() {
    CompressStore s;
    s.init();
    Blob first = put(s, 1);
    const uintptr_t r0 = regionOf(first.p);
    Blob spill{};
    std::vector<Blob> a = fillRegion(s, r0, &spill, 1);
    a.insert(a.begin(), first);
    // The shard now bumps into R1. Keep one blob in four in R0.
    std::vector<Blob> kept;
    for (size_t i = 0; i < a.size(); ++i) {
        if (i % 4 == 0) kept.push_back(a[i]);
        else s.release(a[i].p, a[i].alloc, 0);
    }
    check(s.markDraining(0) == 0, "markDraining(0) marks nothing");
    check(!CompressStore::draining(kept[0].p), "nothing draining before markDraining(50)");
    const size_t n = s.markDraining(50);
    check(n == 1, "markDraining(50): exactly the 25%-live region drains (not the bump region, not the empty ones)");
    check(CompressStore::draining(kept[0].p), "the 25%-live region is draining");
    check(!CompressStore::draining(spill.p), "the region the shard bumps into is never draining");
    check(CompressStore::ownerOf(kept[0].p) == &s, "ownerOf: the store that mapped the region");
    // New blobs fill R1 and then a fresh region, never R0's holes.
    size_t in_r0 = 0;
    for (size_t i = 0; i < 2 * kRegion / kBlob; ++i) {
        Blob b = put(s, 4);
        in_r0 += regionOf(b.p) == r0;
    }
    check(in_r0 == 0, "no new blob lands in a draining region");
    // Move R0's blobs out the way the compressor does: copy, then release.
    size_t held_before = 0;
    regions(s, &held_before);
    std::vector<Blob> moved;
    for (const Blob& b : kept) {
        Blob m{nullptr, 0, b.tag};
        m.p = s.store(b.p, kBlob, &m.alloc, 0);
        check(m.p && regionOf(m.p) != r0, "a moved blob leaves the draining region");
        s.release(b.p, b.alloc, 0);
        moved.push_back(m);
    }
    size_t held_after = 0;
    regions(s, &held_after);
    check(!CompressStore::draining(reinterpret_cast<void*>(r0 + 4096)), "an emptied region is no longer draining");
    check(held_before - held_after >= kRegion / 2, "emptying the draining region gives its bytes back");
    bool ok = true;
    for (const Blob& m : moved) ok = ok && intact(m);
    check(ok, "moved blobs intact");
    check(s.markDraining(50) == 0, "nothing left to drain");
}

}  // namespace

int main() {
    testReuseOfEmptiedRegion();
    testDraining();
    if (failures) return 1;
    std::printf("compress_store_test: all checks passed\n");
    return 0;
}
