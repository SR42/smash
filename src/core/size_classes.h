// smash/src/core/size_classes.h - Size class table and fast mapping
//
// ~36 size classes with sub-linear spacing:
//   step 16:   16 .. 128     (8 classes)
//   step 32:   160 .. 256    (4 classes)
//   step 64:   320 .. 512    (4 classes)
//   step 128:  640 .. 1024   (4 classes)
//   step 256:  1280 .. 2048  (4 classes)
//   step 512:  2560 .. 4096  (4 classes)
//   step 1024: 5120 .. 8192  (4 classes)
//   step 2048: 10240 .. 16384 (4 classes)
#pragma once

#include "smash/config.h"
#include "../util/bitops.h"
#include <cstddef>
#include <cstdint>

namespace smash {

struct SizeClassInfo {
    uint32_t size;       // allocation size for this class
    uint32_t pages;      // pages per span
    uint32_t objects;    // objects per span
};

namespace detail {

constexpr uint32_t kClassSizes[kNumClasses] = {
    16,    32,    48,    64,    80,    96,    112,   128,    // step 16
    160,   192,   224,   256,                                // step 32
    320,   384,   448,   512,                                // step 64
    640,   768,   896,   1024,                               // step 128
    1280,  1536,  1792,  2048,                               // step 256
    2560,  3072,  3584,  4096,                               // step 512
    5120,  6144,  7168,  8192,                               // step 1024
    10240, 12288, 14336, 16384,                              // step 2048
};

constexpr SizeClassInfo computeClassInfo(uint32_t size) {
    uint32_t target_bytes = kTargetObjectsPerSpan * size;
    uint32_t pages = (target_bytes + kPageSize - 1) / kPageSize;
    if (pages < 1) pages = 1;
    if (pages > static_cast<uint32_t>(kMaxSpanPages)) pages = kMaxSpanPages;
    uint32_t objects = (pages * kPageSize) / size;
    return {size, pages, objects};
}

constexpr auto buildClassTable() {
    struct Table { SizeClassInfo entries[kNumClasses]; };
    Table t{};
    for (int i = 0; i < kNumClasses; ++i)
        t.entries[i] = computeClassInfo(kClassSizes[i]);
    return t;
}

constexpr auto kClassTableStorage = buildClassTable();

// Lookup table for sizes 0..1024, indexed by (size + 15) >> 4
// Maps to size class index.
constexpr auto buildSmallLookup() {
    struct Table { uint8_t entries[65]; };
    Table t{};
    // For each 16-byte-aligned bucket, find the smallest class that fits
    for (int idx = 0; idx <= 64; ++idx) {
        uint32_t maxSize = idx * 16;  // max size this bucket covers
        if (maxSize == 0) maxSize = 1;
        // Find first class with size >= maxSize
        for (int c = 0; c < kNumClasses; ++c) {
            if (kClassSizes[c] >= maxSize) {
                t.entries[idx] = static_cast<uint8_t>(c);
                break;
            }
        }
    }
    return t;
}

constexpr auto kSmallLookupStorage = buildSmallLookup();

} // namespace detail

inline constexpr const SizeClassInfo* kSizeClasses = detail::kClassTableStorage.entries;
inline constexpr const uint8_t* kSmallLookup = detail::kSmallLookupStorage.entries;

// Fast size → class index. Returns kNumClasses if size > kMaxSmallSize.
inline uint8_t sizeToClass(size_t size) {
    if (size == 0) size = 1;
    if (size <= 1024) [[likely]] {
        return kSmallLookup[(size + 15) >> 4];
    }
    if (size <= kMaxSmallSize) {
        // Log2-based computation for sizes 1025..16384
        int bits = 63 - __builtin_clzll(size - 1);   // floor(log2(size-1))
        int group = bits - 10;                         // 0..3
        size_t groupBase = 1ULL << bits;               // 1024, 2048, 4096, 8192
        // step = groupBase / 4 (256, 512, 1024, 2048): a shift, not a div.
        int sub = static_cast<int>((size - groupBase - 1) >> (bits - 2));
        return static_cast<uint8_t>(20 + group * 4 + sub);
    }
    return kNumClasses;
}

// Class size for a given class index
inline uint32_t classSize(uint8_t sc) {
    return kSizeClasses[sc].size;
}

} // namespace smash
