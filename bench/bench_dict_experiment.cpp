// bench_dict_experiment.cpp - Dictionary compression experiments
//
// Tests when ZSTD dictionaries help vs hurt, using realistic data patterns.
// Directly exercises CompressEngine (no interposition needed).
//
// Experiments:
//   1. Try-both: dict vs plain ZSTD on every page, track wins/losses
//   2. Level sweep: dict benefit at level 1, 3, 9
//   3. Grouping: train dict from homogeneous vs heterogeneous samples
//   4. Dict size sweep
//   5. Sample count sweep

#include "compress/compress_engine.h"
#include "core/bootstrap_alloc.h"
#include "smash/config.h"
#define ZSTD_STATIC_LINKING_ONLY
#include <zstd.h>
#include <zdict.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include <algorithm>
#include <random>

using namespace smash;

// ── Data generators ─────────────────────────────────────────────────────────
// Each simulates a different allocation pattern at the page level.

// Type A: JSON-like - small objects with string keys and numeric values
// Simulates cJSON nodes: {tag, valueint, valuedouble, *string, *next, *child}
static void fillJsonLike(void* page, int variant, std::mt19937& rng) {
    auto* p = static_cast<uint8_t*>(page);
    __builtin_memset(p, 0, kPageSize);
    constexpr size_t kObjSize = 64;  // cJSON node ~56 bytes, 64 with padding
    for (size_t off = 0; off + kObjSize <= kPageSize; off += kObjSize) {
        // Tag byte (object type)
        p[off] = static_cast<uint8_t>(variant & 0x7);
        // Pointer-like fields (8 bytes each, look like heap addresses)
        uint64_t fake_ptr = 0x100000000ULL + rng() % 0x10000000ULL;
        __builtin_memcpy(p + off + 8, &fake_ptr, 8);
        fake_ptr = 0x100000000ULL + rng() % 0x10000000ULL;
        __builtin_memcpy(p + off + 16, &fake_ptr, 8);
        // String key: short ASCII with common prefixes
        const char* prefixes[] = {"id", "name", "val", "type", "data", "count", "ts", "idx"};
        const char* pfx = prefixes[variant % 8];
        size_t pfx_len = strlen(pfx);
        __builtin_memcpy(p + off + 24, pfx, pfx_len);
        p[off + 24 + pfx_len] = '_';
        // Numeric value
        double val = static_cast<double>(rng() % 10000) / 100.0;
        __builtin_memcpy(p + off + 40, &val, 8);
    }
}

// Type B: KV-store-like - fixed-size key-value pairs with Zipfian-like patterns
static void fillKVLike(void* page, int variant, std::mt19937& rng) {
    auto* p = static_cast<uint8_t*>(page);
    __builtin_memset(p, 0, kPageSize);
    constexpr size_t kObjSize = 128;  // key(32) + value(96)
    for (size_t off = 0; off + kObjSize <= kPageSize; off += kObjSize) {
        // Key: "user:" prefix + numeric ID
        snprintf(reinterpret_cast<char*>(p + off), 32, "user:%08x", static_cast<unsigned>(rng()));
        // Value: JSON-ish payload with repeated structure
        int age = static_cast<int>(rng() % 100);
        snprintf(reinterpret_cast<char*>(p + off + 32), 96,
                 R"({"age":%d,"city":"city_%d","score":%.1f})",
                 age, variant % 50, static_cast<double>(rng() % 1000) / 10.0);
    }
}

// Type C: SQLite-like - B-tree pages with fixed header + variable cells
static void fillSQLiteLike(void* page, int variant, std::mt19937& rng) {
    auto* p = static_cast<uint8_t*>(page);
    __builtin_memset(p, 0, kPageSize);
    // Page header (8 bytes)
    p[0] = 0x0D;  // leaf table b-tree page
    uint16_t ncells = static_cast<uint16_t>(40 + variant % 20);
    __builtin_memcpy(p + 3, &ncells, 2);
    // Cell pointer array
    size_t ptr_end = 8 + ncells * 2;
    // Cell content area
    size_t cell_off = ptr_end;
    for (int i = 0; i < ncells && cell_off + 200 < kPageSize; ++i) {
        uint16_t off16 = static_cast<uint16_t>(cell_off);
        __builtin_memcpy(p + 8 + i * 2, &off16, 2);
        // Varint payload length
        p[cell_off++] = 0x80 | static_cast<uint8_t>((150 + rng() % 50) >> 7);
        p[cell_off++] = static_cast<uint8_t>((150 + rng() % 50) & 0x7F);
        // Row data: integer + text + blob
        int64_t rowid = static_cast<int64_t>(i + variant * 100);
        __builtin_memcpy(p + cell_off, &rowid, 8);
        cell_off += 8;
        // Text column
        int text_len = 20 + static_cast<int>(rng() % 80);
        for (int j = 0; j < text_len && cell_off < kPageSize; ++j)
            p[cell_off++] = static_cast<uint8_t>('a' + rng() % 26);
        // Padding
        cell_off = (cell_off + 7) & ~7ULL;
    }
}

// Type D: Zeroed-out page (simulates freed slots)
static void fillMostlyZero(void* page, int variant, std::mt19937& rng) {
    auto* p = static_cast<uint8_t*>(page);
    __builtin_memset(p, 0, kPageSize);
    // A few live objects scattered
    int live = 2 + static_cast<int>(rng() % 5);
    for (int i = 0; i < live; ++i) {
        size_t off = (rng() % (kPageSize / 64)) * 64;
        p[off] = 0xAA;
        p[off + 1] = static_cast<uint8_t>(variant);
        uint64_t val = rng();
        __builtin_memcpy(p + off + 8, &val, 8);
    }
}

using FillFunc = void(*)(void*, int, std::mt19937&);

struct DataType {
    const char* name;
    FillFunc fill;
};

static DataType g_data_types[] = {
    {"json",    fillJsonLike},
    {"kv",      fillKVLike},
    {"sqlite",  fillSQLiteLike},
    {"zeroed",  fillMostlyZero},
};
static constexpr int kNumDataTypes = sizeof(g_data_types) / sizeof(g_data_types[0]);

// ── Helpers ─────────────────────────────────────────────────────────────────

static void* allocPage() {
    return BootstrapAlloc::instance().allocate(kPageSize, kPageSize);
}

static void* allocBuf(size_t sz) {
    return BootstrapAlloc::instance().allocate(sz, 16);
}

// Train a dictionary from sample pages
static bool trainDict(CompressEngine& engine, uint8_t sc,
                      void** pages, int num_pages) {
    size_t total = static_cast<size_t>(num_pages) * kPageSize;
    auto* sample_data = static_cast<char*>(allocBuf(total));
    auto* sample_sizes = static_cast<size_t*>(allocBuf(num_pages * sizeof(size_t)));
    for (int i = 0; i < num_pages; ++i) {
        __builtin_memcpy(sample_data + static_cast<size_t>(i) * kPageSize,
                         pages[i], kPageSize);
        sample_sizes[i] = kPageSize;
    }
    return engine.trainDictionary(sc, sample_data, sample_sizes,
                                   static_cast<unsigned>(num_pages));
}

// ── Experiment 1: Try-Both ──────────────────────────────────────────────────
// For each data type, compress pages with both dict and plain ZSTD at level 9.
// Report win/loss/tie counts and average delta.

static void experiment1_tryBoth() {
    printf("\n========================================\n");
    printf("  Experiment 1: Try-Both (dict vs plain ZSTD at level 9)\n");
    printf("========================================\n\n");

    constexpr int kTrainPages = 16;
    constexpr int kTestPages = 100;
    size_t max_comp = CompressEngine::maxCompressedSizeAny(kPageSize);

    printf("%-10s %6s %6s %6s %12s %12s %10s\n",
           "DataType", "Wins", "Losses", "Ties", "AvgDelta", "TotalDelta", "DictBetter");

    for (int dt = 0; dt < kNumDataTypes; ++dt) {
        CompressEngine engine;
        engine.init();
        std::mt19937 rng(42 + dt);

        // Generate training pages (same data type, different variants)
        void* train_pages[kTrainPages];
        for (int i = 0; i < kTrainPages; ++i) {
            train_pages[i] = allocPage();
            g_data_types[dt].fill(train_pages[i], i, rng);
        }

        uint8_t sc = static_cast<uint8_t>(dt);  // use data type as size class
        bool trained = trainDict(engine, sc, train_pages, kTrainPages);
        if (!trained) {
            printf("%-10s  (dict training failed)\n", g_data_types[dt].name);
            continue;
        }

        // Generate test pages
        int wins = 0, losses = 0, ties = 0;
        int64_t total_delta = 0;
        auto* buf_dict = static_cast<char*>(allocBuf(max_comp));
        auto* buf_plain = static_cast<char*>(allocBuf(max_comp));

        for (int i = 0; i < kTestPages; ++i) {
            void* page = allocPage();
            g_data_types[dt].fill(page, kTrainPages + i, rng);  // unseen variants

            size_t dict_size = engine.compress(page, buf_dict, kPageSize, max_comp,
                                               CompressAlgo::ZSTD_DICT, sc);
            size_t plain_size = engine.compress(page, buf_plain, kPageSize, max_comp,
                                                CompressAlgo::ZSTD, sc);

            if (dict_size == 0 || plain_size == 0) continue;

            int64_t delta = static_cast<int64_t>(plain_size) - static_cast<int64_t>(dict_size);
            total_delta += delta;

            if (dict_size < plain_size) wins++;
            else if (dict_size > plain_size) losses++;
            else ties++;
        }

        int total = wins + losses + ties;
        double avg_delta = total > 0 ? static_cast<double>(total_delta) / total : 0;
        printf("%-10s %6d %6d %6d %10.1f B %10lld B %9.1f%%\n",
               g_data_types[dt].name, wins, losses, ties,
               avg_delta, static_cast<long long>(total_delta),
               total > 0 ? 100.0 * wins / total : 0.0);

        printf("METRIC dict_exp1_%s_wins %d\n", g_data_types[dt].name, wins);
        printf("METRIC dict_exp1_%s_losses %d\n", g_data_types[dt].name, losses);
        printf("METRIC dict_exp1_%s_avg_delta %.1f\n", g_data_types[dt].name, avg_delta);
    }
}

// ── Experiment 2: Level Sweep ───────────────────────────────────────────────
// Test dict vs no-dict at levels 1, 3, 9 to see if dicts help more at low levels.

static void experiment2_levelSweep() {
    printf("\n========================================\n");
    printf("  Experiment 2: Level Sweep (dict benefit by compression level)\n");
    printf("========================================\n\n");

    // We can't easily change the CDict level after training (it's baked in).
    // Instead, we'll train separate engines at different levels by modifying
    // the trainDictionary call... but the current API uses kZstdDeepLevel.
    // Workaround: compare dict (at its built level) vs plain at various levels.
    // This shows: does a dict at level 9 beat plain zstd at level 9? At level 3?

    constexpr int kTrainPages = 16;
    constexpr int kTestPages = 100;
    size_t max_comp = CompressEngine::maxCompressedSizeAny(kPageSize);
    int levels[] = {1, 3, 9, 15};

    printf("%-10s %6s %12s %12s %12s %12s\n",
           "DataType", "Level", "PlainSize", "DictSize", "Delta", "DictBetter");

    for (int dt = 0; dt < kNumDataTypes; ++dt) {
        CompressEngine engine;
        engine.init();
        std::mt19937 rng(42 + dt);

        void* train_pages[kTrainPages];
        for (int i = 0; i < kTrainPages; ++i) {
            train_pages[i] = allocPage();
            g_data_types[dt].fill(train_pages[i], i, rng);
        }

        uint8_t sc = static_cast<uint8_t>(dt);
        trainDict(engine, sc, train_pages, kTrainPages);

        // Generate test pages
        void* test_pages[kTestPages];
        for (int i = 0; i < kTestPages; ++i) {
            test_pages[i] = allocPage();
            g_data_types[dt].fill(test_pages[i], kTrainPages + i, rng);
        }

        auto* buf1 = static_cast<char*>(allocBuf(max_comp));
        auto* buf2 = static_cast<char*>(allocBuf(max_comp));

        for (int level : levels) {
            size_t total_plain = 0, total_dict = 0;
            int valid = 0;

            for (int i = 0; i < kTestPages; ++i) {
                // Plain ZSTD at this level
                size_t plain = ZSTD_compressCCtx(
                    engine.getZstdCCtx(), buf1, max_comp,
                    test_pages[i], kPageSize, level);
                if (ZSTD_isError(plain)) continue;

                // Dict (always at its trained level)
                size_t dict = engine.compress(test_pages[i], buf2, kPageSize, max_comp,
                                              CompressAlgo::ZSTD_DICT, sc);
                if (dict == 0) continue;

                total_plain += plain;
                total_dict += dict;
                valid++;
            }

            if (valid > 0) {
                double avg_plain = static_cast<double>(total_plain) / valid;
                double avg_dict = static_cast<double>(total_dict) / valid;
                double delta = avg_plain - avg_dict;
                printf("%-10s %6d %10.0f B %10.0f B %10.1f B %10.1f%%\n",
                       g_data_types[dt].name, level,
                       avg_plain, avg_dict, delta,
                       100.0 * delta / avg_plain);

                printf("METRIC dict_exp2_%s_L%d_plain %.0f\n",
                       g_data_types[dt].name, level, avg_plain);
                printf("METRIC dict_exp2_%s_L%d_dict %.0f\n",
                       g_data_types[dt].name, level, avg_dict);
            }
        }
        printf("\n");
    }
}

// ── Experiment 3: Grouping Strategy ─────────────────────────────────────────
// Train dict from: (a) same data type only, (b) mixed data types, (c) global
// Then test compression on each data type.

static void experiment3_grouping() {
    printf("\n========================================\n");
    printf("  Experiment 3: Grouping Strategy\n");
    printf("========================================\n");
    printf("  homogeneous = train from same data type (simulates per-arena dict)\n");
    printf("  mixed       = train from all data types (simulates per-size-class dict)\n");
    printf("  no-dict     = plain ZSTD baseline\n\n");

    constexpr int kTrainPages = 16;
    constexpr int kTestPages = 100;
    size_t max_comp = CompressEngine::maxCompressedSizeAny(kPageSize);

    // Generate all pages upfront
    std::mt19937 rng(42);
    void* train_homo[kNumDataTypes][kTrainPages];
    void* test_pages[kNumDataTypes][kTestPages];

    for (int dt = 0; dt < kNumDataTypes; ++dt) {
        for (int i = 0; i < kTrainPages; ++i) {
            train_homo[dt][i] = allocPage();
            g_data_types[dt].fill(train_homo[dt][i], i, rng);
        }
        for (int i = 0; i < kTestPages; ++i) {
            test_pages[dt][i] = allocPage();
            g_data_types[dt].fill(test_pages[dt][i], kTrainPages + i, rng);
        }
    }

    // Mixed training data: 4 pages from each data type
    void* train_mixed[kTrainPages];
    for (int i = 0; i < kTrainPages; ++i) {
        train_mixed[i] = train_homo[i % kNumDataTypes][i / kNumDataTypes];
    }

    printf("%-10s %15s %15s %15s %15s\n",
           "DataType", "NoDict", "Homogeneous", "Mixed", "HomoVsMixed");

    for (int dt = 0; dt < kNumDataTypes; ++dt) {
        auto* buf = static_cast<char*>(allocBuf(max_comp));

        // No-dict baseline
        CompressEngine engine_none;
        engine_none.init();
        size_t total_none = 0;
        for (int i = 0; i < kTestPages; ++i) {
            size_t sz = engine_none.compress(test_pages[dt][i], buf, kPageSize,
                                              max_comp, CompressAlgo::ZSTD, 0);
            total_none += sz;
        }

        // Homogeneous dict
        CompressEngine engine_homo;
        engine_homo.init();
        trainDict(engine_homo, 0, train_homo[dt], kTrainPages);
        size_t total_homo = 0;
        for (int i = 0; i < kTestPages; ++i) {
            size_t sz = engine_homo.compress(test_pages[dt][i], buf, kPageSize,
                                              max_comp, CompressAlgo::ZSTD_DICT, 0);
            total_homo += sz;
        }

        // Mixed dict
        CompressEngine engine_mixed;
        engine_mixed.init();
        trainDict(engine_mixed, 0, train_mixed, kTrainPages);
        size_t total_mixed = 0;
        for (int i = 0; i < kTestPages; ++i) {
            size_t sz = engine_mixed.compress(test_pages[dt][i], buf, kPageSize,
                                               max_comp, CompressAlgo::ZSTD_DICT, 0);
            total_mixed += sz;
        }

        double avg_none = static_cast<double>(total_none) / kTestPages;
        double avg_homo = static_cast<double>(total_homo) / kTestPages;
        double avg_mixed = static_cast<double>(total_mixed) / kTestPages;

        printf("%-10s %13.0f B %13.0f B %13.0f B %+13.0f B\n",
               g_data_types[dt].name, avg_none, avg_homo, avg_mixed,
               avg_homo - avg_mixed);

        printf("METRIC dict_exp3_%s_nodict %.0f\n", g_data_types[dt].name, avg_none);
        printf("METRIC dict_exp3_%s_homo %.0f\n", g_data_types[dt].name, avg_homo);
        printf("METRIC dict_exp3_%s_mixed %.0f\n", g_data_types[dt].name, avg_mixed);

        // Relative to no-dict
        printf("  → homo vs nodict: %+.1f%%, mixed vs nodict: %+.1f%%\n",
               100.0 * (avg_homo - avg_none) / avg_none,
               100.0 * (avg_mixed - avg_none) / avg_none);
    }
}

// ── Experiment 4 & 5: Dict Size and Sample Count Sweeps ─────────────────────

// These require modifying CompressEngine::trainDictionary's dict capacity
// and sample count. For now, we sweep sample count (easy — just vary how
// many pages we pass to trainDictionary) and report compressed sizes.

static void experiment5_sampleCount() {
    printf("\n========================================\n");
    printf("  Experiment 5: Sample Count Sweep\n");
    printf("========================================\n\n");

    constexpr int kTestPages = 100;
    int sample_counts[] = {4, 8, 16, 32, 64};
    size_t max_comp = CompressEngine::maxCompressedSizeAny(kPageSize);

    printf("%-10s", "DataType");
    for (int sc : sample_counts) printf(" %8d", sc);
    printf(" %10s\n", "NoDictAvg");

    for (int dt = 0; dt < kNumDataTypes; ++dt) {
        std::mt19937 rng(42 + dt);

        // Generate lots of training pages
        constexpr int kMaxSamples = 64;
        void* all_train[kMaxSamples];
        for (int i = 0; i < kMaxSamples; ++i) {
            all_train[i] = allocPage();
            g_data_types[dt].fill(all_train[i], i, rng);
        }

        void* test[kTestPages];
        for (int i = 0; i < kTestPages; ++i) {
            test[i] = allocPage();
            g_data_types[dt].fill(test[i], kMaxSamples + i, rng);
        }

        auto* buf = static_cast<char*>(allocBuf(max_comp));

        // No-dict baseline
        CompressEngine engine_base;
        engine_base.init();
        size_t total_base = 0;
        for (int i = 0; i < kTestPages; ++i) {
            size_t sz = engine_base.compress(test[i], buf, kPageSize,
                                              max_comp, CompressAlgo::ZSTD, 0);
            total_base += sz;
        }
        double avg_base = static_cast<double>(total_base) / kTestPages;

        printf("%-10s", g_data_types[dt].name);
        for (int nsamples : sample_counts) {
            CompressEngine engine;
            engine.init();
            trainDict(engine, 0, all_train, nsamples);

            size_t total = 0;
            for (int i = 0; i < kTestPages; ++i) {
                size_t sz = engine.compress(test[i], buf, kPageSize,
                                             max_comp, CompressAlgo::ZSTD_DICT, 0);
                total += sz;
            }
            double avg = static_cast<double>(total) / kTestPages;
            printf(" %7.0f B", avg);
        }
        printf(" %9.0f B\n", avg_base);

        // Emit METRIC lines separately (don't interleave with table)
        for (int nsamples : sample_counts) {
            CompressEngine engine2;
            engine2.init();
            trainDict(engine2, 0, all_train, nsamples);
            size_t total2 = 0;
            for (int i = 0; i < kTestPages; ++i) {
                size_t sz = engine2.compress(test[i], buf, kPageSize,
                                              max_comp, CompressAlgo::ZSTD_DICT, 0);
                total2 += sz;
            }
            printf("METRIC dict_exp5_%s_n%d %.0f\n",
                   g_data_types[dt].name, nsamples,
                   static_cast<double>(total2) / kTestPages);
        }
    }
}

// ── Experiment 6: CDict Memory Overhead ─────────────────────────────────────
// Estimate how much memory CDict/DDict objects consume per trained dictionary.

static void experiment6_memoryOverhead() {
    printf("\n========================================\n");
    printf("  Experiment 6: CDict/DDict Memory Overhead\n");
    printf("========================================\n\n");

    // ZSTD_sizeof_CDict gives the memory used by a CDict
    constexpr int kTrainPages = 16;
    std::mt19937 rng(42);

    void* train_pages[kTrainPages];
    for (int i = 0; i < kTrainPages; ++i) {
        train_pages[i] = allocPage();
        fillJsonLike(train_pages[i], i, rng);
    }

    size_t total = static_cast<size_t>(kTrainPages) * kPageSize;
    auto* sample_data = static_cast<char*>(allocBuf(total));
    auto* sample_sizes = static_cast<size_t*>(allocBuf(kTrainPages * sizeof(size_t)));
    for (int i = 0; i < kTrainPages; ++i) {
        __builtin_memcpy(sample_data + static_cast<size_t>(i) * kPageSize,
                         train_pages[i], kPageSize);
        sample_sizes[i] = kPageSize;
    }

    constexpr size_t kDictCapacity = 32 * 1024;
    void* dict_buf = allocBuf(kDictCapacity);
    size_t dict_size = ZDICT_trainFromBuffer(dict_buf, kDictCapacity,
                                              sample_data, sample_sizes, kTrainPages);
    if (ZDICT_isError(dict_size)) {
        printf("  Training failed: %s\n", ZDICT_getErrorName(dict_size));
        return;
    }

    printf("  Dict raw size: %zu bytes\n", dict_size);

    // Create CDict at different levels and measure
    int levels[] = {1, 3, 9, 15};
    for (int level : levels) {
        ZSTD_compressionParameters cparams = ZSTD_getCParams(level, kPageSize, dict_size);
        ZSTD_CDict* cdict = ZSTD_createCDict_advanced(
            dict_buf, dict_size, ZSTD_dlm_byCopy, ZSTD_dct_auto, cparams,
            ZSTD_defaultCMem);
        size_t cdict_size = ZSTD_sizeof_CDict(cdict);
        printf("  CDict at level %2d: %6zu KB\n", level, cdict_size / 1024);
        printf("METRIC dict_exp6_cdict_L%d_kb %zu\n", level, cdict_size / 1024);
        ZSTD_freeCDict(cdict);
    }

    ZSTD_DDict* ddict = ZSTD_createDDict(dict_buf, dict_size);
    size_t ddict_size = ZSTD_sizeof_DDict(ddict);
    printf("  DDict: %zu KB\n", ddict_size / 1024);
    printf("METRIC dict_exp6_ddict_kb %zu\n", ddict_size / 1024);
    ZSTD_freeDDict(ddict);

    // Sample buffer overhead per size class
    size_t sample_overhead = static_cast<size_t>(kDictTrainSamples) * kPageSize
                           + kDictTrainSamples * sizeof(size_t);
    printf("\n  Per-size-class overhead:\n");
    printf("    Sample buffer: %zu KB\n", sample_overhead / 1024);
    printf("    Dict buffer:   %zu KB\n", kDictCapacity / 1024);
    printf("  With %d size classes, %d potentially trained:\n", kNumClasses, kNumClasses);
    printf("    Sample buffers total: %zu MB\n",
           static_cast<size_t>(kNumClasses) * sample_overhead / (1024 * 1024));
}

// ── Main ────────────────────────────────────────────────────────────────────

int main() {
    printf("=== Dictionary Compression Experiments ===\n");
    printf("Page size: %zu, DictTrainSamples: %d, DeepLevel: %d\n",
           kPageSize, kDictTrainSamples, kZstdDeepLevel);

    experiment6_memoryOverhead();
    experiment1_tryBoth();
    experiment2_levelSweep();
    experiment3_grouping();
    experiment5_sampleCount();

    return 0;
}
