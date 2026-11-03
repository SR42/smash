// bench_compression.cpp - Compression ratio comparison: LZ4 vs zstd vs zstd+dict
#include "compress/compress_engine.h"
#include "core/bootstrap_alloc.h"
#include "smash/config.h"

#include <cstdio>
#include <cstring>
#include <chrono>

using namespace smash;

static void fillStructuredPage(void* page, uint8_t pattern, size_t obj_size) {
    auto* bytes = static_cast<uint8_t*>(page);
    for (size_t off = 0; off + obj_size <= kPageSize; off += obj_size) {
        bytes[off + 0] = 0xAA;
        bytes[off + 1] = pattern;
        bytes[off + 2] = static_cast<uint8_t>(obj_size & 0xFF);
        bytes[off + 3] = static_cast<uint8_t>(obj_size >> 8);
        for (size_t i = 4; i < 16 && i < obj_size; ++i)
            bytes[off + i] = 0x00;
        for (size_t i = 16; i < obj_size; ++i)
            bytes[off + i] = static_cast<uint8_t>((off + i + pattern) & 0xFF);
    }
}

struct BenchResult {
    size_t compressed_size;
    double compress_us;
    double decompress_us;
};

static BenchResult benchAlgo(CompressEngine& engine, const void* page,
                              CompressAlgo algo, uint8_t size_class, int iters) {
    size_t max_comp = CompressEngine::maxCompressedSizeAny(kPageSize);
    auto* comp_buf = static_cast<char*>(
        BootstrapAlloc::instance().allocate(max_comp, 16));
    auto* decomp_buf = static_cast<char*>(
        BootstrapAlloc::instance().allocate(kPageSize, kPageSize));

    // Warmup
    size_t comp_size = engine.compress(page, comp_buf, kPageSize, max_comp, algo, size_class);

    // Compress benchmark
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iters; ++i) {
        engine.compress(page, comp_buf, kPageSize, max_comp, algo, size_class);
    }
    auto mid = std::chrono::high_resolution_clock::now();

    // Decompress benchmark
    for (int i = 0; i < iters; ++i) {
        engine.decompress(comp_buf, decomp_buf, comp_size, kPageSize, algo, size_class);
    }
    auto end = std::chrono::high_resolution_clock::now();

    return {
        comp_size,
        std::chrono::duration<double, std::micro>(mid - start).count() / iters,
        std::chrono::duration<double, std::micro>(end - mid).count() / iters,
    };
}

static const char* algoName(CompressAlgo algo) {
    switch (algo) {
    case CompressAlgo::LZ4: return "LZ4";
    case CompressAlgo::ZSTD: return "zstd";
    case CompressAlgo::ZSTD_DICT: return "zstd+dict";
    default: return "none";
    }
}

int main() {
    CompressEngine engine;
    engine.init();

    fprintf(stdout, "=== Compression Ratio & Speed Benchmark ===\n");
    fprintf(stdout, "Page size: %zu bytes\n\n", kPageSize);

    size_t obj_sizes[] = { 16, 32, 64, 128, 256, 512, 1024, 4096 };
    constexpr int kIters = 1000;

    for (size_t obj_size : obj_sizes) {
        uint8_t sc = static_cast<uint8_t>(obj_size <= 128 ? obj_size / 16 - 1 : 8);

        // Train dictionary for this "size class". The allocator trains none by
        // default (kDictTrainSamples is 0), so the bench uses its own count:
        // with the allocator's, the zstd+dict row ran without a dictionary.
        constexpr int kSamples = 64;
        auto* sample_buf = static_cast<char*>(
            BootstrapAlloc::instance().allocate(
                static_cast<size_t>(kSamples) * kPageSize, kPageSize));
        size_t sample_sizes[kSamples];
        for (int i = 0; i < kSamples; ++i) {
            fillStructuredPage(sample_buf + static_cast<size_t>(i) * kPageSize,
                              static_cast<uint8_t>(i), obj_size);
            sample_sizes[i] = kPageSize;
        }
        engine.trainDictionary(sc, sample_buf, sample_sizes, kSamples);

        // Test page
        auto* test_page = static_cast<char*>(
            BootstrapAlloc::instance().allocate(kPageSize, kPageSize));
        fillStructuredPage(test_page, 0x42, obj_size);

        fprintf(stdout, "obj_size=%4zu (sc=%d, dict=%s):\n",
                obj_size, sc, engine.hasDictionary(sc) ? "yes" : "no");

        CompressAlgo algos[] = { CompressAlgo::LZ4, CompressAlgo::ZSTD, CompressAlgo::ZSTD_DICT };
        for (CompressAlgo algo : algos) {
            auto r = benchAlgo(engine, test_page, algo, sc, kIters);
            if (r.compressed_size > 0) {
                double ratio = static_cast<double>(r.compressed_size) / kPageSize;
                fprintf(stdout, "  %-10s: %5zu bytes (%.1f%%)  compress: %6.1f us  decompress: %6.1f us\n",
                        algoName(algo), r.compressed_size, ratio * 100.0,
                        r.compress_us, r.decompress_us);
            } else {
                fprintf(stdout, "  %-10s: FAILED\n", algoName(algo));
            }
        }
        fprintf(stdout, "\n");
    }

    return 0;
}
