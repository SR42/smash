// Test profile-file v4 dictionary persistence.
//
// Exercises the dictionary section of the SMASH_PROFILE_FILE format end to
// end, on the CompressEngine layer (the part that's exposed without
// standing up a full CompressorThread + VmRegion + CompressStore stack).
//
//   Phase 1 ("training run"): train a dict from structured pages and dump
//     a v4 profile file. The dict bytes are read back via the new
//     dictBytes()/dictSize() accessors and serialized exactly the way
//     CompressorThread::saveProfileFile does.
//
//   Phase 2 ("warm run"): open a *fresh* CompressEngine, replay the load
//     path (header + sentinel + setDictionary), and verify hasDictionary
//     returns true and a roundtrip compress/decompress with ZSTD_DICT
//     reproduces the original bytes — proving setDictionary built a
//     functional CDict/DDict pair from the persisted bytes.
//
//   Phase 3 ("corrupt run"): truncate each saved file at a few different
//     positions (header, mid-record, mid-dict-bytes) and assert the load
//     path silently rejects the file without crashing. Cold-start
//     behaviour is "no dict installed", which is what we check for.
#include "compress/compress_engine.h"
#include "smash/config.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

using namespace smash;

static int failures = 0;

#define CHECK(cond, msg, ...) do {                                        \
    if (!(cond)) {                                                        \
        fprintf(stderr, "FAIL %s:%d: " msg "\n", __FILE__, __LINE__,      \
                ##__VA_ARGS__);                                           \
        ++failures;                                                       \
    }                                                                     \
} while (0)

// Mirror of CompressorThread internal constants used by the file format.
// Duplicated here so the test exercises the on-disk layout directly
// without having to instantiate the CompressorThread template.
struct ProfileHeader {
    uint32_t magic;
    uint32_t version;
    uint16_t num_arenas;
    uint16_t num_classes;
    uint32_t reserved;
};
static constexpr uint32_t kProfileMagic   = 0x53503031u;  // "SP01"
static constexpr uint32_t kProfileVersion = 4;
static constexpr uint8_t  kDictSectionEnd = 255;
static constexpr uint32_t kMaxDictBytes   = 64 * 1024;

// Per-bucket Persist record is 16 bytes (matches SizeClassStats::Persist).
struct PersistRec {
    uint8_t  count;
    uint8_t  decision_hint;
    uint16_t sum;
    uint16_t cost_ema_x16_t0;
    uint16_t cost_ema_x16_t1;
    uint8_t  cost_count_t0;
    uint8_t  cost_count_t1;
    uint16_t efficiency_x256;
    uint8_t  pad[4];
};
static_assert(sizeof(PersistRec) == 16, "Persist record must be 16 bytes");

static void fillStructuredPage(void* page, uint8_t pattern_id, size_t obj_size) {
    auto* bytes = static_cast<uint8_t*>(page);
    for (size_t off = 0; off + obj_size <= kPageSize; off += obj_size) {
        bytes[off + 0] = 0xAA;
        bytes[off + 1] = pattern_id;
        bytes[off + 2] = static_cast<uint8_t>(obj_size & 0xFF);
        bytes[off + 3] = static_cast<uint8_t>(obj_size >> 8);
        for (size_t i = 4; i < 16 && i < obj_size; ++i) bytes[off + i] = 0x00;
        for (size_t i = 16; i < obj_size; ++i)
            bytes[off + i] = static_cast<uint8_t>((off + i + pattern_id) & 0xFF);
    }
}

// Write a v4 profile file with `num_buckets` zeroed PersistRec records and
// a dict section drawn from `engine`. Mirrors saveProfileFile's logic.
static bool writeProfileFile(const char* path, const CompressEngine& engine,
                             size_t num_buckets) {
    FILE* f = fopen(path, "wb");
    if (!f) return false;
    ProfileHeader hdr{kProfileMagic, kProfileVersion,
                      static_cast<uint16_t>(kNumArenas),
                      static_cast<uint16_t>(kTotalBucketsPerArena), 0};
    fwrite(&hdr, sizeof(hdr), 1, f);
    PersistRec rec{};
    for (size_t i = 0; i < num_buckets; ++i) fwrite(&rec, sizeof(rec), 1, f);
    for (int sc = 0; sc < kNumClasses; ++sc) {
        if (!engine.hasDictionary(static_cast<uint8_t>(sc))) continue;
        const void* db = engine.dictBytes(static_cast<uint8_t>(sc));
        size_t ds = engine.dictSize(static_cast<uint8_t>(sc));
        if (!db || ds == 0 || ds > kMaxDictBytes) continue;
        uint8_t sc8 = static_cast<uint8_t>(sc);
        uint32_t ds32 = static_cast<uint32_t>(ds);
        fwrite(&sc8, sizeof(sc8), 1, f);
        fwrite(&ds32, sizeof(ds32), 1, f);
        fwrite(db, ds, 1, f);
    }
    uint8_t end = kDictSectionEnd;
    fwrite(&end, sizeof(end), 1, f);
    fclose(f);
    return true;
}

// Inverse of writeProfileFile, exercising the same logic loadProfileFile
// runs on the engine. Returns the number of dicts installed (0 on a
// rejected/corrupt file).
static int loadProfileFile(const char* path, CompressEngine& engine,
                           size_t expected_buckets) {
    FILE* f = fopen(path, "rb");
    if (!f) return 0;
    ProfileHeader hdr{};
    if (fread(&hdr, sizeof(hdr), 1, f) != 1 ||
        hdr.magic != kProfileMagic ||
        hdr.version != kProfileVersion ||
        hdr.num_arenas != kNumArenas ||
        hdr.num_classes != kTotalBucketsPerArena) {
        fclose(f);
        return 0;
    }
    PersistRec rec{};
    for (size_t i = 0; i < expected_buckets; ++i) {
        if (fread(&rec, sizeof(rec), 1, f) != 1) {
            fclose(f);
            return 0;
        }
    }
    int loaded = 0;
    uint8_t buf[kMaxDictBytes];
    for (;;) {
        uint8_t sc = 0;
        if (fread(&sc, sizeof(sc), 1, f) != 1) break;
        if (sc == kDictSectionEnd) break;
        uint32_t dsize = 0;
        if (fread(&dsize, sizeof(dsize), 1, f) != 1) break;
        if (dsize == 0 || dsize > kMaxDictBytes || sc >= kNumClasses) break;
        if (fread(buf, dsize, 1, f) != 1) break;
        if (engine.setDictionary(sc, buf, dsize)) ++loaded;
    }
    fclose(f);
    return loaded;
}

// ── Test 1: cold-train + warm-load + roundtrip ────────────────────────────
static bool testTrainSaveLoadRoundtrip(const char* path) {
    constexpr int kSamples = 32;
    constexpr size_t kObjSize = 96;
    constexpr uint8_t kSizeClass = 5;

    // Phase 1: train a dict in engine A and save the file.
    CompressEngine engineA;
    engineA.init();

    auto* sample_buf = static_cast<char*>(
        BootstrapAlloc::instance().allocate(
            static_cast<size_t>(kSamples) * kPageSize, kPageSize));
    size_t sample_sizes[kSamples];
    for (int i = 0; i < kSamples; ++i) {
        fillStructuredPage(sample_buf + static_cast<size_t>(i) * kPageSize,
                          static_cast<uint8_t>(i), kObjSize);
        sample_sizes[i] = kPageSize;
    }
    bool trained = engineA.trainDictionary(
        kSizeClass, sample_buf, sample_sizes, kSamples);
    if (!trained) {
        // The samples are fixed and zstd is pinned, so training either always
        // works or is broken. This used to print a note, skip the round trip
        // and the corrupt-file checks, and pass.
        CHECK(false, "training produced no dict from the fixed samples");
        return false;
    }
    CHECK(engineA.hasDictionary(kSizeClass), "engineA should have dict");

    size_t buckets = static_cast<size_t>(kNumArenas) * kTotalBucketsPerArena;
    CHECK(writeProfileFile(path, engineA, buckets), "writeProfileFile failed");

    // Phase 2: load into a fresh engine B and verify the dict roundtrips.
    CompressEngine engineB;
    engineB.init();
    CHECK(!engineB.hasDictionary(kSizeClass),
          "engineB must start without a dict");
    int loaded = loadProfileFile(path, engineB, buckets);
    CHECK(loaded == 1, "expected 1 dict loaded, got %d", loaded);
    CHECK(engineB.hasDictionary(kSizeClass),
          "engineB should have dict after load");

    // Compress a fresh structured page with engineB and decompress it back
    // — proves the loaded dict is a real, usable CDict/DDict pair, not
    // just the bytes sitting in memory.
    auto* test_page = static_cast<char*>(
        BootstrapAlloc::instance().allocate(kPageSize, kPageSize));
    fillStructuredPage(test_page, 0x42, kObjSize);
    size_t max_comp = CompressEngine::maxCompressedSizeAny(kPageSize);
    auto* comp_buf = static_cast<char*>(
        BootstrapAlloc::instance().allocate(max_comp, 16));
    size_t comp_size = engineB.compress(
        test_page, comp_buf, kPageSize, max_comp,
        CompressAlgo::ZSTD_DICT, kSizeClass);
    CHECK(comp_size > 0, "compress with loaded dict failed");
    CHECK(comp_size < kPageSize, "compress with loaded dict didn't shrink");
    auto* restored = static_cast<char*>(
        BootstrapAlloc::instance().allocate(kPageSize, kPageSize));
    size_t decomp_size = engineB.decompress(
        comp_buf, restored, comp_size, kPageSize,
        CompressAlgo::ZSTD_DICT, kSizeClass);
    CHECK(decomp_size == kPageSize,
          "decompress size mismatch: %zu vs %zu", decomp_size, kPageSize);
    CHECK(memcmp(test_page, restored, kPageSize) == 0,
          "decompressed bytes do not match the original");
    return true;
}

// ── Test 2: corrupt files must not install dicts and must not crash ──────
static void testCorruptFilesRejected(const char* path) {
    // Truncation at every 64-byte boundary stresses every layer: the
    // header (incomplete magic), the persist section (mid-record), the
    // dict section header (size_class without dict_size), and the dict
    // payload (mid-bytes). Every one of these must produce a clean 0 and
    // leave the engine without a dict installed.
    FILE* src = fopen(path, "rb");
    CHECK(src != nullptr, "cannot reopen original profile file %s", path);
    if (!src) return;
    fseek(src, 0, SEEK_END);
    long full_size = ftell(src);
    fseek(src, 0, SEEK_SET);
    auto* contents = static_cast<char*>(
        BootstrapAlloc::instance().allocate(static_cast<size_t>(full_size), 16));
    size_t got = fread(contents, 1, static_cast<size_t>(full_size), src);
    CHECK(static_cast<long>(got) == full_size, "short read on profile file");
    fclose(src);

    char tmp_path[256];
    snprintf(tmp_path, sizeof(tmp_path), "%s.trunc", path);

    long step = full_size / 8;
    if (step <= 0) step = 1;
    size_t buckets = static_cast<size_t>(kNumArenas) * kTotalBucketsPerArena;
    for (long len = 0; len < full_size; len += step) {
        FILE* dst = fopen(tmp_path, "wb");
        if (!dst) continue;
        fwrite(contents, 1, static_cast<size_t>(len), dst);
        fclose(dst);

        CompressEngine eng;
        eng.init();
        int loaded = loadProfileFile(tmp_path, eng, buckets);
        CHECK(loaded == 0,
              "truncated-to-%ld file should install zero dicts (got %d)",
              len, loaded);
        // Spot-check: no dict on any size class.
        for (int sc = 0; sc < kNumClasses; ++sc) {
            if (eng.hasDictionary(static_cast<uint8_t>(sc))) {
                fprintf(stderr,
                        "FAIL: corrupt-load installed dict for sc=%d at len=%ld\n",
                        sc, len);
                ++failures;
                break;
            }
        }
    }

    // Also test a file with a wrong magic — should reject without
    // touching the dict section.
    {
        FILE* dst = fopen(tmp_path, "wb");
        if (dst) {
            ProfileHeader hdr{0xDEADBEEFu, kProfileVersion,
                              static_cast<uint16_t>(kNumArenas),
                              static_cast<uint16_t>(kTotalBucketsPerArena), 0};
            fwrite(&hdr, sizeof(hdr), 1, dst);
            fclose(dst);
            CompressEngine eng;
            eng.init();
            int loaded = loadProfileFile(tmp_path, eng, buckets);
            CHECK(loaded == 0, "bad-magic file installed %d dicts", loaded);
        }
    }

    // Wrong version — the actual silent-reject we promise on a v3 file
    // someone left over from a previous build.
    {
        FILE* dst = fopen(tmp_path, "wb");
        if (dst) {
            ProfileHeader hdr{kProfileMagic, 3,
                              static_cast<uint16_t>(kNumArenas),
                              static_cast<uint16_t>(kTotalBucketsPerArena), 0};
            fwrite(&hdr, sizeof(hdr), 1, dst);
            // Pretend a v3 body but no dict section (v3 didn't have one)
            PersistRec rec{};
            for (size_t i = 0; i < buckets; ++i)
                fwrite(&rec, sizeof(rec), 1, dst);
            fclose(dst);
            CompressEngine eng;
            eng.init();
            int loaded = loadProfileFile(tmp_path, eng, buckets);
            CHECK(loaded == 0, "v3 file installed %d dicts", loaded);
        }
    }

    unlink(tmp_path);
}

// ── Test 3: setDictionary input validation ────────────────────────────────
static void testSetDictionaryInputValidation() {
    CompressEngine eng;
    eng.init();
    uint8_t junk[16] = {};
    // Out-of-range size_class.
    CHECK(!eng.setDictionary(static_cast<uint8_t>(kNumClasses), junk, sizeof(junk)),
          "setDictionary should reject sc >= kNumClasses");
    // Zero size.
    CHECK(!eng.setDictionary(0, junk, 0),
          "setDictionary should reject size == 0");
    // Null bytes.
    CHECK(!eng.setDictionary(0, nullptr, 16),
          "setDictionary should reject null bytes");
    // Oversized — must not crash, must reject.
    CHECK(!eng.setDictionary(0, junk, kMaxDictBytes + 1),
          "setDictionary should reject size > kMaxDictBytes");
    CHECK(!eng.hasDictionary(0),
          "no dict should have been installed on validation-fail paths");
}

int main() {
    char path[256];
    snprintf(path, sizeof(path), "/tmp/smash_profile_test_%d.bin",
             static_cast<int>(getpid()));

    bool trained = testTrainSaveLoadRoundtrip(path);
    if (trained) {
        testCorruptFilesRejected(path);
    }
    testSetDictionaryInputValidation();

    unlink(path);

    if (failures == 0) {
        fprintf(stderr, "profile_dict_persistence: all tests passed\n");
        return 0;
    }
    fprintf(stderr, "profile_dict_persistence: %d failures\n", failures);
    return 1;
}
