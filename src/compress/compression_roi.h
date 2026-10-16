// smash/src/compress/compression_roi.h - ROI-based compression decision model
//
// Replaces fixed cold-tick thresholds with a cost/benefit model that considers
// compression ratio, CPU cost (compress + decompress), and cold duration.
// Auto-calibrates throughput at startup; all parameters configurable via env vars.
//
// Adding a new algorithm: add an AlgoProfile entry in ROIConfig::init().
// The ROI selection loop handles any number of profiles generically.
#pragma once

#include "smash/config.h"
#include "compress_engine.h"
#include "../core/bootstrap_alloc.h"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>

#include <lz4.h>
#define ZSTD_STATIC_LINKING_ONLY
#include <zstd.h>

namespace smash {

// Reward-normalization constant for UCB1-Tuned tier selection.
//
// Reward = bytes_saved / compress_us.  We clamp to [0, kUcbRewardMaxBytesPerUs]
// then divide by the max so the normalized reward is in [0,1] as required by
// UCB1-Tuned.  The cap is set above the realistic max throughput of the fast
// tier on a fully-compressible page (~10 KB/μs for LZ4 on zeros), giving
// real-world rewards mostly in the lower half of [0,1] but never saturating.
inline constexpr double kUcbRewardMaxBytesPerUs = 16384.0;

// Minimum pulls per arm before UCB is used for selection.  Below this we
// force-pull the under-explored arm.  Matches the standard "play each arm
// once" UCB1 initialization, but with a small fixed budget so the
// confidence-bound estimate isn't dominated by a single noisy sample.
// Default; overridable via SMASH_UCB_MIN_PULLS.
inline constexpr uint32_t kUcbMinPullsDefault = 4;

// UCB variant selector.
//   1 = UCB1-Tuned (Auer et al. 2002)
//   2 = UCB-V      (Audibert/Munos/Szepesvári 2009)
enum class UcbVariant : uint8_t {
    UCB1_TUNED = 1,
    UCB_V      = 2,
};

// --- Helper: read environment variables with defaults ---

inline int roiEnvInt(const char* name, int default_val) {
    const char* v = std::getenv(name);
    return v ? std::atoi(v) : default_val;
}

inline double roiEnvDouble(const char* name, double default_val) {
    const char* v = std::getenv(name);
    return v ? std::strtod(v, nullptr) : default_val;
}

// --- Algorithm throughput/cost profile ---

struct AlgoProfile {
    CompressAlgo algo = CompressAlgo::NONE;
    uint8_t  zstd_level = 0;      // zstd compression level (ignored if algo != ZSTD/ZSTD_DICT)
    uint16_t comp_mbs_hi = 0;     // compress throughput (MB/s) at high compressibility
    uint16_t comp_mbs_lo = 0;     // compress throughput (MB/s) at low compressibility
    uint16_t decomp_mbs_hi = 0;   // decompress throughput at high compressibility
    uint16_t decomp_mbs_lo = 0;   // decompress throughput at low compressibility
    uint8_t  min_cold_ticks = 0;  // minimum cold ticks before considering this algo
    uint8_t  ratio_scale_num = 1; // expected ratio vs fast-tier baseline (num/den)
    uint8_t  ratio_scale_den = 1;

    // Linear interpolation: better compression ratio -> higher throughput.
    // ratio_255: savings ratio (255 = 100% saved, 0 = nothing saved).
    uint32_t compThroughput(uint16_t ratio_255) const {
        uint32_t tp = static_cast<uint32_t>(comp_mbs_lo) +
            (static_cast<uint32_t>(comp_mbs_hi) - comp_mbs_lo) * ratio_255 / 255;
        return tp > 0 ? tp : 1;
    }

    uint32_t decompThroughput(uint16_t ratio_255) const {
        uint32_t tp = static_cast<uint32_t>(decomp_mbs_lo) +
            (static_cast<uint32_t>(decomp_mbs_hi) - decomp_mbs_lo) * ratio_255 / 255;
        return tp > 0 ? tp : 1;
    }

    // ROI = benefit / cost (byte-seconds of RSS saved per microsecond of CPU).
    //
    // benefit = kPageSize * (effective_ratio/255) * cold_count   [byte-seconds]
    // cost    = comp_us + decomp_us                              [microseconds]
    //
    // If `observed_comp_us > 0`, the caller has real timing data for a page
    // from the target (arena, size-class) bucket; use it in place of the
    // calibrated compression-throughput estimate.  Decompression cost is
    // still estimated from the calibrated profile because decompression
    // happens at fault time in the signal handler, where we cannot cheaply
    // attribute timing back to a bucket.
    // `redirty_discount_x256` (default 256 = 1.0 = no discount) scales the
    // benefit by the estimated probability the compressed blob SURVIVES without
    // being written again before we'd reclaim the RSS. It is the soft-dirty ROI
    // lever: a page in a (arena,size_class) bucket that historically gets
    // re-written soon after compression has a low survival probability, so its
    // benefit is discounted and it falls below the compress threshold — churn
    // avoided proactively, in the cost/benefit, rather than via reactive
    // backoff. 256 reproduces the exact prior behavior (gate off).
    uint32_t computeROI(uint8_t cold_count, uint16_t ratio_255,
                        uint32_t observed_comp_us = 0,
                        uint32_t redirty_discount_x256 = 256) const {
        // Scale ratio for algorithms that achieve better ratios than the
        // fast-tier baseline.
        uint16_t effective_ratio = ratio_255;
        if (ratio_scale_num != ratio_scale_den && ratio_scale_den > 0) {
            uint32_t scaled = static_cast<uint32_t>(ratio_255) *
                              ratio_scale_num / ratio_scale_den;
            effective_ratio = static_cast<uint16_t>(scaled > 255 ? 255 : scaled);
        }

        uint64_t benefit = static_cast<uint64_t>(kPageSize) *
                           effective_ratio * cold_count / 255;
        // Discount benefit by expected blob survival (soft-dirty ROI). Clamp
        // the multiplier to [0,256]; 256 leaves benefit unchanged.
        if (redirty_discount_x256 < 256) {
            benefit = benefit * redirty_discount_x256 / 256;
        }
        if (benefit == 0) return 0;

        // Cost: use observed microseconds when available, else derive from
        // calibrated throughput.  Both terms in x256 fixed point.
        uint32_t decomp_tp = decompThroughput(effective_ratio);
        uint32_t comp_cost_x256;
        if (observed_comp_us > 0) {
            comp_cost_x256 = observed_comp_us * 256;
        } else {
            uint32_t comp_tp = compThroughput(effective_ratio);
            comp_cost_x256 =
                static_cast<uint32_t>(kPageSize) * 256 / comp_tp;
        }
        uint32_t cost_x256 = comp_cost_x256 +
            static_cast<uint32_t>(kPageSize) * 256 / decomp_tp;
        if (cost_x256 == 0) return UINT32_MAX;

        uint64_t roi = benefit * 256 / cost_x256;
        return roi > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(roi);
    }
};

// --- ROI configuration singleton ---

struct ROIConfig {
    uint32_t roi_threshold = kRoiThresholdDefault;
    double min_compress_ratio = kMinCompressRatio;
    uint32_t cold_ticks_floor = kColdTicksDefault;
    uint32_t very_cold_ticks = kVeryColdTicks;

    // Recompression-thrash back-off (see config.h::kMaxBackoffShift et al.).
    // The master switch defaults to 1 (active). Set SMASH_RECOMPRESS_BACKOFF=0
    // to ablate; phase2 then ignores recompress_count_ and bucket EMAs and
    // gates only on cold_ticks_floor as before.
    bool recompress_backoff = true;

    // Soft-dirty ROI gate (SMASH_SOFTDIRTY_ROI, default OFF). When on, phase2
    // discounts a page's compression ROI benefit by the estimated probability
    // its blob survives without being re-WRITTEN (learned per-bucket from the
    // soft-dirty signal + a per-page write-clean streak). Proactively defers
    // churn-prone buckets in the cost/benefit instead of reactively after a
    // thrash. Off ⇒ discount is hard-256 (1.0) ⇒ behavior identical to today.
    bool softdirty_roi = false;

    // Fast-tier codec: LZ4 when true, else zstd at kZstdFastLevel (the
    // default). SMASH_CODEC=lz4|zstd picks it at startup. Read once in
    // init(), before calibration.
    bool fast_lz4 = false;

    const char* fastCodecName() const { return fast_lz4 ? "lz4" : "zstd1"; }

    AlgoProfile profiles[4];
    int num_profiles = 0;

    bool cold_ticks_overridden = false;
    bool very_cold_ticks_overridden = false;
    bool initialized = false;

    // UCB tier selection (opt-in via SMASH_UCB=1).  When true, the
    // compressor uses per-(arena, size_class) bucket statistics to pick a
    // tier via a multi-armed bandit instead of the cost/benefit ROI model.
    // The shouldCompress() gate is still applied first.  Reward is
    // normalized bytes-saved-per-microsecond (see kUcbRewardMaxBytesPerUs).
    bool use_ucb = false;
    // Which bandit formula to use.  See UcbVariant enum.
    // SMASH_UCB_VARIANT (1 = UCB1-Tuned default, 2 = UCB-V).
    UcbVariant ucb_variant = UcbVariant::UCB1_TUNED;
    // Force-pull threshold per arm before the bandit formula kicks in.
    // SMASH_UCB_MIN_PULLS overrides; useful for raising deep-tier exploration
    // when the reward distribution is heavy-tailed (rare big wins).
    uint32_t ucb_min_pulls = kUcbMinPullsDefault;
    // Per-arena warm-start prior.  When true, an under-explored bucket arm
    // (pulls < min_pulls) is blended with the corresponding arena-aggregate
    // arm posterior so cold buckets don't have to re-discover the deep tier.
    // SMASH_UCB_WARMSTART=1.
    bool ucb_warmstart = false;
    // Force-deep-every-N override.  Periodically pull the deep tier
    // unconditionally to keep its variance estimate live on heavy-tailed
    // reward distributions.  0 = disabled (default).  Per-bucket counter
    // ensures the forcing is spread across buckets, not synchronized.
    // SMASH_UCB_FORCE_DEEP_EVERY=N.
    uint32_t ucb_force_deep_every = 0;

    static ROIConfig& instance() {
        static ROIConfig cfg;
        return cfg;
    }

    void init(CompressEngine* engine) {
        if (initialized) return;
        initialized = true;

        // 1. Read policy env vars
        roi_threshold = static_cast<uint32_t>(
            roiEnvInt("SMASH_ROI_THRESHOLD", kRoiThresholdDefault));
        min_compress_ratio = roiEnvDouble("SMASH_MIN_COMPRESS_RATIO",
                                          kMinCompressRatio);
        use_ucb = roiEnvInt("SMASH_UCB", 0) != 0;
        int variant = roiEnvInt("SMASH_UCB_VARIANT",
                                static_cast<int>(UcbVariant::UCB1_TUNED));
        ucb_variant = (variant == 2) ? UcbVariant::UCB_V
                                      : UcbVariant::UCB1_TUNED;
        int mp = roiEnvInt("SMASH_UCB_MIN_PULLS",
                           static_cast<int>(kUcbMinPullsDefault));
        ucb_min_pulls = mp > 0 ? static_cast<uint32_t>(mp)
                                : kUcbMinPullsDefault;
        ucb_warmstart = roiEnvInt("SMASH_UCB_WARMSTART", 0) != 0;
        int fd = roiEnvInt("SMASH_UCB_FORCE_DEEP_EVERY", 0);
        ucb_force_deep_every = (fd > 0) ? static_cast<uint32_t>(fd) : 0;

        // SMASH_COLD_TIMEOUT_SEC is the primary time-space tradeoff dial.
        // It sets the idle time (in seconds) before a page is compressed.
        // SMASH_COLD_TICKS is the lower-level override (in tick counts).
        // Both are parsed, checked and clamped in one place (config.h).
        cold_ticks_floor = static_cast<uint32_t>(coldTicksSetting().ticks);
        cold_ticks_overridden = coldTicksSetting().overridden;

        const char* vct_env = std::getenv("SMASH_VERY_COLD_TICKS");
        if (vct_env) {
            very_cold_ticks = static_cast<uint32_t>(std::atoi(vct_env));
            very_cold_ticks_overridden = true;
        }

        if (const char* rb = std::getenv("SMASH_RECOMPRESS_BACKOFF")) {
            recompress_backoff = (std::atoi(rb) != 0);
        }
        if (const char* sr = std::getenv("SMASH_SOFTDIRTY_ROI")) {
            softdirty_roi = (std::atoi(sr) != 0);
        }
        if (const char* c = std::getenv("SMASH_CODEC")) {
            if (std::strcmp(c, "lz4") == 0) {
                fast_lz4 = true;
            } else if (std::strcmp(c, "zstd") == 0 || std::strcmp(c, "zstd1") == 0) {
                fast_lz4 = false;
            } else {
                char msg[160];
                const int n = std::snprintf(msg, sizeof(msg),
                    "[smash WARN] SMASH_CODEC=%.40s not recognised (lz4, zstd); using %s\n",
                    c, fastCodecName());
                if (n > 0) (void)!::write(2, msg, std::min(static_cast<size_t>(n), sizeof(msg) - 1));
            }
        }

        // 2. Try loading calibration file
        const char* cal_mode = std::getenv("SMASH_CALIBRATE");
        const char* cal_file = std::getenv("SMASH_CALIBRATION_FILE");
        bool mode_always = cal_mode && std::strcmp(cal_mode, "always") == 0;
        bool mode_never  = cal_mode && std::strcmp(cal_mode, "never") == 0;

        bool loaded = false;
        if (cal_file && !mode_always)
            loaded = loadCalibrationFile(cal_file);

        // 3. Auto-calibrate if needed
        bool just_calibrated = false;
        if (!loaded && !mode_never && engine) {
            calibrate(engine);
            just_calibrated = true;
        }
        if (!loaded && !just_calibrated)
            useDefaults();

        // 4. Apply per-throughput env var overrides
        applyEnvOverrides();

        // 5. Remove the deep-tier profile if very_cold_ticks disabled it.
        // Only the deep level: the fast tier is zstd too unless it is LZ4,
        // and removing every zstd profile left none, so nothing was ever
        // compressed.
        if (very_cold_ticks_overridden && very_cold_ticks >= 9999) {
            int new_count = 0;
            for (int i = 0; i < num_profiles; ++i) {
                if (!(profiles[i].algo == CompressAlgo::ZSTD &&
                      profiles[i].zstd_level == kZstdDeepLevel))
                    profiles[new_count++] = profiles[i];
            }
            num_profiles = new_count;
        }

        // 6. Save calibration file if requested
        if (cal_file && just_calibrated)
            saveCalibrationFile(cal_file);
    }

private:
    // Calibration data (stored for file save).
    //
    // "fast" = the fast-tier algorithm actually run at compression time
    //   (LZ4 when fast_lz4, else zstd-1).
    // "deep" = zstd at kZstdDeepLevel (always).
    struct CalData {
        uint16_t fast_comp_hi = 2000, fast_comp_lo = 100;     // zstd-1 defaults
        uint16_t fast_decomp_hi = 4400, fast_decomp_lo = 1600;
        uint16_t deep_comp_hi = 700, deep_comp_lo = 45;
        uint16_t deep_decomp_hi = 3800, deep_decomp_lo = 1100;
        double fast_ratio_zeros = 0.011, fast_ratio_random = 1.0;
        double deep_ratio_zeros = 0.010, deep_ratio_random = 1.0;
    } cal_;

    void useDefaults() {
        buildProfiles();
    }

    void buildProfiles() {
        num_profiles = 0;

        // Fast-tier profile.  algo and zstd_level match what compressPage()
        // will actually invoke; calibration data reflects that same algo.
        AlgoProfile fast{};
        if (fast_lz4) {
            fast.algo = CompressAlgo::LZ4;
            fast.zstd_level = 0;
        } else {
            fast.algo = CompressAlgo::ZSTD;
            fast.zstd_level = kZstdFastLevel;
        }
        fast.comp_mbs_hi   = cal_.fast_comp_hi;
        fast.comp_mbs_lo   = cal_.fast_comp_lo;
        fast.decomp_mbs_hi = cal_.fast_decomp_hi;
        fast.decomp_mbs_lo = cal_.fast_decomp_lo;
        fast.min_cold_ticks = static_cast<uint8_t>(
            cold_ticks_floor > 255 ? 255 : cold_ticks_floor);
        fast.ratio_scale_num = 1;
        fast.ratio_scale_den = 1;
        profiles[num_profiles++] = fast;

        // Deep-tier profile (zstd at kZstdDeepLevel). Gated off by default —
        // see kDeepTierEnabled in config.h. When disabled, num_profiles stays
        // 1 and selectProfile() short-circuits to profiles[0] (zstd-1), so the
        // system is single-tier.
        if constexpr (kDeepTierEnabled) {
            // Scale the observed fast-tier ratio to estimate deep-tier ratio.
            uint8_t scale_num = 1, scale_den = 1;
            if (cal_.fast_ratio_zeros > 0.001 && cal_.deep_ratio_zeros > 0.001) {
                double fast_sav = 1.0 - cal_.fast_ratio_zeros;
                double deep_sav = 1.0 - cal_.deep_ratio_zeros;
                if (fast_sav > 0.01) {
                    double scale = deep_sav / fast_sav;
                    if      (scale >= 1.5)  { scale_num = 3;  scale_den = 2; }
                    else if (scale >= 1.3)  { scale_num = 4;  scale_den = 3; }
                    else if (scale >= 1.1)  { scale_num = 11; scale_den = 10; }
                }
            }

            AlgoProfile deep{};
            deep.algo = CompressAlgo::ZSTD;
            deep.zstd_level = kZstdDeepLevel;
            deep.comp_mbs_hi   = cal_.deep_comp_hi;
            deep.comp_mbs_lo   = cal_.deep_comp_lo;
            deep.decomp_mbs_hi = cal_.deep_decomp_hi;
            deep.decomp_mbs_lo = cal_.deep_decomp_lo;
            deep.min_cold_ticks = static_cast<uint8_t>(
                very_cold_ticks > 255 ? 255 : very_cold_ticks);
            deep.ratio_scale_num = scale_num;
            deep.ratio_scale_den = scale_den;
            profiles[num_profiles++] = deep;
        }
    }

    void calibrate(CompressEngine* engine) {
        // Allocate test pages from BootstrapAlloc (48 KB permanent)
        void* zeros_page = BootstrapAlloc::instance().allocate(kPageSize, kPageSize);
        void* random_page = BootstrapAlloc::instance().allocate(kPageSize, kPageSize);
        size_t max_comp = CompressEngine::maxCompressedSizeAny(kPageSize);
        void* comp_buf = BootstrapAlloc::instance().allocate(max_comp, 16);
        void* decomp_buf = BootstrapAlloc::instance().allocate(kPageSize, kPageSize);

        if (!zeros_page || !random_page || !comp_buf || !decomp_buf) {
            useDefaults();
            return;
        }

        std::memset(zeros_page, 0, kPageSize);

        // Fill random page with splitmix64
        uint64_t rng = 0xDEADBEEFCAFE1234ULL;
        auto* rp = static_cast<uint64_t*>(random_page);
        for (size_t i = 0; i < kPageSize / 8; ++i) {
            rng ^= rng >> 30;
            rng *= 0xBF58476D1CE4E5B9ULL;
            rng ^= rng >> 27;
            rng *= 0x94D049BB133111EBULL;
            rng ^= rng >> 31;
            rp[i] = rng;
        }

        // Reuse engine's pre-allocated contexts (no extra allocation)
        void* lz4_state = engine->getLz4State();
        ZSTD_CCtx* zstd_cctx = engine->getZstdCCtx();
        ZSTD_DCtx* zstd_dctx = engine->getZstdDCtx();

        if (!lz4_state || !zstd_cctx || !zstd_dctx) {
            useDefaults();
            return;
        }

        static constexpr int kCalIters = 10;
        using Clock = std::chrono::steady_clock;

        struct TimingResult {
            double comp_us;
            double decomp_us;
            size_t comp_size;
        };

        auto benchLZ4 = [&](void* page) -> TimingResult {
            double comp_times[kCalIters], decomp_times[kCalIters];
            size_t last_comp_size = 0;

            for (int iter = 0; iter < kCalIters; ++iter) {
                auto t0 = Clock::now();
                int csz = LZ4_compress_fast_extState(
                    lz4_state,
                    static_cast<const char*>(page),
                    static_cast<char*>(comp_buf),
                    static_cast<int>(kPageSize),
                    static_cast<int>(max_comp), 1);
                auto t1 = Clock::now();
                comp_times[iter] = std::chrono::duration<double, std::micro>(
                    t1 - t0).count();
                last_comp_size = csz > 0 ? static_cast<size_t>(csz) : 0;
            }

            for (int iter = 0; iter < kCalIters; ++iter) {
                auto t0 = Clock::now();
                LZ4_decompress_safe(
                    static_cast<const char*>(comp_buf),
                    static_cast<char*>(decomp_buf),
                    static_cast<int>(last_comp_size),
                    static_cast<int>(kPageSize));
                auto t1 = Clock::now();
                decomp_times[iter] = std::chrono::duration<double, std::micro>(
                    t1 - t0).count();
            }

            std::sort(comp_times, comp_times + kCalIters);
            std::sort(decomp_times, decomp_times + kCalIters);
            return {comp_times[kCalIters / 2],
                    decomp_times[kCalIters / 2],
                    last_comp_size};
        };

        auto benchZstdAtLevel = [&](void* page, int level) -> TimingResult {
            double comp_times[kCalIters], decomp_times[kCalIters];
            size_t last_comp_size = 0;

            for (int iter = 0; iter < kCalIters; ++iter) {
                auto t0 = Clock::now();
                size_t csz = ZSTD_compressCCtx(
                    zstd_cctx, comp_buf, max_comp,
                    page, kPageSize, level);
                auto t1 = Clock::now();
                comp_times[iter] = std::chrono::duration<double, std::micro>(
                    t1 - t0).count();
                last_comp_size = ZSTD_isError(csz) ? 0 : csz;
            }

            for (int iter = 0; iter < kCalIters; ++iter) {
                auto t0 = Clock::now();
                ZSTD_decompressDCtx(zstd_dctx, decomp_buf, kPageSize,
                                     comp_buf, last_comp_size);
                auto t1 = Clock::now();
                decomp_times[iter] = std::chrono::duration<double, std::micro>(
                    t1 - t0).count();
            }

            std::sort(comp_times, comp_times + kCalIters);
            std::sort(decomp_times, decomp_times + kCalIters);
            return {comp_times[kCalIters / 2],
                    decomp_times[kCalIters / 2],
                    last_comp_size};
        };

        auto toMBps = [](double us) -> uint16_t {
            if (us <= 0.001) return 60000;
            double mbs = static_cast<double>(kPageSize) / us;
            return static_cast<uint16_t>(
                std::min(mbs, 60000.0));
        };

        // Benchmark the fast tier (LZ4 or zstd-1, per fast_lz4).
        TimingResult fast_zeros, fast_random;
        if (fast_lz4) {
            fast_zeros  = benchLZ4(zeros_page);
            fast_random = benchLZ4(random_page);
        } else {
            fast_zeros  = benchZstdAtLevel(zeros_page, kZstdFastLevel);
            fast_random = benchZstdAtLevel(random_page, kZstdFastLevel);
        }
        cal_.fast_comp_hi   = toMBps(fast_zeros.comp_us);
        cal_.fast_comp_lo   = toMBps(fast_random.comp_us);
        cal_.fast_decomp_hi = toMBps(fast_zeros.decomp_us);
        cal_.fast_decomp_lo = toMBps(fast_random.decomp_us);
        cal_.fast_ratio_zeros  = fast_zeros.comp_size > 0
            ? static_cast<double>(fast_zeros.comp_size) / kPageSize : 1.0;
        cal_.fast_ratio_random = fast_random.comp_size > 0
            ? static_cast<double>(fast_random.comp_size) / kPageSize : 1.0;

        // Benchmark the deep tier (zstd at kZstdDeepLevel).
        auto deep_zeros  = benchZstdAtLevel(zeros_page, kZstdDeepLevel);
        auto deep_random = benchZstdAtLevel(random_page, kZstdDeepLevel);
        cal_.deep_comp_hi   = toMBps(deep_zeros.comp_us);
        cal_.deep_comp_lo   = toMBps(deep_random.comp_us);
        cal_.deep_decomp_hi = toMBps(deep_zeros.decomp_us);
        cal_.deep_decomp_lo = toMBps(deep_random.decomp_us);
        cal_.deep_ratio_zeros  = deep_zeros.comp_size > 0
            ? static_cast<double>(deep_zeros.comp_size) / kPageSize : 1.0;
        cal_.deep_ratio_random = deep_random.comp_size > 0
            ? static_cast<double>(deep_random.comp_size) / kPageSize : 1.0;

        buildProfiles();
    }

    void applyEnvOverrides() {
        // profiles[0] is the fast tier, profiles[1] is the deep tier.
        const char* v;
        if (num_profiles >= 1) {
            auto& p = profiles[0];
            if ((v = std::getenv("SMASH_FAST_COMP_MBS_HI")))
                p.comp_mbs_hi = static_cast<uint16_t>(std::atoi(v));
            if ((v = std::getenv("SMASH_FAST_COMP_MBS_LO")))
                p.comp_mbs_lo = static_cast<uint16_t>(std::atoi(v));
            if ((v = std::getenv("SMASH_FAST_DECOMP_MBS_HI")))
                p.decomp_mbs_hi = static_cast<uint16_t>(std::atoi(v));
            if ((v = std::getenv("SMASH_FAST_DECOMP_MBS_LO")))
                p.decomp_mbs_lo = static_cast<uint16_t>(std::atoi(v));
        }
        if (num_profiles >= 2) {
            auto& p = profiles[1];
            if ((v = std::getenv("SMASH_DEEP_COMP_MBS_HI")))
                p.comp_mbs_hi = static_cast<uint16_t>(std::atoi(v));
            if ((v = std::getenv("SMASH_DEEP_COMP_MBS_LO")))
                p.comp_mbs_lo = static_cast<uint16_t>(std::atoi(v));
            if ((v = std::getenv("SMASH_DEEP_DECOMP_MBS_HI")))
                p.decomp_mbs_hi = static_cast<uint16_t>(std::atoi(v));
            if ((v = std::getenv("SMASH_DEEP_DECOMP_MBS_LO")))
                p.decomp_mbs_lo = static_cast<uint16_t>(std::atoi(v));
        }
    }

    // Both run inside the heap's constructor (via init()): no stdio here.
    // fopen() allocates its FILE with malloc, which waits for the heap being
    // constructed, so setting SMASH_CALIBRATION_FILE hung every process at
    // startup.
    bool loadCalibrationFile(const char* path) {
        const int fd = ::open(path, O_RDONLY | O_CLOEXEC);
        if (fd < 0) return false;

        char buf[4096];
        size_t n = 0;
        for (ssize_t r; n < sizeof(buf) - 1 &&
                        (r = ::read(fd, buf + n, sizeof(buf) - 1 - n)) != 0;) {
            if (r < 0) {
                if (errno == EINTR) continue;
                break;
            }
            n += static_cast<size_t>(r);
        }
        ::close(fd);
        if (n == 0) return false;
        buf[n] = '\0';

        auto extractUint = [&](const char* key) -> int {
            const char* p = std::strstr(buf, key);
            if (!p) return -1;
            p += std::strlen(key);
            while (*p && *p != ':') ++p;
            if (*p == ':') ++p;
            while (*p == ' ' || *p == '\t') ++p;
            return std::atoi(p);
        };
        auto extractDouble = [&](const char* key) -> double {
            const char* p = std::strstr(buf, key);
            if (!p) return -1.0;
            p += std::strlen(key);
            while (*p && *p != ':') ++p;
            if (*p == ':') ++p;
            while (*p == ' ' || *p == '\t') ++p;
            return std::strtod(p, nullptr);
        };

        // The fast_* figures belong to one codec. A file without the tag
        // predates SMASH_CODEC: zstd-1, unless written by an LZ4 build (the
        // removed SMASH_USE_LZ4 option), which nothing here can tell apart.
        bool file_lz4 = false;
        if (const char* t = std::strstr(buf, "\"fast_codec\"")) {
            t += std::strlen("\"fast_codec\"");
            while (*t && *t != ':') ++t;
            if (*t == ':') ++t;
            while (*t == ' ' || *t == '\t') ++t;
            file_lz4 = std::strncmp(t, "\"lz4\"", 5) == 0;
        }
        if (file_lz4 != fast_lz4) return false;  // recalibrate and rewrite it

        int v;
        double d;
        if ((v = extractUint("\"fast_comp_mbs_hi\""))   >= 0)
            cal_.fast_comp_hi   = static_cast<uint16_t>(v);
        if ((v = extractUint("\"fast_comp_mbs_lo\""))   >= 0)
            cal_.fast_comp_lo   = static_cast<uint16_t>(v);
        if ((v = extractUint("\"fast_decomp_mbs_hi\"")) >= 0)
            cal_.fast_decomp_hi = static_cast<uint16_t>(v);
        if ((v = extractUint("\"fast_decomp_mbs_lo\"")) >= 0)
            cal_.fast_decomp_lo = static_cast<uint16_t>(v);
        if ((v = extractUint("\"deep_comp_mbs_hi\""))  >= 0)
            cal_.deep_comp_hi  = static_cast<uint16_t>(v);
        if ((v = extractUint("\"deep_comp_mbs_lo\""))  >= 0)
            cal_.deep_comp_lo  = static_cast<uint16_t>(v);
        if ((v = extractUint("\"deep_decomp_mbs_hi\""))>= 0)
            cal_.deep_decomp_hi= static_cast<uint16_t>(v);
        if ((v = extractUint("\"deep_decomp_mbs_lo\""))>= 0)
            cal_.deep_decomp_lo= static_cast<uint16_t>(v);

        if ((d = extractDouble("\"fast_ratio_zeros\""))  >= 0)
            cal_.fast_ratio_zeros  = d;
        if ((d = extractDouble("\"fast_ratio_random\"")) >= 0)
            cal_.fast_ratio_random = d;
        if ((d = extractDouble("\"deep_ratio_zeros\"")) >= 0)
            cal_.deep_ratio_zeros = d;
        if ((d = extractDouble("\"deep_ratio_random\""))>= 0)
            cal_.deep_ratio_random= d;

        buildProfiles();
        return true;
    }

    void saveCalibrationFile(const char* path) {
        char out[1024];
        const int n = std::snprintf(out, sizeof(out),
            "{\n"
            "  \"page_size\": %zu,\n"
            "  \"fast_codec\": \"%s\",\n"
            "  \"fast_comp_mbs_hi\": %u,\n"
            "  \"fast_comp_mbs_lo\": %u,\n"
            "  \"fast_decomp_mbs_hi\": %u,\n"
            "  \"fast_decomp_mbs_lo\": %u,\n"
            "  \"fast_ratio_zeros\": %.6f,\n"
            "  \"fast_ratio_random\": %.6f,\n"
            "  \"deep_comp_mbs_hi\": %u,\n"
            "  \"deep_comp_mbs_lo\": %u,\n"
            "  \"deep_decomp_mbs_hi\": %u,\n"
            "  \"deep_decomp_mbs_lo\": %u,\n"
            "  \"deep_ratio_zeros\": %.6f,\n"
            "  \"deep_ratio_random\": %.6f\n"
            "}\n",
            static_cast<size_t>(kPageSize), fastCodecName(),
            cal_.fast_comp_hi, cal_.fast_comp_lo,
            cal_.fast_decomp_hi, cal_.fast_decomp_lo,
            cal_.fast_ratio_zeros, cal_.fast_ratio_random,
            cal_.deep_comp_hi, cal_.deep_comp_lo,
            cal_.deep_decomp_hi, cal_.deep_decomp_lo,
            cal_.deep_ratio_zeros, cal_.deep_ratio_random);
        if (n <= 0 || static_cast<size_t>(n) >= sizeof(out)) return;
        const int fd = ::open(path, O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
        if (fd < 0) return;
        for (size_t off = 0; off < static_cast<size_t>(n);) {
            const ssize_t w = ::write(fd, out + off, static_cast<size_t>(n) - off);
            if (w < 0 && errno == EINTR) continue;
            if (w <= 0) break;
            off += static_cast<size_t>(w);
        }
        ::close(fd);
    }
};

// --- Compression decision functions ---

namespace CompressionROI {

// Should this page be compressed?
// cold_count: ticks without access
// stats_count: SizeClassStats::count (sliding window sample count)
// stats_sum: SizeClassStats::sum (sum of ratio_255 values)
// redirty_discount_x256: soft-dirty ROI survival multiplier (256 = no discount)
inline bool shouldCompress(uint8_t cold_count,
                            uint8_t stats_count, uint16_t stats_sum,
                            uint32_t redirty_discount_x256 = 256) {
    auto& cfg = ROIConfig::instance();

    // Hard floor: never compress before minimum ticks
    if (cold_count < cfg.cold_ticks_floor) return false;

    // Ablation: if cold_ticks was explicitly overridden, use fixed threshold only
    if (cfg.cold_ticks_overridden) return true;

#ifdef SMASH_ABLATION_NO_SKIP_STATS
    return true;
#endif

    // Not enough samples to estimate ratio — optimistically compress.
    // EXCEPTION: even with thin ratio data, a bucket with a strong learned
    // re-dirty signal (deep discount) should still be gated, else churn-prone
    // buckets escape the ROI check during their (long) warm-up window.
    if (stats_count < 8 && redirty_discount_x256 >= 256) return true;

    // Compute LZ4 ROI using average ratio from sliding window. With thin ratio
    // data assume a neutral mid ratio so the discount still has something to act
    // on (otherwise stats_sum/stats_count is undefined for count 0).
    uint16_t avg_ratio = (stats_count >= 8) ? (stats_sum / stats_count) : 128;
    uint32_t roi = cfg.profiles[0].computeROI(cold_count, avg_ratio, 0,
                                              redirty_discount_x256);
    return roi >= cfg.roi_threshold;
}

// Select the best profile for this page.  Returns a pointer into the
// ROIConfig's profiles array; caller reads profile->algo and
// profile->zstd_level to drive compressPage().
//
// When enough samples are available, picks the profile with the highest
// ROI above the per-profile cold-tick threshold.  Otherwise falls back
// to the fast tier (profile 0).
//
// `observed_costs_us[i]`, if > 0, overrides profile[i]'s calibrated
// compression-throughput estimate with the real observed time (μs per
// page) from the workload's actual (arena, size-class) bucket.  Callers
// who do not have per-bucket timing can pass nullptr.
inline const AlgoProfile* selectProfile(uint8_t cold_count,
                                         uint8_t stats_count,
                                         uint16_t stats_sum,
                                         const uint32_t* observed_costs_us = nullptr) {
    auto& cfg = ROIConfig::instance();
    if (cfg.num_profiles == 0) return nullptr;

    if (cfg.num_profiles == 1 || stats_count < 8) {
        return &cfg.profiles[0];
    }

    uint16_t base_ratio = stats_sum / stats_count;

    const AlgoProfile* best = &cfg.profiles[0];
    uint32_t best_roi = 0;
    for (int i = 0; i < cfg.num_profiles; ++i) {
        const auto& p = cfg.profiles[i];
        if (cold_count < p.min_cold_ticks) continue;
        uint32_t observed = observed_costs_us ? observed_costs_us[i] : 0;
        uint32_t roi = p.computeROI(cold_count, base_ratio, observed);
        if (roi > best_roi) {
            best_roi = roi;
            best = &p;
        }
    }
    return best;
}

// UCB1-Tuned tier selection.
//
// Implements Auer/Cesa-Bianchi/Fischer 2002 §4 ("UCB1-Tuned"): for each arm
// i with empirical reward mean x̄_i, sample variance s²_i, and pull count
// n_i, choose the arm maximizing
//
//     x̄_i + sqrt( ln(n) / n_i  ·  min(1/4, V_i(n_i)) )
//
// where V_i(n_i) = s²_i + sqrt(2 ln(n) / n_i) is a per-arm
// variance-aware exploration term and 1/4 is the worst-case variance for a
// reward in [0,1].  The min() is what distinguishes UCB1-Tuned from plain
// UCB1 — arms with low observed variance contract their bound faster.
//
// Inputs:
//   pulls[i]     — number of times arm i has been pulled in this bucket
//   mean[i]      — empirical reward mean (Welford running mean)
//   m2[i]        — Welford M2 (sum of squared deviations)
//   eligible[i]  — true if arm passes the per-profile min_cold_ticks gate
//   num_arms     — length of arrays (must be ≥ 1, ≤ 4)
//   min_pulls    — force-pull threshold per arm (typically 4-32)
//   variant      — UCB1_TUNED or UCB_V
//
// Returns the index of the chosen arm, or -1 if no arms are eligible.
// Forces a pull of any eligible arm with pulls < min_pulls before the
// confidence-bound formula kicks in (otherwise the variance estimate is
// meaningless on small n).
//
// UCB-V (Audibert/Munos/Szepesvári 2009, "Exploration-exploitation tradeoff
// using variance estimates in multi-armed bandits") replaces the UCB1-Tuned
// bound with
//
//     x̄_i + sqrt( 2 · s²_i · ln(n) / n_i ) + 3 · b · ln(n) / n_i
//
// where b is the reward range upper bound (1.0 here, since rewards are
// normalized to [0,1]).  Compared to UCB1-Tuned, the second term decays
// as ln(n)/n_i (linear in ln(n) over n_i) rather than sqrt(ln(n)/n_i),
// which gives UCB-V a longer "exploration tail": low-pull arms hold a
// larger bonus for longer, so rare-big-win arms (deep tier on highly
// compressible workloads) get pulled more aggressively.
inline int selectTierUCB(const uint32_t* pulls, const double* mean,
                         const double* m2, const bool* eligible,
                         int num_arms, uint32_t min_pulls,
                         UcbVariant variant,
                         const uint32_t* prior_pulls = nullptr,
                         const double* prior_mean = nullptr,
                         const double* prior_m2 = nullptr) {
    if (num_arms <= 0) return -1;

    // Optional warm-start: when the bucket arm is under-explored
    // (pulls < min_pulls) and a non-empty arena prior exists, blend the two
    // posteriors via additive Welford combine.  The prior is capped at
    // (min_pulls - bucket_pulls) effective pulls so once the bucket has its
    // own data the prior fades out quickly — this avoids letting a strong
    // arena prior drown out a (legitimately) divergent bucket signal.
    uint32_t eff_pulls[4];
    double   eff_mean[4];
    double   eff_m2[4];
    bool have_prior = (prior_pulls && prior_mean && prior_m2);
    for (int i = 0; i < num_arms; ++i) {
        eff_pulls[i] = pulls[i];
        eff_mean[i]  = mean[i];
        eff_m2[i]    = m2[i];
        if (!have_prior) continue;
        if (pulls[i] >= min_pulls || prior_pulls[i] == 0) continue;
        // Cap prior contribution.
        uint32_t cap = (min_pulls > pulls[i]) ? (min_pulls - pulls[i]) : 0;
        uint32_t p_n = prior_pulls[i] < cap ? prior_pulls[i] : cap;
        if (p_n == 0) continue;
        // Welford parallel combine (Chan/Golub/LeVeque): two streams with
        // (n_a, μ_a, M2_a) and (n_b, μ_b, M2_b) merge to
        //   n   = n_a + n_b
        //   δ   = μ_b - μ_a
        //   μ   = μ_a + δ · n_b / n
        //   M2  = M2_a + M2_b + δ² · n_a · n_b / n
        // Treat the bucket as stream A and the (capped) prior as stream B.
        double n_a = static_cast<double>(pulls[i]);
        double n_b = static_cast<double>(p_n);
        // Scale prior's M2 down by the cap ratio so we keep its variance
        // estimate proportional to the truncated pull count.
        double scale = (prior_pulls[i] > 0)
            ? n_b / static_cast<double>(prior_pulls[i]) : 0.0;
        double m2_b  = prior_m2[i] * scale;
        double total_n = n_a + n_b;
        double delta   = prior_mean[i] - mean[i];
        eff_pulls[i] = pulls[i] + p_n;
        eff_mean[i]  = mean[i] + delta * n_b / total_n;
        eff_m2[i]    = m2[i] + m2_b + delta * delta * n_a * n_b / total_n;
    }

    // Step 1: force-pull any under-explored eligible arm (uses raw bucket
    // pull count, not effective — the prior is for *score*, not for
    // skipping the explore phase).
    int forced = -1;
    uint32_t forced_pulls = min_pulls;
    for (int i = 0; i < num_arms; ++i) {
        if (!eligible[i]) continue;
        if (pulls[i] < forced_pulls) {
            forced = i;
            forced_pulls = pulls[i];
        }
    }
    if (forced >= 0) return forced;

    // Step 2: total effective pulls across eligible arms (n in the UCB formula).
    uint64_t total = 0;
    for (int i = 0; i < num_arms; ++i)
        if (eligible[i]) total += eff_pulls[i];
    if (total == 0) {
        for (int i = 0; i < num_arms; ++i)
            if (eligible[i]) return i;
        return -1;
    }

    double ln_n = std::log(static_cast<double>(total));
    int best = -1;
    double best_score = -1.0;
    for (int i = 0; i < num_arms; ++i) {
        if (!eligible[i] || eff_pulls[i] == 0) continue;
        double n_i = static_cast<double>(eff_pulls[i]);
        // Sample variance (Bessel-corrected when n>1, else 0).
        double var = (eff_pulls[i] > 1) ? eff_m2[i] / (n_i - 1.0) : 0.0;

        double bonus;
        if (variant == UcbVariant::UCB_V) {
            // UCB-V: 2 · s² · ln(n) / n_i  +  3 · b · ln(n) / n_i, b=1.
            bonus = std::sqrt(2.0 * var * ln_n / n_i) +
                    3.0 * ln_n / n_i;
        } else {
            // UCB1-Tuned default.
            double v_i = var + std::sqrt(2.0 * ln_n / n_i);
            if (v_i > 0.25) v_i = 0.25;
            bonus = std::sqrt((ln_n / n_i) * v_i);
        }
        double score = eff_mean[i] + bonus;
        if (score > best_score) {
            best_score = score;
            best = i;
        }
    }
    return best;
}

// Compute a UCB reward in [0,1] from observed compression outcome.
// reward = clamp(bytes_saved / compress_us, 0, kUcbRewardMaxBytesPerUs)
//        / kUcbRewardMaxBytesPerUs
// A page that fails to compress (or compresses larger) yields reward 0;
// a maximally efficient page (huge savings, tiny CPU) saturates at 1.
inline double ucbReward(size_t comp_size, size_t orig_size,
                        uint32_t comp_us) {
    if (comp_us == 0 || orig_size == 0 || comp_size >= orig_size) return 0.0;
    double saved = static_cast<double>(orig_size - comp_size);
    double bps = saved / static_cast<double>(comp_us);
    if (bps <= 0.0) return 0.0;
    if (bps >= kUcbRewardMaxBytesPerUs) return 1.0;
    return bps / kUcbRewardMaxBytesPerUs;
}

// Back-compat wrapper: return just the algorithm.  When the winning
// profile is ZSTD and a trained dictionary exists for this size class,
// switch to ZSTD_DICT (dict is applied at the deep level only).
inline CompressAlgo selectAlgorithm(uint8_t cold_count,
                                     uint8_t stats_count, uint16_t stats_sum,
                                     CompressEngine* engine, uint8_t sc) {
    const AlgoProfile* p = selectProfile(cold_count, stats_count, stats_sum);
    if (!p) return CompressAlgo::NONE;
    CompressAlgo algo = p->algo;
    if (algo == CompressAlgo::ZSTD && p->zstd_level == kZstdDeepLevel &&
        engine && engine->hasDictionary(sc))
        algo = CompressAlgo::ZSTD_DICT;
    return algo;
}

} // namespace CompressionROI
} // namespace smash
