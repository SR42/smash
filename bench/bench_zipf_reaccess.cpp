// bench_zipf_reaccess.cpp - Zipfian re-access microbenchmark for the
// compression churn-avoidance gate (Lindy / Bayesian cold-age models).
//
// WHY THIS EXISTS
// ---------------
// bench_coldhotmix splits objects into a fixed hot set (always touched) and a
// cold set (never touched) — access pattern separates them cleanly, so the
// compressor's cold_count reset already handles it and there is no churn for a
// thrash-prediction gate to govern. bench_sqlite re-accesses uniformly, so
// every page is equally thrashy and there is nothing to discriminate. Neither
// can tell a smart cold-age gate from the blunt exponential backoff.
//
// This benchmark creates the regime where cold-age IS predictive of re-access:
// each surviving object is assigned a HIDDEN access rate drawn from a Zipfian
// (power-law) distribution, then re-accessed continuously by sampling objects
// in proportion to those rates. Because the rate is hidden and Zipfian:
//   - a few objects are hot (touched constantly → cold_count never reaches the
//     floor → never compressed),
//   - a long tail is genuinely cold (touched ~never → safely compressible),
//   - and a MIDDLE BAND is touched on a timescale near the cold timeout — these
//     are the thrash-prone pages.
// Crucially, with Poisson accesses at a Zipf-distributed hidden rate, the
// marginal inter-access time is heavy-tailed: "has been cold for k seconds"
// is real evidence the object is low-rate (decreasing hazard = Lindy's law).
// A cold-age gate that exploits this should compress the cold tail (RSS win)
// WITHOUT compressing the warm middle (avoiding thrash), where the blunt
// backoff must either thrash or back off the whole bucket and lose RSS.
//
// --zipf-s controls the skew: 0.0 = uniform (degenerate, like sqlite, all
// thrash), large = concentrated (degenerate, like coldhotmix, clean split).
// The interesting regime is s ~ 0.6-1.2 (typical cache/web Zipf).
//
// Standalone (uses plain malloc/free); run directly for the system-malloc
// baseline or under LD_PRELOAD=libsmash.so. RSS is measured out-of-process
// (forked monitor) to avoid allocator interference. Deterministic PRNG.

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <chrono>
#include <thread>
#include <vector>
#include <algorithm>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/wait.h>

#if defined(__APPLE__)
#include <libproc.h>
#elif defined(__linux__)
#include <fstream>
#include <string>
#endif

// ── Compressor trigger (no-op under system malloc) ──────────────────────────

static void triggerCompressorStart() {
    for (int i = 0; i < 2; ++i) {
        std::thread helper([] {
            volatile void* p = malloc(64);
            free(const_cast<void*>(p));
        });
        helper.join();
    }
}

// ── Configuration ───────────────────────────────────────────────────────────

static int    g_count      = 500000;
static size_t g_size       = 512;
static double g_free_pct   = 30.0;
static double g_zipf_s     = 0.9;    // Zipf skew exponent (0 = uniform)
static double g_comp       = 0.8;    // compressibility (gradient pattern)
static int    g_cool_sec   = 10;     // idle cooling before re-access begins
static int    g_access_sec = 30;     // continuous Zipf re-access duration
static long   g_ops_per_sec_cap = 0; // 0 = unlimited; else throttle accesses/s
static double g_slow_us    = 2.0;    // latency above this = "slow" (≈ decompress fault)
static double g_write_pct  = 100.0;  // % of re-accesses that WRITE (rest read-only)
static const char* g_out_dir = nullptr;
static const char* g_label   = nullptr;

static void printUsage(const char* prog) {
    fprintf(stderr,
        "Usage: %s [options]\n\n"
        "Zipfian re-access microbenchmark for the compression cold-age gate.\n"
        "Each survivor gets a HIDDEN Zipf-distributed access rate; the access\n"
        "phase samples objects in proportion to those rates, continuously, so\n"
        "cold-age predicts re-access probability (the regime where a Lindy /\n"
        "Bayesian gate can beat the blunt backoff). RSS measured out-of-process.\n\n"
        "Workload:\n"
        "  --count N        objects to allocate                  [default: 500000]\n"
        "  --size S         object size in bytes                 [default: 512]\n"
        "  --free-pct F     %% freed randomly (fragmentation)      [default: 30]\n"
        "  --zipf-s X       Zipf skew exponent; 0=uniform         [default: 0.9]\n"
        "                     0.0   uniform   (all pages thrash, like sqlite)\n"
        "                     0.6-1.2 typical cache/web skew (the useful regime)\n"
        "                     >2.0  concentrated (clean hot/cold, like coldhotmix)\n"
        "  --comp C         compressibility 0..1 (gradient)      [default: 0.8]\n"
        "  --ops-cap N      throttle to N accesses/sec (0=full)  [default: 0]\n"
        "  --slow-us U      latency > U us counts as a slow/fault [default: 2.0]\n"
        "  --write-pct W    %% of re-accesses that WRITE           [default: 100]\n"
        "                     <100 creates read-hot/write-cold pages: soft-dirty\n"
        "                     (write-only) sees them clean; access tracking does not\n\n"
        "Timing:\n"
        "  --cool-sec C     idle cooling before re-access         [default: 10]\n"
        "  --access-sec A   continuous Zipf re-access duration    [default: 30]\n\n"
        "Output:\n"
        "  --out-dir DIR    write <label>_{rss,reaccess}.csv\n"
        "  --label NAME     CSV filename prefix                   [default: zipf]\n"
        "  --quick          --count 200000 --cool-sec 5 --access-sec 15\n\n"
        "stderr: progress+summary. stdout: machine-readable METRIC lines.\n",
        prog);
}

// ── Out-of-process RSS monitor (forked child reads /proc) ───────────────────

static int g_cmd_pipe[2] = {-1, -1};
static int g_rss_pipe[2] = {-1, -1};

// A failed or short transfer on the monitor's pipes ends the run: the parent
// used to record an RSS of 0 for a sample whose read failed.
static void pipeWriteAll(int fd, const void* p, size_t n) {
    ssize_t r;
    do r = write(fd, p, n); while (r < 0 && errno == EINTR);
    if (r != static_cast<ssize_t>(n)) {
        fprintf(stderr, "RSS monitor pipe: wrote %zd of %zu bytes (%s)\n", r, n, r < 0 ? strerror(errno) : "short");
        _exit(1);
    }
}
static void pipeReadAll(int fd, void* p, size_t n) {
    ssize_t r;
    do r = read(fd, p, n); while (r < 0 && errno == EINTR);
    if (r != static_cast<ssize_t>(n)) {
        fprintf(stderr, "RSS monitor pipe: read %zd of %zu bytes (%s)\n", r, n, r < 0 ? strerror(errno) : "short");
        _exit(1);
    }
}

static size_t getRSSBytesForPid(pid_t pid) {
#if defined(__APPLE__)
    struct rusage_info_v4 ri;
    if (proc_pid_rusage(pid, RUSAGE_INFO_V4, (rusage_info_t*)&ri) == 0)
        return static_cast<size_t>(ri.ri_phys_footprint);
    struct proc_taskinfo pti;
    if (proc_pidinfo(pid, PROC_PIDTASKINFO, 0, &pti, sizeof(pti)) ==
        static_cast<int>(sizeof(pti)))
        return pti.pti_resident_size;
    return 0;
#elif defined(__linux__)
    char path[64];
    snprintf(path, sizeof(path), "/proc/%d/status", static_cast<int>(pid));
    FILE* f = fopen(path, "r");
    if (!f) return 0;
    size_t rss = 0;
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        if (strncmp(line, "VmRSS:", 6) == 0) {
            size_t kb = 0;
            sscanf(line, "VmRSS: %zu kB", &kb);
            rss = kb * 1024;
            break;
        }
    }
    fclose(f);
    return rss;
#else
    (void)pid;
    return 0;
#endif
}

[[noreturn]] static void rssMonitorChild(pid_t target_pid) {
    close(g_cmd_pipe[1]);
    close(g_rss_pipe[0]);
    for (;;) {
        char cmd = 0;
        ssize_t n = read(g_cmd_pipe[0], &cmd, 1);
        if (n <= 0 || cmd == 'Q') _exit(0);
        size_t rss = getRSSBytesForPid(target_pid);
        pipeWriteAll(g_rss_pipe[1], &rss, sizeof(rss));
    }
}

static void startRssMonitor() {
    if (pipe(g_cmd_pipe) != 0 || pipe(g_rss_pipe) != 0) { perror("pipe"); _exit(1); }
    pid_t parent = getpid();
    pid_t child = fork();
    if (child < 0) { perror("fork"); _exit(1); }
    if (child == 0) rssMonitorChild(parent);
    close(g_cmd_pipe[0]);
    close(g_rss_pipe[1]);
}

static size_t requestRSS() {
    char cmd = 'M';
    pipeWriteAll(g_cmd_pipe[1], &cmd, 1);
    size_t rss = 0;
    pipeReadAll(g_rss_pipe[0], &rss, sizeof(rss));
    return rss;
}

static void stopRssMonitor() {
    char cmd = 'Q';
    (void)!write(g_cmd_pipe[1], &cmd, 1);   // the child may be gone already
    close(g_cmd_pipe[1]);
    close(g_rss_pipe[0]);
    wait(nullptr);
}

// ── mmap-backed measurement storage (no managed-heap use) ───────────────────

struct RssSample { double time_sec; double rss_mb; };
static constexpr size_t kMaxRssSamples = 1024;
static RssSample* g_rss_timeline = nullptr;
static size_t g_rss_count = 0;
static std::chrono::steady_clock::time_point g_t0;

static void* mmapZeroed(size_t bytes) {
    void* p = mmap(nullptr, bytes, PROT_READ | PROT_WRITE,
                   MAP_PRIVATE | MAP_ANON, -1, 0);
    if (p == MAP_FAILED) { perror("mmap"); _exit(1); }
    return p;
}

static double nowSec() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now() - g_t0).count();
}

static size_t recordAndGetRss() {
    size_t rss = requestRSS();
    if (g_rss_count < kMaxRssSamples)
        g_rss_timeline[g_rss_count++] = {nowSec(), rss / (1024.0 * 1024.0)};
    return rss;
}

// Re-access latency samples (one per sampled access, subsampled).
static constexpr size_t kMaxLat = 4000000;
static double* g_lat = nullptr;
static size_t  g_lat_count = 0;

// ── Deterministic PRNG ──────────────────────────────────────────────────────

static uint64_t s_rng[4] = {0xA5A5A5A5ULL, 0x5A5A5A5AULL, 0x13579BDFULL, 0xFDB97531ULL};
static inline uint64_t rotl(uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }
static uint64_t rng_next() {
    uint64_t result = rotl(s_rng[1] * 5, 7) * 9;
    uint64_t t = s_rng[1] << 17;
    s_rng[2] ^= s_rng[0]; s_rng[3] ^= s_rng[1];
    s_rng[1] ^= s_rng[2]; s_rng[0] ^= s_rng[3];
    s_rng[2] ^= t; s_rng[3] = rotl(s_rng[3], 45);
    return result;
}
static uint64_t rng_range(uint64_t max) { return max ? rng_next() % max : 0; }
static double rng_unit() { return (rng_next() >> 11) * (1.0 / 9007199254740992.0); }

static uint64_t splitmix64(uint64_t& state) {
    state += 0x9E3779B97F4A7C15ULL;
    uint64_t z = state;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

// ── Compressible object fill (8-band gradient, tuned by --comp) ─────────────

static void fillObject(void* ptr, size_t size, uint64_t seed) {
    auto* dst = static_cast<uint8_t*>(ptr);
    uint64_t rstate = seed ^ 0xDEADBEEFCAFEULL;
    uint8_t pat[4] = { (uint8_t)seed, (uint8_t)(seed >> 8),
                       (uint8_t)(seed >> 16), (uint8_t)(seed >> 24) };
    uint8_t konst = static_cast<uint8_t>(seed * 2654435761ULL >> 24);
    static constexpr double kBaseProb[8] = {1.0,1.0,1.0,0.88,0.75,0.50,0.25,0.0};
    double shift = 2.0 * (g_comp - 0.5);
    uint8_t thresh[8];
    for (int b = 0; b < 8; b++) {
        double prob = kBaseProb[b] + shift;
        if (prob < 0.0) prob = 0.0; if (prob > 1.0) prob = 1.0;
        thresh[b] = static_cast<uint8_t>(prob * 255.0);
    }
    size_t band_size = size / 8; if (band_size == 0) band_size = 1;
    for (size_t i = 0; i < size; i++) {
        int band = static_cast<int>(i / band_size); if (band > 7) band = 7;
        uint8_t r = static_cast<uint8_t>(splitmix64(rstate));
        if (r >= thresh[band]) dst[i] = static_cast<uint8_t>(splitmix64(rstate));
        else if (band == 0)    dst[i] = 0;
        else if (band == 1)    dst[i] = konst;
        else                   dst[i] = pat[i & 3];
    }
}

// ── Walker's alias method: O(1) sampling from a discrete weight vector ───────
//
// Lets us draw an object index in proportion to its hidden Zipf weight in O(1),
// so the continuous re-access loop sustains tens of millions of samples/sec.

struct AliasTable {
    size_t n = 0;
    double* prob = nullptr;   // mmap'd
    uint32_t* alias = nullptr;
};

static AliasTable buildAlias(const double* weights, size_t n) {
    AliasTable t;
    t.n = n;
    t.prob  = static_cast<double*>(mmapZeroed(n * sizeof(double)));
    t.alias = static_cast<uint32_t*>(mmapZeroed(n * sizeof(uint32_t)));

    double sum = 0.0;
    for (size_t i = 0; i < n; i++) sum += weights[i];

    // Scaled probabilities (mean 1). small/large worklists via mmap scratch.
    double* scaled = static_cast<double*>(mmapZeroed(n * sizeof(double)));
    uint32_t* small = static_cast<uint32_t*>(mmapZeroed(n * sizeof(uint32_t)));
    uint32_t* large = static_cast<uint32_t*>(mmapZeroed(n * sizeof(uint32_t)));
    size_t ns = 0, nl = 0;
    for (size_t i = 0; i < n; i++) {
        scaled[i] = weights[i] * static_cast<double>(n) / sum;
        if (scaled[i] < 1.0) small[ns++] = static_cast<uint32_t>(i);
        else                 large[nl++] = static_cast<uint32_t>(i);
    }
    while (ns > 0 && nl > 0) {
        uint32_t s = small[--ns];
        uint32_t l = large[--nl];
        t.prob[s]  = scaled[s];
        t.alias[s] = l;
        scaled[l] = (scaled[l] + scaled[s]) - 1.0;
        if (scaled[l] < 1.0) small[ns++] = l;
        else                 large[nl++] = l;
    }
    while (nl > 0) t.prob[large[--nl]] = 1.0;
    while (ns > 0) t.prob[small[--ns]] = 1.0;

    munmap(scaled, n * sizeof(double));
    munmap(small, n * sizeof(uint32_t));
    munmap(large, n * sizeof(uint32_t));
    return t;
}

static inline uint32_t aliasSample(const AliasTable& t) {
    uint32_t i = static_cast<uint32_t>(rng_range(t.n));
    return (rng_unit() < t.prob[i]) ? i : t.alias[i];
}

// ── CSV helpers ──────────────────────────────────────────────────────────────

static void makePath(char* buf, size_t bufsz, const char* suffix) {
    snprintf(buf, bufsz, "%s/%s_%s", g_out_dir, g_label ? g_label : "zipf", suffix);
}

static double pct(double* a, size_t n, double p) {
    if (n == 0) return 0;
    size_t idx = static_cast<size_t>(p * n);
    if (idx >= n) idx = n - 1;
    return a[idx];
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    for (int i = 1; i < argc; ++i) {
        if      (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) { printUsage(argv[0]); return 0; }
        else if (!strcmp(argv[i], "--quick")) { g_count = 200000; g_cool_sec = 5; g_access_sec = 15; }
        else if (!strcmp(argv[i], "--count")      && i+1 < argc) g_count = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--size")       && i+1 < argc) g_size = (size_t)atol(argv[++i]);
        else if (!strcmp(argv[i], "--free-pct")   && i+1 < argc) g_free_pct = atof(argv[++i]);
        else if (!strcmp(argv[i], "--zipf-s")     && i+1 < argc) g_zipf_s = atof(argv[++i]);
        else if (!strcmp(argv[i], "--comp")       && i+1 < argc) { g_comp = atof(argv[++i]); if (g_comp<0) g_comp=0; if (g_comp>1) g_comp=1; }
        else if (!strcmp(argv[i], "--ops-cap")    && i+1 < argc) g_ops_per_sec_cap = atol(argv[++i]);
        else if (!strcmp(argv[i], "--slow-us")    && i+1 < argc) g_slow_us = atof(argv[++i]);
        else if (!strcmp(argv[i], "--write-pct")  && i+1 < argc) { g_write_pct = atof(argv[++i]); if (g_write_pct<0) g_write_pct=0; if (g_write_pct>100) g_write_pct=100; }
        else if (!strcmp(argv[i], "--cool-sec")   && i+1 < argc) g_cool_sec = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--access-sec") && i+1 < argc) g_access_sec = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--out-dir")    && i+1 < argc) g_out_dir = argv[++i];
        else if (!strcmp(argv[i], "--label")      && i+1 < argc) g_label = argv[++i];
        else { fprintf(stderr, "Unknown option: %s\n", argv[i]); printUsage(argv[0]); return 1; }
    }

    g_rss_timeline = static_cast<RssSample*>(mmapZeroed(kMaxRssSamples * sizeof(RssSample)));
    g_lat = static_cast<double*>(mmapZeroed(kMaxLat * sizeof(double)));
    startRssMonitor();
    g_t0 = std::chrono::steady_clock::now();

    fprintf(stderr, "=== Zipfian Re-access Benchmark ===\n");
    fprintf(stderr, "Objects: %d x %zu B = %.1f MB logical; comp=%.0f%%\n",
            g_count, g_size, g_count * g_size / (1024.0*1024.0), g_comp*100.0);
    fprintf(stderr, "Free: %.0f%%, Zipf-s: %.2f, write: %.0f%%, cool: %ds, access: %ds, cap: %ld/s\n\n",
            g_free_pct, g_zipf_s, g_write_pct, g_cool_sec, g_access_sec, g_ops_per_sec_cap);

    // ── Phase 1: allocate + fill ─────────────────────────────────────────────
    fprintf(stderr, "Phase 1: allocating %d objects...\n", g_count);
    auto ta = std::chrono::steady_clock::now();
    std::vector<void*> objects(static_cast<size_t>(g_count));
    for (int i = 0; i < g_count; ++i) {
        objects[(size_t)i] = malloc(g_size);
        fillObject(objects[(size_t)i], g_size, (uint64_t)i);
    }
    double alloc_sec = std::chrono::duration<double>(std::chrono::steady_clock::now()-ta).count();
    size_t peak_rss = recordAndGetRss();
    printf("METRIC peak_rss_mb %.1f\n", peak_rss/(1024.0*1024.0));
    fprintf(stderr, "Allocated in %.2fs, peak RSS %.1f MB\n", alloc_sec, peak_rss/(1024.0*1024.0));

    // ── Phase 2: free random fraction ────────────────────────────────────────
    int num_free = static_cast<int>(g_count * g_free_pct / 100.0);
    fprintf(stderr, "Phase 2: freeing %d objects...\n", num_free);
    std::vector<int> idx(static_cast<size_t>(g_count));
    for (int i = 0; i < g_count; ++i) idx[(size_t)i] = i;
    for (int i = 0; i < num_free; ++i) {
        int j = i + static_cast<int>(rng_range((uint64_t)(g_count - i)));
        std::swap(idx[(size_t)i], idx[(size_t)j]);
    }
    for (int i = 0; i < num_free; ++i) { free(objects[(size_t)idx[(size_t)i]]); objects[(size_t)idx[(size_t)i]] = nullptr; }

    std::vector<void*> surv;
    surv.reserve(static_cast<size_t>(g_count - num_free));
    for (int i = 0; i < g_count; ++i) if (objects[(size_t)i]) surv.push_back(objects[(size_t)i]);
    objects.clear(); objects.shrink_to_fit();
    size_t N = surv.size();
    size_t post_free_rss = recordAndGetRss();
    printf("METRIC survivors %zu\n", N);
    fprintf(stderr, "Survivors: %zu, post-free RSS %.1f MB\n\n", N, post_free_rss/(1024.0*1024.0));

    // ── Phase 3: assign HIDDEN Zipf access rates, shuffled vs address order ──
    // weight(rank r) ∝ 1/(r+1)^s. We shuffle which survivor gets which rank so
    // hot/cold is NOT correlated with allocation order or address — the
    // allocator can't exploit spatial locality to "cheat" the cold-age signal.
    fprintf(stderr, "Phase 3: assigning Zipf weights (s=%.2f) over %zu survivors...\n", g_zipf_s, N);
    double* weights = static_cast<double*>(mmapZeroed(N * sizeof(double)));
    // Random rank permutation (Fisher-Yates) → rank_of[survivor_index].
    uint32_t* rank_of = static_cast<uint32_t*>(mmapZeroed(N * sizeof(uint32_t)));
    for (size_t i = 0; i < N; i++) rank_of[i] = static_cast<uint32_t>(i);
    for (size_t i = N; i > 1; --i) {
        size_t j = static_cast<size_t>(rng_range(i));
        std::swap(rank_of[i-1], rank_of[j]);
    }
    for (size_t i = 0; i < N; i++) {
        double r = static_cast<double>(rank_of[i]) + 1.0;
        weights[i] = (g_zipf_s == 0.0) ? 1.0 : std::pow(r, -g_zipf_s);
    }
    AliasTable alias = buildAlias(weights, N);

    triggerCompressorStart();

    // ── Phase 4: idle cooling ────────────────────────────────────────────────
    fprintf(stderr, "Phase 4: cooling %ds...\n", g_cool_sec);
    for (int s = 1; s <= g_cool_sec; ++s) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        size_t rss = recordAndGetRss();
        fprintf(stderr, "  t=%2ds: RSS=%.1f MB\n", s, rss/(1024.0*1024.0));
    }
    size_t post_cool_rss = requestRSS();
    double cool_red = peak_rss ? (1.0 - (double)post_cool_rss/(double)peak_rss)*100.0 : 0;
    printf("METRIC post_cool_rss_mb %.1f\n", post_cool_rss/(1024.0*1024.0));
    printf("METRIC cool_reduction_pct %.1f\n", cool_red);
    fprintf(stderr, "Post-cool RSS %.1f MB (%.1f%% reduction)\n\n", post_cool_rss/(1024.0*1024.0), cool_red);

    // ── Phase 5: continuous Zipfian re-access ────────────────────────────────
    // This runs WHILE the compressor keeps cooling/compressing in the
    // background. Hot objects are touched constantly (never compress); the cold
    // tail compresses (RSS win); the middle band is touched near the cold
    // timeout (thrash-prone). A slow access (> --slow-us) ≈ a decompression
    // fault = a thrash victim. We split latency by weight rank to show whether
    // the gate kept the warm middle resident while compressing the cold tail.
    fprintf(stderr, "Phase 5: Zipf re-access for %ds...\n", g_access_sec);
    size_t min_rss = requestRSS();
    long total_ops = 0, slow_ops = 0;
    // Per-decile-by-rank op + slow counts (decile 0 = hottest ranks).
    long band_ops[10] = {0};
    long band_slow[10] = {0};
    // Write coin threshold (out of 256). <256 leaves some touches read-only,
    // creating read-hot/write-cold pages that a write-only signal (soft-dirty)
    // can still treat as cold while access-based tracking cannot.
    const uint32_t write_thresh = static_cast<uint32_t>(g_write_pct * 256.0 / 100.0);

    for (int sec = 0; sec < g_access_sec; ++sec) {
        auto sec_start = std::chrono::steady_clock::now();
        long ops_this_sec = 0;
        long cap = g_ops_per_sec_cap;
        while (true) {
            uint32_t s_idx = aliasSample(alias);
            void* obj = surv[s_idx];
            auto o0 = std::chrono::steady_clock::now();
            volatile uint8_t* p = static_cast<volatile uint8_t*>(obj);
            uint8_t v = p[0];                      // always read
            if ((rng_next() & 0xFF) < write_thresh)
                p[g_size/2] = v + 1;               // write only --write-pct of the time
            auto o1 = std::chrono::steady_clock::now();
            double us = std::chrono::duration<double, std::micro>(o1 - o0).count();

            // rank decile: rank_of[s_idx] in [0,N) → decile
            int band = static_cast<int>((uint64_t)rank_of[s_idx] * 10 / N);
            if (band > 9) band = 9;
            band_ops[band]++;
            if (us > g_slow_us) { slow_ops++; band_slow[band]++; }
            if ((ops_this_sec & 0xF) == 0 && g_lat_count < kMaxLat)
                g_lat[g_lat_count++] = us;
            ops_this_sec++;

            if (cap > 0 && ops_this_sec >= cap) {
                // throttle: sleep out the rest of the second
                auto used = std::chrono::steady_clock::now() - sec_start;
                if (used < std::chrono::seconds(1))
                    std::this_thread::sleep_for(std::chrono::seconds(1) - used);
                break;
            }
            if (std::chrono::steady_clock::now() - sec_start >= std::chrono::seconds(1)) break;
        }
        total_ops += ops_this_sec;
        size_t rss = recordAndGetRss();
        if (rss < min_rss) min_rss = rss;
        fprintf(stderr, "  t=%2ds: RSS=%.1f MB  ops=%ld  slow=%ld\n",
                sec+1, rss/(1024.0*1024.0), ops_this_sec, slow_ops);
    }

    size_t steady_rss = requestRSS();
    double access_red = peak_rss ? (1.0 - (double)min_rss/(double)peak_rss)*100.0 : 0;
    double ops_per_sec = static_cast<double>(total_ops) / g_access_sec;
    double slow_pct = total_ops ? 100.0 * (double)slow_ops / (double)total_ops : 0;

    std::sort(g_lat, g_lat + g_lat_count);
    double p50  = pct(g_lat, g_lat_count, 0.50);
    double p99  = pct(g_lat, g_lat_count, 0.99);
    double p999 = pct(g_lat, g_lat_count, 0.999);

    printf("METRIC steady_rss_mb %.1f\n", steady_rss/(1024.0*1024.0));
    printf("METRIC min_access_rss_mb %.1f\n", min_rss/(1024.0*1024.0));
    printf("METRIC access_reduction_pct %.1f\n", access_red);
    printf("METRIC ops_per_sec %.0f\n", ops_per_sec);
    printf("METRIC reaccess_p50_us %.3f\n", p50);
    printf("METRIC reaccess_p99_us %.3f\n", p99);
    printf("METRIC reaccess_p999_us %.3f\n", p999);
    printf("METRIC slow_ops_pct %.3f\n", slow_pct);
    printf("METRIC slow_ops %ld\n", slow_ops);

    // ── Summary ──────────────────────────────────────────────────────────────
    fprintf(stderr, "\n=== Results (Zipf-s=%.2f) ===\n", g_zipf_s);
    fprintf(stderr, "  Peak RSS:       %.1f MB\n", peak_rss/(1024.0*1024.0));
    fprintf(stderr, "  Post-cool RSS:  %.1f MB (%.1f%% red)\n", post_cool_rss/(1024.0*1024.0), cool_red);
    fprintf(stderr, "  Min access RSS: %.1f MB (%.1f%% red)  <- cold-tail compression\n",
            min_rss/(1024.0*1024.0), access_red);
    fprintf(stderr, "  Ops/sec:        %.0f\n", ops_per_sec);
    fprintf(stderr, "  Re-access p50/p99/p99.9: %.2f / %.2f / %.2f us\n", p50, p99, p999);
    fprintf(stderr, "  Slow ops (>%.1fus, ~thrash faults): %ld (%.3f%%)\n", g_slow_us, slow_ops, slow_pct);
    fprintf(stderr, "  Slow-rate by rank decile (0=hottest .. 9=coldest):\n");
    for (int b = 0; b < 10; b++) {
        double br = band_ops[b] ? 100.0 * (double)band_slow[b] / (double)band_ops[b] : 0;
        fprintf(stderr, "    decile %d: ops=%ld slow=%ld (%.2f%%)\n", b, band_ops[b], band_slow[b], br);
    }

    if (g_out_dir) {
        char path[512];
        makePath(path, sizeof(path), "rss.csv");
        FILE* f = fopen(path, "w");
        if (f) { fprintf(f, "time_sec,rss_mb\n");
            for (size_t i = 0; i < g_rss_count; i++) fprintf(f, "%.2f,%.1f\n", g_rss_timeline[i].time_sec, g_rss_timeline[i].rss_mb);
            fclose(f); fprintf(stderr, "Wrote %s\n", path); }
        makePath(path, sizeof(path), "reaccess.csv");
        f = fopen(path, "w");
        if (f) { fprintf(f, "latency_us\n");
            for (size_t i = 0; i < g_lat_count; i++) fprintf(f, "%.3f\n", g_lat[i]);
            fclose(f); fprintf(stderr, "Wrote %s\n", path); }
    }

    for (void* p : surv) free(p);
    stopRssMonitor();
    return 0;
}
