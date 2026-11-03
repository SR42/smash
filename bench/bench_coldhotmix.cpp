// bench_coldhotmix.cpp - Configurable cold/hot memory microbenchmark
//
// Allocates N objects of configurable size, frees a random fraction, then
// accesses only the first K% of surviving objects for a configurable duration.
// Designed for A/B comparison: run directly (system malloc) or with
// DYLD_INSERT_LIBRARIES=libsmash.dylib.
//
// RSS is measured out-of-process: we fork a child that reads the parent's RSS
// via proc_pidinfo (macOS) or /proc/<pid>/status (Linux). This avoids any
// allocator interference from the measurement itself.

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
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

// ── Compressor trigger ──────────────────────────────────────────────────────

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
static size_t g_size       = 256;
static double g_free_pct   = 30.0;
static double g_hot_pct    = 10.0;
static double g_comp       = 0.5;    // compressibility: 0.0 = pure entropy, 1.0 = all zeros
static int    g_cool_sec   = 10;
static int    g_access_sec = 20;
static const char* g_out_dir = nullptr;  // CSV output directory (null = no dump)
static const char* g_label   = nullptr;  // label prefix for CSV filenames

// Data pattern: controls what kind of data fills each object
enum class DataPattern {
    GRADIENT,   // 8-band gradient (original, tuned by --comp)
    JSON,       // JSON-like key-value records (zstd ~2x better than LZ4)
    STRUCT,     // C struct-like records with varying numeric fields
    TEXT,       // Natural text-like data with repeated phrases
    BIASED,     // Non-uniform byte distribution (entropy coding advantage)
};
static DataPattern g_data = DataPattern::GRADIENT;

// ── Help ────────────────────────────────────────────────────────────────────

static void printUsage(const char* prog) {
    fprintf(stderr,
        "Usage: %s [options]\n"
        "\n"
        "Configurable cold/hot memory microbenchmark. Each object is filled with\n"
        "a mix of compressible patterns and entropy, controlled by --comp.\n"
        "Frees a random fraction, then accesses only the hottest K%% of\n"
        "survivors while the rest go cold. Reports RSS reduction and latency.\n"
        "\n"
        "RSS is measured out-of-process to avoid allocator interference.\n"
        "\n"
        "Workload parameters:\n"
        "  --count N        Number of objects to allocate           [default: 500000]\n"
        "  --size S         Object size in bytes                    [default: 256]\n"
        "  --free-pct F     Percentage of objects to free randomly  [default: 30]\n"
        "  --hot-pct K      Percentage of survivors to keep hot     [default: 10]\n"
        "  --data PATTERN   Data fill pattern                       [default: gradient]\n"
        "                     gradient  8-band gradient (tuned by --comp)\n"
        "                     json      JSON-like records (zstd ~2x better)\n"
        "                     struct    C struct records with varying fields\n"
        "                     text      Natural text with repeated phrases\n"
        "                     biased    Biased byte distribution (zstd entropy wins)\n"
        "  --comp C         Compressibility 0.0-1.0 (gradient only) [default: 0.5]\n"
        "                     1.0 = all zeros (perfectly compressible)\n"
        "                     0.5 = 8-band gradient (zeros..patterns..entropy)\n"
        "                     0.0 = pure PRNG entropy (incompressible)\n"
        "\n"
        "Timing parameters:\n"
        "  --cool-sec C     Cooling duration (seconds)              [default: 10]\n"
        "  --access-sec A   Hot-access duration (seconds)           [default: 20]\n"
        "\n"
        "Output parameters:\n"
        "  --out-dir DIR    Write CSV files to DIR:\n"
        "                     <label>_rss.csv      RSS timeline\n"
        "                     <label>_hot.csv      Hot-access latency samples\n"
        "                     <label>_cold.csv     Cold re-access latency samples\n"
        "  --label NAME     Prefix for CSV filenames                [default: coldhotmix]\n"
        "\n"
        "Presets:\n"
        "  --quick          Shorthand for --count 200000 --cool-sec 5 --access-sec 10\n"
        "\n"
        "Examples:\n"
        "  # Baseline (system malloc), dump CSVs\n"
        "  %s --count 1000000 --size 512 --out-dir results/ --label baseline\n"
        "\n"
        "  # With Smash, same workload\n"
        "  DYLD_INSERT_LIBRARIES=./libsmash.dylib \\\n"
        "    %s --count 1000000 --size 512 --out-dir results/ --label smash\n"
        "\n"
        "  # Quick smoke test\n"
        "  %s --quick --out-dir /tmp/bench\n"
        "\n"
        "Output:\n"
        "  Progress and summary are printed to stderr.\n"
        "  Machine-readable METRIC lines are printed to stdout.\n",
        prog, prog, prog, prog);
}

// ── Out-of-process RSS monitor ──────────────────────────────────────────────
//
// The parent sends a 1-byte command over cmd_pipe; the child reads the target
// pid's RSS without touching the managed heap, then writes the result (size_t)
// back over rss_pipe. A command byte of 'Q' tells the child to exit.
// The child's entire working set is stack + pipe I/O — no malloc needed.

static int g_cmd_pipe[2]  = {-1, -1};  // parent writes, child reads
static int g_rss_pipe[2]  = {-1, -1};  // child writes, parent reads

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
    // Use phys_footprint (excludes MADV_FREE reusable pages) for accurate
    // measurement of actual memory pressure. Falls back to resident_size.
    struct rusage_info_v4 ri;
    if (proc_pid_rusage(pid, RUSAGE_INFO_V4, (rusage_info_t*)&ri) == 0)
        return static_cast<size_t>(ri.ri_phys_footprint);
    struct proc_taskinfo pti;
    int ret = proc_pidinfo(pid, PROC_PIDTASKINFO, 0, &pti, sizeof(pti));
    if (ret == static_cast<int>(sizeof(pti)))
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
        if (n <= 0 || cmd == 'Q')
            _exit(0);

        size_t rss = getRSSBytesForPid(target_pid);
        pipeWriteAll(g_rss_pipe[1], &rss, sizeof(rss));
    }
}

static void startRssMonitor() {
    if (pipe(g_cmd_pipe) != 0 || pipe(g_rss_pipe) != 0) {
        perror("pipe");
        _exit(1);
    }

    pid_t parent = getpid();
    pid_t child = fork();
    if (child < 0) {
        perror("fork");
        _exit(1);
    }
    if (child == 0) {
        rssMonitorChild(parent);
    }

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

// ── RSS timeline (pre-allocated via mmap to avoid heap use) ─────────────────

struct RssSample { double time_sec; double rss_mb; };

static constexpr size_t kMaxRssSamples = 1024;
static RssSample* g_rss_timeline = nullptr;
static size_t g_rss_count = 0;
static std::chrono::steady_clock::time_point g_t0;

static void initRssTimeline() {
    void* p = mmap(nullptr, kMaxRssSamples * sizeof(RssSample),
                   PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANON, -1, 0);
    if (p == MAP_FAILED) {
        perror("mmap");
        _exit(1);
    }
    g_rss_timeline = static_cast<RssSample*>(p);
}

static size_t recordAndGetRss() {
    if (g_rss_count >= kMaxRssSamples) return requestRSS();
    double t = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - g_t0).count();
    size_t rss = requestRSS();
    g_rss_timeline[g_rss_count++] = {t, rss / (1024.0 * 1024.0)};
    return rss;
}

static void dumpRssTimeline(const char* path) {
    FILE* f = fopen(path, "w");
    if (!f) { perror(path); return; }
    fprintf(f, "time_sec,rss_mb\n");
    for (size_t i = 0; i < g_rss_count; ++i)
        fprintf(f, "%.2f,%.1f\n", g_rss_timeline[i].time_sec, g_rss_timeline[i].rss_mb);
    fclose(f);
    fprintf(stderr, "Wrote %s (%zu samples)\n", path, g_rss_count);
}

// ── Latency storage (pre-allocated via mmap) ────────────────────────────────

static constexpr size_t kMaxHotLatencies  = 2000000;
static constexpr size_t kMaxColdLatencies = 500000;

static double* g_hot_latencies  = nullptr;
static double* g_cold_latencies = nullptr;
static size_t  g_hot_lat_count  = 0;
static size_t  g_cold_lat_count = 0;

static double* mmapDoubleArray(size_t count) {
    void* p = mmap(nullptr, count * sizeof(double),
                   PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANON, -1, 0);
    if (p == MAP_FAILED) {
        perror("mmap");
        _exit(1);
    }
    return static_cast<double*>(p);
}

static void dumpLatencies(const char* path, const double* data, size_t count) {
    FILE* f = fopen(path, "w");
    if (!f) { perror(path); return; }
    fprintf(f, "latency_us\n");
    for (size_t i = 0; i < count; ++i)
        fprintf(f, "%.3f\n", data[i]);
    fclose(f);
    fprintf(stderr, "Wrote %s (%zu samples)\n", path, count);
}

// ── Deterministic PRNG (xoshiro256**) ───────────────────────────────────────

static uint64_t s_rng[4] = {0xA5A5A5A5ULL, 0x5A5A5A5AULL, 0x13579BDFULL, 0xFDB97531ULL};

static inline uint64_t rotl(uint64_t x, int k) {
    return (x << k) | (x >> (64 - k));
}

static uint64_t rng_next() {
    uint64_t result = rotl(s_rng[1] * 5, 7) * 9;
    uint64_t t = s_rng[1] << 17;
    s_rng[2] ^= s_rng[0];
    s_rng[3] ^= s_rng[1];
    s_rng[1] ^= s_rng[2];
    s_rng[0] ^= s_rng[3];
    s_rng[2] ^= t;
    s_rng[3] = rotl(s_rng[3], 45);
    return result;
}

static uint64_t rng_range(uint64_t max) {
    return rng_next() % max;
}

// ── Fill each object with tunable compressibility ────────────────────────────
//
// Each object is divided into 8 equal bands whose entropy increases from
// band 0 (perfectly compressible) to band 7 (pure PRNG).  The --comp
// parameter (g_comp, 0.0–1.0) shifts the entire gradient:
//
//   g_comp = 1.0  →  all bytes are zeros (perfect compression)
//   g_comp = 0.5  →  full 8-band gradient (zeros → patterns → entropy)
//   g_comp = 0.0  →  all bytes are PRNG (incompressible)
//
// Implementation: each band b ∈ [0,7] has a "pattern probability" that
// determines how likely each byte is to be a compressible pattern vs PRNG.
// At g_comp = 0.5 (default) these probabilities are:
//
//   band 0: 100%  (all zeros)
//   band 1: 100%  (repeating constant)
//   band 2: 100%  (repeating 4-byte word)
//   band 3:  88%  (4-byte word + 12% noise)
//   band 4:  75%  (4-byte word + 25% noise)
//   band 5:  50%  (4-byte word + 50% noise)
//   band 6:  25%  (mostly noise, some pattern)
//   band 7:   0%  (pure entropy)
//
// g_comp > 0.5 shifts all probabilities toward 1.0 (more compressible).
// g_comp < 0.5 shifts all probabilities toward 0.0 (more random).
// The mapping is linear: actual_prob = clamp(base_prob + 2*(g_comp - 0.5)).
//
// A per-object splitmix64 PRNG (seeded from the object index) keeps fills
// deterministic and independent of the allocation-order PRNG.

static uint64_t splitmix64(uint64_t& state) {
    state += 0x9E3779B97F4A7C15ULL;
    uint64_t z = state;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

static void fillObject(void* ptr, size_t size, uint64_t seed) {
    auto* dst = static_cast<uint8_t*>(ptr);
    uint64_t rstate = seed ^ 0xDEADBEEFCAFEULL;

    uint8_t pat[4] = {
        static_cast<uint8_t>(seed),
        static_cast<uint8_t>(seed >> 8),
        static_cast<uint8_t>(seed >> 16),
        static_cast<uint8_t>(seed >> 24),
    };
    uint8_t konst = static_cast<uint8_t>(seed * 2654435761ULL >> 24);

    // Base pattern probability per band (at g_comp = 0.5)
    static constexpr double kBaseProb[8] = {
        1.00, 1.00, 1.00, 0.88, 0.75, 0.50, 0.25, 0.00
    };

    // Shift factor: +1.0 at g_comp=1.0, 0.0 at g_comp=0.5, -1.0 at g_comp=0.0
    double shift = 2.0 * (g_comp - 0.5);

    // Compute per-band PRNG threshold (0–255).  Bytes with PRNG < threshold
    // use the compressible pattern; bytes >= threshold get PRNG output.
    uint8_t thresh[8];
    for (int b = 0; b < 8; b++) {
        double prob = kBaseProb[b] + shift;
        if (prob < 0.0) prob = 0.0;
        if (prob > 1.0) prob = 1.0;
        thresh[b] = static_cast<uint8_t>(prob * 255.0);
    }

    constexpr int kBands = 8;
    size_t band_size = size / kBands;
    if (band_size == 0) band_size = 1;

    for (size_t i = 0; i < size; i++) {
        int band = static_cast<int>(i / band_size);
        if (band >= kBands) band = kBands - 1;

        uint8_t r = static_cast<uint8_t>(splitmix64(rstate));

        if (r >= thresh[band]) {
            // Entropy byte
            dst[i] = static_cast<uint8_t>(splitmix64(rstate));
        } else {
            // Compressible byte — pattern depends on band for variety
            switch (band) {
            case 0:  dst[i] = 0;            break;  // zeros
            case 1:  dst[i] = konst;        break;  // constant
            default: dst[i] = pat[i & 3];   break;  // repeating word
            }
        }
    }
}

// ── Fill with JSON-like data ──────────────────────────────────────────────────
//
// Generates data resembling JSON records:
//   {"id":12345,"name":"user_XXXX","email":"user_XXXX@example.com","score":42,...}
//
// The repeated key names and structural bytes ({, ", :, ,) create statistical
// patterns that zstd's entropy coder captures but LZ4's literal-copy cannot.
// LZ4 can match repeated substrings but wastes bits on the non-uniform byte
// distribution of the literals. zstd typically achieves ~2x better ratio.

static void fillObjectJson(void* ptr, size_t size, uint64_t seed) {
    auto* dst = static_cast<char*>(ptr);
    uint64_t rstate = seed ^ 0xCAFEBABE12345678ULL;

    // Key templates (repeated structure is what zstd exploits)
    static const char* keys[] = {
        "\"id\":", "\"name\":\"", "\"email\":\"", "\"score\":",
        "\"active\":", "\"dept\":\"", "\"level\":", "\"ts\":"
    };
    static const char* depts[] = {
        "engineering", "marketing", "sales", "support", "research"
    };

    size_t pos = 0;
    auto put = [&](char c) { if (pos < size) dst[pos++] = c; };
    auto puts = [&](const char* s) { while (*s && pos < size) dst[pos++] = *s++; };
    auto putn = [&](uint64_t n) {
        char buf[20]; int len = snprintf(buf, sizeof(buf), "%llu", (unsigned long long)n);
        for (int i = 0; i < len && pos < size; i++) dst[pos++] = buf[i];
    };

    // Fill with repeated JSON records until buffer is full
    while (pos < size) {
        uint64_t id = splitmix64(rstate) % 999999;
        uint64_t score = splitmix64(rstate) % 100;
        uint64_t dept_idx = splitmix64(rstate) % 5;
        uint64_t level = splitmix64(rstate) % 10;
        uint64_t ts = 1700000000ULL + (splitmix64(rstate) % 10000000);
        uint64_t user_num = splitmix64(rstate) % 9999;

        put('{');
        puts(keys[0]); putn(id); put(',');
        puts(keys[1]); puts("user_"); putn(user_num); puts("\",");
        puts(keys[2]); puts("user_"); putn(user_num); puts("@example.com\",");
        puts(keys[3]); putn(score); put(',');
        puts(keys[4]); puts((id & 1) ? "true" : "false"); put(',');
        puts(keys[5]); puts(depts[dept_idx]); puts("\",");
        puts(keys[6]); putn(level); put(',');
        puts(keys[7]); putn(ts);
        put('}'); put('\n');
    }
}

// ── Fill with struct-like data ───────────────────────────────────────────────
//
// Simulates C struct records: fixed-size fields with some constant bytes
// (type tags, padding, alignment) and some varying numeric values.
// The alignment padding (zeros) and repeated type tags create patterns
// that zstd's entropy coder handles better than LZ4.

static void fillObjectStruct(void* ptr, size_t size, uint64_t seed) {
    auto* dst = static_cast<uint8_t*>(ptr);
    uint64_t rstate = seed ^ 0xFEEDFACE00000000ULL;

    // 32-byte "struct": 4B type tag, 4B padding(0), 8B id, 8B value, 4B flags, 4B padding(0)
    struct __attribute__((packed)) Record {
        uint32_t type;
        uint32_t pad0;
        uint64_t id;
        uint64_t value;
        uint32_t flags;
        uint32_t pad1;
    };

    static_assert(sizeof(Record) == 32, "Record must be 32 bytes");

    size_t num_records = size / sizeof(Record);
    auto* records = reinterpret_cast<Record*>(dst);

    for (size_t i = 0; i < num_records; i++) {
        records[i].type  = 0x52454301 + static_cast<uint32_t>(splitmix64(rstate) % 4); // "REC\x01" - "REC\x04"
        records[i].pad0  = 0;
        records[i].id    = splitmix64(rstate);
        records[i].value = splitmix64(rstate);
        records[i].flags = static_cast<uint32_t>(splitmix64(rstate) & 0xFF); // sparse flags
        records[i].pad1  = 0;
    }

    // Fill remainder with zeros (alignment padding)
    for (size_t i = num_records * sizeof(Record); i < size; i++)
        dst[i] = 0;
}

// ── Fill with text-like data ─────────────────────────────────────────────────
//
// Generates text with repeated phrases and varying content, resembling
// log lines or natural text. The non-uniform character distribution
// (mostly lowercase ASCII + spaces + digits) is where zstd's entropy
// coder dominates LZ4's byte-level matching.

static void fillObjectText(void* ptr, size_t size, uint64_t seed) {
    auto* dst = static_cast<char*>(ptr);
    uint64_t rstate = seed ^ 0xAAAABBBBCCCCDDDDULL;

    static const char* phrases[] = {
        "the quick brown fox jumps over the lazy dog ",
        "processing request from client ",
        "connection established to server ",
        "cache hit for key ",
        "updating record in database table ",
        "scheduled task completed successfully ",
        "received response with status code ",
        "allocated memory block of size ",
        "compressing data segment number ",
        "user session started at timestamp ",
    };
    static constexpr int kNumPhrases = 10;

    size_t pos = 0;
    while (pos < size) {
        // Pick a phrase
        int phrase_idx = static_cast<int>(splitmix64(rstate) % kNumPhrases);
        const char* phrase = phrases[phrase_idx];

        // Write phrase
        while (*phrase && pos < size)
            dst[pos++] = *phrase++;

        // Append a varying number (like a log line parameter)
        uint64_t num = splitmix64(rstate) % 99999;
        char buf[8];
        int len = snprintf(buf, sizeof(buf), "%llu", (unsigned long long)num);
        for (int i = 0; i < len && pos < size; i++)
            dst[pos++] = buf[i];

        // Newline
        if (pos < size) dst[pos++] = '\n';
    }
}

// ── Fill with biased byte distribution ────────────────────────────────────────
//
// Each byte is drawn from a non-uniform distribution: ~70% of bytes come from
// a narrow range [0x40-0x6F] (like ASCII uppercase/lowercase), ~20% from
// [0x20-0x3F] (like digits/punctuation), ~10% uniform random.
//
// This creates high entropy that LZ4's literal-copy model cannot compress
// (no long repeated byte sequences), but zstd's FSE (finite state entropy)
// coder captures the biased byte frequencies. LZ4 typically achieves ~1.0-1.2x
// while zstd achieves ~1.5-2.5x on this pattern.

static void fillObjectBiased(void* ptr, size_t size, uint64_t seed) {
    auto* dst = static_cast<uint8_t*>(ptr);
    uint64_t rstate = seed ^ 0x1234567890ABCDEFULL;

    for (size_t i = 0; i < size; i++) {
        uint64_t r = splitmix64(rstate);
        uint8_t selector = static_cast<uint8_t>(r >> 56);  // top byte for selection
        uint8_t value = static_cast<uint8_t>(r);            // low byte for value

        if (selector < 179) {
            // 70%: narrow range [0x40-0x6F] (48 values)
            dst[i] = 0x40 + (value % 48);
        } else if (selector < 230) {
            // 20%: medium range [0x20-0x3F] (32 values)
            dst[i] = 0x20 + (value % 32);
        } else {
            // 10%: full range [0x00-0xFF]
            dst[i] = value;
        }
    }
}

// ── Dispatch fill by data pattern ────────────────────────────────────────────

static void fillObjectDispatch(void* ptr, size_t size, uint64_t seed) {
    switch (g_data) {
    case DataPattern::GRADIENT: fillObject(ptr, size, seed); break;
    case DataPattern::JSON:     fillObjectJson(ptr, size, seed); break;
    case DataPattern::STRUCT:   fillObjectStruct(ptr, size, seed); break;
    case DataPattern::TEXT:     fillObjectText(ptr, size, seed); break;
    case DataPattern::BIASED:   fillObjectBiased(ptr, size, seed); break;
    }
}

// ── CSV output helper ───────────────────────────────────────────────────────

static void makePath(char* buf, size_t bufsz, const char* suffix) {
    const char* label = g_label ? g_label : "coldhotmix";
    snprintf(buf, bufsz, "%s/%s_%s", g_out_dir, label, suffix);
}

// ── Main ────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printUsage(argv[0]);
            return 0;
        } else if (strcmp(argv[i], "--quick") == 0) {
            g_count = 200000;
            g_cool_sec = 5;
            g_access_sec = 10;
        } else if (strcmp(argv[i], "--count") == 0 && i + 1 < argc) {
            g_count = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--size") == 0 && i + 1 < argc) {
            g_size = static_cast<size_t>(atol(argv[++i]));
        } else if (strcmp(argv[i], "--free-pct") == 0 && i + 1 < argc) {
            g_free_pct = atof(argv[++i]);
        } else if (strcmp(argv[i], "--hot-pct") == 0 && i + 1 < argc) {
            g_hot_pct = atof(argv[++i]);
        } else if (strcmp(argv[i], "--data") == 0 && i + 1 < argc) {
            ++i;
            if (strcmp(argv[i], "gradient") == 0) g_data = DataPattern::GRADIENT;
            else if (strcmp(argv[i], "json") == 0) g_data = DataPattern::JSON;
            else if (strcmp(argv[i], "struct") == 0) g_data = DataPattern::STRUCT;
            else if (strcmp(argv[i], "text") == 0) g_data = DataPattern::TEXT;
            else if (strcmp(argv[i], "biased") == 0) g_data = DataPattern::BIASED;
            else {
                fprintf(stderr, "Unknown data pattern: %s (use gradient/json/struct/text/biased)\n", argv[i]);
                return 1;
            }
        } else if (strcmp(argv[i], "--comp") == 0 && i + 1 < argc) {
            g_comp = atof(argv[++i]);
            if (g_comp < 0.0) g_comp = 0.0;
            if (g_comp > 1.0) g_comp = 1.0;
        } else if (strcmp(argv[i], "--cool-sec") == 0 && i + 1 < argc) {
            g_cool_sec = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--access-sec") == 0 && i + 1 < argc) {
            g_access_sec = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--out-dir") == 0 && i + 1 < argc) {
            g_out_dir = argv[++i];
        } else if (strcmp(argv[i], "--label") == 0 && i + 1 < argc) {
            g_label = argv[++i];
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            printUsage(argv[0]);
            return 1;
        }
    }

    // Pre-allocate all measurement storage via mmap (before any managed allocs)
    initRssTimeline();
    g_hot_latencies  = mmapDoubleArray(kMaxHotLatencies);
    g_cold_latencies = mmapDoubleArray(kMaxColdLatencies);

    // Fork the out-of-process RSS monitor (before smash starts)
    startRssMonitor();

    g_t0 = std::chrono::steady_clock::now();

    int num_to_free = static_cast<int>(g_count * g_free_pct / 100.0);

    const char* data_name = "gradient";
    switch (g_data) {
    case DataPattern::GRADIENT: data_name = "gradient"; break;
    case DataPattern::JSON:     data_name = "json"; break;
    case DataPattern::STRUCT:   data_name = "struct"; break;
    case DataPattern::TEXT:     data_name = "text"; break;
    case DataPattern::BIASED:   data_name = "biased"; break;
    }

    fprintf(stderr, "=== Cold/Hot Mix Benchmark ===\n");
    fprintf(stderr, "Objects: %d × %zu bytes = %.1f MB logical\n",
            g_count, g_size, g_count * g_size / (1024.0 * 1024.0));
    if (g_data == DataPattern::GRADIENT) {
        fprintf(stderr, "Data: gradient, compressibility=%.0f%%\n", g_comp * 100.0);
    } else {
        fprintf(stderr, "Data: %s (structured, zstd >> LZ4)\n", data_name);
    }
    fprintf(stderr, "Free: %.0f%% (%d objects), Hot: %.0f%% of survivors\n",
            g_free_pct, num_to_free, g_hot_pct);
    fprintf(stderr, "Cool: %ds, Access: %ds\n", g_cool_sec, g_access_sec);
    if (g_out_dir)
        fprintf(stderr, "CSV output: %s/%s_*.csv\n",
                g_out_dir, g_label ? g_label : "coldhotmix");
    fprintf(stderr, "\n");

    // ── Phase 1: Allocate ───────────────────────────────────────────────────

    fprintf(stderr, "Phase 1: Allocating %d objects...\n", g_count);
    auto t_alloc_start = std::chrono::steady_clock::now();

    std::vector<void*> objects(static_cast<size_t>(g_count));
    for (int i = 0; i < g_count; ++i) {
        objects[static_cast<size_t>(i)] = malloc(g_size);
        fillObjectDispatch(objects[static_cast<size_t>(i)], g_size, static_cast<uint64_t>(i));
    }

    auto t_alloc_end = std::chrono::steady_clock::now();
    double alloc_sec = std::chrono::duration<double>(t_alloc_end - t_alloc_start).count();

    size_t peak_rss = recordAndGetRss();
    fprintf(stderr, "Allocated in %.2fs, peak RSS: %.1f MB\n",
            alloc_sec, peak_rss / (1024.0 * 1024.0));

    printf("METRIC alloc_time_sec %.2f\n", alloc_sec);
    printf("METRIC peak_rss_mb %.1f\n", peak_rss / (1024.0 * 1024.0));

    // ── Phase 2: Free random subset ─────────────────────────────────────────

    fprintf(stderr, "Phase 2: Freeing %d objects (%.0f%%)...\n", num_to_free, g_free_pct);

    // Fisher-Yates partial shuffle to pick random indices to free
    std::vector<int> indices(static_cast<size_t>(g_count));
    for (int i = 0; i < g_count; ++i)
        indices[static_cast<size_t>(i)] = i;

    for (int i = 0; i < num_to_free; ++i) {
        int j = i + static_cast<int>(rng_range(static_cast<uint64_t>(g_count - i)));
        std::swap(indices[static_cast<size_t>(i)], indices[static_cast<size_t>(j)]);
    }

    for (int i = 0; i < num_to_free; ++i) {
        int idx = indices[static_cast<size_t>(i)];
        free(objects[static_cast<size_t>(idx)]);
        objects[static_cast<size_t>(idx)] = nullptr;
    }

    // Compact survivors into a contiguous vector
    std::vector<void*> survivors;
    survivors.reserve(static_cast<size_t>(g_count - num_to_free));
    for (int i = 0; i < g_count; ++i) {
        if (objects[static_cast<size_t>(i)] != nullptr)
            survivors.push_back(objects[static_cast<size_t>(i)]);
    }
    objects.clear();
    objects.shrink_to_fit();

    size_t post_free_rss = recordAndGetRss();
    int num_survivors = static_cast<int>(survivors.size());
    int hot_count = std::max(1, static_cast<int>(num_survivors * g_hot_pct / 100.0));

    fprintf(stderr, "Survivors: %d, hot set: %d (first %.0f%%)\n",
            num_survivors, hot_count, g_hot_pct);
    fprintf(stderr, "Post-free RSS: %.1f MB\n\n", post_free_rss / (1024.0 * 1024.0));

    printf("METRIC survivors %d\n", num_survivors);
    printf("METRIC post_free_rss_mb %.1f\n", post_free_rss / (1024.0 * 1024.0));

    // Start the Smash compressor (no-op under system malloc)
    triggerCompressorStart();

    // ── Phase 3: Cooling ────────────────────────────────────────────────────

    fprintf(stderr, "Phase 3: Cooling for %d seconds...\n", g_cool_sec);

    for (int sec = 1; sec <= g_cool_sec; ++sec) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        size_t rss = recordAndGetRss();
        fprintf(stderr, "  t=%2ds: RSS=%.1f MB\n", sec, rss / (1024.0 * 1024.0));
    }

    size_t post_cool_rss = requestRSS();
    double cool_reduction = (peak_rss > 0)
        ? (1.0 - static_cast<double>(post_cool_rss) / static_cast<double>(peak_rss)) * 100.0
        : 0.0;

    fprintf(stderr, "Post-cooling RSS: %.1f MB (%.1f%% reduction from peak)\n\n",
            post_cool_rss / (1024.0 * 1024.0), cool_reduction);

    printf("METRIC post_cool_rss_mb %.1f\n", post_cool_rss / (1024.0 * 1024.0));
    printf("METRIC cool_reduction_pct %.1f\n", cool_reduction);

    // ── Phase 4: Hot-only access ────────────────────────────────────────────

    fprintf(stderr, "Phase 4: Accessing hot set (%d objects) for %ds...\n",
            hot_count, g_access_sec);

    size_t min_access_rss = requestRSS();
    long total_ops = 0;

    for (int sec = 0; sec < g_access_sec; ++sec) {
        int ops_this_sec = 0;
        auto sec_start = std::chrono::steady_clock::now();

        while (true) {
            int idx = static_cast<int>(rng_range(static_cast<uint64_t>(hot_count)));
            void* obj = survivors[static_cast<size_t>(idx)];

            auto op_start = std::chrono::steady_clock::now();

            volatile uint8_t* p = static_cast<volatile uint8_t*>(obj);
            uint8_t v = p[0];
            p[g_size / 2] = v + 1;

            auto op_end = std::chrono::steady_clock::now();

            if ((ops_this_sec & 0xF) == 0 && g_hot_lat_count < kMaxHotLatencies) {
                g_hot_latencies[g_hot_lat_count++] =
                    std::chrono::duration<double, std::micro>(op_end - op_start).count();
            }
            ops_this_sec++;

            if (std::chrono::steady_clock::now() - sec_start >= std::chrono::seconds(1))
                break;
        }

        total_ops += ops_this_sec;
        size_t rss = recordAndGetRss();
        if (rss < min_access_rss) min_access_rss = rss;

        fprintf(stderr, "  t=%2ds: RSS=%.1f MB  ops=%d\n",
                sec + 1, rss / (1024.0 * 1024.0), ops_this_sec);
    }

    size_t steady_rss = requestRSS();
    double access_reduction = (peak_rss > 0)
        ? (1.0 - static_cast<double>(min_access_rss) / static_cast<double>(peak_rss)) * 100.0
        : 0.0;
    double ops_per_sec = static_cast<double>(total_ops) / g_access_sec;

    std::sort(g_hot_latencies, g_hot_latencies + g_hot_lat_count);
    double hot_p50 = g_hot_lat_count == 0 ? 0 : g_hot_latencies[g_hot_lat_count / 2];
    double hot_p99 = g_hot_lat_count == 0 ? 0 : g_hot_latencies[static_cast<size_t>(g_hot_lat_count * 0.99)];

    printf("METRIC steady_rss_mb %.1f\n", steady_rss / (1024.0 * 1024.0));
    printf("METRIC min_access_rss_mb %.1f\n", min_access_rss / (1024.0 * 1024.0));
    printf("METRIC access_reduction_pct %.1f\n", access_reduction);
    printf("METRIC ops_per_sec %.0f\n", ops_per_sec);
    printf("METRIC hot_p50_us %.2f\n", hot_p50);
    printf("METRIC hot_p99_us %.2f\n", hot_p99);

    // ── Phase 5: Cold re-access ─────────────────────────────────────────────

    int cold_start = hot_count;
    int cold_count = num_survivors - hot_count;
    int cold_sample = std::max(1, cold_count / 10);

    fprintf(stderr, "\nPhase 5: Cold re-access (%d of %d cold objects)...\n",
            cold_sample, cold_count);

    for (int i = 0; i < cold_sample && g_cold_lat_count < kMaxColdLatencies; ++i) {
        int idx = cold_start + static_cast<int>(rng_range(static_cast<uint64_t>(cold_count)));
        void* obj = survivors[static_cast<size_t>(idx)];

        auto t0 = std::chrono::steady_clock::now();
        volatile uint8_t v = static_cast<volatile uint8_t*>(obj)[0];
        (void)v;
        auto t1 = std::chrono::steady_clock::now();

        g_cold_latencies[g_cold_lat_count++] =
            std::chrono::duration<double, std::micro>(t1 - t0).count();
    }

    std::sort(g_cold_latencies, g_cold_latencies + g_cold_lat_count);
    double cold_p50 = g_cold_lat_count == 0 ? 0 : g_cold_latencies[g_cold_lat_count / 2];
    double cold_p99 = g_cold_lat_count == 0 ? 0 : g_cold_latencies[static_cast<size_t>(g_cold_lat_count * 0.99)];

    size_t post_reaccess_rss = recordAndGetRss();

    printf("METRIC cold_p50_us %.2f\n", cold_p50);
    printf("METRIC cold_p99_us %.2f\n", cold_p99);
    printf("METRIC post_reaccess_rss_mb %.1f\n", post_reaccess_rss / (1024.0 * 1024.0));

    // ── Dump CSVs ───────────────────────────────────────────────────────────

    if (g_out_dir) {
        char path[512];
        makePath(path, sizeof(path), "rss.csv");
        dumpRssTimeline(path);
        makePath(path, sizeof(path), "hot.csv");
        dumpLatencies(path, g_hot_latencies, g_hot_lat_count);
        makePath(path, sizeof(path), "cold.csv");
        dumpLatencies(path, g_cold_latencies, g_cold_lat_count);
    }

    // ── Summary ─────────────────────────────────────────────────────────────

    fprintf(stderr, "\n=== Results ===\n");
    fprintf(stderr, "  Objects: %d × %zu B, freed %.0f%%, hot %.0f%%\n",
            g_count, g_size, g_free_pct, g_hot_pct);
    fprintf(stderr, "  Peak RSS:        %.1f MB\n", peak_rss / (1024.0 * 1024.0));
    fprintf(stderr, "  Post-free RSS:   %.1f MB\n", post_free_rss / (1024.0 * 1024.0));
    fprintf(stderr, "  Post-cool RSS:   %.1f MB (%.1f%% reduction)\n",
            post_cool_rss / (1024.0 * 1024.0), cool_reduction);
    fprintf(stderr, "  Steady RSS:      %.1f MB\n", steady_rss / (1024.0 * 1024.0));
    fprintf(stderr, "  Min access RSS:  %.1f MB (%.1f%% reduction)\n",
            min_access_rss / (1024.0 * 1024.0), access_reduction);
    fprintf(stderr, "  Ops/sec:         %.0f\n", ops_per_sec);
    fprintf(stderr, "  Hot p50/p99:     %.2f / %.2f us\n", hot_p50, hot_p99);
    fprintf(stderr, "  Cold p50/p99:    %.1f / %.1f us\n", cold_p50, cold_p99);

    // Clean up
    for (void* p : survivors)
        free(p);

    stopRssMonitor();
    munmap(g_rss_timeline, kMaxRssSamples * sizeof(RssSample));
    munmap(g_hot_latencies, kMaxHotLatencies * sizeof(double));
    munmap(g_cold_latencies, kMaxColdLatencies * sizeof(double));

    return 0;
}
