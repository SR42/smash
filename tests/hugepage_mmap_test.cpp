// Application hugepage mappings under smash's mmap interposer.
//
// smash's Linux mmap interposer registers application-direct mappings so the
// compressor can see them (src/linux_syscall_wrappers.cpp, SMASH_TRACK_EXTERNAL=1).
// Its filter, isTrackableMapping(), takes anonymous writable mappings and skips
// MAP_HUGETLB. It did not always skip MAP_HUGETLB, and registerLinuxExternalRange
// walks a mapping in kPageSize (4 KiB) strides, so an application that allocated
//
//     mmap(NULL, n, PROT_READ|PROT_WRITE,
//          MAP_PRIVATE|MAP_ANONYMOUS|MAP_HUGETLB, -1, 0)
//
// had every 4 KiB slice of its huge mapping registered as a separate "external
// page" and marked ACTIVE. The kernel requires huge-page-aligned addresses and
// lengths for hugetlb VMAs, so every operation smash then performed failed:
//
//     mprotect(hugetlb + 4096, 4096, ...)              -> EINVAL
//     madvise(hugetlb + 4096, 4096, MADV_DONTNEED)     -> EINVAL
//     mmap(hugetlb + 4096, 4096, ..., MAP_FIXED|ANON)  -> ENOMEM
//
// (all three verified directly on a host with hugepages reserved).
//
// Three properties this test pins down:
//
//   1. Correctness — the application's data must survive. smash's restore
//      paths use remapPages(), which is MAP_FIXED|MAP_ANON over the target;
//      on a hugetlb VMA a partial replacement would shred the mapping.
//
//   2. No wasted work — nothing records that a page is unmanageable, so a
//      registered hugetlb slice is retried every compressor tick, forever.
//      Before the skip, one 16 MiB mapping produced 8192 failing page
//      operations in 6 s; now it must produce none.
//
//   3. No slot starvation — VmRegion's external-page budget is
//      kTrackMaxPages = 128 Ki pages (512 MiB at 4 KiB). Registered hugetlb
//      slices held that budget permanently while being uncompressible.
//
// Property 2 is a failure only under SMASH_TEST_STRICT_HUGEPAGE=1
// (test_hugepage_no_mprotect_storm, the regression guard for the skip); the
// default registration reports it as a WARN.
//
// Skips cleanly when the host has no hugepages reserved
// (/proc/meminfo HugePages_Free == 0), which is the case on CI runners —
// reserve some with `sysctl -w vm.nr_hugepages=N` to exercise it.

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

#if defined(__linux__)

namespace {

int failures = 0;

#define CHECK(cond, msg, ...) do { \
    if (!(cond)) { \
        fprintf(stderr, "FAIL: " msg "\n", ##__VA_ARGS__); \
        ++failures; \
    } \
} while (0)

constexpr size_t kHugeMappings = 8;
constexpr int kTicksToWait = 6;      // > SMASH_COLD_TIMEOUT_SEC, several ticks

bool strictMode() {
    const char* v = std::getenv("SMASH_TEST_STRICT_HUGEPAGE");
    return v && v[0] == '1';
}

long readMeminfo(const char* key) {
    FILE* f = fopen("/proc/meminfo", "r");
    if (!f) return -1;
    char line[256];
    long v = -1;
    size_t klen = std::strlen(key);
    while (fgets(line, sizeof(line), f)) {
        if (std::strncmp(line, key, klen) == 0) {
            v = std::strtol(line + klen, nullptr, 10);
            break;
        }
    }
    fclose(f);
    return v;
}

// smash logs every failed mprotect to stderr under SMASH_TRACE_MPROTECT_FAIL=1:
//   [smash mprotect-fail] addr=00007F.. size=4096 prot=1 errno=22
// Redirect fd 2 to a temp file for the observation window, then count the
// lines whose address falls inside one of our hugetlb mappings. Same fd-2
// redirect idiom as mode_compression_test.cpp's captureSigusr2Stats().
struct StderrCapture {
    int fd = -1;
    int saved = -1;

    bool begin() {
        char path[] = "/tmp/smash-hugepage-trace-XXXXXX";
        fd = mkstemp(path);
        if (fd < 0) return false;
        unlink(path);
        saved = dup(2);
        if (saved < 0) { close(fd); fd = -1; return false; }
        fflush(stderr);
        if (dup2(fd, 2) < 0) { close(saved); close(fd); fd = -1; return false; }
        return true;
    }

    std::string end() {
        fflush(stderr);
        fsync(fd);
        dup2(saved, 2);
        close(saved);
        lseek(fd, 0, SEEK_SET);
        std::string out;
        char buf[8192];
        for (;;) {
            ssize_t n = read(fd, buf, sizeof(buf));
            if (n <= 0) break;
            out.append(buf, static_cast<size_t>(n));
        }
        close(fd);
        return out;
    }
};

struct Range { uintptr_t lo, hi; };

// Count "[smash mprotect-fail] addr=<hex>" lines landing inside our mappings.
size_t countFailsInRanges(const std::string& log, const std::vector<Range>& rs) {
    size_t count = 0;
    const char* needle = "[smash mprotect-fail] addr=";
    size_t pos = 0;
    while ((pos = log.find(needle, pos)) != std::string::npos) {
        pos += std::strlen(needle);
        uintptr_t a = std::strtoull(log.c_str() + pos, nullptr, 16);
        for (const Range& r : rs) {
            if (a >= r.lo && a < r.hi) { ++count; break; }
        }
    }
    return count;
}

} // namespace

int main() {
    // Unbuffered stdout: full mode used to zero the tail of glibc's buffered
    // stdout page at exit (fixed; guarded by test_stdout_buffer_integrity_full),
    // silently eating a test's own verdict lines. Kept as hygiene.
    setvbuf(stdout, nullptr, _IONBF, 0);
    long huge_kb = readMeminfo("Hugepagesize:");
    long huge_free = readMeminfo("HugePages_Free:");
    if (huge_kb <= 0 || huge_free <= 0) {
        printf("SKIP: no hugepages reserved on this host "
               "(HugePages_Free=%ld). Reserve some with "
               "`sysctl -w vm.nr_hugepages=N` to exercise this test.\n",
               huge_free);
        return 77;
    }

    const size_t huge = static_cast<size_t>(huge_kb) * 1024;
    const size_t want = kHugeMappings;
    printf("hugepage_mmap_test: Hugepagesize=%zu KiB HugePages_Free=%ld "
           "strict=%d\n", huge / 1024, huge_free, strictMode() ? 1 : 0);

    if (static_cast<size_t>(huge_free) < want) {
        printf("SKIP: need %zu free hugepages, have %ld\n", want, huge_free);
        return 77;
    }

    // ── 1. The app's hugetlb mmap must still succeed under interposition ────
    // smash's wrapper calls the real mmap and only then registers the result,
    // so a MAP_HUGETLB request must come back untouched.
    std::vector<void*> maps;
    std::vector<Range> ranges;
    for (size_t i = 0; i < kHugeMappings; ++i) {
        void* p = mmap(nullptr, huge, PROT_READ | PROT_WRITE,
                       MAP_PRIVATE | MAP_ANONYMOUS | MAP_HUGETLB, -1, 0);
        if (p == MAP_FAILED) {
            fprintf(stderr,
                    "FAIL: mmap(MAP_HUGETLB, %zu) failed at mapping %zu "
                    "(errno=%d %s) — smash's mmap interposer must pass "
                    "hugepage requests through unchanged\n",
                    huge, i, errno, std::strerror(errno));
            ++failures;
            break;
        }
        CHECK((reinterpret_cast<uintptr_t>(p) & (huge - 1)) == 0,
              "hugetlb mapping %zu at %p is not huge-page aligned", i, p);
        maps.push_back(p);
        ranges.push_back({reinterpret_cast<uintptr_t>(p),
                          reinterpret_cast<uintptr_t>(p) + huge});
    }
    printf("mapped %zu hugetlb regions of %zu MiB each\n",
           maps.size(), huge / (1024 * 1024));

    if (maps.empty()) {
        printf("hugepage_mmap_test: %d FAILURES\n", failures);
        return 1;
    }

    // Fill each mapping with a distinct pattern.
    for (size_t i = 0; i < maps.size(); ++i)
        std::memset(maps[i], static_cast<int>(0x40 + i), huge);

    // Move the last mapping with mremap. mremap's flags cannot say MAP_HUGETLB,
    // so the interposer tracks a moved range only if its source was tracked;
    // without that, this move alone re-registered the destination and brought
    // the failing-operation storm straight back (4096 in 6 s for an 8 MiB move).
    // Same size to a huge-aligned destination: growing a hugetlb mapping is
    // EINVAL, and some kernels refuse the move too — then this is skipped.
    if (!maps.empty()) {
        void* resv = mmap(nullptr, 2 * huge, PROT_NONE,
                          MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
        if (resv != MAP_FAILED) {
            auto dst = (reinterpret_cast<uintptr_t>(resv) + huge - 1) & ~(uintptr_t{huge} - 1);
            void* moved = mremap(maps.back(), huge, huge, MREMAP_MAYMOVE | MREMAP_FIXED,
                                 reinterpret_cast<void*>(dst));
            if (moved != MAP_FAILED) {
                maps.back() = moved;
                ranges.back() = {reinterpret_cast<uintptr_t>(moved),
                                 reinterpret_cast<uintptr_t>(moved) + huge};
                printf("moved one hugetlb mapping with mremap\n");
            } else {
                printf("NOTE: kernel refused mremap of a hugetlb mapping (errno=%d); "
                       "the mremap path is not exercised\n", errno);
            }
        }
    }

    // Give the compressor something ordinary to work on too, so it is
    // genuinely running its phases rather than idling.
    void* ballast = std::malloc(32u << 20);
    CHECK(ballast != nullptr, "ballast malloc(32 MiB) returned nullptr");
    if (ballast) std::memset(ballast, 0x11, 32u << 20);

    // ── 2. Observe what smash does to those pages over several ticks ────────
    StderrCapture cap;
    bool capturing = cap.begin();
    for (int t = 0; t < kTicksToWait; ++t) sleep(1);
    std::string log = capturing ? cap.end() : std::string();

    // ── 3. The application's data must be byte-exact ────────────────────────
    // This is the assertion that matters most: smash's restore path is
    // remapPages() (MAP_FIXED|MAP_ANON), which over a hugetlb VMA would
    // replace app data with fresh zero pages.
    for (size_t i = 0; i < maps.size(); ++i) {
        auto* b = static_cast<const unsigned char*>(maps[i]);
        unsigned char want_b = static_cast<unsigned char>(0x40 + i);
        size_t bad = SIZE_MAX;
        for (size_t j = 0; j < huge; ++j) {
            if (b[j] != want_b) { bad = j; break; }
        }
        CHECK(bad == SIZE_MAX,
              "hugetlb mapping %zu corrupted at offset %zu: got 0x%02x want "
              "0x%02x — smash performed a destructive page operation on an "
              "application hugepage mapping",
              i, bad, bad == SIZE_MAX ? 0 : b[bad], want_b);
        if (bad != SIZE_MAX) break;
    }

    // ── 4. Allocation still works with the hugepages registered ─────────────
    // Hugetlb pages consume VmRegion's external slot budget
    // (kTrackMaxPages = 128 Ki pages = 512 MiB at 4 KiB) while being
    // permanently uncompressible. Allocation must not start failing.
    std::vector<void*> blocks;
    for (int i = 0; i < 2048; ++i) {
        void* p = std::malloc(64 * 1024);
        if (!p) {
            fprintf(stderr,
                    "FAIL: malloc(64 KiB) returned nullptr at iteration %d "
                    "with %zu hugetlb mappings registered — operator new "
                    "would throw std::bad_alloc here\n", i, maps.size());
            ++failures;
            break;
        }
        std::memset(p, 0x22, 64 * 1024);
        blocks.push_back(p);
    }
    printf("allocated %zu 64 KiB blocks with hugepages registered\n",
           blocks.size());
    for (void* p : blocks) std::free(p);

    // ── 5. The failing-page-operation storm ─────────────────────────────────
    // Only observable when SMASH_TRACE_MPROTECT_FAIL=1 is set (the ctest
    // registration sets it). smash has no record that a page is unmanageable,
    // so if these mappings were ever registered it would retry every tick and
    // the count would grow without bound — 8192 in 6 s before the mmap
    // interposer learned to skip MAP_HUGETLB.
    if (!capturing) {
        printf("NOTE: could not capture stderr; skipping the page-operation "
               "storm check\n");
    } else {
        size_t fails = countFailsInRanges(log, ranges);
        size_t pages = (maps.size() * huge) / 4096;
        printf("failing page operations on hugetlb pages over %d s: %zu "
               "(the mappings span %zu 4 KiB slices)\n", kTicksToWait, fails, pages);

        if (fails == 0) {
            printf("hugepage page operations: none attempted (mappings are "
                   "correctly excluded from external tracking)\n");
        } else if (strictMode()) {
            fprintf(stderr,
                    "FAIL: smash attempted %zu page operations on hugetlb "
                    "pages, all of which the kernel must reject (hugetlb VMAs "
                    "require huge-page-aligned addresses and lengths).\n"
                    "      The mmap interposer in src/linux_syscall_wrappers.cpp "
                    "(isTrackableMapping) must skip MAP_HUGETLB; this means "
                    "each 4 KiB slice of an application hugepage mapping was "
                    "registered as an external page again. Nothing "
                    "records the failure, so the compressor retries every tick "
                    "forever, and the slices permanently occupy VmRegion's "
                    "128 Ki-page external budget while being permanently "
                    "uncompressible.\n"
                    "      Fix: skip MAP_HUGETLB mappings in the mmap "
                    "interposer's filter.\n", fails);
            ++failures;
        } else {
            printf("WARN: smash attempted %zu page operations on hugetlb "
                   "pages, all necessarily rejected by the kernel. "
                   "The MAP_HUGETLB skip in the mmap interposer "
                   "(isTrackableMapping) has regressed; the retries repeat "
                   "every tick and the slices consume "
                   "VmRegion's external slot budget. Run with "
                   "SMASH_TEST_STRICT_HUGEPAGE=1 to make this a failure.\n",
                   fails);
        }
    }

    // ── 6. munmap deregisters cleanly ───────────────────────────────────────
    for (size_t i = 0; i < maps.size(); ++i) {
        int rc = munmap(maps[i], huge);
        CHECK(rc == 0, "munmap(hugetlb mapping %zu) failed (errno=%d %s)",
              i, errno, std::strerror(errno));
    }
    if (ballast) std::free(ballast);

    // Allocation still healthy after deregistration.
    void* after = std::malloc(1u << 20);
    CHECK(after != nullptr,
          "malloc(1 MiB) returned nullptr after unmapping the hugepages");
    if (after) std::free(after);

    if (failures) {
        printf("hugepage_mmap_test: %d FAILURES\n", failures);
        return 1;
    }
    printf("hugepage_mmap_test: all checks passed\n");
    return 0;
}

#else  // !__linux__

int main() {
    printf("SKIP: MAP_HUGETLB is Linux-only\n");
    return 77;
}

#endif
