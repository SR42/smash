// thp_arena_test.cpp - SMASH_THP=1 must put the arena on transparent hugepages,
// and its absence must leave it on 4 KiB pages.
//
// Only meaningful where the host policy is `madvise`: under `always` the
// kernel gives huge pages to everyone (so the off-case cannot be verified) and
// under `never` to no one. The test skips otherwise rather than asserting
// something the policy, not smash, decides.
//
// SMASH_THP=0 is checked under any policy: the arena must be MADV_NOHUGEPAGE
// (VmFlags nh), the way out of huge pages on a host whose policy is `always`.
// Without it the arena must not be: SMASH_WRITE_TRACK=idle used to set it too.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

long smapsField(const char* key) {
    FILE* f = std::fopen("/proc/self/smaps_rollup", "r");
    if (!f) return -1;
    char line[512];
    long v = -1;
    while (std::fgets(line, sizeof(line), f))
        if (!std::strncmp(line, key, std::strlen(key))) { v = std::atol(line + std::strlen(key)); break; }
    std::fclose(f);
    return v;
}

bool policyIsMadvise() {
    FILE* f = std::fopen("/sys/kernel/mm/transparent_hugepage/enabled", "r");
    if (!f) return false;
    char buf[128] = {};
    if (!std::fgets(buf, sizeof(buf), f)) { std::fclose(f); return false; }
    std::fclose(f);
    return std::strstr(buf, "[madvise]") != nullptr;
}

// VmFlags of the mapping holding p, from /proc/self/smaps.
bool mappingHasFlag(const void* p, const char* flag) {
    FILE* f = std::fopen("/proc/self/smaps", "r");
    if (!f) return false;
    char line[512];
    bool in = false, has = false;
    const auto a = reinterpret_cast<uintptr_t>(p);
    while (std::fgets(line, sizeof(line), f)) {
        unsigned long lo, hi;
        char c;
        if (std::sscanf(line, "%lx-%lx %c", &lo, &hi, &c) == 3) { in = a >= lo && a < hi; continue; }
        if (in && !std::strncmp(line, "VmFlags:", 8)) { has = std::strstr(line, flag) != nullptr; break; }
    }
    std::fclose(f);
    return has;
}

}  // namespace

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    {
        const char* v = std::getenv("SMASH_THP");
        const bool want_nh = v && v[0] == '0';
        auto* q = static_cast<unsigned char*>(std::malloc(8u << 20));
        if (!q) { std::fprintf(stderr, "FAIL: malloc\n"); return 1; }
        q[0] = 1;
        const bool nh = mappingHasFlag(q, " nh");
        std::printf("thp_arena_test: SMASH_THP=%s, SMASH_WRITE_TRACK=%s: arena %s MADV_NOHUGEPAGE\n", v ? v : "(unset)",
                    std::getenv("SMASH_WRITE_TRACK") ? std::getenv("SMASH_WRITE_TRACK") : "(unset)", nh ? "is" : "is not");
        std::free(q);
        if (nh != want_nh) {
            std::fprintf(stderr, "FAIL: the arena %s MADV_NOHUGEPAGE\n", nh ? "is" : "is not");
            return 1;
        }
        if (want_nh) { std::printf("thp_arena_test: all checks passed\n"); return 0; }
    }
    if (!policyIsMadvise()) {
        std::printf("SKIP: host THP policy is not [madvise]; the off-case cannot be distinguished\n");
        return 77;
    }
    const char* env = std::getenv("SMASH_THP");
    const bool want_thp = env && env[0] == '1';

    constexpr size_t kBytes = 64u << 20;      // 32 huge pages' worth
    auto* p = static_cast<unsigned char*>(std::malloc(kBytes));
    if (!p) { std::fprintf(stderr, "FAIL: malloc\n"); return 1; }
    for (size_t i = 0; i < kBytes; i += 4096) p[i] = static_cast<unsigned char>(i >> 12);

    const long ahp_kb = smapsField("AnonHugePages:");
    std::printf("thp_arena_test: SMASH_THP=%s -> AnonHugePages %ld MiB after touching %zu MiB\n",
                want_thp ? "1" : "(unset)", ahp_kb >> 10, kBytes >> 20);
    if (ahp_kb < 0) { std::fprintf(stderr, "FAIL: could not read AnonHugePages\n"); return 1; }

    // Half the block is a generous bar: the kernel may decline some.
    const long want_min_kb = static_cast<long>((kBytes >> 11));   // 50% in KiB
    if (want_thp && ahp_kb < want_min_kb) {
        std::fprintf(stderr, "FAIL: SMASH_THP=1 but only %ld MiB of %zu MiB is huge-page backed — "
                     "the arena did not get MADV_HUGEPAGE\n", ahp_kb >> 10, kBytes >> 20);
        return 1;
    }
    if (!want_thp && ahp_kb > want_min_kb) {
        std::fprintf(stderr, "FAIL: no SMASH_THP, yet %ld MiB is huge-page backed — the arena is "
                     "being hinted when it should not be\n", ahp_kb >> 10);
        return 1;
    }
    std::free(p);
    std::printf("thp_arena_test: all checks passed\n");
    return 0;
}
