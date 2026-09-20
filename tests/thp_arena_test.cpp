// thp_arena_test.cpp - SMASH_THP=1 must put the arena on transparent hugepages,
// and its absence must leave it on 4 KiB pages.
//
// Only meaningful where the host policy is `madvise`: under `always` the
// kernel gives huge pages to everyone (so the off-case cannot be verified) and
// under `never` to no one. The test skips otherwise rather than asserting
// something the policy, not smash, decides.
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

}  // namespace

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    if (!policyIsMadvise()) {
        std::printf("SKIP: host THP policy is not [madvise]; the off-case cannot be distinguished\n");
        return 0;
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
