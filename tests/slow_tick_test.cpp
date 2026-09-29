// slow_tick_test.cpp - how soon an idle block leaves RSS must not scale with
// how long compressor ticks take.
//
// Every stage of the pipeline counted ticks: cold after 10 ticks of no
// writes, one more tick watched for reads (PROT_NONE escalation), then two
// ticks in shadow before the page is released — about 14 ticks. A tick is
// meant to be 1 s, but on a 128-core arm64 application with a 145 GB heap
// ticks took 100-724 s, and compression "kicked in after at least 30
// minutes". Idle time is now counted in nominal seconds (a tick that took
// 700 s adds 700, saturating), stopping at the cold floor so the read-watch
// tick still happens.
//
// Ticks are stretched to ~4 s (SMASH_TEST_TICK_DELAY_MS=4000) with the
// default 10-tick cold timeout, under page-protection tracking:
//   - 64 MiB written once must leave RSS (90%) within 40 s (about 60 s by
//     tick count);
//   - 4 MiB written every second must stay resident;
//   - 4 MiB only READ every second must stay resident: a jump in the idle
//     count must not skip the PROT_NONE read watch.
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>

static size_t residentPages(const char* p, size_t len, size_t ps) {
    const size_t n = len / ps;
    static unsigned char vec[65536];
    if (n > sizeof(vec) || mincore(const_cast<char*>(p), len, vec) != 0) return n;
    size_t r = 0;
    for (size_t i = 0; i < n; ++i) r += vec[i] & 1;
    return r;
}

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) { std::printf("SKIP: not running under libsmash\n"); return 0; }
    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    constexpr size_t kIdle = 64u << 20, kHot = 4u << 20;
    auto* idle = static_cast<char*>(malloc(kIdle));
    auto* written = static_cast<char*>(malloc(kHot));
    auto* read_only = static_cast<char*>(malloc(kHot));
    for (size_t i = 0; i < kIdle; ++i) idle[i] = static_cast<char>((i / ps) * 7 + (i % 61));
    std::memset(written, 3, kHot);
    for (size_t i = 0; i < kHot; ++i) read_only[i] = static_cast<char>((i / ps) + (i % 17));

    std::atomic<bool> stop{false};
    std::atomic<unsigned long> sink{0};
    std::thread toucher([&] {
        for (unsigned v = 4; !stop.load(std::memory_order_relaxed); ++v) {
            unsigned long sum = 0;
            for (size_t off = 0; off < kHot; off += ps) {
                written[off] = static_cast<char>(v);
                sum += static_cast<unsigned char>(read_only[off]);
            }
            sink += sum;
            sleep(1);
        }
    });

    size_t idle_res = kIdle / ps;
    int waited = 0;
    for (; waited < 40 && idle_res * 10 > kIdle / ps; ++waited) {
        sleep(1);
        idle_res = residentPages(idle, kIdle, ps);
    }
    const size_t w_res = residentPages(written, kHot, ps);
    const size_t r_res = residentPages(read_only, kHot, ps);
    stop = true;
    toucher.join();

    std::printf("~4 s ticks: idle block %zu of %zu pages resident after %d s; written block "
                "%zu of %zu, read block %zu of %zu resident\n", idle_res, kIdle / ps, waited,
                w_res, kHot / ps, r_res, kHot / ps);
    int failures = 0;
    if (idle_res * 10 > kIdle / ps) {
        std::printf("FAIL: the idle block did not leave RSS within 40 s of 4 s ticks: idle "
                    "time is counted in ticks, not seconds\n");
        ++failures;
    }
    if (w_res * 10 < (kHot / ps) * 9) {
        std::printf("FAIL: a block written every second left RSS\n");
        ++failures;
    }
    if (r_res * 10 < (kHot / ps) * 9) {
        std::printf("FAIL: a block read every second left RSS: the read watch was skipped\n");
        ++failures;
    }
    for (size_t i = 0; i < kIdle; i += 4093)
        if (idle[i] != static_cast<char>((i / ps) * 7 + (i % 61))) { std::printf("FAIL: idle block corrupted\n"); return 1; }
    free(idle);
    free(written);
    free(read_only);
    if (failures) return 1;
    std::printf("slow_tick_test: all checks passed\n");
    return 0;
}
