// tick_budget_test.cpp - SMASH_TICK_PAGE_BUDGET: the most pages one tick
// compresses (src/compress/compressor_thread.h compressPage, tick_compressed_).
//
// A tick compressed everything that was eligible, however much: the first
// cold wave of a large process made ticks of 724 s and 1448 s on a 128-core
// box, during which nothing else the tick does (monitoring, the reclaim of
// shadow pages, the adaptation of the worker count) happened. With a budget
// the tick stops compressing at N pages and the rest, still cold, wait for
// the next tick.
//
//   8 MiB (2048 pages) goes cold under SMASH_TICK_PAGE_BUDGET=1000; once it
//   is all compressed, smash_tick_budget_info must show no tick compressed
//   more than 1000 pages and at least 1000 eligible pages were turned away
//   by the budget. Without the budget one tick compresses all 2048.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <dlfcn.h>
#include <unistd.h>

int main() {
    using info_fn = int (*)(const void*, int*, int*, int*);
    using budget_fn = int (*)(uint64_t*, uint64_t*, uint64_t*);
    auto page_info = reinterpret_cast<info_fn>(dlsym(RTLD_DEFAULT, "smash_page_info"));
    auto budget_info = reinterpret_cast<budget_fn>(dlsym(RTLD_DEFAULT, "smash_tick_budget_info"));
    if (!page_info || !budget_info) { std::fprintf(stderr, "FAIL: diagnostic exports missing\n"); return 1; }
    const char* bv = std::getenv("SMASH_TICK_PAGE_BUDGET");
    const uint64_t budget = bv ? static_cast<uint64_t>(std::atoll(bv)) : 0;
    if (!budget) { std::fprintf(stderr, "FAIL: SMASH_TICK_PAGE_BUDGET must be set\n"); return 1; }
    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    const size_t bytes = 8u << 20;
    const size_t pages = bytes / ps;
    int failures = 0;

    auto* p = static_cast<unsigned char*>(std::malloc(bytes));
    if (!p) return 1;
    for (size_t off = 0; off < bytes; off += 64) p[off] = static_cast<unsigned char>((off >> 12) * 5 + (off >> 6));

    size_t compressed = 0;
    int waited = 0;
    for (; waited < 60000 && compressed < pages * 95 / 100; waited += 200) {
        usleep(200000);
        compressed = 0;
        for (size_t off = 0; off < bytes; off += ps) {
            int st = 0, c = 0, r = 0;
            if (page_info(p + off, &st, &c, &r) == 0 && st == 2) ++compressed;
        }
    }
    uint64_t budget_cfg = 0, max_per_tick = 0, turned_away = 0;
    budget_info(&budget_cfg, &max_per_tick, &turned_away);
    std::printf("tick_budget: budget %llu; %zu of %zu pages compressed after %d ms; most in one tick %llu; %llu eligible pages turned away\n",
                static_cast<unsigned long long>(budget_cfg), compressed, pages, waited,
                static_cast<unsigned long long>(max_per_tick), static_cast<unsigned long long>(turned_away));
    if (compressed < pages * 95 / 100) { std::fprintf(stderr, "FAIL: the block did not compress in 60 s\n"); ++failures; }
    if (budget_cfg != budget) { std::fprintf(stderr, "FAIL: the budget was not read\n"); ++failures; }
    if (max_per_tick > budget) { std::fprintf(stderr, "FAIL: a tick compressed %llu pages, over the budget of %llu\n", static_cast<unsigned long long>(max_per_tick), static_cast<unsigned long long>(budget)); ++failures; }
    if (turned_away < pages - budget) { std::fprintf(stderr, "FAIL: only %llu pages were turned away; the budget did not bind\n", static_cast<unsigned long long>(turned_away)); ++failures; }
    size_t bad = 0;
    for (size_t off = 0; off < bytes; off += 64) bad += p[off] != static_cast<unsigned char>((off >> 12) * 5 + (off >> 6));
    if (bad) { std::fprintf(stderr, "FAIL: %zu bytes wrong\n", bad); ++failures; }
    std::free(p);
    if (failures) { std::fprintf(stderr, "tick_budget: %d failure(s)\n", failures); return 1; }
    std::printf("tick_budget: all tests passed\n");
    return 0;
}
