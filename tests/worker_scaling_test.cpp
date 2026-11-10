// worker_scaling_test.cpp - the compressor's worker count follows the work
// (src/compress/worker_scaling.h, issue #8).
//
// The count used to be ceil(eligible pages / pages compressed per worker),
// which is workers x eligible / compressed. With every eligible page
// compressed it stayed where it was however long the tick took (one worker
// at 100 % for ticks of 3-9 s, seven CPUs idle); with some pages not
// compressed by the default workers (declined, or handed to a NUMA node's
// group) it climbed to the cap for 20 ms of work a tick; and past 2^23
// eligible pages in a tick the 32-bit fixed point went negative. The count
// is now ceil(2 x load), where the load is the CPU time the workers spent in
// the tick's phases per unit of wall time, smoothed.
//
// This drives the policy alone, on a model of the tick loop: the coordinator
// sleeps one interval, the tick does its work with N workers and takes
// work / N, and the count for the next tick comes from the work and the time
// since the previous tick started. The wiring (that the time is measured and
// the count applied) is test_worker_count_*'s.
//
//   1. a light load keeps one worker, tick after tick;
//   2. work arriving at a constant rate r (CPU-seconds per second) gets
//      ceil(2r) workers from the second tick on and keeps them: no
//      overshoot, no oscillation, and the tick is back inside 1.05 intervals
//      within 8 ticks;
//   3. a fixed cost per tick (a scan of a heap with nothing to compress)
//      settles on one count, and that count fits it in the interval;
//   4. a larger sample takes effect on the next tick;
//   5. the count comes down over several ticks once the work stops, and
//      reaches 1;
//   6. no input overflows: the largest values clamp to the cap, a zero
//      period and a cap below 1 are safe.
#include "compress/worker_scaling.h"

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <vector>

using smash::WorkerScaler;

namespace {
int failures = 0;
#define CHECK(cond, ...) do { \
    if (!(cond)) { std::fprintf(stderr, "FAIL: " __VA_ARGS__); std::fputc('\n', stderr); ++failures; } \
} while (0)

constexpr uint64_t kSec = 1000000000ull;

struct Tick { int workers; double duration; };

// The tick loop. `rate`: CPU-seconds of work arriving per second; `fixed`:
// CPU-seconds every tick costs whatever arrived. Starts with 2 workers, as
// the compressor does.
std::vector<Tick> runModel(double rate, double fixed, int ticks, int cap, double interval = 1.0) {
    WorkerScaler s;
    std::vector<Tick> out;
    int n = 2;
    double t = 0, last_start = 0;
    for (int i = 0; i < ticks; ++i) {
        t += interval;                                  // the coordinator sleeps
        const double period = t - last_start;           // start to start
        last_start = t;
        const double work = rate * period + fixed;
        const double d = work / n;                      // every worker busy
        t += d;
        out.push_back({n, d});
        n = s.update(static_cast<uint64_t>(work * 1e9), static_cast<uint64_t>(period * 1e9), cap);
    }
    return out;
}

void testLight() {
    // 20 ms of work a second: what 4 MiB/s of cold data costs.
    const auto r = runModel(0.02, 0, 100, 32);
    int most = 0;
    for (size_t i = 1; i < r.size(); ++i) most = std::max(most, r[i].workers);
    std::printf("light load (20 ms of work per second): %d worker(s) after the first tick\n", most);
    CHECK(most == 1, "20 ms of work per second ran with %d workers", most);
    // Just under half a worker is still one; just over is two.
    const auto under = runModel(0.49, 0, 40, 32), over = runModel(0.51, 0, 40, 32);
    CHECK(under.back().workers == 1, "0.49 CPU-s/s ran with %d workers", under.back().workers);
    CHECK(over.back().workers == 2, "0.51 CPU-s/s ran with %d workers", over.back().workers);
}

void testSustained() {
    for (double rate : {0.9, 1.7, 3.4, 6.2}) {
        const auto r = runModel(rate, 0, 40, 32);
        const int want = static_cast<int>(std::ceil(2 * rate));
        int least = 1 << 30, most = 0;
        double worst_late = 0;
        for (size_t i = 1; i < r.size(); ++i) {
            least = std::min(least, r[i].workers);
            most = std::max(most, r[i].workers);
            if (i >= 8) worst_late = std::max(worst_late, r[i].duration);
        }
        std::printf("sustained %.1f CPU-s/s: ticks 2-40 ran with %d-%d workers; ticks 1-4 took "
                    "%.2f %.2f %.2f %.2f s, the longest from tick 9 on %.2f s\n",
                    rate, least, most, r[0].duration, r[1].duration, r[2].duration, r[3].duration, worst_late);
        CHECK(least == want && most == want, "at %.1f CPU-s/s ticks ran with %d-%d workers, not %d throughout",
              rate, least, most, want);
        CHECK(worst_late <= 1.05, "at %.1f CPU-s/s a tick still took %.2f s after eight ticks", rate, worst_late);
    }
    // At the cap the count stays at the cap.
    const auto capped = runModel(20.0, 0, 20, 8);
    CHECK(capped.back().workers == 8, "20 CPU-s/s under a cap of 8 ran with %d workers", capped.back().workers);
}

void testFixedCost() {
    // 3.5 s of CPU every tick, nothing arriving: 4 workers fit it in the
    // interval. The count starts high (the first tick's 3.5 s came after a
    // sleep of one interval) and must come down to a count that still fits
    // and stay there.
    const auto r = runModel(0, 3.5, 40, 32);
    int least = 1 << 30, most = 0;
    double worst = 0;
    for (size_t i = 10; i < r.size(); ++i) {
        least = std::min(least, r[i].workers);
        most = std::max(most, r[i].workers);
        worst = std::max(worst, r[i].duration);
    }
    std::printf("fixed 3.5 CPU-s per tick: ticks 11-40 ran with %d-%d workers, longest %.2f s\n", least, most, worst);
    CHECK(least == most, "a fixed cost per tick ran with %d-%d workers: the count does not settle", least, most);
    CHECK(worst <= 1.0 + 1e-9, "a fixed cost of 3.5 s per tick still took %.2f s", worst);
    CHECK(most <= 5, "a fixed cost of 3.5 s per tick ran with %d workers", most);
}

void testUpAtOnce() {
    WorkerScaler s;
    for (int i = 0; i < 10; ++i) CHECK(s.update(0, kSec, 32) == 1, "idle ticks gave more than one worker");
    const int n = s.update(5 * kSec, kSec, 32);
    CHECK(n == 10, "5 s of work arriving in 1 s gave %d workers for the next tick, not 10", n);
    WorkerScaler c;
    CHECK(c.update(5 * kSec, kSec, 8) == 8, "the cap of 8 was not applied");
    // Half a worker's worth is one worker; a thousandth more is two.
    WorkerScaler e;
    const int exact = e.update(kSec / 2, kSec, 32);
    CHECK(exact == 1, "0.5 s of work per second gave %d workers", exact);
    WorkerScaler f;
    const int over = f.update(kSec / 2 + kSec / 1000, kSec, 32);
    CHECK(over == 2, "0.501 s of work per second gave %d workers", over);
    // The same work over a longer period is a smaller load.
    WorkerScaler g;
    const int spread = g.update(5 * kSec, 10 * kSec, 32);
    CHECK(spread == 1, "5 s of work that took 10 s to arrive gave %d workers", spread);
}

void testDecay() {
    WorkerScaler s;
    int prev = s.update(5 * kSec, kSec, 32);
    int ticks_to_one = -1, after3 = 0;
    uint64_t last_load = s.load_milli;
    for (int i = 1; i <= 200; ++i) {
        const int n = s.update(0, kSec, 32);
        CHECK(n <= prev, "the count rose from %d to %d with no work", prev, n);
        CHECK(s.load_milli < last_load || last_load == 0, "the load stopped falling at %" PRIu64, last_load);
        last_load = s.load_milli;
        if (i == 3) after3 = n;
        if (n == 1 && ticks_to_one < 0) ticks_to_one = i;
        prev = n;
    }
    std::printf("after a tick worth 10 workers: %d workers three idle ticks later, 1 worker after %d\n",
                after3, ticks_to_one);
    CHECK(after3 >= 2, "the count fell to %d within three idle ticks: no smoothing", after3);
    CHECK(ticks_to_one > 0 && ticks_to_one <= 12, "the count did not reach 1 within 12 idle ticks (%d)", ticks_to_one);
    CHECK(s.load_milli == 0, "the load stopped %" PRIu64 " thousandths above an idle tick's", s.load_milli);
}

void testRange() {
    WorkerScaler s;
    CHECK(s.update(UINT64_MAX, kSec, 32) == 32, "the largest sample did not clamp to the cap");
    CHECK(s.update(UINT64_MAX, 1, 32) == 32, "the largest sample over 1 ns did not clamp to the cap");
    CHECK(s.load_milli == UINT64_MAX, "the largest load was not kept");
    CHECK(s.update(UINT64_MAX - 1, 1, 32) == 32, "a sample just below the largest overflowed");
    CHECK(WorkerScaler::workersFor(UINT64_MAX, 1 << 30) == 1 << 30, "the largest load overflowed the doubling");
    CHECK(WorkerScaler::workersFor(UINT64_MAX / 2 + 1, 32) == 32, "a load past half the range overflowed the doubling");
    // What the 32-bit fixed point could not hold: 2^23 pages at 10 us each
    // is 84 s of work in one tick.
    WorkerScaler big;
    CHECK(big.update(84 * kSec, kSec, 32) == 32, "84 s of work in a second did not give the cap");
    WorkerScaler wide;
    CHECK(wide.update(84 * kSec, 85 * kSec, 128) == 2, "84 s of work over 85 s under a cap of 128 did not give 2 workers");
    WorkerScaler z;
    CHECK(z.update(5 * kSec, 0, 32) == 32, "a zero period did not give the cap");
    CHECK(z.update(0, 0, 32) == 32, "the load fell to nothing in one idle tick");
    WorkerScaler lo;
    CHECK(lo.update(5 * kSec, kSec, 0) == 1, "a cap of 0 did not give one worker");
    CHECK(lo.update(5 * kSec, kSec, -3) == 1, "a negative cap did not give one worker");
    // SMASH_TEST_TICK_MS: the period is whatever the tick's is.
    WorkerScaler t;
    CHECK(t.update(5 * 1000000ull, 2 * 1000000ull, 32) == 5, "5 ms of work in 2 ms did not give 5 workers");
    CHECK(WorkerScaler::workersFor(0, 32) == 1, "no load gave no worker");
}
}  // namespace

int main() {
    testLight();
    testSustained();
    testFixedCost();
    testUpAtOnce();
    testDecay();
    testRange();
    if (failures) {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("OK\n");
    return 0;
}
