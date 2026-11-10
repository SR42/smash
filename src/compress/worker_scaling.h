// worker_scaling.h - how many default compressor workers the next tick gets.
//
// The count follows the work, and the work is measured as CPU time: `busy_ns`
// is what the default workers spent inside one tick's phases, summed over the
// workers (CompressorThread::executePhase), and `period_ns` is the wall time
// from the previous tick's start to this one's. Their quotient is the load:
// how many workers' worth of work arrives per unit of time. It does not
// depend on how many workers did the work, which is what makes it a measure
// to steer by. The next tick runs with
//
//     N = ceil(2 x load)
//
// workers: busy at most half the time, which is the same as a tick taking no
// longer than the sleep between two ticks (with work arriving at rate r, a
// tick lasts r x interval / (N - r), and that is at most the interval when
// N >= 2r). This is Little's law read directly: the mean number of busy
// servers is the arrival rate times the service time, which is the service
// time demanded per unit of time, and a CPU clock measures that.
//
// The load is smoothed: it rises to a larger sample at once and falls a
// quarter of the way to a smaller one per tick.
//
//   - Up at once: a compressor that is behind keeps cold memory resident for
//     as long as it stays behind, and each tick of delay is a long one (the
//     tick that showed the load ran with too few workers). A worker with
//     nothing to take parks at once, so too many for a few ticks cost a wake
//     each.
//   - CPU time, not wall time: a worker that waits for a lock or for a CPU
//     is not doing work another worker could do, and counting the wait asks
//     for more threads exactly when they make things worse.
//   - Per unit of time, not per tick: the work of one tick grows with the
//     time since the last one, so a count taken from it rises after a long
//     tick and falls after a short one, for ever (4, 4, 4, 3, 4, ... at a
//     steady 1.7 workers' worth).
//   - No page counts. The count used to be ceil(eligible / (compressed per
//     worker)), which is workers x eligible / compressed: it stayed where it
//     was when every eligible page was compressed, however long the tick
//     took, and climbed to the cap when some were not (declined, or handed
//     to a NUMA node's group), however little there was to do (issue #8).
//
// Pure, so tests can drive it (tests/worker_scaling_test.cpp).
#pragma once

#include <cstdint>

namespace smash {

struct WorkerScaler {
    // The load in thousandths of a worker, smoothed.
    uint64_t load_milli = 0;

    // One tick's sample. Returns the worker count for the next tick, in
    // [1, max_workers].
    int update(uint64_t busy_ns, uint64_t period_ns, int max_workers) {
        const uint64_t sample = loadMilli(busy_ns, period_ns);
        if (sample >= load_milli) load_milli = sample;
        else load_milli -= (load_milli - sample + 3) / 4;   // rounded up: it arrives
        return workersFor(load_milli, max_workers);
    }

    static uint64_t loadMilli(uint64_t busy_ns, uint64_t period_ns) {
        const uint64_t per_milli = period_ns / 1000;
        return busy_ns / (per_milli ? per_milli : 1);
    }

    static int workersFor(uint64_t load_milli, int max_workers) {
        if (max_workers < 1) max_workers = 1;
        // ceil(2 x load) is at least max_workers from here on; also keeps
        // the doubling below inside 64 bits.
        if (load_milli >= static_cast<uint64_t>(max_workers) * 500) return max_workers;
        const uint64_t n = (2 * load_milli + 999) / 1000;
        return n <= 1 ? 1 : static_cast<int>(n);
    }
};

}  // namespace smash
