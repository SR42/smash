// span_descriptor_race_test.cpp - concurrent large/aligned alloc+free must not
// corrupt the span-descriptor free list.
//
// Every allocation LargeAlloc serves takes a Span descriptor from a free list
// and returns it on free: aligned allocations (alignas(64) new) in full mode,
// and allocations over 16 KiB in large-only mode. The list was a lock-free
// Treiber stack whose pop read s->next_free and then CASed the head — the
// textbook ABA race. With many threads it installed a descriptor that was in
// use, smash then failed to recognise its own block on free and forwarded it
// to glibc, which aborted ("free(): invalid pointer") or the process
// segfaulted. Measured: 5 of 5 runs of a 64-thread stress crashed; with only
// the list replaced by a spinlock stack, 0 of 5. Found by a thread-churn stress
// that crashed the pre-fix library.
//
// Threads allocate, keep a few, and free some blocks from other threads.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

struct alignas(64) Obj { char b[64]; };

int main() {
    const char* lo = std::getenv("SMASH_LARGE_ONLY");
    const bool large_only = lo && lo[0] == '1';
    // Large-only sends small aligned requests to the system allocator, so use
    // blocks that reach LargeAlloc there.
    const size_t big = 32 * 1024;
    constexpr int kThreads = 64, kIters = 40000;
    std::mutex mu;
    std::vector<void*> shared;
    std::vector<std::thread> ts;
    for (int t = 0; t < kThreads; ++t) ts.emplace_back([&, t] {
        unsigned s = t + 1;
        std::vector<void*> mine;
        auto release = [&](void* p) {
            if (large_only) std::free(p); else delete static_cast<Obj*>(p);
        };
        for (int i = 0; i < kIters; ++i) {
            s = s * 1103515245 + 12345;
            void* p = large_only ? std::malloc(big) : static_cast<void*>(new Obj);
            std::memset(p, t, 64);
            mine.push_back(p);
            if (mine.size() > 16) {
                if ((s >> 16) & 1) { std::lock_guard<std::mutex> g(mu); shared.push_back(mine.back()); }
                else release(mine.back());
                mine.pop_back();
            }
            if ((s & 31) == 0) {
                std::lock_guard<std::mutex> g(mu);
                while (!shared.empty()) { release(shared.back()); shared.pop_back(); }
            }
        }
        for (void* p : mine) release(p);
    });
    for (auto& t : ts) t.join();
    for (void* p : shared) large_only ? std::free(p) : delete static_cast<Obj*>(p);
    std::printf("PASS: %d threads x %d %s allocations, no crash\n", kThreads, kIters,
                large_only ? "32 KiB" : "aligned 64-byte");
    return 0;
}
