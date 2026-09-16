// thread_cache_reclaim_test.cpp - A thread's cache must come back when the
// thread exits, whatever created that thread.
//
// A ThreadCache is ~92 KiB and is created lazily on a thread's first
// allocation. Reclaiming it used to depend entirely on alloc8's pthread_create
// interposer calling xxthread_cleanup, and on glibc >= 2.34 that interposer
// never ran for application threads: callers bind pthread_create@GLIBC_2.34
// while libsmash exported only the base version, so libc won. Measured before
// the fix: of 200 application threads 16 were wrapped (all 16 smash's own),
// RSS grew 91.2 KiB per thread — +182 MiB over 2000 threads — and a run of
// 14000 threads segfaulted once BootstrapAlloc was exhausted and
// newThreadCache() placement-new'd onto null.
//
// Two independent mechanisms now cover this, and the ctest runs this same body
// under both:
//   - test_thread_cache_reclaim_full: versioned pthread_create alias, so the
//     interposer wraps application threads and xxthread_cleanup runs;
//   - test_thread_cache_reclaim_tls_only: SMASH_NO_THREAD_HOOKS=1 makes
//     threadInit/threadCleanup no-ops, leaving only smash's pthread_key_t
//     destructor to reclaim the cache;
//   - test_thread_cache_reclaim_large_only: small allocations never reach the
//     slab, so no cache is created at all.
//
// The threads are created and joined one at a time, so at most one worker is
// ever alive: any sustained RSS growth is retained memory, not concurrency.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <pthread.h>
#include <unistd.h>

namespace {

constexpr int kWarmupThreads = 100;   // first threads also fault in libc/smash state
constexpr int kMeasuredThreads = 1500;
// A leaked cache is ~92 KiB per thread; anything under a few KiB is unrelated
// growth (glibc arenas, page tables). 8 KiB leaves wide margin either way.
constexpr double kMaxKiBPerThread = 8.0;

void* worker(void*) {
    void* p = malloc(64);
    if (p) { std::memset(p, 1, 64); free(p); }
    return nullptr;
}

bool churn(int n) {
    for (int i = 0; i < n; ++i) {
        pthread_t t;
        if (pthread_create(&t, nullptr, worker, nullptr) != 0) {
            std::fprintf(stderr, "FAIL: pthread_create failed at thread %d\n", i);
            return false;
        }
        pthread_join(t, nullptr);
    }
    return true;
}

long rssKiB() {
    std::FILE* f = std::fopen("/proc/self/statm", "r");
    if (!f) return -1;
    long total = 0, resident = -1;
    if (std::fscanf(f, "%ld %ld", &total, &resident) != 2) resident = -1;
    std::fclose(f);
    return resident < 0 ? -1 : resident * (sysconf(_SC_PAGESIZE) / 1024);
}

}  // namespace

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);

    if (!churn(kWarmupThreads)) return 1;
    const long base = rssKiB();
    if (base < 0) {
        std::printf("SKIP: cannot read /proc/self/statm\n");
        return 0;
    }

    if (!churn(kMeasuredThreads)) return 1;
    const long grew = rssKiB() - base;
    const double per_thread = static_cast<double>(grew) / kMeasuredThreads;

    std::printf("%d threads created and joined one at a time: RSS %+ld KiB "
                "(%.2f KiB/thread)\n", kMeasuredThreads, grew, per_thread);

    if (per_thread > kMaxKiBPerThread) {
        std::fprintf(stderr,
            "FAIL: %.1f KiB retained per thread (limit %.1f). A ThreadCache is "
            "~92 KiB, so this is the cache leaking on thread exit — check that "
            "pthread_create is exported at the caller's symbol version and that "
            "smash's pthread_key_t destructor still runs.\n",
            per_thread, kMaxKiBPerThread);
        return 1;
    }

    std::printf("PASS: thread caches are reclaimed on thread exit\n");
    return 0;
}
