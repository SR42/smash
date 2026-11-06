// Per-thread state in a TSD slot (macOS, SMASH_TSD_SLOT): lifecycle test.
//
// Runs under interposition (DYLD_INSERT_LIBRARIES). Checks that
//   1. ThreadStates are reclaimed at thread exit and reused: after thousands
//      of short-lived threads, the number ever allocated stays near the peak
//      number of concurrent threads, not the number of threads created;
//   2. threads that never pass through pthread_create (GCD workers) work;
//   3. a thread whose own pthread-key destructor allocates after smash's
//      destructor ran neither crashes nor leaks a state per thread;
//   4. fork() with live threads keeps the free list usable in the child;
// and that every allocation's contents survive. Skips (exit 0) when the
// library has no smash_tsd_stats (TSD slot not compiled in, e.g. Linux).

#include <dispatch/dispatch.h>
#include <dlfcn.h>
#include <pthread.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

using StatsFn = void (*)(size_t*, size_t*, size_t*);
static StatsFn stats_fn;
static std::atomic<int> failures{0};

static void check(bool ok, const char* what) {
    if (!ok) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        failures.fetch_add(1);
    }
}

static size_t allocated() {
    size_t a = 0, l = 0, f = 0;
    stats_fn(&a, &l, &f);
    return a;
}

// Allocate a mix of sizes, write a pattern, verify, free.
static void churn(unsigned seed) {
    void* ptrs[64];
    size_t sizes[64];
    for (int i = 0; i < 64; ++i) {
        sizes[i] = 8 + ((seed * 2654435761u + i * 40503u) % 4096);
        ptrs[i] = std::malloc(sizes[i]);
        std::memset(ptrs[i], (int)(i + seed) & 0xff, sizes[i]);
    }
    for (int i = 0; i < 64; ++i) {
        auto* p = static_cast<unsigned char*>(ptrs[i]);
        check(p[0] == ((i + seed) & 0xff) && p[sizes[i] - 1] == ((i + seed) & 0xff),
              "allocation contents corrupted");
        std::free(ptrs[i]);
    }
}

static void* thread_body(void* arg) {
    churn(static_cast<unsigned>(reinterpret_cast<uintptr_t>(arg)));
    return nullptr;
}

// Test 3: a destructor (registered by the app) that allocates while the
// thread is exiting, possibly after smash's own destructor released the
// thread's state.
static pthread_key_t app_key;
static void app_destructor(void* v) {
    void* p = std::malloc(100);
    std::memset(p, 1, 100);
    std::free(p);
    std::free(v);
}
static void* thread_with_dtor(void*) {
    pthread_setspecific(app_key, std::malloc(32));
    churn(7);
    return nullptr;
}

static void run_threads(int rounds, int width, void* (*body)(void*)) {
    for (int r = 0; r < rounds; ++r) {
        std::vector<pthread_t> ts(width);
        for (int i = 0; i < width; ++i)
            pthread_create(&ts[i], nullptr, body, reinterpret_cast<void*>(uintptr_t(r * width + i)));
        for (auto& t : ts) pthread_join(t, nullptr);
    }
}

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);  // keep progress lines if a phase crashes
    stats_fn = reinterpret_cast<StatsFn>(dlsym(RTLD_DEFAULT, "smash_tsd_stats"));
    if (!stats_fn) {
        std::printf("tsd_thread_test: SKIP (smash_tsd_stats not exported; TSD slot not compiled in)\n");
        return 0;
    }
    churn(1);  // main thread
    const int kWidth = 16;

    // 1. pthread churn: 64 rounds x 16 threads = 1024 thread lifetimes.
    size_t before = allocated();
    run_threads(64, kWidth, thread_body);
    size_t grew = allocated() - before;
    std::printf("pthread churn: 1024 threads, ThreadStates allocated +%zu\n", grew);
    // Reuse bound: at most a round's width plus slack for threads whose exit
    // overlaps the next round's start (join returns before TSD destructors
    // are guaranteed to have finished on some macOS versions).
    check(grew <= 4 * kWidth, "ThreadStates not reused across exiting threads");

    // 2. GCD workers (created by libdispatch, not via pthread_create).
    dispatch_apply(4096, dispatch_get_global_queue(QOS_CLASS_DEFAULT, 0),
                   ^(size_t i) { churn(static_cast<unsigned>(i)); });

    // 3. App destructor that allocates during thread exit.
    pthread_key_create(&app_key, app_destructor);
    before = allocated();
    run_threads(32, kWidth, thread_with_dtor);
    grew = allocated() - before;
    std::printf("destructor-allocates churn: 512 threads, ThreadStates allocated +%zu\n", grew);
    check(grew <= 4 * kWidth, "ThreadStates leaked when a destructor allocates at exit");

    // 4. fork with threads alive (free-list lock must be consistent in child).
    std::atomic<bool> stop{false};
    pthread_t bg[4];
    for (auto& t : bg)
        pthread_create(&t, nullptr, [](void* s) -> void* {
            while (!static_cast<std::atomic<bool>*>(s)->load()) {
                pthread_t inner;
                pthread_create(&inner, nullptr, thread_body, nullptr);
                pthread_join(inner, nullptr);
            }
            return nullptr;
        }, &stop);
    for (int i = 0; i < 20; ++i) {
        pid_t pid = fork();
        if (pid == 0) {
            pthread_t t;  // child: create and reap a thread, allocate
            pthread_create(&t, nullptr, thread_body, nullptr);
            pthread_join(t, nullptr);
            churn(3);
            _exit(failures.load() ? 1 : 0);
        }
        int status = 0;
        waitpid(pid, &status, 0);
        check(WIFEXITED(status) && WEXITSTATUS(status) == 0, "forked child failed");
    }
    stop.store(true);
    for (auto& t : bg) pthread_join(t, nullptr);

    size_t a = 0, l = 0, f = 0;
    stats_fn(&a, &l, &f);
    std::printf("ThreadStates: allocated %zu, live %zu, fallbacks %zu\n", a, l, f);
    check(f == 0, "thread_local fallback used (BootstrapAlloc exhausted?)");
    if (failures.load()) {
        std::fprintf(stderr, "tsd_thread_test: %d failure(s)\n", failures.load());
        return 1;
    }
    std::printf("tsd_thread_test: OK\n");
    return 0;
}
