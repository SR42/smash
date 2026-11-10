// slab_purge_test.cpp - the purge of long-empty slab spans runs without the
// slab's lock (src/core/slab.h Slab::purgeResidentEmpty, issue #49).
//
// A span that empties stays resident for two compressor ticks; the tick then
// drops its pages with madvise. That ran under the slab's lock, one call per
// span over the whole list: after a large free it was tens of thousands of
// system calls in one hold (768 MiB of 256-byte objects: 49,000 calls,
// 270 ms), and every malloc of that class from that arena waited for all of
// it. The spans now leave the slab's lists 64 at a time and their pages are
// dropped with the lock released, neighbours in one call.
//
// This program defines madvise(), so libsmash's calls come here first: it
// sees every purge call, and can hold the purge inside one.
//
//   unlocked   (1) Free 32 MiB of 256-byte objects and wait for the purge.
//              Inside its first madvise a second thread allocates the same
//              number of objects from the same call site, which uses up
//              every empty span of the slab, and fills them. It must finish
//              while the purge is still inside that call (under the lock it
//              cannot start), none of its objects may lie in the range being
//              dropped, and all must still hold their bytes afterwards.
//              (2) Free those objects and count the next purge: it must drop
//              at least 90% of them, in calls of 128 KiB or more on average
//              (one call per span is 16 KiB).
//   fork       Free 64 MiB and slow each purge call by 5 ms. fork() during
//              the purge must come back within 1.5 s and before three
//              quarters of the purge is done: the prepare handler waits for
//              the compressor's tick, and the purge stops at its next batch
//              when asked. The child must be able to allocate from that
//              slab, and the rest must be purged afterwards.
//
// Environment (tests/CMakeLists.txt): one arena, so both threads use one
// slab, and a resident-empty cap above what is freed, so the free itself
// drops nothing.
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <dlfcn.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <unistd.h>

namespace {
constexpr size_t kObj = 256;

// What the madvise hook watches. Set before g_mode.
std::atomic<uintptr_t> g_lo{0}, g_hi{0};
enum Mode { kOff = 0, kHoldFirst, kCount, kSlow };
std::atomic<int> g_mode{kOff};
std::atomic<uint64_t> g_calls{0}, g_bytes{0};
std::atomic<int> g_first{0};
std::atomic<uintptr_t> g_run_lo{0}, g_run_hi{0};
std::atomic<int> g_other_in_time{0};
std::atomic<int> g_slow{0};
sem_t g_go, g_other_done, g_resume, g_first_handled;

bool waitSem(sem_t* s, int seconds) {
    timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += seconds;
    int r;
    while ((r = sem_timedwait(s, &ts)) != 0 && errno == EINTR) {}
    return r == 0;
}

__attribute__((noinline)) void* site(size_t size) {
    void* p = std::malloc(size);
    __asm__ volatile("" ::"r"(p) : "memory");
    return p;
}

void** pointerArray(size_t n) {
    void* m = mmap(nullptr, n * sizeof(void*), PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    return m == MAP_FAILED ? nullptr : static_cast<void**>(m);
}

void range(void** v, size_t n, uintptr_t* lo, uintptr_t* hi) {
    *lo = UINTPTR_MAX;
    *hi = 0;
    for (size_t i = 0; i < n; ++i) {
        const auto p = reinterpret_cast<uintptr_t>(v[i]);
        if (p < *lo) *lo = p;
        if (p + kObj > *hi) *hi = p + kObj;
    }
}

unsigned char patternOf(size_t i) { return static_cast<unsigned char>(i % 251 + 1); }

// Waits until the purge has dropped `want` bytes of the watched range.
bool awaitBytes(uint64_t want, int seconds) {
    for (int i = 0; i < seconds * 20; ++i) {
        if (g_bytes.load() >= want) return true;
        usleep(50000);
    }
    return g_bytes.load() >= want;
}
}  // namespace

// libsmash calls madvise through its PLT; this definition is found first.
// Default visibility and `used`: the tests build with -fvisibility=hidden
// and LTO, which otherwise drop it as an unreferenced local.
extern "C" __attribute__((visibility("default"), used))
int madvise(void* addr, size_t len, int advice) noexcept {
    const int mode = g_mode.load(std::memory_order_acquire);
    const auto a = reinterpret_cast<uintptr_t>(addr);
    if (mode != kOff && advice == MADV_DONTNEED && a >= g_lo.load() && a < g_hi.load()) {
        g_calls.fetch_add(1);
        g_bytes.fetch_add(len);
        if (mode == kHoldFirst && g_first.exchange(1) == 0) {
            // The purge is inside its first call. Let the other thread
            // allocate, and only then drop the pages.
            g_run_lo.store(a);
            g_run_hi.store(a + len);
            sem_post(&g_go);
            g_other_in_time.store(waitSem(&g_other_done, 10) ? 1 : 0);
            const int r = static_cast<int>(syscall(SYS_madvise, addr, len, advice));
            sem_post(&g_resume);
            sem_post(&g_first_handled);
            return r;
        }
        if (mode == kSlow) {
            if (g_first.exchange(1) == 0) sem_post(&g_go);
            if (g_slow.load()) usleep(5000);
        }
    }
    return static_cast<int>(syscall(SYS_madvise, addr, len, advice));
}

namespace {
struct Other {
    void** v;
    size_t n;
    size_t in_run = 0, damaged = 0, failed = 0;
};

void* otherThread(void* arg) {
    auto* o = static_cast<Other*>(arg);
    sem_wait(&g_go);
    for (size_t i = 0; i < o->n; ++i) {
        o->v[i] = site(kObj);
        if (!o->v[i]) { ++o->failed; continue; }
        std::memset(o->v[i], patternOf(i), kObj);
    }
    sem_post(&g_other_done);
    sem_wait(&g_resume);   // the pages of the run have been dropped by now
    const uintptr_t rl = g_run_lo.load(), rh = g_run_hi.load();
    for (size_t i = 0; i < o->n; ++i) {
        if (!o->v[i]) continue;
        const auto p = reinterpret_cast<uintptr_t>(o->v[i]);
        if (p + kObj > rl && p < rh) ++o->in_run;
        const auto* b = static_cast<const unsigned char*>(o->v[i]);
        const unsigned char want = patternOf(i);
        for (size_t k = 0; k < kObj; ++k)
            if (b[k] != want) { ++o->damaged; break; }
    }
    return nullptr;
}

int runUnlocked() {
    int failures = 0;
    const size_t n = (size_t{32} << 20) / kObj;
    void** a = pointerArray(n);
    void** b = pointerArray(n);
    if (!a || !b) { std::fprintf(stderr, "FAIL: mmap\n"); return 1; }
    Other other{b, n};
    pthread_t th;
    if (pthread_create(&th, nullptr, otherThread, &other) != 0) { std::fprintf(stderr, "FAIL: pthread_create\n"); return 1; }

    for (size_t i = 0; i < n; ++i) {
        a[i] = site(kObj);
        if (!a[i]) { std::fprintf(stderr, "FAIL: malloc\n"); return 1; }
        static_cast<char*>(a[i])[0] = 1;
    }
    uintptr_t lo, hi;
    range(a, n, &lo, &hi);
    for (size_t i = 0; i < n; ++i) std::free(a[i]);
    g_lo.store(lo);
    g_hi.store(hi);
    g_mode.store(kHoldFirst, std::memory_order_release);

    // (1) The purge comes two ticks (seconds) after the free.
    if (!waitSem(&g_first_handled, 60)) {
        std::fprintf(stderr, "FAIL: no purge of the freed spans within 60 s\n");
        return 1;
    }
    pthread_join(th, nullptr);
    g_mode.store(kOff, std::memory_order_release);
    size_t same_slab = 0;
    for (size_t i = 0; i < n; ++i) {
        const auto p = reinterpret_cast<uintptr_t>(b[i]);
        if (p >= lo && p < hi) ++same_slab;
    }
    std::printf("purge held inside madvise(%p, %zu KiB): the other thread allocated %zu objects, "
                "%zu of them from the freed spans; in time %d, in the dropped run %zu, damaged %zu\n",
                reinterpret_cast<void*>(g_run_lo.load()), (g_run_hi.load() - g_run_lo.load()) >> 10, n,
                same_slab, g_other_in_time.load(), other.in_run, other.damaged);
    if (other.failed) { std::fprintf(stderr, "FAIL: %zu allocations failed\n", other.failed); ++failures; }
    if (same_slab < n / 2) {
        std::fprintf(stderr, "FAIL: only %zu of %zu objects came from the freed spans: the two threads "
                     "do not share a slab and the test shows nothing\n", same_slab, n);
        ++failures;
    }
    if (!g_other_in_time.load()) {
        std::fprintf(stderr, "FAIL: the other thread could not allocate from the slab while the purge "
                     "was inside madvise (10 s): the purge holds the slab's lock across the call\n");
        ++failures;
    }
    if (other.in_run) {
        std::fprintf(stderr, "FAIL: %zu objects were handed out from the range the purge was dropping\n",
                     other.in_run);
        ++failures;
    }
    if (other.damaged) {
        std::fprintf(stderr, "FAIL: %zu objects lost their bytes to the purge\n", other.damaged);
        ++failures;
    }

    // (2) Free the second set and count the purge that follows.
    uintptr_t lo2, hi2;
    range(b, n, &lo2, &hi2);
    g_lo.store(lo2);
    g_hi.store(hi2);
    g_calls.store(0);
    g_bytes.store(0);
    g_mode.store(kCount, std::memory_order_release);
    for (size_t i = 0; i < n; ++i) std::free(b[i]);
    const uint64_t total = n * kObj;
    const bool purged = awaitBytes(total / 10 * 9, 60);
    usleep(300000);   // let the tick finish its last batch
    const uint64_t calls = g_calls.load(), bytes = g_bytes.load();
    std::printf("second purge: %llu KiB of %llu KiB dropped in %llu madvise calls (%llu KiB per call)\n",
                static_cast<unsigned long long>(bytes >> 10), static_cast<unsigned long long>(total >> 10),
                static_cast<unsigned long long>(calls),
                static_cast<unsigned long long>(calls ? (bytes / calls) >> 10 : 0));
    if (!purged) {
        std::fprintf(stderr, "FAIL: the purge dropped %llu KiB of %llu KiB within 60 s\n",
                     static_cast<unsigned long long>(bytes >> 10), static_cast<unsigned long long>(total >> 10));
        ++failures;
    } else if (bytes < calls * (size_t{128} << 10)) {
        std::fprintf(stderr, "FAIL: %llu madvise calls for %llu KiB: neighbouring spans are not merged\n",
                     static_cast<unsigned long long>(calls), static_cast<unsigned long long>(bytes >> 10));
        ++failures;
    }
    return failures;
}

int runFork() {
    int failures = 0;
    const size_t n = (size_t{64} << 20) / kObj;
    void** a = pointerArray(n);
    if (!a) { std::fprintf(stderr, "FAIL: mmap\n"); return 1; }
    for (size_t i = 0; i < n; ++i) {
        a[i] = site(kObj);
        if (!a[i]) { std::fprintf(stderr, "FAIL: malloc\n"); return 1; }
        static_cast<char*>(a[i])[0] = 1;
    }
    uintptr_t lo, hi;
    range(a, n, &lo, &hi);
    for (size_t i = 0; i < n; ++i) std::free(a[i]);
    const uint64_t total = n * kObj;
    g_lo.store(lo);
    g_hi.store(hi);
    g_slow.store(1);
    g_mode.store(kSlow, std::memory_order_release);

    if (!waitSem(&g_go, 60)) {
        std::fprintf(stderr, "FAIL: no purge of the freed spans within 60 s\n");
        return 1;
    }
    timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    const pid_t pid = fork();
    if (pid == 0) {
        // The child: the slab the purge was working on must be usable.
        constexpr size_t kChild = 4096;
        static void* c[kChild];
        for (size_t i = 0; i < kChild; ++i) {
            c[i] = site(kObj);
            if (!c[i]) _exit(3);
            std::memset(c[i], patternOf(i), kObj);
        }
        for (size_t i = 0; i < kChild; ++i) {
            if (static_cast<unsigned char*>(c[i])[kObj - 1] != patternOf(i)) _exit(4);
            std::free(c[i]);
        }
        _exit(0);
    }
    const uint64_t at_return = g_bytes.load();
    clock_gettime(CLOCK_MONOTONIC, &t1);
    g_slow.store(0);
    if (pid < 0) { std::fprintf(stderr, "FAIL: fork: %s\n", std::strerror(errno)); return 1; }
    int status = 0;
    waitpid(pid, &status, 0);
    const double fork_ms = static_cast<double>(t1.tv_sec - t0.tv_sec) * 1e3 +
                           static_cast<double>(t1.tv_nsec - t0.tv_nsec) * 1e-6;
    const bool rest = awaitBytes(total / 10 * 9, 60);
    std::printf("fork() during the purge took %.1f ms; %llu KiB of %llu KiB were dropped when it returned, "
                "%llu KiB in the end; child status 0x%x\n", fork_ms,
                static_cast<unsigned long long>(at_return >> 10), static_cast<unsigned long long>(total >> 10),
                static_cast<unsigned long long>(g_bytes.load() >> 10), static_cast<unsigned>(status));
    if (at_return >= total / 4 * 3) {
        std::fprintf(stderr, "FAIL: fork() waited for the purge: %llu KiB of %llu KiB were dropped when it "
                     "returned (%.0f ms)\n", static_cast<unsigned long long>(at_return >> 10),
                     static_cast<unsigned long long>(total >> 10), fork_ms);
        ++failures;
    }
    // The prepare handler gives up on a tick after 2 s and on the locks
    // after 3 s more: a fork that took that long did not get the purge to
    // stop (with the lock held across each call it took 5.1 s here).
    if (fork_ms >= 1500) {
        std::fprintf(stderr, "FAIL: fork() during the purge took %.0f ms: it gave up waiting for the tick\n",
                     fork_ms);
        ++failures;
    }
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        std::fprintf(stderr, "FAIL: the child could not use the slab (status 0x%x)\n", static_cast<unsigned>(status));
        ++failures;
    }
    if (!rest) {
        std::fprintf(stderr, "FAIL: after the fork the purge dropped %llu KiB of %llu KiB within 60 s\n",
                     static_cast<unsigned long long>(g_bytes.load() >> 10),
                     static_cast<unsigned long long>(total >> 10));
        ++failures;
    }
    return failures;
}
}  // namespace

int main(int argc, char** argv) {
    setvbuf(stdout, nullptr, _IOLBF, 0);
    using in_arena_fn = int (*)(const void*);
    auto in_arena = reinterpret_cast<in_arena_fn>(dlsym(RTLD_DEFAULT, "smash_in_arena"));
    if (!in_arena) {
        std::fprintf(stderr, "FAIL: not running under libsmash (smash_in_arena not found)\n");
        return 1;
    }
    void* probe = site(kObj);
    if (!probe || !in_arena(probe)) {
        std::fprintf(stderr, "FAIL: a %zu-byte block is not in the arena (SMASH_LARGE_ONLY=0 needed)\n", kObj);
        return 1;
    }
    std::free(probe);
    sem_init(&g_go, 0, 0);
    sem_init(&g_other_done, 0, 0);
    sem_init(&g_resume, 0, 0);
    sem_init(&g_first_handled, 0, 0);
    const bool fork_mode = argc > 1 && std::strcmp(argv[1], "fork") == 0;
    const int failures = fork_mode ? runFork() : runUnlocked();
    if (failures) return 1;
    std::printf("OK\n");
    return 0;
}
