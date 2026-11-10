// smash_thread_alloc_test.cpp - smash's own threads never allocate from the
// heap smash manages (issue #7).
//
// The compressor is what takes arena pages away. If one of its threads keeps
// its own data on such a page, the page is compressed like any other and the
// thread faults on it at moments when the fault cannot be served: while it
// holds that page's lock (the lock is not recursive: the handler refuses and
// the SIGSEGV is fatal), while it holds another page's lock (two workers
// doing that to each other deadlock), or inside the fault handler. So a smash
// thread's allocations go to the system allocator. They did only on the slow
// path: every thread got a thread cache when it started, before it marked
// itself, and malloc's fast path looks at nothing but the cache; memalign
// did not look at the mark at all.
//
// This program defines usleep(), madvise() and sched_getcpu(). libsmash calls
// them through its PLT, so its calls come here first, on its own threads: the
// coordinator sleeps with usleep() between ticks, the purge and the decommit
// thread call madvise(), the node workers call sched_getcpu(). Whichever
// thread arrives that the program did not start itself runs a set of
// allocations once, the way a libc function called by smash would, and
// records which of them came from the arena.
//
//   - The same set on the main thread is the control: in full mode every
//     block must be in the arena, in large-only mode the 2 MiB ones, or the
//     set would prove nothing about the threads.
//   - On each of smash's threads none may be.
//   - At least one of smash's threads must have been reached (the
//     coordinator always is), every block must be usable and freeable, and
//     malloc_usable_size must cover it.
//
// The 2 MiB blocks are only requested from usleep(), where the caller holds
// no lock: with the defect, such a request from inside madvise() could need
// a lock the calling thread holds.
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <dlfcn.h>
#include <malloc.h>
#include <new>
#include <sched.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <unistd.h>

namespace {
using in_arena_fn = int (*)(const void*);
in_arena_fn g_in_arena = nullptr;

constexpr int kSmall = 7, kLarge = 3, kItems = kSmall + kLarge;
const char* const kItemName[kItems] = {
    "malloc(100)", "calloc(4, 100)", "realloc(NULL, 300)", "posix_memalign(64, 256)",
    "new char[100]", "realloc(malloc(100), 5000)", "strdup",
    "malloc(2 MiB)", "aligned_alloc(4096, 2 MiB)", "realloc(malloc(100), 2 MiB)"};
constexpr size_t kBig = size_t{2} << 20;

struct Result {
    std::atomic<int> tid{0};
    int source = 0;            // 1 usleep, 2 madvise, 3 sched_getcpu, 0 the main thread
    int items = 0;             // how many of the set were run
    unsigned in_arena = 0;     // bit per item
    unsigned bad = 0;          // bit per item: null, short usable size, lost bytes
    std::atomic<int> done{0};
};
constexpr int kMaxThreads = 128;
Result g_results[kMaxThreads];
std::atomic<int> g_armed{0};

thread_local bool t_app_thread = false;
thread_local bool t_in_probe = false;

// The set. `items` is kSmall or kItems.
__attribute__((noinline)) void runSet(Result* r, int items) {
    void* p[kItems] = {};
    size_t want[kItems] = {100, 400, 300, 256, 100, 5000, 16, kBig, kBig, kBig};
    p[0] = std::malloc(100);
    p[1] = std::calloc(4, 100);
    p[2] = std::realloc(nullptr, 300);
    if (posix_memalign(&p[3], 64, 256) != 0) p[3] = nullptr;
    p[4] = new (std::nothrow) char[100];
    if (void* q = std::malloc(100)) { std::memset(q, 0x5a, 100); p[5] = std::realloc(q, 5000); }
    p[6] = strdup("smash thread probe");
    if (items > kSmall) {
        p[7] = std::malloc(kBig);
        p[8] = std::aligned_alloc(4096, kBig);
        if (void* q = std::malloc(100)) { std::memset(q, 0x5a, 100); p[9] = std::realloc(q, kBig); }
    }
    for (int i = 0; i < items; ++i) {
        if (!p[i]) { r->bad |= 1u << i; continue; }
        if (g_in_arena(p[i])) r->in_arena |= 1u << i;
        if (malloc_usable_size(p[i]) < want[i]) r->bad |= 1u << i;
        auto* b = static_cast<unsigned char*>(p[i]);
        if ((i == 5 || i == 9) && (b[0] != 0x5a || b[99] != 0x5a)) r->bad |= 1u << i;   // realloc kept the bytes
        if (i != 6) { b[0] = 1; b[want[i] - 1] = 2; }                                  // the block is ours to write
    }
    for (int i = 0; i < items; ++i) {
        if (i == 4) delete[] static_cast<char*>(p[i]);
        else std::free(p[i]);
    }
    r->items = items;
}

// Called from the interposed functions, on whatever thread libsmash is on.
void probe(int source) {
    if (t_app_thread || t_in_probe || !g_armed.load(std::memory_order_acquire)) return;
    t_in_probe = true;
    const int tid = static_cast<int>(syscall(SYS_gettid));
    for (Result& r : g_results) {
        int cur = r.tid.load(std::memory_order_acquire);
        if (cur == tid) break;                         // this thread has run it
        if (cur != 0) continue;
        if (!r.tid.compare_exchange_strong(cur, tid)) {
            if (cur == tid) break;
            continue;
        }
        r.source = source;
        runSet(&r, source == 1 ? kItems : kSmall);
        r.done.store(1, std::memory_order_release);
        break;
    }
    t_in_probe = false;
}
}  // namespace

// Default visibility and `used`: the tests build with -fvisibility=hidden and
// LTO, which otherwise drop these as unreferenced locals.
extern "C" __attribute__((visibility("default"), used))
int usleep(useconds_t us) {
    probe(1);
    timespec ts{static_cast<time_t>(us / 1000000), static_cast<long>(us % 1000000) * 1000};
    return nanosleep(&ts, nullptr);
}

extern "C" __attribute__((visibility("default"), used))
int madvise(void* addr, size_t len, int advice) noexcept {
    probe(2);
    return static_cast<int>(syscall(SYS_madvise, addr, len, advice));
}

extern "C" __attribute__((visibility("default"), used))
int sched_getcpu(void) noexcept {
    probe(3);
    unsigned cpu = 0;
    return syscall(SYS_getcpu, &cpu, nullptr, nullptr) == 0 ? static_cast<int>(cpu) : -1;
}

int main() {
    setvbuf(stdout, nullptr, _IOLBF, 0);
    t_app_thread = true;
    g_in_arena = reinterpret_cast<in_arena_fn>(dlsym(RTLD_DEFAULT, "smash_in_arena"));
    if (!g_in_arena) {
        std::fprintf(stderr, "FAIL: not running under libsmash (smash_in_arena not found)\n");
        return 1;
    }
    const char* lo = std::getenv("SMASH_LARGE_ONLY");
    const bool large_only = lo && lo[0] == '1';
    int failures = 0;

    // The control: this thread's blocks are smash's.
    static Result control;
    runSet(&control, kItems);
    const unsigned expect = large_only ? (0x7u << kSmall) : ((1u << kItems) - 1);
    std::printf("main thread (%s): in the arena 0x%03x of 0x%03x expected\n",
                large_only ? "large-only" : "full mode", control.in_arena, expect);
    if (control.bad) {
        std::fprintf(stderr, "FAIL: main thread: allocation failed or was damaged (items 0x%x)\n", control.bad);
        ++failures;
    }
    if ((control.in_arena & expect) != expect) {
        for (int i = 0; i < kItems; ++i)
            if ((expect & ~control.in_arena) >> i & 1)
                std::fprintf(stderr, "FAIL: main thread: %s is not in the arena, so it cannot show where "
                             "a smash thread's would go\n", kItemName[i]);
        ++failures;
    }

    // Give every kind of smash thread a reason to run: small spans to purge,
    // large blocks to decommit, a cold set to compress.
    g_armed.store(1, std::memory_order_release);
    constexpr size_t kSmallObjs = 65536;
    void** v = static_cast<void**>(mmap(nullptr, kSmallObjs * sizeof(void*), PROT_READ | PROT_WRITE,
                                        MAP_PRIVATE | MAP_ANONYMOUS, -1, 0));
    if (v == MAP_FAILED) { std::fprintf(stderr, "FAIL: mmap\n"); return 1; }
    for (size_t i = 0; i < kSmallObjs; ++i) { v[i] = std::malloc(256); if (v[i]) static_cast<char*>(v[i])[0] = 1; }
    for (size_t i = 0; i < kSmallObjs; ++i) std::free(v[i]);
    char* cold[16];
    for (auto& c : cold) {
        c = static_cast<char*>(std::malloc(kBig));
        if (c) for (size_t off = 0; off < kBig; off += 64) c[off] = static_cast<char>(off >> 12);
    }
    for (int round = 0; round < 8; ++round) {
        void* big[8];
        for (auto& b : big) { b = std::malloc(4 * kBig); if (b) std::memset(b, 1, 4096); }
        for (auto& b : big) std::free(b);
        timespec ts{1, 0};
        nanosleep(&ts, nullptr);
    }
    g_armed.store(0, std::memory_order_release);
    timespec settle{0, 200 * 1000 * 1000};
    nanosleep(&settle, nullptr);   // a set in flight finishes
    for (auto& c : cold) std::free(c);

    int reached = 0, complete = 0;
    static const char* const kSource[] = {"", "usleep", "madvise", "sched_getcpu"};
    for (Result& r : g_results) {
        const int tid = r.tid.load();
        if (!tid) continue;
        ++reached;
        if (!r.done.load()) {
            std::fprintf(stderr, "FAIL: thread %d (from %s) did not finish its allocations\n", tid, kSource[r.source]);
            ++failures;
            continue;
        }
        ++complete;
        std::printf("smash thread %d (from %s, %d allocations): in the arena 0x%03x, failed 0x%03x\n",
                    tid, kSource[r.source], r.items, r.in_arena, r.bad);
        for (int i = 0; i < r.items; ++i) {
            if (r.in_arena >> i & 1) {
                std::fprintf(stderr, "FAIL: thread %d (from %s): %s came from the arena\n", tid, kSource[r.source],
                             kItemName[i]);
                ++failures;
            }
            if (r.bad >> i & 1) {
                std::fprintf(stderr, "FAIL: thread %d (from %s): %s failed, was too small or lost its bytes\n", tid,
                             kSource[r.source], kItemName[i]);
                ++failures;
            }
        }
    }
    std::printf("%d of smash's threads reached, %d finished\n", reached, complete);
    if (reached == 0) {
        std::fprintf(stderr, "FAIL: none of smash's threads called usleep, madvise or sched_getcpu in 8 s: "
                     "the test reached nothing\n");
        ++failures;
    }
    if (failures) return 1;
    std::printf("OK\n");
    return 0;
}
