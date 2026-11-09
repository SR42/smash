// subarena_merge_test.cpp - a sub-arena's dirty and clean free runs merge.
//
// A sub-arena frees a block without decommitting it, and its purger decommits
// such pages later. Dirty and clean runs used to be kept apart: a chunk could
// be nothing but free pages and still serve no request longer than its longest
// run of one kind, so a miss carved fresh region and the arena's high-water mark,
// which every per-tick scan walks, only went up. Now a run can hold both, with
// a dirty bit per page.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <sched.h>
#include <unistd.h>

#define CHECK(cond, ...)                         \
    do {                                         \
        if (!(cond)) {                           \
            std::printf("FAIL: " __VA_ARGS__);   \
            std::printf("\n");                   \
            return 1;                            \
        }                                        \
    } while (0)

using Info = int (*)(uint64_t*, uint64_t*);
static Info info;

static uint64_t dirtyBytes() {
    uint64_t d = 0, p = 0;
    info(&d, &p);
    return d;
}

static bool waitNoDirty(int ms) {
    for (int i = 0; i < ms / 5; ++i) {
        if (dirtyBytes() == 0) return true;
        usleep(5000);
    }
    return false;
}

int main() {
    info = reinterpret_cast<Info>(dlsym(RTLD_DEFAULT, "smash_subarena_info"));
    if (!info) { std::fprintf(stderr, "FAIL: not running under libsmash\n"); return 1; }
    // One CPU, so one sub-arena serves every block below.
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(sched_getcpu(), &set);
    CHECK(sched_setaffinity(0, sizeof(set), &set) == 0, "cannot pin");

    const size_t mib = 1u << 20;
    auto* a = static_cast<char*>(std::malloc(2 * mib));
    auto* b = static_cast<char*>(std::malloc(2 * mib));
    auto* guard = static_cast<char*>(std::malloc(2 * mib));
    CHECK(a && b && guard, "malloc failed");
    CHECK(b == a + 2 * mib && guard == b + 2 * mib, "blocks are not neighbours: %p %p %p", a, b, guard);
    std::memset(a, 0xAA, 2 * mib);
    std::memset(b, 0xBB, 2 * mib);
    std::memset(guard, 0xCC, 2 * mib);

    // `a` is purged: clean. `b` is freed behind it: dirty. Neither holds 4 MiB,
    // together they do.
    std::free(a);
    CHECK(waitNoDirty(5000), "%llu bytes still dirty after 5 s", (unsigned long long)dirtyBytes());
    std::free(b);
    CHECK(dirtyBytes() == 2 * mib, "%llu bytes dirty after freeing 2 MiB", (unsigned long long)dirtyBytes());
    auto* c = static_cast<unsigned char*>(std::calloc(1, 4 * mib));
    CHECK(c, "calloc failed");
    std::printf("4 MiB over a clean and a dirty run: %s\n",
                c == reinterpret_cast<unsigned char*>(a) ? "served from them" : "fresh pages");
    CHECK(c == reinterpret_cast<unsigned char*>(a), "a clean run and a dirty run next to it did not merge: "
          "4 MiB at %p, the runs are at %p", c, a);

    // calloc must see the dirty half for what it is, and only it counts as dirty.
    for (size_t i = 0; i < 4 * mib; ++i)
        CHECK(c[i] == 0, "calloc byte %zu reads %#x", i, c[i]);
    CHECK(dirtyBytes() == 0, "%llu bytes still counted dirty after their pages were taken",
          (unsigned long long)dirtyBytes());

    // The other way round: dirty first, clean behind, a block across part of each.
    std::free(c);    // 4 MiB dirty again
    CHECK(waitNoDirty(5000), "%llu bytes still dirty after 5 s", (unsigned long long)dirtyBytes());
    auto* d = static_cast<char*>(std::malloc(2 * mib));
    CHECK(d == a, "2 MiB did not come from the front of the run: %p, %p", d, a);
    std::memset(d, 0xDD, 2 * mib);
    std::free(d);    // [a, a + 2 MiB) dirty, the next 2 MiB clean
    CHECK(dirtyBytes() == 2 * mib, "%llu bytes dirty", (unsigned long long)dirtyBytes());
    auto* e = static_cast<unsigned char*>(std::malloc(3 * mib));
    CHECK(e == reinterpret_cast<unsigned char*>(a), "3 MiB did not come from the merged run: %p, %p", e, a);
    CHECK(dirtyBytes() == 0, "%llu bytes dirty after the block took them all", (unsigned long long)dirtyBytes());
    std::free(e);
    CHECK(guard[0] == static_cast<char>(0xCC), "the guard block changed");
    std::free(guard);
    std::printf("PASS\n");
    return 0;
}
