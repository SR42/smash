// cold_arena_env_test.cpp - SMASH_COLD_ARENA_FEEDBACK=1 must route a call
// site's new allocations to its cold sub-arena once its pages keep getting
// compressed, and only then.
//
// Added because the feedback was a build option only
// (-DSMASH_COLD_ARENA_FEEDBACK): with it, a (call-site arena, size class)
// whose pages have been compressed SMASH_COLD_ARENA_THRESHOLD times (default
// 8) sends later allocations to a second slab, so objects that go cold stop
// sharing pages with ones that stay hot. Full mode only: SMASH_LARGE_ONLY
// passes these sizes to glibc.
//
// Each case runs in a child process (the variables are read at startup) with
// SMASH_NUM_ARENAS=8 unless the case sets it. The child allocates 8 MiB of
// 256-byte objects from one call site, leaves them idle until most of their
// pages are out of RAM, then allocates 512 more from the same call site and
// prints the arenas (smash_arena_for) of the first block and of the last new
// object, and the stats line's cold_arenas=. The last, not the first: the
// thread cache serves slots the hot slab handed out before the flip first.
// Checks:
//   - feedback on: the new objects come from arena old + 8, cold_arenas >= 1;
//   - feedback off (unset): same arena, cold_arenas=0;
//   - SMASH_COLD_ARENA_THRESHOLD=1000000: the flip never happens;
//   - SMASH_NUM_ARENAS=128 with feedback: the hot arenas are capped at 64 so
//     the cold half fits the slab table (old < 64, new = old + 64);
//   - every child reads its objects back intact;
//   - a hot call site is never moved: 2048 objects from another call site
//     (routed to another arena) are written every 250 ms throughout, and the
//     last of 512 new objects from it comes from its old arena in every case,
//     with none of its pages out of RAM. Without this, a flip that moved every
//     site, or a counter shared across sites, would pass. Skipped when the
//     child reports track=none (arm64 before Linux 6.7: no soft-dirty, no
//     userfaultfd write-protect): with no write signal the compressor finds
//     cold memory by compressing pages and re-checking them, hot ones too, so
//     the feedback sees the hot site's pages compressed and moves it.
#include <cerrno>
#include <climits>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <signal.h>
#include <string>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

namespace {

constexpr size_t kObj = 256, kCount = (8u << 20) / kObj, kPage = 4096;

__attribute__((noinline)) void* site() {
    void* p = malloc(kObj);
    asm volatile("" ::: "memory");  // keep this call site a real call
    return p;
}

// Candidates for the hot call site: the first whose arena differs from the
// cold site's is used.
static volatile uintptr_t g_hot_sink;
#define HOT_SITE(n)                                            \
    __attribute__((noinline)) void* hot_site_##n() {          \
        void* p = malloc(kObj);                                \
        g_hot_sink = reinterpret_cast<uintptr_t>(p) + (n);    \
        return p;                                              \
    }
HOT_SITE(0) HOT_SITE(1) HOT_SITE(2) HOT_SITE(3) HOT_SITE(4) HOT_SITE(5)
using SiteFn = void* (*)();
SiteFn kHotSites[] = {hot_site_0, hot_site_1, hot_site_2, hot_site_3, hot_site_4, hot_site_5};
constexpr size_t kHotCount = 2048;

unsigned char want(size_t obj, size_t i) { return i < 64 ? static_cast<unsigned char>(obj + i) : 0; }

int child() {
    using ArenaFor = int (*)(const void*);
    auto arena_for = reinterpret_cast<ArenaFor>(dlsym(RTLD_DEFAULT, "smash_arena_for"));
    if (!arena_for) return 2;
    char path[] = "/tmp/smash_coldarena_XXXXXX";
    const int log = mkstemp(path);
    if (log < 0) {
        // Says why: one child of test_codec_env, run as root mid-suite,
        // exited 2 with no output at all (twice; 30 of 30 runs on their own
        // passed). This is the only exit 2 in the child.
        std::printf("child: mkstemp failed: %s\n", std::strerror(errno));
        return 2;
    }
    unlink(path);
    const int saved = dup(2);
    dup2(log, 2);

    std::vector<unsigned char*> objs(kCount);
    for (size_t k = 0; k < kCount; ++k) {
        objs[k] = static_cast<unsigned char*>(site());
        for (size_t i = 0; i < kObj; ++i) objs[k][i] = want(k, i);
    }
    const int old_arena = arena_for(objs[0]);

    // The hot site: kept hot by a write to each object every 250 ms below.
    SiteFn hot_site = kHotSites[0];
    for (SiteFn h : kHotSites) {
        void* p = h();
        const int a = arena_for(p);
        free(p);
        if (a != old_arena) {
            hot_site = h;
            break;
        }
    }
    std::vector<unsigned char*> hot(kHotCount);
    for (size_t k = 0; k < kHotCount; ++k) {
        hot[k] = static_cast<unsigned char*>(hot_site());
        for (size_t i = 0; i < kObj; ++i) hot[k][i] = want(k + 7, i);
    }
    const int hot_old = arena_for(hot[0]);
    unsigned round = 0;
    auto heatUp = [&] {
        ++round;
        for (auto* h : hot) h[kObj - 1] = static_cast<unsigned char>(round | 1);
    };

    // Wait until at least half of the objects' pages are out of RAM.
    std::vector<uintptr_t> pages;
    for (auto* o : objs) {
        const uintptr_t pg = reinterpret_cast<uintptr_t>(o) & ~(kPage - 1);
        if (pages.empty() || pages.back() != pg) pages.push_back(pg);
    }
    size_t out = 0;
    for (int s = 0; s < 240; ++s) {
        out = 0;
        unsigned char v;
        for (uintptr_t pg : pages)
            if (mincore(reinterpret_cast<void*>(pg), kPage, &v) == 0 && !(v & 1)) ++out;
        if (out * 2 >= pages.size()) break;
        heatUp();
        usleep(250 * 1000);
    }
    for (int i = 0; i < 4; ++i) {  // let the compressor's callback see the last compressions
        heatUp();
        usleep(250 * 1000);
    }

    // The thread cache can still hold slots the hot slab handed out before
    // the flip, and serves them first: judge by the last of 512.
    std::vector<void*> fresh(512);
    size_t from_old = 0;
    for (void*& f : fresh) {
        f = site();
        from_old += arena_for(f) == old_arena;
    }
    const int new_arena = arena_for(fresh.back());
    for (void* f : fresh) free(f);
    std::vector<void*> hot_fresh(512);
    for (void*& f : hot_fresh) f = hot_site();
    const int hot_new = arena_for(hot_fresh.back());
    for (void* f : hot_fresh) free(f);
    size_t hot_out = 0;
    {
        std::vector<uintptr_t> hp;
        for (auto* h : hot) {
            const uintptr_t pg = reinterpret_cast<uintptr_t>(h) & ~(kPage - 1);
            if (hp.empty() || hp.back() != pg) hp.push_back(pg);
        }
        unsigned char v;
        for (uintptr_t pg : hp)
            if (mincore(reinterpret_cast<void*>(pg), kPage, &v) == 0 && !(v & 1)) ++hot_out;
    }

    raise(SIGUSR2);
    usleep(100 * 1000);
    dup2(saved, 2);
    std::string text;
    char buf[8192];
    lseek(log, 0, SEEK_SET);
    for (ssize_t n; (n = read(log, buf, sizeof(buf))) > 0;) text.append(buf, static_cast<size_t>(n));
    long cold = -1;
    const size_t at = text.rfind(" cold_arenas=");
    if (at != std::string::npos) cold = std::atol(text.c_str() + at + 13);
    char track[16] = "?";
    if (const size_t t = text.rfind(" track="); t != std::string::npos) std::sscanf(text.c_str() + t + 7, "%15s", track);

    size_t bad = 0;
    for (size_t k = 0; k < kCount; ++k)
        for (size_t i = 0; i < kObj; ++i) bad += objs[k][i] != want(k, i);
    for (size_t k = 0; k < kHotCount; ++k)
        for (size_t i = 0; i + 1 < kObj; ++i) bad += hot[k][i] != want(k + 7, i);
    for (auto* o : objs) free(o);
    for (auto* h : hot) free(h);
    std::printf("old=%d new=%d cold=%ld out=%zu/%zu bad=%zu from_old=%zu hot_old=%d hot_new=%d hot_out=%zu track=%s\n",
                old_arena, new_arena, cold, out, pages.size(), bad, from_old, hot_old, hot_new, hot_out, track);
    return 0;
}

// Runs `cmd` and returns its output (the child's stdout and stderr), read with
// read(2) into a stack buffer, never through a stdio stream: a stream's buffer
// is malloc'd, in full mode that is smash memory, and a read blocked while the
// child runs can find it compressed. glibc's internal read then gets EFAULT and
// the output is lost (seen as root: test_cold_arena_env's children "did not
// report"). On exit, *status is pclose()'s.
std::string runChild(const std::string& cmd, int* status) {
    std::string out;
    *status = -1;
    FILE* p = popen(cmd.c_str(), "r");
    if (!p) return out;
    char buf[4096];
    for (ssize_t n; (n = read(fileno(p), buf, sizeof(buf))) != 0;) {
        if (n < 0) {
            if (errno == EINTR) continue;
            break;
        }
        out.append(buf, static_cast<size_t>(n));
    }
    *status = pclose(p);
    return out;
}

// The last lines of a child's output, to show why it did not report.
void showTail(const std::string& out) {
    size_t at = out.size(), lines = 0;
    while (at > 0 && lines < 6) {
        at = out.rfind('\n', at - 1);
        if (at == std::string::npos) { at = 0; break; }
        ++lines;
    }
    std::printf("    child output (last lines, status above):\n%s\n", out.substr(at).c_str());
}

struct Result {
    bool ok = false;
    int old_arena = -1, new_arena = -1;
    long cold = -1;
    size_t out = 0, pages = 0, bad = 0, from_old = 0, hot_out = 0;
    int hot_old = -1, hot_new = -1;
    char track[16] = "?";
};

Result run(const char* self, const char* env) {
    Result r;
    const std::string cmd = std::string("timeout 120 env -u SMASH_COLD_ARENA_FEEDBACK -u SMASH_COLD_ARENA_THRESHOLD "
                                        "SMASH_NUM_ARENAS=8 ") + env + " '" + self + "' child 2>&1";
    int st = 0;
    const std::string out = runChild(cmd, &st);
    // The report line starts with "old="; it also ends with "from_old=".
    size_t at = out.rfind("\nold=");
    at = at != std::string::npos ? at + 1 : (out.compare(0, 4, "old=") == 0 ? 0 : std::string::npos);
    if (at != std::string::npos &&
        std::sscanf(out.c_str() + at,
                    "old=%d new=%d cold=%ld out=%zu/%zu bad=%zu from_old=%zu hot_old=%d hot_new=%d hot_out=%zu track=%15s",
                    &r.old_arena, &r.new_arena, &r.cold, &r.out, &r.pages, &r.bad, &r.from_old, &r.hot_old,
                    &r.hot_new, &r.hot_out, r.track) == 11)
        r.ok = true;
    r.ok = r.ok && st == 0;
    std::printf("%-58s -> arena %d then %d (%zu of 512 new still from %d), cold_arenas=%ld, %zu of %zu pages out, "
                "%zu bad bytes; hot site: arena %d then %d, %zu pages out; track=%s (status %d)\n", env,
                r.old_arena, r.new_arena, r.from_old, r.old_arena, r.cold, r.out, r.pages, r.bad, r.hot_old,
                r.hot_new, r.hot_out, r.track, st);
    if (!r.ok) showTail(out);
    return r;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "child") == 0) return child();
    if (!dlsym(RTLD_DEFAULT, "smash_arena_for")) { std::printf("SKIP: not running under libsmash\n"); return 0; }
    char self[PATH_MAX];
    const ssize_t n = readlink("/proc/self/exe", self, sizeof(self) - 1);
    if (n <= 0) return 1;
    self[n] = '\0';

    struct Case { const char* env; int shift; bool flipped; };
    const Case cases[] = {
        {"", 0, false},
        {"SMASH_COLD_ARENA_FEEDBACK=1", 8, true},
        {"SMASH_COLD_ARENA_FEEDBACK=1 SMASH_COLD_ARENA_THRESHOLD=1000000", 0, false},
        {"SMASH_COLD_ARENA_FEEDBACK=1 SMASH_NUM_ARENAS=128", 64, true},
    };
    int failures = 0, hot_skipped = 0;
    for (const Case& c : cases) {
        const Result r = run(self, c.env);
        const bool no_tracking = std::strcmp(r.track, "none") == 0;
        std::string why;
        if (!r.ok) why = "the child did not report";
        else if (r.out * 2 < r.pages) why = "the objects' pages never left RAM";
        else if (r.bad) why = std::to_string(r.bad) + " bytes read back wrong";
        else if (r.old_arena < 0 || r.new_arena != r.old_arena + c.shift)
            why = "arena " + std::to_string(r.old_arena) + " then " + std::to_string(r.new_arena) + ", want +" +
                  std::to_string(c.shift);
        else if (c.shift == 64 && r.old_arena >= 64) why = "hot arena " + std::to_string(r.old_arena) + " above 63";
        else if (c.flipped ? r.cold < 1 : r.cold != 0) why = "cold_arenas=" + std::to_string(r.cold);
        else if (r.hot_old == r.old_arena) why = "no hot call site routed away from the cold one's arena";
        else if (no_tracking) ++hot_skipped;  // see the header: the hot site looks cold
        else if (r.hot_new != r.hot_old)
            why = "the hot call site moved from arena " + std::to_string(r.hot_old) + " to " +
                  std::to_string(r.hot_new);
        else if (r.hot_out) why = std::to_string(r.hot_out) + " hot pages out of RAM while written every 250 ms";
        if (!why.empty()) {
            std::printf("FAIL: %s: %s\n", *c.env ? c.env : "unset", why.c_str());
            ++failures;
        }
    }
    if (failures) return 1;
    if (hot_skipped)
        std::printf("hot call site check skipped in %d of 4 cases: track=none (no write tracking on this kernel; "
                    "SMASH_SOFTDIRTY=0 enables it)\n", hot_skipped);
    std::printf("cold_arena_env_test: all checks passed\n");
    return 0;
}
