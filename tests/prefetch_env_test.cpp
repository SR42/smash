// prefetch_env_test.cpp - SMASH_PREFETCH_WINDOW must set how many compressed
// neighbours a fault restores on each side of the faulting page.
//
// Added because the window was a build option only (-DSMASH_PREFETCH_WINDOW,
// default 2): a fault restores up to 4 neighbours one page at a time, which
// measured 53 us per fault on random access against 15 us with the window
// compiled out, and an application could not turn it off without a rebuild.
//
// Each case runs in a child process (the variable is read at startup). The
// child writes 16 MiB, every page different and compressible, waits until no
// page of it is resident, reads one byte in the middle, and counts the
// block's resident pages: the faulting page plus the neighbours prefetched.
// Checks:
//   - unset: 5 pages (the default window, 2 each side);
//   - SMASH_PREFETCH_WINDOW=0: 1 page; =4: 9 pages; =1000: capped at 64
//     each side, 129 pages;
//   - SMASH_PREFETCH_WINDOW=abc: a [smash WARN] and the default 5;
//   - every child reads its block back intact.
#include <cerrno>
#include <climits>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <string>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

namespace {

constexpr size_t kLen = 16u << 20, kPage = 4096, kPages = kLen / kPage;

unsigned char want(size_t page, size_t i) {
    return i < 512 ? static_cast<unsigned char>(page * 7 + i / 64) : 0;
}

size_t resident(const unsigned char* p) {
    std::vector<unsigned char> v(kPages);
    if (mincore(const_cast<unsigned char*>(p), kLen, v.data()) != 0) return kPages;
    size_t r = 0;
    for (auto c : v) r += c & 1;
    return r;
}

int child() {
    auto* b = static_cast<unsigned char*>(malloc(kLen));
    for (size_t p = 0; p < kPages; ++p)
        for (size_t i = 0; i < kPage; ++i) b[p * kPage + i] = want(p, i);
    size_t left = kPages;
    for (int s = 0; s < 240 && (left = resident(b)) > 0; ++s) usleep(250 * 1000);
    const size_t mid = kPages / 2;
    const volatile unsigned char got = b[mid * kPage + 1];
    const size_t after = left == 0 ? resident(b) : 0;
    size_t bad = got != want(mid, 1);
    for (size_t p = 0; p < kPages; ++p)
        for (size_t i = 0; i < kPage; ++i) bad += b[p * kPage + i] != want(p, i);
    std::printf("left=%zu after=%zu bad=%zu\n", left, after, bad);
    free(b);
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

struct Result { bool ok = false; size_t left = 0, after = 0, bad = 0; bool warned = false; };

Result run(const char* self, const char* env) {
    Result r;
    // stderr too: smash reads the variable, and warns, before the child runs.
    const std::string cmd = std::string("timeout 120 env -u SMASH_PREFETCH_WINDOW ") + env + " '" + self +
                            "' child 2>&1";
    int st = 0;
    const std::string out = runChild(cmd, &st);
    r.warned = out.find("[smash WARN] SMASH_PREFETCH_WINDOW") != std::string::npos;
    size_t at = out.rfind("\nleft=");
    at = at != std::string::npos ? at + 1 : (out.compare(0, 5, "left=") == 0 ? 0 : std::string::npos);
    if (at != std::string::npos &&
        std::sscanf(out.c_str() + at, "left=%zu after=%zu bad=%zu", &r.left, &r.after, &r.bad) == 3)
        r.ok = true;
    r.ok = r.ok && st == 0;
    std::printf("%-28s -> %zu pages resident after one fault (%zu left before it), %zu bad bytes, warned=%d "
                "(status %d)\n", env, r.after, r.left, r.bad, r.warned ? 1 : 0, st);
    if (!r.ok) showTail(out);
    return r;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "child") == 0) return child();
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) { std::printf("SKIP: not running under libsmash\n"); return 0; }
    char self[PATH_MAX];
    const ssize_t n = readlink("/proc/self/exe", self, sizeof(self) - 1);
    if (n <= 0) return 1;
    self[n] = '\0';

    struct Case { const char* env; size_t pages; bool warn; };
    const Case cases[] = {
        {"", 5, false},
        {"SMASH_PREFETCH_WINDOW=0", 1, false},
        {"SMASH_PREFETCH_WINDOW=4", 9, false},
        {"SMASH_PREFETCH_WINDOW=1000", 129, false},
        {"SMASH_PREFETCH_WINDOW=abc", 5, true},
    };
    int failures = 0;
    for (const Case& c : cases) {
        const Result r = run(self, c.env);
        std::string why;
        if (!r.ok) why = "the child did not report";
        else if (r.left) why = std::to_string(r.left) + " pages never left RSS";
        else if (r.bad) why = std::to_string(r.bad) + " bytes read back wrong";
        else if (r.after != c.pages)
            why = std::to_string(r.after) + " pages resident after the fault, want " + std::to_string(c.pages);
        else if (r.warned != c.warn) why = c.warn ? "no [smash WARN]" : "an unexpected [smash WARN]";
        if (!why.empty()) {
            std::printf("FAIL: %s: %s\n", *c.env ? c.env : "unset", why.c_str());
            ++failures;
        }
    }
    if (failures) return 1;
    std::printf("prefetch_env_test: all checks passed\n");
    return 0;
}
