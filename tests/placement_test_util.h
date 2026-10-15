// placement_test_util.h - shared by the full-mode placement tests
// (test_callsite_temperature, test_cooling_close, test_thrash_backoff): run a
// case in a fresh child process, read its output, and ask smash what the
// compressor thinks of a page.
#pragma once

#include <cerrno>
#include <climits>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <string>
#include <time.h>
#include <unistd.h>

namespace placement {

constexpr size_t kPage = 4096;

// PageState values reported by smash_page_info (src/vm/page_state.h).
enum : int { kEmpty = 0, kActive = 1, kCompressed = 2, kCompressing = 3, kMonitoring = 4, kShadow = 5 };

using PageInfoFn = int (*)(const void*, int*, int*, int*);
using ArenaForFn = int (*)(const void*);

inline PageInfoFn pageInfoFn() {
    return reinterpret_cast<PageInfoFn>(dlsym(RTLD_DEFAULT, "smash_page_info"));
}
inline ArenaForFn arenaForFn() {
    return reinterpret_cast<ArenaForFn>(dlsym(RTLD_DEFAULT, "smash_arena_for"));
}

// True if the compressor holds the page's data compressed (a shadow copy
// counts: the page is past the decision this test is about).
inline bool compressedState(int st) { return st == kCompressed || st == kShadow; }

inline uintptr_t pageOf(const void* p) { return reinterpret_cast<uintptr_t>(p) & ~(kPage - 1); }

inline double nowSec() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

// Runs `cmd` and returns its output, read with read(2) into a stack buffer,
// never through a stdio stream (see test_stdio_buffer for why that once lost
// a child's output in full mode). *status is pclose()'s.
inline std::string runChild(const std::string& cmd, int* status) {
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

// The last lines of a child's output, to show why it failed.
inline void showTail(const std::string& out) {
    size_t at = out.size(), lines = 0;
    while (at > 0 && lines < 8) {
        at = out.rfind('\n', at - 1);
        if (at == std::string::npos) { at = 0; break; }
        ++lines;
    }
    std::printf("    child output (last lines):\n%s\n", out.substr(at).c_str());
}

// The line of `out` starting with `tag`, or "" (the child prints its result
// as one tagged line; smash may print warnings around it).
inline std::string taggedLine(const std::string& out, const char* tag) {
    size_t at = out.rfind(std::string("\n") + tag);
    if (at != std::string::npos) ++at;
    else if (out.compare(0, std::strlen(tag), tag) == 0) at = 0;
    else return "";
    const size_t end = out.find('\n', at);
    return out.substr(at, end == std::string::npos ? std::string::npos : end - at);
}

inline bool selfPath(char (&self)[PATH_MAX]) {
    const ssize_t n = readlink("/proc/self/exe", self, sizeof(self) - 1);
    if (n <= 0) return false;
    self[n] = '\0';
    return true;
}

}  // namespace placement
