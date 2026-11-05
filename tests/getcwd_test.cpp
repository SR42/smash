// getcwd_test.cpp - getcwd(NULL, size) must keep glibc's contract under
// libsmash.
//
// smash wraps getcwd so that the kernel writes the path into a stack buffer
// (a fresh heap buffer can be a watched or compressed smash page, and the
// syscall then fails with EFAULT). For buf == NULL it returned strdup(path)
// whatever `size` was. glibc documents that with buf == NULL and size > 0
// the buffer is `size` bytes, and a path that does not fit is ERANGE:
//
//     char* p = getcwd(NULL, PATH_MAX);
//     strcat(p, "/file");          // a heap overflow under smash only
//
// And the stack buffer is PATH_MAX bytes, so a working directory longer than
// that failed outright, where glibc's getcwd(NULL, 0) builds the path anyway.
// Every check here passes under glibc alone.
#include <cerrno>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <malloc.h>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

namespace {
int g_failures = 0;
#define FAIL(...) do { std::printf("FAIL: " __VA_ARGS__); std::printf("\n"); ++g_failures; } while (0)
}  // namespace

int main() {
    char here[PATH_MAX];
    if (!getcwd(here, sizeof(here))) { std::printf("FAIL: getcwd into a stack buffer\n"); return 1; }
    const size_t len = std::strlen(here) + 1;

    // size == 0: as large as needed.
    char* p = getcwd(nullptr, 0);
    if (!p || std::strcmp(p, here) != 0) FAIL("getcwd(NULL, 0) = %s", p ? p : "NULL");
    std::free(p);

    // size > 0: a buffer of `size` bytes, writable to its end.
    for (size_t size : {len, len + 1, size_t{256}, size_t{PATH_MAX}, size_t{3 * PATH_MAX}}) {
        if (size < len) continue;
        p = getcwd(nullptr, size);
        if (!p || std::strcmp(p, here) != 0) {
            FAIL("getcwd(NULL, %zu) = %s", size, p ? p : "NULL");
        } else if (malloc_usable_size(p) < size) {
            FAIL("getcwd(NULL, %zu) returned a %zu-byte buffer: the caller may write %zu", size,
                 malloc_usable_size(p), size);
        } else {
            std::memset(p + len, 'x', size - len);   // the caller's to use
            p[size - 1] = '\0';
        }
        std::free(p);
    }

    // size > 0 but too small for the path: ERANGE.
    for (size_t size : {size_t{1}, len - 1}) {
        errno = 0;
        p = getcwd(nullptr, size);
        if (p || errno != ERANGE) FAIL("getcwd(NULL, %zu) = %p errno %d, want NULL with ERANGE", size, p, errno);
        std::free(p);
    }
    std::printf("getcwd(NULL, size): size 0 as needed, size > 0 that many bytes, too small ERANGE\n");

    // A working directory longer than PATH_MAX.
    char tmpl[] = "/tmp/getcwd_test.XXXXXX";
    if (!mkdtemp(tmpl) || chdir(tmpl) != 0) { FAIL("setup: mkdtemp/chdir"); return 1; }
    const std::string comp(200, 'd');
    std::string deep = tmpl;
    int depth = 0;
    while (deep.size() <= PATH_MAX + 200) {
        if (mkdir(comp.c_str(), 0700) != 0 || chdir(comp.c_str()) != 0) { FAIL("setup: mkdir depth %d", depth); break; }
        deep += "/" + comp;
        ++depth;
    }
    p = getcwd(nullptr, 0);
    if (!p || deep != p)
        FAIL("getcwd(NULL, 0) in a %zu-byte working directory = %s (errno %d)", deep.size(),
             p ? "a different path" : "NULL", errno);
    else
        std::printf("getcwd(NULL, 0): the %zu-byte working directory\n", deep.size());
    std::free(p);
    // Clean up from the bottom.
    for (int i = 0; i < depth; ++i) {
        if (chdir("..") != 0) break;
        rmdir(comp.c_str());
    }
    if (chdir(here) == 0) rmdir(tmpl);

    if (g_failures) { std::printf("getcwd_test: %d FAILURES\n", g_failures); return 1; }
    std::printf("getcwd_test: all checks passed\n");
    return 0;
}
