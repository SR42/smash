// stdio_buffer_test.cpp - in full mode, every stdio read must survive a
// stream whose data arrives after its buffer could have gone cold, not only
// the functions smash wraps.
//
// Added because glibc reads into a stream's buffer with its own internal
// read(), which never reaches smash's EFAULT retry, and allocates that buffer
// with malloc, which in full mode is smash memory the compressor owns. smash
// recovered only inside the stdio functions it wraps (fgets, fread, fgetc,
// getc and now their fortified forms); every other reader lost the data with
// errno 14, each one in a fresh process: getline, getdelim, fscanf,
// getc_unlocked, fgets_unlocked, fread_unlocked. The buffer now comes from the
// system allocator (glibc allocates it in _IO_file_doallocate; smash sends
// that function's allocations there).
//
// Each case runs in its own child process, so no earlier stream has warmed
// the memory the next buffer lands on. The child writes 8 MiB of heap, frees
// half, waits 3 s (cold timeout 1 s), then popen()s a command that sleeps 2 s
// and prints "12345 hello-from-child", and reads it with one function.
// Checks, for each of fgets, getline, getdelim, fscanf, getc_unlocked,
// fgets_unlocked, fread_unlocked:
//   - the data arrives;
//   - the stream's buffer is not in smash's arena (smash_in_arena == 0);
//   - a 4 MiB new[] and a 4 MiB malloc right after are in it: smash's
//     versioned operator new entries did not record their caller, so with
//     the stdio routing the block followed the leftover stdio call site to the
//     system allocator.
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <string>
#include <sys/wait.h>
#include <unistd.h>

namespace {

using InArena = int (*)(const void*);

int child(const char* fn) {
    auto in_arena = reinterpret_cast<InArena>(dlsym(RTLD_DEFAULT, "smash_in_arena"));
    static void* junk[2048];
    for (void*& j : junk) {
        j = malloc(4096);
        std::memset(j, 1, 4096);
    }
    for (size_t i = 0; i < 2048; i += 2) free(junk[i]);
    sleep(3);

    FILE* p = popen("sleep 2; echo 12345 hello-from-child", "r");
    if (!p) return 2;
    errno = 0;
    bool ok = false;
    const std::string f = fn;
    if (f == "fgets") {
        char b[128];
        ok = fgets(b, sizeof b, p) && std::strstr(b, "hello");
    } else if (f == "getline") {
        char* l = nullptr;
        size_t n = 0;
        ok = getline(&l, &n, p) > 0 && std::strstr(l, "hello");
        free(l);
    } else if (f == "getdelim") {
        char* l = nullptr;
        size_t n = 0;
        ok = getdelim(&l, &n, '\n', p) > 0 && std::strstr(l, "hello");
        free(l);
    } else if (f == "fscanf") {
        int v = 0;
        ok = fscanf(p, "%d", &v) == 1 && v == 12345;
    } else if (f == "getc_unlocked") {
        ok = getc_unlocked(p) == '1';
    } else if (f == "fgets_unlocked") {
        char b[128];
        ok = fgets_unlocked(b, sizeof b, p) && std::strstr(b, "hello");
    } else if (f == "fread_unlocked") {
        char b[16];
        ok = fread_unlocked(b, 1, 10, p) == 10 && std::memcmp(b, "12345 hell", 10) == 0;
    }
    const int e = errno;
    const int buf_in_arena = p->_IO_buf_base && in_arena ? in_arena(p->_IO_buf_base) : -1;
    // The last allocation was the stream's buffer, made from glibc. A C++
    // allocation through libstdc++'s versioned operator new must still go to
    // the arena: those entries did not record their caller, smash read the
    // stdio call site left over and sent the block to the system allocator.
    char* big_new = new char[4u << 20];
    void* big_malloc = malloc(4u << 20);
    const int new_in_arena = in_arena ? in_arena(big_new) : -1;
    const int malloc_in_arena = in_arena ? in_arena(big_malloc) : -1;
    delete[] big_new;
    free(big_malloc);
    std::printf("%-15s %s errno=%d buffer_in_arena=%d new_in_arena=%d malloc_in_arena=%d\n", fn,
                ok ? "ok" : "LOST", e, buf_in_arena, new_in_arena, malloc_in_arena);
    pclose(p);
    return ok && buf_in_arena == 0 && new_in_arena == 1 && malloc_in_arena == 1 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
    setvbuf(stdout, nullptr, _IONBF, 0);
    if (argc > 1) return child(argv[1]);
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) { std::fprintf(stderr, "FAIL: not running under libsmash\n"); return 1; }
    const char* fns[] = {"fgets", "getline", "getdelim", "fscanf", "getc_unlocked", "fgets_unlocked",
                         "fread_unlocked"};
    // Run the children in parallel: each spends ~5 s waiting.
    pid_t pids[sizeof fns / sizeof *fns];
    for (size_t i = 0; i < sizeof fns / sizeof *fns; ++i) {
        pids[i] = fork();
        if (pids[i] == 0) {
            execl("/proc/self/exe", argv[0], fns[i], static_cast<char*>(nullptr));
            _exit(127);
        }
    }
    int failures = 0;
    for (size_t i = 0; i < sizeof fns / sizeof *fns; ++i) {
        int st = 0;
        waitpid(pids[i], &st, 0);
        if (!WIFEXITED(st) || WEXITSTATUS(st) != 0) {
            std::printf("FAIL: %s (%s %d)\n", fns[i], WIFEXITED(st) ? "exit" : "signal",
                        WIFEXITED(st) ? WEXITSTATUS(st) : WTERMSIG(st));
            ++failures;
        }
    }
    if (failures) return 1;
    std::printf("stdio_buffer_test: all checks passed\n");
    return 0;
}
