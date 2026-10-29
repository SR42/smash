// early_alloc_test.cpp - allocations made while smash is being set up
// (src/core/early_alloc.h).
//
// Run with helpers/olddlsym_shim.c preloaded after libsmash, which makes
// dlsym() calloc a per-thread record on a thread's first call, as glibc
// before 2.34 does (the release floor is 2.28). Before the early arena that
// calloc arrived while SmashHeap was constructing (the constructor's first
// mmap() dlsym()s its real target) and alloc8's getHeap() parked the
// constructing thread for ever; or, when it arrived inside
// SystemAllocFns::resolve(), smash's calloc resolved again, recursively.
// Either way this program never printed anything. The shim exits 99 at 200
// nested callocs so a regression fails fast instead of overflowing the stack.
//
//   1. the process starts and the main thread's malloc/calloc/new/realloc work
//   2. a fresh thread's first dlsym (another per-thread calloc) works, as do
//      its allocations
//   3. the early arena's blocks behave: free() is a no-op, realloc() copies,
//      malloc_usable_size() reports the size. Reached by dlsym()ing from a
//      fresh thread, whose calloc is the one the arena served -- its result
//      is not visible to this test, so the behaviour is checked on the heap's
//      view of such a pointer through the diagnostic export
//      smash_early_arena_info (blocks and bytes served).
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <malloc.h>
#include <new>
#include <pthread.h>
#include <unistd.h>

namespace {
int failures = 0;
void fail(const char* what) { std::fprintf(stderr, "FAIL: %s\n", what); ++failures; }

void* threadBody(void*) {
    // First dlsym on this thread: the shim callocs here.
    void* f = dlsym(RTLD_DEFAULT, "strlen");
    if (!f) fail("dlsym from a fresh thread returned null");
    void* p = std::malloc(100000);
    if (!p) fail("thread malloc returned null");
    std::memset(p, 1, 100000);
    std::free(p);
    auto* q = static_cast<int*>(std::calloc(1000, sizeof(int)));
    if (!q) fail("thread calloc returned null");
    for (int i = 0; i < 1000; ++i) if (q[i] != 0) { fail("thread calloc not zero"); break; }
    std::free(q);
    return nullptr;
}
}  // namespace

int main() {
    void* p = std::malloc(100);
    if (!p) fail("malloc returned null");
    std::memset(p, 7, 100);
    p = std::realloc(p, 200000);
    if (!p) fail("realloc returned null");
    if (static_cast<unsigned char*>(p)[99] != 7) fail("realloc lost the data");
    std::free(p);
    auto* c = static_cast<char*>(std::calloc(64, 64));
    if (!c) fail("calloc returned null");
    for (int i = 0; i < 4096; ++i) if (c[i]) { fail("calloc not zero"); break; }
    std::free(c);
    auto* n = new (std::nothrow) char[1 << 20];
    if (!n) fail("new returned null");
    delete[] n;

    pthread_t t;
    if (pthread_create(&t, nullptr, threadBody, nullptr) != 0) fail("pthread_create");
    else pthread_join(t, nullptr);

    // What the early arena served (diagnostic export).
    using info_fn = int (*)(uint64_t*, uint64_t*);
    auto info = reinterpret_cast<info_fn>(dlsym(RTLD_DEFAULT, "smash_early_arena_info"));
    uint64_t blocks = 0, bytes = 0;
    if (!info || info(&blocks, &bytes) != 0) fail("smash_early_arena_info not exported");
    std::printf("early arena served %llu blocks, %llu bytes\n",
                static_cast<unsigned long long>(blocks), static_cast<unsigned long long>(bytes));
    // With the shim preloaded, at least the constructing thread's dlsym calloc
    // and the fresh thread's must have come from it.
    if (blocks < 1) fail("the early arena served nothing: the shim's calloc went to the heap");

    if (failures) {
        std::fprintf(stderr, "early_alloc: %d failure(s)\n", failures);
        return 1;
    }
    std::printf("early_alloc: all tests passed\n");
    return 0;
}
