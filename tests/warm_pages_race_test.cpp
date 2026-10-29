// warm_pages_race_test.cpp - the write touch a wrapper makes on a buffer it is
// only warming must not lose another thread's store (src/vm/syscall_compat.h,
// warmPages / touchByteWritable).
//
// Wrappers with no retry surface (pthread_sigmask, the affinity calls, stdio,
// getcwd) touch the first byte of each page of the buffer the kernel will
// read or write, so a compressed page is restored and a PROT_READ page made
// writable before the real call. That touch was `c = *p; *p = c`: a thread
// storing to that byte between the load and the store had its store undone.
// The byte is the first of a page, which in a shared large buffer is as likely
// to be live data as any other. The touch is an atomic +0 now.
//
//   writer: stores an incrementing byte to the first byte of page 2 of a large
//           block and reads it back a moment later; a mismatch is a lost store
//   toucher: pthread_sigmask(SIG_BLOCK, set) with `set` on that page, which
//           warms the page (the set's bits are whatever the writer left; the
//           toucher's own mask is of no consequence)
//
// With the plain read-then-write this lost hundreds of stores a second; with
// the atomic touch it must lose none. SMASH_NO_COMPRESSOR=1 keeps page
// protection out of it: the page is plain read-write throughout.
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <dlfcn.h>
#include <thread>
#include <unistd.h>

int main() {
    using in_arena_fn = int (*)(const void*);
    auto in_arena = reinterpret_cast<in_arena_fn>(dlsym(RTLD_DEFAULT, "smash_in_arena"));
    if (!in_arena) { std::fprintf(stderr, "FAIL: smash_in_arena not exported\n"); return 1; }
    const size_t ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    auto* block = static_cast<unsigned char*>(std::malloc(2u << 20));
    if (!block || !in_arena(block)) { std::fprintf(stderr, "FAIL: no 2 MiB arena block\n"); return 1; }
    for (size_t off = 0; off < (2u << 20); off += ps) block[off] = 0;
    volatile unsigned char* byte = block + ps;   // first byte of page 2
    auto* set = reinterpret_cast<sigset_t*>(block + ps);   // on that page

    std::atomic<bool> stop{false};
    std::atomic<uint64_t> lost{0}, writes{0}, touches{0};
    std::thread writer([&] {
        unsigned char v = 0;
        while (!stop.load(std::memory_order_relaxed)) {
            ++v;
            *byte = v;
            for (int i = 0; i < 16; ++i) asm volatile("" ::: "memory");
            if (*byte != v) lost.fetch_add(1, std::memory_order_relaxed);
            writes.fetch_add(1, std::memory_order_relaxed);
        }
    });
    std::thread toucher([&] {
        while (!stop.load(std::memory_order_relaxed)) {
            pthread_sigmask(SIG_BLOCK, set, nullptr);
            touches.fetch_add(1, std::memory_order_relaxed);
        }
    });
    std::this_thread::sleep_for(std::chrono::seconds(2));
    stop.store(true);
    writer.join();
    toucher.join();
    std::printf("warm_pages_race: %llu stores, %llu touches, %llu lost\n",
                static_cast<unsigned long long>(writes.load()),
                static_cast<unsigned long long>(touches.load()),
                static_cast<unsigned long long>(lost.load()));
    if (lost.load() != 0) {
        std::fprintf(stderr, "FAIL: %llu stores lost under the warming touch\n",
                     static_cast<unsigned long long>(lost.load()));
        return 1;
    }
    std::free(block);
    std::printf("warm_pages_race: all tests passed\n");
    return 0;
}
