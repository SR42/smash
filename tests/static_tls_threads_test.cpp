// static_tls_threads_test.cpp - smash's threads must start in an application
// that carries a lot of static TLS.
//
// glibc puts a new thread's static TLS block (the initial-exec TLS of every
// loaded module) and its guard page inside the thread's stack. smash created
// its coordinator and helpers with a fixed 128 KiB stack, so once the
// application carried ~128 KiB of static TLS every pthread_create failed with
// EINVAL at library load:
//
//   [smash WARN] pthread_create(coordinator) failed; compression disabled
//
// and smash silently ran as a plain allocator for the whole process. Reported
// on aarch64; not architecture-specific. Measured: 100 KiB of static TLS
// worked, 128 KiB and 256 KiB did not. Stacks are now sized as the thread's
// own need plus glibc's minimum (util/thread_stack.h).
//
// The check is the property itself: compressible memory must leave RSS. With
// the coordinator unable to start, nothing ever ticks and it never does. (A
// thread count is not enough: on a multi-node host the NUMA group workers,
// which have larger stacks, started even when the coordinator did not.)
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/mman.h>
#include <unistd.h>

// 512 KiB of initial-exec TLS in the executable: well past the old limit.
__thread char g_big_tls[512 * 1024];

static constexpr size_t kBytes = 16u << 20;   // >= 1 MiB: compressible in both modes
static constexpr size_t kPage = 4096;

// Fraction of [p, p+len) resident. The vector is static on purpose: a
// malloc'd one can sit on a compressed page, where mincore fails with EFAULT.
static double residentFraction(const char* p, size_t len) {
    static unsigned char vec[kBytes / kPage];
    if (mincore(const_cast<char*>(p), len, vec) != 0) return -1;
    size_t r = 0;
    for (size_t i = 0; i < len / kPage; ++i) r += vec[i] & 1;
    return static_cast<double>(r) / (len / kPage);
}

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    g_big_tls[0] = 1;  // keep the TLS block referenced
    if (sysconf(_SC_PAGESIZE) != static_cast<long>(kPage)) {
        printf("SKIP: this check assumes 4 KiB pages\n");
        return 0;
    }
    char* p = static_cast<char*>(malloc(kBytes));
    if (!p) { fprintf(stderr, "FAIL: malloc(16 MiB) returned nullptr\n"); return 1; }
    for (size_t i = 0; i < kBytes; ++i) p[i] = static_cast<char>('a' + (i / 64) % 8);

    double frac = 1.0;
    for (int t = 0; t < 40 && frac > 0.5; ++t) {   // up to 20 s
        usleep(500000);
        frac = residentFraction(p, kBytes);
        if (frac < 0) { printf("SKIP: mincore failed\n"); return 0; }
    }
    if (frac > 0.5) {
        fprintf(stderr,
                "FAIL: %.0f%% of a cold, compressible 16 MiB block is still resident "
                "after 20 s: smash's compressor never ran with 512 KiB of static TLS. "
                "Its thread stacks must leave room for the application's static TLS "
                "(util/thread_stack.h).\n", 100 * frac);
        return 1;
    }
    printf("PASS: cold block down to %.0f%% resident with 512 KiB of static TLS\n", 100 * frac);
    return 0;
}
