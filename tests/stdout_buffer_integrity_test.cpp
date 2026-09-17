// Buffered stdio writes must survive compression of the FILE buffer's page.
//
// glibc allocates stdout's 4 KiB buffer with malloc on first use, so under
// full mode it is an ordinary smash slab object. An application that writes
// to stdout in bursts separated by quiet periods lets that page go cold
// between bursts; the compressor then compresses it while it still holds
// unflushed bytes, and the application keeps writing into it afterwards.
//
// Observed on x86-64 with the compressor active: bytes written into the
// buffer AFTER it was first compressed were flushed as NUL. A run producing
// 513 bytes of output emitted 285 correct bytes followed by 228 zero bytes —
// the tail was silently replaced, deterministically, on every run. Without
// the compressor (SMASH_DEFER_PHASES_MS large), in large-only mode, or with
// no libsmash at all, the same workload is byte-perfect.
//
// FIXED, and this test is the regression guard. The cause was not compression
// losing writes: it was drainAllForShutdown() reinstating a stale snapshot
// over live data at exit. That walker treated COMPRESSED and
// COMPRESSED_SHADOW alike and decompressed the blob back over the page — but
// a shadow page is still mapped PROT_RW and holds the application's LIVE
// bytes. Nothing faults on a write to it, so its blob goes stale silently and
// only reclaimShadowRange()'s content_matches check ever notices; a page that
// is never reclaimed is never checked. The drain now keeps the page and drops
// the blob, as handleFault's COMPRESSED_SHADOW case already did.
//
// The surviving prefix is the tell: it ends exactly where the buffer's write
// pointer stood when the snapshot was taken (offset 285, on a marker-line
// boundary, with a single NUL run to EOF and no non-zero byte after it). That
// is a reverted page, not a zeroed one — the never-written tail of the buffer
// was simply still zero in the blob.
//
// This is silent data corruption, and it is a plausible route to the
// std::bad_alloc reports that motivated this test group: a zeroed or
// half-zeroed length or capacity field in an application data structure
// turns into an absurd allocation request, and `operator new` throws
// std::bad_alloc for a request the machine could never satisfy. Corruption
// of application buffers is the more serious finding either way.
//
// Structure: the check has to observe a process that has actually exited,
// because the damage is only visible once glibc flushes the buffer at exit.
// So the test re-execs ITSELF with SMASH_TEST_STDIO_CHILD=1 and the child's
// stdout pointed at a temp file, then inspects the file after reaping it.
// It must be exec, not a bare fork: fork clones only the calling thread, so a
// forked child has no compressor thread and would never compress anything.
// LD_PRELOAD / DYLD_INSERT_LIBRARIES survive exec, so the child gets its own
// fully initialised libsmash.
//
// Knobs that do NOT change the outcome, and why that is misleading rather
// than informative: SMASH_SOFTDIRTY=0, SMASH_DEFER_MADVISE=0,
// SMASH_DEFER_MADVISE_TICKS, SMASH_NO_DECOMMIT=1, SMASH_MADV_FREE=1,
// SMASH_NO_MONITOR=1, SMASH_FIXAV and SMASH_P2_CHUNK all give byte-identical
// output. Every one of them steers an arm of the non-deferred `else` in
// compressPage(), while SMASH_DEFERRED_RECLAIM (default ON, in
// include/smash/config.h) takes the `if`. Byte-identical results across
// supposedly different configs mean the flag is inert, not that the suspect
// is innocent — read the enclosing branch before trusting an A/B.
//
// The flags that DO discriminate: SMASH_DEFERRED_RECLAIM=0 (no shadow state
// exists, so nothing can be reverted) and SMASH_DEFERRED_RECLAIM_DELAY.
// DELAY=1 reclaims to a real COMPRESSED page before the late writes, so they
// fault and invalidate the blob; DELAY=100000 never reclaims, keeping the
// OLDEST blob, and makes the loss predictably worse — 399 NUL with the
// boundary at offset 114 instead of 228 at 285. Note also that the
// corruption is deterministic (byte-identical across runs), so it was never
// a race, and strace perturbs it away completely.

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <new>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

namespace {

int failures = 0;

// Ballast keeps the compressor genuinely busy so the stdout buffer's page is
// a realistic compression candidate rather than the only page in the region.
constexpr size_t kBallastChunk = 4u << 20;
constexpr size_t kBallastChunks = 128;      // 512 MiB

// Bursts separated by quiet periods: the gap is what lets the buffer's page
// go cold and be compressed between writes.
constexpr int kBursts = 9;           // marker lines the child emits

constexpr size_t kThpRegion = 64u << 20;
constexpr size_t kThpAlign = 2u << 20;

// Each line is tagged so the parent can tell "missing" from "zeroed".
constexpr const char* kLineFmt =
    "MARK %02d ================================================\n";

const char* modeName() {
    const char* e = std::getenv("SMASH_LARGE_ONLY");
    return (e && e[0] == '1') ? "large-only" : "full";
}

// The child: a realistic mixed workload — large compressible heap blocks, a
// transparent-hugepage-hinted region, a partial refault, and late
// allocations — with buffered stdout writes interleaved between the quiet
// stretches. This is the shape the corruption was first observed in.
// Returns normally so glibc's exit-time flush is what actually writes the
// buffer out.
[[noreturn]] void runChild() {
    int mark = 0;
    auto emit = [&mark]() { printf(kLineFmt, mark); ++mark; };

    std::vector<void*> heap;
    heap.reserve(kBallastChunks);
    for (size_t i = 0; i < kBallastChunks; ++i) {
        void* p = nullptr;
        try {
            p = ::operator new(kBallastChunk);
        } catch (const std::bad_alloc&) {
            break;   // parent reports whatever the output shows
        }
        std::memset(p, static_cast<int>(i & 0xff), kBallastChunk);
        heap.push_back(p);
    }
    emit();

    // THP-hinted region. A partial mprotect inside a transparent hugepage
    // forces the kernel to split it, which is extra pressure on the same
    // machinery.
    char* thp = nullptr;
    void* thp_raw = mmap(nullptr, kThpRegion + kThpAlign,
                         PROT_READ | PROT_WRITE,
                         MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (thp_raw != MAP_FAILED) {
        auto a = reinterpret_cast<uintptr_t>(thp_raw);
        thp = reinterpret_cast<char*>((a + kThpAlign - 1) & ~(kThpAlign - 1));
        madvise(thp, kThpRegion, MADV_HUGEPAGE);
        std::memset(thp, 0x5A, kThpRegion);
    }
    emit();

    // Quiet stretch: the stdout buffer goes cold and is compressed here.
    for (int t = 0; t < 8; ++t) {
        sleep(1);
        if (t % 3 == 2) emit();
    }

    // Partial refault: fault-driven decompression restores pages one at a time.
    for (size_t i = 0; i < heap.size(); i += 2)
        static_cast<volatile char*>(heap[i])[0] = 1;
    if (thp) {
        for (size_t off = 0; off < kThpRegion; off += 64u << 10)
            static_cast<volatile char*>(thp)[off] = 1;
    }
    emit();

    // Second quiet stretch, then the final burst. Everything from here on is
    // what came out as NUL when this was first observed.
    for (int t = 0; t < 4; ++t) sleep(1);
    emit();

    std::vector<void*> late;
    for (int i = 0; i < 64; ++i) {
        try {
            void* p = ::operator new(2u << 20);
            std::memset(p, 0x33, 2u << 20);
            late.push_back(p);
        } catch (const std::bad_alloc&) {
            break;
        }
    }
    emit();
    emit();
    emit();

    // exit(), not _exit(): the exit-time stdio flush is exactly what is under
    // test — the buffer's contents are not written until this point.
    std::exit(0);
}

} // namespace

int main(int argc, char** argv) {
    // ── Child role ──────────────────────────────────────────────────────────
    // Re-exec'd by the parent with stdout already pointed at the temp file.
    if (std::getenv("SMASH_TEST_STDIO_CHILD")) runChild();

    // Parent only — the child's stdout buffering is the thing under test.
    setvbuf(stdout, nullptr, _IONBF, 0);
    printf("stdout_buffer_integrity_test: mode=%s markers=%d ballast=%zu MiB\n",
           modeName(), kBursts,
           (kBallastChunks * kBallastChunk) / (1024 * 1024));
    fflush(stdout);

    // Resolve our own path for the re-exec. /proc/self/exe is exact; argv[0]
    // is the fallback.
    char exe[4096];
    ssize_t elen = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
    if (elen > 0) exe[elen] = '\0';
    else if (argc > 0) { std::snprintf(exe, sizeof(exe), "%s", argv[0]); }
    else {
        printf("SKIP: cannot resolve own executable path to re-exec\n");
        return 0;
    }

    char path[] = "/tmp/smash-stdout-integrity-XXXXXX";
    int fd = mkstemp(path);
    if (fd < 0) {
        fprintf(stderr, "FAIL: mkstemp errno=%d (%s)\n",
                errno, std::strerror(errno));
        return 1;
    }

    pid_t pid = fork();
    if (pid < 0) {
        fprintf(stderr, "FAIL: fork errno=%d (%s)\n",
                errno, std::strerror(errno));
        close(fd);
        unlink(path);
        return 1;
    }

    if (pid == 0) {
        // Child: stdout to the temp file. A regular file means stdout is
        // fully buffered, which is the condition under test.
        if (dup2(fd, 1) < 0) std::_Exit(127);
        close(fd);
        setenv("SMASH_TEST_STDIO_CHILD", "1", 1);
        char* args[] = {exe, nullptr};
        execv(exe, args);
        std::_Exit(127);   // exec failed
    }

    int status = 0;
    if (waitpid(pid, &status, 0) < 0) {
        fprintf(stderr, "FAIL: waitpid errno=%d (%s)\n",
                errno, std::strerror(errno));
        close(fd);
        unlink(path);
        return 1;
    }
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        fprintf(stderr, "FAIL: child did not exit cleanly (status=%d)%s\n",
                status,
                (WIFEXITED(status) && WEXITSTATUS(status) == 127)
                    ? " — execv of the child failed" : "");
        ++failures;
    }

    // Slurp the child's output.
    lseek(fd, 0, SEEK_SET);
    std::string out;
    char buf[8192];
    for (;;) {
        ssize_t n = read(fd, buf, sizeof(buf));
        if (n <= 0) break;
        out.append(buf, static_cast<size_t>(n));
    }
    close(fd);
    unlink(path);

    // ── 1. No NUL bytes ─────────────────────────────────────────────────────
    // The buffer is flushed with the right length but zeroed content, so the
    // damage shows up as NUL rather than as missing bytes.
    size_t nuls = 0, first_nul = out.size();
    for (size_t i = 0; i < out.size(); ++i) {
        if (out[i] == '\0') {
            if (nuls == 0) first_nul = i;
            ++nuls;
        }
    }

    // ── 2. Every line the child wrote is present and intact ─────────────────
    size_t found = 0;
    for (int i = 0; i < kBursts; ++i) {
        char needle[32];
        snprintf(needle, sizeof(needle), "MARK %02d ", i);
        if (out.find(needle) != std::string::npos) ++found;
    }

    printf("child produced %zu bytes; %zu/%d marker lines intact; "
           "%zu NUL bytes\n", out.size(), found, kBursts, nuls);

    if (nuls > 0) {
        fprintf(stderr,
                "FAIL: %zu of the child's %zu stdout bytes were flushed as NUL "
                "(first at offset %zu). Writes into glibc's stdout buffer were "
                "silently lost after the compressor compressed the buffer's "
                "page — the bytes before that point survived, everything "
                "written afterwards came out as zeros.\n"
                "      This is application data corruption, not just lost "
                "output: any long-lived buffer written in bursts is exposed. A "
                "zeroed length or capacity field turns the next allocation "
                "request into an absurd size, which is one route to the "
                "std::bad_alloc this test group investigates.\n"
                "      Not affected by SMASH_SOFTDIRTY, SMASH_DEFER_MADVISE, "
                "SMASH_NO_DECOMMIT, SMASH_MADV_FREE or SMASH_NO_MONITOR; only "
                "disabling compression avoids it. SMASH_LARGE_ONLY=1 (the "
                "production-supported mode) is unaffected.\n",
                nuls, out.size(), first_nul);
        ++failures;
    }

    if (found != static_cast<size_t>(kBursts)) {
        fprintf(stderr,
                "FAIL: only %zu of %d marker lines survived the child's "
                "buffered writes\n", found, kBursts);
        ++failures;
    }

    if (failures) {
        printf("stdout_buffer_integrity_test (%s): %d FAILURES\n",
               modeName(), failures);
        return 1;
    }
    printf("stdout_buffer_integrity_test (%s): all checks passed\n",
           modeName());
    return 0;
}
