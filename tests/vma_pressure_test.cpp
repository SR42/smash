// smash must not drive the process toward vm.max_map_count.
//
// Every page smash mprotects individually splits the enclosing VMA. The
// kernel caps a process at /proc/sys/vm/max_map_count mappings (65530 by
// default), and crossing it is not a soft failure:
//
//   * mprotect() starts returning ENOMEM — platform_mem.h documents this,
//     and compressor_thread.h records the resulting livelock ("stuck at 2M
//     rows, VMAs pinned at the 65531 cap"), where a page could no longer be
//     restored and the faulting access re-faulted forever;
//   * and, decisively for this test, **mmap() also starts returning ENOMEM**.
//     That is the direct path to the reported symptom: vm::mapPages() returns
//     nullptr, LargeAlloc::allocate() returns nullptr, malloc returns nullptr,
//     and the operator-new wrappers in src/linux_syscall_wrappers.cpp turn
//     nullptr into `throw std::bad_alloc()`.
//
// smash has three mitigations for this — soft-dirty access tracking instead
// of PROT_READ monitoring (default ON on Linux, no VMA cost), chunked
// mprotect over runs of contiguous pages (kProtectChunkPages), and a mid-loop
// VMA budget in reclaimShadowRange() — but nothing pins them in place. This
// test drives a compress-and-refault workload and asserts the VMA count stays
// far below the cap, so a regression that reverts to per-page splitting is
// caught here rather than as an out-of-memory report from a large workload.
//
// Transparent hugepages make this sharper. A partial mprotect inside a THP
// forces the kernel to split it, so on a host with THP enabled an app whose
// memory is huge-page backed converts each 2 MiB region into up to 512 VMAs.
// That is why this test allocates 2 MiB-aligned, MADV_HUGEPAGE-hinted memory
// alongside ordinary heap blocks.
//
// Linux only — /proc/self/maps and vm.max_map_count have no analogue
// elsewhere.

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <vector>

#if defined(__linux__)

#include <sys/mman.h>
#include <unistd.h>

namespace {

int failures = 0;

constexpr size_t kHeapChunk = 4u << 20;     // 4 MiB, > kLargeAllocVmThreshold
constexpr size_t kHeapChunks = 128;         // 512 MiB of compressible heap
constexpr size_t kThpRegion = 64u << 20;    // 64 MiB of THP-hinted memory
constexpr size_t kThpAlign = 2u << 20;
constexpr int kSettleSeconds = 8;

// A workload of this size would produce ~131072 VMAs if smash split one per
// compressed 4 KiB page. Anything under a few thousand proves the coalescing
// mitigations are active; measured peak on x86-64 with defaults is ~175.
constexpr long kVmaBudget = 8192;

long countVmas() {
    FILE* f = fopen("/proc/self/maps", "r");
    if (!f) return -1;
    char line[512];
    long n = 0;
    while (fgets(line, sizeof(line), f)) ++n;
    fclose(f);
    return n;
}

long maxMapCount() {
    FILE* f = fopen("/proc/sys/vm/max_map_count", "r");
    if (!f) return -1;
    long v = -1;
    if (fscanf(f, "%ld", &v) != 1) v = -1;
    fclose(f);
    return v;
}

} // namespace

int main() {
    // Unbuffered stdout: full mode used to zero the tail of glibc's buffered
    // stdout page at exit (fixed; guarded by test_stdout_buffer_integrity_full),
    // silently eating a test's own verdict lines. Kept as hygiene.
    setvbuf(stdout, nullptr, _IONBF, 0);
    const long cap = maxMapCount();
    const long start = countVmas();
    printf("vma_pressure_test: vm.max_map_count=%ld VMAs at start=%ld\n",
           cap, start);
    if (start < 0) {
        printf("SKIP: /proc/self/maps unreadable\n");
        return 0;
    }

    long peak = start;
    auto sample = [&peak](const char* what) {
        long n = countVmas();
        if (n > peak) peak = n;
        printf("  %-28s VMAs=%ld\n", what, n);
        return n;
    };

    // ── Ordinary compressible heap ──────────────────────────────────────────
    std::vector<void*> heap;
    heap.reserve(kHeapChunks);
    for (size_t i = 0; i < kHeapChunks; ++i) {
        void* p = nullptr;
        try {
            p = ::operator new(kHeapChunk);
        } catch (const std::bad_alloc&) {
            fprintf(stderr,
                    "FAIL: std::bad_alloc allocating heap chunk %zu of %zu — "
                    "VMA count was %ld against a cap of %ld, so mmap was "
                    "refused with ENOMEM\n",
                    i, kHeapChunks, countVmas(), cap);
            ++failures;
            break;
        }
        // Compressible content: long runs, so the compressor will act on it.
        std::memset(p, static_cast<int>(i & 0xff), kHeapChunk);
        heap.push_back(p);
    }
    sample("after heap alloc");

    // ── THP-hinted region ───────────────────────────────────────────────────
    // A partial mprotect inside a transparent hugepage splits it, so this is
    // where per-page protection turns into a VMA explosion.
    void* thp_raw = mmap(nullptr, kThpRegion + kThpAlign,
                         PROT_READ | PROT_WRITE,
                         MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    char* thp = nullptr;
    if (thp_raw == MAP_FAILED) {
        printf("NOTE: THP region mmap failed (%s); continuing with heap only\n",
               std::strerror(errno));
    } else {
        auto a = reinterpret_cast<uintptr_t>(thp_raw);
        thp = reinterpret_cast<char*>((a + kThpAlign - 1) & ~(kThpAlign - 1));
        // Best-effort: no-op where THP is disabled, which is fine.
        madvise(thp, kThpRegion, MADV_HUGEPAGE);
        std::memset(thp, 0x5A, kThpRegion);
        sample("after THP region");
    }

    // ── Let the compressor run ──────────────────────────────────────────────
    for (int t = 0; t < kSettleSeconds; ++t) {
        sleep(1);
        if (t % 3 == 2) sample("during compression");
    }

    // ── Refault: touch a fraction of everything ─────────────────────────────
    // Fault-driven decompression restores pages individually, which is the
    // other per-page mprotect path.
    for (size_t i = 0; i < heap.size(); i += 2)
        static_cast<volatile char*>(heap[i])[0] = 1;
    if (thp) {
        for (size_t off = 0; off < kThpRegion; off += 64u << 10)
            static_cast<volatile char*>(thp)[off] = 1;
    }
    sample("after partial refault");

    for (int t = 0; t < 4; ++t) sleep(1);
    sample("after settle");

    // ── Allocation must still work ──────────────────────────────────────────
    // The symptom this whole test exists for: once VMAs are exhausted, mmap
    // returns ENOMEM and operator new throws.
    std::vector<void*> late;
    for (int i = 0; i < 64; ++i) {
        try {
            void* p = ::operator new(2u << 20);
            std::memset(p, 0x33, 2u << 20);
            late.push_back(p);
        } catch (const std::bad_alloc&) {
            fprintf(stderr,
                    "FAIL: std::bad_alloc on late allocation %d — VMA count "
                    "%ld against cap %ld\n", i, countVmas(), cap);
            ++failures;
            break;
        }
    }
    sample("after late allocs");

    // ── Verdict ─────────────────────────────────────────────────────────────
    const long grew = peak - start;
    printf("peak VMAs=%ld (grew %ld from %ld) against a %ld budget "
           "and a %ld cap\n", peak, grew, start, kVmaBudget, cap);

    if (grew > kVmaBudget) {
        fprintf(stderr,
                "FAIL: smash grew the process VMA count by %ld (peak %ld) for "
                "a %zu MiB workload, over the %ld budget.\n"
                "      Per-page mprotect splits one VMA per page; at this rate "
                "a larger workload crosses vm.max_map_count (%ld here, 65530 "
                "by default), after which mmap returns ENOMEM, malloc returns "
                "nullptr, and operator new throws std::bad_alloc.\n"
                "      Check that soft-dirty tracking (SMASH_SOFTDIRTY, "
                "default on), chunked Phase-3 mprotect, and the "
                "reclaimShadowRange VMA budget are all still active.\n",
                grew, peak,
                (kHeapChunks * kHeapChunk + kThpRegion) / (1024 * 1024),
                kVmaBudget, cap);
        ++failures;
    }

    if (cap > 0 && peak > cap / 4) {
        fprintf(stderr,
                "FAIL: peak VMA count %ld is over a quarter of "
                "vm.max_map_count (%ld) — too little headroom before mmap "
                "starts failing with ENOMEM\n", peak, cap);
        ++failures;
    }

    for (void* p : heap) ::operator delete(p);
    for (void* p : late) ::operator delete(p);
    if (thp_raw != MAP_FAILED) munmap(thp_raw, kThpRegion + kThpAlign);

    if (failures) {
        printf("vma_pressure_test: %d FAILURES\n", failures);
        return 1;
    }
    printf("vma_pressure_test: all checks passed\n");
    return 0;
}

#else  // !__linux__

int main() {
    printf("SKIP: vm.max_map_count and /proc/self/maps are Linux-only\n");
    return 0;
}

#endif
