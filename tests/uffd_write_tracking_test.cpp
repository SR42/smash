// uffd_write_tracking_test.cpp - userfaultfd write tracking (SMASH_WRITE_TRACK=uffd).
//
// arm64 kernels have no soft-dirty (CONFIG_MEM_SOFT_DIRTY does not exist
// there). smash used to trust soft-dirty because /proc/self/clear_refs and
// pagemap opened, so on arm64 it saw every page as never written: hot pages
// were compressed anyway and only Phase B's content check caught it. Where
// soft-dirty does not work, smash now tracks writes with async userfaultfd
// write-protect + PAGEMAP_SCAN (Linux 6.7+), chosen by probing, not by config.
//
// Driven in-process, deterministic, with the tracker forced to uffd:
//   hot    written before every tick          -> must never compress
//   cold   never written                       -> must compress
//   remap  like hot, but remapPages()'d first  -> must never compress
// The last one needs remapPages() to re-register the range: a MAP_FIXED
// mapping is a new VMA the userfaultfd no longer covers, so without that
// hook its writes are invisible and it compresses like a cold page.
//
// Skips when the kernel lacks async userfaultfd write-protect.
#include "vm/vm_region.h"
#include "vm/page_state.h"
#include "vm/uffd_uapi.h"
#include "compress/compress_store.h"
#include "compress/compress_engine.h"
#include "compress/compressor_thread.h"
#include "core/page_map.h"
#include "core/span.h"
#include "smash/config.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

using namespace smash;

static int failures = 0;
#define CHECK(cond, msg, ...) do { \
    if (!(cond)) { fprintf(stderr, "FAIL: " msg "\n", ##__VA_ARGS__); ++failures; } \
} while (0)

// Can this kernel do what the tracker needs? (Same request smash makes.)
static bool kernelHasAsyncUffdWp() {
    int fd = static_cast<int>(syscall(__NR_userfaultfd, O_CLOEXEC | O_NONBLOCK | UFFD_USER_MODE_ONLY));
    if (fd < 0) fd = static_cast<int>(syscall(__NR_userfaultfd, O_CLOEXEC | O_NONBLOCK));
    if (fd < 0) return false;
    uffdio_api api{};
    api.api = UFFD_API;
    api.features = UFFD_FEATURE_WP_ASYNC | UFFD_FEATURE_WP_UNPOPULATED;
    const bool ok = ioctl(fd, UFFDIO_API, &api) == 0;
    close(fd);
    return ok;
}

static bool isCompressed(PageState s) {
    return s == PageState::COMPRESSED || s == PageState::COMPRESSED_SHADOW;
}

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    // Pin the tracker config in the test itself, not only in the ctest
    // registration. selectWriteTracker() checks SMASH_SOFTDIRTY *before* it
    // looks at SMASH_WRITE_TRACK, so an inherited SMASH_SOFTDIRTY=0 (a whole
    // suite run forcing Phase 3 PROT_READ monitoring) silently overrides the
    // forced tracker: Phase 3 then mprotects this test's own pages PROT_READ,
    // and since the test installs no fault handler, its next write to the hot
    // page dies with SIGSEGV instead of testing anything.
    setenv("SMASH_SOFTDIRTY", "1", 1);
    setenv("SMASH_WRITE_TRACK", "uffd", 1);
    if (!kernelHasAsyncUffdWp()) {
        printf("SKIP: kernel lacks async userfaultfd write-protect (needs Linux 6.7+)\n");
        return 77;
    }

    VmRegion vm;
    CHECK(vm.init(64 * kPageSize), "VmRegion init failed");
    PageStateTable states; states.init(vm.totalPages());
    PageLockTable locks; locks.init(vm.totalPages());
    CompressStore store; store.init();
    CompressEngine engine; engine.init();
    PageMap page_map; page_map.init();
    CompressorThread c;
    c.init(&vm, &states, &locks, &store, &engine, &page_map);

    constexpr int kPages = 3;
    void* base = vm.allocatePages(kPages);
    CHECK(base != nullptr, "page alloc failed");
    if (!base) return 1;
    Span* span = newSpanDescriptor();
    span->init(base, kPages, 0);
    span->is_large = true;  // keep zeroFreeSlots away from test data
    page_map.setRange(reinterpret_cast<uintptr_t>(base), kPages, span);
    const size_t b = vm.pageIndex(reinterpret_cast<uintptr_t>(base));
    const size_t hot = b, cold = b + 1, remapped = b + 2;
    auto page = [&](size_t i) { return static_cast<volatile unsigned char*>(vm.pageAddress(i)); };

    for (int p = 0; p < kPages; ++p) {
        states.set(b + p, PageState::ACTIVE);
        memset(vm.pageAddress(b + p), 0x30 + p, kPageSize);
    }

    c.compressTick();  // first tick selects and sets up the tracker
    // Replace the third page's mapping, as smash's restore/reclaim fallbacks do.
    CHECK(vm::remapPages(vm.pageAddress(remapped), kPageSize, true, true),
          "remapPages failed");
    memset(vm.pageAddress(remapped), 0x55, kPageSize);

    // Check EVERY tick, not just the last: an untracked page that is still
    // being written gets compressed into a shadow, then Phase B's content
    // check discards the stale blob and it is ACTIVE again — so a final-state
    // check alone can land on an ACTIVE phase and miss it.
    int hot_compressed_at = -1, remap_compressed_at = -1;
    for (int t = 0; t < 12; ++t) {
        page(hot)[t % kPageSize] = static_cast<unsigned char>(t);
        page(remapped)[t % kPageSize] = static_cast<unsigned char>(t);
        c.compressTick();
        if (hot_compressed_at < 0 && isCompressed(states.get(hot))) hot_compressed_at = t;
        if (remap_compressed_at < 0 && isCompressed(states.get(remapped))) remap_compressed_at = t;
    }

    const auto sc = states.get(cold);
    printf("  12 ticks: cold=%d at the end; hot compressed at tick %d, remapped at tick %d "
           "(-1 = never)\n", static_cast<int>(sc), hot_compressed_at, remap_compressed_at);
    CHECK(isCompressed(sc), "the never-written page should have compressed, state %d — "
          "is the compressor running at all?", static_cast<int>(sc));
    CHECK(hot_compressed_at < 0, "a page written before every tick was compressed at tick "
          "%d: the userfaultfd tracker did not report its writes", hot_compressed_at);
    CHECK(remap_compressed_at < 0, "a page written before every tick after remapPages() was "
          "compressed at tick %d: the remapped range was not re-registered with the "
          "userfaultfd (vm::g_after_remap_track_hook)", remap_compressed_at);

    if (failures) { printf("uffd_write_tracking_test: %d FAILURES\n", failures); return 1; }
    printf("uffd_write_tracking_test: all checks passed\n");
    return 0;
}
