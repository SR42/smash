// blob_compact_test.cpp - blob memory left behind by freed compressed data
// must go back to the OS, and the blobs moved to free it must stay correct.
//
// Added for a 128-core arm64 application whose [smash stats] blob= went from
// 1.7/5.2 GiB (live/held) to 1.6/15.7 GiB over three hours: the store's 16 MiB
// regions each kept a few live blobs among holes, and a region was returned
// only once every blob in it was gone.
//
// 384 blocks of 1 MiB, each page 2 KiB of seeded random bytes and 2 KiB of
// zeros (~2 KiB blobs, ~192 MiB in all: every shard fills at least one
// region), are compressed. Then three blocks in four are freed, which leaves
// every region a quarter live. Checks:
//   - blob= held falls to within 16 MiB of live: a region drains once under
//     half live (SMASH_STORE_DRAIN_PCT=50), and here every region, including
//     the one each shard bumps into, is a quarter live;
//   - every byte of the kept blocks reads back as written (their blobs are
//     the ones that moved).
#include <csignal>
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

constexpr size_t kBlock = 1u << 20, kBlocks = 384, kPage = 4096;
int g_log = -1;

struct Dump { bool ok = false; size_t live = 0, held = 0, regions = 0; };

Dump dump() {
    Dump d;
    raise(SIGUSR2);
    usleep(100 * 1000);
    std::string log;
    char buf[8192];
    lseek(g_log, 0, SEEK_SET);
    for (ssize_t n; (n = read(g_log, buf, sizeof(buf))) > 0;) log.append(buf, static_cast<size_t>(n));
    const size_t at = log.rfind("[smash stats]");
    if (at == std::string::npos) return d;
    const size_t b = log.find(" blob=", at);
    if (b == std::string::npos || b > log.find('\n', at)) return d;
    d.ok = std::sscanf(log.c_str() + b, " blob=%zu/%zuMiB(%zu regions", &d.live, &d.held, &d.regions) == 3;
    return d;
}

uint64_t seedOf(size_t blk, size_t pg) { return (blk * 1000003ull + pg) * 0x9E3779B97F4A7C15ull | 1; }

void fill(unsigned char* p, size_t blk) {
    for (size_t pg = 0; pg < kBlock / kPage; ++pg) {
        uint64_t x = seedOf(blk, pg);
        unsigned char* q = p + pg * kPage;
        for (size_t i = 0; i < 2048; i += 8) {
            x ^= x << 13; x ^= x >> 7; x ^= x << 17;
            std::memcpy(q + i, &x, 8);
        }
        std::memset(q + 2048, 0, kPage - 2048);
    }
}

bool verify(const unsigned char* p, size_t blk) {
    for (size_t pg = 0; pg < kBlock / kPage; ++pg) {
        uint64_t x = seedOf(blk, pg);
        const unsigned char* q = p + pg * kPage;
        for (size_t i = 0; i < 2048; i += 8) {
            x ^= x << 13; x ^= x >> 7; x ^= x << 17;
            if (std::memcmp(q + i, &x, 8) != 0) return false;
        }
        for (size_t i = 2048; i < kPage; ++i)
            if (q[i]) return false;
    }
    return true;
}

size_t resident(const std::vector<unsigned char*>& b) {
    std::vector<unsigned char> v(kBlock / kPage);
    size_t r = 0;
    for (unsigned char* p : b) {
        if (!p) continue;
        if (mincore(p, kBlock, v.data()) != 0) return SIZE_MAX;
        for (auto c : v) r += c & 1;
    }
    return r;
}

void show(const char* when, const Dump& d) {
    std::printf("%-22s blob=%zu/%zuMiB(%zu regions)\n", when, d.live, d.held, d.regions);
}

}  // namespace

int main() {
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) { std::fprintf(stderr, "FAIL: not running under libsmash\n"); return 1; }
    char path[] = "/tmp/smash_blobcompact_XXXXXX";
    g_log = mkstemp(path);
    if (g_log < 0) { std::printf("SKIP: mkstemp failed\n"); return 77; }
    unlink(path);
    const int saved = dup(2);
    dup2(g_log, 2);

    int failures = 0;
    auto fail = [&](const char* what) { std::printf("FAIL: %s\n", what); ++failures; };

    std::vector<unsigned char*> b(kBlocks);
    for (size_t i = 0; i < kBlocks; ++i) {
        b[i] = static_cast<unsigned char*>(malloc(kBlock));
        fill(b[i], i);
    }
    size_t left = SIZE_MAX;
    for (int s = 0; s < 480 && (left = resident(b)) > 0; ++s) usleep(250 * 1000);
    const Dump packed = dump();
    // Free three blocks in four: every region keeps a quarter of its blobs.
    for (size_t i = 0; i < kBlocks; ++i)
        if (i % 4) { free(b[i]); b[i] = nullptr; }
    Dump freed = dump();
    for (int s = 0; s < 40 && freed.ok && freed.live * 3 > packed.live; ++s) {
        usleep(250 * 1000);
        freed = dump();
    }
    // Held as it stood once the frees had been processed: the compactor may
    // already be moving blobs, so take the larger of the two readings.
    const size_t held_freed = std::max(packed.held, freed.held);
    Dump settled = freed;
    for (int s = 0; s < 120 && settled.ok && settled.held > settled.live + 16; ++s) {
        usleep(250 * 1000);
        settled = dump();
    }
    size_t bad = 0;
    for (size_t i = 0; i < kBlocks; i += 4) bad += !verify(b[i], i);
    const Dump end = dump();
    dup2(saved, 2);

    show("compressed", packed);
    show("3 in 4 blocks freed", freed);
    show("settled", settled);
    show("kept blocks read", end);
    if (!packed.ok || !freed.ok || !settled.ok) {
        fail("no blob= on the [smash stats] line");
        return 1;
    }
    if (left > 0) {
        std::printf("FAIL: %zu pages still resident after 120 s\n", left);
        return 1;
    }
    if (packed.live < 128) fail("less than 128 MiB of blobs: the store never filled a region per shard");
    if (freed.live * 3 > packed.live) fail("freeing three blocks in four did not release their blobs");
    if (held_freed < settled.live + 64) fail("freeing three blocks in four left no holes to reclaim");
    if (settled.held > settled.live + 16) fail("blob held did not settle within 16 MiB of live");
    if (bad) {
        std::printf("FAIL: %zu of %zu kept blocks read back wrong\n", bad, kBlocks / 4);
        ++failures;
    }
    if (failures) return 1;
    std::printf("blob_compact_test: all checks passed\n");
    return 0;
}
