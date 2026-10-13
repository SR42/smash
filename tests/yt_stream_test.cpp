// yt_stream_test.cpp - the reader of youngtrace's scanner output
// (src/compress/yt_stream.h), on streams built here byte by byte in the
// module's layout: no kernel, no root.
//
// Checks that a young present page is reported and an old one is not; that a
// transparent huge page item counts 512 pages and a run item its own page
// count; that the node/active/ref bits above the address are not part of it;
// that non-present classes are skipped; that a stream fed in pieces of every
// size (a read can stop anywhere) gives the same answer; and that a cut
// stream (status bit 0), a stream ending inside a record's items, a foreign
// record magic and a zero-length run are each told apart from a complete one.
#include "compress/yt_stream.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <utility>
#include <vector>

using namespace smash::yt;

namespace {

int failures = 0;
void check(bool ok, const char* what) {
    if (!ok) { std::printf("FAIL: %s\n", what); ++failures; }
}

struct Stream {
    std::vector<unsigned char> b;
    void u64(uint64_t v) {
        unsigned char x[8];
        std::memcpy(x, &v, 8);
        b.insert(b.end(), x, x + 8);
    }
    void status(uint64_t s) { u64(s); }
    void record(uint64_t items, uint32_t magic = kRecordMagic) {
        std::vector<unsigned char> r(kRecordBytes, 0);
        std::memcpy(r.data(), &magic, 4);
        std::memcpy(r.data() + kRecordItemsOffset, &items, 8);
        b.insert(b.end(), r.begin(), r.end());
    }
    // class present, young unless old, size log2(bytes) - 12
    void item(uint64_t addr, bool old = false, unsigned size = 0, uint64_t cls = kClassPresent,
              uint64_t high = 0) {
        u64(addr | cls | (old ? kItemOld : 0) | (uint64_t{size} << kItemSizeShift) | high);
    }
    void run(uint64_t addr, uint64_t pages, bool old = false) {
        const uint64_t meta = kClassPresent | (old ? kItemOld : 0);
        u64(addr | meta | kItemRun);
        u64(pages);
        u64(meta);
    }
};

using Hits = std::vector<std::pair<uint64_t, uint64_t>>;

// Feed `s` in pieces of `piece` bytes the way ytRequest does: keep what the
// parser did not consume in front of the next piece.
Hits parse(const Stream& s, size_t piece, StreamParser& ps) {
    Hits hits;
    std::vector<unsigned char> buf;
    for (size_t off = 0; off < s.b.size(); off += piece) {
        const size_t n = std::min(piece, s.b.size() - off);
        buf.insert(buf.end(), s.b.begin() + static_cast<long>(off), s.b.begin() + static_cast<long>(off + n));
        const size_t used = ps.feed(buf.data(), buf.size(),
                                    [&](uint64_t a, uint64_t p) { hits.emplace_back(a, p); });
        buf.erase(buf.begin(), buf.begin() + static_cast<long>(used));
        if (ps.malformed()) break;
    }
    return hits;
}

}  // namespace

int main() {
    const uint64_t A = 0x7f0000000000ull;
    Stream s;
    s.status(0);
    s.record(5);
    s.item(A + 0x1000);                                   // young 4 KiB
    s.item(A + 0x2000, /*old=*/true);                     // old: not reported
    s.run(A + 0x10000, 10);                               // young run of 10
    s.item(A + 0x200000, false, 9);                       // young THP: 512 pages
    s.item(A + 0x800000, false, 0, kClassPresent, 0x2f00000000000000ull);  // node 15, ref
    s.record(0);                                          // a VMA with nothing young
    s.record(2);
    s.item(A + 0x900000, false, 0, 3);                    // swap entry: skipped
    s.run(A + 0xa00000, 4, /*old=*/true);                 // old run: skipped

    const Hits want = {{A + 0x1000, 1}, {A + 0x10000, 10}, {A + 0x200000, 512}, {A + 0x800000, 1}};
    for (size_t piece : {s.b.size(), size_t{1}, size_t{7}, size_t{8}, size_t{24}, size_t{119},
                         size_t{120}, size_t{121}}) {
        StreamParser ps;
        const Hits got = parse(s, piece, ps);
        char what[96];
        std::snprintf(what, sizeof(what), "young pages, fed %zu bytes at a time", piece);
        check(got == want, what);
        check(ps.complete() && !ps.truncated() && !ps.malformed(), "complete stream");
        check(ps.records() == 3, "three records");
        check(ps.reached() == A + 0xa00000 + 4 * 4096, "reached = end of the last item");
    }

    {   // cut by the module: status bit 0
        Stream t;
        t.status(kStatusTruncated);
        t.record(1);
        t.item(A);
        StreamParser ps;
        const Hits got = parse(t, t.b.size(), ps);
        check(got.size() == 1 && ps.truncated(), "truncated status is reported");
        check(ps.reached() == A + 4096, "truncated: reached");
    }
    {   // the stream ends inside a record's items
        Stream t;
        t.status(0);
        t.record(3);
        t.item(A);
        StreamParser ps;
        parse(t, t.b.size(), ps);
        check(!ps.complete(), "missing items: not complete");
    }
    {   // a record layout this reader does not know
        Stream t;
        t.status(0);
        t.record(1, 0x41565459u);   // "YTVA": a name trailer follows
        t.item(A);
        StreamParser ps;
        const Hits got = parse(t, t.b.size(), ps);
        check(ps.malformed() && got.empty() && !ps.complete(), "foreign magic is malformed");
    }
    {   // a run of zero pages cannot come from the module
        Stream t;
        t.status(0);
        t.record(1);
        t.run(A, 0);
        StreamParser ps;
        parse(t, t.b.size(), ps);
        check(ps.malformed(), "zero-length run is malformed");
    }
    {   // nothing at all: not even the status word
        StreamParser ps;
        check(!ps.complete(), "empty output: not complete");
    }
    if (failures) return 1;
    std::printf("yt_stream_test: all checks passed\n");
    return 0;
}
