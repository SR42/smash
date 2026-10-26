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
// That each young item's first word reaches the caller, node bits [59:56]
// included (SMASH_YT_NODE_HINTS reads them). And that the module's mapped
// buffer (SMASH_YT_MMAP), the same stream followed by zeros to the buffer's
// end, or by one zero word and an earlier scan's records (the module since yt
// 992e189), reads the same: a zero word where a record would start ends it.
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
using Words = std::vector<uint64_t>;

// Feed `s` in pieces of `piece` bytes the way ytRequest does: keep what the
// parser did not consume in front of the next piece.
Hits parse(const Stream& s, size_t piece, StreamParser& ps, Words* words = nullptr) {
    Hits hits;
    std::vector<unsigned char> buf;
    for (size_t off = 0; off < s.b.size(); off += piece) {
        const size_t n = std::min(piece, s.b.size() - off);
        buf.insert(buf.end(), s.b.begin() + static_cast<long>(off), s.b.begin() + static_cast<long>(off + n));
        const size_t used = ps.feed(buf.data(), buf.size(),
                                    [&](uint64_t a, uint64_t p, uint64_t w) {
                                        hits.emplace_back(a, p);
                                        if (words) words->push_back(w);
                                    });
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

    {   // each young item's first word: the node bits of the last one
        Words words;
        StreamParser ps;
        parse(s, s.b.size(), ps, &words);
        check(words.size() == 4 && (words[3] >> kItemNodeShift & kItemNodeMask) == 15 &&
                  (words[0] >> kItemNodeShift & kItemNodeMask) == 0 && (words[1] & kItemRun),
              "the item word reaches the caller, node bits and run bit included");
    }
    {   // the mapped buffer: the stream, then zeros to the buffer's end
        Stream m = s;
        m.b.resize(m.b.size() + 4096, 0);
        StreamParser ps;
        Hits got;
        const size_t used = ps.feed(m.b.data(), m.b.size(),
                                    [&](uint64_t a, uint64_t p, uint64_t) { got.emplace_back(a, p); });
        check(got == want, "mapped buffer: young pages");
        check(ps.complete() && !ps.truncated() && !ps.malformed() && ps.ended(),
              "mapped buffer: the zero padding ends a complete stream");
        check(used == s.b.size(), "mapped buffer: nothing past the stream is read");
    }
    {   // yt 992e189 on: one zero word, then an earlier, longer scan's output
        Stream m = s;
        m.u64(0);
        Stream old;
        old.record(2);
        old.item(A + 0x5000000);
        old.run(A + 0x6000000, 8);
        m.b.insert(m.b.end(), old.b.begin(), old.b.end());
        StreamParser ps;
        Hits got;
        const size_t used = ps.feed(m.b.data(), m.b.size(),
                                    [&](uint64_t a, uint64_t p, uint64_t) { got.emplace_back(a, p); });
        check(got == want, "mapped buffer: an earlier scan's records past the terminator are not read");
        check(ps.complete() && ps.ended() && ps.records() == 3 && used == s.b.size(),
              "mapped buffer: the zero word ends the stream");
    }
    {   // padding inside a record's items is not an end
        Stream m;
        m.status(0);
        m.record(2);
        m.item(A);
        m.b.resize(m.b.size() + 4096, 0);
        StreamParser ps;
        ps.feed(m.b.data(), m.b.size(), [](uint64_t, uint64_t, uint64_t) {});
        check(!ps.complete(), "mapped buffer: zeros where an item is due are not complete");
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
