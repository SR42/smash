#pragma once
// yt_stream.h - reader for the output of youngtrace's /dev/youngtrace/scanner,
// the access source behind SMASH_WRITE_TRACK=yt.
//
// youngtrace is a separate loadable module (the `yt` tree). A write to its
// device is one scan request ("pid=N mode=1 lo=0x.. hi=0x.."): the module walks
// that process's page tables under mmap_read_lock, reports the present pages
// whose ACCESSED bit was set, and (mode=1) clears the bit, without a TLB flush.
// A read then returns the scan as a little-endian stream (or the descriptor's
// buffer, mmap'd, holds it, followed by a zero word unless it fills the
// buffer; since yt 992e189 only that one word is zeroed, and the bytes after it
// may still hold an earlier, longer scan's output):
//
//   u64 status            bit 0: the output was cut at the module's buffer cap
//   per VMA intersecting [lo, hi):
//     120-byte record     magic "YTV9" at 0, the number of payload items at 32
//     items               one u64, or three when bit 63 is set (a run):
//                           w0  address | metadata (low 12 bits, high 8 bits)
//                           w1  pages, in 4 KiB units        (runs only)
//                           w2  w0's low 12 bits again       (runs only)
//
// Item metadata: [2:0] class (1 = present), [3] old (ACCESSED was clear),
// [10:6] log2(page bytes) - 12 (a THP item is one entry of 512 pages),
// [59:56] NUMA node (valid when the module's page_meta parameter is Y; a
// run's is its first page's: node is not part of the run key).
//
// The constants below are the wire values of the module's include/format.h
// (its v5 block); they are restated rather than included, so smash builds
// without the yt tree. A layout change there moves the record magic, which
// this reader refuses (malformed()) rather than guessing at.
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace smash::yt {

static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "the scanner stream is little-endian");

inline constexpr uint32_t kRecordMagic = 0x39565459u;      // "YTV9"
inline constexpr size_t kStatusBytes = 8;
inline constexpr size_t kRecordBytes = 120;
inline constexpr size_t kRecordStartOffset = 8;
inline constexpr size_t kRecordItemsOffset = 32;
inline constexpr uint64_t kStatusTruncated = 1;
inline constexpr uint64_t kItemRun = 1ull << 63;
inline constexpr uint64_t kItemClassMask = 0x7;
inline constexpr uint64_t kClassPresent = 1;
inline constexpr uint64_t kClassProtNone = 2;
inline constexpr uint64_t kItemOld = 0x8;
inline constexpr unsigned kItemSizeShift = 6;
inline constexpr uint64_t kItemSizeMask = 0x07c0;
inline constexpr uint64_t kItemAddrMask = ~(0x0fffull | (0xffull << 56));
inline constexpr unsigned kItemNodeShift = 56;
inline constexpr uint64_t kItemNodeMask = 0xf;
// Upper bound on the output per 4 KiB page of a request: a record of its own
// (every page a separate VMA) plus a single-word item. Runs are 3 words for at
// least 4 pages. A request of at most (cap - 8) / 128 pages therefore always
// fits the module's buffer and is never cut.
inline constexpr size_t kMaxBytesPerPage = kRecordBytes + 8;

// Incremental parser: feed() whatever a read returned, keep the bytes it did
// not consume (an incomplete record or item) in front of the next read; or
// feed the whole mapped buffer once. For every young present item it calls
// on_young(addr, pages_4k, word), word being the item's first u64 (address
// and metadata). A zero word where a record would start is the mapped
// buffer's terminator: the stream ends there (ended()), and nothing after it
// is read.
class StreamParser {
public:
    template <class F>
    size_t feed(const unsigned char* p, size_t n, F&& on_young) {
        size_t off = 0;
        while (!malformed_) {
            const size_t left = n - off;
            if (!have_status_) {
                if (left < kStatusBytes) break;
                truncated_ = (word(p + off) & kStatusTruncated) != 0;
                have_status_ = true;
                off += kStatusBytes;
            } else if (items_left_ == 0) {
                if (ended_) break;
                if (left >= 8 && word(p + off) == 0) { ended_ = true; break; }
                if (left < kRecordBytes) break;
                uint32_t magic;
                std::memcpy(&magic, p + off, sizeof(magic));
                if (magic != kRecordMagic) { malformed_ = true; break; }
                items_left_ = word(p + off + kRecordItemsOffset);
                last_start_ = word(p + off + kRecordStartOffset);
                last_items_ = items_left_;
                last_end_ = 0;
                ++records_;
                off += kRecordBytes;
            } else {
                if (left < 8) break;
                const uint64_t w = word(p + off);
                // Address 0 is never mapped, so no item is all zeros: this
                // is padding where an item is due.
                if (w == 0) { malformed_ = true; break; }
                uint64_t pages;
                size_t len = 8;
                if (w & kItemRun) {
                    if (left < 24) break;
                    pages = word(p + off + 8);
                    len = 24;
                } else {
                    const uint64_t cls = w & kItemClassMask;
                    pages = (cls == kClassPresent || cls == kClassProtNone)
                        ? 1ull << ((w & kItemSizeMask) >> kItemSizeShift) : 1;
                }
                const uint64_t addr = w & kItemAddrMask;
                const uint64_t end = addr + pages * 4096;
                if (pages == 0 || end < addr) { malformed_ = true; break; }
                if (end > reached_) reached_ = end;
                if (end > last_end_) last_end_ = end;
                if ((w & kItemClassMask) == kClassPresent && !(w & kItemOld))
                    on_young(addr, pages, w);
                --items_left_;
                off += len;
            }
        }
        return off;
    }

    bool truncated() const { return truncated_; }
    bool malformed() const { return malformed_; }
    // The zero word after the last record was reached (mapped buffer).
    bool ended() const { return ended_; }
    // A complete stream ends on a record boundary with every item read.
    bool complete() const { return have_status_ && items_left_ == 0 && !malformed_; }
    // End of the furthest item seen (0 if none): where a cut scan stopped
    // reporting. Pages from there to the request's end are unknown, and in
    // mode=1 their ACCESSED bits may already have been cleared.
    uint64_t reached() const { return reached_; }
    // Where a cut scan resumes, as youngtrace's chunk_resume_v5() does: the
    // end of the last record's furthest item, or that record's start when it
    // has none (the cap fell right after it). 0 before any record. Exact only
    // with a module that stops before what it has no room to report (yt
    // c56f1b6, #86): an older one cleared bits past this point that it never
    // reported.
    uint64_t resumePoint() const { return last_items_ ? last_end_ : last_start_; }
    uint64_t records() const { return records_; }

private:
    static uint64_t word(const unsigned char* p) {
        uint64_t v;
        std::memcpy(&v, p, sizeof(v));
        return v;   // little-endian hosts only (x86-64, arm64)
    }
    bool have_status_ = false;
    bool truncated_ = false;
    bool malformed_ = false;
    bool ended_ = false;
    uint64_t items_left_ = 0;
    uint64_t reached_ = 0;
    uint64_t records_ = 0;
    uint64_t last_start_ = 0;   // the last record's start (clipped to the request)
    uint64_t last_items_ = 0;   // and its item count
    uint64_t last_end_ = 0;     // the end of its furthest item
};

}  // namespace smash::yt
