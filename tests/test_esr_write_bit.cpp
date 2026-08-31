// aarch64 fault read/write classification.
//
// smash's soft-dirty ROI model wants to know whether a decompress fault was a
// READ or a WRITE: a read fault would have left the compressed blob valid, so
// scoring it as churn wrongly defers future compression of that page.
// signalHandler captures the answer into faultWasWrite() and
// compressor_thread.h's recordRedirtyForPage() consumes it, treating the
// unknown value (-1) as a write.
//
// On x86-64 the answer is bit 1 of the page-fault error code in
// gregs[REG_ERR]. On aarch64 it is bit 6 (WnR) of ESR_EL1, which the kernel
// hands to the signal handler in an esr_context record inside the chain of
// _aarch64_ctx records living in sigcontext::__reserved. Before
// aarch64EsrWriteBit() existed the aarch64 path had no decoder at all, so
// faultWasWrite() stayed -1 for every fault and the ROI model ran blind.
//
// This test drives the decoder with synthetic signal frames. It is
// arch-independent on purpose — the decoder is a pure function over bytes, so
// the aarch64-specific logic is verified on whatever host CI runs, not only on
// aarch64 hardware where no CI runner exists today.
//
// Layouts are from the kernel's arch/arm64/include/uapi/asm/sigcontext.h:
//     struct _aarch64_ctx { u32 magic; u32 size; };
//     struct esr_context  { struct _aarch64_ctx head; u64 esr; };
// with FPSIMD_MAGIC / ESR_MAGIC / EXTRA_MAGIC as below, and magic == 0
// terminating the chain.

#include "vm/fault_handler.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

using smash::vm::aarch64EsrWriteBit;

static int failures = 0;

#define CHECK_EQ(actual, expected, msg, ...) do { \
    int _a = (actual); \
    int _e = (expected); \
    if (_a != _e) { \
        fprintf(stderr, "FAIL: " msg " (got %d, want %d)\n", ##__VA_ARGS__, \
                _a, _e); \
        ++failures; \
    } \
} while (0)

namespace {

constexpr uint32_t kFpsimdMagic = 0x46508001u;
constexpr uint32_t kEsrMagic    = 0x45535201u;
constexpr uint32_t kExtraMagic  = 0x45585401u;

// Exception classes from ESR_EL1 bits 31:26.
constexpr uint32_t kEcInstAbortLower = 0x20;
constexpr uint32_t kEcInstAbortSame  = 0x21;
constexpr uint32_t kEcDataAbortLower = 0x24;  // userspace fault: what we get
constexpr uint32_t kEcDataAbortSame  = 0x25;

// A whole sigcontext::__reserved area, built record by record.
class Frame {
public:
    Frame() : buf_(kReservedLen, 0) {}

    // Append a record header plus `payload_len` bytes of body. The kernel
    // rounds record sizes to 16; we mirror that so offsets look realistic.
    void addRecord(uint32_t magic, uint32_t size) {
        writeHeader(magic, size);
        pos_ += size;
    }

    void addEsr(uint64_t esr, uint32_t size = 16) {
        writeHeader(kEsrMagic, size);
        std::memcpy(buf_.data() + pos_ + 8, &esr, sizeof(esr));
        pos_ += size;
    }

    // The kernel's fpsimd_context is 528 bytes and is emitted first, before
    // esr_context. Including it keeps the tests honest about the decoder
    // having to walk past a real leading record.
    void addFpsimd() { addRecord(kFpsimdMagic, 528); }

    // magic == 0, size == 0 ends the chain.
    void terminate() { writeHeader(0, 0); }

    const void* data() const { return buf_.data(); }
    size_t size() const { return buf_.size(); }

private:
    void writeHeader(uint32_t magic, uint32_t size) {
        std::memcpy(buf_.data() + pos_, &magic, sizeof(magic));
        std::memcpy(buf_.data() + pos_ + 4, &size, sizeof(size));
    }

    static constexpr size_t kReservedLen = 4096;  // sigcontext::__reserved
    std::vector<unsigned char> buf_;
    size_t pos_ = 0;
};

uint64_t makeEsr(uint32_t ec, bool wnr, bool cm = false, uint32_t dfsc = 0x07) {
    uint64_t esr = static_cast<uint64_t>(ec) << 26;
    if (wnr) esr |= 1ull << 6;
    if (cm)  esr |= 1ull << 8;
    esr |= dfsc;  // DFSC: 0b000111 = translation fault, level 3
    return esr;
}

// ── The two answers that matter ─────────────────────────────────────────────
// A userspace store to a PROT_NONE page raises a data abort from a lower
// exception level with WnR set; a load raises the same abort with WnR clear.
// These are the exact frames smash sees when the compressor has compressed a
// page and the application touches it again.
void testWriteAndReadFaults() {
    {
        Frame f;
        f.addFpsimd();
        f.addEsr(makeEsr(kEcDataAbortLower, /*wnr=*/true));
        f.terminate();
        CHECK_EQ(aarch64EsrWriteBit(f.data(), f.size()), 1,
                 "store to a compressed page must classify as a write");
    }
    {
        Frame f;
        f.addFpsimd();
        f.addEsr(makeEsr(kEcDataAbortLower, /*wnr=*/false));
        f.terminate();
        CHECK_EQ(aarch64EsrWriteBit(f.data(), f.size()), 0,
                 "load from a compressed page must classify as a read");
    }
    {
        // Data abort taken at the same exception level. Not what an app fault
        // looks like, but the field layout is identical and the decoder should
        // not care.
        Frame f;
        f.addEsr(makeEsr(kEcDataAbortSame, /*wnr=*/true));
        f.terminate();
        CHECK_EQ(aarch64EsrWriteBit(f.data(), f.size()), 1,
                 "EC=0x25 data abort must decode like EC=0x24");
    }
}

// An instruction abort is a fetch, never a store — report read, not unknown.
void testInstructionAbort() {
    for (uint32_t ec : {kEcInstAbortLower, kEcInstAbortSame}) {
        Frame f;
        f.addFpsimd();
        // WnR is RES0 for an instruction abort; set it to prove we ignore it.
        f.addEsr(makeEsr(ec, /*wnr=*/true));
        f.terminate();
        CHECK_EQ(aarch64EsrWriteBit(f.data(), f.size()), 0,
                 "instruction abort (EC=0x%02x) is a fetch, not a store", ec);
    }
}

// ── Cases that must report unknown rather than guess ────────────────────────
// The reader treats -1 as "write", which is the conservative direction (it
// defers compression). Guessing "read" where we do not know would be the
// unsafe error: it would let a genuinely re-dirtied page keep a stale blob's
// worth of ROI credit.
void testUnknownCases() {
    {
        // CM (bit 8): a fault from a cache-maintenance or address-translation
        // instruction. The architecture reports WnR == 1 for these regardless
        // of the actual direction, so the bit carries no information.
        Frame f;
        f.addEsr(makeEsr(kEcDataAbortLower, /*wnr=*/true, /*cm=*/true));
        f.terminate();
        CHECK_EQ(aarch64EsrWriteBit(f.data(), f.size()), -1,
                 "CM=1 makes WnR meaningless; must not claim a write");
    }
    {
        // Some other exception class entirely (0x00 = unknown reason).
        Frame f;
        f.addEsr(makeEsr(0x00, /*wnr=*/true));
        f.terminate();
        CHECK_EQ(aarch64EsrWriteBit(f.data(), f.size()), -1,
                 "non-abort exception class must not be classified");
    }
    {
        // No esr_context at all. The kernel only emits one when
        // current->thread.fault_code is set, so this frame shape is real.
        Frame f;
        f.addFpsimd();
        f.terminate();
        CHECK_EQ(aarch64EsrWriteBit(f.data(), f.size()), -1,
                 "frame without an esr_context must report unknown");
    }
    {
        // Empty chain.
        Frame f;
        f.terminate();
        CHECK_EQ(aarch64EsrWriteBit(f.data(), f.size()), -1,
                 "empty chain must report unknown");
    }
    {
        // EXTRA_MAGIC continues the chain in an out-of-line buffer we do not
        // follow. The kernel emits esr_context before it, so hitting it first
        // means there is nothing for us to find.
        Frame f;
        f.addFpsimd();
        f.addRecord(kExtraMagic, 16);
        f.terminate();
        CHECK_EQ(aarch64EsrWriteBit(f.data(), f.size()), -1,
                 "EXTRA_MAGIC must end the walk, not be treated as a record");
    }
    {
        CHECK_EQ(aarch64EsrWriteBit(nullptr, 4096), -1,
                 "null frame must report unknown");
    }
}

// ── Malformed frames must not read out of bounds ────────────────────────────
// This runs inside a SIGSEGV handler. A frame the decoder mis-parses must end
// the walk, never wander off the end of __reserved — a fault there would
// re-enter the handler and take the process down.
void testMalformedFramesAreBounded() {
    {
        // size < the 8-byte header.
        Frame f;
        f.addRecord(kFpsimdMagic, 4);
        CHECK_EQ(aarch64EsrWriteBit(f.data(), f.size()), -1,
                 "undersized record must abort the walk");
    }
    {
        // size runs past the end of __reserved.
        Frame f;
        f.addRecord(kFpsimdMagic, 8192);
        CHECK_EQ(aarch64EsrWriteBit(f.data(), f.size()), -1,
                 "oversized record must abort the walk");
    }
    {
        // A valid ESR magic whose record is too short to hold the esr field.
        Frame f;
        f.addRecord(kEsrMagic, 8);
        f.terminate();
        CHECK_EQ(aarch64EsrWriteBit(f.data(), f.size()), -1,
                 "truncated esr_context must report unknown");
    }
    {
        // Buffer shorter than a single header.
        Frame f;
        f.addFpsimd();
        CHECK_EQ(aarch64EsrWriteBit(f.data(), 4), -1,
                 "buffer too small for a header must report unknown");
        CHECK_EQ(aarch64EsrWriteBit(f.data(), 0), -1,
                 "zero-length buffer must report unknown");
    }
    {
        // A chain of minimum-size records that never terminates. The loop cap
        // must stop it; without one this walks to the end of the buffer.
        Frame f;
        for (int i = 0; i < 400; ++i) f.addRecord(kFpsimdMagic, 8);
        CHECK_EQ(aarch64EsrWriteBit(f.data(), f.size()), -1,
                 "non-terminating chain must be bounded by the iteration cap");
    }
}

// The decoder has to walk past however many records precede esr_context.
void testEsrFoundAfterLeadingRecords() {
    Frame f;
    f.addFpsimd();
    f.addRecord(0x53564501u, 144);  // SVE_MAGIC, plausible size
    f.addEsr(makeEsr(kEcDataAbortLower, /*wnr=*/true));
    f.terminate();
    CHECK_EQ(aarch64EsrWriteBit(f.data(), f.size()), 1,
             "esr_context must be found behind other records");
}

}  // namespace

int main() {
    printf("test_esr_write_bit: decoding synthetic aarch64 signal frames\n");

    testWriteAndReadFaults();
    testInstructionAbort();
    testUnknownCases();
    testMalformedFramesAreBounded();
    testEsrFoundAfterLeadingRecords();

    if (failures) {
        printf("test_esr_write_bit: %d FAILURES\n", failures);
        return 1;
    }
    printf("test_esr_write_bit: all checks passed\n");
    return 0;
}
