// stdio_partial_retry_test.cpp - smash's stdio EFAULT retry must never repeat,
// skip or hide data.
//
// fread/fwrite/fgets retry a call that failed with EFAULT, because glibc
// allocates a FILE's buffer lazily inside the first call and that buffer can
// land on a page the compressor owns (test_stdio_after_compression). A retry
// is only correct when the failed call moved nothing. Two ways it moved
// something while looking as if it had not:
//
//   - Item granularity. fread/fwrite return whole items, so a call with
//     size > 1 that transferred part of an item returns 0. fwrite(buf, 8192, 1)
//     that wrote 4096 bytes and then failed was retried in full: the first
//     4096 bytes went out twice. fread and fgets have the same shape.
//   - Buffered data. When glibc's flush fails it resets the write pointers, so
//     bytes an earlier call had already accepted are gone. A zero-progress
//     fwrite can therefore be the one that lost 4096 bytes, and retrying it
//     turns glibc's error into a success with a hole in the output.
//
// The EFAULTs are injected with fopencookie, which is what the kernel reports
// for a buffer on a compressed page, but deterministic. Each hazardous case is
// run through glibc's own function (dlopen'd, so it bypasses smash) and through
// the interposed one, and the two must agree on the return value, the error
// flag and every byte transferred. The positive controls check that the cases
// the retry exists for — a call that transferred nothing, on a stream with no
// buffer yet or with an empty one — still recover, exactly once.
//
// The functions are called through dlsym'd pointers: _FORTIFY_SOURCE turns a
// direct fread/fgets into __fread_chk/__fgets_chk, which smash deliberately
// does not wrap, so a direct call could bypass the code under test.
#define _GNU_SOURCE 1
#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <sys/types.h>

namespace {

using fwrite_fn = size_t (*)(const void*, size_t, size_t, FILE*);
using fread_fn = size_t (*)(void*, size_t, size_t, FILE*);
using fgets_fn = char* (*)(char*, int, FILE*);

int failures = 0;

void fail(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    std::fprintf(stderr, "FAIL: ");
    std::vfprintf(stderr, fmt, ap);
    std::fputc('\n', stderr);
    va_end(ap);
    ++failures;
}

// ── A cookie stream that fails exactly one call ─────────────────────────────
// Call number `fail_call` transfers `partial` bytes (0 = none) and reports
// EFAULT; every other call transfers everything asked of it. A short count
// with errno set is what a kernel write that faulted midway looks like; for a
// read, a short count followed by an EFAULT on the next call is.
struct Cookie {
    int calls = 0;
    int fail_call = 0;
    size_t partial = 0;
    bool fail_read_short = false;     // reads: the failing call returns -1
    unsigned char sink[65536];
    size_t delivered = 0;             // bytes written into sink
    size_t consumed = 0;              // bytes read from the source
};

unsigned char source(size_t pos) {
    return (pos % 64 == 63) ? '\n' : static_cast<unsigned char>('a' + pos % 26);
}

ssize_t cookieWrite(void* c, const char* buf, size_t n) {
    auto* k = static_cast<Cookie*>(c);
    ++k->calls;
    size_t take = n;
    const bool failing = k->calls == k->fail_call;
    if (failing) take = k->partial < n ? k->partial : n;
    if (k->delivered + take > sizeof(k->sink)) take = sizeof(k->sink) - k->delivered;
    std::memcpy(k->sink + k->delivered, buf, take);
    k->delivered += take;
    // A short count (0 included) is how glibc's own file write reports a
    // write(2) that failed: _IO_cookie_write sets the error flag for any short
    // count, and a -1 here would be stored in glibc's unsigned byte count.
    if (failing) errno = EFAULT;
    return static_cast<ssize_t>(take);
}

ssize_t cookieRead(void* c, char* buf, size_t n) {
    auto* k = static_cast<Cookie*>(c);
    ++k->calls;
    if (k->calls == k->fail_call) {
        if (k->fail_read_short) { errno = EFAULT; return -1; }
        // A short read that succeeded; the EFAULT comes on the next call.
        k->fail_call = k->calls + 1;
        k->fail_read_short = true;
        n = k->partial < n ? k->partial : n;
    }
    for (size_t i = 0; i < n; ++i) buf[i] = static_cast<char>(source(k->consumed + i));
    k->consumed += n;
    return static_cast<ssize_t>(n);
}

FILE* openCookie(Cookie* k, const char* mode, char* setbuf_to) {
    cookie_io_functions_t io{};
    io.read = cookieRead;
    io.write = cookieWrite;
    FILE* f = fopencookie(k, mode, io);
    if (f && setbuf_to) setvbuf(f, setbuf_to, _IOFBF, 4096);
    return f;
}

// What one run of a scenario produced.
struct Outcome {
    size_t ret = 0;          // items / non-NULL for fgets
    int error = 0;           // ferror right after the call
    size_t bytes = 0;        // bytes delivered after a flush (write) or returned (read)
    unsigned char data[16384];
};

unsigned char g_payload[16384];
char g_buf_ref[4096], g_buf_smash[4096];

// ── Writes ──────────────────────────────────────────────────────────────────
// pre_fill: bytes accepted into the stream's buffer before the failing call.
Outcome runWrite(fwrite_fn fw, char* setbuf_to, int fail_call, size_t partial,
                 size_t pre_fill, size_t size, size_t nmemb) {
    auto* k = new Cookie;
    k->fail_call = fail_call;
    k->partial = partial;
    FILE* f = openCookie(k, "w", setbuf_to);
    Outcome o;
    if (!f) { fail("fopencookie failed"); delete k; return o; }
    if (pre_fill) {
        // Fill without writing: glibc copies into the buffer while it fits.
        const size_t half = pre_fill / 2;
        fw(g_payload, 1, half, f);
        fw(g_payload + half, 1, pre_fill - half, f);
        if (k->calls != 0) fail("setup: pre-filling the buffer already wrote");
        k->fail_call = fail_call;
    }
    errno = 0;
    o.ret = fw(g_payload + pre_fill, size, nmemb, f);
    o.error = ferror(f) != 0;
    // What reaches the output is the property that matters, and a successful
    // call may still hold its bytes in the buffer: flush before counting.
    clearerr(f);
    fflush(f);
    o.bytes = k->delivered;
    std::memcpy(o.data, k->sink, o.bytes < sizeof(o.data) ? o.bytes : sizeof(o.data));
    fclose(f);
    delete k;
    return o;
}

void compareWrite(const char* name, fwrite_fn ref, fwrite_fn smash, bool buffered,
                  int fail_call, size_t partial, size_t pre_fill, size_t size, size_t nmemb) {
    const Outcome a = runWrite(ref, buffered ? g_buf_ref : nullptr, fail_call, partial,
                               pre_fill, size, nmemb);
    const Outcome b = runWrite(smash, buffered ? g_buf_smash : nullptr, fail_call, partial,
                               pre_fill, size, nmemb);
    std::printf("  %-44s glibc: ret=%zu err=%d out=%zu | smash: ret=%zu err=%d out=%zu\n",
                name, a.ret, a.error, a.bytes, b.ret, b.error, b.bytes);
    if (a.ret != b.ret || a.error != b.error || a.bytes != b.bytes ||
        std::memcmp(a.data, b.data, a.bytes < sizeof(a.data) ? a.bytes : sizeof(a.data)))
        fail("%s: smash's retry changed the outcome — glibc returns %zu with the error "
             "flag %s and %zu bytes written; smash returns %zu, error %s, %zu bytes",
             name, a.ret, a.error ? "set" : "clear", a.bytes, b.ret,
             b.error ? "set" : "clear", b.bytes);
}

// ── Reads ───────────────────────────────────────────────────────────────────
Outcome runRead(fread_fn fr, fgets_fn fg, char* setbuf_to, int fail_call, size_t partial,
                size_t size, size_t nmemb) {
    auto* k = new Cookie;
    k->fail_call = fail_call;
    k->partial = partial;
    k->fail_read_short = partial == 0;
    FILE* f = openCookie(k, "r", setbuf_to);
    Outcome o;
    if (!f) { fail("fopencookie failed"); delete k; return o; }
    errno = 0;
    std::memset(o.data, 0, sizeof(o.data));
    if (fr) {
        o.ret = fr(o.data, size, nmemb, f);
        o.bytes = o.ret * size;
    } else {
        char* s = fg(reinterpret_cast<char*>(o.data), static_cast<int>(size), f);
        o.ret = s != nullptr;
        o.bytes = s ? std::strlen(s) : 0;
    }
    o.error = ferror(f) != 0;
    fclose(f);
    delete k;
    return o;
}

// A read that reports data must report the stream's own first bytes.
void checkPrefix(const char* name, const Outcome& o) {
    for (size_t i = 0; i < o.bytes && i < sizeof(o.data); ++i) {
        if (o.data[i] != source(i)) {
            fail("%s: returned %zu bytes, but byte %zu is the stream's byte from later on "
                 "— the retry skipped data the failed call had consumed", name, o.bytes, i);
            return;
        }
    }
}

void compareRead(const char* name, fread_fn ref_r, fgets_fn ref_g, fread_fn smash_r,
                 fgets_fn smash_g, bool buffered, int fail_call, size_t partial,
                 size_t size, size_t nmemb) {
    const Outcome a = runRead(ref_r, ref_g, buffered ? g_buf_ref : nullptr, fail_call,
                              partial, size, nmemb);
    const Outcome b = runRead(smash_r, smash_g, buffered ? g_buf_smash : nullptr, fail_call,
                              partial, size, nmemb);
    std::printf("  %-44s glibc: ret=%zu err=%d | smash: ret=%zu err=%d\n",
                name, a.ret, a.error, b.ret, b.error);
    checkPrefix(name, b);
    if (a.ret != b.ret || a.error != b.error)
        fail("%s: smash's retry changed the outcome — glibc returns %zu with the error "
             "flag %s, smash returns %zu with it %s", name, a.ret,
             a.error ? "set" : "clear", b.ret, b.error ? "set" : "clear");
}

}  // namespace

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) {
        std::fprintf(stderr, "FAIL: not running under libsmash\n");
        return 1;
    }
    void* libc = dlopen("libc.so.6", RTLD_NOW | RTLD_NOLOAD);
    if (!libc) { std::printf("SKIP: cannot open libc.so.6\n"); return 77; }
    auto ref_w = reinterpret_cast<fwrite_fn>(dlsym(libc, "fwrite"));
    auto ref_r = reinterpret_cast<fread_fn>(dlsym(libc, "fread"));
    auto ref_g = reinterpret_cast<fgets_fn>(dlsym(libc, "fgets"));
    auto sm_w = reinterpret_cast<fwrite_fn>(dlsym(RTLD_DEFAULT, "fwrite"));
    auto sm_r = reinterpret_cast<fread_fn>(dlsym(RTLD_DEFAULT, "fread"));
    auto sm_g = reinterpret_cast<fgets_fn>(dlsym(RTLD_DEFAULT, "fgets"));
    if (!ref_w || !ref_r || !ref_g || !sm_w || !sm_r || !sm_g) {
        fail("dlsym of the stdio functions failed");
        return 1;
    }
    if (ref_w == sm_w || ref_r == sm_r || ref_g == sm_g) {
        fail("fwrite/fread/fgets are not interposed — the test would compare glibc "
             "with itself");
        return 1;
    }
    for (size_t i = 0; i < sizeof(g_payload); ++i)
        g_payload[i] = static_cast<unsigned char>(i * 37 + (i >> 8));

    std::printf("fwrite:\n");
    // The reported case: one 8 KiB item, half of it written before the fault.
    compareWrite("1 x 8192, 4096 written then EFAULT", ref_w, sm_w, true, 1, 4096, 0, 8192, 1);
    // Same on a stream whose buffer does not exist yet.
    compareWrite("1 x 8192, fresh stream, 4096 then EFAULT", ref_w, sm_w, false, 1, 4096, 0,
                 8192, 1);
    // Zero progress, but the failed flush discarded 4096 buffered bytes.
    compareWrite("full buffer, 1 byte, flush fails", ref_w, sm_w, true, 1, 0, 4096, 1, 1);
    compareWrite("half buffer, 4096 x 1, flush fails", ref_w, sm_w, true, 1, 0, 100, 4096, 1);

    std::printf("fread / fgets:\n");
    compareRead("fread 1 x 8192, fresh, 4096 then EFAULT", ref_r, nullptr, sm_r, nullptr,
                false, 1, 4096, 8192, 1);
    compareRead("fread 1 x 8192, buffered, 1000 then EFAULT", ref_r, nullptr, sm_r, nullptr,
                true, 1, 1000, 8192, 1);
    compareRead("fgets, buffered, 3 bytes then EFAULT", nullptr, ref_g, nullptr, sm_g,
                true, 1, 3, 100, 0);

    // ── Positive controls: the cases the retry exists for ───────────────────
    // A call that fails having moved nothing must still recover, both on a
    // stream with no buffer yet (glibc allocates it lazily, inside the call)
    // and on one whose buffer exists but is empty — the program blocked in
    // fgets(stdin) past the cold timeout, whose buffer was compressed while it
    // waited. A fix that only retried bufferless streams failed the second.
    for (int buffered = 0; buffered < 2; ++buffered) {
        char* sb = buffered ? g_buf_smash : nullptr;
        std::printf("recovery on a %s stream:\n", buffered ? "buffered, empty" : "fresh");
        {
            const Outcome o = runWrite(sm_w, sb, 1, 0, 0, 8192, 1);
            std::printf("  fwrite 1 x 8192, first write EFAULT: ret=%zu err=%d out=%zu\n",
                        o.ret, o.error, o.bytes);
            // Exactly 8192: the item is delivered once, not once per attempt.
            if (o.ret != 1 || o.error || o.bytes != 8192 || std::memcmp(o.data, g_payload, 8192))
                fail("an fwrite that failed before writing anything was not recovered "
                     "exactly once (ret=%zu, %zu bytes out)", o.ret, o.bytes);
        }
        {
            const Outcome o = runRead(sm_r, nullptr, sb, 1, 0, 8192, 1);
            std::printf("  fread 1 x 8192, first read EFAULT: ret=%zu err=%d\n", o.ret, o.error);
            if (o.ret != 1 || o.error)
                fail("fread that failed before reading anything was not recovered");
            checkPrefix("fread recovery", o);
        }
        {
            const Outcome o = runRead(nullptr, sm_g, sb, 1, 0, 100, 0);
            std::printf("  fgets, first read EFAULT: ret=%zu len=%zu\n", o.ret, o.bytes);
            if (o.ret != 1 || o.bytes != 64)
                fail("fgets that failed before reading anything was not recovered");
            checkPrefix("fgets recovery", o);
        }
        {
            // size 2: the whole call is the one character.
            const Outcome o = runRead(nullptr, sm_g, sb, 1, 0, 2, 0);
            std::printf("  fgets size 2, first read EFAULT: ret=%zu len=%zu\n", o.ret, o.bytes);
            if (o.ret != 1 || o.bytes != 1) fail("fgets(s, 2) was not recovered");
            checkPrefix("fgets size-2 recovery", o);
        }
    }

    if (failures) { std::printf("stdio_partial_retry_test: %d FAILURES\n", failures); return 1; }
    std::printf("stdio_partial_retry_test: all checks passed\n");
    return 0;
}
