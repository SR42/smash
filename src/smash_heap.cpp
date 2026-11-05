// smash/src/smash_heap.cpp - SmashHeap alloc8 integration + thread cache methods
#include <new>
#include "smash_heap.h"
#include "vm/syscall_compat.h"
#include <alloc8/alloc8.h>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <unistd.h>

#ifdef __APPLE__
#include <sys/types.h>
#include <sys/event.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/mount.h>
#include <sys/attr.h>
#include <sys/wait.h>
#include <sys/resource.h>
#include <sys/statvfs.h>
#include <poll.h>
#include <mach/mach.h>
#include <mach/mach_vm.h>
#endif

// ── Thread init counter for deferred compression start ──────────────────────
// pthread_create during early DYLD_INSERT init crashes the ObjC runtime's
// task_restartable_ranges_register on macOS. We count threadInit() calls
// and only start compression after the second call (first = main thread
// during early init, subsequent = real threads after init is complete).

std::atomic<int> smash::g_thread_init_count{0};

// mpaland/printf requires _putchar() to satisfy printf_() / vprintf_() which
// are referenced unconditionally inside printf.cpp. We never call those
// stdout-facing forms (we only use snprintf_/vsnprintf_ via safe_printf.h),
// so this implementation is a stub: write the byte to stderr through write(2)
// without going through stdio.
extern "C" void _putchar(char c) {
    (void)::write(STDERR_FILENO, &c, 1);
}

// ── System allocator function pointers for compress-only mode ───────────────
smash::SystemAllocFns smash::g_system_alloc;

// Early init: resolve system malloc/free before alloc8 interposition.
// Needed for compress-only mode and large-only mode (small alloc passthrough).
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wprio-ctor-dtor"
__attribute__((constructor(50)))  // Run before alloc8 (priority 101)
static void smash_resolve_system_alloc() {
    smash::g_system_alloc.resolve();
}

// SMASH_BANNER=1: print a one-line banner at library-load time so it's
// visible whether DYLD_INSERT_LIBRARIES / LD_PRELOAD actually loaded
// libsmash. Especially useful for multi-process apps like Firefox where
// the launcher may exec a different binary or the path may not be picked
// up at all. Prints to stderr via direct write() to be safe in early
// init (before stdio is fully wired up).
__attribute__((constructor(60)))
static void smash_print_banner() {
    const char* on = std::getenv("SMASH_BANNER");
    if (!on || on[0] != '1') return;
    char ts[32] = {};
    smash::vm::formatTimestamp(ts, sizeof(ts));
    char buf[320];
    int n = std::snprintf(buf, sizeof(buf),
        "[smash] [%s] loaded pid=%d ppid=%d "
#ifdef __APPLE__
        "platform=darwin"
#else
        "platform=linux"
#endif
        "\n",
        ts, (int)getpid(), (int)getppid());
    if (n > 0) (void)!write(2, buf, (size_t)n);
}
#pragma GCC diagnostic pop

// ── Thread cache methods that depend on Slab ─────────────────────────────────

namespace smash {

void* ThreadCache::refill(uint8_t sc, uint8_t arena, Slab* slab) {
    // Batch-allocate from slab into the arena-specific lane
    auto c = lane(idx(arena, sc));
    size_t batch = kThreadCacheBatchSize;
    if (batch > static_cast<size_t>(kPerLaneDepth)) batch = kPerLaneDepth;

    // Magazine fast path: try to satisfy the refill from the lock-free depot
    // (a batch a peer thread freed). No slab lock_, no bitmap scan.
    if (Slab::magazineEnabled()) {
        size_t got = slab->allocateBatchMag(c.ptrs);
        if (got > 0) {
            c.count = static_cast<uint16_t>(got - 1);
            return c.ptrs[got - 1];
        }
    }

    size_t got = slab->allocateBatch(c.ptrs, batch);
    if (got == 0) return nullptr;

    // Return one, keep the rest in cache
    c.count = static_cast<uint16_t>(got - 1);
    return c.ptrs[got - 1];
}

namespace {

// Return the `n` pointers at `ptrs[start..start+n)` to their arenas' slabs for
// size class `sc`. Resolves each span exactly once and groups by arena with a
// counting sort into small fixed-size stack buffers (~10 KB), then dispatches
// one deallocateBatchResolved() per non-empty arena.
//
// Replaces the previous approach, which (1) allocated a
// kTotalArenas x kThreadCacheMaxPerClass bucket matrix — 256 KB zeroed on every
// drain — and (2) re-resolved every pointer's span a second time inside
// deallocateBatch. n is bounded by kThreadCacheMaxPerClass (cache capacity),
// so all buffers are statically sized.
// Span of a small-object pointer: the flat contiguous-arena table when the
// pointer is in it (one load), else the PageMap radix walk.
inline Span* resolveSpanForDrain(PageMap* page_map, void* p) {
    const uintptr_t a = reinterpret_cast<uintptr_t>(p);
    const uintptr_t off = a - g_fast_span_table.base;
    if (off < g_fast_span_table.size)
        return g_fast_span_table.table[off >> kPageShift].load(std::memory_order_acquire);
    return page_map->get(a);
}

// Hand one arena's resolved batch to its slab.
void dispatchToSlab(Slab& slab, void** ptrs, Span** spans, size_t n) {
    // SMASH_DEFER_FREE (default on): on slab-lock contention, push the resolved
    // batch onto the slab's lock-free pending stack instead of blocking. The
    // next lock holder applies it. Removes the per-arena slab lock from the
    // contended cross-thread-free path (Larson @ high thread counts). Off uses
    // the blocking resolved path for A/B comparison.
    // Hand-rolled lazy init (not a magic static): drains run on the
    // cross-thread-free path, so skip the guard-variable acquire-load.
    static std::atomic<int> defer_free_cached{-1};
    int defer_free = defer_free_cached.load(std::memory_order_relaxed);
    if (defer_free < 0) [[unlikely]] {
        const char* v = std::getenv("SMASH_DEFER_FREE");
        defer_free = (!v || v[0] != '0') ? 1 : 0;
        defer_free_cached.store(defer_free, std::memory_order_relaxed);
    }
    // Magazine fast path: stash the freed batch in the lock-free depot for a
    // peer thread's next refill. Falls through to the real free path only if
    // the depot is full (bounds memory).
    if (Slab::magazineEnabled() && slab.deallocateBatchMag(ptrs, spans, n))
        return;
    if (defer_free)
        slab.deallocateBatchDeferred(ptrs, spans, n);
    else
        slab.deallocateBatchResolved(ptrs, spans, n);
}

// Return the `n` pointers at `ptrs[start..start+n)` to their arenas' slabs for
// size class `sc`. Resolves each span exactly once. The common case -- every
// pointer from one arena (a lane holds one arena unless arenas outnumber
// lanes) -- goes straight to that slab; otherwise group by arena with a
// counting sort into small fixed-size stack buffers and dispatch one batch
// per non-empty arena. n is bounded by kThreadCacheMaxPerClass (cache
// capacity), so all buffers are statically sized.
void drainRangeToSlabs(void** ptrs, size_t start, size_t n,
                       uint8_t sc, Slab* all_slabs, PageMap* page_map) {
    if (n == 0) return;
    const int total_arenas = getTotalArenas();

    Span* spans[kThreadCacheMaxPerClass];
    uint8_t arenas[kThreadCacheMaxPerClass];
    bool one_arena = true;
    for (size_t i = 0; i < n; ++i) {
        Span* span = resolveSpanForDrain(page_map, ptrs[start + i]);
        // arena_id is in [0, getTotalArenas()) when cold-arena feedback is on.
        uint8_t arena = (span && !span->is_large) ? span->arena_id : 0;
        if (arena >= total_arenas) arena = 0;  // defensive
        spans[i] = span;
        arenas[i] = arena;
        one_arena &= (arena == arenas[0]);
    }
    if (one_arena) {
        dispatchToSlab(all_slabs[arenas[0] * kNumClasses + sc], ptrs + start, spans, n);
        return;
    }

    uint32_t counts[kTotalArenas]{};
    for (size_t i = 0; i < n; ++i) counts[arenas[i]]++;
    // Prefix sums → per-arena offsets, then scatter into arena-contiguous order.
    uint32_t offsets[kTotalArenas];
    uint32_t cursor[kTotalArenas];
    uint32_t acc = 0;
    for (int a = 0; a < total_arenas; ++a) { offsets[a] = acc; cursor[a] = acc; acc += counts[a]; }

    void* out_ptrs[kThreadCacheMaxPerClass];
    Span* out_spans[kThreadCacheMaxPerClass];
    for (size_t i = 0; i < n; ++i) {
        const uint32_t pos = cursor[arenas[i]]++;
        out_ptrs[pos] = ptrs[start + i];
        out_spans[pos] = spans[i];
    }
    for (int a = 0; a < total_arenas; ++a) {
        if (counts[a] == 0) continue;
        dispatchToSlab(all_slabs[a * kNumClasses + sc],
                       out_ptrs + offsets[a], out_spans + offsets[a], counts[a]);
    }
}

} // namespace

void ThreadCache::drain(uint8_t sc, uint8_t arena, Slab* all_slabs, PageMap* page_map) {
    auto c = lane(idx(arena, sc));
    // Drain half the lane
    size_t to_drain = c.count / 2;
    if (to_drain == 0) to_drain = c.count;
    const size_t start = c.count - to_drain;
    drainRangeToSlabs(c.ptrs, start, to_drain, sc, all_slabs, page_map);
    c.count = static_cast<uint16_t>(start);
}

void ThreadCache::drainAll(Slab* all_slabs, PageMap* page_map) {
    // Sweep every (lane, size class) — pointers live in lane(ln*kNumClasses+sc)
    // for lane = arena & (kCacheLanes-1). The old loop covered only lane 0's
    // classes (i < kNumClasses), leaking every non-zero-lane pointer on thread
    // teardown. drainRangeToSlabs re-resolves each span's true arena, so passing
    // sc alone is sufficient for correct routing back to the slabs.
    for (int ln = 0; ln < kCacheLanes; ++ln) {
        for (int sc = 0; sc < kNumClasses; ++sc) {
            auto c = this->lane(ln * kNumClasses + sc);
            if (c.count > 0) {
                drainRangeToSlabs(c.ptrs, 0, c.count, static_cast<uint8_t>(sc), all_slabs, page_map);
                c.count = 0;
            }
        }
    }
}

} // namespace smash

// ── Global VmRegion pointer for syscall wrappers ─────────────────────────────

smash::VmRegion* smash::g_smash_vm_region = nullptr;

// Set when compression is initialized; nullptr in pass-through builds where
// SmashHeap never wires up page_states_. The mmap / Mach VM interposers
// gate every state mutation on this being non-null.
smash::PageStateTable* smash::g_smash_page_states_for_external = nullptr;

// See header. Published alongside g_smash_page_states_for_external so the
// munmap / Mach VM deallocate interposers can take the per-page lock while
// untracking an external page, serializing against compressPage's snapshot.
smash::PageLockTable* smash::g_smash_page_locks_for_external = nullptr;

// g_smash_skip_external_tracking is now an inline variable defined in
// compress/compressor_thread.h (single definition across libsmash and the
// compress-only build); no separate definition needed here.

// TLS for the malloc fast path.  initial-exec model: libsmash is always
// LD_PRELOAD'd, so its TLS block is part of the program's startup TLS
// reservation, and accesses use a direct tpidr_el0 + offset load instead
// of a __tls_get_addr indirection.
#if !defined(SMASH_USE_TSD)
__attribute__((tls_model("initial-exec")))
thread_local smash::ThreadCache* smash::g_thread_cache = nullptr;

__attribute__((tls_model("initial-exec")))
thread_local int smash::g_full_mode_cached = -1;

__attribute__((tls_model("initial-exec")))
thread_local smash::ThreadCache* smash::g_fast_tc = nullptr;

// g_compressor_thread is now defined as inline thread_local in
// compressor_thread.h (C++17 inline variable, single definition).

__attribute__((tls_model("initial-exec")))
thread_local uintptr_t smash::g_arena_stack_base = 0;
#endif  // !SMASH_USE_TSD

#if !defined(SMASH_USE_TSD)
__attribute__((tls_model("initial-exec")))
thread_local smash::ArenaMemo smash::g_arena_memo[smash::kArenaMemoSlots] = {};
#endif

// ── Syscall interposition for kernel buffer compatibility ────────────────────
//
// Kernel syscalls access userspace buffers directly without triggering SIGSEGV.
// We interpose on syscalls that read/write userspace buffers and touch each
// Smash-managed page first to ensure PROT_READ|PROT_WRITE.

#ifdef __APPLE__

// ── Interpose data structure (same pattern as alloc8) ────────────────────────

typedef struct {
    void* replacement;
    void* original;
} smash_interpose_t;

#define SMASH_INTERPOSE(replacement, original) \
    __attribute__((used)) \
    static const smash_interpose_t smash_interpose_##replacement \
    __attribute__((section("__DATA, __interpose"))) = { \
        reinterpret_cast<void*>(replacement), \
        reinterpret_cast<void*>(original) \
    }

// To call the real (original) function from within an interpose wrapper, we
// read the .original field of the interpose struct. The linker resolves this
// to the real function address at load time, before dyld processes the section.
// This avoids dlsym(RTLD_NEXT) which returns our own wrapper on macOS.
//
// Pattern: forward-declare wrapper, define interpose entry, define wrapper
// body that reads interpose_entry.original.

// ── kevent ───────────────────────────────────────────────────────────────────

extern "C" int smash_kevent(int kq, const struct kevent* changelist, int nchanges,
                            struct kevent* eventlist, int nevents,
                            const struct timespec* timeout);
SMASH_INTERPOSE(smash_kevent, kevent);

using kevent_fn = int(*)(int, const struct kevent*, int, struct kevent*, int, const struct timespec*);

extern "C" int smash_kevent(int kq, const struct kevent* changelist, int nchanges,
                            struct kevent* eventlist, int nevents,
                            const struct timespec* timeout) {
    auto* vm = smash::g_smash_vm_region;
    int ret = smash::vm::retryWithDecompress(
        [&] {
            return reinterpret_cast<kevent_fn>(smash_interpose_smash_kevent.original)(
                kq, changelist, nchanges, eventlist, nevents, timeout);
        },
        [&] {
            if (vm) {
                if (changelist && nchanges > 0)
                    smash::vm::walkPagesForFault(changelist, nchanges * sizeof(struct kevent), vm);
                if (eventlist && nevents > 0)
                    smash::vm::walkPagesForFault(eventlist, nevents * sizeof(struct kevent), vm);
            }
        });
    return ret;
}

// ── kevent64 ─────────────────────────────────────────────────────────────────
// Firefox's IPC I/O thread uses kevent64 (not kevent) on macOS via
// message_pump_kqueue.cc. The events_ buffer is a std::vector<kevent64_s>
// allocated through our malloc, so it lives in our VmRegion and the
// compressor can mprotect its pages. Without this interposer the kernel
// returns EFAULT on the page-fault path and Firefox MOZ_CRASHes.

extern "C" int smash_kevent64(int kq,
                              const struct kevent64_s* changelist, int nchanges,
                              struct kevent64_s* eventlist, int nevents,
                              unsigned int flags,
                              const struct timespec* timeout);
SMASH_INTERPOSE(smash_kevent64, kevent64);

using kevent64_fn = int(*)(int, const struct kevent64_s*, int,
                           struct kevent64_s*, int, unsigned int,
                           const struct timespec*);

extern "C" int smash_kevent64(int kq,
                              const struct kevent64_s* changelist, int nchanges,
                              struct kevent64_s* eventlist, int nevents,
                              unsigned int flags,
                              const struct timespec* timeout) {
    auto* vm = smash::g_smash_vm_region;
    int ret = smash::vm::retryWithDecompress(
        [&] {
            return reinterpret_cast<kevent64_fn>(smash_interpose_smash_kevent64.original)(
                kq, changelist, nchanges, eventlist, nevents, flags, timeout);
        },
        [&] {
            if (vm) {
                if (changelist && nchanges > 0)
                    smash::vm::walkPagesForFault(changelist, nchanges * sizeof(struct kevent64_s), vm);
                if (eventlist && nevents > 0)
                    smash::vm::walkPagesForFault(eventlist, nevents * sizeof(struct kevent64_s), vm);
            }
        });
    return ret;
}

// ── mach_msg ─────────────────────────────────────────────────────────────────
// CoreFoundation's runloop (CFRunLoopServiceMachPort) calls mach_msg with a
// caller-provided buffer. For receive, the kernel writes the message into the
// buffer; for send, it reads. If the buffer's pages are smash-managed and the
// compressor has stripped write access (Phase 3 PROT_READ for monitoring) or
// read access (Phase 2 PROT_NONE for compression), the kernel returns
// MACH_RCV_INVALID_DATA / MACH_SEND_INVALID_DATA and CF __builtin_traps via
// __CFRunLoopServiceMachPort.cold.1. Pin and warm the buffer pages so the
// compressor leaves them alone for the duration of the call.

using mach_msg_fn = mach_msg_return_t(*)(mach_msg_header_t*, mach_msg_option_t,
                                         mach_msg_size_t, mach_msg_size_t,
                                         mach_port_name_t, mach_msg_timeout_t,
                                         mach_port_name_t);

extern "C" mach_msg_return_t smash_mach_msg(mach_msg_header_t* msg,
                                            mach_msg_option_t option,
                                            mach_msg_size_t send_size,
                                            mach_msg_size_t rcv_size,
                                            mach_port_name_t rcv_name,
                                            mach_msg_timeout_t timeout,
                                            mach_port_name_t notify);
SMASH_INTERPOSE(smash_mach_msg, mach_msg);

// Up to this many OOL descriptors per message will get their target buffers
// pinned. CFRunLoop messages typically carry 0–1 OOL descriptors; raising
// this only matters for unusual senders. The cap keeps the unpin tracker
// on-stack and bounds worst-case latency.
static constexpr size_t kSmashMaxOolPins = 16;

struct OolPinTracker {
    struct Entry { void* addr; size_t size; };
    Entry entries[kSmashMaxOolPins];
    size_t count = 0;
    void add(void* addr, size_t size) {
        if (count < kSmashMaxOolPins) {
            entries[count].addr = addr;
            entries[count].size = size;
            ++count;
        }
    }
};

// Walk the descriptor body of a complex mach message, pin and warm each
// OOL descriptor's target buffer, and record it in `tracker` so we can
// unpin on return.
//
// Layout: after mach_msg_header_t comes mach_msg_body_t (a single u32
// descriptor count) when MACH_MSGH_BITS_COMPLEX is set in msgh_bits.
// Then `count` descriptors of varying sizes follow. Every descriptor
// shares the same prefix `mach_msg_type_descriptor_t` (12 bytes on
// LP64), with the type byte at offset 11. We use that to discriminate
// before reading the full descriptor struct.
static void smash_pin_ool_descriptors(mach_msg_header_t* msg,
                                      smash::VmRegion* vm,
                                      OolPinTracker& tracker) {
    if (!msg || !vm) return;
    if (!(msg->msgh_bits & MACH_MSGH_BITS_COMPLEX)) return;

    auto* body = reinterpret_cast<mach_msg_body_t*>(msg + 1);
    uint32_t desc_count = body->msgh_descriptor_count;
    auto* p = reinterpret_cast<uint8_t*>(body + 1);
    // Don't walk past the message body — guard against a corrupt count.
    auto* msg_end = reinterpret_cast<uint8_t*>(msg) + msg->msgh_size;

    for (uint32_t i = 0; i < desc_count; ++i) {
        if (p + 12 > msg_end) return;
        uint8_t type = p[11];  // type byte: same offset across all variants
        size_t advance = 0;
        switch (type) {
        case MACH_MSG_PORT_DESCRIPTOR:
            advance = sizeof(mach_msg_port_descriptor_t);
            break;
        case MACH_MSG_OOL_DESCRIPTOR:
        case MACH_MSG_OOL_VOLATILE_DESCRIPTOR: {
            auto* d = reinterpret_cast<mach_msg_ool_descriptor_t*>(p);
            if (d->address && d->size > 0) {
                smash::vm::warmPages(d->address, d->size, vm);
                tracker.add(d->address, d->size);
            }
            advance = sizeof(mach_msg_ool_descriptor_t);
            break;
        }
        case MACH_MSG_OOL_PORTS_DESCRIPTOR: {
            auto* d = reinterpret_cast<mach_msg_ool_ports_descriptor_t*>(p);
            if (d->address && d->count > 0) {
                size_t bytes = static_cast<size_t>(d->count) *
                               sizeof(mach_port_t);
                smash::vm::warmPages(d->address, bytes, vm);
                tracker.add(d->address, bytes);
            }
            advance = sizeof(mach_msg_ool_ports_descriptor_t);
            break;
        }
        case MACH_MSG_GUARDED_PORT_DESCRIPTOR:
            advance = sizeof(mach_msg_guarded_port_descriptor_t);
            break;
        default:
            // Unknown descriptor type — bail rather than guess. The
            // already-pinned descriptors will be unpinned on return.
            return;
        }
        if (p + advance > msg_end) return;
        p += advance;
    }
}

extern "C" mach_msg_return_t smash_mach_msg(mach_msg_header_t* msg,
                                            mach_msg_option_t option,
                                            mach_msg_size_t send_size,
                                            mach_msg_size_t rcv_size,
                                            mach_port_name_t rcv_name,
                                            mach_msg_timeout_t timeout,
                                            mach_port_name_t notify) {
    auto* vm = smash::g_smash_vm_region;
    // The buffer holds either send_size (send) or rcv_size (receive); cover
    // both by using the max so a combined send/receive is also safe.
    size_t buf_size = send_size > rcv_size ? send_size : rcv_size;
    // Walk descriptor body for OOL targets — the kernel reads/writes
    // those during message dispatch (mach_msg2_trap), and a compressed
    // page hit there is unrecoverable (kernel checks protection
    // synchronously). Only relevant for SEND; on RECEIVE the kernel
    // writes new descriptor entries and any OOL data is kernel-mapped
    // into us, not user-side.
    OolPinTracker ool_tracker;
    if (vm && msg && (option & MACH_SEND_MSG)) {
        smash_pin_ool_descriptors(msg, vm, ool_tracker);
    }
    // Retry on the buffer-validity errors specifically; other returns
    // (timeout, port died, etc.) are not buffer-related.
    mach_msg_return_t ret = smash::vm::retryMachOnInvalidData(
        [&] {
            return reinterpret_cast<mach_msg_fn>(smash_interpose_smash_mach_msg.original)(
                msg, option, send_size, rcv_size, rcv_name, timeout, notify);
        },
        [&] {
            if (vm && msg && buf_size > 0)
                smash::vm::walkPagesForFault(msg, buf_size, vm);
            for (size_t i = 0; i < ool_tracker.count; ++i)
                smash::vm::walkPagesForFault(ool_tracker.entries[i].addr,
                                              ool_tracker.entries[i].size, vm);
        });
    return ret;
}

// ── mach_msg_overwrite ──────────────────────────────────────────────────────
// Same hazard as mach_msg, but called by libxpc / libdispatch on a separate
// path. __DATA_INTERPOSE only catches cross-dylib calls, so libsystem-internal
// chains (mach_msg → mach_msg_overwrite → mach_msg2_internal → trap) bypass
// our smash_mach_msg interposer entirely. Adding this interposer catches
// libxpc/libdispatch direct calls to mach_msg_overwrite. Same buffer-pinning
// strategy: warm + pin send buffer, walk OOL descriptors on send, warm the
// separate receive buffer if one was supplied.

using mach_msg_overwrite_fn = mach_msg_return_t(*)(
    mach_msg_header_t*, mach_msg_option_t,
    mach_msg_size_t, mach_msg_size_t,
    mach_port_name_t, mach_msg_timeout_t,
    mach_port_name_t, mach_msg_header_t*, mach_msg_size_t);

extern "C" mach_msg_return_t smash_mach_msg_overwrite(
    mach_msg_header_t* msg, mach_msg_option_t option,
    mach_msg_size_t send_size, mach_msg_size_t rcv_size,
    mach_port_name_t rcv_name, mach_msg_timeout_t timeout,
    mach_port_name_t notify, mach_msg_header_t* rcv_msg,
    mach_msg_size_t rcv_limit);
SMASH_INTERPOSE(smash_mach_msg_overwrite, mach_msg_overwrite);

extern "C" mach_msg_return_t smash_mach_msg_overwrite(
    mach_msg_header_t* msg, mach_msg_option_t option,
    mach_msg_size_t send_size, mach_msg_size_t rcv_size,
    mach_port_name_t rcv_name, mach_msg_timeout_t timeout,
    mach_port_name_t notify, mach_msg_header_t* rcv_msg,
    mach_msg_size_t rcv_limit) {
    auto* vm = smash::g_smash_vm_region;
    size_t send_buf_size = send_size;
    size_t rcv_buf_size = rcv_msg ? rcv_limit : rcv_size;
    OolPinTracker ool_tracker;
    if (vm && msg && (option & MACH_SEND_MSG)) {
        smash_pin_ool_descriptors(msg, vm, ool_tracker);
    }
    mach_msg_return_t ret = smash::vm::retryMachOnInvalidData(
        [&] {
            return reinterpret_cast<mach_msg_overwrite_fn>(
                smash_interpose_smash_mach_msg_overwrite.original)(
                msg, option, send_size, rcv_size, rcv_name, timeout, notify,
                rcv_msg, rcv_limit);
        },
        [&] {
            if (vm) {
                if (msg && send_buf_size > 0)
                    smash::vm::walkPagesForFault(msg, send_buf_size, vm);
                if (rcv_msg && rcv_buf_size > 0)
                    smash::vm::walkPagesForFault(rcv_msg, rcv_buf_size, vm);
                for (size_t i = 0; i < ool_tracker.count; ++i)
                    smash::vm::walkPagesForFault(ool_tracker.entries[i].addr,
                                                  ool_tracker.entries[i].size, vm);
            }
        });
    return ret;
}

// ── mach_msg2_internal ───────────────────────────────────────────────────────
// The internal entry that mach_msg / mach_msg_overwrite / mach_msg2 all
// route through before invoking the kernel trap. Some libsystem-resident
// callers (libxpc, libdispatch) reach this via paths we can't see — but
// any *cross-dylib* call to mach_msg2_internal goes through interposition,
// so adding this catches the direct callers.
//
// Signature is from Apple's libsyscall/mach/mach_msg.c:
//   mach_msg_return_t mach_msg2_internal(void *data, uint64_t options,
//       uint64_t msgh_bits_and_send_size,
//       uint64_t msgh_remote_and_local_port,
//       uint64_t msgh_voucher_and_id,
//       uint64_t desc_count_and_rcv_size,
//       uint64_t rcv_name_and_timeout,
//       uint32_t priority);
//
// Send size is the low 32 bits of msgh_bits_and_send_size; receive size is
// the low 32 bits of desc_count_and_rcv_size. We pin the buffer over the
// max of the two so both directions are covered.

// mach_msg2_internal is exported from libsystem_kernel but not declared in
// any public header. Declare here so we can take its address for the
// interpose entry.
extern "C" mach_msg_return_t mach_msg2_internal(
    void* data, uint64_t options, uint64_t bits_and_send_size,
    uint64_t remote_and_local_port, uint64_t voucher_and_id,
    uint64_t desc_count_and_rcv_size, uint64_t rcv_name_and_timeout,
    uint32_t priority);

using mach_msg2_internal_fn = mach_msg_return_t(*)(
    void*, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t,
    uint32_t);

extern "C" mach_msg_return_t smash_mach_msg2_internal(
    void* data, uint64_t options, uint64_t bits_and_send_size,
    uint64_t remote_and_local_port, uint64_t voucher_and_id,
    uint64_t desc_count_and_rcv_size, uint64_t rcv_name_and_timeout,
    uint32_t priority);
SMASH_INTERPOSE(smash_mach_msg2_internal, mach_msg2_internal);

extern "C" mach_msg_return_t smash_mach_msg2_internal(
    void* data, uint64_t options, uint64_t bits_and_send_size,
    uint64_t remote_and_local_port, uint64_t voucher_and_id,
    uint64_t desc_count_and_rcv_size, uint64_t rcv_name_and_timeout,
    uint32_t priority) {
    auto* vm = smash::g_smash_vm_region;
    uint32_t send_size = static_cast<uint32_t>(bits_and_send_size);
    uint32_t rcv_size = static_cast<uint32_t>(desc_count_and_rcv_size);
    size_t buf_size = send_size > rcv_size ? send_size : rcv_size;
    // OOL descriptor walking only makes sense if `data` looks like a
    // mach_msg_header_t with the COMPLEX bit set; cheap to check.
    OolPinTracker ool_tracker;
    if (vm && data && send_size >= sizeof(mach_msg_header_t)) {
        auto* hdr = static_cast<mach_msg_header_t*>(data);
        if (hdr->msgh_bits & MACH_MSGH_BITS_COMPLEX) {
            smash_pin_ool_descriptors(hdr, vm, ool_tracker);
        }
    }
    mach_msg_return_t ret = smash::vm::retryMachOnInvalidData(
        [&] {
            return reinterpret_cast<mach_msg2_internal_fn>(
                smash_interpose_smash_mach_msg2_internal.original)(
                data, options, bits_and_send_size, remote_and_local_port,
                voucher_and_id, desc_count_and_rcv_size, rcv_name_and_timeout,
                priority);
        },
        [&] {
            if (vm && data && buf_size > 0)
                smash::vm::walkPagesForFault(data, buf_size, vm);
            for (size_t i = 0; i < ool_tracker.count; ++i)
                smash::vm::walkPagesForFault(ool_tracker.entries[i].addr,
                                              ool_tracker.entries[i].size, vm);
        });
    return ret;
}

// ── recv / send ──────────────────────────────────────────────────────────────

using recv_fn = ssize_t(*)(int, void*, size_t, int);
using send_fn = ssize_t(*)(int, const void*, size_t, int);

extern "C" ssize_t smash_recv(int s, void* buf, size_t len, int flags);
SMASH_INTERPOSE(smash_recv, recv);
extern "C" ssize_t smash_recv(int s, void* buf, size_t len, int flags) {
    return smash::vm::retryWith1Buf(
        [&] { return reinterpret_cast<recv_fn>(smash_interpose_smash_recv.original)(s, buf, len, flags); },
        buf, len);
}

extern "C" ssize_t smash_send(int s, const void* buf, size_t len, int flags);
SMASH_INTERPOSE(smash_send, send);
extern "C" ssize_t smash_send(int s, const void* buf, size_t len, int flags) {
    return smash::vm::retryWith1Buf(
        [&] { return reinterpret_cast<send_fn>(smash_interpose_smash_send.original)(s, buf, len, flags); },
        buf, len);
}

// ── recvfrom / sendto ────────────────────────────────────────────────────────

using recvfrom_fn = ssize_t(*)(int, void*, size_t, int, struct sockaddr*, socklen_t*);
using sendto_fn = ssize_t(*)(int, const void*, size_t, int, const struct sockaddr*, socklen_t);

extern "C" ssize_t smash_recvfrom(int s, void* buf, size_t len, int flags,
                                   struct sockaddr* from, socklen_t* fromlen);
SMASH_INTERPOSE(smash_recvfrom, recvfrom);
extern "C" ssize_t smash_recvfrom(int s, void* buf, size_t len, int flags,
                                   struct sockaddr* from, socklen_t* fromlen) {
    auto* vm = smash::g_smash_vm_region;
    ssize_t ret = smash::vm::retryWithDecompress(
        [&] { return reinterpret_cast<recvfrom_fn>(smash_interpose_smash_recvfrom.original)(
            s, buf, len, flags, from, fromlen); },
        [&] { if (vm && buf && len) smash::vm::walkPagesForFault(buf, len, vm); });
    return ret;
}

extern "C" ssize_t smash_sendto(int s, const void* buf, size_t len, int flags,
                                 const struct sockaddr* to, socklen_t tolen);
SMASH_INTERPOSE(smash_sendto, sendto);
extern "C" ssize_t smash_sendto(int s, const void* buf, size_t len, int flags,
                                 const struct sockaddr* to, socklen_t tolen) {
    auto* vm = smash::g_smash_vm_region;
    ssize_t ret = smash::vm::retryWithDecompress(
        [&] { return reinterpret_cast<sendto_fn>(smash_interpose_smash_sendto.original)(
            s, buf, len, flags, to, tolen); },
        [&] { if (vm && buf && len) smash::vm::walkPagesForFault(buf, len, vm); });
    return ret;
}

// ── recvmsg / sendmsg ────────────────────────────────────────────────────────

using recvmsg_fn = ssize_t(*)(int, struct msghdr*, int);
using sendmsg_fn = ssize_t(*)(int, const struct msghdr*, int);

extern "C" ssize_t smash_recvmsg(int s, struct msghdr* msg, int flags);
SMASH_INTERPOSE(smash_recvmsg, recvmsg);
extern "C" ssize_t smash_recvmsg(int s, struct msghdr* msg, int flags) {
    auto* vm = smash::g_smash_vm_region;
    ssize_t ret = smash::vm::retryWithDecompress(
        [&] { return reinterpret_cast<recvmsg_fn>(smash_interpose_smash_recvmsg.original)(s, msg, flags); },
        [&] {
            if (vm && msg && msg->msg_iov && msg->msg_iovlen > 0)
                smash::vm::walkIovecForFault(msg->msg_iov, static_cast<int>(msg->msg_iovlen), vm);
        });
    return ret;
}

extern "C" ssize_t smash_sendmsg(int s, const struct msghdr* msg, int flags);
SMASH_INTERPOSE(smash_sendmsg, sendmsg);
extern "C" ssize_t smash_sendmsg(int s, const struct msghdr* msg, int flags) {
    auto* vm = smash::g_smash_vm_region;
    ssize_t ret = smash::vm::retryWithDecompress(
        [&] { return reinterpret_cast<sendmsg_fn>(smash_interpose_smash_sendmsg.original)(s, msg, flags); },
        [&] {
            if (vm && msg && msg->msg_iov && msg->msg_iovlen > 0)
                smash::vm::walkIovecForFault(msg->msg_iov, static_cast<int>(msg->msg_iovlen), vm);
        });
    return ret;
}

// ── poll ─────────────────────────────────────────────────────────────────────

using poll_fn = int(*)(struct pollfd*, nfds_t, int);

extern "C" int smash_poll(struct pollfd* fds, nfds_t nfds, int timeout);
SMASH_INTERPOSE(smash_poll, poll);
extern "C" int smash_poll(struct pollfd* fds, nfds_t nfds, int timeout) {
    auto* vm = smash::g_smash_vm_region;
    size_t fds_bytes = nfds * sizeof(struct pollfd);
    int ret = smash::vm::retryWithDecompress(
        [&] { return reinterpret_cast<poll_fn>(smash_interpose_smash_poll.original)(fds, nfds, timeout); },
        [&] { if (vm && fds && nfds > 0) smash::vm::walkPagesForFault(fds, fds_bytes, vm); });
    return ret;
}

// epoll_wait interposition is Linux-only (see linux_syscall_wrappers.cpp)

// ── fstat / fstatfs ─────────────────────────────────────────────────────────
// Both write a struct (stat / statfs) into a userspace buffer. The buffer
// is normally on the stack but heap-allocated struct stat / struct statfs
// is a real pattern (e.g., callers that pass a heap-allocated *st through
// many layers). With the EFAULT-retry pattern, adding wrappers is cheap.

using fstat_fn = int(*)(int, struct stat*);
using fstatfs_fn = int(*)(int, struct statfs*);

extern "C" int smash_fstat(int fd, struct stat* st);
SMASH_INTERPOSE(smash_fstat, fstat);
extern "C" int smash_fstat(int fd, struct stat* st) {
    auto* vm = smash::g_smash_vm_region;
    return smash::vm::retryWithDecompress(
        [&] { return reinterpret_cast<fstat_fn>(smash_interpose_smash_fstat.original)(fd, st); },
        [&] { if (vm && st) smash::vm::walkPagesForFault(st, sizeof(struct stat), vm); });
}

extern "C" int smash_fstatfs(int fd, struct statfs* st);
SMASH_INTERPOSE(smash_fstatfs, fstatfs);
extern "C" int smash_fstatfs(int fd, struct statfs* st) {
    auto* vm = smash::g_smash_vm_region;
    return smash::vm::retryWithDecompress(
        [&] { return reinterpret_cast<fstatfs_fn>(smash_interpose_smash_fstatfs.original)(fd, st); },
        [&] { if (vm && st) smash::vm::walkPagesForFault(st, sizeof(struct statfs), vm); });
}

// ── stat / lstat / fstatat ──────────────────────────────────────────────────
// Same hazard as fstat — kernel writes a struct stat into a userspace buffer.
// stat / lstat take a path and write to *st; fstatat takes a dirfd + path.

using stat_fn = int(*)(const char*, struct stat*);
using lstat_fn = int(*)(const char*, struct stat*);
using fstatat_fn = int(*)(int, const char*, struct stat*, int);

extern "C" int smash_stat(const char* path, struct stat* st);
SMASH_INTERPOSE(smash_stat, stat);
extern "C" int smash_stat(const char* path, struct stat* st) {
    return smash::vm::retryWith1Buf(
        [&] { return reinterpret_cast<stat_fn>(smash_interpose_smash_stat.original)(path, st); },
        st, sizeof(struct stat));
}

extern "C" int smash_lstat(const char* path, struct stat* st);
SMASH_INTERPOSE(smash_lstat, lstat);
extern "C" int smash_lstat(const char* path, struct stat* st) {
    return smash::vm::retryWith1Buf(
        [&] { return reinterpret_cast<lstat_fn>(smash_interpose_smash_lstat.original)(path, st); },
        st, sizeof(struct stat));
}

extern "C" int smash_fstatat(int dirfd, const char* path, struct stat* st, int flags);
SMASH_INTERPOSE(smash_fstatat, fstatat);
extern "C" int smash_fstatat(int dirfd, const char* path, struct stat* st, int flags) {
    return smash::vm::retryWith1Buf(
        [&] { return reinterpret_cast<fstatat_fn>(smash_interpose_smash_fstatat.original)(dirfd, path, st, flags); },
        st, sizeof(struct stat));
}

// ── readlink / readlinkat ───────────────────────────────────────────────────
// Kernel writes the symlink target string into the user buffer.

using readlink_fn = ssize_t(*)(const char*, char*, size_t);
using readlinkat_fn = ssize_t(*)(int, const char*, char*, size_t);

extern "C" ssize_t smash_readlink(const char* path, char* buf, size_t bufsize);
SMASH_INTERPOSE(smash_readlink, readlink);
extern "C" ssize_t smash_readlink(const char* path, char* buf, size_t bufsize) {
    return smash::vm::retryWith1Buf(
        [&] { return reinterpret_cast<readlink_fn>(smash_interpose_smash_readlink.original)(path, buf, bufsize); },
        buf, bufsize);
}

extern "C" ssize_t smash_readlinkat(int dirfd, const char* path, char* buf, size_t bufsize);
SMASH_INTERPOSE(smash_readlinkat, readlinkat);
extern "C" ssize_t smash_readlinkat(int dirfd, const char* path, char* buf, size_t bufsize) {
    return smash::vm::retryWith1Buf(
        [&] { return reinterpret_cast<readlinkat_fn>(smash_interpose_smash_readlinkat.original)(dirfd, path, buf, bufsize); },
        buf, bufsize);
}

// ── preadv / pwritev (positioned vectored I/O) ──────────────────────────────
using preadv_fn = ssize_t(*)(int, const struct iovec*, int, off_t);
using pwritev_fn = ssize_t(*)(int, const struct iovec*, int, off_t);

extern "C" ssize_t smash_preadv(int fd, const struct iovec* iov, int iovcnt, off_t offset);
SMASH_INTERPOSE(smash_preadv, preadv);
extern "C" ssize_t smash_preadv(int fd, const struct iovec* iov, int iovcnt, off_t offset) {
    return smash::vm::retryWithIovec(
        [&] { return reinterpret_cast<preadv_fn>(smash_interpose_smash_preadv.original)(fd, iov, iovcnt, offset); },
        iov, iovcnt);
}

extern "C" ssize_t smash_pwritev(int fd, const struct iovec* iov, int iovcnt, off_t offset);
SMASH_INTERPOSE(smash_pwritev, pwritev);
extern "C" ssize_t smash_pwritev(int fd, const struct iovec* iov, int iovcnt, off_t offset) {
    return smash::vm::retryWithIovec(
        [&] { return reinterpret_cast<pwritev_fn>(smash_interpose_smash_pwritev.original)(fd, iov, iovcnt, offset); },
        iov, iovcnt);
}

// ── getattrlist / getattrlistbulk (macOS Cocoa file APIs) ────────────────────
// NSFileManager / NSOpenPanel / Spotlight all funnel through these. Kernel
// writes a packed sequence of attribute values into the user-supplied
// `attrBuf` of size `attrBufSize` — that's the EFAULT risk.

using getattrlist_fn = int(*)(const char*, struct attrlist*, void*, size_t, unsigned long);
using getattrlistbulk_fn = int(*)(int, struct attrlist*, void*, size_t, uint64_t);

extern "C" int smash_getattrlist(const char* path, struct attrlist* alist,
                                  void* attrBuf, size_t attrBufSize,
                                  unsigned long options);
SMASH_INTERPOSE(smash_getattrlist, getattrlist);
extern "C" int smash_getattrlist(const char* path, struct attrlist* alist,
                                  void* attrBuf, size_t attrBufSize,
                                  unsigned long options) {
    return smash::vm::retryWith1Buf(
        [&] { return reinterpret_cast<getattrlist_fn>(smash_interpose_smash_getattrlist.original)(
            path, alist, attrBuf, attrBufSize, options); },
        attrBuf, attrBufSize);
}

extern "C" int smash_getattrlistbulk(int dirfd, struct attrlist* alist,
                                      void* attrBuf, size_t attrBufSize,
                                      uint64_t options);
SMASH_INTERPOSE(smash_getattrlistbulk, getattrlistbulk);
extern "C" int smash_getattrlistbulk(int dirfd, struct attrlist* alist,
                                      void* attrBuf, size_t attrBufSize,
                                      uint64_t options) {
    return smash::vm::retryWith1Buf(
        [&] { return reinterpret_cast<getattrlistbulk_fn>(smash_interpose_smash_getattrlistbulk.original)(
            dirfd, alist, attrBuf, attrBufSize, options); },
        attrBuf, attrBufSize);
}

// ── waitid / wait4 (process management) ─────────────────────────────────────
// waitid writes a 128-byte siginfo_t into *infop; wait4 writes int wstatus
// AND struct rusage. wstatus is usually stack (4 bytes); rusage may be heap
// for accumulating perf counters.

using waitid_fn = int(*)(idtype_t, id_t, siginfo_t*, int);
using wait4_fn = pid_t(*)(pid_t, int*, int, struct rusage*);

extern "C" int smash_waitid(idtype_t idtype, id_t id, siginfo_t* info, int options);
SMASH_INTERPOSE(smash_waitid, waitid);
extern "C" int smash_waitid(idtype_t idtype, id_t id, siginfo_t* info, int options) {
    return smash::vm::retryWith1Buf(
        [&] { return reinterpret_cast<waitid_fn>(smash_interpose_smash_waitid.original)(idtype, id, info, options); },
        info, sizeof(siginfo_t));
}

extern "C" pid_t smash_wait4(pid_t pid, int* wstatus, int options, struct rusage* rusage);
SMASH_INTERPOSE(smash_wait4, wait4);
extern "C" pid_t smash_wait4(pid_t pid, int* wstatus, int options, struct rusage* rusage) {
    auto* vm = smash::g_smash_vm_region;
    return smash::vm::retryWithDecompress(
        [&] { return reinterpret_cast<wait4_fn>(smash_interpose_smash_wait4.original)(pid, wstatus, options, rusage); },
        [&] {
            if (vm) {
                if (wstatus) smash::vm::walkPagesForFault(wstatus, sizeof(int), vm);
                if (rusage) smash::vm::walkPagesForFault(rusage, sizeof(struct rusage), vm);
            }
        });
}

// ── statvfs / fstatvfs / getrusage ──────────────────────────────────────────
// Cross-platform output-buffer wrappers. xattr is intentionally not wrapped
// on macOS — the Apple API takes extra (position, options) parameters and
// is rarely called with heap-allocated values; do it later if needed.

using statvfs_fn = int(*)(const char*, struct statvfs*);
using fstatvfs_fn = int(*)(int, struct statvfs*);
using getrusage_fn = int(*)(int, struct rusage*);

extern "C" int smash_statvfs(const char* path, struct statvfs* st);
SMASH_INTERPOSE(smash_statvfs, statvfs);
extern "C" int smash_statvfs(const char* path, struct statvfs* st) {
    return smash::vm::retryWith1Buf(
        [&] { return reinterpret_cast<statvfs_fn>(smash_interpose_smash_statvfs.original)(path, st); },
        st, sizeof(struct statvfs));
}

extern "C" int smash_fstatvfs(int fd, struct statvfs* st);
SMASH_INTERPOSE(smash_fstatvfs, fstatvfs);
extern "C" int smash_fstatvfs(int fd, struct statvfs* st) {
    return smash::vm::retryWith1Buf(
        [&] { return reinterpret_cast<fstatvfs_fn>(smash_interpose_smash_fstatvfs.original)(fd, st); },
        st, sizeof(struct statvfs));
}

extern "C" int smash_getrusage(int who, struct rusage* usage);
SMASH_INTERPOSE(smash_getrusage, getrusage);
extern "C" int smash_getrusage(int who, struct rusage* usage) {
    return smash::vm::retryWith1Buf(
        [&] { return reinterpret_cast<getrusage_fn>(smash_interpose_smash_getrusage.original)(who, usage); },
        usage, sizeof(struct rusage));
}

// ── read / write ────────────────────────────────────────────────────────────

using read_fn = ssize_t(*)(int, void*, size_t);
using write_fn = ssize_t(*)(int, const void*, size_t);

extern "C" ssize_t smash_read(int fd, void* buf, size_t count);
SMASH_INTERPOSE(smash_read, read);
extern "C" ssize_t smash_read(int fd, void* buf, size_t count) {
    return smash::vm::retryWith1Buf(
        [&] { return reinterpret_cast<read_fn>(smash_interpose_smash_read.original)(fd, buf, count); },
        buf, count);
}

extern "C" ssize_t smash_write(int fd, const void* buf, size_t count);
SMASH_INTERPOSE(smash_write, write);
extern "C" ssize_t smash_write(int fd, const void* buf, size_t count) {
    return smash::vm::retryWith1Buf(
        [&] { return reinterpret_cast<write_fn>(smash_interpose_smash_write.original)(fd, buf, count); },
        buf, count);
}

// ── pread / pwrite ──────────────────────────────────────────────────────────

using pread_fn = ssize_t(*)(int, void*, size_t, off_t);
using pwrite_fn = ssize_t(*)(int, const void*, size_t, off_t);

extern "C" ssize_t smash_pread(int fd, void* buf, size_t count, off_t offset);
SMASH_INTERPOSE(smash_pread, pread);
extern "C" ssize_t smash_pread(int fd, void* buf, size_t count, off_t offset) {
    return smash::vm::retryWith1Buf(
        [&] { return reinterpret_cast<pread_fn>(smash_interpose_smash_pread.original)(fd, buf, count, offset); },
        buf, count);
}

extern "C" ssize_t smash_pwrite(int fd, const void* buf, size_t count, off_t offset);
SMASH_INTERPOSE(smash_pwrite, pwrite);
extern "C" ssize_t smash_pwrite(int fd, const void* buf, size_t count, off_t offset) {
    return smash::vm::retryWith1Buf(
        [&] { return reinterpret_cast<pwrite_fn>(smash_interpose_smash_pwrite.original)(fd, buf, count, offset); },
        buf, count);
}

// ── readv / writev ──────────────────────────────────────────────────────────

using readv_fn = ssize_t(*)(int, const struct iovec*, int);
using writev_fn = ssize_t(*)(int, const struct iovec*, int);

extern "C" ssize_t smash_readv(int fd, const struct iovec* iov, int iovcnt);
SMASH_INTERPOSE(smash_readv, readv);
extern "C" ssize_t smash_readv(int fd, const struct iovec* iov, int iovcnt) {
    return smash::vm::retryWithIovec(
        [&] { return reinterpret_cast<readv_fn>(smash_interpose_smash_readv.original)(fd, iov, iovcnt); },
        iov, iovcnt);
}

extern "C" ssize_t smash_writev(int fd, const struct iovec* iov, int iovcnt);
SMASH_INTERPOSE(smash_writev, writev);
extern "C" ssize_t smash_writev(int fd, const struct iovec* iov, int iovcnt) {
    return smash::vm::retryWithIovec(
        [&] { return reinterpret_cast<writev_fn>(smash_interpose_smash_writev.original)(fd, iov, iovcnt); },
        iov, iovcnt);
}

// ── fread / fgets (buffered I/O) ────────────────────────────────────────────
//
// DYLD interposition can't intercept intra-dylib calls. libc's fread/fgets
// call read() internally within libSystem, bypassing our smash_read.
// We interpose the higher-level functions to warm/pin the FILE's internal
// buffer before they call read() under the hood.

// Helper: warm and pin a FILE's internal read buffer
static inline void warmFileBuffer(FILE* stream, smash::VmRegion* vm) {
    // macOS FILE struct: _bf._base is the buffer, _bf._size is its length
    if (stream && stream->_bf._base && stream->_bf._size > 0) {
        smash::vm::warmPages(stream->_bf._base, stream->_bf._size, vm);
    }
}

using fread_fn = size_t(*)(void*, size_t, size_t, FILE*);
using fgets_fn = char*(*)(char*, int, FILE*);

extern "C" size_t smash_fread(void* ptr, size_t size, size_t nitems, FILE* stream);
SMASH_INTERPOSE(smash_fread, fread);
extern "C" size_t smash_fread(void* ptr, size_t size, size_t nitems, FILE* stream) {
    auto* vm = smash::g_smash_vm_region;
    size_t total = size * nitems;
    if (vm) {
        if (ptr && total) smash::vm::warmPages(ptr, total, vm);
        warmFileBuffer(stream, vm);
    }
    size_t ret = reinterpret_cast<fread_fn>(smash_interpose_smash_fread.original)(ptr, size, nitems, stream);
    if (vm) {
    }
    return ret;
}

extern "C" char* smash_fgets(char* str, int size, FILE* stream);
SMASH_INTERPOSE(smash_fgets, fgets);
extern "C" char* smash_fgets(char* str, int size, FILE* stream) {
    auto* vm = smash::g_smash_vm_region;
    if (vm) {
        if (str && size > 0) smash::vm::warmPages(str, size, vm);
        warmFileBuffer(stream, vm);
    }
    char* ret = reinterpret_cast<fgets_fn>(smash_interpose_smash_fgets.original)(str, size, stream);
    if (vm) {
    }
    return ret;
}

// ── fgetc / getc (character-at-a-time input) ────────────────────────────────
//
// DuckDB and C++ iostream (std::cin) use fgetc/getc for character reads.
// These internally call read() within libSystem when the FILE buffer is empty.
// We warm/pin the FILE buffer before each call to prevent kernel EFAULT.

using fgetc_fn = int(*)(FILE*);
using getc_fn = int(*)(FILE*);

extern "C" int smash_fgetc(FILE* stream);
SMASH_INTERPOSE(smash_fgetc, fgetc);
extern "C" int smash_fgetc(FILE* stream) {
    auto* vm = smash::g_smash_vm_region;
    if (vm) warmFileBuffer(stream, vm);
    int ret = reinterpret_cast<fgetc_fn>(smash_interpose_smash_fgetc.original)(stream);
    return ret;
}

extern "C" int smash_getc(FILE* stream);
SMASH_INTERPOSE(smash_getc, getc);
extern "C" int smash_getc(FILE* stream) {
    auto* vm = smash::g_smash_vm_region;
    if (vm) warmFileBuffer(stream, vm);
    int ret = reinterpret_cast<getc_fn>(smash_interpose_smash_getc.original)(stream);
    return ret;
}

// ── fwrite / fflush (buffered output) ───────────────────────────────────────
//
// libc's fwrite/fflush internally call write() when flushing the buffer.
// If the FILE output buffer page is PROT_NONE (compressed), the kernel's
// copyin from the buffer would fail. Pin the FILE buffer during these calls.

using fwrite_fn = size_t(*)(const void*, size_t, size_t, FILE*);
using fflush_fn = int(*)(FILE*);

extern "C" size_t smash_fwrite(const void* ptr, size_t size, size_t nitems, FILE* stream);
SMASH_INTERPOSE(smash_fwrite, fwrite);
extern "C" size_t smash_fwrite(const void* ptr, size_t size, size_t nitems, FILE* stream) {
    auto* vm = smash::g_smash_vm_region;
    size_t total = size * nitems;
    if (vm) {
        if (ptr && total) smash::vm::warmPages(ptr, total, vm);
        warmFileBuffer(stream, vm);
    }
    size_t ret = reinterpret_cast<fwrite_fn>(smash_interpose_smash_fwrite.original)(ptr, size, nitems, stream);
    return ret;
}

extern "C" int smash_fflush(FILE* stream);
SMASH_INTERPOSE(smash_fflush, fflush);
extern "C" int smash_fflush(FILE* stream) {
    auto* vm = smash::g_smash_vm_region;
    if (vm) warmFileBuffer(stream, vm);
    int ret = reinterpret_cast<fflush_fn>(smash_interpose_smash_fflush.original)(stream);
    return ret;
}

// ── External-mapping interposers ────────────────────────────────────────────
//
// Standard smash's main allocator hands out memory from a single big
// MAP_ANON reservation that smash itself owns; the compressor compresses
// pages within that reservation. Application code that calls mmap() /
// vm_allocate() *directly* (SpiderMonkey JS GC arenas, Skia surfaces via
// mozalloc_aligned, etc.) bypasses the malloc path entirely and so escapes
// the compressor.
//
// The interposers below register such mappings via VmRegion's external-
// page hash so the compressor's tick can see them. Strict filters apply:
//
//   • mmap     — only MAP_ANON | PROT_WRITE. File-backed mappings would
//                break msync semantics under compression and the OS
//                already evicts them; non-writable mappings won't dirty.
//   • mach_vm_allocate — anonymous by design; tracked when caller-
//                requested protection includes VM_PROT_WRITE.
//
// munmap / vm_deallocate counterparts mark the pages EMPTY so the
// compressor stops scanning them. Compressed-page leakage on unmap is
// possible (we don't free associated compressed buffers) but bounded
// by the workload's churn rate.
//
// External tracking is OFF by default. Set SMASH_TRACK_EXTERNAL=1 to
// enable. Firefox 5-tab Wikipedia at 90 s crashes ~25 s earlier with
// tracking on (35 s vs 60 s pre-port baseline) — the cause hasn't been
// root-caused yet. Targets that don't have Firefox's allocation patterns
// (e.g., a one-shot redb workload) are safe to opt in. The interposers
// themselves still install (cost: one branch per mmap / mach_vm call);
// only the page registration path is gated.

namespace {

inline bool externalTrackingEnabled() {
    static const bool enabled = []{
        const char* v = std::getenv("SMASH_TRACK_EXTERNAL");
        return v && v[0] == '1';
    }();
    return enabled;
}

// Register every page in [base, base+len) with the VmRegion's external
// tracker and set its initial PageState to ACTIVE.
inline void registerExternalRange(smash::VmRegion* vm, void* base, size_t len) {
    if (!vm || !base || !len) return;
    if (!externalTrackingEnabled()) return;
    auto start = reinterpret_cast<uintptr_t>(base) & ~(uintptr_t{smash::kPageSize} - 1);
    auto end = (reinterpret_cast<uintptr_t>(base) + len + smash::kPageSize - 1)
               & ~(uintptr_t{smash::kPageSize} - 1);
    for (uintptr_t p = start; p < end; p += smash::kPageSize) {
        size_t idx = vm->trackExternalPage(p);
        if (idx == 0) continue;  // table full or page in contiguous arena
        smash::g_smash_page_states_for_external
            ? smash::g_smash_page_states_for_external->set(idx, smash::PageState::ACTIVE)
            : (void)0;
    }
}

inline void deregisterExternalRange(smash::VmRegion* vm, void* base, size_t len) {
    if (!vm || !base || !len) return;
    if (!externalTrackingEnabled()) return;
    auto start = reinterpret_cast<uintptr_t>(base) & ~(uintptr_t{smash::kPageSize} - 1);
    auto end = (reinterpret_cast<uintptr_t>(base) + len + smash::kPageSize - 1)
               & ~(uintptr_t{smash::kPageSize} - 1);
    for (uintptr_t p = start; p < end; p += smash::kPageSize) {
        // Untrack UNDER the per-page lock: serializes against a compressor
        // worker that may be mid-snapshot on this external page, so the real
        // munmap (in the caller, AFTER this returns) can't pull the mapping out
        // from under the worker's memcpy. Proven necessary + deadlock-free by
        // model/SmashExternalRace.{lean,tla}. No mprotect/munmap is done while
        // the lock is held here.
        vm->untrackExternalPageLocked(p,
            smash::g_smash_page_locks_for_external,
            smash::g_smash_page_states_for_external);
    }
}

}  // namespace

// ── mmap / munmap ───────────────────────────────────────────────────────────

using mmap_fn = void*(*)(void*, size_t, int, int, int, off_t);
using munmap_fn = int(*)(void*, size_t);

extern "C" void* smash_mmap(void* addr, size_t len, int prot, int flags, int fd, off_t offset);
SMASH_INTERPOSE(smash_mmap, mmap);
extern "C" void* smash_mmap(void* addr, size_t len, int prot, int flags, int fd, off_t offset) {
    void* ret = reinterpret_cast<mmap_fn>(smash_interpose_smash_mmap.original)(
        addr, len, prot, flags, fd, offset);
    if (ret == MAP_FAILED) return ret;
    // Filter to anonymous, writable, non-zero-length mappings. File-backed
    // (fd != -1 or !MAP_ANON) and read-only mappings are ineligible.
    bool anon = (flags & MAP_ANON) != 0;
    bool writable = (prot & PROT_WRITE) != 0;
    if (!anon || !writable || len == 0) return ret;
    auto* vm = smash::g_smash_vm_region;
    if (vm) registerExternalRange(vm, ret, len);
    return ret;
}

extern "C" int smash_munmap(void* addr, size_t len);
SMASH_INTERPOSE(smash_munmap, munmap);
extern "C" int smash_munmap(void* addr, size_t len) {
    auto* vm = smash::g_smash_vm_region;
    if (vm && addr && len) deregisterExternalRange(vm, addr, len);
    return reinterpret_cast<munmap_fn>(smash_interpose_smash_munmap.original)(addr, len);
}

// ── mach_vm_allocate / mach_vm_deallocate ────────────────────────────────────
//
// SpiderMonkey on macOS uses mach_vm_allocate (not POSIX mmap) for its GC
// arenas. CoreGraphics/IOSurface paths also reach the kernel via Mach VM.
// mach_vm_allocate is anonymous-by-design (file-backed mappings go through
// mach_vm_map with a memory object); tracking is unconditional within Mach
// VM, gated only on the requested protection including write.
//
// Note: a few mach_vm_allocate callers immediately mach_vm_protect down to
// PROT_NONE. Tracking those is harmless — the compressor will see them as
// EMPTY-equivalent (no write-fault → no dirty bit → no compress).

using mach_vm_allocate_fn = kern_return_t(*)(vm_map_t, mach_vm_address_t*,
                                              mach_vm_size_t, int);
using mach_vm_deallocate_fn = kern_return_t(*)(vm_map_t, mach_vm_address_t,
                                                mach_vm_size_t);
using vm_allocate_fn = kern_return_t(*)(vm_map_t, vm_address_t*, vm_size_t, int);
using vm_deallocate_fn = kern_return_t(*)(vm_map_t, vm_address_t, vm_size_t);

extern "C" kern_return_t smash_mach_vm_allocate(vm_map_t target,
                                                 mach_vm_address_t* address,
                                                 mach_vm_size_t size, int flags);
SMASH_INTERPOSE(smash_mach_vm_allocate, mach_vm_allocate);
extern "C" kern_return_t smash_mach_vm_allocate(vm_map_t target,
                                                 mach_vm_address_t* address,
                                                 mach_vm_size_t size, int flags) {
    kern_return_t kr = reinterpret_cast<mach_vm_allocate_fn>(
        smash_interpose_smash_mach_vm_allocate.original)(target, address, size, flags);
    if (kr != KERN_SUCCESS || !address || size == 0) return kr;
    // Only track allocations in our own task — cross-task allocations
    // belong to a child process and we can't compress remote pages.
    if (target != mach_task_self()) return kr;
    auto* vm = smash::g_smash_vm_region;
    if (vm) registerExternalRange(vm, reinterpret_cast<void*>(*address),
                                   static_cast<size_t>(size));
    return kr;
}

extern "C" kern_return_t smash_mach_vm_deallocate(vm_map_t target,
                                                   mach_vm_address_t address,
                                                   mach_vm_size_t size);
SMASH_INTERPOSE(smash_mach_vm_deallocate, mach_vm_deallocate);
extern "C" kern_return_t smash_mach_vm_deallocate(vm_map_t target,
                                                   mach_vm_address_t address,
                                                   mach_vm_size_t size) {
    if (target == mach_task_self() && address && size) {
        auto* vm = smash::g_smash_vm_region;
        if (vm)
            deregisterExternalRange(vm, reinterpret_cast<void*>(address),
                                     static_cast<size_t>(size));
    }
    return reinterpret_cast<mach_vm_deallocate_fn>(
        smash_interpose_smash_mach_vm_deallocate.original)(target, address, size);
}

extern "C" kern_return_t smash_vm_allocate(vm_map_t target, vm_address_t* address,
                                            vm_size_t size, int flags);
SMASH_INTERPOSE(smash_vm_allocate, vm_allocate);
extern "C" kern_return_t smash_vm_allocate(vm_map_t target, vm_address_t* address,
                                            vm_size_t size, int flags) {
    kern_return_t kr = reinterpret_cast<vm_allocate_fn>(
        smash_interpose_smash_vm_allocate.original)(target, address, size, flags);
    if (kr != KERN_SUCCESS || !address || size == 0) return kr;
    if (target != mach_task_self()) return kr;
    auto* vm = smash::g_smash_vm_region;
    if (vm) registerExternalRange(vm, reinterpret_cast<void*>(*address),
                                   static_cast<size_t>(size));
    return kr;
}

extern "C" kern_return_t smash_vm_deallocate(vm_map_t target, vm_address_t address,
                                              vm_size_t size);
SMASH_INTERPOSE(smash_vm_deallocate, vm_deallocate);
extern "C" kern_return_t smash_vm_deallocate(vm_map_t target, vm_address_t address,
                                              vm_size_t size) {
    if (target == mach_task_self() && address && size) {
        auto* vm = smash::g_smash_vm_region;
        if (vm)
            deregisterExternalRange(vm, reinterpret_cast<void*>(address),
                                     static_cast<size_t>(size));
    }
    return reinterpret_cast<vm_deallocate_fn>(
        smash_interpose_smash_vm_deallocate.original)(target, address, size);
}

#endif // __APPLE__

// Linux syscall interposition is in src/linux_syscall_wrappers.cpp

// ── alloc8 integration ───────────────────────────────────────────────────────

// Required by alloc8's thread interposition for thread-created flag
extern "C" {
volatile int xxthread_created_flag = 0;
}

using SmashRedirect = alloc8::HeapRedirect<smash::SmashHeap>;

// Lean entry points for xxmalloc / xxfree. The common case -- a small
// object served from, or returned to, this thread's cache -- is handled here
// with a single per-thread gate (g_fast_tc, published only when that is valid)
// and no call; anything else tail-calls the full SmashHeap path, which
// handles every mode, first call, refill, drain and large block exactly as
// before. Keeping the fallbacks out of line lets the fast path run without
// saving callee-saved registers.
struct SmashFastRedirect : SmashRedirect {
    [[gnu::noinline]] static void* mallocFull(size_t sz) { return SmashRedirect::malloc(sz); }
    [[gnu::noinline]] static void freeFull(void* p) { SmashRedirect::free(p); }

    [[gnu::always_inline]] static void* malloc(size_t sz) {
        using namespace smash;
        ThreadCache* tc = g_fast_tc;
        if (tc && sz - 1 < kMaxSmallSize) [[likely]] {
#if defined(SMASH_USE_TSD)
            const auto ra = reinterpret_cast<uintptr_t>(threadState()->caller_ra);
#else
            const auto ra = reinterpret_cast<uintptr_t>(alloc8_caller_ra);
#endif
            const uint8_t sc = sizeToClass(sz);
            const uintptr_t key = ra ^ (static_cast<uintptr_t>(sc) << 56);
            const size_t slot = (key * 0x9E3779B97F4A7C15ULL >> 56) & (kArenaMemoSlots - 1);
            const ArenaMemo& m = g_arena_memo[slot];
            if (ra && m.valid && m.key == key) [[likely]] {
                if (void* p = tc->allocate(sc, m.base)) [[likely]] return p;
            }
        }
        return mallocFull(sz);
    }

    [[gnu::noinline]] static void* callocFull(size_t n, size_t sz) { return SmashRedirect::calloc(n, sz); }
    [[gnu::noinline]] static void* reallocFull(void* p, size_t sz) { return SmashRedirect::realloc(p, sz); }

    // Small calloc: the lean malloc plus a memset of the object.
    static void* calloc(size_t n, size_t sz) {
        using namespace smash;
        size_t total;
        if (!__builtin_mul_overflow(n, sz, &total) && g_fast_tc &&
            total - 1 < kMaxSmallSize) [[likely]] {
            void* p = malloc(total);
            if (p) __builtin_memset(p, 0, total);
            return p;
        }
        return callocFull(n, sz);
    }

    // Small-to-small realloc: the page tag gives the old block's class, so
    // stay in place when it still fits and is at least half used, else move
    // through the lean malloc/free. Anything else takes the full path.
    static void* realloc(void* p, size_t sz) {
        using namespace smash;
        const uintptr_t off = reinterpret_cast<uintptr_t>(p) - g_fast_span_table.base;
        if (g_fast_tc && off < g_fast_span_table.size && sz - 1 < kMaxSmallSize) [[likely]] {
            const uint16_t tag =
                g_fast_span_table.tags[off >> kPageShift].load(std::memory_order_acquire);
            if (tag & VmRegion::kPageTagSmall) [[likely]] {
                const size_t cap = classSize(static_cast<uint8_t>(tag & 63));
                if (sz <= cap && sz >= cap / 2) return p;
                void* q = malloc(sz);
                if (q) {
                    __builtin_memcpy(q, p, cap < sz ? cap : sz);
                    free(p);
                }
                return q;
            }
        }
        return reallocFull(p, sz);
    }

    [[gnu::always_inline]] static void free(void* p) {
        using namespace smash;
        ThreadCache* tc = g_fast_tc;
        const uintptr_t off = reinterpret_cast<uintptr_t>(p) - g_fast_span_table.base;
        if (tc && off < g_fast_span_table.size) [[likely]] {
            // One 2-byte load names the lane; the Span is not touched.
            const uint16_t tag =
                g_fast_span_table.tags[off >> kPageShift].load(std::memory_order_acquire);
            if (tag & VmRegion::kPageTagSmall) [[likely]] {
                const uint8_t sc = tag & 63;
                const uint8_t arena = static_cast<uint8_t>(tag >> 6);
                if (tc->deallocate(sc, arena, p)) [[likely]] return;
            }
        }
        freeFull(p);
    }
};

// When SMASH_NO_THREAD_HOOKS_BUILD is defined (via -D), use ALLOC8_REDIRECT
// which doesn't register xxthread_init/xxthread_cleanup. This avoids alloc8's
// pthread_create interposition which can cause crashes during process shutdown.
#ifdef SMASH_NO_THREAD_HOOKS_BUILD
ALLOC8_REDIRECT(SmashFastRedirect);
#else
ALLOC8_REDIRECT(SmashFastRedirect)
ALLOC8_THREAD_REDIRECT(alloc8::ThreadRedirect<smash::SmashHeap>);
#endif

extern "C" bool xxowns_active() { return true; }

extern "C" bool xxowns(const void* ptr) {
    if (!ptr) return false;
    if (smash::BootstrapAlloc::instance().owns(ptr)) return true;
    auto* vm = smash::g_smash_vm_region;
    if (!vm) return false;
    auto addr = reinterpret_cast<uintptr_t>(ptr);
    // Ownership means "smash's malloc allocated this pointer, so smash must
    // free/realloc/size it" — i.e. the contiguous arena (full mode) or the
    // tracking hash when it IS the allocator (compress-only mode).
    //
    // Under SMASH_TRACK_EXTERNAL=1, VmRegion::contains() ALSO returns true for
    // application-direct mmap pages that smash merely *tracks* for the
    // compressor to scan — smash did NOT allocate them. Claiming ownership of
    // those routes the app's own free()/realloc() (e.g. libcrypto freeing a
    // pointer into an mmap'd OpenSSL config arena) into smash's slab path,
    // which has no valid span for the pointer → state corruption → SIGSEGV
    // (or a decommitPages OOB trap on the tracked page). Tracking is NOT
    // ownership: exclude externally-tracked pages from xxowns.
    if (vm->isTrackingMode())
        return vm->contains(addr);   // compress-only: hash IS the allocator
    if (vm->inContigArena(addr)) return true;  // full mode: smash's own arena
    // ...or a large block smash mmap'd itself (registered in page_map_).
    auto* heap = SmashRedirect::getHeap();
    return heap && heap->hasPageMapSpan(addr);
}

// ── Start compressor from constructor ─────────────────────────────────────────
// On macOS, threadInit() requires two calls before starting compression (to
// avoid crashing the ObjC runtime during early DYLD init — pthread_create in
// startCompression() triggers task_restartable_ranges_register before ObjC is
// ready). The alloc8 pthread hooks constructor at priority 200 guarantees that
// any threads created by the ObjC runtime during early init pass through
// without calling xxthread_init(). By priority 201, DYLD init is complete and
// it is safe to start the compressor. We call xxthread_init() twice to satisfy
// the >= 1 guard, ensuring compression starts even for non-ObjC programs
// (e.g. Python via DYLD_INSERT_LIBRARIES) and single-threaded programs.
// On Linux, the same applies: no threads are created during LD_PRELOAD init.
#ifndef SMASH_NO_THREAD_HOOKS_BUILD
__attribute__((constructor(201)))  // After alloc8 pthread hooks init (200)
static void smash_start_main_thread() {
    xxthread_init();
#ifdef __APPLE__
    // Second call satisfies the macOS >= 1 guard in threadInit().
    xxthread_init();
#endif
}
#endif  // SMASH_NO_THREAD_HOOKS_BUILD

// ── Restart the compressor after fork() ──────────────────────────────────────
// Linux fork() only clones the calling thread, so the compressor's coordinator
// + helper threads vanish in the child. Without this handler the child inherits
// `compression_started_=true` and dead pthread_t handles, so its first malloc's
// `if (!started) startCompression()` check short-circuits and the child runs
// with no compressor. Postgres backends, Redis daemonized children, etc. show
// committed=N / compressed=0 because of this. Re-init the bookkeeping in the
// child so the next allocation re-runs startCompression() with fresh threads.
//
// These run on macOS too: pthread_atfork handlers fire there, and a
// multi-threaded program may fork (the child is restricted to async-signal-
// safe calls until exec, but allocators still run in it -- e.g. Python's
// subprocess, test harnesses). Any lock another thread may hold at fork time
// must be taken in prepare and released in parent/child. Locks that
// allocations need are NOT taken here but in SmashHeap::lock()/unlock()
// (xxmalloc_lock), which alloc8 runs at the allocator's point in the fork
// sequence -- see the comment there.
// ── Per-thread state in a TSD slot (macOS; see util/tsd_slot.h) ─────────────
#if defined(SMASH_USE_TSD)
namespace {
// Fallback storage, used only if BootstrapAlloc cannot supply a ThreadState.
// The slot then stays null, so every access takes the slow call (+ a
// _tlv_get_addr): correct but slow -- a safety net, not a configuration. There
// is deliberately no runtime switch to fallback: the fast path reads the slot
// unconditionally, so it cannot be made to ignore a slot that is in use
// without paying a check on every access. Disable at build time instead
// (-DSMASH_TSD_SLOT=OFF) or pick another slot (-DSMASH_TSD_SLOT_INDEX=N).
thread_local smash::ThreadState g_tsd_fallback_state;
std::atomic<size_t> g_ts_fallbacks{0};

// Reclaimed ThreadStates. Threads come and go rarely relative to allocations,
// so a spinlocked list is plenty; it also avoids a lock-free stack's ABA.
smash::Spinlock g_ts_free_lock;
smash::ThreadState* g_ts_free = nullptr;
std::atomic<size_t> g_ts_allocated{0};   // ever carved from BootstrapAlloc
std::atomic<size_t> g_ts_live{0};        // currently owned by a thread

// Destructor key: its value is the thread's ThreadState, so pthread calls
// threadStateRelease at thread exit for EVERY thread -- including GCD
// workqueue threads, which never pass through alloc8's pthread_create
// trampoline (and so never get xxthread_cleanup).
std::atomic<bool> g_ts_key_ready{false};
pthread_key_t g_ts_key;

void threadStateRelease(void* p) {
    auto* ts = static_cast<smash::ThreadState*>(p);
    if (!ts || ts->magic != smash::kThreadStateMagic) return;
    // Drain a thread cache the alloc8 trampoline did not (GCD threads, or a
    // destructor that allocated after xxthread_cleanup). The slot still points
    // at ts here, so threadCleanup() sees this thread's cache.
    if (ts->thread_cache) {
        if (auto* heap = SmashRedirect::getHeap()) heap->threadCleanup();
    }
    // Clear the slot BEFORE publishing ts for reuse: a later destructor that
    // allocates then builds a fresh state (and re-arms this key; pthread
    // repeats destructors up to PTHREAD_DESTRUCTOR_ITERATIONS) instead of
    // touching one another thread may already own.
    smash::tsdSet(nullptr);
    {
        smash::LockGuard g(g_ts_free_lock);
        ts->next_free = g_ts_free;
        g_ts_free = ts;
    }
    g_ts_live.fetch_sub(1, std::memory_order_relaxed);
}
}  // namespace

smash::ThreadState* smash::threadStateSlow() {
    ThreadState* ts = nullptr;
    {
        LockGuard g(g_ts_free_lock);
        if ((ts = g_ts_free)) g_ts_free = ts->next_free;
    }
    if (!ts) {
        void* mem = BootstrapAlloc::instance().allocate(sizeof(ThreadState), 64);
        if (!mem) {                                // out of metadata: degrade, don't crash
            g_ts_fallbacks.fetch_add(1, std::memory_order_relaxed);
            return &g_tsd_fallback_state;
        }
        ts = static_cast<ThreadState*>(mem);
        g_ts_allocated.fetch_add(1, std::memory_order_relaxed);
    }
    new (ts) ThreadState();
    tsdSet(ts);
    g_ts_live.fetch_add(1, std::memory_order_relaxed);
    // Arm the exit-time destructor. (A thread that set up its state before
    // the key existed -- only the main thread, during dyld init -- has none;
    // the main thread never runs TSD destructors anyway.)
    if (g_ts_key_ready.load(std::memory_order_acquire))
        pthread_setspecific(g_ts_key, ts);
    return ts;
}

void smash::threadStateAtforkPrepare() { g_ts_free_lock.lock(); }
void smash::threadStateAtforkParent() { g_ts_free_lock.unlock(); }
void smash::threadStateAtforkChild() {
    // Only the forking thread survives; it held the lock across fork.
    g_ts_free_lock.unlock();
}

extern "C" void smash_tsd_store_caller_ra(void* ra) {
    smash::threadState()->caller_ra = ra;
}

// Runs early (before most other constructors), on the main thread.
__attribute__((constructor(101)))
static void smash_tsd_init() {
    // The slot is unofficial (reserved by libpthread, unused by the OS), so
    // another component could pick the same one -- e.g. a second malloc
    // replacement. If the main thread's slot already holds something that is
    // not our ThreadState, sharing it would corrupt both; refuse loudly.
    void* cur = smash::tsdGet();
    if (cur && static_cast<smash::ThreadState*>(cur)->magic != smash::kThreadStateMagic) {
        static const char msg[] =
            "[smash] FATAL: pthread TSD slot " SMASH_TSD_SLOT_STR " is already in use by another "
            "library (another malloc replacement loaded?). Rebuild smash with "
            "-DSMASH_TSD_SLOT_INDEX=<free slot> or -DSMASH_TSD_SLOT=OFF.\n";
        (void)!::write(STDERR_FILENO, msg, sizeof(msg) - 1);
        abort();
    }
    if (pthread_key_create(&g_ts_key, threadStateRelease) == 0)
        g_ts_key_ready.store(true, std::memory_order_release);
    // Arm the destructor for a state the main thread set up during dyld init
    // (harmless: the main thread does not run TSD destructors on exit()).
    if (cur && g_ts_key_ready.load(std::memory_order_relaxed))
        pthread_setspecific(g_ts_key, cur);
}

// Exported diagnostic / test hook: ThreadStates ever allocated, currently
// owned by a thread, and how often the thread_local fallback was used.
extern "C" __attribute__((visibility("default")))
void smash_tsd_stats(size_t* allocated, size_t* live, size_t* fallbacks) {
    if (allocated) *allocated = g_ts_allocated.load(std::memory_order_relaxed);
    if (live) *live = g_ts_live.load(std::memory_order_relaxed);
    if (fallbacks) *fallbacks = g_ts_fallbacks.load(std::memory_order_relaxed);
}
#endif  // SMASH_USE_TSD

extern "C" {
static void smash_atfork_prepare() {
    auto* heap = SmashRedirect::getHeap();
    if (heap) heap->preparePauseForFork();
}
static void smash_atfork_parent() {
    auto* heap = SmashRedirect::getHeap();
    if (heap) heap->resumeAfterFork();
}
static void smash_atfork_child() {
    // The HeapRedirect singleton is already constructed by the time anyone is
    // forking — we accessed it on the very first malloc in the parent. Calling
    // getHeap() here doesn't allocate.
    auto* heap = SmashRedirect::getHeap();
    if (heap) heap->resetForFork();
}
}

// Exported diagnostic / test hook. Returns the arena id a smash-managed
// pointer was routed to, or -1 for a non-smash pointer. Default visibility
// (the TU is built with -fvisibility=hidden) + a version-script entry on
// Linux make it dlsym-resolvable from a program running under LD_PRELOAD /
// DYLD_INSERT_LIBRARIES. Used by test_arena_routing to guard against a
// regression of the call-site-arena collapse (return-address capture bug).
extern "C" __attribute__((visibility("default")))
int smash_arena_for(const void* ptr) {
    auto* heap = SmashRedirect::getHeap();
    return heap ? heap->arenaFor(ptr) : -1;
}

__attribute__((constructor(202)))  // After smash_start_main_thread (201)
static void smash_register_atfork() {
    pthread_atfork(smash_atfork_prepare,
                   smash_atfork_parent,
                   smash_atfork_child);
}

// High-priority destructor: runs BEFORE other destructors in
// __cxa_finalize order (lower priority number = later registration =
// runs first). Priority 101 puts this ahead of typical static-storage
// destructors and Python's interpreter teardown C-side cleanup, so
// compressed pages are drained back to PROT_RW while the compressor
// machinery is still alive. Without this, CPython's interpreter shutdown
// touches a still-COMPRESSED page, faults, the handler runs, decompresses
// using stale state, and we crash with SIGSEGV (F139).
__attribute__((destructor(101)))
static void smash_shutdown_compressor() {
    auto* heap = SmashRedirect::getHeap();
    if (heap) heap->shutdownCompressor();
}
