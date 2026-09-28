// smash/src/vm/fault_handler.h - Platform-specific page fault interception
//
// macOS:
//   - Default: SIGSEGV/SIGBUS signal handlers (compatible with ObjC
//     restartable ranges).
//   - SMASH_USE_MACH_EXCEPTIONS=1: task-level Mach exception ports for
//     EXC_BAD_ACCESS, with a dedicated handler thread. Lets smash
//     intercept protection faults before they're converted to signals,
//     which avoids the SpiderMonkey/V8 signal-handler-chain races.
// Linux: SIGSEGV/SIGBUS signal handler (userfaultfd in Phase 5)
// Windows: VEH (stub for now)
#pragma once

#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <execinfo.h>
#include <cstring>
#if defined(__linux__)
#include <dlfcn.h>
#include <pthread.h>
#endif
#ifdef __linux__
#include <sys/syscall.h>
#endif
#include "smash/config.h"
#include "../util/safe_printf.h"  // signal-handler-safe snprintf

#ifdef __APPLE__
#include <mach/mach.h>
#include <mach/exception_types.h>
#include <mach/task.h>
#include <pthread.h>
#endif

namespace smash::vm {

// Callback type: receives faulting address, returns true if handled.
using FaultCallback = bool(*)(uintptr_t fault_addr, void* context);

// Read-vs-write classification of the in-flight fault, for the decompress
// callback (soft-dirty ROI). signalHandler sets it on the faulting thread just
// before calling the callback, so a thread_local is race-free across concurrent
// faults. 1 = write, 0 = read, -1 = unknown (Mach path, or an abort we cannot
// classify). NOTE: we rely on REG_ERR/ucontext_t already being visible via
// <csignal> — do NOT add <ucontext.h> or <asm/sigcontext.h>, which conflict
// with the signal includes on glibc and corrupt the handler's ucontext view
// (caused a teardown SIGSEGV/SIGBUS).
// Accessor over a FUNCTION-LOCAL thread_local. A namespace-scope inline
// thread_local in this widely-included, LD_PRELOAD'd header broke the static
// TLS block (teardown SIGSEGV); a single .cpp definition fails to link in the
// targets that don't pull in smash_heap.cpp (compress-only, tests). A
// function-local static thread_local is header-safe (one lazily-initialized
// instance per thread, no static-TLS-block slot) and links everywhere.
//
// TLS model is PLATFORM-SPLIT and the split is load-bearing:
//   Linux: initial-exec — resolved as a fixed tpidr offset (no __tls_get_addr).
//     This is read/written from inside the SIGSEGV/SIGBUS handler, where the
//     default local-dynamic model is fatal: __tls_get_addr lazily allocates the
//     dynamic TLS block via the allocator (back into smash) and is not
//     async-signal-safe, so it faults, re-enters the handler, and recurses
//     until the stack overflows. initial-exec sidesteps it entirely.
//   macOS: DEFAULT model (no attribute). Apple's linker does NOT support the
//     initial-exec (static-offset) TLS model for a dylib loaded via
//     DYLD_INSERT_LIBRARIES — the attribute on a function-local static there
//     miscompiles into a TLS-descriptor sequence that traps with SIGILL inside
//     the fault handler under a live compressor (caught by macOS CI; Linux is
//     fine). macOS uses the signal-handler path too, but the write-bit
//     classification below is guarded on __linux__, so on Apple silicon
//     faultWasWrite is only ever -1 and the default-model lazy init is not
//     exercised the way Linux's recursive case is.
#if defined(__linux__)
inline int& faultWasWrite() {
    static thread_local __attribute__((tls_model("initial-exec"))) int v = -1;
    return v;
}
#else
inline int& faultWasWrite() {
    static thread_local int v = -1;
    return v;
}
#endif

// Decode the read/write classification of an aarch64 data abort from the
// signal frame's __reserved area. Returns 1 = write, 0 = read, -1 = unknown.
//
// The kernel appends an esr_context record ({u32 magic, u32 size, u64 esr},
// magic == ESR_MAGIC) to the chain of _aarch64_ctx records that lives in
// sigcontext::__reserved, right after the fpsimd record. ESR_EL1 bit 6 (WnR)
// is the write-not-read bit. We walk the chain by hand rather than including
// <asm/sigcontext.h>: the note on faultWasWrite() above applies — the kernel
// and glibc signal headers conflict here, and pulling either in corrupts the
// handler's ucontext view.
//
// Split out as a pure function over bytes (no arch-specific types) so it is
// testable on any host — see tests/test_esr_write_bit.cpp, which feeds it
// synthetic frames. Async-signal-safe: bounded loop, fixed-size memcpys,
// no allocation.
inline int aarch64EsrWriteBit(const void* reserved, size_t reserved_len) {
    constexpr uint32_t kEsrMagic   = 0x45535201u;  // ESR_MAGIC
    constexpr uint32_t kExtraMagic = 0x45585401u;  // EXTRA_MAGIC
    constexpr size_t kCtxHeaderLen = 8;            // {u32 magic, u32 size}

    if (!reserved) return -1;
    const unsigned char* p = static_cast<const unsigned char*>(reserved);
    const unsigned char* const end = p + reserved_len;

    // The chain is bounded by __reserved (4 KiB), so records are few; the
    // iteration cap is belt-and-braces against a malformed size field that
    // still passes the bounds check.
    for (int guard = 0; guard < 64 && p + kCtxHeaderLen <= end; ++guard) {
        uint32_t magic = 0, size = 0;
        __builtin_memcpy(&magic, p, sizeof(magic));
        __builtin_memcpy(&size, p + 4, sizeof(size));
        // magic == 0 is the end-of-chain terminator. A size that is too small
        // or runs past the buffer means the frame is not what we expect.
        if (magic == 0 || size < kCtxHeaderLen ||
            static_cast<size_t>(size) > static_cast<size_t>(end - p)) {
            return -1;
        }
        if (magic == kEsrMagic) {
            if (size < kCtxHeaderLen + sizeof(uint64_t)) return -1;
            uint64_t esr = 0;
            __builtin_memcpy(&esr, p + kCtxHeaderLen, sizeof(esr));
            // EC is ESR bits 31:26.
            const uint32_t ec = static_cast<uint32_t>((esr >> 26) & 0x3fu);
            if (ec == 0x20 || ec == 0x21) {
                // Instruction abort (lower EL / same EL): an instruction
                // fetch, never a store.
                return 0;
            }
            if (ec == 0x24 || ec == 0x25) {
                // Data abort. WnR (bit 6) is only meaningful when CM (bit 8,
                // "fault from a cache-maintenance or address-translation
                // instruction") is clear — the architecture reports WnR == 1
                // for those regardless of direction, so report unknown rather
                // than a write we did not observe.
                if (esr & (1ull << 8)) return -1;
                return (esr & (1ull << 6)) ? 1 : 0;
            }
            return -1;  // Not an abort we can classify.
        }
        // EXTRA_MAGIC continues the chain in an out-of-line buffer. The
        // kernel emits esr_context before it, so reaching this means there is
        // no ESR record to find.
        if (magic == kExtraMagic) return -1;
        p += size;
    }
    return -1;
}

// Per-thread recursion depth for the SIGSEGV/SIGBUS handler (bounded re-entry
// guard — see signalHandler). LINUX ONLY.
//
// The unbounded recursion this guards against is a Linux mechanism: a fault
// inside libc's __dtoa/__Balloc malloc path re-enters the SA_NODEFER handler
// on a fresh address each time, and the default-model TLS access itself
// recurses via __tls_get_addr → stack overflow. On macOS we must NOT touch
// any TLS from inside the signal handler at all: the first per-thread access
// routes through the dyld thread-local resolver, whose lazy init traps SIGILL
// in a signal context (the same reason faultWasWrite is kept untouched on
// macOS; adding this guard there made bench_sqlite die -4 in CI). macOS
// therefore keeps master's guard-free handler, which is stable in CI.
//
// initial-exec so the static-offset access needs no __tls_get_addr; __thread
// (C-style) has no lazy-init guard.
#if defined(__linux__)
inline __thread __attribute__((tls_model("initial-exec"))) int g_fault_depth = 0;
inline int& faultDepth() { return g_fault_depth; }
#endif

#if defined(__APPLE__) || defined(__linux__)

// Signal-based fault handler for macOS and Linux.
// Uses SIGSEGV/SIGBUS to detect accesses to protected pages.
class FaultHandler {
    FaultCallback callback_ = nullptr;
    void* callback_ctx_ = nullptr;
    std::atomic<bool> running_{false};

    // The handler a fault smash does not resolve is passed on to, per signal
    // ([0] SIGSEGV, [1] SIGBUS): whatever was installed before smash, or what
    // the application has installed since through sigaction()/signal(),
    // which smash interposes (see interposedSigaction). The application
    // writes it from any thread while the signal handler may be reading it,
    // so it is a seqlock: writers serialise on chain_lock_ and bump `seq`
    // around the copy; the handler retries a read that raced a write. A
    // writer only ever touches smash's own memory while holding the lock, so
    // it cannot fault there and deadlock against the handler.
    struct Chain {
        std::atomic<uint32_t> seq{0};
        struct sigaction act{};
    };
    Chain chain_[2];
    static inline std::atomic_flag chain_lock_ = ATOMIC_FLAG_INIT;
    static int chainIndex(int sig) { return sig == SIGBUS ? 1 : 0; }
    void writeChain(int sig, const struct sigaction& a) {
        Chain& c = chain_[chainIndex(sig)];
        while (chain_lock_.test_and_set(std::memory_order_acquire)) {}
        const uint32_t s0 = c.seq.load(std::memory_order_relaxed);
        c.seq.store(s0 + 1, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
        std::memcpy(&c.act, &a, sizeof(a));
        c.seq.store(s0 + 2, std::memory_order_release);
        chain_lock_.clear(std::memory_order_release);
    }
    struct sigaction readChain(int sig) const {
        const Chain& c = chain_[chainIndex(sig)];
        struct sigaction out;
        for (;;) {
            const uint32_t s0 = c.seq.load(std::memory_order_acquire);
            if (s0 & 1) continue;
            std::memcpy(&out, &c.act, sizeof(out));
            std::atomic_thread_fence(std::memory_order_acquire);
            if (c.seq.load(std::memory_order_relaxed) == s0) return out;
        }
    }

    // libc's own sigaction. smash exports sigaction() (and signal() and
    // friends) on Linux, so a plain call from smash's code would land in its
    // own wrapper; installing and restoring the kernel's handler must go
    // straight to libc. Resolved in start(), before any signal can arrive,
    // so the signal handler only reads the pointer.
    using SigactionFn = int (*)(int, const struct sigaction*, struct sigaction*);
    static inline SigactionFn real_sigaction_ = nullptr;
    static int sysSigaction(int sig, const struct sigaction* act, struct sigaction* oldact) {
#if defined(__linux__)
        if (!real_sigaction_)
            real_sigaction_ = reinterpret_cast<SigactionFn>(dlsym(RTLD_NEXT, "sigaction"));
        if (real_sigaction_) return real_sigaction_(sig, act, oldact);
#endif
        return ::sigaction(sig, act, oldact);
    }
    // SMASH_NO_SIGACTION_WRAP=1: the application's sigaction()/signal() go
    // to libc untouched and smash re-claims its handler on compressor ticks
    // only (the behaviour before the interposition).
    static inline bool sigaction_wrap_disabled_ = false;

#ifdef __APPLE__
    // Mach exception ports mode (SMASH_USE_MACH_EXCEPTIONS=1).
    bool mach_mode_ = false;
    mach_port_t mach_exc_port_ = MACH_PORT_NULL;
    pthread_t mach_handler_thread_ = 0;
#endif

    static FaultHandler* instance_;

    // Set when we've decided this signal terminates the process.
    // ensureInstalled() checks it before re-arming our handler so the
    // compressor's periodic re-install doesn't race with the
    // sigaction(SIG_DFL) → raise() chain we use to propagate
    // user-space-synthesized SIGSEGV/SIGBUS to the original
    // disposition.
    static inline std::atomic<bool> shutting_down_{false};

    // Signal-handler env flags, cached at install time. getenv() is NOT
    // async-signal-safe: calling it from signalHandler() can fault inside
    // libc's environ walk, and with SA_NODEFER that re-enters signalHandler,
    // which calls getenv() again → unbounded recursion → stack-overflow
    // SIGSEGV. (Observed as redis-server under LD_PRELOAD dumping core with a
    // getenv → signalHandler → __restore_rt → getenv … backtrace.) Read these
    // once in start()/ensureInstalled() and consult the cached bools only.
    static inline bool sigtrace_enabled_ = false;

public:
    // Set by SmashHeap. Fills `buf` with a short description of `addr` (arena
    // page and state, smash metadata, tracked external page, foreign memory).
    // Must be async-signal-safe: it runs in a signal handler, in a process
    // that is about to die.
    static inline void (*describe_hook_)(const void* addr, char* buf, size_t len) = nullptr;
private:
    static inline bool dump_crash_bt_enabled_ = false;

    // Detect signals that were synthesized by user-space (raise(),
    // pthread_kill(), kill(), tgkill()) rather than from a real memory
    // fault. For these, info->si_code is non-positive (SI_USER=0,
    // SI_TKILL=-6, SI_QUEUE=-1, etc.) and info->si_addr is not a real
    // faulting address. POSIX guarantees positive si_code values are
    // kernel-synchronous fault sources (SEGV_MAPERR=1, SEGV_ACCERR=2,
    // BUS_ADRALN, etc.).
    //
    // For user-synthesized SIGSEGV/SIGBUS the correct chain action is
    // "act as if smash hadn't installed a handler at all": uninstall
    // ourselves and re-raise so the original disposition (often
    // SIG_DFL → terminate, sometimes a Python C-level handler) takes
    // over. Without this carve-out, a user-space raise(SIGSEGV)
    // fed through smash's handler chains to whatever old_sigsegv_
    // was — which on Linux+Python is SIG_DFL — and the chain logic
    // works *most* of the time, but races with ensureInstalled() can
    // re-arm our handler in the µs-window between signal(SIG_DFL)
    // and raise(), producing an infinite loop.
    //
    // WalrusDriver.py:518 calls signal.raise_signal(-rc) on backend
    // SIGSEGV exits, which is the production trigger for this path.
    static bool isKernelFault(const siginfo_t* info) {
        return info->si_code > 0;
    }

    // Bounded re-entrancy guard. The SIGSEGV/SIGBUS handlers are installed
    // with SA_NODEFER (deliberate: a legitimate decompress can itself fault
    // once, e.g. prefetching an adjacent compressed page, and that nested
    // fault must be handled). But if callback_ (handleFault) faults on a
    // DIFFERENT address every time — e.g. when the originating fault came
    // from inside libc's __dtoa/__Balloc malloc path while formatting a
    // float — the per-page reentrancy guard never matches and the handler
    // recurses without bound until it overflows the signal stack
    // ("Could not determine thread index for stack guard region").
    // Cap the depth: past kMaxFaultDepth nested kernel faults on one thread
    // we stop trying to handle and let the fault chain to the previous
    // disposition, producing a clean crash with a real backtrace instead of
    // a stack-overflow. 2 allows one legitimate nested decompress fault.
    static constexpr int kMaxFaultDepth = 2;

    static void signalHandler(int sig, siginfo_t* info, void* ucontext) {
        int saved_errno = errno;
        // Bounded re-entry guard is LINUX ONLY — see faultDepth() decl for why
        // macOS must not touch TLS in the handler. On macOS depthOk is always
        // true (master's guard-free behavior); the Linux path caps recursion.
#if defined(__linux__)
        int& depth = faultDepth();
        const bool depthOk = depth < kMaxFaultDepth;
#else
        const bool depthOk = true;
#endif
        if (isKernelFault(info) && instance_ && instance_->callback_ && depthOk) {
#if defined(__linux__)
            ++depth;
            struct DepthGuard {
                int& d;
                ~DepthGuard() { --d; }
            } depth_guard{depth};
#endif
            uintptr_t addr = reinterpret_cast<uintptr_t>(info->si_addr);
            // Classify read vs write for the soft-dirty ROI re-dirty signal.
            // x86-64: bit 1 of the page-fault error code (gregs[REG_ERR]) is the
            // write bit. aarch64: bit 6 (WnR) of ESR_EL1, which the kernel
            // passes in an esr_context record inside the signal frame — see
            // aarch64EsrWriteBit(). Both reach it through the ucontext_t that
            // <csignal> already gives us; we deliberately do NOT include
            // <ucontext.h> or <asm/sigcontext.h>.
            //
            // LINUX ONLY: touch faultWasWrite() here. On macOS the value is
            // never set meaningfully (the classification is guarded on
            // __linux__) and the lone reader (compressor_thread.h) already
            // treats the unset/-1 case as "write". More importantly, the macOS
            // function-local thread_local carries a lazy-init guard that, when
            // first run INSIDE this signal handler under a live compressor,
            // traps with SIGILL (observed deterministically in macOS CI:
            // bench_sqlite dies in the fault/decompress phase). Not touching
            // the TLS in the handler on macOS avoids that guard entirely; the
            // compressor thread inits it lazily (safely, not in a handler) on
            // its first read.
#if defined(__linux__)
            faultWasWrite() = -1;
#if defined(__x86_64__) && defined(REG_ERR)
            if (ucontext) {
                auto* uc = static_cast<const ucontext_t*>(ucontext);
                faultWasWrite() =
                    (uc->uc_mcontext.gregs[REG_ERR] & 0x2) ? 1 : 0;
            }
#elif defined(__aarch64__)
            // aarch64: the write bit lives in ESR_EL1 (bit 6, WnR), which the
            // kernel hands us in an esr_context record inside the signal
            // frame's __reserved area. Without this the value stayed -1 on
            // every aarch64 fault and the sole reader
            // (compressor_thread.h's soft-dirty ROI re-dirty signal) counted
            // every decompress fault as a write — safe, but blind: a
            // read-faulting page would have kept its blob and should not have
            // been scored as churn. sigcontext::__reserved is reachable
            // through the ucontext_t that <csignal> already gives us, so no
            // extra header is needed.
            if (ucontext) {
                auto* uc = static_cast<const ucontext_t*>(ucontext);
                faultWasWrite() =
                    aarch64EsrWriteBit(&uc->uc_mcontext.__reserved,
                                       sizeof(uc->uc_mcontext.__reserved));
            }
#endif
#else
            (void)ucontext;
#endif
            if (instance_->callback_(addr, instance_->callback_ctx_)) {
                errno = saved_errno;
                return;  // Fault handled, resume execution
            }
        }
        // Not our fault — chain to previous handler.
        // KNOWN ISSUE on macOS with SpiderMonkey/V8/wasm: those engines
        // install their own SIGBUS handler in a way that, combined with
        // ensureInstalled() re-claiming, can recurse smash → engine →
        // smash. The clean fix is task-level Mach exception ports
        // instead of POSIX signal handlers. Tracked in FIREFOX_PLAN.md.
        struct sigaction chained = instance_->readChain(sig);
        struct sigaction* old = &chained;

        // SMASH_SIGTRACE=1 — async-signal-safe one-line trace per chain.
        // Flag cached at install time; getenv() here would not be
        // async-signal-safe (see sigtrace_enabled_ declaration).
        if (sigtrace_enabled_) {
            char buf[480];
            const char* kind = "?";
            if (old->sa_handler == SIG_DFL) kind = "SIG_DFL";
            else if (old->sa_handler == SIG_IGN) kind = "SIG_IGN";
            else if (old->sa_flags & SA_SIGINFO) kind = "SA_SIGINFO";
            else kind = "OTHER";
            // What smash thinks the address is, as the fatal line says: the
            // one thing needed when the application's own handler takes the
            // fault next and smash's fatal line never prints.
            char what[288];
            what[0] = '\0';
            if (describe_hook_) describe_hook_(info->si_addr, what, sizeof(what));
            int n = smash::safe_snprintf(buf, sizeof(buf),
                "[smash sig] sig=%d si_code=%d si_addr=%p chain_kind=%s flags=0x%x %s\n",
                sig, info->si_code, info->si_addr, kind,
                (unsigned)old->sa_flags, what[0] ? what : "address not classified");
            if (n > 0) (void)!write(2, buf, (size_t)n);
        }

        // A fault smash did not handle, with SIG_DFL behind us, kills the
        // process. Say so: an application whose only symptom is "it died with
        // no message" cannot tell a smash bug from its own, and the faulting
        // address plus what smash thinks that address is settles it.
        if (old->sa_handler == SIG_DFL) {
            char what[288];
            what[0] = '\0';
            if (describe_hook_) describe_hook_(info->si_addr, what, sizeof(what));
            char buf[448];
            const int n = smash::safe_snprintf(buf, sizeof(buf),
                "[smash FATAL] unhandled sig=%d si_code=%d at addr=%p — the process is "
                "about to die. %s\n",
                sig, info->si_code, info->si_addr,
                what[0] ? what : "address not classified");
            if (n > 0) (void)!write(2, buf, static_cast<size_t>(n));
        }

        // For non-kernel-synchronous faults (raise/kill/etc.) that we
        // didn't handle, we MUST uninstall ourselves before returning;
        // otherwise the kernel resumes the faulting instruction (the
        // raise() syscall return), the caller's next instruction
        // executes, and the user's intent (terminate the process) is
        // silently dropped. By restoring the old sigaction and calling
        // raise() ourselves, we let the original disposition fire.
        if (!isKernelFault(info)) {
            shutting_down_.store(true, std::memory_order_release);
            sysSigaction(sig, old, nullptr);
            raise(sig);
            errno = saved_errno;
            return;
        }

        // SMASH_DUMP_CRASH_BT=1 — dump a backtrace before terminating.
        // Useful for diagnosing real segfaults that smash didn't handle
        // (e.g. compiler crashes in app code under full smash mode).
        // Only fires for kernel-fault SIGSEGV/SIGBUS that chain to
        // SIG_DFL — i.e. the process is about to die anyway.
        // Flag cached at install time (getenv() is not async-signal-safe).
        bool want_bt = dump_crash_bt_enabled_ &&
                       isKernelFault(info) &&
                       old->sa_handler == SIG_DFL;
        if (want_bt) {
            // Portable thread id: SYS_gettid is Linux-only; macOS uses
            // pthread_threadid_np. Diagnostic-only.
#if defined(__linux__)
            long tid = syscall(SYS_gettid);
#elif defined(__APPLE__)
            uint64_t tid64 = 0; pthread_threadid_np(nullptr, &tid64);
            long tid = static_cast<long>(tid64);
#else
            long tid = 0;
#endif
            char hdr[160];
            int n = smash::safe_snprintf(hdr, sizeof(hdr),
                "[smash crash] pid=%d tid=%ld sig=%d si_code=%d si_addr=%p\n",
                (int)getpid(), tid,
                sig, info->si_code, info->si_addr);
            if (n > 0) (void)!write(2, hdr, (size_t)n);
            void* frames[64];
            int nframes = backtrace(frames, 64);
            backtrace_symbols_fd(frames, nframes, 2);
        }

        // SIG_DFL and SIG_IGN are decided by the handler value alone, as the
        // kernel does: sa_handler and sa_sigaction share storage, and a
        // one-shot SA_SIGINFO handler reads back SIG_DFL with SA_SIGINFO
        // still set once it has fired.
        if (old->sa_handler == SIG_DFL) {
            shutting_down_.store(true, std::memory_order_release);
            sysSigaction(sig, old, nullptr);
            raise(sig);
        } else if (old->sa_handler == SIG_IGN) {
            // Synchronous fault + caller asked to ignore = the
            // faulting instruction would re-execute forever. Force
            // SIG_DFL termination instead.
            shutting_down_.store(true, std::memory_order_release);
            struct sigaction dfl{};
            dfl.sa_handler = SIG_DFL;
            sigemptyset(&dfl.sa_mask);
            sysSigaction(sig, &dfl, nullptr);
            raise(sig);
        } else {
            // The application's handler, run as the kernel would have run it:
            // with its sa_mask (plus the signal itself unless SA_NODEFER)
            // blocked, and reset to SIG_DFL first if it asked for SA_RESETHAND
            // (only the handler, as the kernel resets it; the flags stay).
            if (old->sa_flags & SA_RESETHAND) {
                struct sigaction reset = *old;
                reset.sa_handler = SIG_DFL;
                instance_->writeChain(sig, reset);
            }
            sigset_t block = old->sa_mask, prev;
            if (!(old->sa_flags & SA_NODEFER)) sigaddset(&block, sig);
            pthread_sigmask(SIG_BLOCK, &block, &prev);
            if (old->sa_flags & SA_SIGINFO) old->sa_sigaction(sig, info, ucontext);
            else old->sa_handler(sig);
            pthread_sigmask(SIG_SETMASK, &prev, nullptr);
        }
    }

#ifdef __APPLE__
    // Layout copied from /usr/include/mach/mach_exc.defs (the
    // exception_raise routine the kernel uses to deliver
    // EXC_BAD_ACCESS to a task-level exception port). Same shape
    // SpiderMonkey uses in WasmSignalHandlers.cpp:605.
#pragma pack(push, 4)
    struct ExceptionRaiseRequest {
        mach_msg_header_t Head;
        mach_msg_body_t msgh_body;
        mach_msg_port_descriptor_t thread;
        mach_msg_port_descriptor_t task;
        NDR_record_t NDR;
        exception_type_t exception;
        mach_msg_type_number_t codeCnt;
        int64_t code[2];
    };
    struct ExceptionRaiseRequestWithTrailer {
        ExceptionRaiseRequest body;
        mach_msg_trailer_t trailer;
    };
    struct ExceptionRaiseReply {
        mach_msg_header_t Head;
        NDR_record_t NDR;
        kern_return_t RetCode;
    };
#pragma pack(pop)

    static void* machHandlerThreadEntry(void* arg) {
        auto* self = static_cast<FaultHandler*>(arg);
        // Standard reply id is request.msgh_id + 100 per mach_exc.defs.
        for (;;) {
            ExceptionRaiseRequestWithTrailer req;
            kern_return_t kr = mach_msg(&req.body.Head, MACH_RCV_MSG, 0,
                                        sizeof(req), self->mach_exc_port_,
                                        MACH_MSG_TIMEOUT_NONE,
                                        MACH_PORT_NULL);
            if (kr != KERN_SUCCESS) continue;

            // For EXC_BAD_ACCESS with MACH_EXCEPTION_CODES:
            //   code[0] = subcode (KERN_INVALID_ADDRESS, KERN_PROTECTION_FAILURE)
            //   code[1] = faulting address
            bool handled = false;
            if (req.body.exception == EXC_BAD_ACCESS &&
                req.body.codeCnt >= 2 && self->callback_) {
                uintptr_t addr =
                    static_cast<uintptr_t>(req.body.code[1]);
                handled = self->callback_(addr, self->callback_ctx_);
            }
            // Optional one-line stderr trace per exception, for tuning.
            // SMASH_MACH_TRACE=1 to enable. Async-signal-safe ish (not
            // strictly, but this thread isn't inside a signal handler).
            static const bool trace = []{
                const char* v = std::getenv("SMASH_MACH_TRACE");
                return v && v[0] == '1';
            }();
            if (trace) {
                char buf[128];
                int n = smash::safe_snprintf(buf, sizeof(buf),
                    "[smash mach] exc=%d code=[%llx,%llx] handled=%d\n",
                    req.body.exception,
                    (unsigned long long)req.body.code[0],
                    (unsigned long long)(req.body.codeCnt >= 2 ? req.body.code[1] : 0),
                    handled);
                if (n > 0) write(2, buf, (size_t)n);
            }

            ExceptionRaiseReply reply;
            reply.Head.msgh_bits = MACH_MSGH_BITS(
                MACH_MSGH_BITS_REMOTE(req.body.Head.msgh_bits), 0);
            reply.Head.msgh_size = sizeof(reply);
            reply.Head.msgh_remote_port = req.body.Head.msgh_remote_port;
            reply.Head.msgh_local_port = MACH_PORT_NULL;
            reply.Head.msgh_id = req.body.Head.msgh_id + 100;
            reply.NDR = NDR_record;
            // KERN_SUCCESS  → resume faulting thread (we fixed the page)
            // KERN_FAILURE  → kernel falls back to host (= POSIX signal),
            //                 so any installed signal handler still gets
            //                 a chance.
            reply.RetCode = handled ? KERN_SUCCESS : KERN_FAILURE;
            mach_msg(&reply.Head, MACH_SEND_MSG, sizeof(reply), 0,
                     MACH_PORT_NULL, MACH_MSG_TIMEOUT_NONE,
                     MACH_PORT_NULL);
        }
        return nullptr;
    }

    bool startMach() {
        kern_return_t kr;
        kr = mach_port_allocate(mach_task_self(),
                                MACH_PORT_RIGHT_RECEIVE,
                                &mach_exc_port_);
        if (kr != KERN_SUCCESS) return false;

        kr = mach_port_insert_right(mach_task_self(), mach_exc_port_,
                                    mach_exc_port_,
                                    MACH_MSG_TYPE_MAKE_SEND);
        if (kr != KERN_SUCCESS) {
            mach_port_destroy(mach_task_self(), mach_exc_port_);
            mach_exc_port_ = MACH_PORT_NULL;
            return false;
        }

        // Spawn the listener BEFORE registering the port; the kernel
        // could deliver an exception immediately afterwards.
        if (pthread_create(&mach_handler_thread_, nullptr,
                           machHandlerThreadEntry, this) != 0) {
            mach_port_destroy(mach_task_self(), mach_exc_port_);
            mach_exc_port_ = MACH_PORT_NULL;
            return false;
        }

        kr = task_set_exception_ports(
            mach_task_self(),
            EXC_MASK_BAD_ACCESS,
            mach_exc_port_,
            EXCEPTION_DEFAULT | MACH_EXCEPTION_CODES,
            THREAD_STATE_NONE);
        if (kr != KERN_SUCCESS) {
            // Listener thread is still spinning on mach_msg from a port
            // we haven't registered — it'll just block forever. Leave
            // it; we're doomed anyway since fault interception is what
            // the whole library exists to do.
            return false;
        }
        mach_mode_ = true;
        return true;
    }
#endif // __APPLE__

public:
    bool start(FaultCallback cb, void* ctx) {
        callback_ = cb;
        callback_ctx_ = ctx;
        instance_ = this;

        // Cache signal-handler env flags ONCE here, before the handler can
        // ever run. signalHandler() must not call getenv() (not
        // async-signal-safe; recurses under SA_NODEFER — see flag decls).
        const char* st = std::getenv("SMASH_SIGTRACE");
        sigtrace_enabled_ = st && st[0] == '1';
        const char* bt = std::getenv("SMASH_DUMP_CRASH_BT");
        dump_crash_bt_enabled_ = bt && bt[0] == '1';

#ifdef __APPLE__
        const char* mach_env = std::getenv("SMASH_USE_MACH_EXCEPTIONS");
        if (mach_env && mach_env[0] == '1') {
            if (startMach()) {
                running_.store(true);
                return true;
            }
            // Fall through to signals if Mach setup failed.
        }
#endif

        const char* nw = std::getenv("SMASH_NO_SIGACTION_WRAP");
        sigaction_wrap_disabled_ = nw && nw[0] == '1';

        // No SA_ONSTACK, deliberately: smash's handler decompresses pages,
        // and alternate signal stacks are often small (Rust gives every
        // thread one of ~8 KiB). It runs on the faulting thread's own stack,
        // as it always has — so a stack overflow still cannot reach an
        // application's alternate-stack handler through smash.
        struct sigaction sa{};
        sa.sa_sigaction = signalHandler;
        sa.sa_flags = SA_SIGINFO | SA_NODEFER;
        sigemptyset(&sa.sa_mask);

        struct sigaction prev{};
        sysSigaction(SIGSEGV, &sa, &prev);
        writeChain(SIGSEGV, prev);
        sysSigaction(SIGBUS, &sa, &prev);
        writeChain(SIGBUS, prev);
        running_.store(true, std::memory_order_release);
        return true;
    }

    // sigaction() for the application (smash exports it on Linux, and
    // __sigaction, signal, bsd_signal, ssignal, sysv_signal, sigset on top).
    // For SIGSEGV and SIGBUS while smash's handler is installed, the
    // application's handler becomes the one smash passes foreign faults to,
    // `oldact` reports the one it replaces, and the kernel keeps smash's
    // handler: before, the application's handler replaced smash's until the
    // next compressor tick (0.3-3 s), and every fault smash had to resolve
    // in that window — a compressed or monitored page — reached the
    // application as a crash. Everything else goes to libc.
    static int interposedSigaction(int sig, const struct sigaction* act,
                                   struct sigaction* oldact) {
        FaultHandler* h = instance_;
        if ((sig != SIGSEGV && sig != SIGBUS) || !h || sigaction_wrap_disabled_ ||
            !h->running_.load(std::memory_order_acquire) ||
            shutting_down_.load(std::memory_order_acquire))
            return sysSigaction(sig, act, oldact);
#ifdef __APPLE__
        if (h->mach_mode_) return sysSigaction(sig, act, oldact);
#endif
        // Copy the caller's structs outside the lock: they are the
        // application's memory, and touching one can fault into smash.
        struct sigaction in;
        if (act) std::memcpy(&in, act, sizeof(in));
        const struct sigaction prev = h->readChain(sig);
        if (act) h->writeChain(sig, in);
        if (oldact) std::memcpy(oldact, &prev, sizeof(prev));
        return 0;
    }

    void stop() {
        if (!running_.load()) return;
#ifdef __APPLE__
        if (mach_mode_) {
            // Releasing the task exception port and the listener thread
            // cleanly is fiddly; for our use case stop() only fires at
            // process teardown so the kernel reaps both. Just mark down.
            running_.store(false);
            instance_ = nullptr;
            return;
        }
#endif
        const struct sigaction segv = readChain(SIGSEGV), bus = readChain(SIGBUS);
        sysSigaction(SIGSEGV, &segv, nullptr);
        sysSigaction(SIGBUS, &bus, nullptr);
        running_.store(false);
        instance_ = nullptr;
    }

    // Idempotent, thread-safe entry point for "make sure the signal
    // handlers are still ours." Safe to call from any thread at any
    // time; no-op if start() hasn't been called yet.
    static void ensureInstalledIfRunning() {
        if (instance_) instance_->ensureInstalled();
    }

    // Check if our signal handlers are still installed; reinstall if
    // overwritten. Called periodically from CompressorThread::tick().
    // We deliberately reinstall **only SIGSEGV**, not SIGBUS — unless
    // deferred-reclaim mode is active. macOS delivers SIGBUS for
    // KERN_PROTECTION_FAILURE on anonymous pages. SpiderMonkey installs
    // its own SIGBUS handler with proper chaining, so re-installing SIGBUS
    // would invert that chain. But for MongoDB (deferred-reclaim target),
    // MongoDB's SIGBUS handler does NOT chain — it aborts. So we must
    // reclaim SIGBUS to handle page faults from PROT_NONE pages.
    void ensureInstalled() {
        if (!running_.load(std::memory_order_relaxed)) return;
        // Don't fight the chained termination path. signalHandler sets
        // this when it's restored SIG_DFL (or another disposition) and
        // is on its way out via raise(); reinstalling our handler here
        // would race the kernel's pending delivery and loop.
        if (shutting_down_.load(std::memory_order_acquire)) return;
#ifdef __APPLE__
        if (mach_mode_) return;
#endif

        // With sigaction() interposed this only fires for a handler
        // installed around libc (the raw rt_sigaction syscall, or the
        // interposition switched off).
        struct sigaction current{};
        sysSigaction(SIGSEGV, nullptr, &current);
        bool need_reinstall = (current.sa_sigaction != signalHandler);

#ifdef __APPLE__
        if (!need_reinstall && isDeferredReclaimMode()) {
            sysSigaction(SIGBUS, nullptr, &current);
            need_reinstall = (current.sa_sigaction != signalHandler);
        }
#endif

        if (!need_reinstall) return;

        struct sigaction sa{};
        sa.sa_sigaction = signalHandler;
        sa.sa_flags = SA_SIGINFO | SA_NODEFER;
        sigemptyset(&sa.sa_mask);
        struct sigaction displaced{};
        sysSigaction(SIGSEGV, &sa, &displaced);
        writeChain(SIGSEGV, displaced);
#ifdef __APPLE__
        if (isDeferredReclaimMode()) {
            sysSigaction(SIGBUS, &sa, &displaced);
            writeChain(SIGBUS, displaced);
        }
#endif
    }

    ~FaultHandler() { stop(); }
};

inline FaultHandler* FaultHandler::instance_ = nullptr;

#else

// Windows / other: stub
class FaultHandler {
public:
    bool start(FaultCallback, void*) { return false; }
    void stop() {}
};

#endif

} // namespace smash::vm
