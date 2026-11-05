#pragma once
// Per-thread allocator state in a reserved pthread TSD slot (macOS).
//
// On macOS every access to a thread_local from a dylib goes through
// _tlv_get_addr (a function call), even with tls_model("initial-exec"):
// Mach-O has no static TLS model. Under smash that call was the hottest
// non-waiting frame in allocation-heavy programs (jaq: ~half of all busy
// samples). mimalloc avoids it by keeping its per-thread heap pointer in a
// TSD slot that libpthread reserves for system frameworks but leaves unused:
// slot 89, __PTK_FRAMEWORK_OLDGC_KEY9 (the Objective-C garbage collector,
// removed in OS X 10.8). pthread_key_create() never hands out reserved slots,
// so no ordinary pthread key can collide with it. Reading it is one `mrs`
// plus a load.
//
// Slot SMASH_TSD_SLOT holds the thread's ThreadState* (smash_heap.h). TSD
// slots are zero in a new thread, so null means "not set up yet"; the cold
// path (threadStateSlow, smash_heap.cpp) allocates or reuses one, and a
// pthread key destructor returns it at thread exit.
//
// This header is force-included into every libsmash TU, including alloc8's
// wrapper sources, so ALLOC8_CALLER_RA_HOOK stores the caller hint straight
// into ThreadState instead of alloc8's own thread_local.

// arm64 only (Apple silicon, arm64 and arm64e). x86-64 macOS keeps the
// thread_locals: the %gs-relative equivalent is easy to write (mimalloc has
// one) but could not be validated -- smash itself is unstable on x86-64 under
// Rosetta with or without this change, and no Intel Mac was available.
#if defined(__APPLE__) && defined(SMASH_TSD_SLOT) && defined(__aarch64__)
#define SMASH_USE_TSD 1

namespace smash {

// Not volatile: within one function the thread cannot change, so the thread
// pointer may be reused; the slot itself is read as memory, so the compiler
// reloads it after any call that may have installed a new state.
// The asm yields only the thread pointer; the slot is read by an ordinary
// load, which the compiler redoes after any call that may have written it
// (threadStateSlow installs the state). Do NOT fold the load into the asm
// without declaring the slot as a memory operand.
[[gnu::always_inline]] static inline void** tsdThreadPointer() {
    void** tcb;
    __asm__("mrs %0, tpidrro_el0\n\tbic %0, %0, #7" : "=r"(tcb));
    return tcb;
}

[[gnu::always_inline]] static inline void* tsdGet() {
    return tsdThreadPointer()[SMASH_TSD_SLOT];
}

[[gnu::always_inline]] static inline void tsdSet(void* v) {
    tsdThreadPointer()[SMASH_TSD_SLOT] = v;
}

}  // namespace smash

// Cold path of the caller-hint store: the thread has no ThreadState yet (or
// BootstrapAlloc was exhausted). Creates the state if needed, so even a
// thread's first allocation keeps its call site.
extern "C" void smash_tsd_store_caller_ra(void* ra);

// ThreadState::caller_ra is the struct's first field, so the fast path needs
// only the slot.
#define ALLOC8_CALLER_RA_HOOK(ra)                                             \
    do {                                                                       \
        void** smash_ts_ = static_cast<void**>(::smash::tsdGet());             \
        if (__builtin_expect(smash_ts_ != nullptr, 1)) smash_ts_[0] = (ra);    \
        else smash_tsd_store_caller_ra(ra);                                    \
    } while (0)

#endif
