// early_alloc.h - a static arena for allocations made while smash itself is
// being set up.
//
// Two windows exist in which an allocation must not reach the heap:
//
// - SmashHeap's constructor, on the thread constructing it. alloc8's
//   HeapRedirect::getHeap() parks every other thread until construction is
//   done and makes no provision for the constructing thread coming back, so
//   an allocation from inside the constructor spun forever with no message.
//   glibc before 2.34 does exactly that: its dlsym() callocs a per-thread
//   dlerror record on a thread's first call, and the constructor's first
//   mmap() goes through smash's wrapper, which dlsym()s its real target.
//   The release floor (glibc 2.28) is such a glibc.
// - SystemAllocFns::resolve(), which dlsym()s the system malloc family. On
//   the same glibc its calloc arrived at smash's calloc with no system
//   calloc to hand it to, which resolved again, recursively, until the
//   stack ran out.
//
// A thread inside either window has its malloc/calloc/memalign served from
// here: a bump pointer over a zeroed static buffer, never reused. free() of
// such a block is a no-op, realloc() copies it out, malloc_usable_size()
// reads its header. Threads outside a window never look at this file beyond
// one relaxed load of g_early_active.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace smash::early {

// Threads currently inside a window (any thread). Zero almost always: the
// malloc path reads this and nothing else.
inline constinit std::atomic<int> g_early_active{0};
// This thread's window depth (the resolve window nests inside the constructor's).
extern __attribute__((tls_model("initial-exec"))) constinit thread_local int t_early_depth;

inline bool inWindow() {
    if (g_early_active.load(std::memory_order_relaxed) == 0) [[likely]] return false;
    return t_early_depth > 0;
}

struct Window {
    Window() {
        if (t_early_depth++ == 0) g_early_active.fetch_add(1, std::memory_order_relaxed);
    }
    ~Window() {
        if (--t_early_depth == 0) g_early_active.fetch_sub(1, std::memory_order_relaxed);
    }
    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;
};

// 256 KiB: a dlerror record is 40 bytes per thread and libstdc++'s emergency
// exception pool (~73 KiB) is the largest allocation seen in a window. Zero
// (BSS) until written and never reused, so calloc needs no clearing.
inline constexpr size_t kArenaBytes = 256u << 10;
inline constexpr size_t kHeader = 16;   // block size, then the block (16-aligned)
alignas(4096) inline unsigned char g_arena[kArenaBytes];
inline constinit std::atomic<size_t> g_arena_used{0};
inline constinit std::atomic<uint64_t> g_arena_blocks{0};   // served so far (info)

inline bool owns(const void* p) {
    const auto a = reinterpret_cast<uintptr_t>(p);
    const auto b = reinterpret_cast<uintptr_t>(g_arena);
    return a >= b && a < b + kArenaBytes;
}

inline size_t sizeOf(const void* p) {
    return *reinterpret_cast<const size_t*>(static_cast<const unsigned char*>(p) - kHeader);
}

// nullptr once the arena is spent: the caller reports ENOMEM, which is a
// diagnosable failure where the old behaviour was a silent hang.
inline void* allocate(size_t size, size_t alignment) {
    if (alignment < kHeader) alignment = kHeader;
    if (size == 0) size = 1;
    size_t used = g_arena_used.load(std::memory_order_relaxed);
    for (;;) {
        const uintptr_t base = reinterpret_cast<uintptr_t>(g_arena);
        // Header, then the block at `alignment`.
        uintptr_t block = (base + used + kHeader + alignment - 1) & ~(alignment - 1);
        const size_t end = block - base + size;
        if (end > kArenaBytes) return nullptr;
        if (g_arena_used.compare_exchange_weak(used, end, std::memory_order_relaxed)) {
            *reinterpret_cast<size_t*>(block - kHeader) = size;
            g_arena_blocks.fetch_add(1, std::memory_order_relaxed);
            return reinterpret_cast<void*>(block);
        }
    }
}

}  // namespace smash::early
