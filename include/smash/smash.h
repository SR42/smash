// smash/smash.h - Public API for Smash compression-aware allocator
//
// Smash works as a transparent malloc replacement via alloc8 interposition.
// This header provides optional application-level hints for advanced usage.
#pragma once

#include <cstddef>
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

// Allocate `size` bytes whose physical pages are bound to NUMA node `numa`.
// The result is an ordinary heap block: free() it, realloc() it, and query it
// with malloc_usable_size() as usual.
//
// Policy: MPOL_BIND | MPOL_F_STATIC_NODES (strict). Pages come only from
// `numa`. If that node runs out of memory the kernel reclaims or OOM-kills
// instead of spilling onto another node. Pages are placed lazily on first
// touch, so node exhaustion shows up then, not as a NULL return.
//
// Blocks are rounded up to whole pages, so use this for buffers, not small
// objects. They come from a per-node arena inside smash's heap region, and
// smash compresses their cold pages. On a host with two or more memory nodes,
// each node's pages are compressed by threads pinned to that node's CPUs,
// using working memory on the node, and the compressed copies are stored on
// the node (SMASH_NUMA_COMPRESSOR=0 turns this off). A page read or written
// again is decompressed on the accessing thread and placed back on `numa`.
// If the region is exhausted, the block is a separate bound mapping that is
// not compressed. realloc() returns an unbound block; to grow while staying
// on the node, use malloc_numa + memcpy + free.
//
// Returns NULL with errno set:
//   EINVAL  `numa` is not an online node with memory in this process's cpuset
//   ENOMEM  the mapping failed
//   EPERM / ENOSYS  the kernel or a seccomp filter refused mbind
//   ENOTSUP under SMASH_PASSTHROUGH=1 or SMASH_MODE=compress_only
// Without NUMA (macOS), node 0 is a plain malloc and any other node is EINVAL.
void* malloc_numa(uint32_t numa, size_t size);

// Hint to smash that a region is about to go cold (eligible for compression).
// No-op in Phase 1.
void smash_hint_cold(void* ptr, size_t size);

// Hint to smash that a region is about to be heavily accessed (keep decompressed).
// No-op in Phase 1.
void smash_hint_hot(void* ptr, size_t size);

// Get compression statistics.
struct SmashStats {
    size_t total_allocated;
    size_t total_freed;
    size_t compressed_pages;
    size_t compressed_bytes_saved;
    size_t active_spans;
    size_t active_large_allocs;
};

void smash_get_stats(SmashStats* stats);

#ifdef __cplusplus
}
#endif
