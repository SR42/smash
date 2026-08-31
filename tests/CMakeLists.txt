# smash/tests/CMakeLists.txt
# Each test is a separate executable that tests internal components directly.

# Build tests as arm64-only on macOS. The fat arm64+arm64e binary (needed for
# libsmash.dylib to interpose system arm64e processes) causes SIGILL in the
# arm64e slice for tests that exercise the fault handler — the thread_local
# lazy-init guard traps when first run inside a signal handler under pointer
# authentication. Tests don't need arm64e; they test allocator correctness.
if(APPLE)
  set(CMAKE_OSX_ARCHITECTURES "arm64")
endif()

set(SMASH_TEST_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
  ${CMAKE_CURRENT_SOURCE_DIR}/../src
  ${printf_SOURCE_DIR}
)

# Helper to add a test executable. Tests transitively pull in headers that
# depend on the mpaland/printf TU (via util/safe_printf.h), so every test
# must link printf.cpp.
function(smash_add_test name source)
  add_executable(${name} ${source} ${printf_SOURCE_DIR}/printf.cpp)
  target_include_directories(${name} PRIVATE ${SMASH_TEST_INCLUDE_DIRS})
  target_link_libraries(${name} PRIVATE alloc8::headers)
  if(APPLE)
    target_compile_options(${name} PRIVATE -ftls-model=initial-exec)
  endif()
  add_test(NAME ${name} COMMAND ${name})
endfunction()

# ── aarch64 / page-geometry regression guards ────────────────────────────────
# An application running under libsmash on aarch64 was throwing std::bad_alloc.
# The only way smash produces that exception is malloc returning nullptr (the
# operator-new wrappers in src/linux_syscall_wrappers.cpp turn nullptr into
# `throw std::bad_alloc()`), so these tests attack the arch-dependent geometry
# that makes malloc fail. See tests/alloc_contract_test.cpp for the full chain.
#
# test_page_size: constants. alloc8 hard-codes ALLOC8_PAGE_SIZE=4096 for every
# Linux target, but aarch64 kernels commonly use 16 KiB or 64 KiB base pages
# (RHEL/Rocky/Oracle aarch64 ship 64 KiB). It asserts kPageSize == the running
# kernel's page size, that kPageShift actually describes kPageSize (config.h
# derives it with a ternary that only knows 4 KiB and 16 KiB), that this
# machine's addresses fit the hard-coded kAddressBits=48 PageMap, and that
# every size class still has a valid span geometry at this page size.
smash_add_test(test_page_size test_page_size.cpp)

# test_vm_granularity: the same mismatch at the syscall level. Issues the exact
# reserve/commit/protect/decommit sequence VmRegion and the compressor depend
# on, at kPageSize offsets. Under a page-size mismatch each call is rejected
# with EINVAL; VmRegion::ensureCommitted ignores commitPages()' return value
# and advances its watermark anyway, so callers get PROT_NONE memory back.
smash_add_test(test_vm_granularity test_vm_granularity.cpp)

smash_add_test(test_size_classes test_size_classes.cpp)
smash_add_test(test_bootstrap test_bootstrap.cpp)
smash_add_test(test_span test_span.cpp)
smash_add_test(test_slab test_slab.cpp)

# VM region test (no LZ4 dependency)
smash_add_test(test_vm_region test_vm_region.cpp)

# SpillStore test (file-backed very-cold blob storage; no codec dependency)
smash_add_test(test_spill_store test_spill_store.cpp)

# Integration test includes smash_heap.cpp for thread cache methods (needs LZ4 + zstd)
add_executable(test_integration
  test_integration.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/smash_heap.cpp
  ${printf_SOURCE_DIR}/printf.cpp
)
target_include_directories(test_integration PRIVATE ${SMASH_TEST_INCLUDE_DIRS})
target_link_libraries(test_integration PRIVATE alloc8::headers lz4_static libzstd_static ${CMAKE_DL_LIBS})
if(APPLE)
  target_compile_options(test_integration PRIVATE -ftls-model=initial-exec)
endif()
add_test(NAME test_integration COMMAND test_integration)

# Compression test (needs LZ4 + zstd)
add_executable(test_compression test_compression.cpp ${printf_SOURCE_DIR}/printf.cpp)
target_include_directories(test_compression PRIVATE ${SMASH_TEST_INCLUDE_DIRS})
target_link_libraries(test_compression PRIVATE alloc8::headers lz4_static libzstd_static)
if(APPLE)
  target_compile_options(test_compression PRIVATE -ftls-model=initial-exec)
endif()
add_test(NAME test_compression COMMAND test_compression)
# Drive the compressor through ticks deterministically — bypass the
# CPU-pressure hard-skip and floor multiplier with an explicit
# cold-ticks override. Also pin SMASH_SOFTDIRTY=0 because this test
# directly checks Phase-3 PROT_READ states (ACTIVE_MONITORING) which
# don't exist under the soft-dirty path.
set_tests_properties(test_compression PROPERTIES
  ENVIRONMENT "SMASH_COLD_TICKS=2;SMASH_SOFTDIRTY=0;SMASH_FIXAV=0")

# Compression-ratio test (needs LZ4 + zstd): verifies the engine hits the
# paper's claimed ratios (RQ2), not just "compressed > 0". In-process,
# direct-linked, no compressor thread — a deterministic guard on the codec.
add_executable(compression_ratio_test compression_ratio_test.cpp ${printf_SOURCE_DIR}/printf.cpp)
target_include_directories(compression_ratio_test PRIVATE ${SMASH_TEST_INCLUDE_DIRS})
target_link_libraries(compression_ratio_test PRIVATE alloc8::headers lz4_static libzstd_static)
if(APPLE)
  target_compile_options(compression_ratio_test PRIVATE -ftls-model=initial-exec)
endif()
add_test(NAME test_compression_ratio COMMAND compression_ratio_test)

# Dictionary training test (needs LZ4 + zstd)
add_executable(test_dictionary test_dictionary.cpp ${printf_SOURCE_DIR}/printf.cpp)
target_include_directories(test_dictionary PRIVATE ${SMASH_TEST_INCLUDE_DIRS})
target_link_libraries(test_dictionary PRIVATE alloc8::headers lz4_static libzstd_static)
if(APPLE)
  target_compile_options(test_dictionary PRIVATE -ftls-model=initial-exec)
endif()
add_test(NAME test_dictionary COMMAND test_dictionary)

# Profile + dictionary persistence test (needs LZ4 + zstd). Validates the
# v4 SMASH_PROFILE_FILE format roundtrip — train, save, load, verify dict
# is usable — plus corrupt-file rejection.
add_executable(test_profile_dict_persistence test_profile_dict_persistence.cpp ${printf_SOURCE_DIR}/printf.cpp)
target_include_directories(test_profile_dict_persistence PRIVATE ${SMASH_TEST_INCLUDE_DIRS})
target_link_libraries(test_profile_dict_persistence PRIVATE alloc8::headers lz4_static libzstd_static)
if(APPLE)
  target_compile_options(test_profile_dict_persistence PRIVATE -ftls-model=initial-exec)
endif()
add_test(NAME test_profile_dict_persistence COMMAND test_profile_dict_persistence)

# Prefetch test (needs LZ4 + zstd)
add_executable(test_prefetch test_prefetch.cpp ${printf_SOURCE_DIR}/printf.cpp)
target_include_directories(test_prefetch PRIVATE ${SMASH_TEST_INCLUDE_DIRS})
target_link_libraries(test_prefetch PRIVATE alloc8::headers lz4_static libzstd_static)
if(APPLE)
  target_compile_options(test_prefetch PRIVATE -ftls-model=initial-exec)
endif()
add_test(NAME test_prefetch COMMAND test_prefetch)
# Pin SMASH_SOFTDIRTY=0 — test directly drives PROT_READ-based Phase 3.
set_tests_properties(test_prefetch PROPERTIES
  ENVIRONMENT "SMASH_COLD_TICKS=2;SMASH_SOFTDIRTY=0;SMASH_FIXAV=0")

# Contention test (needs LZ4 + zstd + smash_heap.cpp)
add_executable(test_contention
  test_contention.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/smash_heap.cpp
  ${printf_SOURCE_DIR}/printf.cpp
)
target_include_directories(test_contention PRIVATE ${SMASH_TEST_INCLUDE_DIRS})
target_link_libraries(test_contention PRIVATE alloc8::headers lz4_static libzstd_static ${CMAKE_DL_LIBS})
if(APPLE)
  target_compile_options(test_contention PRIVATE -ftls-model=initial-exec)
endif()
add_test(NAME test_contention COMMAND test_contention)

# Fault cycle test - real SIGSEGV handling (needs LZ4 + zstd)
add_executable(test_fault_cycle test_fault_cycle.cpp ${printf_SOURCE_DIR}/printf.cpp)
target_include_directories(test_fault_cycle PRIVATE ${SMASH_TEST_INCLUDE_DIRS})
target_link_libraries(test_fault_cycle PRIVATE alloc8::headers lz4_static libzstd_static)
if(APPLE)
  target_compile_options(test_fault_cycle PRIVATE -ftls-model=initial-exec)
endif()
add_test(NAME test_fault_cycle COMMAND test_fault_cycle)
# This test exercises the same compress→fault→recompress pattern that the
# recompression-thrash back-off is designed to suppress, and runs on busy CI
# runners where the cold-tick floor multiplier would otherwise keep pages
# from being compressed within the test's tick budget. Disable both for this
# test so it can verify the underlying fault-cycle correctness.
# SMASH_COLD_TIMEOUT_SEC=1 sets cold_ticks_overridden, which bypasses the
# CPU-pressure floor multiplier (see compressor_thread.h::phase2Range).
set_tests_properties(test_fault_cycle PROPERTIES
  ENVIRONMENT "SMASH_RECOMPRESS_BACKOFF=0;SMASH_COLD_TIMEOUT_SEC=1;SMASH_FIXAV=0"
  LABELS "known_issue"
  DISABLED TRUE)  # Signal-delivery timing varies across CI environments

# Interposition test: built without alloc8 linkage, run under DYLD_INSERT
add_executable(interpose_test interpose_test.cpp)
set_target_properties(interpose_test PROPERTIES
  CXX_STANDARD 20
)
add_test(
  NAME test_interpose
  COMMAND ${CMAKE_COMMAND} -E env
    "DYLD_INSERT_LIBRARIES=$<TARGET_FILE:smash>"
    "DYLD_FORCE_FLAT_NAMESPACE=1"
    $<TARGET_FILE:interpose_test>
)

# External-mapping test: exercises the mmap / Mach VM interposers under
# a live compressor. Same DYLD/LD_PRELOAD pattern as test_interpose;
# requires SMASH_LARGE_ONLY=0 + a short SMASH_COLD_TIMEOUT_SEC so the
# compressor processes pages within the test's 3 s sleep.
add_executable(external_mapping_test external_mapping_test.cpp)
set_target_properties(external_mapping_test PROPERTIES
  CXX_STANDARD 20
)
if(APPLE)
  add_test(
    NAME test_external_mapping
    COMMAND ${CMAKE_COMMAND} -E env
      "DYLD_INSERT_LIBRARIES=$<TARGET_FILE:smash>"
      "DYLD_FORCE_FLAT_NAMESPACE=1"
      "MallocNanoZone=0"
      "SMASH_LARGE_ONLY=0"
      "SMASH_DEFER_PHASES_MS=0"
      "SMASH_COLD_TIMEOUT_SEC=1"
      "SMASH_TRACK_EXTERNAL=1"
      $<TARGET_FILE:external_mapping_test>
  )
else()
  add_test(
    NAME test_external_mapping
    COMMAND ${CMAKE_COMMAND} -E env
      "LD_PRELOAD=$<TARGET_FILE:smash>"
      "SMASH_LARGE_ONLY=0"
      "SMASH_DEFER_PHASES_MS=0"
      "SMASH_COLD_TIMEOUT_SEC=1"
      "SMASH_TRACK_EXTERNAL=1"
      $<TARGET_FILE:external_mapping_test>
  )
endif()
set_tests_properties(test_external_mapping PROPERTIES
  TIMEOUT 60)
# Re-enabled: the OOB-on-external-pages crash class was root-caused and fixed
# in issue #84 (self-interposition registered smash's own direct-mmap fallback
# pages as external; free-path routing then poisoned the arena free list).

# Mode compression test: ONE executable (mode_compression_test.cpp), run twice
# with SMASH_LARGE_ONLY={0,1}, so FULL mode (test_malloc_compression) and
# LARGE-ONLY mode (test_large_only_compression) execute byte-identical
# assertions and cannot drift apart in coverage. The test allocates large
# (>= 1 MiB, smash-managed -> compressible VmRegion in BOTH modes) and small
# (<= 16 KiB, slab in full / system-passthrough in large-only) compressible
# chunks, asserts compressed>0 via SIGUSR2 stats, then verifies a byte-exact
# read-back of BOTH classes. The mode is read from the env at runtime, never a
# compile flag — that's what guarantees the assertions are identical.
add_executable(mode_compression_test mode_compression_test.cpp)
set_target_properties(mode_compression_test PROPERTIES CXX_STANDARD 20)

# add_mode_compression_test(<ctest-name> <SMASH_LARGE_ONLY value>)
function(add_mode_compression_test test_name large_only)
  if(APPLE)
    add_test(
      NAME ${test_name}
      COMMAND ${CMAKE_COMMAND} -E env
        "DYLD_INSERT_LIBRARIES=$<TARGET_FILE:smash>"
        "DYLD_FORCE_FLAT_NAMESPACE=1"
        "MallocNanoZone=0"
        "SMASH_LARGE_ONLY=${large_only}"
        "SMASH_DEFER_PHASES_MS=0"
        "SMASH_COLD_TIMEOUT_SEC=1"
        "SMASH_STATS_SIGNAL=1"
        $<TARGET_FILE:mode_compression_test>
    )
  else()
    add_test(
      NAME ${test_name}
      COMMAND ${CMAKE_COMMAND} -E env
        "LD_PRELOAD=$<TARGET_FILE:smash>"
        "SMASH_LARGE_ONLY=${large_only}"
        "SMASH_DEFER_PHASES_MS=0"
        "SMASH_COLD_TIMEOUT_SEC=1"
        "SMASH_STATS_SIGNAL=1"
        $<TARGET_FILE:mode_compression_test>
    )
  endif()
  set_tests_properties(${test_name} PROPERTIES TIMEOUT 60)
endfunction()

add_mode_compression_test(test_malloc_compression 0)      # full mode
add_mode_compression_test(test_large_only_compression 1)  # large-only mode

# Arena-routing regression guard: runs under interposition and asserts that
# allocations from many distinct call sites do NOT all collapse into one arena
# (the LTO/wrapper return-address bug). Full mode so 256 B allocations take the
# slab path and receive an arena; CPU/thread hashing pinned off so the arena is
# a pure function of the return address. Fast + deterministic (no compressor).
add_executable(arena_routing_test arena_routing_test.cpp)
set_target_properties(arena_routing_test PROPERTIES CXX_STANDARD 20)
target_link_libraries(arena_routing_test PRIVATE ${CMAKE_DL_LIBS})
if(APPLE)
  add_test(
    NAME test_arena_routing
    COMMAND ${CMAKE_COMMAND} -E env
      "DYLD_INSERT_LIBRARIES=$<TARGET_FILE:smash>"
      "DYLD_FORCE_FLAT_NAMESPACE=1"
      "MallocNanoZone=0"
      "SMASH_LARGE_ONLY=0"
      "SMASH_CPU_ARENA=0"
      $<TARGET_FILE:arena_routing_test>
  )
else()
  add_test(
    NAME test_arena_routing
    COMMAND ${CMAKE_COMMAND} -E env
      "LD_PRELOAD=$<TARGET_FILE:smash>"
      "SMASH_LARGE_ONLY=0"
      "SMASH_CPU_ARENA=0"
      $<TARGET_FILE:arena_routing_test>
  )
endif()
set_tests_properties(test_arena_routing PROPERTIES TIMEOUT 30)

# Syscall EFAULT-retry test: allocates a buffer, forces compression, then
# issues real syscalls (read/writev/readv) directly on the compressed
# buffer. Verifies that the EFAULT-driven retry path in syscall_compat.h
# actually catches kernel-level buffer protection faults and recovers.
# Without retryWithDecompress, the kernel returns -1/EFAULT and the test
# fails. Also exercises the iovec walk path in the rewarm callback.
add_executable(syscall_efault_test syscall_efault_test.cpp)
set_target_properties(syscall_efault_test PROPERTIES
  CXX_STANDARD 20
)
if(APPLE)
  add_test(
    NAME test_syscall_efault
    COMMAND ${CMAKE_COMMAND} -E env
      "DYLD_INSERT_LIBRARIES=$<TARGET_FILE:smash>"
      "DYLD_FORCE_FLAT_NAMESPACE=1"
      "MallocNanoZone=0"
      "SMASH_LARGE_ONLY=0"
      "SMASH_DEFER_PHASES_MS=0"
      "SMASH_COLD_TIMEOUT_SEC=1"
      "SMASH_STATS_SIGNAL=1"
      $<TARGET_FILE:syscall_efault_test>
  )
else()
  add_test(
    NAME test_syscall_efault
    COMMAND ${CMAKE_COMMAND} -E env
      "LD_PRELOAD=$<TARGET_FILE:smash>"
      "SMASH_LARGE_ONLY=0"
      "SMASH_DEFER_PHASES_MS=0"
      "SMASH_COLD_TIMEOUT_SEC=1"
      "SMASH_STATS_SIGNAL=1"
      # SMASH_SOFTDIRTY=0: this test relies on the deterministic
      # PROT_READ Phase-3 timing to drive the buffer to COMPRESSED
      # before the syscall. Soft-dirty's pagemap-read sees the
      # initial-fault writes for one extra tick, missing the test's
      # 3 s window.
      "SMASH_SOFTDIRTY=0"
      $<TARGET_FILE:syscall_efault_test>
  )
endif()
set_tests_properties(test_syscall_efault PROPERTIES TIMEOUT 180)

# ── Allocation-contract tests (run under interposition, both modes) ──────────
# Registered through a shared helper because each one, like
# add_mode_compression_test above, is ONE executable run twice with only
# SMASH_LARGE_ONLY differing — the mode is read from the environment at
# runtime, never a compile flag, so full mode and the production-supported
# large-only mode provably execute identical assertions.
#
# add_smash_mode_test(<ctest-name> <target> <SMASH_LARGE_ONLY> <timeout> [extra env...])
function(add_smash_mode_test test_name target large_only timeout)
  set(_env
    "SMASH_LARGE_ONLY=${large_only}"
    "SMASH_DEFER_PHASES_MS=0"
    ${ARGN})
  if(APPLE)
    add_test(
      NAME ${test_name}
      COMMAND ${CMAKE_COMMAND} -E env
        "DYLD_INSERT_LIBRARIES=$<TARGET_FILE:smash>"
        "DYLD_FORCE_FLAT_NAMESPACE=1"
        "MallocNanoZone=0"
        ${_env}
        $<TARGET_FILE:${target}>
    )
  else()
    add_test(
      NAME ${test_name}
      COMMAND ${CMAKE_COMMAND} -E env
        "LD_PRELOAD=$<TARGET_FILE:smash>"
        ${_env}
        $<TARGET_FILE:${target}>
    )
  endif()
  set_tests_properties(${test_name} PROPERTIES TIMEOUT ${timeout})
endfunction()

# Allocation contract: a satisfiable request must never yield nullptr (which
# operator new turns into std::bad_alloc). Sweeps every routing boundary —
# size-class edges, kMaxSmallSize (16 KiB), kLargeAllocVmThreshold (1 MiB),
# page multiples for 4/16/64 KiB pages, and alignments from 16 B to 64 KiB —
# across malloc, operator new/new[]/nothrow/align_val_t, posix_memalign,
# aligned_alloc, calloc, realloc chains, and sustained mixed churn. This is
# the direct regression test for the reported aarch64 std::bad_alloc.
add_executable(alloc_contract_test alloc_contract_test.cpp)
set_target_properties(alloc_contract_test PROPERTIES CXX_STANDARD 20)
target_link_libraries(alloc_contract_test PRIVATE ${CMAKE_DL_LIBS})
add_smash_mode_test(test_alloc_contract_full       alloc_contract_test 0 300)
add_smash_mode_test(test_alloc_contract_large_only alloc_contract_test 1 300)

# Over-aligned footprint: SmashHeap::memalign() sends every alignment > 16 B to
# LargeAlloc, which rounds up to a whole page — so an `alignas(64)` 64-byte
# object costs a full kernel page (64x on 4 KiB pages, 256x on macOS arm64's
# 16 KiB, 1024x on a 64 KiB-page aarch64 kernel). Two-tier like
# test_compression_ratio: a hard RSS budget that a 64 KiB page size blows
# through, plus a WARN documenting the amplification on smaller pages.
# SMASH_DEFER_PHASES_MS keeps the compressor out of the measurement window so
# the test reads allocation footprint, not compression.
add_executable(aligned_alloc_footprint_test aligned_alloc_footprint_test.cpp)
set_target_properties(aligned_alloc_footprint_test PROPERTIES CXX_STANDARD 20)
target_link_libraries(aligned_alloc_footprint_test PRIVATE ${CMAKE_DL_LIBS})
if(APPLE)
  add_test(
    NAME test_aligned_alloc_footprint_full
    COMMAND ${CMAKE_COMMAND} -E env
      "DYLD_INSERT_LIBRARIES=$<TARGET_FILE:smash>"
      "DYLD_FORCE_FLAT_NAMESPACE=1"
      "MallocNanoZone=0"
      "SMASH_LARGE_ONLY=0"
      "SMASH_DEFER_PHASES_MS=60000"
      $<TARGET_FILE:aligned_alloc_footprint_test>
  )
  add_test(
    NAME test_aligned_alloc_footprint_large_only
    COMMAND ${CMAKE_COMMAND} -E env
      "DYLD_INSERT_LIBRARIES=$<TARGET_FILE:smash>"
      "DYLD_FORCE_FLAT_NAMESPACE=1"
      "MallocNanoZone=0"
      "SMASH_LARGE_ONLY=1"
      "SMASH_DEFER_PHASES_MS=60000"
      $<TARGET_FILE:aligned_alloc_footprint_test>
  )
else()
  add_test(
    NAME test_aligned_alloc_footprint_full
    COMMAND ${CMAKE_COMMAND} -E env
      "LD_PRELOAD=$<TARGET_FILE:smash>"
      "SMASH_LARGE_ONLY=0"
      "SMASH_DEFER_PHASES_MS=60000"
      $<TARGET_FILE:aligned_alloc_footprint_test>
  )
  add_test(
    NAME test_aligned_alloc_footprint_large_only
    COMMAND ${CMAKE_COMMAND} -E env
      "LD_PRELOAD=$<TARGET_FILE:smash>"
      "SMASH_LARGE_ONLY=1"
      "SMASH_DEFER_PHASES_MS=60000"
      $<TARGET_FILE:aligned_alloc_footprint_test>
  )
endif()
set_tests_properties(test_aligned_alloc_footprint_full
                     test_aligned_alloc_footprint_large_only
                     PROPERTIES TIMEOUT 120)

# VmRegion exhaustion: with SMASH_VM_GIB=1 the contiguous arena is 1 GiB, and
# 1.5 GiB of live 4 MiB chunks overflows it, forcing LargeAlloc's direct-mmap
# fallback. config.h names the failure this guards against by name — exhausting
# the region "trigger[s] malloc=NULL -> std::bad_alloc". Also covers the mixed
# arena/fallback free routing (issue #84's class) by freeing everything and
# reallocating. Full mode only: large-only routes the same >= 1 MiB chunks
# through the identical LargeAlloc path, so a second registration would add no
# coverage for a gigabyte of reservation.
add_executable(vm_exhaustion_test vm_exhaustion_test.cpp)
set_target_properties(vm_exhaustion_test PROPERTIES CXX_STANDARD 20)
target_link_libraries(vm_exhaustion_test PRIVATE ${CMAKE_DL_LIBS})
add_smash_mode_test(test_vm_exhaustion vm_exhaustion_test 0 300 "SMASH_VM_GIB=1")

# ── Hugepage / VMA-pressure guards ──────────────────────────────────────────
# Follow-ups to the aarch64 std::bad_alloc investigation for the case where the
# application allocates hugepages with mmap. Both are Linux-only and skip
# cleanly elsewhere (and, for the hugepage test, on hosts with no hugepages
# reserved — CI runners have none).

# smash's mmap interposer filters on `anon && writable` with no MAP_HUGETLB
# carve-out, so every 4 KiB slice of an application hugepage mapping is
# registered as an external page. hugetlb VMAs require huge-page-aligned
# addresses and lengths, so every page operation smash then performs on them is
# rejected (mprotect/madvise EINVAL, MAP_FIXED mmap ENOMEM — all three verified
# on a host with hugepages reserved). The test asserts what must hold: the
# app's mmap still succeeds, its data stays byte-exact (smash's restore path is
# MAP_FIXED|MAP_ANON, which would shred a hugetlb VMA), and allocation keeps
# working while the mapping is registered.
add_executable(hugepage_mmap_test hugepage_mmap_test.cpp)
set_target_properties(hugepage_mmap_test PROPERTIES CXX_STANDARD 20)
add_smash_mode_test(test_hugepage_mmap hugepage_mmap_test 0 120
  "SMASH_TRACK_EXTERNAL=1"
  "SMASH_COLD_TIMEOUT_SEC=1"
  "SMASH_TRACE_MPROTECT_FAIL=1")

# Same binary, but SMASH_TEST_STRICT_HUGEPAGE=1 promotes the wasted-work
# finding from a WARN to a failure: smash retries the rejected page operations
# on every compressor tick forever (~8192 failing mprotect calls in a few
# seconds for a 16 MiB mapping) and the slices permanently occupy VmRegion's
# 128 Ki-page external budget while being permanently uncompressible.
# Registered DISABLED with the known_issue label — the same treatment
# test_fault_cycle gets — because it documents a real defect that is not fixed
# yet. Enable it once the mmap interposer skips MAP_HUGETLB.
add_smash_mode_test(test_hugepage_no_mprotect_storm hugepage_mmap_test 0 120
  "SMASH_TRACK_EXTERNAL=1"
  "SMASH_COLD_TIMEOUT_SEC=1"
  "SMASH_TRACE_MPROTECT_FAIL=1"
  "SMASH_TEST_STRICT_HUGEPAGE=1")
set_tests_properties(test_hugepage_no_mprotect_storm PROPERTIES
  LABELS "known_issue"
  DISABLED TRUE)  # registerLinuxExternalRange has no MAP_HUGETLB carve-out

# VMA pressure: crossing vm.max_map_count makes mmap return ENOMEM, which makes
# malloc return nullptr, which makes operator new throw std::bad_alloc. smash
# mitigates this with soft-dirty tracking, chunked mprotect and a reclaim-loop
# VMA budget; this pins them in place. Drives a 576 MiB compress-and-refault
# workload including a THP-hinted region (a partial mprotect inside a
# transparent hugepage splits it, up to 512 VMAs per 2 MiB) and asserts the VMA
# count stays far below the cap.
add_executable(vma_pressure_test vma_pressure_test.cpp)
set_target_properties(vma_pressure_test PROPERTIES CXX_STANDARD 20)
add_smash_mode_test(test_vma_pressure vma_pressure_test 0 180
  "SMASH_COLD_TIMEOUT_SEC=1")

# ── Buffered-stdio integrity under compression ──────────────────────────────
# glibc allocates stdout's 4 KiB buffer with malloc, so in full mode it is an
# ordinary smash slab object. An application writing in bursts separated by
# quiet stretches lets that page go cold; the compressor compresses it while it
# still holds unflushed bytes, and everything written afterwards is flushed as
# NUL at exit. Measured here: 228 of 513 bytes zeroed and 4 of 9 marker lines
# lost, identically on every run. Byte-perfect with no libsmash, in large-only
# mode, and in full mode with compression deferred.
#
# The check re-execs itself (not a bare fork — fork clones only the calling
# thread, so a forked child has no compressor and never compresses anything)
# with the child's stdout pointed at a temp file, then inspects the file after
# the child exits, because the damage only appears at the exit-time flush.
add_executable(stdout_buffer_integrity_test stdout_buffer_integrity_test.cpp)
set_target_properties(stdout_buffer_integrity_test PROPERTIES CXX_STANDARD 20)

# Large-only is the production-supported mode and is unaffected; this guards
# that it stays that way.
add_smash_mode_test(test_stdout_buffer_integrity_large_only
  stdout_buffer_integrity_test 1 180 "SMASH_COLD_TIMEOUT_SEC=1")

# Full mode reproduces the corruption deterministically. Registered DISABLED
# with the known_issue label — the same treatment test_fault_cycle gets —
# because it documents a real, currently-unfixed defect. Enable it once
# writes into a compressed page are no longer lost.
add_smash_mode_test(test_stdout_buffer_integrity_full
  stdout_buffer_integrity_test 0 180 "SMASH_COLD_TIMEOUT_SEC=1")
set_tests_properties(test_stdout_buffer_integrity_full PROPERTIES
  LABELS "known_issue"
  DISABLED TRUE)  # writes into a compressed slab page are flushed as NUL
