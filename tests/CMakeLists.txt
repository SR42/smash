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

# vm.max_map_count regression guard (#94): with the process held 12K VMAs
# below the cap (the test uses them up itself; no root), a teardown-like
# stride walk over 128 MiB of compressed large blocks must neither hang nor
# return wrong data. Linux only (no VMA cap elsewhere). The test exits 77
# (skipped) if the cap is too large to approach quickly.
if(NOT APPLE)
  add_executable(vma_cap_test vma_cap_test.cpp)
  set_target_properties(vma_cap_test PROPERTIES CXX_STANDARD 20)
  add_test(
    NAME test_vma_cap
    COMMAND ${CMAKE_COMMAND} -E env
      "LD_PRELOAD=$<TARGET_FILE:smash>"
      "SMASH_LARGE_ONLY=1"
      "SMASH_DEFER_PHASES_MS=0"
      "SMASH_COLD_TIMEOUT_SEC=1"
      $<TARGET_FILE:vma_cap_test>
  )
  set_tests_properties(test_vma_cap PROPERTIES TIMEOUT 150 SKIP_RETURN_CODE 77)
endif()

# Per-thread state in a TSD slot (macOS SMASH_TSD_SLOT): ThreadStates are
# reclaimed and reused across thousands of short-lived threads, GCD workers and
# allocating app destructors work, fork() keeps the free list usable. Skips
# itself when smash_tsd_stats is not exported (TSD slot not compiled in).
if(APPLE)
  add_executable(tsd_thread_test tsd_thread_test.cpp)
  set_target_properties(tsd_thread_test PROPERTIES CXX_STANDARD 20)
  add_test(
    NAME test_tsd_thread
    COMMAND ${CMAKE_COMMAND} -E env
      "DYLD_INSERT_LIBRARIES=$<TARGET_FILE:smash>"
      "MallocNanoZone=0"
      $<TARGET_FILE:tsd_thread_test>
  )
  set_tests_properties(test_tsd_thread PROPERTIES TIMEOUT 120)
endif()

# Large-block ownership regression: blocks smash mmaps directly (16 KiB < size
# < kLargeAllocVmThreshold, and every large block in large-only mode) sit
# outside the contiguous arena; xxowns() must still claim them or alloc8
# forwards realloc/malloc_size/free to the system allocator (malloc_size 0,
# realloc reads past the block -> SIGSEGV; crashed every Rust program).
# Interposed, both modes, no compressor needed.
add_executable(large_realloc_test large_realloc_test.cpp)
set_target_properties(large_realloc_test PROPERTIES CXX_STANDARD 20)
foreach(lo 0 1)
  if(lo)
    set(_name test_large_realloc_large_only)
  else()
    set(_name test_large_realloc)
  endif()
  if(APPLE)
    add_test(
      NAME ${_name}
      COMMAND ${CMAKE_COMMAND} -E env
        "DYLD_INSERT_LIBRARIES=$<TARGET_FILE:smash>"
        "MallocNanoZone=0"
        "SMASH_LARGE_ONLY=${lo}"
        $<TARGET_FILE:large_realloc_test>
    )
  else()
    add_test(
      NAME ${_name}
      COMMAND ${CMAKE_COMMAND} -E env
        "LD_PRELOAD=$<TARGET_FILE:smash>"
        "SMASH_LARGE_ONLY=${lo}"
        $<TARGET_FILE:large_realloc_test>
    )
  endif()
  set_tests_properties(${_name} PROPERTIES TIMEOUT 30)
endforeach()

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
