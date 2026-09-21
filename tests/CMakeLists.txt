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

# test_esr_write_bit: the aarch64 read-vs-write fault classification. The
# signal handler's write-bit decode was x86-only (gregs[REG_ERR] bit 1), so on
# aarch64 faultWasWrite() stayed -1 for every fault and the compressor's
# soft-dirty ROI re-dirty signal counted every decompress as a write.
# aarch64EsrWriteBit() decodes ESR_EL1's WnR bit out of the esr_context record
# in the signal frame; it is a pure function over bytes, so this test feeds it
# synthetic frames and runs on any host — there is no aarch64 CI runner.
smash_add_test(test_esr_write_bit test_esr_write_bit.cpp)

smash_add_test(test_size_classes test_size_classes.cpp)
smash_add_test(test_bootstrap test_bootstrap.cpp)
smash_add_test(test_span test_span.cpp)
smash_add_test(test_slab test_slab.cpp)

# VM region test (no LZ4 dependency)
smash_add_test(test_vm_region test_vm_region.cpp)

# PageRunMap (coalescing free page runs) against a trivially correct model.
smash_add_test(test_page_run_map test_page_run_map.cpp)

# test_numa_arena: VmRegion's per-node arenas behind malloc_numa, direct-linked
# so every step is deterministic. Checks grid-aligned chunks bound to the node,
# freed pages reused only by the same node, no node pages in the shared free
# lists, and remapPages() binding a replaced node page again (the fallback that
# otherwise drops the policy). Linux only; skips when mbind is refused.
smash_add_test(test_numa_arena test_numa_arena.cpp)

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

# Idle-page read tracking: the root-only read-detection path (see the file
# header). Skips unprivileged — i.e. in CI — so run it as root to cover it.
add_executable(test_idle_read_tracking idle_read_tracking_test.cpp ${printf_SOURCE_DIR}/printf.cpp)
target_include_directories(test_idle_read_tracking PRIVATE ${SMASH_TEST_INCLUDE_DIRS})
target_link_libraries(test_idle_read_tracking PRIVATE alloc8::headers lz4_static libzstd_static)
add_test(NAME test_idle_read_tracking COMMAND test_idle_read_tracking)
set_tests_properties(test_idle_read_tracking PROPERTIES
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
  ENVIRONMENT "SMASH_RECOMPRESS_BACKOFF=0;SMASH_COLD_TIMEOUT_SEC=1;SMASH_FIXAV=0")
# Same body with eager (non-deferred) reclaim, where a COMPRESSED page stays
# readable until the deferred-madvise sweeper protects it. This mode is what
# failed 20/20 in June and got the test disabled as "signal-delivery timing";
# the test now waits for the page to be truly inaccessible in both modes.
add_test(NAME test_fault_cycle_eager_reclaim COMMAND test_fault_cycle)
set_tests_properties(test_fault_cycle_eager_reclaim PROPERTIES
  ENVIRONMENT "SMASH_RECOMPRESS_BACKOFF=0;SMASH_COLD_TIMEOUT_SEC=1;SMASH_FIXAV=0;SMASH_DEFERRED_RECLAIM=0")

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
# ── Fortified-wrapper interposition (Linux) ─────────────────────────────────
# A __*_chk wrapper only runs if it is exported at the version the caller
# binds, and that version differs per architecture (GLIBC_2.4/2.5/2.16 on
# x86-64, GLIBC_2.17 on aarch64). Taking each function's address emits the
# same versioned reference a call would, so dladdr says who won.
if(NOT APPLE)
  add_executable(chk_interposition_test chk_interposition_test.cpp)
  set_target_properties(chk_interposition_test PROPERTIES CXX_STANDARD 20)
  target_compile_options(chk_interposition_test PRIVATE -O1 -U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=2)
  target_link_libraries(chk_interposition_test PRIVATE ${CMAKE_DL_LIBS})
  add_test(NAME test_chk_interposition COMMAND ${CMAKE_COMMAND} -E env
    "LD_PRELOAD=$<TARGET_FILE:smash>" $<TARGET_FILE:chk_interposition_test>)
  set_tests_properties(test_chk_interposition PROPERTIES TIMEOUT 60)
endif()

# ── Mid-size bins (Linux) ───────────────────────────────────────────────────
# SMASH_MID_BINS=1 serves the (16 KiB, 1 MiB) band from size-binned chunks in
# the arena instead of one direct mmap each, which is what makes that band
# compressible at all. One body, two registrations: the off-run asserts the
# blocks stay out of the arena, the on-run that they enter it and compress.
if(NOT APPLE)
  add_executable(mid_bins_test mid_bins_test.cpp)
  set_target_properties(mid_bins_test PROPERTIES CXX_STANDARD 20)
  add_test(NAME test_mid_bins_off COMMAND ${CMAKE_COMMAND} -E env
    "LD_PRELOAD=$<TARGET_FILE:smash>" "SMASH_LARGE_ONLY=1"
    "SMASH_COLD_TIMEOUT_SEC=1" "SMASH_DEFER_PHASES_MS=0"
    $<TARGET_FILE:mid_bins_test>)
  add_test(NAME test_mid_bins_on COMMAND ${CMAKE_COMMAND} -E env
    "LD_PRELOAD=$<TARGET_FILE:smash>" "SMASH_LARGE_ONLY=1" "SMASH_MID_BINS=1"
    "SMASH_COLD_TIMEOUT_SEC=1" "SMASH_DEFER_PHASES_MS=0"
    $<TARGET_FILE:mid_bins_test>)
  set_tests_properties(test_mid_bins_off test_mid_bins_on PROPERTIES TIMEOUT 180)
endif()

# ── Transparent hugepages for the arena (Linux) ─────────────────────────────
# SMASH_THP=1 madvises the arena MADV_HUGEPAGE. One body, two registrations;
# skips unless the host policy is [madvise], where the two cases differ.
if(NOT APPLE)
  add_executable(thp_arena_test thp_arena_test.cpp)
  set_target_properties(thp_arena_test PROPERTIES CXX_STANDARD 20)
  add_test(NAME test_thp_arena_off COMMAND ${CMAKE_COMMAND} -E env
    "LD_PRELOAD=$<TARGET_FILE:smash>" "SMASH_LARGE_ONLY=0"
    $<TARGET_FILE:thp_arena_test>)
  add_test(NAME test_thp_arena_on COMMAND ${CMAKE_COMMAND} -E env
    "LD_PRELOAD=$<TARGET_FILE:smash>" "SMASH_LARGE_ONLY=0" "SMASH_THP=1"
    $<TARGET_FILE:thp_arena_test>)
  set_tests_properties(test_thp_arena_off test_thp_arena_on PROPERTIES TIMEOUT 120)
endif()

# ── Runtime large-alloc threshold ───────────────────────────────────────────
# SMASH_LARGE_ALLOC_VM_THRESHOLD decides which allocations reach the compressor
# at all. One body, two registrations: the expectation is derived from the env
# var, so the default run and the lowered run assert the same invariant.
add_executable(large_alloc_threshold_test large_alloc_threshold_test.cpp)
set_target_properties(large_alloc_threshold_test PROPERTIES CXX_STANDARD 20)
function(add_large_alloc_threshold_test test_name)
  if(APPLE)
    add_test(NAME ${test_name} COMMAND ${CMAKE_COMMAND} -E env
      "DYLD_INSERT_LIBRARIES=$<TARGET_FILE:smash>" "DYLD_FORCE_FLAT_NAMESPACE=1"
      "MallocNanoZone=0" "SMASH_LARGE_ONLY=1" ${ARGN}
      $<TARGET_FILE:large_alloc_threshold_test>)
  else()
    add_test(NAME ${test_name} COMMAND ${CMAKE_COMMAND} -E env
      "LD_PRELOAD=$<TARGET_FILE:smash>" "SMASH_LARGE_ONLY=1" ${ARGN}
      $<TARGET_FILE:large_alloc_threshold_test>)
  endif()
  set_tests_properties(${test_name} PROPERTIES TIMEOUT 120)
endfunction()
add_large_alloc_threshold_test(test_large_alloc_threshold_default)
add_large_alloc_threshold_test(test_large_alloc_threshold_lowered
                               "SMASH_LARGE_ALLOC_VM_THRESHOLD=262144")

# ── Buffered reads after compression (Linux) ────────────────────────────────
# glibc allocates a FILE's buffer lazily, inside the first read, so under smash
# it can land on a page the compressor already owns. The kernel returns EFAULT
# from copy_to_user, and because glibc's internal read is intra-libc the read()
# wrapper never sees it — the error reaches the application. Measured before
# the fix: fgets over /proc/self/maps returned 0 lines with ferror set and
# errno EFAULT, while a raw read(2) into a heap buffer worked.
if(NOT APPLE)
  add_executable(stdio_after_compression_test stdio_after_compression_test.cpp)
  set_target_properties(stdio_after_compression_test PROPERTIES CXX_STANDARD 20)
  add_test(
    NAME test_stdio_after_compression
    COMMAND ${CMAKE_COMMAND} -E env
      "LD_PRELOAD=$<TARGET_FILE:smash>"
      "SMASH_LARGE_ONLY=0"
      "SMASH_DEFER_PHASES_MS=0"
      "SMASH_COLD_TIMEOUT_SEC=1"
      $<TARGET_FILE:stdio_after_compression_test>
  )
  set_tests_properties(test_stdio_after_compression PROPERTIES TIMEOUT 180)
endif()

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
set_tests_properties(test_syscall_efault PROPERTIES TIMEOUT 420)

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

# malloc_numa(node, size), the exported NUMA API. Checks the MPOL_BIND |
# MPOL_F_STATIC_NODES binding (get_mempolicy) and physical placement on every
# online node with the writer pinned off-node (move_pages). Then checks that
# cold blocks are compressed out of RSS and read back byte-exact with every
# page on the node. Also checks heap-block semantics (usable size, realloc,
# free returns pages to the node arena), EINVAL for bad nodes, and that no
# binding leaks into ordinary malloc memory. Same one-body-two-modes contract
# as alloc_contract_test.
add_executable(malloc_numa_test malloc_numa_test.cpp)
set_target_properties(malloc_numa_test PROPERTIES CXX_STANDARD 20)
target_link_libraries(malloc_numa_test PRIVATE ${CMAKE_DL_LIBS})
add_smash_mode_test(test_malloc_numa_full       malloc_numa_test 0 120
                    "SMASH_COLD_TIMEOUT_SEC=1")
add_smash_mode_test(test_malloc_numa_large_only malloc_numa_test 1 120
                    "SMASH_COLD_TIMEOUT_SEC=1")

# A malloc_numa churn loop must keep reusing its node arena instead of taking
# fresh region. A freed run reaches the free map only when its queued decommit
# is processed, so a tight loop misses the map; the miss used to go straight to
# a new 64 MiB chunk, which consumed the arena and then served allocations from
# uncompressible direct mappings. SMASH_VM_GIB=1 is load-bearing: the default
# 16 GiB never reaches exhaustion and the check would pass vacuously.
add_executable(numa_arena_churn_test numa_arena_churn_test.cpp)
set_target_properties(numa_arena_churn_test PROPERTIES CXX_STANDARD 20)
target_link_libraries(numa_arena_churn_test PRIVATE ${CMAKE_DL_LIBS} Threads::Threads)
add_smash_mode_test(test_numa_arena_churn_full       numa_arena_churn_test 0 180
                    "SMASH_VM_GIB=1")
add_smash_mode_test(test_numa_arena_churn_large_only numa_arena_churn_test 1 180
                    "SMASH_VM_GIB=1")

# malloc_numa_aligned(node, size, align): malloc_numa plus an alignment
# guarantee. Sweeps alignments from one page to 16 MiB against sizes across the
# routing boundaries on every online node, and re-checks the whole malloc_numa
# contract on the aligned blocks (binding, placement from off-node CPUs, cold
# compression and restore). The reuse check is the one that fails if the
# over-allocated head and tail are leaked instead of returned to the node's own
# free runs.
add_executable(malloc_numa_aligned_test malloc_numa_aligned_test.cpp)
set_target_properties(malloc_numa_aligned_test PROPERTIES CXX_STANDARD 20)
target_link_libraries(malloc_numa_aligned_test PRIVATE ${CMAKE_DL_LIBS})
# SMASH_VM_GIB=1 is load-bearing, not incidental: it is what makes the leak
# check bite. A leaked head/tail costs `align` per round trip, so 200 round
# trips at 16 MiB alignment need 3.2 GiB and fall out of a 1 GiB arena; in the
# default 16 GiB arena the same leak would pass unnoticed.
add_smash_mode_test(test_malloc_numa_aligned_full       malloc_numa_aligned_test 0 180
                    "SMASH_COLD_TIMEOUT_SEC=1;SMASH_VM_GIB=4")
add_smash_mode_test(test_malloc_numa_aligned_large_only malloc_numa_aligned_test 1 180
                    "SMASH_COLD_TIMEOUT_SEC=1;SMASH_VM_GIB=4")

# NUMA compressor groups: on a host with two or more memory nodes, each node's
# malloc_numa pages are compressed by threads pinned to that node, with working
# memory and compressed blobs on the node. Checks thread affinity, on-node
# compression counts, the placement of worker memory and of every blob, and an
# on-node byte-exact restore, via the smash_numa_compressor_info and
# smash_compressed_blob_for diagnostic exports. Skips on single-node hosts
# (CI runners).
add_executable(numa_compressor_test numa_compressor_test.cpp)
set_target_properties(numa_compressor_test PROPERTIES CXX_STANDARD 20)
target_link_libraries(numa_compressor_test PRIVATE ${CMAKE_DL_LIBS})
add_smash_mode_test(test_numa_compressor_full       numa_compressor_test 0 120
                    "SMASH_COLD_TIMEOUT_SEC=1")
add_smash_mode_test(test_numa_compressor_large_only numa_compressor_test 1 120
                    "SMASH_COLD_TIMEOUT_SEC=1")

# Thread-cache reclaim: a ~92 KiB ThreadCache is created lazily on a thread's
# first allocation and must be returned when the thread exits. Registered three
# ways so each reclaim mechanism is covered on its own: full mode (alloc8's
# interposer, reachable only because pthread_create is now exported at
# GLIBC_2.34), full mode with the thread hooks disabled (smash's pthread_key_t
# destructor alone), and large-only (no cache is ever created).
add_executable(thread_cache_reclaim_test thread_cache_reclaim_test.cpp)
set_target_properties(thread_cache_reclaim_test PROPERTIES CXX_STANDARD 20)
target_link_libraries(thread_cache_reclaim_test PRIVATE ${CMAKE_DL_LIBS} pthread)
# Double-free hardening: Span::deallocate tests the bitmap bit before setting
# it (1 = free), so an already-free slot is a double free. Registered in full
# mode only — under SMASH_LARGE_ONLY=1 a 3-byte allocation passes through to
# glibc, which aborts on the double free before the consistency probe runs.
# The test forks the offending child so an abort policy cannot take the test
# process down, and asserts detection fires, the slab stays consistent, and
# SMASH_DOUBLE_FREE=off restores the historical silent behaviour.
add_executable(double_free_test double_free_test.cpp)
set_target_properties(double_free_test PROPERTIES CXX_STANDARD 20)
target_link_libraries(double_free_test PRIVATE ${CMAKE_DL_LIBS})
add_smash_mode_test(test_double_free_full double_free_test 0 60 "SMASH_DOUBLE_FREE=warn")

# Malloc override + routing, ported from mimalloc's test/main-override.c. The
# original asserts every pointer lands in mimalloc's own heap, proving the
# malloc override took effect; this uses smash_arena_for() for the same
# purpose, and additionally asserts the opposite truth per mode — small
# allocations are smash-owned in full mode and passed through in large-only,
# while a 2 MiB block is owned in both. Fast and deterministic (no compressor).
add_executable(malloc_override_test malloc_override_test.cpp)
set_target_properties(malloc_override_test PROPERTIES CXX_STANDARD 20)
target_link_libraries(malloc_override_test PRIVATE ${CMAKE_DL_LIBS})
add_smash_mode_test(test_malloc_override_full       malloc_override_test 0 60)
add_smash_mode_test(test_malloc_override_large_only malloc_override_test 1 60)

# Shutdown race: stop() joins the NUMA node workers at process exit, and a
# worker that reached its park loop after the stop signal slept forever on a
# futex nothing would wake — hanging the join under __cxa_finalize. Needs
# startup and teardown to nearly coincide, so the test spawns many very
# short-lived preloaded children (2.3% of them hung before the fix).
add_executable(shutdown_race_test shutdown_race_test.cpp)
set_target_properties(shutdown_race_test PROPERTIES CXX_STANDARD 20)
target_link_libraries(shutdown_race_test PRIVATE ${CMAKE_DL_LIBS} pthread)
add_smash_mode_test(test_shutdown_race_full       shutdown_race_test 0 180)
add_smash_mode_test(test_shutdown_race_large_only shutdown_race_test 1 180)

# Process exit after main retires with pthread_exit(): smash's background
# threads keep glibc's thread count above zero, so the process hung forever
# until the exit guard stopped them when the last application thread went away.
# Both modes hang without the fix, so both are registered.
add_executable(main_pthread_exit_test main_pthread_exit_test.cpp)
set_target_properties(main_pthread_exit_test PROPERTIES CXX_STANDARD 20)
target_link_libraries(main_pthread_exit_test PRIVATE ${CMAKE_DL_LIBS} pthread)
add_smash_mode_test(test_main_pthread_exit_full       main_pthread_exit_test 0 120)
add_smash_mode_test(test_main_pthread_exit_large_only main_pthread_exit_test 1 120)

add_smash_mode_test(test_thread_cache_reclaim_full       thread_cache_reclaim_test 0 180)
add_smash_mode_test(test_thread_cache_reclaim_tls_only   thread_cache_reclaim_test 0 180
                    "SMASH_NO_THREAD_HOOKS=1")
add_smash_mode_test(test_thread_cache_reclaim_large_only thread_cache_reclaim_test 1 180)

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
# finding from a WARN to a failure. Before the mmap interposer skipped
# MAP_HUGETLB, smash retried the rejected page operations on every compressor
# tick forever (8192 failing page operations in 6 s for a 16 MiB mapping) and
# the slices permanently held VmRegion's 128 Ki-page external budget while
# being permanently uncompressible. This is the regression guard for that skip.
add_smash_mode_test(test_hugepage_no_mprotect_storm hugepage_mmap_test 0 120
  "SMASH_TRACK_EXTERNAL=1"
  "SMASH_COLD_TIMEOUT_SEC=1"
  "SMASH_TRACE_MPROTECT_FAIL=1"
  "SMASH_TEST_STRICT_HUGEPAGE=1")

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

# Full mode used to reproduce the corruption deterministically (228 of 513
# bytes flushed as NUL). Fixed: drainAllForShutdown() no longer overwrites a
# COMPRESSED_SHADOW page — which is still mapped and holds the application's
# live bytes — with its stale blob. Enabled, and it is the regression guard
# for that: with the drain's shadow branch removed it fails again.
add_smash_mode_test(test_stdout_buffer_integrity_full
  stdout_buffer_integrity_test 0 180 "SMASH_COLD_TIMEOUT_SEC=1")

# smash's threads must start in an application with a lot of static TLS.
# glibc puts static TLS inside each thread's stack; smash's fixed 128 KiB
# stacks made every pthread_create fail once an app carried ~128 KiB of it,
# silently disabling compression ("pthread_create(coordinator) failed").
# The test checks that a cold, compressible block actually leaves RSS.
add_executable(static_tls_threads_test static_tls_threads_test.cpp)
set_target_properties(static_tls_threads_test PROPERTIES CXX_STANDARD 20)
add_smash_mode_test(test_static_tls_threads_full static_tls_threads_test 0 60
  "SMASH_COLD_TIMEOUT_SEC=1")
add_smash_mode_test(test_static_tls_threads_large_only static_tls_threads_test 1 60
  "SMASH_COLD_TIMEOUT_SEC=1")

# userfaultfd write tracking: where soft-dirty does not exist (arm64), smash
# tracks writes with async uffd write-protect + PAGEMAP_SCAN. Forced to uffd
# here so it runs on any Linux 6.7+ host; skips on older kernels.
add_executable(test_uffd_write_tracking uffd_write_tracking_test.cpp ${printf_SOURCE_DIR}/printf.cpp)
target_include_directories(test_uffd_write_tracking PRIVATE ${SMASH_TEST_INCLUDE_DIRS})
target_link_libraries(test_uffd_write_tracking PRIVATE alloc8::headers lz4_static libzstd_static)
add_test(NAME test_uffd_write_tracking COMMAND test_uffd_write_tracking)
set_tests_properties(test_uffd_write_tracking PROPERTIES
  ENVIRONMENT "SMASH_WRITE_TRACK=uffd;SMASH_SOFTDIRTY=1;SMASH_COLD_TICKS=2;SMASH_FIXAV=0")

# Small allocations after the VmRegion arena is exhausted: slab spans used to
# have no fallback, so operator new threw std::bad_alloc (multithreaded apps
# died with "terminate called recursively"). A 1 GiB arena and three phases of
# different size classes exhaust it; every object's pattern is verified.
add_executable(small_arena_exhaustion_test small_arena_exhaustion_test.cpp)
set_target_properties(small_arena_exhaustion_test PROPERTIES CXX_STANDARD 20)
add_smash_mode_test(test_small_arena_exhaustion small_arena_exhaustion_test 0 300
  "SMASH_VM_GIB=1")

# Freed memory must be reused by the compressible arena across size classes
# and sizes: empty slab spans returned, freed page runs coalesced. 1 GiB arena,
# phases of ~70% each; every phase must land >= 95% inside the arena.
add_executable(arena_reuse_test arena_reuse_test.cpp)
set_target_properties(arena_reuse_test PROPERTIES CXX_STANDARD 20)
target_link_libraries(arena_reuse_test PRIVATE ${CMAKE_DL_LIBS})
add_smash_mode_test(test_arena_reuse arena_reuse_test 0 600 "SMASH_VM_GIB=1")

# Concurrent LargeAlloc traffic must not corrupt the span-descriptor free list
# (was a lock-free Treiber stack with an ABA race: aborts / segfaults under
# many threads). Both modes: aligned objects in full mode, 32 KiB blocks in
# large-only. Probabilistic: the old stack crashed about 2 runs in 3 here.
add_executable(span_descriptor_race_test span_descriptor_race_test.cpp)
set_target_properties(span_descriptor_race_test PROPERTIES CXX_STANDARD 20)
target_link_libraries(span_descriptor_race_test PRIVATE Threads::Threads)
add_smash_mode_test(test_span_descriptor_race_full span_descriptor_race_test 0 300)
add_smash_mode_test(test_span_descriptor_race_large_only span_descriptor_race_test 1 300)
