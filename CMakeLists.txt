cmake_minimum_required(VERSION 3.15)
project(smash
  VERSION 0.1.0
  LANGUAGES C CXX
  DESCRIPTION "Compression-aware memory allocator"
)

# ── Default build type ──────────────────────────────────────────────────────
# Default to RelWithDebInfo (optimized + debug info) when the user hasn't picked
# a build type. Skipped for multi-config generators (Xcode, VS), which select
# the config at build time rather than configure time.
get_property(_smash_multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(NOT _smash_multi_config AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING
      "Choose the build type." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
      Debug Release RelWithDebInfo MinSizeRel)
endif()

# Generate build/compile_commands.json so clangd (see .clangd) and other
# language servers can resolve include paths, defines, and per-file flags
# without needing the user to remember a CMake option.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Allow FetchContent_Populate for older CMake compatibility
if(POLICY CMP0169)
  cmake_policy(SET CMP0169 OLD)
endif()

# Timestamp URL-downloaded archives at extraction time rather than preserving
# the timestamps stored in the tarball.  Without this, bumping a dependency URL
# can leave the extracted sources looking older than their build outputs, so
# nothing rebuilds.  Applies to every FetchContent/ExternalProject URL download
# in the tree (LZ4, zstd, gperftools, the bench deps, ...).
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

# ── Default build type ────────────────────────────────────────────────────────
# smash is a performance-critical allocator, so a plain `cmake ..` must never
# produce an unoptimized library.  RelWithDebInfo keeps the symbols/line tables
# the allocator needs for profiling and for readable crash reports out of the
# fault handler; the -O3 promotion just below makes it as fast as Release, which
# would otherwise be the only way to get -O3 (at the cost of -g).
# We set the default here rather than letting alloc8 do it (it FORCEs Release
# into the cache when unset, which is where the "Setting build type to
# 'Release'" message came from): our optimization level should not depend on a
# fetched dependency's defaults, and anything between here and
# add_subdirectory(alloc8) should see the final value.  Multi-config generators
# (Xcode, Visual Studio) choose the config at build time, so they are left alone.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
               "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
endif()

# ── -O3 for every optimized config ────────────────────────────────────────────
# CMake hardcodes -O2 for RelWithDebInfo (and for Release on some toolchains).
# Peak optimization matters more here than compile time, so promote -O2 to -O3
# in the optimized configs.  These are set as *normal* variables, which shadow
# the cache in this directory and every subdirectory — so alloc8, LZ4, zstd,
# mimalloc and the rest of the FetchContent/add_subdirectory tree get -O3 too,
# not just smash's own targets.  Substitution (rather than assignment) preserves
# whatever else the toolchain put in the flags, notably -g and -DNDEBUG.
if(NOT MSVC)
  foreach(_cfg RELWITHDEBINFO RELEASE)
    foreach(_lang C CXX)
      string(REPLACE "-O2" "-O3"
             CMAKE_${_lang}_FLAGS_${_cfg} "${CMAKE_${_lang}_FLAGS_${_cfg}}")
    endforeach()
  endforeach()
endif()

# ── C++ Standard ──────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ── Threads ───────────────────────────────────────────────────────────────────
find_package(Threads REQUIRED)

# ── macOS: build fat arm64+arm64e so interposition works with all binaries ───
# System binaries (ls, cat, etc.) are arm64e (pointer authentication).
# DYLD_INSERT_LIBRARIES requires the dylib to match the process architecture.
if(APPLE AND CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
  if("${CMAKE_OSX_ARCHITECTURES}" STREQUAL "")
    set(CMAKE_OSX_ARCHITECTURES "arm64;arm64e" CACHE STRING
        "Build universal arm64+arm64e for dyld interposition" FORCE)
  endif()
endif()

# ── Compiler flags ────────────────────────────────────────────────────────────
if(NOT WIN32)
  add_compile_options(
    -fvisibility=hidden
    -fno-builtin-malloc
    -fno-builtin-free
    -fno-builtin-realloc
    -fno-builtin-calloc
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

# ── Sanitizers ────────────────────────────────────────────────────────────────
# -DSMASH_SANITIZE=undefined (also: address, or address,undefined) instruments
# smash and everything built beside it. Set here, before the tests and bench
# subdirectories, so the library and the executables that preload it are built
# and linked alike — a sanitized library loaded by an uninstrumented program
# still works, but then only the library's own frames are checked.
#
# Always use a separate build directory: the sanitized libsmash is slower and
# pulls in libubsan/libasan, so it is neither what you benchmark nor what you
# ship, and reusing the normal build directory is how you end up testing a
# library you did not mean to test.
#
# UBSan is the one that fits this project. ASan replaces malloc, which is
# exactly what smash exists to do, so an ASan build of a malloc-replacement
# library fights itself; the option still accepts it for instrumenting a single
# test binary, but not for the preloaded library.
#
# Findings are recoverable by default so the first one does not mask the rest.
# Set UBSAN_OPTIONS=halt_on_error=1 to stop at the first instead.
set(SMASH_SANITIZE "" CACHE STRING
    "Sanitizers to build with, e.g. undefined / address / address,undefined")
if(SMASH_SANITIZE)
  add_compile_options(-fsanitize=${SMASH_SANITIZE} -fno-omit-frame-pointer -g)
  add_link_options(-fsanitize=${SMASH_SANITIZE})
  # vptr instrumentation needs RTTI and fails to build without it.
  if(CMAKE_CXX_FLAGS MATCHES "-fno-rtti")
    add_compile_options(-fno-sanitize=vptr)
  endif()
  message(STATUS "Sanitizers ENABLED: -fsanitize=${SMASH_SANITIZE}")
endif()

# ── Pull in alloc8 ────────────────────────────────────────────────────────────
# By default smash is self-contained: it clones alloc8 from GitHub via
# FetchContent. To build against a local checkout instead (developer workflow,
# offline builds), pass -DALLOC8_DIR=/path/to/alloc8. ALLOC8_DIR is empty by
# default — we do NOT silently prefer a sibling ../alloc8, so the build is
# reproducible regardless of what happens to sit next to the source tree.
set(ALLOC8_DIR "" CACHE PATH "Local alloc8 source dir (empty = fetch from GitHub)")

# On Linux, use Smash's version script (extends alloc8's with syscall wrappers).
# Generated from src/smash_version_script.map.in with a per-arch GLIBC base
# version so malloc/free etc. are exported at a version that satisfies the
# binaries' requests (aarch64 binaries link against malloc@GLIBC_2.17, so
# emitting only @GLIBC_2.2.5 silently bypasses LD_PRELOAD interception).
# Must be set BEFORE add_subdirectory(alloc8) so alloc8 uses this path.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(SMASH_GLIBC_BASE_VERSION "GLIBC_2.17")
  else()
    set(SMASH_GLIBC_BASE_VERSION "GLIBC_2.2.5")
  endif()
  # _FORTIFY_SOURCE's __*_chk symbols are versioned GLIBC_2.4 / 2.5 / 2.16 on
  # x86-64, and the map exports the wrappers at exactly those versions. On
  # aarch64 the whole port starts at GLIBC_2.17, so every one of them is
  # __*_chk@@GLIBC_2.17 there (verified against an aarch64 libc) and NONE of
  # those exports matched: the entire fortify carve-out was inert on aarch64,
  # so any fortified read into a heap buffer bypassed the EFAULT retry.
  # Reported as test_syscall_efault failing with "__getgroups_chk returned -1
  # errno=14" — that test is simply the first to call a __*_chk by name.
  #
  # Where the base version IS 2.17, exporting the bare names in the base node
  # is what the callers bind. It is emitted only there: on x86-64 the bare
  # names must stay local, because exporting a bare __*_chk alongside its own
  # _NN alias at the SAME version is the self-jump that pinned a process at
  # 100% CPU (see the map's comment).
  if(SMASH_GLIBC_BASE_VERSION STREQUAL "GLIBC_2.17")
    set(SMASH_CHK_BASE_EXPORTS
"    __read_chk;
    __pread_chk;
    __pread64_chk;
    __recv_chk;
    __recvfrom_chk;
    __poll_chk;
    __ppoll_chk;
    __readlink_chk;
    __readlinkat_chk;
    __getgroups_chk;
    __fgets_chk;
    __fread_chk;")
  else()
    set(SMASH_CHK_BASE_EXPORTS "")
  endif()

  # aligned_alloc appeared in glibc 2.16, so a caller binds
  # aligned_alloc@GLIBC_2.16 (or @GLIBC_2.17 on aarch64, whose ABI baseline is
  # higher). Exporting it only in a GLIBC_2.2.5 base node leaves that reference
  # unbound and ld.so silently uses libc's aligned_alloc instead: measured,
  # smash_arena_for() said NO for every aligned_alloc block while
  # posix_memalign, memalign and valloc were all smash-owned. The blocks then
  # come back to smash's free(), which forwards them to libc — correct, but the
  # memory is never tracked or compressed.
  #
  # A version node can never precede the arch's ABI baseline, so emit the 2.16
  # node only when the base is below it; where the base is already >= 2.16
  # (aarch64, GLIBC_2.17) the bare name in the base node is what callers bind,
  # and a second node of the same name would be a duplicate definition.
  string(REPLACE "GLIBC_" "" _smash_base_ver "${SMASH_GLIBC_BASE_VERSION}")
  # The map already has a GLIBC_2.16 node (the __*_chk_216 aliases), so
  # aligned_alloc joins that one — a second node of the same version name is a
  # link error.
  if(_smash_base_ver VERSION_GREATER_EQUAL "2.16")
    set(SMASH_ALIGNED_ALLOC_IN_BASE "    aligned_alloc;")
    set(SMASH_ALIGNED_ALLOC_IN_216 "")
  else()
    set(SMASH_ALIGNED_ALLOC_IN_BASE "")
    set(SMASH_ALIGNED_ALLOC_IN_216 "    aligned_alloc;")
  endif()

  # clock_nanosleep@GLIBC_2.17 needs its own version node on x86-64, but on
  # aarch64 GLIBC_2.17 *is* the base version: defining it twice would both fail
  # to link and, if it linked, export clock_nanosleep at one version as the
  # default and as an alias — the self-jump trap documented in the map.
  if(SMASH_GLIBC_BASE_VERSION STREQUAL "GLIBC_2.17")
    set(SMASH_GLIBC_217_NODE "")
  else()
    set(SMASH_GLIBC_217_NODE
"GLIBC_2.17 {
  global:
    clock_nanosleep;
    clock_nanosleep_217;
} ${SMASH_GLIBC_BASE_VERSION};")
  endif()

  # recvmmsg / sendmmsg: glibc versions them at GLIBC_2.12 / 2.14 on x86-64,
  # and callers bind those, so base-node-only exports never ran (a
  # recvmmsg into a watched buffer returned EFAULT: dgram_recv_test). Only the
  # _212/_214 aliases go in these nodes: the bare name at the same version
  # too is the self-jump trap documented in the map. On aarch64 the base
  # (GLIBC_2.17) is what callers bind already.
  if(SMASH_GLIBC_BASE_VERSION STREQUAL "GLIBC_2.17")
    set(SMASH_GLIBC_212_214_NODES "")
  else()
    set(SMASH_GLIBC_212_214_NODES
"GLIBC_2.12 {
  global:
    recvmmsg_212;
} ${SMASH_GLIBC_BASE_VERSION};

GLIBC_2.14 {
  global:
    sendmmsg_214;
} GLIBC_2.12;")
  endif()

  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/smash_version_script.map.in
    ${CMAKE_CURRENT_BINARY_DIR}/smash_version_script.map
    @ONLY
  )
  set(SMASH_VERSION_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/smash_version_script.map")
  set(ALLOC8_VERSION_SCRIPT "${SMASH_VERSION_SCRIPT}" CACHE INTERNAL "Version script" FORCE)
endif()

include(FetchContent)

# alloc8 is pinned to the commit smash is tested against (its main branch moved
# under every fresh configure, release builds included), and the fixes in
# patches/alloc8 are applied on top: aligned_alloc of any size, with EINVAL for
# an alignment that is not a power of two; pvalloc of a size within a page of
# SIZE_MAX is ENOMEM. Moving the pin means checking that each patch still
# applies (the patch step fails if one does not) and is still needed.
set(SMASH_ALLOC8_COMMIT 816c933b79d68c40c0e1960318554f531832d9d0)
set(SMASH_ALLOC8_PATCHES "${CMAKE_CURRENT_SOURCE_DIR}/patches/alloc8")
find_package(Git QUIET)

if(ALLOC8_DIR)
  # Explicit local checkout requested.
  if(NOT EXISTS "${ALLOC8_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
      "ALLOC8_DIR=${ALLOC8_DIR} does not contain a CMakeLists.txt. "
      "Point it at a valid alloc8 checkout, or unset it to fetch from GitHub.")
  endif()
  message(STATUS "Using local alloc8 from ALLOC8_DIR=${ALLOC8_DIR}")
  # Its tree is not ours to patch: say which fixes it lacks.
  if(GIT_FOUND)
    file(GLOB _a8_patches "${SMASH_ALLOC8_PATCHES}/*.patch")
    foreach(_p IN LISTS _a8_patches)
      execute_process(COMMAND "${GIT_EXECUTABLE}" apply --reverse --check "${_p}"
                      WORKING_DIRECTORY "${ALLOC8_DIR}"
                      RESULT_VARIABLE _a8_applied OUTPUT_QUIET ERROR_QUIET)
      if(NOT _a8_applied EQUAL 0)
        get_filename_component(_name "${_p}" NAME)
        message(WARNING "ALLOC8_DIR lacks patches/alloc8/${_name}; apply it there "
                        "(git apply) or the build keeps the bug it fixes.")
      endif()
    endforeach()
  endif()
  add_subdirectory("${ALLOC8_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/alloc8")
else()
  # Default: self-contained. Clone alloc8 from GitHub at the pinned commit.
  message(STATUS "Fetching alloc8 ${SMASH_ALLOC8_COMMIT} from GitHub "
                 "(set -DALLOC8_DIR=... to use a local checkout)")
  # The patches' digest is part of the patch command, so editing or adding
  # one re-runs the patch step.
  file(GLOB _a8_patches "${SMASH_ALLOC8_PATCHES}/*.patch")
  list(SORT _a8_patches)
  set(_a8_digest "")
  foreach(_p IN LISTS _a8_patches)
    file(SHA256 "${_p}" _h)
    string(APPEND _a8_digest "${_h}")
  endforeach()
  string(SHA256 _a8_digest "${_a8_digest}")
  FetchContent_Declare(
    alloc8
    GIT_REPOSITORY https://github.com/emeryberger/alloc8.git
    GIT_TAG ${SMASH_ALLOC8_COMMIT}
    PATCH_COMMAND ${CMAKE_COMMAND} -DPATCH_DIR=${SMASH_ALLOC8_PATCHES}
                  -DGIT=${GIT_EXECUTABLE} -DPATCH_DIGEST=${_a8_digest}
                  -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/apply_patches.cmake
  )
  FetchContent_MakeAvailable(alloc8)
endif()

# ── emeryberger/printf — allocation-free printf/snprintf ────────────────────
# Standard glibc snprintf may call malloc internally (for large field widths
# or when %g/%a paths grow scratch buffers). Inside the malloc fast path,
# fault handlers, or compressor threads, that recursion can deadlock or
# overflow the stack. We compile this header-then-included library directly
# into libsmash and #include "printf.h" wherever we format diagnostic output
# from those paths.
FetchContent_Declare(
  printf
  GIT_REPOSITORY https://github.com/emeryberger/printf.git
  GIT_TAG        master
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(printf)

# ── LZ4 compression library ──────────────────────────────────────────────────
FetchContent_Declare(
  lz4
  GIT_REPOSITORY https://github.com/lz4/lz4.git
  GIT_TAG v1.9.4
  GIT_SHALLOW TRUE
)
FetchContent_GetProperties(lz4)
if(NOT lz4_POPULATED)
  FetchContent_Populate(lz4)
  add_library(lz4_static STATIC ${lz4_SOURCE_DIR}/lib/lz4.c ${lz4_SOURCE_DIR}/lib/lz4hc.c)
  target_include_directories(lz4_static PUBLIC ${lz4_SOURCE_DIR}/lib)
  set_target_properties(lz4_static PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

# ── Zstandard compression library ───────────────────────────────────────────
FetchContent_Declare(
  zstd
  GIT_REPOSITORY https://github.com/facebook/zstd.git
  GIT_TAG v1.5.6
  GIT_SHALLOW TRUE
  SOURCE_SUBDIR build/cmake
)
set(ZSTD_BUILD_SHARED OFF CACHE BOOL "" FORCE)
set(ZSTD_BUILD_PROGRAMS OFF CACHE BOOL "" FORCE)
set(ZSTD_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(ZSTD_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(zstd)

# ── LTO ───────────────────────────────────────────────────────────────────────
include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error)
if(ipo_supported)
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# ── libsmash shared library ──────────────────────────────────────────────────
# On Linux, gnu_wrapper.cpp already #includes new_delete.inc, so linking
# ALLOC8_COMMON_SOURCES (which contains new_delete.cpp) causes duplicate
# operator new/delete definitions. Only include common sources on non-Linux.
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set(_SMASH_COMMON_SOURCES ${ALLOC8_COMMON_SOURCES})
endif()

# Option to disable thread hooks (workaround for walrus crash)
option(SMASH_NO_THREAD_HOOKS "Disable alloc8 pthread interposition" OFF)

if(SMASH_NO_THREAD_HOOKS)
  set(_SMASH_THREAD_SOURCES "")
  set(_SMASH_THREAD_DEFINE "SMASH_NO_THREAD_HOOKS_BUILD")
  message(STATUS "Thread hooks DISABLED")
else()
  set(_SMASH_THREAD_SOURCES ${ALLOC8_THREAD_SOURCES})
  set(_SMASH_THREAD_DEFINE "")
  message(STATUS "Thread hooks enabled")
endif()

add_library(smash SHARED
  src/smash_heap.cpp
  src/linux_syscall_wrappers.cpp
  ${printf_SOURCE_DIR}/printf.cpp
  ${ALLOC8_INTERPOSE_SOURCES}
  ${_SMASH_THREAD_SOURCES}
  ${_SMASH_COMMON_SOURCES}
)
target_include_directories(smash PRIVATE ${printf_SOURCE_DIR})

# Linux syscall wrappers need default visibility for LD_PRELOAD interposition.
# The global -fvisibility=hidden flag hides them otherwise.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set_source_files_properties(src/linux_syscall_wrappers.cpp PROPERTIES
    COMPILE_FLAGS "-fvisibility=default -fno-lto"
  )
endif()

target_include_directories(smash PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Clang Thread Safety Analysis (https://clang.llvm.org/docs/ThreadSafetyAnalysis.html).
# Off by default (annotations in src/util/thread_safety.h are no-ops without it).
# Enable with -DSMASH_THREAD_SAFETY=ON under clang to statically prove that
# GUARDED_BY fields (e.g. VmRegion::FreeShard free_list/free_pool) are only
# touched while their lock is held, and REQUIRES functions are called locked.
# -Werror so a lock-discipline regression (like the 2026-06-25 free_pool race)
# fails the build instead of returning as a runtime livelock.
option(SMASH_THREAD_SAFETY "Enable clang -Wthread-safety static analysis (clang only)" OFF)
if(SMASH_THREAD_SAFETY)
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(smash PRIVATE -Wthread-safety -Werror=thread-safety)
    message(STATUS "Clang Thread Safety Analysis ENABLED (-Wthread-safety -Werror)")
  else()
    message(WARNING "SMASH_THREAD_SAFETY=ON ignored: -Wthread-safety needs clang")
  endif()
endif()

# On Linux, link against alloc8 components manually instead of alloc8::interpose
# to avoid the INTERFACE version script and use Smash's instead (includes syscall wrappers)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_link_libraries(smash PRIVATE alloc8::headers pthread dl)
  target_link_options(smash PRIVATE
    "LINKER:--version-script=${SMASH_VERSION_SCRIPT}"
    "-Bsymbolic"
  )
  # The version script controls which symbols (and at which versions) libsmash
  # exports, so a change to it changes the library — but CMake does not infer a
  # link dependency from a linker flag. Without this, editing
  # smash_version_script.map.in regenerates the .map and `make` relinks nothing,
  # so the next build silently tests the previous library. That cost a full
  # verification cycle: two rounds of results were measured against a stale .so
  # while the generated map already held the new content.
  set_target_properties(smash PROPERTIES LINK_DEPENDS "${SMASH_VERSION_SCRIPT}")
else()
  target_link_libraries(smash PRIVATE alloc8::interpose)
endif()
target_link_libraries(smash PRIVATE lz4_static libzstd_static Threads::Threads ${CMAKE_DL_LIBS})

# Add thread hooks define if disabled
if(SMASH_NO_THREAD_HOOKS)
  target_compile_definitions(smash PRIVATE SMASH_NO_THREAD_HOOKS_BUILD)
endif()

set_target_properties(smash PROPERTIES
  OUTPUT_NAME "smash"
  PREFIX "lib"
)
# Bind all PLT entries at load time (no lazy binding). The SIGSEGV handler
# must never trigger _dl_runtime_resolve — the dynamic linker's hash tables
# may live on smash-compressed pages, causing infinite signal recursion.
if(NOT APPLE)
  target_link_options(smash PRIVATE "LINKER:-z,now")
endif()

# ── Ablation defines (override config.h defaults via -DSMASH_*=value) ────────
# Numeric overrides (only when non-empty)
foreach(_var NUM_ARENAS COLD_TICKS VERY_COLD_TICKS DICT_TRAIN_SAMPLES
            PREFETCH_WINDOW COMPRESSOR_WORKERS COMPRESS_STORE_SHARDS
            LARGE_ALLOC_VM_THRESHOLD
            MAX_SLOTS_PER_PAGE COLD_ARENA_THRESHOLD
            ADAPTIVE_CAP_TARGET_PCT ADAPTIVE_CAP_MIN ADAPTIVE_CAP_MIN_SAMPLES)
  if(DEFINED SMASH_${_var} AND NOT "${SMASH_${_var}}" STREQUAL "")
    target_compile_definitions(smash PRIVATE "SMASH_${_var}=${SMASH_${_var}}")
  endif()
endforeach()
# Boolean ablation flags (define to enable, must be truthy)
foreach(_flag ABLATION_NO_ZERO_EAGER ABLATION_NO_ZERO_DEFERRED
              ABLATION_NO_SKIP_STATS ABLATION_NO_CHUNK_BITMAP
              ABLATION_NO_CALLSITE_ARENA
              COLD_ARENA_FEEDBACK PAGE_LOCAL_BATCH THREAD_ARENA_HASH
              ADAPTIVE_CAP MEASURE_COHORTS)
  if(SMASH_${_flag})
    target_compile_definitions(smash PRIVATE "SMASH_${_flag}=1")
  endif()
endforeach()
# The codec is a runtime choice now; say so rather than build zstd-1 silently.
if(SMASH_USE_LZ4)
  message(WARNING "SMASH_USE_LZ4 was removed and is ignored: the library uses "
                  "zstd-1 unless run with SMASH_CODEC=lz4.")
endif()

if(APPLE)
  set_target_properties(smash PROPERTIES
    SUFFIX ".dylib"
  )
endif()

# ── libsmash_noopt: compression without allocation optimizations ─────────────
# Same code, but arenas=1, no zero-on-free (eager or deferred).
# Used to isolate the contribution of Smash's allocation design.
add_library(smash_noopt SHARED
  src/smash_heap.cpp
  src/linux_syscall_wrappers.cpp
  ${printf_SOURCE_DIR}/printf.cpp
  ${ALLOC8_INTERPOSE_SOURCES}
  ${ALLOC8_THREAD_SOURCES}
  ${_SMASH_COMMON_SOURCES}
)

target_include_directories(smash_noopt PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${printf_SOURCE_DIR}
)

# On Linux, use custom version script (same as smash target)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_link_libraries(smash_noopt PRIVATE alloc8::headers pthread dl)
  target_link_options(smash_noopt PRIVATE
    "LINKER:--version-script=${SMASH_VERSION_SCRIPT}"
    "-Bsymbolic"
  )
  # Same link dependency as the smash target — see the note there.
  set_target_properties(smash_noopt PROPERTIES LINK_DEPENDS "${SMASH_VERSION_SCRIPT}")
else()
  target_link_libraries(smash_noopt PRIVATE alloc8::interpose)
endif()
target_link_libraries(smash_noopt PRIVATE lz4_static libzstd_static Threads::Threads ${CMAKE_DL_LIBS})

set_target_properties(smash_noopt PROPERTIES
  OUTPUT_NAME "smash_noopt"
  PREFIX "lib"
)

target_compile_definitions(smash_noopt PRIVATE
  SMASH_ABLATION_NO_CALLSITE_ARENA=1
  SMASH_ABLATION_NO_ZERO_EAGER=1
  SMASH_ABLATION_NO_ZERO_DEFERRED=1
)

if(APPLE)
  set_target_properties(smash_noopt PROPERTIES SUFFIX ".dylib")
endif()

# ── libsmash_compress_only: DEPRECATED - use libsmash with SMASH_MODE=compress_only
# Kept for backwards compatibility. The main libsmash library now supports
# compress-only mode at runtime: SMASH_MODE=compress_only LD_PRELOAD=libsmash.so
#
# This separate library can still be useful when you want a smaller binary
# that doesn't include the full Smash allocator code.
add_library(smash_compress_only SHARED
  src/compress_only/co_interpose_mac.cpp
  src/compress_only/co_interpose_linux.cpp
  ${printf_SOURCE_DIR}/printf.cpp
)

target_include_directories(smash_compress_only PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${lz4_SOURCE_DIR}/lib
  ${zstd_SOURCE_DIR}/lib
  ${printf_SOURCE_DIR}
)

target_link_libraries(smash_compress_only PRIVATE alloc8::headers lz4_static libzstd_static Threads::Threads ${CMAKE_DL_LIBS})

set_target_properties(smash_compress_only PROPERTIES
  OUTPUT_NAME "smash_compress_only"
  PREFIX "lib"
)

target_compile_definitions(smash_compress_only PRIVATE SMASH_COMPRESS_ONLY=1)

# No fno-builtin-malloc since we want to forward to the real system malloc
target_compile_options(smash_compress_only PRIVATE
  -fvisibility=hidden -Wall -Wextra -Wpedantic
)

if(APPLE)
  set_target_properties(smash_compress_only PROPERTIES SUFFIX ".dylib")
endif()

# ── Tests ─────────────────────────────────────────────────────────────────────
option(SMASH_BUILD_TESTS "Build smash tests" ON)
if(SMASH_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

# ── External comparison allocators ───────────────────────────────────────────
# Builds mimalloc / jemalloc / tcmalloc / Hoard as preloadable shared libraries
# into ${CMAKE_BINARY_DIR}/allocators/lib.  All BUILD_* options default OFF, so
# this is a no-op unless asked for (-DBUILD_ALL_ALLOCATORS=ON).  Added before
# bench/ so its allocator sources are configured first.
#
# Guarded on the directory existing: add_subdirectory() on a missing directory
# is a hard CMake error, which would break configure entirely for anyone whose
# tree lacks it (a partial checkout, an exported tarball, a sparse clone). The
# comparison allocators are a benchmarking convenience, never required to build
# or test smash, so degrade to a warning rather than taking the build down.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/allocators/CMakeLists.txt)
  add_subdirectory(allocators)
elseif(BUILD_ALL_ALLOCATORS OR BUILD_MIMALLOC OR BUILD_JEMALLOC
       OR BUILD_TCMALLOC OR BUILD_HOARD)
  # Only worth mentioning if they actually asked for the allocators.
  message(WARNING
    "Comparison allocators requested, but allocators/ is missing from this "
    "source tree — skipping. The BUILD_* allocator options will have no effect.")
endif()

# ── Benchmarks ───────────────────────────────────────────────────────────────
option(SMASH_BUILD_BENCH "Build smash benchmarks" OFF)
if(SMASH_BUILD_BENCH)
  add_subdirectory(bench)
endif()
