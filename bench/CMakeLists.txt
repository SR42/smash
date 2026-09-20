# smash/bench/CMakeLists.txt - Standalone benchmark executables
#
# Three layers, each on its own switch (entered only when SMASH_BUILD_BENCH=ON
# is set in the parent CMakeLists.txt):
#
#   default (always built when SMASH_BUILD_BENCH=ON):
#     bench_rss, bench_sqlite, bench_throughput, bench_latency,
#     bench_compression, bench_dict_experiment, bench_algo_compare,
#     bench_kv_store, bench_coldhotmix, bench_json,
#     bench_heap_compress*, sample_heap_pages
#
#   SMASH_BUILD_BENCH_ALLOCATORS=ON  (default):
#     allocator-comparison binaries — mimalloc / jemalloc / tcmalloc
#     (Bazel-built google/tcmalloc when bazel + glibc>=2.35; gperftools
#     fallback otherwise) / hoard / mesh / diehard / dieharder, plus
#     bench_allocator_compare.py. Pulls in tcmalloc via Bazel using
#     bench/tcmalloc_patch_build.cmake; the rest via find_library.
#
#   SMASH_BUILD_BENCH_DEPS=ON  (default):
#     a `bench_deps` make target that builds Redis / memcached /
#     RocksDB from source (separate ExternalProject_Add).
#     Adds 10–20 minutes to the first build. Set OFF to skip the
#     custom-built versions and use system installs (brew etc.).
#
# The CI workflow (.github/workflows/ci.yml) sets both sub-flags OFF
# so it builds only the smash-internal default layer — fast, no
# external dependencies, sufficient for regression spotting.

find_package(Threads REQUIRED)

# ── Architectures for benches and their deps (macOS) ─────────────────────────
# libsmash itself is built fat (arm64 + arm64e) so it can interpose into BOTH
# kinds of process. The benches are the opposite case: they are the application
# under test, and they must be arm64-ONLY.
#
# A fat executable runs its arm64e slice on Apple Silicon, and dyld refuses to
# load an arm64-only dylib into an arm64e process ("incompatible architecture
# (have 'arm64', need 'arm64e')" — the process aborts before main). Everything
# we preload into a bench is arm64-only:
#
#   - the RELEASED libsmash-macos-arm64.dylib. release.yml builds it arm64-only
#     on purpose: a user-built arm64e slice cannot be codesigned or loaded
#     without Apple's ARMA entitlement, so arm64 is the drop-in slice for
#     ordinary apps. A fat bench cannot load the artifact we actually ship.
#   - the comparison allocators (mimalloc, jemalloc, tcmalloc) in allocators/.
#
# Real applications are arm64, so an arm64 bench is also the honest model of
# one. Set it at directory scope: it covers every bench target here (present and
# future) and flows into the ExternalProject deps below, so their static libs
# match and the link doesn't drop objects for a mismatched slice.
if(APPLE)
  set(CMAKE_OSX_ARCHITECTURES "arm64")
endif()

# CMAKE_OSX_ARCHITECTURES is a ;-list, and ExternalProject_Add splits CMAKE_ARGS
# on ';' — so re-encode with '|' and pair with LIST_SEPARATOR | at each use.
# Empty on Linux, where -DCMAKE_OSX_ARCHITECTURES= is simply ignored.
string(REPLACE ";" "|" SMASH_BENCH_OSX_ARCHS "${CMAKE_OSX_ARCHITECTURES}")

# ── SDK env prefix for autoconf/Makefile sub-builds ─────────────────────────
# The CMake-based ExternalProjects below inherit SDK detection automatically.
# The autoconf/Makefile ones (varnish, memcached, tcmalloc, redis) shell out to
# a bare compiler that does NOT: on macOS, if the invoking shell has no SDKROOT,
# their configure/link fails with `ld: library 'System' not found`. Prefix those
# sub-build commands with this to inject SDKROOT. On Linux it stays empty and
# expands to nothing (the compiler needs no sysroot hint), so the prefix is a
# no-op there — the same command line works on both platforms.
set(SMASH_BENCH_SDK_ENV "")
if(APPLE)
  if(NOT CMAKE_OSX_SYSROOT STREQUAL "")
    set(_SMASH_SDKROOT "${CMAKE_OSX_SYSROOT}")
  else()
    execute_process(COMMAND xcrun --show-sdk-path
                    OUTPUT_VARIABLE _SMASH_SDKROOT
                    OUTPUT_STRIP_TRAILING_WHITESPACE
                    ERROR_QUIET)
  endif()
  if(NOT _SMASH_SDKROOT STREQUAL "")
    set(SMASH_BENCH_SDK_ENV ${CMAKE_COMMAND} -E env "SDKROOT=${_SMASH_SDKROOT}")
    message(STATUS "Bench autoconf sub-builds will use SDKROOT=${_SMASH_SDKROOT}")
  endif()
endif()

# ── Option to build external benchmark dependencies ──────────────────────────
# ON by default when SMASH_BUILD_BENCH=ON.  Set -DSMASH_BUILD_BENCH_DEPS=OFF to skip.
option(SMASH_BUILD_BENCH_DEPS "Build external benchmark dependencies (Redis, memcached, RocksDB)" ON)

# ── Option to build allocator-comparison benches ─────────────────────────────
# Pulls in tcmalloc/jemalloc/mimalloc/hoard/etc. via FetchContent +
# find_library + ExternalProject_Add. Keep ON for paper experiments;
# set OFF for the regression-spotting CI where those benches add a
# lot of build time and the Bazel-based google/tcmalloc path requires
# `bazel`/`bazelisk` on PATH plus glibc >= 2.35.
option(SMASH_BUILD_BENCH_ALLOCATORS "Build allocator-comparison benchmarks (mimalloc, jemalloc, tcmalloc, hoard, mesh, diehard)" ON)

set(SMASH_BENCH_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
  ${CMAKE_CURRENT_SOURCE_DIR}/../src
  # smash headers transitively #include "printf.h" (via src/util/safe_printf.h,
  # the allocation-free diagnostic formatter added in 1169dc3). Any bench TU
  # that pulls in a smash header therefore needs the vendored printf on its
  # include path, or the build fails with "'::vsnprintf_' has not been declared".
  ${printf_SOURCE_DIR}
)

# Helper to add a benchmark executable (linked against smash internals)
function(smash_add_bench name source)
  # The vendored printf.cpp defines vsnprintf_, which smash's allocation-free
  # safe_* formatters call. Without it these targets fail to link with
  # "undefined reference to vsnprintf_" -- in every configuration, sanitized or
  # not, which is why bench_compression, bench_dict_experiment and
  # bench_algo_compare could not be built at all. The other two bench helpers
  # already add it.
  add_executable(${name} ${source} ${printf_SOURCE_DIR}/printf.cpp)
  target_include_directories(${name} PRIVATE ${SMASH_BENCH_INCLUDE_DIRS})
  target_link_libraries(${name} PRIVATE alloc8::headers lz4_static libzstd_static ${CMAKE_DL_LIBS})
  if(APPLE)
    target_compile_options(${name} PRIVATE -ftls-model=initial-exec)
  endif()
endfunction()

# Helper: filter out -fno-builtin-* flags from directory compile options.
# The root CMakeLists.txt sets these globally for smash internals, but standalone
# benchmarks use system malloc/free and must not have them.
function(_get_filtered_dir_compile_options out_var)
  get_property(dir_opts DIRECTORY PROPERTY COMPILE_OPTIONS)
  set(filtered "")
  foreach(opt IN LISTS dir_opts)
    if(NOT opt MATCHES "^-fno-builtin-")
      list(APPEND filtered "${opt}")
    endif()
  endforeach()
  set(${out_var} "${filtered}" PARENT_SCOPE)
endfunction()

# Helper to add a standalone benchmark executable (uses system malloc/free).
# These are designed to run directly (baseline) or via DYLD_INSERT_LIBRARIES/LD_PRELOAD
# with libsmash for A/B comparison.
function(smash_add_standalone_bench name source)
  add_executable(${name} ${source})
  # Override compile options to exclude -fno-builtin-malloc etc.
  _get_filtered_dir_compile_options(clean_opts)
  set_target_properties(${name} PROPERTIES COMPILE_OPTIONS "${clean_opts}")
  target_link_libraries(${name} PRIVATE Threads::Threads)
endfunction()

# ── cJSON library (for bench_json) ──────────────────────────────────────────
include(FetchContent)
FetchContent_Declare(
  cjson
  GIT_REPOSITORY https://github.com/DaveGamble/cJSON.git
  GIT_TAG v1.7.18
  GIT_SHALLOW TRUE
)
FetchContent_GetProperties(cjson)
if(NOT cjson_POPULATED)
  FetchContent_Populate(cjson)
  add_library(cjson_static STATIC ${cjson_SOURCE_DIR}/cJSON.c)
  target_include_directories(cjson_static PUBLIC ${cjson_SOURCE_DIR})
  set_target_properties(cjson_static PROPERTIES POSITION_INDEPENDENT_CODE ON)
  # cJSON uses builtin malloc — strip the global -fno-builtin-* flags
  _get_filtered_dir_compile_options(cjson_opts)
  set_target_properties(cjson_static PROPERTIES COMPILE_OPTIONS "${cjson_opts}")
endif()

# ── Existing internal benchmarks ────────────────────────────────────────────

# Malloc/free throughput across size classes
add_executable(bench_throughput
  bench_throughput.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/smash_heap.cpp
  # smash_heap.cpp uses the allocation-free safe_* printf wrappers (vsnprintf_),
  # whose definition lives in the vendored printf.cpp — link it here, else the
  # build fails at link with "undefined reference to vsnprintf_".
  ${printf_SOURCE_DIR}/printf.cpp
)
target_include_directories(bench_throughput PRIVATE ${SMASH_BENCH_INCLUDE_DIRS})
target_link_libraries(bench_throughput PRIVATE alloc8::headers lz4_static libzstd_static ${CMAKE_DL_LIBS})
if(APPLE)
  target_compile_options(bench_throughput PRIVATE -ftls-model=initial-exec)
endif()

# Compression ratio comparison: LZ4 vs zstd vs zstd+dict
smash_add_bench(bench_compression bench_compression.cpp)

# Dictionary compression experiments
smash_add_bench(bench_dict_experiment bench_dict_experiment.cpp)

# Algorithm comparison: WKdm vs LZ4 vs zstd, multi-page compression
smash_add_bench(bench_algo_compare bench_algo_compare.cpp)

# RSS reduction over time. Uses the standard malloc/free API and links NO
# smash internals — run it under DYLD_INSERT_LIBRARIES / LD_PRELOAD to
# exercise smash (or with no preload for a system-malloc baseline). This
# mirrors real application use and avoids compiling a second static copy of
# the allocator into the benchmark.
add_executable(bench_rss bench_rss.cpp)
target_link_libraries(bench_rss PRIVATE ${CMAKE_DL_LIBS})

# Allocation speed — standalone (threadtest, larson, linux-scalability)
add_executable(bench_alloc_speed bench_alloc_speed.cpp)
target_link_libraries(bench_alloc_speed PRIVATE Threads::Threads)

# Compression ratio comparison: smash vs OS compressor (standalone, LD_PRELOAD)
add_executable(bench_compression_vs_zswap bench_compression_vs_zswap.cpp)
target_link_libraries(bench_compression_vs_zswap PRIVATE Threads::Threads lz4_static libzstd_static)
target_include_directories(bench_compression_vs_zswap PRIVATE
  ${lz4_SOURCE_DIR}/lib ${zstd_SOURCE_DIR}/lib)

# Alloc/free latency percentiles
add_executable(bench_latency
  bench_latency.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/smash_heap.cpp
  # smash_heap.cpp uses the allocation-free safe_* printf wrappers (vsnprintf_),
  # whose definition lives in the vendored printf.cpp — link it here, else the
  # build fails at link with "undefined reference to vsnprintf_".
  ${printf_SOURCE_DIR}/printf.cpp
)
target_include_directories(bench_latency PRIVATE ${SMASH_BENCH_INCLUDE_DIRS})
target_link_libraries(bench_latency PRIVATE alloc8::headers lz4_static libzstd_static ${CMAKE_DL_LIBS})
if(APPLE)
  target_compile_options(bench_latency PRIVATE -ftls-model=initial-exec)
endif()

# ── Standalone real-world benchmarks ────────────────────────────────────────

# JSON processing workload (cJSON DOM tree with hot/cold access)
smash_add_standalone_bench(bench_json bench_json.cpp)
target_link_libraries(bench_json PRIVATE cjson_static)

# Key-value store simulation (unordered_map with Zipfian access)
smash_add_standalone_bench(bench_kv_store bench_kv_store.cpp)

# Churn re-heat: measures cooling-page closing (allocation churn re-heating
# pages that hold cold survivors). A/B via SMASH_COOLING_CLOSE_TICKS=0.
smash_add_standalone_bench(bench_churn_reheat bench_churn_reheat.cpp)

# Configurable cold/hot mix microbenchmark (raw malloc/free)
smash_add_standalone_bench(bench_coldhotmix bench_coldhotmix.cpp)

# Zipfian re-access microbenchmark — cold-age IS predictive of re-access, the
# regime that discriminates a Lindy/Bayesian compress gate from blunt backoff.
smash_add_standalone_bench(bench_zipf_reaccess bench_zipf_reaccess.cpp)

# ML model serving scenario (multiple models, sparse activation)
smash_add_standalone_bench(bench_model_serving bench_model_serving.cpp)

# Concurrent builds scenario (multiple jobs with cold IR heaps)
smash_add_standalone_bench(bench_concurrent_builds bench_concurrent_builds.cpp)

# Arena segregation vs. mixed-page compression advantage
smash_add_standalone_bench(bench_arena_segregation bench_arena_segregation.cpp)
target_link_libraries(bench_arena_segregation PRIVATE lz4_static libzstd_static)

# ── SQLite amalgamation (for bench_sqlite) ─────────────────────────────────
FetchContent_Declare(
  sqlite3
  URL https://www.sqlite.org/2026/sqlite-amalgamation-3510200.zip
)
FetchContent_GetProperties(sqlite3)
if(NOT sqlite3_POPULATED)
  FetchContent_Populate(sqlite3)
  add_library(sqlite3_static STATIC ${sqlite3_SOURCE_DIR}/sqlite3.c)
  target_include_directories(sqlite3_static PUBLIC ${sqlite3_SOURCE_DIR})
  target_compile_definitions(sqlite3_static PRIVATE
    SQLITE_THREADSAFE=0
    SQLITE_OMIT_LOAD_EXTENSION
  )
  set_target_properties(sqlite3_static PROPERTIES POSITION_INDEPENDENT_CODE ON)
  _get_filtered_dir_compile_options(sqlite3_opts)
  set_target_properties(sqlite3_static PROPERTIES COMPILE_OPTIONS "${sqlite3_opts}")
endif()

# SQLite in-memory database workload
smash_add_standalone_bench(bench_sqlite bench_sqlite.cpp)
target_link_libraries(bench_sqlite PRIVATE sqlite3_static)

# Heatmap benchmark: coldness × compressibility sweep
smash_add_standalone_bench(bench_heatmap bench_heatmap.cpp)

# ── Allocator substrate comparison benchmarks ───────────────────────────────
if(SMASH_BUILD_BENCH_ALLOCATORS)

# Helper to add allocator comparison benchmark variant
function(smash_add_alloc_bench name alloc_define)
  smash_add_standalone_bench(${name} bench_allocator_compare.cpp)
  target_compile_definitions(${name} PRIVATE ALLOC_NAME="${alloc_define}" HAS_SQLITE)
  target_link_libraries(${name} PRIVATE lz4_static libzstd_static sqlite3_static)
endfunction()

# Helper: allocator bench with ZERO_ON_FREE enabled (benchmark zeros before free)
function(smash_add_alloc_bench_zero name alloc_define)
  smash_add_alloc_bench(${name} "${alloc_define}")
  target_compile_definitions(${name} PRIVATE ZERO_ON_FREE)
endfunction()

# System malloc (also used as Smash target via DYLD_INSERT_LIBRARIES)
smash_add_alloc_bench(bench_alloc_system "system")
smash_add_alloc_bench_zero(bench_alloc_system_zero "system+zero")

# mimalloc
FetchContent_Declare(mimalloc
  GIT_REPOSITORY https://github.com/microsoft/mimalloc.git
  GIT_TAG v2.1.2
  GIT_SHALLOW TRUE
)
set(MI_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MI_BUILD_SHARED OFF CACHE BOOL "" FORCE)
set(MI_BUILD_OBJECT OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(mimalloc)

smash_add_alloc_bench(bench_alloc_mimalloc "mimalloc")
target_link_libraries(bench_alloc_mimalloc PRIVATE mimalloc-static)
smash_add_alloc_bench_zero(bench_alloc_mimalloc_zero "mimalloc+zero")
target_link_libraries(bench_alloc_mimalloc_zero PRIVATE mimalloc-static)

# jemalloc (system install, optional)
find_library(JEMALLOC_LIB jemalloc HINTS /opt/homebrew/lib)
find_path(JEMALLOC_INCLUDE jemalloc/jemalloc.h HINTS /opt/homebrew/include)
if(JEMALLOC_LIB AND JEMALLOC_INCLUDE)
  smash_add_alloc_bench(bench_alloc_jemalloc "jemalloc")
  target_link_libraries(bench_alloc_jemalloc PRIVATE ${JEMALLOC_LIB})
  target_include_directories(bench_alloc_jemalloc PRIVATE ${JEMALLOC_INCLUDE})
  smash_add_alloc_bench_zero(bench_alloc_jemalloc_zero "jemalloc+zero")
  target_link_libraries(bench_alloc_jemalloc_zero PRIVATE ${JEMALLOC_LIB})
  target_include_directories(bench_alloc_jemalloc_zero PRIVATE ${JEMALLOC_INCLUDE})
  message(STATUS "jemalloc found: ${JEMALLOC_LIB}")
else()
  message(STATUS "jemalloc not found (optional)")
endif()

# tcmalloc - build from source
#
# google/tcmalloc (Bazel-based, per-CPU caches) requires glibc >= 2.35 for
# RSEQ TLS support.  On older glibc (e.g., Amazon Linux 2 with glibc 2.26)
# it segfaults during ld.so initialization.  We attempt google/tcmalloc first
# when both Bazel and a suitable glibc are available; otherwise fall back to
# gperftools 2.16 (the stable autotools-based fork).
include(ExternalProject)

# Detect glibc version for google/tcmalloc compatibility.  "0" means "not
# glibc" (macOS, musl) or unparseable `ldd --version` output — either way there
# is no RSEQ TLS, so google/tcmalloc is off the table.
set(_GLIBC_VERSION "0")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  execute_process(
    COMMAND ldd --version
    OUTPUT_VARIABLE _ldd_output ERROR_VARIABLE _ldd_output
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  string(REGEX MATCH "[0-9]+\\.[0-9]+" _GLIBC_VERSION "${_ldd_output}")
endif()

find_program(BAZEL_PROG bazel bazelisk)
if(BAZEL_PROG AND _GLIBC_VERSION VERSION_GREATER_EQUAL "2.35")
  # ── google/tcmalloc via Bazel ────────────────────────────────────────────
  set(TCMALLOC_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/tcmalloc)
  set(TCMALLOC_SO ${TCMALLOC_PREFIX}/lib/libtcmalloc.so)

  ExternalProject_Add(tcmalloc_build
    GIT_REPOSITORY https://github.com/google/tcmalloc.git
    GIT_TAG master
    GIT_SHALLOW TRUE
    PREFIX ${TCMALLOC_PREFIX}
    CONFIGURE_COMMAND ""
    PATCH_COMMAND
      ${CMAKE_COMMAND} -E echo "" > <SOURCE_DIR>/tcmalloc/tcmalloc_so.cc &&
      ${CMAKE_COMMAND}
        -DBUILD_FILE=<SOURCE_DIR>/tcmalloc/BUILD
        -P ${CMAKE_CURRENT_SOURCE_DIR}/tcmalloc_patch_build.cmake
    BUILD_IN_SOURCE TRUE
    BUILD_COMMAND
      ${CMAKE_COMMAND} -E env USE_BAZEL_VERSION=8.2.1
      ${BAZEL_PROG} build -c opt //tcmalloc:libtcmalloc_preload.so
    INSTALL_COMMAND
      ${CMAKE_COMMAND} -E make_directory ${TCMALLOC_PREFIX}/lib &&
      ${CMAKE_COMMAND} -E copy
        <SOURCE_DIR>/bazel-bin/tcmalloc/libtcmalloc_preload.so
        ${TCMALLOC_SO}
    BUILD_BYPRODUCTS ${TCMALLOC_SO}
  )

  smash_add_alloc_bench(bench_alloc_tcmalloc "tcmalloc")
  target_link_libraries(bench_alloc_tcmalloc PRIVATE ${TCMALLOC_SO} Threads::Threads)
  add_dependencies(bench_alloc_tcmalloc tcmalloc_build)

  smash_add_alloc_bench_zero(bench_alloc_tcmalloc_zero "tcmalloc+zero")
  target_link_libraries(bench_alloc_tcmalloc_zero PRIVATE ${TCMALLOC_SO} Threads::Threads)
  add_dependencies(bench_alloc_tcmalloc_zero tcmalloc_build)

  set(BENCH_TCMALLOC_LIB ${TCMALLOC_SO} CACHE PATH "")
  message(STATUS "google/tcmalloc (latest) will be built from source using Bazel 8.2.1")
else()
  # ── gperftools/tcmalloc 2.16 (works on all glibc versions) ──────────────
  if(BAZEL_PROG AND NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(STATUS "${CMAKE_SYSTEM_NAME}: google/tcmalloc requires glibc >= 2.35 (RSEQ TLS); using gperftools instead")
  elseif(BAZEL_PROG AND _GLIBC_VERSION STREQUAL "0")
    message(STATUS "glibc version undetected: google/tcmalloc requires glibc >= 2.35 (RSEQ TLS); using gperftools instead")
  elseif(BAZEL_PROG)
    message(STATUS "glibc ${_GLIBC_VERSION} < 2.35: google/tcmalloc requires RSEQ TLS; using gperftools instead")
  endif()

  include(ProcessorCount)
  ProcessorCount(_TCMALLOC_NCPU)
  if(_TCMALLOC_NCPU EQUAL 0)
    set(_TCMALLOC_NCPU 4)
  endif()

  set(GPERFTOOLS_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/gperftools)
  if(APPLE)
    set(_TCMALLOC_SHLIB_SUFFIX ".dylib")
  else()
    set(_TCMALLOC_SHLIB_SUFFIX ".so")
  endif()
  ExternalProject_Add(tcmalloc_build
    URL https://github.com/gperftools/gperftools/releases/download/gperftools-2.16/gperftools-2.16.tar.gz
    URL_HASH SHA256=f12624af5c5987f2cc830ee534f754c3c5961eec08004c26a8b80de015cf056f
    PREFIX ${GPERFTOOLS_PREFIX}
    CONFIGURE_COMMAND
      ${SMASH_BENCH_SDK_ENV} <SOURCE_DIR>/configure
        --prefix=${GPERFTOOLS_PREFIX}/install
        --enable-minimal
        --disable-debugalloc
    BUILD_COMMAND ${SMASH_BENCH_SDK_ENV} make -j${_TCMALLOC_NCPU}
    INSTALL_COMMAND make install
    BUILD_BYPRODUCTS
      ${GPERFTOOLS_PREFIX}/install/lib/libtcmalloc_minimal${_TCMALLOC_SHLIB_SUFFIX}
      ${GPERFTOOLS_PREFIX}/install/lib/libtcmalloc_minimal.a
  )
  set(TCMALLOC_LIB_BUILTIN ${GPERFTOOLS_PREFIX}/install/lib/libtcmalloc_minimal${_TCMALLOC_SHLIB_SUFFIX})
  set(TCMALLOC_INCLUDE_BUILTIN ${GPERFTOOLS_PREFIX}/install/include)

  smash_add_alloc_bench(bench_alloc_tcmalloc "tcmalloc")
  target_link_libraries(bench_alloc_tcmalloc PRIVATE ${TCMALLOC_LIB_BUILTIN} Threads::Threads)
  target_include_directories(bench_alloc_tcmalloc PRIVATE ${TCMALLOC_INCLUDE_BUILTIN})
  add_dependencies(bench_alloc_tcmalloc tcmalloc_build)

  smash_add_alloc_bench_zero(bench_alloc_tcmalloc_zero "tcmalloc+zero")
  target_link_libraries(bench_alloc_tcmalloc_zero PRIVATE ${TCMALLOC_LIB_BUILTIN} Threads::Threads)
  target_include_directories(bench_alloc_tcmalloc_zero PRIVATE ${TCMALLOC_INCLUDE_BUILTIN})
  add_dependencies(bench_alloc_tcmalloc_zero tcmalloc_build)

  set(BENCH_TCMALLOC_LIB ${TCMALLOC_LIB_BUILTIN} CACHE PATH "")
  message(STATUS "gperftools/tcmalloc 2.16 will be built from source")
endif()

# Hoard (system install, optional)
find_library(HOARD_LIB hoard libhoard)
if(HOARD_LIB)
  smash_add_alloc_bench(bench_alloc_hoard "hoard")
  target_link_libraries(bench_alloc_hoard PRIVATE ${HOARD_LIB})
  smash_add_alloc_bench_zero(bench_alloc_hoard_zero "hoard+zero")
  target_link_libraries(bench_alloc_hoard_zero PRIVATE ${HOARD_LIB})
  message(STATUS "Hoard found: ${HOARD_LIB}")
else()
  message(STATUS "Hoard not found (optional)")
endif()

# Mesh (external metadata, virtual remapping, no zeroing) - optional
# Prefer static lib to avoid Bazel rpath issues with the dylib
find_library(MESH_LIB_STATIC NAMES libmesh.a HINTS /usr/local/lib)
find_library(MESH_LIB_SHARED mesh HINTS /usr/local/lib)
find_path(MESH_INCLUDE plasma/mesh.h HINTS /usr/local/include)
if(MESH_LIB_STATIC)
  set(MESH_LIB ${MESH_LIB_STATIC})
elseif(MESH_LIB_SHARED)
  set(MESH_LIB ${MESH_LIB_SHARED})
endif()
if(MESH_LIB)
  smash_add_alloc_bench(bench_alloc_mesh "mesh")
  target_link_libraries(bench_alloc_mesh PRIVATE ${MESH_LIB})
  if(MESH_INCLUDE)
    target_include_directories(bench_alloc_mesh PRIVATE ${MESH_INCLUDE})
  endif()
  # Mesh links against Foundation on macOS
  if(APPLE)
    target_link_libraries(bench_alloc_mesh PRIVATE "-framework Foundation")
  endif()
  smash_add_alloc_bench_zero(bench_alloc_mesh_zero "mesh+zero")
  target_link_libraries(bench_alloc_mesh_zero PRIVATE ${MESH_LIB})
  if(MESH_INCLUDE)
    target_include_directories(bench_alloc_mesh_zero PRIVATE ${MESH_INCLUDE})
  endif()
  if(APPLE)
    target_link_libraries(bench_alloc_mesh_zero PRIVATE "-framework Foundation")
  endif()
  message(STATUS "Mesh found: ${MESH_LIB}")
else()
  message(STATUS "Mesh not found (optional)")
endif()

# DieHard (external metadata, no zeroing) - optional
find_library(DIEHARD_LIB diehard libdiehard)
if(DIEHARD_LIB)
  smash_add_alloc_bench(bench_alloc_diehard "diehard")
  target_link_libraries(bench_alloc_diehard PRIVATE ${DIEHARD_LIB})
  smash_add_alloc_bench_zero(bench_alloc_diehard_zero "diehard+zero")
  target_link_libraries(bench_alloc_diehard_zero PRIVATE ${DIEHARD_LIB})
  message(STATUS "DieHard found: ${DIEHARD_LIB}")
else()
  message(STATUS "DieHard not found (optional)")
endif()

# DieHarder (external metadata + security zeroing) - optional
find_library(DIEHARDER_LIB dieharder libdieharder)
if(DIEHARDER_LIB)
  smash_add_alloc_bench(bench_alloc_dieharder "dieharder")
  target_link_libraries(bench_alloc_dieharder PRIVATE ${DIEHARDER_LIB})
  message(STATUS "DieHarder found: ${DIEHARDER_LIB}")
else()
  message(STATUS "DieHarder not found (optional)")
endif()

# ── Preloadable shared allocators (jemalloc, mimalloc) for whole-application
# LD_PRELOAD comparisons ──────────────────────────────────────────────────────
# The bench_alloc_* microbenchmarks above STATIC-link mimalloc and use a
# system jemalloc, which is fine for direct-linked micro comparisons. But the
# application-level RSS study (redis/valkey/memcached/mongo/... under
# LD_PRELOAD) needs preloadable *.so builds at pinned, reproducible versions.
# Build them here from source (same pins as elsewhere) and export their paths,
# mirroring the tcmalloc-preload pattern above.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  ProcessorCount(_ALLOC_NCPU)
  if(_ALLOC_NCPU EQUAL 0)
    set(_ALLOC_NCPU 4)
  endif()
  set(ALLOC_PRELOAD_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/alloc_preload)
  set(JEMALLOC_PRELOAD_SO  ${ALLOC_PRELOAD_PREFIX}/lib/libjemalloc.so)
  set(MIMALLOC_PRELOAD_SO  ${ALLOC_PRELOAD_PREFIX}/lib/libmimalloc.so)

  ExternalProject_Add(jemalloc_preload_build
    GIT_REPOSITORY https://github.com/jemalloc/jemalloc.git
    GIT_TAG 5.3.0
    GIT_SHALLOW TRUE
    PREFIX ${ALLOC_PRELOAD_PREFIX}/jemalloc
    CONFIGURE_COMMAND cd <SOURCE_DIR> && ./autogen.sh &&
      <SOURCE_DIR>/configure --prefix=${ALLOC_PRELOAD_PREFIX} --enable-shared --disable-static
    BUILD_COMMAND cd <SOURCE_DIR> && make -j${_ALLOC_NCPU}
    INSTALL_COMMAND cd <SOURCE_DIR> && make install
    BUILD_IN_SOURCE TRUE
    BUILD_BYPRODUCTS ${JEMALLOC_PRELOAD_SO}
  )
  message(STATUS "  jemalloc 5.3.0 (preloadable .so)")

  ExternalProject_Add(mimalloc_preload_build
    GIT_REPOSITORY https://github.com/microsoft/mimalloc.git
    GIT_TAG v2.1.2
    GIT_SHALLOW TRUE
    PREFIX ${ALLOC_PRELOAD_PREFIX}/mimalloc
    CMAKE_ARGS
      -DCMAKE_BUILD_TYPE=Release
      -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
      -DMI_BUILD_SHARED=ON -DMI_BUILD_STATIC=OFF -DMI_BUILD_OBJECT=OFF
      -DMI_BUILD_TESTS=OFF
      -DCMAKE_INSTALL_PREFIX=${ALLOC_PRELOAD_PREFIX}
      -DCMAKE_INSTALL_LIBDIR=lib
    BUILD_COMMAND ${CMAKE_COMMAND} --build . -- -j${_ALLOC_NCPU}
    INSTALL_COMMAND ${CMAKE_COMMAND} --build . --target install
    BUILD_BYPRODUCTS ${MIMALLOC_PRELOAD_SO}
  )
  message(STATUS "  mimalloc v2.1.2 (preloadable .so)")

  set(BENCH_JEMALLOC_PRELOAD ${JEMALLOC_PRELOAD_SO} CACHE PATH "")
  set(BENCH_MIMALLOC_PRELOAD ${MIMALLOC_PRELOAD_SO} CACHE PATH "")
endif()

endif() # SMASH_BUILD_BENCH_ALLOCATORS

# Python runner (configure paths)
if(SMASH_BUILD_BENCH_ALLOCATORS)
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/bench_allocator_compare.py.in
    ${CMAKE_CURRENT_BINARY_DIR}/bench_allocator_compare.py
    @ONLY
  )
endif()

# ── Shell script benchmarks (configure paths) ──────────────────────────────
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/bench_runner.sh.in
  ${CMAKE_CURRENT_BINARY_DIR}/bench_runner.sh
  @ONLY
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/bench_redis.sh.in
  ${CMAKE_CURRENT_BINARY_DIR}/bench_redis.sh
  @ONLY
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/bench_python.sh.in
  ${CMAKE_CURRENT_BINARY_DIR}/bench_python.sh
  @ONLY
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/bench_sort.sh.in
  ${CMAKE_CURRENT_BINARY_DIR}/bench_sort.sh
  @ONLY
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/bench_memcached.sh.in
  ${CMAKE_CURRENT_BINARY_DIR}/bench_memcached.sh
  @ONLY
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/bench_memcached.py.in
  ${CMAKE_CURRENT_BINARY_DIR}/bench_memcached.py
  @ONLY
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/bench_memcached_alloc.sh.in
  ${CMAKE_CURRENT_BINARY_DIR}/bench_memcached_alloc.sh
  @ONLY
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/bench_redis_alloc.sh.in
  ${CMAKE_CURRENT_BINARY_DIR}/bench_redis_alloc.sh
  @ONLY
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/bench_polars.sh.in
  ${CMAKE_CURRENT_BINARY_DIR}/bench_polars.sh
  @ONLY
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/bench_sklearn.sh.in
  ${CMAKE_CURRENT_BINARY_DIR}/bench_sklearn.sh
  @ONLY
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/bench_networkx.sh.in
  ${CMAKE_CURRENT_BINARY_DIR}/bench_networkx.sh
  @ONLY
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/bench_rocksdb.sh.in
  ${CMAKE_CURRENT_BINARY_DIR}/bench_rocksdb.sh
  @ONLY
)

# ── RocksDB benchmark (system install fallback) ──────────────────────────────
# Only use brew/system RocksDB if we are NOT building deps from source.
#
# SMASH_ROCKSDB_VERSION is the single source of truth for which RocksDB the
# benches target: it drives the from-source GIT_TAG below *and* the acceptance
# check on a system install, so a stray system RocksDB cannot silently change
# the workload under measurement.
#
# Pinned to 11.1.2 (current release).  The previous pin was 9.8.4 — the version
# the original paper figures were measured against — and an A/B of the two on
# the EC2 Linux box showed the version does not move the result (82.6% vs 82.4%
# cool-phase RSS reduction, a gap smaller than the run-to-run spread), which is
# what makes moving the pin forward safe.  Building against 10+ at all requires
# the ROCKSDB_MAJOR conditional in bench_rocksdb.cpp / bench_heap_compress.cpp
# for the DB::Open signature change in 10.0.1 (`DB**` → `std::unique_ptr<DB>*`).
#
# A system install newer than the pin is usable but reported loudly (the paper's
# numbers describe the pinned version); older than the pin is refused.
set(SMASH_ROCKSDB_VERSION "11.1.2" CACHE STRING
    "RocksDB version the benches target")

set(SMASH_SYSTEM_ROCKSDB_OK FALSE)
if(NOT SMASH_BUILD_BENCH_DEPS)
  # SMASH_ROCKSDB_ROOT lets a matching out-of-tree build be used without a full
  # deps build (e.g. -DSMASH_ROCKSDB_ROOT=/path/to/build/bench/deps).  Empty by
  # default: we never silently prefer a scratch tree we happen to find.
  set(SMASH_ROCKSDB_ROOT "" CACHE PATH
      "Prefix holding a RocksDB ${SMASH_ROCKSDB_VERSION} lib/ + include/ (empty = search the system)")

  find_library(ROCKSDB_LIB rocksdb
    HINTS ${SMASH_ROCKSDB_ROOT}/lib /opt/homebrew/lib)
  find_path(ROCKSDB_INCLUDE rocksdb/db.h
    HINTS ${SMASH_ROCKSDB_ROOT}/include /opt/homebrew/include)

  # Reconstruct major.minor.patch from the found headers so the check compares
  # against the pin above rather than a hardcoded API cutoff.
  set(_ROCKSDB_FOUND_VERSION "0")
  if(ROCKSDB_INCLUDE AND EXISTS ${ROCKSDB_INCLUDE}/rocksdb/version.h)
    set(_rocksdb_ver_parts "")
    foreach(_part MAJOR MINOR PATCH)
      file(STRINGS ${ROCKSDB_INCLUDE}/rocksdb/version.h _line
           REGEX "^#define[ \t]+ROCKSDB_${_part}[ \t]+[0-9]+")
      string(REGEX MATCH "[0-9]+" _num "${_line}")
      list(APPEND _rocksdb_ver_parts "${_num}")
    endforeach()
    string(JOIN "." _ROCKSDB_FOUND_VERSION ${_rocksdb_ver_parts})
  endif()

  if(ROCKSDB_LIB AND ROCKSDB_INCLUDE
     AND _ROCKSDB_FOUND_VERSION VERSION_GREATER_EQUAL SMASH_ROCKSDB_VERSION)
    set(SMASH_SYSTEM_ROCKSDB_OK TRUE)
    smash_add_standalone_bench(bench_rocksdb bench_rocksdb.cpp)
    target_link_libraries(bench_rocksdb PRIVATE ${ROCKSDB_LIB})
    target_include_directories(bench_rocksdb PRIVATE ${ROCKSDB_INCLUDE})
    # Homebrew's shared librocksdb carries its own LZ4/ZSTD, but a static
    # librocksdb.a (what the pinned from-source build produces, and what
    # SMASH_ROCKSDB_ROOT usually points at) leaves LZ4_*/ZSTD_*/ZDICT_*
    # undefined — RocksDB is configured WITH_LZ4=ON WITH_ZSTD=ON.  Supply them
    # from the versions the bench tree already builds.
    if(ROCKSDB_LIB MATCHES "\\.a$")
      target_link_libraries(bench_rocksdb PRIVATE lz4_static libzstd_static)
    endif()
    if(_ROCKSDB_FOUND_VERSION VERSION_EQUAL SMASH_ROCKSDB_VERSION)
      message(STATUS "RocksDB ${_ROCKSDB_FOUND_VERSION} found (matches pin): ${ROCKSDB_LIB}")
    else()
      message(WARNING "RocksDB ${_ROCKSDB_FOUND_VERSION} found (${ROCKSDB_LIB}), newer than "
                      "the ${SMASH_ROCKSDB_VERSION} pin the paper's RocksDB figures were "
                      "measured against. The benches compile against it, but numbers from "
                      "this build are NOT comparable to the published ones — use "
                      "-DSMASH_BUILD_BENCH_DEPS=ON (or -DSMASH_ROCKSDB_ROOT=<prefix>) for "
                      "a ${SMASH_ROCKSDB_VERSION} build when reproducing paper results.")
    endif()
  elseif(ROCKSDB_LIB AND ROCKSDB_INCLUDE)
    message(STATUS "RocksDB ${_ROCKSDB_FOUND_VERSION} found (${ROCKSDB_LIB}) but is older "
                   "than the ${SMASH_ROCKSDB_VERSION} pin the benches target; skipping "
                   "RocksDB benches. Use -DSMASH_BUILD_BENCH_DEPS=ON to build the pin from "
                   "source, or -DSMASH_ROCKSDB_ROOT=<prefix> to point at an existing "
                   "${SMASH_ROCKSDB_VERSION}+ tree.")
  else()
    message(STATUS "RocksDB not found (optional)")
  endif()
endif()

# ── Heap page compression sampling ────────────────────────────────────────────

# SQLite variant
smash_add_standalone_bench(bench_heap_compress_sqlite bench_heap_compress.cpp)
target_compile_definitions(bench_heap_compress_sqlite PRIVATE HAS_SQLITE)
target_link_libraries(bench_heap_compress_sqlite PRIVATE sqlite3_static lz4_static libzstd_static)

# Generic variant (memcached/redis heap fill patterns)
smash_add_standalone_bench(bench_heap_compress bench_heap_compress.cpp)
target_link_libraries(bench_heap_compress PRIVATE lz4_static libzstd_static)

# RocksDB variant (only when using system RocksDB — deps build has its own)
if(SMASH_SYSTEM_ROCKSDB_OK)
  smash_add_standalone_bench(bench_heap_compress_rocksdb bench_heap_compress.cpp)
  target_compile_definitions(bench_heap_compress_rocksdb PRIVATE HAS_ROCKSDB)
  target_link_libraries(bench_heap_compress_rocksdb PRIVATE
    ${ROCKSDB_LIB} lz4_static libzstd_static)
  target_include_directories(bench_heap_compress_rocksdb PRIVATE ${ROCKSDB_INCLUDE})
endif()

# Remote process heap page sampler (macOS only — uses mach_vm_read)
if(APPLE)
  smash_add_standalone_bench(sample_heap_pages sample_heap_pages.cpp)
  target_link_libraries(sample_heap_pages PRIVATE lz4_static libzstd_static)
endif()

# Preloadable heap sampling library (SIGUSR2 to trigger)
# Works on macOS (DYLD_INSERT_LIBRARIES) and Linux (LD_PRELOAD)
add_library(sample_heap SHARED libsample_heap.cpp)
target_link_libraries(sample_heap PRIVATE lz4_static libzstd_static)
set_target_properties(sample_heap PROPERTIES OUTPUT_NAME sample_heap)

# ══════════════════════════════════════════════════════════════════════════════
# External Benchmark Dependencies (Redis, Memcached, RocksDB)
# ══════════════════════════════════════════════════════════════════════════════
# Built from source so we can:
#   1. Pin exact versions for reproducibility
#   2. Compile Redis with MALLOC=libc (default is jemalloc)
#   3. Ensure memcached links against system malloc
#   4. Build RocksDB as a static library with controlled flags
#
# Disable with: cmake .. -DSMASH_BUILD_BENCH=ON -DSMASH_BUILD_BENCH_DEPS=OFF

if(SMASH_BUILD_BENCH_DEPS)
  include(ExternalProject)

  include(ProcessorCount)
  ProcessorCount(NCPU)
  if(NCPU EQUAL 0)
    set(NCPU 4)
  endif()

  set(BENCH_DEPS_DIR ${CMAKE_CURRENT_BINARY_DIR}/deps)
  file(MAKE_DIRECTORY ${BENCH_DEPS_DIR}/bin)
  file(MAKE_DIRECTORY ${BENCH_DEPS_DIR}/lib)
  file(MAKE_DIRECTORY ${BENCH_DEPS_DIR}/include)

  # ── Redis 8.0.2 (MALLOC=libc — no jemalloc) ───────────────────────────────
  ExternalProject_Add(redis_build
    GIT_REPOSITORY https://github.com/redis/redis.git
    GIT_TAG 8.0.2
    GIT_SHALLOW TRUE
    PREFIX ${BENCH_DEPS_DIR}/redis
    CONFIGURE_COMMAND ""
    BUILD_IN_SOURCE TRUE
    BUILD_COMMAND ${SMASH_BENCH_SDK_ENV} make MALLOC=libc -j${NCPU}
    INSTALL_COMMAND
      ${CMAKE_COMMAND} -E copy <SOURCE_DIR>/src/redis-server ${BENCH_DEPS_DIR}/bin/redis-server-libc
      COMMAND ${CMAKE_COMMAND} -E copy <SOURCE_DIR>/src/redis-cli ${BENCH_DEPS_DIR}/bin/redis-cli
      COMMAND ${CMAKE_COMMAND} -E copy <SOURCE_DIR>/src/redis-benchmark ${BENCH_DEPS_DIR}/bin/redis-benchmark
    BUILD_BYPRODUCTS
      ${BENCH_DEPS_DIR}/bin/redis-server-libc
      ${BENCH_DEPS_DIR}/bin/redis-cli
      ${BENCH_DEPS_DIR}/bin/redis-benchmark
  )

  # ── Valkey 9.1.0 (MALLOC=libc — no jemalloc) ──────────────────────────────
  # Redis fork (Linux Foundation). Same plain Makefile build as redis: build
  # with MALLOC=libc so libsmash can interpose malloc. Binaries are named
  # valkey-* but stay RESP-compatible. Installed alongside redis so the harness
  # can drive it as a separate opt-in app.
  ExternalProject_Add(valkey_build
    GIT_REPOSITORY https://github.com/valkey-io/valkey.git
    GIT_TAG 9.1.0
    GIT_SHALLOW TRUE
    PREFIX ${BENCH_DEPS_DIR}/valkey
    CONFIGURE_COMMAND ""
    BUILD_IN_SOURCE TRUE
    BUILD_COMMAND ${SMASH_BENCH_SDK_ENV} make MALLOC=libc -j${NCPU}
    INSTALL_COMMAND
      ${CMAKE_COMMAND} -E copy <SOURCE_DIR>/src/valkey-server ${BENCH_DEPS_DIR}/bin/valkey-server-libc
      COMMAND ${CMAKE_COMMAND} -E copy <SOURCE_DIR>/src/valkey-cli ${BENCH_DEPS_DIR}/bin/valkey-cli
      COMMAND ${CMAKE_COMMAND} -E copy <SOURCE_DIR>/src/valkey-benchmark ${BENCH_DEPS_DIR}/bin/valkey-benchmark
    BUILD_BYPRODUCTS
      ${BENCH_DEPS_DIR}/bin/valkey-server-libc
      ${BENCH_DEPS_DIR}/bin/valkey-cli
      ${BENCH_DEPS_DIR}/bin/valkey-benchmark
  )

  # ── libevent (required by memcached) ───────────────────────────────────────
  # Prefer system/brew install; build from source only if missing.
  find_path(LIBEVENT_INCLUDE_DIR event.h
    HINTS /opt/homebrew/include /usr/include /usr/local/include)
  find_library(LIBEVENT_LIBRARY event
    HINTS /opt/homebrew/lib /usr/lib /usr/lib64 /usr/local/lib)

  if(LIBEVENT_INCLUDE_DIR AND LIBEVENT_LIBRARY)
    # Use system libevent — derive prefix from library path for configure
    get_filename_component(LIBEVENT_PREFIX "${LIBEVENT_LIBRARY}" DIRECTORY)
    get_filename_component(LIBEVENT_PREFIX "${LIBEVENT_PREFIX}" DIRECTORY)
    set(_MEMCACHED_LIBEVENT_DEP "")
    message(STATUS "libevent found: ${LIBEVENT_LIBRARY}")
  else()
    # Build libevent from source
    ExternalProject_Add(libevent_build
      GIT_REPOSITORY https://github.com/libevent/libevent.git
      GIT_TAG release-2.1.12-stable
      GIT_SHALLOW TRUE
      PREFIX ${BENCH_DEPS_DIR}/libevent
      CMAKE_ARGS
        -DCMAKE_BUILD_TYPE=Release
        -DCMAKE_INSTALL_PREFIX=${BENCH_DEPS_DIR}
        -DEVENT__DISABLE_TESTS=ON
        -DEVENT__DISABLE_SAMPLES=ON
        -DEVENT__DISABLE_BENCHMARK=ON
      BUILD_COMMAND ${CMAKE_COMMAND} --build . -- -j${NCPU}
      INSTALL_COMMAND ${CMAKE_COMMAND} --build . --target install
      BUILD_BYPRODUCTS
        ${BENCH_DEPS_DIR}/lib/libevent.a
        ${BENCH_DEPS_DIR}/include/event.h
    )
    set(LIBEVENT_PREFIX ${BENCH_DEPS_DIR})
    set(_MEMCACHED_LIBEVENT_DEP libevent_build)
    message(STATUS "Will build libevent from source")
  endif()

  # ── Memcached 1.6.34 ──────────────────────────────────────────────────────
  # Uses its own slab allocator on top of malloc; we just need system malloc
  # underneath (not jemalloc).  Requires autotools + libevent.
  find_program(AUTORECONF autoreconf)
  if(AUTORECONF)
    ExternalProject_Add(memcached_build
      GIT_REPOSITORY https://github.com/memcached/memcached.git
      GIT_TAG 1.6.34
      GIT_SHALLOW TRUE
      PREFIX ${BENCH_DEPS_DIR}/memcached
      CONFIGURE_COMMAND
        cd <SOURCE_DIR> && ./autogen.sh &&
        ${SMASH_BENCH_SDK_ENV} <SOURCE_DIR>/configure
          --prefix=${BENCH_DEPS_DIR}
          --with-libevent=${LIBEVENT_PREFIX}
          --disable-extstore
          --disable-tls
      BUILD_IN_SOURCE TRUE
      BUILD_COMMAND ${SMASH_BENCH_SDK_ENV} make -j${NCPU}
      INSTALL_COMMAND
        ${CMAKE_COMMAND} -E copy <SOURCE_DIR>/memcached ${BENCH_DEPS_DIR}/bin/memcached
      BUILD_BYPRODUCTS ${BENCH_DEPS_DIR}/bin/memcached
    )
    if(_MEMCACHED_LIBEVENT_DEP)
      add_dependencies(memcached_build ${_MEMCACHED_LIBEVENT_DEP})
    endif()
    message(STATUS "Will build memcached 1.6.34 from source")
  else()
    message(WARNING "autoreconf not found — cannot build memcached.  Install: brew install autoconf automake")
  endif()

  # ── LZ4 1.9.4 (static, installed into BENCH_DEPS_DIR) ─────────────────────
  # RocksDB's rocksdb-lz4 / smash+lz4 configs need LZ4 linked in. System
  # lz4-devel isn't guaranteed on the runner, so we vendor it alongside the
  # other bench deps. zstd is vendored the same way below; both are also
  # passed to rocksdb_build via CMAKE_PREFIX_PATH so RocksDB's
  # find_package / find_library calls locate them.
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(_BENCH_LZ4_STATIC ${BENCH_DEPS_DIR}/lib/liblz4.a)
    set(_BENCH_ZSTD_STATIC ${BENCH_DEPS_DIR}/lib/libzstd.a)
  else()
    set(_BENCH_LZ4_STATIC ${BENCH_DEPS_DIR}/lib/liblz4.a)
    set(_BENCH_ZSTD_STATIC ${BENCH_DEPS_DIR}/lib/libzstd.a)
  endif()

  ExternalProject_Add(bench_lz4_build
    GIT_REPOSITORY https://github.com/lz4/lz4.git
    GIT_TAG v1.9.4
    GIT_SHALLOW TRUE
    PREFIX ${BENCH_DEPS_DIR}/lz4
    SOURCE_SUBDIR build/cmake
    LIST_SEPARATOR |
    CMAKE_ARGS
      -DCMAKE_BUILD_TYPE=Release
      -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
      -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
      -DCMAKE_OSX_SYSROOT=${CMAKE_OSX_SYSROOT}
      "-DCMAKE_OSX_ARCHITECTURES=${SMASH_BENCH_OSX_ARCHS}"
      # lz4 v1.9.4 declares cmake_minimum_required(2.8.12); CMake >= 4 dropped
      # compatibility with < 3.5.
      -DCMAKE_POLICY_VERSION_MINIMUM=3.5
      -DBUILD_STATIC_LIBS=ON
      -DBUILD_SHARED_LIBS=OFF
      -DLZ4_BUILD_CLI=OFF
      -DLZ4_BUILD_LEGACY_LZ4C=OFF
      -DCMAKE_INSTALL_PREFIX=${BENCH_DEPS_DIR}
      -DCMAKE_INSTALL_LIBDIR=lib
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    BUILD_COMMAND ${CMAKE_COMMAND} --build . -- -j${NCPU}
    INSTALL_COMMAND ${CMAKE_COMMAND} --build . --target install
    BUILD_BYPRODUCTS
      ${_BENCH_LZ4_STATIC}
      ${BENCH_DEPS_DIR}/include/lz4.h
  )

  # ── Zstandard 1.5.6 (static, installed into BENCH_DEPS_DIR) ───────────────
  ExternalProject_Add(bench_zstd_build
    GIT_REPOSITORY https://github.com/facebook/zstd.git
    GIT_TAG v1.5.6
    GIT_SHALLOW TRUE
    PREFIX ${BENCH_DEPS_DIR}/zstd
    SOURCE_SUBDIR build/cmake
    LIST_SEPARATOR |
    CMAKE_ARGS
      -DCMAKE_BUILD_TYPE=Release
      -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
      -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
      -DCMAKE_OSX_SYSROOT=${CMAKE_OSX_SYSROOT}
      "-DCMAKE_OSX_ARCHITECTURES=${SMASH_BENCH_OSX_ARCHS}"
      -DZSTD_BUILD_STATIC=ON
      -DZSTD_BUILD_SHARED=OFF
      -DZSTD_BUILD_PROGRAMS=OFF
      -DZSTD_BUILD_TESTS=OFF
      -DZSTD_BUILD_CONTRIB=OFF
      -DCMAKE_INSTALL_PREFIX=${BENCH_DEPS_DIR}
      -DCMAKE_INSTALL_LIBDIR=lib
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    BUILD_COMMAND ${CMAKE_COMMAND} --build . -- -j${NCPU}
    INSTALL_COMMAND ${CMAKE_COMMAND} --build . --target install
    BUILD_BYPRODUCTS
      ${_BENCH_ZSTD_STATIC}
      ${BENCH_DEPS_DIR}/include/zstd.h
  )

  # ── RocksDB 9.8.4 (static library, no gflags/tools) ───────────────────────
  # -Wno-error=nontrivial-memcall is Clang-only; GCC doesn't recognize it.
  # Conditionally include the entire argument to avoid passing an empty
  # -DCMAKE_CXX_FLAGS= which would clear any existing flags.
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(_ROCKSDB_CXX_FLAGS_ARG "-DCMAKE_CXX_FLAGS=-Wno-error=nontrivial-memcall")
  else()
    set(_ROCKSDB_CXX_FLAGS_ARG "")
  endif()

  ExternalProject_Add(rocksdb_build
    GIT_REPOSITORY https://github.com/facebook/rocksdb.git
    GIT_TAG v${SMASH_ROCKSDB_VERSION}   # see SMASH_ROCKSDB_VERSION (paper pin)
    GIT_SHALLOW TRUE
    PREFIX ${BENCH_DEPS_DIR}/rocksdb
    DEPENDS bench_lz4_build bench_zstd_build
    LIST_SEPARATOR |
    CMAKE_ARGS
      -DCMAKE_BUILD_TYPE=Release
      -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
      -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
      -DCMAKE_OSX_SYSROOT=${CMAKE_OSX_SYSROOT}
      "-DCMAKE_OSX_ARCHITECTURES=${SMASH_BENCH_OSX_ARCHS}"
      ${_ROCKSDB_CXX_FLAGS_ARG}
      -DWITH_TESTS=OFF
      -DWITH_BENCHMARK_TOOLS=OFF
      -DWITH_TOOLS=OFF
      -DWITH_GFLAGS=OFF
      -DWITH_LZ4=ON
      -DWITH_ZSTD=ON
      -DCMAKE_PREFIX_PATH=${BENCH_DEPS_DIR}
      -Dlz4_ROOT=${BENCH_DEPS_DIR}
      -Dzstd_ROOT=${BENCH_DEPS_DIR}
      -DROCKSDB_BUILD_SHARED=OFF
      -DCMAKE_INSTALL_PREFIX=${BENCH_DEPS_DIR}
      -DCMAKE_INSTALL_LIBDIR=lib
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    BUILD_COMMAND ${CMAKE_COMMAND} --build . -- -j${NCPU}
    INSTALL_COMMAND ${CMAKE_COMMAND} --build . --target install
    BUILD_BYPRODUCTS
      ${BENCH_DEPS_DIR}/lib/librocksdb.a
      ${BENCH_DEPS_DIR}/include/rocksdb/db.h
  )

  # ── Snappy 1.2.1 (static, vendored) ──────────────────────────────────────────
  # LevelDB compresses SST blocks with Snappy when it detects the library at
  # configure time (check_library_exists(snappy ...)). We vendor it so LevelDB
  # is built WITH compression enabled (matching a realistic deployment) and the
  # build is self-contained (no reliance on a system libsnappy). Static + PIC so
  # it links cleanly into bench_leveldb.
  ExternalProject_Add(bench_snappy_build
    GIT_REPOSITORY https://github.com/google/snappy.git
    GIT_TAG 1.2.1
    GIT_SHALLOW TRUE
    PREFIX ${BENCH_DEPS_DIR}/snappy
    LIST_SEPARATOR |
    CMAKE_ARGS
      -DCMAKE_BUILD_TYPE=Release
      -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
      -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
      -DCMAKE_OSX_SYSROOT=${CMAKE_OSX_SYSROOT}
      "-DCMAKE_OSX_ARCHITECTURES=${SMASH_BENCH_OSX_ARCHS}"
      -DBUILD_SHARED_LIBS=OFF
      -DSNAPPY_BUILD_TESTS=OFF
      -DSNAPPY_BUILD_BENCHMARKS=OFF
      -DSNAPPY_INSTALL=ON
      -DCMAKE_INSTALL_PREFIX=${BENCH_DEPS_DIR}
      -DCMAKE_INSTALL_LIBDIR=lib
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    BUILD_COMMAND ${CMAKE_COMMAND} --build . -- -j${NCPU}
    INSTALL_COMMAND ${CMAKE_COMMAND} --build . --target install
    BUILD_BYPRODUCTS
      ${BENCH_DEPS_DIR}/lib/libsnappy.a
      ${BENCH_DEPS_DIR}/include/snappy.h
  )
  message(STATUS "  Snappy 1.2.1 (static, vendored for LevelDB)")

  # ── LevelDB 1.23 (static library) ──────────────────────────────────────────
  # LevelDB uses an arena allocator over malloc — Smash should work.
  # Built WITH vendored Snappy: CMAKE_PREFIX_PATH points its
  # check_library_exists(snappy ...) probe at BENCH_DEPS_DIR so HAVE_SNAPPY is
  # set and SST blocks are Snappy-compressed.
  ExternalProject_Add(leveldb_build
    GIT_REPOSITORY https://github.com/google/leveldb.git
    GIT_TAG 1.23
    GIT_SHALLOW TRUE
    PREFIX ${BENCH_DEPS_DIR}/leveldb
    DEPENDS bench_snappy_build
    LIST_SEPARATOR |
    CMAKE_ARGS
      -DCMAKE_BUILD_TYPE=Release
      -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
      -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
      -DCMAKE_OSX_SYSROOT=${CMAKE_OSX_SYSROOT}
      "-DCMAKE_OSX_ARCHITECTURES=${SMASH_BENCH_OSX_ARCHS}"
      -DLEVELDB_BUILD_TESTS=OFF
      -DLEVELDB_BUILD_BENCHMARKS=OFF
      -DBUILD_SHARED_LIBS=OFF
      -DCMAKE_PREFIX_PATH=${BENCH_DEPS_DIR}
      -DCMAKE_LIBRARY_PATH=${BENCH_DEPS_DIR}/lib
      -DCMAKE_INCLUDE_PATH=${BENCH_DEPS_DIR}/include
      # check_library_exists(snappy ...) links a probe via try_compile, which
      # honors CMAKE_EXE_LINKER_FLAGS (not CMAKE_LIBRARY_PATH) — point it at our
      # vendored libsnappy.a so HAVE_SNAPPY is detected without a system libsnappy.
      "-DCMAKE_EXE_LINKER_FLAGS=-L${BENCH_DEPS_DIR}/lib"
      -DCMAKE_INSTALL_PREFIX=${BENCH_DEPS_DIR}
      -DCMAKE_INSTALL_LIBDIR=lib
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    BUILD_COMMAND ${CMAKE_COMMAND} --build . -- -j${NCPU}
    INSTALL_COMMAND ${CMAKE_COMMAND} --build . --target install
    BUILD_BYPRODUCTS
      ${BENCH_DEPS_DIR}/lib/libleveldb.a
      ${BENCH_DEPS_DIR}/include/leveldb/db.h
  )
  message(STATUS "  LevelDB 1.23 (static, Snappy-enabled)")

  # ── Varnish 7.6.0 (HTTP cache, supports -s malloc backend) ────────────────
  # Varnish with -s malloc uses system malloc for cache storage.
  # Requires: autotools, pcre2, python3
  find_program(AUTORECONF autoreconf)
  find_library(PCRE2_LIB pcre2-8 HINTS /usr/lib /usr/lib64 /usr/local/lib)
  find_path(PCRE2_INCLUDE pcre2.h HINTS /usr/include /usr/local/include)
  if(AUTORECONF AND PCRE2_LIB)
    # Varnish 7.6.0 predates C23 and uses `bool` as a struct member name, so it
    # needs a pre-C23 dialect on toolchains that now default to C23. Its
    # configure also appends -Werror unconditionally, and clang (unlike gcc)
    # diagnoses `char buf[lim + sizeof(x)]` with a const int lim as a folded
    # VLA (cache_vpi.c). Disable that clang-only warning; a disabled warning is
    # not promoted by the later -Werror.
    set(VARNISH_CFLAGS "-std=gnu17")
    if(CMAKE_C_COMPILER_ID MATCHES "Clang")
      set(VARNISH_CFLAGS "${VARNISH_CFLAGS} -Wno-gnu-folding-constant")
    endif()
    ExternalProject_Add(varnish_build
      GIT_REPOSITORY https://github.com/varnishcache/varnish-cache.git
      GIT_TAG varnish-7.6.0
      GIT_SHALLOW TRUE
      PREFIX ${BENCH_DEPS_DIR}/varnish
      CONFIGURE_COMMAND
        cd <SOURCE_DIR> && ./autogen.sh &&
        ${SMASH_BENCH_SDK_ENV} ${CMAKE_COMMAND} -E env CC=${CMAKE_C_COMPILER} "CFLAGS=${VARNISH_CFLAGS}"
        <SOURCE_DIR>/configure
          --prefix=${BENCH_DEPS_DIR}
          --disable-dependency-tracking
          --without-jemalloc
      BUILD_IN_SOURCE TRUE
      BUILD_COMMAND ${SMASH_BENCH_SDK_ENV} make -j${NCPU}
      INSTALL_COMMAND make install
      BUILD_BYPRODUCTS
        ${BENCH_DEPS_DIR}/sbin/varnishd
        ${BENCH_DEPS_DIR}/bin/varnishstat
    )
    message(STATUS "  Varnish 7.6.0 (without jemalloc)")
  else()
    if(NOT AUTORECONF)
      message(STATUS "  Varnish: skipped (autoreconf not found)")
    elseif(NOT PCRE2_LIB)
      message(STATUS "  Varnish: skipped (pcre2 not found - install pcre2-devel)")
    endif()
  endif()

  # ── MongoDB 8.0 (community server, without tcmalloc) ──────────────────────
  # MongoDB bundles tcmalloc by default. Build with --allocator=system for
  # system malloc. This is a large build (~30 min).
  # Also build a tcmalloc variant for baseline comparison.
  # MongoDB 8.0 requires Python 3.9-3.11 (pkg_resources incompatible with 3.12+)
  find_program(PYTHON3_11 python3.11)
  find_program(PYTHON3_10 python3.10)
  find_program(PYTHON3_9 python3.9)
  if(PYTHON3_11)
    set(MONGODB_PYTHON ${PYTHON3_11})
  elseif(PYTHON3_10)
    set(MONGODB_PYTHON ${PYTHON3_10})
  elseif(PYTHON3_9)
    set(MONGODB_PYTHON ${PYTHON3_9})
  else()
    set(MONGODB_PYTHON "")
  endif()
  # MongoDB's buildscripts/scons.py needs poetry in the interpreter it runs
  # under; without it the build step dies with "No module named 'poetry'".
  # Provision it ourselves into a private venv rather than pip-installing into
  # the user's interpreter — Homebrew/system Pythons are PEP-668
  # "externally managed" and refuse `pip install` without
  # --break-system-packages. The venv is created once and reused.
  if(MONGODB_PYTHON)
    execute_process(COMMAND ${MONGODB_PYTHON} -c "import poetry"
                    RESULT_VARIABLE _POETRY_MISSING OUTPUT_QUIET ERROR_QUIET)
    if(NOT _POETRY_MISSING EQUAL 0)
      set(MONGODB_VENV ${BENCH_DEPS_DIR}/mongodb-venv)
      set(MONGODB_VENV_PYTHON ${MONGODB_VENV}/bin/python)
      execute_process(COMMAND ${MONGODB_VENV_PYTHON} -c "import poetry"
                      RESULT_VARIABLE _VENV_POETRY_MISSING OUTPUT_QUIET ERROR_QUIET)
      if(NOT _VENV_POETRY_MISSING EQUAL 0)
        message(STATUS "  MongoDB: installing poetry into ${MONGODB_VENV} ...")
        execute_process(COMMAND ${MONGODB_PYTHON} -m venv ${MONGODB_VENV}
                        RESULT_VARIABLE _VENV_RC OUTPUT_QUIET ERROR_QUIET)
        if(_VENV_RC EQUAL 0)
          execute_process(
            COMMAND ${MONGODB_VENV_PYTHON} -m pip install --quiet
                    "poetry==1.5.1" "packaging" "setuptools"
            RESULT_VARIABLE _PIP_RC
          )
        else()
          set(_PIP_RC 1)
        endif()
      else()
        set(_PIP_RC 0)
      endif()
      if(_PIP_RC EQUAL 0)
        set(MONGODB_PYTHON ${MONGODB_VENV_PYTHON})
        message(STATUS "  MongoDB: using ${MONGODB_PYTHON} (poetry installed)")
      else()
        message(STATUS "  MongoDB: skipped (could not install poetry for ${MONGODB_PYTHON})")
        set(MONGODB_PYTHON "")
        set(MONGODB_SKIP_REPORTED TRUE)
      endif()
    endif()
  endif()
  if(MONGODB_PYTHON)
    # MongoDB without tcmalloc (uses system malloc)
    # Patch SConstruct to use packaging.version instead of deprecated pkg_resources.
    # Uses --linker=bfd (system ld) since lld/gold may not be installed.
    ExternalProject_Add(mongodb_build
      EXCLUDE_FROM_ALL TRUE
      GIT_REPOSITORY https://github.com/mongodb/mongo.git
      GIT_TAG r8.0.0
      GIT_SHALLOW TRUE
      PREFIX ${BENCH_DEPS_DIR}/mongodb
      CONFIGURE_COMMAND ""
      PATCH_COMMAND
        sed -i.bak "s/from pkg_resources import parse_version/from packaging.version import parse as parse_version/" <SOURCE_DIR>/SConstruct
      BUILD_IN_SOURCE TRUE
      # scons refuses to build until its poetry-managed build deps are in sync
      # ("Detected one or more packages are out of date"). Run the sync it asks
      # for; the null keyring backend keeps poetry from blocking on a keychain
      # prompt in a non-interactive build.
      BUILD_COMMAND
        ${CMAKE_COMMAND} -E env PYTHON_KEYRING_BACKEND=keyring.backends.null.Keyring
          ${MONGODB_PYTHON} -m poetry install --no-root --sync
      COMMAND
        ${MONGODB_PYTHON} buildscripts/scons.py
          DESTDIR=${BENCH_DEPS_DIR}/mongodb-system
          --allocator=system
          --linker=bfd
          --disable-warnings-as-errors
          -j${NCPU}
          install-mongod
      INSTALL_COMMAND
        ${CMAKE_COMMAND} -E copy
          <SOURCE_DIR>/build/install/bin/mongod
          ${BENCH_DEPS_DIR}/bin/mongod-system
      BUILD_BYPRODUCTS ${BENCH_DEPS_DIR}/bin/mongod-system
    )

    # MongoDB with tcmalloc (baseline for comparison)
    ExternalProject_Add(mongodb_tcmalloc_build
      EXCLUDE_FROM_ALL TRUE
      GIT_REPOSITORY https://github.com/mongodb/mongo.git
      GIT_TAG r8.0.0
      GIT_SHALLOW TRUE
      PREFIX ${BENCH_DEPS_DIR}/mongodb-tcmalloc
      CONFIGURE_COMMAND ""
      PATCH_COMMAND
        sed -i.bak "s/from pkg_resources import parse_version/from packaging.version import parse as parse_version/" <SOURCE_DIR>/SConstruct
      BUILD_IN_SOURCE TRUE
      # scons refuses to build until its poetry-managed build deps are in sync
      # ("Detected one or more packages are out of date"). Run the sync it asks
      # for; the null keyring backend keeps poetry from blocking on a keychain
      # prompt in a non-interactive build.
      BUILD_COMMAND
        ${CMAKE_COMMAND} -E env PYTHON_KEYRING_BACKEND=keyring.backends.null.Keyring
          ${MONGODB_PYTHON} -m poetry install --no-root --sync
      COMMAND
        ${MONGODB_PYTHON} buildscripts/scons.py
          DESTDIR=${BENCH_DEPS_DIR}/mongodb-tcmalloc
          --allocator=tcmalloc-google
          --linker=bfd
          --disable-warnings-as-errors
          -j${NCPU}
          install-mongod
      INSTALL_COMMAND
        ${CMAKE_COMMAND} -E copy
          <SOURCE_DIR>/build/install/bin/mongod
          ${BENCH_DEPS_DIR}/bin/mongod-tcmalloc
      BUILD_BYPRODUCTS ${BENCH_DEPS_DIR}/bin/mongod-tcmalloc
    )
    message(STATUS "  MongoDB 8.0 (system malloc) using ${MONGODB_PYTHON}")
    message(STATUS "  MongoDB 8.0 (tcmalloc, baseline)")
  elseif(NOT MONGODB_SKIP_REPORTED)
    message(STATUS "  MongoDB: skipped (python3.9/3.10/3.11 not found)")
  endif()

  # ── bench_leveldb linked against our static LevelDB ─────────────────────────
  add_executable(bench_leveldb bench_leveldb.cpp)
  _get_filtered_dir_compile_options(leveldb_opts)
  set_target_properties(bench_leveldb PROPERTIES COMPILE_OPTIONS "${leveldb_opts}")
  target_include_directories(bench_leveldb PRIVATE ${BENCH_DEPS_DIR}/include)
  # LevelDB was built with Snappy, so libleveldb.a has unresolved snappy_*
  # symbols; link the vendored static libsnappy.a after it.
  target_link_libraries(bench_leveldb PRIVATE
    ${BENCH_DEPS_DIR}/lib/libleveldb.a
    ${BENCH_DEPS_DIR}/lib/libsnappy.a
    Threads::Threads
    ${CMAKE_DL_LIBS}
  )
  add_dependencies(bench_leveldb leveldb_build bench_snappy_build)

  # ── Export paths ───────────────────────────────────────────────────────────
  set(BENCH_REDIS_SERVER   ${BENCH_DEPS_DIR}/bin/redis-server-libc CACHE PATH "")
  set(BENCH_REDIS_CLI      ${BENCH_DEPS_DIR}/bin/redis-cli         CACHE PATH "")
  set(BENCH_REDIS_BENCHMARK ${BENCH_DEPS_DIR}/bin/redis-benchmark  CACHE PATH "")
  set(BENCH_MEMCACHED      ${BENCH_DEPS_DIR}/bin/memcached         CACHE PATH "")
  set(BENCH_ROCKSDB_LIB    ${BENCH_DEPS_DIR}/lib/librocksdb.a      CACHE PATH "")
  set(BENCH_ROCKSDB_INCLUDE ${BENCH_DEPS_DIR}/include              CACHE PATH "")
  set(BENCH_LEVELDB_LIB    ${BENCH_DEPS_DIR}/lib/libleveldb.a      CACHE PATH "")
  set(BENCH_LEVELDB_INCLUDE ${BENCH_DEPS_DIR}/include              CACHE PATH "")
  set(BENCH_VARNISHD       ${BENCH_DEPS_DIR}/sbin/varnishd         CACHE PATH "")
  set(BENCH_MONGOD_SYSTEM  ${BENCH_DEPS_DIR}/bin/mongod-system     CACHE PATH "")
  set(BENCH_MONGOD_TCMALLOC ${BENCH_DEPS_DIR}/bin/mongod-tcmalloc  CACHE PATH "")

  # Convenience target: `make bench_deps`
  add_custom_target(bench_deps)
  add_dependencies(bench_deps redis_build rocksdb_build leveldb_build)
  if(TARGET libevent_build)
    add_dependencies(bench_deps libevent_build)
  endif()
  if(TARGET memcached_build)
    add_dependencies(bench_deps memcached_build)
  endif()
  if(TARGET tcmalloc_build)
    add_dependencies(bench_deps tcmalloc_build)
  endif()
  if(TARGET varnish_build)
    add_dependencies(bench_deps varnish_build)
  endif()
  if(TARGET mongodb_build)
    add_dependencies(bench_deps mongodb_build mongodb_tcmalloc_build)
  endif()

  message(STATUS "Benchmark dependencies (${BENCH_DEPS_DIR}):")
  message(STATUS "  Redis 8.0.2 (libc malloc)")
  message(STATUS "  Memcached 1.6.34")
  message(STATUS "  RocksDB 9.8.4 (static)")
  if(TARGET tcmalloc_build)
    if(BAZEL_PROG AND _GLIBC_VERSION VERSION_GREATER_EQUAL "2.35")
      message(STATUS "  google/tcmalloc (latest, via Bazel 8.2.1)")
    else()
      message(STATUS "  gperftools/tcmalloc 2.16")
    endif()
  endif()

  # ── bench_rocksdb linked against our static RocksDB ────────────────────────
  add_executable(bench_rocksdb_builtin bench_rocksdb.cpp)
  _get_filtered_dir_compile_options(rocksdb_builtin_opts)
  set_target_properties(bench_rocksdb_builtin PROPERTIES COMPILE_OPTIONS "${rocksdb_builtin_opts}")
  target_include_directories(bench_rocksdb_builtin PRIVATE ${BENCH_DEPS_DIR}/include)
  target_link_libraries(bench_rocksdb_builtin PRIVATE
    ${BENCH_DEPS_DIR}/lib/librocksdb.a
    Threads::Threads
    ${CMAKE_DL_LIBS}
  )
  # RocksDB static lib needs its transitive compression dependencies.
  # LZ4 and zstd are vendored (bench_lz4_build / bench_zstd_build); link
  # the static archives we just installed. zlib/bz2/snappy are optional
  # system libs — include when present.
  find_package(ZLIB)
  find_library(BZ2_LIB bz2)
  find_library(SNAPPY_LIB snappy)
  target_link_libraries(bench_rocksdb_builtin PRIVATE
    ${_BENCH_LZ4_STATIC}
    ${_BENCH_ZSTD_STATIC}
  )
  if(ZLIB_FOUND)
    target_link_libraries(bench_rocksdb_builtin PRIVATE ZLIB::ZLIB)
  endif()
  if(BZ2_LIB)
    target_link_libraries(bench_rocksdb_builtin PRIVATE ${BZ2_LIB})
  endif()
  if(SNAPPY_LIB)
    target_link_libraries(bench_rocksdb_builtin PRIVATE ${SNAPPY_LIB})
  endif()
  add_dependencies(bench_rocksdb_builtin rocksdb_build)

endif() # SMASH_BUILD_BENCH_DEPS

# ── Configure benchmark scripts with dependency paths ────────────────────────
# Prefer built binaries; fall back to system PATH.
if(DEFINED BENCH_REDIS_SERVER)
  set(REDIS_SERVER_PATH ${BENCH_REDIS_SERVER})
else()
  set(REDIS_SERVER_PATH "redis-server")
endif()
if(DEFINED BENCH_MEMCACHED)
  set(MEMCACHED_PATH ${BENCH_MEMCACHED})
else()
  set(MEMCACHED_PATH "memcached")
endif()
