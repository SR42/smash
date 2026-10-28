# cmake -DPATCH_DIR=<dir> -DPATCH_DIGEST=<digest> [-DGIT=<git>] -P apply_patches.cmake
#
# Applies every *.patch in PATCH_DIR, in name order, to the current directory
# (a FetchContent PATCH_COMMAND runs in the source tree). In a git checkout the
# tracked files are first restored to the pinned commit, so a patch set that
# changed applies over a tree an older one already modified. A patch that
# applies neither way is an error: the pin and the patch no longer match.
#
# The patch step runs on every configure (after the git update step), so a tree
# already patched with this PATCH_DIGEST is left untouched: rewriting the files
# would recompile and relink smash each time.
if(NOT GIT)
  set(GIT git)
endif()
set(_marker "${CMAKE_CURRENT_SOURCE_DIR}/.smash-patch-digest")
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.git")
  set(_marker "${CMAKE_CURRENT_SOURCE_DIR}/.git/smash-patch-digest")
endif()
if(PATCH_DIGEST AND EXISTS "${_marker}")
  file(READ "${_marker}" _done)
  if(_done STREQUAL PATCH_DIGEST)
    return()
  endif()
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.git")
  execute_process(COMMAND "${GIT}" checkout -- . RESULT_VARIABLE _rc)
  if(NOT _rc EQUAL 0)
    message(FATAL_ERROR "could not restore ${CMAKE_CURRENT_SOURCE_DIR} before patching")
  endif()
endif()
file(GLOB _patches "${PATCH_DIR}/*.patch")
list(SORT _patches)
foreach(_p IN LISTS _patches)
  get_filename_component(_name "${_p}" NAME)
  execute_process(COMMAND "${GIT}" apply --reverse --check "${_p}"
                  RESULT_VARIABLE _applied OUTPUT_QUIET ERROR_QUIET)
  if(_applied EQUAL 0)   # not a git checkout, and patched already
    message(STATUS "${_name}: already applied")
    continue()
  endif()
  execute_process(COMMAND "${GIT}" apply "${_p}" RESULT_VARIABLE _rc)
  if(NOT _rc EQUAL 0)
    message(FATAL_ERROR "${_name} does not apply to ${CMAKE_CURRENT_SOURCE_DIR}")
  endif()
  message(STATUS "${_name}: applied")
endforeach()
if(PATCH_DIGEST)
  file(WRITE "${_marker}" "${PATCH_DIGEST}")
endif()
