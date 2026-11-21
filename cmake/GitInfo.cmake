# Writes the current git commit and dirty flag into a C++ source file.
# Run in script mode on every build:
#   cmake -DGIT_EXECUTABLE=... -DSRC_DIR=... -DOUT=... -P GitInfo.cmake
# The output is only rewritten when its content changes, so unchanged builds stay incremental.

set(_commit "unknown")
set(_dirty "false")

if(GIT_EXECUTABLE AND EXISTS "${GIT_EXECUTABLE}")
    execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${SRC_DIR}" rev-parse HEAD
        OUTPUT_VARIABLE _out OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE _rc ERROR_QUIET)
    if(_rc EQUAL 0 AND _out MATCHES "^[0-9a-f]+$")
        set(_commit "${_out}")
        # Only changes to tracked files count; local outputs that git ignores or does not
        # track do not make the build dirty.
        execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${SRC_DIR}" status --porcelain
                --untracked-files=no
            OUTPUT_VARIABLE _status OUTPUT_STRIP_TRAILING_WHITESPACE
            RESULT_VARIABLE _rc ERROR_QUIET)
        if(NOT _rc EQUAL 0 OR NOT _status STREQUAL "")
            set(_dirty "true")
        endif()
    endif()
endif()

set(_content "// Generated at build time by cmake/GitInfo.cmake. Do not edit.
#include \"hp/version.hpp\"

namespace hp {

const char* git_commit() { return \"${_commit}\"; }
bool git_dirty() { return ${_dirty}; }

}  // namespace hp
")

file(WRITE "${OUT}.tmp" "${_content}")
execute_process(COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${OUT}.tmp" "${OUT}")
file(REMOVE "${OUT}.tmp")
