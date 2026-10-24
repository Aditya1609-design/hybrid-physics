# Third-party dependencies, pinned to tag archives with SHA256 hashes so every machine
# builds the same sources. Archives are kept in HP_DEPS_CACHE_DIR, so a clean build
# re-extracts them instead of downloading again.

include(FetchContent)

# Give extracted files the extraction time, so rebuilds after a re-fetch are correct.
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

# Eigen: download only and wrap the headers in our own target. Eigen's CMake project
# configures its tests and BLAS checks, which we do not need.
FetchContent_Declare(eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    URL_HASH SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72
    DOWNLOAD_DIR ${HP_DEPS_CACHE_DIR}
    SOURCE_SUBDIR do-not-configure)

FetchContent_Declare(doctest
    URL https://github.com/doctest/doctest/archive/refs/tags/v2.4.12.tar.gz
    URL_HASH SHA256=73381c7aa4dee704bd935609668cf41880ea7f19fa0504a200e13b74999c2d70
    DOWNLOAD_DIR ${HP_DEPS_CACHE_DIR})

FetchContent_Declare(nlohmann_json
    URL https://github.com/nlohmann/json/archive/refs/tags/v3.11.3.tar.gz
    URL_HASH SHA256=0d8ef5af7f9794e3263480193c491549b2ba6cc74bb018906202ada498a79406
    DOWNLOAD_DIR ${HP_DEPS_CACHE_DIR})

set(DOCTEST_WITH_TESTS OFF CACHE BOOL "" FORCE)
set(DOCTEST_WITH_MAIN_IN_STATIC_LIB OFF CACHE BOOL "" FORCE)
set(DOCTEST_NO_INSTALL ON CACHE BOOL "" FORCE)
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
set(JSON_Install OFF CACHE BOOL "" FORCE)

# doctest 2.4.12 declares cmake_minimum_required(3.5), which CMake 4 reports as deprecated.
# Raise the effective minimum for the dependencies only (CMake < 4 ignores this variable).
set(CMAKE_POLICY_VERSION_MINIMUM 3.10)
FetchContent_MakeAvailable(eigen doctest nlohmann_json)
unset(CMAKE_POLICY_VERSION_MINIMUM)

add_library(hp_eigen INTERFACE)
add_library(hp::eigen ALIAS hp_eigen)
target_include_directories(hp_eigen SYSTEM INTERFACE ${eigen_SOURCE_DIR})

# Mark third-party headers as system headers so our strict warnings skip them.
foreach(_hp_dep doctest nlohmann_json)
    get_target_property(_hp_inc ${_hp_dep} INTERFACE_INCLUDE_DIRECTORIES)
    set_target_properties(${_hp_dep} PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_hp_inc}")
endforeach()
