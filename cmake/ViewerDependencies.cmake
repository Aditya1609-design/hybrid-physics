# raylib for the viewer only. Included only when HP_BUILD_VIEWER is on, so builds without
# the viewer never download it. Pinned to the 5.5 tag archive and checked by SHA256.

include(FetchContent)

# Give extracted files the extraction time (set again here: include() scopes policies).
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

FetchContent_Declare(raylib
    URL https://github.com/raysan5/raylib/archive/refs/tags/5.5.tar.gz
    URL_HASH SHA256=aea98ecf5bc5c5e0b789a76de0083a21a70457050ea4cc2aec7566935f5e258e
    DOWNLOAD_NAME raylib-5.5.tar.gz
    DOWNLOAD_DIR ${HP_DEPS_CACHE_DIR})

# Desktop OpenGL with the bundled GLFW and no examples. Audio stays on (the default):
# turning it off needs CUSTOMIZE_BUILD, which makes raylib 5.5 warn about redefined macros.
set(PLATFORM "Desktop" CACHE STRING "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(USE_EXTERNAL_GLFW "OFF" CACHE STRING "" FORCE)

# raylib 5.5 declares cmake_minimum_required(3.5), which CMake 4 rejects without this.
set(CMAKE_POLICY_VERSION_MINIMUM 3.10)
FetchContent_MakeAvailable(raylib)
unset(CMAKE_POLICY_VERSION_MINIMUM)

# Mark raylib's headers as system headers so our strict warnings skip them.
get_target_property(_hp_raylib_inc raylib INTERFACE_INCLUDE_DIRECTORIES)
set_target_properties(raylib PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_hp_raylib_inc}")
