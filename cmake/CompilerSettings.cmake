# Compiler flags for our own targets. Linking hp::build_flags applies them; third-party
# code built as its own target is left alone.

if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    message(WARNING "hybrid_physics is developed and tested with clang; got ${CMAKE_CXX_COMPILER_ID}")
endif()

# -O2 rather than CMake's default -O3 for Release, so optimized builds stay predictable.
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

add_library(hp_build_flags INTERFACE)
add_library(hp::build_flags ALIAS hp_build_flags)

# Determinism: never fuse a*b+c into an FMA and never allow fast-math reassociation.
# Without these, results can change with optimization level or surrounding code.
target_compile_options(hp_build_flags INTERFACE -ffp-contract=off -fno-fast-math)

target_compile_options(hp_build_flags INTERFACE
    -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion)

if(HP_WARNINGS_AS_ERRORS)
    target_compile_options(hp_build_flags INTERFACE -Werror)
endif()

# Sanitizer builds: memory errors and undefined behaviour abort the test run.
# Leak detection is not available with Apple clang on arm64, so it is not requested.
if(HP_SANITIZE)
    set(_hp_sanitize_flags -fsanitize=address,undefined -fno-sanitize-recover=undefined
        -fno-omit-frame-pointer)
    target_compile_options(hp_build_flags INTERFACE ${_hp_sanitize_flags} -O1 -g)
    target_link_options(hp_build_flags INTERFACE ${_hp_sanitize_flags})
endif()
