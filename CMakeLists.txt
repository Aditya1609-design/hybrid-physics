cmake_minimum_required(VERSION 3.20)

project(hybrid_physics VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(HP_BUILD_TESTS "Build the unit tests" ON)
option(HP_BUILD_APPS "Build the command-line apps (benchmarks)" ON)
option(HP_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(HP_SANITIZE "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
set(HP_DEPS_CACHE_DIR "${PROJECT_SOURCE_DIR}/.cache/deps" CACHE PATH
    "Where downloaded dependency archives are kept between clean builds")

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(CompilerSettings)
include(Dependencies)
include(DependencyGuard)

add_subdirectory(engine)

if(HP_BUILD_APPS)
    add_subdirectory(apps)
endif()

if(HP_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Runs last so it sees every link added to the engine from anywhere in the project.
hp_check_engine_dependencies(hp_engine)
