# Single test executable; each doctest case is registered with ctest separately.
add_executable(hp_tests
    test_main.cpp
    core/test_version.cpp
    core/test_determinism_flags.cpp
    core/test_deps.cpp)

target_link_libraries(hp_tests PRIVATE
    hp::engine hp::build_flags doctest::doctest nlohmann_json::nlohmann_json)

include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)
doctest_discover_tests(hp_tests)
