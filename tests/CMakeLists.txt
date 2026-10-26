# Single test executable; each doctest case is registered with ctest separately.
add_executable(hp_tests
    test_main.cpp
    core/test_version.cpp
    core/test_determinism_flags.cpp
    core/test_deps.cpp
    core/test_rng.cpp
    core/test_sim_state.cpp
    io/test_obj.cpp
    mesh/test_generators.cpp
    mesh/test_graph.cpp
    mesh/test_topology.cpp
    metrics/test_metrics.cpp
    scene/test_make_state.cpp
    scene/test_pin.cpp
    solver/test_solver.cpp
    solver/test_xpbd.cpp
    solver/test_xpbd_constraints.cpp)

target_link_libraries(hp_tests PRIVATE
    hp::engine hp::build_flags doctest::doctest nlohmann_json::nlohmann_json)

include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)
doctest_discover_tests(hp_tests)
