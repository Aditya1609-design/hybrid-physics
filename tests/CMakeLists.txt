# Drop repeated static libraries from link lines (the engine arrives directly and through
# the viewer core); otherwise Apple's linker warns about duplicates. CMake 3.29 and later.
if(POLICY CMP0156)
    cmake_policy(SET CMP0156 NEW)
endif()

# Golden reference results: shared by the regression test and the hp_golden tool that
# rewrites tests/regression/golden.json after an approved physics change.
add_library(hp_golden_support STATIC regression/golden.cpp)
target_include_directories(hp_golden_support PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/regression)
target_link_libraries(hp_golden_support PUBLIC hp::engine nlohmann_json::nlohmann_json
    PRIVATE hp::build_flags)
target_compile_definitions(hp_golden_support PUBLIC
    HP_SCENARIO_DIR="${PROJECT_SOURCE_DIR}/scenarios"
    HP_GOLDEN_FILE="${CMAKE_CURRENT_SOURCE_DIR}/regression/golden.json")

add_executable(hp_golden regression/hp_golden.cpp)
target_link_libraries(hp_golden PRIVATE hp_golden_support hp::build_flags)

# Single test executable; each doctest case is registered with ctest separately.
add_executable(hp_tests
    test_main.cpp
    collision/test_collider.cpp
    core/test_version.cpp
    core/test_determinism_flags.cpp
    core/test_deps.cpp
    core/test_rng.cpp
    core/test_sim_state.cpp
    core/test_state_hash.cpp
    io/test_obj.cpp
    io/test_trajectory.cpp
    mesh/test_generators.cpp
    mesh/test_mesh_conformity.cpp
    mesh/test_graph.cpp
    mesh/test_topology.cpp
    regression/test_golden.cpp
    metrics/test_frame_metrics.cpp
    metrics/test_metrics.cpp
    metrics/test_penetration.cpp
    scene/test_make_state.cpp
    scene/test_pin.cpp
    scene/test_example_scenarios.cpp
    scene/test_scenario.cpp
    scene/test_scene.cpp
    solver/test_solver.cpp
    solver/test_xpbd.cpp
    solver/test_xpbd_accuracy.cpp
    solver/test_xpbd_contact.cpp
    solver/test_xpbd_friction.cpp
    solver/test_xpbd_constraints.cpp)

target_link_libraries(hp_tests PRIVATE
    hp::engine hp::build_flags doctest::doctest nlohmann_json::nlohmann_json hp_golden_support)

# Viewer logic tests, only when the viewer is part of the build.
if(TARGET hp_viewer_core)
    target_sources(hp_tests PRIVATE
        viewer/test_collider_shapes.cpp
        viewer/test_colors.cpp
        viewer/test_orbit_camera.cpp
        viewer/test_sim_driver.cpp
        viewer/test_surface_mesh.cpp)
    target_link_libraries(hp_tests PRIVATE hp_viewer_core)
endif()

# HP_SCENARIO_DIR (scenarios/ from any build directory) comes from hp_golden_support.

include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)
doctest_discover_tests(hp_tests)
