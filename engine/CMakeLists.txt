# Headless engine library: no graphics or ML dependencies (enforced by DependencyGuard).

# The git commit is written into a generated source on every build, so recordings always name
# the commit their binary was built from. The file only changes when the commit or dirty flag
# does, so this costs no recompilation otherwise.
find_package(Git QUIET)
set(HP_GIT_INFO_SOURCE ${CMAKE_CURRENT_BINARY_DIR}/generated/git_info.cpp)
set(_hp_git_info_cmd ${CMAKE_COMMAND} -DGIT_EXECUTABLE=${GIT_EXECUTABLE}
    -DSRC_DIR=${PROJECT_SOURCE_DIR} -DOUT=${HP_GIT_INFO_SOURCE}
    -P ${PROJECT_SOURCE_DIR}/cmake/GitInfo.cmake)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/generated)
execute_process(COMMAND ${_hp_git_info_cmd})  # so the file exists before the first build
add_custom_target(hp_git_info
    COMMAND ${_hp_git_info_cmd}
    BYPRODUCTS ${HP_GIT_INFO_SOURCE}
    COMMENT "Updating git commit info"
    VERBATIM)

add_library(hp_engine STATIC
    src/collision/collider.cpp
    src/core/rng.cpp
    src/core/sim_state.cpp
    src/core/state_hash.cpp
    src/core/version.cpp
    src/io/obj.cpp
    src/io/trajectory_reader.cpp
    src/io/trajectory_writer.cpp
    src/mesh/generators.cpp
    src/mesh/graph.cpp
    src/mesh/mass.cpp
    src/mesh/tet_mesh.cpp
    src/mesh/topology.cpp
    src/metrics/constraint_error.cpp
    src/metrics/energy.cpp
    src/metrics/frame_metrics.cpp
    src/metrics/penetration.cpp
    src/scene/make_state.cpp
    src/scene/pin.cpp
    src/scene/scenario.cpp
    src/scene/scene.cpp
    src/solver/solver.cpp
    src/solver/xpbd.cpp
    src/solver/xpbd_constraints.cpp
    src/solver/xpbd_kernel.cpp
    ${HP_GIT_INFO_SOURCE})
add_library(hp::engine ALIAS hp_engine)
add_dependencies(hp_engine hp_git_info)

target_include_directories(hp_engine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Build flags are PUBLIC so inline engine code compiled in other targets keeps the
# same floating-point rules.
target_link_libraries(hp_engine PUBLIC hp::eigen hp::build_flags)

# JSON is only used inside the scenario loader; public headers do not expose it.
target_link_libraries(hp_engine PRIVATE nlohmann_json::nlohmann_json)

target_compile_definitions(hp_engine PRIVATE
    HP_VERSION_STRING="${PROJECT_VERSION}"
    HP_BUILD_TYPE="$<IF:$<CONFIG:>,unspecified,$<CONFIG>>"
    HP_SANITIZED=$<BOOL:${HP_SANITIZE}>)
