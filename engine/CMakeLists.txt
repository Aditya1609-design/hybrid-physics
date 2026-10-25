# Headless engine library: no graphics or ML dependencies (enforced by DependencyGuard).
add_library(hp_engine STATIC
    src/core/rng.cpp
    src/core/version.cpp
    src/mesh/generators.cpp
    src/mesh/tet_mesh.cpp
    src/mesh/topology.cpp)
add_library(hp::engine ALIAS hp_engine)

target_include_directories(hp_engine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Build flags are PUBLIC so inline engine code compiled in other targets keeps the
# same floating-point rules.
target_link_libraries(hp_engine PUBLIC hp::eigen hp::build_flags)

target_compile_definitions(hp_engine PRIVATE
    HP_VERSION_STRING="${PROJECT_VERSION}"
    HP_BUILD_TYPE="$<IF:$<CONFIG:>,unspecified,$<CONFIG>>"
    HP_SANITIZED=$<BOOL:${HP_SANITIZE}>)
