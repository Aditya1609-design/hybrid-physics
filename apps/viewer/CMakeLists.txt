# Viewer logic without graphics (camera, surface mesh, colors, collider shapes, stepping and
# pacing), so it can be unit-tested in every build. Depends on the engine only.
add_library(hp_viewer_core STATIC
    core/collider_shapes.cpp
    core/colors.cpp
    core/orbit_camera.cpp
    core/sim_driver.cpp
    core/surface_mesh.cpp)
target_include_directories(hp_viewer_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(hp_viewer_core PUBLIC hp::engine hp::build_flags)

# Live 3D viewer. raylib is linked here only; the engine stays free of graphics.
add_executable(hp_view main.cpp overlay.cpp render.cpp)
target_link_libraries(hp_view PRIVATE hp_viewer_core raylib)
