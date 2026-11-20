# Live 3D viewer. raylib is linked here only; the engine stays free of graphics.
add_executable(hp_view main.cpp)
target_link_libraries(hp_view PRIVATE hp::engine hp::build_flags raylib)
