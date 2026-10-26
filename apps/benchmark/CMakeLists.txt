# Per-frame timing of the XPBD presets across mesh sizes.
add_executable(hp_xpbd_speed xpbd_speed.cpp)
target_link_libraries(hp_xpbd_speed PRIVATE hp::engine hp::build_flags)
