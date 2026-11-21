# Runs a scenario file without graphics, printing metrics and a final state hash, and
# optionally recording a trajectory.
add_executable(hp_run main.cpp machine_info.cpp)
target_link_libraries(hp_run PRIVATE hp::engine hp::build_flags)
