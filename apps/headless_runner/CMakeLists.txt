# Runs a scenario file without graphics, printing metrics and a final state hash.
add_executable(hp_run main.cpp)
target_link_libraries(hp_run PRIVATE hp::engine hp::build_flags)
