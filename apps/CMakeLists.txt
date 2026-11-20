# Command-line tools and the viewer, all built on the engine.
add_subdirectory(benchmark)
add_subdirectory(headless_runner)

if(HP_BUILD_VIEWER)
    add_subdirectory(viewer)
endif()
