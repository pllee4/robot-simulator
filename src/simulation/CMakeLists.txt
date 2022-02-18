add_library(simulation STATIC
    src/profile/simulation_profile.cpp
    src/simulation.cpp
)
target_link_libraries(simulation PUBLIC 
beacon kalman_filter robot sensor helper display)
target_include_directories(simulation PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)