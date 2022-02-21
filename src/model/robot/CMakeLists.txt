add_library(robot STATIC 
    src/details/bicycle_mode.cpp 
    src/robot.cpp)
target_link_libraries(robot 
    PRIVATE display helper 
    PUBLIC types)
target_include_directories(robot PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)