add_library(display STATIC src/display.cpp)
target_link_libraries(display 
    types         
    SDL2
    SDL2_ttf
    Eigen3::Eigen)
target_include_directories(display PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)