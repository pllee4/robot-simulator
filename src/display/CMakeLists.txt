add_library(display STATIC src/display.cpp)
target_link_libraries(display
    PUBLIC types SDL2 SDL2_ttf)
target_include_directories(display PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_definitions(display PRIVATE "ASSET_PATH=\"${PROJECT_SOURCE_DIR}/assets\"")