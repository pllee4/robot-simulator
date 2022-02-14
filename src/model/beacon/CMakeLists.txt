add_library(beacon STATIC src/beacon.cpp)
target_link_libraries(beacon display helper)
target_include_directories(beacon PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)