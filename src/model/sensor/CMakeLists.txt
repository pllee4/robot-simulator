add_library(sensor STATIC
  src/gps_sensor.cpp
  src/gyro_sensor.cpp
  src/lidar_sensor.cpp
)
target_link_libraries(sensor PRIVATE beacon helper)
target_include_directories(sensor PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)