add_executable(app_kalman_filter app_kalman_filter.cpp)
target_link_libraries(app_kalman_filter PRIVATE robot simulation display)