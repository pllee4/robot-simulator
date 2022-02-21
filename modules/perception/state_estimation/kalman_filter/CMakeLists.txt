if(NOT DEFINED KALMAN_FILTER_TYPE)
    set(KALMAN_FILTER_TYPE "lkf")
endif()

add_library(kalman_filter STATIC
    src/${KALMAN_FILTER_TYPE}/kalman_filter_${KALMAN_FILTER_TYPE}.cpp
)
target_link_libraries(kalman_filter 
    PRIVATE beacon sensor helper
    PUBLIC robot Eigen3::Eigen)
target_include_directories(kalman_filter PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)