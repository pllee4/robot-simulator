add_library(helper STATIC 
  src/details/transformation.cpp
  src/helper.cpp  
)
target_link_libraries(helper Eigen3::Eigen)
target_include_directories(helper PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)