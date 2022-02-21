add_library(helper STATIC 
  src/details/transformation.cpp
  src/helper.cpp  
)
target_link_libraries(helper 
  PRIVATE Eigen3::Eigen
  PUBLIC types)
target_include_directories(helper PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)