cmake_minimum_required(VERSION 3.10.2)

project(robot_simulator VERSION 1.0.0)

set(USER_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${USER_CMAKE_PATH}/modules")
list(APPEND CMAKE_PREFIX_PATH "/usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu/cmake")

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 11)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# add_subdirectory(app)
# add_subdirectory(modules)
# add_subdirectory(src)
add_subdirectory(utils)