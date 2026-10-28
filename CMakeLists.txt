# Superbuild without ROS: core library, tests and bench.
# The ROS 2 packages (sim/, ros2_node/) are built with colcon instead (see README).
cmake_minimum_required(VERSION 3.22)
project(cobot LANGUAGES CXX)

option(COBOT_BUILD_TESTS "Build unit, component and e2e test binaries" ON)
option(COBOT_BUILD_BENCH "Build the bench binary (cobot_bench)" ON)

add_subdirectory(core)
if(COBOT_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
if(COBOT_BUILD_BENCH AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/bench/CMakeLists.txt")
  add_subdirectory(bench)
endif()
