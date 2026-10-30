# Superbuild without ROS: core library, tests and bench.
# The ROS 2 packages (sim/, ros2_node/) are built with colcon instead (see README).
cmake_minimum_required(VERSION 3.22)
project(cobot LANGUAGES CXX)

option(COBOT_BUILD_TESTS "Build unit, component and e2e test binaries" ON)
option(COBOT_BUILD_BENCH "Build the bench binary (cobot_bench)" ON)
option(COBOT_WITH_KDL_REF "Build tools/gen_reference_dynamics (KDL from /opt/ros/jazzy)" OFF)

add_subdirectory(src/core)
if(COBOT_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
if(COBOT_BUILD_BENCH AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/bench/CMakeLists.txt")
  add_subdirectory(src/bench)
endif()
if(COBOT_WITH_KDL_REF)
  add_subdirectory(tools)
endif()
