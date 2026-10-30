cmake_minimum_required(VERSION 3.22)
project(cobot_sim NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes worlds config launch rviz DESTINATION share/${PROJECT_NAME})

# Gazebo resolves package://cobot_sim/... through GZ_SIM_RESOURCE_PATH=<prefix>/share.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/cobot_sim.dsv.in")
ament_package()
