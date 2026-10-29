# cobot_ros2: the ros2_control controller plugin around pipeline::ControlLoop (design 5.5).
# The ROS-free core is built from ../core as part of this package (ADR-012), so one colcon
# build of sim/ and ros2_node/ needs no separately installed core.
cmake_minimum_required(VERSION 3.22)
project(cobot_ros2 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)

set(COBOT_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
add_subdirectory("${COBOT_REPO_ROOT}/core" "${CMAKE_CURRENT_BINARY_DIR}/core")

add_library(${PROJECT_NAME} SHARED src/cobot_controller.cpp)
cobot_set_target_defaults(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} PRIVATE
  cobot::pipeline
  ament_index_cpp::ament_index_cpp
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  ${std_msgs_TARGETS})
pluginlib_export_plugin_description_file(controller_interface cobot_ros2.xml)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
# Default parameters: share/cobot_ros2/config/default.yaml, whose urdf_path
# (../sim/urdf/cobot30_5.urdf) resolves to the installed copy of the URDF.
install(DIRECTORY "${COBOT_REPO_ROOT}/config" DESTINATION share/${PROJECT_NAME})
install(FILES "${COBOT_REPO_ROOT}/sim/urdf/cobot30_5.urdf" DESTINATION share/${PROJECT_NAME}/sim/urdf)

ament_package()
