# cobot_ros2: the ros2_control controller plugin around pipeline::ControlLoop (design 5.5).
# The ROS-free core is built from ../core as part of this package (ADR-012), so one colcon
# build of sim/ and ros2_node/ needs no separately installed core.
cmake_minimum_required(VERSION 3.22)
project(cobot_ros2 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cobot_interfaces REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(COBOT_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
add_subdirectory("${COBOT_REPO_ROOT}/src/core" "${CMAKE_CURRENT_BINARY_DIR}/core")

add_library(${PROJECT_NAME} SHARED src/cobot_controller.cpp src/waypoint_server.cpp)
cobot_set_target_defaults(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} PRIVATE
  cobot::pipeline
  ament_index_cpp::ament_index_cpp
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  ${cobot_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS})
pluginlib_export_plugin_description_file(controller_interface cobot_ros2.xml)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
# Waypoint client of the external command mode: `ros2 run cobot_ros2 send_waypoints.py FILE`.
install(PROGRAMS "${COBOT_REPO_ROOT}/tools/send_waypoints.py" DESTINATION lib/cobot_ros2)
# Default parameters: share/cobot_ros2/config/default.yaml, whose urdf_path
# (../sim/urdf/cobot30_5.urdf) resolves to the installed copy of the URDF.
# Its log.output_dir is not used: without log_path the CSV goes to results/<run-id>/run.csv
# under the working directory of the controller manager (ADR-012).
install(DIRECTORY "${COBOT_REPO_ROOT}/config" DESTINATION share/${PROJECT_NAME})
install(FILES "${COBOT_REPO_ROOT}/src/sim/urdf/cobot30_5.urdf" DESTINATION share/${PROJECT_NAME}/sim/urdf)

ament_package()
