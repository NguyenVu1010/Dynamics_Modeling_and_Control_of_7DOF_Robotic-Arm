# cobot_gui: RViz2 panel cobot_gui/WaypointPanel editing waypoint lists for the controller's
# external command mode (design docs/design/2026-10-29-waypoint-gui-design.md 6.5). The
# ROS-free core is built from ../core as part of this package, as in cobot_ros2 (ADR-012).
cmake_minimum_required(VERSION 3.22)
project(cobot_gui LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(cobot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(COBOT_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
add_subdirectory("${COBOT_REPO_ROOT}/core" "${CMAKE_CURRENT_BINARY_DIR}/core")

# Data model without Qt, shared by the panel and the unit test.
add_library(cobot_gui_model STATIC src/waypoint_table.cpp)
cobot_set_target_defaults(cobot_gui_model)
set_target_properties(cobot_gui_model PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(cobot_gui_model PUBLIC include)
target_link_libraries(cobot_gui_model PUBLIC
  cobot::pipeline ${cobot_interfaces_TARGETS} ${geometry_msgs_TARGETS})

set(CMAKE_AUTOMOC ON)
add_library(${PROJECT_NAME} SHARED src/waypoint_panel.cpp src/waypoint_panel.hpp)
cobot_set_target_defaults(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} PRIVATE
  cobot_gui_model
  interactive_markers::interactive_markers
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rviz_common::rviz_common
  Qt5::Widgets
  ${nav_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${visualization_msgs_TARGETS})
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_waypoint_table test/test_waypoint_table.cpp)
  target_link_libraries(test_waypoint_table cobot_gui_model)
endif()

ament_package()
