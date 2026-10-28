# Test infrastructure that needs ROS packages; added only with -DCOBOT_WITH_KDL_REF=ON.
# KDL never reaches core/ or the test binaries (ADR-004): this tool writes a CSV that is
# committed under tests/data/. Configure from a shell that sourced /opt/ros/jazzy/setup.bash
# (ament CMake configs need its PYTHONPATH).
list(APPEND CMAKE_PREFIX_PATH /opt/ros/jazzy)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)

add_executable(cobot_gen_reference_dynamics gen_reference_dynamics.cpp)
cobot_set_target_defaults(cobot_gen_reference_dynamics)
target_include_directories(cobot_gen_reference_dynamics SYSTEM PRIVATE ${orocos_kdl_INCLUDE_DIRS})
target_link_libraries(cobot_gen_reference_dynamics PRIVATE
  kdl_parser::kdl_parser urdf::urdf ${orocos_kdl_LIBRARIES})
