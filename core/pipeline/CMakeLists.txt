# pipeline: ControlLoop, config loading (only place that parses YAML), CSV log.
find_package(yaml-cpp REQUIRED)
cobot_add_module(pipeline
  SOURCES src/config.cpp src/log_record.cpp src/control_loop.cpp src/sim_joint_interface.cpp
  DEPS cobot::control cobot::dynamics cobot::kinematics cobot::planning cobot::imperfect cobot::hal yaml-cpp::yaml-cpp)
