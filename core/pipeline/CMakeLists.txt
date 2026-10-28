# pipeline: ControlLoop, config loading (only place that parses YAML), CSV log.
find_package(yaml-cpp REQUIRED)
cobot_add_module(pipeline
  SOURCES src/config.cpp src/log_record.cpp
  DEPS cobot::control cobot::planning cobot::imperfect cobot::hal yaml-cpp::yaml-cpp)
