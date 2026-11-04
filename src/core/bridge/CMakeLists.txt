# bridge: the studio bridge in POSIX shared memory (feature shm-bridge, ADR-022), Linux only. No
# ROS, no Qt, no YAML.
cobot_add_module(bridge
  SOURCES src/wire.cpp src/endpoint.cpp
  DEPS cobot::pipeline)
