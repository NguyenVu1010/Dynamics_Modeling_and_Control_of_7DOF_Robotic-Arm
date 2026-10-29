# Feature loop-external: ControlLoop external command mode (standby, load, request_stop, reset),
# the new cobot.* config keys and the waypoint file. The loop tests reuse the shared rig of
# tests/component/control_loop (read only).
set(COBOT_LOOP_RIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../component/control_loop)

cobot_add_test(test_loop_external
  test_loop_external.cpp
  test_loop_external_stop.cpp
  test_loop_external_reset.cpp)
target_include_directories(test_loop_external PRIVATE ${COBOT_LOOP_RIG_DIR})

# Replaces the global operator new: its own binary.
cobot_add_test(test_loop_external_no_alloc test_loop_external_no_alloc.cpp)
target_include_directories(test_loop_external_no_alloc PRIVATE ${COBOT_LOOP_RIG_DIR})

cobot_add_test(test_loop_external_io
  test_config_external.cpp
  test_waypoint_file.cpp
  test_loop_mode_standby.cpp)
target_include_directories(test_loop_external_io PRIVATE ${COBOT_LOOP_RIG_DIR})

# Config mode against a reference recorded before the feature (tests/data/loop_external).
cobot_add_test(test_loop_config_regression test_loop_config_regression.cpp)
target_include_directories(test_loop_config_regression PRIVATE ${COBOT_LOOP_RIG_DIR})
