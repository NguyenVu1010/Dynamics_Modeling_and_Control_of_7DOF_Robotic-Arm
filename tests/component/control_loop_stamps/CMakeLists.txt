# Review gaps of feature control-loop: hold while stamps are still frozen, and a stamp going
# backwards. Reuses the shared rig of tests/component/control_loop (read only).
cobot_add_test(test_control_loop_stamps test_control_loop_stamps.cpp)
target_include_directories(test_control_loop_stamps
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../control_loop)
