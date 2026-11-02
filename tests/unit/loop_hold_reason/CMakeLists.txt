# Feature comm-timing: the reason ControlLoop names when it enters kHolding (HoldReason), kept
# until reset(), and the `hold` column of the Run CSV. Reuses the shared rig of
# tests/component/control_loop (read only).
cobot_add_test(test_loop_hold_reason test_loop_hold_reason.cpp)
target_include_directories(test_loop_hold_reason
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../component/control_loop)
