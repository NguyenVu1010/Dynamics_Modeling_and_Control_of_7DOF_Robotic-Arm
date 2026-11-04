# Feature manual-commands: ControlLoop jog_joint, jog_tcp, move_joint, load(..., kMoveTcp),
# request_stop on a manual command, estop (ADR-022 "Lệnh tay"). The tests reuse the shared rig of
# tests/component/control_loop and the external-mode helpers of tests/unit/loop_external (read
# only).
set(COBOT_MANUAL_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../component/control_loop
    ${CMAKE_CURRENT_SOURCE_DIR}/../loop_external)

cobot_add_test(test_manual_loop
  test_manual_jog_joint.cpp
  test_manual_move_joint.cpp
  test_manual_jog_tcp.cpp
  test_manual_estop.cpp)
target_include_directories(test_manual_loop PRIVATE ${COBOT_MANUAL_INCLUDES})

# Replaces the global operator new: its own binary.
cobot_add_test(test_manual_no_alloc test_manual_no_alloc.cpp)
target_include_directories(test_manual_no_alloc PRIVATE ${COBOT_MANUAL_INCLUDES})
