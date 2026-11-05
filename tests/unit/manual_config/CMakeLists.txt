# cobot.manual in load_config (feature manual-commands, ADR-022 "Lệnh tay").
cobot_add_test(test_manual_config test_manual_config.cpp)

# Feature jog-headroom: joint jog and plan_joint_move scale qd_max by
# cobot.manual.joint_speed_fraction. Reuses the rig of tests/unit/manual_loop (read only).
cobot_add_test(test_manual_speed_fraction test_manual_speed_fraction.cpp)
target_include_directories(test_manual_speed_fraction PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../manual_loop
  ${CMAKE_CURRENT_SOURCE_DIR}/../../component/control_loop
  ${CMAKE_CURRENT_SOURCE_DIR}/../loop_external)
