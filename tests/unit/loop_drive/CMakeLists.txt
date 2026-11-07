# Feature drive-loop (ADR-023, cobot_studio phase C PR 3): control::DrivePosition, the third
# registry entry, cobot.drive in load_config and ControlLoop with drive_position (set point, torque
# feed-forward, seq, deflection shift, hold through the drive). The loop tests reuse the shared rig
# of tests/component/control_loop (read only) with a SimDrive attached.
set(COBOT_LOOP_RIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../component/control_loop)

cobot_add_test(test_loop_drive
  test_drive_position.cpp
  test_drive_config.cpp
  test_loop_drive.cpp)
target_include_directories(test_loop_drive PRIVATE ${COBOT_LOOP_RIG_DIR})

# Replaces the global operator new: its own binary.
cobot_add_test(test_loop_drive_no_alloc test_loop_drive_no_alloc.cpp)
target_include_directories(test_loop_drive_no_alloc PRIVATE ${COBOT_LOOP_RIG_DIR})

# The bench hosts (run engine, timing, serve rejection): the static library cobot_bench_run.
cobot_add_test(test_loop_drive_bench test_loop_drive_bench.cpp LIBS cobot_bench_run cobot::pipeline)
target_include_directories(test_loop_drive_bench PRIVATE ${PROJECT_SOURCE_DIR}/src/bench)
