# ControlLoop recording into a host-owned Recorder (feature pipeline-recorder, cobot_studio design
# 6.1). Reuses the shared rig of tests/component/control_loop (read only). The allocation test
# replaces the global operator new, so it is a binary of its own.
cobot_add_test(test_control_loop_recorder test_control_loop_recorder.cpp)
target_include_directories(test_control_loop_recorder
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../control_loop)
cobot_add_test(test_control_loop_recorder_no_alloc test_control_loop_recorder_no_alloc.cpp)
target_include_directories(test_control_loop_recorder_no_alloc
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../control_loop)
