# ShmStorage, dump_shm_log and recover_shm_log (feature blackbox-log, cobot_studio design 6.1
# "Hộp đen", ADR-020). Regions are named /cobot_log_test_<pid>_<n> and removed by RAII guards.
# The crash tests fork a child that runs the ControlLoop rig of tests/component/control_loop
# (read only). The allocation test replaces the global operator new, so it is a binary of its own.
cobot_add_test(test_recorder_shm test_recorder_shm.cpp)
cobot_add_test(test_recorder_shm_crash test_recorder_shm_crash.cpp)
target_include_directories(test_recorder_shm_crash
                           PRIVATE ${PROJECT_SOURCE_DIR}/tests/component/control_loop)
cobot_add_test(test_recorder_shm_no_alloc test_recorder_shm_no_alloc.cpp)
