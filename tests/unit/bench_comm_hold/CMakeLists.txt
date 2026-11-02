# Feature comm-timing: the safety hold on packet loss (ControlLoop + ImperfectJointInterface +
# SimPlant), SimJointInterface overrun, and the bench side (hold columns of the Run and Summary
# CSV, config.<section>.<key> sweep params, overrun in the run engine). The bench engine is the
# static library cobot_bench_run (src/bench/CMakeLists.txt); the loop rig is shared read only.
cobot_add_test(test_bench_comm_hold test_bench_comm_hold.cpp)
target_include_directories(test_bench_comm_hold
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../component/control_loop)
cobot_add_test(test_bench_comm_sweep test_bench_comm_sweep.cpp LIBS cobot_bench_run cobot::pipeline)
target_include_directories(test_bench_comm_sweep PRIVATE ${PROJECT_SOURCE_DIR}/src/bench)
