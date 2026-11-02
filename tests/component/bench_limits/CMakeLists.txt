# Torque limits of the controller model in the bench Summary metrics (feature joint-limits).
cobot_add_test(test_bench_limits test_bench_limits.cpp LIBS cobot_bench_run cobot::pipeline)
target_include_directories(test_bench_limits PRIVATE ${PROJECT_SOURCE_DIR}/src/bench)
