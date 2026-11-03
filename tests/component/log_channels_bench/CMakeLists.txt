# Feature log-channels (cobot_studio design 5.1, 6.2 and 7): the plant-truth channels of bench
# runs (run engine in cobot_bench_run, bench/engine.hpp) and ControlLoop::step with a PlantTruth.
# The loop test replaces the global operator new, so it is a binary of its own; it reuses the
# shared rig of tests/component/control_loop (read only).
cobot_add_test(test_log_channels_bench test_log_channels_bench.cpp
  LIBS cobot_bench_run cobot::pipeline)
target_include_directories(test_log_channels_bench PRIVATE ${PROJECT_SOURCE_DIR}/src/bench)
cobot_add_test(test_log_channels_loop test_log_channels_loop.cpp)
target_include_directories(test_log_channels_loop
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../control_loop)
