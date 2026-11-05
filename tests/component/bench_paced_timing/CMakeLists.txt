# PF-02 (feature shm-bridge, ADR-022 "PF-02"): bench::timing with TimingOptions (paced, log mode,
# reader) and the `cobot_bench timing --paced --log-mode` command line, its path passed as
# COBOT_BENCH_EXE. Wall-clock measurements, so the tests run alone (RUN_SERIAL) with a 900 s
# limit: under Debug + ASan a step takes about 2 ms, longer than the 1 ms period, and the
# allocation test (18 000 paced steps) needs about 110 s on WSL and more on CI runners. The
# gtest_discover_tests call of cobot_add_test cannot take test properties, hence the same steps
# written out here. The global operator new counter is tests/unit/bridge_endpoint/
# count_new.hpp. Skipped where the bench is not built.
if(NOT TARGET cobot_bench_run)
  return()
endif()
add_executable(test_bench_paced_timing test_bench_paced_timing.cpp)
cobot_set_target_defaults(test_bench_paced_timing)
target_link_libraries(test_bench_paced_timing PRIVATE
  cobot_bench_run cobot::pipeline GTest::gtest GTest::gtest_main)
target_include_directories(test_bench_paced_timing PRIVATE
  ${PROJECT_SOURCE_DIR}/src/bench ${PROJECT_SOURCE_DIR}/tests/unit/bridge_endpoint)
target_compile_definitions(test_bench_paced_timing PRIVATE
  COBOT_REPO_DIR="${PROJECT_SOURCE_DIR}"
  COBOT_TEST_OUTPUT_DIR="${CMAKE_BINARY_DIR}/test_output"
  COBOT_BENCH_EXE="$<$<TARGET_EXISTS:cobot_bench>:$<TARGET_FILE:cobot_bench>>"
  EIGEN_RUNTIME_NO_MALLOC)
target_compile_options(test_bench_paced_timing PRIVATE -UNDEBUG)
if(TARGET cobot_bench)
  add_dependencies(test_bench_paced_timing cobot_bench)
endif()
gtest_discover_tests(test_bench_paced_timing DISCOVERY_TIMEOUT 60
                     PROPERTIES RUN_SERIAL TRUE TIMEOUT 900)
