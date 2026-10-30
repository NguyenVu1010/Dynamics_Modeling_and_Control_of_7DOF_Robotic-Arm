# `cobot_bench run --waypoints FILE` (feature bench-waypoints, design 2026-10-29 section 6.6):
# the engine bench::run in cobot_bench_run (bench/run.hpp) and the exit codes of the cobot_bench
# binary, which the test runs as a child process (path passed as COBOT_BENCH_EXE).
# Own directory so the frozen bench-e2e CMakeLists stays untouched.
cobot_add_test(test_bench_waypoints test_bench_waypoints.cpp LIBS cobot_bench_run cobot::pipeline)
target_include_directories(test_bench_waypoints PRIVATE ${PROJECT_SOURCE_DIR}/src/bench)
target_compile_definitions(test_bench_waypoints PRIVATE
  COBOT_BENCH_EXE="$<TARGET_FILE:cobot_bench>")
add_dependencies(test_bench_waypoints cobot_bench)
