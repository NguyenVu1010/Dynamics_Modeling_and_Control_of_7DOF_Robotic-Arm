# Feature auto-tune (cobot_studio phase C PR 5, ADR-023 "Tự chỉnh hệ số trước khi kết luận"): the gain
# family per law, config.pd_gravity.omega, the selection rule on synthetic tables, tune.csv, the
# post-goal swing of RunSummary (test_bench_tune, every build), and the engine and the CLI
# (test_bench_tune_runs; real runs in release builds only, the error paths in every build). The API
# is src/bench/tune.hpp in the static library cobot_bench_run; the child process is COBOT_BENCH_EXE.
if(NOT TARGET cobot_bench_run)
  return()
endif()
cobot_add_test(test_bench_tune
  test_gain_family.cpp
  test_selection.cpp
  test_tune_csv.cpp
  test_summary_swing.cpp
  LIBS cobot_bench_run cobot::pipeline)
target_include_directories(test_bench_tune PRIVATE ${PROJECT_SOURCE_DIR}/src/bench)

cobot_add_test(test_bench_tune_runs
  test_tune_errors.cpp
  test_tune_runs.cpp
  test_tune_cli.cpp
  LIBS cobot_bench_run cobot::pipeline)
target_include_directories(test_bench_tune_runs PRIVATE ${PROJECT_SOURCE_DIR}/src/bench)
target_compile_definitions(test_bench_tune_runs PRIVATE
  COBOT_BENCH_EXE="$<TARGET_FILE:cobot_bench>")
add_dependencies(test_bench_tune_runs cobot_bench)
