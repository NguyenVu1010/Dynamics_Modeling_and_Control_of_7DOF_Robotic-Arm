# Feature verify (cobot_studio phase C PR 6a, ADR-023 "Kết luận ba mức từ Monte Carlo", "Báo cáo"):
# Wilson, the verdict and the per-run pass, the settle time on synthetic rows, sha256, the report
# writer (test_bench_verify, every build), and the engine and the CLI (test_bench_verify_runs; real
# runs in release builds only, the error paths in every build). The API is src/bench/verify.hpp,
# report.hpp and sha256.hpp in the static library cobot_bench_run; the child process is
# COBOT_BENCH_EXE.
if(NOT TARGET cobot_bench_run)
  return()
endif()
cobot_add_test(test_bench_verify
  test_statistics.cpp
  test_settle_time.cpp
  test_sha256.cpp
  test_report.cpp
  LIBS cobot_bench_run cobot::pipeline)
target_include_directories(test_bench_verify PRIVATE ${PROJECT_SOURCE_DIR}/src/bench)

cobot_add_test(test_bench_verify_runs
  test_verify_runs.cpp
  test_verify_cli.cpp
  test_verify_cancel.cpp
  LIBS cobot_bench_run cobot::pipeline)
target_include_directories(test_bench_verify_runs PRIVATE ${PROJECT_SOURCE_DIR}/src/bench)
target_compile_definitions(test_bench_verify_runs PRIVATE
  COBOT_BENCH_EXE="$<TARGET_FILE:cobot_bench>")
add_dependencies(test_bench_verify_runs cobot_bench)
