# Feature verify-analysis (cobot_studio phase C PR 6b, ADR-023 "Báo cáo", design 10.2 G16): the
# sensitivity rule, the profile variants and group off overrides, the suggestion table and the
# main cause, the comparison rows and files, the report with the analysis (test_bench_verify_analysis,
# every build), and the engine and the CLI with lists (test_bench_verify_analysis_runs; real runs in
# release builds only, the error paths in every build). The API is src/bench/analysis.hpp,
# compare.hpp, verify.hpp and report.hpp in the static library cobot_bench_run, and
# pipeline::profile_value; the JSON reader and the child process helpers are
# tests/unit/bench_verify/verify_test_util.hpp.
if(NOT TARGET cobot_bench_run)
  return()
endif()
cobot_add_test(test_bench_verify_analysis
  test_sensitivity_rule.cpp
  test_variants.cpp
  test_suggestions.cpp
  test_comparison.cpp
  test_report_analysis.cpp
  LIBS cobot_bench_run cobot::pipeline)
target_include_directories(test_bench_verify_analysis PRIVATE
  ${PROJECT_SOURCE_DIR}/src/bench ${CMAKE_CURRENT_SOURCE_DIR}/../bench_verify)

cobot_add_test(test_bench_verify_analysis_runs
  test_analysis_runs.cpp
  test_analysis_cli.cpp
  LIBS cobot_bench_run cobot::pipeline)
target_include_directories(test_bench_verify_analysis_runs PRIVATE
  ${PROJECT_SOURCE_DIR}/src/bench ${CMAKE_CURRENT_SOURCE_DIR}/../bench_verify)
target_compile_definitions(test_bench_verify_analysis_runs PRIVATE
  COBOT_BENCH_EXE="$<TARGET_FILE:cobot_bench>")
add_dependencies(test_bench_verify_analysis_runs cobot_bench)
