# Follow-up of verify (#81, #82): what the first nightly reports showed. The Wilson bounds are
# exactly 0 with no success and exactly 1 with no failure, and an assumption with neither range
# nor scale (a bool or choice key) has "-" as its bounds in report.md.
if(NOT TARGET cobot_bench_run)
  return()
endif()
cobot_add_test(test_bench_report_fixes test_report_fixes.cpp LIBS cobot_bench_run cobot::pipeline)
target_include_directories(test_bench_report_fixes PRIVATE ${PROJECT_SOURCE_DIR}/src/bench)
