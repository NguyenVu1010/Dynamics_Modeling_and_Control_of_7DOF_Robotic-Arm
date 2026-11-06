# Studio metrics, error traces, chart series and the sweep worst case on synthetic runs (feature
# studio-core, cobot_studio design 5.1 and 5.2, ADR-021). Skipped where src/studio is not built;
# the root CMake adds tests/ after the applications.
if(NOT TARGET cobot_studio_core)
  return()
endif()
cobot_add_test(test_studio_metrics test_studio_metrics.cpp test_series.cpp test_live_time.cpp
  LIBS cobot_studio_core cobot_bench_run cobot::pipeline)
target_include_directories(test_studio_metrics PRIVATE ${PROJECT_SOURCE_DIR}/src/bench)
