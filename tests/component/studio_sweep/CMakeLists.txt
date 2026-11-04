# Studio sweeps against cobot_bench sweep (feature studio-core, ADR-021): every RunSummary equal
# to the matching Summary CSV row, order, records of the first seed only, progress, cancel, level
# validation and max_parallel. Skipped where src/studio is not built; the root CMake adds tests/
# after the applications.
if(NOT TARGET cobot_studio_core)
  return()
endif()
cobot_add_test(test_studio_sweep test_studio_sweep.cpp
  LIBS cobot_studio_core cobot_bench_run cobot::pipeline)
target_include_directories(test_studio_sweep PRIVATE ${PROJECT_SOURCE_DIR}/src/bench)
