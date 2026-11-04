# One studio run against the bench (feature studio-core, ADR-021): run_once materialises the
# configuration and runs bench::run_in_memory; same numbers and Run CSV bytes as cobot_bench on the
# materialised files; metrics from the stored ErrorTrace equal those from the records; stop
# waypoint numbers match cobot_eval. Skipped where src/studio is not built; the root CMake adds
# tests/ after the applications.
if(NOT TARGET cobot_studio_core)
  return()
endif()
cobot_add_test(test_studio_run test_studio_run.cpp
  LIBS cobot_studio_core cobot_bench_run cobot::pipeline)
target_include_directories(test_studio_run PRIVATE ${PROJECT_SOURCE_DIR}/src/bench)
