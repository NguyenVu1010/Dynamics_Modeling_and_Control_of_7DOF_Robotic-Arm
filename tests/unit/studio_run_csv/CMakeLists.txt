# The Run CSV read back by the studio (feature studio-control, cobot/studio/run_csv.hpp): a bench
# run written by bench::write_run_files and read with read_run_csv, the legacy 57-column layout and
# the errors. Skipped where src/studio is not built; the root CMake adds tests/ after the
# applications.
if(NOT TARGET cobot_studio_core)
  return()
endif()
cobot_add_test(test_studio_run_csv test_studio_run_csv.cpp
  LIBS cobot_studio_core cobot_bench_run cobot::pipeline)
target_include_directories(test_studio_run_csv PRIVATE ${PROJECT_SOURCE_DIR}/src/bench)
