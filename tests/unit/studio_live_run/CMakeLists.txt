# A live run turned into a RunEntry (feature studio-control, cobot/studio/live_run.hpp), without a
# bridge: a bench waypoint run written as a Run CSV plays the exported Recorder segment. Skipped
# where src/studio is not built; the root CMake adds tests/ after the applications.
if(NOT TARGET cobot_studio_core)
  return()
endif()
cobot_add_test(test_studio_live_run test_studio_live_run.cpp
  LIBS cobot_studio_core cobot_bench_run cobot::bridge cobot::pipeline)
target_include_directories(test_studio_live_run PRIVATE ${PROJECT_SOURCE_DIR}/src/bench)
