# cobot_studio_core configuration side (feature studio-core, cobot_studio design 6.6 and 9,
# ADR-021): base pairs, the parameter catalog, the studio file, validate, materialize, RunStore and
# the no-Qt rule. Headers: src/studio/core/include/cobot/studio/*.hpp. Skipped where src/studio is
# not built (the Pi 5 image); the root CMake adds tests/ after the applications.
if(NOT TARGET cobot_studio_core)
  return()
endif()
find_package(yaml-cpp REQUIRED)
cobot_add_test(test_studio_config
  test_profile_catalog.cpp
  test_param_catalog.cpp
  test_studio_file.cpp
  test_run_store.cpp
  test_no_qt.cpp
  LIBS cobot_studio_core cobot_bench_run cobot::pipeline yaml-cpp::yaml-cpp)
target_include_directories(test_studio_config PRIVATE ${PROJECT_SOURCE_DIR}/src/bench)
