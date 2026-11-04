# cobot_studio_core (feature studio-core, ADR-021): base pairs, parameter catalog, studio file,
# run store, metrics, chart series and the in-process runner on the bench engine. No Qt.
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
add_library(cobot_studio_core STATIC
  src/profile_catalog.cpp
  src/param_catalog.cpp
  src/studio_config.cpp
  src/run_store.cpp
  src/metrics.cpp
  src/series.cpp
  src/runner.cpp)
cobot_set_target_defaults(cobot_studio_core)
target_include_directories(cobot_studio_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# yaml-cpp only for the studio file (ADR-021); the controller and imperfection files go through
# pipeline (overrides.hpp).
target_link_libraries(cobot_studio_core PUBLIC cobot_bench_run cobot::pipeline
  PRIVATE yaml-cpp::yaml-cpp Threads::Threads)
