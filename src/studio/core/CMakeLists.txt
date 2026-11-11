# cobot_studio_core (feature studio-core, ADR-021): base pairs, parameter catalog, studio file,
# run store, metrics, chart series and the in-process runner on the bench engine; the 3D view's
# RobotScene, STL loader and playback helpers (feature studio-3d); the Run CSV reader, live runs
# and the bridge client (feature studio-control); the verify job (feature studio-verify); the
# noise level (feature studio-noise-level). No Qt.
find_package(yaml-cpp REQUIRED)
find_package(tinyxml2 REQUIRED)
add_library(cobot_studio_core STATIC
  src/profile_catalog.cpp
  src/param_catalog.cpp
  src/studio_config.cpp
  src/run_store.cpp
  src/metrics.cpp
  src/series.cpp
  src/runner.cpp
  src/robot_scene.cpp
  src/stl.cpp
  src/playback.cpp
  src/run_csv.cpp
  src/live_run.cpp
  src/bridge_client.cpp
  src/verify_job.cpp
  src/noise_level.cpp)
cobot_set_target_defaults(cobot_studio_core)
target_include_directories(cobot_studio_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# yaml-cpp only for the studio file (ADR-021); the controller and imperfection files go through
# pipeline (overrides.hpp).
target_link_libraries(cobot_studio_core PUBLIC cobot_bench_run cobot::pipeline cobot::bridge
  PRIVATE yaml-cpp::yaml-cpp tinyxml2::tinyxml2)
