# Typed key overrides of the controller and imperfection YAML (feature studio-core, cobot_studio
# design 6.6, ADR-021): pipeline::KeyOverride, load_config_with / load_imperfections_with on a list
# of overrides, write_config_with / write_imperfections_with (cobot/pipeline/overrides.hpp), and
# the virtual key config.computed_torque.omega. The rerun check runs bench::run (cobot_bench_run)
# on the written files. Skipped where src/bench is not built; the root CMake adds tests/ after the
# applications so the target is known here.
if(NOT TARGET cobot_bench_run)
  return()
endif()
cobot_add_test(test_pipeline_overrides test_overrides.cpp test_overrides_rerun.cpp
  LIBS cobot_bench_run cobot::pipeline)
target_include_directories(test_pipeline_overrides PRIVATE ${PROJECT_SOURCE_DIR}/src/bench)
