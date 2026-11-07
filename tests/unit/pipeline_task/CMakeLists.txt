# Feature hardware-catalog (cobot_studio phase C PR 4, ADR-023): config/tasks/<task>.yaml ->
# pipeline::Task, run_overrides, and the controller section cobot.payload. The helpers live next to
# the hardware model tests.
cobot_add_test(test_pipeline_task
  test_task.cpp
  test_run_overrides.cpp
  test_payload_section.cpp)
target_include_directories(test_pipeline_task PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../pipeline_hardware_model)

# pick_place_3kg on premium with computed torque through bench::run (static library
# cobot_bench_run): the run itself only in optimised builds. Skipped where src/bench is not built.
if(NOT TARGET cobot_bench_run)
  return()
endif()
cobot_add_test(test_pipeline_task_run test_task_run.cpp LIBS cobot_bench_run cobot::pipeline)
target_include_directories(test_pipeline_task_run PRIVATE
  ${PROJECT_SOURCE_DIR}/src/bench
  ${CMAKE_CURRENT_SOURCE_DIR}/../pipeline_hardware_model)
