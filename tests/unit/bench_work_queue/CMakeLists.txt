# Follow-up of feature auto-tune (#78): bench::run_queue on its own, the cases the review of #78 listed
# as uncovered (the tune tests reach it only through the loader pre-check).
if(NOT TARGET cobot_bench_run)
  return()
endif()
cobot_add_test(test_bench_work_queue test_work_queue.cpp LIBS cobot_bench_run)
target_include_directories(test_bench_work_queue PRIVATE ${PROJECT_SOURCE_DIR}/src/bench)
