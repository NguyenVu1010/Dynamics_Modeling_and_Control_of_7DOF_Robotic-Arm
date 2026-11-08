# The studio's verify job (feature studio-verify, cobot_studio design 10 G14/G16, ADR-023): bad
# input and a cancel write nothing, new_verify_dir, and (release builds only) the same bytes as
# `cobot_bench verify` run as the child process COBOT_BENCH_EXE. Qt-free:
# src/studio/core/include/cobot/studio/verify_job.hpp in cobot_studio_core. The JSON reader and the
# child process helpers are tests/unit/bench_verify/verify_test_util.hpp. Skipped where src/studio
# is not built; the root CMake adds tests/ after the applications.
if(NOT TARGET cobot_studio_core)
  return()
endif()
cobot_add_test(test_studio_verify_job test_verify_job.cpp
  LIBS cobot_studio_core cobot_bench_run cobot::pipeline)
target_include_directories(test_studio_verify_job PRIVATE
  ${PROJECT_SOURCE_DIR}/src/bench ${CMAKE_CURRENT_SOURCE_DIR}/../bench_verify)
target_compile_definitions(test_studio_verify_job PRIVATE
  COBOT_BENCH_EXE="$<TARGET_FILE:cobot_bench>")
add_dependencies(test_studio_verify_job cobot_bench)
