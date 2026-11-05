# studio::BridgeClient (feature studio-control, cobot/studio/bridge_client.hpp; no Qt) against
# bench::serve on a thread of the test (config/default.yaml + ideal, region
# /cobot_cobot_test_client_<pid>). Real time and shared memory: one ctest entry, run serially, 300 s
# timeout (not cobot_add_test). Skipped where src/studio is not built.
if(NOT TARGET cobot_studio_core OR NOT TARGET cobot_bench_run)
  return()
endif()

add_executable(test_studio_bridge_client test_studio_bridge_client.cpp)
cobot_set_target_defaults(test_studio_bridge_client)
target_include_directories(test_studio_bridge_client PRIVATE ${PROJECT_SOURCE_DIR}/src/bench)
target_link_libraries(test_studio_bridge_client PRIVATE
  cobot_studio_core cobot_bench_run cobot::bridge cobot::pipeline GTest::gtest GTest::gtest_main)
target_compile_definitions(test_studio_bridge_client PRIVATE
  COBOT_REPO_DIR="${PROJECT_SOURCE_DIR}"
  COBOT_TEST_OUTPUT_DIR="${CMAKE_BINARY_DIR}/test_output")
target_compile_options(test_studio_bridge_client PRIVATE -UNDEBUG)
add_test(NAME test_studio_bridge_client COMMAND test_studio_bridge_client)
set_tests_properties(test_studio_bridge_client PROPERTIES TIMEOUT 300 RUN_SERIAL TRUE)
