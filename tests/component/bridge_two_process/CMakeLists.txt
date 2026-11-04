# The studio bridge between two processes (feature shm-bridge, cobot_studio design 7 "Cầu nối",
# ADR-022): forked children play the controller or a studio. Timing-sensitive (paced writer,
# heartbeat window), so it runs alone with a 120 s limit per test; the gtest_discover_tests call of
# cobot_add_test cannot take test properties, hence the same steps written out here. Regions are
# named /cobot_test_<pid>_<n> (helpers of tests/unit/bridge_endpoint).
set(bridge_libs cobot::pipeline)
if(TARGET cobot::bridge)
  list(APPEND bridge_libs cobot::bridge)
endif()
find_package(Threads REQUIRED)
add_executable(test_bridge_two_process test_bridge_two_process.cpp)
cobot_set_target_defaults(test_bridge_two_process)
target_link_libraries(test_bridge_two_process PRIVATE
  ${bridge_libs} Threads::Threads GTest::gtest GTest::gtest_main)
target_include_directories(test_bridge_two_process
                           PRIVATE ${PROJECT_SOURCE_DIR}/tests/unit/bridge_endpoint)
target_compile_definitions(test_bridge_two_process PRIVATE
  COBOT_REPO_DIR="${PROJECT_SOURCE_DIR}"
  COBOT_TEST_OUTPUT_DIR="${CMAKE_BINARY_DIR}/test_output"
  EIGEN_RUNTIME_NO_MALLOC)
target_compile_options(test_bridge_two_process PRIVATE -UNDEBUG)
gtest_discover_tests(test_bridge_two_process DISCOVERY_TIMEOUT 60
                     PROPERTIES RUN_SERIAL TRUE TIMEOUT 120)
