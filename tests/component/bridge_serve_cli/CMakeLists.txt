# cobot_bench serve and cobot_bridge_cli (feature controller-bridge, plan owner question B3): the
# paced bench host on a thread of the test (real time) driven by the headless client run as a
# child process, plus the exit codes of both tools. Real time and shared memory: one ctest entry,
# run serially, 300 s timeout (not cobot_add_test). The tools' paths come in as COBOT_BENCH_EXE and
# COBOT_BRIDGE_CLI_EXE; src/bench is read before tests/, a missing tool leaves its path empty and
# the tests fail naming it.
if(NOT TARGET cobot_bench_run)
  return()
endif()

add_executable(test_bridge_serve_cli test_bridge_serve_cli.cpp)
cobot_set_target_defaults(test_bridge_serve_cli)
target_include_directories(test_bridge_serve_cli PRIVATE ${PROJECT_SOURCE_DIR}/src/bench)
target_link_libraries(test_bridge_serve_cli PRIVATE
  cobot_bench_run cobot_bridge cobot::pipeline GTest::gtest GTest::gtest_main)
target_compile_definitions(test_bridge_serve_cli PRIVATE
  COBOT_REPO_DIR="${PROJECT_SOURCE_DIR}"
  COBOT_TEST_OUTPUT_DIR="${CMAKE_BINARY_DIR}/test_output"
  COBOT_BENCH_EXE="$<$<TARGET_EXISTS:cobot_bench>:$<TARGET_FILE:cobot_bench>>"
  COBOT_BRIDGE_CLI_EXE="$<$<TARGET_EXISTS:cobot_bridge_cli>:$<TARGET_FILE:cobot_bridge_cli>>")
target_compile_options(test_bridge_serve_cli PRIVATE -UNDEBUG)
add_test(NAME test_bridge_serve_cli COMMAND test_bridge_serve_cli)
set_tests_properties(test_bridge_serve_cli PROPERTIES TIMEOUT 300 RUN_SERIAL TRUE)

function(cobot_bridge_serve_cli_depends)
  foreach(tool cobot_bench cobot_bridge_cli)
    if(TARGET ${tool})
      add_dependencies(test_bridge_serve_cli ${tool})
    endif()
  endforeach()
endfunction()
cmake_language(DEFER DIRECTORY ${PROJECT_SOURCE_DIR} CALL cobot_bridge_serve_cli_depends)
