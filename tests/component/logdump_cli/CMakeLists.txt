# cobot_logdump (feature blackbox-log, cobot_studio design 6.1 "Hộp đen"): the test fills a
# region with ShmStorage and runs the built tool as a child process, its path passed as
# COBOT_LOGDUMP_EXE. src/bench is added after tests/, so the dependency on the tool is added
# once the whole tree is read; without the target the path is empty and the tests fail.
cobot_add_test(test_logdump_cli test_logdump_cli.cpp)
target_include_directories(test_logdump_cli
                           PRIVATE ${PROJECT_SOURCE_DIR}/tests/unit/pipeline_recorder_shm)
target_compile_definitions(test_logdump_cli PRIVATE
  COBOT_LOGDUMP_EXE="$<$<TARGET_EXISTS:cobot_logdump>:$<TARGET_FILE:cobot_logdump>>")

function(cobot_logdump_cli_depends)
  if(TARGET cobot_logdump)
    add_dependencies(test_logdump_cli cobot_logdump)
  endif()
endfunction()
cmake_language(DEFER DIRECTORY ${PROJECT_SOURCE_DIR} CALL cobot_logdump_cli_depends)
