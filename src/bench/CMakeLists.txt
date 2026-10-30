# cobot_bench (feature bench-e2e): the control loop against SimPlant without ROS. The run
# engine is the static library cobot_bench_run so tests and later modes reuse it.
include(GNUInstallDirs)
find_package(Threads REQUIRED)
# Recorded in the run's meta file; stays empty without git or a checkout (Docker build).
execute_process(COMMAND git describe --always --dirty --tags
                WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                OUTPUT_VARIABLE COBOT_GIT_DESCRIBE OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)

add_library(cobot_bench_run STATIC run.cpp sweep.cpp)
cobot_set_target_defaults(cobot_bench_run)
target_include_directories(cobot_bench_run PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(cobot_bench_run PUBLIC cobot::pipeline PRIVATE Threads::Threads)
target_compile_definitions(cobot_bench_run PRIVATE COBOT_GIT_DESCRIBE="${COBOT_GIT_DESCRIBE}")

add_executable(cobot_bench main.cpp)
cobot_set_target_defaults(cobot_bench)
target_link_libraries(cobot_bench PRIVATE cobot_bench_run)
install(TARGETS cobot_bench RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
