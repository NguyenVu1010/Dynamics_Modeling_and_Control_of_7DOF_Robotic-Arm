# The run engine of `cobot_bench run` lives in bench/run.cpp (declared in bench/run.hpp) and is
# built by bench/CMakeLists.txt as the static library cobot_bench_run, which cobot_bench links.
cobot_add_test(test_bench_run test_bench_run.cpp LIBS cobot_bench_run cobot::pipeline)
target_include_directories(test_bench_run PRIVATE ${PROJECT_SOURCE_DIR}/src/bench)
