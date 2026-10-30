# Monte Carlo and sweep modes of cobot_bench (feature sensitivity), declared in bench/sweep.hpp.
# They reuse the run engine of bench/run.cpp, so their sources are built into the same static
# library cobot_bench_run (bench/CMakeLists.txt, feature bench-e2e) that cobot_bench links.
# Own directory (not tests/component/bench) so the frozen bench-e2e CMakeLists stays untouched.
cobot_add_test(test_bench_sweep test_bench_sweep.cpp LIBS cobot_bench_run cobot::pipeline)
target_include_directories(test_bench_sweep PRIVATE ${PROJECT_SOURCE_DIR}/src/bench)
