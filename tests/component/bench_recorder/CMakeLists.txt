# The bench run engine on a Recorder (feature pipeline-recorder, cobot_studio design 6.3): CSV
# written after the loop, the in-memory log limit, timing() with log.mode off.
cobot_add_test(test_bench_recorder test_bench_recorder.cpp LIBS cobot_bench_run cobot::pipeline)
target_include_directories(test_bench_recorder PRIVATE ${PROJECT_SOURCE_DIR}/src/bench)
