# pipeline::Recorder<N> on HeapStorage (feature pipeline-recorder, cobot_studio design 6.1,
# ADR-020): full policies, buffer swap with a concurrent exporter, CSV export. The allocation
# test replaces the global operator new, so it is a binary of its own.
cobot_add_test(test_recorder test_recorder.cpp test_recorder_csv.cpp)
cobot_add_test(test_recorder_no_alloc test_recorder_no_alloc.cpp)
