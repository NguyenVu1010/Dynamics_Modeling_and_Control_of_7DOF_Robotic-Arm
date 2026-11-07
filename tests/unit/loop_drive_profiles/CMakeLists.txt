# Feature drive-loop (ADR-023 prerequisite, cobot_studio phase C PR 3): the budget re-check with
# the drive-side position loop (config/budget_xarm7_drive.yaml, seeds 1-3, margin gates), the ideal
# world and the premium / mid profiles with drive_position, through bench::run_in_memory (static
# library cobot_bench_run). Each run is 12000 elastic control cycles with 4 drive sub-steps: release
# builds only, as tests/unit/loop_profile_stability.
cobot_add_test(test_drive_profiles test_drive_profiles.cpp LIBS cobot_bench_run cobot::pipeline)
target_include_directories(test_drive_profiles PRIVATE ${PROJECT_SOURCE_DIR}/src/bench)
