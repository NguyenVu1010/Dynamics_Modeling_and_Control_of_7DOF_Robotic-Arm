# Feature payload-inertia: payload as a solid sphere at an offset in the tip frame, merged into
# the last link by the parallel-axis theorem; YAML keys model.payload_offset_m and
# model.payload_radius_m. Links cobot::pipeline (the default), which brings every core module.
cobot_add_test(test_imperfect_payload test_payload_merge.cpp test_payload_config.cpp)
