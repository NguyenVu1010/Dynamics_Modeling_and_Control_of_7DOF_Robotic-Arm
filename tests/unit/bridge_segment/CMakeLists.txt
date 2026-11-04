# Segment export of the studio bridge (feature controller-bridge, cobot_studio design 6.5, plan
# ambiguity 6): bridge::copy_csv_range copies the header and the [t0, t1] rows of a Run CSV.
# Linked by the real target name, so a missing module is a link error, not a configure error.
cobot_add_test(test_bridge_segment test_bridge_segment.cpp LIBS cobot_bridge cobot::pipeline)
