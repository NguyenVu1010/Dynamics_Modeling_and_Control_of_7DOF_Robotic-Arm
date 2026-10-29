# Review gaps of features imperfect-io (signed zero with plant effects, qd noise statistics) and
# sim-plant (lower joint stop, negative effects, payload after the mass scale). Own directory so
# the frozen imperfect and sim_plant CMakeLists stay as they are.
cobot_add_test(test_imperfect_plant_gaps test_imperfect_plant_gaps.cpp LIBS cobot::imperfect)
