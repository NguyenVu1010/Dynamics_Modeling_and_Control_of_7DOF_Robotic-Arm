# Review gaps of features planning-traj (endpoint continuity) and planning-clik (lower joint
# limit). Own directory so the frozen planning CMakeLists stay as they are.
cobot_add_test(test_planning_limits test_planning_limits.cpp LIBS cobot::planning)
