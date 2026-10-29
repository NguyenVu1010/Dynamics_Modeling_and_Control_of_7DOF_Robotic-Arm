# planning::WaypointTrajectory (feature `waypoint-traj`): geometry, timing, errors, and a
# library-level no-allocation check (replaced global operator new) in its own binary.
cobot_add_test(test_waypoint_trajectory test_waypoint_trajectory.cpp LIBS cobot::planning)
cobot_add_test(test_waypoint_trajectory_no_alloc test_waypoint_trajectory_no_alloc.cpp
               LIBS cobot::planning)
