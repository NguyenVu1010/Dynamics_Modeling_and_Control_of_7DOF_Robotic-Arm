# planning: Cartesian trajectory and CLIK joint reference.
cobot_add_module(planning SOURCES src/cartesian_trajectory.cpp src/clik.cpp
                  src/waypoint_trajectory.cpp DEPS cobot::kinematics)
