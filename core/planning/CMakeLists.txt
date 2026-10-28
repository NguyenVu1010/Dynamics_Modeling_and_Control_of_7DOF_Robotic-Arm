# planning: Cartesian trajectory and CLIK joint reference.
cobot_add_module(planning SOURCES src/cartesian_trajectory.cpp DEPS cobot::kinematics)
