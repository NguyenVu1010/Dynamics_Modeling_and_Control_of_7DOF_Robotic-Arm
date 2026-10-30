# kinematics: forward kinematics, geometric Jacobian, Jdot*qdot.
cobot_add_module(kinematics SOURCES src/kinematics.cpp DEPS cobot::model)
