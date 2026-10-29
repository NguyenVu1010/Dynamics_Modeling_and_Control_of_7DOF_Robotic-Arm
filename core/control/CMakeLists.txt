# control: computed torque, friction feed-forward and hold controller.
cobot_add_module(control
  SOURCES src/computed_torque.cpp src/friction_compensation.cpp src/hold.cpp
  DEPS cobot::dynamics cobot::hal cobot::planning)
