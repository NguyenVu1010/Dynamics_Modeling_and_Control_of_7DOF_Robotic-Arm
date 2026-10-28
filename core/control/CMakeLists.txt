# control: computed torque and hold controller.
cobot_add_module(control
  SOURCES src/computed_torque.cpp src/hold.cpp
  DEPS cobot::dynamics cobot::hal cobot::planning)
