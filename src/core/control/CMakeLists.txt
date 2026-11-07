# control: the tracking laws (computed torque, PD plus gravity) and their registry, friction
# feed-forward and hold controller.
cobot_add_module(control
  SOURCES src/algorithm.cpp src/computed_torque.cpp src/friction_compensation.cpp src/hold.cpp
          src/pd_gravity.cpp src/tracking_controller.cpp
  DEPS cobot::dynamics cobot::hal cobot::planning)
