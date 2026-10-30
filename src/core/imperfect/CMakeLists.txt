# imperfect: sensor/actuator/model imperfections for sim-to-real studies.
cobot_add_module(imperfect
  SOURCES src/imperfect_joint_interface.cpp src/perturb.cpp
  DEPS cobot::dynamics cobot::hal)
