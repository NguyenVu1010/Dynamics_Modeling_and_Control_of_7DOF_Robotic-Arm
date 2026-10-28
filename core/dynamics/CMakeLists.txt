# dynamics: RNEA, CRBA, gravity, Coriolis, forward dynamics, SimPlant.
cobot_add_module(dynamics
  SOURCES src/rnea.cpp src/crba.cpp src/dynamics.cpp
  DEPS cobot::kinematics)
