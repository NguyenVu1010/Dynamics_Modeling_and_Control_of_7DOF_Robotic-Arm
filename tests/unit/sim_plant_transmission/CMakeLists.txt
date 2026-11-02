# Feature transmission (realism-2 design 6.2): elastic joint with a three-segment Harmonic Drive
# spring, transmission error, lost motion, motor-side encoder; rigid (k3 = 0) equal to today.
cobot_add_test(test_sim_plant_transmission
  test_harmonic_spring.cpp
  test_sim_plant_transmission.cpp
  test_transmission_io.cpp
  test_transmission_config.cpp
  test_rigid_regression.cpp)
