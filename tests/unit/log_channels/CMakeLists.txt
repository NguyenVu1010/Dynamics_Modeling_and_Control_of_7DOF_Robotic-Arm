# Feature log-channels (cobot_studio design 6.2, ADR-020): the Run CSV channel columns after
# `hold`, CommandMode, PlantTruth, SimPlant::friction_torque() and SimTruthProbe.
cobot_add_test(test_log_channels
  test_log_columns.cpp
  test_friction_torque.cpp
  test_truth_probe.cpp)
