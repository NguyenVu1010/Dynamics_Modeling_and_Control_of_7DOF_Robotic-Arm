# Feature drive-loop (ADR-023, cobot_studio phase C PR 3): imperfect::ActuatorStage, the torque
# shaping of ImperfectJointInterface::write moved into a class shared with the simulated drive.
cobot_add_test(test_actuator_stage test_actuator_stage.cpp)
