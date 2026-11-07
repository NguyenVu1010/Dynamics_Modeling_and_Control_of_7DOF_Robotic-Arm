# Feature drive-loop (ADR-023, cobot_studio phase C PR 3): position commands (hal::DriveMode) through
# ImperfectJointInterface: servo hold and command delay only, no torque shaping.
cobot_add_test(test_position_mode test_position_mode.cpp)
