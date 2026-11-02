# Feature actuator-limits (realism-2 design 6.1): decorator effects and their config keys.
cobot_add_test(test_actuator_limits test_actuator_limits.cpp LIBS cobot::imperfect)
cobot_add_test(test_actuator_limits_config test_actuator_limits_config.cpp)
