# Feature comm-timing (realism-2 design 5, 6.3): servo-rate hold, packet loss and latency bursts
# in ImperfectJointInterface, and the comm/overrun config keys.
cobot_add_test(test_imperfect_comm test_imperfect_comm.cpp LIBS cobot::imperfect)
cobot_add_test(test_comm_config test_comm_config.cpp)
# All-zero comm and overrun against a golden recorded before the feature (tests/data/imperfect_comm).
cobot_add_test(test_comm_golden test_comm_golden.cpp)
