# Feature drive-loop (ADR-023, cobot_studio phase C PR 3): pipeline::SimDrive, the simulated joint
# drive (position loop on the motor encoder at drive.rate_hz, actuator chain, plant sub-steps), and
# SimJointInterface::attach_drive.
cobot_add_test(test_sim_drive test_sim_drive.cpp test_sim_joint_interface_drive.cpp)

# Replaces the global operator new: its own binary.
cobot_add_test(test_sim_drive_no_alloc test_sim_drive_no_alloc.cpp)
