# pipeline::plan_joint_move, JointMove::sample and pipeline::plan_tcp_move (feature
# manual-commands, ADR-022 "Lệnh tay"): the off-thread planners, no ControlLoop.
cobot_add_test(test_manual_plan test_manual_plan.cpp)
