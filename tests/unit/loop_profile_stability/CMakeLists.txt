# Feature profiles (realism-2 design 8, stability on each profile; design 4.6, hold on budget):
# the default trajectory on premium_fr3, mid_kr1205 and budget_xarm7 with seeds 1-3, wired as
# cobot_bench wires a run (perturbed URDF plant with the profile's plant effects, SimJointInterface
# with jitter and overrun, ImperfectJointInterface, ControlLoop on config/default.yaml), with the
# true link angle read from the plant. Each run is 12000 elastic plant steps: release builds only.
cobot_add_test(test_loop_profile_stability test_profile_stability.cpp)
