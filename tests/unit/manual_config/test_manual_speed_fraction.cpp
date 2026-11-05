// Feature jog-headroom: the joint jog and plan_joint_move scale the URDF qd_max by
// cobot.manual.joint_speed_fraction f (not a fixed 0.9):
//   joint jog v_max = speed * f * qd_max_j,  plan_joint_move s_v_max = min_j speed f qd_max_j / d_j
// Rig: ideal SimPlant (tests/e2e/configs/ideal_world.yaml) in external mode with the cobot.manual
// block of config/default.yaml, 1 ms steps (tests/unit/manual_loop/manual_loop_util.hpp).
#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include "manual_loop_util.hpp"

namespace cobot::pipeline::test {
namespace {

// Peak |qd_ref_j| of a jog of joint j (0-based), +1, refreshed every 50 steps for `steps` steps.
double jog_peak(const Setup& setup, const Config<kN>& cfg, int j, double speed, int steps) {
  Host h = make_host(setup, cfg);
  EXPECT_TRUE(h.loop().jog_joint(j, +1, speed).is_ok());
  step_refreshed(h, steps, [&] { EXPECT_TRUE(h.loop().jog_joint(j, +1, speed).is_ok()); });
  double peak = 0.0;
  for (const Record& r : h.rows) {
    EXPECT_NE(r.mode, LoopMode::kHolding);
    peak = std::max(peak, std::abs(r.qd_ref(j)));
  }
  return peak;
}

// Joint 1 at 20 %, f = 0.5: v_max = 0.2 * 0.5 * 2.1 = 0.21 rad/s, a = 0.2 * 15 = 3 rad/s^2, ramp
// 0.21 / 3 = 0.07 s = 70 steps (< 300); travel about 0.21 * 0.3 = 0.063 rad from q1 = 0.
// Joint 5 at 10 %, f = 1.0: v_max = 0.1 * 1.0 * 3.1 = 0.31 rad/s, a = 0.1 * 15 = 1.5 rad/s^2,
// ramp 0.31 / 1.5 = 0.2067 s = 207 steps (< 400); travel < 0.31 * 0.4 = 0.124 rad from q5 = 0.
TEST(ManualSpeedFraction, JointJogTargetsSpeedTimesFractionTimesQdMax) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  Config<kN> cfg = manual_config(*setup);
  cfg.manual.joint_speed_fraction = 0.5;
  EXPECT_NEAR(jog_peak(*setup, cfg, 0, 0.2, 300), 0.21, 1e-9);
  cfg.manual.joint_speed_fraction = 1.0;
  EXPECT_NEAR(jog_peak(*setup, cfg, 4, 0.1, 400), 0.31, 1e-9);
}

// manual_plan's first case (home -> q1 + 0.5 rad, q2 - 0.25 rad, speed 0.1), s_a_max = 3.0:
//   f = 1.0: s_v_max = min(0.21 / 0.5, 0.21 / 0.25) = 0.42, duration = 1 / 0.42 + 0.42 / 3 =
//            2.380952 + 0.14 = 2.520952 s
//   f = 0.5: s_v_max = min(0.105 / 0.5, 0.105 / 0.25) = 0.21, duration = 1 / 0.21 + 0.21 / 3 =
//            4.761905 + 0.07 = 4.831905 s
TEST(ManualSpeedFraction, PlanJointMoveUsesTheFraction) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  Config<kN> cfg = manual_config(*setup);
  const Vec q0 = cfg.home_q_rad;
  Vec goal = q0;
  goal(0) += 0.5;
  goal(1) -= 0.25;

  cfg.manual.joint_speed_fraction = 1.0;
  const auto full = plan_joint_move<kN>(planner_model(*setup, cfg), cfg, q0, goal, 0.1);
  ASSERT_TRUE(full) << full.reason();
  EXPECT_NEAR(full.value().s_v_max, 0.42, 1e-12);
  EXPECT_NEAR(full.value().s_a_max, 3.0, 1e-12);
  EXPECT_NEAR(full.value().duration_s, 2.520952, 1e-5);

  cfg.manual.joint_speed_fraction = 0.5;
  const auto half = plan_joint_move<kN>(planner_model(*setup, cfg), cfg, q0, goal, 0.1);
  ASSERT_TRUE(half) << half.reason();
  EXPECT_NEAR(half.value().s_v_max, 0.21, 1e-12);
  EXPECT_NEAR(half.value().s_a_max, 3.0, 1e-12);
  EXPECT_NEAR(half.value().duration_s, 4.831905, 1e-5);
}

}  // namespace
}  // namespace cobot::pipeline::test
