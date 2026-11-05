// ControlLoop::move_joint (feature manual-commands, ADR-022 "Lệnh tay", synchronised trapezoid):
// q_ref = q_ref0 + s(t) Δ, qd_ref = s' Δ, qdd_ref = s'' Δ with Δ = q_goal - joint_reference()
// (re-based), t = 0 on the first step after the call; at duration_s -> standby at FK(q_goal) with
// kFinished. Joint-mode rows: T_b_d = FK(q_ref), e1..e6 = pose_error(T_b_d, T_b_e). Stop: the
// ADR-018 time scaling over stop_time_s applied to s(t).
//
// The plan of manual_plan's first case, from the loop's joint_reference() (home after 100 standby
// steps): Δ = (+0.5, -0.25, 0, ...) rad at 10 %, joint_speed_fraction 0.9 (feature jog-headroom):
// speed * 0.9 * qd_max = 0.1 * 0.9 * 2.1 = 0.189 rad/s, s_v_max = min(0.189 / 0.5, 0.189 / 0.25)
// = 0.378, s_a_max = min(1.5 / 0.5, 0.75 / 0.25) = 3.0, duration 1 / 0.378 + 0.378 / 3 =
// 2.645503 + 0.126 = 2.771503 s; peak |qd_ref| = 0.378 * (0.5, 0.25) = (0.189, 0.0945) rad/s.
// (Before jog-headroom: s_v_max 0.42, 2.520952 s, (0.21, 0.105) rad/s.)
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/lie.hpp"

#include "manual_loop_util.hpp"

namespace cobot::pipeline::test {
namespace {

JointMove<kN> first_case_plan(const Setup& setup, const Config<kN>& cfg, const Vec& q_start) {
  Vec goal = q_start;
  goal(0) += 0.5;
  goal(1) -= 0.25;
  const auto plan = plan_joint_move<kN>(planner_model(setup, cfg), cfg, q_start, goal, kSpeed);
  EXPECT_TRUE(plan) << plan.reason();
  return plan ? plan.value() : JointMove<kN>{};
}

// First row index (0-based from the first step after move_joint) with |q_ref_j - goal_j| <= tol.
int first_at_goal(const std::vector<Record>& rows, int j, double goal, double tol) {
  for (std::size_t i = 0; i < rows.size(); ++i) {
    if (std::abs(rows[i].q_ref(j) - goal) <= tol) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

TEST(ManualMoveJoint, JointsArriveTogetherAndTheLoopFinishesInStandby) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const Config<kN> cfg = manual_config(*setup);
  Host h = make_host(*setup, cfg);
  const Vec q0 = h.loop().joint_reference();
  const JointMove<kN> mv = first_case_plan(*setup, cfg, q0);
  ASSERT_NEAR(mv.duration_s, 2.771503, 1e-5);

  ASSERT_TRUE(h.loop().move_joint(mv).is_ok());
  const int steps = step_until_standby(h, 4000);
  ASSERT_EQ(h.mode(), LoopMode::kStandby);
  for (int k = 0; k < 20; ++k) {
    h.step();
  }
  EXPECT_EQ(h.loop().manual_stop(), ManualStop::kFinished);
  EXPECT_EQ(h.loop().command_mode(), CommandMode::kMoveJoint);

  // ceil(2.771503 / 0.001) = 2772: row 2772 samples t = 2.772 s >= duration (s = 1); row 2771
  // samples t = 2.771 s, 0.503 ms before the end: 1 - s = 0.5 * 3 * (0.503e-3)^2 = 3.8e-7, so
  // |q_ref_1 - goal_1| = 0.5 * 3.8e-7 = 1.9e-7 there.
  const int k_end = static_cast<int>(std::ceil(mv.duration_s / kDt));
  ASSERT_EQ(k_end, 2772);
  EXPECT_LE(steps, k_end + 2);
  EXPECT_EQ(first_at_goal(h.rows, 0, mv.q_goal(0), 1e-12), k_end);
  EXPECT_EQ(first_at_goal(h.rows, 1, mv.q_goal(1), 1e-12), k_end);
  ASSERT_GT(h.rows.size(), static_cast<std::size_t>(k_end));
  for (int j = 0; j < kN; ++j) {
    EXPECT_LE(std::abs(h.rows[static_cast<std::size_t>(k_end)].q_ref(j) - mv.q_goal(j)), 1e-12)
        << "joint " << j + 1;
  }
  // The standby pose is FK(q_goal).
  EXPECT_LT(pose_max_diff(h.loop().reference_pose(), fk_pose(setup->model, mv.q_goal)), 1e-12);

  double qd1 = 0.0;
  double qd2 = 0.0;
  std::size_t tracking = 0;
  for (std::size_t i = 0; i < h.rows.size(); ++i) {
    const Record& r = h.rows[i];
    ASSERT_NE(r.mode, LoopMode::kHolding) << i;
    EXPECT_EQ(r.cmd_mode, CommandMode::kMoveJoint) << i;
    if (r.mode != LoopMode::kTracking) {
      continue;
    }
    ++tracking;
    qd1 = std::max(qd1, std::abs(r.qd_ref(0)));
    qd2 = std::max(qd2, std::abs(r.qd_ref(1)));
    // Joint-mode rows: T_b_d is the tip pose of q_ref, e is pose_error(T_b_d, T_b_e).
    EXPECT_LT(pose_max_diff(r.T_b_d, fk_pose(setup->model, r.q_ref)), 1e-12) << i;
    const common::Vec6 e = common::pose_error(r.T_b_d, r.T_b_e);
    EXPECT_LT((r.task_error - e).cwiseAbs().maxCoeff(), 1e-12) << i;
    // Straight line in joint space: the joints without motion keep q_ref0.
    for (int j = 2; j < kN; ++j) {
      EXPECT_LE(std::abs(r.q_ref(j) - q0(j)), 1e-12) << "row " << i << " joint " << j + 1;
    }
    // qdd_ref = s'' Δ: joint 2's is -0.5 times joint 1's.
    EXPECT_NEAR(r.qdd_ref(1), -0.5 * r.qdd_ref(0), 1e-12) << i;
  }
  EXPECT_GE(tracking, 2500U);
  EXPECT_NEAR(qd1, 0.189, 1e-9);   // 0.378 * 0.5
  EXPECT_NEAR(qd2, 0.0945, 1e-9);  // 0.378 * 0.25
}

// request_stop() at t = 1.0 s (cruise): s(t) slows down over stop_time_s = 0.5 s, standby within
// 0.5 s + 2 ms; the reference stays on the joint-space line.
TEST(ManualMoveJoint, StopDuringAMoveStaysOnTheLine) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const Config<kN> cfg = manual_config(*setup);
  ASSERT_DOUBLE_EQ(cfg.stop_time_s, 0.5);
  Host h = make_host(*setup, cfg);
  const Vec q0 = h.loop().joint_reference();
  const JointMove<kN> mv = first_case_plan(*setup, cfg, q0);
  ASSERT_TRUE(h.loop().move_joint(mv).is_ok());
  for (int k = 0; k < 1000; ++k) {  // t = 0 .. 0.999 s
    h.step();
  }
  ASSERT_EQ(h.mode(), LoopMode::kTracking);
  h.loop().request_stop();
  const int steps = step_until_standby(h, 2000);
  ASSERT_EQ(h.mode(), LoopMode::kStandby);
  EXPECT_LE(steps, 502);  // stop_time_s + 2 ms
  EXPECT_EQ(h.loop().manual_stop(), ManualStop::kStopped);

  const Record* last = nullptr;
  for (const Record& r : h.rows) {
    ASSERT_NE(r.mode, LoopMode::kHolding);
    if (r.mode == LoopMode::kTracking) {
      last = &r;
    }
  }
  ASSERT_NE(last, nullptr);
  const Vec delta = mv.q_goal - q0;
  const double f1 = (last->q_ref(0) - q0(0)) / delta(0);
  const double f2 = (last->q_ref(1) - q0(1)) / delta(1);
  EXPECT_NEAR(f1, f2, 1e-9);
  // Stopped before the goal, beyond s(1.0 s) = 0.378 * (1.0 - 0.063) = 0.354186 (cruise: s =
  // s_v_max (t - s_v_max / (2 s_a_max)), 0.378 / 6 = 0.063). The smoothstep ramp adds s_v_max *
  // stop_time_s / 2 = 0.378 * 0.25 = 0.0945: s ~ 0.4487 at rest. (Before jog-headroom: 0.3906,
  // 0.4956.)
  EXPECT_GT(f1, 0.354);
  EXPECT_LT(f1, 0.55);
  EXPECT_LT(std::abs(last->qd_ref(0)), 1e-2);
  for (int j = 2; j < kN; ++j) {
    EXPECT_LE(std::abs(last->q_ref(j) - q0(j)), 1e-12) << "joint " << j + 1;
  }
}

// move.q_start must be within 1e-3 rad (max norm) of joint_reference(); the move runs from the
// reference (re-based), so the first q_ref is the reference itself.
TEST(ManualMoveJoint, PlanMustStartAtTheReference) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const Config<kN> cfg = manual_config(*setup);
  Host h = make_host(*setup, cfg);
  const Vec ref = h.loop().joint_reference();

  Vec far = ref;
  far(2) += 2e-3;
  const JointMove<kN> off = first_case_plan(*setup, cfg, far);
  const common::Status st = h.loop().move_joint(off);
  ASSERT_FALSE(st.is_ok());
  EXPECT_TRUE(contains(st.reason(), "does not start at the reference")) << st.reason();
  h.step();
  EXPECT_EQ(h.mode(), LoopMode::kStandby);

  const Vec ref1 = h.loop().joint_reference();
  Vec near = ref1;
  near(2) += 5e-4;
  const JointMove<kN> close = first_case_plan(*setup, cfg, near);
  ASSERT_TRUE(h.loop().move_joint(close).is_ok());
  h.step();
  EXPECT_EQ(h.mode(), LoopMode::kTracking);
  EXPECT_LE((h.rows.back().q_ref - ref1).cwiseAbs().maxCoeff(), 1e-12);  // no step
  for (int k = 0; k < 100; ++k) {
    h.step();
  }
  // Re-based on the reference: joint 3 moves Δ_3 = goal_3 - ref_3 = 5e-4 rad along the profile
  // instead of jumping to the plan's start: |Δq_ref| per step stays far below 1e-3 rad.
  for (std::size_t i = 1; i < h.rows.size(); ++i) {
    EXPECT_LT((h.rows[i].q_ref - h.rows[i - 1].q_ref).cwiseAbs().maxCoeff(), 1e-3) << i;
  }
}

TEST(ManualMoveJoint, MoveOutsideStandbyIsRejected) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const Config<kN> cfg = manual_config(*setup);
  Host h = make_host(*setup, cfg);
  const JointMove<kN> mv = first_case_plan(*setup, cfg, h.loop().joint_reference());
  ASSERT_TRUE(h.loop().jog_joint(0, +1, kSpeed).is_ok());
  h.step();
  EXPECT_FALSE(h.loop().move_joint(mv).is_ok());
  EXPECT_EQ(h.loop().command_mode(), CommandMode::kJogJoint);
}

// Feature jog-headroom: move_joint at 100 % over the full joint-1 range inside the jog band never
// holds. Joint 1 jog band: |q1| <= 2.9671 - 0.0872665 = 2.8798335 rad; goals 1e-3 rad inside it,
// q1 = +-2.8788335. Three moves from the loop's joint_reference(): home -> +2.8788 -> -2.8788 ->
// home, only joint 1 moving, speed 1.0:
//   s_v_max = 1.0 * 0.9 * 2.1 / d = 1.89 / d, s_a_max = 15 / d; s_v_max^2 / s_a_max = 1.89^2 /
//   (15 d) = 0.238 / d < 1 for every d >= 0.25: trapezoids, cruise |qd_ref_1| = 1.89 rad/s.
//   home -> +2.8788: d = 2.8788, duration = 2.8788 / 1.89 + 1.89 / 15 = 1.5232 + 0.126 = 1.649 s
//   +2.8788 -> -2.8788: d = 5.7577, duration = 3.0464 + 0.126 = 3.172 s
// Each ends kFinished in standby within 5000 steps; no row in kHolding; measured max |qd_i| <
// qd_max_i (before jog-headroom the cruise was exactly 2.1 rad/s = qd_max_1: overspeed hold).
TEST(ManualMoveJoint, FullSpeedMoveNeverHolds) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const Config<kN> cfg = manual_config(*setup);
  const Vec qd_max = qd_max_of(setup->model);
  const double band = cfg.safety.soft_margin_rad + cfg.manual.jog_margin_rad;
  ASSERT_NEAR(band, 5.0 * kDegree, 1e-12);
  const double q1_hi = q_max_of(setup->model)(0) - band - 1e-3;
  const double q1_lo = q_min_of(setup->model)(0) + band + 1e-3;
  ASSERT_NEAR(q1_hi, 2.8788335, 1e-6);
  ASSERT_NEAR(q1_lo, -2.8788335, 1e-6);

  Host h = make_host(*setup, cfg);
  const Vec home = h.loop().joint_reference();
  Vec hi = home;
  hi(0) = q1_hi;
  Vec lo = home;
  lo(0) = q1_lo;
  for (const Vec& goal : {hi, lo, home}) {
    SCOPED_TRACE("goal q1 " + std::to_string(goal(0)));
    const Vec start = h.loop().joint_reference();
    const auto plan = plan_joint_move<kN>(planner_model(*setup, cfg), cfg, start, goal, 1.0);
    ASSERT_TRUE(plan) << plan.reason();
    ASSERT_TRUE(h.loop().move_joint(plan.value()).is_ok());
    const int steps = step_until_standby(h, 5000);
    ASSERT_EQ(h.mode(), LoopMode::kStandby)
        << "after " << steps << " steps, hold reason " << to_string(h.loop().hold_reason());
    EXPECT_EQ(h.loop().manual_stop(), ManualStop::kFinished);
    EXPECT_NEAR(h.loop().joint_reference()(0), goal(0), 1e-9);
  }

  Vec qd_seen = Vec::Zero();
  double qd_ref_seen = 0.0;
  std::size_t holding = 0;
  for (const Record& r : h.rows) {
    if (r.mode == LoopMode::kHolding || r.hold_reason != HoldReason::kNone) {
      ++holding;
    }
    qd_seen = qd_seen.cwiseMax(r.qd.cwiseAbs());
    qd_ref_seen = std::max(qd_ref_seen, std::abs(r.qd_ref(0)));
  }
  EXPECT_EQ(holding, 0U);
  for (int i = 0; i < kN; ++i) {
    EXPECT_LT(qd_seen(i), qd_max(i)) << "joint " << i + 1 << " max |qd| " << qd_seen(i);
  }
  EXPECT_NEAR(qd_ref_seen, kJointSpeedFraction * qd_max(0), 1e-9);  // 0.9 * 2.1 = 1.89 rad/s
}

}  // namespace
}  // namespace cobot::pipeline::test
