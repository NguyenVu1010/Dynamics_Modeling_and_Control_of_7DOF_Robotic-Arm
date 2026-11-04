// ControlLoop::move_joint (feature manual-commands, ADR-022 "Lệnh tay", synchronised trapezoid):
// q_ref = q_ref0 + s(t) Δ, qd_ref = s' Δ, qdd_ref = s'' Δ with Δ = q_goal - joint_reference()
// (re-based), t = 0 on the first step after the call; at duration_s -> standby at FK(q_goal) with
// kFinished. Joint-mode rows: T_b_d = FK(q_ref), e1..e6 = pose_error(T_b_d, T_b_e). Stop: the
// ADR-018 time scaling over stop_time_s applied to s(t).
//
// The plan of manual_plan's first case, from the loop's joint_reference() (home after 100 standby
// steps): Δ = (+0.5, -0.25, 0, ...) rad at 10 %: s_v_max 0.42, s_a_max 3.0, duration
// 1 / 0.42 + 0.42 / 3 = 2.520952 s; peak |qd_ref| = (0.21, 0.105) rad/s.
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
  ASSERT_NEAR(mv.duration_s, 2.520952, 1e-5);

  ASSERT_TRUE(h.loop().move_joint(mv).is_ok());
  const int steps = step_until_standby(h, 4000);
  ASSERT_EQ(h.mode(), LoopMode::kStandby);
  for (int k = 0; k < 20; ++k) {
    h.step();
  }
  EXPECT_EQ(h.loop().manual_stop(), ManualStop::kFinished);
  EXPECT_EQ(h.loop().command_mode(), CommandMode::kMoveJoint);

  // ceil(2.520952 / 0.001) = 2521: row 2521 samples t = 2.521 s >= duration (s = 1); row 2520
  // samples t = 2.520 s, 0.95 ms before the end: 1 - s = 0.5 * 3 * (0.95e-3)^2 = 1.4e-6, so
  // |q_ref_1 - goal_1| = 6.8e-7 there.
  const int k_end = static_cast<int>(std::ceil(mv.duration_s / kDt));
  ASSERT_EQ(k_end, 2521);
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
  EXPECT_NEAR(qd1, 0.21, 1e-9);
  EXPECT_NEAR(qd2, 0.105, 1e-9);
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
  // Stopped before the goal, beyond s(1.0 s) = 0.42 * (1.0 - 0.07) = 0.3906 (cruise: s =
  // s_v_max (t - s_v_max / (2 s_a_max))). The smoothstep ramp adds s_v_max * stop_time_s / 2 =
  // 0.105: s ~ 0.4956 at rest.
  EXPECT_GT(f1, 0.39);
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

}  // namespace
}  // namespace cobot::pipeline::test
