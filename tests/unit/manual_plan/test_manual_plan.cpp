// Off-thread planners of the manual commands (feature manual-commands, ADR-022 "Lệnh tay" and
// "Đã chốt ở PR manual-commands"): plan_joint_move (synchronised trapezoid), JointMove::sample
// and plan_tcp_move (validate_path with the jog band as joint margin, plan ambiguity 11).
//
// Arm and config: config/default.yaml (cobot.manual block of the brief) on the real arm, the
// controller's model (pipeline::controller_model). URDF: qd_max 2.1 rad/s for joints 1-4, 3.1 for
// 5-7; joint_4 in [-3.1416, -0.4] rad. manual.joint_a_max_rad_s2 = (15, 7.5, 12.5, 12.5, 15, 20,
// 20). Home q = (0, 30, 0, -90, 0, 60, 0) deg. Jog band = soft_margin 3 deg + jog_margin 2 deg =
// 5 deg = 0.0872665 rad. manual.joint_speed_fraction f = 0.9 (feature jog-headroom; the key is
// absent or 0.9 in default.yaml, 0.9 either way).
//
// Synchronised trapezoid, d_j = |q_goal_j - q_start_j|:
//   s_v_max = min_j speed f qd_max_j / d_j,  s_a_max = min_j speed a_max_j / d_j
//   trapezoid when s_v_max^2 / s_a_max < 1: duration = 1 / s_v_max + s_v_max / s_a_max
//   triangle otherwise: duration = 2 sqrt(1 / s_a_max), peak s' = sqrt(s_a_max)
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/kinematics/kinematics.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/manual.hpp"
#include "cobot/pipeline/path_validation.hpp"
#include "cobot/planning/waypoint_trajectory.hpp"

namespace cobot::pipeline {
namespace {

namespace fs = std::filesystem;
using common::Pose;
using common::Vec3;
using Vec7 = common::VecN<7>;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegree = kPi / 180.0;
constexpr double kDt = 1e-3;

struct Arm {
  model::Model<7> model;  // the controller's model
  Config<7> cfg;
};

Arm load_arm() {
  const auto cfg = load_config<7>(fs::path(COBOT_REPO_DIR) / "config/default.yaml");
  EXPECT_TRUE(cfg) << cfg.reason();
  const auto chain =
      model::parse_urdf_chain(cfg.value().urdf_path, cfg.value().base_link, cfg.value().tip_link);
  EXPECT_TRUE(chain) << chain.reason();
  const auto arm = model::make_model<7>(chain.value(), cfg.value().gravity_b);
  EXPECT_TRUE(arm) << arm.reason();
  return Arm{controller_model(arm.value(), cfg.value()), cfg.value()};
}

Pose fk_pose(const model::Model<7>& m, const Vec7& q) {
  kinematics::FkState<7> fk;
  kinematics::forward_kinematics(m, q, fk);
  return fk.T_b_e;
}

bool contains(const std::string& text, const std::string& part) {
  return text.find(part) != std::string::npos;
}

// The first case of the brief: home -> q1 + 0.5 rad, q2 - 0.25 rad at 10 % speed.
//   d = (0.5, 0.25); speed f qd_max = 0.1 * 0.9 * 2.1 = (0.189, 0.189); speed a_max = (1.5, 0.75)
//   s_v_max = min(0.189 / 0.5, 0.189 / 0.25) = min(0.378, 0.756) = 0.378
//   s_a_max = min(1.5 / 0.5, 0.75 / 0.25) = min(3.0, 3.0) = 3.0
//   0.378^2 / 3 = 0.047628 < 1 -> trapezoid, duration = 1 / 0.378 + 0.378 / 3 = 2.645503 + 0.126
//   = 2.771503 s; peak qd = 0.378 d = (0.189, 0.0945) rad/s, peak qdd = 3 d = (1.5, 0.75) rad/s^2.
//   (Before jog-headroom, f = 1: s_v_max 0.42, 2.520952 s, peak qd (0.21, 0.105) rad/s.)
TEST(ManualPlan, JointMoveTrapezoidNumbers) {
  const Arm arm = load_arm();
  const Vec7 q0 = arm.cfg.home_q_rad;
  ASSERT_NEAR(q0(1), 30.0 * kDegree, 1e-12);
  Vec7 goal = q0;
  goal(0) += 0.5;
  goal(1) -= 0.25;
  const auto plan = plan_joint_move<7>(arm.model, arm.cfg, q0, goal, 0.1);
  ASSERT_TRUE(plan) << plan.reason();
  const JointMove<7>& mv = plan.value();
  EXPECT_NEAR(mv.s_v_max, 0.378, 1e-12);
  EXPECT_NEAR(mv.s_a_max, 3.0, 1e-12);
  EXPECT_NEAR(mv.duration_s, 1.0 / 0.378 + 0.378 / 3.0, 1e-5);
  EXPECT_NEAR(mv.duration_s, 2.771503, 1e-5);
  for (int j = 0; j < 7; ++j) {
    EXPECT_EQ(mv.q_start(j), q0(j)) << j;
    EXPECT_EQ(mv.q_goal(j), goal(j)) << j;
  }

  double s = 0.0;
  double s_d = 0.0;
  double s_dd = 0.0;
  mv.sample(0.5 * mv.duration_s, s, s_d, s_dd);
  EXPECT_NEAR(s, 0.5, 1e-12);
  EXPECT_NEAR(s_d, 0.378, 1e-12);  // cruise
  EXPECT_NEAR(s_dd, 0.0, 1e-12);

  // Peaks sampled every 1 ms over [0, duration]: qd_j = s' d_j, qdd_j = s'' d_j.
  const Vec7 d = goal - q0;
  double qd1 = 0.0;
  double qd2 = 0.0;
  double qdd1 = 0.0;
  double qdd2 = 0.0;
  double s_prev = -1.0;
  const int steps = static_cast<int>(std::ceil(mv.duration_s / kDt));
  for (int k = 0; k <= steps; ++k) {
    mv.sample(k * kDt, s, s_d, s_dd);
    EXPECT_GE(s, s_prev) << "s not monotonic at step " << k;
    EXPECT_GE(s, 0.0);
    EXPECT_LE(s, 1.0);
    s_prev = s;
    qd1 = std::max(qd1, std::abs(s_d * d(0)));
    qd2 = std::max(qd2, std::abs(s_d * d(1)));
    qdd1 = std::max(qdd1, std::abs(s_dd * d(0)));
    qdd2 = std::max(qdd2, std::abs(s_dd * d(1)));
  }
  EXPECT_NEAR(qd1, 0.189, 1e-9);   // 0.378 * 0.5
  EXPECT_NEAR(qd2, 0.0945, 1e-9);  // 0.378 * 0.25
  EXPECT_NEAR(qdd1, 1.5, 1e-9);
  EXPECT_NEAR(qdd2, 0.75, 1e-9);
}

// sample() clamps t to [0, duration]: at rest at both ends, s(0) = 0, s(T) = 1, and the profile
// is symmetric: s(t) + s(T - t) = 1.
TEST(ManualPlan, JointMoveSampleEndsAndSymmetry) {
  const Arm arm = load_arm();
  const Vec7 q0 = arm.cfg.home_q_rad;
  Vec7 goal = q0;
  goal(0) += 0.5;
  goal(1) -= 0.25;
  const auto plan = plan_joint_move<7>(arm.model, arm.cfg, q0, goal, 0.1);
  ASSERT_TRUE(plan) << plan.reason();
  const JointMove<7>& mv = plan.value();
  const double T = mv.duration_s;
  double s = 0.0;
  double s_d = 0.0;
  double s_dd = 0.0;
  for (const double t : {-1.0, 0.0}) {
    mv.sample(t, s, s_d, s_dd);
    EXPECT_EQ(s, 0.0) << t;
    EXPECT_EQ(s_d, 0.0) << t;
  }
  for (const double t : {T, T + 1.0}) {
    mv.sample(t, s, s_d, s_dd);
    EXPECT_NEAR(s, 1.0, 1e-12) << t;
    EXPECT_NEAR(s_d, 0.0, 1e-12) << t;
  }
  // During the acceleration phase (t < s_v_max / s_a_max = 0.378 / 3 = 0.126 s): s'' = s_a_max.
  mv.sample(0.05, s, s_d, s_dd);
  EXPECT_NEAR(s_dd, 3.0, 1e-12);
  EXPECT_NEAR(s_d, 3.0 * 0.05, 1e-12);        // 0.15
  EXPECT_NEAR(s, 0.5 * 3.0 * 0.0025, 1e-12);  // 0.00375
  for (const double t : {0.01, 0.1, 0.5, 1.0, 1.7}) {
    double s_a = 0.0;
    double s_b = 0.0;
    mv.sample(t, s_a, s_d, s_dd);
    mv.sample(T - t, s_b, s_d, s_dd);
    EXPECT_NEAR(s_a + s_b, 1.0, 1e-12) << t;
  }
}

// Joint 1 + 0.01 rad: s_v_max = 0.1 * 0.9 * 2.1 / 0.01 = 18.9, s_a_max = 1.5 / 0.01 = 150;
// 18.9^2 / 150 = 357.21 / 150 = 2.3814 >= 1 -> triangle: duration = 2 sqrt(1 / 150) = 0.163299 s,
// peak s' = sqrt(150) = 12.2474 at T / 2, peak qd = 0.01 * 12.2474 = 0.122474 rad/s (below the
// 0.189 rad/s cap).
TEST(ManualPlan, JointMoveTriangleForAShortMove) {
  const Arm arm = load_arm();
  const Vec7 q0 = arm.cfg.home_q_rad;
  Vec7 goal = q0;
  goal(0) += 0.01;
  const auto plan = plan_joint_move<7>(arm.model, arm.cfg, q0, goal, 0.1);
  ASSERT_TRUE(plan) << plan.reason();
  const JointMove<7>& mv = plan.value();
  EXPECT_NEAR(mv.s_a_max, 150.0, 1e-9);
  EXPECT_NEAR(mv.duration_s, 2.0 * std::sqrt(1.0 / 150.0), 1e-6);
  EXPECT_NEAR(mv.duration_s, 0.163299, 1e-6);
  double s = 0.0;
  double s_d = 0.0;
  double s_dd = 0.0;
  mv.sample(0.5 * mv.duration_s, s, s_d, s_dd);
  EXPECT_NEAR(s, 0.5, 1e-9);
  EXPECT_NEAR(s_d * 0.01, 0.122474, 1e-6);
  // Never above the peak anywhere (1 ms grid).
  const int steps = static_cast<int>(std::ceil(mv.duration_s / kDt));
  for (int k = 0; k <= steps; ++k) {
    mv.sample(k * kDt, s, s_d, s_dd);
    EXPECT_LE(s_d * 0.01, 0.122474 + 1e-6) << k;
  }
}

TEST(ManualPlan, JointMoveSpeedOutsideTheUnitIntervalIsAnError) {
  const Arm arm = load_arm();
  const Vec7 q0 = arm.cfg.home_q_rad;
  Vec7 goal = q0;
  goal(0) += 0.5;
  for (const double speed : {0.0, 1.5, -0.1}) {
    const auto plan = plan_joint_move<7>(arm.model, arm.cfg, q0, goal, speed);
    ASSERT_FALSE(plan) << speed;
    EXPECT_TRUE(contains(plan.reason(), "move_joint: speed")) << plan.reason();
    EXPECT_TRUE(contains(plan.reason(), "outside (0, 1]")) << plan.reason();
  }
  // Speed is checked first: a bad speed with a NaN goal reports the speed.
  Vec7 nan_goal = goal;
  nan_goal(2) = std::numeric_limits<double>::quiet_NaN();
  const auto both = plan_joint_move<7>(arm.model, arm.cfg, q0, nan_goal, 1.5);
  ASSERT_FALSE(both);
  EXPECT_TRUE(contains(both.reason(), "speed")) << both.reason();
  // 1.0 is inside (0, 1] (a small wrist move: joint 7 + 0.1 rad, 20 rad/s^2 on a light link).
  Vec7 wrist = q0;
  wrist(6) += 0.1;
  const auto full = plan_joint_move<7>(arm.model, arm.cfg, q0, wrist, 1.0);
  EXPECT_TRUE(full) << full.reason();
}

TEST(ManualPlan, JointMoveNonFiniteGoalNamesTheJoint) {
  const Arm arm = load_arm();
  const Vec7 q0 = arm.cfg.home_q_rad;
  Vec7 goal = q0;
  goal(2) = std::numeric_limits<double>::quiet_NaN();  // joint 3
  const auto plan = plan_joint_move<7>(arm.model, arm.cfg, q0, goal, 0.1);
  ASSERT_FALSE(plan);
  EXPECT_TRUE(contains(plan.reason(), "move_joint: goal joint 3")) << plan.reason();
  EXPECT_TRUE(contains(plan.reason(), "not finite")) << plan.reason();
}

// joint_4 upper limit -0.4 rad; jog band 5 deg = 0.0872665 rad: a goal of -0.45 rad is 0.05 rad
// from the limit -> error. -0.4 - 0.0872665 - 0.01 = -0.4972665 rad is outside the band -> ok.
TEST(ManualPlan, JointMoveGoalInsideTheJogBandIsAnError) {
  const Arm arm = load_arm();
  const double band = arm.cfg.safety.soft_margin_rad + arm.cfg.manual.jog_margin_rad;
  ASSERT_NEAR(band, 0.0872665, 1e-6);  // 0.0523599 + 0.0349066
  ASSERT_NEAR(arm.model.joints[3].q_max, -0.4, 1e-12);
  const Vec7 q0 = arm.cfg.home_q_rad;
  Vec7 goal = q0;
  goal(3) = -0.45;
  const auto plan = plan_joint_move<7>(arm.model, arm.cfg, q0, goal, 0.1);
  ASSERT_FALSE(plan);
  EXPECT_TRUE(contains(plan.reason(), "move_joint: goal joint 4")) << plan.reason();
  EXPECT_TRUE(contains(plan.reason(), "within")) << plan.reason();
  EXPECT_TRUE(contains(plan.reason(), "limit")) << plan.reason();

  goal(3) = -0.4 - band - 0.01;
  const auto outside = plan_joint_move<7>(arm.model, arm.cfg, q0, goal, 0.1);
  EXPECT_TRUE(outside) << outside.reason();
}

TEST(ManualPlan, JointMoveAlreadyAtTheGoalIsAnError) {
  const Arm arm = load_arm();
  const Vec7 q0 = arm.cfg.home_q_rad;
  const auto plan = plan_joint_move<7>(arm.model, arm.cfg, q0, q0, 0.1);
  ASSERT_FALSE(plan);
  EXPECT_TRUE(contains(plan.reason(), "move_joint: already at the goal")) << plan.reason();
  Vec7 tiny = q0;
  tiny(4) += 5e-10;  // <= 1e-9
  const auto tiny_plan = plan_joint_move<7>(arm.model, arm.cfg, q0, tiny, 0.1);
  ASSERT_FALSE(tiny_plan);
  EXPECT_TRUE(contains(tiny_plan.reason(), "already at the goal")) << tiny_plan.reason();
}

// torque_ratio 0.01: |tau| along the profile is far above 1 % of tau_max on some joint (gravity
// alone on joint_2 at home is tens of N m against 0.01 * 204 = 2.04 N m).
TEST(ManualPlan, JointMoveTorqueAboveTheRatioIsAnError) {
  Arm arm = load_arm();
  arm.cfg.path_validation.torque_ratio = 0.01;
  const Vec7 q0 = arm.cfg.home_q_rad;
  Vec7 goal = q0;
  goal(0) += 0.5;
  goal(1) -= 0.25;
  const auto plan = plan_joint_move<7>(arm.model, arm.cfg, q0, goal, 0.1);
  ASSERT_FALSE(plan);
  const std::string reason = plan.reason();
  const auto at = reason.find("move_joint: torque joint ");
  ASSERT_NE(at, std::string::npos) << reason;
  const std::size_t digit = at + std::string("move_joint: torque joint ").size();
  ASSERT_LT(digit, reason.size()) << reason;
  EXPECT_GE(reason[digit], '1') << reason;
  EXPECT_LE(reason[digit], '7') << reason;
  EXPECT_TRUE(contains(reason, "t = ")) << reason;
}

// 0.1 m along +x from the home tip pose at 10 % speed: CartesianLimits = manual tcp limits * 0.1
// = (0.025 m/s, 0.05 m/s^2, 0.1 rad/s, 0.2 rad/s^2). Quintic over d = 0.1 m:
//   peak velocity 1.875 d / T <= 0.025 -> T >= 1.875 * 0.1 / 0.025 = 7.5 s (binds)
//   peak acceleration 5.7735 d / T^2 <= 0.05 -> T >= sqrt(5.7735 * 0.1 / 0.05) = 3.40 s
TEST(ManualPlan, TcpMoveShortStraightLineIsOk) {
  const Arm arm = load_arm();
  const Vec7 q0 = arm.cfg.home_q_rad;
  const Pose start = fk_pose(arm.model, q0);
  const Pose goal{start.R, start.p + Vec3(0.1, 0.0, 0.0)};
  const auto plan = plan_tcp_move<7>(arm.model, arm.cfg, q0, start, goal, 0.1);
  ASSERT_TRUE(plan) << plan.reason();
  const PathValidation<7>& pv = plan.value();
  ASSERT_TRUE(pv.ok);
  EXPECT_TRUE(pv.issues.empty());
  ASSERT_EQ(pv.segment_durations_s.size(), 1U);
  EXPECT_NEAR(pv.segment_durations_s[0], 7.5, 0.05);
  ASSERT_TRUE(pv.trajectory.has_value());
  EXPECT_EQ(pv.trajectory->size(), 1U);
  EXPECT_NEAR(pv.trajectory->duration_s(), 7.5, 0.05);
  EXPECT_FALSE(pv.trajectory->user_duration(0));
  EXPECT_TRUE(pv.trajectory->stops_at(0));
  EXPECT_LT((pv.trajectory->start_pose().p - start.p).norm(), 1e-15);
}

// A goal 3 m away: not ok, with the unreachable issue validate_path reports for the same
// arguments (limits * speed, path_validation with joint_margin = soft + jog margin).
TEST(ManualPlan, TcpMoveUnreachableMatchesValidatePath) {
  const Arm arm = load_arm();
  const Vec7 q0 = arm.cfg.home_q_rad;
  const Pose start = fk_pose(arm.model, q0);
  const Pose goal{start.R, start.p + Vec3(3.0, 0.0, 0.0)};
  const double speed = 0.1;
  const auto plan = plan_tcp_move<7>(arm.model, arm.cfg, q0, start, goal, speed);
  ASSERT_TRUE(plan) << plan.reason();
  ASSERT_FALSE(plan.value().ok);

  const ManualConfig<7>& m = arm.cfg.manual;
  planning::CartesianLimits lim;
  lim.v_max_m_s = m.tcp_v_max_m_s * speed;
  lim.a_max_m_s2 = m.tcp_a_max_m_s2 * speed;
  lim.w_max_rad_s = m.tcp_w_max_rad_s * speed;
  lim.alpha_max_rad_s2 = m.tcp_alpha_max_rad_s2 * speed;
  ValidationLimits vl = arm.cfg.path_validation;
  vl.joint_margin_rad = arm.cfg.safety.soft_margin_rad + m.jog_margin_rad;
  planning::Waypoint w;
  w.T_b_goal = goal;
  w.duration_s = 0.0;
  w.stop = true;
  const auto direct = validate_path<7>(
      arm.model, arm.cfg.clik, arm.cfg.friction_compensation, q0, start, {w}, lim, vl);
  ASSERT_TRUE(direct) << direct.reason();
  ASSERT_FALSE(direct.value().ok);

  const auto find_unreachable = [](const std::vector<WaypointIssue>& issues) {
    return std::find_if(issues.begin(), issues.end(), [](const WaypointIssue& i) {
      return i.issue == PathIssue::kUnreachable;
    });
  };
  const auto& got = plan.value().issues;
  const auto& want = direct.value().issues;
  const auto g = find_unreachable(got);
  const auto e = find_unreachable(want);
  ASSERT_NE(g, got.end());
  ASSERT_NE(e, want.end());
  EXPECT_EQ(g->waypoint, e->waypoint);
  EXPECT_EQ(g->issue, e->issue);
  EXPECT_EQ(g->joint, e->joint);
  EXPECT_DOUBLE_EQ(g->value, e->value);
  EXPECT_DOUBLE_EQ(g->limit, e->limit);
  EXPECT_EQ(got.size(), want.size());
}

// The joint margin of plan_tcp_move is the jog band (5 deg), not path_validation's 2 deg: with
// the jog margin raised to 170 deg the band covers every joint at home, so the same short move
// reports joint_limit issues while validate_path with the plain 2 deg margin does not.
TEST(ManualPlan, TcpMoveUsesTheJogBandAsJointMargin) {
  Arm arm = load_arm();
  const Vec7 q0 = arm.cfg.home_q_rad;
  const Pose start = fk_pose(arm.model, q0);
  const Pose goal{start.R, start.p + Vec3(0.02, 0.0, 0.0)};
  arm.cfg.manual.jog_margin_rad = 170.0 * kDegree;
  const auto plan = plan_tcp_move<7>(arm.model, arm.cfg, q0, start, goal, 0.5);
  ASSERT_TRUE(plan) << plan.reason();
  EXPECT_FALSE(plan.value().ok);
  const auto& issues = plan.value().issues;
  EXPECT_TRUE(std::any_of(issues.begin(), issues.end(), [](const WaypointIssue& i) {
    return i.issue == PathIssue::kJointLimit;
  }));
}

TEST(ManualPlan, TcpMoveSpeedOutsideTheUnitIntervalIsAnError) {
  const Arm arm = load_arm();
  const Vec7 q0 = arm.cfg.home_q_rad;
  const Pose start = fk_pose(arm.model, q0);
  const Pose goal{start.R, start.p + Vec3(0.1, 0.0, 0.0)};
  for (const double speed : {0.0, 1.5}) {
    const auto plan = plan_tcp_move<7>(arm.model, arm.cfg, q0, start, goal, speed);
    EXPECT_FALSE(plan) << speed;
    EXPECT_TRUE(contains(plan.reason(), "speed")) << plan.reason();
  }
}

TEST(ManualPlan, ManualStopStrings) {
  EXPECT_STREQ(to_string(ManualStop::kNone), "none");
  EXPECT_STREQ(to_string(ManualStop::kFinished), "finished");
  EXPECT_STREQ(to_string(ManualStop::kStopped), "stopped");
  EXPECT_STREQ(to_string(ManualStop::kDeadman), "deadman");
  EXPECT_STREQ(to_string(ManualStop::kSoftLimit), "soft_limit");
  EXPECT_STREQ(to_string(ManualStop::kSingularity), "singularity");
}

// The bridge wire uses the enum values.
TEST(ManualPlan, JogEnumValues) {
  EXPECT_EQ(static_cast<int>(JogAxis::kX), 0);
  EXPECT_EQ(static_cast<int>(JogAxis::kY), 1);
  EXPECT_EQ(static_cast<int>(JogAxis::kZ), 2);
  EXPECT_EQ(static_cast<int>(JogAxis::kRx), 3);
  EXPECT_EQ(static_cast<int>(JogAxis::kRy), 4);
  EXPECT_EQ(static_cast<int>(JogAxis::kRz), 5);
  EXPECT_EQ(static_cast<int>(JogFrame::kBase), 0);
  EXPECT_EQ(static_cast<int>(JogFrame::kTool), 1);
}

}  // namespace
}  // namespace cobot::pipeline
