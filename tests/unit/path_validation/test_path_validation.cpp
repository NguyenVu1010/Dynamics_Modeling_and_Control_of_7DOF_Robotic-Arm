#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "cobot/common/lie.hpp"
#include "cobot/common/types.hpp"
#include "cobot/kinematics/kinematics.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/path_validation.hpp"
#include "cobot/planning/waypoint_trajectory.hpp"

// pipeline::validate_path (feature path-validation, design 2026-10-29-waypoint-gui-design.md 6.2):
// CLIK run offline at 1 ms along WaypointTrajectory::create(T_b_start, waypoints, cartesian) on
// the real arm (sim/urdf/cobot30_5.urdf) with the gains and friction of config/default.yaml.
//
// The scenario numbers below come from running ClikReference along CartesianTrajectory (which a
// single stop waypoint must reproduce) on this arm, with RNEA + friction feed-forward:
//   home q = (0, 30, 0, -90, 0, 60, 0) deg, tip at p = (0.110, -0.807, 0.389) m.
//   MVP goal (-0.5, -0.6, 0.9) m, rpy (0, 0, 90) deg: 0.822 m and 90 deg away. Auto duration with
//     the default CartesianLimits ~6.2 s; worst |qd|/qd_max 0.12, |tau|/tau_max 0.54, joint margin
//     >= 12.7 deg, task error at the node 3.6e-6 m / 6.9e-6 rad -> no issue.
//   home + (0, 0, -0.4) m in 0.3 s: |qd_2| = 1.24 qd_max, |qd_6| = 1.33 qd_max, |tau_2| = 3.5
//     tau_max, task error at the node 3.8e-4 m. The same move in 3.0 s: |qd|/qd_max <= 0.13,
//     |tau|/tau_max <= 0.67 -> ok. |tau_2|/tau_max against T: 0.34 s 2.90, 0.53 s 1.57,
//     0.83 s 1.02, 1.0 s 0.90, 1.04 s 0.88, 1.2 s 0.82: below 0.9 only for T >~ 1.01 s.
//   home + (2, 0, 0) m: the CLIK task error stays > 2 m -> unreachable.
namespace cobot::pipeline {
namespace {

namespace fs = std::filesystem;
using common::Pose;
using common::Vec3;
using common::VecN;
using planning::CartesianLimits;
using planning::Waypoint;
using planning::WaypointTrajectory;

constexpr double kPi = 3.14159265358979323846;
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

struct Arm {
  model::Model<7> model;
  Config<7> cfg;
};

Arm load_arm() {
  Arm out;
  const auto cfg = load_config<7>(fs::path(COBOT_REPO_DIR) / "config/default.yaml");
  EXPECT_TRUE(cfg.is_ok()) << cfg.reason();
  out.cfg = cfg.value();
  const auto chain =
      model::parse_urdf_chain(out.cfg.urdf_path, out.cfg.base_link, out.cfg.tip_link);
  EXPECT_TRUE(chain.is_ok()) << chain.reason();
  const auto m = model::make_model<7>(chain.value(), out.cfg.gravity_b);
  EXPECT_TRUE(m.is_ok()) << m.reason();
  out.model = m.value();
  return out;
}

const Arm& arm() {
  static const Arm instance = load_arm();
  return instance;
}

Pose fk(const VecN<7>& q) {
  kinematics::FkState<7> s;
  kinematics::forward_kinematics(arm().model, q, s);
  return s.T_b_e;
}

VecN<7> home_q() {
  return arm().cfg.home_q_rad;
}
Pose home_pose() {
  return fk(home_q());
}

// config/default.yaml trajectory goal: R = Rz(yaw) Ry(pitch) Rx(roll).
Pose mvp_goal() {
  const TrajectoryConfig& t = arm().cfg.trajectory;
  Pose g;
  g.p = t.goal_position_m;
  g.R = (Eigen::AngleAxisd(t.goal_rpy_rad(2), Vec3::UnitZ()) *
         Eigen::AngleAxisd(t.goal_rpy_rad(1), Vec3::UnitY()) *
         Eigen::AngleAxisd(t.goal_rpy_rad(0), Vec3::UnitX()))
            .toRotationMatrix();
  return g;
}

Pose shifted(const Pose& T, const Vec3& dp) {
  Pose out = T;
  out.p += dp;
  return out;
}

Waypoint wp(const Pose& T_b_goal, double duration_s = 0.0, bool stop = true) {
  Waypoint w;
  w.T_b_goal = T_b_goal;
  w.duration_s = duration_s;
  w.stop = stop;
  return w;
}

common::Result<PathValidation<7>> validate(const std::vector<Waypoint>& waypoints,
                                           const ValidationLimits& limits = ValidationLimits{},
                                           const CartesianLimits& cartesian = CartesianLimits{},
                                           const VecN<7>& q_start = home_q()) {
  return validate_path<7>(arm().model,
                          arm().cfg.clik,
                          arm().cfg.friction_compensation,
                          q_start,
                          home_pose(),
                          waypoints,
                          cartesian,
                          limits);
}

// Duration WaypointTrajectory gives segment i before any stretching.
double unstretched_duration(const std::vector<Waypoint>& waypoints,
                            std::size_t i,
                            const CartesianLimits& cartesian = CartesianLimits{}) {
  const auto t = WaypointTrajectory::create(home_pose(), waypoints, cartesian);
  EXPECT_TRUE(t.is_ok()) << t.reason();
  return t.is_ok() ? t.value().segment_duration_s(i) : kNan;
}

// k such that ratio == 1.25^k (relative 1e-9), or -1.
int stretch_power(double ratio) {
  double f = 1.0;
  for (int k = 0; k <= 64; ++k) {
    if (std::abs(ratio - f) <= 1e-9 * f) {
      return k;
    }
    f *= 1.25;
  }
  return -1;
}

double pos_dist(const Pose& a, const Pose& b) {
  return (a.p - b.p).norm();
}
double rot_dist(const Pose& a, const Pose& b) {
  return common::so3_log(a.R.transpose() * b.R).norm();
}

const WaypointIssue* find_issue(const PathValidation<7>& v, std::size_t waypoint, PathIssue kind) {
  for (const WaypointIssue& i : v.issues) {
    if (i.waypoint == waypoint && i.issue == kind) {
      return &i;
    }
  }
  return nullptr;
}

bool has_issue(const PathValidation<7>& v, std::size_t waypoint, PathIssue kind) {
  return find_issue(v, waypoint, kind) != nullptr;
}

std::string describe(const PathValidation<7>& v) {
  std::string s;
  for (const WaypointIssue& i : v.issues) {
    s += "[wp " + std::to_string(i.waypoint) + " " + to_string(i.issue) + " joint " +
         std::to_string(i.joint) + " value " + std::to_string(i.value) + " limit " +
         std::to_string(i.limit) + "] ";
  }
  return s;
}

// One issue per (waypoint, kind).
void expect_unique_kinds(const PathValidation<7>& v) {
  for (std::size_t a = 0; a < v.issues.size(); ++a) {
    for (std::size_t b = a + 1; b < v.issues.size(); ++b) {
      EXPECT_FALSE(v.issues[a].waypoint == v.issues[b].waypoint &&
                   v.issues[a].issue == v.issues[b].issue)
          << "duplicate issue: " << describe(v);
    }
  }
}

// Compares bit patterns, so -0.0 vs 0.0 or two NaNs are told apart.
bool bit_equal(double a, double b) {
  std::uint64_t x = 0;
  std::uint64_t y = 0;
  std::memcpy(&x, &a, sizeof(x));
  std::memcpy(&y, &b, sizeof(y));
  return x == y;
}

bool bit_equal(const Pose& a, const Pose& b) {
  for (int i = 0; i < 9; ++i) {
    if (!bit_equal(a.R.data()[i], b.R.data()[i])) {
      return false;
    }
  }
  for (int i = 0; i < 3; ++i) {
    if (!bit_equal(a.p(i), b.p(i))) {
      return false;
    }
  }
  return true;
}

// Fast Cartesian limits make the auto duration of the 0.4 m move short. For a quintic:
//   max(1.875 * 0.4 / 1.5, sqrt(5.7735 * 0.4 / 10)) = max(0.5, 0.48) = 0.5 s,
// where |tau_2| ~ 1.6 tau_max (header table): a stretch factor of about 2 or more is needed.
CartesianLimits fast_cartesian() {
  CartesianLimits c;
  c.v_max_m_s = 1.5;
  c.a_max_m_s2 = 10.0;
  c.w_max_rad_s = 8.0;
  c.alpha_max_rad_s2 = 16.0;
  return c;
}

TEST(PathValidation, IssueNames) {
  EXPECT_STREQ(to_string(PathIssue::kUnreachable), "unreachable");
  EXPECT_STREQ(to_string(PathIssue::kJointLimit), "joint_limit");
  EXPECT_STREQ(to_string(PathIssue::kJointVelocity), "joint_velocity");
  EXPECT_STREQ(to_string(PathIssue::kTorque), "torque");
}

TEST(PathValidation, DefaultLimitsMatchTheDesign) {
  const ValidationLimits l;
  EXPECT_DOUBLE_EQ(l.reach_pos_m, 1e-3);                    // 1 mm
  EXPECT_DOUBLE_EQ(l.reach_rot_rad, 0.5 * kPi / 180.0);     // 0.5 deg = 8.727e-3 rad
  EXPECT_DOUBLE_EQ(l.joint_margin_rad, 2.0 * kPi / 180.0);  // 2 deg = 3.491e-2 rad
  EXPECT_DOUBLE_EQ(l.torque_ratio, 0.9);
  EXPECT_DOUBLE_EQ(l.max_stretch, 3.0);
  EXPECT_DOUBLE_EQ(l.dt_s, 1e-3);
  EXPECT_DOUBLE_EQ(l.path_sample_dt_s, 0.01);
}

TEST(PathValidation, MvpGoalFromHomeIsOk) {
  const Pose start = home_pose();
  const Pose goal = mvp_goal();
  const std::vector<Waypoint> wps{wp(goal)};
  const auto r = validate(wps);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const PathValidation<7>& v = r.value();
  EXPECT_TRUE(v.ok) << describe(v);
  EXPECT_TRUE(v.issues.empty()) << describe(v);
  ASSERT_EQ(v.segment_durations_s.size(), 1U);
  // No issue -> no stretching: the auto duration (~6.2 s) is kept.
  EXPECT_DOUBLE_EQ(v.segment_durations_s[0], unstretched_duration(wps, 0));
  ASSERT_TRUE(v.trajectory.has_value());
  const WaypointTrajectory& t = *v.trajectory;
  EXPECT_EQ(t.size(), 1U);
  EXPECT_DOUBLE_EQ(t.segment_duration_s(0), v.segment_durations_s[0]);
  EXPECT_DOUBLE_EQ(t.duration_s(), v.segment_durations_s[0]);
  EXPECT_LT(pos_dist(t.start_pose(), start), 1e-12);
  EXPECT_LT(rot_dist(t.start_pose(), start), 1e-12);

  ASSERT_FALSE(v.path.empty());
  EXPECT_LT(pos_dist(v.path.front(), start), 1e-6);
  EXPECT_LT(rot_dist(v.path.front(), start), 1e-6);
  EXPECT_LT(pos_dist(v.path.back(), goal), 1e-6);
  EXPECT_LT(rot_dist(v.path.back(), goal), 1e-6);
  // One pose every 0.01 s over [0, T], both ends included: T / 0.01 + 1 samples, +-1 for a last
  // partial step (T = 6.166 s -> 617.6).
  EXPECT_NEAR(static_cast<double>(v.path.size()), v.segment_durations_s[0] / 0.01 + 1.0, 1.5);
  for (const Pose& p : v.path) {
    ASSERT_TRUE(p.p.allFinite());
    ASSERT_TRUE(p.R.allFinite());
  }
}

TEST(PathValidation, PathSampleSpacingFollowsTheLimit) {
  ValidationLimits limits;
  limits.path_sample_dt_s = 0.1;
  const auto r = validate({wp(mvp_goal())}, limits);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const PathValidation<7>& v = r.value();
  ASSERT_TRUE(v.ok) << describe(v);
  ASSERT_EQ(v.segment_durations_s.size(), 1U);
  // T / 0.1 + 1 samples (T = 6.166 s -> 62.7).
  EXPECT_NEAR(static_cast<double>(v.path.size()), v.segment_durations_s[0] / 0.1 + 1.0, 1.5);
  EXPECT_LT(pos_dist(v.path.front(), home_pose()), 1e-6);
  EXPECT_LT(pos_dist(v.path.back(), mvp_goal()), 1e-6);
}

TEST(PathValidation, PointTwoMetresAwayIsUnreachable) {
  const std::vector<Waypoint> wps{wp(shifted(home_pose(), Vec3(2.0, 0.0, 0.0)))};
  const auto r = validate(wps);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const PathValidation<7>& v = r.value();
  EXPECT_FALSE(v.ok);
  EXPECT_FALSE(v.trajectory.has_value());
  const WaypointIssue* i = find_issue(v, 0, PathIssue::kUnreachable);
  ASSERT_NE(i, nullptr) << describe(v);
  EXPECT_EQ(i->joint, -1);
  EXPECT_GT(i->value, i->limit);
  for (const WaypointIssue& x : v.issues) {
    EXPECT_EQ(x.waypoint, 0U);
  }
  expect_unique_kinds(v);
  // unreachable is never fixed by stretching: the auto duration is kept.
  ASSERT_EQ(v.segment_durations_s.size(), 1U);
  EXPECT_DOUBLE_EQ(v.segment_durations_s[0], unstretched_duration(wps, 0));
}

TEST(PathValidation, PointNearJoint2LimitIsJointLimitOnJoint2) {
  // Target = FK(q*), q* = home with q_2 = 1.82 rad and q_4 = -1.57 rad: p = (0.110, -0.441,
  // -0.556) m. joint_2 upper limit 1.8326 rad: 1.8326 - 1.82 = 0.0126 rad = 0.72 deg < 2 deg.
  // CLIK from home reaches q_2 = 1.821 rad (0.69 deg from the limit); every other joint stays
  // more than 50 deg from its limits, |tau|/tau_max <= 0.64, |qd|/qd_max <= 0.16, task error
  // at the node 2.5e-6 m.
  VecN<7> q_star = home_q();
  q_star(1) = 1.82;
  q_star(3) = -1.57;
  const std::vector<Waypoint> wps{wp(fk(q_star))};
  const auto r = validate(wps);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const PathValidation<7>& v = r.value();
  EXPECT_FALSE(v.ok);
  EXPECT_FALSE(v.trajectory.has_value());
  const WaypointIssue* i = find_issue(v, 0, PathIssue::kJointLimit);
  ASSERT_NE(i, nullptr) << describe(v);
  EXPECT_EQ(i->joint, 1);  // 0-based: joint_2
  expect_unique_kinds(v);
  // joint_limit is never fixed by stretching.
  ASSERT_EQ(v.segment_durations_s.size(), 1U);
  EXPECT_DOUBLE_EQ(v.segment_durations_s[0], unstretched_duration(wps, 0));
}

TEST(PathValidation, ShortUserDurationIsReportedAndNeverStretched) {
  // 0.4 m straight down in 0.3 s: peak tip speed 1.875 * 0.4 / 0.3 = 2.5 m/s;
  // |qd_6| = 1.33 qd_max and |tau_2| = 3.5 tau_max on the reference.
  const Pose goal = shifted(home_pose(), Vec3(0.0, 0.0, -0.4));
  ValidationLimits limits;
  limits.max_stretch = 20.0;  // even a large allowance must not touch a user duration
  const auto r = validate({wp(goal, 0.3)}, limits);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const PathValidation<7>& v = r.value();
  EXPECT_FALSE(v.ok);
  EXPECT_FALSE(v.trajectory.has_value());
  EXPECT_TRUE(has_issue(v, 0, PathIssue::kJointVelocity) || has_issue(v, 0, PathIssue::kTorque))
      << describe(v);
  for (const WaypointIssue& x : v.issues) {
    EXPECT_EQ(x.waypoint, 0U);
    if (x.issue == PathIssue::kJointVelocity || x.issue == PathIssue::kTorque) {
      ASSERT_GE(x.joint, 0);
      ASSERT_LT(x.joint, 7);
      EXPECT_GT(x.value, x.limit) << describe(v);
    }
    if (x.issue == PathIssue::kJointVelocity) {
      // rad/s against the URDF velocity limit of that joint (2.5 or 3 rad/s).
      EXPECT_DOUBLE_EQ(x.limit, arm().model.joints[static_cast<std::size_t>(x.joint)].qd_max);
    }
  }
  expect_unique_kinds(v);
  ASSERT_EQ(v.segment_durations_s.size(), 1U);
  EXPECT_EQ(v.segment_durations_s[0], 0.3);
}

TEST(PathValidation, SameMoveWithAutoDurationIsOk) {
  // Auto duration with the default CartesianLimits, quintic: max(1.875 * 0.4 / 0.25,
  // sqrt(5.7735 * 0.4 / 0.5)) = max(3.0, 2.15) = 3.0 s; worst |tau|/tau_max 0.67 -> no stretch
  // needed, but a stretch by 1.25^k <= 3 is accepted too.
  const Pose goal = shifted(home_pose(), Vec3(0.0, 0.0, -0.4));
  const std::vector<Waypoint> wps{wp(goal)};
  const auto r = validate(wps);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const PathValidation<7>& v = r.value();
  EXPECT_TRUE(v.ok) << describe(v);
  EXPECT_TRUE(v.issues.empty()) << describe(v);
  ASSERT_TRUE(v.trajectory.has_value());
  ASSERT_EQ(v.segment_durations_s.size(), 1U);
  const double auto_s = unstretched_duration(wps, 0);
  EXPECT_GE(v.segment_durations_s[0], auto_s);
  const int k = stretch_power(v.segment_durations_s[0] / auto_s);
  EXPECT_GE(k, 0) << v.segment_durations_s[0] << " / " << auto_s;
  EXPECT_LE(std::pow(1.25, k), 3.0);
  EXPECT_DOUBLE_EQ(v.trajectory->segment_duration_s(0), v.segment_durations_s[0]);
  ASSERT_FALSE(v.path.empty());
  EXPECT_LT(pos_dist(v.path.back(), goal), 1e-6);
}

TEST(PathValidation, AutoDurationTooFastIsStretched) {
  const Pose goal = shifted(home_pose(), Vec3(0.0, 0.0, -0.4));
  const std::vector<Waypoint> wps{wp(goal)};
  const CartesianLimits cart = fast_cartesian();
  const double auto_s = unstretched_duration(wps, 0, cart);
  ASSERT_LT(auto_s, 0.9) << "precondition: the auto duration must be too fast for this test";

  ValidationLimits limits;
  limits.max_stretch = 20.0;  // 1.25^13 = 18.2 <= 20 < 1.25^14 = 22.7: up to 13 steps
  const auto r = validate(wps, limits, cart);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const PathValidation<7>& v = r.value();
  EXPECT_TRUE(v.ok) << describe(v);
  EXPECT_TRUE(v.issues.empty()) << describe(v);
  ASSERT_EQ(v.segment_durations_s.size(), 1U);
  const int k = stretch_power(v.segment_durations_s[0] / auto_s);
  EXPECT_GE(k, 1) << v.segment_durations_s[0] << " / " << auto_s;
  EXPECT_LE(std::pow(1.25, k), 20.0);
  // The first factor that passes: |tau_2| < 0.9 tau_max needs T >~ 1.0 s (header table), and
  // T / 1.25 failed, so T < ~1.01 * 1.25 = 1.26 s. Bounds with 0.05 s slack: (0.95, 1.31).
  EXPECT_GT(v.segment_durations_s[0], 0.95);
  EXPECT_LT(v.segment_durations_s[0], 1.31);
  ASSERT_TRUE(v.trajectory.has_value());
  EXPECT_DOUBLE_EQ(v.trajectory->segment_duration_s(0), v.segment_durations_s[0]);
  EXPECT_DOUBLE_EQ(v.trajectory->duration_s(), v.segment_durations_s[0]);
  ASSERT_FALSE(v.path.empty());
  EXPECT_LT(pos_dist(v.path.back(), goal), 1e-6);
}

TEST(PathValidation, StretchingStopsAtMaxStretch) {
  // max_stretch = 1: the first step (x1.25) already exceeds it, so the issue is reported with the
  // auto duration unchanged.
  const Pose goal = shifted(home_pose(), Vec3(0.0, 0.0, -0.4));
  const std::vector<Waypoint> wps{wp(goal)};
  const CartesianLimits cart = fast_cartesian();
  const double auto_s = unstretched_duration(wps, 0, cart);
  ASSERT_LT(auto_s, 0.9) << "precondition: the auto duration must be too fast for this test";

  ValidationLimits limits;
  limits.max_stretch = 1.0;
  const auto r = validate(wps, limits, cart);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const PathValidation<7>& v = r.value();
  EXPECT_FALSE(v.ok);
  EXPECT_FALSE(v.trajectory.has_value());
  EXPECT_TRUE(has_issue(v, 0, PathIssue::kTorque) || has_issue(v, 0, PathIssue::kJointVelocity))
      << describe(v);
  ASSERT_EQ(v.segment_durations_s.size(), 1U);
  EXPECT_DOUBLE_EQ(v.segment_durations_s[0], auto_s);
}

TEST(PathValidation, SecondBadWaypointIsTheOnlyOneReported) {
  // Waypoint 0: 0.4 m down, stop, auto (ok on its own, see above). Waypoint 1: 2 m further along x.
  const Pose a = shifted(home_pose(), Vec3(0.0, 0.0, -0.4));
  const Pose b = shifted(a, Vec3(2.0, 0.0, 0.0));
  const std::vector<Waypoint> wps{wp(a, 0.0, true), wp(b)};
  const auto r = validate(wps);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const PathValidation<7>& v = r.value();
  EXPECT_FALSE(v.ok);
  EXPECT_FALSE(v.trajectory.has_value());
  ASSERT_FALSE(v.issues.empty());
  EXPECT_TRUE(has_issue(v, 1, PathIssue::kUnreachable)) << describe(v);
  for (const WaypointIssue& x : v.issues) {
    EXPECT_EQ(x.waypoint, 1U) << describe(v);
  }
  expect_unique_kinds(v);
  ASSERT_EQ(v.segment_durations_s.size(), 2U);
  EXPECT_DOUBLE_EQ(v.segment_durations_s[0], unstretched_duration(wps, 0));
}

TEST(PathValidation, TwoGoodWaypointsAreOk) {
  // home -> 0.4 m down (stop) -> MVP goal; both segments auto.
  const Pose a = shifted(home_pose(), Vec3(0.0, 0.0, -0.4));
  const std::vector<Waypoint> wps{wp(a, 0.0, true), wp(mvp_goal())};
  const auto r = validate(wps);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const PathValidation<7>& v = r.value();
  EXPECT_TRUE(v.ok) << describe(v);
  ASSERT_EQ(v.segment_durations_s.size(), 2U);
  ASSERT_TRUE(v.trajectory.has_value());
  EXPECT_EQ(v.trajectory->size(), 2U);
  for (std::size_t i = 0; i < 2; ++i) {
    EXPECT_DOUBLE_EQ(v.trajectory->segment_duration_s(i), v.segment_durations_s[i]);
  }
  EXPECT_NEAR(
      v.trajectory->duration_s(), v.segment_durations_s[0] + v.segment_durations_s[1], 1e-9);
  ASSERT_FALSE(v.path.empty());
  EXPECT_LT(pos_dist(v.path.back(), mvp_goal()), 1e-6);
}

TEST(PathValidation, InvalidWaypointsGiveTheTrajectoryError) {
  const auto empty = validate({});
  EXPECT_FALSE(empty.is_ok());
  EXPECT_FALSE(empty.reason().empty());

  const auto bad = validate({wp(mvp_goal(), kNan)});
  ASSERT_FALSE(bad.is_ok());
  EXPECT_NE(bad.reason().find("waypoints[0].duration_s"), std::string::npos) << bad.reason();

  CartesianLimits cart;
  cart.v_max_m_s = 0.0;
  const auto bad_cart = validate({wp(mvp_goal())}, ValidationLimits{}, cart);
  EXPECT_FALSE(bad_cart.is_ok());
}

TEST(PathValidation, NonFiniteStartIsAnError) {
  for (const double x : {kNan, kInf, -kInf}) {
    VecN<7> q = home_q();
    q(3) = x;
    const auto r = validate({wp(mvp_goal())}, ValidationLimits{}, CartesianLimits{}, q);
    ASSERT_FALSE(r.is_ok());
    EXPECT_NE(r.reason().find("q_start"), std::string::npos) << r.reason();
  }
}

TEST(PathValidation, InvalidLimitsNameTheField) {
  struct Field {
    const char* name;
    double ValidationLimits::*member;
  };
  const std::vector<Field> fields{
      {"reach_pos_m", &ValidationLimits::reach_pos_m},
      {"reach_rot_rad", &ValidationLimits::reach_rot_rad},
      {"joint_margin_rad", &ValidationLimits::joint_margin_rad},
      {"torque_ratio", &ValidationLimits::torque_ratio},
      {"max_stretch", &ValidationLimits::max_stretch},
      {"dt_s", &ValidationLimits::dt_s},
      {"path_sample_dt_s", &ValidationLimits::path_sample_dt_s},
  };
  for (const Field& f : fields) {
    for (const double bad : {0.0, -1.0, kNan, kInf}) {
      ValidationLimits limits;
      limits.*f.member = bad;
      const auto r = validate({wp(mvp_goal())}, limits);
      ASSERT_FALSE(r.is_ok()) << f.name << " = " << bad;
      EXPECT_NE(r.reason().find(f.name), std::string::npos) << f.name << ": " << r.reason();
    }
  }
}

void expect_identical(const PathValidation<7>& va, const PathValidation<7>& vb) {
  EXPECT_EQ(va.ok, vb.ok);
  ASSERT_EQ(va.segment_durations_s.size(), vb.segment_durations_s.size());
  for (std::size_t i = 0; i < va.segment_durations_s.size(); ++i) {
    EXPECT_TRUE(bit_equal(va.segment_durations_s[i], vb.segment_durations_s[i])) << i;
  }
  ASSERT_EQ(va.path.size(), vb.path.size());
  for (std::size_t i = 0; i < va.path.size(); ++i) {
    ASSERT_TRUE(bit_equal(va.path[i], vb.path[i])) << "path sample " << i;
  }
  ASSERT_EQ(va.issues.size(), vb.issues.size());
  for (std::size_t i = 0; i < va.issues.size(); ++i) {
    EXPECT_EQ(va.issues[i].waypoint, vb.issues[i].waypoint);
    EXPECT_EQ(va.issues[i].issue, vb.issues[i].issue);
    EXPECT_EQ(va.issues[i].joint, vb.issues[i].joint);
    EXPECT_TRUE(bit_equal(va.issues[i].value, vb.issues[i].value));
    EXPECT_TRUE(bit_equal(va.issues[i].limit, vb.issues[i].limit));
  }
  EXPECT_EQ(va.trajectory.has_value(), vb.trajectory.has_value());
}

TEST(PathValidation, DeterministicAfterStretching) {
  // Fast limits and a large allowance: ok only after the stretching loop (see above).
  const std::vector<Waypoint> wps{wp(shifted(home_pose(), Vec3(0.0, 0.0, -0.4)))};
  ValidationLimits limits;
  limits.max_stretch = 20.0;
  const auto a = validate(wps, limits, fast_cartesian());
  const auto b = validate(wps, limits, fast_cartesian());
  ASSERT_TRUE(a.is_ok()) << a.reason();
  ASSERT_TRUE(b.is_ok()) << b.reason();
  ASSERT_TRUE(a.value().ok) << describe(a.value());
  ASSERT_FALSE(a.value().path.empty());
  expect_identical(a.value(), b.value());
}

TEST(PathValidation, DeterministicWithPassThroughWaypoint) {
  const std::vector<Waypoint> wps{wp(shifted(home_pose(), Vec3(0.0, 0.0, -0.4)), 0.0, false),
                                  wp(mvp_goal())};
  const auto a = validate(wps);
  const auto b = validate(wps);
  ASSERT_TRUE(a.is_ok()) << a.reason();
  ASSERT_TRUE(b.is_ok()) << b.reason();
  expect_identical(a.value(), b.value());
}

}  // namespace
}  // namespace cobot::pipeline
