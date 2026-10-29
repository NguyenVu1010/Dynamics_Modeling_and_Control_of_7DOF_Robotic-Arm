// planning::WaypointTrajectory (feature `waypoint-traj`, design
// docs/design/2026-10-29-waypoint-gui-design.md section 6.1): exact pass-through, C1 at gliding
// points, rest at the start, at stop points and at the end, SO(3) the shorter way, auto durations
// inside the Cartesian limits, user durations as given, one point equal to CartesianTrajectory.
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "cobot/common/lie.hpp"
#include "cobot/common/result.hpp"
#include "cobot/common/types.hpp"
#include "cobot/planning/cartesian_trajectory.hpp"
#include "cobot/planning/waypoint_trajectory.hpp"

namespace cobot::planning {
namespace {

using common::Mat3;
using common::Pose;
using common::so3_exp;
using common::so3_log;
using common::Vec3;
using Built = common::Result<WaypointTrajectory>;

constexpr double kPi = 3.14159265358979323846;
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
static_assert(kMaxWaypoints == 32);

Mat3 axis_angle(double angle, const Vec3& axis) {
  return Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
}

Pose pose(const Mat3& R, const Vec3& p) {
  Pose out;
  out.R = R;
  out.p = p;
  return out;
}

double rot_dist(const Mat3& A, const Mat3& B) {
  return so3_log(A.transpose() * B).norm();
}

Waypoint wp(const Pose& T, double duration_s = 0.0, bool stop = false) {
  Waypoint w;
  w.T_b_goal = T;
  w.duration_s = duration_s;
  w.stop = stop;
  return w;
}

Pose start() {
  return pose(axis_angle(0.7, Vec3(1.0, 2.0, 3.0)), Vec3(0.1, -0.2, 0.5));
}

// Auto, user 1.5 s, auto + stop, user 2.0 s, auto (the last point; its stop flag is false).
std::vector<Waypoint> mixed_points() {
  return {wp(pose(start().R * so3_exp(Vec3(0.0, 0.0, 0.5)), Vec3(0.4, -0.1, 0.6))),
          wp(pose(axis_angle(1.0, Vec3(0.0, 1.0, 1.0)), Vec3(0.5, 0.2, 0.5)), 1.5),
          wp(pose(axis_angle(2.0, Vec3(1.0, 0.0, 0.0)), Vec3(0.3, 0.4, 0.7)), 0.0, true),
          wp(pose(axis_angle(0.3, Vec3(1.0, 1.0, 1.0)), Vec3(0.0, 0.3, 0.8)), 2.0),
          wp(pose(axis_angle(-0.8, Vec3(0.0, 0.0, 1.0)), Vec3(-0.2, 0.1, 0.6)))};
}

// The same points with every duration computed from the limits.
std::vector<Waypoint> auto_points() {
  std::vector<Waypoint> pts = mixed_points();
  for (Waypoint& w : pts) {
    w.duration_s = 0.0;
  }
  return pts;
}

Built make(const std::vector<Waypoint>& pts, const CartesianLimits& lim = CartesianLimits{}) {
  return WaypointTrajectory::create(start(), pts, lim);
}

void expect_error(const Built& r, const std::string& part) {
  ASSERT_FALSE(r.is_ok()) << "expected an error naming " << part;
  EXPECT_NE(r.reason().find(part), std::string::npos) << r.reason();
}

bool at_rest(const CartesianSample& s) {
  return s.xd_d.norm() < 1e-9 && s.xdd_d.norm() < 1e-9;
}

TEST(WaypointTrajectory, AcceptsUpToMaxWaypointsAndRejectsEmptyOrMore) {
  expect_error(make({}), "waypoints");
  std::vector<Waypoint> pts;
  for (std::size_t i = 0; i <= kMaxWaypoints; ++i) {
    const double x = 0.01 * static_cast<double>(i);
    pts.push_back(wp(pose(axis_angle(10.0 * x, Vec3::UnitZ()), Vec3(0.3 + x, 0.0, 0.5))));
  }
  expect_error(make(pts), "waypoints");  // 33 points
  pts.pop_back();
  const Built r = make(pts);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().size(), kMaxWaypoints);
}

TEST(WaypointTrajectory, ErrorsNameTheOffendingWaypoint) {
  const auto edited = [](std::size_t i, auto edit) {
    std::vector<Waypoint> pts = mixed_points();
    edit(pts[i]);
    return make(pts);
  };
  expect_error(edited(2, [](Waypoint& w) { w.T_b_goal.p.y() = kNan; }), "waypoints[2]");
  expect_error(edited(4, [](Waypoint& w) { w.T_b_goal.p.x() = kInf; }), "waypoints[4]");
  expect_error(edited(1, [](Waypoint& w) { w.T_b_goal.R(0, 1) = kNan; }), "waypoints[1]");
  expect_error(edited(3, [](Waypoint& w) { w.duration_s = kNan; }), "waypoints[3].duration_s");
  expect_error(edited(0, [](Waypoint& w) { w.duration_s = kInf; }), "waypoints[0].duration_s");
  // Adding d to R(1,2) changes (R^T R)(j,2) by R(1,j) d for j != 2; the largest |R(1,j)| of a
  // unit row is >= 1/sqrt(3), so the error is >= 1e-5 / sqrt(3) = 5.8e-6 > 1e-6.
  expect_error(edited(2, [](Waypoint& w) { w.T_b_goal.R(1, 2) += 1e-5; }), "waypoints[2]");
  // R^T R = 1.001^2 I = 1.002001 I: error 2.0e-3.
  expect_error(edited(0, [](Waypoint& w) { w.T_b_goal.R *= 1.001; }), "waypoints[0]");
  // d = 1e-8 changes R^T R by at most 2 d + d^2 = 2e-8 < 1e-6: accepted.
  const Built near = edited(2, [](Waypoint& w) { w.T_b_goal.R(1, 2) += 1e-8; });
  EXPECT_TRUE(near.is_ok()) << near.reason();
  // A negative duration means "compute it".
  const Built neg = edited(1, [](Waypoint& w) { w.duration_s = -1.0; });
  ASSERT_TRUE(neg.is_ok()) << neg.reason();
  EXPECT_FALSE(neg.value().user_duration(1));
  EXPECT_GT(neg.value().segment_duration_s(1), 0.0);
}

TEST(WaypointTrajectory, ErrorsNameTheOffendingLimit) {
  for (const double bad : {0.0, -1.0, kNan, kInf}) {
    CartesianLimits lim;
    lim.v_max_m_s = bad;
    expect_error(make(mixed_points(), lim), "v_max_m_s");
    lim = CartesianLimits{};
    lim.a_max_m_s2 = bad;
    expect_error(make(mixed_points(), lim), "a_max_m_s2");
    lim = CartesianLimits{};
    lim.w_max_rad_s = bad;
    expect_error(make(mixed_points(), lim), "w_max_rad_s");
    lim = CartesianLimits{};
    lim.alpha_max_rad_s2 = bad;
    expect_error(make(mixed_points(), lim), "alpha_max_rad_s2");
  }
}

TEST(WaypointTrajectory, NodeTimesDurationsAndFlags) {
  std::vector<Waypoint> pts = mixed_points();
  pts[0].duration_s = 0.01;  // below the 0.1 s auto minimum and beyond the limits: used as given
  const Built r = make(pts);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const WaypointTrajectory& traj = r.value();
  ASSERT_EQ(traj.size(), 5U);
  EXPECT_TRUE(traj.start_pose().p == start().p && traj.start_pose().R == start().R);
  EXPECT_EQ(traj.segment_duration_s(0), 0.01);
  EXPECT_EQ(traj.segment_duration_s(1), 1.5);
  EXPECT_EQ(traj.segment_duration_s(3), 2.0);
  const bool stops[] = {false, false, true, false, true};  // the last point always stops
  double prev = 0.0;
  for (std::size_t i = 0; i < traj.size(); ++i) {
    EXPECT_GT(traj.node_time_s(i), prev) << "i = " << i;
    EXPECT_NEAR(traj.node_time_s(i) - prev, traj.segment_duration_s(i), 1e-12) << "i = " << i;
    EXPECT_EQ(traj.waypoint(i).duration_s, traj.segment_duration_s(i)) << "i = " << i;
    EXPECT_TRUE(traj.waypoint(i).T_b_goal.p == pts[i].T_b_goal.p) << "i = " << i;
    EXPECT_TRUE(traj.waypoint(i).T_b_goal.R == pts[i].T_b_goal.R) << "i = " << i;
    EXPECT_EQ(traj.user_duration(i), pts[i].duration_s > 0.0) << "i = " << i;
    EXPECT_EQ(traj.stops_at(i), stops[i]) << "i = " << i;
    prev = traj.node_time_s(i);
  }
  EXPECT_EQ(traj.duration_s(), traj.node_time_s(4));
}

TEST(WaypointTrajectory, SegmentAtBoundaries) {
  const Built r = make(mixed_points());
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const WaypointTrajectory& traj = r.value();
  EXPECT_EQ(traj.segment_at(-1.0), 0U);
  EXPECT_EQ(traj.segment_at(0.0), 0U);
  for (std::size_t i = 0; i < traj.size(); ++i) {
    const double tn = traj.node_time_s(i);
    EXPECT_EQ(traj.segment_at(tn - 1e-6), i) << "i = " << i;
    EXPECT_EQ(traj.segment_at(tn), i + 1) << "a node time belongs to the next segment, i = " << i;
  }
  EXPECT_EQ(traj.segment_at(traj.duration_s() + 1.0), traj.size());
}

TEST(WaypointTrajectory, PassesExactlyThroughEveryWaypoint) {
  for (const auto& pts : {mixed_points(), auto_points()}) {
    const Built r = make(pts);
    ASSERT_TRUE(r.is_ok()) << r.reason();
    for (std::size_t i = 0; i < pts.size(); ++i) {
      const CartesianSample s = r.value().sample(r.value().node_time_s(i));
      EXPECT_LT((s.T_b_d.p - pts[i].T_b_goal.p).norm(), 1e-9) << "i = " << i;
      EXPECT_LT(rot_dist(s.T_b_d.R, pts[i].T_b_goal.R), 1e-9) << "i = " << i;
    }
  }
}

TEST(WaypointTrajectory, RestAtStartStopPointsAndEndAndHoldsOutside) {
  const std::vector<Waypoint> pts = mixed_points();
  const Built r = make(pts);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const WaypointTrajectory& traj = r.value();
  const double T = traj.duration_s();
  EXPECT_TRUE(at_rest(traj.sample(0.0)));
  EXPECT_TRUE(at_rest(traj.sample(traj.node_time_s(2))));  // the stop point
  EXPECT_TRUE(at_rest(traj.sample(T)));
  for (const double t : {-5.0, -1e-3}) {
    const CartesianSample s = traj.sample(t);
    EXPECT_LT((s.T_b_d.p - start().p).norm(), 1e-9) << "t = " << t;
    EXPECT_LT(rot_dist(s.T_b_d.R, start().R), 1e-9) << "t = " << t;
    EXPECT_TRUE(at_rest(s)) << "t = " << t;
  }
  for (const double t : {T + 1e-3, T + 5.0}) {
    const CartesianSample s = traj.sample(t);
    EXPECT_LT((s.T_b_d.p - pts[4].T_b_goal.p).norm(), 1e-9) << "t = " << t;
    EXPECT_LT(rot_dist(s.T_b_d.R, pts[4].T_b_goal.R), 1e-9) << "t = " << t;
    EXPECT_TRUE(at_rest(s)) << "t = " << t;
  }
}

TEST(WaypointTrajectory, TwistContinuousAtGlidingPoints) {
  const Built r = make(auto_points());
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const WaypointTrajectory& traj = r.value();
  // Within the limits the twist moves by at most 2e-7 s * (0.5 + 1.6) = 4.2e-7 < 1e-6 across
  // the 2e-7 s window, so a larger jump is a discontinuity.
  for (std::size_t i = 0; i + 1 < traj.size(); ++i) {
    if (traj.stops_at(i)) {
      continue;
    }
    const double tn = traj.node_time_s(i);
    const common::Vec6 jump = traj.sample(tn + 1e-7).xd_d - traj.sample(tn - 1e-7).xd_d;
    EXPECT_LT(jump.norm(), 1e-6) << "i = " << i;
  }
}

TEST(WaypointTrajectory, GlidesThroughIntermediatePoints) {
  // Collinear points: without a stop flag the tip keeps moving at the middle point.
  const Vec3 x(0.2, 0.0, 0.0);
  const Built r =
      make({wp(pose(start().R, start().p + x)), wp(pose(start().R, start().p + 2.0 * x))});
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_GT(r.value().sample(r.value().node_time_s(0)).xd_d.head<3>().norm(), 1e-3);
}

TEST(WaypointTrajectory, TwistAndDerivativeMatchCentralDifferences) {
  const Built r = make(mixed_points());
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const WaypointTrajectory& traj = r.value();
  const double h = 1e-6;
  for (std::size_t i = 0; i < traj.size(); ++i) {
    for (const double f : {0.13, 0.5, 0.87}) {
      const double t = traj.node_time_s(i) - (1.0 - f) * traj.segment_duration_s(i);
      const CartesianSample sp = traj.sample(t + h);
      const CartesianSample sm = traj.sample(t - h);
      const CartesianSample s = traj.sample(t);
      const Vec3 v_num = (sp.T_b_d.p - sm.T_b_d.p) / (2.0 * h);
      const Vec3 w_num = so3_log(sp.T_b_d.R * sm.T_b_d.R.transpose()) / (2.0 * h);
      EXPECT_LT((s.xd_d.head<3>() - v_num).cwiseAbs().maxCoeff(), 1e-6) << "t = " << t;
      EXPECT_LT((s.xd_d.tail<3>() - w_num).cwiseAbs().maxCoeff(), 1e-6) << "t = " << t;
      const common::Vec6 a_num = (sp.xd_d - sm.xd_d) / (2.0 * h);
      EXPECT_LT((s.xdd_d - a_num).cwiseAbs().maxCoeff(), 1e-5) << "t = " << t;
    }
  }
}

TEST(WaypointTrajectory, AutoDurationsRespectCartesianLimits) {
  CartesianLimits tight;
  tight.v_max_m_s = 0.1;
  tight.a_max_m_s2 = 0.2;
  tight.w_max_rad_s = 0.3;
  tight.alpha_max_rad_s2 = 0.4;
  for (const CartesianLimits& lim : {CartesianLimits{}, tight}) {
    const Built r = make(auto_points(), lim);
    ASSERT_TRUE(r.is_ok()) << r.reason();
    const WaypointTrajectory& traj = r.value();
    const double k = 1.0 + 1e-6;
    double used = 0.0;  // largest fraction of any limit reached
    for (int n = 0; n * 1e-3 <= traj.duration_s() + 1e-3; ++n) {
      const double t = n * 1e-3;
      const CartesianSample s = traj.sample(t);
      ASSERT_LE(s.xd_d.head<3>().norm(), lim.v_max_m_s * k) << "t = " << t;
      ASSERT_LE(s.xdd_d.head<3>().norm(), lim.a_max_m_s2 * k) << "t = " << t;
      ASSERT_LE(s.xd_d.tail<3>().norm(), lim.w_max_rad_s * k) << "t = " << t;
      ASSERT_LE(s.xdd_d.tail<3>().norm(), lim.alpha_max_rad_s2 * k) << "t = " << t;
      used = std::max({used,
                       s.xd_d.head<3>().norm() / lim.v_max_m_s,
                       s.xdd_d.head<3>().norm() / lim.a_max_m_s2,
                       s.xd_d.tail<3>().norm() / lim.w_max_rad_s,
                       s.xdd_d.tail<3>().norm() / lim.alpha_max_rad_s2});
    }
    EXPECT_GT(used, 0.5) << "auto durations far longer than the binding limit needs";
  }
}

TEST(WaypointTrajectory, ShortAndZeroLengthSegmentsTakeTheMinimumAndRest) {
  const Pose s = start();
  const Pose b = pose(s.R * so3_exp(Vec3(0.3, 0.0, 0.0)), s.p + Vec3(0.2, 0.1, 0.0));
  const Pose tiny = pose(b.R, b.p + Vec3(1e-4, 0.0, 0.0));
  // Segments 0 and 2 have zero length, segment 3 is 0.1 mm long.
  const Built r = make({wp(s), wp(b), wp(b), wp(tiny), wp(pose(s.R, s.p + Vec3(0.0, 0.2, 0.1)))});
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const WaypointTrajectory& traj = r.value();
  for (std::size_t i = 0; i < traj.size(); ++i) {
    EXPECT_GE(traj.segment_duration_s(i), 0.1 - 1e-12) << "i = " << i;
  }
  for (const std::size_t i : {std::size_t{0}, std::size_t{2}}) {
    const Pose& at = i == 0 ? s : b;
    for (int k = 0; k <= 100; ++k) {
      const double t = traj.node_time_s(i) - traj.segment_duration_s(i) * k / 100.0;
      const CartesianSample smp = traj.sample(t);
      ASSERT_LT((smp.T_b_d.p - at.p).norm(), 1e-9) << "i = " << i << ", t = " << t;
      ASSERT_LT(rot_dist(smp.T_b_d.R, at.R), 1e-9) << "i = " << i << ", t = " << t;
      ASSERT_TRUE(at_rest(smp)) << "i = " << i << ", t = " << t;
    }
  }
  for (int n = 0; n * 1e-3 <= traj.duration_s(); ++n) {
    const CartesianSample smp = traj.sample(n * 1e-3);
    ASSERT_TRUE(smp.T_b_d.p.allFinite() && smp.T_b_d.R.allFinite()) << "t = " << n * 1e-3;
    ASSERT_TRUE(smp.xd_d.allFinite() && smp.xdd_d.allFinite()) << "t = " << n * 1e-3;
  }
}

TEST(WaypointTrajectory, RotationTakesTheShorterWay) {
  // 270 deg about z equals -90 deg: no sample may be more than 90 deg from the start.
  const Pose goal = pose(start().R * axis_angle(1.5 * kPi, Vec3::UnitZ()), start().p);
  const Built r = make({wp(goal, 0.0, true)});
  ASSERT_TRUE(r.is_ok()) << r.reason();
  for (int k = 0; k <= 1000; ++k) {
    const double t = r.value().duration_s() * k / 1000.0;
    ASSERT_LE(rot_dist(start().R, r.value().sample(t).T_b_d.R), 0.5 * kPi + 1e-9) << "t = " << t;
  }
}

TEST(WaypointTrajectory, NearHalfTurnsStayFiniteAndSmooth) {
  // Consecutive points 179.9 deg = pi - 1.745e-3 rad apart about z, gliding through the first.
  const double a = 179.9 / 180.0 * kPi;
  const Built r = make({wp(pose(start().R * axis_angle(a, Vec3::UnitZ()), start().p)),
                        wp(pose(start().R * axis_angle(2.0 * a, Vec3::UnitZ()), start().p))});
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const WaypointTrajectory& traj = r.value();
  const double w_max = CartesianLimits{}.w_max_rad_s;
  Mat3 prev = start().R;
  for (int n = 0; n * 1e-3 <= traj.duration_s() + 1e-3; ++n) {
    const CartesianSample s = traj.sample(n * 1e-3);
    ASSERT_TRUE(s.T_b_d.R.allFinite() && s.xd_d.allFinite() && s.xdd_d.allFinite()) << n;
    ASSERT_LE(s.xd_d.tail<3>().norm(), w_max * (1.0 + 1e-6)) << "t = " << n * 1e-3;
    // No flip between 1 ms samples: at most w_max * 1 ms = 8e-4 rad (+1 % margin).
    ASSERT_LE(rot_dist(prev, s.T_b_d.R), w_max * 1e-3 * 1.01) << "t = " << n * 1e-3;
    prev = s.T_b_d.R;
  }
  EXPECT_LT(rot_dist(traj.sample(traj.node_time_s(0)).T_b_d.R, traj.waypoint(0).T_b_goal.R), 1e-9);
}

TEST(WaypointTrajectory, OnePointWithDurationEqualsCartesianTrajectory) {
  const Pose goal =
      pose(start().R * so3_exp(1.2 * Vec3(0.3, -0.5, 0.8).normalized()), Vec3(-0.4, 0.3, 0.9));
  const double T = 2.0;
  const CartesianTrajectory ref(start(), goal, T);
  for (const bool stop : {true, false}) {  // the last point stops either way
    const Built r = make({wp(goal, T, stop)});
    ASSERT_TRUE(r.is_ok()) << r.reason();
    EXPECT_EQ(r.value().duration_s(), T);
    for (int k = -100; k <= 2100; ++k) {  // t in [-0.1, T + 0.1] at 1 ms
      const double t = k * 1e-3;
      const CartesianSample a = r.value().sample(t);
      const CartesianSample b = ref.sample(t);
      ASSERT_LT((a.T_b_d.p - b.T_b_d.p).norm(), 1e-9) << "t = " << t;
      ASSERT_LT((a.T_b_d.R - b.T_b_d.R).cwiseAbs().maxCoeff(), 1e-9) << "t = " << t;
      ASSERT_LT((a.xd_d - b.xd_d).cwiseAbs().maxCoeff(), 1e-9) << "t = " << t;
      ASSERT_LT((a.xdd_d - b.xdd_d).cwiseAbs().maxCoeff(), 1e-9) << "t = " << t;
    }
  }
}

}  // namespace
}  // namespace cobot::planning
