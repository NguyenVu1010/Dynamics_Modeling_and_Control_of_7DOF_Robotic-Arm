#include <algorithm>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "cobot/common/lie.hpp"
#include "cobot/common/types.hpp"
#include "cobot/planning/cartesian_trajectory.hpp"

namespace cobot::planning {
namespace {

using common::Mat3;
using common::so3_exp;
using common::so3_log;
using common::Vec3;

constexpr double kPi = 3.14159265358979323846;

Mat3 axis_angle(double angle, const Vec3& axis) {
  return Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
}

// Generic test problem: arbitrary start orientation, rotation of 1.2 rad about a skew axis,
// dp = (-0.5, 0.5, 0.4), |dp| = sqrt(0.25 + 0.25 + 0.16) = sqrt(0.66), T = 2 s.
struct Problem {
  Pose start;
  Pose goal;
  Vec3 dp;
  Vec3 phi;  // so3_log(R0^T RT): rotation vector in the start frame
  double T = 2.0;
};

Problem problem() {
  Problem pr;
  pr.start.R = axis_angle(0.7, Vec3(1.0, 2.0, 3.0));
  pr.start.p = Vec3(0.1, -0.2, 0.5);
  pr.phi = 1.2 * Vec3(0.3, -0.5, 0.8).normalized();
  pr.goal.R = pr.start.R * so3_exp(pr.phi);
  pr.goal.p = Vec3(-0.4, 0.3, 0.9);
  pr.dp = pr.goal.p - pr.start.p;
  return pr;
}

double max_abs(const Mat3& m) {
  return m.cwiseAbs().maxCoeff();
}

// Angular velocity in b from two rotations 2h apart: so3_log(R+ R-^T) / (2h).
Vec3 omega_central(const Mat3& R_plus, const Mat3& R_minus, double h) {
  return so3_log(R_plus * R_minus.transpose()) / (2.0 * h);
}

TEST(CartesianTrajectory, DurationIsStored) {
  const Problem pr = problem();
  const CartesianTrajectory traj(pr.start, pr.goal, pr.T);
  EXPECT_EQ(traj.duration_s(), pr.T);
}

TEST(CartesianTrajectory, EndpointsExactWithZeroDerivatives) {
  const Problem pr = problem();
  const CartesianTrajectory traj(pr.start, pr.goal, pr.T);

  const CartesianSample s0 = traj.sample(0.0);
  EXPECT_LT((s0.T_b_d.p - pr.start.p).norm(), 1e-12);
  EXPECT_LT(max_abs(s0.T_b_d.R - pr.start.R), 1e-12);
  EXPECT_LT(s0.xd_d.norm(), 1e-12);
  EXPECT_LT(s0.xdd_d.norm(), 1e-12);

  const CartesianSample s_end = traj.sample(pr.T);
  EXPECT_LT((s_end.T_b_d.p - pr.goal.p).norm(), 1e-12);
  EXPECT_LT(max_abs(s_end.T_b_d.R - pr.goal.R), 1e-12);
  EXPECT_LT(s_end.xd_d.norm(), 1e-12);
  EXPECT_LT(s_end.xdd_d.norm(), 1e-12);
}

TEST(CartesianTrajectory, HoldsStartBeforeZeroAndGoalAfterDuration) {
  const Problem pr = problem();
  const CartesianTrajectory traj(pr.start, pr.goal, pr.T);
  for (const double t : {-100.0, -1.0, -1e-3}) {
    const CartesianSample s = traj.sample(t);
    EXPECT_LT((s.T_b_d.p - pr.start.p).norm(), 1e-12) << "t = " << t;
    EXPECT_LT(max_abs(s.T_b_d.R - pr.start.R), 1e-12) << "t = " << t;
    EXPECT_LT(s.xd_d.norm(), 1e-12) << "t = " << t;
    EXPECT_LT(s.xdd_d.norm(), 1e-12) << "t = " << t;
  }
  for (const double t : {pr.T + 1e-3, pr.T + 1.0, 1000.0}) {
    const CartesianSample s = traj.sample(t);
    EXPECT_LT((s.T_b_d.p - pr.goal.p).norm(), 1e-12) << "t = " << t;
    EXPECT_LT(max_abs(s.T_b_d.R - pr.goal.R), 1e-12) << "t = " << t;
    EXPECT_LT(s.xd_d.norm(), 1e-12) << "t = " << t;
    EXPECT_LT(s.xdd_d.norm(), 1e-12) << "t = " << t;
  }
}

TEST(CartesianTrajectory, MidpointIsHalfwayInPositionAndAngle) {
  const Problem pr = problem();
  const CartesianTrajectory traj(pr.start, pr.goal, pr.T);
  // s(1/2) = 10/8 - 15/16 + 6/32 = 1.25 - 0.9375 + 0.1875 = 0.5
  const CartesianSample s = traj.sample(0.5 * pr.T);
  const Vec3 mid = 0.5 * (pr.start.p + pr.goal.p);
  EXPECT_LT((s.T_b_d.p - mid).norm(), 1e-12);
  // Half the angle about the same axis: so3_log(R0^T R(T/2)) = phi / 2.
  const Vec3 half = so3_log(pr.start.R.transpose() * s.T_b_d.R);
  EXPECT_LT((half - 0.5 * pr.phi).norm(), 1e-12);
  EXPECT_NEAR(half.norm(), 0.6, 1e-12);  // 1.2 rad / 2
  EXPECT_LT(half.normalized().cross(pr.phi.normalized()).norm(), 1e-12);
}

TEST(CartesianTrajectory, QuarterPointKnownValues) {
  const Problem pr = problem();
  const CartesianTrajectory traj(pr.start, pr.goal, pr.T);
  // tau = 1/4:
  //   s         = 10/64 - 15/256 + 6/1024 = 0.15625 - 0.05859375 + 0.005859375 = 0.103515625
  //   ds/dtau   = 30/16 - 60/64 + 30/256  = 1.875 - 0.9375 + 0.1171875 = 1.0546875
  //   d2s/dtau2 = 60/4 - 180/16 + 120/64  = 15 - 11.25 + 1.875 = 5.625
  // With T = 2 s: sdot = 1.0546875 / 2 1/s, sddot = 5.625 / 4 1/s^2.
  const double s_val = 0.103515625;
  const double sd = 1.0546875 / pr.T;
  const double sdd = 5.625 / (pr.T * pr.T);
  const CartesianSample s = traj.sample(0.25 * pr.T);

  EXPECT_LT((s.T_b_d.p - (pr.start.p + s_val * pr.dp)).norm(), 1e-12);
  EXPECT_LT(max_abs(s.T_b_d.R - pr.start.R * so3_exp(s_val * pr.phi)), 1e-12);

  const Vec3 w_b = pr.start.R * pr.phi;  // rotation vector expressed in b
  EXPECT_LT((s.xd_d.head<3>() - sd * pr.dp).norm(), 1e-12);
  EXPECT_LT((s.xd_d.tail<3>() - sd * w_b).norm(), 1e-12);
  EXPECT_LT((s.xdd_d.head<3>() - sdd * pr.dp).norm(), 1e-12);
  EXPECT_LT((s.xdd_d.tail<3>() - sdd * w_b).norm(), 1e-12);
}

TEST(CartesianTrajectory, PathIsTheStraightSegment) {
  const Problem pr = problem();
  const CartesianTrajectory traj(pr.start, pr.goal, pr.T);
  const Vec3 u = pr.dp.normalized();
  const double len = pr.dp.norm();
  double prev_along = 0.0;
  for (int k = 0; k <= 1000; ++k) {
    const double t = pr.T * k / 1000.0;
    const Vec3 rel = traj.sample(t).T_b_d.p - pr.start.p;
    const double along = rel.dot(u);
    const Vec3 off = rel - along * u;
    ASSERT_LT(off.norm(), 1e-12) << "t = " << t;
    ASSERT_GE(along, -1e-12) << "t = " << t;
    ASSERT_LE(along, len + 1e-12) << "t = " << t;
    ASSERT_GE(along, prev_along - 1e-12) << "progress must be monotone, t = " << t;
    prev_along = along;
  }
}

TEST(CartesianTrajectory, TwistMatchesCentralDifferenceOfPose) {
  const Problem pr = problem();
  const CartesianTrajectory traj(pr.start, pr.goal, pr.T);
  const double h = 1e-6;
  for (int k = 1; k < 40; ++k) {
    const double t = pr.T * k / 40.0;
    const CartesianSample sp = traj.sample(t + h);
    const CartesianSample sm = traj.sample(t - h);
    const CartesianSample s = traj.sample(t);
    const Vec3 v_num = (sp.T_b_d.p - sm.T_b_d.p) / (2.0 * h);
    const Vec3 w_num = omega_central(sp.T_b_d.R, sm.T_b_d.R, h);
    EXPECT_LT((s.xd_d.head<3>() - v_num).cwiseAbs().maxCoeff(), 1e-7) << "t = " << t;
    EXPECT_LT((s.xd_d.tail<3>() - w_num).cwiseAbs().maxCoeff(), 1e-7) << "t = " << t;
    const common::Vec6 a_num = (sp.xd_d - sm.xd_d) / (2.0 * h);
    EXPECT_LT((s.xdd_d - a_num).cwiseAbs().maxCoeff(), 1e-6) << "t = " << t;
  }
}

TEST(CartesianTrajectory, PeakSpeedIsQuinticValue) {
  const Problem pr = problem();
  const CartesianTrajectory traj(pr.start, pr.goal, pr.T);
  // ds/dtau(1/2) = 30/4 - 60/8 + 30/16 = 7.5 - 7.5 + 1.875 = 1.875, the maximum since
  // d2s/dtau2(1/2) = 60/2 - 180/4 + 120/8 = 30 - 45 + 15 = 0.
  const double peak = 1.875 * pr.dp.norm() / pr.T;
  const double peak_w = 1.875 * pr.phi.norm() / pr.T;
  double v_max = 0.0;
  for (int k = 0; k <= 2000; ++k) {
    const CartesianSample s = traj.sample(pr.T * k / 2000.0);
    v_max = std::max(v_max, s.xd_d.head<3>().norm());
  }
  EXPECT_NEAR(v_max, peak, 1e-9 * peak);
  const CartesianSample mid = traj.sample(0.5 * pr.T);
  EXPECT_NEAR(mid.xd_d.head<3>().norm(), peak, 1e-9 * peak);
  EXPECT_NEAR(mid.xd_d.tail<3>().norm(), peak_w, 1e-9 * peak_w);
  EXPECT_LT(mid.xdd_d.norm(), 1e-12);
}

TEST(CartesianTrajectory, MvpTrajectorySamplesFiniteAt1kHz) {
  // Goal from config/default.yaml: (-0.5, -0.6, 0.9) m, rpy (0, 0, 90) deg, T = 10 s,
  // hold 2 s. The start pose is arbitrary (rotation to the goal is about 2.69 rad).
  Pose start;
  start.R = axis_angle(2.5, Vec3(1.0, 1.0, 0.0));
  start.p = Vec3(0.4, 0.3, 0.7);
  Pose goal;
  goal.R = axis_angle(0.5 * kPi, Vec3::UnitZ());
  goal.p = Vec3(-0.5, -0.6, 0.9);
  const CartesianTrajectory traj(start, goal, 10.0);
  for (int k = 0; k <= 12000; ++k) {
    const double t = k * 1e-3;
    const CartesianSample s = traj.sample(t);
    ASSERT_TRUE(s.T_b_d.p.allFinite()) << "t = " << t;
    ASSERT_TRUE(s.T_b_d.R.allFinite()) << "t = " << t;
    ASSERT_TRUE(s.xd_d.allFinite()) << "t = " << t;
    ASSERT_TRUE(s.xdd_d.allFinite()) << "t = " << t;
    ASSERT_LT(max_abs(s.T_b_d.R.transpose() * s.T_b_d.R - Mat3::Identity()), 1e-12) << "t = " << t;
  }
  const CartesianSample end = traj.sample(12.0);
  EXPECT_LT((end.T_b_d.p - goal.p).norm(), 1e-12);
  EXPECT_LT(max_abs(end.T_b_d.R - goal.R), 1e-12);
  EXPECT_LT(end.xd_d.norm(), 1e-12);
  EXPECT_LT(end.xdd_d.norm(), 1e-12);
}

TEST(CartesianTrajectory, IdenticalStartAndGoalStaysPut) {
  const Problem pr = problem();
  const CartesianTrajectory traj(pr.start, pr.start, pr.T);
  for (int k = -10; k <= 30; ++k) {
    const double t = pr.T * k / 20.0;
    const CartesianSample s = traj.sample(t);
    ASSERT_LT((s.T_b_d.p - pr.start.p).norm(), 1e-12) << "t = " << t;
    ASSERT_LT(max_abs(s.T_b_d.R - pr.start.R), 1e-12) << "t = " << t;
    ASSERT_TRUE(s.xd_d.allFinite() && s.xdd_d.allFinite()) << "t = " << t;
    ASSERT_LT(s.xd_d.norm(), 1e-12) << "t = " << t;
    ASSERT_LT(s.xdd_d.norm(), 1e-12) << "t = " << t;
  }
}

TEST(CartesianTrajectory, SampleDoesNotAllocate) {
  const Problem pr = problem();
  const CartesianTrajectory traj(pr.start, pr.goal, pr.T);
  CartesianSample a;
  CartesianSample b;
  CartesianSample c;
  Eigen::internal::set_is_malloc_allowed(false);
  a = traj.sample(-1.0);
  b = traj.sample(0.37 * pr.T);
  c = traj.sample(pr.T + 1.0);
  Eigen::internal::set_is_malloc_allowed(true);
  EXPECT_TRUE(a.xd_d.allFinite());
  EXPECT_TRUE(b.xd_d.allFinite());
  EXPECT_TRUE(c.xd_d.allFinite());
}

}  // namespace
}  // namespace cobot::planning
