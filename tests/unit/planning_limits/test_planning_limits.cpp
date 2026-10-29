#include <cmath>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "cobot/common/lie.hpp"
#include "cobot/common/types.hpp"
#include "cobot/model/model.hpp"
#include "cobot/planning/cartesian_trajectory.hpp"
#include "cobot/planning/clik.hpp"

namespace cobot::planning {
namespace {

using common::Mat3;
using common::Vec3;
constexpr double kPi = 3.14159265358979323846;

Mat3 axis_angle(double angle, const Vec3& axis) {
  return Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
}

// ---------------------------------------------------------------- trajectory continuity

// Rotation of 1.2 rad about a skew axis, dp = (-0.5, 0.5, 0.4), T = 2 s. Near the ends the
// quintic gives s ~ 10 tau^3, s' ~ 30 tau^2 / T, s'' ~ 60 tau / T^2 (tau = eps / T), so the
// samples just inside [0, T] must approach the held endpoint samples: pose O(eps^3), twist
// O(eps^2), acceleration O(eps). With eps = 1e-6 s: |xdd| = 60 eps / T^3 * |(dp; w)| = 1.1e-5.
TEST(CartesianTrajectory, SamplesAreContinuousAtBothEnds) {
  Pose start;
  start.R = axis_angle(0.7, Vec3(1.0, 2.0, 3.0));
  start.p = Vec3(0.1, -0.2, 0.5);
  Pose goal;
  goal.R = start.R * common::so3_exp(1.2 * Vec3(0.3, -0.5, 0.8).normalized());
  goal.p = Vec3(-0.4, 0.3, 0.9);
  constexpr double kT = 2.0;
  constexpr double kEps = 1e-6;
  const CartesianTrajectory traj(start, goal, kT);

  const struct {
    double t_in;
    double t_out;
  } ends[] = {{kEps, -kEps}, {kT - kEps, kT + kEps}};
  for (const auto& end : ends) {
    const CartesianSample in = traj.sample(end.t_in);
    const CartesianSample out = traj.sample(end.t_out);
    EXPECT_LT((in.T_b_d.p - out.T_b_d.p).norm(), 1e-12) << "t = " << end.t_in;
    EXPECT_LT((in.T_b_d.R - out.T_b_d.R).cwiseAbs().maxCoeff(), 1e-12) << "t = " << end.t_in;
    EXPECT_LT((in.xd_d - out.xd_d).norm(), 1e-8) << "t = " << end.t_in;
    EXPECT_LT((in.xdd_d - out.xdd_d).norm(), 1e-4) << "t = " << end.t_in;
    EXPECT_TRUE(out.xd_d.isZero(0.0));
    EXPECT_TRUE(out.xdd_d.isZero(0.0));
  }
}

// ---------------------------------------------------------------- CLIK lower limit

// Planar arm about z, links 1.0 and 0.7 m along x. q2 limited to [-0.5, 0.5]; the target is
// the arm's pose at q* = (-0.5, -0.8), so q2 runs into its LOWER limit (mirror image of the
// upper-limit case in tests/unit/planning_clik).
model::Model<2> planar() {
  model::Model<2> m;
  for (int i = 0; i < 2; ++i) {
    m.joints[i].q_min = -kPi;
    m.joints[i].q_max = kPi;
    m.joints[i].qd_max = 10.0;
    m.joints[i].tau_max = 10.0;
    m.links[i].mass_kg = 1.0;
  }
  m.joints[1].T_parent_joint.p = Vec3(1.0, 0.0, 0.0);
  m.joints[1].q_min = -0.5;
  m.joints[1].q_max = 0.5;
  m.T_last_tip.p = Vec3(0.7, 0.0, 0.0);
  return m;
}

TEST(Clik, LowerLimitSaturationStopsVelocityIntoTheLimit) {
  const model::Model<2> m = planar();
  ClikReference<2> clik(m, ClikConfig{});
  clik.reset(VecN<2>(-0.2, -0.4));
  CartesianSample x;
  x.T_b_d.p = Vec3(std::cos(-0.5) + 0.7 * std::cos(-1.3), std::sin(-0.5) + 0.7 * std::sin(-1.3), 0);
  x.T_b_d.R = Eigen::AngleAxisd(-1.3, Vec3::UnitZ()).toRotationMatrix();
  int at_limit = 0;
  for (int k = 0; k < 3000; ++k) {  // 3 s at 1 kHz
    const JointReference<2>& r = clik.step(x, 1e-3);
    ASSERT_GE(r.q(1), -0.5) << "step " << k;
    ASSERT_LE(r.q(1), 0.5) << "step " << k;
    ASSERT_TRUE(r.qd.allFinite() && r.qdd.allFinite());
    if (r.q(1) == -0.5) {
      ++at_limit;
      ASSERT_GE(r.qd(1), 0.0) << "step " << k;
    }
  }
  EXPECT_GT(at_limit, 0);
  EXPECT_EQ(clik.reference().q(1), -0.5);  // the target keeps pulling q2 below the limit
}

}  // namespace
}  // namespace cobot::planning
