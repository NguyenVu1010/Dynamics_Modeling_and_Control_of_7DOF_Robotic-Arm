// SimPlant caps the applied torque at the plant model's tau_max (feature joint-limits, design
// docs/design/2026-11-02-realism-2-design.md section 5): the plant model is the URDF one, whose
// tau_max is the momentary torque M of the joint (eRob70H: 70 N m). After the motor lag,
// tau_applied = clamp(lagged command, -tau_max, tau_max); a torque within the cap is untouched.
//
// "Unchanged bit for bit" is checked against the same plant with tau_max = +inf (no cap), so
// the tests do not depend on how the integrator rounds.
#include <limits>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/dynamics/sim_plant.hpp"
#include "cobot/model/model.hpp"

namespace cobot::dynamics {
namespace {

using common::Mat3;
using common::Vec3;
using V1 = VecN<1>;
using V2 = VecN<2>;

constexpr double kDt = 1e-3;  // s
constexpr double kM = 70.0;   // N m, eRob70H momentary torque (joints 5-7)
constexpr int kSteps = 100;   // 0.1 s
constexpr double kInf = std::numeric_limits<double>::infinity();

// One revolute joint about z (parallel to gravity, so gravity does no work), 1 kg on the axis,
// I_zz = 0.05 kg m^2, no damping. At 2 M the joint would reach 140 / 0.05 * 0.1 = 280 rad/s;
// limits +-1e6 rad keep the hard stops out of the way.
model::Model<1> joint_model(double tau_max) {
  model::Model<1> mdl;
  mdl.joints[0].axis_joint = Vec3::UnitZ();
  mdl.joints[0].q_min = -1e6;
  mdl.joints[0].q_max = 1e6;
  mdl.joints[0].qd_max = 1e6;
  mdl.joints[0].tau_max = tau_max;
  mdl.links[0].mass_kg = 1.0;
  mdl.links[0].com_i = Vec3::Zero();
  mdl.links[0].I_com_i = 0.05 * Mat3::Identity();
  return mdl;
}

TEST(SimPlantTorqueCap, TwiceMomentaryTorqueIsAppliedAsExactlyM) {
  for (const double sign : {1.0, -1.0}) {
    SimPlant<1> capped(joint_model(kM), V1::Zero());
    SimPlant<1> reference(joint_model(kInf), V1::Zero());  // commanded M itself, no cap
    for (int k = 0; k < kSteps; ++k) {
      capped.step(V1::Constant(sign * 2.0 * kM), kDt);  // 2 M = 140 N m
      reference.step(V1::Constant(sign * kM), kDt);
      ASSERT_EQ(capped.tau_applied()(0), sign * kM) << "step " << k;
      ASSERT_EQ(capped.q()(0), reference.q()(0)) << "step " << k;
      ASSERT_EQ(capped.qd()(0), reference.qd()(0)) << "step " << k;
    }
    // qd(0.1 s) = M / I * t = 70 / 0.05 * 0.1 = 140 rad/s (2 M uncapped would give 280).
    EXPECT_NEAR(capped.qd()(0), sign * 140.0, 1e-9);
  }
}

TEST(SimPlantTorqueCap, TorqueBelowMIsUnchangedBitForBit) {
  SimPlant<1> capped(joint_model(kM), V1::Zero());
  SimPlant<1> reference(joint_model(kInf), V1::Zero());
  for (int k = 0; k < kSteps; ++k) {
    // Varies within +-0.99 M, including the boundary region: 69.3 N m at k = 0, mod 7.
    const double tau = kM * 0.99 * (1.0 - (2.0 * static_cast<double>(k % 7) / 6.0));
    capped.step(V1::Constant(tau), kDt);
    reference.step(V1::Constant(tau), kDt);
    ASSERT_EQ(capped.tau_applied()(0), tau) << "step " << k;
    ASSERT_EQ(capped.q()(0), reference.q()(0)) << "step " << k;
    ASSERT_EQ(capped.qd()(0), reference.qd()(0)) << "step " << k;
  }
}

TEST(SimPlantTorqueCap, TorqueEqualToMIsNotChanged) {
  SimPlant<1> capped(joint_model(kM), V1::Zero());
  capped.step(V1::Constant(kM), kDt);
  EXPECT_EQ(capped.tau_applied()(0), kM);
  capped.step(V1::Constant(-kM), kDt);
  EXPECT_EQ(capped.tau_applied()(0), -kM);
}

TEST(SimPlantTorqueCap, CapAppliesAfterTheMotorLag) {
  // Motor time constant 10 ms, command 2 M from rest. The lagged torque
  // 140 (1 - exp(-t / 0.01)) is 13.3 N m after one step (140 * (1 - e^-0.1) = 140 * 0.0952),
  // crosses 70 N m at t = 0.01 ln 2 = 6.9 ms and reaches 140 (1 - e^-10) = 139.99 N m at 0.1 s.
  PlantEffects<1> fx;
  fx.motor_time_constant_s = V1::Constant(0.01);
  SimPlant<1> capped(joint_model(kM), V1::Zero(), fx);
  SimPlant<1> reference(joint_model(kInf), V1::Zero(), fx);
  int capped_steps = 0;
  for (int k = 0; k < kSteps; ++k) {
    capped.step(V1::Constant(2.0 * kM), kDt);
    reference.step(V1::Constant(2.0 * kM), kDt);
    const double lagged = reference.tau_applied()(0);
    if (lagged < kM) {
      // Below the cap the lag is untouched, so the two plants are still identical.
      ASSERT_EQ(capped.tau_applied()(0), lagged) << "step " << k;
      ASSERT_EQ(capped.q()(0), reference.q()(0)) << "step " << k;
    } else {
      ASSERT_EQ(capped.tau_applied()(0), kM) << "step " << k;
      ++capped_steps;
    }
  }
  EXPECT_NEAR(reference.tau_applied()(0), 139.99, 0.01);
  // Steps 7..100 (t >= 7 ms > 6.9 ms) are capped: 94 of them.
  EXPECT_EQ(capped_steps, 94);
  EXPECT_EQ(capped.tau_applied()(0), kM);
}

TEST(SimPlantTorqueCap, EachJointUsesItsOwnLimit) {
  // Two joints about z, 87 N m and 10 N m caps (the rated values of joints 1 and 5, only used as
  // distinct numbers here). Commands 100 and 5 N m: only joint 1 is capped.
  model::Model<2> mdl;
  for (int i = 0; i < 2; ++i) {
    mdl.joints[i].axis_joint = Vec3::UnitZ();
    mdl.joints[i].q_min = -1e6;
    mdl.joints[i].q_max = 1e6;
    mdl.joints[i].qd_max = 1e6;
    mdl.links[i].mass_kg = 1.0;
    mdl.links[i].I_com_i = 0.05 * Mat3::Identity();
  }
  mdl.joints[0].tau_max = 87.0;
  mdl.joints[1].tau_max = 10.0;
  SimPlant<2> plant(mdl, V2::Zero());
  for (int k = 0; k < 10; ++k) {
    plant.step(V2(100.0, 5.0), kDt);
    ASSERT_EQ(plant.tau_applied()(0), 87.0) << "step " << k;
    ASSERT_EQ(plant.tau_applied()(1), 5.0) << "step " << k;
    plant.step(V2(-3.0, -25.0), kDt);
    ASSERT_EQ(plant.tau_applied()(0), -3.0) << "step " << k;
    ASSERT_EQ(plant.tau_applied()(1), -10.0) << "step " << k;
  }
}

}  // namespace
}  // namespace cobot::dynamics
