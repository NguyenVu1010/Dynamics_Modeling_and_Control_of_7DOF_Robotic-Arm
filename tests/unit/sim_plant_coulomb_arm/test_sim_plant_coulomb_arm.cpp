// Coulomb stick-slip on the 7-DOF arm with friction on every joint (review gap of
// `sim-plant-coulomb`). The joints are coupled through M(q), so a per-joint projection could let
// a sliding joint restart a stuck one; the plant must solve the stick set jointly.
//
// Zero gravity and zero velocity: with tau applied and qdd = 0, friction must cancel tau
// exactly, which is admissible whenever |tau_i| < coulomb_i. Every joint must then stay put.
#include <array>
#include <cmath>
#include <string>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "cobot/dynamics/sim_plant.hpp"
#include "cobot/model/model.hpp"

namespace cobot::dynamics {
namespace {

using common::Vec3;
using V7 = VecN<7>;

constexpr double kDt = 1e-3;  // s
constexpr double kPi = 3.14159265358979323846;

model::Arm7 load_arm_zero_gravity() {
  const std::string urdf = std::string(COBOT_REPO_DIR) + "/src/sim/urdf/cobot30_5.urdf";
  const auto chain = model::parse_urdf_chain(urdf, "base_link", "link_7");
  EXPECT_TRUE(chain.is_ok()) << chain.reason();
  const auto arm = model::make_model<7>(chain.value(), Vec3::Zero());
  EXPECT_TRUE(arm.is_ok()) << arm.reason();
  return arm.value();
}

V7 home() {
  V7 q;  // config/default.yaml home_q_deg
  q << 0.0, 30.0, 0.0, -90.0, 0.0, 60.0, 0.0;
  return q * kPi / 180.0;
}

// config/imperfections/realistic.yaml friction, all joints.
PlantEffects<7> realistic_friction() {
  PlantEffects<7> fx;
  fx.coulomb_nm << 1.7, 1.7, 1.7, 1.7, 0.24, 0.24, 0.24;
  fx.viscous_nm_s_rad << 0.5, 0.5, 0.5, 0.5, 0.1, 0.1, 0.1;
  return fx;
}

class ArmStick : public ::testing::TestWithParam<double> {};

TEST_P(ArmStick, EveryJointStaysStuckBelowBreakaway) {
  const model::Arm7 arm = load_arm_zero_gravity();
  const PlantEffects<7> fx = realistic_friction();
  const V7 q0 = home();
  SimPlant<7> plant(arm, q0, fx);
  // Mixed signs so the couplings of M push some joints against their own torque.
  const std::array<double, 7> sign = {1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0};
  V7 tau;
  for (int i = 0; i < 7; ++i) {
    tau(i) = GetParam() * sign[static_cast<std::size_t>(i)] * fx.coulomb_nm(i);
  }
  for (int k = 0; k < 2000; ++k) {
    plant.step(tau, kDt);
    ASSERT_EQ(plant.qd(), V7::Zero()) << "step " << k << ", qd = " << plant.qd().transpose();
  }
  EXPECT_LT((plant.q() - q0).cwiseAbs().maxCoeff(), 1e-8);
}

INSTANTIATE_TEST_SUITE_P(FractionOfBreakaway, ArmStick, ::testing::Values(0.3, 0.6, 0.9));

TEST(ArmSlip, OnlyTheJointAboveBreakawayMovesAndInItsTorqueDirection) {
  // Joints 1-6 at 90 % of breakaway (mixed signs), joint 7 at 150 % in the negative direction.
  // Releasing joint 7 alone must not drag the others out of stick: its reaction on joint i is
  // M_i7 qdd_7, and with the wrist's small inertia that stays far below the 10 % margin.
  const model::Arm7 arm = load_arm_zero_gravity();
  const PlantEffects<7> fx = realistic_friction();
  const V7 q0 = home();
  SimPlant<7> plant(arm, q0, fx);
  V7 tau;
  tau << 0.9, -0.9, 0.9, -0.9, 0.9, -0.9, -1.5;
  tau = tau.cwiseProduct(fx.coulomb_nm);
  for (int k = 0; k < 200; ++k) {
    plant.step(tau, kDt);
  }
  EXPECT_LT(plant.qd()(6), 0.0);
  EXPECT_LT(plant.q()(6), q0(6));
  EXPECT_LT((plant.q().head<6>() - q0.head<6>()).cwiseAbs().maxCoeff(), 1e-6);
}

}  // namespace
}  // namespace cobot::dynamics
