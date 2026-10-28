#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "cobot/dynamics/dynamics.hpp"
#include "cobot/dynamics/sim_plant.hpp"
#include "cobot/model/model.hpp"

namespace cobot::dynamics {
namespace {

using common::Mat3;
using common::Vec3;
using V1 = VecN<1>;
using V7 = VecN<7>;

constexpr double kG = 9.80665;  // m/s^2
constexpr double kDt = 1e-3;    // s
constexpr double kPi = 3.14159265358979323846;

// One revolute joint at the base origin; link of mass m with CoM at `com` and
// I_com = diag(0.01) kg m^2. Limits +-q_lim rad, no damping (default 0).
model::Model<1> one_link(const Vec3& axis, double m, const Vec3& com, double q_lim = 3.0) {
  model::Model<1> mdl;
  mdl.joints[0].axis_joint = axis;
  mdl.joints[0].q_min = -q_lim;
  mdl.joints[0].q_max = q_lim;
  mdl.joints[0].tau_max = 100.0;
  mdl.links[0].mass_kg = m;
  mdl.links[0].com_i = com;
  mdl.links[0].I_com_i = 0.01 * Mat3::Identity();
  return mdl;
}

// Pendulum about x, CoM 0.5 m below the pivot, m = 2 kg:
// I_pivot = 0.01 + 2 * 0.5^2 = 0.51 kg m^2, m g l = 2 * 9.80665 * 0.5 = 9.80665 N m.
model::Model<1> pendulum() {
  return one_link(Vec3::UnitX(), 2.0, Vec3(0.0, 0.0, -0.5));
}

// Axis parallel to gravity, so gravity does no work. m = 4 kg at 0.5 m:
// I_pivot = 0.01 + 4 * 0.25 = 1.01 kg m^2.
model::Model<1> horizontal(double q_lim = 3.0) {
  return one_link(Vec3::UnitZ(), 4.0, Vec3(0.5, 0.0, 0.0), q_lim);
}

model::Arm7 load_arm() {
  const std::string urdf = std::string(COBOT_REPO_DIR) + "/sim/urdf/cobot30_5.urdf";
  const auto chain = model::parse_urdf_chain(urdf, "base_link", "link_7");
  EXPECT_TRUE(chain.is_ok()) << chain.reason();
  const auto arm = model::make_model<7>(chain.value(), Vec3(0.0, 0.0, -kG));
  EXPECT_TRUE(arm.is_ok()) << arm.reason();
  return arm.value();
}

// config/default.yaml home_q_deg = [0, 30, 0, -90, 0, 60, 0]; every joint is >= 0.2 rad
// inside its limits (checked in ArmGravityCompensationIsStatic).
V7 home() {
  V7 q;
  q << 0.0, 30.0, 0.0, -90.0, 0.0, 60.0, 0.0;
  return q * kPi / 180.0;
}

TEST(SimPlant, PendulumSmallAnglePeriod) {
  SimPlant<1> plant(pendulum(), V1::Constant(0.01));
  // T0 = 2 pi sqrt(I_pivot / (m g l)) = 2 pi sqrt(0.51 / 9.80665) = 1.43286 s; the
  // amplitude correction (1 + theta0^2 / 16) = 1 + 6.25e-6 is far below 0.5 %.
  const double t0 = 2.0 * kPi * std::sqrt(0.51 / kG);
  std::vector<double> crossings;  // downward zero crossings, linearly interpolated
  double q_prev = plant.q()(0);
  for (int k = 0; k < 10000; ++k) {
    plant.step(V1::Zero(), kDt);
    const double q = plant.q()(0);
    if (q_prev > 0.0 && q <= 0.0) {
      crossings.push_back(plant.time_s() - kDt * q / (q - q_prev));
    }
    q_prev = q;
  }
  ASSERT_GE(crossings.size(), 6U);  // 10 s / 1.433 s = 6.98 periods
  const double period =
      (crossings.back() - crossings.front()) / static_cast<double>(crossings.size() - 1);
  EXPECT_NEAR(period, t0, 0.005 * t0);
}

TEST(SimPlant, PendulumConservesEnergy) {
  SimPlant<1> plant(pendulum(), V1::Constant(0.5));
  // E = 1/2 I_pivot qd^2 + m g z_com, z_com = -0.5 cos q: m g z_com = -9.80665 cos q.
  const auto energy = [&plant] {
    const double qd = plant.qd()(0);
    return 0.5 * 0.51 * qd * qd - kG * std::cos(plant.q()(0));
  };
  const double e0 = energy();
  // Oscillation energy above the bottom: m g l (1 - cos 0.5) = 9.80665 * 0.12242 = 1.2006 J.
  const double swing = kG * (1.0 - std::cos(0.5));
  double max_drift = 0.0;
  for (int k = 0; k < 10000; ++k) {
    plant.step(V1::Zero(), kDt);
    max_drift = std::max(max_drift, std::abs(energy() - e0));
  }
  EXPECT_LT(max_drift / swing, 1e-6);
}

TEST(SimPlant, ArmGravityCompensationIsStatic) {
  const model::Arm7 arm = load_arm();
  const V7 q0 = home();
  for (int i = 0; i < 7; ++i) {
    ASSERT_GE(q0(i) - arm.joints[i].q_min, 0.2) << i;
    ASSERT_GE(arm.joints[i].q_max - q0(i), 0.2) << i;
  }
  SimPlant<7> plant(arm, q0);
  const V7 tau = gravity_torque(arm, q0);
  for (int k = 0; k < 2000; ++k) {
    plant.step(tau, kDt);
  }
  EXPECT_LT((plant.q() - q0).cwiseAbs().maxCoeff(), 1e-6);
}

TEST(SimPlant, ArmJointStopClampsPositionAndVelocity) {
  const model::Arm7 arm = load_arm();
  V7 q0 = home();
  q0(0) = arm.joints[0].q_max - 0.2;
  SimPlant<7> plant(arm, q0);
  // Joint 1 pushed with +tau_max; joints 2..7 held by inverse dynamics, a = 100 e - 20 qd.
  for (int k = 0; k < 5000; ++k) {
    V7 a = 100.0 * (q0 - plant.q()) - 20.0 * plant.qd();
    a(0) = 0.0;
    V7 tau = rnea(arm, plant.q(), plant.qd(), a, arm.gravity_b);
    tau(0) = arm.joints[0].tau_max;
    plant.step(tau, kDt);
    for (int i = 0; i < 7; ++i) {
      ASSERT_GE(plant.q()(i), arm.joints[i].q_min) << "joint " << i << " step " << k;
      ASSERT_LE(plant.q()(i), arm.joints[i].q_max) << "joint " << i << " step " << k;
    }
  }
  EXPECT_EQ(plant.q()(0), arm.joints[0].q_max);
  EXPECT_EQ(plant.qd()(0), 0.0);
}

TEST(SimPlant, JointStopReleasesInward) {
  SimPlant<1> plant(horizontal(0.5), V1::Zero());
  // 1 N m / 1.01 kg m^2 = 0.99 rad/s^2: 0.5 rad is reached after sqrt(2 * 0.5 / 0.99) = 1.0 s
  for (int k = 0; k < 2000; ++k) {
    plant.step(V1::Constant(1.0), kDt);
  }
  EXPECT_EQ(plant.q()(0), 0.5);
  EXPECT_EQ(plant.qd()(0), 0.0);
  for (int k = 0; k < 100; ++k) {  // velocity pointing inward is kept
    plant.step(V1::Constant(-1.0), kDt);
  }
  EXPECT_LT(plant.q()(0), 0.5);
  EXPECT_LT(plant.qd()(0), 0.0);
}

TEST(SimPlant, MotorLagIsExactFirstOrder) {
  PlantEffects<1> fx;
  fx.motor_time_constant_s = V1::Constant(0.01);
  SimPlant<1> plant(horizontal(), V1::Zero(), fx);
  plant.step(V1::Constant(1.0), kDt);
  // 1 - exp(-dt / tau_m) = 1 - exp(-0.001 / 0.01) = 1 - exp(-0.1) = 0.0951625819640405
  EXPECT_NEAR(plant.tau_applied()(0), 1.0 - std::exp(-0.1), 1e-12);
  for (int k = 1; k < 10; ++k) {
    plant.step(V1::Constant(1.0), kDt);
  }
  // after 10 steps: 1 - exp(-10 * 0.1) = 1 - exp(-1) = 0.6321205588285577
  EXPECT_NEAR(plant.tau_applied()(0), 1.0 - std::exp(-1.0), 1e-12);
  EXPECT_NEAR(plant.time_s(), 0.01, 1e-15);
  SimPlant<1> ideal(horizontal(), V1::Zero());  // tau_m = 0: no lag
  ideal.step(V1::Constant(0.7), kDt);
  EXPECT_EQ(ideal.tau_applied()(0), 0.7);
}

TEST(SimPlant, CoulombFrictionHoldsBelowBreakaway) {
  PlantEffects<1> fx;
  fx.coulomb_nm = V1::Constant(1.0);
  for (const double tau : {0.5, -0.5}) {
    SimPlant<1> plant(horizontal(), V1::Constant(0.3), fx);
    for (int k = 0; k < 1000; ++k) {
      plant.step(V1::Constant(tau), kDt);
    }
    // tanh(qd / 1e-3) = 0.5 -> creep qd = 1e-3 atanh(0.5) = 5.49e-4 rad/s -> 5.5e-4 rad in 1 s
    EXPECT_LT(std::abs(plant.q()(0) - 0.3), 1e-3) << tau;
  }
  // Above breakaway: (1.5 - 1) / 1.01 = 0.495 rad/s^2 -> q - q0 = 0.5 * 0.495 * 1^2 = 0.2475 rad
  SimPlant<1> plant(horizontal(), V1::Constant(0.3), fx);
  for (int k = 0; k < 1000; ++k) {
    plant.step(V1::Constant(1.5), kDt);
  }
  EXPECT_GT(plant.q()(0) - 0.3, 0.2);
  EXPECT_LT(plant.q()(0) - 0.3, 0.3);
}

TEST(SimPlant, ViscousFrictionTerminalVelocity) {
  PlantEffects<1> fx;
  fx.viscous_nm_s_rad = V1::Constant(2.0);
  SimPlant<1> plant(horizontal(10.0), V1::Zero(), fx);  // q(10 s) = 4.75 rad: stay off the stop
  for (int k = 0; k < 10000; ++k) {
    plant.step(V1::Constant(1.0), kDt);
  }
  // qd = (1 / 2) (1 - exp(-t * 2 / 1.01)); at t = 10 s: exp(-19.8) = 2.5e-9
  EXPECT_NEAR(plant.qd()(0), 0.5, 1e-6);
}

PlantEffects<7> arm_effects() {
  PlantEffects<7> fx;
  fx.coulomb_nm = V7::Constant(0.5);
  fx.viscous_nm_s_rad = V7::Constant(0.1);
  fx.motor_time_constant_s = V7::Constant(0.005);
  return fx;
}

V7 wobble(const V7& g, int k) {
  return g + V7::Constant(2.0 * std::sin(0.01 * static_cast<double>(k)));
}

TEST(SimPlant, Deterministic) {
  const model::Arm7 arm = load_arm();
  SimPlant<7> a(arm, home(), arm_effects());
  SimPlant<7> b(arm, home(), arm_effects());
  const V7 g = gravity_torque(arm, home());
  for (int k = 0; k < 2000; ++k) {
    a.step(wobble(g, k), kDt);
    b.step(wobble(g, k), kDt);
  }
  for (int i = 0; i < 7; ++i) {
    EXPECT_EQ(a.q()(i), b.q()(i)) << i;
    EXPECT_EQ(a.qd()(i), b.qd()(i)) << i;
  }
  EXPECT_GT((a.q() - home()).norm(), 1e-6);  // the arm did move
}

TEST(SimPlant, StepDoesNotAllocate) {
  const model::Arm7 arm = load_arm();
  SimPlant<7> plant(arm, home(), arm_effects());
  const V7 g = gravity_torque(arm, home());
  Eigen::internal::set_is_malloc_allowed(false);
  for (int k = 0; k < 100; ++k) {
    plant.step(wobble(g, k), kDt);
  }
  Eigen::internal::set_is_malloc_allowed(true);
  EXPECT_NEAR(plant.time_s(), 0.1, 1e-12);
}

}  // namespace
}  // namespace cobot::dynamics
