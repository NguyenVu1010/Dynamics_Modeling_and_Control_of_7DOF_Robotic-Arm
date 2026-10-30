// Coulomb friction in SimPlant on small-inertia joints (fix `sim-plant-coulomb`).
//
// With I ~ 1e-4 kg m^2 and h = 1 ms, a smoothed sign tanh(qd / 1e-3) inside RK4 has
// h * b / I = h * (coulomb / 1e-3) / I = 1e-3 * 240 / 1e-4 = 2400, far beyond RK4's limit
// (~2.8): the velocity dithers around zero and the Coulomb torque averages out. These tests
// pin the stick-slip behaviour the fixed plant must show: a true stick below breakaway, the
// exact net acceleration above it, a coast that stops and stays stopped, and the friction
// power on the 7-DOF wrist.
#include <cmath>
#include <cstddef>
#include <string>

#include <Eigen/Cholesky>
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

constexpr double kDt = 1e-3;  // s
constexpr double kPi = 3.14159265358979323846;
constexpr double kWristCoulomb = 0.24;  // N m, config/imperfections/realistic.yaml joints 5-7
constexpr double kWristViscous = 0.1;   // N m s / rad, same file

// One revolute joint about z (parallel to gravity, so gravity does no work), link of mass
// 0.1 kg with the CoM on the axis and I_com = diag(1e-4) kg m^2, so I_zz = 1e-4 kg m^2.
// URDF damping `damping` (default none). Limits +-1000 rad: the 0.5 N m case reaches
// q = 13 rad in 0.1 s.
model::Model<1> wrist_like(double damping = 0.0) {
  model::Model<1> mdl;
  mdl.joints[0].axis_joint = Vec3::UnitZ();
  mdl.joints[0].q_min = -1000.0;
  mdl.joints[0].q_max = 1000.0;
  mdl.joints[0].tau_max = 10.0;
  mdl.joints[0].damping = damping;
  mdl.links[0].mass_kg = 0.1;
  mdl.links[0].com_i = Vec3::Zero();
  mdl.links[0].I_com_i = 1e-4 * Mat3::Identity();
  return mdl;
}

PlantEffects<1> wrist_coulomb() {
  PlantEffects<1> fx;
  fx.coulomb_nm = V1::Constant(kWristCoulomb);
  return fx;
}

// Effective inertia of the 1-DOF test joint as the model sees it (any URDF/armature-like
// term included): M(q)(0, 0). Expected 1e-4 kg m^2.
double i_eff_1dof(const model::Model<1>& mdl, double q) {
  const V1 q_vec = V1::Constant(q);
  return mass_matrix(mdl, q_vec)(0, 0);
}

TEST(SimPlantCoulomb, TestModelInertiaIsAsDesigned) {
  // I_zz = I_com_zz + m * |com_xy|^2 = 1e-4 + 0.1 * 0 = 1e-4 kg m^2
  EXPECT_NEAR(i_eff_1dof(wrist_like(), 0.0), 1e-4, 1e-12);
}

TEST(SimPlantCoulomb, SticksBelowBreakaway) {
  const model::Model<1> mdl = wrist_like();
  for (const double tau : {0.1, -0.1}) {
    SimPlant<1> plant(mdl, V1::Constant(0.3), wrist_coulomb());
    // |tau| = 0.1 < coulomb = 0.24: the joint must not move at all. (The smoothed model
    // creeps at 1e-3 * atanh(0.1 / 0.24) = 4.4e-4 rad/s -> 8.9e-4 rad in 2 s, or dithers.)
    for (int k = 0; k < 2000; ++k) {
      plant.step(V1::Constant(tau), kDt);
    }
    EXPECT_LT(std::abs(plant.q()(0) - 0.3), 1e-6) << tau;
    EXPECT_EQ(plant.qd()(0), 0.0) << tau;
  }
}

TEST(SimPlantCoulomb, SticksBelowBreakawayWithViscousAndUrdfDamping) {
  // Same stick with the realistic wrist viscous term (0.1 N m s/rad) and the URDF joint damping
  // of cobot30_5.urdf (0.003 N m s/rad). A stuck joint must not creep: a velocity-dependent
  // force evaluated on a velocity that is only removed afterwards (in-step predictor
  // qd = h * 0.1 / 1e-4 = 1 rad/s) shifts q by about h^2 / 12 * (b / I) * 1 rad/s per step,
  // e.g. 1e-6 / 12 * 1030 = 8.6e-5 rad -> 0.17 rad in 2 s.
  PlantEffects<1> fx = wrist_coulomb();
  fx.viscous_nm_s_rad = V1::Constant(kWristViscous);
  const model::Model<1> mdl = wrist_like(0.003);
  for (const double tau : {0.1, -0.1}) {
    SimPlant<1> plant(mdl, V1::Constant(0.3), fx);
    for (int k = 0; k < 2000; ++k) {
      plant.step(V1::Constant(tau), kDt);
    }
    EXPECT_LT(std::abs(plant.q()(0) - 0.3), 1e-6) << tau;
    EXPECT_EQ(plant.qd()(0), 0.0) << tau;
  }
}

TEST(SimPlantCoulomb, AcceleratesWithNetTorqueAboveBreakaway) {
  const model::Model<1> mdl = wrist_like();
  const double i_eff = i_eff_1dof(mdl, 0.0);
  for (const double tau : {0.5, -0.5}) {
    SimPlant<1> plant(mdl, V1::Zero(), wrist_coulomb());
    for (int k = 0; k < 100; ++k) {
      plant.step(V1::Constant(tau), kDt);
    }
    // qd(0.1 s) = (|tau| - coulomb) / I_eff * t = (0.5 - 0.24) / 1e-4 * 0.1 = 260 rad/s,
    // signed like tau; q = 0.5 * 2600 * 0.01 = 13 rad, inside the +-1000 rad limits.
    const double expected = std::copysign((std::abs(tau) - kWristCoulomb) / i_eff * 0.1, tau);
    EXPECT_NEAR(plant.qd()(0), expected, 0.01 * std::abs(expected)) << tau;
  }
}

TEST(SimPlantCoulomb, CoastDeceleratesAtCoulombOverInertiaAndStops) {
  const model::Model<1> mdl = wrist_like();
  const double i_eff = i_eff_1dof(mdl, 0.0);
  const double decel = kWristCoulomb / i_eff;  // 0.24 / 1e-4 = 2400 rad/s^2
  // A coast from 1 rad/s would stop in 1 / 2400 = 0.42 ms, inside one step. Spin up for
  // 10 steps at 0.5 N m instead: qd0 = 2600 * 0.01 = 26 rad/s, stop time 26 / 2400 = 10.8 ms.
  SimPlant<1> plant(mdl, V1::Zero(), wrist_coulomb());
  for (int k = 0; k < 10; ++k) {
    plant.step(V1::Constant(0.5), kDt);
  }
  const double qd0 = plant.qd()(0);
  const double q0 = plant.q()(0);
  ASSERT_NEAR(qd0, 26.0, 0.02 * 26.0);
  const double t_stop_expected = qd0 / decel;  // ~ 10.8e-3 s

  int stop_step = -1;
  double qd_prev = qd0;
  for (int k = 1; k <= 100; ++k) {
    plant.step(V1::Zero(), kDt);
    const double qd = plant.qd()(0);
    ASSERT_GE(qd, 0.0) << "reversal at coast step " << k;
    if (qd > 0.0) {
      // While sliding, every step removes h * coulomb / I_eff = 1e-3 * 2400 = 2.4 rad/s.
      EXPECT_NEAR(qd - qd_prev, -kDt * decel, 0.01 * kDt * decel) << "coast step " << k;
    } else if (stop_step < 0) {
      stop_step = k;
    }
    qd_prev = qd;
  }
  ASSERT_GT(stop_step, 0) << "joint never stopped";
  // Stop time within 2 steps of qd0 / decel (10.8 ms -> step 11).
  EXPECT_NEAR(static_cast<double>(stop_step) * kDt, t_stop_expected, 2.0 * kDt);
  // Coast distance qd0^2 / (2 decel) = 26^2 / 4800 = 0.1408 rad; the discrete stop differs by
  // at most one step at the last velocity decrement, h * h * decel = 2.4e-3 rad.
  const double q_stop = plant.q()(0);
  EXPECT_NEAR(q_stop - q0, qd0 * qd0 / (2.0 * decel), kDt * kDt * decel);

  // Then it stays exactly at rest: no reversal, no creep.
  for (int k = 0; k < 1000; ++k) {
    plant.step(V1::Zero(), kDt);
    ASSERT_EQ(plant.qd()(0), 0.0) << "rest step " << k;
  }
  EXPECT_EQ(plant.q()(0), q_stop);
}

model::Arm7 load_arm_zero_gravity() {
  const std::string urdf = std::string(COBOT_REPO_DIR) + "/src/sim/urdf/cobot30_5.urdf";
  const auto chain = model::parse_urdf_chain(urdf, "base_link", "link_7");
  EXPECT_TRUE(chain.is_ok()) << chain.reason();
  // Zero gravity: the mechanical energy is the kinetic energy 1/2 qd^T M(q) qd alone.
  const auto arm = model::make_model<7>(chain.value(), Vec3::Zero());
  EXPECT_TRUE(arm.is_ok()) << arm.reason();
  return arm.value();
}

double kinetic_energy(const model::Arm7& arm, const V7& q, const V7& qd) {
  return 0.5 * qd.dot(mass_matrix(arm, q) * qd);
}

struct FrictionPower {
  double measured_w = 0.0;  // (work in - kinetic energy gain) / T over the window
  double expected_w = 0.0;  // mean of sum_i coulomb_i |qd_i| + (viscous_i + damping_i) qd_i^2
  double mean_qd7 = 0.0;    // rad/s over the window
};

// Joint 7 of the arm driven at qd_ref by a PI on qd (1 kHz, 3 s) with the nominal friction as
// feed-forward; joints 1-6 held at home by computed torque. Only joint 7 has plant friction
// (realistic wrist values). Returns the friction power over the last 2 s from the energy
// balance: the work of the joint torques, sum_k tau_k . (q_{k+1} - q_k) (exact for a torque
// held over the step), minus the kinetic energy gained.
FrictionPower drive_wrist(double qd_ref) {
  const model::Arm7 arm = load_arm_zero_gravity();
  V7 q0;  // config/default.yaml home_q_deg, joint 7 moved to -1 rad (limits +-2.9671 rad)
  q0 << 0.0, 30.0, 0.0, -90.0, 0.0, 60.0, 0.0;
  q0 *= kPi / 180.0;
  q0(6) = -1.0;
  PlantEffects<7> fx;
  fx.coulomb_nm(6) = kWristCoulomb;
  fx.viscous_nm_s_rad(6) = kWristViscous;
  SimPlant<7> plant(arm, q0, fx);

  // Effective inertia of joint 7: 1 / (M^-1)_77 (~1.8e-4 kg m^2). PI gains from it: fast pole
  // h * kd / I = 0.3 (well inside the explicit-step limit), ki = kd^2 / (16 I) (damping ratio 2).
  const Eigen::LDLT<common::MatN<7>> ldlt(mass_matrix(arm, q0));
  const double i7 = 1.0 / ldlt.solve(V7::Unit(6))(6);
  const double kd = 0.3 * i7 / kDt;
  const double ki = kd * kd / (16.0 * i7);
  const double ff = (kWristCoulomb * (qd_ref > 0.0 ? 1.0 : -1.0)) + (kWristViscous * qd_ref);

  constexpr int kSteps = 3000;   // 3 s
  constexpr int kWindow = 2000;  // last 2 s
  double integral = 0.0;
  double work = 0.0;
  double expected_energy = 0.0;
  double sum_qd7 = 0.0;
  double e_start = 0.0;
  for (int k = 0; k < kSteps; ++k) {
    if (k == kSteps - kWindow) {
      e_start = kinetic_energy(arm, plant.q(), plant.qd());
    }
    V7 a = 400.0 * (q0 - plant.q()) - 40.0 * plant.qd();
    a(6) = 0.0;
    V7 tau = rnea(arm, plant.q(), plant.qd(), a, Vec3::Zero());
    const double err = qd_ref - plant.qd()(6);
    integral += err * kDt;
    tau(6) = ff + (kd * err) + (ki * integral);
    const V7 q_before = plant.q();
    plant.step(tau, kDt);
    if (k >= kSteps - kWindow) {
      work += tau.dot(plant.q() - q_before);
      // Friction power at the end of the step (rectangle rule over the window).
      for (int i = 0; i < 7; ++i) {
        const double v = plant.qd()(i);
        const double b = fx.viscous_nm_s_rad(i) + arm.joints[static_cast<std::size_t>(i)].damping;
        expected_energy += ((fx.coulomb_nm(i) * std::abs(v)) + (b * v * v)) * kDt;
      }
      sum_qd7 += plant.qd()(6);
    }
  }
  const double e_end = kinetic_energy(arm, plant.q(), plant.qd());
  const double window_s = kWindow * kDt;
  FrictionPower out;
  out.measured_w = (work - (e_end - e_start)) / window_s;
  out.expected_w = expected_energy / window_s;
  out.mean_qd7 = sum_qd7 / static_cast<double>(kWindow);
  return out;
}

TEST(SimPlantCoulomb, ArmWristFrictionPowerAtSteadyVelocity) {
  // qd7 = 0.5 rad/s: coulomb |qd| + (viscous + URDF damping) qd^2
  //   = 0.24 * 0.5 + (0.1 + 0.003) * 0.25 = 0.12 + 0.02575 = 0.14575 W;
  // viscous only (Coulomb lost) would be 0.02575 W. Joint 7 travels -1 -> ~0.5 rad.
  const FrictionPower p = drive_wrist(0.5);
  EXPECT_NEAR(p.mean_qd7, 0.5, 0.01);
  EXPECT_NEAR(p.expected_w, 0.14575, 0.01 * 0.14575);
  EXPECT_NEAR(p.measured_w, p.expected_w, 0.05 * p.expected_w);
}

TEST(SimPlantCoulomb, ArmWristFrictionPowerAtCreepVelocity) {
  // qd7 = 2 mrad/s, where the smoothed sign has slope coulomb / 1e-3 * sech^2(2) = 17 N m s/rad
  // and h * 17 / 1.8e-4 = 94: the old plant dithers here. Expected power
  //   0.24 * 0.002 + 0.103 * 4e-6 = 4.80e-4 + 4.1e-7 = 4.804e-4 W.
  const FrictionPower p = drive_wrist(0.002);
  // The PI settles slowly through stick-slip at this speed; these two only check that the
  // joint really slides (a stuck joint would give mean qd7 = 0 and expected power ~ 0).
  EXPECT_NEAR(p.mean_qd7, 0.002, 0.25 * 0.002);
  EXPECT_NEAR(p.expected_w, 4.804e-4, 0.25 * 4.804e-4);
  EXPECT_NEAR(p.measured_w, p.expected_w, 0.05 * p.expected_w);
}

}  // namespace
}  // namespace cobot::dynamics
