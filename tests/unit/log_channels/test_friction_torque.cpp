// SimPlant::friction_torque() (feature log-channels, contract 3): the friction torque of the last
// step(), Coulomb + PlantEffects viscous (URDF damping excluded), opposing the motion; on an
// elastic joint the motor-side friction averaged over the sub-steps; 0 before the first step.
// Small models in the style of tests/unit/sim_plant_coulomb and sim_plant_transmission.
#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/dynamics/sim_plant.hpp"
#include "cobot/model/model.hpp"

namespace cobot::dynamics {
namespace {

using common::Mat3;
using common::Vec3;
using V1 = VecN<1>;

constexpr double kDt = 1e-3;  // s

// One revolute joint about z (parallel to gravity: no gravity torque), 0.1 kg on the axis,
// I_zz = 1e-4 kg m^2, URDF damping `damping`, limits +-1000 rad.
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

PlantEffects<1> coulomb_viscous(double coulomb, double viscous) {
  PlantEffects<1> fx;
  fx.coulomb_nm = V1::Constant(coulomb);
  fx.viscous_nm_s_rad = V1::Constant(viscous);
  return fx;
}

TEST(FrictionTorque, ZeroBeforeTheFirstStep) {
  const SimPlant<1> plant(wrist_like(), V1::Zero(), coulomb_viscous(2.0, 0.5));
  EXPECT_EQ(plant.friction_torque()(0), 0.0);
}

TEST(FrictionTorque, ExactlyZeroWithoutEffectsEvenWithUrdfDamping) {
  for (const double damping : {0.0, 0.003}) {
    SimPlant<1> plant(wrist_like(damping), V1::Zero());
    for (int k = 0; k < 1000; ++k) {
      plant.step(V1::Constant(0.05 * std::sin(0.01 * k) + 0.01), kDt);
      ASSERT_EQ(plant.friction_torque()(0), 0.0) << "damping " << damping << " step " << k;
    }
  }
}

TEST(FrictionTorque, ViscousOnlyJointIsTheViscousTorqueAtTheEndState) {
  // No Coulomb: the viscous term is integrated inside RK4; friction = -0.5 * qd at the end state.
  SimPlant<1> plant(wrist_like(), V1::Zero(), coulomb_viscous(0.0, 0.5));
  for (int k = 0; k < 200; ++k) {
    plant.step(V1::Constant(0.3 * std::sin(0.05 * k)), kDt);
    const double want = -0.5 * plant.qd()(0);
    EXPECT_NEAR(plant.friction_torque()(0), want, 1e-12 * std::max(1.0, std::abs(want)))
        << "step " << k;
  }
}

TEST(FrictionTorque, SlidingJointIsCoulombPlusViscousOpposingTheVelocity) {
  // tau = +-3 N m, coulomb 2, viscous 0.5: steady |qd| = (3 - 2) / 0.5 = 2 rad/s (time constant
  // I / b = 1e-4 / 0.5 = 0.2 ms, reached within a few steps). With URDF damping 0.003 the plant
  // slides at (3 - 2) / 0.503 = 1.988 rad/s, and the friction torque still excludes the damping.
  for (const double damping : {0.0, 0.003}) {
    for (const double tau : {3.0, -3.0}) {
      SimPlant<1> plant(wrist_like(damping), V1::Zero(), coulomb_viscous(2.0, 0.5));
      for (int k = 0; k < 200; ++k) {
        plant.step(V1::Constant(tau), kDt);
        const double qd = plant.qd()(0);
        ASSERT_NE(qd, 0.0) << "step " << k;
        const double want = -(2.0 + 0.5 * std::abs(qd)) * std::copysign(1.0, qd);
        EXPECT_NEAR(plant.friction_torque()(0), want, 1e-9 * std::abs(want))
            << "damping " << damping << " tau " << tau << " step " << k;
      }
      const double v_ss = 1.0 / (0.5 + damping);  // (|tau| - coulomb) / (viscous + damping)
      EXPECT_NEAR(plant.qd()(0), std::copysign(v_ss, tau), 1e-6) << damping << " " << tau;
    }
  }
}

TEST(FrictionTorque, StuckJointBalancesTheNetTorque) {
  // |tau| = 1 < coulomb 2 from rest: the joint never moves and static friction = -tau.
  for (const double tau : {1.0, -1.0}) {
    SimPlant<1> plant(wrist_like(), V1::Constant(0.3), coulomb_viscous(2.0, 0.5));
    for (int k = 0; k < 100; ++k) {
      plant.step(V1::Constant(tau), kDt);
      ASSERT_EQ(plant.qd()(0), 0.0) << "step " << k;
      EXPECT_NEAR(plant.friction_torque()(0), -tau, 1e-9) << "tau " << tau << " step " << k;
    }
  }
}

TEST(FrictionTorque, ElasticJointReportsTheMotorSideCoulombWhileTheMotorSlides) {
  // Link I = 0.01 kg m^2, motor B = 0.22674 kg m^2 (URDF armature of joints 5-7), SHG-14 spring,
  // 1 % damping, motor Coulomb 2 N m, no viscous. Drive +-5 N m from rest: the whole joint
  // accelerates at (5 - 2) / (B + I) = 3 / 0.23674 = 12.7 rad/s^2 and the motor starts sliding in
  // the first sub-step (h * (5 - 2) / B > 0), so every sub-step carries -2 * sign(qd_motor).
  model::Model<1> mdl;
  mdl.joints[0].axis_joint = Vec3::UnitZ();
  mdl.joints[0].q_min = -1e6;
  mdl.joints[0].q_max = 1e6;
  mdl.joints[0].qd_max = 1e6;
  mdl.joints[0].tau_max = 1e6;
  mdl.joints[0].armature = 0.22674;
  mdl.links[0].mass_kg = 1.0;
  mdl.links[0].com_i = Vec3::Zero();
  mdl.links[0].I_com_i = 0.01 * Mat3::Identity();
  PlantEffects<1> fx;
  fx.stiffness_k1_nm_rad(0) = 4.7e3;  // SHG-14
  fx.stiffness_k2_nm_rad(0) = 6.1e3;
  fx.stiffness_k3_nm_rad(0) = 7.1e3;
  fx.torque_t1_nm(0) = 2.0;
  fx.torque_t2_nm(0) = 6.9;
  fx.damping_ratio(0) = 0.01;
  fx.error_amp_rad(0) = 0.0;
  fx.lost_motion_rad(0) = 0.0;
  fx.ratio(0) = 100.0;
  fx.coulomb_nm(0) = 2.0;
  for (const double tau : {5.0, -5.0}) {
    SimPlant<1> plant(mdl, V1::Zero(), fx);
    for (int k = 0; k < 200; ++k) {
      plant.step(V1::Constant(tau), kDt);
      const double v = plant.qd_motor()(0);
      ASSERT_GT(v * tau, 0.0) << "motor not sliding with the drive at step " << k;
      EXPECT_NEAR(plant.friction_torque()(0), -2.0 * std::copysign(1.0, v), 1e-6)
          << "tau " << tau << " step " << k;
    }
  }
}

}  // namespace
}  // namespace cobot::dynamics
