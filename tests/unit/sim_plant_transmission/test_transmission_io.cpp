// Motor-side measurement (brief "transmission" contract 1, 3, 5 and the decorator rule):
// SimJointInterface fills q_motor, qd_motor and motor_valid; with sensor.position_source = motor
// the decorator takes q_motor / qd_motor as the raw position and velocity when motor_valid (else
// the link values), and the torque-speed ceiling uses qd_motor when valid.
// imperfect::plant_effects copies the transmission group and draws motor_phase_rad in [0, 2 pi).
#include <cmath>

#include <gtest/gtest.h>

#include "cobot/dynamics/dynamics.hpp"
#include "cobot/imperfect/imperfect_joint_interface.hpp"
#include "cobot/imperfect/perturb.hpp"
#include "cobot/pipeline/sim_joint_interface.hpp"

#include "transmission_test_util.hpp"

namespace cobot::dynamics::test {
namespace {

using hal::JointCommand;
using hal::JointState;
using imperfect::ImperfectionConfig;
using imperfect::ImperfectJointInterface;
using imperfect::PositionSource;

TEST(TransmissionGravity, StaticDeflectionIsTheGravityTorqueOverK1AndTheMotorEncoderSeesIt) {
  // Joint 2-like: axis y (horizontal), 2 kg at 0.5 m on x, so at q = 0 the link is horizontal and
  // |tau_g| = 2 * 9.80665 * 0.5 = 9.80665 N m < t1 = 14 (SHG-25, in K1):
  // q_motor - q = 9.80665 / 3.1e4 = 3.1634e-4 rad. Armature 3.08632 (joint 2).
  // The test holds the motor at 0 with a PD on q_motor plus the gravity torque of the measured q;
  // at rest tau_m = tau_s = G(q), so phi = G(q) / k1.
  const model::Model<1> mdl = one_joint(0.01, 3.08632, Vec3::UnitY(), Vec3(0.5, 0.0, 0.0), 2.0);
  PlantEffects<1> fx;
  set_elastic(fx, 0, kShg25, 0.01);
  SimPlant<1> plant(mdl, V1::Zero(), fx);
  pipeline::SimJointInterface<1> io(plant, kDt);
  for (int k = 0; k < 3000; ++k) {
    const double g = gravity_torque(mdl, plant.q())(0);
    const double tau = g - (200.0 * plant.q_motor()(0)) - (50.0 * plant.qd_motor()(0));
    ASSERT_TRUE(io.write(JointCommand<1>{V1::Constant(tau)}).is_ok());
    io.advance();
  }
  const double g = gravity_torque(mdl, plant.q())(0);
  ASSERT_NEAR(std::abs(g), 9.80665, 1e-3);
  const double phi = plant.q_motor()(0) - plant.q()(0);
  EXPECT_NEAR(phi, g / kShg25.k1, 0.02 * std::abs(g / kShg25.k1));

  JointState<1> s;
  ASSERT_TRUE(io.read(s).is_ok());
  EXPECT_TRUE(s.motor_valid);
  EXPECT_EQ(s.q, plant.q());
  EXPECT_EQ(s.q_motor, plant.q_motor());
  EXPECT_EQ(s.qd_motor, plant.qd_motor());

  // Decorator, noise off: the motor source reports q_motor, the link source q.
  ImperfectionConfig<1> cfg;
  cfg.sensor.position_source = PositionSource::kMotor;
  ImperfectJointInterface<1> motor_enc(io, cfg, kDt);
  ASSERT_TRUE(motor_enc.read(s).is_ok());
  EXPECT_EQ(s.q(0), plant.q_motor()(0));
  EXPECT_EQ(s.qd(0), plant.qd_motor()(0));
  EXPECT_NE(s.q(0), plant.q()(0));
  cfg.sensor.position_source = PositionSource::kLink;
  ImperfectJointInterface<1> link_enc(io, cfg, kDt);
  ASSERT_TRUE(link_enc.read(s).is_ok());
  EXPECT_EQ(s.q(0), plant.q()(0));
  EXPECT_EQ(s.qd(0), plant.qd()(0));
}

TEST(TransmissionGravity, RigidPlantReportsTheLinkAsMotor) {
  SimPlant<1> plant(one_joint(0.2, 0.1), V1::Constant(0.3));
  pipeline::SimJointInterface<1> io(plant, kDt);
  ASSERT_TRUE(io.write(JointCommand<1>{V1::Constant(1.0)}).is_ok());
  io.advance();
  EXPECT_EQ(plant.q_motor(), plant.q());
  EXPECT_EQ(plant.qd_motor(), plant.qd());
  JointState<1> s;
  ASSERT_TRUE(io.read(s).is_ok());
  EXPECT_TRUE(s.motor_valid);
  EXPECT_EQ(s.q_motor, s.q);
  EXPECT_EQ(s.qd_motor, s.qd);
}

// Inner interface returning a fixed state and recording the last command.
class FakeJoint final : public hal::JointInterface<1> {
 public:
  explicit FakeJoint(const JointState<1>& s) : state_(s) {}
  common::Status read(JointState<1>& out) override {
    out = state_;
    return common::Status::ok();
  }
  common::Status write(const JointCommand<1>& cmd) override {
    last_ = cmd;
    return common::Status::ok();
  }
  const JointCommand<1>& last() const { return last_; }

 private:
  JointState<1> state_;
  JointCommand<1> last_;
};

JointState<1> fake_state(bool motor_valid) {
  JointState<1> s;
  s.q(0) = 0.1;
  s.qd(0) = 0.0;
  s.q_motor(0) = 0.1003;
  s.qd_motor(0) = 5.0;  // above the zero-torque speed 3.6 used below
  s.motor_valid = motor_valid;
  return s;
}

TEST(TransmissionDecorator, MotorSourceFallsBackToTheLinkWhenMotorIsInvalid) {
  ImperfectionConfig<1> cfg;
  cfg.sensor.position_source = PositionSource::kMotor;
  for (const bool valid : {true, false}) {
    FakeJoint inner(fake_state(valid));
    ImperfectJointInterface<1> io(inner, cfg, kDt);
    JointState<1> s;
    ASSERT_TRUE(io.read(s).is_ok());
    EXPECT_EQ(s.q(0), valid ? 0.1003 : 0.1) << valid;
    EXPECT_EQ(s.qd(0), valid ? 5.0 : 0.0) << valid;
  }
}

TEST(TransmissionDecorator, TorqueSpeedCeilingUsesTheMotorVelocityWhenValid) {
  // Knee 2.2, zero torque 3.6 rad/s, P = 36 N m. Link at rest, motor at 5 rad/s: a valid motor
  // velocity puts +10 N m (motoring) above the zero-torque speed -> 0; an invalid one leaves the
  // link speed 0 -> 10 N m passes.
  ImperfectionConfig<1> cfg;
  cfg.actuator.knee_speed_rad_s(0) = 2.2;
  cfg.actuator.zero_torque_speed_rad_s(0) = 3.6;
  for (const bool valid : {true, false}) {
    FakeJoint inner(fake_state(valid));
    ImperfectJointInterface<1> io(inner, cfg, kDt, false, V1::Constant(36.0));
    JointState<1> s;
    ASSERT_TRUE(io.read(s).is_ok());
    ASSERT_TRUE(io.write(JointCommand<1>{V1::Constant(10.0)}).is_ok());
    EXPECT_EQ(inner.last().tau(0), valid ? 0.0 : 10.0) << valid;
  }
}

TEST(TransmissionPlantEffects, CopiesTheGroupAndDrawsTheMotorPhaseFromTheSeed) {
  ImperfectionConfig<2> cfg;
  imperfect::TransmissionConfig<2>& t = cfg.transmission;
  t.stiffness_k1_nm_rad << 3.1e4, 4.7e3;
  t.stiffness_k2_nm_rad << 5.0e4, 6.1e3;
  t.stiffness_k3_nm_rad << 5.7e4, 7.1e3;
  t.torque_t1_nm << 14.0, 2.0;
  t.torque_t2_nm << 48.0, 6.9;
  t.damping_ratio << 0.02, 0.0;
  t.error_amp_rad << 4.4e-4, 2.9e-4;
  t.lost_motion_rad << 2.9e-4, 8.7e-4;
  t.ratio << 100.0, 120.0;
  cfg.friction.coulomb_nm << 1.7, 0.2;
  cfg.seed = 7;
  const PlantEffects<2> a = imperfect::plant_effects(cfg);
  EXPECT_EQ(a.stiffness_k1_nm_rad, t.stiffness_k1_nm_rad);
  EXPECT_EQ(a.stiffness_k2_nm_rad, t.stiffness_k2_nm_rad);
  EXPECT_EQ(a.stiffness_k3_nm_rad, t.stiffness_k3_nm_rad);
  EXPECT_EQ(a.torque_t1_nm, t.torque_t1_nm);
  EXPECT_EQ(a.torque_t2_nm, t.torque_t2_nm);
  EXPECT_EQ(a.damping_ratio, t.damping_ratio);
  EXPECT_EQ(a.error_amp_rad, t.error_amp_rad);
  EXPECT_EQ(a.lost_motion_rad, t.lost_motion_rad);
  EXPECT_EQ(a.ratio, t.ratio);
  EXPECT_EQ(a.coulomb_nm, cfg.friction.coulomb_nm);
  for (int j = 0; j < 2; ++j) {
    EXPECT_GE(a.motor_phase_rad(j), 0.0);
    EXPECT_LT(a.motor_phase_rad(j), 2.0 * kPi);
  }
  EXPECT_NE(a.motor_phase_rad(0), a.motor_phase_rad(1));
  // Same seed, same phases; another seed, other phases; other groups do not move the stream.
  ImperfectionConfig<2> same = cfg;
  same.friction.coulomb_nm.setZero();
  same.sensor.q_noise_std_rad.setConstant(1e-3);
  EXPECT_EQ(imperfect::plant_effects(same).motor_phase_rad, a.motor_phase_rad);
  ImperfectionConfig<2> other = cfg;
  other.seed = 8;
  EXPECT_NE(imperfect::plant_effects(other).motor_phase_rad, a.motor_phase_rad);
}

TEST(TransmissionPlantEffects, DefaultConfigIsRigidWithTheDocumentedDefaults) {
  const ImperfectionConfig<2> cfg;
  EXPECT_TRUE(cfg.transmission.stiffness_k3_nm_rad.isZero(0.0));
  EXPECT_EQ(cfg.transmission.damping_ratio, VecN<2>::Constant(0.01));
  EXPECT_EQ(cfg.transmission.ratio, VecN<2>::Constant(100.0));
  EXPECT_EQ(cfg.sensor.position_source, PositionSource::kLink);
  EXPECT_TRUE(imperfect::plant_effects(cfg).stiffness_k3_nm_rad.isZero(0.0));
}

}  // namespace
}  // namespace cobot::dynamics::test
