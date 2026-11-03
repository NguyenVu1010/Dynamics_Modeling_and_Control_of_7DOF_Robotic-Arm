// hal::JointState::q_from_motor (brief "deflection-comp" contract 1): true only when q / qd come
// from the motor encoder, i.e. ImperfectJointInterface::read with position_source motor and a
// motor_valid inner state. Every other producer (SimJointInterface, the decorator with the link
// source or an invalid motor reading) leaves it false.
#include <array>

#include <gtest/gtest.h>

#include "cobot/dynamics/sim_plant.hpp"
#include "cobot/imperfect/imperfect_joint_interface.hpp"
#include "cobot/pipeline/sim_joint_interface.hpp"

#include "deflection_test_util.hpp"

namespace cobot::pipeline::test {
namespace {

using imperfect::ImperfectionConfig;
using imperfect::ImperfectJointInterface;
using imperfect::PositionSource;
using V1 = common::VecN<1>;

constexpr double kDt = 1e-3;

// Inner interface returning a fixed state.
class FakeJoint final : public hal::JointInterface<1> {
 public:
  explicit FakeJoint(const hal::JointState<1>& s) : state_(s) {}
  common::Status read(hal::JointState<1>& out) override {
    out = state_;
    return common::Status::ok();
  }
  common::Status write(const hal::JointCommand<1>& /*cmd*/) override {
    return common::Status::ok();
  }

 private:
  hal::JointState<1> state_;
};

hal::JointState<1> fake_state(bool motor_valid) {
  hal::JointState<1> s;
  s.q(0) = 0.1;
  s.qd(0) = 0.2;
  s.q_motor(0) = 0.1003;
  s.qd_motor(0) = 0.25;
  s.motor_valid = motor_valid;
  return s;
}

TEST(QFromMotor, DefaultIsFalse) {
  const hal::JointState<1> s;
  EXPECT_FALSE(s.q_from_motor);
}

TEST(QFromMotor, DecoratorSetsItOnlyForTheMotorSourceWithAValidMotorReading) {
  struct Case {
    PositionSource source;
    bool motor_valid;
    bool expected;
    double q;
  };
  const std::array<Case, 4> cases = {{
      {PositionSource::kMotor, true, true, 0.1003},
      {PositionSource::kMotor, false, false, 0.1},
      {PositionSource::kLink, true, false, 0.1},
      {PositionSource::kLink, false, false, 0.1},
  }};
  for (const Case& c : cases) {
    ImperfectionConfig<1> cfg;
    cfg.sensor.position_source = c.source;
    FakeJoint inner(fake_state(c.motor_valid));
    ImperfectJointInterface<1> io(inner, cfg, kDt);
    hal::JointState<1> s;
    s.q_from_motor = !c.expected;  // read() must overwrite it either way
    for (int k = 0; k < 3; ++k) {
      ASSERT_TRUE(io.read(s).is_ok());
      EXPECT_EQ(s.q_from_motor, c.expected)
          << "source " << (c.source == PositionSource::kMotor ? "motor" : "link") << ", valid "
          << c.motor_valid << ", read " << k;
      EXPECT_EQ(s.q(0), c.q);
    }
  }
}

TEST(QFromMotor, SimJointInterfaceLeavesItFalseAlthoughTheMotorIsValid) {
  dynamics::SimPlant<1> plant(
      [] {
        model::Model<1> mdl;
        mdl.joints[0].q_min = -10.0;
        mdl.joints[0].q_max = 10.0;
        mdl.joints[0].qd_max = 10.0;
        mdl.joints[0].tau_max = 10.0;
        mdl.links[0].mass_kg = 1.0;
        mdl.links[0].I_com_i = 0.1 * common::Mat3::Identity();
        return mdl;
      }(),
      V1::Constant(0.3));
  SimJointInterface<1> io(plant, kDt);
  hal::JointState<1> s;
  ASSERT_TRUE(io.read(s).is_ok());
  EXPECT_TRUE(s.motor_valid);
  EXPECT_FALSE(s.q_from_motor);
}

}  // namespace
}  // namespace cobot::pipeline::test
