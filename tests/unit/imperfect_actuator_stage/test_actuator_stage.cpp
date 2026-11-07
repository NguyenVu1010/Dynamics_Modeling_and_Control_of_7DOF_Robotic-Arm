// imperfect::ActuatorStage (brief drive-loop 2): the per-joint torque shaping of
// ImperfectJointInterface::write moved into a class used by the decorator (torque mode) and by the
// simulated drive. Same expressions in the same order, so the decorator's torque output is
// bit-identical to stage.limit(stage.shape(tau, q_true), qd_true):
//   shape: scale (+ asym) -> ripple at q_true -> deadband -> resolution rounding; stateless;
//   limit: slew over period_s -> bus voltage from the previous output and qd_true -> torque-speed
//          ceiling at qd_true; updates the previous output and the bus voltage.
// The frozen goldens (tests/unit/imperfect_comm, imperfect_actuator_limits) pin the decorator.
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <limits>
#include <random>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/hal/joint_interface.hpp"
#include "cobot/imperfect/actuator_stage.hpp"
#include "cobot/imperfect/config.hpp"
#include "cobot/imperfect/imperfect_joint_interface.hpp"
#include "cobot/pipeline/config.hpp"

namespace cobot::imperfect {
namespace {

constexpr int kN = 7;
constexpr double kDt = 1e-3;  // s, control rate 1 kHz
using Vec = common::VecN<kN>;
using Io = ImperfectJointInterface<kN>;
using Stage = ActuatorStage<kN>;

std::uint64_t bits_of(double v) {
  std::uint64_t u = 0;
  std::memcpy(&u, &v, sizeof(u));
  return u;
}

// Inner interface that returns `state` and records the last command.
class RecordingInner final : public JointInterface<kN> {
 public:
  Status read(JointState<kN>& out) override {
    out = state;
    return Status::ok();
  }
  Status write(const JointCommand<kN>& cmd) override {
    last = cmd;
    ++writes;
    return Status::ok();
  }
  JointState<kN> state;
  JointCommand<kN> last;
  int writes = 0;
};

Vec uniform(std::mt19937_64& rng, const Vec& lo, const Vec& hi) {
  Vec v;
  for (int j = 0; j < kN; ++j) {
    std::uniform_real_distribution<double> d(lo(j), hi(j));
    v(j) = d(rng);
  }
  return v;
}

// P of config/budget_xarm7.yaml (cobot.limits.peak_torque_nm), the P of the torque-speed curve.
Vec budget_peak() {
  return (Vec() << 204.0, 204.0, 107.0, 107.0, 36.0, 36.0, 36.0).finished();
}

// The budget actuator group (scale std 0.10, asym, ripple, deadband, resolution, slew,
// torque-speed curve, bus sag), comm off and command delay 0, 500 random cycles: |tau| up to 1.3 P
// (resolution, slew and ceiling all active), q in [-3, 3] rad (ripple phase), qd in [-4, 4] rad/s
// (beyond the knee 1.6 / 2.2 and the zero-torque speed 2.6 / 3.6 rad/s, both signs).
TEST(ActuatorStage, MatchesDecoratorTorquePath) {
  const auto imp = pipeline::load_imperfections<kN>(std::filesystem::path(COBOT_REPO_DIR) /
                                                    "config/imperfections/budget_xarm7.yaml");
  ASSERT_TRUE(imp) << imp.reason();
  ImperfectionConfig<kN> p = imp.value();
  p.comm = CommConfig{};
  p.actuator.command_delay_cycles = 0;
  ASSERT_GT(p.actuator.torque_scale_std(0), 0.0);
  ASSERT_GT(p.actuator.bus.v_nom_v, 0.0);

  const Vec peak = budget_peak();
  RecordingInner inner;
  Io io(inner, p, kDt, false, peak);
  ASSERT_NE(io.torque_scale(), Vec::Ones()) << "the drawn scale is used";
  Stage stage(p.actuator, io.torque_scale(), peak, kDt);

  std::mt19937_64 rng(20261107);
  const Vec q_max = Vec::Constant(3.0);
  const Vec qd_max = Vec::Constant(4.0);
  const Vec tau_max = 1.3 * peak;
  int bad = 0;
  for (int k = 0; k < 500 && bad < 5; ++k) {
    const Vec q = uniform(rng, -q_max, q_max);
    const Vec qd = uniform(rng, -qd_max, qd_max);
    const Vec tau = uniform(rng, -tau_max, tau_max);
    inner.state.q = q;
    inner.state.qd = qd;
    inner.state.motor_valid = false;  // the decorator's qd_true is then qd
    JointState<kN> s;
    ASSERT_TRUE(io.read(s).is_ok());
    JointCommand<kN> cmd;
    cmd.tau = tau;
    ASSERT_TRUE(io.write(cmd).is_ok());
    ASSERT_EQ(inner.writes, k + 1);

    const Vec shaped = stage.shape(tau, q);
    const Vec want = stage.limit(shaped, qd);
    for (int j = 0; j < kN; ++j) {
      if (bits_of(inner.last.tau(j)) != bits_of(want(j))) {
        ADD_FAILURE() << "cycle " << k << " joint " << j << ": decorator " << inner.last.tau(j)
                      << ", stage " << want(j);
        ++bad;
      }
    }
    EXPECT_EQ(inner.last.mode, hal::DriveMode::kTorque) << "cycle " << k;
    EXPECT_EQ(bits_of(stage.bus_voltage_v()), bits_of(io.bus_voltage_v())) << "cycle " << k;
  }
  EXPECT_EQ(bad, 0);
}

// shape() is stateless: the same input gives the same bits whatever limit() did in between.
TEST(ActuatorStage, ShapeIsStateless) {
  const auto imp = pipeline::load_imperfections<kN>(std::filesystem::path(COBOT_REPO_DIR) /
                                                    "config/imperfections/budget_xarm7.yaml");
  ASSERT_TRUE(imp) << imp.reason();
  const ImperfectionConfig<kN>& p = imp.value();
  Stage stage(p.actuator, Vec::Constant(1.07), budget_peak(), kDt);
  const Vec tau = (Vec() << 50.0, -30.0, 12.0, -7.5, 3.0, -1.0, 0.02).finished();
  const Vec q = Vec::Constant(0.4);
  const Vec first = stage.shape(tau, q);
  static_cast<void>(stage.limit(first, Vec::Constant(2.0)));
  static_cast<void>(stage.limit(-first, Vec::Constant(-3.0)));
  const Vec again = stage.shape(tau, q);
  for (int j = 0; j < kN; ++j) {
    EXPECT_EQ(bits_of(again(j)), bits_of(first(j))) << "joint " << j;
  }
}

// Every effect off, unit scale: shape and limit return the input bit for bit (signed zeros too),
// and the bus voltage is the normalised nominal 1.0, as the decorator reports it.
TEST(ActuatorStage, AllOffIsIdentity) {
  const Vec inf = Vec::Constant(std::numeric_limits<double>::infinity());
  Stage stage(ActuatorConfig<kN>{}, Vec::Ones(), inf, kDt);
  std::mt19937_64 rng(5);
  for (int k = 0; k < 100; ++k) {
    Vec tau = uniform(rng, Vec::Constant(-100.0), Vec::Constant(100.0));
    if (k == 0) {
      tau(3) = -0.0;
    }
    const Vec q = uniform(rng, Vec::Constant(-3.0), Vec::Constant(3.0));
    const Vec qd = uniform(rng, Vec::Constant(-5.0), Vec::Constant(5.0));
    const Vec shaped = stage.shape(tau, q);
    const Vec out = stage.limit(shaped, qd);
    for (int j = 0; j < kN; ++j) {
      EXPECT_EQ(bits_of(shaped(j)), bits_of(tau(j))) << "cycle " << k << " joint " << j;
      EXPECT_EQ(bits_of(out(j)), bits_of(tau(j))) << "cycle " << k << " joint " << j;
    }
    EXPECT_EQ(stage.bus_voltage_v(), 1.0);
  }
}

}  // namespace
}  // namespace cobot::imperfect
