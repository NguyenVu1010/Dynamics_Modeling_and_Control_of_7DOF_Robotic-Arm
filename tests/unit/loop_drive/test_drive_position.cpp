// control::DrivePosition (brief drive-loop 3 and 4) and the third registry entry:
//   compute: mode kPosition, q = ref.q, qd = ref.qd, tau = clamp_or_zero(ff): kNone 0,
//            kGravity G(ref.q), kInverseDynamics rnea(ref.q, ref.qd, ref.qdd) + D ref.qd;
//            a non-finite input gives the all-zero torque command;
//   hold:    mode kPosition, q = q_hold, qd = 0, tau = 0 (kNone) else clamp(G(q_hold)).
// Neither depends on the measured state (the drive closes the loop on its own encoder).
#include <cstdint>
#include <limits>
#include <random>
#include <string>
#include <type_traits>
#include <utility>

#include <gtest/gtest.h>

#include "cobot/control/algorithm.hpp"
#include "cobot/control/drive_position.hpp"
#include "cobot/control/tracking_controller.hpp"
#include "cobot/planning/clik.hpp"

#include "loop_drive_util.hpp"

namespace cobot::pipeline::test {
namespace {

using control::DriveFeedForward;
using control::DrivePosition;
using Ref = planning::JointReference<kN>;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

Vec random_vec(std::mt19937_64& rng, double a) {
  std::uniform_real_distribution<double> d(-a, a);
  Vec v;
  for (int j = 0; j < kN; ++j) {
    v(j) = d(rng);
  }
  return v;
}

TEST(Algorithms, DrivePositionIsTheThirdEntry) {
  static_assert(control::kAlgorithms.size() == 3);
  EXPECT_EQ(static_cast<int>(control::Algorithm::kDrivePosition), 2);
  const control::AlgorithmInfo& a = control::kAlgorithms[2];
  EXPECT_EQ(a.id, control::Algorithm::kDrivePosition);
  EXPECT_STREQ(a.name, "drive_position");
  EXPECT_STREQ(a.summary, "position loop in the joint drive, set points from the host");
  EXPECT_STREQ(control::to_string(control::Algorithm::kDrivePosition), "drive_position");
  EXPECT_EQ(control::algorithm_from_string("drive_position"), control::Algorithm::kDrivePosition);
  EXPECT_EQ(control::registered_algorithm_names(),
            std::string("computed_torque, pd_gravity, drive_position"));
}

TEST(DrivePosition, StaticGuards) {
  static_assert(noexcept(std::declval<const DrivePosition<kN>&>().compute(
      std::declval<const State&>(), std::declval<const Ref&>())));
  static_assert(
      noexcept(std::declval<const DrivePosition<kN>&>().hold(std::declval<const Vec&>())));
  static_assert(noexcept(
      std::declval<const control::TrackingController<kN>&>().hold(std::declval<const Vec&>())));
  static_assert(!std::is_polymorphic_v<DrivePosition<kN>>);
  static_assert(!std::is_polymorphic_v<control::TrackingController<kN>>);
  SUCCEED();
}

TEST(DrivePosition, ComputeAndHold) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const model::Arm7& m = setup->model;
  std::mt19937_64 rng(31);
  for (const DriveFeedForward ff :
       {DriveFeedForward::kNone, DriveFeedForward::kGravity, DriveFeedForward::kInverseDynamics}) {
    const DrivePosition<kN> law(m, ff);
    for (int k = 0; k < 50; ++k) {
      State s;
      s.q = setup->config.home_q_rad + random_vec(rng, 0.5);
      s.qd = random_vec(rng, 1.0);
      Ref ref;
      ref.q = setup->config.home_q_rad + random_vec(rng, 0.5);
      ref.qd = random_vec(rng, 1.0);
      ref.qdd = random_vec(rng, 50.0);  // large enough to saturate some joints
      const hal::JointCommand<kN> out = law.compute(s, ref);
      EXPECT_EQ(out.mode, hal::DriveMode::kPosition);
      EXPECT_TRUE(bits_equal(out.q, ref.q)) << k;
      EXPECT_TRUE(bits_equal(out.qd, ref.qd)) << k;
      const Vec want = clamp_or_zero(m, raw_feedforward(m, ff, ref.q, ref.qd, ref.qdd));
      EXPECT_TRUE(bits_equal(out.tau, want)) << k << ": " << out.tau.transpose();

      // Not a function of the measured state.
      State other = s;
      other.q += Vec::Constant(0.1);
      other.qd = -s.qd;
      const hal::JointCommand<kN> again = law.compute(other, ref);
      EXPECT_TRUE(bits_equal(again.tau, out.tau)) << k;
      EXPECT_TRUE(bits_equal(again.q, out.q)) << k;

      const Vec q_hold = ref.q;
      const hal::JointCommand<kN> h = law.hold(q_hold);
      EXPECT_EQ(h.mode, hal::DriveMode::kPosition);
      EXPECT_TRUE(bits_equal(h.q, q_hold));
      EXPECT_TRUE(h.qd.isZero(0.0));
      const Vec hold_tau = ff == DriveFeedForward::kNone
                               ? Vec(Vec::Zero())
                               : clamp_or_zero(m, dynamics::gravity_torque(m, q_hold));
      EXPECT_TRUE(bits_equal(h.tau, hold_tau)) << k;
    }
  }
}

TEST(DrivePosition, NonFiniteInputGivesTheZeroTorqueCommand) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const DrivePosition<kN> law(setup->model, DriveFeedForward::kInverseDynamics);
  State s;
  s.q = setup->config.home_q_rad;
  Ref good;
  good.q = setup->config.home_q_rad;
  for (int which = 0; which < 3; ++which) {
    Ref ref = good;
    if (which == 0) {
      ref.q(1) = kNaN;
    } else if (which == 1) {
      ref.qd(3) = std::numeric_limits<double>::infinity();
    } else {
      ref.qdd(6) = kNaN;
    }
    const hal::JointCommand<kN> out = law.compute(s, ref);
    EXPECT_EQ(out.mode, hal::DriveMode::kTorque) << "case " << which;
    EXPECT_TRUE(out.tau.isZero(0.0)) << "case " << which;
    EXPECT_TRUE(out.q.allFinite()) << "case " << which;
    EXPECT_TRUE(out.qd.allFinite()) << "case " << which;
  }
}

}  // namespace
}  // namespace cobot::pipeline::test
