// ControlLoop with cobot.control.algorithm drive_position (brief drive-loop 4 and 5):
// - tracking/standby: out = DrivePosition::compute(state, ref): mode kPosition, q = ref.q,
//   qd = ref.qd, tau = clamp_or_zero(ff) with ff kNone 0, kGravity G(ref.q), kInverseDynamics
//   rnea(ref.q, ref.qd, ref.qdd) + D ref.qd; then the friction feed-forward is added to out.tau as
//   for the torque laws (add_clamped);
// - with deflection_comp enabled and state.q_from_motor: out.q_j += harmonic_spring_deflection(
//   G_j(out.q), k1..t2 of joint j) (G of the controller model at the set point), in tracking and in
//   the hold;
// - holding: out = TrackingController::hold(q_hold): kPosition, q = q_hold, qd = 0, tau = 0 (kNone)
//   else clamp(G(q_hold));
// - every step, every algorithm: out.seq = ++seq_ (1, 2, 3, ...);
// - the Run CSV is unchanged: q_ref = the set point before the deflection shift.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "cobot/control/friction_compensation.hpp"
#include "cobot/dynamics/harmonic_spring.hpp"

#include "loop_drive_util.hpp"

namespace cobot::pipeline::test {
namespace {

using control::DriveFeedForward;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

const char* name_of(DriveFeedForward ff) {
  switch (ff) {
    case DriveFeedForward::kNone:
      return "none";
    case DriveFeedForward::kGravity:
      return "gravity";
    case DriveFeedForward::kInverseDynamics:
      return "inverse_dynamics";
  }
  return "?";
}

// Per-joint static deflection of the gravity torque of `m` at q (the shift of a set point q).
Vec deflection_at(const model::Arm7& m, const DeflectionCompConfig<kN>& d, const Vec& q) {
  const Vec g = dynamics::gravity_torque(m, q);
  Vec out;
  for (int j = 0; j < kN; ++j) {
    out(j) = dynamics::harmonic_spring_deflection(g(j),
                                                  d.stiffness_k1_nm_rad(j),
                                                  d.stiffness_k2_nm_rad(j),
                                                  d.stiffness_k3_nm_rad(j),
                                                  d.torque_t1_nm(j),
                                                  d.torque_t2_nm(j));
  }
  return out;
}

// The Harmonic Drive values of config/budget_xarm7.yaml (cobot.deflection_comp), enabled.
DeflectionCompConfig<kN> budget_deflection() {
  const auto cfg =
      load_config<kN>(std::filesystem::path(COBOT_REPO_DIR) / "config/budget_xarm7.yaml");
  EXPECT_TRUE(cfg) << cfg.reason();
  DeflectionCompConfig<kN> d = cfg ? cfg.value().deflection_comp : DeflectionCompConfig<kN>{};
  EXPECT_TRUE(d.enabled);
  return d;
}

void expect_tracking_command(
    const model::Arm7& m, const Config<kN>& c, const Command& out, const Record& rec, int k) {
  const DriveFeedForward ff = c.drive.torque_feedforward;
  EXPECT_EQ(out.mode, hal::DriveMode::kPosition) << name_of(ff) << " step " << k;
  EXPECT_TRUE(bits_equal(out.q, rec.q_ref)) << name_of(ff) << " step " << k;
  EXPECT_TRUE(bits_equal(out.qd, rec.qd_ref)) << name_of(ff) << " step " << k;
  const control::FrictionCompensation<kN> friction(c.friction_compensation);
  const Vec base = clamp_or_zero(m, raw_feedforward(m, ff, rec.q_ref, rec.qd_ref, rec.qdd_ref));
  const Vec want = add_clamped(m, base, friction.torque(rec.qd_ref));
  EXPECT_TRUE(bits_equal(out.tau, want))
      << name_of(ff) << " step " << k << ": tau " << out.tau.transpose() << ", expected "
      << want.transpose();
  EXPECT_TRUE(bits_equal(rec.tau, out.tau)) << "Run CSV tau = the feed-forward sent, step " << k;
}

class DriveCommand : public ::testing::TestWithParam<DriveFeedForward> {};

TEST_P(DriveCommand, SetPointAndFeedForward) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  for (const bool friction_on : {false, true}) {
    Config<kN> c = drive_config(setup->config, GetParam());
    c.friction_compensation.enabled = friction_on;
    c.friction_compensation.coulomb_nm.setConstant(1.5);
    c.friction_compensation.viscous_nm_s_rad.setConstant(0.5);
    c.friction_compensation.velocity_eps_rad_s = 0.01;
    const auto rig = std::make_unique<DriveRig>(setup->model, c);
    ASSERT_EQ(rig->loop.algorithm(), control::Algorithm::kDrivePosition);
    ASSERT_TRUE(rig->start().is_ok());
    State s;
    Command out;
    bool moved = false;
    for (int k = 0; k < 300; ++k) {
      ASSERT_TRUE(rig->cycle(s, out).is_ok()) << "step " << k;
      ASSERT_EQ(rig->loop.mode(), LoopMode::kTracking) << "step " << k;
      const Record& rec = rig->loop.last_record();
      expect_tracking_command(setup->model, c, out, rec, k);
      moved = moved || rec.qd_ref.cwiseAbs().maxCoeff() > 1e-6;
    }
    EXPECT_TRUE(moved) << "the reference moves in the first 0.3 s";
  }
}

INSTANTIATE_TEST_SUITE_P(FeedForward,
                         DriveCommand,
                         ::testing::Values(DriveFeedForward::kNone,
                                           DriveFeedForward::kGravity,
                                           DriveFeedForward::kInverseDynamics),
                         [](const ::testing::TestParamInfo<DriveFeedForward>& info) {
                           return std::string(name_of(info.param));
                         });

// seq = 1, 2, 3, ... every step, also in holding rows; for the torque laws too.
TEST(LoopDrive, Seq) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  {
    const auto rig = std::make_unique<DriveRig>(
        setup->model, drive_config(setup->config, DriveFeedForward::kInverseDynamics));
    ASSERT_TRUE(rig->start().is_ok());
    State s;
    Command out;
    for (int k = 0; k < 40; ++k) {
      static_cast<void>(rig->cycle(s, out, [k](State& st) {
        if (k == 30) {
          st.q(2) = kNaN;  // -> kHolding (kNonFinite)
        }
      }));
      EXPECT_EQ(out.seq, static_cast<std::uint32_t>(k + 1)) << "step " << k;
    }
    EXPECT_EQ(rig->loop.mode(), LoopMode::kHolding);
  }
  {
    const auto rig = make_rig(*setup, setup->config);  // computed_torque, no drive
    ASSERT_TRUE(rig->start().is_ok());
    State s;
    Command out;
    for (int k = 0; k < 20; ++k) {
      ASSERT_TRUE(rig->cycle(s, out).is_ok());
      EXPECT_EQ(out.seq, static_cast<std::uint32_t>(k + 1)) << "computed_torque step " << k;
      EXPECT_EQ(out.mode, hal::DriveMode::kTorque) << "step " << k;
    }
  }
}

// deflection_comp enabled with the budget Harmonic Drive values and a state with q_from_motor:
// out.q = q_ref + deflection(G(q_ref)) per joint (1e-15 rad); the Run CSV keeps q_ref unshifted.
// Joint 2 at home carries ~100 N m of gravity torque: a shift of ~3e-3 rad, so the check is not
// vacuous. With q_from_motor false the set point is q_ref bit for bit.
TEST(LoopDrive, DeflectionShift) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  Config<kN> c = drive_config(setup->config, DriveFeedForward::kInverseDynamics);
  c.deflection_comp = budget_deflection();
  {
    const auto rig = std::make_unique<DriveRig>(setup->model, c);
    ASSERT_TRUE(rig->start().is_ok());
    State s;
    Command out;
    double largest = 0.0;
    const auto from_motor = [](State& st) { st.q_from_motor = true; };
    for (int k = 0; k < 200; ++k) {
      ASSERT_TRUE(rig->cycle(s, out, from_motor).is_ok()) << "step " << k;
      ASSERT_EQ(rig->loop.mode(), LoopMode::kTracking) << "step " << k;
      const Record& rec = rig->loop.last_record();
      const Vec shift = deflection_at(setup->model, c.deflection_comp, rec.q_ref);
      for (int j = 0; j < kN; ++j) {
        EXPECT_NEAR(out.q(j), rec.q_ref(j) + shift(j), 1e-15) << "step " << k << " joint " << j;
      }
      EXPECT_TRUE(bits_equal(out.qd, rec.qd_ref)) << "step " << k;
      largest = std::max(largest, shift.cwiseAbs().maxCoeff());
    }
    EXPECT_GT(largest, 1e-4) << "the shift is not vacuous";

    // The hold through the drive gets the same shift: NaN read -> hold at the last finite
    // (compensated) q, set point q_hold + deflection(G(q_hold)).
    const Vec q_before = rig->loop.last_record().q;
    static_cast<void>(rig->cycle(s, out, [](State& st) {
      st.q_from_motor = true;
      st.q(0) = kNaN;
    }));
    ASSERT_EQ(rig->loop.mode(), LoopMode::kHolding);
    const Record& rec = rig->loop.last_record();
    EXPECT_TRUE(bits_equal(rec.q_ref, q_before)) << "Run CSV q_ref = the hold target";
    const Vec shift = deflection_at(setup->model, c.deflection_comp, q_before);
    for (int j = 0; j < kN; ++j) {
      EXPECT_NEAR(out.q(j), q_before(j) + shift(j), 1e-15) << "hold joint " << j;
    }
  }
  {
    const auto rig = std::make_unique<DriveRig>(setup->model, c);
    ASSERT_TRUE(rig->start().is_ok());
    State s;
    Command out;
    for (int k = 0; k < 50; ++k) {
      ASSERT_TRUE(rig->cycle(s, out).is_ok()) << "step " << k;  // q_from_motor false
      EXPECT_TRUE(bits_equal(out.q, rig->loop.last_record().q_ref)) << "step " << k;
    }
  }
}

// NaN read -> kHolding at the last finite q; the hold goes through the drive: kPosition, q = the
// hold target, qd = 0, tau = clamp(G(q_hold)) (kGravity, kInverseDynamics) or 0 (kNone), every
// holding step after it too (finite reads again).
TEST(LoopDrive, Hold) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  for (const DriveFeedForward ff :
       {DriveFeedForward::kNone, DriveFeedForward::kGravity, DriveFeedForward::kInverseDynamics}) {
    const auto rig = std::make_unique<DriveRig>(setup->model, drive_config(setup->config, ff));
    ASSERT_TRUE(rig->start().is_ok());
    State s;
    Command out;
    for (int k = 0; k < 20; ++k) {
      ASSERT_TRUE(rig->cycle(s, out).is_ok()) << name_of(ff) << " step " << k;
    }
    const Vec q_hold = s.q;  // the last finite state given to step()
    static_cast<void>(rig->cycle(s, out, [](State& st) { st.q(4) = kNaN; }));
    ASSERT_EQ(rig->loop.mode(), LoopMode::kHolding) << name_of(ff);
    EXPECT_EQ(rig->loop.hold_reason(), HoldReason::kNonFinite) << name_of(ff);
    // The hold's feed-forward is G(q_hold) for kGravity and kInverseDynamics, 0 for kNone.
    const DriveFeedForward hold_ff =
        ff == DriveFeedForward::kNone ? DriveFeedForward::kNone : DriveFeedForward::kGravity;
    const Vec tau_hold = clamp_or_zero(
        setup->model, raw_feedforward(setup->model, hold_ff, q_hold, Vec::Zero(), Vec::Zero()));
    for (int k = 0; k < 10; ++k) {
      if (k > 0) {
        static_cast<void>(rig->cycle(s, out));
      }
      ASSERT_EQ(rig->loop.mode(), LoopMode::kHolding) << name_of(ff) << " hold step " << k;
      EXPECT_EQ(out.mode, hal::DriveMode::kPosition) << name_of(ff) << " hold step " << k;
      EXPECT_TRUE(bits_equal(out.q, q_hold)) << name_of(ff) << " hold step " << k;
      EXPECT_TRUE(out.qd.isZero(0.0)) << name_of(ff) << " hold step " << k;
      EXPECT_TRUE(bits_equal(out.tau, tau_hold))
          << name_of(ff) << " hold step " << k << ": " << out.tau.transpose() << ", expected "
          << tau_hold.transpose();
      EXPECT_TRUE(bits_equal(rig->loop.last_record().q_ref, q_hold)) << name_of(ff);
    }
    if (ff == DriveFeedForward::kNone) {
      EXPECT_TRUE(tau_hold.isZero(0.0));
    } else {
      EXPECT_GT(tau_hold.cwiseAbs().maxCoeff(), 1.0) << "gravity at the hold target";
    }
  }
}

}  // namespace
}  // namespace cobot::pipeline::test
