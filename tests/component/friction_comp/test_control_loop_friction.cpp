#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/control/friction_compensation.hpp"
#include "cobot/dynamics/sim_plant.hpp"

#include "../control_loop/control_loop_test_util.hpp"

// ControlLoop with friction compensation (feature `friction-comp`): in tracking
//   tau = clamp(computed_torque(state, ref) + coulomb .* tanh(qd_ref / eps) + viscous .* qd_ref,
//               -tau_max, tau_max),
// the hold and the fault are unchanged, and the logged tau is the final command.
namespace cobot::pipeline::test {
namespace {

constexpr std::int64_t kStepNs = 1000000;  // 1 / 1000 Hz = 1 ms

// The nominal identified values of config/default.yaml, set here explicitly so these tests do
// not depend on the file: coulomb (1.36 x4, 0.192 x3) N m, viscous (0.4 x4, 0.08 x3) N m s/rad.
control::FrictionCompensationConfig<kN> nominal_compensation(bool enabled) {
  control::FrictionCompensationConfig<kN> f;
  f.enabled = enabled;
  f.coulomb_nm << 1.36, 1.36, 1.36, 1.36, 0.192, 0.192, 0.192;
  f.viscous_nm_s_rad << 0.4, 0.4, 0.4, 0.4, 0.08, 0.08, 0.08;
  f.velocity_eps_rad_s = 0.01;
  return f;
}

Config<kN> with_compensation(const Config<kN>& base,
                             const control::FrictionCompensationConfig<kN>& f) {
  Config<kN> c = base;
  c.friction_compensation = f;
  return c;
}

// The feed-forward law written out, independent of the class under test.
Vec feedforward(const control::FrictionCompensationConfig<kN>& f, const Vec& qd_ref) {
  Vec tau;
  for (int i = 0; i < kN; ++i) {
    tau(i) = f.coulomb_nm(i) * std::tanh(qd_ref(i) / f.velocity_eps_rad_s) +
             f.viscous_nm_s_rad(i) * qd_ref(i);
  }
  return tau;
}

Record pop_one(ControlLoop<kN>& loop) {
  Record r;
  EXPECT_TRUE(loop.log().pop(r));
  return r;
}

// Two loops (compensation off / on) fed the same state each cycle. CLIK never reads the
// measured state, so both produce the same reference; the state follows the reference one
// cycle late (q = q_ref and qd = qd_ref of the previous cycle), which keeps the computed torque
// small and away from the clamp.
struct Pair {
  Pair(const Setup& setup, const Config<kN>& off_cfg, const Config<kN>& on_cfg)
      : off(setup.model, off_cfg), on(setup.model, on_cfg) {
    state.t = common::Timestamp{0};
    state.q = setup.config.home_q_rad;
    EXPECT_TRUE(off.start(state).is_ok());
    EXPECT_TRUE(on.start(state).is_ok());
  }

  // Step k at t = k ms with the current state; returns false if either step failed.
  bool step(int k) {
    state.t = common::Timestamp{static_cast<std::int64_t>(k) * kStepNs};
    const bool ok_off = off.step(state, cmd_off).is_ok();
    const bool ok_on = on.step(state, cmd_on).is_ok();
    rec_off = pop_one(off);
    rec_on = pop_one(on);
    return ok_off && ok_on;
  }

  // Next state: the reference of this cycle.
  void follow() {
    state.q = rec_off.q_ref;
    state.qd = rec_off.qd_ref;
  }

  ControlLoop<kN> off;
  ControlLoop<kN> on;
  State state;
  Command cmd_off;
  Command cmd_on;
  Record rec_off;
  Record rec_on;
};

TEST(ControlLoopFriction, TrackingAddsTheFeedforwardOfTheReferenceVelocity) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const control::FrictionCompensationConfig<kN> f = nominal_compensation(true);
  Pair pair(*setup,
            with_compensation(setup->config, nominal_compensation(false)),
            with_compensation(setup->config, f));
  const Vec tau_max = tau_max_of(setup->model);
  int moving = 0;  // joint-steps with |qd_ref| > eps, where the feed-forward is near full size
  constexpr int kSteps = 3000;  // 3 s of the 10 s trajectory
  for (int k = 0; k < kSteps; ++k) {
    ASSERT_TRUE(pair.step(k)) << "step " << k;
    ASSERT_EQ(pair.rec_off.mode, LoopMode::kTracking) << "step " << k;
    ASSERT_EQ(pair.rec_on.mode, LoopMode::kTracking) << "step " << k;
    ASSERT_EQ(pair.rec_on.qd_ref, pair.rec_off.qd_ref) << "step " << k;
    EXPECT_EQ(pair.rec_on.tau, pair.cmd_on.tau) << "logged tau is the command, step " << k;
    const Vec ff = feedforward(f, pair.rec_off.qd_ref);
    for (int i = 0; i < kN; ++i) {
      const double off = pair.cmd_off.tau(i);
      ASSERT_LT(std::abs(off), tau_max(i)) << "set-up keeps the uncompensated torque unclamped";
      const double expected = std::clamp(off + ff(i), -tau_max(i), tau_max(i));
      EXPECT_NEAR(pair.cmd_on.tau(i), expected, 1e-9 * std::max(1.0, std::abs(expected)))
          << "step " << k << ", joint " << i + 1;
      if (std::abs(pair.rec_off.qd_ref(i)) > f.velocity_eps_rad_s) {
        ++moving;
      }
    }
    pair.follow();
  }
  EXPECT_GT(moving, 1000) << "the trajectory must exercise the full-size feed-forward";
}

TEST(ControlLoopFriction, DisabledLeavesTheTorqueUnchanged) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  // Off with zero values (the missing-section default) against off with nonzero values.
  Pair pair(*setup,
            with_compensation(setup->config, control::FrictionCompensationConfig<kN>{}),
            with_compensation(setup->config, nominal_compensation(false)));
  for (int k = 0; k < 1500; ++k) {
    ASSERT_TRUE(pair.step(k)) << "step " << k;
    ASSERT_EQ(pair.cmd_on.tau, pair.cmd_off.tau) << "step " << k;
    pair.follow();
  }
}

TEST(ControlLoopFriction, CompensatedTorqueIsClampedToTauMax) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  control::FrictionCompensationConfig<kN> huge = nominal_compensation(true);
  huge.coulomb_nm.setConstant(1000.0);
  huge.viscous_nm_s_rad.setZero();
  Pair pair(*setup,
            with_compensation(setup->config, nominal_compensation(false)),
            with_compensation(setup->config, huge));
  const Vec tau_max = tau_max_of(setup->model);
  int saturated = 0;
  for (int k = 0; k < 2000; ++k) {
    ASSERT_TRUE(pair.step(k)) << "step " << k;
    ASSERT_TRUE(torque_ok(setup->model, pair.cmd_on.tau))
        << "step " << k << ": " << pair.cmd_on.tau.transpose();
    EXPECT_EQ(pair.rec_on.tau, pair.cmd_on.tau) << "logged tau is the clamped command, step " << k;
    for (int i = 0; i < kN; ++i) {
      const double qd_ref = pair.rec_off.qd_ref(i);
      // |qd_ref| >= eps: 1000 * tanh(>= 1) >= 761.6 N m, more than tau_max (<= 87 N m) plus the
      // uncompensated torque (< tau_max), so the sum saturates with the sign of qd_ref.
      if (std::abs(qd_ref) >= huge.velocity_eps_rad_s &&
          std::abs(pair.cmd_off.tau(i)) < tau_max(i)) {
        EXPECT_EQ(pair.cmd_on.tau(i), std::copysign(tau_max(i), qd_ref))
            << "step " << k << ", joint " << i + 1;
        ++saturated;
      }
    }
    pair.follow();
  }
  EXPECT_GT(saturated, 100);
}

TEST(ControlLoopFriction, HoldIsNotCompensated) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  Pair pair(*setup,
            with_compensation(setup->config, nominal_compensation(false)),
            with_compensation(setup->config, nominal_compensation(true)));
  constexpr int kTrigger = 1500;
  for (int k = 0; k < kTrigger; ++k) {
    ASSERT_TRUE(pair.step(k)) << "step " << k;
    pair.follow();
  }
  // |qd_1| above qd_max (2.5 rad/s for joint_1) -> both loops hold at the q of this moment.
  pair.state.qd(0) = qd_max_of(setup->model)(0) + 0.5;
  pair.step(kTrigger);
  ASSERT_EQ(pair.off.mode(), LoopMode::kHolding);
  ASSERT_EQ(pair.on.mode(), LoopMode::kHolding);
  pair.state.qd.setZero();
  for (int k = kTrigger; k < kTrigger + 300; ++k) {
    if (k > kTrigger) {
      pair.step(k);
      // Drift a little so the hold torque is not constant.
      pair.state.q(1) += 1e-5;
    }
    ASSERT_EQ(pair.rec_on.mode, LoopMode::kHolding) << "step " << k;
    EXPECT_EQ(pair.cmd_on.tau, pair.cmd_off.tau) << "step " << k;
    EXPECT_EQ(pair.rec_on.tau, pair.cmd_on.tau) << "step " << k;
  }
}

// Closed loop on a SimPlant whose Coulomb and viscous friction equal the compensation values:
// a 2 s straight-line trajectory from home to the default goal, every cycle in tracking.
struct FrictionRun {
  double rms_rad = 0.0;  // RMS of q_ref - q over every joint and every cycle
  bool all_tracking = true;
  bool torque_in_range = true;
};

FrictionRun run_on_friction_plant(const Setup& setup, bool compensate) {
  Config<kN> cfg = with_compensation(setup.config, nominal_compensation(compensate));
  cfg.trajectory.duration_s = 2.0;
  cfg.trajectory.hold_s = 0.0;
  const control::FrictionCompensationConfig<kN> f = nominal_compensation(true);
  dynamics::PlantEffects<kN> effects;
  effects.coulomb_nm = f.coulomb_nm;
  effects.viscous_nm_s_rad = f.viscous_nm_s_rad;
  dynamics::SimPlant<kN> plant(setup.model, cfg.home_q_rad, effects);
  SimJointInterface<kN> io(plant, 1.0 / cfg.control_rate_hz);
  ControlLoop<kN> loop(setup.model, cfg);
  State s;
  EXPECT_TRUE(io.read(s).is_ok());
  EXPECT_TRUE(loop.start(s).is_ok());

  FrictionRun run;
  double sum_sq = 0.0;
  constexpr int kSteps = 2000;  // t = 0 .. 1.999 s, all before duration + hold = 2 s
  for (int k = 0; k < kSteps; ++k) {
    Command cmd;
    EXPECT_TRUE(io.read(s).is_ok());
    EXPECT_TRUE(loop.step(s, cmd).is_ok()) << "step " << k;
    EXPECT_TRUE(io.write(cmd).is_ok());
    io.advance();
    const Record r = pop_one(loop);
    run.all_tracking = run.all_tracking && r.mode == LoopMode::kTracking;
    run.torque_in_range = run.torque_in_range && torque_ok(setup.model, cmd.tau);
    sum_sq += (r.q_ref - r.q).squaredNorm();
  }
  run.rms_rad = std::sqrt(sum_sq / (kSteps * kN));
  return run;
}

TEST(ControlLoopFriction, CompensationHalvesTheTrackingErrorOnAFrictionPlant) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const FrictionRun off = run_on_friction_plant(*setup, false);
  const FrictionRun on = run_on_friction_plant(*setup, true);
  EXPECT_TRUE(off.all_tracking);
  EXPECT_TRUE(on.all_tracking);
  EXPECT_TRUE(off.torque_in_range);
  EXPECT_TRUE(on.torque_in_range);
  EXPECT_GT(off.rms_rad, 0.0);
  EXPECT_LT(on.rms_rad, 0.5 * off.rms_rad) << "RMS on " << on.rms_rad << ", off " << off.rms_rad;
}

}  // namespace
}  // namespace cobot::pipeline::test
