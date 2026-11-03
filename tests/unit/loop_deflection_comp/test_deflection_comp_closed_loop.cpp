// Closed loop on the 7-DOF arm (brief "deflection-comp", tests): elastic plant on every joint
// with the catalog stiffness (1 % damping, no transmission error, no lost motion), everything else
// ideal, position_source motor through the decorator, the E2E-01 trajectory (10 s + 2 s hold).
// The controller's nominal values are the catalog ones; the mismatch case scales the plant's
// k1, k2, k3 by 0.8 (breakpoint torques unchanged), so every true deflection is 1 / 0.8 = 1.25
// times the nominal one and the compensation removes 80 % of it.
//
// Transmission measurement (PR #48, no compensation, motor source): no hold, max |q_meas - q_ref|
// after the goal 1.15e-6 rad, while the true link lags by the static deflection, up to 9.9e-4 rad
// (joint 4), i.e. a TCP offset of the order of 1 mm.
#include <algorithm>
#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include "cobot/dynamics/sim_plant.hpp"
#include "cobot/imperfect/imperfect_joint_interface.hpp"
#include "cobot/kinematics/kinematics.hpp"
#include "cobot/pipeline/control_loop.hpp"
#include "cobot/pipeline/sim_joint_interface.hpp"

#include "deflection_test_util.hpp"

namespace cobot::pipeline::test {
namespace {

struct LoopRun {
  bool finite = true;
  bool ever_held = false;
  LoopMode final_mode = LoopMode::kIdle;
  int flagged_reads = 0;  // states with q_from_motor (expected: every read)
  int reads = 0;
  double ctrl_peak_first = 0.0;  // max_j |q_ctrl - q_ref| over [goal, goal + 0.5 s)
  double ctrl_peak_last = 0.0;   // ... over the last 0.5 s
  double true_err_last = 0.0;    // max_j |q_true - q_ref| over the last 0.5 s
  double tcp_err_final_m = 0.0;  // |p_tcp(q_true) - goal| at the last cycle
};

// One full run: read (decorator) -> step -> write -> advance, as the bench wires it.
// q_ctrl is the measured position the controller works on: the motor reading itself without the
// compensation, q_hat (compensated(), pinned by DeflectionCompLoop) with it. The raw motor
// reading minus q_ref is not used for the envelope: with the compensation it is the static
// deflection itself (~9.9e-4 rad), which still creeps by a few 1e-9 rad as the arm settles.
LoopRun run_loop(const Setup& setup, double plant_stiffness_scale, bool comp) {
  Config<kN> cfg = setup.config;
  enable_comp(cfg);
  cfg.deflection_comp.enabled = comp;
  const double dt = 1.0 / cfg.control_rate_hz;
  const int goal_k = static_cast<int>(std::lround(cfg.trajectory.duration_s / dt));  // 10000
  const int cycles =
      static_cast<int>(std::lround((cfg.trajectory.duration_s + cfg.trajectory.hold_s) / dt));
  const int window = static_cast<int>(std::lround(0.5 / dt));  // 500

  dynamics::SimPlant<kN> plant(
      setup.plant_model, cfg.home_q_rad, elastic_effects(plant_stiffness_scale));
  SimJointInterface<kN> sim(plant, dt);
  imperfect::ImperfectionConfig<kN> icfg;
  icfg.sensor.position_source = imperfect::PositionSource::kMotor;
  imperfect::ImperfectJointInterface<kN> io(sim, icfg, dt, false, cfg.limits.peak_torque_nm);
  ControlLoop<kN> loop(setup.controller_model, cfg);

  LoopRun r;
  State s;
  Command cmd;
  EXPECT_TRUE(io.read(s).is_ok());
  EXPECT_TRUE(loop.start(s).is_ok());
  for (int k = 0; k < cycles; ++k) {
    EXPECT_TRUE(io.read(s).is_ok());
    ++r.reads;
    r.flagged_reads += s.q_from_motor ? 1 : 0;
    const Vec& q_true = plant.q();  // link state at the time of this read (used before advance)
    EXPECT_TRUE(loop.step(s, cmd).is_ok());
    r.ever_held = r.ever_held || loop.mode() == LoopMode::kHolding;
    const Vec& q_ref = loop.last_record().q_ref;
    r.finite = r.finite && s.q.allFinite() && q_true.allFinite() && cmd.tau.allFinite();
    if (k >= goal_k && k < goal_k + window) {
      const Vec q_ctrl = comp ? compensated(setup.controller_model, s.q) : s.q;
      r.ctrl_peak_first = std::max(r.ctrl_peak_first, (q_ctrl - q_ref).cwiseAbs().maxCoeff());
    }
    if (k >= cycles - window) {
      const Vec q_ctrl = comp ? compensated(setup.controller_model, s.q) : s.q;
      r.ctrl_peak_last = std::max(r.ctrl_peak_last, (q_ctrl - q_ref).cwiseAbs().maxCoeff());
      r.true_err_last = std::max(r.true_err_last, (q_true - q_ref).cwiseAbs().maxCoeff());
    }
    EXPECT_TRUE(io.write(cmd).is_ok());
    sim.advance();
  }
  r.final_mode = loop.mode();
  kinematics::FkState<kN> fk;
  kinematics::forward_kinematics(setup.plant_model, plant.q(), fk);
  r.tcp_err_final_m = (fk.T_b_e.p - cfg.trajectory.goal_position_m).norm();
  std::printf(
      "[ deflection ] plant stiffness x%.2f, comp %s: held %d, |q_ctrl - q_ref| peak first 0.5 s "
      "%.3e, last 0.5 s %.3e rad; true link error last 0.5 s %.3e rad; TCP %.4f mm\n",
      plant_stiffness_scale,
      comp ? "on" : "off",
      r.ever_held ? 1 : 0,
      r.ctrl_peak_first,
      r.ctrl_peak_last,
      r.true_err_last,
      1e3 * r.tcp_err_final_m);
  return r;
}

// Stable: finite, never held (still tracking or finished at the last cycle, t = 11.999 s), every
// read from the motor encoder, and the post-goal envelope of |q_ctrl - q_ref| decays: its peak
// in the last 0.5 s is below its peak in the first 0.5 s after the goal.
void expect_stable(const LoopRun& r) {
  EXPECT_TRUE(r.finite);
  EXPECT_FALSE(r.ever_held);
  EXPECT_TRUE(r.final_mode == LoopMode::kTracking || r.final_mode == LoopMode::kFinished);
  EXPECT_EQ(r.flagged_reads, r.reads);
  EXPECT_LT(r.ctrl_peak_last, r.ctrl_peak_first);
}

// Each run is 12000 elastic plant steps (RK4 sub-steps at the wrist stiffness, ~0.33 ms per step
// optimised): ~3.5 s per run in release, but well over 10 min per run in the unoptimised
// sanitizer build, beyond the ctest timeout. The release jobs run these; the sanitizer job
// compiles them and skips.
#if defined(__OPTIMIZE__)
constexpr bool kOptimised = true;
#else
constexpr bool kOptimised = false;
#endif

// Reference implementation of the brief's formula, release build (not the shipped code; the
// numbers only check that the thresholds are reachable):
//   stiffness x1.0, comp off: envelope 1.147e-6 -> 1.143e-6 rad, true 9.910e-4 rad, TCP 1.60 mm
//   stiffness x1.0, comp on:  envelope 1.540e-8 -> 7.437e-9 rad, true 1.197e-6 rad, TCP 0.0014 mm
//   stiffness x0.8, comp off: envelope 1.433e-6 -> 1.429e-6 rad, true 1.239e-3 rad, TCP 2.00 mm
//   stiffness x0.8, comp on:  envelope 2.907e-7 -> 2.864e-7 rad, true 2.480e-4 rad, TCP 0.40 mm
TEST(DeflectionCompClosedLoop, NominalStiffness) {
  if (!kOptimised) {
    GTEST_SKIP() << "elastic 7-DOF closed loop: release builds only (run time)";
  }
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const LoopRun off = run_loop(*setup, 1.0, false);
  const LoopRun on = run_loop(*setup, 1.0, true);
  expect_stable(off);
  expect_stable(on);
  // The plant really deflects: the uncompensated true link lags by ~1e-3 rad (measured 9.9e-4).
  EXPECT_GT(off.true_err_last, 1e-4);
  // Compensated: at most a fifth of the uncompensated true error, and below 2e-4 rad. What is left
  // comes from evaluating g at the motor angle instead of the link angle, at most
  // |dg/dq| phi / k ~ 1e2 * 1e-3 / 1.3e4 ~ 1e-5 rad, plus the loop's static error (~1e-6 rad).
  EXPECT_LE(on.true_err_last, 0.2 * off.true_err_last);
  EXPECT_LT(on.true_err_last, 2e-4);
  // TCP of the true link: < 0.2 mm with the compensation (1.6 mm without).
  EXPECT_LT(on.tcp_err_final_m, 0.2e-3);
  EXPECT_LT(on.tcp_err_final_m, off.tcp_err_final_m);
}

TEST(DeflectionCompClosedLoop, TwentyPercentSofterPlantStillStableAndImproved) {
  if (!kOptimised) {
    GTEST_SKIP() << "elastic 7-DOF closed loop: release builds only (run time)";
  }
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const LoopRun off = run_loop(*setup, 0.8, false);
  const LoopRun on = run_loop(*setup, 0.8, true);
  expect_stable(off);
  expect_stable(on);
  // Residual ~ (1.25 - 1) / 1.25 = 20 % of the uncompensated true error (1.239e-3 * 0.2 =
  // 2.48e-4 rad, as measured); only "reduced" is required.
  EXPECT_LT(on.true_err_last, off.true_err_last);
  EXPECT_LT(on.tcp_err_final_m, off.tcp_err_final_m);
}

}  // namespace
}  // namespace cobot::pipeline::test
