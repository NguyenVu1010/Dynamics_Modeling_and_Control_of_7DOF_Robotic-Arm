// Stability on each reference profile (brief "profiles", tests; realism-2 design 8 and 4.6).
//
// The default trajectory (10 s + 2 s hold, deflection compensation enabled)
// on premium_fr3, mid_kr1205 and budget_xarm7 with seeds 1, 2, 3, wired as cobot_bench wires a run
// (bench/run.cpp): the plant is the URDF model perturbed by the profile's model group with the
// profile's plant effects (friction, motor lag, elastic transmission), driven through
// SimJointInterface (period jitter, overrun) and ImperfectJointInterface (sensor, actuator,
// comm); the controller is ControlLoop on the nominal controller model (peak torque P).
//
// Controller config per profile (owner decision 2026-11-03): config/<profile>.yaml, equal to
// config/default.yaml except the computed-torque gains, critically damped at w = 50 (premium_fr3),
// 40 (mid_kr1205) and 20 rad/s (budget_xarm7). Real arms are tuned to their own
// hardware; w = 80 of default.yaml (ideal hardware) leaves a ~60 Hz oscillation on premium and mid
// with the profiles' 1 + 1 cycles of sensor and command delay on the elastic joints, and torque at
// P about half the time. Gain sweep w = 20..80, seeds 1-3 (cobot_bench montecarlo, final TCP error,
// peak torque ratio): premium 0.04-0.12 mm at w = 50 with peak 0.42, saturated from w = 60; mid
// 0.07-0.58 mm at w = 40 with peak 0.47, saturated from w = 50; budget held on every seed at every
// gain (overspeed or state_timeout; with hal_timeout 20 ms and w = 20, seed 1 still holds with
// overspeed at 8.6 s).
//
// Pass criteria per run, premium_fr3 and mid_kr1205:
//   - finite state and torque, and every read comes from the motor encoder (q_from_motor), so
//     the deflection compensation is active;
//   - no hold;
//   - the post-goal oscillation of the joint error decays: with e_j = q_j - q_ref_j, its swing
//     max_j (max e_j - min e_j) over the last 0.5 s (t in [11.5, 12) s) is below its swing over
//     the first 0.5 s after the goal (t in [10, 10.5) s). q is the controller's working position
//     (LogRecord::q: q_hat = q_motor - deflection(g(q)) with the compensation active, the Run CSV
//     `q`, docs/conventions.md). The swing (range per joint) is the amplitude of design 8 ("bien do
//     dao dong sai so khop sau khi toi dich giam dan"). The peak of max_j |e_j| is printed but not
//     gated: after settling, e_j is dominated by a quasi-static offset (friction, lost motion,
//     deadband), so that peak is the same in both windows and its comparison is decided by noise.
// budget_xarm7 (with and without the comm and overrun groups): reported, not gated, beyond finite
// values and motor-encoder reads. Torque commands from the host at servo 250 Hz with the delays and
// elastic joints are unstable at every gain measured; arms of this class close the position and
// current loops in the joint drive at kHz rates and take position set points from the host. That
// drive-side loop is a later feature; until then the budget result is a reported finding.
// Reported, not gated: the true-link error max_j |q_true_j - q_ref_j| over the last 0.5 s (lost
// motion and transmission error are not compensated, PR #50).
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/dynamics/sim_plant.hpp"
#include "cobot/imperfect/config.hpp"
#include "cobot/imperfect/imperfect_joint_interface.hpp"
#include "cobot/imperfect/perturb.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/control_loop.hpp"
#include "cobot/pipeline/log_record.hpp"
#include "cobot/pipeline/sim_joint_interface.hpp"

namespace cobot::pipeline::test {
namespace {

namespace fs = std::filesystem;
constexpr int kN = 7;
using Vec = common::VecN<kN>;
using Profile = imperfect::ImperfectionConfig<kN>;

constexpr double kUsPerS = 1e6;
constexpr double kWindowS = 0.5;  // envelope windows after the goal and before the end, s

// Each run is 12000 elastic plant steps (RK4 sub-steps at the wrist stiffness): a few seconds
// optimised, beyond the ctest timeout in the unoptimised sanitizer build, which compiles these
// tests and skips them (as tests/unit/loop_deflection_comp does).
#if defined(__OPTIMIZE__)
constexpr bool kOptimised = true;
#else
constexpr bool kOptimised = false;
#endif

struct Case {
  std::string profile;
  std::uint64_t seed;
  bool comm_off;  // budget variant: packet loss, latency bursts and overrun off
};

std::ostream& operator<<(std::ostream& os, const Case& c) {
  return os << c.profile << " seed " << c.seed << (c.comm_off ? " comm off" : "");
}

struct RunResult {
  bool finite = true;
  int reads = 0;
  int motor_reads = 0;  // states with q_from_motor
  bool held = false;
  HoldReason hold_reason = HoldReason::kNone;
  double hold_t_s = 0.0;
  double peak_first = 0.0;     // max_j |q - q_ref| over [goal, goal + 0.5 s)
  double peak_last = 0.0;      // ... over the last 0.5 s
  double true_err_last = 0.0;  // max_j |q_true - q_ref| over the last 0.5 s
  double swing_first = 0.0;    // max_j (max - min of q_j - q_ref_j) over [goal, goal + 0.5 s)
  double swing_last = 0.0;     // ... over the last 0.5 s
};

// Per joint, the range (max - min) of a signal over a window.
struct Swing {
  Vec lo = Vec::Constant(std::numeric_limits<double>::infinity());
  Vec hi = Vec::Constant(-std::numeric_limits<double>::infinity());
  void add(const Vec& e) {
    lo = lo.cwiseMin(e);
    hi = hi.cwiseMax(e);
  }
  double max_range() const { return (hi - lo).maxCoeff(); }
};

struct Rig {
  Config<kN> config;
  model::Arm7 urdf;     // plant model before perturbation (torque cap M)
  model::Arm7 nominal;  // controller model (peak torque P)
};

// The profile's controller config (config/<profile>.yaml).
std::optional<Rig> load_rig(const std::string& profile) {
  const auto cfg = load_config<kN>(fs::path(COBOT_REPO_DIR) / "config" / (profile + ".yaml"));
  if (!cfg) {
    ADD_FAILURE() << "load_config: " << cfg.reason();
    return std::nullopt;
  }
  const Config<kN>& c = cfg.value();
  const auto chain = model::parse_urdf_chain(c.urdf_path, c.base_link, c.tip_link);
  if (!chain) {
    ADD_FAILURE() << "parse_urdf_chain: " << chain.reason();
    return std::nullopt;
  }
  const auto arm = model::make_model<kN>(chain.value(), c.gravity_b);
  if (!arm) {
    ADD_FAILURE() << "make_model: " << arm.reason();
    return std::nullopt;
  }
  return Rig{c, arm.value(), controller_model<kN>(arm.value(), c)};
}

std::optional<Profile> load_profile(const Case& c) {
  const fs::path path = fs::path(COBOT_REPO_DIR) / "config/imperfections" / (c.profile + ".yaml");
  const auto r = load_imperfections<kN>(path);
  if (!r) {
    ADD_FAILURE() << path << ": " << r.reason();
    return std::nullopt;
  }
  Profile p = r.value();
  p.seed = c.seed;
  if (c.comm_off) {
    p.comm.packet_loss_prob = 0.0;
    p.comm.latency_burst_prob = 0.0;
    p.timing.overrun_prob = 0.0;
  }
  return p;
}

// One run, read -> step -> write -> advance per cycle as bench/run.cpp does it.
RunResult run(const Rig& rig, const Profile& p) {
  const Config<kN>& cfg = rig.config;
  const double dt = 1.0 / cfg.control_rate_hz;
  // 10 s / 1 ms = 10000 cycles to the goal; (10 + 2) s / 1 ms = 12000 cycles; 0.5 s = 500 cycles.
  const int goal_k = static_cast<int>(std::lround(cfg.trajectory.duration_s / dt));
  const int cycles =
      static_cast<int>(std::lround((cfg.trajectory.duration_s + cfg.trajectory.hold_s) / dt));
  const int window = static_cast<int>(std::lround(kWindowS / dt));

  const model::Arm7 plant_model = imperfect::perturb_model(rig.urdf, p.model, p.seed);
  dynamics::SimPlant<kN> plant(plant_model, cfg.home_q_rad, imperfect::plant_effects(p));
  SimJointInterface<kN> sim(plant,
                            dt,
                            p.timing.period_jitter_std_us / kUsPerS,
                            p.seed,
                            p.timing.overrun_prob,
                            p.timing.overrun_cycles);
  imperfect::ImperfectJointInterface<kN> io(sim, p, dt, false, cfg.limits.peak_torque_nm);
  const auto loop = std::make_unique<ControlLoop<kN>>(rig.nominal, cfg);

  RunResult r;
  Swing first;
  Swing last;
  hal::JointState<kN> s;
  hal::JointCommand<kN> cmd;
  static_cast<void>(io.read(s));
  EXPECT_TRUE(loop->start(s).is_ok());
  for (int k = 0; k < cycles; ++k) {
    static_cast<void>(io.read(s));
    ++r.reads;
    r.motor_reads += s.q_from_motor ? 1 : 0;
    const Vec& q_true = plant.q();  // link angle at the time of this read (used before advance)
    EXPECT_TRUE(loop->step(s, cmd).is_ok());
    const LogRecord<kN>& rec = loop->last_record();
    r.finite = r.finite && rec.q.allFinite() && q_true.allFinite() && cmd.tau.allFinite();
    if (!r.held && loop->mode() == LoopMode::kHolding) {
      r.held = true;
      r.hold_reason = loop->hold_reason();
      r.hold_t_s = k * dt;
    }
    const Vec e = rec.q - rec.q_ref;
    const double err = e.cwiseAbs().maxCoeff();
    if (k >= goal_k && k < goal_k + window) {
      r.peak_first = std::max(r.peak_first, err);
      first.add(e);
    }
    if (k >= cycles - window) {
      r.peak_last = std::max(r.peak_last, err);
      last.add(e);
      r.true_err_last = std::max(r.true_err_last, (q_true - rec.q_ref).cwiseAbs().maxCoeff());
    }
    static_cast<void>(io.write(cmd));
    sim.advance();
  }
  r.swing_first = first.max_range();
  r.swing_last = last.max_range();
  return r;
}

class ProfileStability : public ::testing::TestWithParam<Case> {};

TEST_P(ProfileStability, PremiumAndMidHoldFreeAndDecayingBudgetReported) {
  if (!kOptimised) {
    GTEST_SKIP() << "elastic 7-DOF closed loop: release builds only (run time)";
  }
  const Case& c = GetParam();
  const auto rig = load_rig(c.profile);
  ASSERT_TRUE(rig);
  const auto profile = load_profile(c);
  ASSERT_TRUE(profile);
  ASSERT_EQ(profile->sensor.position_source, imperfect::PositionSource::kMotor) << c;
  ASSERT_TRUE(rig->config.deflection_comp.enabled);

  const RunResult r = run(*rig, *profile);
  std::printf(
      "[ profile ] %s seed %llu%s: held %d (%s at %.3f s), max_j |q - q_ref| peak first 0.5 s "
      "after goal %.3e, last 0.5 s %.3e rad; swing first %.3e, last %.3e rad; true link error "
      "last 0.5 s %.3e rad (reported)\n",
      c.profile.c_str(),
      static_cast<unsigned long long>(c.seed),
      c.comm_off ? " comm off" : "",
      r.held ? 1 : 0,
      to_string(r.hold_reason),
      r.hold_t_s,
      r.peak_first,
      r.peak_last,
      r.swing_first,
      r.swing_last,
      r.true_err_last);

  EXPECT_TRUE(r.finite) << c;
  EXPECT_EQ(r.motor_reads, r.reads) << c << ": every read from the motor encoder";
  if (c.profile == "budget_xarm7") {
    return;  // reported finding (header): no drive-side loop yet
  }
  EXPECT_FALSE(r.held) << c << ": held (" << to_string(r.hold_reason) << ") at " << r.hold_t_s
                       << " s";
  if (!r.held) {
    EXPECT_LT(r.swing_last, r.swing_first) << c << ": post-goal oscillation does not decay";
  }
}

std::vector<Case> cases() {
  std::vector<Case> out;
  for (const char* profile : {"premium_fr3", "mid_kr1205", "budget_xarm7"}) {
    for (std::uint64_t seed = 1; seed <= 3; ++seed) {
      out.push_back({profile, seed, false});
    }
  }
  for (std::uint64_t seed = 1; seed <= 3; ++seed) {
    out.push_back({"budget_xarm7", seed, true});
  }
  return out;
}

INSTANTIATE_TEST_SUITE_P(Profiles,
                         ProfileStability,
                         ::testing::ValuesIn(cases()),
                         [](const ::testing::TestParamInfo<Case>& info) {
                           return info.param.profile + "_seed" + std::to_string(info.param.seed) +
                                  (info.param.comm_off ? "_comm_off" : "");
                         });

}  // namespace
}  // namespace cobot::pipeline::test
