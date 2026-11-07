// The re-check of the budget profile with the drive-side position loop (brief drive-loop, "What the
// budget profile must now reach"; ADR-023 prerequisite): config/budget_xarm7_drive.yaml on
// budget_xarm7, the default trajectory (10 s + 2 s hold), through bench::run_in_memory (the run
// engine of cobot_bench run / montecarlo, which attaches the SimDrive), seeds 1, 2, 3, comm and
// overrun on, and the same with packet loss, latency bursts and overrun off:
//   - no hold;
//   - final_pos_err_m <= 4.0e-3, final_rot_err_rad <= 0.0279 (1.6 deg), rms_joint_err_rad <= 0.016:
//     the E2E-12 thresholds (5 mm, 2 deg, 0.02 rad, proposed) with the 20 % margin ADR-023 needs
//     for a PASS: 0.8 * 5e-3 = 4.0e-3 m, 0.8 * 2 deg = 1.6 deg = 0.02793 rad, 0.8 * 0.02 = 0.016
//     rad;
//   - the post-goal swing decays: with e_j = q_j - q_ref_j (Run CSV q and qref), max_j (max e_j -
//     min e_j) over the last 0.5 s ([11.5, 12) s) below the same over [10, 10.5) s, as
//     tests/unit/loop_profile_stability defines it.
// Also: the ideal world with drive_position tracks (no hold, final_pos_err_m < 1e-3,
// rms_joint_err_rad < 0.005), and premium_fr3 / mid_kr1205 with drive_position are reported
// (finite, printed, not gated). Each run is 12000 control cycles of 4 drive sub-steps on elastic
// plants: release builds only (the sanitizer build compiles and skips them).
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <optional>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/imperfect/config.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/log_record.hpp"
#include "cobot/pipeline/overrides.hpp"

#include "engine.hpp"
#include "run_data.hpp"

namespace cobot::bench {
namespace {

namespace fs = std::filesystem;
const fs::path kRepo = COBOT_REPO_DIR;

#if defined(__OPTIMIZE__)
constexpr bool kOptimised = true;
#else
constexpr bool kOptimised = false;
#endif

constexpr double kWindowS = 0.5;  // swing windows after the goal and before the end, s

// Margin gates (header).
constexpr double kMaxFinalPosErrM = 4.0e-3;
constexpr double kMaxFinalRotErrRad = 0.0279;
constexpr double kMaxRmsJointErrRad = 0.016;
// Range of the measured q - q_ref over a 0.5 s window when the arm is at rest: budget_xarm7 has
// q_noise_std_rad 5e-5 and a 17-bit encoder (LSB 4.79e-5 rad); the measured q at rest has a
// std of up to 9.8e-5 rad per joint, and the range of a 0.5 s window is about 5 std (4.9e-4 rad).
// A swing at or below this floor is sensor noise, not a residual oscillation.
constexpr double kSwingNoiseFloorRad = 5.0e-4;

std::optional<imperfect::ImperfectionConfig<kN>> load_profile(const std::string& name,
                                                              std::uint64_t seed) {
  const auto r =
      pipeline::load_imperfections<kN>(kRepo / "config/imperfections" / (name + ".yaml"));
  if (!r) {
    ADD_FAILURE() << name << ": " << r.reason();
    return std::nullopt;
  }
  imperfect::ImperfectionConfig<kN> p = r.value();
  p.seed = seed;
  return p;
}

// The drive_position overrides at the brief's starting values.
std::vector<pipeline::KeyOverride> drive_overrides() {
  return {
      {"config.control.algorithm", std::string("drive_position")},
      {"config.drive.rate_hz", 4000.0},
      {"config.drive.omega_rad_s", 60.0},
      {"config.drive.zeta", 1.0},
      {"config.drive.velocity_filter_hz", 500.0},
      {"config.drive.torque_feedforward", std::string("inverse_dynamics")},
  };
}

// `src` with the drive overrides, written to the test output directory.
std::optional<fs::path> drive_config_of(const fs::path& src, const std::string& name) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "loop_drive_profiles";
  fs::create_directories(dir);
  fs::path dst = dir / (name + "_drive.yaml");
  const common::Status st = pipeline::write_config_with(src, drive_overrides(), dst);
  if (!st.is_ok()) {
    ADD_FAILURE() << src << ": " << st.reason();
    return std::nullopt;
  }
  return dst;
}

struct Swings {
  double first = 0.0;  // max_j range of q_j - q_ref_j over [goal, goal + 0.5 s)
  double last = 0.0;   // ... over the last 0.5 s
};

// Rows are one per control cycle from the first: row k is at k * period after the start.
Swings swings(const RunData& r) {
  const double dt = r.meta.period_s;
  const auto goal_k = static_cast<std::size_t>(std::lround(r.meta.goal_time_s / dt));
  const auto window = static_cast<std::size_t>(std::lround(kWindowS / dt));
  const std::size_t n = r.records.size();
  Vec lo_first = Vec::Constant(std::numeric_limits<double>::infinity());
  Vec hi_first = -lo_first;
  Vec lo_last = lo_first;
  Vec hi_last = -lo_first;
  for (std::size_t k = 0; k < n; ++k) {
    const Vec e = r.records[k].q - r.records[k].q_ref;
    if (k >= goal_k && k < goal_k + window) {
      lo_first = lo_first.cwiseMin(e);
      hi_first = hi_first.cwiseMax(e);
    }
    if (k + window >= n) {
      lo_last = lo_last.cwiseMin(e);
      hi_last = hi_last.cwiseMax(e);
    }
  }
  return {(hi_first - lo_first).maxCoeff(), (hi_last - lo_last).maxCoeff()};
}

void print(const char* what, const RunData& r, const Swings& s) {
  const RunSummary& m = r.summary;
  std::printf(
      "[ drive ] %s: held %d (%s at %.3f s), final_pos_err_m %.3e, final_rot_err_rad %.3e, "
      "rms_joint_err_rad %.3e, peak_tau_ratio %.3f, sat_time_s %.3f, rms_tau_ratio %.3f, swing "
      "first %.3e last %.3e rad\n",
      what,
      m.held ? 1 : 0,
      pipeline::to_string(m.hold_reason),
      m.hold_time_s,
      m.final_pos_err_m,
      m.final_rot_err_rad,
      m.rms_joint_err_rad,
      m.peak_tau_ratio,
      m.sat_time_s,
      m.rms_tau_ratio,
      s.first,
      s.last);
}

bool finite(const RunSummary& m) {
  return std::isfinite(m.final_pos_err_m) && std::isfinite(m.final_rot_err_rad) &&
         std::isfinite(m.rms_joint_err_rad) && std::isfinite(m.peak_tau_ratio) &&
         std::isfinite(m.rms_tau_ratio);
}

std::optional<RunData> run(const fs::path& config,
                           const imperfect::ImperfectionConfig<kN>& p,
                           const std::string& label) {
  RunRequest request;
  request.config_yaml = config;
  request.imperfections = p;
  request.label = label;
  const auto r = run_in_memory(request);
  if (!r) {
    ADD_FAILURE() << label << ": " << r.reason();
    return std::nullopt;
  }
  return r.value();
}

struct Case {
  std::uint64_t seed;
  bool comm_off;  // packet loss, latency bursts and overrun off
};

std::ostream& operator<<(std::ostream& os, const Case& c) {
  return os << "budget_xarm7 seed " << c.seed << (c.comm_off ? " comm off" : "");
}

class BudgetDrive : public ::testing::TestWithParam<Case> {};

TEST_P(BudgetDrive, BudgetDriveMeetsTheMargin) {
  if (!kOptimised) {
    GTEST_SKIP() << "elastic 7-DOF closed loop with drive sub-steps: release builds only";
  }
  const Case& c = GetParam();
  auto p = load_profile("budget_xarm7", c.seed);
  ASSERT_TRUE(p);
  ASSERT_EQ(p->comm.servo_rate_hz, 250.0);
  if (c.comm_off) {
    p->comm.packet_loss_prob = 0.0;
    p->comm.latency_burst_prob = 0.0;
    p->timing.overrun_prob = 0.0;
  }
  std::ostringstream label;
  label << c;
  const auto r = run(kRepo / "config/budget_xarm7_drive.yaml", *p, label.str());
  ASSERT_TRUE(r);
  const Swings s = swings(*r);
  print(label.str().c_str(), *r, s);
  const RunSummary& m = r->summary;
  ASSERT_TRUE(finite(m)) << c;
  EXPECT_FALSE(m.held) << c << ": held (" << pipeline::to_string(m.hold_reason) << ") at "
                       << m.hold_time_s << " s";
  EXPECT_LE(m.final_pos_err_m, kMaxFinalPosErrM) << c;
  EXPECT_LE(m.final_rot_err_rad, kMaxFinalRotErrRad) << c;
  EXPECT_LE(m.rms_joint_err_rad, kMaxRmsJointErrRad) << c;
  EXPECT_TRUE(s.last < s.first || s.last <= kSwingNoiseFloorRad)
      << c << ": post-goal swing does not decay (" << s.first << " -> " << s.last << " rad)";
}

INSTANTIATE_TEST_SUITE_P(Seeds,
                         BudgetDrive,
                         ::testing::Values(Case{1, false},
                                           Case{2, false},
                                           Case{3, false},
                                           Case{1, true},
                                           Case{2, true},
                                           Case{3, true}),
                         [](const ::testing::TestParamInfo<Case>& info) {
                           return "seed" + std::to_string(info.param.seed) +
                                  (info.param.comm_off ? "_comm_off" : "");
                         });

TEST(DriveProfiles, IdealDriveTracks) {
  if (!kOptimised) {
    GTEST_SKIP() << "release builds only (run time)";
  }
  const auto config = drive_config_of(kRepo / "tests/e2e/configs/ideal_world.yaml", "ideal_world");
  ASSERT_TRUE(config);
  const auto p = load_profile("ideal", 1);
  ASSERT_TRUE(p);
  const auto r = run(*config, *p, "ideal");
  ASSERT_TRUE(r);
  print("ideal_world drive_position", *r, swings(*r));
  EXPECT_FALSE(r->summary.held) << pipeline::to_string(r->summary.hold_reason);
  EXPECT_LT(r->summary.final_pos_err_m, 1e-3);
  EXPECT_LT(r->summary.rms_joint_err_rad, 0.005);
}

TEST(DriveProfiles, PremiumAndMidReported) {
  if (!kOptimised) {
    GTEST_SKIP() << "release builds only (run time)";
  }
  for (const char* profile : {"premium_fr3", "mid_kr1205"}) {
    const std::string name = profile;
    const auto config = drive_config_of(kRepo / "config" / (name + ".yaml"), name);
    ASSERT_TRUE(config) << name;
    const auto p = load_profile(name, 1);
    ASSERT_TRUE(p) << name;
    const auto r = run(*config, *p, name);
    ASSERT_TRUE(r) << name;
    print((name + " seed 1 drive_position").c_str(), *r, swings(*r));
    EXPECT_TRUE(finite(r->summary)) << name;
  }
}

}  // namespace
}  // namespace cobot::bench
