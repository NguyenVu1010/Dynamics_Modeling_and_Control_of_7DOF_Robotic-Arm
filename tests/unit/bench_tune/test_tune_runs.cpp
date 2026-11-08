// bench::tune on real runs (brief auto-tune, section 5; ADR-023). Each run is 12000 elastic control
// cycles: release builds only (kOptimised), as tests/unit/loop_drive_profiles.
//
// Numbers measured on main e750b01 through bench::run_in_memory with the same override lists (WSL
// Ubuntu 24.04, x86_64, Release; about 3.5 s per torque run and 6 s per drive run), reference_move:
//   premium_fr3 CT omega 50, seeds 1..3: no hold, sat 0, rms 3.17e-4 2.99e-4 2.93e-4 rad, final pos
//     7.7e-5 1.2e-4 4.1e-5 m, swing of q 2.6e-4 -> 3.0e-4 rad (under the floor 5e-4)
//   premium_fr3 CT omega 80, seeds 1..3: no hold, sat fraction 0.53 0.49 0.55 (#51)
//   budget_xarm7 CT omega 20, seeds 1, 2: held by state_timeout at 0.33 and 0.49 s, sat 0
//   budget_xarm7 CT omega 40, seeds 1, 2: held by state_timeout / overspeed at 0.33 / 0.34 s, sat
//     fraction 0.98
//   budget_xarm7 drive omega 80, seeds 1, 2: no hold, 0 rows at P in taum, rms 3.43e-3 1.22e-3 rad,
//     swing of q 3.8e-4 -> 4.3e-4 and 3.4e-4 -> 4.3e-4 rad (under the floor)
//   budget_xarm7 drive omega 160, seed 1: no hold, 26 rows at P in taum and 0 rows at P in tau, the
//     case that tells the two torques apart
//   premium_fr3 pd_gravity kp 2500, kd 100 (omega 50), seed 1: no hold, rms 2.97e-4 rad
// Levels sit far from the 5 % limit, so the expectations are not sensitive to the last bits.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/control/algorithm.hpp"
#include "cobot/pipeline/hardware_model.hpp"
#include "cobot/pipeline/log_record.hpp"
#include "cobot/pipeline/overrides.hpp"
#include "cobot/pipeline/task.hpp"

#include "run_data.hpp"
#include "tune.hpp"
#include "tune_test_util.hpp"

namespace cobot::bench {
namespace {

using control::Algorithm;
using namespace tune_test;  // NOLINT(google-build-using-namespace)

constexpr double kSatFraction = 0.999;  // metrics.cpp, cobot_eval.metrics.SAT_FRACTION
constexpr double kSatLimit = 0.05;      // saturation under 5 % of the run

bool same_number(double a, double b) {
  return a == b || (std::isnan(a) && std::isnan(b));
}

void expect_same_result(const TuneResult& a, const TuneResult& b) {
  EXPECT_TRUE(a.algorithm == b.algorithm);
  EXPECT_EQ(a.key, b.key);
  EXPECT_EQ(a.levels, b.levels);
  EXPECT_EQ(a.chosen, b.chosen);
  EXPECT_EQ(a.reason, b.reason);
  ASSERT_EQ(a.rows.size(), b.rows.size());
  for (std::size_t i = 0; i < a.rows.size(); ++i) {
    const TuneRow& x = a.rows[i];
    const TuneRow& y = b.rows[i];
    EXPECT_EQ(x.level, y.level) << i;
    EXPECT_EQ(x.seeds, y.seeds) << i;
    EXPECT_EQ(x.held, y.held) << i;
    EXPECT_TRUE(same_number(x.sat_frac_max, y.sat_frac_max))
        << i << ": " << x.sat_frac_max << " / " << y.sat_frac_max;
    EXPECT_EQ(x.decays_all, y.decays_all) << i;
    EXPECT_TRUE(same_number(x.rms_joint_err_median, y.rms_joint_err_median))
        << i << ": " << x.rms_joint_err_median << " / " << y.rms_joint_err_median;
    EXPECT_TRUE(same_number(x.final_pos_err_max, y.final_pos_err_max))
        << i << ": " << x.final_pos_err_max << " / " << y.final_pos_err_max;
  }
}

// One run outside bench::tune, from a written config: what tune must reproduce. `level` empty keeps
// the model's own gain.
std::optional<RunData> run_direct(const pipeline::HardwareModel& model,
                                  Algorithm algorithm,
                                  const pipeline::Task& task,
                                  std::optional<double> level,
                                  std::uint64_t seed,
                                  const fs::path& dir) {
  const pipeline::RunOverrides ro = pipeline::run_overrides(model, algorithm, task);
  const fs::path config = dir / "direct.yaml";
  const common::Status written = pipeline::write_config_with(
      model.controller, level ? with_gain(ro.config, algorithm, *level) : ro.config, config);
  if (!written.is_ok()) {
    ADD_FAILURE() << written.reason();
    return std::nullopt;
  }
  auto profile = pipeline::load_imperfections_with<kN>(model.imperfections, ro.imperfections);
  if (!profile.is_ok()) {
    ADD_FAILURE() << profile.reason();
    return std::nullopt;
  }
  profile.value().seed = seed;
  RunRequest request;
  request.config_yaml = config;
  request.imperfections = profile.value();
  request.label = model.name;
  request.waypoints_yaml = ro.waypoints;
  auto run = run_in_memory(request);
  if (!run.is_ok()) {
    ADD_FAILURE() << run.reason();
    return std::nullopt;
  }
  return std::move(run.value());
}

#define COBOT_RELEASE_ONLY()                                          \
  if (!kOptimised) {                                                  \
    GTEST_SKIP() << "elastic 7-DOF closed loop: release builds only"; \
  }

TEST(TuneRuns, PremiumComputedTorqueChoosesFiftyAndEightySaturates) {
  COBOT_RELEASE_ONLY();
  const auto r = tune(load_model("premium_fr3"),
                      Algorithm::kComputedTorque,
                      load_task("reference_move"),
                      options({50.0, 80.0}, {1, 2, 3}, 2));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const TuneResult& t = r.value();
  EXPECT_TRUE(t.algorithm == Algorithm::kComputedTorque);
  EXPECT_EQ(t.key, "config.computed_torque.omega");
  EXPECT_EQ(t.levels, (std::vector<double>{50.0, 80.0}));
  ASSERT_EQ(t.rows.size(), 2U);

  const TuneRow& fifty = t.rows[0];
  EXPECT_EQ(fifty.level, 50.0);
  EXPECT_EQ(fifty.seeds, 3U);
  EXPECT_EQ(fifty.held, 0U);
  EXPECT_LT(fifty.sat_frac_max, kSatLimit);
  EXPECT_TRUE(fifty.decays_all);
  EXPECT_LT(fifty.rms_joint_err_median, 1e-3);
  EXPECT_LT(fifty.final_pos_err_max, 1e-3);

  const TuneRow& eighty = t.rows[1];
  EXPECT_EQ(eighty.level, 80.0);
  EXPECT_EQ(eighty.held, 0U);
  EXPECT_GE(eighty.sat_frac_max, kSatLimit);

  ASSERT_TRUE(t.chosen.has_value()) << t.reason;
  EXPECT_EQ(*t.chosen, 50.0) << t.reason;
  EXPECT_FALSE(contains(t.reason, "no admissible gain")) << t.reason;
}

TEST(TuneRuns, BudgetComputedTorqueHasNoAdmissibleGain) {
  COBOT_RELEASE_ONLY();
  const auto r = tune(load_model("budget_xarm7"),
                      Algorithm::kComputedTorque,
                      load_task("reference_move"),
                      options({20.0, 40.0}, {1, 2}, 2));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const TuneResult& t = r.value();
  ASSERT_EQ(t.rows.size(), 2U);
  for (const TuneRow& row : t.rows) {
    EXPECT_EQ(row.seeds, 2U) << row.level;
    EXPECT_EQ(row.held, 2U) << row.level;
    EXPECT_FALSE(row.decays_all) << row.level << ": a held run has a NaN swing";
    EXPECT_TRUE(std::isnan(row.rms_joint_err_median)) << row.level;
    EXPECT_TRUE(std::isnan(row.final_pos_err_max)) << row.level;
  }
  EXPECT_FALSE(t.chosen.has_value());
  EXPECT_TRUE(contains(t.reason, "no admissible gain")) << t.reason;
  // Both levels hold and do not decay; only 40 saturates.
  EXPECT_TRUE(contains(t.reason, "held 2")) << t.reason;
  EXPECT_TRUE(contains(t.reason, "saturated 1")) << t.reason;
  EXPECT_TRUE(contains(t.reason, "not decaying 2")) << t.reason;
}

TEST(TuneRuns, BudgetDriveDoesNotHold) {
  COBOT_RELEASE_ONLY();
  const auto r = tune(load_model("budget_xarm7"),
                      Algorithm::kDrivePosition,
                      load_task("reference_move"),
                      options({80.0}, {1, 2}, 2));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const TuneResult& t = r.value();
  EXPECT_EQ(t.key, "config.drive.omega_rad_s");
  ASSERT_EQ(t.rows.size(), 1U);
  EXPECT_EQ(t.rows[0].held, 0U);
  EXPECT_LT(t.rows[0].sat_frac_max, kSatLimit);
  EXPECT_TRUE(t.rows[0].decays_all);
  EXPECT_TRUE(std::isfinite(t.rows[0].rms_joint_err_median));
  EXPECT_TRUE(std::isfinite(t.rows[0].final_pos_err_max));
  ASSERT_TRUE(t.chosen.has_value()) << t.reason;
  EXPECT_EQ(*t.chosen, 80.0);
}

TEST(TuneRuns, ADriveRunSaturatesOnTheMotorTorque) {
  COBOT_RELEASE_ONLY();
  // Omega 160: taum reaches P and the feed-forward tau never does (ambiguity 7).
  const pipeline::HardwareModel model = load_model("budget_xarm7");
  const pipeline::Task task = load_task("reference_move");
  const auto r = tune(model, Algorithm::kDrivePosition, task, options({160.0}, {1}, 1));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  ASSERT_EQ(r.value().rows.size(), 1U);

  const fs::path dir = test_dir();
  const auto run = run_direct(model, Algorithm::kDrivePosition, task, 160.0, 1, dir);
  ASSERT_TRUE(run.has_value());
  std::size_t motor_rows = 0;
  std::size_t command_rows = 0;
  for (const pipeline::LogRecord<kN>& row : run->records) {
    const auto ratio = [&run](const Vec& tau) {
      return (tau.cwiseAbs().array() / run->meta.tau_max.array()).maxCoeff();
    };
    motor_rows += ratio(row.tau_motor) >= kSatFraction ? 1 : 0;
    command_rows += ratio(row.tau) >= kSatFraction ? 1 : 0;
  }
  ASSERT_GT(motor_rows, 0U);
  ASSERT_EQ(command_rows, 0U);

  EXPECT_GT(r.value().rows[0].sat_frac_max, 0.0);
  EXPECT_EQ(r.value().rows[0].sat_frac_max, run->summary.sat_time_s / run->summary.run_time_s);
}

TEST(TuneRuns, ThePdGravityGainIsTheModelsAtTheSameOmega) {
  COBOT_RELEASE_ONLY();
  // premium_fr3 sets kp 2500, kd 100: omega 50 is the same law (50^2 = 2500, 2 * 50 = 100).
  const pipeline::HardwareModel model = load_model("premium_fr3");
  const pipeline::Task task = load_task("reference_move");
  const auto r = tune(model, Algorithm::kPdGravity, task, options({50.0}, {1}, 1));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().key, "config.pd_gravity.omega");
  ASSERT_EQ(r.value().rows.size(), 1U);

  const fs::path dir = test_dir();
  const auto run = run_direct(model, Algorithm::kPdGravity, task, std::nullopt, 1, dir);
  ASSERT_TRUE(run.has_value());
  ASSERT_FALSE(run->summary.held);
  EXPECT_EQ(r.value().rows[0].held, 0U);
  EXPECT_EQ(r.value().rows[0].rms_joint_err_median, run->summary.rms_joint_err_rad);
  EXPECT_EQ(r.value().rows[0].final_pos_err_max, run->summary.final_pos_err_m);
}

TEST(TuneRuns, AWaypointTaskRunsTheListWithItsGoalTimeAndPayload) {
  COBOT_RELEASE_ONLY();
  // tests/data/waypoints/tune_short.yaml: goal at 4.86 s, 6860 rows at 1 kHz with the 2 s hold (the
  // controller files say trajectory.duration_s 10, so a window placed by that finds no rows).
  const pipeline::HardwareModel model = load_model("premium_fr3");
  pipeline::Task task = load_task("pick_place_3kg");
  task.waypoints = kRepo / "tests/data/waypoints/tune_short.yaml";
  const auto r = tune(model, Algorithm::kComputedTorque, task, options({50.0}, {1}, 1));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  ASSERT_EQ(r.value().rows.size(), 1U);

  const fs::path dir = test_dir();
  const auto run = run_direct(model, Algorithm::kComputedTorque, task, 50.0, 1, dir);
  ASSERT_TRUE(run.has_value());
  ASSERT_LT(run->meta.goal_time_s, 8.0);
  EXPECT_NEAR(run->summary.run_time_s, run->meta.window_end_s, 2.0 * run->meta.period_s);
  EXPECT_TRUE(std::isfinite(run->summary.swing_first_rad));
  EXPECT_TRUE(std::isfinite(run->summary.swing_last_rad));

  const TuneRow& row = r.value().rows[0];
  EXPECT_EQ(row.rms_joint_err_median, run->summary.rms_joint_err_rad);
  EXPECT_EQ(row.final_pos_err_max, run->summary.final_pos_err_m);
  EXPECT_EQ(row.sat_frac_max, run->summary.sat_time_s / run->summary.run_time_s);
  EXPECT_EQ(row.decays_all,
            swing_decays(run->summary.swing_first_rad, run->summary.swing_last_rad));
}

TEST(TuneRuns, EmptySeedsMeanOneToTen) {
  COBOT_RELEASE_ONLY();
  const auto r = tune(load_model("premium_fr3"),
                      Algorithm::kComputedTorque,
                      load_task("reference_move"),
                      options({50.0}, {}, 0));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  ASSERT_EQ(r.value().rows.size(), 1U);
  EXPECT_EQ(r.value().rows[0].seeds, 10U);
}

TEST(TuneRuns, TheFirstErrorOfAnyRunComesBackWithWorkersBusy) {
  COBOT_RELEASE_ONLY();
  const auto r = tune(load_model("premium_fr3"),
                      Algorithm::kComputedTorque,
                      load_task("reference_move"),
                      options({50.0, -1.0}, {1}, 2));
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(r.reason(), "omega")) << r.reason();
}

struct JobsCase {
  const char* name;
  const char* model;
  Algorithm algorithm;
  std::vector<double> levels;
  std::vector<std::uint64_t> seeds;
};

class TuneJobs : public ::testing::TestWithParam<JobsCase> {};

TEST_P(TuneJobs, OneAndTwoJobsGiveTheSameResultAndTheSameBytes) {
  COBOT_RELEASE_ONLY();
  const JobsCase& c = GetParam();
  const pipeline::HardwareModel model = load_model(c.model);
  const pipeline::Task task = load_task("reference_move");
  const auto one = tune(model, c.algorithm, task, options(c.levels, c.seeds, 1));
  ASSERT_TRUE(one.is_ok()) << one.reason();
  const auto two = tune(model, c.algorithm, task, options(c.levels, c.seeds, 2));
  ASSERT_TRUE(two.is_ok()) << two.reason();
  expect_same_result(one.value(), two.value());

  const fs::path dir = test_dir();
  const fs::path csv_one = dir / "one.csv";
  const fs::path csv_two = dir / "two.csv";
  ASSERT_TRUE(write_tune_csv(csv_one, one.value()).is_ok());
  ASSERT_TRUE(write_tune_csv(csv_two, two.value()).is_ok());
  const std::string bytes = read_text(csv_one);
  ASSERT_FALSE(bytes.empty());
  EXPECT_TRUE(bytes == read_text(csv_two)) << bytes << "\n---\n" << read_text(csv_two);
}

INSTANTIATE_TEST_SUITE_P(
    Laws,
    TuneJobs,
    ::testing::Values(
        JobsCase{"premium_ct", "premium_fr3", Algorithm::kComputedTorque, {50.0, 80.0}, {1, 2}},
        JobsCase{"budget_drive", "budget_xarm7", Algorithm::kDrivePosition, {80.0}, {1, 2}}),
    [](const ::testing::TestParamInfo<JobsCase>& info) { return info.param.name; });

}  // namespace
}  // namespace cobot::bench
