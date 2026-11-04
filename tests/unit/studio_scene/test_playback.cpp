// Playback helpers of the 3D view (feature studio-3d, cobot_studio design 6.9): the row shown at a
// cursor time, the joint angles drawn (the plant's true q when known, design "use the true q when
// available"), and the overlay traces of a run.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/imperfect/config.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/log_record.hpp"
#include "cobot/studio/playback.hpp"
#include "cobot/studio/robot_scene.hpp"

#include "run_data.hpp"
#include "studio_scene_test_util.hpp"

namespace cobot::studio {
namespace {

using test::kRepo;
using test::kUrdf;
using test::repo_packages;
namespace fs = std::filesystem;
constexpr int kN = model::kArmDof;
using Row = pipeline::LogRecord<kN>;

constexpr std::int64_t kNsPerS = 1'000'000'000;

// Rows at t = k * step_ns from t = 0.
bench::RunData rows_every(std::int64_t step_ns, std::size_t count) {
  bench::RunData run;
  run.records.resize(count);
  for (std::size_t k = 0; k < count; ++k) {
    run.records[k].t = common::Timestamp{static_cast<std::int64_t>(k) * step_ns};
  }
  return run;
}

TEST(RowAt, ClampsBeforeTheFirstAndAfterTheLastRow) {
  // Rows at 0, 0.25, 0.5, 0.75 s.
  const bench::RunData run = rows_every(kNsPerS / 4, 4);
  EXPECT_EQ(row_at(run, -0.1), 0U);
  EXPECT_EQ(row_at(run, -1e9), 0U);
  EXPECT_EQ(row_at(run, 0.75), 3U);
  EXPECT_EQ(row_at(run, 0.8), 3U);
  EXPECT_EQ(row_at(run, 1e9), 3U);
}

TEST(RowAt, ExactTimeIsThatRowAndBetweenRowsIsTheEarlierOne) {
  const bench::RunData run = rows_every(kNsPerS / 4, 4);
  EXPECT_EQ(row_at(run, 0.0), 0U);
  EXPECT_EQ(row_at(run, 0.25), 1U);
  EXPECT_EQ(row_at(run, 0.5), 2U);
  EXPECT_EQ(row_at(run, 0.1), 0U);
  EXPECT_EQ(row_at(run, 0.49), 1U);
  EXPECT_EQ(row_at(run, 0.74), 2U);
}

TEST(RowAt, ExactMillisecondTimesOfAOneKilohertzRun) {
  // Rows every 1 ms (1e6 ns). t_s values such as 0.123 are not exact in binary, so a time must
  // be compared in integer nanoseconds (docs/conventions.md: never compare floating-point
  // times): llround(0.123 * 1e9) = 123000000 ns -> row 123.
  const bench::RunData run = rows_every(kNsPerS / 1000, 1000);
  EXPECT_EQ(row_at(run, 0.001), 1U);
  EXPECT_EQ(row_at(run, 0.123), 123U);
  EXPECT_EQ(row_at(run, 0.5), 500U);
  EXPECT_EQ(row_at(run, 0.9995), 999U);  // between rows 999 (0.999 s) and the end
  EXPECT_EQ(row_at(run, 0.0005), 0U);
}

TEST(DisplayQ, FiniteTrueAnglesAreDrawn) {
  Row row;
  row.q = common::VecN<kN>::Constant(0.1);
  row.q_true = common::VecN<kN>::Constant(0.2);
  EXPECT_EQ(&display_q(row), &row.q_true);
}

TEST(DisplayQ, AnyNonFiniteTrueAngleFallsBackToTheMeasuredOne) {
  Row row;
  row.q = common::VecN<kN>::Constant(0.1);
  // Default LogRecord: q_true unknown (NaN) on every joint, as on the real robot.
  EXPECT_EQ(&display_q(row), &row.q);
  row.q_true = common::VecN<kN>::Constant(0.2);
  row.q_true[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(&display_q(row), &row.q);
  row.q_true[3] = 0.2;
  row.q_true[6] = std::numeric_limits<double>::infinity();
  EXPECT_EQ(&display_q(row), &row.q);
}

TEST(DisplayQ, TrueAnglesIgnoreTheMeasurementEvenWhenItIsNaN) {
  Row row;
  row.q = common::VecN<kN>::Constant(std::numeric_limits<double>::quiet_NaN());
  row.q_true = common::VecN<kN>::Constant(0.2);
  EXPECT_EQ(&display_q(row), &row.q_true);
}

imperfect::ImperfectionConfig<kN> ideal_profile() {
  auto r = pipeline::load_imperfections<kN>(kRepo / "config/imperfections/ideal.yaml");
  EXPECT_TRUE(r.is_ok()) << r.reason();
  return r.is_ok() ? r.value() : imperfect::ImperfectionConfig<kN>{};
}

std::optional<bench::RunData> run_bench(const fs::path& config_yaml,
                                        std::optional<double> duration_s) {
  bench::RunRequest request;
  request.config_yaml = config_yaml;
  request.imperfections = ideal_profile();
  request.label = "ideal";
  request.duration_s = duration_s;
  auto run = bench::run_in_memory(request);
  if (!run.is_ok()) {
    ADD_FAILURE() << run.reason();
    return std::nullopt;
  }
  return std::move(run).value();
}

common::Result<RobotScene> load_scene() {
  return RobotScene::load(kUrdf, "base_link", "link_7", repo_packages());
}

TEST(MakeTraces, IdealOneSecondRunTracesTheTipAndTheDesiredPath) {
  const auto scene = load_scene();
  ASSERT_TRUE(scene.is_ok()) << scene.reason();
  const auto run = run_bench(kRepo / "config/default.yaml", 1.0);
  ASSERT_TRUE(run);
  ASSERT_EQ(run->records.size(), 1000U);  // 1 s at control_rate_hz 1000
  ASSERT_FALSE(run->summary.held);
  const Traces traces = make_traces(scene.value(), *run);
  ASSERT_EQ(traces.tip.size(), run->records.size());
  ASSERT_EQ(traces.desired.size(), run->records.size());
  for (std::size_t k = 0; k < run->records.size(); ++k) {
    const Row& row = run->records[k];
    // Ideal profile: the plant truth is known on every row, so the tip comes from q_true.
    ASSERT_TRUE(row.q_true.allFinite()) << "row " << k;
    const common::Vec3 tip = scene.value().tip_pose(row.q_true).p;
    ASSERT_LE((traces.tip[k] - tip).norm(), 1e-12) << "row " << k;
    ASSERT_LE((traces.desired[k] - row.T_b_d.p).norm(), 0.0) << "row " << k;
  }
  for (const common::Vec3& goal : traces.goals) {
    EXPECT_TRUE(goal.allFinite());
  }
  EXPECT_FALSE(traces.hold_t_s.has_value());
}

TEST(MakeTraces, RunThatHoldsMarksTheHoldTimeOfItsSummary) {
  // E2E-04: unreachable goal, joint 4 reaches its soft margin and the loop holds; full duration
  // (trajectory 10 s + hold 2 s).
  const auto scene = load_scene();
  ASSERT_TRUE(scene.is_ok()) << scene.reason();
  const auto run = run_bench(kRepo / "tests/e2e/configs/e2e_04_safety_limit.yaml", std::nullopt);
  ASSERT_TRUE(run);
  ASSERT_TRUE(run->summary.held);
  const Traces traces = make_traces(scene.value(), *run);
  EXPECT_EQ(traces.tip.size(), run->records.size());
  EXPECT_EQ(traces.desired.size(), run->records.size());
  ASSERT_TRUE(traces.hold_t_s.has_value());
  EXPECT_DOUBLE_EQ(*traces.hold_t_s, run->summary.hold_time_s);
}

}  // namespace
}  // namespace cobot::studio
