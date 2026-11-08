// RunSummary::settle_time_s on synthetic rows (brief verify, section 1; plan ambiguity 11, the
// task's cycle_time_s). Reading pinned here: the time from the first row until the TCP enters and
// stays within the final thresholds around the final goal, the desired pose T_b_d of the
// evaluation row (the last row with t <= t_eval_ns, the row of final_pos_err_m), that is
// |p_d(eval) - p_e| <= pos_m and angle(R_d(eval) R_e^T) <= rot_rad on every row from that instant
// up to the evaluation row. The last entry counts; never entering, or being outside at the
// evaluation row, is NaN; already inside at the first row is 0; a held run is NaN. Without the
// thresholds the field is NaN and no other field changes.
//
// Rows every 1 ms from t = 0.1 s (so "from the first row" is not "from t = 0"); the evaluation row
// is row 999 (t = 1.099 s); rows 1000.. lie after it and are ignored. Goal pose: the reference_move
// goal, p = (-0.5, -0.6, 0.9) m, R = Rz(90 deg). Thresholds 5 mm and 2 deg.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/pipeline/log_record.hpp"

#include "engine.hpp"
#include "metrics.hpp"
#include "run_data.hpp"

namespace cobot::bench {
namespace {

using Row = pipeline::LogRecord<kN>;
using pipeline::LoopMode;

constexpr double kPi = 3.14159265358979323846;
constexpr std::int64_t kRowNs = 1000000;      // 1 ms
constexpr std::int64_t kFirstNs = 100000000;  // first row at 0.1 s
constexpr std::size_t kEvalRow = 999;
constexpr std::size_t kRows = 1200;
constexpr std::int64_t kEvalNs = kFirstNs + static_cast<std::int64_t>(kEvalRow) * kRowNs;
constexpr std::int64_t kGoalNs = kFirstNs + 600 * kRowNs;
constexpr double kPosM = 0.005;
constexpr double kRotRad = 2.0 * kPi / 180.0;
constexpr double kTolS = 1e-12;

common::Pose goal() {
  common::Pose T;
  T.p = common::Vec3(-0.5, -0.6, 0.9);
  T.R = Eigen::AngleAxisd(kPi / 2.0, common::Vec3::UnitZ()).toRotationMatrix();
  return T;
}

// The goal moved by `dy` m along y and turned by `deg` about its z axis.
common::Pose near_goal(double dy, double deg) {
  common::Pose T = goal();
  T.p.y() += dy;
  T.R = T.R * Eigen::AngleAxisd(deg * kPi / 180.0, common::Vec3::UnitZ()).toRotationMatrix();
  return T;
}

// kRows rows: T_b_d at the goal, T_b_e 10 mm away (outside), tracking then finished, small
// joint errors and torques so the other fields are not all zero.
std::vector<Row> make_rows() {
  std::vector<Row> rows(kRows);
  for (std::size_t k = 0; k < kRows; ++k) {
    Row& r = rows[k];
    r.t.ns = kFirstNs + static_cast<std::int64_t>(k) * kRowNs;
    r.mode = k < 600 ? LoopMode::kTracking : LoopMode::kFinished;
    r.q = Vec::Constant(0.3);
    r.q_ref = Vec::Constant(0.3);
    r.q_ref(2) += 1e-3 * static_cast<double>(k % 7);
    r.tau = Vec::Constant(5.0 + static_cast<double>(k % 3));
    r.tau_motor = r.tau;
    r.T_b_d = goal();
    r.T_b_e = near_goal(0.010, 0.0);
  }
  return rows;
}

// T_b_e of rows [begin, end) at (dy, deg) from the goal.
void place(std::vector<Row>& rows, std::size_t begin, std::size_t end, double dy, double deg) {
  for (std::size_t k = begin; k < end; ++k) {
    rows[k].T_b_e = near_goal(dy, deg);
  }
}

const Vec kTauMax = Vec::Constant(100.0);
const Vec kRated = Vec::Constant(50.0);

RunSummary with_thresholds(const std::vector<Row>& rows) {
  return summarize(rows,
                   kTauMax,
                   kRated,
                   kEvalNs,
                   TorqueSource::kCommand,
                   kGoalNs,
                   SettleThresholds{kPosM, kRotRad});
}

TEST(SettleTime, EntersLeavesAndReentersTheLastEntryCounts) {
  std::vector<Row> rows = make_rows();
  place(rows, 200, 400, 0.001, 0.5);     // inside
  place(rows, 400, 450, 0.010, 0.0);     // out again
  place(rows, 450, kRows, 0.002, 1.0);   // inside up to the evaluation row
  place(rows, 1000, kRows, 0.020, 0.0);  // after the evaluation row: ignored
  // Row 450 at t = 0.55 s, first row at 0.1 s: 0.45 s.
  EXPECT_NEAR(with_thresholds(rows).settle_time_s, 0.45, kTolS);
}

TEST(SettleTime, TheRotationMustBeInsideToo) {
  std::vector<Row> rows = make_rows();
  place(rows, 0, kRows, 0.001, 3.0);  // position inside, 3 deg > 2 deg
  place(rows, 300, kRows, 0.001, 1.0);
  // Row 300: 0.3 s after the first row.
  EXPECT_NEAR(with_thresholds(rows).settle_time_s, 0.3, kTolS);
}

TEST(SettleTime, NeverEnteringIsNan) {
  EXPECT_TRUE(std::isnan(with_thresholds(make_rows()).settle_time_s));
}

TEST(SettleTime, OutsideAtTheEvaluationRowIsNan) {
  std::vector<Row> rows = make_rows();
  place(rows, 100, kEvalRow, 0.001, 0.0);  // inside up to row 998, out at row 999
  EXPECT_TRUE(std::isnan(with_thresholds(rows).settle_time_s));
}

TEST(SettleTime, InsideFromTheFirstRowIsZero) {
  std::vector<Row> rows = make_rows();
  place(rows, 0, kRows, 0.001, 0.5);
  EXPECT_EQ(with_thresholds(rows).settle_time_s, 0.0);
}

TEST(SettleTime, TheBandIsAroundTheFinalGoalNotTheMovingReference) {
  // T_b_d still moves until row 600 (x offset (600 - k) * 0.1 mm); T_b_e sits at the final goal
  // from row 300. Around the evaluation row's T_b_d: row 300, 0.3 s. Around each row's own T_b_d it
  // would be row 550 ((600 - 550) * 1e-4 = 5 mm), 0.55 s.
  std::vector<Row> rows = make_rows();
  for (std::size_t k = 0; k < 600; ++k) {
    rows[k].T_b_d.p.x() += static_cast<double>(600 - k) * 1e-4;
  }
  place(rows, 300, kRows, 0.0, 0.0);
  EXPECT_NEAR(with_thresholds(rows).settle_time_s, 0.3, kTolS);
}

TEST(SettleTime, AHeldRunIsNan) {
  std::vector<Row> rows = make_rows();
  place(rows, 0, kRows, 0.001, 0.5);  // inside throughout
  rows[700].mode = LoopMode::kHolding;
  for (std::size_t k = 701; k < kRows; ++k) {
    rows[k].mode = LoopMode::kHolding;
  }
  const RunSummary s = with_thresholds(rows);
  EXPECT_TRUE(s.held);
  EXPECT_TRUE(std::isnan(s.settle_time_s));
}

bool same_bits(double a, double b) {
  std::uint64_t x = 0;
  std::uint64_t y = 0;
  std::memcpy(&x, &a, sizeof x);
  std::memcpy(&y, &b, sizeof y);
  return x == y;
}

void expect_same_other_fields(const RunSummary& a, const RunSummary& b) {
  EXPECT_TRUE(same_bits(a.final_pos_err_m, b.final_pos_err_m));
  EXPECT_TRUE(same_bits(a.final_rot_err_rad, b.final_rot_err_rad));
  EXPECT_TRUE(same_bits(a.rms_joint_err_rad, b.rms_joint_err_rad));
  EXPECT_TRUE(same_bits(a.peak_tau_ratio, b.peak_tau_ratio));
  EXPECT_TRUE(same_bits(a.sat_time_s, b.sat_time_s));
  EXPECT_TRUE(same_bits(a.rms_tau_ratio, b.rms_tau_ratio));
  EXPECT_EQ(a.held, b.held);
  EXPECT_TRUE(same_bits(a.hold_time_s, b.hold_time_s));
  EXPECT_TRUE(a.hold_reason == b.hold_reason);
  EXPECT_TRUE(same_bits(a.run_time_s, b.run_time_s));
  EXPECT_TRUE(same_bits(a.swing_first_rad, b.swing_first_rad));
  EXPECT_TRUE(same_bits(a.swing_last_rad, b.swing_last_rad));
}

TEST(SettleTime, WithoutThresholdsItIsNanAndNothingElseChanges) {
  std::vector<Row> rows = make_rows();
  place(rows, 250, kRows, 0.002, 1.0);
  rows[1100].mode = LoopMode::kHolding;  // after the evaluation row: held, hold time set
  rows[1100].hold_reason = pipeline::HoldReason::kStateTimeout;
  for (const bool held : {false, true}) {
    std::vector<Row> r = rows;
    if (!held) {
      r[1100].mode = LoopMode::kFinished;
      r[1100].hold_reason = pipeline::HoldReason::kNone;
    }
    const RunSummary with = with_thresholds(r);
    const RunSummary without =
        summarize(r, kTauMax, kRated, kEvalNs, TorqueSource::kCommand, kGoalNs, std::nullopt);
    const RunSummary old = summarize(r, kTauMax, kRated, kEvalNs, TorqueSource::kCommand, kGoalNs);
    const RunSummary no_goal = summarize(r, kTauMax, kRated, kEvalNs);
    EXPECT_TRUE(std::isnan(without.settle_time_s)) << held;
    EXPECT_TRUE(std::isnan(old.settle_time_s)) << held;
    EXPECT_TRUE(std::isnan(no_goal.settle_time_s)) << held;
    if (!held) {
      EXPECT_NEAR(with.settle_time_s, 0.25, kTolS);  // row 250
    }
    expect_same_other_fields(with, old);
    expect_same_other_fields(without, old);
  }
}

}  // namespace
}  // namespace cobot::bench
