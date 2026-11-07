// RunSummary fields of auto-tune on synthetic rows (brief auto-tune, section 3): run_time_s and the
// post-goal swing, the largest peak-to-peak of q over the joints in the first and the last 0.5 s of
// the hold (its two halves when the hold is under 1 s; NaN for a run that held or has no hold
// rows). q_ref is constant in these rows, so q and q - q_ref (tests/unit/loop_drive_profiles) have
// the same peak-to-peak. A drive run rates `taum` against P, not the feed-forward `tau`; the old
// fields and the Summary CSV do not change.
//
// Rows every 1 ms from t = 0. Rows within kGuardRows of a window boundary stay at the base pose, so
// the windows are pinned without pinning which side a boundary row falls on.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <sstream>
#include <vector>

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

constexpr std::int64_t kRowNs = 1000000;  // 1 ms
constexpr double kRowS = 1.0e-3;          // s
constexpr std::size_t kGoalRow = 1000;    // goal at t = 1 s
constexpr std::int64_t kGoalNs = 1000000000;
constexpr std::size_t kGuardRows = 3;
constexpr double kTolRad = 1e-12;
constexpr double kTauMaxNm = 100.0;
constexpr double kRatedNm = 50.0;
constexpr double kBaseRad = 0.5;  // q and q_ref at rest

// Row k: tracking before the goal, finished after, q = q_ref = base, no torque, no error.
std::vector<Row> make_rows(std::size_t count) {
  std::vector<Row> rows(count);
  for (std::size_t k = 0; k < count; ++k) {
    rows[k].t.ns = static_cast<std::int64_t>(k) * kRowNs;
    rows[k].mode = k < kGoalRow ? LoopMode::kTracking : LoopMode::kFinished;
    rows[k].q = Vec::Constant(kBaseRad);
    rows[k].q_ref = Vec::Constant(kBaseRad);
    rows[k].tau = Vec::Zero();
    rows[k].tau_motor = Vec::Zero();
  }
  return rows;
}

// Joint j swings +amp, -amp, +amp, ... around the base in rows [begin + guard, end - guard):
// peak-to-peak 2 amp.
void swing(std::vector<Row>& rows, int j, std::size_t begin, std::size_t end, double amp) {
  for (std::size_t k = begin + kGuardRows; k + kGuardRows < end; ++k) {
    rows[k].q(j) = kBaseRad + (k % 2 == 0 ? amp : -amp);
  }
}

RunSummary summarize_rows(const std::vector<Row>& rows,
                          TorqueSource torque = TorqueSource::kCommand) {
  const std::int64_t t_end = static_cast<std::int64_t>(rows.size()) * kRowNs;
  return summarize(rows, Vec::Constant(kTauMaxNm), Vec::Constant(kRatedNm), t_end, torque, kGoalNs);
}

TEST(SummarySwing, RunTimeIsTheSimulatedTime) {
  // 2000 rows of 1 ms: 2 s, whether counted to the end of the last cycle or to its row.
  EXPECT_NEAR(summarize_rows(make_rows(2000)).run_time_s, 2.0, 1.01 * kRowS);
}

TEST(SummarySwing, FirstAndLastHalfSecondOfAOneSecondHold) {
  std::vector<Row> rows = make_rows(2000);  // hold [1, 2) s
  // Before the goal joint 0 moves about 1 rad: not part of any window.
  for (std::size_t k = 0; k + 10 < kGoalRow; ++k) {
    rows[k].q(0) = kBaseRad + static_cast<double>(k) * kRowS;
  }
  // First window [1, 1.5) s: joint 3 p-p 2 * 2e-3 = 4e-3 (joint 1: 2 * 1e-3 = 2e-3).
  swing(rows, 3, 1000, 1500, 2.0e-3);
  swing(rows, 1, 1000, 1500, 1.0e-3);
  // Last window [1.5, 2) s: joint 1 p-p 2 * 3e-3 = 6e-3 (joint 3: 2 * 5e-4 = 1e-3).
  swing(rows, 1, 1500, 2000, 3.0e-3);
  swing(rows, 3, 1500, 2000, 5.0e-4);
  const RunSummary s = summarize_rows(rows);
  EXPECT_NEAR(s.swing_first_rad, 4.0e-3, kTolRad);
  EXPECT_NEAR(s.swing_last_rad, 6.0e-3, kTolRad);
  EXPECT_FALSE(s.held);
}

TEST(SummarySwing, TheMiddleOfALongHoldIsInNoWindow) {
  std::vector<Row> rows = make_rows(4000);  // hold [1, 4) s
  swing(rows, 2, 1000, 1500, 1.0e-3);       // first [1, 1.5) s: p-p 2e-3
  swing(rows, 2, 2000, 3000, 0.25);         // [2, 3) s: p-p 0.5 rad, outside both windows
  swing(rows, 2, 3500, 4000, 2.5e-4);       // last [3.5, 4) s: p-p 5e-4
  const RunSummary s = summarize_rows(rows);
  EXPECT_NEAR(s.swing_first_rad, 2.0e-3, kTolRad);
  EXPECT_NEAR(s.swing_last_rad, 5.0e-4, kTolRad);
}

TEST(SummarySwing, AHoldUnderOneSecondUsesItsTwoHalves) {
  // Hold [1, 1.6) s: halves [1, 1.3) and [1.3, 1.6) s. Windows of 0.5 s would overlap, [1, 1.5) and
  // [1.1, 1.6), and would mix the amplitudes in one or the other case below.
  std::vector<Row> growing = make_rows(1600);
  swing(growing, 4, 1000, 1300, 1.0e-3);  // p-p 2e-3
  swing(growing, 4, 1300, 1600, 2.0e-3);  // p-p 4e-3
  const RunSummary g = summarize_rows(growing);
  EXPECT_NEAR(g.swing_first_rad, 2.0e-3, kTolRad);
  EXPECT_NEAR(g.swing_last_rad, 4.0e-3, kTolRad);

  std::vector<Row> decaying = make_rows(1600);
  swing(decaying, 4, 1000, 1300, 2.0e-3);  // p-p 4e-3
  swing(decaying, 4, 1300, 1600, 1.0e-3);  // p-p 2e-3
  const RunSummary d = summarize_rows(decaying);
  EXPECT_NEAR(d.swing_first_rad, 4.0e-3, kTolRad);
  EXPECT_NEAR(d.swing_last_rad, 2.0e-3, kTolRad);
}

TEST(SummarySwing, NoHoldRowsGiveNan) {
  const RunSummary s = summarize_rows(make_rows(kGoalRow));  // last row t = 0.999 s < goal
  EXPECT_TRUE(std::isnan(s.swing_first_rad));
  EXPECT_TRUE(std::isnan(s.swing_last_rad));
}

TEST(SummarySwing, ARunThatHeldGivesNan) {
  for (const std::size_t first_held : {std::size_t{500}, std::size_t{1700}}) {
    std::vector<Row> rows = make_rows(2000);
    swing(rows, 3, 1000, 1500, 2.0e-3);
    for (std::size_t k = first_held; k < rows.size(); ++k) {
      rows[k].mode = LoopMode::kHolding;
    }
    const RunSummary s = summarize_rows(rows);
    EXPECT_TRUE(s.held) << first_held;
    EXPECT_TRUE(std::isnan(s.swing_first_rad)) << first_held;
    EXPECT_TRUE(std::isnan(s.swing_last_rad)) << first_held;
  }
}

// Joint 0 at the peak torque in rows [100, 200): 100 cycles of 1 ms.
void saturate(std::vector<Row>& rows, Vec Row::*field) {
  for (std::size_t k = 100; k < 200; ++k) {
    (rows[k].*field)(0) = kTauMaxNm;
  }
}

TEST(SummarySaturation, ADriveRunCountsTheMotorTorque) {
  std::vector<Row> rows = make_rows(2000);
  saturate(rows, &Row::tau_motor);
  EXPECT_NEAR(summarize_rows(rows, TorqueSource::kMotor).sat_time_s, 0.1, 1e-12);  // 100 * 1 ms
  EXPECT_EQ(summarize_rows(rows, TorqueSource::kCommand).sat_time_s, 0.0);
}

TEST(SummarySaturation, ADriveRunIgnoresTheFeedForward) {
  std::vector<Row> rows = make_rows(2000);
  saturate(rows, &Row::tau);
  EXPECT_EQ(summarize_rows(rows, TorqueSource::kMotor).sat_time_s, 0.0);
  EXPECT_NEAR(summarize_rows(rows, TorqueSource::kCommand).sat_time_s, 0.1, 1e-12);
}

TEST(SummaryOldFields, ComputedTorqueValuesAreUnchanged) {
  std::vector<Row> rows = make_rows(2000);
  saturate(rows, &Row::tau);
  for (Row& r : rows) {
    r.q_ref(2) = r.q(2) + 0.01;  // joint 2: constant error 0.01 rad, RMS 0.01
    r.T_b_e.p = common::Vec3(0.003, 0.0, 0.0);
  }
  // Joint 3: |error| 2e-3 over part of the rows, RMS below joint 2's.
  swing(rows, 3, 1000, 1500, 2.0e-3);
  const std::int64_t t_end = static_cast<std::int64_t>(rows.size()) * kRowNs;
  const Vec tau_max = Vec::Constant(kTauMaxNm);
  const Vec rated = Vec::Constant(kRatedNm);
  const RunSummary with_goal =
      summarize(rows, tau_max, rated, t_end, TorqueSource::kCommand, kGoalNs);
  const RunSummary old = summarize(rows, tau_max, rated, t_end);
  for (const RunSummary* s : {&with_goal, &old}) {
    EXPECT_NEAR(s->final_pos_err_m, 0.003, 1e-15);
    EXPECT_EQ(s->final_rot_err_rad, 0.0);
    EXPECT_NEAR(s->rms_joint_err_rad, 0.01, 1e-12);
    EXPECT_EQ(s->peak_tau_ratio, 1.0);
    EXPECT_NEAR(s->sat_time_s, 0.1, 1e-12);
    // RMS(tau) of joint 0 = sqrt(100 * 100^2 / 2000) = sqrt(500); over the rated 50 N m.
    EXPECT_NEAR(s->rms_tau_ratio, std::sqrt(500.0) / kRatedNm, 1e-12);
    EXPECT_FALSE(s->held);
    EXPECT_EQ(s->hold_time_s, 0.0);
    EXPECT_TRUE(s->hold_reason == pipeline::HoldReason::kNone);
  }
}

TEST(SummaryCsv, NoNewColumn) {
  std::ostringstream header;
  write_summary_header(header);
  EXPECT_EQ(header.str(),
            "run_id,param,level,seed,final_pos_err_m,final_rot_err_rad,rms_joint_err_rad,"
            "peak_tau_ratio,sat_time_s,rms_tau_ratio,held,hold_time_s,hold_reason,passed\n");
  SummaryRow row;
  row.param = "none";
  row.summary = summarize_rows(make_rows(2000));
  std::ostringstream line;
  write_summary_row(line, row);
  std::size_t commas = 0;
  for (const char c : line.str()) {
    commas += c == ',' ? 1 : 0;
  }
  EXPECT_EQ(commas, 13U) << line.str();  // 14 columns
}

}  // namespace
}  // namespace cobot::bench
