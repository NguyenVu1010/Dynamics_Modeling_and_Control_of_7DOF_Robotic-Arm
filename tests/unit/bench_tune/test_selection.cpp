// The selection rule of auto-tune on synthetic tables (brief auto-tune, section 4; ADR-023):
// swing_decays with its noise floor, select_gain (admissible: no hold, saturation under 5 % of the
// run, every seed decays; among those the lowest median rms, an exact tie to the lower level),
// tune_reason (the one line for the report, with the number of levels failing each criterion) and
// tune_row (one row of the table from the RunSummary of each seed).
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "engine.hpp"
#include "tune.hpp"

namespace cobot::bench {
namespace {

const double kNan = std::numeric_limits<double>::quiet_NaN();

bool contains(const std::string& text, const std::string& part) {
  return text.find(part) != std::string::npos;
}

// An admissible row of 3 seeds.
TuneRow row(double level, double rms) {
  TuneRow r{};
  r.level = level;
  r.seeds = 3;
  r.held = 0;
  r.sat_frac_max = 0.01;
  r.decays_all = true;
  r.rms_joint_err_median = rms;
  r.final_pos_err_max = 1e-3;
  return r;
}

TuneRow held_row(double level, std::size_t held) {
  TuneRow r = row(level, 0.001);
  r.held = held;
  return r;
}

TuneRow saturated_row(double level, double sat_frac) {
  TuneRow r = row(level, 0.001);
  r.sat_frac_max = sat_frac;
  return r;
}

TuneRow growing_row(double level) {
  TuneRow r = row(level, 0.001);
  r.decays_all = false;
  return r;
}

std::optional<std::size_t> index(std::size_t i) {
  return i;
}

TEST(SwingDecays, TheFloorIsTheMeasuredNoise) {
  EXPECT_EQ(kSwingNoiseFloorRad, 5.0e-4);
}

TEST(SwingDecays, DecayingIsTrue) {
  EXPECT_TRUE(swing_decays(2.0e-3, 1.0e-3));
  EXPECT_TRUE(swing_decays(2.0e-3, 2.0e-3));  // last <= first
}

TEST(SwingDecays, GrowingAboveTheFloorIsFalse) {
  EXPECT_FALSE(swing_decays(1.0e-3, 2.0e-3));
  EXPECT_FALSE(swing_decays(1.0e-4, 5.1e-4));
}

TEST(SwingDecays, GrowingUnderTheFloorIsTrue) {
  EXPECT_TRUE(swing_decays(1.0e-4, 4.0e-4));
  EXPECT_TRUE(swing_decays(1.0e-4, 5.0e-4));  // last <= floor
}

TEST(SwingDecays, ANanSwingDoesNotDecay) {
  EXPECT_FALSE(swing_decays(kNan, kNan));
  EXPECT_FALSE(swing_decays(kNan, 1.0e-4));
  EXPECT_FALSE(swing_decays(1.0e-3, kNan));
}

TEST(SelectGain, OneAdmissible) {
  EXPECT_EQ(select_gain({held_row(20.0, 1), row(30.0, 0.01), growing_row(40.0)}), index(1));
}

TEST(SelectGain, LowestRmsWinsAmongSeveral) {
  EXPECT_EQ(select_gain({row(20.0, 0.03), row(30.0, 0.01), row(40.0, 0.02)}), index(1));
}

TEST(SelectGain, ALowerRmsOnAnInadmissibleLevelDoesNotWin) {
  TuneRow saturated = saturated_row(80.0, 0.2);
  saturated.rms_joint_err_median = 1e-6;
  EXPECT_EQ(select_gain({row(50.0, 0.01), saturated}), index(0));
}

TEST(SelectGain, ExactTieGoesToTheLowerLevel) {
  EXPECT_EQ(select_gain({row(50.0, 0.01), row(80.0, 0.01)}), index(0));
  // Whatever the order of the levels in the table.
  EXPECT_EQ(select_gain({row(80.0, 0.01), row(50.0, 0.01)}), index(1));
}

TEST(SelectGain, HeldExcludes) {
  EXPECT_EQ(select_gain({held_row(20.0, 1)}), std::nullopt);
  EXPECT_EQ(select_gain({held_row(20.0, 1), row(30.0, 0.5)}), index(1));
}

TEST(SelectGain, SaturationAtFivePercentExcludesAndJustBelowDoesNot) {
  EXPECT_EQ(select_gain({saturated_row(50.0, 0.05)}), std::nullopt);
  EXPECT_EQ(select_gain({saturated_row(50.0, 0.0499)}), index(0));
}

TEST(SelectGain, NanSaturationExcludes) {
  EXPECT_EQ(select_gain({saturated_row(50.0, kNan)}), std::nullopt);
}

TEST(SelectGain, NotDecayingExcludes) {
  EXPECT_EQ(select_gain({growing_row(50.0)}), std::nullopt);
}

TEST(SelectGain, NoneAdmissibleIsEmpty) {
  EXPECT_EQ(select_gain({held_row(20.0, 2), saturated_row(30.0, 0.5), growing_row(40.0)}),
            std::nullopt);
  EXPECT_EQ(select_gain({}), std::nullopt);
}

TEST(SelectGain, NanRmsIsNeverChosen) {
  EXPECT_EQ(select_gain({row(50.0, kNan)}), std::nullopt);
  EXPECT_EQ(select_gain({row(50.0, kNan), row(60.0, 0.5)}), index(1));
  EXPECT_EQ(select_gain({row(50.0, 0.5), row(60.0, kNan)}), index(0));
  EXPECT_EQ(select_gain({row(40.0, kNan), row(50.0, 0.5), row(60.0, kNan), row(70.0, 0.25)}),
            index(3));
}

TEST(TuneReason, NoneAdmissibleCountsTheLevelsFailingEachCriterion) {
  // Five levels; a level can fail several criteria, so the counts add up to more than five:
  //   20: held and not decaying (a held run has a NaN swing)
  //   30: held, saturated, not decaying
  //   40: saturated (exactly at the limit)
  //   50: not decaying
  //   60: held, not decaying
  // held 3 (20, 30, 60), saturated 2 (30, 40), not decaying 4 (20, 30, 50, 60).
  TuneRow l20 = growing_row(20.0);
  l20.held = 1;
  TuneRow l30 = saturated_row(30.0, 0.2);
  l30.held = 3;
  l30.decays_all = false;
  TuneRow l60 = growing_row(60.0);
  l60.held = 2;
  const std::vector<TuneRow> rows{l20, l30, saturated_row(40.0, 0.05), growing_row(50.0), l60};
  ASSERT_EQ(select_gain(rows), std::nullopt);
  const std::string reason = tune_reason(rows, std::nullopt);
  EXPECT_TRUE(contains(reason, "no admissible gain")) << reason;
  EXPECT_TRUE(contains(reason, "held 3")) << reason;
  EXPECT_TRUE(contains(reason, "saturated 2")) << reason;
  EXPECT_TRUE(contains(reason, "not decaying 4")) << reason;
  EXPECT_EQ(reason.find('\n'), std::string::npos) << "one line: " << reason;
}

TEST(TuneReason, AChosenLevelIsNamed) {
  const std::vector<TuneRow> rows{row(50.0, 0.01), saturated_row(80.0, 0.2)};
  const std::string reason = tune_reason(rows, select_gain(rows));
  EXPECT_FALSE(contains(reason, "no admissible gain")) << reason;
  EXPECT_TRUE(contains(reason, "50")) << reason;
  EXPECT_EQ(reason.find('\n'), std::string::npos) << "one line: " << reason;
}

// A run that did not hold, with a decaying swing, 12 s long.
RunSummary run_summary(double rms, double final_pos, double sat_time_s) {
  RunSummary s;
  s.rms_joint_err_rad = rms;
  s.final_pos_err_m = final_pos;
  s.sat_time_s = sat_time_s;
  s.run_time_s = 12.0;
  s.swing_first_rad = 2.0e-3;
  s.swing_last_rad = 1.0e-3;
  return s;
}

RunSummary held_summary(double rms, double final_pos, double sat_time_s) {
  RunSummary s = run_summary(rms, final_pos, sat_time_s);
  s.held = true;
  s.swing_first_rad = kNan;
  s.swing_last_rad = kNan;
  return s;
}

TEST(TuneRowOf, AggregatesTheSeeds) {
  // sat_time_s 0.12, 0.36, 0.24 of 12 s: fractions 0.01, 0.03, 0.02, max 0.03.
  // rms 0.03, 0.01, 0.02: median 0.02. final_pos max 3e-3.
  const TuneRow r = tune_row(60.0,
                             {run_summary(0.03, 1e-3, 0.12),
                              run_summary(0.01, 3e-3, 0.36),
                              run_summary(0.02, 2e-3, 0.24)});
  EXPECT_EQ(r.level, 60.0);
  EXPECT_EQ(r.seeds, 3U);
  EXPECT_EQ(r.held, 0U);
  EXPECT_DOUBLE_EQ(r.sat_frac_max, 0.03);
  EXPECT_TRUE(r.decays_all);
  EXPECT_EQ(r.rms_joint_err_median, 0.02);
  EXPECT_EQ(r.final_pos_err_max, 3e-3);
}

TEST(TuneRowOf, MedianOfAnEvenCountIsTheMeanOfTheMiddleTwo) {
  // rms 0.04, 0.01, 0.03, 0.02: sorted 0.01, 0.02, 0.03, 0.04; (0.02 + 0.03) / 2 = 0.025.
  const TuneRow r = tune_row(60.0,
                             {run_summary(0.04, 1e-3, 0.0),
                              run_summary(0.01, 1e-3, 0.0),
                              run_summary(0.03, 1e-3, 0.0),
                              run_summary(0.02, 1e-3, 0.0)});
  EXPECT_DOUBLE_EQ(r.rms_joint_err_median, 0.025);
}

TEST(TuneRowOf, ErrorColumnsAreOverTheSeedsThatDidNotHold) {
  // A held seed has huge errors and the largest saturation: it counts in held, sat_frac_max
  // (2.4 / 12 = 0.2) and decays_all, not in the error columns. rms 0.01, 0.03: median 0.02.
  const TuneRow r = tune_row(
      60.0,
      {run_summary(0.01, 1e-3, 0.0), held_summary(9.0, 9.0, 2.4), run_summary(0.03, 2e-3, 0.0)});
  EXPECT_EQ(r.seeds, 3U);
  EXPECT_EQ(r.held, 1U);
  EXPECT_DOUBLE_EQ(r.sat_frac_max, 0.2);
  EXPECT_FALSE(r.decays_all);
  EXPECT_DOUBLE_EQ(r.rms_joint_err_median, 0.02);
  EXPECT_EQ(r.final_pos_err_max, 2e-3);
}

TEST(TuneRowOf, AllSeedsHeldGivesNanErrorColumns) {
  const TuneRow r = tune_row(20.0, {held_summary(1.0, 1.0, 0.0), held_summary(2.0, 2.0, 0.0)});
  EXPECT_EQ(r.seeds, 2U);
  EXPECT_EQ(r.held, 2U);
  EXPECT_FALSE(r.decays_all);
  EXPECT_TRUE(std::isnan(r.rms_joint_err_median));
  EXPECT_TRUE(std::isnan(r.final_pos_err_max));
}

TEST(TuneRowOf, OneGrowingSeedFailsDecaysAll) {
  RunSummary growing = run_summary(0.01, 1e-3, 0.0);
  growing.swing_last_rad = 3.0e-3;  // above first 2e-3 and above the floor 5e-4
  const TuneRow r = tune_row(60.0, {run_summary(0.01, 1e-3, 0.0), growing});
  EXPECT_FALSE(r.decays_all);
  EXPECT_EQ(r.held, 0U);
}

}  // namespace
}  // namespace cobot::bench
