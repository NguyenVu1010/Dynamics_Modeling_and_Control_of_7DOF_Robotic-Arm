// Wilson interval, the three-level verdict and the per-run pass (brief verify, sections 2 and 3;
// ADR-023 "Kết luận ba mức từ Monte Carlo"; plan ambiguities 13 and 14, owner questions C11, C14).
//
// Wilson score interval, z = 1.959963984540054, z^2 = 3.841458820694124:
//   centre = (p + z^2 / 2n) / (1 + z^2 / n), half = z sqrt(p (1 - p) / n + z^2 / 4n^2) / (1 + z^2 /
//   n) all n passing: lo = n / (n + z^2), hi = 1.
// Hand values (n = 100: z^2 / n = 0.03841459, 1 + z^2 / n = 1.03841459, z^2 / 2n = 0.01920729,
// z^2 / 4n^2 = 9.6036e-5):
//   100/100: lo = 100 / 103.84146 = 0.963015
//   30/30:   lo = 30 / 33.84146 = 0.886488
//   3/3:     lo = 3 / 6.84146 = 0.438503            (0.44 to 2 decimals)
//   99/100:  centre num 0.99 + 0.0192073 = 1.0092073, sqrt(9.9e-5 + 9.6036e-5) = 0.0139655,
//            z * 0.0139655 = 0.0273719, lo = (1.0092073 - 0.0273719) / 1.0384146 = 0.945510
//   91/100:  centre num 0.9292073, sqrt(8.19e-4 + 9.6036e-5) = 0.0302496, z * = 0.0592882,
//            hi = (0.9292073 + 0.0592882) / 1.0384146 = 0.951929
//   90/100:  centre num 0.9192073, sqrt(9.0e-4 + 9.6036e-5) = 0.0315600, z * = 0.0618565,
//            hi = 0.9810638 / 1.0384146 = 0.944771 < 0.95
//   5/10:    z^2 / n = 0.3841459, centre num 0.5 + 0.1920729 = 0.6920729,
//            sqrt(0.025 + 0.0096036) = 0.1860204, z * = 0.3645933, hi = 1.0566662 / 1.3841459 =
//            0.763407
//   20/100:  centre num 0.2192073, sqrt(1.6e-3 + 9.6036e-5) = 0.0411830, z * = 0.0807172,
//            lo = 0.1384901 / 1.0384146 = 0.133367
//   6/100:   centre num 0.0792073, sqrt(5.64e-4 + 9.6036e-5) = 0.0256912, z * = 0.0503539,
//            lo = 0.0288534 / 1.0384146 = 0.027786
//   5/100:   centre num 0.0692073, sqrt(4.75e-4 + 9.6036e-5) = 0.0238964, z * = 0.0468360,
//            lo = 0.0223713 / 1.0384146 = 0.021544
//   94/100:  centre num 0.9592073, z * 0.0256912 = 0.0503539, lo = 0.9088534 / 1.0384146 = 0.875232
//   95/100:  centre num 0.9692073, z * 0.0238964 = 0.0468360, lo = 0.9223713 / 1.0384146 = 0.888252
//   80/100:  lo = (0.8192073 - 0.0807172) / 1.0384146 = 0.711178
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/pipeline/task.hpp"

#include "engine.hpp"
#include "verify.hpp"

namespace cobot::bench {
namespace {

constexpr double kZ = 1.959963984540054;
constexpr double kPi = 3.14159265358979323846;
const double kNan = std::numeric_limits<double>::quiet_NaN();

// ---- wilson ---------------------------------------------------------------------------------

TEST(Wilson, AllPassingLowerBoundsAreTheHandValues) {
  EXPECT_NEAR(wilson(100, 100).lo, 0.963, 5e-4);
  EXPECT_NEAR(wilson(30, 30).lo, 0.886, 5e-4);
  EXPECT_NEAR(wilson(3, 3).lo, 0.44, 5e-3);
  for (const std::size_t n : {std::size_t{3}, std::size_t{30}, std::size_t{73}, std::size_t{100}}) {
    const auto nd = static_cast<double>(n);
    EXPECT_NEAR(wilson(n, n).lo, nd / (nd + kZ * kZ), 1e-12) << n;
    EXPECT_NEAR(wilson(n, n).hi, 1.0, 1e-12) << n;
  }
  // 73 / (73 + 3.8414588) = 0.950015: the smallest all-passing n whose bound reaches 95 %.
  EXPECT_GE(wilson(73, 73).lo, 0.95);
  EXPECT_LT(wilson(72, 72).lo, 0.95);  // 72 / 75.8414588 = 0.949348
}

TEST(Wilson, OneFailureInAHundredAndTheFailBoundary) {
  EXPECT_NEAR(wilson(99, 100).lo, 0.9455, 5e-5);
  EXPECT_NEAR(wilson(91, 100).hi, 0.952, 5e-4);
  EXPECT_NEAR(wilson(90, 100).hi, 0.944771, 5e-6);
  EXPECT_NEAR(wilson(5, 10).hi, 0.763407, 5e-6);
  EXPECT_NEAR(wilson(20, 100).lo, 0.133367, 5e-6);
}

TEST(Wilson, TheIntervalHoldsThePointEstimate) {
  for (const std::size_t k : {std::size_t{0}, std::size_t{1}, std::size_t{50}, std::size_t{99}}) {
    const Interval w = wilson(k, 100);
    const double p = static_cast<double>(k) / 100.0;
    EXPECT_LE(w.lo, p + 1e-12) << k;  // 0/n: lo is 0 up to rounding
    EXPECT_GE(w.hi, p - 1e-12) << k;
    EXPECT_GE(w.lo, -1e-12) << k;  // 0 up to rounding
    EXPECT_LE(w.hi, 1.0 + 1e-12) << k;
  }
  EXPECT_NEAR(wilson(0, 100).lo, 0.0, 1e-12);
}

TEST(Wilson, NoRunsIsTheWholeRange) {
  const Interval w = wilson(0, 0);
  EXPECT_EQ(w.lo, 0.0);
  EXPECT_EQ(w.hi, 1.0);
}

TEST(Wilson, TheDefaultZIsNinetyFivePercentTwoSided) {
  const Interval a = wilson(42, 77);
  const Interval b = wilson(42, 77, kZ);
  EXPECT_EQ(a.lo, b.lo);
  EXPECT_EQ(a.hi, b.hi);
  // A wider z widens the interval.
  const Interval c = wilson(42, 77, 3.0);
  EXPECT_LT(c.lo, a.lo);
  EXPECT_GT(c.hi, a.hi);
}

// ---- verdict ----------------------------------------------------------------------------------

pipeline::TaskRequirements requirements(double target = 0.95, double max_hold = 0.05) {
  pipeline::TaskRequirements r;
  r.final_pos_err_m = 0.005;
  r.final_rot_err_deg = 2.0;
  r.rms_joint_err_rad = 0.02;
  r.cycle_time_s = 12.0;
  r.max_hold_rate = max_hold;
  r.target_pass_rate = target;
  r.margin_frac = 0.2;
  return r;
}

// Worst metrics in the report's order, each at `frac` of its threshold; limit = 0.8 threshold.
std::vector<WorstMetric> worst_at(double frac) {
  const pipeline::TaskRequirements r = requirements();
  const double m = 1.0 - r.margin_frac;
  return {
      {"final_pos_err_m", frac * r.final_pos_err_m, r.final_pos_err_m, m * r.final_pos_err_m},
      {"final_rot_err_deg",
       frac * r.final_rot_err_deg,
       r.final_rot_err_deg,
       m * r.final_rot_err_deg},
      {"rms_joint_err_rad",
       frac * r.rms_joint_err_rad,
       r.rms_joint_err_rad,
       m * r.rms_joint_err_rad},
      {"cycle_time_s", frac * r.cycle_time_s, r.cycle_time_s, m * r.cycle_time_s},
  };
}

Counts counts(std::size_t runs, std::size_t passed, std::size_t held) {
  return Counts{runs, passed, held};
}

TEST(Verdict, AllHundredPassingWellInsideTheMarginIsPass) {
  EXPECT_TRUE(verdict(counts(100, 100, 0), worst_at(0.2), requirements(), false) == Verdict::kPass);
}

TEST(Verdict, QuickModeNeverPasses) {
  EXPECT_TRUE(verdict(counts(100, 100, 0), worst_at(0.2), requirements(), true) ==
              Verdict::kMarginal);
  // Fewer than 100 runs is quick mode too: 99/99 has lo = 99 / 102.84 = 0.9626 >= 0.95.
  EXPECT_TRUE(verdict(counts(99, 99, 0), worst_at(0.2), requirements(), false) ==
              Verdict::kMarginal);
}

TEST(Verdict, OneFailureInAHundredIsMarginal) {
  // lo 0.9455 < 0.95 <= hi.
  EXPECT_TRUE(verdict(counts(100, 99, 0), worst_at(0.2), requirements(), false) ==
              Verdict::kMarginal);
}

TEST(Verdict, NinetyOneIsMarginalAndNinetyIsFail) {
  // 91/100: hi 0.952 >= 0.95, not FAIL; 90/100: hi 0.9448 < 0.95.
  EXPECT_TRUE(verdict(counts(100, 91, 0), worst_at(0.2), requirements(), false) ==
              Verdict::kMarginal);
  EXPECT_TRUE(verdict(counts(100, 90, 0), worst_at(0.2), requirements(), false) == Verdict::kFail);
}

TEST(Verdict, AWorstValueBetweenLimitAndThresholdBlocksPass) {
  for (std::size_t i = 0; i < 4; ++i) {
    std::vector<WorstMetric> worst = worst_at(0.2);
    worst[i].value = 0.9 * worst[i].threshold;  // above 0.8 threshold, under the threshold
    EXPECT_TRUE(verdict(counts(100, 100, 0), worst, requirements(), false) == Verdict::kMarginal)
        << worst[i].name;
  }
  // At the limit exactly is still PASS (value <= margin_limit).
  std::vector<WorstMetric> at_limit = worst_at(0.2);
  for (WorstMetric& w : at_limit) {
    w.value = w.margin_limit;
  }
  EXPECT_TRUE(verdict(counts(100, 100, 0), at_limit, requirements(), false) == Verdict::kPass);
}

TEST(Verdict, QuickModeStillFails) {
  // 5/10: hi 0.763 < 0.95.
  EXPECT_TRUE(verdict(counts(10, 5, 0), worst_at(0.2), requirements(), true) == Verdict::kFail);
  // Every run held (0/3): hi = z^2 / (3 + z^2) = 0.5615 < 0.95; the worst values are NaN.
  EXPECT_TRUE(verdict(counts(3, 0, 3), worst_at(kNan), requirements(), true) == Verdict::kFail);
}

TEST(Verdict, AProvableHoldRateFailsEvenWhenThePassRateWouldNot) {
  // Target 0.5 so that the pass rate alone is not FAIL: 80/100 has lo 0.711 >= 0.5. Holds 20/100:
  // Wilson lo 0.133 > max_hold_rate 0.05 -> FAIL (without the rule it would be MARGINAL, the point
  // estimate 0.2 > 0.05 only blocking PASS).
  EXPECT_TRUE(verdict(counts(100, 80, 20), worst_at(0.2), requirements(0.5, 0.05), false) ==
              Verdict::kFail);
}

TEST(Verdict, AHoldPointEstimateAboveTheRateBlocksPassOnly) {
  // Holds 6/100: Wilson lo 0.0278 <= 0.05 (no FAIL), point 0.06 > 0.05 (no PASS); 94/100 passes
  // with lo >= 0.5.
  EXPECT_TRUE(verdict(counts(100, 94, 6), worst_at(0.2), requirements(0.5, 0.05), false) ==
              Verdict::kMarginal);
  // Holds 5/100: point 0.05 <= 0.05, lo 0.0215; 95/100 lo 0.888 >= 0.5 -> PASS.
  EXPECT_TRUE(verdict(counts(100, 95, 5), worst_at(0.2), requirements(0.5, 0.05), false) ==
              Verdict::kPass);
}

TEST(Verdict, ANanWorstValueNeverPasses) {
  for (std::size_t i = 0; i < 4; ++i) {
    std::vector<WorstMetric> worst = worst_at(0.2);
    worst[i].value = kNan;
    EXPECT_TRUE(verdict(counts(100, 100, 0), worst, requirements(), false) == Verdict::kMarginal)
        << worst[i].name;
  }
}

// ---- run_passes ---------------------------------------------------------------------------------

RunSummary good_run() {
  RunSummary s;
  s.final_pos_err_m = 1e-4;
  s.final_rot_err_rad = 1e-3;
  s.rms_joint_err_rad = 1e-3;
  s.settle_time_s = 10.5;
  s.held = false;
  return s;
}

// final_rot_err_rad of a rotation of `deg` degrees, nudged by `factor`: the deg/rad conversion
// may round either way, so "at the threshold" is taken one part in 1e12 inside it.
double rad_of(double deg, double factor) {
  return deg * kPi / 180.0 * factor;
}

TEST(RunPasses, AGoodRunPasses) {
  EXPECT_TRUE(run_passes(good_run(), requirements()));
}

TEST(RunPasses, EachMetricAtItsThresholdPasses) {
  const pipeline::TaskRequirements r = requirements();
  RunSummary s = good_run();
  s.final_pos_err_m = r.final_pos_err_m;
  s.final_rot_err_rad = rad_of(r.final_rot_err_deg, 1.0 - 1e-12);
  s.rms_joint_err_rad = r.rms_joint_err_rad;
  s.settle_time_s = r.cycle_time_s;
  EXPECT_TRUE(run_passes(s, r));
}

TEST(RunPasses, EachMetricJustAboveItsThresholdFails) {
  const pipeline::TaskRequirements r = requirements();
  RunSummary pos = good_run();
  pos.final_pos_err_m = r.final_pos_err_m * (1.0 + 1e-9);
  EXPECT_FALSE(run_passes(pos, r));
  RunSummary rot = good_run();
  rot.final_rot_err_rad = rad_of(r.final_rot_err_deg, 1.0 + 1e-9);
  EXPECT_FALSE(run_passes(rot, r));
  RunSummary rms = good_run();
  rms.rms_joint_err_rad = r.rms_joint_err_rad * (1.0 + 1e-9);
  EXPECT_FALSE(run_passes(rms, r));
  RunSummary settle = good_run();
  settle.settle_time_s = r.cycle_time_s * (1.0 + 1e-9);
  EXPECT_FALSE(run_passes(settle, r));
}

TEST(RunPasses, TheRotationIsJudgedInDegrees) {
  // 1.5 deg = 0.02618 rad passes a 2 deg threshold; 0.05 rad = 2.865 deg does not (a threshold
  // read as radians would let both pass).
  const pipeline::TaskRequirements r = requirements();
  RunSummary s = good_run();
  s.final_rot_err_rad = rad_of(1.5, 1.0);
  EXPECT_TRUE(run_passes(s, r));
  s.final_rot_err_rad = 0.05;
  EXPECT_FALSE(run_passes(s, r));
}

TEST(RunPasses, NanFails) {
  const pipeline::TaskRequirements r = requirements();
  RunSummary pos = good_run();
  pos.final_pos_err_m = kNan;
  EXPECT_FALSE(run_passes(pos, r));
  RunSummary rot = good_run();
  rot.final_rot_err_rad = kNan;
  EXPECT_FALSE(run_passes(rot, r));
  RunSummary rms = good_run();
  rms.rms_joint_err_rad = kNan;
  EXPECT_FALSE(run_passes(rms, r));
  RunSummary settle = good_run();
  settle.settle_time_s = kNan;  // never settled
  EXPECT_FALSE(run_passes(settle, r));
}

TEST(RunPasses, AHeldRunFails) {
  RunSummary s = good_run();
  s.held = true;
  EXPECT_FALSE(run_passes(s, requirements()));
}

TEST(RunSummaryDefaults, SettleTimeIsNanUntilMeasured) {
  EXPECT_TRUE(std::isnan(RunSummary{}.settle_time_s));
}

}  // namespace
}  // namespace cobot::bench
