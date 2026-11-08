// The sensitivity rule of one assumption (brief verify-analysis, section 1; plan ambiguity 16,
// owner question C12), a pure function: "may change" when at either end (the key at its low or
// its high value, sensitivity_seeds seeds) the pass rate is on the other side of
// target_pass_rate than the main Monte Carlo's pass rate, or when holds appear at an end where
// the base (the same seeds of the main Monte Carlo) had none; else "stable". A key without range
// or scale (bool, choice) is "not varied". A rate equal to the target is on the side at or above
// it (the verdict's PASS clause uses >=).
//
// Target 0.95 (reference_move). Rates by hand: 100/100 = 1, 80/100 = 0.8, 20/20 = 1,
// 19/20 = 0.95 (exactly the double of 0.95: both are the correctly rounded value of 19/20),
// 18/20 = 0.9, 15/20 = 0.75, 10/20 = 0.5.
#include <array>
#include <optional>
#include <string>

#include <gtest/gtest.h>

#include "analysis.hpp"
#include "verify.hpp"

namespace cobot::bench {
namespace {

constexpr double kTarget = 0.95;

Counts counts(std::size_t runs, std::size_t passed, std::size_t held) {
  Counts c;
  c.runs = runs;
  c.passed = passed;
  c.held = held;
  return c;
}

std::optional<std::array<Counts, 2>> ends(Counts low, Counts high) {
  return std::array<Counts, 2>{low, high};
}

TEST(SensitivityRule, NamesAreTheReportTexts) {
  EXPECT_STREQ(to_string(Sensitivity::kStable), "stable");
  EXPECT_STREQ(to_string(Sensitivity::kMayChange), "may change");
  EXPECT_STREQ(to_string(Sensitivity::kNotVaried), "not varied");
}

TEST(SensitivityRule, BothEndsOnTheSameSideAboveIsStable) {
  // main 1 >= 0.95; low 1 and high 0.95 both >= 0.95.
  const Counts main = counts(100, 100, 0);
  const Counts base = counts(20, 20, 0);
  EXPECT_EQ(sensitivity_rule(main, base, ends(counts(20, 20, 0), counts(20, 20, 0)), kTarget),
            Sensitivity::kStable);
  EXPECT_EQ(sensitivity_rule(main, base, ends(counts(20, 20, 0), counts(20, 19, 0)), kTarget),
            Sensitivity::kStable);
}

TEST(SensitivityRule, BothEndsOnTheSameSideBelowIsStable) {
  // main 0.8 < 0.95; low 0.75 and high 0.5 both < 0.95: the verdict side does not move.
  const Counts main = counts(100, 80, 0);
  const Counts base = counts(20, 16, 0);
  EXPECT_EQ(sensitivity_rule(main, base, ends(counts(20, 15, 0), counts(20, 10, 0)), kTarget),
            Sensitivity::kStable);
}

TEST(SensitivityRule, OneEndBelowTheTargetMayChange) {
  // main 1 >= 0.95, one end 0.9 < 0.95; either end alone is enough.
  const Counts main = counts(100, 100, 0);
  const Counts base = counts(20, 20, 0);
  EXPECT_EQ(sensitivity_rule(main, base, ends(counts(20, 18, 0), counts(20, 20, 0)), kTarget),
            Sensitivity::kMayChange);
  EXPECT_EQ(sensitivity_rule(main, base, ends(counts(20, 20, 0), counts(20, 18, 0)), kTarget),
            Sensitivity::kMayChange);
}

TEST(SensitivityRule, OneEndAboveTheTargetFromBelowMayChange) {
  // main 0.8 < 0.95, high 1 >= 0.95: crossing upwards counts too.
  const Counts main = counts(100, 80, 0);
  const Counts base = counts(20, 16, 0);
  EXPECT_EQ(sensitivity_rule(main, base, ends(counts(20, 15, 0), counts(20, 20, 0)), kTarget),
            Sensitivity::kMayChange);
  // 0.95 itself is on the side at or above the target.
  EXPECT_EQ(sensitivity_rule(main, base, ends(counts(20, 19, 0), counts(20, 15, 0)), kTarget),
            Sensitivity::kMayChange);
}

TEST(SensitivityRule, HoldsAppearingFromZeroMayChange) {
  // Pass rates all on the same side (main 1, low 0.95, high 1), but the low end holds once where
  // the base held never.
  const Counts main = counts(100, 100, 0);
  const Counts base = counts(20, 20, 0);
  EXPECT_EQ(sensitivity_rule(main, base, ends(counts(20, 19, 1), counts(20, 20, 0)), kTarget),
            Sensitivity::kMayChange);
  EXPECT_EQ(sensitivity_rule(main, base, ends(counts(20, 20, 0), counts(20, 19, 1)), kTarget),
            Sensitivity::kMayChange);
  // Below the target on both sides, holds appearing still count.
  const Counts low_main = counts(100, 50, 0);
  const Counts low_base = counts(20, 10, 0);
  EXPECT_EQ(
      sensitivity_rule(low_main, low_base, ends(counts(20, 10, 2), counts(20, 10, 0)), kTarget),
      Sensitivity::kMayChange);
}

TEST(SensitivityRule, HoldsAtTheBaseAndAtAnEndGoByThePassRateOnly) {
  // main 0.8 with holds, base held 2: an end with held 5 is not "holds appearing".
  const Counts main = counts(100, 80, 10);
  const Counts base = counts(20, 16, 2);
  EXPECT_EQ(sensitivity_rule(main, base, ends(counts(20, 15, 5), counts(20, 16, 1)), kTarget),
            Sensitivity::kStable);
  // The same holds with an end crossing the target (high 1 >= 0.95): may change.
  EXPECT_EQ(sensitivity_rule(main, base, ends(counts(20, 15, 5), counts(20, 20, 0)), kTarget),
            Sensitivity::kMayChange);
}

TEST(SensitivityRule, AKeyWithoutEndsIsNotVaried) {
  // A bool or choice key has no range or scale: nothing ran, whatever the counts.
  const Counts main = counts(100, 100, 0);
  const Counts base = counts(20, 20, 0);
  EXPECT_EQ(sensitivity_rule(main, base, std::nullopt, kTarget), Sensitivity::kNotVaried);
  EXPECT_EQ(sensitivity_rule(counts(100, 10, 50), counts(20, 2, 10), std::nullopt, kTarget),
            Sensitivity::kNotVaried);
}

TEST(SensitivityRule, TheTargetIsTheTasks) {
  // main 0.8 and ends 0.75 / 0.75: stable at 0.95, may change at 0.78 (0.8 >= 0.78 > 0.75).
  const Counts main = counts(100, 80, 0);
  const Counts base = counts(20, 16, 0);
  const auto e = ends(counts(20, 15, 0), counts(20, 15, 0));
  EXPECT_EQ(sensitivity_rule(main, base, e, 0.95), Sensitivity::kStable);
  EXPECT_EQ(sensitivity_rule(main, base, e, 0.78), Sensitivity::kMayChange);
}

}  // namespace
}  // namespace cobot::bench
