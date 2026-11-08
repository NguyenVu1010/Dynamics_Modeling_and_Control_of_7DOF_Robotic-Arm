// What the first nightly verify reports (run 42474837887, main 4c2ad89) showed: budget_xarm7 CT
// 0/100 printed a Wilson lower bound of 3.469e-18 instead of 0, and the bool and choice keys of
// the assumption table printed "scale 0 .. 0" as their bounds.
#include <cstddef>
#include <string>

#include <gtest/gtest.h>

#include "report.hpp"
#include "verify.hpp"

namespace cobot::bench {
namespace {

TEST(WilsonEnds, NoSuccessGivesExactlyZeroAndNoFailureExactlyOne) {
  for (const std::size_t n : {1U, 3U, 30U, 100U, 1000U}) {
    EXPECT_EQ(wilson(0, n).lo, 0.0) << n;
    EXPECT_EQ(wilson(n, n).hi, 1.0) << n;
    EXPECT_GT(wilson(0, n).hi, 0.0) << n;
    EXPECT_LT(wilson(n, n).lo, 1.0) << n;
  }
}

TEST(AssumptionBounds, AKeyWithNeitherRangeNorScaleShowsADash) {
  VerifyResult r;
  r.model = "m";
  r.task = "t";
  Assumption flag;
  flag.key = "sensor.qd_from_diff";
  flag.sensitivity = "not varied";
  Assumption ranged;
  ranged.key = "sensor.q_offset_std_rad";
  ranged.range = std::array<double, 2>{0.0005, 0.005};
  r.assumptions = {flag, ranged};
  const std::string md = report_md(r, {});
  EXPECT_NE(md.find("| sensor.qd_from_diff | assumed | - |"), std::string::npos) << md;
  EXPECT_EQ(md.find("scale 0 .. 0"), std::string::npos) << md;
  EXPECT_NE(md.find("range 0.0005 .. 0.005"), std::string::npos) << md;
}

}  // namespace
}  // namespace cobot::bench
