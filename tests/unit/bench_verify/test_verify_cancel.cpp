// Cancel of bench::verify (brief verify, section 4), the cases the review of #81 listed: a cancel
// set before the start needs no run, so it is checked in every build; a cancel during the tune step
// stops the tune between its runs and no Monte Carlo run starts (release builds only).
#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/control/algorithm.hpp"

#include "verify.hpp"
#include "verify_test_util.hpp"

namespace cobot::bench {
namespace {

using control::Algorithm;
using namespace verify_test;  // NOLINT(google-build-using-namespace)

VerifyOptions tuned(std::size_t runs,
                    std::vector<double> levels,
                    std::vector<std::uint64_t> tune_seeds,
                    unsigned jobs) {
  VerifyOptions o;
  o.runs = runs;
  o.seed0 = 1;
  o.quick = true;
  o.tune = true;
  o.tuning.levels = std::move(levels);
  o.tuning.seeds = std::move(tune_seeds);
  o.tuning.jobs = jobs;
  o.jobs = jobs;
  return o;
}

TEST(VerifyCancel, SetBeforeTheStartIsAnErrorInEveryBuild) {
  const std::atomic<bool> cancel{true};
  VerifyOptions o = tuned(5, {50.0}, {1}, 2);
  o.cancel = &cancel;
  std::atomic<std::size_t> finished{0};
  o.progress = [&finished](std::string_view, std::size_t done, std::size_t) {
    finished += done > 0 ? 1 : 0;
  };
  const auto r =
      verify(load_model("premium_fr3"), Algorithm::kComputedTorque, load_task("reference_move"), o);
  ASSERT_FALSE(r.is_ok());
  EXPECT_NE(r.reason().find("cancelled"), std::string::npos) << r.reason();
  EXPECT_EQ(finished.load(), 0U);
}

// One job, 2 levels x 2 tune seeds = 4 tune runs: the flag set after the first tune run is seen
// before the next one starts, and the Monte Carlo phase never reports.
TEST(VerifyCancel, DuringTheTuneStopsTheTuneAndNoMonteCarloRunStarts) {
  if (!kOptimised) {
    GTEST_SKIP() << "elastic 7-DOF closed loop: release builds only";
  }
  std::atomic<bool> cancel{false};
  VerifyOptions o = tuned(5, {40.0, 50.0}, {1, 2}, 1);
  o.cancel = &cancel;
  std::size_t tune_done = 0;
  std::size_t other_done = 0;
  o.progress = [&](std::string_view phase, std::size_t done, std::size_t) {
    if (phase == "tune") {
      tune_done = std::max(tune_done, done);
      if (done >= 1) {
        cancel = true;
      }
    } else {
      other_done = std::max(other_done, done);
    }
  };
  const auto r =
      verify(load_model("premium_fr3"), Algorithm::kComputedTorque, load_task("reference_move"), o);
  ASSERT_FALSE(r.is_ok());
  EXPECT_NE(r.reason().find("cancelled"), std::string::npos) << r.reason();
  EXPECT_GE(tune_done, 1U);
  EXPECT_LE(tune_done, 2U);
  EXPECT_EQ(other_done, 0U);
}

}  // namespace
}  // namespace cobot::bench
