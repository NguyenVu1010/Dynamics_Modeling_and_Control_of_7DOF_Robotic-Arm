// Cases the review of #82 listed (brief verify-analysis, sections 3 and 4):
// every build (no run starts): an algorithm or a model given twice in the lists is exit 1 with no
// file; bench::verify refuses sensitivity_seeds or ablation_seeds of 0; bench::compare returns a
// cancel as the bare "cancelled" of verify.
// Release builds only (runs): two models and one algorithm that only one of them lists still give
// the comparison layout, and the other pair is named "not listed".
#include <algorithm>
#include <atomic>
#include <filesystem>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/control/algorithm.hpp"

#include "analysis_test_util.hpp"
#include "compare.hpp"
#include "verify.hpp"
#include "verify_test_util.hpp"

namespace cobot::bench {
namespace {

using namespace analysis_test;  // NOLINT(google-build-using-namespace)
using control::Algorithm;
using verify_test::Cli;
using verify_test::contains;
using verify_test::model_path;
using verify_test::quoted;
using verify_test::run_bench;
using verify_test::task_path;

std::string verify_args(const std::string& hardware,
                        const std::string& algorithms,
                        const fs::path& out) {
  return "verify --hardware '" + hardware + "' --algorithm " + algorithms + " --task " +
         quoted(task_path("reference_move")) + " --out " + quoted(out) + " --runs 1 --no-tune";
}

void expect_bad_input(const std::string& a, const fs::path& out, const std::string& mentions) {
  const Cli cli = run_bench(a);
  EXPECT_EQ(cli.exit_code, 1) << a << "\n" << cli.output;
  EXPECT_TRUE(contains(cli.output, mentions)) << a << "\n" << cli.output;
  EXPECT_TRUE(files_under(out).empty()) << a;
}

TEST(ReviewFollowups, AnAlgorithmGivenTwiceExitsOneBeforeAnyRun) {
  const fs::path out = test_dir() / "out";
  expect_bad_input(
      verify_args(
          model_path("premium_fr3").string(), "computed_torque,pd_gravity,computed_torque", out),
      out,
      "computed_torque given twice");
}

TEST(ReviewFollowups, AModelGivenTwiceExitsOneBeforeAnyRun) {
  const fs::path out = test_dir() / "out";
  const std::string p = model_path("premium_fr3").string();
  const std::string b = model_path("budget_xarm7").string();
  expect_bad_input(
      verify_args(p + "," + b + "," + p, "computed_torque", out), out, "premium_fr3 given twice");
}

TEST(ReviewFollowups, ZeroAnalysisSeedsAreRefusedByTheLibrary) {
  const auto model = verify_test::load_model("premium_fr3");
  const auto task = verify_test::load_task("reference_move");
  for (const bool sensitivity : {true, false}) {
    VerifyOptions o;
    o.runs = 1;
    o.tune = false;
    (sensitivity ? o.sensitivity_seeds : o.ablation_seeds) = 0;
    const auto r = verify(model, Algorithm::kComputedTorque, task, o);
    ASSERT_FALSE(r.is_ok()) << sensitivity;
    EXPECT_TRUE(contains(r.reason(), sensitivity ? "sensitivity_seeds" : "ablation_seeds"))
        << r.reason();
  }
}

TEST(ReviewFollowups, ACancelDuringACompareIsTheBareCancelled) {
  const std::atomic<bool> cancel{true};
  VerifyOptions o;
  o.runs = 1;
  o.tune = false;
  o.cancel = &cancel;
  const auto rows =
      compare({verify_test::load_model("premium_fr3"), verify_test::load_model("budget_xarm7")},
              {Algorithm::kComputedTorque},
              verify_test::load_task("reference_move"),
              o);
  ASSERT_FALSE(rows.is_ok());
  EXPECT_EQ(rows.reason(), "cancelled");
}

TEST(ReviewFollowups, OneListedPairOfTwoStillGivesTheComparisonAndNamesTheOther) {
  if (!verify_test::kOptimised) {
    GTEST_SKIP() << "elastic 7-DOF closed loop: release builds only";
  }
  const fs::path out = test_dir() / "out";
  const std::string two =
      model_path("premium_fr3").string() + "," + model_path("budget_xarm7").string();
  const Cli cli = run_bench(verify_args(two, "drive_position", out) +
                            " --no-sensitivity --ablation-seeds 1 --jobs 2");
  ASSERT_EQ(cli.exit_code, 0) << cli.output;
  std::vector<fs::path> files = files_under(out);
  std::sort(files.begin(), files.end());
  EXPECT_EQ(files,
            (std::vector<fs::path>{"budget_xarm7__drive_position/report.json",
                                   "budget_xarm7__drive_position/report.md",
                                   "comparison.json",
                                   "comparison.md"}));
  const auto doc = verify_test::parse_json(verify_test::read_text(out / "comparison.json"));
  ASSERT_TRUE(doc.has_value());
  const verify_test::Json* rows = doc->find("rows");
  ASSERT_NE(rows, nullptr);
  ASSERT_EQ(rows->items.size(), 2U);
  EXPECT_EQ(rows->items[0].find("model")->text, "premium_fr3");
  EXPECT_EQ(rows->items[0].find("verdict")->text, "not listed");
  EXPECT_EQ(rows->items[1].find("model")->text, "budget_xarm7");
}

}  // namespace
}  // namespace cobot::bench
