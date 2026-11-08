// `cobot_bench verify` with lists (brief verify-analysis, sections 3 and 4) as a child process,
// COBOT_BENCH_EXE.
// Every build (no run starts): an empty item in --hardware or --algorithm, an unknown model file
// anywhere in the list, an unknown algorithm in the list, no listed pair, a bad
// --sensitivity-seeds or --ablation-seeds: exit 1 and no file at all under --out.
// Release builds only (runs):
//  - one pair keeps 6a's layout: report.json and report.md in --out, no subdirectory, no
//    comparison file (E2E-15 unchanged);
//  - two pairs: <out>/<model>__<algorithm>/report.{json,md} each, exactly the bytes a single
//    call of that pair writes, plus comparison.json and comparison.md; --jobs 1 and --jobs 3 give
//    the same bytes for every file.
//
// The two-pair run, by hand (reference_move, --runs 1, --no-tune): premium_fr3 CT at its own
// omega passes (#78), 1/1 is quick: wilson(1, 1).hi = 1 >= 0.95, not FAIL -> marginal.
// budget_xarm7 CT at its own omega 20 holds (#81): 0/1, wilson(0, 1).hi = z^2 / (1 + z^2) =
// 3.8415 / 4.8415 = 0.793 < 0.95 -> fail, so the ablation runs (7 groups x --ablation-seeds 1).
#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "analysis_test_util.hpp"
#include "verify_test_util.hpp"

namespace cobot::bench {
namespace {

using namespace analysis_test;  // NOLINT(google-build-using-namespace)
using verify_test::Cli;
using verify_test::contains;
using verify_test::model_path;
using verify_test::quoted;
using verify_test::read_text;
using verify_test::run_bench;
using verify_test::task_path;

#define COBOT_RELEASE_ONLY()                                          \
  if (!verify_test::kOptimised) {                                     \
    GTEST_SKIP() << "elastic 7-DOF closed loop: release builds only"; \
  }

std::string models(const std::vector<std::string>& names) {
  std::string text;
  for (const std::string& n : names) {
    text += (text.empty() ? "" : ",") + model_path(n).string();
  }
  return text;
}

// `cobot_bench verify` with a raw --hardware and --algorithm text, --task reference_move.
std::string args(const std::string& hardware, const std::string& algorithms, const fs::path& out) {
  return "verify --hardware '" + hardware + "' --algorithm " + algorithms + " --task " +
         quoted(task_path("reference_move")) + " --out " + quoted(out);
}

void expect_bad_input(const std::string& a, const fs::path& out, const std::string& mentions) {
  const Cli cli = run_bench(a);
  EXPECT_EQ(cli.exit_code, 1) << a << "\n" << cli.output;
  EXPECT_TRUE(contains(cli.output, mentions)) << a << "\n" << cli.output;
  EXPECT_TRUE(files_under(out).empty()) << a;
}

TEST(AnalysisCli, AnEmptyItemInAListExitsOne) {
  const fs::path out = test_dir() / "out";
  const std::string two = models({"premium_fr3", "budget_xarm7"});
  const std::string p = model_path("premium_fr3").string();
  const std::string b = model_path("budget_xarm7").string();
  expect_bad_input(args(p + ",," + b, "computed_torque", out) + " --runs 1", out, "--hardware");
  expect_bad_input(args(p + ",", "computed_torque", out) + " --runs 1", out, "--hardware");
  expect_bad_input(args("," + p, "computed_torque", out) + " --runs 1", out, "--hardware");
  expect_bad_input(args(two, "computed_torque,", out) + " --runs 1", out, "--algorithm");
  expect_bad_input(args(two, "computed_torque,,pd_gravity", out) + " --runs 1", out, "--algorithm");
  expect_bad_input(args(two, ",computed_torque", out) + " --runs 1", out, "--algorithm");
}

TEST(AnalysisCli, AnUnknownModelFileInTheListExitsOneBeforeAnyRun) {
  const fs::path dir = test_dir();
  const fs::path out = dir / "out";
  const fs::path missing = dir / "no_such_model.yaml";
  const std::string p = model_path("premium_fr3").string();
  // Second in the list: the first pair must not have run or written anything.
  expect_bad_input(args(p + "," + missing.string(), "computed_torque", out) + " --runs 1 --no-tune",
                   out,
                   "no_such_model.yaml");
  expect_bad_input(args(missing.string() + "," + p, "computed_torque", out) + " --runs 1 --no-tune",
                   out,
                   "no_such_model.yaml");
}

TEST(AnalysisCli, AnUnknownAlgorithmInTheListExitsOne) {
  const fs::path out = test_dir() / "out";
  expect_bad_input(
      args(models({"premium_fr3"}), "computed_torque,no_such_law", out) + " --runs 1 --no-tune",
      out,
      "no_such_law");
}

TEST(AnalysisCli, NoListedPairExitsOne) {
  const fs::path out = test_dir() / "out";
  // premium_fr3 and mid_kr1205 list computed_torque and pd_gravity only.
  expect_bad_input(
      args(models({"premium_fr3", "mid_kr1205"}), "drive_position", out) + " --runs 1 --no-tune",
      out,
      "drive_position");
}

TEST(AnalysisCli, BadAnalysisSeedCountsExitOne) {
  const fs::path out = test_dir() / "out";
  const std::string good = args(models({"premium_fr3"}), "computed_torque", out) + " --runs 1";
  expect_bad_input(good + " --sensitivity-seeds 0", out, "--sensitivity-seeds");
  expect_bad_input(good + " --sensitivity-seeds abc", out, "--sensitivity-seeds");
  expect_bad_input(good + " --ablation-seeds 0", out, "--ablation-seeds");
  expect_bad_input(good + " --ablation-seeds -2", out, "--ablation-seeds");
}

// ---- runs ------------------------------------------------------------------------------------

std::vector<fs::path> sorted_files(const fs::path& dir) {
  std::vector<fs::path> files = files_under(dir);
  std::sort(files.begin(), files.end());
  return files;
}

TEST(AnalysisCli, OnePairKeepsTheSingleLayout) {
  COBOT_RELEASE_ONLY();
  const fs::path out = test_dir() / "out";
  const Cli cli = run_bench(args(models({"premium_fr3"}), "computed_torque", out) +
                            " --runs 1 --quick --no-tune --no-sensitivity --jobs 1");
  ASSERT_EQ(cli.exit_code, 0) << cli.output;
  EXPECT_EQ(sorted_files(out), (std::vector<fs::path>{"report.json", "report.md"}));
  for (const fs::directory_entry& e : fs::directory_iterator(out)) {
    EXPECT_FALSE(e.is_directory()) << e.path();
  }
}

TEST(AnalysisCli, TwoPairsWriteOneDirectoryEachAndAComparisonWhateverTheJobs) {
  COBOT_RELEASE_ONLY();
  const fs::path dir = test_dir();
  const std::string two = models({"premium_fr3", "budget_xarm7"});
  const std::string flags = " --runs 1 --no-tune --ablation-seeds 1";
  const Cli one = run_bench(args(two, "computed_torque", dir / "jobs1") + flags + " --jobs 1");
  ASSERT_EQ(one.exit_code, 0) << one.output;
  const Cli three = run_bench(args(two, "computed_torque", dir / "jobs3") + flags + " --jobs 3");
  ASSERT_EQ(three.exit_code, 0) << three.output;

  const std::vector<fs::path> want = {
      "budget_xarm7__computed_torque/report.json",
      "budget_xarm7__computed_torque/report.md",
      "comparison.json",
      "comparison.md",
      "premium_fr3__computed_torque/report.json",
      "premium_fr3__computed_torque/report.md",
  };
  ASSERT_EQ(sorted_files(dir / "jobs1"), want);
  ASSERT_EQ(sorted_files(dir / "jobs3"), want);
  for (const fs::path& f : want) {
    EXPECT_EQ(read_text(dir / "jobs1" / f), read_text(dir / "jobs3" / f)) << f;
  }

  // The comparison: disclaimer first, one row per pair in the given order.
  const std::string md = read_text(dir / "jobs1" / "comparison.md");
  EXPECT_EQ(verify_test::first_line(md), verify_test::kDisclaimer);
  const auto doc = verify_test::parse_json(read_text(dir / "jobs1" / "comparison.json"));
  ASSERT_TRUE(doc.has_value());
  ASSERT_FALSE(doc->keys.empty());
  EXPECT_EQ(doc->keys[0], "disclaimer");
  const verify_test::Json* rows = doc->find("rows");
  ASSERT_NE(rows, nullptr);
  ASSERT_EQ(rows->items.size(), 2U);
  EXPECT_EQ(rows->items[0].find("model")->text, "premium_fr3");
  EXPECT_EQ(rows->items[0].find("verdict")->text, "marginal");
  EXPECT_EQ(rows->items[1].find("model")->text, "budget_xarm7");
  EXPECT_EQ(rows->items[1].find("verdict")->text, "fail");

  // The budget FAIL has its causes in its own report.
  const auto budget = verify_test::parse_json(
      read_text(dir / "jobs1" / "budget_xarm7__computed_torque" / "report.json"));
  ASSERT_TRUE(budget.has_value());
  EXPECT_EQ(budget->find("verdict")->text, "fail");
  const verify_test::Json* causes = budget->find("causes");
  ASSERT_NE(causes, nullptr);
  EXPECT_FALSE(causes->items.empty() && causes->keys.empty());

  // Each pair's files are exactly what a single call of that pair writes.
  const Cli single = run_bench(args(models({"premium_fr3"}), "computed_torque", dir / "single") +
                               flags + " --jobs 2");
  ASSERT_EQ(single.exit_code, 0) << single.output;
  for (const char* name : {"report.json", "report.md"}) {
    EXPECT_EQ(read_text(dir / "single" / name),
              read_text(dir / "jobs1" / "premium_fr3__computed_torque" / name))
        << name;
  }
}

}  // namespace
}  // namespace cobot::bench
