// `cobot_bench verify` (brief verify, section 6) as a child process, COBOT_BENCH_EXE: writes
// report.json, report.md and tune.csv (when tuned) into --out and prints the MD. Exit 0 for every
// verdict (a FAIL is a result), 1 on bad input with no file written; the `tune` parsing and its
// messages for --levels and --jobs. Bad input needs no run, so it is checked in every build; the
// runs are release builds only.
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>

#include <gtest/gtest.h>

#include "sha256.hpp"
#include "verify_test_util.hpp"

namespace cobot::bench {
namespace {

using namespace verify_test;  // NOLINT(google-build-using-namespace)

#define COBOT_RELEASE_ONLY()                                          \
  if (!kOptimised) {                                                  \
    GTEST_SKIP() << "elastic 7-DOF closed loop: release builds only"; \
  }

void expect_bad_input(const std::string& args, const fs::path& out, const std::string& mentions) {
  const Cli cli = run_bench(args);
  EXPECT_EQ(cli.exit_code, 1) << args << "\n" << cli.output;
  EXPECT_TRUE(contains(cli.output, mentions)) << args << "\n" << cli.output;
  EXPECT_TRUE(no_report_file(out)) << args;
}

std::string with_out(const std::string& args, const fs::path& out) {
  return args + " --out " + quoted(out);
}

TEST(VerifyCli, UnknownFlagExitsOneAndWritesNoFile) {
  const fs::path out = test_dir() / "out";
  expect_bad_input(
      with_out(verify_args("premium_fr3", "computed_torque"), out) + " --runs 5 --no-such-flag 1",
      out,
      "--no-such-flag");
}

TEST(VerifyCli, MissingOutExitsOne) {
  const fs::path dir = test_dir();
  const Cli cli = run_bench(verify_args("premium_fr3", "computed_torque") + " --runs 5 --quick");
  EXPECT_EQ(cli.exit_code, 1) << cli.output;
  EXPECT_TRUE(contains(cli.output, "--out")) << cli.output;
  EXPECT_TRUE(no_report_file(dir));
  EXPECT_TRUE(no_report_file(fs::current_path()));
}

TEST(VerifyCli, BadRunsExitOne) {
  const fs::path out = test_dir() / "out";
  const std::string good = with_out(verify_args("premium_fr3", "computed_torque"), out);
  expect_bad_input(good + " --runs abc", out, "--runs");
  expect_bad_input(good + " --runs 0", out, "--runs");
  expect_bad_input(good + " --runs -3", out, "--runs");
  expect_bad_input(good + " --runs 5x", out, "--runs");
}

TEST(VerifyCli, OtherBadNumbersExitOne) {
  const fs::path out = test_dir() / "out";
  const std::string good = with_out(verify_args("premium_fr3", "computed_torque"), out);
  expect_bad_input(good + " --runs 5 --seed0 abc", out, "--seed0");
  expect_bad_input(good + " --runs 5 --tune-seeds 0", out, "--tune-seeds");
  expect_bad_input(good + " --runs 5 --tune-seeds abc", out, "--tune-seeds");
  // Same messages as `cobot_bench tune`.
  expect_bad_input(good + " --runs 5 --levels abc", out, "--levels");
  expect_bad_input(good + " --runs 5 --levels 50,,80", out, "--levels");
  expect_bad_input(good + " --runs 5 --jobs abc", out, "--jobs");
  expect_bad_input(good + " --runs 5 --levels -1 --tune-seeds 1", out, "omega");
}

TEST(VerifyCli, MissingOrUnknownInputsExitOne) {
  const fs::path out = test_dir() / "out";
  const std::string model = quoted(model_path("premium_fr3"));
  const std::string task = quoted(task_path("reference_move"));
  expect_bad_input(
      with_out("verify --algorithm computed_torque --task " + task, out), out, "--hardware");
  expect_bad_input(
      with_out("verify --hardware " + model + " --task " + task, out), out, "--algorithm");
  expect_bad_input(
      with_out("verify --hardware " + model + " --algorithm computed_torque", out), out, "--task");
  expect_bad_input(
      with_out(verify_args("premium_fr3", "no_such_law"), out) + " --runs 5", out, "no_such_law");
  expect_bad_input(with_out(verify_args("no_such_model", "computed_torque"), out) + " --runs 5",
                   out,
                   "no_such_model");
  // premium_fr3 does not list drive_position: named before any run.
  expect_bad_input(with_out(verify_args("premium_fr3", "drive_position"), out) + " --runs 5",
                   out,
                   "premium_fr3");
}

// The inputs of report.json name files whose bytes hash to the recorded sha256.
void expect_input_hashes(const Json& doc) {
  const Json* versions = doc.find("versions");
  ASSERT_NE(versions, nullptr);
  const Json* in = versions->find("inputs");
  ASSERT_NE(in, nullptr);
  ASSERT_EQ(in->items.size(), 4U);  // model, imperfections, controller, task
  for (const Json& f : in->items) {
    const Json* path = f.find("path");
    const Json* sha = f.find("sha256");
    ASSERT_NE(path, nullptr);
    ASSERT_NE(sha, nullptr);
    ASSERT_TRUE(fs::exists(path->text)) << path->text;
    EXPECT_EQ(sha->text, sha256_hex(read_text(path->text))) << path->text;
  }
}

TEST(VerifyCli, JobsOneAndThreeWriteTheSameBytes) {
  COBOT_RELEASE_ONLY();
  const fs::path dir = test_dir();
  const std::string args = verify_args("premium_fr3", "computed_torque") +
                           " --quick --runs 3 --levels 50 --tune-seeds 1";
  const Cli one = run_bench(with_out(args, dir / "one") + " --jobs 1");
  ASSERT_EQ(one.exit_code, 0) << one.output;
  // --out does not exist yet: it is created, as for tune.
  const Cli three = run_bench(with_out(args, dir / "nested" / "three") + " --jobs 3");
  ASSERT_EQ(three.exit_code, 0) << three.output;

  for (const char* name : {"report.json", "report.md", "tune.csv"}) {
    const std::string a = read_text(dir / "one" / name);
    ASSERT_FALSE(a.empty()) << name;
    EXPECT_TRUE(a == read_text(dir / "nested" / "three" / name))
        << name << "\n"
        << a << "\n---\n"
        << read_text(dir / "nested" / "three" / name);
  }
  // It prints the MD.
  const std::string md = read_text(dir / "one" / "report.md");
  EXPECT_TRUE(contains(one.output, md)) << one.output;
  EXPECT_TRUE(contains(first_line(md), kDisclaimer)) << md;

  const std::optional<Json> doc = parse_json(read_text(dir / "one" / "report.json"));
  ASSERT_TRUE(doc.has_value());
  EXPECT_EQ(doc->find("verdict")->text, "marginal");
  EXPECT_TRUE(doc->find("quick")->boolean);
  EXPECT_EQ(doc->find("pass_count")->number, 3.0);
  const Json* seeds = doc->find("seeds");
  ASSERT_NE(seeds, nullptr);
  ASSERT_EQ(seeds->items.size(), 2U);
  EXPECT_EQ(seeds->items[0].number, 1.0);
  EXPECT_EQ(seeds->items[1].number, 3.0);
  expect_input_hashes(*doc);
}

TEST(VerifyCli, NoTuneUsesTheModelsGainAndWritesNoTuneCsv) {
  COBOT_RELEASE_ONLY();
  const fs::path out = test_dir() / "out";
  const Cli cli = run_bench(with_out(verify_args("premium_fr3", "computed_torque"), out) +
                            " --no-tune --quick --runs 2 --seed0 4 --jobs 2");
  ASSERT_EQ(cli.exit_code, 0) << cli.output;
  EXPECT_FALSE(fs::exists(out / "tune.csv"));
  const std::optional<Json> doc = parse_json(read_text(out / "report.json"));
  ASSERT_TRUE(doc.has_value());
  const Json* tuning = doc->find("tuning");
  ASSERT_NE(tuning, nullptr);
  EXPECT_EQ(tuning->find("gain_source")->text, "default_tuning_off");
  // config/hardware/premium_fr3.yaml: config.computed_torque.omega 50.
  EXPECT_EQ(tuning->find("gain_used")->number, 50.0);
  EXPECT_EQ(tuning->find("key")->text, "config.computed_torque.omega");
  const Json* seeds = doc->find("seeds");
  ASSERT_NE(seeds, nullptr);
  ASSERT_EQ(seeds->items.size(), 2U);
  EXPECT_EQ(seeds->items[0].number, 4.0);
  EXPECT_EQ(seeds->items[1].number, 5.0);
  EXPECT_EQ(doc->find("runs")->number, 2.0);
  const std::string md = read_text(out / "report.md");
  EXPECT_TRUE(contains(md, "tuning off")) << md;
  EXPECT_TRUE(contains(cli.output, kDisclaimer)) << cli.output;
}

TEST(VerifyCli, AFailIsAResultAndExitsZero) {
  COBOT_RELEASE_ONLY();
  const fs::path out = test_dir() / "out";
  const Cli cli = run_bench(with_out(verify_args("budget_xarm7", "computed_torque"), out) +
                            " --runs 2 --levels 20 --tune-seeds 1");
  ASSERT_EQ(cli.exit_code, 0) << cli.output;
  const std::optional<Json> doc = parse_json(read_text(out / "report.json"));
  ASSERT_TRUE(doc.has_value());
  EXPECT_EQ(doc->find("verdict")->text, "fail");
  EXPECT_TRUE(doc->find("quick")->boolean);  // 2 runs < 100
  EXPECT_TRUE(fs::exists(out / "tune.csv"));
  EXPECT_TRUE(contains(cli.output, "FAIL")) << cli.output;
}

}  // namespace
}  // namespace cobot::bench
