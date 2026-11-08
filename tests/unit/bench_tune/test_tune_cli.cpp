// `cobot_bench tune` (brief auto-tune, section 6) as a child process, COBOT_BENCH_EXE: writes
// tune.csv, prints the table, the chosen level and the reason. Exit 0 also when nothing is
// admissible (that is a result); exit 1 on bad input, with no tune.csv. Bad input needs no run, so
// it is checked in every build; the runs are release builds only.
#include <array>
#include <cstdio>
#include <filesystem>
#include <string>
#include <sys/wait.h>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/control/algorithm.hpp"

#include "tune.hpp"
#include "tune_test_util.hpp"

namespace cobot::bench {
namespace {

using namespace tune_test;  // NOLINT(google-build-using-namespace)

const char* const kHeader =
    "level,seeds,held,sat_frac_max,decays_all,rms_joint_err_median,final_pos_err_max,chosen\n";

struct Cli {
  int exit_code = -1;
  std::string output;
};

Cli run_bench(const std::string& args) {
  const std::string cmd = std::string("'") + COBOT_BENCH_EXE + "' " + args + " 2>&1";
  Cli cli;
  FILE* pipe = popen(cmd.c_str(), "r");
  if (pipe == nullptr) {
    return cli;
  }
  std::array<char, 4096> buf{};
  while (fgets(buf.data(), static_cast<int>(buf.size()), pipe) != nullptr) {
    cli.output += buf.data();
  }
  const int status = pclose(pipe);
  cli.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  return cli;
}

std::string quoted(const fs::path& p) {
  return "'" + p.string() + "'";
}

// `cobot_bench tune` on a model, an algorithm and reference_move, then `extra` flags.
std::string tune_args(const std::string& model, const std::string& algorithm, const fs::path& out) {
  return "tune --hardware " + quoted(model_path(model)) + " --algorithm " + algorithm + " --task " +
         quoted(task_path("reference_move")) + " --out " + quoted(out);
}

std::size_t line_count(const std::string& text) {
  std::size_t lines = 0;
  for (const char c : text) {
    lines += c == '\n' ? 1 : 0;
  }
  return lines;
}

void expect_bad_input(const std::string& args, const fs::path& out, const std::string& mentions) {
  const Cli cli = run_bench(args);
  EXPECT_EQ(cli.exit_code, 1) << args << "\n" << cli.output;
  EXPECT_TRUE(contains(cli.output, mentions)) << args << "\n" << cli.output;
  EXPECT_FALSE(fs::exists(out / "tune.csv")) << args;
}

TEST(TuneCli, AnUnlistedAlgorithmNamesTheModelAndWritesNoFile) {
  const fs::path out = test_dir() / "out";
  expect_bad_input(tune_args("premium_fr3", "drive_position", out) + " --levels 80 --seeds 1",
                   out,
                   "premium_fr3");
}

TEST(TuneCli, BadInputExitsOneAndWritesNoFile) {
  const fs::path out = test_dir() / "out";
  const std::string good = tune_args("premium_fr3", "computed_torque", out);
  expect_bad_input(
      tune_args("premium_fr3", "no_such_law", out) + " --levels 50 --seeds 1", out, "no_such_law");
  expect_bad_input(tune_args("no_such_model", "computed_torque", out) + " --levels 50 --seeds 1",
                   out,
                   "no_such_model");
  expect_bad_input(good + " --levels abc --seeds 1", out, "--levels");
  expect_bad_input(good + " --levels 50,,80 --seeds 1", out, "--levels");
  expect_bad_input(good + " --levels 50x --seeds 1", out, "--levels");
  expect_bad_input(good + " --levels 50 --seeds 0", out, "--seeds");
  expect_bad_input(good + " --levels 50 --seeds abc", out, "--seeds");
  expect_bad_input(good + " --levels 50 --seeds 1 --jobs abc", out, "--jobs");
  expect_bad_input(good + " --levels -1 --seeds 1", out, "omega");
  expect_bad_input(good + " --no-such-flag 1", out, "--no-such-flag");
}

TEST(TuneCli, MissingRequiredFlagsExitOne) {
  const fs::path out = test_dir() / "out";
  const std::string model = quoted(model_path("premium_fr3"));
  const std::string task = quoted(task_path("reference_move"));
  expect_bad_input("tune --algorithm computed_torque --task " + task + " --out " + quoted(out),
                   out,
                   "--hardware");
  expect_bad_input(
      "tune --hardware " + model + " --task " + task + " --out " + quoted(out), out, "--algorithm");
  expect_bad_input("tune --hardware " + model + " --algorithm computed_torque --out " + quoted(out),
                   out,
                   "--task");
  const Cli no_out =
      run_bench("tune --hardware " + model + " --algorithm computed_torque --task " + task);
  EXPECT_EQ(no_out.exit_code, 1) << no_out.output;
  EXPECT_TRUE(contains(no_out.output, "--out")) << no_out.output;
}

TEST(TuneCli, WritesTheSameFileAsTheEngineAndPrintsTheChoice) {
  if (!kOptimised) {
    GTEST_SKIP() << "elastic 7-DOF closed loop: release builds only";
  }
  // --out does not exist yet: it is created, as for montecarlo and sweep.
  const fs::path dir = test_dir();
  const fs::path out = dir / "nested" / "out";
  const Cli cli = run_bench(tune_args("premium_fr3", "computed_torque", out) +
                            " --levels 50,80 --seeds 1 --jobs 2");
  ASSERT_EQ(cli.exit_code, 0) << cli.output;
  const std::string csv = read_text(out / "tune.csv");
  ASSERT_EQ(line_count(csv), 3U) << csv;  // header + two levels
  EXPECT_EQ(csv.compare(0, std::string(kHeader).size(), kHeader), 0) << csv;
  EXPECT_TRUE(contains(cli.output, "chosen")) << cli.output;
  EXPECT_TRUE(contains(cli.output, "50")) << cli.output;
  EXPECT_TRUE(contains(cli.output, "80")) << cli.output;

  // --seeds 1 is seeds 1..1: the bytes of the engine on the same input.
  const auto r = tune(load_model("premium_fr3"),
                      control::Algorithm::kComputedTorque,
                      load_task("reference_move"),
                      options({50.0, 80.0}, {1}, 2));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const fs::path engine_csv = dir / "engine.csv";
  ASSERT_TRUE(write_tune_csv(engine_csv, r.value()).is_ok());
  EXPECT_TRUE(csv == read_text(engine_csv)) << csv << "\n---\n" << read_text(engine_csv);
  ASSERT_TRUE(r.value().chosen.has_value());
  EXPECT_TRUE(contains(cli.output, r.value().reason)) << cli.output;
}

TEST(TuneCli, NothingAdmissibleIsAResultAndExitsZero) {
  if (!kOptimised) {
    GTEST_SKIP() << "elastic 7-DOF closed loop: release builds only";
  }
  const fs::path out = test_dir() / "out";
  const Cli cli =
      run_bench(tune_args("budget_xarm7", "computed_torque", out) + " --levels 20 --seeds 1");
  ASSERT_EQ(cli.exit_code, 0) << cli.output;
  EXPECT_TRUE(contains(cli.output, "no admissible gain")) << cli.output;
  const std::string csv = read_text(out / "tune.csv");
  ASSERT_EQ(line_count(csv), 2U) << csv;
  // Budget omega 20, seed 1 holds: chosen column 0 on the only row, the line ends with ",0".
  EXPECT_EQ(csv.compare(csv.size() - 3, 3, ",0\n"), 0) << csv;
}

}  // namespace
}  // namespace cobot::bench
