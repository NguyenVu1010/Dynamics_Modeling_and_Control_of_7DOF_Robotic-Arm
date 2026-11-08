// The studio's verify job (feature studio-verify, cobot_studio design 10 G14/G16, ADR-023 "Hệ quả":
// studio and `cobot_bench verify` never disagree; ADR-021: the studio runs the bench engine in
// process). Qt-free: cobot_studio_core and cobot_bench_run.
//
// API under test, as these tests read the brief (src/studio/core/include/cobot/studio/
// verify_job.hpp, namespace cobot::studio):
//
//   // phase: the bench phase of VerifyOptions::progress ("tune", "runs", "sensitivity",
//   // "ablation"); pair: 0-based index of the running pair among the LISTED pairs (the ones that
//   // run), pairs: their number; done / total: as VerifyOptions::progress.
//   using VerifyProgressFn = std::function<void(std::string_view phase, std::size_t pair,
//                                               std::size_t pairs, std::size_t done,
//                                               std::size_t total)>;
//
//   struct VerifyRequest {
//     std::vector<fs::path> hardware;                 // config/hardware/<model>.yaml files
//     std::vector<control::Algorithm> algorithms;
//     fs::path task;                                  // config/tasks/<task>.yaml
//     bench::VerifyOptions options;                   // progress and cancel are the job's own
//     fs::path out_dir;                               // where the outputs go; empty: an error
//   };
//
//   struct VerifyOutcome {
//     std::vector<bench::ComparisonRow> rows;  // bench::compare's rows
//     std::vector<fs::path> files;             // every file written, each out_dir / <relative>
//   };
//
//   // Loads the task and the models and checks them as `cobot_bench verify` does, with the same
//   // messages (reason() is the text the CLI prints after "cobot_bench verify: "), all before the
//   // first run; then bench::compare; then bench::write_verify_outputs into out_dir (the CLI's
//   // writer: all files or none, 1x1 call -> report.{json,md} (+ tune.csv) in out_dir itself).
//   // `cancel` set at any time -> the error "cancelled" and no file. progress may be empty.
//   common::Result<VerifyOutcome> run_verify_job(const VerifyRequest& request,
//                                                const std::atomic<bool>& cancel,
//                                                const VerifyProgressFn& progress);
//
//   // A new, empty <work_dir>/verify_NNNN (the first free number from 0001, like run_NNNN);
//   // work_dir is created when missing.
//   common::Result<fs::path> new_verify_dir(const fs::path& work_dir);
//
// Every build: bad input fails before any run and writes nothing; a cancel set before the start;
// new_verify_dir. Release builds only (closed-loop runs): the job's files are the CLI's bytes for
// a 1x1 quick call, for two models and for a pair the model does not list, and a cancel between
// two pairs leaves no file. The CLI is the child process COBOT_BENCH_EXE.
#if __has_include("cobot/studio/verify_job.hpp")

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <filesystem>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/control/algorithm.hpp"
#include "cobot/studio/verify_job.hpp"

#include "compare.hpp"
#include "verify.hpp"
#include "verify_test_util.hpp"

namespace cobot::studio {
namespace {

namespace fs = std::filesystem;
using bench::verify_test::Cli;
using bench::verify_test::contains;
using bench::verify_test::kOptimised;
using bench::verify_test::model_path;
using bench::verify_test::quoted;
using bench::verify_test::read_text;
using bench::verify_test::run_bench;
using bench::verify_test::task_path;
using control::Algorithm;

#define COBOT_RELEASE_ONLY()                                          \
  if (!kOptimised) {                                                  \
    GTEST_SKIP() << "elastic 7-DOF closed loop: release builds only"; \
  }

// An empty directory of its own for the running test (ctest runs tests in parallel processes).
fs::path test_dir() {
  const ::testing::TestInfo* info = ::testing::UnitTest::GetInstance()->current_test_info();
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "studio_verify_job" /
                       (std::string(info->test_suite_name()) + "." + info->name());
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir;
}

// Every regular file under `dir`, relative to it, sorted (empty when `dir` does not exist).
std::vector<fs::path> files_under(const fs::path& dir) {
  std::vector<fs::path> out;
  std::error_code ec;
  if (!fs::exists(dir, ec)) {
    return out;
  }
  for (const fs::directory_entry& e : fs::recursive_directory_iterator(dir)) {
    if (e.is_regular_file()) {
      out.push_back(e.path().lexically_relative(dir));
    }
  }
  std::sort(out.begin(), out.end());
  return out;
}

// `dir` holds nothing at all (or does not exist).
bool empty_dir(const fs::path& dir) {
  std::error_code ec;
  return !fs::exists(dir, ec) || fs::is_empty(dir, ec);
}

struct Call {
  std::string phase;
  std::size_t pair = 0;
  std::size_t pairs = 0;
  std::size_t done = 0;
  std::size_t total = 0;
};

// The progress calls of one job.
struct Recorder {
  std::vector<Call> calls;
  VerifyProgressFn fn() {
    return [this](std::string_view phase,
                  std::size_t pair,
                  std::size_t pairs,
                  std::size_t done,
                  std::size_t total) {
      calls.push_back(Call{std::string(phase), pair, pairs, done, total});
    };
  }
  bool any_run_done() const {
    return std::any_of(calls.begin(), calls.end(), [](const Call& c) { return c.done > 0; });
  }
};

VerifyRequest request(const std::vector<std::string>& models,
                      const std::vector<Algorithm>& algorithms,
                      const fs::path& out_dir,
                      const std::string& task = "reference_move") {
  VerifyRequest r;
  for (const std::string& m : models) {
    r.hardware.push_back(model_path(m));
  }
  r.algorithms = algorithms;
  r.task = task_path(task);
  r.out_dir = out_dir;
  return r;
}

// The brief's quick job: --quick --runs 3 --levels 50 --tune-seeds 1.
void make_quick(VerifyRequest& r) {
  r.options.quick = true;
  r.options.runs = 3;
  r.options.tuning.levels = {50.0};
  r.options.tuning.seeds = {1};
}

// --runs 1 --no-tune.
void make_one_untuned(VerifyRequest& r) {
  r.options.runs = 1;
  r.options.tune = false;
}

std::string hardware_arg(const std::vector<std::string>& models) {
  std::string list;
  for (const std::string& m : models) {
    list += (list.empty() ? "" : ",") + model_path(m).string();
  }
  return "'" + list + "'";
}

std::string cli_args(const std::vector<std::string>& models,
                     const std::string& algorithms,
                     const fs::path& out,
                     const std::string& task = "reference_move") {
  return "verify --hardware " + hardware_arg(models) + " --algorithm " + algorithms + " --task " +
         quoted(task_path(task)) + " --out " + quoted(out);
}

// The job's files are exactly the CLI's, byte for byte; the outcome lists exactly them.
void expect_same_files(const fs::path& job_dir,
                       const fs::path& cli_dir,
                       const VerifyOutcome& outcome,
                       const std::vector<fs::path>& want) {
  ASSERT_EQ(files_under(cli_dir), want);
  ASSERT_EQ(files_under(job_dir), want);
  for (const fs::path& f : want) {
    const std::string job = read_text(job_dir / f);
    const std::string cli = read_text(cli_dir / f);
    EXPECT_FALSE(job.empty()) << f;
    EXPECT_TRUE(job == cli) << f << "\n--- job\n" << job << "\n--- cli\n" << cli;
  }
  std::vector<fs::path> listed;
  for (const fs::path& p : outcome.files) {
    const fs::path rel = p.lexically_relative(job_dir);
    EXPECT_FALSE(rel.empty() || *rel.begin() == "..") << p << " not under " << job_dir;
    listed.push_back(rel);
  }
  std::sort(listed.begin(), listed.end());
  EXPECT_EQ(listed, want);
}

// ---- every build: no run starts ---------------------------------------------------------------

TEST(StudioVerifyJob, NewVerifyDirTakesTheFirstFreeNumber) {
  const fs::path work = test_dir() / "work";  // does not exist yet
  const auto first = new_verify_dir(work);
  ASSERT_TRUE(first.is_ok()) << first.reason();
  EXPECT_EQ(first.value(), work / "verify_0001");
  EXPECT_TRUE(fs::is_directory(first.value()));
  EXPECT_TRUE(fs::is_empty(first.value()));

  const auto second = new_verify_dir(work);
  ASSERT_TRUE(second.is_ok()) << second.reason();
  EXPECT_EQ(second.value(), work / "verify_0002");

  // verify_0001 gone: it is free again (first free number, not the next one).
  fs::remove_all(work / "verify_0001");
  const auto again = new_verify_dir(work);
  ASSERT_TRUE(again.is_ok()) << again.reason();
  EXPECT_EQ(again.value(), work / "verify_0001");
  const auto third = new_verify_dir(work);
  ASSERT_TRUE(third.is_ok()) << third.reason();
  EXPECT_EQ(third.value(), work / "verify_0003");
}

// The job's reason is the text the CLI prints after "cobot_bench verify: " for the same input.
void expect_cli_message(const std::string& reason, const std::string& args, const fs::path& out) {
  const Cli cli = run_bench(args);
  EXPECT_EQ(cli.exit_code, 1) << cli.output;
  EXPECT_TRUE(contains(cli.output, "cobot_bench verify: " + reason + "\n"))
      << "job: " << reason << "\ncli: " << cli.output;
  EXPECT_TRUE(empty_dir(out)) << out;
}

TEST(StudioVerifyJob, UnknownModelFileIsAnErrorBeforeAnyRunAndWritesNothing) {
  const fs::path dir = test_dir();
  const auto out = new_verify_dir(dir / "work");
  ASSERT_TRUE(out.is_ok()) << out.reason();
  // The bad file second: every model is checked before the first run.
  const VerifyRequest r =
      request({"premium_fr3", "no_such_model"}, {Algorithm::kComputedTorque}, out.value());
  Recorder rec;
  const std::atomic<bool> cancel{false};
  const auto job = run_verify_job(r, cancel, rec.fn());
  ASSERT_FALSE(job.is_ok());
  EXPECT_TRUE(contains(job.reason(), "no_such_model")) << job.reason();
  EXPECT_TRUE(rec.calls.empty());
  EXPECT_TRUE(empty_dir(out.value()));
  expect_cli_message(job.reason(),
                     cli_args({"premium_fr3", "no_such_model"}, "computed_torque", dir / "cli"),
                     dir / "cli");
}

TEST(StudioVerifyJob, UnknownTaskFileIsAnErrorBeforeAnyRunAndWritesNothing) {
  const fs::path dir = test_dir();
  const fs::path out = dir / "out";
  const VerifyRequest r =
      request({"premium_fr3"}, {Algorithm::kComputedTorque}, out, "no_such_task");
  Recorder rec;
  const std::atomic<bool> cancel{false};
  const auto job = run_verify_job(r, cancel, rec.fn());
  ASSERT_FALSE(job.is_ok());
  EXPECT_TRUE(contains(job.reason(), "no_such_task")) << job.reason();
  EXPECT_TRUE(rec.calls.empty());
  EXPECT_TRUE(empty_dir(out));
  expect_cli_message(job.reason(),
                     cli_args({"premium_fr3"}, "computed_torque", dir / "cli", "no_such_task"),
                     dir / "cli");
}

TEST(StudioVerifyJob, NoListedPairIsAnErrorNamingTheAlgorithm) {
  const fs::path dir = test_dir();
  const fs::path out = dir / "out";
  // Neither premium_fr3 nor mid_kr1205 lists drive_position (config/hardware/*.yaml).
  const VerifyRequest r = request({"premium_fr3", "mid_kr1205"}, {Algorithm::kDrivePosition}, out);
  Recorder rec;
  const std::atomic<bool> cancel{false};
  const auto job = run_verify_job(r, cancel, rec.fn());
  ASSERT_FALSE(job.is_ok());
  EXPECT_TRUE(contains(job.reason(), "drive_position")) << job.reason();
  EXPECT_TRUE(rec.calls.empty());
  EXPECT_TRUE(empty_dir(out));
  expect_cli_message(job.reason(),
                     cli_args({"premium_fr3", "mid_kr1205"}, "drive_position", dir / "cli"),
                     dir / "cli");
}

TEST(StudioVerifyJob, EmptyListsAreErrors) {
  const fs::path out = test_dir() / "out";
  const std::atomic<bool> cancel{false};
  {
    Recorder rec;
    const auto job = run_verify_job(request({"premium_fr3"}, {}, out), cancel, rec.fn());
    EXPECT_FALSE(job.is_ok());
    EXPECT_FALSE(job.reason().empty());
    EXPECT_TRUE(rec.calls.empty());
  }
  {
    Recorder rec;
    const auto job =
        run_verify_job(request({}, {Algorithm::kComputedTorque}, out), cancel, rec.fn());
    EXPECT_FALSE(job.is_ok());
    EXPECT_FALSE(job.reason().empty());
    EXPECT_TRUE(rec.calls.empty());
  }
  EXPECT_TRUE(empty_dir(out));
}

TEST(StudioVerifyJob, RepeatedModelOrAlgorithmIsAnError) {
  const fs::path out = test_dir() / "out";
  const std::atomic<bool> cancel{false};
  // The CLI: "--hardware: model premium_fr3 given twice", "--algorithm: computed_torque given
  // twice".
  const auto models = run_verify_job(
      request({"premium_fr3", "premium_fr3"}, {Algorithm::kComputedTorque}, out), cancel, {});
  ASSERT_FALSE(models.is_ok());
  EXPECT_TRUE(contains(models.reason(), "premium_fr3")) << models.reason();
  EXPECT_TRUE(contains(models.reason(), "twice")) << models.reason();
  const auto algorithms = run_verify_job(
      request({"premium_fr3"}, {Algorithm::kComputedTorque, Algorithm::kComputedTorque}, out),
      cancel,
      {});
  ASSERT_FALSE(algorithms.is_ok());
  EXPECT_TRUE(contains(algorithms.reason(), "computed_torque")) << algorithms.reason();
  EXPECT_TRUE(contains(algorithms.reason(), "twice")) << algorithms.reason();
  EXPECT_TRUE(empty_dir(out));
}

TEST(StudioVerifyJob, BadOptionsAndAnEmptyOutDirAreErrorsBeforeAnyRun) {
  const fs::path out = test_dir() / "out";
  const std::atomic<bool> cancel{false};
  {
    VerifyRequest r = request({"premium_fr3"}, {Algorithm::kComputedTorque}, out);
    r.options.runs = 0;  // bench::verify: "verify: runs must be >= 1"
    Recorder rec;
    const auto job = run_verify_job(r, cancel, rec.fn());
    EXPECT_FALSE(job.is_ok());
    EXPECT_FALSE(rec.any_run_done());
  }
  EXPECT_TRUE(empty_dir(out));
  {
    // No out_dir: never the current directory (it may be the repository).
    VerifyRequest r = request({"premium_fr3"}, {Algorithm::kComputedTorque}, fs::path());
    make_quick(r);
    Recorder rec;
    const auto job = run_verify_job(r, cancel, rec.fn());
    EXPECT_FALSE(job.is_ok());
    EXPECT_FALSE(rec.any_run_done());
  }
}

TEST(StudioVerifyJob, CancelSetBeforeTheStartReturnsCancelledAndWritesNothing) {
  const fs::path dir = test_dir();
  const auto out = new_verify_dir(dir / "work");
  ASSERT_TRUE(out.is_ok()) << out.reason();
  VerifyRequest r = request({"premium_fr3"}, {Algorithm::kComputedTorque}, out.value());
  make_quick(r);
  Recorder rec;
  const std::atomic<bool> cancel{true};
  const auto job = run_verify_job(r, cancel, rec.fn());
  ASSERT_FALSE(job.is_ok());
  EXPECT_EQ(job.reason(), "cancelled");
  EXPECT_FALSE(rec.any_run_done());
  EXPECT_TRUE(empty_dir(out.value()));
}

// ---- release builds only: the same bytes as the CLI -------------------------------------------

TEST(StudioVerifyJob, QuickSinglePairWritesTheCliBytes) {
  COBOT_RELEASE_ONLY();
  const fs::path dir = test_dir();
  const auto out = new_verify_dir(dir / "work");
  ASSERT_TRUE(out.is_ok()) << out.reason();
  VerifyRequest r = request({"premium_fr3"}, {Algorithm::kComputedTorque}, out.value());
  make_quick(r);
  Recorder rec;
  const std::atomic<bool> cancel{false};
  const auto job = run_verify_job(r, cancel, rec.fn());
  ASSERT_TRUE(job.is_ok()) << job.reason();

  const Cli cli = run_bench(cli_args({"premium_fr3"}, "computed_torque", dir / "cli") +
                            " --quick --runs 3 --levels 50 --tune-seeds 1");
  ASSERT_EQ(cli.exit_code, 0) << cli.output;
  // 1x1: the single layout, as `cobot_bench verify` always wrote it.
  expect_same_files(
      out.value(), dir / "cli", job.value(), {"report.json", "report.md", "tune.csv"});

  // The rows the tab reads: premium_fr3 CT at omega 50 holds no seed (#78), 3/3 passed; quick
  // (3 runs < 100) -> never PASS; wilson(3, 3).hi = 1 >= 0.95, not FAIL -> MARGINAL.
  ASSERT_EQ(job.value().rows.size(), 1U);
  const bench::ComparisonRow& row = job.value().rows[0];
  EXPECT_EQ(row.model, "premium_fr3");
  EXPECT_EQ(row.algorithm, Algorithm::kComputedTorque);
  ASSERT_TRUE(row.result.has_value());
  EXPECT_EQ(row.result->verdict, bench::Verdict::kMarginal);
  EXPECT_TRUE(row.result->quick);
  EXPECT_EQ(row.result->counts.runs, 3U);
  EXPECT_EQ(row.result->counts.passed, 3U);
  EXPECT_EQ(row.result->worst.size(), 4U);
  ASSERT_TRUE(row.result->tuning.has_value());
  ASSERT_EQ(row.result->tuning->rows.size(), 1U);  // one level x one seed
  ASSERT_TRUE(row.result->tuning->chosen.has_value());
  EXPECT_EQ(*row.result->tuning->chosen, 50.0);
  EXPECT_EQ(row.result->gain_source, bench::GainSource::kTuned);
  EXPECT_TRUE(row.result->causes.empty());

  // Progress: one pair; "tune" (1 level x 1 seed = 1 run) before "runs" (3 runs), which ends at
  // done == total == 3. The quick mode skips the sensitivity, and a MARGINAL has no ablation.
  ASSERT_FALSE(rec.calls.empty());
  for (const Call& c : rec.calls) {
    EXPECT_EQ(c.pair, 0U) << c.phase;
    EXPECT_EQ(c.pairs, 1U) << c.phase;
    EXPECT_LE(c.done, c.total) << c.phase;
    EXPECT_TRUE(c.phase == "tune" || c.phase == "runs") << c.phase;
  }
  EXPECT_EQ(rec.calls.front().phase, "tune");
  EXPECT_EQ(rec.calls.front().total, 1U);
  EXPECT_EQ(rec.calls.back().phase, "runs");
  EXPECT_EQ(rec.calls.back().done, 3U);
  EXPECT_EQ(rec.calls.back().total, 3U);
}

TEST(StudioVerifyJob, TwoModelsWriteTheCliBytesIncludingTheComparison) {
  COBOT_RELEASE_ONLY();
  const fs::path dir = test_dir();
  const fs::path out = dir / "job";
  VerifyRequest r = request({"premium_fr3", "budget_xarm7"}, {Algorithm::kComputedTorque}, out);
  make_one_untuned(r);
  Recorder rec;
  const std::atomic<bool> cancel{false};
  const auto job = run_verify_job(r, cancel, rec.fn());
  ASSERT_TRUE(job.is_ok()) << job.reason();

  const Cli cli =
      run_bench(cli_args({"premium_fr3", "budget_xarm7"}, "computed_torque", dir / "cli") +
                " --runs 1 --no-tune");
  ASSERT_EQ(cli.exit_code, 0) << cli.output;
  expect_same_files(out,
                    dir / "cli",
                    job.value(),
                    {"budget_xarm7__computed_torque/report.json",
                     "budget_xarm7__computed_torque/report.md",
                     "comparison.json",
                     "comparison.md",
                     "premium_fr3__computed_torque/report.json",
                     "premium_fr3__computed_torque/report.md"});

  // tests/unit/bench_verify_analysis/test_analysis_cli.cpp, by hand: premium 1/1, quick ->
  // MARGINAL; budget CT at its own omega 20 holds, 0/1, wilson(0, 1).hi = 3.8415 / 4.8415 =
  // 0.793 < 0.95 -> FAIL, with causes (the ablation ran).
  ASSERT_EQ(job.value().rows.size(), 2U);
  EXPECT_EQ(job.value().rows[0].model, "premium_fr3");
  EXPECT_EQ(job.value().rows[1].model, "budget_xarm7");
  ASSERT_TRUE(job.value().rows[0].result.has_value());
  ASSERT_TRUE(job.value().rows[1].result.has_value());
  EXPECT_EQ(job.value().rows[0].result->verdict, bench::Verdict::kMarginal);
  EXPECT_EQ(job.value().rows[1].result->verdict, bench::Verdict::kFail);
  EXPECT_FALSE(job.value().rows[1].result->causes.empty());

  // Progress: two listed pairs, the first one's calls before the second one's.
  ASSERT_FALSE(rec.calls.empty());
  std::size_t last_pair = 0;
  for (const Call& c : rec.calls) {
    EXPECT_EQ(c.pairs, 2U) << c.phase;
    EXPECT_LT(c.pair, 2U) << c.phase;
    EXPECT_GE(c.pair, last_pair) << c.phase;
    last_pair = c.pair;
  }
  EXPECT_EQ(rec.calls.front().pair, 0U);
  EXPECT_EQ(rec.calls.back().pair, 1U);
}

TEST(StudioVerifyJob, APairTheModelDoesNotListIsInTheComparisonOnly) {
  COBOT_RELEASE_ONLY();
  const fs::path dir = test_dir();
  const fs::path out = dir / "job";
  // premium_fr3 lists computed_torque and pd_gravity, not drive_position: two pairs in the cross
  // product, one of them runs.
  VerifyRequest r =
      request({"premium_fr3"}, {Algorithm::kComputedTorque, Algorithm::kDrivePosition}, out);
  make_one_untuned(r);
  Recorder rec;
  const std::atomic<bool> cancel{false};
  const auto job = run_verify_job(r, cancel, rec.fn());
  ASSERT_TRUE(job.is_ok()) << job.reason();

  const Cli cli =
      run_bench(cli_args({"premium_fr3"}, "computed_torque,drive_position", dir / "cli") +
                " --runs 1 --no-tune");
  ASSERT_EQ(cli.exit_code, 0) << cli.output;
  expect_same_files(out,
                    dir / "cli",
                    job.value(),
                    {"comparison.json",
                     "comparison.md",
                     "premium_fr3__computed_torque/report.json",
                     "premium_fr3__computed_torque/report.md"});
  EXPECT_TRUE(contains(read_text(out / "comparison.md"), "not listed"));

  ASSERT_EQ(job.value().rows.size(), 2U);
  EXPECT_TRUE(job.value().rows[0].result.has_value());
  EXPECT_EQ(job.value().rows[1].algorithm, Algorithm::kDrivePosition);
  EXPECT_FALSE(job.value().rows[1].result.has_value());
  // Only the listed pair runs: pair 0 of 1.
  ASSERT_FALSE(rec.calls.empty());
  for (const Call& c : rec.calls) {
    EXPECT_EQ(c.pair, 0U) << c.phase;
    EXPECT_EQ(c.pairs, 1U) << c.phase;
  }
}

TEST(StudioVerifyJob, CancelBetweenTwoPairsWritesNothing) {
  COBOT_RELEASE_ONLY();
  const fs::path out = test_dir() / "job";
  VerifyRequest r = request({"premium_fr3", "budget_xarm7"}, {Algorithm::kComputedTorque}, out);
  make_one_untuned(r);
  std::atomic<bool> cancel{false};
  std::size_t second_pair_calls = 0;
  const VerifyProgressFn progress =
      [&](std::string_view, std::size_t pair, std::size_t, std::size_t, std::size_t) {
        // The first pair is done once the second one reports: cancel there. The writer runs only
        // after every run, so the first pair's report must not appear either.
        if (pair == 1) {
          ++second_pair_calls;
          cancel.store(true);
        }
      };
  const auto job = run_verify_job(r, cancel, progress);
  ASSERT_FALSE(job.is_ok());
  EXPECT_EQ(job.reason(), "cancelled");
  EXPECT_GE(second_pair_calls, 1U);
  EXPECT_TRUE(files_under(out).empty());
}

}  // namespace
}  // namespace cobot::studio

#else  // before the implementation: one failing test instead of a missing-header error

#include <gtest/gtest.h>

TEST(StudioVerifyJob, HeaderExists) {
  FAIL() << "src/studio/core/include/cobot/studio/verify_job.hpp is missing (feature "
            "studio-verify)";
}

#endif
