// bench::verify on real runs (brief verify, section 4; ADR-023): the tune step, the Monte Carlo
// runs by seed, the verdict and the report of the result. Each run is 12000 elastic control cycles:
// release builds only (kOptimised), as tests/unit/bench_tune. The errors that need no run are
// checked in every build.
//
// Expected from the numbers of #78 (tests/unit/bench_tune/test_tune_runs.cpp), reference_move:
//   premium_fr3 CT omega 50: no hold, rms about 3e-4 rad (requirement 0.02), final pos about
//     1e-4 m (0.005): every run passes; 5 runs is quick mode, so MARGINAL (wilson(5, 5).hi = 1, not
//     FAIL; quick caps PASS).
//   budget_xarm7 CT omega 20 seed 1 holds by state_timeout: nothing admissible with levels {20},
//     the model's own gain (config.computed_torque.omega 20 in config/hardware/budget_xarm7.yaml)
//     holds every run (plan ambiguity 6), 0/3: wilson(0, 3).hi = z^2 / (3 + z^2) = 0.5615 < 0.95 ->
//     FAIL; no run left to take a worst value from, so every worst value is NaN (null in JSON).
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/control/algorithm.hpp"
#include "cobot/pipeline/hardware_model.hpp"
#include "cobot/pipeline/task.hpp"

#include "report.hpp"
#include "tune.hpp"
#include "verify.hpp"
#include "verify_test_util.hpp"

namespace cobot::bench {
namespace {

using control::Algorithm;
using namespace verify_test;  // NOLINT(google-build-using-namespace)

constexpr double kPi = 3.14159265358979323846;

#define COBOT_RELEASE_ONLY()                                          \
  if (!kOptimised) {                                                  \
    GTEST_SKIP() << "elastic 7-DOF closed loop: release builds only"; \
  }

VerifyOptions options(std::size_t runs,
                      std::vector<double> levels,
                      std::vector<std::uint64_t> tune_seeds,
                      bool quick,
                      unsigned jobs) {
  VerifyOptions o;
  o.runs = runs;
  o.seed0 = 1;
  o.quick = quick;
  o.tune = true;
  o.tuning.levels = std::move(levels);
  o.tuning.seeds = std::move(tune_seeds);
  o.tuning.jobs = jobs;
  o.jobs = jobs;
  return o;
}

// The model's own gain for `algorithm`: the value of tuning_key in its override list.
std::optional<double> model_gain(const pipeline::HardwareModel& model, Algorithm algorithm) {
  const auto it = model.algorithms.find(algorithm);
  if (it == model.algorithms.end()) {
    return std::nullopt;
  }
  for (const pipeline::KeyOverride& o : it->second) {
    if (o.key == tuning_key(algorithm) && std::holds_alternative<double>(o.value)) {
      return std::get<double>(o.value);
    }
  }
  return std::nullopt;
}

// Every provenance entry of kind assumed or derived, in file order.
std::vector<pipeline::ProvenanceEntry> assumed_or_derived(const pipeline::HardwareModel& model) {
  std::vector<pipeline::ProvenanceEntry> out;
  for (const pipeline::ProvenanceEntry& e : model.provenance) {
    if (e.kind != pipeline::ProvenanceKind::kPublished) {
      out.push_back(e);
    }
  }
  return out;
}

struct Progress {
  std::string phase;
  std::size_t done = 0;
  std::size_t total = 0;
};

TEST(VerifyErrors, AnUnlistedAlgorithmIsAnErrorBeforeAnyRun) {
  // premium_fr3 lists computed_torque and pd_gravity only.
  for (const bool tune : {true, false}) {
    VerifyOptions o = options(100, {50.0}, {1}, false, 1);
    o.tune = tune;
    std::size_t calls_with_work = 0;
    o.progress = [&calls_with_work](std::string_view, std::size_t done, std::size_t) {
      calls_with_work += done > 0 ? 1 : 0;
    };
    const auto r = verify(
        load_model("premium_fr3"), Algorithm::kDrivePosition, load_task("reference_move"), o);
    ASSERT_FALSE(r.is_ok()) << tune;
    EXPECT_TRUE(contains(r.reason(), "premium_fr3")) << r.reason();
    EXPECT_EQ(calls_with_work, 0U) << tune;
  }
}

TEST(VerifyRuns, PremiumComputedTorqueQuickIsMarginalWithEveryRunPassing) {
  COBOT_RELEASE_ONLY();
  const pipeline::HardwareModel model = load_model("premium_fr3");
  const pipeline::Task task = load_task("reference_move");
  VerifyOptions o = options(5, {50.0}, {1}, true, 3);
  std::mutex mutex;
  std::vector<Progress> calls;
  o.progress = [&mutex, &calls](std::string_view phase, std::size_t done, std::size_t total) {
    const std::lock_guard<std::mutex> lock(mutex);
    Progress p{std::string(phase), done, total};
    calls.push_back(std::move(p));
  };
  const auto res = verify(model, Algorithm::kComputedTorque, task, o);
  ASSERT_TRUE(res.is_ok()) << res.reason();
  const VerifyResult& r = res.value();

  EXPECT_TRUE(r.algorithm == Algorithm::kComputedTorque);
  EXPECT_EQ(r.model, "premium_fr3");
  EXPECT_EQ(r.task, "reference_move");
  EXPECT_EQ(r.gain_key, "config.computed_torque.omega");
  EXPECT_TRUE(r.gain_source == GainSource::kTuned);
  EXPECT_EQ(r.gain_used, 50.0);
  ASSERT_TRUE(r.tuning.has_value());
  ASSERT_TRUE(r.tuning->chosen.has_value()) << r.tuning->reason;
  EXPECT_EQ(*r.tuning->chosen, 50.0);
  EXPECT_EQ(r.tuning->levels, (std::vector<double>{50.0}));
  ASSERT_EQ(r.tuning->rows.size(), 1U);
  EXPECT_EQ(r.tuning->rows[0].seeds, 1U);

  EXPECT_TRUE(r.quick);
  EXPECT_TRUE(r.verdict == Verdict::kMarginal);
  EXPECT_EQ(r.counts.runs, 5U);
  EXPECT_EQ(r.counts.passed, 5U);
  EXPECT_EQ(r.counts.held, 0U);
  EXPECT_EQ(r.pass_rate, 1.0);
  EXPECT_EQ(r.hold_rate, 0.0);
  EXPECT_EQ(r.wilson.lo, wilson(5, 5).lo);
  EXPECT_EQ(r.wilson.hi, wilson(5, 5).hi);

  // Seeds 1..5 in seed order, whatever the scheduling; each judged by run_passes.
  ASSERT_EQ(r.runs.size(), 5U);
  const pipeline::TaskRequirements& req = task.requirements;
  for (std::size_t i = 0; i < r.runs.size(); ++i) {
    const SeedRun& s = r.runs[i];
    EXPECT_EQ(s.seed, i + 1);
    EXPECT_FALSE(s.summary.held) << s.seed;
    EXPECT_EQ(s.passed, run_passes(s.summary, req)) << s.seed;
    EXPECT_TRUE(s.passed) << s.seed;
    // Settles before the evaluation time (trajectory 10 s + hold 2 s).
    EXPECT_TRUE(std::isfinite(s.summary.settle_time_s)) << s.seed;
    EXPECT_GT(s.summary.settle_time_s, 0.0) << s.seed;
    EXPECT_LE(s.summary.settle_time_s, req.cycle_time_s) << s.seed;
  }

  // Worst over the runs that did not hold, in the report's order; limit = (1 - margin) threshold.
  ASSERT_EQ(r.worst.size(), 4U);
  double pos = 0.0;
  double rot_deg = 0.0;
  double rms = 0.0;
  double settle = 0.0;
  for (const SeedRun& s : r.runs) {
    pos = std::max(pos, s.summary.final_pos_err_m);
    rot_deg = std::max(rot_deg, s.summary.final_rot_err_rad * 180.0 / kPi);
    rms = std::max(rms, s.summary.rms_joint_err_rad);
    settle = std::max(settle, s.summary.settle_time_s);
  }
  const double m = 1.0 - req.margin_frac;
  const struct {
    const char* name;
    double value;
    double threshold;
  } expected[] = {
      {"final_pos_err_m", pos, req.final_pos_err_m},
      {"final_rot_err_deg", rot_deg, req.final_rot_err_deg},
      {"rms_joint_err_rad", rms, req.rms_joint_err_rad},
      {"cycle_time_s", settle, req.cycle_time_s},
  };
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_EQ(r.worst[i].name, expected[i].name);
    EXPECT_NEAR(r.worst[i].value, expected[i].value, 1e-12 * (1.0 + expected[i].value))
        << expected[i].name;
    EXPECT_EQ(r.worst[i].threshold, expected[i].threshold) << expected[i].name;
    EXPECT_NEAR(r.worst[i].margin_limit, m * expected[i].threshold, 1e-15 * expected[i].threshold)
        << expected[i].name;
    EXPECT_LE(r.worst[i].value, r.worst[i].threshold) << expected[i].name;
  }
  EXPECT_TRUE(r.verdict == verdict(r.counts, r.worst, req, true));

  // The seam of 6b.
  const std::vector<pipeline::ProvenanceEntry> entries = assumed_or_derived(model);
  ASSERT_EQ(r.assumptions.size(), entries.size());
  for (std::size_t i = 0; i < entries.size(); ++i) {
    EXPECT_EQ(r.assumptions[i].key, entries[i].key);
    EXPECT_TRUE(r.assumptions[i].kind == entries[i].kind) << entries[i].key;
    EXPECT_EQ(r.assumptions[i].range, entries[i].range) << entries[i].key;
    EXPECT_EQ(r.assumptions[i].scale, entries[i].scale) << entries[i].key;
    EXPECT_EQ(r.assumptions[i].sensitivity, "not run") << entries[i].key;
  }
  EXPECT_TRUE(r.causes.empty());
  EXPECT_TRUE(r.suggestions.empty());

  // Progress: never past its total, and the Monte Carlo phase reaches 5 of 5.
  bool runs_done = false;
  for (const Progress& p : calls) {
    EXPECT_LE(p.done, p.total) << p.phase;
    runs_done = runs_done || (p.total == 5 && p.done == 5);
  }
  EXPECT_TRUE(runs_done);

  // The report of this result.
  const std::optional<Json> doc = parse_json(report_json(r, {}));
  ASSERT_TRUE(doc.has_value());
  EXPECT_EQ(doc->find("verdict")->text, "marginal");
  EXPECT_EQ(doc->find("pass_count")->number, 5.0);
  EXPECT_TRUE(doc->find("quick")->boolean);
  EXPECT_EQ(doc->find("tuning")->find("gain_source")->text, "tuned");
}

TEST(VerifyRuns, BudgetComputedTorqueFailsWithEveryRunHeld) {
  COBOT_RELEASE_ONLY();
  const pipeline::HardwareModel model = load_model("budget_xarm7");
  const pipeline::Task task = load_task("reference_move");
  // Not --quick: 3 runs < 100 is quick mode all the same.
  const auto res =
      verify(model, Algorithm::kComputedTorque, task, options(3, {20.0}, {1}, false, 3));
  ASSERT_TRUE(res.is_ok()) << res.reason();
  const VerifyResult& r = res.value();

  EXPECT_TRUE(r.verdict == Verdict::kFail);
  EXPECT_TRUE(r.quick);
  EXPECT_EQ(r.counts.runs, 3U);
  EXPECT_EQ(r.counts.passed, 0U);
  EXPECT_EQ(r.counts.held, 3U);
  EXPECT_EQ(r.hold_rate, 1.0);
  EXPECT_EQ(r.pass_rate, 0.0);
  ASSERT_EQ(r.runs.size(), 3U);
  for (const SeedRun& s : r.runs) {
    EXPECT_TRUE(s.summary.held) << s.seed;
    EXPECT_FALSE(s.passed) << s.seed;
    EXPECT_TRUE(std::isnan(s.summary.settle_time_s)) << s.seed;
  }

  EXPECT_TRUE(r.gain_source == GainSource::kDefaultNoAdmissible);
  const std::optional<double> own = model_gain(model, Algorithm::kComputedTorque);
  ASSERT_TRUE(own.has_value());
  EXPECT_EQ(r.gain_used, *own);  // 20
  ASSERT_TRUE(r.tuning.has_value());
  EXPECT_FALSE(r.tuning->chosen.has_value());
  EXPECT_TRUE(contains(r.tuning->reason, "no admissible gain")) << r.tuning->reason;

  ASSERT_EQ(r.worst.size(), 4U);
  for (const WorstMetric& w : r.worst) {
    EXPECT_TRUE(std::isnan(w.value)) << w.name;
  }

  const std::string json = report_json(r, {});
  const std::optional<Json> doc = parse_json(json);
  ASSERT_TRUE(doc.has_value()) << json;
  EXPECT_EQ(doc->find("verdict")->text, "fail");
  const Json* worst = doc->find("worst");
  ASSERT_NE(worst, nullptr);
  ASSERT_EQ(worst->values.size(), 4U);
  for (std::size_t i = 0; i < worst->values.size(); ++i) {
    const Json* value = worst->values[i].find("value");
    ASSERT_NE(value, nullptr);
    EXPECT_TRUE(value->is(Json::Type::kNull)) << worst->keys[i] << ": " << value->text;
  }
  EXPECT_EQ(doc->find("tuning")->find("gain_source")->text, "default_no_admissible");
  const std::string md = report_md(r, {});
  EXPECT_TRUE(contains(md, "no admissible gain: verdict on the model's default gain")) << md;
}

TEST(VerifyRuns, CancelSetBeforeTheStartRunsNothing) {
  COBOT_RELEASE_ONLY();
  const std::atomic<bool> cancel{true};
  VerifyOptions o = options(5, {50.0}, {1}, true, 2);
  o.cancel = &cancel;
  std::atomic<std::size_t> work{0};
  o.progress = [&work](std::string_view, std::size_t done, std::size_t) {
    work += done > 0 ? 1 : 0;
  };
  const auto r =
      verify(load_model("premium_fr3"), Algorithm::kComputedTorque, load_task("reference_move"), o);
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(r.reason(), "cancelled")) << r.reason();
  EXPECT_EQ(work.load(), 0U);
}

TEST(VerifyRuns, CancelDuringTheRunsStopsStartingRuns) {
  COBOT_RELEASE_ONLY();
  // One job: the cancel flag set after the first Monte Carlo run is seen before the next starts.
  std::atomic<bool> cancel{false};
  VerifyOptions o = options(20, {50.0}, {1}, true, 1);
  o.cancel = &cancel;
  std::size_t max_done = 0;
  o.progress = [&cancel, &max_done](std::string_view, std::size_t done, std::size_t total) {
    if (total == 20) {
      max_done = std::max(max_done, done);
      if (done >= 1) {
        cancel = true;
      }
    }
  };
  const auto r =
      verify(load_model("premium_fr3"), Algorithm::kComputedTorque, load_task("reference_move"), o);
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(r.reason(), "cancelled")) << r.reason();
  EXPECT_GE(max_done, 1U);
  EXPECT_LE(max_done, 2U);
}

}  // namespace
}  // namespace cobot::bench
