// bench::verify with the analysis of PR 6b on real runs (brief verify-analysis, sections 1, 2 and
// 4). Each run is 12000 elastic control cycles (about 3.5 s per torque run on one core, #78):
// release builds only (kOptimised), each test under about a minute with 3 jobs.
//
// Premium sensitivity (test hook VerifyOptions::sensitivity_keys, two keys, 3 seeds): runs 20 is
// quick mode, which normally switches the sensitivity off; the hook runs it anyway. 1 tune run +
// 20 runs + 2 keys x 2 ends x 3 seeds = 33 runs. premium_fr3 CT at omega 50 passes every run of
// reference_move (#78, #81): 20/20, wilson(20, 20).hi = 1 -> not FAIL; quick -> MARGINAL, so no
// ablation. The base of each key is the first 3 runs of the main Monte Carlo (seeds 1..3).
//   sensor.q_offset_std_rad: range [5.0e-5, 1.0e-3] (config/hardware/premium_fr3.yaml)
//   actuator.torque_slew_nm_s: scale [0.5, 2.0]
//
// Budget ablation: budget_xarm7 CT, levels {20}, tune seeds {1}, runs 3, ablation_seeds 2. Nothing
// admissible (#81), the model's omega 20 holds every run by state_timeout: 0/3, wilson(0, 3).hi =
// z^2 / (3 + z^2) = 3.8415 / 6.8415 = 0.5615 < 0.95 -> FAIL. Every one of the 7 groups is active
// in the budget profile: 7 x 2 = 14 ablation runs. CT is a torque law, comm.servo_rate_hz = 250 >
// 0 and the holds are state_timeout -> suggestion 1 first; no admissible gain -> suggestion 6
// last. Runs 3 < 100 is quick: the sensitivity does not run ("not run").
#include <algorithm>
#include <array>
#include <cstddef>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/control/algorithm.hpp"
#include "cobot/pipeline/hardware_model.hpp"
#include "cobot/pipeline/task.hpp"

#include "analysis.hpp"
#include "report.hpp"
#include "verify.hpp"
#include "verify_test_util.hpp"

namespace cobot::bench {
namespace {

using control::Algorithm;
using namespace verify_test;  // NOLINT(google-build-using-namespace)

#define COBOT_RELEASE_ONLY()                                          \
  if (!kOptimised) {                                                  \
    GTEST_SKIP() << "elastic 7-DOF closed loop: release builds only"; \
  }

const std::string kPositionLoop = "needs a position loop in the drive (algorithm drive_position)";
const std::string kNoGain = "no admissible gain in the sweep: widen the levels or change the law";

struct Progress {
  std::mutex mutex;
  std::map<std::string, std::pair<std::size_t, std::size_t>> max;  // phase -> (done, total)

  void record(std::string_view phase, std::size_t done, std::size_t total) {
    const std::lock_guard<std::mutex> lock(mutex);
    auto& m = max[std::string(phase)];
    m.first = std::max(m.first, done);
    m.second = std::max(m.second, total);
    EXPECT_LE(done, total) << phase;
  }
};

VerifyOptions options(std::size_t runs, double level, unsigned jobs) {
  VerifyOptions o;
  o.runs = runs;
  o.seed0 = 1;
  o.quick = false;
  o.tune = true;
  o.tuning.levels = {level};
  o.tuning.seeds = {1};
  o.jobs = jobs;
  return o;
}

Counts first_runs(const VerifyResult& r, std::size_t n) {
  Counts c;
  c.runs = n;
  for (std::size_t i = 0; i < n && i < r.runs.size(); ++i) {
    c.passed += r.runs[i].passed ? 1 : 0;
    c.held += r.runs[i].summary.held ? 1 : 0;
  }
  return c;
}

TEST(AnalysisRuns, PremiumSensitivityOnTwoKeys) {
  COBOT_RELEASE_ONLY();
  const pipeline::HardwareModel model = load_model("premium_fr3");
  const pipeline::Task task = load_task("reference_move");
  VerifyOptions o = options(20, 50.0, 3);
  o.sensitivity = true;
  o.sensitivity_seeds = 3;
  const std::vector<std::string> keys = {"sensor.q_offset_std_rad", "actuator.torque_slew_nm_s"};
  o.sensitivity_keys = keys;
  Progress progress;
  o.progress = [&progress](std::string_view phase, std::size_t done, std::size_t total) {
    progress.record(phase, done, total);
  };
  const auto res = verify(model, Algorithm::kComputedTorque, task, o);
  ASSERT_TRUE(res.is_ok()) << res.reason();
  const VerifyResult& r = res.value();
  ASSERT_EQ(r.runs.size(), 20U);
  EXPECT_TRUE(r.verdict == Verdict::kMarginal);
  EXPECT_TRUE(r.causes.empty());
  EXPECT_TRUE(r.suggestions.empty());

  const Counts base = first_runs(r, 3);
  std::size_t ran = 0;
  for (const Assumption& a : r.assumptions) {
    SCOPED_TRACE(a.key);
    if (std::find(keys.begin(), keys.end(), a.key) == keys.end()) {
      EXPECT_EQ(a.sensitivity, "not run");
      continue;
    }
    ++ran;
    EXPECT_TRUE(a.sensitivity == "stable" || a.sensitivity == "may change") << a.sensitivity;
    ASSERT_TRUE(a.base.has_value());
    EXPECT_EQ(a.base->passed, base.passed);
    EXPECT_EQ(a.base->held, base.held);
    ASSERT_TRUE(a.low.has_value());
    ASSERT_TRUE(a.high.has_value());
    EXPECT_EQ(a.low->counts.runs, 3U);
    EXPECT_EQ(a.high->counts.runs, 3U);
    EXPECT_LE(a.low->counts.passed + a.low->counts.held, 3U);
    EXPECT_LE(a.high->counts.passed + a.high->counts.held, 3U);
    const std::array<double, 2> bounds = a.range ? *a.range : *a.scale;
    EXPECT_EQ(a.low->value, bounds[0]);  // the bound itself, or the scale factor
    EXPECT_EQ(a.high->value, bounds[1]);
    // The pure rule decides.
    const Sensitivity rule = sensitivity_rule(r.counts,
                                              *a.base,
                                              std::array<Counts, 2>{a.low->counts, a.high->counts},
                                              task.requirements.target_pass_rate);
    EXPECT_EQ(a.sensitivity, to_string(rule));
  }
  EXPECT_EQ(ran, 2U);

  // 2 keys x 2 ends x 3 seeds.
  ASSERT_EQ(progress.max.count("sensitivity"), 1U);
  EXPECT_EQ(progress.max["sensitivity"].second, 12U);
  EXPECT_EQ(progress.max["sensitivity"].first, 12U);
  EXPECT_TRUE(progress.max.count("ablation") == 0 || progress.max["ablation"].second == 0);

  // The report entries of the two keys.
  const std::string json = report_json(r, {});
  const std::optional<Json> doc = parse_json(json);
  ASSERT_TRUE(doc.has_value()) << json;
  const Json* list = doc->find("assumptions");
  ASSERT_NE(list, nullptr);
  std::size_t seen = 0;
  for (const Json& e : list->items) {
    if (std::find(keys.begin(), keys.end(), e.find("key")->text) == keys.end()) {
      continue;
    }
    ++seen;
    ASSERT_NE(e.find("base"), nullptr);
    EXPECT_EQ(e.find("base")->find("passed")->text, std::to_string(base.passed));
    ASSERT_NE(e.find("low"), nullptr);
    ASSERT_NE(e.find("high"), nullptr);
  }
  EXPECT_EQ(seen, 2U);
}

TEST(AnalysisRuns, BudgetFailHasCausesAndThePositionLoopSuggestion) {
  COBOT_RELEASE_ONLY();
  const pipeline::HardwareModel model = load_model("budget_xarm7");
  const pipeline::Task task = load_task("reference_move");
  VerifyOptions o = options(3, 20.0, 3);
  o.ablation_seeds = 2;
  Progress progress;
  o.progress = [&progress](std::string_view phase, std::size_t done, std::size_t total) {
    progress.record(phase, done, total);
  };
  const auto res = verify(model, Algorithm::kComputedTorque, task, o);
  ASSERT_TRUE(res.is_ok()) << res.reason();
  const VerifyResult& r = res.value();
  ASSERT_TRUE(r.verdict == Verdict::kFail);
  EXPECT_TRUE(r.gain_source == GainSource::kDefaultNoAdmissible);

  // One ablation row per active group, in the groups' order, against the first 2 runs.
  ASSERT_EQ(r.causes.size(), kAblationGroups.size());
  const Counts base = first_runs(r, 2);
  for (std::size_t i = 0; i < r.causes.size(); ++i) {
    EXPECT_EQ(r.causes[i].group, kAblationGroups[i]);
    EXPECT_EQ(r.causes[i].base_passed, base.passed) << r.causes[i].group;
    EXPECT_LE(r.causes[i].passed_without, 2U) << r.causes[i].group;
  }
  std::vector<Cause> marked = r.causes;
  const bool any_main = mark_main_causes(marked);
  std::vector<std::string> main_groups;
  for (std::size_t i = 0; i < r.causes.size(); ++i) {
    EXPECT_EQ(r.causes[i].main, marked[i].main) << r.causes[i].group;
    if (marked[i].main) {
      main_groups.push_back(marked[i].group);
    }
  }
  EXPECT_EQ(any_main, !main_groups.empty());

  // The hold reasons of the main runs, counted.
  std::size_t holds = 0;
  for (const auto& [reason, n] : r.hold_reasons) {
    holds += n;
  }
  EXPECT_EQ(holds, r.counts.held);

  ASSERT_FALSE(r.suggestions.empty());
  EXPECT_EQ(r.suggestions.front(), kPositionLoop);
  EXPECT_EQ(r.suggestions.back(), kNoGain);
  SuggestionFacts facts;
  facts.algorithm = Algorithm::kComputedTorque;
  facts.servo_rate_hz = 250.0;  // comm.servo_rate_hz of config/imperfections/budget_xarm7.yaml
  facts.hold_reasons = r.hold_reasons;
  facts.main_causes = main_groups;
  facts.no_admissible_gain = true;
  EXPECT_EQ(r.suggestions, suggest(facts));

  // 7 groups x 2 seeds; quick mode: no sensitivity.
  ASSERT_EQ(progress.max.count("ablation"), 1U);
  EXPECT_EQ(progress.max["ablation"].second, 14U);
  EXPECT_EQ(progress.max["ablation"].first, 14U);
  EXPECT_TRUE(progress.max.count("sensitivity") == 0 || progress.max["sensitivity"].second == 0);
  for (const Assumption& a : r.assumptions) {
    EXPECT_EQ(a.sensitivity, "not run") << a.key;
  }

  const std::string md = report_md(r, {});
  EXPECT_TRUE(contains(md, kPositionLoop)) << md;
}

}  // namespace
}  // namespace cobot::bench
