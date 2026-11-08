// report.json and report.md with the analysis of PR 6b (brief verify-analysis, section 4), on
// hand-made VerifyResult values:
//  - the top-level keys keep 6a's order; new keys only inside assumptions[] (and causes);
//  - a key that ran: {key, kind, range | scale, sensitivity, base: {passed, held}, low: {value,
//    passed, held}, high: {value, passed, held}} in this order, value the bound or the factor;
//    a bool key "not varied", a key that did not run "not run";
//  - the MD's assumption table shows the low / high pass counts and the stable column; a FAIL
//    has a causes section (every group, the hold reasons, the main cause or "no single group
//    explains the failure") and a suggestions section; the JSON suggestions are the lines;
//  - a result that is not a FAIL keeps causes and suggestions [] in the JSON.
#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/control/algorithm.hpp"
#include "cobot/pipeline/hardware_model.hpp"

#include "analysis.hpp"
#include "report.hpp"
#include "verify.hpp"
#include "verify_test_util.hpp"

namespace cobot::bench {
namespace {

using namespace verify_test;  // NOLINT(google-build-using-namespace)

const std::vector<std::string> kTopKeys = {"schema",
                                           "disclaimer",
                                           "verdict",
                                           "quick",
                                           "runs",
                                           "seeds",
                                           "pass_count",
                                           "pass_rate",
                                           "wilson",
                                           "hold_rate",
                                           "worst",
                                           "tuning",
                                           "assumptions",
                                           "causes",
                                           "suggestions",
                                           "versions"};

const std::string kPositionLoop = "needs a position loop in the drive (algorithm drive_position)";
const std::string kNoGain = "no admissible gain in the sweep: widen the levels or change the law";

Counts counts(std::size_t runs, std::size_t passed, std::size_t held) {
  Counts c;
  c.runs = runs;
  c.passed = passed;
  c.held = held;
  return c;
}

SensitivityEnd end(double value, std::size_t passed, std::size_t held) {
  SensitivityEnd e;
  e.value = value;
  e.counts = counts(20, passed, held);
  return e;
}

SeedRun seed_run(std::uint64_t seed, bool passed, bool held) {
  SeedRun r;
  r.seed = seed;
  r.summary.held = held;
  r.summary.final_pos_err_m = 1e-4;
  r.summary.final_rot_err_rad = 1e-3;
  r.summary.rms_joint_err_rad = 1e-3;
  r.summary.settle_time_s = 10.5;
  r.passed = passed;
  return r;
}

// 100 runs (seeds 1..100), `passed` of them pass, `held` hold; sensitivity on three keys.
VerifyResult sample(Verdict v, std::size_t passed, std::size_t held) {
  VerifyResult r;
  r.algorithm = control::Algorithm::kComputedTorque;
  r.model = "budget_xarm7";
  r.task = "reference_move";
  r.gain_key = "config.computed_torque.omega";
  r.gain_used = 20.0;
  r.gain_source = GainSource::kDefaultTuningOff;
  for (std::size_t i = 0; i < 100; ++i) {
    r.runs.push_back(seed_run(i + 1, i < passed, i >= 100 - held));
  }
  r.counts = counts(100, passed, held);
  r.pass_rate = static_cast<double>(passed) / 100.0;
  r.hold_rate = static_cast<double>(held) / 100.0;
  r.wilson = wilson(passed, 100);
  for (const char* name :
       {"final_pos_err_m", "final_rot_err_deg", "rms_joint_err_rad", "cycle_time_s"}) {
    r.worst.push_back(WorstMetric{name, 1e-4, 1.0, 0.8});
  }
  r.verdict = v;
  r.quick = false;

  Assumption ranged;
  ranged.key = "actuator.bus.r_source_ohm";
  ranged.kind = pipeline::ProvenanceKind::kAssumed;
  ranged.range = std::array<double, 2>{0.05, 0.5};
  ranged.sensitivity = "stable";
  ranged.base = counts(20, 17, 3);
  ranged.low = end(0.05, 18, 2);
  ranged.high = end(0.5, 11, 9);

  Assumption scaled;
  scaled.key = "friction.coulomb_nm";
  scaled.kind = pipeline::ProvenanceKind::kDerived;
  scaled.scale = std::array<double, 2>{0.5, 1.5};
  scaled.sensitivity = "may change";
  scaled.base = counts(20, 17, 3);
  scaled.low = end(0.5, 20, 0);
  scaled.high = end(1.5, 4, 16);

  Assumption flag;
  flag.key = "sensor.qd_from_diff";
  flag.kind = pipeline::ProvenanceKind::kAssumed;
  flag.sensitivity = "not varied";

  r.assumptions = {ranged, scaled, flag};
  return r;
}

VerifyResult failing() {
  VerifyResult r = sample(Verdict::kFail, 17, 83);
  Cause comm;
  comm.group = "comm";
  comm.passed_without = 19;
  comm.base_passed = 3;
  comm.main = true;
  Cause sensor;
  sensor.group = "sensor";
  sensor.passed_without = 5;
  sensor.base_passed = 3;
  sensor.main = false;
  r.causes = {sensor, comm};
  r.hold_reasons = {{"state_timeout", 80}, {"overspeed", 3}};
  r.suggestions = {kPositionLoop, "raise the servo rate or reduce packet loss", kNoGain};
  return r;
}

const Json* entry(const Json& doc, std::size_t i) {
  const Json* a = doc.find("assumptions");
  if (a == nullptr || i >= a->items.size()) {
    return nullptr;
  }
  return &a->items[i];
}

TEST(ReportAnalysis, TopLevelKeysKeepTheirOrder) {
  for (const VerifyResult& r : {sample(Verdict::kMarginal, 92, 1), failing()}) {
    const std::string json = report_json(r, {});
    const std::optional<Json> doc = parse_json(json);
    ASSERT_TRUE(doc.has_value()) << json;
    std::vector<std::string> in_order;
    for (const std::string& k : doc->keys) {
      for (const std::string& t : kTopKeys) {
        if (k == t) {
          in_order.push_back(k);
        }
      }
    }
    EXPECT_EQ(in_order, kTopKeys) << json;
    for (const char* name : {"model", "algorithm", "task"}) {
      EXPECT_NE(doc->find(name), nullptr) << name;
    }
  }
}

TEST(ReportAnalysis, AKeyThatRanHasBaseLowAndHigh) {
  const std::string json = report_json(sample(Verdict::kMarginal, 92, 1), {});
  const std::optional<Json> doc = parse_json(json);
  ASSERT_TRUE(doc.has_value()) << json;

  const Json* ranged = entry(*doc, 0);
  ASSERT_NE(ranged, nullptr) << json;
  EXPECT_EQ(
      ranged->keys,
      (std::vector<std::string>{"key", "kind", "range", "sensitivity", "base", "low", "high"}));
  EXPECT_EQ(ranged->find("key")->text, "actuator.bus.r_source_ohm");
  EXPECT_EQ(ranged->find("sensitivity")->text, "stable");
  const Json* base = ranged->find("base");
  ASSERT_NE(base, nullptr);
  EXPECT_EQ(base->keys, (std::vector<std::string>{"passed", "held"}));
  EXPECT_EQ(base->find("passed")->text, "17");
  EXPECT_EQ(base->find("held")->text, "3");
  const Json* low = ranged->find("low");
  ASSERT_NE(low, nullptr);
  EXPECT_EQ(low->keys, (std::vector<std::string>{"value", "passed", "held"}));
  EXPECT_EQ(low->find("value")->text, g17(0.05));
  EXPECT_EQ(low->find("passed")->text, "18");
  EXPECT_EQ(low->find("held")->text, "2");
  const Json* high = ranged->find("high");
  ASSERT_NE(high, nullptr);
  EXPECT_EQ(high->find("value")->text, g17(0.5));
  EXPECT_EQ(high->find("passed")->text, "11");
  EXPECT_EQ(high->find("held")->text, "9");

  const Json* scaled = entry(*doc, 1);
  ASSERT_NE(scaled, nullptr);
  EXPECT_EQ(
      scaled->keys,
      (std::vector<std::string>{"key", "kind", "scale", "sensitivity", "base", "low", "high"}));
  EXPECT_EQ(scaled->find("sensitivity")->text, "may change");
  EXPECT_EQ(scaled->find("low")->find("value")->text, g17(0.5));  // the factor
  EXPECT_EQ(scaled->find("high")->find("value")->text, g17(1.5));
  EXPECT_EQ(scaled->find("high")->find("passed")->text, "4");
  EXPECT_EQ(scaled->find("high")->find("held")->text, "16");

  const Json* flag = entry(*doc, 2);
  ASSERT_NE(flag, nullptr);
  EXPECT_EQ(flag->find("key")->text, "sensor.qd_from_diff");
  EXPECT_EQ(flag->find("sensitivity")->text, "not varied");
}

TEST(ReportAnalysis, NotAFailKeepsCausesAndSuggestionsEmpty) {
  const std::optional<Json> doc = parse_json(report_json(sample(Verdict::kMarginal, 92, 1), {}));
  ASSERT_TRUE(doc.has_value());
  const Json* causes = doc->find("causes");
  ASSERT_NE(causes, nullptr);
  EXPECT_TRUE(causes->items.empty() && causes->keys.empty());
  const Json* suggestions = doc->find("suggestions");
  ASSERT_NE(suggestions, nullptr);
  ASSERT_TRUE(suggestions->is(Json::Type::kArray));
  EXPECT_TRUE(suggestions->items.empty());
}

TEST(ReportAnalysis, AFailCarriesCausesHoldReasonsAndSuggestions) {
  const VerifyResult r = failing();
  const std::string json = report_json(r, {});
  const std::optional<Json> doc = parse_json(json);
  ASSERT_TRUE(doc.has_value()) << json;
  const Json* suggestions = doc->find("suggestions");
  ASSERT_NE(suggestions, nullptr);
  ASSERT_EQ(suggestions->items.size(), 3U) << json;
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(suggestions->items[i].text, r.suggestions[i]);
  }
  const Json* causes = doc->find("causes");
  ASSERT_NE(causes, nullptr);
  EXPECT_FALSE(causes->items.empty() && causes->keys.empty()) << json;
  for (const char* text : {"\"comm\"",
                           "\"sensor\"",
                           "\"passed_without\"",
                           "\"base_passed\"",
                           "\"main\"",
                           "\"hold_reasons\"",
                           "\"state_timeout\"",
                           "\"overspeed\""}) {
    EXPECT_TRUE(contains(json, text)) << text << "\n" << json;
  }

  const std::string md = report_md(r, {});
  EXPECT_EQ(first_line(md), kDisclaimer);
  for (const std::string& line : r.suggestions) {
    EXPECT_TRUE(contains(md, line)) << line << "\n" << md;
  }
  for (const char* text : {"comm", "sensor", "state_timeout", "80", "overspeed", "may change"}) {
    EXPECT_TRUE(contains(md, text)) << text << "\n" << md;
  }
  EXPECT_FALSE(contains(md, "no single group explains the failure")) << md;
}

TEST(ReportAnalysis, AFailWithoutMainCauseSaysSo) {
  VerifyResult r = failing();
  for (Cause& c : r.causes) {
    c.main = false;
  }
  r.suggestions = {kNoGain};
  const std::string md = report_md(r, {});
  EXPECT_TRUE(contains(md, "no single group explains the failure")) << md;
}

TEST(ReportAnalysis, TheAssumptionTableShowsTheEnds) {
  const std::string md = report_md(sample(Verdict::kMarginal, 92, 1), {});
  for (const char* text : {"actuator.bus.r_source_ohm",
                           "friction.coulomb_nm",
                           "sensor.qd_from_diff",
                           "stable",
                           "may change",
                           "not varied",
                           "18",
                           "11"}) {
    EXPECT_TRUE(contains(md, text)) << text << "\n" << md;
  }
}

}  // namespace
}  // namespace cobot::bench
