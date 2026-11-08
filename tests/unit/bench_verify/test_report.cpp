// report.json and report.md of a verify result (brief verify, section 5; plan "Data contracts",
// Verify report), on hand-made VerifyResult values so every field is known:
//  - JSON keys exactly as the contract and in its order: schema, disclaimer, verdict, quick, runs,
//    seeds, pass_count, pass_rate, wilson, hold_rate, worst, tuning, assumptions, causes,
//    suggestions, versions, plus model, algorithm and task (anywhere); the nested objects too;
//  - every number is the %.17g text of its value (round trip), NaN is null, the document is
//    strict JSON (no nan, inf or trailing comma);
//  - the disclaimer is the ADR-023 sentence in the JSON and on the first line of the MD, the MD
//    says the verdict ("PASS" / "FAIL" / "MARGINAL, measure on hardware") and "quick mode: never
//    PASS" when quick, and why the gain is the model's when it is;
//  - the 6b seam: assumptions with "sensitivity": "not run", causes and suggestions empty;
//  - report_inputs: the model file, its imperfection and controller files, the task file and its
//    waypoint file, each with the sha256 of its bytes.
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/control/algorithm.hpp"
#include "cobot/pipeline/hardware_model.hpp"

#include "engine.hpp"
#include "report.hpp"
#include "sha256.hpp"
#include "tune.hpp"
#include "verify.hpp"
#include "verify_test_util.hpp"

namespace cobot::bench {
namespace {

using namespace verify_test;  // NOLINT(google-build-using-namespace)
using Type = Json::Type;

const double kNan = std::numeric_limits<double>::quiet_NaN();

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
const std::vector<std::string> kWorstKeys = {
    "final_pos_err_m", "final_rot_err_deg", "rms_joint_err_rad", "cycle_time_s"};
const std::vector<std::string> kTuningKeys = {
    "key", "gain_used", "gain_source", "levels", "rows", "chosen", "reason"};
// tune.csv columns (docs/conventions.md "Tune CSV").
const std::vector<std::string> kTuneRowKeys = {"level",
                                               "seeds",
                                               "held",
                                               "sat_frac_max",
                                               "decays_all",
                                               "rms_joint_err_median",
                                               "final_pos_err_max",
                                               "chosen"};

RunSummary run_summary(double pos, bool held) {
  RunSummary s;
  s.final_pos_err_m = pos;
  s.final_rot_err_rad = 1e-3;
  s.rms_joint_err_rad = 2e-3;
  s.settle_time_s = held ? kNan : 10.5;
  s.held = held;
  return s;
}

SeedRun seed_run(std::uint64_t seed, bool held) {
  SeedRun r;
  r.seed = seed;
  r.summary = run_summary(1e-4 * static_cast<double>(seed), held);
  r.passed = !held;
  return r;
}

TuneResult tune_result() {
  TuneResult t;
  t.algorithm = control::Algorithm::kComputedTorque;
  t.key = "config.computed_torque.omega";
  t.levels = {40.0, 50.0};
  TuneRow a;
  a.level = 40.0;
  a.seeds = 2;
  a.held = 0;
  a.sat_frac_max = 0.0;
  a.decays_all = true;
  a.rms_joint_err_median = 3.1e-4;
  a.final_pos_err_max = 1.2e-4;
  TuneRow b;
  b.level = 50.0;
  b.seeds = 2;
  b.held = 2;
  b.sat_frac_max = 0.01;
  b.decays_all = false;
  b.rms_joint_err_median = kNan;  // no seed without a hold
  b.final_pos_err_max = kNan;
  t.rows = {a, b};
  t.chosen = 40.0;
  t.reason = "chosen 40: lowest median rms";
  return t;
}

Assumption assumption(const char* key,
                      pipeline::ProvenanceKind kind,
                      std::optional<std::array<double, 2>> range,
                      std::optional<std::array<double, 2>> scale) {
  Assumption a;
  a.key = key;
  a.kind = kind;
  a.range = range;
  a.scale = scale;
  a.sensitivity = "not run";
  return a;
}

// Three runs, the third held: counts 3 / 2 / 1; one worst value NaN; one that needs 17 digits.
VerifyResult sample(Verdict v, bool quick, GainSource source) {
  VerifyResult r;
  r.algorithm = control::Algorithm::kComputedTorque;
  r.model = "premium_fr3";
  r.task = "reference_move";
  r.gain_key = "config.computed_torque.omega";
  r.gain_used = source == GainSource::kTuned ? 40.0 : 50.0;
  r.gain_source = source;
  if (source != GainSource::kDefaultTuningOff) {
    r.tuning = tune_result();
    if (source == GainSource::kDefaultNoAdmissible) {
      r.tuning->chosen = std::nullopt;
      r.tuning->reason = "no admissible gain: held 2";
    }
  }
  r.runs = {seed_run(7, false), seed_run(8, false), seed_run(9, true)};
  r.counts = Counts{3, 2, 1};
  r.pass_rate = 2.0 / 3.0;
  r.wilson = wilson(2, 3);
  r.hold_rate = 1.0 / 3.0;
  r.worst = {
      WorstMetric{"final_pos_err_m", 2e-4, 0.005, 0.8 * 0.005},
      WorstMetric{"final_rot_err_deg", 0.1 + 0.2, 2.0, 0.8 * 2.0},
      WorstMetric{"rms_joint_err_rad", kNan, 0.02, 0.8 * 0.02},
      WorstMetric{"cycle_time_s", 10.5, 12.0, 0.8 * 12.0},
  };
  r.verdict = v;
  r.quick = quick;
  r.assumptions = {
      assumption("sensor.q_offset_std_rad",
                 pipeline::ProvenanceKind::kAssumed,
                 std::array<double, 2>{5.0e-5, 1.0e-3},
                 std::nullopt),
      assumption("actuator.knee_speed_rad_s",
                 pipeline::ProvenanceKind::kDerived,
                 std::nullopt,
                 std::array<double, 2>{0.8, 1.25}),
  };
  return r;
}

std::vector<InputFile> inputs() {
  InputFile a;
  a.path = "config/hardware/premium_fr3.yaml";
  a.sha256 = "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855";
  InputFile b;
  b.path = "config/tasks/reference_move.yaml";
  b.sha256 = "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad";
  return {a, b};
}

Json parsed(const VerifyResult& r) {
  const std::string text = report_json(r, inputs());
  const std::optional<Json> doc = parse_json(text);
  EXPECT_TRUE(doc.has_value()) << "not strict JSON:\n" << text;
  return doc.value_or(Json{});
}

// A copy of the value of `key` (a copy, so that at(parsed(r), ...) never dangles); null when
// absent.
Json at(const Json& obj, const std::string& key) {
  const Json* v = obj.find(key);
  EXPECT_NE(v, nullptr) << "missing key " << key;
  return v != nullptr ? *v : Json{};
}

// Every number of the document is written with %.17g.
void expect_g17_numbers(const Json& v, const std::string& where) {
  if (v.is(Type::kNumber)) {
    EXPECT_EQ(v.text, g17(v.number)) << where;
  }
  for (std::size_t i = 0; i < v.items.size(); ++i) {
    expect_g17_numbers(v.items[i], where + "[" + std::to_string(i) + "]");
  }
  for (std::size_t i = 0; i < v.values.size(); ++i) {
    expect_g17_numbers(v.values[i], where + "." + v.keys[i]);
  }
}

// `v` is the number `x`, bit for bit (via the %.17g text), or null when x is NaN.
void expect_number(const Json& v, double x, const std::string& where) {
  if (std::isnan(x)) {
    EXPECT_TRUE(v.is(Type::kNull)) << where << ": " << v.text;
    return;
  }
  ASSERT_TRUE(v.is(Type::kNumber)) << where;
  EXPECT_EQ(v.text, g17(x)) << where;
  EXPECT_EQ(v.number, x) << where;
}

TEST(ReportJson, TopLevelKeysAreTheContractsInOrder) {
  const Json doc = parsed(sample(Verdict::kFail, true, GainSource::kTuned));
  ASSERT_TRUE(doc.is(Type::kObject));
  std::vector<std::string> listed;
  for (const std::string& k : doc.keys) {
    if (k != "model" && k != "algorithm" && k != "task") {
      listed.push_back(k);
    }
  }
  EXPECT_EQ(listed, kTopKeys);
  EXPECT_EQ(doc.keys.size(), kTopKeys.size() + 3);
  EXPECT_EQ(at(doc, "model").text, "premium_fr3");
  EXPECT_EQ(at(doc, "algorithm").text, "computed_torque");
  EXPECT_EQ(at(doc, "task").text, "reference_move");
}

TEST(ReportJson, ScalarsAndCounts) {
  const VerifyResult r = sample(Verdict::kFail, true, GainSource::kTuned);
  const Json doc = parsed(r);
  expect_number(at(doc, "schema"), 1.0, "schema");
  EXPECT_EQ(at(doc, "disclaimer").text, kDisclaimer);
  EXPECT_EQ(at(doc, "verdict").text, "fail");
  ASSERT_TRUE(at(doc, "quick").is(Type::kBool));
  EXPECT_TRUE(at(doc, "quick").boolean);
  expect_number(at(doc, "runs"), 3.0, "runs");
  const Json& seeds = at(doc, "seeds");
  ASSERT_EQ(seeds.items.size(), 2U);
  expect_number(seeds.items[0], 7.0, "seeds[0]");
  expect_number(seeds.items[1], 9.0, "seeds[1]");
  expect_number(at(doc, "pass_count"), 2.0, "pass_count");
  expect_number(at(doc, "pass_rate"), 2.0 / 3.0, "pass_rate");  // 0.66666666666666663
  const Json& w = at(doc, "wilson");
  ASSERT_EQ(w.items.size(), 2U);
  expect_number(w.items[0], r.wilson.lo, "wilson[0]");
  expect_number(w.items[1], r.wilson.hi, "wilson[1]");
  expect_number(at(doc, "hold_rate"), 1.0 / 3.0, "hold_rate");
  expect_g17_numbers(doc, "report");
}

TEST(ReportJson, VerdictNames) {
  EXPECT_EQ(at(parsed(sample(Verdict::kPass, false, GainSource::kTuned)), "verdict").text, "pass");
  EXPECT_EQ(at(parsed(sample(Verdict::kMarginal, true, GainSource::kTuned)), "verdict").text,
            "marginal");
  const Json not_quick = parsed(sample(Verdict::kPass, false, GainSource::kTuned));
  EXPECT_FALSE(at(not_quick, "quick").boolean);
}

TEST(ReportJson, WorstMetricsInOrderWithNullForNan) {
  const VerifyResult r = sample(Verdict::kFail, true, GainSource::kTuned);
  const Json& worst = at(parsed(r), "worst");
  ASSERT_TRUE(worst.is(Type::kObject));
  EXPECT_EQ(worst.keys, kWorstKeys);
  ASSERT_EQ(worst.values.size(), r.worst.size());
  for (std::size_t i = 0; i < r.worst.size(); ++i) {
    const Json& m = worst.values[i];
    EXPECT_EQ(m.keys, (std::vector<std::string>{"value", "threshold", "margin_limit"}));
    expect_number(at(m, "value"), r.worst[i].value, worst.keys[i] + ".value");
    expect_number(at(m, "threshold"), r.worst[i].threshold, worst.keys[i] + ".threshold");
    expect_number(at(m, "margin_limit"), r.worst[i].margin_limit, worst.keys[i] + ".margin_limit");
  }
  // 0.1 + 0.2 needs all 17 digits to come back as the same double.
  EXPECT_EQ(at(at(worst, "final_rot_err_deg"), "value").text, "0.30000000000000004");
  EXPECT_TRUE(at(at(worst, "rms_joint_err_rad"), "value").is(Type::kNull));
}

TEST(ReportJson, TuningIsTheTuneTable) {
  const VerifyResult r = sample(Verdict::kFail, true, GainSource::kTuned);
  const Json& t = at(parsed(r), "tuning");
  EXPECT_EQ(t.keys, kTuningKeys);
  EXPECT_EQ(at(t, "key").text, "config.computed_torque.omega");
  expect_number(at(t, "gain_used"), 40.0, "gain_used");
  EXPECT_EQ(at(t, "gain_source").text, "tuned");
  const Json& levels = at(t, "levels");
  ASSERT_EQ(levels.items.size(), 2U);
  expect_number(levels.items[0], 40.0, "levels[0]");
  expect_number(levels.items[1], 50.0, "levels[1]");
  const Json& rows = at(t, "rows");
  ASSERT_EQ(rows.items.size(), 2U);
  for (std::size_t i = 0; i < 2; ++i) {
    EXPECT_EQ(rows.items[i].keys, kTuneRowKeys) << i;
    expect_number(at(rows.items[i], "level"), r.tuning->rows[i].level, "rows.level");
    expect_number(at(rows.items[i], "seeds"), 2.0, "rows.seeds");
    expect_number(at(rows.items[i], "rms_joint_err_median"),
                  r.tuning->rows[i].rms_joint_err_median,
                  "rows.rms");
    expect_number(at(rows.items[i], "final_pos_err_max"),
                  r.tuning->rows[i].final_pos_err_max,
                  "rows.final_pos_err_max");
  }
  expect_number(at(rows.items[1], "held"), 2.0, "rows[1].held");
  expect_number(at(t, "chosen"), 40.0, "chosen");
  EXPECT_EQ(at(t, "reason").text, "chosen 40: lowest median rms");
}

TEST(ReportJson, GainSourcesWithoutATunedLevel) {
  const Json none =
      at(parsed(sample(Verdict::kFail, true, GainSource::kDefaultNoAdmissible)), "tuning");
  EXPECT_EQ(none.keys, kTuningKeys);
  EXPECT_EQ(at(none, "gain_source").text, "default_no_admissible");
  expect_number(at(none, "gain_used"), 50.0, "gain_used");
  EXPECT_TRUE(at(none, "chosen").is(Type::kNull));
  EXPECT_EQ(at(none, "rows").items.size(), 2U);

  const Json off =
      at(parsed(sample(Verdict::kFail, true, GainSource::kDefaultTuningOff)), "tuning");
  EXPECT_EQ(off.keys, kTuningKeys);
  EXPECT_EQ(at(off, "gain_source").text, "default_tuning_off");
  EXPECT_EQ(at(off, "key").text, "config.computed_torque.omega");
  expect_number(at(off, "gain_used"), 50.0, "gain_used");
  EXPECT_TRUE(at(off, "levels").is(Type::kArray));
  EXPECT_TRUE(at(off, "levels").items.empty());
  EXPECT_TRUE(at(off, "rows").is(Type::kArray));
  EXPECT_TRUE(at(off, "rows").items.empty());
  EXPECT_TRUE(at(off, "chosen").is(Type::kNull));
}

TEST(ReportJson, AssumptionsAreTheSeamOfTheNextPr) {
  const Json doc = parsed(sample(Verdict::kFail, true, GainSource::kTuned));
  const Json& a = at(doc, "assumptions");
  ASSERT_EQ(a.items.size(), 2U);
  EXPECT_EQ(a.items[0].keys, (std::vector<std::string>{"key", "kind", "range", "sensitivity"}));
  EXPECT_EQ(at(a.items[0], "key").text, "sensor.q_offset_std_rad");
  EXPECT_EQ(at(a.items[0], "kind").text, "assumed");
  const Json& range = at(a.items[0], "range");
  ASSERT_EQ(range.items.size(), 2U);
  expect_number(range.items[0], 5.0e-5, "range[0]");
  expect_number(range.items[1], 1.0e-3, "range[1]");
  EXPECT_EQ(at(a.items[0], "sensitivity").text, "not run");

  EXPECT_EQ(a.items[1].keys, (std::vector<std::string>{"key", "kind", "scale", "sensitivity"}));
  EXPECT_EQ(at(a.items[1], "kind").text, "derived");
  const Json& scale = at(a.items[1], "scale");
  ASSERT_EQ(scale.items.size(), 2U);
  expect_number(scale.items[0], 0.8, "scale[0]");
  expect_number(scale.items[1], 1.25, "scale[1]");
  EXPECT_EQ(at(a.items[1], "sensitivity").text, "not run");

  ASSERT_TRUE(at(doc, "causes").is(Type::kArray));
  EXPECT_TRUE(at(doc, "causes").items.empty());
  ASSERT_TRUE(at(doc, "suggestions").is(Type::kArray));
  EXPECT_TRUE(at(doc, "suggestions").items.empty());
}

TEST(ReportJson, VersionsNameTheBuildAndTheInputs) {
  const Json& v = at(parsed(sample(Verdict::kFail, true, GainSource::kTuned)), "versions");
  EXPECT_EQ(v.keys, (std::vector<std::string>{"git", "inputs"}));
  EXPECT_TRUE(at(v, "git").is(Type::kString));
  const Json& in = at(v, "inputs");
  const std::vector<InputFile> expected = inputs();
  ASSERT_EQ(in.items.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(in.items[i].keys, (std::vector<std::string>{"path", "sha256"})) << i;
    EXPECT_EQ(at(in.items[i], "path").text, expected[i].path) << i;
    EXPECT_EQ(at(in.items[i], "sha256").text, expected[i].sha256) << i;
  }
}

TEST(ReportJson, SameResultSameBytes) {
  const VerifyResult r = sample(Verdict::kMarginal, true, GainSource::kTuned);
  EXPECT_EQ(report_json(r, inputs()), report_json(r, inputs()));
  EXPECT_EQ(report_md(r, inputs()), report_md(r, inputs()));
}

// ---- report.md ----------------------------------------------------------------------------------

TEST(ReportMd, TheDisclaimerIsTheFirstLine) {
  for (const Verdict v : {Verdict::kPass, Verdict::kFail, Verdict::kMarginal}) {
    const std::string md = report_md(sample(v, v != Verdict::kPass, GainSource::kTuned), inputs());
    EXPECT_TRUE(contains(first_line(md), kDisclaimer)) << md;
  }
}

TEST(ReportMd, TheVerdictLineAndTheQuickLabel) {
  const std::string pass = report_md(sample(Verdict::kPass, false, GainSource::kTuned), inputs());
  EXPECT_TRUE(contains(pass, "PASS")) << pass;
  EXPECT_FALSE(contains(pass, "quick mode")) << pass;
  EXPECT_FALSE(contains(pass, "MARGINAL")) << pass;

  const std::string fail = report_md(sample(Verdict::kFail, false, GainSource::kTuned), inputs());
  EXPECT_TRUE(contains(fail, "FAIL")) << fail;
  EXPECT_FALSE(contains(fail, "quick mode")) << fail;

  const std::string marginal =
      report_md(sample(Verdict::kMarginal, true, GainSource::kTuned), inputs());
  EXPECT_TRUE(contains(marginal, "MARGINAL, measure on hardware")) << marginal;
  EXPECT_TRUE(contains(marginal, "quick mode: never PASS")) << marginal;
}

TEST(ReportMd, TablesNameTheMetricsTheLevelsAndTheAssumptions) {
  const std::string md = report_md(sample(Verdict::kFail, true, GainSource::kTuned), inputs());
  EXPECT_TRUE(contains(md, "|")) << md;
  for (const std::string& name : kWorstKeys) {
    EXPECT_TRUE(contains(md, name)) << name << "\n" << md;
  }
  EXPECT_TRUE(contains(md, "sensor.q_offset_std_rad")) << md;
  EXPECT_TRUE(contains(md, "actuator.knee_speed_rad_s")) << md;
  EXPECT_TRUE(contains(md, "not run")) << md;
  EXPECT_TRUE(contains(md, "config.computed_torque.omega")) << md;
  EXPECT_TRUE(contains(md, "chosen 40: lowest median rms")) << md;
}

TEST(ReportMd, SaysWhyTheGainIsTheModels) {
  const std::string none =
      report_md(sample(Verdict::kFail, true, GainSource::kDefaultNoAdmissible), inputs());
  EXPECT_TRUE(contains(none, "no admissible gain: verdict on the model's default gain")) << none;
  const std::string off =
      report_md(sample(Verdict::kFail, true, GainSource::kDefaultTuningOff), inputs());
  EXPECT_TRUE(contains(off, "tuning off")) << off;
  const std::string tuned = report_md(sample(Verdict::kFail, true, GainSource::kTuned), inputs());
  EXPECT_FALSE(contains(tuned, "tuning off")) << tuned;
  EXPECT_FALSE(contains(tuned, "verdict on the model's default gain")) << tuned;
}

// ---- report_inputs ------------------------------------------------------------------------------

void expect_inputs(const std::vector<InputFile>& got, const std::vector<fs::path>& files) {
  ASSERT_EQ(got.size(), files.size());
  for (std::size_t i = 0; i < files.size(); ++i) {
    ASSERT_TRUE(fs::exists(got[i].path)) << i << ": " << got[i].path;
    EXPECT_TRUE(fs::equivalent(got[i].path, files[i])) << i << ": " << got[i].path;
    EXPECT_EQ(got[i].sha256, sha256_hex(read_text(files[i]))) << got[i].path;
    EXPECT_EQ(got[i].sha256.size(), 64U) << got[i].path;
  }
}

TEST(ReportInputs, TheModelItsTwoFilesAndTheTask) {
  const pipeline::HardwareModel model = load_model("premium_fr3");
  const pipeline::Task task = load_task("reference_move");
  const auto r = report_inputs(model_path("premium_fr3"), model, task_path("reference_move"), task);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  expect_inputs(r.value(),
                {model_path("premium_fr3"),
                 kRepo / "config/imperfections/premium_fr3.yaml",
                 kRepo / "config/premium_fr3.yaml",
                 task_path("reference_move")});
}

TEST(ReportInputs, AWaypointTaskAddsItsWaypointFile) {
  const pipeline::HardwareModel model = load_model("budget_xarm7");
  const pipeline::Task task = load_task("pick_place_3kg");
  const auto r =
      report_inputs(model_path("budget_xarm7"), model, task_path("pick_place_3kg"), task);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  expect_inputs(r.value(),
                {model_path("budget_xarm7"),
                 kRepo / "config/imperfections/budget_xarm7.yaml",
                 kRepo / "config/budget_xarm7.yaml",
                 task_path("pick_place_3kg"),
                 task_path("pick_place_waypoints")});
}

TEST(ReportInputs, AnUnreadableFileIsAnErrorNamingIt) {
  const pipeline::HardwareModel model = load_model("premium_fr3");
  const pipeline::Task task = load_task("reference_move");
  const fs::path missing = test_dir() / "no_such_task.yaml";
  const auto r = report_inputs(model_path("premium_fr3"), model, missing, task);
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(r.reason(), "no_such_task.yaml")) << r.reason();
}

}  // namespace
}  // namespace cobot::bench
