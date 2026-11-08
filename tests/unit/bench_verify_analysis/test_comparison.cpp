// The comparison of several models and algorithms (brief verify-analysis, section 3; design 10.2
// G16), without runs:
//  - comparison_pairs: the cross product, models outer and algorithms inner, a pair the model
//    does not list kept as "not listed"; no listed pair is an error;
//  - pair_name: "<model>__<algorithm>", the directory of a pair's report files;
//  - comparison_json / comparison_md on hand-made rows: the disclaimer first (the first key of
//    the JSON, the first line of the MD), then one row per pair in order with model, algorithm,
//    verdict, pass count / runs, Wilson interval, worst value of each metric, gain used and its
//    source; an unlisted pair is named "not listed".
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/control/algorithm.hpp"
#include "cobot/pipeline/hardware_model.hpp"

#include "compare.hpp"
#include "report.hpp"
#include "verify.hpp"
#include "verify_test_util.hpp"

namespace cobot::bench {
namespace {

using control::Algorithm;
using namespace verify_test;  // NOLINT(google-build-using-namespace)

const double kNan = std::numeric_limits<double>::quiet_NaN();

const std::vector<std::string> kMetrics = {
    "final_pos_err_m", "final_rot_err_deg", "rms_joint_err_rad", "cycle_time_s"};

// premium_fr3 and mid_kr1205 list computed_torque and pd_gravity; budget_xarm7 lists all three.
std::vector<pipeline::HardwareModel> models() {
  return {load_model("premium_fr3"), load_model("mid_kr1205"), load_model("budget_xarm7")};
}

TEST(PairName, IsModelTwoUnderscoresAlgorithm) {
  EXPECT_EQ(pair_name("budget_xarm7", Algorithm::kDrivePosition), "budget_xarm7__drive_position");
  EXPECT_EQ(pair_name("premium_fr3", Algorithm::kComputedTorque), "premium_fr3__computed_torque");
}

TEST(ComparisonPairs, CrossProductWithTheUnlistedPairsNamed) {
  // E2E-16: three models x {computed_torque, drive_position}: six pairs, two not listed.
  const auto pairs =
      comparison_pairs(models(), {Algorithm::kComputedTorque, Algorithm::kDrivePosition});
  ASSERT_TRUE(pairs.is_ok()) << pairs.reason();
  const struct {
    std::size_t model;
    Algorithm algorithm;
    bool listed;
  } want[] = {
      {0, Algorithm::kComputedTorque, true},
      {0, Algorithm::kDrivePosition, false},
      {1, Algorithm::kComputedTorque, true},
      {1, Algorithm::kDrivePosition, false},
      {2, Algorithm::kComputedTorque, true},
      {2, Algorithm::kDrivePosition, true},
  };
  ASSERT_EQ(pairs.value().size(), 6U);
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_EQ(pairs.value()[i].model, want[i].model) << i;
    EXPECT_TRUE(pairs.value()[i].algorithm == want[i].algorithm) << i;
    EXPECT_EQ(pairs.value()[i].listed, want[i].listed) << i;
  }
}

TEST(ComparisonPairs, OnePairIsTheSingleCase) {
  const auto pairs = comparison_pairs({load_model("premium_fr3")}, {Algorithm::kComputedTorque});
  ASSERT_TRUE(pairs.is_ok()) << pairs.reason();
  ASSERT_EQ(pairs.value().size(), 1U);
  EXPECT_TRUE(pairs.value()[0].listed);
}

TEST(ComparisonPairs, NoListedPairIsAnError) {
  const auto none = comparison_pairs({load_model("premium_fr3"), load_model("mid_kr1205")},
                                     {Algorithm::kDrivePosition});
  ASSERT_FALSE(none.is_ok());
  EXPECT_TRUE(contains(none.reason(), "drive_position")) << none.reason();
  EXPECT_FALSE(comparison_pairs({}, {Algorithm::kComputedTorque}).is_ok());
  EXPECT_FALSE(comparison_pairs(models(), {}).is_ok());
}

// ---- comparison.json / comparison.md
// -------------------------------------------------------------

VerifyResult result(const char* model,
                    Algorithm algorithm,
                    Verdict v,
                    std::size_t runs,
                    std::size_t passed,
                    std::size_t held) {
  VerifyResult r;
  r.algorithm = algorithm;
  r.model = model;
  r.task = "reference_move";
  r.gain_key = tuning_key(algorithm);
  r.gain_used = algorithm == Algorithm::kDrivePosition ? 80.0 : 50.0;
  r.gain_source = v == Verdict::kFail ? GainSource::kDefaultNoAdmissible : GainSource::kTuned;
  r.counts.runs = runs;
  r.counts.passed = passed;
  r.counts.held = held;
  r.pass_rate = static_cast<double>(passed) / static_cast<double>(runs);
  r.hold_rate = static_cast<double>(held) / static_cast<double>(runs);
  r.wilson = wilson(passed, runs);
  r.verdict = v;
  r.quick = runs < 100;
  const double values[] = {1.0 / 3.0 * 1e-3, 0.0123, held == runs ? kNan : 2.5e-3, 10.75};
  const double thresholds[] = {0.005, 2.0, 0.02, 12.0};
  for (std::size_t i = 0; i < 4; ++i) {
    r.worst.push_back(WorstMetric{kMetrics[i], values[i], thresholds[i], 0.8 * thresholds[i]});
  }
  return r;
}

std::vector<ComparisonRow> rows() {
  ComparisonRow a;
  a.model = "premium_fr3";
  a.algorithm = Algorithm::kComputedTorque;
  a.result = result("premium_fr3", Algorithm::kComputedTorque, Verdict::kPass, 100, 100, 0);
  ComparisonRow b;
  b.model = "premium_fr3";
  b.algorithm = Algorithm::kDrivePosition;  // not listed
  ComparisonRow c;
  c.model = "budget_xarm7";
  c.algorithm = Algorithm::kComputedTorque;
  c.result = result("budget_xarm7", Algorithm::kComputedTorque, Verdict::kFail, 100, 0, 100);
  ComparisonRow d;
  d.model = "budget_xarm7";
  d.algorithm = Algorithm::kDrivePosition;
  d.result = result("budget_xarm7", Algorithm::kDrivePosition, Verdict::kMarginal, 100, 92, 1);
  return {a, b, c, d};
}

TEST(ComparisonJson, DisclaimerFirstThenOneRowPerPair) {
  const std::string json = comparison_json(rows());
  const std::optional<Json> doc = parse_json(json);
  ASSERT_TRUE(doc.has_value()) << json;
  ASSERT_TRUE(doc->is(Json::Type::kObject));
  ASSERT_FALSE(doc->keys.empty());
  EXPECT_EQ(doc->keys[0], "disclaimer");
  EXPECT_EQ(doc->values[0].text, kDisclaimer);
  const Json* list = doc->find("rows");
  ASSERT_NE(list, nullptr) << json;
  ASSERT_TRUE(list->is(Json::Type::kArray));
  ASSERT_EQ(list->items.size(), 4U);

  const char* models[] = {"premium_fr3", "premium_fr3", "budget_xarm7", "budget_xarm7"};
  const char* algorithms[] = {
      "computed_torque", "drive_position", "computed_torque", "drive_position"};
  const char* verdicts[] = {"pass", "not listed", "fail", "marginal"};
  for (std::size_t i = 0; i < 4; ++i) {
    const Json& row = list->items[i];
    ASSERT_NE(row.find("model"), nullptr) << i;
    EXPECT_EQ(row.find("model")->text, models[i]) << i;
    ASSERT_NE(row.find("algorithm"), nullptr) << i;
    EXPECT_EQ(row.find("algorithm")->text, algorithms[i]) << i;
    ASSERT_NE(row.find("verdict"), nullptr) << i;
    EXPECT_EQ(row.find("verdict")->text, verdicts[i]) << i;
  }
}

TEST(ComparisonJson, ARowCarriesCountsIntervalWorstAndGain) {
  const std::vector<ComparisonRow> r = rows();
  const std::optional<Json> doc = parse_json(comparison_json(r));
  ASSERT_TRUE(doc.has_value());
  const Json* list = doc->find("rows");
  ASSERT_NE(list, nullptr);
  ASSERT_EQ(list->items.size(), 4U);
  for (const std::size_t i : {std::size_t{0}, std::size_t{2}, std::size_t{3}}) {
    SCOPED_TRACE(i);
    const Json& row = list->items[i];
    const VerifyResult& v = *r[i].result;
    ASSERT_NE(row.find("pass_count"), nullptr);
    EXPECT_EQ(row.find("pass_count")->text, std::to_string(v.counts.passed));
    ASSERT_NE(row.find("runs"), nullptr);
    EXPECT_EQ(row.find("runs")->text, std::to_string(v.counts.runs));
    const Json* w = row.find("wilson");
    ASSERT_NE(w, nullptr);
    ASSERT_EQ(w->items.size(), 2U);
    EXPECT_EQ(w->items[0].text, g17(v.wilson.lo));
    EXPECT_EQ(w->items[1].text, g17(v.wilson.hi));
    const Json* worst = row.find("worst");
    ASSERT_NE(worst, nullptr);
    ASSERT_EQ(worst->keys, kMetrics);
    for (std::size_t k = 0; k < 4; ++k) {
      if (std::isnan(v.worst[k].value)) {
        EXPECT_TRUE(worst->values[k].is(Json::Type::kNull)) << kMetrics[k];
      } else {
        EXPECT_EQ(worst->values[k].text, g17(v.worst[k].value)) << kMetrics[k];
      }
    }
    ASSERT_NE(row.find("gain_used"), nullptr);
    EXPECT_EQ(row.find("gain_used")->text, g17(v.gain_used));
    ASSERT_NE(row.find("gain_source"), nullptr);
    EXPECT_EQ(row.find("gain_source")->text, to_string(v.gain_source));
  }
}

TEST(ComparisonMd, StartsWithTheDisclaimerAndNamesEveryPair) {
  const std::string md = comparison_md(rows());
  EXPECT_EQ(first_line(md), kDisclaimer);
  EXPECT_EQ(first_line(md), std::string(kVerifyDisclaimer));
  for (const char* text : {"premium_fr3",
                           "budget_xarm7",
                           "computed_torque",
                           "drive_position",
                           "not listed",
                           "92",
                           "100"}) {
    EXPECT_TRUE(contains(md, text)) << text << "\n" << md;
  }
  // The rows in the pairs' order: premium before budget, CT before drive within budget.
  const std::size_t premium = md.find("premium_fr3");
  const std::size_t budget_ct = md.find("budget_xarm7");
  ASSERT_NE(premium, std::string::npos);
  ASSERT_NE(budget_ct, std::string::npos);
  EXPECT_LT(premium, budget_ct);
  const std::size_t budget_drive = md.find("drive_position", budget_ct);
  EXPECT_NE(budget_drive, std::string::npos);
}

TEST(ComparisonJson, TheSameRowsGiveTheSameBytes) {
  EXPECT_EQ(comparison_json(rows()), comparison_json(rows()));
  EXPECT_EQ(comparison_md(rows()), comparison_md(rows()));
}

}  // namespace
}  // namespace cobot::bench
