// Causes and suggestions of a FAIL (brief verify-analysis, section 2; plan ambiguity 17), pure
// functions:
//  - mark_main_causes: the group whose removal raises the pass count most is the main cause, ties
//    are all main, none raising it -> no main cause ("no single group explains the failure");
//  - suggest: the fixed rule table of the brief, each rule alone, several together in the table's
//    order, none. The texts are the brief's, one English line each:
//      1. torque law (computed_torque or pd_gravity), comm.servo_rate_hz > 0, holds mostly
//         state_timeout or overspeed (more than half of the holds)
//         -> "needs a position loop in the drive (algorithm drive_position)"
//      2. main cause comm         -> "raise the servo rate or reduce packet loss"
//      3. main cause sensor       -> "a finer encoder or less sensor noise"
//      4. main cause transmission -> "a stiffer transmission or less lost motion"
//      5. main cause friction     -> "friction identification and compensation"
//      6. no admissible gain      -> "no admissible gain in the sweep: widen the levels or change
//                                     the law"
#include <cstddef>
#include <map>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/control/algorithm.hpp"

#include "analysis.hpp"

namespace cobot::bench {
namespace {

using control::Algorithm;

const std::string kPositionLoop = "needs a position loop in the drive (algorithm drive_position)";
const std::string kServo = "raise the servo rate or reduce packet loss";
const std::string kEncoder = "a finer encoder or less sensor noise";
const std::string kStiffer = "a stiffer transmission or less lost motion";
const std::string kFriction = "friction identification and compensation";
const std::string kNoGain = "no admissible gain in the sweep: widen the levels or change the law";

Cause cause(const char* group, std::size_t passed_without, std::size_t base_passed) {
  Cause c;
  c.group = group;
  c.passed_without = passed_without;
  c.base_passed = base_passed;
  c.main = false;
  return c;
}

// Facts under which no rule applies: drive law, no servo rate, no hold, no main cause, tuned.
SuggestionFacts quiet() {
  SuggestionFacts f;
  f.algorithm = Algorithm::kDrivePosition;
  f.servo_rate_hz = 0.0;
  f.hold_reasons = {};
  f.main_causes = {};
  f.no_admissible_gain = false;
  return f;
}

// ---- main cause --------------------------------------------------------------------------------

TEST(MainCause, TheLargestRiseIsMain) {
  // base 2 of 20; without comm 15, without sensor 5, without friction 2 (no rise).
  std::vector<Cause> causes = {
      cause("sensor", 5, 2), cause("comm", 15, 2), cause("friction", 2, 2)};
  EXPECT_TRUE(mark_main_causes(causes));
  ASSERT_EQ(causes.size(), 3U);
  EXPECT_EQ(causes[0].group, "sensor");  // order kept
  EXPECT_FALSE(causes[0].main);
  EXPECT_TRUE(causes[1].main);
  EXPECT_FALSE(causes[2].main);
}

TEST(MainCause, TiesAreAllMain) {
  std::vector<Cause> causes = {cause("sensor", 9, 0), cause("comm", 9, 0), cause("timing", 3, 0)};
  EXPECT_TRUE(mark_main_causes(causes));
  EXPECT_TRUE(causes[0].main);
  EXPECT_TRUE(causes[1].main);
  EXPECT_FALSE(causes[2].main);
}

TEST(MainCause, NoRiseMeansNoMainCause) {
  // Removing a group never raises the pass count above the base 4: equal or lower.
  std::vector<Cause> causes = {cause("sensor", 4, 4), cause("comm", 3, 4), cause("model", 0, 4)};
  EXPECT_FALSE(mark_main_causes(causes));
  for (const Cause& c : causes) {
    EXPECT_FALSE(c.main) << c.group;
  }
  std::vector<Cause> none;
  EXPECT_FALSE(mark_main_causes(none));
}

TEST(MainCause, AStaleMainFlagIsCleared) {
  std::vector<Cause> causes = {cause("sensor", 1, 0), cause("comm", 6, 0)};
  causes[0].main = true;
  EXPECT_TRUE(mark_main_causes(causes));
  EXPECT_FALSE(causes[0].main);
  EXPECT_TRUE(causes[1].main);
}

// ---- suggestion table --------------------------------------------------------------------------

TEST(Suggestions, NoRuleNoSuggestion) {
  EXPECT_TRUE(suggest(quiet()).empty());
}

TEST(Suggestions, PositionLoopForATorqueLawWithServoRateAndTimeoutHolds) {
  for (const Algorithm a : {Algorithm::kComputedTorque, Algorithm::kPdGravity}) {
    SuggestionFacts f = quiet();
    f.algorithm = a;
    f.servo_rate_hz = 250.0;
    f.hold_reasons = {{"state_timeout", 3}};
    EXPECT_EQ(suggest(f), (std::vector<std::string>{kPositionLoop})) << control::to_string(a);
    f.hold_reasons = {{"overspeed", 2}};
    EXPECT_EQ(suggest(f), (std::vector<std::string>{kPositionLoop})) << control::to_string(a);
    // Mixed: 2 state_timeout + 1 overspeed of 4 holds = 3 > 4 / 2.
    f.hold_reasons = {{"state_timeout", 2}, {"overspeed", 1}, {"joint_limit", 1}};
    EXPECT_EQ(suggest(f), (std::vector<std::string>{kPositionLoop})) << control::to_string(a);
  }
}

TEST(Suggestions, PositionLoopNeedsEveryCondition) {
  SuggestionFacts f = quiet();
  f.algorithm = Algorithm::kComputedTorque;
  f.servo_rate_hz = 250.0;
  f.hold_reasons = {{"state_timeout", 3}};
  ASSERT_EQ(suggest(f).size(), 1U);

  SuggestionFacts drive = f;  // the drive already closes the position loop
  drive.algorithm = Algorithm::kDrivePosition;
  EXPECT_TRUE(suggest(drive).empty());

  SuggestionFacts no_servo = f;  // servo_rate_hz 0: the drive is read every control cycle
  no_servo.servo_rate_hz = 0.0;
  EXPECT_TRUE(suggest(no_servo).empty());

  SuggestionFacts no_hold = f;
  no_hold.hold_reasons = {};
  EXPECT_TRUE(suggest(no_hold).empty());

  SuggestionFacts other_holds = f;  // joint_limit and non_finite only
  other_holds.hold_reasons = {{"joint_limit", 2}, {"non_finite", 1}};
  EXPECT_TRUE(suggest(other_holds).empty());

  SuggestionFacts half = f;  // 2 of 4 is half, not "mostly"
  half.hold_reasons = {{"state_timeout", 1}, {"overspeed", 1}, {"joint_limit", 2}};
  EXPECT_TRUE(suggest(half).empty());
}

TEST(Suggestions, EachMainCauseRuleAlone) {
  const struct {
    const char* group;
    const std::string* text;
  } cases[] = {
      {"comm", &kServo},
      {"sensor", &kEncoder},
      {"transmission", &kStiffer},
      {"friction", &kFriction},
  };
  for (const auto& c : cases) {
    SuggestionFacts f = quiet();
    f.main_causes = {c.group};
    EXPECT_EQ(suggest(f), (std::vector<std::string>{*c.text})) << c.group;
  }
  // Groups without a rule: nothing.
  for (const char* group : {"actuator", "model", "timing"}) {
    SuggestionFacts f = quiet();
    f.main_causes = {group};
    EXPECT_TRUE(suggest(f).empty()) << group;
  }
}

TEST(Suggestions, NoAdmissibleGainAlone) {
  SuggestionFacts f = quiet();
  f.no_admissible_gain = true;
  EXPECT_EQ(suggest(f), (std::vector<std::string>{kNoGain}));
}

TEST(Suggestions, SeveralInTheTablesOrder) {
  SuggestionFacts f = quiet();
  f.algorithm = Algorithm::kComputedTorque;
  f.servo_rate_hz = 250.0;
  f.hold_reasons = {{"state_timeout", 20}};
  // Tied main causes given in another order than the table's.
  f.main_causes = {"friction", "transmission", "sensor", "comm"};
  f.no_admissible_gain = true;
  EXPECT_EQ(
      suggest(f),
      (std::vector<std::string>{kPositionLoop, kServo, kEncoder, kStiffer, kFriction, kNoGain}));

  SuggestionFacts g = quiet();
  g.main_causes = {"friction", "sensor"};
  g.no_admissible_gain = true;
  EXPECT_EQ(suggest(g), (std::vector<std::string>{kEncoder, kFriction, kNoGain}));
}

}  // namespace
}  // namespace cobot::bench
