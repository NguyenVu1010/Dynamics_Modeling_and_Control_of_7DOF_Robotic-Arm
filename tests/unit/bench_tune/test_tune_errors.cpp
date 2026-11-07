// bench::tune on inputs that fail before any loop runs (brief auto-tune, section 5), so in every
// build: an algorithm the model does not list is an error naming the model; a level the loader
// refuses and a rejected waypoint list are errors, not rows of the table.
#include <string>

#include <gtest/gtest.h>

#include "cobot/control/algorithm.hpp"
#include "cobot/pipeline/hardware_model.hpp"
#include "cobot/pipeline/task.hpp"

#include "tune.hpp"
#include "tune_test_util.hpp"

namespace cobot::bench {
namespace {

using control::Algorithm;
using namespace tune_test;  // NOLINT(google-build-using-namespace)

TEST(TuneErrors, AnUnlistedAlgorithmNamesTheModel) {
  const pipeline::HardwareModel model = load_model("premium_fr3");
  const pipeline::Task task = load_task("reference_move");
  const auto r = tune(model, Algorithm::kDrivePosition, task, options({80.0}, {1}, 1));
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(r.reason(), "premium_fr3")) << r.reason();
}

TEST(TuneErrors, ALevelTheLoaderRefusesIsAnError) {
  const pipeline::HardwareModel model = load_model("budget_xarm7");
  const pipeline::Task task = load_task("reference_move");
  for (const Algorithm a :
       {Algorithm::kComputedTorque, Algorithm::kPdGravity, Algorithm::kDrivePosition}) {
    for (const double level : {0.0, -1.0}) {
      const auto r = tune(model, a, task, options({level}, {1}, 1));
      ASSERT_FALSE(r.is_ok()) << control::to_string(a) << " " << level;
      EXPECT_TRUE(contains(r.reason(), "omega")) << r.reason();
    }
  }
}

TEST(TuneErrors, TheFirstErrorStopsTheRemainingWork) {
  // With one job the bad level comes first: the error comes back without running level 50.
  const pipeline::HardwareModel model = load_model("premium_fr3");
  const pipeline::Task task = load_task("reference_move");
  const auto r = tune(model, Algorithm::kComputedTorque, task, options({-1.0, 50.0}, {1, 2}, 1));
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(r.reason(), "omega")) << r.reason();
}

TEST(TuneErrors, ARejectedWaypointListIsAnError) {
  const pipeline::HardwareModel model = load_model("premium_fr3");
  pipeline::Task task = load_task("pick_place_3kg");
  task.waypoints = kRepo / "tests/data/waypoints/unreachable_last.yaml";
  const auto r = tune(model, Algorithm::kComputedTorque, task, options({50.0}, {1}, 1));
  ASSERT_FALSE(r.is_ok()) << "a rejected list is not a held run";
  EXPECT_TRUE(contains(r.reason(), "unreachable_last.yaml")) << r.reason();
  EXPECT_TRUE(contains(r.reason(), "path validation failed")) << r.reason();
}

TEST(TuneErrors, AWaypointFileThatDoesNotParseIsAnError) {
  const pipeline::HardwareModel model = load_model("premium_fr3");
  pipeline::Task task = load_task("pick_place_3kg");
  task.waypoints = kRepo / "tests/data/waypoints/bad_position.yaml";
  const auto r = tune(model, Algorithm::kComputedTorque, task, options({50.0}, {1}, 1));
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(r.reason(), "bad_position.yaml")) << r.reason();
}

}  // namespace
}  // namespace cobot::bench
