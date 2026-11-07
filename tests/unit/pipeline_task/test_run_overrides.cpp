// run_overrides (brief hardware-catalog 4, ambiguity 12, owner question C9): what a model, an
// algorithm and a task change in a run. imperfections get model.payload_{kg,offset_m,radius_m} of
// the task (the plant carries it); config gets the trajectory keys and, when known is true,
// config.payload.*. The Algorithm argument means config also holds what config_for applies, so
// load_config_with(model.controller, overrides.config) is the run's whole config.
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/control/algorithm.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/hardware_model.hpp"
#include "cobot/pipeline/overrides.hpp"
#include "cobot/pipeline/task.hpp"

#include "hw_test_util.hpp"
#include "task_fixture.hpp"

namespace cobot::pipeline {
namespace {

using namespace hw_test;
using common::Vec3;
using control::Algorithm;

constexpr double kTol = 1e-12;

HardwareModel model_named(const std::string& name) {
  const auto r = load_hardware_model(kRepo / "config/hardware" / (name + ".yaml"));
  if (!r.is_ok()) {
    ADD_FAILURE() << name << ": " << r.reason();
    return HardwareModel{};
  }
  return r.value();
}

Task task_of(const fs::path& path) {
  const auto r = load_task(path);
  if (!r.is_ok()) {
    ADD_FAILURE() << path << ": " << r.reason();
    return Task{};
  }
  return r.value();
}

bool has_key(const std::vector<KeyOverride>& list, const std::string& key) {
  for (const KeyOverride& o : list) {
    if (o.key == key) {
      return true;
    }
  }
  return false;
}

bool has_prefix(const std::vector<KeyOverride>& list, const std::string& prefix) {
  for (const KeyOverride& o : list) {
    if (o.key.rfind(prefix, 0) == 0) {
      return true;
    }
  }
  return false;
}

// Fixture task: 2 kg at 0.1 m along tip z, radius 0.03 m.
TEST(RunOverrides, KnownPayloadGoesToConfigAndImperfections) {
  const HardwareModel m = model_named("premium_fr3");
  const RunOverrides ro =
      run_overrides(m, Algorithm::kComputedTorque, task_of(write_task(task_fixture_text())));

  EXPECT_TRUE(has_key(ro.imperfections, "model.payload_kg"));
  EXPECT_TRUE(has_key(ro.imperfections, "model.payload_offset_m"));
  EXPECT_TRUE(has_key(ro.imperfections, "model.payload_radius_m"));
  EXPECT_TRUE(has_key(ro.config, "config.payload.kg"));
  EXPECT_TRUE(has_key(ro.config, "config.payload.offset_m"));
  EXPECT_TRUE(has_key(ro.config, "config.payload.radius_m"));

  const auto imp = load_imperfections_with<kN>(m.imperfections, ro.imperfections);
  ASSERT_TRUE(imp.is_ok()) << imp.reason();
  EXPECT_EQ(imp.value().model.payload_kg, 2.0);
  EXPECT_TRUE(imp.value().model.payload_offset_m == Vec3(0.0, 0.0, 0.1));
  EXPECT_EQ(imp.value().model.payload_radius_m, 0.03);

  const auto c = load_config_with<kN>(m.controller, ro.config);
  ASSERT_TRUE(c.is_ok()) << c.reason();
  EXPECT_EQ(c.value().payload.kg, 2.0);
  EXPECT_TRUE(c.value().payload.offset_m == Vec3(0.0, 0.0, 0.1));
  EXPECT_EQ(c.value().payload.radius_m, 0.03);
}

TEST(RunOverrides, UnknownPayloadGoesToImperfectionsOnly) {
  const HardwareModel m = model_named("premium_fr3");
  const RunOverrides ro = run_overrides(
      m,
      Algorithm::kComputedTorque,
      task_of(write_task(set_line(task_fixture_text(), "    known:", "    known: false"))));

  EXPECT_FALSE(has_prefix(ro.config, "config.payload"));
  const auto c = load_config_with<kN>(m.controller, ro.config);
  ASSERT_TRUE(c.is_ok()) << c.reason();
  EXPECT_EQ(c.value().payload.kg, 0.0);

  const auto imp = load_imperfections_with<kN>(m.imperfections, ro.imperfections);
  ASSERT_TRUE(imp.is_ok()) << imp.reason();
  EXPECT_EQ(imp.value().model.payload_kg, 2.0);
  EXPECT_TRUE(imp.value().model.payload_offset_m == Vec3(0.0, 0.0, 0.1));
  EXPECT_EQ(imp.value().model.payload_radius_m, 0.03);
}

TEST(RunOverrides, OnlyThePayloadOfTheProfileChanges) {
  const HardwareModel m = model_named("premium_fr3");
  const RunOverrides ro =
      run_overrides(m, Algorithm::kComputedTorque, task_of(write_task(task_fixture_text())));
  EXPECT_EQ(ro.imperfections.size(), 3U);
  const auto base = load_imperfections<kN>(m.imperfections);
  ASSERT_TRUE(base.is_ok()) << base.reason();
  const auto imp = load_imperfections_with<kN>(m.imperfections, ro.imperfections);
  ASSERT_TRUE(imp.is_ok()) << imp.reason();
  EXPECT_TRUE(imp.value().model.link_mass_scale_std == base.value().model.link_mass_scale_std);
  EXPECT_TRUE(imp.value().friction.coulomb_nm == base.value().friction.coulomb_nm);
  EXPECT_EQ(imp.value().seed, base.value().seed);
}

TEST(RunOverrides, TrajectoryKeysReachTheConfig) {
  const HardwareModel m = model_named("premium_fr3");
  const RunOverrides ro =
      run_overrides(m, Algorithm::kComputedTorque, task_of(write_task(task_fixture_text())));
  for (const char* key : {"config.trajectory.goal_position_m",
                          "config.trajectory.goal_rpy_deg",
                          "config.trajectory.duration_s",
                          "config.trajectory.hold_s"}) {
    EXPECT_TRUE(has_key(ro.config, key)) << key;
  }
  EXPECT_FALSE(ro.waypoints.has_value());

  const auto c = load_config_with<kN>(m.controller, ro.config);
  ASSERT_TRUE(c.is_ok()) << c.reason();
  EXPECT_TRUE(c.value().trajectory.goal_position_m == Vec3(-0.4, -0.6, 0.8));
  // 45 deg in rad: 45 * pi / 180 = 0.78539816...
  EXPECT_NEAR((c.value().trajectory.goal_rpy_rad - Vec3(0.0, 0.0, 45.0 * kDeg)).norm(), 0.0, kTol);
  EXPECT_EQ(c.value().trajectory.duration_s, 7.0);
  EXPECT_EQ(c.value().trajectory.hold_s, 1.0);
}

TEST(RunOverrides, WaypointsPathIsResolvedRelativeToTheTaskFile) {
  const fs::path list = write_text(test_dir() / "lists" / "list.yaml", "frame: base_link\n");
  const fs::path file = write_task(task_waypoints_text("../lists/list.yaml"));
  const RunOverrides ro =
      run_overrides(model_named("premium_fr3"), Algorithm::kComputedTorque, task_of(file));
  ASSERT_TRUE(ro.waypoints.has_value());
  EXPECT_TRUE(same_path(*ro.waypoints, list)) << *ro.waypoints;
  EXPECT_FALSE(has_prefix(ro.config, "config.trajectory"));
}

// The shipped waypoint task: config/tasks/pick_place_waypoints.yaml, known 3 kg.
TEST(RunOverrides, PickPlaceOnPremium) {
  const HardwareModel m = model_named("premium_fr3");
  const RunOverrides ro = run_overrides(
      m, Algorithm::kComputedTorque, task_of(kRepo / "config/tasks/pick_place_3kg.yaml"));
  ASSERT_TRUE(ro.waypoints.has_value());
  EXPECT_TRUE(same_path(*ro.waypoints, kRepo / "config/tasks/pick_place_waypoints.yaml"));
  const auto c = load_config_with<kN>(m.controller, ro.config);
  ASSERT_TRUE(c.is_ok()) << c.reason();
  EXPECT_EQ(c.value().payload.kg, 3.0);
  EXPECT_TRUE(c.value().payload.offset_m == Vec3(0.0, 0.0, 0.05));
  EXPECT_EQ(c.value().payload.radius_m, 0.05);
}

// reference_move has the trajectory of config/premium_fr3.yaml and 0 kg: bit for bit the config of
// no task, including rpy (0, 0, 90) deg, and the controller's model stays untouched.
TEST(RunOverrides, ZeroKgTaskOnPremiumGivesTheConfigOfNoTask) {
  const HardwareModel m = model_named("premium_fr3");
  const RunOverrides ro = run_overrides(
      m, Algorithm::kComputedTorque, task_of(kRepo / "config/tasks/reference_move.yaml"));
  const auto with_task = load_config_with<kN>(m.controller, ro.config);
  ASSERT_TRUE(with_task.is_ok()) << with_task.reason();
  const auto no_task = config_for<kN>(m, Algorithm::kComputedTorque);
  ASSERT_TRUE(no_task.is_ok()) << no_task.reason();
  expect_same_config(with_task.value(), no_task.value());
  const auto file = load_config<kN>(kRepo / "config/premium_fr3.yaml");
  ASSERT_TRUE(file.is_ok()) << file.reason();
  expect_same_config(with_task.value(), file.value());

  const auto imp = load_imperfections_with<kN>(m.imperfections, ro.imperfections);
  ASSERT_TRUE(imp.is_ok()) << imp.reason();
  EXPECT_EQ(imp.value().model.payload_kg, 0.0);
  EXPECT_EQ(imp.value().model.payload_radius_m, 0.0);
  EXPECT_TRUE(imp.value().model.payload_offset_m == Vec3::Zero());
}

// The chosen algorithm reaches the run: the Algorithm argument, the limits and the overrides.
TEST(RunOverrides, ConfigListIsTheWholeRecipeOfTheAlgorithm) {
  const HardwareModel m = model_named("budget_xarm7");
  const RunOverrides ro = run_overrides(
      m, Algorithm::kDrivePosition, task_of(kRepo / "config/tasks/reference_move.yaml"));
  const auto c = load_config_with<kN>(m.controller, ro.config);
  ASSERT_TRUE(c.is_ok()) << c.reason();
  const auto file = load_config<kN>(kRepo / "config/budget_xarm7_drive.yaml");
  ASSERT_TRUE(file.is_ok()) << file.reason();
  expect_same_config(c.value(), file.value());
}

TEST(RunOverrides, NoKeyTwice) {
  const HardwareModel m = model_named("budget_xarm7");
  const Task task = task_of(write_task(task_fixture_text()));
  for (const Algorithm a :
       {Algorithm::kComputedTorque, Algorithm::kPdGravity, Algorithm::kDrivePosition}) {
    const RunOverrides ro = run_overrides(m, a, task);
    std::set<std::string> seen;
    for (const KeyOverride& o : ro.config) {
      EXPECT_TRUE(seen.insert(o.key).second) << o.key;
    }
    for (const KeyOverride& o : ro.imperfections) {
      EXPECT_TRUE(seen.insert(o.key).second) << o.key;
    }
    // load_config_with refuses a key twice and omega together with kp or kd.
    const auto c = load_config_with<kN>(m.controller, ro.config);
    EXPECT_TRUE(c.is_ok()) << c.reason();
  }
}

}  // namespace
}  // namespace cobot::pipeline
