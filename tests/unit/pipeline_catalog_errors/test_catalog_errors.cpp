// Cases of the hardware model and task loaders that the review of #76 listed as uncovered: the
// payload section's required and optional keys, errors at load rather than at a later config_for,
// the keys a model may not set because the task owns them, and run_overrides on an algorithm the
// model does not list (a precondition: the error comes from the controller loader).
#include <algorithm>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/control/algorithm.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/hardware_model.hpp"
#include "cobot/pipeline/overrides.hpp"
#include "cobot/pipeline/task.hpp"

#include "hardware_fixture.hpp"
#include "hw_test_util.hpp"
#include "task_fixture.hpp"

namespace cobot::pipeline {
namespace {

using namespace hw_test;
using common::Vec3;
using control::Algorithm;

TEST(TaskPayloadKeys, MissingKgIsNamed) {
  const fs::path path = write_task(drop_block(task_fixture_text(), "    kg:"));
  const auto r = load_task(path);
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(r.reason(), path.filename().string())) << r.reason();
  EXPECT_TRUE(contains(r.reason(), "task.payload.kg")) << r.reason();
}

TEST(TaskPayloadKeys, OffsetRadiusAndKnownAreOptional) {
  std::string text = drop_block(task_fixture_text(), "    offset_m:");
  text = drop_block(text, "    radius_m:");
  text = drop_block(text, "    known:");
  const auto r = load_task(write_task(text));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const TaskPayload& p = r.value().payload;
  EXPECT_EQ(p.kg, 2.0);
  EXPECT_TRUE(p.offset_m == Vec3::Zero());
  EXPECT_EQ(p.radius_m, 0.0);
  EXPECT_TRUE(p.known);
}

// load_hardware_model itself reports a bad override, under the algorithm's key: the frozen test of
// this case also accepts a later config_for.
TEST(HardwareModelErrors, BadOverrideFailsAtLoadUnderTheAlgorithmKey) {
  const fs::path path = write_fixture(insert_after(hardware_fixture_text(),
                                                   "        config.computed_torque.omega:",
                                                   "        config.safety.bogus_key: 1.0"));
  expect_load_error(path, {"hardware.algorithms.computed_torque", "safety.bogus_key"});
}

TEST(HardwareModelErrors, OverrideOfATaskOwnedKeyIsNamed) {
  const std::vector<std::string> keys{"config.payload.kg", "config.trajectory.duration_s"};
  int n = 0;
  for (const std::string& key : keys) {
    SCOPED_TRACE(key);
    const fs::path path =
        write_fixture(insert_after(hardware_fixture_text(),
                                   "        config.computed_torque.omega:",
                                   "        " + key + ": 1.0"),
                      "case" + std::to_string(n++));
    expect_load_error(path,
                      {"hardware.algorithms.computed_torque.overrides." + key, "owned by the task"});
  }
}

TEST(HardwareModelErrors, ControllerFileWithAPayloadIsRefused) {
  const fs::path controller = test_dir() / "controller_with_payload.yaml";
  const std::vector<KeyOverride> payload{{"config.payload.kg", 1.0}};
  const auto written = write_config_with(kRepo / "config/premium_fr3.yaml", payload, controller);
  ASSERT_TRUE(written.is_ok()) << written.reason();
  const fs::path path = write_fixture(set_line(
      hardware_fixture_text(), "  controller:", "  controller: " + controller.generic_string()));
  expect_load_error(path, {"hardware.controller", "cobot.payload"});
}

TEST(HardwareModelErrors, ControllerFileWithAZeroPayloadLoads) {
  const fs::path controller = test_dir() / "controller_zero_payload.yaml";
  const std::vector<KeyOverride> payload{{"config.payload.kg", 0.0}};
  const auto written = write_config_with(kRepo / "config/premium_fr3.yaml", payload, controller);
  ASSERT_TRUE(written.is_ok()) << written.reason();
  const fs::path path = write_fixture(set_line(
      hardware_fixture_text(), "  controller:", "  controller: " + controller.generic_string()));
  const auto r = load_hardware_model(path);
  EXPECT_TRUE(r.is_ok()) << r.reason();
}

// The documented behaviour of task.hpp: no gains of the model, and the controller loader judges
// the result. premium_fr3.yaml has no cobot.drive section, which drive_position requires.
TEST(RunOverridesPrecondition, UnlistedAlgorithmHasNoModelGainsAndTheControllerLoaderRefusesIt) {
  const auto model = load_hardware_model(kRepo / "config/hardware/premium_fr3.yaml");
  ASSERT_TRUE(model.is_ok()) << model.reason();
  const auto task = load_task(kRepo / "config/tasks/reference_move.yaml");
  ASSERT_TRUE(task.is_ok()) << task.reason();

  const RunOverrides o = run_overrides(model.value(), Algorithm::kDrivePosition, task.value());
  const auto has_prefix = [&o](const std::string& prefix) {
    return std::any_of(o.config.begin(), o.config.end(), [&prefix](const KeyOverride& k) {
      return k.key.rfind(prefix, 0) == 0;
    });
  };
  EXPECT_TRUE(has_prefix("config.control.algorithm"));
  EXPECT_FALSE(has_prefix("config.computed_torque."));
  EXPECT_FALSE(has_prefix("config.drive."));

  const auto config = load_config_with<kN>(model.value().controller, o.config);
  ASSERT_FALSE(config.is_ok());
  EXPECT_TRUE(contains(config.reason(), "drive")) << config.reason();

  const auto checked = config_for<kN>(model.value(), Algorithm::kDrivePosition);
  ASSERT_FALSE(checked.is_ok());
  EXPECT_TRUE(contains(checked.reason(), "premium_fr3")) << checked.reason();
}

}  // namespace
}  // namespace cobot::pipeline
