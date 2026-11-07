// Hardware model loader (brief hardware-catalog 1, ADR-023, ADR-009 strict keys):
// config/hardware/<model>.yaml -> pipeline::HardwareModel through load_hardware_model. A missing
// required key, an unknown key at any level, a bad type or an unknown algorithm name is an error
// naming the file and the full key; files named by the model must exist and load. Paths are
// relative to the model file.
#include <filesystem>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/control/algorithm.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/hardware_model.hpp"
#include "cobot/pipeline/overrides.hpp"

#include "hardware_fixture.hpp"
#include "hw_test_util.hpp"

namespace cobot::pipeline {
namespace {

using namespace hw_test;

// The fixture itself loads: without this every error test below could pass for a wrong reason.
TEST(HardwareModelLoader, FixtureLoadsWithItsFields) {
  const fs::path path = write_fixture(hardware_fixture_text());
  const auto r = load_hardware_model(path);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const HardwareModel& m = r.value();
  EXPECT_EQ(m.name, "fixture_model");
  EXPECT_EQ(m.description, "a model for the loader tests");
  EXPECT_TRUE(same_path(m.imperfections, path.parent_path() / "fixture_profile.yaml"))
      << m.imperfections;
  EXPECT_TRUE(same_path(m.controller, kRepo / "config/premium_fr3.yaml")) << m.controller;
  EXPECT_EQ(m.provenance.size(), 6U);
}

// The fixture's files are valid for today's loaders, whatever load_hardware_model does.
TEST(HardwareModelFixture, ProfileAndOverridesLoadThroughTheExistingLoaders) {
  const fs::path path = write_fixture(hardware_fixture_text());
  const auto profile = load_imperfections<kN>(path.parent_path() / "fixture_profile.yaml");
  EXPECT_TRUE(profile.is_ok()) << profile.reason();
  const std::vector<double> kp{900, 900, 900, 900, 900, 900, 800};
  const std::vector<double> rated{80, 80, 50, 50, 9, 9, 9};
  const std::vector<double> peak{150, 150, 90, 90, 30, 30, 30};
  const auto c =
      load_config_with<kN>(kRepo / "config/premium_fr3.yaml",
                           {KeyOverride{"config.control.algorithm", std::string("pd_gravity")},
                            KeyOverride{"config.limits.rated_torque_nm", rated},
                            KeyOverride{"config.limits.peak_torque_nm", peak},
                            KeyOverride{"config.pd_gravity.kp", kp},
                            KeyOverride{"config.pd_gravity.kd", 60.0},
                            KeyOverride{"config.deflection_comp.enabled", false},
                            KeyOverride{"config.log.mode", std::string("off")}});
  ASSERT_TRUE(c.is_ok()) << c.reason();
  EXPECT_TRUE(c.value().log_mode == LogMode::kOff);
  EXPECT_FALSE(c.value().deflection_comp.enabled);
}

TEST(HardwareModelLoader, EachMissingRequiredKeyIsNamed) {
  struct Case {
    const char* line;  // the fixture block removed
    const char* key;
  };
  const std::vector<Case> cases{
      {"  name:", "hardware.name"},
      {"  description:", "hardware.description"},
      {"  imperfections:", "hardware.imperfections"},
      {"  controller:", "hardware.controller"},
      {"  limits:", "hardware.limits"},
      {"    rated_torque_nm:", "hardware.limits.rated_torque_nm"},
      {"    peak_torque_nm:", "hardware.limits.peak_torque_nm"},
      {"  algorithms:", "hardware.algorithms"},
      {"      overrides:", "hardware.algorithms.computed_torque.overrides"},
      {"  provenance:", "hardware.provenance"},
  };
  int n = 0;
  for (const Case& c : cases) {
    SCOPED_TRACE(c.key);
    expect_load_error(
        write_fixture(drop_block(hardware_fixture_text(), c.line), "missing" + std::to_string(n++)),
        {c.key});
  }
}

TEST(HardwareModelLoader, MissingKeyOfAProvenanceEntryIsNamed) {
  const std::string text = hardware_fixture_text();
  expect_load_error(write_fixture(set_line(text, kEntryBits, "    - {kind: published}"), "no_key"),
                    {"hardware.provenance[0].key"});
  expect_load_error(
      write_fixture(set_line(text, kEntryBits, "    - {key: sensor.encoder_bits}"), "no_kind"),
      {"hardware.provenance[0].kind"});
}

TEST(HardwareModelLoader, MissingHardwareSectionIsNamed) {
  expect_load_error(write_fixture("other: 1\n"), {"hardware"});
}

TEST(HardwareModelLoader, UnknownKeyAtEachLevelIsNamed) {
  const std::string text = hardware_fixture_text();
  expect_load_error(write_fixture(text + "bogus_key: 1\n", "root"), {"bogus_key"});
  expect_load_error(write_fixture(insert_after(text, "  name:", "  bogus_key: 1"), "hardware"),
                    {"hardware.bogus_key"});
  expect_load_error(write_fixture(insert_after(text, "  limits:", "    bogus_key: [1]"), "limits"),
                    {"hardware.limits.bogus_key"});
  expect_load_error(
      write_fixture(insert_after(text, "    computed_torque:", "      bogus_key: 1"), "algorithm"),
      {"hardware.algorithms.computed_torque.bogus_key"});
  expect_load_error(
      write_fixture(
          set_line(
              text, kEntryBits, "    - {key: sensor.encoder_bits, kind: published, bogus_key: 1}"),
          "entry"),
      {"hardware.provenance[0].bogus_key"});
}

// An override that is not a key of the controller file is the strict loader's error, at the latest
// from config_for, naming the key.
TEST(HardwareModelLoader, UnknownOverrideKeyIsNamed) {
  const fs::path path = write_fixture(insert_after(hardware_fixture_text(),
                                                   "        config.computed_torque.omega:",
                                                   "        config.safety.bogus_key: 1.0"));
  const auto r = load_hardware_model(path);
  if (!r.is_ok()) {
    EXPECT_TRUE(contains(r.reason(), "safety.bogus_key")) << r.reason();
    return;
  }
  const auto c = config_for<kN>(r.value(), control::Algorithm::kComputedTorque);
  ASSERT_FALSE(c.is_ok());
  EXPECT_TRUE(contains(c.reason(), "safety.bogus_key")) << c.reason();
}

TEST(HardwareModelLoader, UnknownAlgorithmNameIsNamed) {
  expect_load_error(
      write_fixture(insert_after(
          hardware_fixture_text(), "  algorithms:", "    no_such_law: {overrides: {}}")),
      {"hardware.algorithms.no_such_law"});
}

TEST(HardwareModelLoader, BadTypesAreNamed) {
  struct Case {
    const char* line;
    const char* replacement;
    const char* key;
  };
  const std::vector<Case> cases{
      {"  name:", "  name: [a, b]", "hardware.name"},
      {"  description:", "  description: {a: 1}", "hardware.description"},
      {"  imperfections:", "  imperfections: [a]", "hardware.imperfections"},
      {"  controller:", "  controller: [a]", "hardware.controller"},
      {"  limits:", "  limits: 5", "hardware.limits"},
      {"    rated_torque_nm:",
       "    rated_torque_nm: [80, 80, 50, 50, 9, 9]",
       "hardware.limits.rated_torque_nm"},
      {"    peak_torque_nm:",
       "    peak_torque_nm: [a, 150, 90, 90, 30, 30, 30]",
       "hardware.limits.peak_torque_nm"},
      {"  algorithms:", "  algorithms: 5", "hardware.algorithms"},
      {"      overrides:", "      overrides: 5", "hardware.algorithms.computed_torque.overrides"},
      {"        config.computed_torque.omega:",
       "        config.computed_torque.omega: {a: 1}",
       "config.computed_torque.omega"},
      {"  provenance:", "  provenance: {a: 1}", "hardware.provenance"},
  };
  int n = 0;
  for (const Case& c : cases) {
    SCOPED_TRACE(c.key);
    expect_load_error(write_fixture(set_block(hardware_fixture_text(), c.line, c.replacement),
                                    "bad" + std::to_string(n++)),
                      {c.key});
  }
  expect_load_error(
      write_fixture(set_line(hardware_fixture_text(), kEntryBits, "    - 5"), "entry"),
      {"hardware.provenance[0]"});
}

TEST(HardwareModelLoader, MissingModelFileIsAnErrorNamingIt) {
  const auto r = load_hardware_model(test_dir() / "no_such_model.yaml");
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(r.reason(), "no_such_model.yaml")) << r.reason();
}

TEST(HardwareModelLoader, MissingReferencedFileNamesTheKeyAndThePath) {
  const std::string text = hardware_fixture_text();
  expect_load_error(
      write_fixture(set_line(text, "  imperfections:", "  imperfections: no_such_profile.yaml"),
                    "profile"),
      {"hardware.imperfections", "no_such_profile.yaml"});
  expect_load_error(
      write_fixture(set_line(text, "  controller:", "  controller: no_such_controller.yaml"),
                    "controller"),
      {"hardware.controller", "no_such_controller.yaml"});
}

TEST(HardwareModelLoader, ReferencedFilesMustLoad) {
  // The profile has an unknown key: the error is the profile's own, naming it.
  const std::string bad_profile =
      insert_after(profile_fixture_text(), "    encoder_bits:", "    bogus_key: 1");
  const auto r = load_hardware_model(write_fixture(hardware_fixture_text(), "p", bad_profile));
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(r.reason(), "imperfections.sensor.bogus_key")) << r.reason();

  // The controller file is not a controller config.
  write_text(test_dir() / "c" / "broken_controller.yaml", "cobot: {bogus_key: 1}\n");
  const fs::path model = write_fixture(
      set_line(hardware_fixture_text(), "  controller:", "  controller: broken_controller.yaml"),
      "c");
  const auto c = load_hardware_model(model);
  ASSERT_FALSE(c.is_ok());
  EXPECT_TRUE(contains(c.reason(), "broken_controller.yaml")) << c.reason();
}

// Peak torque above the URDF effort M: the controller loader's rule, at the latest from config_for.
TEST(HardwareModelLoader, PeakAboveTheUrdfEffortIsNamed) {
  const fs::path path =
      write_fixture(set_line(hardware_fixture_text(),
                             "    peak_torque_nm:",
                             "    peak_torque_nm: [1.0e6, 150, 90, 90, 30, 30, 30]"));
  const auto r = load_hardware_model(path);
  if (!r.is_ok()) {
    EXPECT_TRUE(contains(r.reason(), "limits.peak_torque_nm")) << r.reason();
    return;
  }
  const auto c = config_for<kN>(r.value(), control::Algorithm::kComputedTorque);
  ASSERT_FALSE(c.is_ok());
  EXPECT_TRUE(contains(c.reason(), "limits.peak_torque_nm")) << c.reason();
}

// --- the shipped models ---

TEST(HardwareModelFiles, AllThreeLoadWithTheirFiles) {
  for (const char* name : {"premium_fr3", "mid_kr1205", "budget_xarm7"}) {
    SCOPED_TRACE(name);
    const std::string file = std::string(name) + ".yaml";
    const fs::path path = kRepo / "config/hardware" / file;
    const auto r = load_hardware_model(path);
    ASSERT_TRUE(r.is_ok()) << r.reason();
    EXPECT_EQ(r.value().name, name);
    EXPECT_FALSE(r.value().description.empty());
    EXPECT_TRUE(same_path(r.value().imperfections, kRepo / "config/imperfections" / file));
    EXPECT_TRUE(same_path(r.value().controller, kRepo / "config" / file));
    EXPECT_EQ(read_text(path).rfind('#', 0), 0U) << "a comment at the top of the file";
  }
}

}  // namespace
}  // namespace cobot::pipeline
