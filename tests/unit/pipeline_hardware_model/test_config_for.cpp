// config_for (brief hardware-catalog 1): base controller file + config.control.algorithm + the
// model's limits + that algorithm's overrides, through load_config_with. An algorithm the model
// does not list is an error naming the model and the listed algorithms.
//
// Gains of the shipped models (config/<model>.yaml, critically damped, kp = w^2, kd = 2 w):
//   premium_fr3 w = 50: kp = 2500, kd = 100; mid_kr1205 w = 40: kp = 1600, kd = 80;
//   budget_xarm7 w = 20: kp = 400, kd = 40. pd_gravity uses the same omega (brief 1).
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
using control::Algorithm;
using Vec = VecN<kN>;

HardwareModel model_named(const std::string& name) {
  const auto r = load_hardware_model(kRepo / "config/hardware" / (name + ".yaml"));
  if (!r.is_ok()) {
    ADD_FAILURE() << name << ": " << r.reason();
    return HardwareModel{};
  }
  return r.value();
}

struct Tier {
  const char* name;
  double omega;
  double kp;
  double kd;
};

const std::vector<Tier>& tiers() {
  static const std::vector<Tier> all{{"premium_fr3", 50.0, 2500.0, 100.0},
                                     {"mid_kr1205", 40.0, 1600.0, 80.0},
                                     {"budget_xarm7", 20.0, 400.0, 40.0}};
  return all;
}

std::vector<double> to_list(const Vec& v) {
  return {v.data(), v.data() + kN};
}

// The torque levels of config/<model>.yaml, the same for all three.
Vec rated() {
  Vec v;
  v << 87, 87, 52, 52, 10, 10, 10;
  return v;
}

Vec peak() {
  Vec v;
  v << 204, 204, 107, 107, 36, 36, 36;
  return v;
}

TEST(ConfigFor, ComputedTorqueEqualsLoadConfigWithOfTheSameOverrides) {
  for (const Tier& t : tiers()) {
    SCOPED_TRACE(t.name);
    const HardwareModel m = model_named(t.name);
    const auto got = config_for<kN>(m, Algorithm::kComputedTorque);
    ASSERT_TRUE(got.is_ok()) << got.reason();
    const std::vector<KeyOverride> same{
        {"config.control.algorithm", std::string("computed_torque")},
        {"config.limits.rated_torque_nm", to_list(rated())},
        {"config.limits.peak_torque_nm", to_list(peak())},
        {kOmegaKey, t.omega},
    };
    const auto expected = load_config_with<kN>(m.controller, same);
    ASSERT_TRUE(expected.is_ok()) << expected.reason();
    expect_same_config(got.value(), expected.value());
  }
}

TEST(ConfigFor, ComputedTorqueEqualsThePerProfileConfig) {
  for (const Tier& t : tiers()) {
    SCOPED_TRACE(t.name);
    const auto got = config_for<kN>(model_named(t.name), Algorithm::kComputedTorque);
    ASSERT_TRUE(got.is_ok()) << got.reason();
    EXPECT_TRUE(got.value().algorithm == Algorithm::kComputedTorque);
    EXPECT_TRUE(got.value().computed_torque.kp == Vec::Constant(t.kp));
    EXPECT_TRUE(got.value().computed_torque.kd == Vec::Constant(t.kd));
    const auto base = load_config<kN>(kRepo / "config" / (std::string(t.name) + ".yaml"));
    ASSERT_TRUE(base.is_ok()) << base.reason();
    expect_same_config(got.value(), base.value());
  }
}

// No virtual key yet: explicit kp, kd at the omega of the model's computed torque.
TEST(ConfigFor, PdGravityGainsAtTheSameOmega) {
  for (const Tier& t : tiers()) {
    SCOPED_TRACE(t.name);
    const auto got = config_for<kN>(model_named(t.name), Algorithm::kPdGravity);
    ASSERT_TRUE(got.is_ok()) << got.reason();
    EXPECT_TRUE(got.value().algorithm == Algorithm::kPdGravity);
    EXPECT_TRUE(got.value().pd_gravity.kp == Vec::Constant(t.kp)) << got.value().pd_gravity.kp;
    EXPECT_TRUE(got.value().pd_gravity.kd == Vec::Constant(t.kd)) << got.value().pd_gravity.kd;
    // The rest stays the computed-torque config.
    const auto ct = config_for<kN>(model_named(t.name), Algorithm::kComputedTorque);
    ASSERT_TRUE(ct.is_ok()) << ct.reason();
    EXPECT_TRUE(got.value().computed_torque.kp == ct.value().computed_torque.kp);
    EXPECT_TRUE(got.value().limits.peak_torque_nm == ct.value().limits.peak_torque_nm);
    EXPECT_EQ(got.value().safety.hal_timeout_s, ct.value().safety.hal_timeout_s);
  }
}

// The brief's reading of budget_xarm7_drive.yaml, checked on the loader alone: these overrides on
// the base file are the whole difference between the two files.
TEST(ConfigFor, BudgetDriveFileIsTheBaseWithTheBriefsOverrides) {
  const std::vector<KeyOverride> drive{
      {"config.control.algorithm", std::string("drive_position")},
      {"config.drive.rate_hz", 4000.0},
      {"config.drive.omega_rad_s", 80.0},
      {"config.drive.zeta", 1.0},
      {"config.drive.velocity_filter_hz", 500.0},
      {"config.drive.torque_feedforward", std::string("inverse_dynamics")},
      {"config.safety.hal_timeout_ms", 20.0},
  };
  const auto with = load_config_with<kN>(kRepo / "config/budget_xarm7.yaml", drive);
  ASSERT_TRUE(with.is_ok()) << with.reason();
  const auto file = load_config<kN>(kRepo / "config/budget_xarm7_drive.yaml");
  ASSERT_TRUE(file.is_ok()) << file.reason();
  expect_same_config(with.value(), file.value());
}

TEST(ConfigFor, BudgetDrivePositionEqualsTheDriveFile) {
  const auto got = config_for<kN>(model_named("budget_xarm7"), Algorithm::kDrivePosition);
  ASSERT_TRUE(got.is_ok()) << got.reason();
  const auto file = load_config<kN>(kRepo / "config/budget_xarm7_drive.yaml");
  ASSERT_TRUE(file.is_ok()) << file.reason();
  expect_same_config(got.value(), file.value());

  // Spelled out from the file: 4 kHz, omega 80, zeta 1, 500 Hz, 20 ms = 0.02 s.
  const Config<kN>& c = got.value();
  EXPECT_TRUE(c.algorithm == Algorithm::kDrivePosition);
  EXPECT_EQ(c.drive.rate_hz, 4000.0);
  EXPECT_TRUE(c.drive.omega_rad_s == Vec::Constant(80.0));
  EXPECT_EQ(c.drive.zeta, 1.0);
  EXPECT_EQ(c.drive.velocity_filter_hz, 500.0);
  EXPECT_TRUE(c.drive.torque_feedforward == control::DriveFeedForward::kInverseDynamics);
  EXPECT_DOUBLE_EQ(c.safety.hal_timeout_s, 0.020);
  EXPECT_TRUE(c.limits.rated_torque_nm == rated());
  EXPECT_TRUE(c.limits.peak_torque_nm == peak());
}

// Premium and mid list computed_torque and pd_gravity only (brief 1).
TEST(ConfigFor, AlgorithmNotListedNamesTheModelAndTheListedAlgorithms) {
  for (const char* name : {"premium_fr3", "mid_kr1205"}) {
    SCOPED_TRACE(name);
    const auto r = config_for<kN>(model_named(name), Algorithm::kDrivePosition);
    ASSERT_FALSE(r.is_ok());
    EXPECT_TRUE(contains(r.reason(), name)) << r.reason();
    EXPECT_TRUE(contains(r.reason(), "computed_torque")) << r.reason();
    EXPECT_TRUE(contains(r.reason(), "pd_gravity")) << r.reason();
  }
}

TEST(ConfigFor, FixtureAlgorithmNotListedNamesBoth) {
  const auto m =
      load_hardware_model(write_fixture(drop_block(hardware_fixture_text(), "    pd_gravity:")));
  ASSERT_TRUE(m.is_ok()) << m.reason();
  const auto r = config_for<kN>(m.value(), Algorithm::kPdGravity);
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(r.reason(), "fixture_model")) << r.reason();
  EXPECT_TRUE(contains(r.reason(), "computed_torque")) << r.reason();
}

// Fixture on config/premium_fr3.yaml: limits replace the file's; omega 30 gives kp = 900, kd = 60.
TEST(ConfigFor, LimitsAndOverridesReplaceTheBaseFile) {
  const auto m = load_hardware_model(write_fixture(hardware_fixture_text()));
  ASSERT_TRUE(m.is_ok()) << m.reason();
  const auto ct = config_for<kN>(m.value(), Algorithm::kComputedTorque);
  ASSERT_TRUE(ct.is_ok()) << ct.reason();
  Vec r;
  r << 80, 80, 50, 50, 9, 9, 9;
  Vec p;
  p << 150, 150, 90, 90, 30, 30, 30;
  EXPECT_TRUE(ct.value().limits.rated_torque_nm == r) << ct.value().limits.rated_torque_nm;
  EXPECT_TRUE(ct.value().limits.peak_torque_nm == p) << ct.value().limits.peak_torque_nm;
  EXPECT_TRUE(ct.value().computed_torque.kp == Vec::Constant(900.0));
  EXPECT_TRUE(ct.value().computed_torque.kd == Vec::Constant(60.0));
  // The overrides of the other algorithm are not applied.
  EXPECT_TRUE(ct.value().deflection_comp.enabled);
  EXPECT_TRUE(ct.value().log_mode == LogMode::kRecorder);
}

// Each value type of an override: list, scalar broadcast, bool, string.
TEST(ConfigFor, OverrideValuesOfEveryType) {
  const auto m = load_hardware_model(write_fixture(hardware_fixture_text()));
  ASSERT_TRUE(m.is_ok()) << m.reason();
  const auto pd = config_for<kN>(m.value(), Algorithm::kPdGravity);
  ASSERT_TRUE(pd.is_ok()) << pd.reason();
  const Config<kN>& c = pd.value();
  EXPECT_TRUE(c.algorithm == Algorithm::kPdGravity);
  Vec kp;
  kp << 900, 900, 900, 900, 900, 900, 800;
  EXPECT_TRUE(c.pd_gravity.kp == kp) << c.pd_gravity.kp;
  EXPECT_TRUE(c.pd_gravity.kd == Vec::Constant(60.0)) << c.pd_gravity.kd;
  EXPECT_FALSE(c.deflection_comp.enabled);
  EXPECT_TRUE(c.log_mode == LogMode::kOff);
}

}  // namespace
}  // namespace cobot::pipeline
