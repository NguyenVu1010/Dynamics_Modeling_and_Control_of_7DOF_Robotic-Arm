// The gain family of auto-tune (brief auto-tune, section 1; plan "Gain family per algorithm"):
// tuning_key and default_levels per law, the virtual key config.pd_gravity.omega (kp = w^2, kd = 2
// w, as config.computed_torque.omega), and with_gain, which replaces what the model sets for that
// gain so the list loads: "the same key twice" and "omega with kp" are loader errors.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <map>
#include <string>
#include <variant>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/control/algorithm.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/hardware_model.hpp"
#include "cobot/pipeline/overrides.hpp"
#include "cobot/pipeline/task.hpp"

#include "run_data.hpp"
#include "tune.hpp"

namespace cobot::bench {
namespace {

namespace fs = std::filesystem;
using control::Algorithm;
using pipeline::KeyOverride;
using Overrides = std::vector<KeyOverride>;

const fs::path kRepo = COBOT_REPO_DIR;
const fs::path kDefault = kRepo / "config/default.yaml";
const std::string kCtOmega = "config.computed_torque.omega";
const std::string kCtKp = "config.computed_torque.kp";
const std::string kCtKd = "config.computed_torque.kd";
const std::string kPdOmega = "config.pd_gravity.omega";
const std::string kPdKp = "config.pd_gravity.kp";
const std::string kPdKd = "config.pd_gravity.kd";
const std::string kDriveOmega = "config.drive.omega_rad_s";
const std::string kHalTimeout = "config.safety.hal_timeout_ms";

bool contains(const std::string& text, const std::string& part) {
  return text.find(part) != std::string::npos;
}

std::size_t count_key(const Overrides& list, const std::string& key) {
  return static_cast<std::size_t>(std::count_if(
      list.begin(), list.end(), [&key](const KeyOverride& o) { return o.key == key; }));
}

// The double value of the first entry of `key`; NaN when it is missing or not a double.
double value_of(const Overrides& list, const std::string& key) {
  for (const KeyOverride& o : list) {
    const double* v = o.key == key ? std::get_if<double>(&o.value) : nullptr;
    if (v != nullptr) {
      return *v;
    }
  }
  return std::numeric_limits<double>::quiet_NaN();
}

template <typename Gains>
void expect_gains(const Gains& gains, double kp, double kd) {
  for (int j = 0; j < kN; ++j) {
    EXPECT_EQ(gains.kp(j), kp) << "joint " << j;
    EXPECT_EQ(gains.kd(j), kd) << "joint " << j;
  }
}

TEST(TuningKey, IsTheOneGainOfEachLaw) {
  EXPECT_STREQ(tuning_key(Algorithm::kComputedTorque), "config.computed_torque.omega");
  EXPECT_STREQ(tuning_key(Algorithm::kPdGravity), "config.pd_gravity.omega");
  EXPECT_STREQ(tuning_key(Algorithm::kDrivePosition), "config.drive.omega_rad_s");
}

TEST(TuningKey, EveryRegisteredAlgorithmHasAKeyAndLevels) {
  for (const control::AlgorithmInfo& info : control::kAlgorithms) {
    const char* key = tuning_key(info.id);
    ASSERT_NE(key, nullptr) << info.name;
    EXPECT_TRUE(contains(key, "config.")) << info.name << ": " << key;
    EXPECT_FALSE(default_levels(info.id).empty()) << info.name;
  }
}

TEST(DefaultLevels, TorqueLawsSweep20To150By10) {
  // (150 - 20) / 10 + 1 = 14 levels.
  const std::vector<double> expected{
      20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 110.0, 120.0, 130.0, 140.0, 150.0};
  EXPECT_EQ(default_levels(Algorithm::kComputedTorque), expected);
  EXPECT_EQ(default_levels(Algorithm::kPdGravity), expected);
}

TEST(DefaultLevels, DriveSweeps20To200By10) {
  // (200 - 20) / 10 + 1 = 19 levels.
  const std::vector<double> expected{20.0,
                                     30.0,
                                     40.0,
                                     50.0,
                                     60.0,
                                     70.0,
                                     80.0,
                                     90.0,
                                     100.0,
                                     110.0,
                                     120.0,
                                     130.0,
                                     140.0,
                                     150.0,
                                     160.0,
                                     170.0,
                                     180.0,
                                     190.0,
                                     200.0};
  EXPECT_EQ(default_levels(Algorithm::kDrivePosition), expected);
}

TEST(PdGravityOmega, SetsKpAndKdOnEveryJoint) {
  // omega 70: kp = 70^2 = 4900, kd = 2 * 70 = 140.
  const auto r = pipeline::load_config_with<kN>(
      kDefault,
      Overrides{KeyOverride{kPdOmega, 70.0},
                KeyOverride{"config.control.algorithm", std::string("pd_gravity")}});
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_TRUE(r.value().algorithm == Algorithm::kPdGravity);
  expect_gains(r.value().pd_gravity, 4900.0, 140.0);
}

TEST(PdGravityOmega, SingleKeyLoaderAcceptsIt) {
  // The cobot_bench sweep path, as config.computed_torque.omega: omega 40 -> 1600, 80.
  const auto r = pipeline::load_config_with<kN>(kDefault, kPdOmega, 40.0);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  expect_gains(r.value().pd_gravity, 1600.0, 80.0);
}

TEST(PdGravityOmega, WrittenFileHoldsTheGains) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "bench_tune";
  fs::create_directories(dir);
  const fs::path dst = dir / "pd_omega.yaml";
  fs::remove(dst);
  const common::Status st =
      pipeline::write_config_with(kDefault, Overrides{KeyOverride{kPdOmega, 30.0}}, dst);
  ASSERT_TRUE(st.is_ok()) << st.reason();
  // omega 30: kp = 900, kd = 60.
  const auto r = pipeline::load_config<kN>(dst);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  expect_gains(r.value().pd_gravity, 900.0, 60.0);
}

TEST(PdGravityOmega, MustBeAFiniteNumberAboveZero) {
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const double w : {0.0, -5.0, inf, nan}) {
    const auto r = pipeline::load_config_with<kN>(kDefault, Overrides{KeyOverride{kPdOmega, w}});
    ASSERT_FALSE(r.is_ok()) << w;
    EXPECT_TRUE(contains(r.reason(), kPdOmega)) << r.reason();
  }
  const auto text =
      pipeline::load_config_with<kN>(kDefault, Overrides{KeyOverride{kPdOmega, std::string("70")}});
  EXPECT_FALSE(text.is_ok());
}

TEST(PdGravityOmega, WithKpOrKdInTheSameListIsAnError) {
  const auto with_kp = pipeline::load_config_with<kN>(
      kDefault, Overrides{KeyOverride{kPdOmega, 70.0}, KeyOverride{kPdKp, 900.0}});
  ASSERT_FALSE(with_kp.is_ok());
  EXPECT_TRUE(contains(with_kp.reason(), kPdOmega)) << with_kp.reason();
  const auto with_kd = pipeline::load_config_with<kN>(
      kDefault, Overrides{KeyOverride{kPdKd, 60.0}, KeyOverride{kPdOmega, 70.0}});
  ASSERT_FALSE(with_kd.is_ok());
  EXPECT_TRUE(contains(with_kd.reason(), kPdOmega)) << with_kd.reason();
}

TEST(PdGravityOmega, IsUnusedByComputedTorque) {
  const auto base = pipeline::load_config<kN>(kDefault);
  ASSERT_TRUE(base.is_ok()) << base.reason();
  const auto r = pipeline::load_config_with<kN>(kDefault, Overrides{KeyOverride{kPdOmega, 70.0}});
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_TRUE(r.value().algorithm == Algorithm::kComputedTorque);
  EXPECT_TRUE(r.value().computed_torque.kp == base.value().computed_torque.kp);
  EXPECT_TRUE(r.value().computed_torque.kd == base.value().computed_torque.kd);
}

TEST(PdGravityOmega, TheTwoVirtualKeysDoNotMix) {
  // CT omega 50 -> 2500, 100; PD omega 70 -> 4900, 140; each with the other's explicit gains.
  const auto both = pipeline::load_config_with<kN>(
      kDefault, Overrides{KeyOverride{kCtOmega, 50.0}, KeyOverride{kPdOmega, 70.0}});
  ASSERT_TRUE(both.is_ok()) << both.reason();
  expect_gains(both.value().computed_torque, 2500.0, 100.0);
  expect_gains(both.value().pd_gravity, 4900.0, 140.0);
  const auto ct_with_pd_gains = pipeline::load_config_with<kN>(
      kDefault,
      Overrides{KeyOverride{kCtOmega, 50.0}, KeyOverride{kPdKp, 900.0}, KeyOverride{kPdKd, 60.0}});
  ASSERT_TRUE(ct_with_pd_gains.is_ok()) << ct_with_pd_gains.reason();
  expect_gains(ct_with_pd_gains.value().pd_gravity, 900.0, 60.0);
  const auto pd_with_ct_gains = pipeline::load_config_with<kN>(
      kDefault,
      Overrides{KeyOverride{kPdOmega, 70.0}, KeyOverride{kCtKp, 900.0}, KeyOverride{kCtKd, 60.0}});
  ASSERT_TRUE(pd_with_ct_gains.is_ok()) << pd_with_ct_gains.reason();
  expect_gains(pd_with_ct_gains.value().computed_torque, 900.0, 60.0);
}

TEST(WithGain, ReplacesTheSameKeyAndLeavesTheOthers) {
  const Overrides out =
      with_gain(Overrides{KeyOverride{kCtOmega, 50.0}, KeyOverride{kHalTimeout, 7.0}},
                Algorithm::kComputedTorque,
                70.0);
  ASSERT_EQ(out.size(), 2U);
  EXPECT_EQ(count_key(out, kCtOmega), 1U);
  EXPECT_EQ(value_of(out, kCtOmega), 70.0);
  EXPECT_EQ(value_of(out, kHalTimeout), 7.0);
}

TEST(WithGain, RemovesTheExplicitGainsOfTheVirtualKeysSection) {
  const Overrides ct = with_gain(Overrides{KeyOverride{kCtKp, 900.0}, KeyOverride{kCtKd, 60.0}},
                                 Algorithm::kComputedTorque,
                                 70.0);
  ASSERT_EQ(ct.size(), 1U);
  EXPECT_EQ(value_of(ct, kCtOmega), 70.0);

  const Overrides pd = with_gain(
      Overrides{
          KeyOverride{kPdKp, 2500.0}, KeyOverride{kHalTimeout, 7.0}, KeyOverride{kPdKd, 100.0}},
      Algorithm::kPdGravity,
      70.0);
  ASSERT_EQ(pd.size(), 2U);
  EXPECT_EQ(value_of(pd, kPdOmega), 70.0);
  EXPECT_EQ(value_of(pd, kHalTimeout), 7.0);
  EXPECT_EQ(count_key(pd, kPdKp), 0U);
  EXPECT_EQ(count_key(pd, kPdKd), 0U);
}

TEST(WithGain, LeavesTheGainsOfTheOtherLaws) {
  const Overrides all{KeyOverride{kCtKp, 1.0},
                      KeyOverride{kCtKd, 2.0},
                      KeyOverride{kPdKp, 3.0},
                      KeyOverride{kPdKd, 4.0},
                      KeyOverride{kDriveOmega, 5.0}};
  const Overrides ct = with_gain(all, Algorithm::kComputedTorque, 70.0);
  EXPECT_EQ(ct.size(), 4U);  // 5 - kp - kd + omega
  EXPECT_EQ(value_of(ct, kPdKp), 3.0);
  EXPECT_EQ(value_of(ct, kDriveOmega), 5.0);

  const Overrides pd = with_gain(all, Algorithm::kPdGravity, 70.0);
  EXPECT_EQ(pd.size(), 4U);
  EXPECT_EQ(value_of(pd, kCtKp), 1.0);
  EXPECT_EQ(value_of(pd, kDriveOmega), 5.0);

  const Overrides drive = with_gain(all, Algorithm::kDrivePosition, 70.0);
  EXPECT_EQ(drive.size(), 5U);  // omega_rad_s replaced, nothing else removed
  EXPECT_EQ(value_of(drive, kDriveOmega), 70.0);
  EXPECT_EQ(value_of(drive, kCtKp), 1.0);
  EXPECT_EQ(value_of(drive, kPdKd), 4.0);
}

TEST(WithGain, AddsTheLevelToAnEmptyList) {
  const Overrides out = with_gain({}, Algorithm::kDrivePosition, 70.0);
  ASSERT_EQ(out.size(), 1U);
  EXPECT_EQ(value_of(out, kDriveOmega), 70.0);
}

struct ModelCase {
  const char* name;
  std::vector<Algorithm> listed;
};

class WithGainOnModels : public ::testing::TestWithParam<ModelCase> {
 protected:
  static pipeline::HardwareModel load_model() {
    const auto m = pipeline::load_hardware_model(kRepo / "config/hardware" /
                                                 (std::string(GetParam().name) + ".yaml"));
    EXPECT_TRUE(m.is_ok()) << m.reason();
    return m ? m.value() : pipeline::HardwareModel{};
  }
};

// omega 70: kp = 70^2 = 4900, kd = 2 * 70 = 140 (torque laws); drive omega 70 on every joint.
void expect_level_70(const pipeline::Config<kN>& c, Algorithm a) {
  EXPECT_TRUE(c.algorithm == a);
  switch (a) {
    case Algorithm::kComputedTorque:
      expect_gains(c.computed_torque, 4900.0, 140.0);
      break;
    case Algorithm::kPdGravity:
      expect_gains(c.pd_gravity, 4900.0, 140.0);
      break;
    case Algorithm::kDrivePosition:
      for (int j = 0; j < kN; ++j) {
        EXPECT_EQ(c.drive.omega_rad_s(j), 70.0) << "joint " << j;
      }
      break;
  }
}

TEST_P(WithGainOnModels, ListsTheExpectedAlgorithms) {
  const pipeline::HardwareModel model = load_model();
  std::vector<Algorithm> listed;
  listed.reserve(model.algorithms.size());
  for (const auto& entry : model.algorithms) {
    listed.push_back(entry.first);
  }
  EXPECT_TRUE(listed == GetParam().listed);
}

TEST_P(WithGainOnModels, LoadsWithTheExpectedGainsForEachListedAlgorithm) {
  const pipeline::HardwareModel model = load_model();
  for (const Algorithm a : GetParam().listed) {
    const auto r = pipeline::load_config_with<kN>(
        model.controller, with_gain(pipeline::model_overrides(model, a), a, 70.0));
    ASSERT_TRUE(r.is_ok()) << control::to_string(a) << ": " << r.reason();
    expect_level_70(r.value(), a);
    // The model's torque levels stay (every model file: 204 N m on joint 0).
    EXPECT_EQ(r.value().limits.peak_torque_nm(0), 204.0) << control::to_string(a);
  }
}

TEST_P(WithGainOnModels, LoadsOnTheRunOverridesOfEachTask) {
  const pipeline::HardwareModel model = load_model();
  for (const char* file : {"reference_move.yaml", "pick_place_3kg.yaml"}) {
    const auto task = pipeline::load_task(kRepo / "config/tasks" / file);
    ASSERT_TRUE(task.is_ok()) << task.reason();
    for (const Algorithm a : GetParam().listed) {
      const pipeline::RunOverrides ro = pipeline::run_overrides(model, a, task.value());
      const auto r =
          pipeline::load_config_with<kN>(model.controller, with_gain(ro.config, a, 70.0));
      ASSERT_TRUE(r.is_ok()) << file << " " << control::to_string(a) << ": " << r.reason();
      expect_level_70(r.value(), a);
      EXPECT_EQ(r.value().payload.kg, task.value().payload.kg) << file;  // both tasks: known
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
    ReferenceModels,
    WithGainOnModels,
    ::testing::Values(
        ModelCase{"premium_fr3", {Algorithm::kComputedTorque, Algorithm::kPdGravity}},
        ModelCase{"mid_kr1205", {Algorithm::kComputedTorque, Algorithm::kPdGravity}},
        ModelCase{"budget_xarm7",
                  {Algorithm::kComputedTorque, Algorithm::kPdGravity, Algorithm::kDrivePosition}}),
    [](const ::testing::TestParamInfo<ModelCase>& info) { return info.param.name; });

TEST(WithGainOnDrive, KeepsTheModelsOtherDriveSettings) {
  const auto model = pipeline::load_hardware_model(kRepo / "config/hardware/budget_xarm7.yaml");
  ASSERT_TRUE(model.is_ok()) << model.reason();
  const Algorithm a = Algorithm::kDrivePosition;
  const auto r = pipeline::load_config_with<kN>(
      model.value().controller, with_gain(pipeline::model_overrides(model.value(), a), a, 70.0));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().drive.rate_hz, 4000.0);
  EXPECT_EQ(r.value().drive.zeta, 1.0);
  EXPECT_DOUBLE_EQ(r.value().safety.hal_timeout_s, 0.020);  // hal_timeout_ms 20
}

TEST(WithGainOnTorqueLaws, AppendingTheLevelWithoutItDoesNotLoad) {
  // Why with_gain exists: the model files set the CT omega and config.pd_gravity.kp/kd.
  const auto model = pipeline::load_hardware_model(kRepo / "config/hardware/premium_fr3.yaml");
  ASSERT_TRUE(model.is_ok()) << model.reason();
  for (const Algorithm a : {Algorithm::kComputedTorque, Algorithm::kPdGravity}) {
    Overrides list = pipeline::model_overrides(model.value(), a);
    list.push_back(KeyOverride{tuning_key(a), 70.0});
    EXPECT_FALSE(pipeline::load_config_with<kN>(model.value().controller, list).is_ok())
        << control::to_string(a);
  }
}

}  // namespace
}  // namespace cobot::bench
