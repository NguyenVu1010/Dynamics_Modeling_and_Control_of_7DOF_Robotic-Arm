// The noise level of the Configuration tab (feature studio-noise-level, studio config UI design
// 3.4): src/studio/core/include/cobot/studio/noise_level.hpp, Qt-free. The level is computed from
// the values (override, else base) with exact ==, the product being base * k; a base of 0 matches
// every k, so a zero key never forces Custom; k = 1 removes the five overrides and keeps the
// others.
#if __has_include("cobot/studio/noise_level.hpp")

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/pipeline/overrides.hpp"
#include "cobot/studio/noise_level.hpp"
#include "cobot/studio/param_catalog.hpp"
#include "cobot/studio/profile_catalog.hpp"
#include "cobot/studio/studio_config.hpp"

namespace cobot::studio {
namespace {

namespace fs = std::filesystem;
using pipeline::KeyOverride;
using Base = std::map<std::string, KeyOverride>;
using List = std::vector<double>;
using Value = decltype(KeyOverride::value);

constexpr const char* kQNoise = "sensor.q_noise_std_rad";
constexpr const char* kQOffset = "sensor.q_offset_std_rad";
constexpr const char* kQdNoise = "sensor.qd_noise_std_rad_s";
constexpr const char* kTorque = "actuator.torque_scale_std";
constexpr const char* kJitter = "timing.period_jitter_std_us";

// Five noise keys (qd's joint 1 has base 0, so it must stay 0 at every level) and omega, which is
// not a noise key.
Base hand_base() {
  Base b;
  b[kQNoise] = {kQNoise, List{1e-5, 2e-5, 3e-5, 4e-5, 5e-5, 6e-5, 7e-5}};
  b[kQOffset] = {kQOffset, List(7, 1e-3)};
  b[kQdNoise] = {kQdNoise, List{0.0, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02}};
  b[kTorque] = {kTorque, List(7, 0.1)};
  b[kJitter] = {kJitter, 100.0};
  b[pipeline::kOmegaKey] = {pipeline::kOmegaKey, 20.0};
  return b;
}

// The base value with every element multiplied by k (b * k, as the contract computes it).
Value scaled(const Value& v, double k) {
  if (const auto* d = std::get_if<double>(&v)) {
    return *d * k;
  }
  List out = std::get<List>(v);
  for (double& x : out) {
    x *= k;
  }
  return out;
}

// hand_base() with the five noise keys at 0 on every element.
Base zero_base() {
  Base b = hand_base();
  for (const char* key : kNoiseKeys) {
    b[key].value = scaled(b[key].value, 0.0);
  }
  return b;
}

KeyOverride* find(StudioConfig& c, const std::string& key) {
  const auto it = std::find_if(
      c.overrides.begin(), c.overrides.end(), [&](const KeyOverride& o) { return o.key == key; });
  return it == c.overrides.end() ? nullptr : &*it;
}

std::set<std::string> keys(const StudioConfig& c) {
  std::set<std::string> s;
  for (const KeyOverride& o : c.overrides) {
    s.insert(o.key);
  }
  return s;
}

StudioConfig with(std::vector<KeyOverride> overrides) {
  StudioConfig c;
  c.overrides = std::move(overrides);
  return c;
}

Base pair_base(const std::string& name) {
  for (const BasePair& p : discover_base_pairs(fs::path(COBOT_REPO_DIR) / "config")) {
    if (p.name == name) {
      return base_values(p);
    }
  }
  ADD_FAILURE() << "no base pair " << name;
  return {};
}

TEST(NoiseLevel, FactorsAndTheKeyList) {
  EXPECT_EQ(noise_factor(NoiseLevel::kOff), 0.0);
  EXPECT_EQ(noise_factor(NoiseLevel::kProfile), 1.0);
  EXPECT_EQ(noise_factor(NoiseLevel::kX2), 2.0);
  EXPECT_EQ(noise_factor(NoiseLevel::kX5), 5.0);
  EXPECT_TRUE(std::isnan(noise_factor(NoiseLevel::kCustom)));
  EXPECT_EQ(std::set<std::string>(kNoiseKeys.begin(), kNoiseKeys.end()),
            (std::set<std::string>{kQNoise, kQOffset, kQdNoise, kTorque, kJitter}));
}

TEST(NoiseLevel, NoOverridesIsProfile) {
  EXPECT_EQ(noise_level(StudioConfig{}, hand_base()), NoiseLevel::kProfile);
}

// x2: 1e-5 * 2 = 2e-5 ... 7e-5 * 2 = 1.4e-4; 1e-3 * 2 = 2e-3; qd [0, 0.04 x 6] (0 * 2 = 0);
// 0.1 * 2 = 0.2; 100 * 2 = 200 us. x5: 5e-5 ... 3.5e-4; 5e-3; [0, 0.1 x 6]; 0.5; 500 us. Off: all
// 0. Every key has a non-zero element, so every level adds all five overrides.
TEST(NoiseLevel, EachLevelScalesEveryElementOfTheFiveKeys) {
  const Base base = hand_base();
  for (const NoiseLevel level : {NoiseLevel::kX2, NoiseLevel::kX5, NoiseLevel::kOff}) {
    const double k = noise_factor(level);
    StudioConfig c;
    apply_noise_level(c, base, level);
    ASSERT_EQ(c.overrides.size(), 5U) << "k " << k;
    for (const char* key : kNoiseKeys) {
      const KeyOverride* o = find(c, key);
      ASSERT_NE(o, nullptr) << key << " k " << k;
      EXPECT_TRUE(o->value == scaled(base.at(key).value, k)) << key << " k " << k;
    }
    EXPECT_EQ(std::get<List>(find(c, kQdNoise)->value)[0], 0.0);
    EXPECT_EQ(noise_level(c, base), level) << "k " << k;
  }
  StudioConfig off;
  apply_noise_level(off, base, NoiseLevel::kOff);
  EXPECT_EQ(std::get<double>(find(off, kJitter)->value), 0.0);
  EXPECT_EQ(std::get<List>(find(off, kQNoise)->value), List(7, 0.0));
}

// Unrelated overrides keep their order, before the noise keys; x5 then Profile leaves only them.
TEST(NoiseLevel, ProfileRemovesTheFiveAndKeepsTheOthersInOrder) {
  const Base base = hand_base();
  const KeyOverride omega{pipeline::kOmegaKey, 30.0};
  const KeyOverride diff{"sensor.qd_from_diff", false};
  StudioConfig c = with({omega, {kQNoise, List(7, 9e-5)}, diff});
  apply_noise_level(c, base, NoiseLevel::kX5);
  ASSERT_EQ(c.overrides.size(), 7U);
  EXPECT_EQ(c.overrides[0].key, omega.key);
  EXPECT_EQ(c.overrides[1].key, diff.key);
  EXPECT_EQ(noise_level(c, base), NoiseLevel::kX5);
  apply_noise_level(c, base, NoiseLevel::kProfile);
  ASSERT_EQ(c.overrides.size(), 2U);
  EXPECT_EQ(c.overrides[0].key, omega.key);
  EXPECT_TRUE(c.overrides[0].value == omega.value);
  EXPECT_EQ(c.overrides[1].key, diff.key);
  EXPECT_TRUE(c.overrides[1].value == diff.value);
  EXPECT_EQ(noise_level(c, base), NoiseLevel::kProfile);
}

TEST(NoiseLevel, AHandEditAfterX2IsCustom) {
  const Base base = hand_base();
  StudioConfig c;
  apply_noise_level(c, base, NoiseLevel::kX2);
  auto& offset = std::get<List>(find(c, kQOffset)->value);
  offset[3] = 1e-3 * 3.0;  // joint 4 at x3, the others at x2
  EXPECT_EQ(noise_level(c, base), NoiseLevel::kCustom);

  StudioConfig jitter;
  apply_noise_level(jitter, base, NoiseLevel::kX5);
  find(jitter, kJitter)->value = 150.0;  // 1.5 x 100
  EXPECT_EQ(noise_level(jitter, base), NoiseLevel::kCustom);
}

// torque_scale_std at 0 on every joint: 0 * k = 0 == base, so no override, and it matches x2.
TEST(NoiseLevel, AnAllZeroKeyGetsNoOverrideAndDoesNotForceCustom) {
  Base base = hand_base();
  base[kTorque].value = List(7, 0.0);
  for (const NoiseLevel level : {NoiseLevel::kX2, NoiseLevel::kOff}) {
    StudioConfig c;
    apply_noise_level(c, base, level);
    EXPECT_EQ(c.overrides.size(), 4U);
    EXPECT_EQ(find(c, kTorque), nullptr);
    EXPECT_EQ(noise_level(c, base), level);
  }
}

// Without the jitter key in the base, apply removes its override and adds none, and noise_level
// does not look at it.
TEST(NoiseLevel, AKeyMissingFromTheBaseIsIgnored) {
  Base base = hand_base();
  base.erase(kJitter);
  StudioConfig c = with({{kJitter, 123.0}});
  apply_noise_level(c, base, NoiseLevel::kX2);
  EXPECT_EQ(keys(c), (std::set<std::string>{kQNoise, kQOffset, kQdNoise, kTorque}));
  EXPECT_EQ(noise_level(c, base), NoiseLevel::kX2);
  c.overrides.push_back({kJitter, 123.0});
  EXPECT_EQ(noise_level(c, base), NoiseLevel::kX2);
}

// Every level matches an all-zero base: Profile. A non-zero value on a zero base matches none.
TEST(NoiseLevel, AllZeroBasesAreProfile) {
  for (const Base& base : {pair_base("ideal"), zero_base()}) {
    for (const char* key : kNoiseKeys) {
      ASSERT_EQ(base.count(key), 1U) << key;
      ASSERT_TRUE(base.at(key).value == scaled(base.at(key).value, 0.0)) << key;
    }
    EXPECT_EQ(noise_level(StudioConfig{}, base), NoiseLevel::kProfile);
    StudioConfig c;
    apply_noise_level(c, base, NoiseLevel::kX5);
    EXPECT_TRUE(c.overrides.empty());
    EXPECT_EQ(noise_level(c, base), NoiseLevel::kProfile);
    c.overrides.push_back({kJitter, 1.0});
    EXPECT_EQ(noise_level(c, base), NoiseLevel::kCustom);
  }
}

TEST(NoiseLevel, ApplyingCustomChangesNothing) {
  const Base base = hand_base();
  const std::vector<KeyOverride> before{
      {pipeline::kOmegaKey, 30.0}, {kQNoise, List{1, 2, 3, 4, 5, 6, 7}}, {kJitter, 7.0}};
  StudioConfig c = with(before);
  apply_noise_level(c, base, NoiseLevel::kCustom);
  ASSERT_EQ(c.overrides.size(), before.size());
  for (std::size_t i = 0; i < before.size(); ++i) {
    EXPECT_EQ(c.overrides[i].key, before[i].key);
    EXPECT_TRUE(c.overrides[i].value == before[i].value) << before[i].key;
  }
}

// budget_xarm7: q_noise 5e-5, q_offset 2e-3, qd_noise 0 (all joints), torque_scale_std 0.10,
// jitter 100 us. x2 and x5 add four overrides (qd stays at its zero base); the loaders accept them.
TEST(NoiseLevel, BudgetPairScalesFourKeysAndStaysValid) {
  const Base base = pair_base("budget_xarm7");
  for (const char* key : kNoiseKeys) {
    ASSERT_EQ(base.count(key), 1U) << key;
  }
  ASSERT_EQ(std::get<List>(base.at(kQNoise).value), List(7, 5e-5));
  ASSERT_EQ(std::get<List>(base.at(kQdNoise).value), List(7, 0.0));
  const std::vector<BasePair> pairs = discover_base_pairs(fs::path(COBOT_REPO_DIR) / "config");
  const auto pair = std::find_if(
      pairs.begin(), pairs.end(), [](const BasePair& p) { return p.name == "budget_xarm7"; });
  ASSERT_NE(pair, pairs.end());
  for (const NoiseLevel level : {NoiseLevel::kX2, NoiseLevel::kX5, NoiseLevel::kOff}) {
    StudioConfig c;
    c.base = *pair;
    apply_noise_level(c, base, level);
    EXPECT_EQ(keys(c), (std::set<std::string>{kQNoise, kQOffset, kTorque, kJitter}));
    for (const KeyOverride& o : c.overrides) {
      EXPECT_TRUE(o.value == scaled(base.at(o.key).value, noise_factor(level))) << o.key;
    }
    EXPECT_EQ(noise_level(c, base), level);
    EXPECT_TRUE(validate(c).empty()) << validate(c).front();
  }
}

}  // namespace
}  // namespace cobot::studio

#else  // before the implementation: one failing test instead of a missing-header error

#include <gtest/gtest.h>

TEST(NoiseLevel, HeaderExists) {
  FAIL() << "src/studio/core/include/cobot/studio/noise_level.hpp is missing (feature "
            "studio-noise-level)";
}

#endif
