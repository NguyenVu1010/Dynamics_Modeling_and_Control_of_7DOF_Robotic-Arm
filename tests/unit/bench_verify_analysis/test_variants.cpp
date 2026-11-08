// The profile variants of the sensitivity and of the ablation (brief verify-analysis, sections 1
// and 2), on the reference model files, no run:
//  - pipeline::profile_value: a scalar, a list, an absent key is an error naming it;
//  - variant_override: a `range` key gives exactly lo and hi; a `scale` key gives every element
//    of the profile's value times lo / hi; the profile loaded with it equals the base profile
//    except that key (every field compared);
//  - group_off_overrides: for each ablation group, the keys equal the active keys of that group
//    (pipeline::active_imperfection_keys) less the two whose off value the loader refuses
//    (sensor.qd_filter_hz, transmission.ratio), and the profile written with them has no active
//    key of that group left but transmission.ratio.
//
// budget_xarm7 values (config/imperfections/budget_xarm7.yaml) and products, by hand:
//   actuator.deadband_nm [0.435 x2, 0.26 x2, 0.05 x3], scale [0.2, 3.0]:
//     x 0.2 = [0.087, 0.087, 0.052, 0.052, 0.01, 0.01, 0.01]
//     x 3.0 = [1.305, 1.305, 0.78, 0.78, 0.15, 0.15, 0.15]
//   friction.coulomb_nm [3.5 x2, 2.1 x2, 0.4 x3], scale [0.5, 1.5]:
//     x 0.5 = [1.75, 1.75, 1.05, 1.05, 0.2, 0.2, 0.2]
//     x 1.5 = [5.25, 5.25, 3.15, 3.15, 0.6, 0.6, 0.6]
//   actuator.bus.r_source_ohm 0.25, range [0.05, 0.50]; sensor.sensor_delay_cycles 1, range [0, 3].
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iterator>
#include <string>
#include <variant>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/hardware_model.hpp"
#include "cobot/pipeline/overrides.hpp"

#include "analysis.hpp"
#include "analysis_test_util.hpp"

namespace cobot::bench {
namespace {

using namespace analysis_test;  // NOLINT(google-build-using-namespace)
using pipeline::KeyOverride;
using pipeline::ProvenanceEntry;

constexpr double kRel = 1e-12;

const std::array<const char*, 3> kModels = {"premium_fr3", "mid_kr1205", "budget_xarm7"};

pipeline::HardwareModel model(const std::string& name) {
  return verify_test::load_model(name);
}

Profile load_profile(const fs::path& yaml, const std::vector<KeyOverride>& overrides = {}) {
  const auto p = pipeline::load_imperfections_with<model::kArmDof>(yaml, overrides);
  EXPECT_TRUE(p.is_ok()) << p.reason();
  return p ? p.value() : Profile{};
}

ProvenanceEntry entry_of(const pipeline::HardwareModel& m, const std::string& key) {
  for (const ProvenanceEntry& e : m.provenance) {
    if (e.key == key) {
      return e;
    }
  }
  ADD_FAILURE() << m.name << " has no provenance entry " << key;
  return ProvenanceEntry{};
}

void expect_list_near(const std::vector<double>& got, const std::vector<double>& want) {
  ASSERT_EQ(got.size(), want.size());
  for (std::size_t i = 0; i < want.size(); ++i) {
    EXPECT_NEAR(got[i], want[i], kRel * std::abs(want[i])) << "element " << i;
  }
}

// ---- profile_value ---------------------------------------------------------------------------

TEST(ProfileValue, AScalarKeyIsADouble) {
  const auto v = pipeline::profile_value(profile_path("budget_xarm7"), "actuator.bus.r_source_ohm");
  ASSERT_TRUE(v.is_ok()) << v.reason();
  ASSERT_TRUE(std::holds_alternative<double>(v.value()));
  EXPECT_EQ(std::get<double>(v.value()), 0.25);

  const auto bits = pipeline::profile_value(profile_path("budget_xarm7"), "sensor.encoder_bits");
  ASSERT_TRUE(bits.is_ok()) << bits.reason();
  ASSERT_TRUE(std::holds_alternative<double>(bits.value()));
  EXPECT_EQ(std::get<double>(bits.value()), 17.0);
}

TEST(ProfileValue, AListKeyIsAList) {
  const auto v = pipeline::profile_value(profile_path("budget_xarm7"), "friction.coulomb_nm");
  ASSERT_TRUE(v.is_ok()) << v.reason();
  ASSERT_TRUE(std::holds_alternative<std::vector<double>>(v.value()));
  EXPECT_EQ(std::get<std::vector<double>>(v.value()),
            (std::vector<double>{3.5, 3.5, 2.1, 2.1, 0.4, 0.4, 0.4}));
}

TEST(ProfileValue, AnAbsentKeyIsAnErrorNamingIt) {
  for (const char* key : {"comm.no_such_key", "no_such_group.key", "actuator.bus.no_such_key"}) {
    const auto v = pipeline::profile_value(profile_path("budget_xarm7"), key);
    ASSERT_FALSE(v.is_ok()) << key;
    EXPECT_TRUE(verify_test::contains(v.reason(), key)) << v.reason();
  }
}

// ---- variant_override --------------------------------------------------------------------------

TEST(Variants, ARangeKeyGivesExactlyItsBounds) {
  const pipeline::HardwareModel m = model("budget_xarm7");
  const struct {
    const char* key;
    double lo;
    double hi;
  } cases[] = {
      {"actuator.bus.r_source_ohm", 0.05, 0.50},
      {"sensor.sensor_delay_cycles", 0.0, 3.0},
      {"sensor.q_offset_std_rad", 5.0e-4, 5.0e-3},
  };
  for (const auto& c : cases) {
    const ProvenanceEntry e = entry_of(m, c.key);
    for (const End end : {End::kLow, End::kHigh}) {
      const auto o = variant_override(m.imperfections, e, end);
      ASSERT_TRUE(o.is_ok()) << c.key << ": " << o.reason();
      EXPECT_EQ(o.value().key, c.key);
      ASSERT_TRUE(std::holds_alternative<double>(o.value().value)) << c.key;
      EXPECT_EQ(std::get<double>(o.value().value), end == End::kLow ? c.lo : c.hi) << c.key;
    }
  }
}

TEST(Variants, AScaleKeyGivesEveryElementTimesTheFactor) {
  const pipeline::HardwareModel m = model("budget_xarm7");
  const struct {
    const char* key;
    std::vector<double> low;
    std::vector<double> high;
  } cases[] = {
      {"actuator.deadband_nm",
       {0.087, 0.087, 0.052, 0.052, 0.01, 0.01, 0.01},
       {1.305, 1.305, 0.78, 0.78, 0.15, 0.15, 0.15}},
      {"friction.coulomb_nm",
       {1.75, 1.75, 1.05, 1.05, 0.2, 0.2, 0.2},
       {5.25, 5.25, 3.15, 3.15, 0.6, 0.6, 0.6}},
  };
  for (const auto& c : cases) {
    SCOPED_TRACE(c.key);
    const ProvenanceEntry e = entry_of(m, c.key);
    for (const End end : {End::kLow, End::kHigh}) {
      const auto o = variant_override(m.imperfections, e, end);
      ASSERT_TRUE(o.is_ok()) << o.reason();
      EXPECT_EQ(o.value().key, c.key);
      ASSERT_TRUE(std::holds_alternative<std::vector<double>>(o.value().value));
      expect_list_near(std::get<std::vector<double>>(o.value().value),
                       end == End::kLow ? c.low : c.high);
    }
  }
}

TEST(Variants, EveryRangeAndScaleKeyOfEveryModelChangesOnlyThatKey) {
  for (const char* name : kModels) {
    const pipeline::HardwareModel m = model(name);
    const Profile base = load_profile(m.imperfections);
    for (const ProvenanceEntry& e : m.provenance) {
      if (!e.range && !e.scale) {
        continue;
      }
      SCOPED_TRACE(std::string(name) + " " + e.key);
      const auto value = pipeline::profile_value(m.imperfections, e.key);
      ASSERT_TRUE(value.is_ok()) << value.reason();
      const std::array<double, 2> bounds = e.range ? *e.range : *e.scale;
      for (const End end : {End::kLow, End::kHigh}) {
        const double bound = bounds[end == End::kLow ? 0 : 1];
        const auto o = variant_override(m.imperfections, e, end);
        ASSERT_TRUE(o.is_ok()) << o.reason();
        ASSERT_EQ(o.value().key, e.key);
        if (e.range) {
          ASSERT_TRUE(std::holds_alternative<double>(o.value().value));
          EXPECT_EQ(std::get<double>(o.value().value), bound);
        } else if (std::holds_alternative<double>(value.value())) {
          ASSERT_TRUE(std::holds_alternative<double>(o.value().value));
          const double want = std::get<double>(value.value()) * bound;
          EXPECT_NEAR(std::get<double>(o.value().value), want, kRel * std::abs(want));
        } else {
          ASSERT_TRUE(std::holds_alternative<std::vector<double>>(o.value().value));
          std::vector<double> want = std::get<std::vector<double>>(value.value());
          for (double& w : want) {
            w *= bound;
          }
          expect_list_near(std::get<std::vector<double>>(o.value().value), want);
        }
        // The loaded profile equals the base except that key. A variant equal to the file's
        // value (a scale of exactly 1, a bound at the value) changes nothing at all.
        const Profile varied = load_profile(m.imperfections, {o.value()});
        const std::vector<std::string> changed = differing_keys(base, varied);
        EXPECT_LE(changed.size(), 1U);
        for (const std::string& k : changed) {
          EXPECT_EQ(k, e.key);
        }
      }
    }
  }
}

TEST(Variants, AScaleOfExactlyOneStillGivesThatEnd) {
  // premium_fr3 has no scale with a bound at 1; a hand-made entry on the budget profile: the low
  // end of scale [1, 2] is the file's value itself, still a variant.
  ProvenanceEntry e;
  e.key = "friction.viscous_nm_s_rad";
  e.kind = pipeline::ProvenanceKind::kAssumed;
  e.scale = std::array<double, 2>{1.0, 2.0};
  const auto o = variant_override(profile_path("budget_xarm7"), e, End::kLow);
  ASSERT_TRUE(o.is_ok()) << o.reason();
  ASSERT_TRUE(std::holds_alternative<std::vector<double>>(o.value().value));
  expect_list_near(std::get<std::vector<double>>(o.value().value),
                   {1.0, 1.0, 1.0, 1.0, 0.2, 0.2, 0.2});
}

TEST(Variants, AnEntryWithoutRangeOrScaleIsAnError) {
  const pipeline::HardwareModel m = model("budget_xarm7");
  for (const char* key : {"sensor.qd_from_diff", "sensor.position_source"}) {
    const auto o = variant_override(m.imperfections, entry_of(m, key), End::kLow);
    ASSERT_FALSE(o.is_ok()) << key;
    EXPECT_TRUE(verify_test::contains(o.reason(), key)) << o.reason();
  }
}

// ---- group_off_overrides -----------------------------------------------------------------------

std::vector<std::string> active_keys(const fs::path& profile) {
  const auto keys = pipeline::active_imperfection_keys(profile);
  EXPECT_TRUE(keys.is_ok()) << keys.reason();
  return keys ? keys.value() : std::vector<std::string>{};
}

bool loader_refuses_off(const std::string& key) {
  return key == "sensor.qd_filter_hz" || key == "transmission.ratio";
}

TEST(GroupOff, TheGroupsAreTheBriefs) {
  const std::vector<std::string> want = {
      "sensor", "actuator", "friction", "model", "timing", "comm", "transmission"};
  ASSERT_EQ(kAblationGroups.size(), want.size());
  for (std::size_t i = 0; i < want.size(); ++i) {
    EXPECT_EQ(kAblationGroups[i], want[i]);
  }
}

TEST(GroupOff, BudgetEveryGroupListsItsActiveKeysAndSwitchesThemOff) {
  const pipeline::HardwareModel m = model("budget_xarm7");
  const std::vector<std::string> active = active_keys(m.imperfections);
  const fs::path dir = test_dir();
  for (const char* group : kAblationGroups) {
    SCOPED_TRACE(group);
    std::vector<std::string> want;
    for (const std::string& k : active) {
      if (group_of(k) == group && !loader_refuses_off(k)) {
        want.push_back(k);
      }
    }
    // Every group of the budget profile is active (it has a key that needs provenance).
    EXPECT_FALSE(want.empty());
    const auto off = group_off_overrides(m.imperfections, group);
    ASSERT_TRUE(off.is_ok()) << off.reason();
    std::vector<std::string> keys;
    for (const KeyOverride& o : off.value()) {
      keys.push_back(o.key);
    }
    EXPECT_EQ(keys, want);

    // Off values: 0 (every element of a list), false, link.
    for (const KeyOverride& o : off.value()) {
      if (std::holds_alternative<double>(o.value)) {
        EXPECT_EQ(std::get<double>(o.value), 0.0) << o.key;
      } else if (std::holds_alternative<std::vector<double>>(o.value)) {
        for (const double v : std::get<std::vector<double>>(o.value)) {
          EXPECT_EQ(v, 0.0) << o.key;
        }
      } else if (std::holds_alternative<bool>(o.value)) {
        EXPECT_FALSE(std::get<bool>(o.value)) << o.key;
      } else {
        EXPECT_EQ(o.key, "sensor.position_source");
        EXPECT_EQ(std::get<std::string>(o.value), "link") << o.key;
      }
    }

    // The profile with them applied loads and has no active key of that group left (but the
    // gear ratio), and every other group is as it was.
    const fs::path written = dir / (std::string(group) + "_off.yaml");
    const common::Status w =
        pipeline::write_imperfections_with(m.imperfections, off.value(), written);
    ASSERT_TRUE(w.is_ok()) << w.reason();
    const std::vector<std::string> after = active_keys(written);
    for (const std::string& k : after) {
      if (group_of(k) == group) {
        EXPECT_EQ(k, "transmission.ratio");
      }
    }
    std::vector<std::string> others_before;
    std::vector<std::string> others_after;
    std::copy_if(active.begin(),
                 active.end(),
                 std::back_inserter(others_before),
                 [group](const std::string& k) { return group_of(k) != group; });
    std::copy_if(after.begin(),
                 after.end(),
                 std::back_inserter(others_after),
                 [group](const std::string& k) { return group_of(k) != group; });
    EXPECT_EQ(others_after, others_before);
    const Profile base = load_profile(m.imperfections);
    const Profile ablated = load_profile(m.imperfections, off.value());
    for (const std::string& k : differing_keys(base, ablated)) {
      EXPECT_EQ(group_of(k), group) << k;
    }
  }
}

TEST(GroupOff, EveryReferenceProfileAppliesCleanly) {
  for (const char* name : kModels) {
    const pipeline::HardwareModel m = model(name);
    const std::vector<std::string> active = active_keys(m.imperfections);
    for (const char* group : kAblationGroups) {
      SCOPED_TRACE(std::string(name) + " " + group);
      const auto off = group_off_overrides(m.imperfections, group);
      ASSERT_TRUE(off.is_ok()) << off.reason();
      std::vector<std::string> want;
      for (const std::string& k : active) {
        if (group_of(k) == group && !loader_refuses_off(k)) {
          want.push_back(k);
        }
      }
      std::vector<std::string> keys;
      for (const KeyOverride& o : off.value()) {
        keys.push_back(o.key);
      }
      EXPECT_EQ(keys, want);
      const auto loaded =
          pipeline::load_imperfections_with<model::kArmDof>(m.imperfections, off.value());
      EXPECT_TRUE(loaded.is_ok()) << loaded.reason();
    }
  }
}

TEST(GroupOff, TheModelGroupLeavesThePayloadToTheTask) {
  const auto off = group_off_overrides(profile_path("budget_xarm7"), "model");
  ASSERT_TRUE(off.is_ok()) << off.reason();
  for (const KeyOverride& o : off.value()) {
    EXPECT_NE(o.key.rfind("model.payload_", 0), 0U) << o.key;
  }
}

TEST(GroupOff, AnUnknownGroupIsAnErrorNamingIt) {
  const auto off = group_off_overrides(profile_path("budget_xarm7"), "no_such_group");
  ASSERT_FALSE(off.is_ok());
  EXPECT_TRUE(verify_test::contains(off.reason(), "no_such_group")) << off.reason();
}

}  // namespace
}  // namespace cobot::bench
