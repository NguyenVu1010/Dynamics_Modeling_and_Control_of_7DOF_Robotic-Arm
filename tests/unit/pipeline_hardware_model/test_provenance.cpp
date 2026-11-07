// Provenance of a hardware model (brief hardware-catalog 2, design G10, owner question C8: a model
// with an unsourced key does not load). Every active imperfection key needs one entry; an entry for
// a key the profile does not set, a duplicate and a kind outside published | derived | assumed are
// errors. derived and assumed need exactly one of range (absolute, scalar keys, lo <= the file's
// value <= hi) or scale (multiplier of every element, list keys, lo <= 1 <= hi, lo >= 0); published
// may omit both; bool and choice keys take neither, whatever the kind.
#include <map>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/pipeline/hardware_model.hpp"

#include "hardware_fixture.hpp"
#include "hw_test_util.hpp"

namespace cobot::pipeline {
namespace {

using namespace hw_test;

// The fixture text with one provenance line replaced.
std::string with_entry(const char* entry, const std::string& line) {
  return set_line(hardware_fixture_text(), entry, line);
}

// One provenance line; `bound` is empty or "range: [lo, hi]" / "scale: [lo, hi]".
std::string entry_line(const std::string& key, const std::string& kind, const std::string& bound) {
  std::string line = "    - {key: " + key + ", kind: " + kind;
  if (!bound.empty()) {
    line += ", ";
    line += bound;
  }
  line += "}";
  return line;
}

TEST(Provenance, EntriesAreParsed) {
  const auto r = load_hardware_model(write_fixture(hardware_fixture_text()));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const std::vector<ProvenanceEntry>& p = r.value().provenance;
  ASSERT_EQ(p.size(), 6U);

  EXPECT_EQ(p[0].key, "sensor.encoder_bits");
  EXPECT_TRUE(p[0].kind == ProvenanceKind::kPublished);
  EXPECT_FALSE(p[0].range.has_value());
  EXPECT_FALSE(p[0].scale.has_value());

  EXPECT_EQ(p[1].key, "sensor.q_offset_std_rad");
  EXPECT_TRUE(p[1].kind == ProvenanceKind::kAssumed);
  ASSERT_TRUE(p[1].range.has_value());
  EXPECT_EQ((*p[1].range)[0], 5.0e-4);
  EXPECT_EQ((*p[1].range)[1], 5.0e-3);
  EXPECT_FALSE(p[1].scale.has_value());

  EXPECT_EQ(p[5].key, "friction.coulomb_nm");
  EXPECT_TRUE(p[5].kind == ProvenanceKind::kDerived);
  EXPECT_FALSE(p[5].range.has_value());
  ASSERT_TRUE(p[5].scale.has_value());
  EXPECT_EQ((*p[5].scale)[0], 0.5);
  EXPECT_EQ((*p[5].scale)[1], 1.5);
}

TEST(Provenance, ActiveKeyWithoutEntryIsNamed) {
  struct Case {
    const char* entry;
    const char* key;
  };
  const std::vector<Case> cases{
      {kEntryBits, "sensor.encoder_bits"},
      {kEntryFromDiff, "sensor.qd_from_diff"},
      {kEntryFilter, "sensor.qd_filter_hz"},
      {kEntrySource, "sensor.position_source"},
      {kEntryCoulomb, "friction.coulomb_nm"},
  };
  int n = 0;
  for (const Case& c : cases) {
    SCOPED_TRACE(c.key);
    expect_load_error(
        write_fixture(drop_block(hardware_fixture_text(), c.entry), "case" + std::to_string(n++)),
        {c.key});
  }
}

TEST(Provenance, EntryForAKeyTheProfileDoesNotSetIsNamed) {
  // At the off value, in an absent group, misspelt, and never listed by the rule of
  // active_imperfection_keys.
  const std::vector<std::string> keys{"sensor.q_noise_std_rad",
                                      "friction.viscous_nm_s_rad",
                                      "transmission.ratio",
                                      "actuator.bus.v_nom_v",
                                      "sensor.no_such_key",
                                      "seed",
                                      "sensor.source",
                                      "model.payload_kg",
                                      "model.payload_offset_m",
                                      "model.payload_radius_m"};
  int n = 0;
  for (const std::string& key : keys) {
    SCOPED_TRACE(key);
    expect_load_error(write_fixture(insert_after(hardware_fixture_text(),
                                                 kEntryCoulomb,
                                                 "    - {key: " + key + ", kind: published}"),
                                    "case" + std::to_string(n++)),
                      {key});
  }
}

// qd_filter_hz is inactive when qd_from_diff is false, so its entry is one too many; same for
// ripple_periods_per_rev with no ripple (ideal.yaml sets 6.0 with ripple 0).
TEST(Provenance, EntryForAnInactiveDependentKeyIsNamed) {
  const std::string profile =
      set_line(profile_fixture_text(), "    qd_from_diff:", "    qd_from_diff: false");
  expect_load_error(
      write_fixture(drop_block(hardware_fixture_text(), kEntryFromDiff), "filter", profile),
      {"sensor.qd_filter_hz"});
  expect_load_error(
      write_fixture(insert_after(hardware_fixture_text(),
                                 kEntryCoulomb,
                                 "    - {key: actuator.ripple_periods_per_rev, kind: published}"),
                    "ripple"),
      {"actuator.ripple_periods_per_rev"});
}

TEST(Provenance, DuplicateKeyIsNamed) {
  expect_load_error(write_fixture(insert_after(hardware_fixture_text(),
                                               kEntryBits,
                                               "    - {key: sensor.encoder_bits, kind: assumed, "
                                               "range: [10, 20]}")),
                    {"sensor.encoder_bits"});
}

TEST(Provenance, BadKindIsNamedWithItsIndex) {
  expect_load_error(
      write_fixture(with_entry(
          kEntryFilter, "    - {key: sensor.qd_filter_hz, kind: guessed, range: [25, 100]}")),
      {"hardware.provenance[3].kind", "guessed"});
}

TEST(Provenance, DerivedAndAssumedNeedRangeOrScale) {
  for (const std::string kind : {"derived", "assumed"}) {
    SCOPED_TRACE(kind);
    expect_load_error(
        write_fixture(
            with_entry(kEntryOffset, "    - {key: sensor.q_offset_std_rad, kind: " + kind + "}"),
            kind + "_scalar"),
        {"hardware.provenance[1]"});
    expect_load_error(
        write_fixture(
            with_entry(kEntryCoulomb, "    - {key: friction.coulomb_nm, kind: " + kind + "}"),
            kind + "_list"),
        {"hardware.provenance[5]"});
  }
}

TEST(Provenance, RangeAndScaleTogetherAreAnError) {
  expect_load_error(write_fixture(with_entry(kEntryOffset,
                                             "    - {key: sensor.q_offset_std_rad, kind: assumed, "
                                             "range: [1.0e-3, 3.0e-3], scale: [0.5, 1.5]}")),
                    {"hardware.provenance[1]"});
}

TEST(Provenance, ValueOutsideRangeIsAnError) {
  // The file's value is 2.0e-3: above, below, and an inverted range.
  int n = 0;
  for (const std::string range : {"[3.0e-3, 5.0e-3]", "[5.0e-4, 1.0e-3]", "[5.0e-3, 5.0e-4]"}) {
    SCOPED_TRACE(range);
    expect_load_error(
        write_fixture(
            with_entry(kEntryOffset,
                       "    - {key: sensor.q_offset_std_rad, kind: assumed, range: " + range + "}"),
            "case" + std::to_string(n++)),
        {"hardware.provenance[1].range"});
  }
}

TEST(Provenance, RangeBoundsIncludeTheValue) {
  const auto r = load_hardware_model(write_fixture(
      with_entry(kEntryOffset,
                 "    - {key: sensor.q_offset_std_rad, kind: assumed, range: [2.0e-3, 2.0e-3]}")));
  EXPECT_TRUE(r.is_ok()) << r.reason();
}

TEST(Provenance, ScaleMustContainOneWithANonNegativeLowerBound) {
  int n = 0;
  for (const std::string scale : {"[1.1, 2.0]", "[0.5, 0.9]", "[-0.1, 1.5]", "[1.5, 0.5]"}) {
    SCOPED_TRACE(scale);
    expect_load_error(
        write_fixture(
            with_entry(kEntryCoulomb,
                       "    - {key: friction.coulomb_nm, kind: derived, scale: " + scale + "}"),
            "case" + std::to_string(n++)),
        {"hardware.provenance[5].scale"});
  }
}

TEST(Provenance, ScaleBoundsMayBeOneAndZero) {
  int n = 0;
  for (const std::string scale : {"[1.0, 1.0]", "[0.0, 1.0]", "[1.0, 3.0]"}) {
    SCOPED_TRACE(scale);
    const auto r = load_hardware_model(write_fixture(
        with_entry(kEntryCoulomb,
                   "    - {key: friction.coulomb_nm, kind: derived, scale: " + scale + "}"),
        "case" + std::to_string(n++)));
    EXPECT_TRUE(r.is_ok()) << r.reason();
  }
}

TEST(Provenance, RangeIsForScalarKeysAndScaleForListKeys) {
  expect_load_error(
      write_fixture(
          with_entry(kEntryCoulomb,
                     "    - {key: friction.coulomb_nm, kind: derived, range: [0.1, 5.0]}"),
          "range_on_list"),
      {"hardware.provenance[5]"});
  expect_load_error(
      write_fixture(
          with_entry(kEntryOffset,
                     "    - {key: sensor.q_offset_std_rad, kind: assumed, scale: [0.5, 1.5]}"),
          "scale_on_scalar"),
      {"hardware.provenance[1]"});
}

TEST(Provenance, BoolAndChoiceKeysTakeNeitherWhateverTheKind) {
  int n = 0;
  for (const std::string kind : {"published", "derived", "assumed"}) {
    SCOPED_TRACE(kind);
    for (const std::string bound : {"range: [0, 1]", "scale: [0.5, 1.5]"}) {
      SCOPED_TRACE(bound);
      expect_load_error(
          write_fixture(with_entry(kEntryFromDiff, entry_line("sensor.qd_from_diff", kind, bound)),
                        "bool" + std::to_string(n)),
          {"hardware.provenance[2]"});
      expect_load_error(
          write_fixture(with_entry(kEntrySource, entry_line("sensor.position_source", kind, bound)),
                        "choice" + std::to_string(n)),
          {"hardware.provenance[4]"});
      ++n;
    }
  }
}

TEST(Provenance, BoolAndChoiceKeysLoadWithAnyKindAndNoBounds) {
  int n = 0;
  for (const std::string kind : {"published", "derived", "assumed"}) {
    SCOPED_TRACE(kind);
    std::string text = with_entry(kEntryFromDiff, entry_line("sensor.qd_from_diff", kind, ""));
    text = set_line(text, kEntrySource, entry_line("sensor.position_source", kind, ""));
    const auto r = load_hardware_model(write_fixture(text, "case" + std::to_string(n++)));
    EXPECT_TRUE(r.is_ok()) << r.reason();
  }
}

TEST(Provenance, PublishedWithoutBoundsLoads) {
  std::string text =
      with_entry(kEntryOffset, "    - {key: sensor.q_offset_std_rad, kind: published}");
  text = set_line(text, kEntryCoulomb, "    - {key: friction.coulomb_nm, kind: published}");
  const auto r = load_hardware_model(write_fixture(text));
  EXPECT_TRUE(r.is_ok()) << r.reason();
}

TEST(Provenance, MalformedBoundsAreNamed) {
  expect_load_error(
      write_fixture(
          with_entry(kEntryOffset,
                     "    - {key: sensor.q_offset_std_rad, kind: assumed, range: [1.0e-3]}"),
          "short"),
      {"hardware.provenance[1].range"});
  expect_load_error(
      write_fixture(
          with_entry(kEntryOffset,
                     "    - {key: sensor.q_offset_std_rad, kind: assumed, range: [a, b]}"),
          "text"),
      {"hardware.provenance[1].range"});
  expect_load_error(
      write_fixture(
          with_entry(kEntryCoulomb, "    - {key: friction.coulomb_nm, kind: derived, scale: 1.0}"),
          "scalar"),
      {"hardware.provenance[5].scale"});
}

// A profile with no active key needs no entry.
TEST(Provenance, IdealProfileLoadsWithAnEmptyList) {
  const std::string text = set_block(hardware_fixture_text(), "  provenance:", "  provenance: []");
  const auto r = load_hardware_model(
      write_fixture(text, "ideal", read_text(kRepo / "config/imperfections/ideal.yaml")));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_TRUE(r.value().provenance.empty());
}

// --- the shipped models ---

std::map<std::string, ProvenanceEntry> entries_of(const HardwareModel& m) {
  std::map<std::string, ProvenanceEntry> out;
  for (const ProvenanceEntry& e : m.provenance) {
    out.emplace(e.key, e);
  }
  return out;
}

// One entry per active key, nothing else: the load rule, seen from the outside.
TEST(ProvenanceFiles, EntriesAreExactlyTheActiveKeys) {
  for (const char* name : {"premium_fr3", "mid_kr1205", "budget_xarm7"}) {
    SCOPED_TRACE(name);
    const auto m = load_hardware_model(kRepo / "config/hardware" / (std::string(name) + ".yaml"));
    ASSERT_TRUE(m.is_ok()) << m.reason();
    const auto active = active_imperfection_keys(m.value().imperfections);
    ASSERT_TRUE(active.is_ok()) << active.reason();
    std::set<std::string> listed;
    for (const ProvenanceEntry& e : m.value().provenance) {
      EXPECT_TRUE(listed.insert(e.key).second) << e.key;
    }
    EXPECT_EQ(listed, std::set<std::string>(active.value().begin(), active.value().end()));
  }
}

struct KindPin {
  const char* key;
  ProvenanceKind kind;
};

void expect_kinds(const std::string& model, const std::vector<KindPin>& pins) {
  SCOPED_TRACE(model);
  const auto m = load_hardware_model(kRepo / "config/hardware" / (model + ".yaml"));
  ASSERT_TRUE(m.is_ok()) << m.reason();
  const std::map<std::string, ProvenanceEntry> entries = entries_of(m.value());
  for (const KindPin& pin : pins) {
    const auto it = entries.find(pin.key);
    ASSERT_NE(it, entries.end()) << pin.key;
    EXPECT_TRUE(it->second.kind == pin.kind) << pin.key;
  }
}

// The kinds the profile `source` texts state word for word (config/imperfections/<model>.yaml).
constexpr ProvenanceKind kP = ProvenanceKind::kPublished;
constexpr ProvenanceKind kD = ProvenanceKind::kDerived;
constexpr ProvenanceKind kA = ProvenanceKind::kAssumed;

TEST(ProvenanceFiles, BudgetKindsFollowTheSourceTexts) {
  expect_kinds("budget_xarm7",
               {{"sensor.encoder_bits", kP},
                {"sensor.q_noise_std_rad", kD},
                {"sensor.q_offset_std_rad", kA},
                {"sensor.qd_filter_hz", kA},
                {"sensor.sensor_delay_cycles", kA},
                {"actuator.torque_scale_std", kA},
                {"actuator.torque_cmd_resolution_nm", kA},
                {"actuator.knee_speed_rad_s", kD},
                {"actuator.zero_torque_speed_rad_s", kD},
                {"actuator.bus.v_nom_v", kP},
                {"actuator.bus.r_source_ohm", kA},
                {"friction.coulomb_nm", kD},
                {"friction.viscous_nm_s_rad", kA},
                {"model.link_mass_scale_std", kA},
                {"model.com_shift_std_m", kA},
                {"timing.period_jitter_std_us", kP},
                {"timing.overrun_prob", kA},
                {"comm.servo_rate_hz", kP},
                {"comm.packet_loss_prob", kA},
                {"transmission.stiffness_k1_nm_rad", kP},
                {"transmission.torque_t2_nm", kP},
                {"transmission.ratio", kP},
                {"transmission.damping_ratio", kA},
                {"transmission.error_amp_rad", kA},
                {"transmission.lost_motion_rad", kA}});
}

TEST(ProvenanceFiles, MidKindsFollowTheSourceTexts) {
  expect_kinds("mid_kr1205",
               {{"sensor.encoder_bits", kP},
                {"sensor.q_noise_std_rad", kD},
                {"actuator.torque_scale_std", kA},
                {"actuator.torque_cmd_resolution_nm", kP},
                {"friction.coulomb_nm", kD},
                {"timing.period_jitter_std_us", kA},
                {"comm.packet_loss_prob", kA}});
}

TEST(ProvenanceFiles, PremiumKindsFollowTheSourceTexts) {
  expect_kinds("premium_fr3",
               {{"sensor.encoder_bits", kP},
                {"actuator.torque_scale_std", kP},
                {"actuator.torque_cmd_resolution_nm", kP},
                {"actuator.motor_time_constant_s", kA},
                {"friction.coulomb_nm", kP},
                {"friction.viscous_nm_s_rad", kA},
                {"timing.period_jitter_std_us", kP},
                {"comm.packet_loss_prob", kA},
                {"transmission.error_amp_rad", kP},
                {"transmission.lost_motion_rad", kP},
                {"transmission.damping_ratio", kA}});
}

}  // namespace
}  // namespace cobot::pipeline
