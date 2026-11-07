// active_imperfection_keys (brief hardware-catalog 2): dotted keys of the leaves of a profile that
// are not at their off value. Off = 0, false, or the group/key absent. Never listed: seed, source,
// model.payload_* (owned by the task). sensor.qd_filter_hz counts only with qd_from_diff true,
// actuator.ripple_periods_per_rev only with a non-zero ripple, sensor.position_source when it
// differs from the loader's default (link). The tests compare sorted: the order is not contract.
#include <algorithm>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/hardware_model.hpp"

#include "hw_test_util.hpp"

namespace cobot::pipeline {
namespace {

using namespace hw_test;
using Keys = std::vector<std::string>;

Keys sorted(Keys keys) {
  std::sort(keys.begin(), keys.end());
  return keys;
}

Keys active_of(const fs::path& profile) {
  const auto r = active_imperfection_keys(profile);
  if (!r.is_ok()) {
    ADD_FAILURE() << r.reason();
    return {"<load failed>"};
  }
  return sorted(r.value());
}

// The edited profile must be valid for the loader first: these tests are about the listing.
Keys active_of_text(const std::string& text) {
  const fs::path path = write_text(test_dir() / "profile.yaml", text);
  const auto valid = load_imperfections<kN>(path);
  EXPECT_TRUE(valid.is_ok()) << "fixture does not load: " << valid.reason();
  return active_of(path);
}

std::string ideal_text() {
  return read_text(kRepo / "config/imperfections/ideal.yaml");
}

// config/imperfections/budget_xarm7.yaml, group by group; the keys left out are at their off value
// or never listed:
//   sensor: 8 leaves without source, qd_noise_std_rad_s is 0 -> 7
//   actuator: 11 flat leaves + bus 3, all non-zero -> 14
//   friction 2; model: payload_* never, the other 2 listed; timing 3; comm 4; transmission 9
// 7 + 14 + 2 + 2 + 3 + 4 + 9 = 41.
Keys budget_keys() {
  return {"sensor.encoder_bits",
          "sensor.q_noise_std_rad",
          "sensor.q_offset_std_rad",
          "sensor.qd_from_diff",
          "sensor.qd_filter_hz",
          "sensor.sensor_delay_cycles",
          "sensor.position_source",
          "actuator.torque_scale_std",
          "actuator.torque_scale_asym_frac",
          "actuator.torque_ripple_frac",
          "actuator.ripple_periods_per_rev",
          "actuator.deadband_nm",
          "actuator.torque_cmd_resolution_nm",
          "actuator.motor_time_constant_s",
          "actuator.command_delay_cycles",
          "actuator.torque_slew_nm_s",
          "actuator.knee_speed_rad_s",
          "actuator.zero_torque_speed_rad_s",
          "actuator.bus.v_nom_v",
          "actuator.bus.r_source_ohm",
          "actuator.bus.v_min_v",
          "friction.coulomb_nm",
          "friction.viscous_nm_s_rad",
          "model.link_mass_scale_std",
          "model.com_shift_std_m",
          "timing.period_jitter_std_us",
          "timing.overrun_prob",
          "timing.overrun_cycles",
          "comm.servo_rate_hz",
          "comm.packet_loss_prob",
          "comm.latency_burst_prob",
          "comm.burst_cycles",
          "transmission.stiffness_k1_nm_rad",
          "transmission.stiffness_k2_nm_rad",
          "transmission.stiffness_k3_nm_rad",
          "transmission.torque_t1_nm",
          "transmission.torque_t2_nm",
          "transmission.damping_ratio",
          "transmission.error_amp_rad",
          "transmission.lost_motion_rad",
          "transmission.ratio"};
}

Keys without(Keys keys, const Keys& gone) {
  for (const std::string& g : gone) {
    keys.erase(std::remove(keys.begin(), keys.end(), g), keys.end());
  }
  return sorted(keys);
}

TEST(ActiveKeys, IdealIsEmpty) {
  EXPECT_EQ(active_of(kRepo / "config/imperfections/ideal.yaml"), Keys{});
}

TEST(ActiveKeys, BudgetIsThePinnedList) {
  ASSERT_EQ(budget_keys().size(), 41U);
  EXPECT_EQ(active_of(kRepo / "config/imperfections/budget_xarm7.yaml"), sorted(budget_keys()));
}

// Mid sets everything budget does but servo_rate_hz (0: every cycle). Premium also has deadband 0.
TEST(ActiveKeys, MidAndPremiumDifferFromBudgetByTheirOffValues) {
  EXPECT_EQ(active_of(kRepo / "config/imperfections/mid_kr1205.yaml"),
            without(budget_keys(), {"comm.servo_rate_hz"}));
  EXPECT_EQ(active_of(kRepo / "config/imperfections/premium_fr3.yaml"),
            without(budget_keys(), {"comm.servo_rate_hz", "actuator.deadband_nm"}));
}

TEST(ActiveKeys, FilterCountsOnlyWithDifferentiatedVelocity) {
  // ideal.yaml: qd_filter_hz 200 and qd_from_diff false -> empty (IdealIsEmpty). With it on:
  EXPECT_EQ(active_of_text(set_line(ideal_text(), "    qd_from_diff:", "    qd_from_diff: true")),
            sorted({"sensor.qd_from_diff", "sensor.qd_filter_hz"}));
}

TEST(ActiveKeys, FalseOwnerKeepsTheFilterOutWhateverItsValue) {
  EXPECT_EQ(active_of_text(set_line(ideal_text(), "    qd_filter_hz:", "    qd_filter_hz: 50.0")),
            Keys{});
}

TEST(ActiveKeys, RipplePeriodsCountOnlyWithRipple) {
  // ideal.yaml: ripple_periods_per_rev 6 and torque_ripple_frac 0 -> empty (IdealIsEmpty).
  EXPECT_EQ(active_of_text(
                set_line(ideal_text(), "    torque_ripple_frac:", "    torque_ripple_frac: 0.05")),
            sorted({"actuator.torque_ripple_frac", "actuator.ripple_periods_per_rev"}));
}

TEST(ActiveKeys, RipplePeriodsCountWithRippleOnOneJoint) {
  EXPECT_EQ(active_of_text(set_line(ideal_text(),
                                    "    torque_ripple_frac:",
                                    "    torque_ripple_frac: [0, 0, 0, 0, 0, 0, 0.05]")),
            sorted({"actuator.torque_ripple_frac", "actuator.ripple_periods_per_rev"}));
}

TEST(ActiveKeys, ZeroRippleKeepsThePeriodsOutWhateverTheirValue) {
  EXPECT_EQ(active_of_text(set_line(
                ideal_text(), "    ripple_periods_per_rev:", "    ripple_periods_per_rev: 12.0")),
            Keys{});
}

TEST(ActiveKeys, PositionSourceCountsWhenItDiffersFromTheDefault) {
  const std::string delay = "    sensor_delay_cycles:";
  EXPECT_EQ(active_of_text(set_line(
                ideal_text(), delay, "    sensor_delay_cycles: 0\n    position_source: motor")),
            Keys{"sensor.position_source"});
  EXPECT_EQ(active_of_text(set_line(
                ideal_text(), delay, "    sensor_delay_cycles: 0\n    position_source: link")),
            Keys{});
}

TEST(ActiveKeys, SeedSourceAndPayloadAreNeverListed) {
  std::string text = set_line(ideal_text(), "  seed:", "  seed: 42");
  text = set_line(
      text, "    payload_kg:", "    payload_kg: 3.0\n    payload_offset_m: [0.0, 0.0, 0.05]");
  text = set_line(text, "    payload_radius_m:", "    payload_radius_m: 0.05");
  EXPECT_EQ(active_of_text(text), Keys{});
}

// A list is off only when every element is 0.
TEST(ActiveKeys, ListWithOneNonZeroElementIsActive) {
  EXPECT_EQ(active_of_text(set_line(
                ideal_text(), "    coulomb_nm:", "    coulomb_nm: [0, 0, 0, 0, 0, 0, 0.1]")),
            Keys{"friction.coulomb_nm"});
  EXPECT_EQ(active_of_text(
                set_line(ideal_text(), "    coulomb_nm:", "    coulomb_nm: [0, 0, 0, 0, 0, 0, 0]")),
            Keys{});
}

// Bus keys are dotted three levels deep, each judged on its own; an absent group lists nothing
// (ideal.yaml has no bus and no transmission group, IdealIsEmpty).
TEST(ActiveKeys, NestedBusKeysAreDotted) {
  const std::string text = set_line(ideal_text(),
                                    "    zero_torque_speed_rad_s:",
                                    "    zero_torque_speed_rad_s: 0.0\n    bus: {v_nom_v: 48.0, "
                                    "r_source_ohm: 0.0, v_min_v: 40.0}");
  EXPECT_EQ(active_of_text(text), sorted({"actuator.bus.v_nom_v", "actuator.bus.v_min_v"}));
}

TEST(ActiveKeys, ErrorsNameTheFile) {
  const auto missing = active_imperfection_keys(test_dir() / "no_such_profile.yaml");
  ASSERT_FALSE(missing.is_ok());
  EXPECT_TRUE(contains(missing.reason(), "no_such_profile.yaml")) << missing.reason();

  // Strict like load_imperfections: a bad key is its usual error.
  const fs::path bad =
      write_text(test_dir() / "bad_profile.yaml",
                 set_line(ideal_text(), "    coulomb_nm:", "    coulomb_nmm: 0.0"));
  const auto r = active_imperfection_keys(bad);
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(r.reason(), "bad_profile.yaml")) << r.reason();
  EXPECT_TRUE(contains(r.reason(), "imperfections.friction.coulomb_nmm")) << r.reason();
}

}  // namespace
}  // namespace cobot::pipeline
