#include <array>
#include <filesystem>
#include <fstream>
#include <ostream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "cobot/common/types.hpp"
#include "cobot/imperfect/config.hpp"
#include "cobot/pipeline/config.hpp"

// Feature profiles (brief "profiles", realism-2 design G3 and section 7): the three reference
// profiles load, and one value per group per profile equals the design table, with the per-joint
// expansions of the brief (R = [87, 87, 52, 52, 10, 10, 10] rated, P = [204, 204, 107, 107, 36,
// 36, 36] peak torque, N m):
//   deadband        0 | 0.001 R | 0.005 R    0.001 * 87 = 0.087, 0.001 * 52 = 0.052,
//                                            0.001 * 10 = 0.01; 0.005 * 87 = 0.435,
//                                            0.005 * 52 = 0.26, 0.005 * 10 = 0.05
//   resolution      R/1000 | R/1000 | R/250  87 / 250 = 0.348, 52 / 250 = 0.208, 10 / 250 = 0.04
//   slew            P/1 ms | P/2 ms | P/5 ms 204 / 0.001 = 204000, 107 / 0.001 = 107000,
//                                            36 / 0.001 = 36000; / 0.002: 102000, 53500, 18000;
//                                            / 0.005: 40800, 21400, 7200 (N m/s)
// Transmission stiffness and breakpoints are the catalog values merged with feature transmission
// (SHG-25 joints 1-2, SHG-20 joints 3-4, SHG-14 joints 5-7), equal to cobot.deflection_comp of
// config/default.yaml, the same for all three profiles. Every profile reads the motor encoder
// (design 4.7, update 2026-11-03).
namespace cobot::pipeline {
namespace {

namespace fs = std::filesystem;
constexpr int kN = 7;
using Vec = common::VecN<kN>;
using Profile = imperfect::ImperfectionConfig<kN>;

Vec vec(const std::array<double, kN>& a) {
  Vec v;
  for (int j = 0; j < kN; ++j) {
    v(j) = a[static_cast<std::size_t>(j)];
  }
  return v;
}

// Per-joint value of three joint groups: joints 1-2, 3-4, 5-7.
Vec groups(double j12, double j34, double j57) {
  return vec({j12, j12, j34, j34, j57, j57, j57});
}

fs::path profile_path(const std::string& name) {
  return fs::path(COBOT_REPO_DIR) / "config/imperfections" / (name + ".yaml");
}

void expect_vec(const Vec& got, const Vec& want, const std::string& key) {
  for (int j = 0; j < kN; ++j) {
    EXPECT_DOUBLE_EQ(got(j), want(j)) << key << "[" << j << "]";
  }
}

struct Expected {
  std::string name;
  // sensor
  double encoder_bits;
  double q_noise_std_rad;
  double q_offset_std_rad;
  double qd_filter_hz;
  // actuator
  double torque_scale_std;
  double torque_scale_asym_frac;
  double torque_ripple_frac;
  Vec deadband_nm;
  Vec torque_cmd_resolution_nm;
  double motor_time_constant_s;
  Vec torque_slew_nm_s;
  double bus_r_source_ohm;
  // friction
  Vec coulomb_nm;
  Vec viscous_nm_s_rad;
  // model
  double link_mass_scale_std;
  double com_shift_std_m;
  // timing
  double period_jitter_std_us;
  double overrun_prob;
  int overrun_cycles;
  // comm
  double servo_rate_hz;
  double packet_loss_prob;
  double latency_burst_prob;
  int burst_cycles;
  // transmission
  double error_amp_rad;
  double lost_motion_rad;
};

// gtest looks this name up for parameter printing.
void PrintTo(const Expected& e, std::ostream* os) {  // NOLINT(readability-identifier-naming)
  *os << e.name;
}

const Vec kResolutionFine = groups(0.087, 0.052, 0.01);  // R / 1000

std::vector<Expected> expected_profiles() {
  return {
      {"premium_fr3",
       20,
       5e-6,
       2e-4,
       200,
       0.02,
       0.005,
       0.01,
       Vec::Zero(),
       kResolutionFine,
       0.0005,
       groups(204000, 107000, 36000),
       0.05,
       groups(1.3, 0.8, 0.3),
       groups(0.3, 0.3, 0.05),
       0.02,
       0.002,
       5,
       1e-5,
       1,
       0,
       1e-5,
       1e-5,
       1,
       2.9e-4,
       2.9e-4},
      {"mid_kr1205",
       19,
       1e-5,
       1e-3,
       100,
       0.05,
       0.01,
       0.02,
       groups(0.087, 0.052, 0.01),
       kResolutionFine,
       0.001,
       groups(102000, 53500, 18000),
       0.10,
       groups(1.7, 1.0, 0.2),
       groups(0.5, 0.5, 0.1),
       0.05,
       0.005,
       20,
       1e-4,
       1,
       0,
       1e-4,
       1e-4,
       2,
       4.4e-4,
       2.9e-4},
      {"budget_xarm7",
       17,
       5e-5,
       2e-3,
       50,
       0.10,
       0.03,
       0.05,
       groups(0.435, 0.26, 0.05),
       groups(0.348, 0.208, 0.04),
       0.002,
       groups(40800, 21400, 7200),
       0.25,
       groups(3.5, 2.1, 0.4),
       groups(1.0, 1.0, 0.2),
       0.10,
       0.010,
       100,
       1e-3,
       3,
       250,
       1e-3,
       1e-3,
       5,
       8.7e-4,
       8.7e-4},
  };
}

class ProfileValues : public ::testing::TestWithParam<Expected> {};

TEST_P(ProfileValues, LoadsWithTheDesignTableValues) {
  const Expected& e = GetParam();
  const auto r = load_imperfections<kN>(profile_path(e.name));
  ASSERT_TRUE(r.is_ok()) << e.name << ": " << r.reason();
  const Profile& c = r.value();

  // sensor
  expect_vec(c.sensor.encoder_bits, Vec::Constant(e.encoder_bits), "sensor.encoder_bits");
  expect_vec(c.sensor.q_noise_std_rad, Vec::Constant(e.q_noise_std_rad), "sensor.q_noise_std_rad");
  expect_vec(
      c.sensor.q_offset_std_rad, Vec::Constant(e.q_offset_std_rad), "sensor.q_offset_std_rad");
  expect_vec(c.sensor.qd_filter_hz, Vec::Constant(e.qd_filter_hz), "sensor.qd_filter_hz");
  EXPECT_TRUE(c.sensor.qd_from_diff);
  EXPECT_EQ(c.sensor.sensor_delay_cycles, 1);
  EXPECT_EQ(c.sensor.position_source, imperfect::PositionSource::kMotor);

  // actuator
  const imperfect::ActuatorConfig<kN>& a = c.actuator;
  expect_vec(a.torque_scale_std, Vec::Constant(e.torque_scale_std), "actuator.torque_scale_std");
  expect_vec(a.torque_scale_asym_frac,
             Vec::Constant(e.torque_scale_asym_frac),
             "actuator.torque_scale_asym_frac");
  expect_vec(
      a.torque_ripple_frac, Vec::Constant(e.torque_ripple_frac), "actuator.torque_ripple_frac");
  expect_vec(a.ripple_periods_per_rev, Vec::Constant(6.0), "actuator.ripple_periods_per_rev");
  expect_vec(a.deadband_nm, e.deadband_nm, "actuator.deadband_nm");
  expect_vec(
      a.torque_cmd_resolution_nm, e.torque_cmd_resolution_nm, "actuator.torque_cmd_resolution_nm");
  expect_vec(a.motor_time_constant_s,
             Vec::Constant(e.motor_time_constant_s),
             "actuator.motor_time_constant_s");
  EXPECT_EQ(a.command_delay_cycles, 1);
  expect_vec(a.torque_slew_nm_s, e.torque_slew_nm_s, "actuator.torque_slew_nm_s");
  expect_vec(a.knee_speed_rad_s, groups(1.6, 1.6, 2.2), "actuator.knee_speed_rad_s");
  expect_vec(a.zero_torque_speed_rad_s, groups(2.6, 2.6, 3.6), "actuator.zero_torque_speed_rad_s");
  EXPECT_DOUBLE_EQ(a.bus.v_nom_v, 48.0);
  EXPECT_DOUBLE_EQ(a.bus.r_source_ohm, e.bus_r_source_ohm);
  EXPECT_DOUBLE_EQ(a.bus.v_min_v, 40.0);

  // friction
  expect_vec(c.friction.coulomb_nm, e.coulomb_nm, "friction.coulomb_nm");
  expect_vec(c.friction.viscous_nm_s_rad, e.viscous_nm_s_rad, "friction.viscous_nm_s_rad");

  // model: payload 0, offset 0, radius 0 in all three (swept separately)
  expect_vec(c.model.link_mass_scale_std,
             Vec::Constant(e.link_mass_scale_std),
             "model.link_mass_scale_std");
  EXPECT_DOUBLE_EQ(c.model.com_shift_std_m, e.com_shift_std_m);
  EXPECT_EQ(c.model.payload_kg, 0.0);
  EXPECT_EQ(c.model.payload_offset_m, common::Vec3::Zero());
  EXPECT_EQ(c.model.payload_radius_m, 0.0);

  // timing
  EXPECT_DOUBLE_EQ(c.timing.period_jitter_std_us, e.period_jitter_std_us);
  EXPECT_DOUBLE_EQ(c.timing.overrun_prob, e.overrun_prob);
  EXPECT_EQ(c.timing.overrun_cycles, e.overrun_cycles);

  // comm
  EXPECT_DOUBLE_EQ(c.comm.servo_rate_hz, e.servo_rate_hz);
  EXPECT_DOUBLE_EQ(c.comm.packet_loss_prob, e.packet_loss_prob);
  EXPECT_DOUBLE_EQ(c.comm.latency_burst_prob, e.latency_burst_prob);
  EXPECT_EQ(c.comm.burst_cycles, e.burst_cycles);

  // transmission: catalog stiffness (elastic on every joint), gear ratio 100 (eRob)
  const imperfect::TransmissionConfig<kN>& t = c.transmission;
  expect_vec(t.stiffness_k1_nm_rad, groups(31000, 13000, 4700), "transmission.stiffness_k1_nm_rad");
  expect_vec(t.stiffness_k2_nm_rad, groups(50000, 25000, 6100), "transmission.stiffness_k2_nm_rad");
  expect_vec(t.stiffness_k3_nm_rad, groups(57000, 29000, 7100), "transmission.stiffness_k3_nm_rad");
  expect_vec(t.torque_t1_nm, groups(14, 7, 2), "transmission.torque_t1_nm");
  expect_vec(t.torque_t2_nm, groups(48, 25, 6.9), "transmission.torque_t2_nm");
  expect_vec(t.ratio, Vec::Constant(100.0), "transmission.ratio");
  expect_vec(t.error_amp_rad, Vec::Constant(e.error_amp_rad), "transmission.error_amp_rad");
  expect_vec(t.lost_motion_rad, Vec::Constant(e.lost_motion_rad), "transmission.lost_motion_rad");
}

INSTANTIATE_TEST_SUITE_P(Profiles,
                         ProfileValues,
                         ::testing::ValuesIn(expected_profiles()),
                         [](const ::testing::TestParamInfo<Expected>& info) {
                           return info.param.name;
                         });

// The controller sees the same nominal deflection values as the plant's catalog stiffness, so the
// compensation is active and matched on all three profiles (brief: "cobot.deflection_comp
// (default.yaml, enabled) is active on all three profiles since they use the motor encoder").
TEST(ProfileValues, DefaultConfigCompensationMatchesTheProfilesCatalogStiffness) {
  const auto cfg = load_config<kN>(fs::path(COBOT_REPO_DIR) / "config/default.yaml");
  ASSERT_TRUE(cfg.is_ok()) << cfg.reason();
  const DeflectionCompConfig<kN>& d = cfg.value().deflection_comp;
  EXPECT_TRUE(d.enabled);
  for (const char* name : {"premium_fr3", "mid_kr1205", "budget_xarm7"}) {
    const auto r = load_imperfections<kN>(profile_path(name));
    ASSERT_TRUE(r.is_ok()) << name << ": " << r.reason();
    const imperfect::TransmissionConfig<kN>& t = r.value().transmission;
    expect_vec(t.stiffness_k1_nm_rad, d.stiffness_k1_nm_rad, std::string(name) + " k1");
    expect_vec(t.stiffness_k2_nm_rad, d.stiffness_k2_nm_rad, std::string(name) + " k2");
    expect_vec(t.stiffness_k3_nm_rad, d.stiffness_k3_nm_rad, std::string(name) + " k3");
    expect_vec(t.torque_t1_nm, d.torque_t1_nm, std::string(name) + " t1");
    expect_vec(t.torque_t2_nm, d.torque_t2_nm, std::string(name) + " t2");
  }
}

// An unknown key added to a copy of a profile's comm, transmission or actuator.bus group is
// rejected, and the error names it with its full path (review focus 5). The copy is edited as a
// YAML tree, so the test does not depend on how the profile is formatted.
struct UnknownKeyCase {
  std::vector<std::string> group;  // path below `imperfections`
  std::string full_key;            // expected in the error
};

Result<Profile> load_with_unknown_key(const std::string& profile,
                                      const std::vector<std::string>& group,
                                      std::size_t index) {
  YAML::Node root = YAML::LoadFile(profile_path(profile).string());
  YAML::Node node = root["imperfections"];
  for (const std::string& key : group) {
    node.reset(node[key]);
  }
  EXPECT_TRUE(node.IsMap()) << profile << ": group not found";
  node["bogus_key_nm"] = 1.0;
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "pipeline_config_profiles";
  fs::create_directories(dir);
  const fs::path out = dir / (profile + "_unknown_" + std::to_string(index) + ".yaml");
  YAML::Emitter emitter;
  emitter << root;
  std::ofstream(out, std::ios::binary) << emitter.c_str() << "\n";
  return load_imperfections<kN>(out);
}

TEST(ProfileValues, UnknownKeyInNewGroupsRejectedNamingIt) {
  const std::vector<UnknownKeyCase> cases = {
      {{"comm"}, "imperfections.comm.bogus_key_nm"},
      {{"transmission"}, "imperfections.transmission.bogus_key_nm"},
      {{"actuator", "bus"}, "imperfections.actuator.bus.bogus_key_nm"},
  };
  for (const char* name : {"premium_fr3", "mid_kr1205", "budget_xarm7"}) {
    ASSERT_TRUE(fs::exists(profile_path(name))) << profile_path(name);
    // The unedited file loads, so a rejection below comes from the added key.
    const auto clean = load_imperfections<kN>(profile_path(name));
    ASSERT_TRUE(clean.is_ok()) << name << ": " << clean.reason();
    for (std::size_t i = 0; i < cases.size(); ++i) {
      const auto r = load_with_unknown_key(name, cases[i].group, i);
      ASSERT_FALSE(r.is_ok()) << name << " " << cases[i].full_key;
      EXPECT_NE(r.reason().find(cases[i].full_key), std::string::npos)
          << name << ": " << r.reason();
    }
  }
}

}  // namespace
}  // namespace cobot::pipeline
