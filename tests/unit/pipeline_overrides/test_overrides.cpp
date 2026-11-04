// Typed key overrides (feature studio-core, cobot_studio design 6.6, ADR-021,
// cobot/pipeline/overrides.hpp). A KeyOverride names "config.<path under cobot:>" or
// "<path under imperfections:>"; its value is a double (broadcast to per-joint keys), a per-joint
// list, a bool or a string. A key absent from the file but valid in the schema is allowed (the map
// entries are created) and the strict loader decides; an unknown key is the loader's usual error
// naming the full key; a list of the wrong length names the key and both lengths; the same key
// twice is an error. The virtual key config.computed_torque.omega sets kp = omega^2, kd = 2 omega
// on every joint, also through the single-key load_config_with used by cobot_bench sweep.
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <system_error>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/imperfect/config.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/overrides.hpp"
#include "cobot/pipeline/sweep_config.hpp"

namespace cobot::pipeline {
namespace {

namespace fs = std::filesystem;
constexpr int kN = 7;
using Overrides = std::vector<KeyOverride>;

const fs::path kRepo = COBOT_REPO_DIR;
const fs::path kDefault = kRepo / "config/default.yaml";
const fs::path kIdeal = kRepo / "config/imperfections/ideal.yaml";
const fs::path kRealistic = kRepo / "config/imperfections/realistic.yaml";

fs::path out_dir() {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "pipeline_overrides";
  fs::create_directories(dir);
  return dir;
}

std::string read_text(const fs::path& p) {
  std::ifstream in(p, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

bool contains(const std::string& text, const std::string& part) {
  return text.find(part) != std::string::npos;
}

// --- imperfection keys ---

TEST(PipelineOverrides, KeyOfAGroupAbsentFromTheFileLoads) {
  // ideal.yaml has no transmission group: the override creates it.
  const auto r = load_imperfections_with<kN>(
      kIdeal, Overrides{KeyOverride{"transmission.lost_motion_rad", 2.9e-4}});
  ASSERT_TRUE(r.is_ok()) << r.reason();
  for (int j = 0; j < kN; ++j) {
    EXPECT_EQ(r.value().transmission.lost_motion_rad(j), 2.9e-4) << j;
    EXPECT_EQ(r.value().transmission.stiffness_k3_nm_rad(j), 0.0) << "rigid joint " << j;
  }
}

TEST(PipelineOverrides, UnknownImperfectionKeyIsTheLoaderErrorNamingTheFullKey) {
  const auto r =
      load_imperfections_with<kN>(kIdeal, Overrides{KeyOverride{"sensor.q_noise_stdd", 1.0e-4}});
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(r.reason(), "imperfections.sensor.q_noise_stdd")) << r.reason();
}

TEST(PipelineOverrides, ListOfTheWrongLengthNamesTheKeyAndBothLengths) {
  const std::vector<double> six{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  const auto r =
      load_imperfections_with<kN>(kIdeal, Overrides{KeyOverride{"friction.coulomb_nm", six}});
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(r.reason(), "friction.coulomb_nm")) << r.reason();
  EXPECT_TRUE(contains(r.reason(), "7")) << r.reason();
  EXPECT_TRUE(contains(r.reason(), "6")) << r.reason();
}

TEST(PipelineOverrides, PerJointListScalarBroadcastAndChoiceApply) {
  const std::vector<double> coulomb{1.3, 1.3, 0.8, 0.8, 0.3, 0.3, 0.3};
  const auto r = load_imperfections_with<kN>(
      kIdeal,
      Overrides{KeyOverride{"friction.coulomb_nm", coulomb},
                KeyOverride{"sensor.q_noise_std_rad", 1.0e-4},
                KeyOverride{"sensor.position_source", std::string("motor")}});
  ASSERT_TRUE(r.is_ok()) << r.reason();
  for (int j = 0; j < kN; ++j) {
    EXPECT_EQ(r.value().friction.coulomb_nm(j), coulomb[static_cast<std::size_t>(j)]) << j;
    EXPECT_EQ(r.value().sensor.q_noise_std_rad(j), 1.0e-4) << j;
  }
  EXPECT_EQ(r.value().sensor.position_source, imperfect::PositionSource::kMotor);
}

TEST(PipelineOverrides, BadChoiceIsTheLoaderError) {
  const auto r = load_imperfections_with<kN>(
      kIdeal, Overrides{KeyOverride{"sensor.position_source", std::string("joint")}});
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(r.reason(), "imperfections.sensor.position_source")) << r.reason();
}

TEST(PipelineOverrides, NestedKeyAndBoolApply) {
  // realistic.yaml has actuator.bus (all 0, off): v_min_v 0 < v_nom_v 48 turns it on.
  const auto r = load_imperfections_with<kN>(kRealistic,
                                             Overrides{KeyOverride{"actuator.bus.v_nom_v", 48.0},
                                                       KeyOverride{"sensor.qd_from_diff", false}});
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().actuator.bus.v_nom_v, 48.0);
  EXPECT_FALSE(r.value().sensor.qd_from_diff);  // true in realistic.yaml
}

TEST(PipelineOverrides, SameKeyTwiceIsAnError) {
  const auto r = load_imperfections_with<kN>(
      kIdeal,
      Overrides{KeyOverride{"model.payload_kg", 1.0}, KeyOverride{"model.payload_kg", 2.0}});
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(r.reason(), "model.payload_kg")) << r.reason();
}

TEST(PipelineOverrides, NoOverridesEqualsThePlainLoader) {
  const auto plain = load_imperfections<kN>(kRealistic);
  const auto with = load_imperfections_with<kN>(kRealistic, Overrides{});
  ASSERT_TRUE(plain.is_ok() && with.is_ok()) << with.reason();
  EXPECT_EQ(with.value().friction.coulomb_nm, plain.value().friction.coulomb_nm);
  EXPECT_EQ(with.value().sensor.encoder_bits, plain.value().sensor.encoder_bits);
  EXPECT_EQ(with.value().seed, plain.value().seed);
}

TEST(PipelineOverrides, ScalarOverrideMatchesTheSingleKeySweepLoader) {
  // Same text (%.17g) as load_imperfections_with(yaml, name, level): bit-equal values.
  const double level = 0.1 + 0.2;  // 0.30000000000000004, needs all 17 digits
  const auto multi = load_imperfections_with<kN>(
      kRealistic, Overrides{KeyOverride{"actuator.torque_ripple_frac", level}});
  const auto single =
      load_imperfections_with<kN>(kRealistic, std::string("actuator.torque_ripple_frac"), level);
  ASSERT_TRUE(multi.is_ok() && single.is_ok()) << multi.reason() << single.reason();
  EXPECT_EQ(multi.value().actuator.torque_ripple_frac, single.value().actuator.torque_ripple_frac);
}

// --- controller keys ---

TEST(PipelineOverrides, OmegaSetsKpAndKdOnEveryJoint) {
  // omega 50: kp = 50^2 = 2500, kd = 2 * 50 = 100 (default.yaml has 6400 / 160).
  const auto r =
      load_config_with<kN>(kDefault, Overrides{KeyOverride{"config.computed_torque.omega", 50.0}});
  ASSERT_TRUE(r.is_ok()) << r.reason();
  for (int j = 0; j < kN; ++j) {
    EXPECT_EQ(r.value().computed_torque.kp(j), 2500.0) << j;
    EXPECT_EQ(r.value().computed_torque.kd(j), 100.0) << j;
  }
}

TEST(PipelineOverrides, OmegaWithKpOrKdInTheSameListIsAnError) {
  const auto with_kp =
      load_config_with<kN>(kDefault,
                           Overrides{KeyOverride{"config.computed_torque.omega", 50.0},
                                     KeyOverride{"config.computed_torque.kp", 900.0}});
  EXPECT_FALSE(with_kp.is_ok());
  EXPECT_TRUE(contains(with_kp.reason(), "omega")) << with_kp.reason();
  const auto with_kd =
      load_config_with<kN>(kDefault,
                           Overrides{KeyOverride{"config.computed_torque.kd", 60.0},
                                     KeyOverride{"config.computed_torque.omega", 50.0}});
  EXPECT_FALSE(with_kd.is_ok());
}

TEST(PipelineOverrides, OmegaMustBePositive) {
  EXPECT_FALSE(
      load_config_with<kN>(kDefault, Overrides{KeyOverride{"config.computed_torque.omega", 0.0}})
          .is_ok());
  EXPECT_FALSE(
      load_config_with<kN>(kDefault, Overrides{KeyOverride{"config.computed_torque.omega", -5.0}})
          .is_ok());
}

TEST(PipelineOverrides, SingleKeyLoaderAcceptsOmega) {
  // cobot_bench sweep param: omega 40 -> kp = 40^2 = 1600, kd = 2 * 40 = 80.
  const auto r = load_config_with<kN>(kDefault, std::string("config.computed_torque.omega"), 40.0);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  for (int j = 0; j < kN; ++j) {
    EXPECT_EQ(r.value().computed_torque.kp(j), 1600.0) << j;
    EXPECT_EQ(r.value().computed_torque.kd(j), 80.0) << j;
  }
}

TEST(PipelineOverrides, ScalarBroadcastsToAPerJointConfigList) {
  const auto r =
      load_config_with<kN>(kDefault, Overrides{KeyOverride{"config.computed_torque.kp", 900.0}});
  ASSERT_TRUE(r.is_ok()) << r.reason();
  for (int j = 0; j < kN; ++j) {
    EXPECT_EQ(r.value().computed_torque.kp(j), 900.0) << j;
    EXPECT_EQ(r.value().computed_torque.kd(j), 160.0) << "untouched " << j;
  }
}

TEST(PipelineOverrides, BoolConfigKeyApplies) {
  const auto r = load_config_with<kN>(
      kDefault, Overrides{KeyOverride{"config.deflection_comp.enabled", false}});
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_FALSE(r.value().deflection_comp.enabled);  // true in default.yaml
}

TEST(PipelineOverrides, ConfigScalarMatchesTheSingleKeyLoader) {
  const double level = 0.1 + 0.2;  // ms, 17 significant digits
  const auto multi =
      load_config_with<kN>(kDefault, Overrides{KeyOverride{"config.safety.hal_timeout_ms", level}});
  const auto single =
      load_config_with<kN>(kDefault, std::string("config.safety.hal_timeout_ms"), level);
  ASSERT_TRUE(multi.is_ok() && single.is_ok()) << multi.reason() << single.reason();
  EXPECT_EQ(multi.value().safety.hal_timeout_s, single.value().safety.hal_timeout_s);
}

TEST(PipelineOverrides, UnknownConfigKeyNamesTheFullKey) {
  const auto r =
      load_config_with<kN>(kDefault, Overrides{KeyOverride{"config.safety.hal_timeout_mss", 5.0}});
  ASSERT_FALSE(r.is_ok());
  // The loader's usual "cobot.safety.hal_timeout_mss: unknown key".
  EXPECT_TRUE(contains(r.reason(), "safety.hal_timeout_mss")) << r.reason();
}

TEST(PipelineOverrides, ConfigKeyTwiceIsAnError) {
  const auto r = load_config_with<kN>(kDefault,
                                      Overrides{KeyOverride{"config.safety.hal_timeout_ms", 5.0},
                                                KeyOverride{"config.safety.hal_timeout_ms", 8.0}});
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(r.reason(), "config.safety.hal_timeout_ms")) << r.reason();
}

// --- writers ---

TEST(PipelineOverrides, WrittenConfigLoadsWithThePlainLoaderFromAnotherDirectory) {
  const fs::path dst = out_dir() / "written_omega" / "config.yaml";
  fs::create_directories(dst.parent_path());
  fs::remove(dst);
  const Overrides ov{KeyOverride{"config.computed_torque.omega", 50.0},
                     KeyOverride{"config.safety.hal_timeout_ms", 0.1 + 0.2}};
  const common::Status s = write_config_with(kDefault, ov, dst);
  ASSERT_TRUE(s.is_ok()) << s.reason();
  const auto written = load_config<kN>(dst);  // no virtual key needed: kp and kd are in the file
  const auto expected = load_config_with<kN>(kDefault, ov);
  ASSERT_TRUE(written.is_ok() && expected.is_ok()) << written.reason() << expected.reason();
  EXPECT_EQ(written.value().computed_torque.kp, expected.value().computed_torque.kp);
  EXPECT_EQ(written.value().computed_torque.kd, expected.value().computed_torque.kd);
  EXPECT_EQ(written.value().safety.hal_timeout_s, expected.value().safety.hal_timeout_s);
  // Relative paths of default.yaml are written absolute, so they still name the same places.
  std::error_code ec;
  EXPECT_TRUE(fs::equivalent(written.value().urdf_path, expected.value().urdf_path, ec))
      << written.value().urdf_path;
  EXPECT_EQ(written.value().log_output_dir.lexically_normal(),
            expected.value().log_output_dir.lexically_normal());
  EXPECT_EQ(written.value().home_q_rad, expected.value().home_q_rad);
  EXPECT_FALSE(contains(read_text(dst), "omega:")) << "the virtual key is not a file key";
}

TEST(PipelineOverrides, WrittenImperfectionsLoadWithThePlainLoader) {
  const fs::path dst = out_dir() / "written_imp" / "imperfections.yaml";
  fs::create_directories(dst.parent_path());
  fs::remove(dst);
  const Overrides ov{KeyOverride{"transmission.lost_motion_rad", 2.9e-4},
                     KeyOverride{"sensor.q_noise_std_rad", 1.0e-4}};
  const common::Status s = write_imperfections_with(kIdeal, ov, dst);
  ASSERT_TRUE(s.is_ok()) << s.reason();
  const auto written = load_imperfections<kN>(dst);
  ASSERT_TRUE(written.is_ok()) << written.reason();
  EXPECT_EQ(written.value().transmission.lost_motion_rad(3), 2.9e-4);
  EXPECT_EQ(written.value().sensor.q_noise_std_rad(6), 1.0e-4);
}

TEST(PipelineOverrides, WriterValidatesFirstAndWritesNothingOnError) {
  const fs::path dir = out_dir() / "written_bad";
  fs::create_directories(dir);
  fs::remove(dir / "config.yaml");
  fs::remove(dir / "imperfections.yaml");
  const common::Status c = write_config_with(
      kDefault, Overrides{KeyOverride{"config.computed_torque.omega", -1.0}}, dir / "config.yaml");
  EXPECT_FALSE(c.is_ok());
  EXPECT_FALSE(fs::exists(dir / "config.yaml"));
  const common::Status i = write_imperfections_with(
      kIdeal, Overrides{KeyOverride{"sensor.q_noise_stdd", 1.0}}, dir / "imperfections.yaml");
  ASSERT_FALSE(i.is_ok());
  EXPECT_TRUE(contains(i.reason(), "imperfections.sensor.q_noise_stdd")) << i.reason();
  EXPECT_FALSE(fs::exists(dir / "imperfections.yaml"));
}

}  // namespace
}  // namespace cobot::pipeline
