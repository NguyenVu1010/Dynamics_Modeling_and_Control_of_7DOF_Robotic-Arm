#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/pipeline/hardware_model.hpp"

#include "hw_test_util.hpp"

// A small valid hardware model and its imperfection profile, written per test so that one edit
// shows one error. The profile is config/imperfections/ideal.yaml with six active keys:
//   sensor.encoder_bits 17, sensor.q_offset_std_rad 2e-3 (scalars), sensor.qd_from_diff true
//   (bool), sensor.qd_filter_hz 50 (active through qd_from_diff), sensor.position_source motor
//   (choice), friction.coulomb_nm (a per-joint list).
namespace cobot::pipeline::hw_test {

inline std::string profile_fixture_text() {
  std::string t = read_text(kRepo / "config/imperfections/ideal.yaml");
  t = set_line(t, "    encoder_bits:", "    encoder_bits: 17");
  t = set_line(t, "    q_offset_std_rad:", "    q_offset_std_rad: 2.0e-3");
  t = set_line(t, "    qd_from_diff:", "    qd_from_diff: true");
  t = set_line(t, "    qd_filter_hz:", "    qd_filter_hz: 50.0");
  t = set_line(
      t, "    sensor_delay_cycles:", "    sensor_delay_cycles: 0\n    position_source: motor");
  t = set_line(t, "    coulomb_nm:", "    coulomb_nm: [3.5, 3.5, 2.1, 2.1, 0.4, 0.4, 0.4]");
  return t;
}

// `@CONTROLLER@` is replaced by the path of config/premium_fr3.yaml relative to the written file.
// Provenance entries, by index:
//   0 encoder_bits, 1 q_offset_std_rad, 2 qd_from_diff, 3 qd_filter_hz, 4 position_source,
//   5 coulomb_nm.
inline std::string hardware_fixture_text() {
  return R"(hardware:
  name: fixture_model
  description: "a model for the loader tests"
  imperfections: fixture_profile.yaml
  controller: @CONTROLLER@
  limits:
    rated_torque_nm: [80, 80, 50, 50, 9, 9, 9]
    peak_torque_nm: [150, 150, 90, 90, 30, 30, 30]
  algorithms:
    computed_torque:
      overrides:
        config.computed_torque.omega: 30.0
    pd_gravity:
      overrides:
        config.pd_gravity.kp: [900, 900, 900, 900, 900, 900, 800]
        config.pd_gravity.kd: 60.0
        config.deflection_comp.enabled: false
        config.log.mode: "off"
  provenance:
    - {key: sensor.encoder_bits, kind: published}
    - {key: sensor.q_offset_std_rad, kind: assumed, range: [5.0e-4, 5.0e-3]}
    - {key: sensor.qd_from_diff, kind: assumed}
    - {key: sensor.qd_filter_hz, kind: assumed, range: [25.0, 100.0]}
    - {key: sensor.position_source, kind: published}
    - {key: friction.coulomb_nm, kind: derived, scale: [0.5, 1.5]}
)";
}

inline constexpr const char* kEntryBits = "    - {key: sensor.encoder_bits,";
inline constexpr const char* kEntryOffset = "    - {key: sensor.q_offset_std_rad,";
inline constexpr const char* kEntryFromDiff = "    - {key: sensor.qd_from_diff,";
inline constexpr const char* kEntryFilter = "    - {key: sensor.qd_filter_hz,";
inline constexpr const char* kEntrySource = "    - {key: sensor.position_source,";
inline constexpr const char* kEntryCoulomb = "    - {key: friction.coulomb_nm,";

// Writes the profile and the model into a directory of this test (`variant` tells several writes
// of one test apart); returns the model's path.
inline fs::path write_fixture(const std::string& hardware_text,
                              const std::string& variant = "base",
                              const std::string& profile_text = profile_fixture_text()) {
  const fs::path dir = test_dir() / variant;
  fs::create_directories(dir);
  write_text(dir / "fixture_profile.yaml", profile_text);
  std::string text = hardware_text;
  const std::string marker = "@CONTROLLER@";
  const std::size_t at = text.find(marker);
  if (at != std::string::npos) {
    const fs::path controller = fs::relative(kRepo / "config/premium_fr3.yaml", dir);
    text.replace(at, marker.size(), controller.generic_string());
  }
  return write_text(dir / "fixture_hardware.yaml", text);
}

// load_hardware_model fails and the message names the file and every part.
inline void expect_load_error(const fs::path& path, const std::vector<std::string>& parts) {
  const auto r = load_hardware_model(path);
  ASSERT_FALSE(r.is_ok()) << path;
  EXPECT_TRUE(contains(r.reason(), path.filename().string())) << r.reason();
  for (const std::string& part : parts) {
    EXPECT_TRUE(contains(r.reason(), part)) << "missing '" << part << "' in: " << r.reason();
  }
}

}  // namespace cobot::pipeline::hw_test
