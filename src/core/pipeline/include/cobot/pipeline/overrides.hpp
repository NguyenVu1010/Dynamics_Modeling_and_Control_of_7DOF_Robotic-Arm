#pragma once

#include <filesystem>
#include <string>
#include <variant>
#include <vector>

#include "cobot/common/result.hpp"
#include "cobot/imperfect/config.hpp"
#include "cobot/pipeline/config.hpp"

// Typed key overrides of the controller and imperfection files (feature studio-core, cobot_studio
// design 6.6, ADR-021). The only YAML code for these files stays in pipeline: the studio keeps the
// base files' text and applies its edits here, so a written file reruns with cobot_bench exactly.
namespace cobot::pipeline {

// One edited key.
//   key: "config.<path under cobot:>" (any depth, e.g. config.safety.hal_timeout_ms,
//        config.computed_torque.kp) or "<path under imperfections:>" (sensor.q_noise_std_rad,
//        actuator.bus.v_nom_v, transmission.lost_motion_rad);
//   value: a double (written %.17g; it replaces every entry of a list already in the file, so a
//        scalar broadcasts to per-joint keys), a list (written as is; a wrong length is the
//        loader's error naming the key and both lengths), a bool or a string (choice keys).
// A key absent from the file but valid in the schema is allowed: the map entries are created (an
// optional imperfection group that is absent gets its required keys at their off values) and the
// strict loader decides, so an unknown key is its usual error naming the full key. The same key
// twice is an error. The virtual key config.computed_torque.omega (double > 0) sets kp = omega^2
// and kd = 2 omega on every joint; overriding it together with kp or kd is an error.
struct KeyOverride {
  std::string key;
  std::variant<double, std::vector<double>, bool, std::string> value;
};

inline constexpr const char* kOmegaKey = "config.computed_torque.omega";

// load_config / load_imperfections with the overrides applied before validation. Config keys
// start with "config."; imperfection keys never do.
template <int N>
Result<Config<N>> load_config_with(const std::filesystem::path& yaml,
                                   const std::vector<KeyOverride>& overrides);
template <int N>
Result<imperfect::ImperfectionConfig<N>> load_imperfections_with(
    const std::filesystem::path& yaml, const std::vector<KeyOverride>& overrides);

// Validates by loading first (model::kArmDof joints), then writes `src` with the overrides applied
// to `dst`; nothing is written on an error. Untouched values keep their source text (comments are
// dropped); the relative paths cobot.urdf_path and cobot.log.output_dir are written absolute.
common::Status write_config_with(const std::filesystem::path& src,
                                 const std::vector<KeyOverride>& overrides,
                                 const std::filesystem::path& dst);
common::Status write_imperfections_with(const std::filesystem::path& src,
                                        const std::vector<KeyOverride>& overrides,
                                        const std::filesystem::path& dst);

}  // namespace cobot::pipeline
