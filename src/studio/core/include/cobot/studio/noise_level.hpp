#pragma once

#include <array>
#include <map>
#include <string>

#include "cobot/pipeline/overrides.hpp"
#include "cobot/studio/studio_config.hpp"

// The noise level of the Configuration tab (feature studio-noise-level, studio config UI design
// 3.4, ADR-021): one factor k scales every noise standard deviation of the base pair. Qt-free.
namespace cobot::studio {

// The scaled keys: kNumber (the jitter key) or per-joint lists, every element scaled.
inline constexpr std::array<const char*, 5> kNoiseKeys = {"sensor.q_noise_std_rad",
                                                          "sensor.q_offset_std_rad",
                                                          "sensor.qd_noise_std_rad_s",
                                                          "actuator.torque_scale_std",
                                                          "timing.period_jitter_std_us"};

enum class NoiseLevel { kOff, kProfile, kX2, kX5, kCustom };

double noise_factor(NoiseLevel level);  // 0, 1, 2, 5; kCustom: NaN

// Effective value = the override if present, else base. Returns the one level whose factor k
// gives value == base * k for every element of every key present in `base` (exact ==, the
// product computed as base * k). A key absent from `base` is ignored. Elements with base 0 must
// be 0 and match every k. When several levels match (all bases 0) -> kProfile; none -> kCustom.
NoiseLevel noise_level(const StudioConfig& config,
                       const std::map<std::string, pipeline::KeyOverride>& base);

// Removes the overrides of kNoiseKeys, then for k != 1 adds base * k for each key present in
// `base` whose scaled value differs from base (keys with all-zero base get no override). Other
// overrides keep their order. kCustom: no change.
void apply_noise_level(StudioConfig& config,
                       const std::map<std::string, pipeline::KeyOverride>& base,
                       NoiseLevel level);

}  // namespace cobot::studio
