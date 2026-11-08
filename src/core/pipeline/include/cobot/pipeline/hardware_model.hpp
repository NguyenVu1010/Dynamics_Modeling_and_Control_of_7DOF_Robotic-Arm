#pragma once

#include <array>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "cobot/common/result.hpp"
#include "cobot/control/algorithm.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/overrides.hpp"

// Hardware model file config/hardware/<model>.yaml (feature hardware-catalog, ADR-023, format in
// docs/conventions.md "Hardware model file"): the imperfection profile, the base controller file,
// the torque levels, the gains per algorithm and the provenance of every active imperfection key.
namespace cobot::pipeline {

enum class ProvenanceKind { kPublished, kDerived, kAssumed };

// One entry of hardware.provenance. range: absolute bounds of a key the profile writes as a scalar,
// lo <= the file's value <= hi. scale: multiplier of every element of a key the profile writes as a
// list, 0 <= lo <= 1 <= hi. derived and assumed need exactly one of the two; published may omit
// both; bool and choice keys take neither, whatever the kind.
struct ProvenanceEntry {
  std::string key;  // dotted, under imperfections: (actuator.bus.v_nom_v)
  ProvenanceKind kind = ProvenanceKind::kAssumed;
  std::optional<std::array<double, 2>> range;
  std::optional<std::array<double, 2>> scale;
};

struct HardwareModel {
  std::string name;
  std::string description;
  std::filesystem::path imperfections;  // resolved relative to the model file
  std::filesystem::path controller;     // resolved relative to the model file
  std::vector<double> rated_torque_nm;  // model::kArmDof values, config.limits.rated_torque_nm
  std::vector<double> peak_torque_nm;   // model::kArmDof values, config.limits.peak_torque_nm
  std::map<control::Algorithm, std::vector<KeyOverride>> algorithms;  // the listed ones
  std::vector<ProvenanceEntry> provenance;
};

// Errors name the file and the full key (`hardware.provenance[3].kind: ...`), all of them at once.
// An unknown key, a missing required key, a bad type, an unknown algorithm name, a file that does
// not load, an override or a limit the controller loader refuses for a listed algorithm, a key the
// task owns (a non-zero cobot.payload in the controller file, an override under config.payload. or
// config.trajectory.), and a provenance list that breaks the rule above or
// active_imperfection_keys are all errors.
Result<HardwareModel> load_hardware_model(const std::filesystem::path& yaml);

// What the model applies on its controller file for `algorithm`: config.control.algorithm, the two
// torque lists and that algorithm's overrides (none when the model does not list it).
std::vector<KeyOverride> model_overrides(const HardwareModel& model, control::Algorithm algorithm);

// load_config_with(model.controller, model_overrides(model, algorithm)). An algorithm the model
// does not list is an error naming the model and the listed algorithms.
template <int N>
Result<Config<N>> config_for(const HardwareModel& model, control::Algorithm algorithm);

// Dotted keys of the leaves of a profile that need a provenance entry (strict load first, its
// errors come back as they are). A leaf is active unless at its off value: 0 (every element of a
// list), false, or the group or key absent. Never listed: seed, source and model.payload_* (the
// task owns the payload). Dependent keys count only when their owner is active: sensor.qd_filter_hz
// needs sensor.qd_from_diff true, actuator.ripple_periods_per_rev needs a non-zero
// actuator.torque_ripple_frac. sensor.position_source counts when it differs from the loader's
// default, link. Order: as in the file.
Result<std::vector<std::string>> active_imperfection_keys(const std::filesystem::path& profile);

// The value of a numeric leaf of a profile file (dotted key under imperfections:, as in the
// provenance list; feature verify-analysis): a scalar as a double, a list as a list. A key absent
// from the file, or a leaf that is not a number or a list of numbers, is an error naming the key.
using ProfileValue = std::variant<double, std::vector<double>>;
Result<ProfileValue> profile_value(const std::filesystem::path& profile, const std::string& key);

}  // namespace cobot::pipeline
