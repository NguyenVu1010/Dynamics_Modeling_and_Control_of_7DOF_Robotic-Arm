#pragma once

#include <limits>
#include <map>
#include <string>
#include <string_view>
#include <vector>

#include "cobot/common/result.hpp"
#include "cobot/pipeline/overrides.hpp"
#include "cobot/studio/profile_catalog.hpp"

// The studio's parameter catalog (feature studio-core, cobot_studio design 9, ADR-021):
// hand-listed, key names exactly as the YAML files (pipeline::KeyOverride keys).
namespace cobot::studio {

enum class ParamKind { kNumber, kPerJoint, kBool, kChoice };

// Which noise line a changed key gives (design 5.1): sensor q - q_true, actuator
// tau_motor[k+1] - tau[k], friction tau_friction; none for model, timing and comm keys.
enum class NoiseSource { kNone, kSensor, kActuator, kFriction };

struct ParamInfo {
  std::string key, group, unit;
  ParamKind kind = ParamKind::kNumber;
  // Input limits (every entry of a per-joint list); omega [10, 200] rad/s, hal_timeout_ms
  // [1, 50] ms. The loaders' own range rules apply as well.
  double min = -std::numeric_limits<double>::infinity();
  double max = std::numeric_limits<double>::infinity();
  std::vector<std::string> choices;  // kChoice
  // Numeric keys a studio sweep can vary as one level. bench::sweep also needs the key in the
  // base file (a scalar for config keys): a studio sweep of an imperfection key the base file
  // does not set is rerun with cobot_bench run on each run's materialised files instead.
  bool sweepable = false;
  NoiseSource noise = NoiseSource::kNone;
  bool gazebo_supported = true;  // phase B greys the others
  // Presentation (studio config UI design 3.2). English, label at most 28 characters and
  // description at most 90; display_unit is UTF-8 (e.g. "µrad") and "" exactly when unit is "".
  std::string label;           // short name, e.g. "Position noise q"
  std::string description;     // one line, e.g. "Encoder noise standard deviation, per joint"
  std::string display_unit;    // unit shown in the UI
  double display_scale = 1.0;  // display value = YAML value * display_scale
  bool xyz = false;            // a list of x, y, z, not joints (model.payload_offset_m only)
};

const std::vector<ParamInfo>& param_catalog();
const ParamInfo* find_param(std::string_view key);  // nullptr when not listed

// A display value to three significant digits, trailing zeros dropped and never an exponent:
// 1.745 -> "1.75", 0.0005 -> "0.0005", -2.5 -> "-2.5"; a magnitude of at least 1000 is rounded
// to an integer (1234.6 -> "1235", 50000 -> "50000").
std::string format_display(double display_value);

// Value of `key` in the base pair in YAML units (e.g. ms for hal_timeout_ms, degrees for _deg
// keys; omega = sqrt(kp of joint 1)), read through the pipeline loaders, so a key the file does
// not set gives the loader's default. kNumber: double; kPerJoint: list; kBool; kChoice: string.
common::Result<pipeline::KeyOverride> base_value(const BasePair& base, const std::string& key);
// base_value of every catalog key, each file read once (window start-up); the keys of a file that
// does not load are left out.
std::map<std::string, pipeline::KeyOverride> base_values(const BasePair& base);

}  // namespace cobot::studio
