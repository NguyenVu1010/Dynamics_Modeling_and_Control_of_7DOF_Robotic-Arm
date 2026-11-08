#include "analysis.hpp"

#include <algorithm>
#include <utility>
#include <variant>

namespace cobot::bench {
namespace {

using pipeline::KeyOverride;

bool at_or_above(const Counts& c, double target) {
  return c.runs > 0 && static_cast<double>(c.passed) / static_cast<double>(c.runs) >= target;
}

bool has(const std::vector<std::string>& list, const char* item) {
  return std::find(list.begin(), list.end(), item) != list.end();
}

}  // namespace

const char* to_string(Sensitivity s) {
  switch (s) {
    case Sensitivity::kStable:
      return "stable";
    case Sensitivity::kMayChange:
      return "may change";
    case Sensitivity::kNotVaried:
      return "not varied";
  }
  return "not varied";
}

Sensitivity sensitivity_rule(const Counts& main,
                             const Counts& base,
                             const std::optional<std::array<Counts, 2>>& ends,
                             double target_pass_rate) {
  if (!ends) {
    return Sensitivity::kNotVaried;
  }
  const bool side = at_or_above(main, target_pass_rate);
  for (const Counts& e : *ends) {
    if (at_or_above(e, target_pass_rate) != side || (e.held > 0 && base.held == 0)) {
      return Sensitivity::kMayChange;
    }
  }
  return Sensitivity::kStable;
}

common::Result<KeyOverride> variant_override(const std::filesystem::path& profile,
                                             const pipeline::ProvenanceEntry& entry,
                                             End end) {
  using Out = common::Result<KeyOverride>;
  const std::size_t i = end == End::kLow ? 0 : 1;
  if (entry.range) {
    return Out::ok(KeyOverride{entry.key, (*entry.range)[i]});
  }
  if (!entry.scale) {
    return Out::error(entry.key + ": no range or scale to vary");
  }
  const common::Result<pipeline::ProfileValue> value = pipeline::profile_value(profile, entry.key);
  if (!value) {
    return Out::error(value.reason());
  }
  const double factor = (*entry.scale)[i];
  if (std::holds_alternative<double>(value.value())) {
    return Out::ok(KeyOverride{entry.key, std::get<double>(value.value()) * factor});
  }
  std::vector<double> list = std::get<std::vector<double>>(value.value());
  for (double& v : list) {
    v *= factor;
  }
  return Out::ok(KeyOverride{entry.key, std::move(list)});
}

common::Result<std::vector<KeyOverride>> group_off_overrides(const std::filesystem::path& profile,
                                                             const std::string& group) {
  using Out = common::Result<std::vector<KeyOverride>>;
  if (std::find(kAblationGroups.begin(), kAblationGroups.end(), group) == kAblationGroups.end()) {
    return Out::error("unknown imperfection group " + group);
  }
  const common::Result<std::vector<std::string>> keys = pipeline::active_imperfection_keys(profile);
  if (!keys) {
    return Out::error(keys.reason());
  }
  std::vector<KeyOverride> out;
  for (const std::string& key : keys.value()) {
    if (key.compare(0, group.size() + 1, group + ".") != 0 || key == "sensor.qd_filter_hz" ||
        key == "transmission.ratio") {
      continue;
    }
    if (key == "sensor.position_source") {
      out.push_back(KeyOverride{key, std::string("link")});
    } else if (key == "sensor.qd_from_diff") {
      out.push_back(KeyOverride{key, false});
    } else {
      out.push_back(KeyOverride{key, 0.0});  // a scalar replaces every element of a list
    }
  }
  return Out::ok(std::move(out));
}

bool mark_main_causes(std::vector<Cause>& causes) {
  std::size_t best = 0;
  bool any = false;
  for (const Cause& c : causes) {
    if (c.passed_without > c.base_passed && (!any || c.passed_without > best)) {
      best = c.passed_without;
      any = true;
    }
  }
  for (Cause& c : causes) {
    c.main = any && c.passed_without == best && c.passed_without > c.base_passed;
  }
  return any;
}

std::vector<std::string> suggest(const SuggestionFacts& f) {
  std::vector<std::string> out;
  std::size_t holds = 0;
  std::size_t timeouts = 0;
  for (const auto& [reason, n] : f.hold_reasons) {
    holds += n;
    timeouts += reason == "state_timeout" || reason == "overspeed" ? n : 0;
  }
  const bool torque_law = f.algorithm == control::Algorithm::kComputedTorque ||
                          f.algorithm == control::Algorithm::kPdGravity;
  if (torque_law && f.servo_rate_hz > 0.0 && holds > 0 && 2 * timeouts > holds) {
    out.emplace_back("needs a position loop in the drive (algorithm drive_position)");
  }
  const std::array<std::pair<const char*, const char*>, 4> by_cause = {{
      {"comm", "raise the servo rate or reduce packet loss"},
      {"sensor", "a finer encoder or less sensor noise"},
      {"transmission", "a stiffer transmission or less lost motion"},
      {"friction", "friction identification and compensation"},
  }};
  for (const auto& [group, text] : by_cause) {
    if (has(f.main_causes, group)) {
      out.emplace_back(text);
    }
  }
  if (f.no_admissible_gain) {
    out.emplace_back("no admissible gain in the sweep: widen the levels or change the law");
  }
  return out;
}

}  // namespace cobot::bench
