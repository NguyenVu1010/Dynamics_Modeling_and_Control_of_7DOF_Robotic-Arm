#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "cobot/common/result.hpp"
#include "cobot/control/algorithm.hpp"
#include "cobot/pipeline/hardware_model.hpp"
#include "cobot/pipeline/overrides.hpp"

// The analysis of a verify result (feature verify-analysis, ADR-023 "Báo cáo"): the sensitivity
// of each assumption, the ablation of the imperfection groups on a FAIL and the suggestion table.
// Pure functions; verify.cpp runs the variants.
namespace cobot::bench {

struct Counts {
  std::size_t runs = 0;
  std::size_t passed = 0;
  std::size_t held = 0;
};

enum class Sensitivity { kStable, kMayChange, kNotVaried };
const char* to_string(Sensitivity s);  // "stable" | "may change" | "not varied"

// Plan ambiguity 16 (C12). `ends` absent (a bool or choice key) -> kNotVaried. Else kMayChange when
// at either end (ends[0] low, ends[1] high) the pass rate is on the other side of target_pass_rate
// than the main pass rate (a rate equal to the target is on the side at or above it), or when an
// end has held > 0 while base.held == 0; else kStable.
Sensitivity sensitivity_rule(const Counts& main,
                             const Counts& base,
                             const std::optional<std::array<Counts, 2>>& ends,
                             double target_pass_rate);

enum class End { kLow, kHigh };

// The override of a provenance entry at one end: range -> the bound itself (a double); scale ->
// every element of the profile's value times the factor (a list stays a list, a scalar a scalar).
// An entry with neither is an error naming the key.
common::Result<pipeline::KeyOverride> variant_override(const std::filesystem::path& profile,
                                                       const pipeline::ProvenanceEntry& entry,
                                                       End end);

// The ablation groups (plan ambiguity 17), in this order.
inline constexpr std::array<const char*, 7> kAblationGroups = {
    "sensor", "actuator", "friction", "model", "timing", "comm", "transmission"};

// One override per active key of `group` (pipeline::active_imperfection_keys, in its order) at its
// off value: 0 for a number or a list, false for sensor.qd_from_diff, "link" for
// sensor.position_source. sensor.qd_filter_hz (inactive once qd_from_diff is false) and
// transmission.ratio (a gear ratio has no off value) are left as they are. An unknown group is an
// error naming it.
common::Result<std::vector<pipeline::KeyOverride>> group_off_overrides(
    const std::filesystem::path& profile, const std::string& group);

// One ablation row: the pass count over the ablation seeds without the group, and the pass count
// of the same seeds in the main Monte Carlo.
struct Cause {
  std::string group;
  std::size_t passed_without = 0;
  std::size_t base_passed = 0;
  bool main = false;
};

// main = true on the rows whose passed_without is the largest and > base_passed (ties: all of
// them), false on the others; returns whether any row is main.
bool mark_main_causes(std::vector<Cause>& causes);

struct SuggestionFacts {
  control::Algorithm algorithm = control::Algorithm::kComputedTorque;
  double servo_rate_hz = 0.0;                       // comm.servo_rate_hz of the profile
  std::map<std::string, std::size_t> hold_reasons;  // to_string(HoldReason) -> runs
  std::vector<std::string> main_causes;             // groups of the rows with main = true
  bool no_admissible_gain = false;                  // the tune step ran and chose nothing
};

// The fixed rule table of the brief, in its order, one English line each. "Mostly": the
// state_timeout and overspeed holds are more than half of all holds.
std::vector<std::string> suggest(const SuggestionFacts& facts);

}  // namespace cobot::bench
