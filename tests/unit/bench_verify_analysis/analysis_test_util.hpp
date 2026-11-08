#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/imperfect/config.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/hardware_model.hpp"

#include "verify_test_util.hpp"

// Helpers of the bench_verify_analysis tests (brief verify-analysis, PR 6b). The JSON reader, the
// reference file paths and the child cobot_bench come from tests/unit/bench_verify
// (verify_test_util.hpp).
//
// API under test, as these tests read the brief (src/bench unless said otherwise):
//
//   cobot/pipeline/hardware_model.hpp (next to active_imperfection_keys; only pipeline parses YAML)
//     using ProfileValue = std::variant<double, std::vector<double>>;
//     // The value of a numeric leaf of a profile file (dotted key under imperfections:, as in
//     // the provenance list): a scalar as a double, a list as a list. A key absent from the file
//     // is an error naming the key.
//     Result<ProfileValue> profile_value(const std::filesystem::path& profile,
//                                        const std::string& key);
//
//   analysis.hpp
//     enum class Sensitivity { kStable, kMayChange, kNotVaried };
//     const char* to_string(Sensitivity);            // "stable" | "may change" | "not varied"
//     // The rule of ambiguity 16 (C12). `ends` absent: a key without range or scale (bool,
//     // choice) -> kNotVaried. Else kMayChange when at either end the pass rate (passed / runs)
//     // is on the other side of target_pass_rate than the main pass rate (a rate equal to the
//     // target is on the side at or above it, as the verdict's >=), or when an end has held > 0
//     // while base.held == 0; else kStable. ends[0] low, ends[1] high.
//     Sensitivity sensitivity_rule(const Counts& main, const Counts& base,
//                                  const std::optional<std::array<Counts, 2>>& ends,
//                                  double target_pass_rate);
//
//     enum class End { kLow, kHigh };
//     // The profile override of a provenance entry at one end: range -> the bound itself (a
//     // double); scale -> every element of the profile's value times the factor (a list stays a
//     // list of the same length, a scalar stays a scalar). An entry with neither is an error.
//     common::Result<pipeline::KeyOverride> variant_override(
//         const std::filesystem::path& profile, const pipeline::ProvenanceEntry& entry, End end);
//
//     // The ablation groups, in this order.
//     inline constexpr std::array<const char*, 7> kAblationGroups = {
//         "sensor", "actuator", "friction", "model", "timing", "comm", "transmission"};
//     // One override per active key of `group` (pipeline::active_imperfection_keys, in its
//     // order) at its off value: 0 for a number (a list: every element 0), false for a bool,
//     // "link" for sensor.position_source. Keys whose off value the loader refuses are left as
//     // they are: sensor.qd_filter_hz (> 0; it goes inactive with sensor.qd_from_diff false)
//     // and transmission.ratio (> 0; a gear ratio has no off value). An unknown group is an
//     // error naming it.
//     common::Result<std::vector<pipeline::KeyOverride>> group_off_overrides(
//         const std::filesystem::path& profile, const std::string& group);
//
//     // One ablation row (ambiguity 17): pass count over the ablation seeds without the group,
//     // and the pass count of the same seeds in the main Monte Carlo.
//     struct Cause { std::string group; std::size_t passed_without = 0;
//                    std::size_t base_passed = 0; bool main = false; };
//     // main = true on the rows whose passed_without is the largest and > base_passed (ties: all
//     // of them); returns whether any row is main ("no single group explains the failure").
//     bool mark_main_causes(std::vector<Cause>& causes);
//
//     struct SuggestionFacts {
//       control::Algorithm algorithm = control::Algorithm::kComputedTorque;
//       double servo_rate_hz = 0.0;                       // comm.servo_rate_hz of the profile
//       std::map<std::string, std::size_t> hold_reasons;  // to_string(HoldReason) -> runs
//       std::vector<std::string> main_causes;             // groups of the rows with main = true
//       bool no_admissible_gain = false;                  // the tune step ran and chose nothing
//     };
//     // The fixed rule table of the brief, in its order, one English line each. "Mostly":
//     // state_timeout + overspeed holds are more than half of all holds.
//     std::vector<std::string> suggest(const SuggestionFacts& facts);
//
//   verify.hpp
//     struct SensitivityEnd { double value = 0.0; Counts counts; };  // value: bound or factor
//     Assumption gains: std::optional<Counts> base; std::optional<SensitivityEnd> low, high;
//       (set when the key ran) and sensitivity "not run" | "stable" | "may change" |
//       "not varied".
//     VerifyOptions gains: bool sensitivity = true; std::size_t sensitivity_seeds = 20;
//       std::size_t ablation_seeds = 20;
//       std::vector<std::string> sensitivity_keys;  // TEST HOOK: non-empty -> only these keys
//         // run (the others stay "not run"), and the sensitivity also runs in quick mode
//         // caused by runs < 100 (never with the --quick flag, never with sensitivity false).
//     VerifyResult: causes becomes std::vector<Cause> (filled only on FAIL), plus
//       std::map<std::string, std::size_t> hold_reasons (the main Monte Carlo's, on FAIL);
//       suggestions stays std::vector<std::string> (suggest(), only on FAIL).
//     Progress phases "sensitivity" and "ablation" (done / total runs of the phase).
//
//   compare.hpp
//     // "<model>__<algorithm>": the directory of a pair's report files.
//     std::string pair_name(const std::string& model, control::Algorithm algorithm);
//     struct ComparisonPair { std::size_t model = 0;   // index into the models
//                             control::Algorithm algorithm = ...; bool listed = false; };
//     // Cross product, models outer and algorithms inner, in the given orders; a pair the
//     // model does not list has listed false. No listed pair (empty lists included) -> error.
//     common::Result<std::vector<ComparisonPair>> comparison_pairs(
//         const std::vector<pipeline::HardwareModel>& models,
//         const std::vector<control::Algorithm>& algorithms);
//     struct ComparisonRow { std::string model; control::Algorithm algorithm = ...;
//                            std::optional<VerifyResult> result; };  // absent: not listed
//     // verify of every listed pair, one after the other (each with options.jobs), rows in the
//     // order of comparison_pairs.
//     common::Result<std::vector<ComparisonRow>> compare(
//         const std::vector<pipeline::HardwareModel>& models,
//         const std::vector<control::Algorithm>& algorithms, const pipeline::Task& task,
//         const VerifyOptions& options);
//     // comparison.json: {"disclaimer": kVerifyDisclaimer, ..., "rows": [{model, algorithm,
//     // verdict ("pass" | "fail" | "marginal" | "not listed"), pass_count, runs, wilson: [lo, hi],
//     // worst: {<metric>: value}, gain_used, gain_source}, ...]}, disclaimer the first key;
//     // comparison.md: the disclaimer on its first line, then one table row per pair.
//     std::string comparison_json(const std::vector<ComparisonRow>& rows);
//     std::string comparison_md(const std::vector<ComparisonRow>& rows);
//
//   cobot_bench verify: --hardware a.yaml,b.yaml and --algorithm x,y (lists), --no-sensitivity,
//     --sensitivity-seeds n, --ablation-seeds n (n >= 1, the messages name the flag).
namespace cobot::bench::analysis_test {

namespace fs = std::filesystem;
using verify_test::kRepo;

inline fs::path profile_path(const std::string& name) {
  return kRepo / "config/imperfections" / (name + ".yaml");
}

// An empty directory of its own for the running test (ctest runs tests in parallel processes).
inline fs::path test_dir() {
  const ::testing::TestInfo* info = ::testing::UnitTest::GetInstance()->current_test_info();
  std::string name = std::string(info->test_suite_name()) + "." + info->name();
  for (char& c : name) {
    c = c == '/' ? '_' : c;
  }
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "bench_verify_analysis" / name;
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir;
}

// The group of a dotted key: the text before its first dot.
inline std::string group_of(const std::string& key) {
  return key.substr(0, key.find('.'));
}

// ---- field-by-field comparison of two loaded profiles ----------------------------------------

using Profile = imperfect::ImperfectionConfig<model::kArmDof>;

namespace detail {

// Scalars, enums and Eigen vectors (Eigen's == is true when every element is equal).
template <typename A>
void diff(const std::string& key, const A& a, const A& b, std::vector<std::string>& out) {
  if (!(a == b)) {
    out.push_back(key);
  }
}

}  // namespace detail

// The dotted keys (provenance spelling) of the fields where a and b differ, exactly (seed too).
inline std::vector<std::string> differing_keys(const Profile& a, const Profile& b) {
  std::vector<std::string> out;
  using detail::diff;
#define COBOT_DIFF(group, field) diff(#group "." #field, a.group.field, b.group.field, out)
  COBOT_DIFF(sensor, encoder_bits);
  COBOT_DIFF(sensor, q_noise_std_rad);
  COBOT_DIFF(sensor, q_offset_std_rad);
  COBOT_DIFF(sensor, qd_from_diff);
  COBOT_DIFF(sensor, qd_filter_hz);
  COBOT_DIFF(sensor, qd_noise_std_rad_s);
  COBOT_DIFF(sensor, sensor_delay_cycles);
  COBOT_DIFF(sensor, position_source);
  COBOT_DIFF(actuator, torque_scale_std);
  COBOT_DIFF(actuator, torque_ripple_frac);
  COBOT_DIFF(actuator, ripple_periods_per_rev);
  COBOT_DIFF(actuator, torque_cmd_resolution_nm);
  COBOT_DIFF(actuator, motor_time_constant_s);
  COBOT_DIFF(actuator, command_delay_cycles);
  COBOT_DIFF(actuator, torque_scale_asym_frac);
  COBOT_DIFF(actuator, deadband_nm);
  COBOT_DIFF(actuator, torque_slew_nm_s);
  COBOT_DIFF(actuator, knee_speed_rad_s);
  COBOT_DIFF(actuator, zero_torque_speed_rad_s);
  COBOT_DIFF(actuator, bus.v_nom_v);
  COBOT_DIFF(actuator, bus.r_source_ohm);
  COBOT_DIFF(actuator, bus.v_min_v);
  COBOT_DIFF(friction, coulomb_nm);
  COBOT_DIFF(friction, viscous_nm_s_rad);
  COBOT_DIFF(model, link_mass_scale_std);
  COBOT_DIFF(model, com_shift_std_m);
  COBOT_DIFF(model, payload_kg);
  COBOT_DIFF(model, payload_offset_m);
  COBOT_DIFF(model, payload_radius_m);
  COBOT_DIFF(timing, period_jitter_std_us);
  COBOT_DIFF(timing, overrun_prob);
  COBOT_DIFF(timing, overrun_cycles);
  COBOT_DIFF(comm, servo_rate_hz);
  COBOT_DIFF(comm, packet_loss_prob);
  COBOT_DIFF(comm, latency_burst_prob);
  COBOT_DIFF(comm, burst_cycles);
  COBOT_DIFF(transmission, stiffness_k1_nm_rad);
  COBOT_DIFF(transmission, stiffness_k2_nm_rad);
  COBOT_DIFF(transmission, stiffness_k3_nm_rad);
  COBOT_DIFF(transmission, torque_t1_nm);
  COBOT_DIFF(transmission, torque_t2_nm);
  COBOT_DIFF(transmission, damping_ratio);
  COBOT_DIFF(transmission, error_amp_rad);
  COBOT_DIFF(transmission, lost_motion_rad);
  COBOT_DIFF(transmission, ratio);
#undef COBOT_DIFF
  diff("seed", a.seed, b.seed, out);
  return out;
}

// Every file under `dir`, recursively (none when it does not exist).
inline std::vector<fs::path> files_under(const fs::path& dir) {
  std::vector<fs::path> out;
  if (!fs::exists(dir)) {
    return out;
  }
  for (const fs::directory_entry& e : fs::recursive_directory_iterator(dir)) {
    if (e.is_regular_file()) {
      out.push_back(fs::relative(e.path(), dir));
    }
  }
  return out;
}

}  // namespace cobot::bench::analysis_test
