#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>

#include "cobot/common/result.hpp"
#include "cobot/imperfect/config.hpp"
#include "cobot/model/model.hpp"

// Internal run engine shared by the run, montecarlo and sweep modes (bench/run.cpp).
namespace cobot::bench {

// Summary CSV metrics of one run (docs/conventions.md). Same definitions as cobot_eval.metrics,
// duplicated here so that Monte Carlo and sweep runs need no Run CSV on disk; keep both in step.
struct RunSummary {
  double final_pos_err_m = 0.0;    // |p_d - p| at the last row with t <= duration_s + hold_s
  double final_rot_err_rad = 0.0;  // angle of R_d R^T at that row
  double rms_joint_err_rad = 0.0;  // max over joints of RMS(qref - q) in tracking/finished rows
  double peak_tau_ratio = 0.0;     // max |tau| / tau_max
  double sat_time_s = 0.0;         // time with any |tau| >= 0.999 tau_max
  bool held = false;               // any row in holding
};

// Waypoint list of a run (feature bench-waypoints): the loop runs in external command mode and
// executes the list once pipeline::validate_path accepts it from the standby pose.
struct WaypointInput {
  std::filesystem::path yaml;
  bool path_invalid = false;  // set when validate_path rejects the list (cobot_bench exit 1)
};

// One run with an in-memory imperfection profile (its seed is used as is). `label` names the
// profile in errors and in the meta file. An empty out_csv writes no Run CSV and no meta file.
// With `waypoints`, a rejected list is an error with one line per issue and writes no file.
common::Result<RunSummary> run_profile(
    const std::filesystem::path& config_yaml,
    const imperfect::ImperfectionConfig<model::kArmDof>& imperfections,
    const std::filesystem::path& label,
    const std::filesystem::path& out_csv,
    std::optional<double> duration_s,
    WaypointInput* waypoints = nullptr);

// bench::run, with `path_invalid` telling a rejected waypoint list from bad input.
common::Status run_checked(const std::filesystem::path& config_yaml,
                           const std::filesystem::path& imperfections_yaml,
                           std::optional<std::uint64_t> seed,
                           const std::filesystem::path& out_csv,
                           std::optional<double> duration_s,
                           const std::optional<std::filesystem::path>& waypoints_yaml,
                           bool& path_invalid);

}  // namespace cobot::bench
