#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>

#include "cobot/common/result.hpp"

// Engines of the cobot_bench modes (feature bench-e2e, design 11.2). The host side of the
// control loop: it owns the control thread and the CSV writer thread (ADR-008).
namespace cobot::bench {

// One run of the configured trajectory against SimPlant with the imperfection profile.
//   config_yaml, imperfections_yaml: as for pipeline::load_config / load_imperfections;
//   seed: overrides the profile's seed when set;
//   out_csv: Run CSV (docs/conventions.md); `<out_csv>.meta.yaml` is written next to it
//            (config and imperfections paths, seed, values drawn from the seed, git describe);
//   duration_s: simulated time in s; when empty, trajectory.duration_s + hold_s. The loop
//               runs round(duration_s * control_rate_hz) cycles and logs one row per cycle.
//   waypoints_yaml: waypoint file (pipeline::load_waypoints, feature bench-waypoints). When set,
//               the run forces command_source external, starts the loop (standby at the home
//               pose) and checks the list with pipeline::validate_path from the standby pose
//               and the measured q. A rejected list is an error with one line per issue,
//               "waypoint <i>: <issue> [joint <j>] <value> <op> <limit>" (i, j 0-based), and
//               writes neither the Run CSV nor its meta file. Otherwise the validated
//               trajectory is loaded; an empty duration_s means its duration + hold_s, and the
//               meta file gains waypoints_file, waypoint_node_times_s, waypoint_stop and
//               waypoint_positions_m (tip positions in base_link, one [x, y, z] per waypoint).
// Errors (bad paths, bad YAML, unwritable output) come back as a Status naming the file.
common::Status run(const std::filesystem::path& config_yaml,
                   const std::filesystem::path& imperfections_yaml,
                   std::optional<std::uint64_t> seed,
                   const std::filesystem::path& out_csv,
                   std::optional<double> duration_s,
                   const std::optional<std::filesystem::path>& waypoints_yaml);

// The configured trajectory: run(..., duration_s, std::nullopt).
common::Status run(const std::filesystem::path& config_yaml,
                   const std::filesystem::path& imperfections_yaml,
                   std::optional<std::uint64_t> seed,
                   const std::filesystem::path& out_csv,
                   std::optional<double> duration_s);

// Monte Carlo and sweep modes (feature sensitivity, bench/sweep.cpp), built on the run engine.
// Both write `<out_dir>/summary.csv` (Summary CSV, docs/conventions.md) with metrics computed
// in C++ by the definitions of cobot_eval.metrics (engine.hpp). `passed` judges the G3
// thresholds; scenario runners judge again from the metric columns.
//
// montecarlo: seeds seed0 .. seed0 + runs - 1 on one profile; rows have param = none, level = 0.
// With keep_runs, each run's Run CSV is written as `<out_dir>/run_<run_id>.csv`. runs == 0 is an
// error.
common::Status montecarlo(const std::filesystem::path& config_yaml,
                          const std::filesystem::path& imperfections_yaml,
                          std::size_t runs,
                          std::uint64_t seed0,
                          const std::filesystem::path& out_dir,
                          bool keep_runs,
                          std::optional<double> duration_s);

// sweep: one at a time (config/sweeps/*.yaml): for each param, level and seed, the base profile
// with that one key replaced (scalar broadcast to every joint). Every param and level is
// validated before the first run; an unknown param is an error naming it.
common::Status sweep(const std::filesystem::path& config_yaml,
                     const std::filesystem::path& sweep_yaml,
                     const std::filesystem::path& out_dir,
                     std::optional<double> duration_s);

// Wall time of ControlLoop::step() over `steps` cycles of an ideal run (no imperfections,
// deterministic forced off), in microseconds.
struct TimingStats {
  double p50_us = 0.0;
  double p95_us = 0.0;
  double max_us = 0.0;
};

common::Result<TimingStats> timing(const std::filesystem::path& config_yaml, std::int64_t steps);

}  // namespace cobot::bench
