#pragma once

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
// Errors (bad paths, bad YAML, unwritable output) come back as a Status naming the file.
common::Status run(const std::filesystem::path& config_yaml,
                   const std::filesystem::path& imperfections_yaml,
                   std::optional<std::uint64_t> seed,
                   const std::filesystem::path& out_csv,
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
