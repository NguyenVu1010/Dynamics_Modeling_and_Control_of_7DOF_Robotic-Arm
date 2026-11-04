#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "cobot/common/result.hpp"
#include "cobot/pipeline/overrides.hpp"
#include "cobot/studio/profile_catalog.hpp"

// The studio configuration (feature studio-core, cobot_studio design 6.6, ADR-021): a base pair,
// typed key overrides and the studio's own settings, saved as the studio file
// (docs/conventions.md "Studio file").
namespace cobot::studio {

inline constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

// Tracking tolerances of the metrics (design 5.2 defaults).
struct Tolerance {
  double goal_pos_m = 1e-3;
  double goal_rot_rad = 0.5 * kDegToRad;
  double path_pos_m = 2e-3;
  double path_rot_rad = 1.0 * kDegToRad;
};

struct StudioConfig {
  BasePair base;
  std::vector<pipeline::KeyOverride> overrides;  // at most one per key, insertion order kept
  std::optional<std::uint64_t> seed;             // empty: the profile's seed
  Tolerance tolerance;
  double speed_pct = 10.0;  // saved; used from phase B (Q8), [1, 100]
  std::optional<fs::path> waypoints_yaml;
  std::optional<double> duration_s;  // empty: goal time + hold_s
};

// Root `studio:`, version 1; paths relative to the file; tolerance in mm and degrees. Unknown
// keys are errors naming them.
common::Result<StudioConfig> load_studio_file(const fs::path& yaml);
common::Status save_studio_file(const StudioConfig& config, const fs::path& yaml);

// Empty = runnable; one line per issue naming the key: catalog membership and input limits,
// speed_pct in [1, 100], then the loaders' errors on the overridden pair.
std::vector<std::string> validate(const StudioConfig& config);

// The pair the run uses, written under `dir` as config.yaml and imperfections.yaml (overrides
// applied, the seed written into the profile when set): cobot_bench reruns it as is.
struct Materialized {
  fs::path config_yaml, imperfections_yaml;
};
common::Result<Materialized> materialize(const StudioConfig& config, const fs::path& dir);

// The overrides in order, "omega=50, sensor.q_noise_std_rad=0.0001" (config keys by their last
// name); empty without overrides.
std::string describe_diff(const StudioConfig& config);

}  // namespace cobot::studio
