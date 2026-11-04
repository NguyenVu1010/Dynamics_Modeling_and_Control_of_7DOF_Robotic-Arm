#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "cobot/common/result.hpp"
#include "cobot/common/types.hpp"
#include "cobot/imperfect/config.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/log_record.hpp"

#include "engine.hpp"

// In-memory run API of the bench engine (feature studio-core, ADR-021): one run's records, the
// values of its meta file and its RunSummary, without any file. run_profile is built on it, so a
// studio run gives the numbers and (bench::write_run_files) the bytes of `cobot_bench run`.
namespace cobot::bench {

inline constexpr int kN = model::kArmDof;
using Vec = common::VecN<kN>;

// What `<run.csv>.meta.yaml` holds, plus the times the studio metrics need.
struct RunMeta {
  std::filesystem::path config_yaml, imperfections_label;
  std::uint64_t seed = 0;
  Vec q_offset_rad = Vec::Zero();        // drawn from the seed (sensor.q_offset_std_rad)
  Vec torque_scale = Vec::Ones();        // drawn from the seed (actuator.torque_scale_std)
  Vec plant_link_mass_kg = Vec::Zero();  // perturbed plant model
  Vec tau_max = Vec::Zero();             // controller model, peak torque P (N m)
  Vec rated = Vec::Zero();               // cobot.limits.rated_torque_nm (N m)
  double period_s = 0.0;                 // control period
  // From the first row: the reference reaches the final goal (trajectory.duration_s, or the
  // waypoint trajectory duration).
  double goal_time_s = 0.0;
  double window_end_s = 0.0;  // goal_time_s + hold_s (the RunSummary evaluation time)
  std::vector<double> waypoint_node_times_s;  // waypoint runs only
  std::vector<bool> waypoint_stop;
  std::vector<common::Vec3> waypoint_positions_m;  // tip positions in base_link
  std::optional<std::filesystem::path> waypoints_yaml;
  std::string git;  // git describe of the build, empty without one
};

struct RunData {
  std::vector<pipeline::LogRecord<kN>> records;  // one per cycle
  RunMeta meta;
  RunSummary summary;
};

struct RunRequest {
  std::filesystem::path config_yaml;                // materialised or repo file
  imperfect::ImperfectionConfig<kN> imperfections;  // seed already set
  std::filesystem::path label;                      // names the profile in errors and the meta
  std::optional<double> duration_s;                 // empty: goal time + hold_s
  std::optional<std::filesystem::path> waypoints_yaml;
  std::optional<ConfigOverride> config_override;  // sweep param (single key)
};

// One run as run_profile does it, keeping the records; errors as run_profile's.
common::Result<RunData> run_in_memory(const RunRequest& request);

}  // namespace cobot::bench
