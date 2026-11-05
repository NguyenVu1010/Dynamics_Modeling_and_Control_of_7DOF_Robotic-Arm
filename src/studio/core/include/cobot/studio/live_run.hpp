#pragma once

#include <cstdint>
#include <filesystem>
#include <vector>

#include "cobot/bridge/wire.hpp"
#include "cobot/common/result.hpp"
#include "cobot/common/types.hpp"
#include "cobot/pipeline/log_record.hpp"
#include "cobot/studio/run_store.hpp"
#include "cobot/studio/studio_config.hpp"

#include "run_data.hpp"

// A run driven through the studio bridge (feature studio-control, cobot_studio design 5.2, plan
// ambiguity 15): the Recorder segment the host exported, as a Tracking-tab entry.
namespace cobot::studio {

struct LiveRunInfo {
  pipeline::CommandMode cmd = pipeline::CommandMode::kMoveJoint;
  std::uint64_t command_id = 0;             // 0 for jogs
  std::int64_t t_start_ns = 0;              // response t_start_ns, or the first jog row
  std::int64_t t_end_ns = 0;                // first standby (or holding) row after tracking
  double settle_window_s = 1.0;             // design 5.2, external / Gazebo runs
  std::vector<double> segment_durations_s;  // run_waypoints / move_tcp (node times = sums)
  std::vector<bool> waypoint_stop;
  std::vector<common::Vec3> waypoint_positions_m;  // tip positions
  bool truncated = false;
  StudioConfig config;  // what the studio last applied, else the session's files
};

// RunData{records, meta, summary}: meta.goal_time_s = t_end - first row t, window_end_s = goal +
// settle_window_s, tau_max / rated from the session, waypoint fields from info; summary =
// bench::summarize(rows, tau_max, rated, t_end + settle window); errors = make_error_trace; kind
// kGazebo; label "live <cmd> #<number>" plus " (truncated)" when info.truncated (then also note).
common::Result<RunEntry> make_live_entry(const fs::path& segment_csv,
                                         const LiveRunInfo& info,
                                         const bridge::SessionInfo<bench::kN>& session,
                                         int number);

}  // namespace cobot::studio
