#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "cobot/common/result.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/planning/waypoint_trajectory.hpp"

// Waypoint file (feature `loop-external`, design docs/design/2026-10-29-waypoint-gui-design.md
// 6.6, format in docs/conventions.md "Waypoint file"), shared by the GUI, the bench and E2E:
//   frame: base_link
//   waypoints:
//     - {position_m: [x, y, z], rpy_deg: [roll, pitch, yaw], duration_s: 0, stop: false}
// duration_s (>= 0, 0 = from the Cartesian limits) and stop are optional. The file holds TCP
// poses; the code works with tip (link_7) poses: T_b_tip = T_b_tcp * T_tip_tcp^-1 on load and
// T_b_tcp = T_b_tip * T_tip_tcp on save. Not on the hot path: both allocate.
namespace cobot::pipeline {

// Errors name the file and the key (`waypoints[1].position_m: expected 3 values, got 2`); the
// frame must equal `base_link`.
common::Result<std::vector<planning::Waypoint>> load_waypoints(const std::filesystem::path& yaml,
                                                               const std::string& base_link,
                                                               const TcpOffset& tcp);

// Writes every number with %.17g, so load_waypoints(save_waypoints(x)) == x within 1e-9.
common::Status save_waypoints(const std::filesystem::path& yaml,
                              const std::string& base_link,
                              const std::vector<planning::Waypoint>& tip_waypoints,
                              const TcpOffset& tcp);

}  // namespace cobot::pipeline
