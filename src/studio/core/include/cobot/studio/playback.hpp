#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "cobot/common/types.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/log_record.hpp"
#include "cobot/studio/robot_scene.hpp"

#include "run_data.hpp"

// Playback helpers of the 3D view (feature studio-3d, cobot_studio design 6.9). No Qt.
namespace cobot::studio {

// Last row with t <= t_s (compared in integer nanoseconds), clamped to [0, rows - 1]; 0 when
// the run is empty.
std::size_t row_at(const bench::RunData& run, double t_s);

// Joint angles to draw: q_true when every element is finite, else q.
const common::VecN<model::kArmDof>& display_q(const pipeline::LogRecord<model::kArmDof>& row);

struct Traces {
  std::vector<common::Vec3> tip, desired;  // one per row, in base_link
  std::vector<common::Vec3> goals;  // waypoint positions, else the last row's desired position
  std::optional<double> hold_t_s;   // summary.hold_time_s when the run held
};

// tip from display_q (RobotScene::tip_pose), desired from T_b_d.
Traces make_traces(const RobotScene& scene, const bench::RunData& run);

}  // namespace cobot::studio
