#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "cobot/common/result.hpp"
#include "cobot/common/types.hpp"
#include "cobot/control/algorithm.hpp"
#include "cobot/pipeline/hardware_model.hpp"
#include "cobot/pipeline/overrides.hpp"

// Task file config/tasks/<task>.yaml (feature hardware-catalog, ADR-023, format in
// docs/conventions.md "Task file"): what to move, what is carried and what counts as good enough.
namespace cobot::pipeline {

struct TaskTrajectory {
  common::Vec3 goal_position_m = common::Vec3::Zero();  // tip position in b
  common::Vec3 goal_rpy_deg = common::Vec3::Zero();     // as in the file, written back unchanged
  double duration_s = 10.0;                             // > 0
  double hold_s = 2.0;                                  // >= 0
};

// Optional in the file: absent is 0 kg, known. kg required in the section, the rest optional.
struct TaskPayload {
  double kg = 0.0;                               // >= 0
  common::Vec3 offset_m = common::Vec3::Zero();  // in the tip frame, from the tip origin
  double radius_m = 0.0;                         // >= 0, solid sphere
  bool known = true;                             // the controller is told the payload (C9)
};

// Every value > 0.
struct TaskRequirements {
  double final_pos_err_m = 0.0;
  double final_rot_err_deg = 0.0;
  double rms_joint_err_rad = 0.0;
  // Settle time: from the first row until the TCP position and rotation errors enter and stay
  // within the final thresholds (ADR-023, ambiguity 11). Stored here; verify measures it.
  double cycle_time_s = 0.0;
  double max_hold_rate = 0.0;      // in (0, 1]
  double target_pass_rate = 0.95;  // optional, in (0, 1]
  double margin_frac = 0.2;        // optional, in (0, 1)
};

struct Task {
  std::string name;
  std::optional<TaskTrajectory> trajectory;        // exactly one of these two
  std::optional<std::filesystem::path> waypoints;  // resolved relative to the task file
  TaskPayload payload;
  TaskRequirements requirements;
};

// Errors name the file and the full key, all of them at once. The waypoint file must exist; its
// parsing stays with the run, since load_waypoints needs the base link and the TCP.
Result<Task> load_task(const std::filesystem::path& yaml);

// What a model, an algorithm and a task change in a run. config is the whole list for
// load_config_with(model.controller, ...): model_overrides of the algorithm, the trajectory keys
// (when the task has a trajectory) and config.payload.* (when the payload is known).
// imperfections is model.payload_{kg,offset_m,radius_m} on model.imperfections: the plant always
// carries the payload.
struct RunOverrides {
  std::vector<KeyOverride> config;
  std::vector<KeyOverride> imperfections;
  std::optional<std::filesystem::path> waypoints;
};

RunOverrides run_overrides(const HardwareModel& model,
                           control::Algorithm algorithm,
                           const Task& task);

}  // namespace cobot::pipeline
