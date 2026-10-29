#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "cobot/common/result.hpp"
#include "cobot/common/types.hpp"
#include "cobot/control/friction_compensation.hpp"
#include "cobot/model/model.hpp"
#include "cobot/planning/clik.hpp"
#include "cobot/planning/waypoint_trajectory.hpp"

// Offline check of a waypoint list before it is run (feature `path-validation`, design
// docs/design/2026-10-29-waypoint-gui-design.md section 6.2). Lives in pipeline because it needs
// planning (CLIK, WaypointTrajectory), dynamics (RNEA) and control (friction feed-forward).
// Not on the hot path: it allocates.
namespace cobot::pipeline {

enum class PathIssue { kUnreachable, kJointLimit, kJointVelocity, kTorque };
// "unreachable", "joint_limit", "joint_velocity", "torque"
const char* to_string(PathIssue issue);

struct WaypointIssue {
  std::size_t waypoint = 0;  // index of the waypoint whose segment (or node) shows the issue
  PathIssue issue = PathIssue::kUnreachable;
  int joint = -1;  // 0-based joint for joint_* and torque, -1 otherwise
  // Worst value seen, against `limit`:
  //   unreachable    position error (m) vs reach_pos_m, or rotation error (rad) vs reach_rot_rad
  //   joint_limit    distance of q_ref to the nearer URDF limit (rad) vs joint_margin_rad
  //   joint_velocity |qd_ref| (rad/s) vs qd_max
  //   torque         |tau| / tau_max vs torque_ratio
  double value = 0.0;
  double limit = 0.0;
};

struct ValidationLimits {  // all > 0 and finite
  double reach_pos_m = 1e-3;
  double reach_rot_rad = 0.5 * 3.14159265358979323846 / 180.0;
  double joint_margin_rad = 2.0 * 3.14159265358979323846 / 180.0;  // distance to a URDF limit
  double torque_ratio = 0.9;                                       // of model tau_max
  double max_stretch = 3.0;        // factor on an auto segment's duration
  double dt_s = 1e-3;              // CLIK step for the offline run
  double path_sample_dt_s = 0.01;  // spacing of the returned path samples
};

template <int N>
struct PathValidation {
  bool ok = false;
  std::vector<WaypointIssue> issues;        // empty when ok
  std::vector<common::Pose> path;           // desired tip poses every path_sample_dt_s and at T
  std::vector<double> segment_durations_s;  // after stretching, one per waypoint
  std::optional<planning::WaypointTrajectory> trajectory;  // set when ok: exactly what Run executes
};

// Runs CLIK offline from (q_start, zero velocity) along
// WaypointTrajectory::create(T_b_start, waypoints, cartesian). T_b_start is the loop's reference
// pose (standby), passed explicitly, not recomputed from q_start.
// Checks on every CLIK step (q_ref, qd_ref, qdd_ref):
//   joint_limit     q_ref within joint_margin_rad of a finite URDF limit
//   joint_velocity  |qd_ref| > qd_max
//   torque          |RNEA(q_ref, qd_ref, qdd_ref) + friction(qd_ref)| > torque_ratio * tau_max
//   unreachable     at the step nearest each node time, task error > reach_pos_m or reach_rot_rad
// An issue during segment i or at node i belongs to waypoint i; one issue per (waypoint, kind),
// the worst kept. A waypoint with an auto duration and only joint_velocity/torque issues gets its
// duration multiplied by 1.25 and the run is repeated, while the factor stays <= max_stretch.
// A stretched segment is passed to create() as a fixed duration (so the returned trajectory
// reports user_duration() == true for it); the other auto segments stay auto and keep the
// Cartesian limits. If no auto duration fits next to a stretch, the last run is reported.
// Error (Result) for invalid input: create() fails (its message), non-finite q_start, any
// ValidationLimits value <= 0 or non-finite (message names the field). Otherwise a value, ok or
// not.
template <int N>
common::Result<PathValidation<N>> validate_path(
    const model::Model<N>& model,
    const planning::ClikConfig& clik,
    const control::FrictionCompensationConfig<N>& friction,
    const common::VecN<N>& q_start,
    const common::Pose& T_b_start,
    const std::vector<planning::Waypoint>& waypoints,
    const planning::CartesianLimits& cartesian,
    const ValidationLimits& limits);

}  // namespace cobot::pipeline
