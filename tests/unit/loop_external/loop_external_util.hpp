#pragma once

#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/lie.hpp"
#include "cobot/common/types.hpp"
#include "cobot/kinematics/kinematics.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/planning/waypoint_trajectory.hpp"

#include "control_loop_test_util.hpp"

// Helpers for the external command mode tests (feature `loop-external`): the shared rig of
// tests/component/control_loop with command_source = external, poses and trajectories.
namespace cobot::pipeline::test {

using planning::CartesianLimits;
using planning::Waypoint;
using planning::WaypointTrajectory;

constexpr double kDt = 1e-3;  // 1 / control_rate_hz (1000 Hz)

// tests/e2e/configs/ideal_world.yaml with command_source = external.
inline Config<kN> external_config(const Setup& setup) {
  Config<kN> cfg = setup.config;
  cfg.command_source = CommandSource::kExternal;
  return cfg;
}

inline Pose fk_pose(const model::Arm7& m, const Vec& q) {
  kinematics::FkState<kN> fk;
  kinematics::forward_kinematics(m, q, fk);
  return fk.T_b_e;
}

inline double pos_dist(const Pose& a, const Pose& b) {
  return (a.p - b.p).norm();
}

// Angle of R_a R_b^T, rad.
inline double rot_dist(const Pose& a, const Pose& b) {
  const Mat3 r = a.R * b.R.transpose();
  return common::so3_log(r).norm();
}

// Translational and angular velocity of the reference: |J(q_ref) qd_ref| (null-space motion of
// the redundant arm does not count).
inline double task_speed(const model::Arm7& m, const Vec& q_ref, const Vec& qd_ref) {
  kinematics::FkState<kN> fk;
  kinematics::forward_kinematics(m, q_ref, fk);
  common::Mat6N<kN> J;
  kinematics::geometric_jacobian(fk, J);
  const common::Vec6 v = J * qd_ref;
  return v.norm();
}

inline std::optional<WaypointTrajectory> make_trajectory(const Pose& start,
                                                         const std::vector<Waypoint>& waypoints) {
  const auto r = WaypointTrajectory::create(start, waypoints, CartesianLimits{});
  if (!r) {
    ADD_FAILURE() << "WaypointTrajectory::create: " << r.reason();
    return std::nullopt;
  }
  return r.value();
}

inline Waypoint waypoint(const Pose& goal, double duration_s, bool stop) {
  Waypoint w;
  w.T_b_goal = goal;
  w.duration_s = duration_s;
  w.stop = stop;
  return w;
}

// A short move from `from`: 5 cm back in x and 5 cm up, same orientation (|dp| = 0.0707 m).
inline Pose short_goal(const Pose& from) {
  return Pose{from.R, from.p + Vec3(-0.05, 0.0, 0.05)};
}

}  // namespace cobot::pipeline::test
