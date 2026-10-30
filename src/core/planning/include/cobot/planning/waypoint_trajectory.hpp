#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include "cobot/common/result.hpp"
#include "cobot/common/types.hpp"
#include "cobot/planning/cartesian_trajectory.hpp"

// Multi-point Cartesian trajectory (feature `waypoint-traj`, design
// docs/design/2026-10-29-waypoint-gui-design.md section 6.1, derivation
// docs/derivations/waypoint_trajectory.md). Each segment is a quintic Hermite curve on the
// position and on the rotation vector relative to the segment start; node velocities come from
// the neighbouring points (zero at the start, at stop points and at the end), node accelerations
// of the curve parameters are zero. Segments without a user duration get the shortest duration
// (at least 0.1 s) whose sampled twist and twist derivative stay inside the Cartesian limits.
namespace cobot::planning {

inline constexpr std::size_t kMaxWaypoints = 32;

struct Waypoint {
  common::Pose T_b_goal;    // tip pose in b (rotation must be orthonormal)
  double duration_s = 0.0;  // time from the previous point; <= 0 -> computed from the limits
  bool stop = false;        // come to rest here (the last waypoint always stops)
};

struct CartesianLimits {  // all > 0
  double v_max_m_s = 0.25;
  double a_max_m_s2 = 0.5;
  double w_max_rad_s = 0.8;
  double alpha_max_rad_s2 = 1.6;
};

class WaypointTrajectory {
 public:
  // Errors name the offending waypoint index ("waypoints[2].duration_s: not finite"):
  // empty list, more than kMaxWaypoints, non-finite position/rotation/duration, rotation not
  // orthonormal (|R^T R - I|_max > 1e-6), any limit <= 0 or non-finite.
  static common::Result<WaypointTrajectory> create(const common::Pose& T_b_start,
                                                   const std::vector<Waypoint>& waypoints,
                                                   const CartesianLimits& limits);
  // No allocation; t < 0 -> start at rest, t >= T -> last pose at rest.
  CartesianSample sample(double t_s) const;
  double duration_s() const;                       // total
  std::size_t size() const;                        // number of waypoints
  double node_time_s(std::size_t i) const;         // time waypoint i is reached (i < size())
  double segment_duration_s(std::size_t i) const;  // duration of the segment ending at waypoint i
  bool user_duration(std::size_t i) const;         // true when waypoint i had duration_s > 0
  bool stops_at(std::size_t i) const;              // stop flag, true for the last waypoint
  // Index of the waypoint being approached; size() after the end.
  std::size_t segment_at(double t_s) const;
  const common::Pose& start_pose() const;
  const Waypoint& waypoint(std::size_t i) const;  // with duration_s replaced by the used duration

 private:
  // Segment i runs from the previous node (the start for i = 0) to waypoint i.
  struct Segment {
    common::Vec3 dp = common::Vec3::Zero();   // p_i - p_{i-1}, m, in b
    common::Vec3 phi = common::Vec3::Zero();  // so3_log(R_{i-1}^T R_i), rad, segment start frame
    common::Vec3 v0 = common::Vec3::Zero();   // linear velocity at the segment start, m/s, in b
    common::Vec3 v1 = common::Vec3::Zero();   // linear velocity at the segment end, m/s, in b
    common::Vec3 u0 = common::Vec3::Zero();   // d(rotation vector)/dt at the start, rad/s
    common::Vec3 u1 = common::Vec3::Zero();   // d(rotation vector)/dt at the end, rad/s
  };

  WaypointTrajectory() = default;
  const common::Pose& node_pose(std::size_t i) const;  // the start for i = 0, else waypoint i - 1
  CartesianSample sample_segment(std::size_t i, double tau) const;
  double peak_ratio(std::size_t i, const CartesianLimits& limits) const;
  void set_node_rates();

  // Fixed-size storage only: copying never allocates.
  common::Pose start_;
  std::array<Waypoint, kMaxWaypoints> waypoints_{};
  std::array<Segment, kMaxWaypoints> segments_{};
  std::array<double, kMaxWaypoints> node_time_s_{};
  std::array<bool, kMaxWaypoints> user_duration_{};
  std::size_t count_ = 0;
};

}  // namespace cobot::planning
