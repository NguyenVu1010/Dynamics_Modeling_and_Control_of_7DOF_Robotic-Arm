#pragma once

#include "cobot/common/types.hpp"

// Point-to-point Cartesian trajectory (feature `planning-traj`):
//   s(t) quintic on [0, T] with s(0)=0, s(T)=1 and zero first/second derivatives at both ends;
//   p(t) = p0 + s (pT - p0);  R(t) = R0 * so3_exp(s * so3_log(R0^T RT)).
// After T the goal is held with zero velocity and acceleration; before 0 the start is held.
namespace cobot::planning {

using common::Pose;
using common::Vec6;

struct CartesianSample {
  Pose T_b_d;                   // desired tip pose in b
  Vec6 xd_d = Vec6::Zero();     // desired twist [v; w] in b
  Vec6 xdd_d = Vec6::Zero();    // desired twist derivative in b
};

class CartesianTrajectory {
 public:
  // duration_s must be > 0 (checked by the caller at configuration time).
  CartesianTrajectory(const Pose& T_b_start, const Pose& T_b_goal, double duration_s);

  CartesianSample sample(double t_s) const;
  double duration_s() const { return duration_s_; }

 private:
  Pose T_b_start_;
  Pose T_b_goal_;
  double duration_s_;
};

}  // namespace cobot::planning
