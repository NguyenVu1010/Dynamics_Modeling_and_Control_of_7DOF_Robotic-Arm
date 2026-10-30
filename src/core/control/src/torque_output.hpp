#pragma once

#include <cstddef>

#include "cobot/common/types.hpp"
#include "cobot/hal/joint_interface.hpp"
#include "cobot/model/model.hpp"

// Internal to `control`: shared output stage of the computed-torque and hold laws.
namespace cobot::control::detail {

using common::VecN;
using hal::JointCommand;
using model::Model;

// Element-wise clamp of a finite raw torque to [-tau_max, tau_max]; a non-finite raw torque
// gives the all-zero command (the loop turns that into a fault). No allocation.
template <int N>
JointCommand<N> clamp_or_zero(const Model<N>& model, const VecN<N>& raw) {
  JointCommand<N> cmd;
  if (!raw.allFinite()) {
    return cmd;
  }
  for (std::size_t i = 0; i < static_cast<std::size_t>(N); ++i) {
    const auto k = static_cast<Eigen::Index>(i);
    const double limit = model.joints[i].tau_max;
    double t = raw(k);
    if (t > limit) {
      t = limit;
    } else if (t < -limit) {
      t = -limit;
    }
    cmd.tau(k) = t;
  }
  return cmd;
}

// Joint damping as a vector (N m s / rad).
template <int N>
VecN<N> damping_of(const Model<N>& model) {
  VecN<N> d;
  for (std::size_t i = 0; i < static_cast<std::size_t>(N); ++i) {
    d(static_cast<Eigen::Index>(i)) = model.joints[i].damping;
  }
  return d;
}

}  // namespace cobot::control::detail
