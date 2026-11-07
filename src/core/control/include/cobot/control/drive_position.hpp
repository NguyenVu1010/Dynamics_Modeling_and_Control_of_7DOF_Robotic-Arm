#pragma once

#include "cobot/common/types.hpp"
#include "cobot/control/algorithm.hpp"
#include "cobot/hal/joint_interface.hpp"
#include "cobot/model/model.hpp"
#include "cobot/planning/clik.hpp"

// Host side of the position loop in the joint drive (algorithm drive_position, feature drive-loop,
// ADR-023). The host sends a set point, a velocity feed-forward and a torque feed-forward; the
// drive (pipeline::SimDrive in the bench) closes the loop on its own motor encoder at
// cobot.drive.rate_hz with kp_j = J_j omega_j^2 (N m/rad) and kd_j = 2 zeta J_j omega_j
// (N m s/rad), J = diag(M(home_q)) of the controller model (as pd_gravity). Neither function reads
// the measured state. The torque feed-forward is clamped element-wise to tau_max
// (detail::clamp_or_zero). No allocation, no exception.
namespace cobot::control {

using common::VecN;
using hal::JointCommand;
using hal::JointState;
using model::Model;
using planning::JointReference;

template <int N>
class DrivePosition {
 public:
  DrivePosition(const Model<N>& model, DriveFeedForward ff);
  // mode kPosition, q = ref.q, qd = ref.qd, tau = clamp(ff): kNone 0, kGravity G(ref.q),
  // kInverseDynamics rnea(ref.q, ref.qd, ref.qdd) + D ref.qd. A non-finite ref.q, qd or qdd (or
  // feed-forward) gives the all-zero torque command.
  JointCommand<N> compute(const JointState<N>& s, const JointReference<N>& ref) const noexcept;
  // The hold through the drive: kPosition, q = q_hold, qd = 0, tau = 0 (kNone) else
  // clamp(G(q_hold)). A non-finite q_hold gives the all-zero torque command.
  JointCommand<N> hold(const VecN<N>& q_hold) const noexcept;

 private:
  Model<N> model_;
  DriveFeedForward ff_;
};

}  // namespace cobot::control
