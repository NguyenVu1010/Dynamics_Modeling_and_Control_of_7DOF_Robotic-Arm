#pragma once

#include "cobot/control/algorithm.hpp"
#include "cobot/control/computed_torque.hpp"
#include "cobot/control/drive_position.hpp"
#include "cobot/control/pd_gravity.hpp"

// The tracking law of ControlLoop, selected at construction (ADR-023, feature
// controller-registry). Closed registry, no virtual dispatch: every registered law is held by
// value and constructed (a few model copies, off the hot path); compute() is a switch on the
// enum that returns that law's command unchanged. Adding a law = an entry in kAlgorithms
// (algorithm.hpp), a member here and a case in that switch. Not polymorphic, no allocation, no
// exception. The hold of the torque laws is not part of it: ControlLoop keeps its HoldController
// with the computed-torque gains for them (bit-identity of the hold path); drive_position holds
// through the drive (hold()).
namespace cobot::control {

template <int N>
struct ControllerConfig {
  Algorithm algorithm = Algorithm::kComputedTorque;
  ComputedTorqueGains<N> computed_torque;
  PdGravityGains<N> pd_gravity;
  VecN<N> q_inertia = VecN<N>::Zero();  // where J of pd_gravity is taken: Config::home_q_rad
  DriveFeedForward drive_feedforward = DriveFeedForward::kInverseDynamics;  // drive_position
};

template <int N>
class TrackingController {
 public:
  TrackingController(const Model<N>& model, const ControllerConfig<N>& config);
  // The selected law's command, bit for bit; all zero for a value outside the table.
  JointCommand<N> compute(const JointState<N>& s, const JointReference<N>& ref) const noexcept;
  // The hold through the drive (DrivePosition::hold); meaningful for kDrivePosition only.
  JointCommand<N> hold(const VecN<N>& q_hold) const noexcept {
    return drive_position_.hold(q_hold);
  }
  Algorithm algorithm() const noexcept { return algorithm_; }

 private:
  Algorithm algorithm_;
  ComputedTorque<N> computed_torque_;
  PdGravity<N> pd_gravity_;
  DrivePosition<N> drive_position_;
};

}  // namespace cobot::control
