#pragma once

#include "cobot/common/types.hpp"
#include "cobot/model/model.hpp"

// Internal simulator used by tests and bench (Gazebo is used for the demo).
// Implemented in feature `sim-plant`: RK4 on forward_dynamics with hard joint stops
// (position clamped to [q_min, q_max], velocity into the stop set to zero), unmodelled
// friction and a first-order motor lag from PlantEffects.
namespace cobot::dynamics {

using common::VecN;
using model::Model;

template <int N>
struct PlantEffects {
  VecN<N> coulomb_nm = VecN<N>::Zero();             // N m, sign(qd) friction
  VecN<N> viscous_nm_s_rad = VecN<N>::Zero();       // N m s / rad, on top of URDF damping
  VecN<N> motor_time_constant_s = VecN<N>::Zero();  // 0 = ideal torque source
};

template <int N>
class SimPlant {
 public:
  SimPlant(const Model<N>& model, const VecN<N>& q0, const PlantEffects<N>& effects = {});

  // Advance by dt_s with commanded torque tau held constant over the step.
  void step(const VecN<N>& tau_cmd, double dt_s);

  const VecN<N>& q() const { return q_; }
  const VecN<N>& qd() const { return qd_; }
  const VecN<N>& tau_applied() const { return tau_applied_; }  // after motor lag
  double time_s() const { return time_s_; }

 private:
  Model<N> model_;
  PlantEffects<N> effects_;
  VecN<N> q_;
  VecN<N> qd_ = VecN<N>::Zero();
  VecN<N> tau_applied_ = VecN<N>::Zero();
  double time_s_ = 0.0;
};

}  // namespace cobot::dynamics
