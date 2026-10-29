#pragma once

#include "cobot/common/types.hpp"
#include "cobot/model/model.hpp"

// Internal simulator used by tests and bench (Gazebo is used for the demo).
// Implemented in feature `sim-plant`: RK4 on forward_dynamics with hard joint stops
// (position clamped to [q_min, q_max], velocity into the stop set to zero), unmodelled
// friction (Coulomb as a stick-slip projection after each step) and a first-order motor lag
// from PlantEffects.
namespace cobot::dynamics {

using common::VecN;
using model::Model;

template <int N>
struct PlantEffects {
  // N m, stick-slip: applied after each RK4 step as a velocity projection (true stick below
  // breakaway, coulomb * sign(qd) while sliding), see sim_plant.cpp
  VecN<N> coulomb_nm = VecN<N>::Zero();
  // N m s / rad, on top of URDF damping; on a joint with coulomb > 0 both are applied in the
  // stick-slip projection (implicitly), otherwise inside RK4
  VecN<N> viscous_nm_s_rad = VecN<N>::Zero();
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
  // Friction split (sim_plant.cpp): viscous torque per joint inside RK4 (-URDF damping on a
  // joint with Coulomb friction, cancelling it there) and in the stick-slip projection.
  VecN<N> viscous_rk4_ = VecN<N>::Zero();
  VecN<N> viscous_projection_ = VecN<N>::Zero();
  bool has_coulomb_ = false;  // any coulomb_nm > 0: the projection needs M(q) each step
  // Joint friction impulses of the last projection (N m s), warm start of the next one.
  VecN<N> friction_impulse_ = VecN<N>::Zero();
};

}  // namespace cobot::dynamics
