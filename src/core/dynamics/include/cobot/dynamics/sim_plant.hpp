#pragma once

#include "cobot/common/types.hpp"
#include "cobot/model/model.hpp"

// Internal simulator used by tests and bench (Gazebo is used for the demo).
// Implemented in feature `sim-plant`: RK4 on forward_dynamics with hard joint stops
// (position clamped to [q_min, q_max], velocity into the stop set to zero), unmodelled
// friction (Coulomb as a stick-slip projection after each step) and a first-order motor lag
// from PlantEffects. The applied torque is capped at +-model.joints[i].tau_max after the motor
// lag (feature joint-limits): the plant model is the URDF one, whose effort is the drive's
// momentary torque M; the controller clamps lower, at the peak torque P of its own model.
// Elastic joints (feature transmission, realism-2 design 6.2): a joint with stiffness_k3_nm_rad
// > 0 gets a motor side of inertia B = its URDF armature, coupled to the link through the
// three-segment spring (harmonic_spring.hpp) with transmission error and lost motion; its
// Coulomb and viscous friction (and URDF damping) act on the motor. Integrated with RK4
// sub-steps, see docs/derivations/sim_plant_transmission.md. k3 == 0 on every joint is the rigid
// plant, bit for bit.
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
  // Transmission (imperfect::TransmissionConfig); k3 == 0: rigid joint, the rest is ignored.
  VecN<N> stiffness_k1_nm_rad = VecN<N>::Zero();
  VecN<N> stiffness_k2_nm_rad = VecN<N>::Zero();
  VecN<N> stiffness_k3_nm_rad = VecN<N>::Zero();
  VecN<N> torque_t1_nm = VecN<N>::Zero();           // N m, 0 < t1 < t2 when elastic
  VecN<N> torque_t2_nm = VecN<N>::Zero();           // N m
  VecN<N> damping_ratio = VecN<N>::Constant(0.01);  // of 2 sqrt(k1 B), >= 0
  VecN<N> error_amp_rad = VecN<N>::Zero();          // e = amp sin(2 ratio theta + phase)
  VecN<N> lost_motion_rad = VecN<N>::Zero();        // total width of the dead zone, >= 0
  VecN<N> ratio = VecN<N>::Constant(100.0);         // gear ratio, > 0
  VecN<N> motor_phase_rad = VecN<N>::Zero();        // phase of the transmission error
};

template <int N>
class SimPlant {
 public:
  SimPlant(const Model<N>& model, const VecN<N>& q0, const PlantEffects<N>& effects = {});

  // Advance by dt_s with commanded torque tau held constant over the step.
  void step(const VecN<N>& tau_cmd, double dt_s);

  const VecN<N>& q() const { return q_; }
  const VecN<N>& qd() const { return qd_; }
  const VecN<N>& tau_applied() const { return tau_applied_; }  // after motor lag and cap
  double time_s() const { return time_s_; }
  // Motor angle (rad) and velocity (rad/s) in link-side units (motor / ratio); q and qd on a
  // rigid joint.
  const VecN<N>& q_motor() const { return theta_; }
  const VecN<N>& qd_motor() const { return thetad_; }
  // Spring torque tau_s at the current state (N m); tau_applied on a rigid joint.
  const VecN<N>& spring_torque() const { return tau_spring_; }

 private:
  // State of the elastic path: link (q, qd) and motor (theta, thetad).
  struct State {
    VecN<N> q;
    VecN<N> qd;
    VecN<N> theta;
    VecN<N> thetad;
  };
  State derivative(const State& x) const;
  VecN<N> spring(const State& x) const;
  void substep(double h);

  Model<N> model_;
  PlantEffects<N> effects_;
  VecN<N> q_;
  VecN<N> qd_ = VecN<N>::Zero();
  VecN<N> tau_lagged_ = VecN<N>::Zero();  // motor lag state, before the tau_max cap
  VecN<N> tau_applied_ = VecN<N>::Zero();
  double time_s_ = 0.0;
  // Friction split (sim_plant.cpp): viscous torque per joint inside RK4 (-URDF damping on a
  // joint with Coulomb friction, cancelling it there) and in the stick-slip projection.
  VecN<N> viscous_rk4_ = VecN<N>::Zero();
  VecN<N> viscous_projection_ = VecN<N>::Zero();
  bool has_coulomb_ = false;  // any coulomb_nm > 0: the projection needs M(q) each step
  // Joint friction impulses of the last projection (N m s), warm start of the next one.
  VecN<N> friction_impulse_ = VecN<N>::Zero();

  // Transmission (elastic path, used only when a joint is elastic).
  bool has_elastic_ = false;
  VecN<N> elastic_ = VecN<N>::Zero();             // 1 elastic, 0 rigid
  Model<N> link_model_;                           // elastic joints without armature and damping
  VecN<N> motor_inertia_ = VecN<N>::Ones();       // B, kg m^2
  VecN<N> spring_damping_ = VecN<N>::Zero();      // d, N m s / rad
  VecN<N> stiffness_max_ = VecN<N>::Zero();       // max(k1, k2, k3), sets the sub-step count
  VecN<N> motor_viscous_rk4_ = VecN<N>::Zero();   // motor viscous inside RK4 (no Coulomb)
  VecN<N> motor_viscous_proj_ = VecN<N>::Zero();  // motor viscous in its stick-slip step
  VecN<N> rigid_coulomb_ = VecN<N>::Zero();       // coupled projection: rigid joints only
  VecN<N> theta_;
  VecN<N> thetad_ = VecN<N>::Zero();
  VecN<N> tau_spring_ = VecN<N>::Zero();
  VecN<N> motor_stuck_ = VecN<N>::Zero();  // 1: motor held by static friction this sub-step
  VecN<N> link_locked_ = VecN<N>::Zero();  // 1: link pressed into its stop this sub-step
};

}  // namespace cobot::dynamics
