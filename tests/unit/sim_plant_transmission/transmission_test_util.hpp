#pragma once

#include <algorithm>
#include <cmath>

#include "cobot/common/types.hpp"
#include "cobot/dynamics/sim_plant.hpp"
#include "cobot/model/model.hpp"

// Shared set-up of the transmission tests (realism-2 design 6.2, brief "transmission"):
// one-joint models with a given link inertia and motor inertia (URDF armature, which an elastic
// joint moves to the motor side), the Harmonic Drive starting values and the spring energy.
namespace cobot::dynamics::test {

using common::Mat3;
using common::Vec3;
using V1 = VecN<1>;

constexpr double kDt = 1e-3;  // s, control period
constexpr double kPi = 3.14159265358979323846;

// Catalog-style three-segment stiffness (N m/rad) with breakpoints t1, t2 (N m).
struct Gear {
  double k1;
  double k2;
  double k3;
  double t1;
  double t2;
};
constexpr Gear kShg25{3.1e4, 5.0e4, 5.7e4, 14.0, 48.0};  // joints 1-2
constexpr Gear kShg20{1.3e4, 2.5e4, 2.9e4, 7.0, 25.0};   // joints 3-4
constexpr Gear kShg14{4.7e3, 6.1e3, 7.1e3, 2.0, 6.9};    // joints 5-7

constexpr double kArmatureWrist = 0.22674;  // kg m^2, URDF cobot_armature of joints 5-7

// Makes joint j elastic with gear g and the other transmission values given; every value is set
// explicitly so the tests do not depend on PlantEffects defaults.
template <int N>
void set_elastic(PlantEffects<N>& fx,
                 int j,
                 const Gear& g,
                 double damping_ratio,
                 double error_amp_rad = 0.0,
                 double lost_motion_rad = 0.0,
                 double ratio = 100.0,
                 double motor_phase_rad = 0.0) {
  fx.stiffness_k1_nm_rad(j) = g.k1;
  fx.stiffness_k2_nm_rad(j) = g.k2;
  fx.stiffness_k3_nm_rad(j) = g.k3;
  fx.torque_t1_nm(j) = g.t1;
  fx.torque_t2_nm(j) = g.t2;
  fx.damping_ratio(j) = damping_ratio;
  fx.error_amp_rad(j) = error_amp_rad;
  fx.lost_motion_rad(j) = lost_motion_rad;
  fx.ratio(j) = ratio;
  fx.motor_phase_rad(j) = motor_phase_rad;
}

// One revolute joint about `axis`, 1 kg (or `mass`) at `com`, isotropic inertia i_com about the
// centre of mass, armature B (the motor side of an elastic joint). With com on a vertical axis the
// link inertia about the axis is exactly i_com and gravity does no work.
inline model::Model<1> one_joint(double i_com,
                                 double B,
                                 const Vec3& axis = Vec3::UnitZ(),
                                 const Vec3& com = Vec3::Zero(),
                                 double mass = 1.0) {
  model::Model<1> mdl;
  mdl.joints[0].axis_joint = axis;
  mdl.joints[0].q_min = -1e6;
  mdl.joints[0].q_max = 1e6;
  mdl.joints[0].qd_max = 1e6;
  mdl.joints[0].tau_max = 1e6;
  mdl.joints[0].armature = B;
  mdl.links[0].mass_kg = mass;
  mdl.links[0].com_i = com;
  mdl.links[0].I_com_i = i_com * Mat3::Identity();
  return mdl;
}

// Potential energy of the three-segment spring at deflection phi (integral of
// harmonic_spring_torque from 0), J.
inline double spring_energy(double phi, const Gear& g) {
  const double a = std::abs(phi);
  const double phi1 = g.t1 / g.k1;
  const double phi2 = phi1 + ((g.t2 - g.t1) / g.k2);
  if (a <= phi1) {
    return 0.5 * g.k1 * a * a;
  }
  const double u1 = 0.5 * g.t1 * phi1;
  if (a <= phi2) {
    return u1 + (g.t1 * (a - phi1)) + (0.5 * g.k2 * (a - phi1) * (a - phi1));
  }
  const double u2 = u1 + (0.5 * (g.t1 + g.t2) * (phi2 - phi1));
  return u2 + (g.t2 * (a - phi2)) + (0.5 * g.k3 * (a - phi2) * (a - phi2));
}

}  // namespace cobot::dynamics::test
