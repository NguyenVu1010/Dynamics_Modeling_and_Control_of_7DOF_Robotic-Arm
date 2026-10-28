#pragma once

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <optional>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/control/computed_torque.hpp"
#include "cobot/dynamics/dynamics.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"

// Shared models and the exact-model plant (RK4 of forward_dynamics, torque held over a step)
// for the control tests.
namespace cobot::control::test {

using common::Mat3;
using common::Vec3;

// Pendulum about +y of the base, CoM at (l, 0, 0) of the link frame:
//   m = 2 kg, l = 0.5 m, I_com_yy = 0.1 kg m^2, damping 0.1 N m s/rad, tau_max 50 N m.
//   M = I_com_yy + m l^2 = 0.1 + 2 * 0.25 = 0.6 kg m^2.
//   With R_y(q) the CoM sits at (l cos q, 0, -l sin q); gravity (0, 0, -g) gives the torque
//   m g l cos q about +y, so the joint must supply G(q) = -m g l cos q = -9.80665 cos q.
//   C(q, qd) qd = 0 for one joint.
inline constexpr double kPendulumMassKg = 2.0;
inline constexpr double kPendulumLengthM = 0.5;
inline constexpr double kPendulumIyy = 0.1;
inline constexpr double kPendulumDamping = 0.1;
inline constexpr double kPendulumTauMax = 50.0;

inline model::Model<1> make_pendulum() {
  model::Model<1> m;
  m.joints[0].name = "pivot";
  m.joints[0].axis_joint = Vec3::UnitY();
  m.joints[0].q_min = -3.0;
  m.joints[0].q_max = 3.0;
  m.joints[0].qd_max = 10.0;
  m.joints[0].tau_max = kPendulumTauMax;
  m.joints[0].damping = kPendulumDamping;
  m.links[0].mass_kg = kPendulumMassKg;
  m.links[0].com_i = Vec3(kPendulumLengthM, 0.0, 0.0);
  const Mat3 inertia = Vec3(0.05, kPendulumIyy, 0.05).asDiagonal();
  m.links[0].I_com_i = inertia;
  return m;
}

struct Arm {
  model::Arm7 model;
  VecN<7> home = VecN<7>::Zero();  // config/default.yaml home_q_deg, in rad
};

// The real arm from config/default.yaml (URDF, links, gravity, home).
inline std::optional<Arm> load_arm() {
  const std::filesystem::path yaml = std::filesystem::path(COBOT_REPO_DIR) / "config/default.yaml";
  const auto cfg = pipeline::load_config<7>(yaml);
  if (!cfg) {
    ADD_FAILURE() << "load_config: " << cfg.reason();
    return std::nullopt;
  }
  const auto chain =
      model::parse_urdf_chain(cfg.value().urdf_path, cfg.value().base_link, cfg.value().tip_link);
  if (!chain) {
    ADD_FAILURE() << "parse_urdf_chain: " << chain.reason();
    return std::nullopt;
  }
  const auto model = model::make_model<7>(chain.value(), cfg.value().gravity_b);
  if (!model) {
    ADD_FAILURE() << "make_model: " << model.reason();
    return std::nullopt;
  }
  return Arm{model.value(), cfg.value().home_q_rad};
}

template <int N>
VecN<N> damping_of(const Model<N>& m) {
  VecN<N> d;
  for (int i = 0; i < N; ++i) {
    d(i) = m.joints[static_cast<std::size_t>(i)].damping;
  }
  return d;
}

template <int N>
VecN<N> tau_max_of(const Model<N>& m) {
  VecN<N> t;
  for (int i = 0; i < N; ++i) {
    t(i) = m.joints[static_cast<std::size_t>(i)].tau_max;
  }
  return t;
}

// Unclamped computed torque, built from the dynamics functions:
//   M(q) (qdd_ref + kd (qd_ref - qd) + kp (q_ref - q)) + C(q, qd) qd + G(q) + D qd
template <int N>
VecN<N> computed_torque_raw(const Model<N>& m,
                            const ComputedTorqueGains<N>& g,
                            const JointState<N>& s,
                            const JointReference<N>& r) {
  const VecN<N> v = r.qdd + g.kd.cwiseProduct(r.qd - s.qd) + g.kp.cwiseProduct(r.q - s.q);
  const VecN<N> tau = dynamics::mass_matrix(m, s.q) * v + dynamics::coriolis_torque(m, s.q, s.qd) +
                      dynamics::gravity_torque(m, s.q) + damping_of(m).cwiseProduct(s.qd);
  return tau;
}

// One RK4 step of the exact plant qdd = forward_dynamics(q, qd, tau), tau held constant.
template <int N>
void rk4_step(const Model<N>& m, VecN<N>& q, VecN<N>& qd, const VecN<N>& tau, double dt) {
  const VecN<N> a1 = dynamics::forward_dynamics(m, q, qd, tau);
  const VecN<N> q2 = q + 0.5 * dt * qd;
  const VecN<N> v2 = qd + 0.5 * dt * a1;
  const VecN<N> a2 = dynamics::forward_dynamics(m, q2, v2, tau);
  const VecN<N> q3 = q + 0.5 * dt * v2;
  const VecN<N> v3 = qd + 0.5 * dt * a2;
  const VecN<N> a3 = dynamics::forward_dynamics(m, q3, v3, tau);
  const VecN<N> q4 = q + dt * v3;
  const VecN<N> v4 = qd + dt * a3;
  const VecN<N> a4 = dynamics::forward_dynamics(m, q4, v4, tau);
  q += dt / 6.0 * (qd + 2.0 * v2 + 2.0 * v3 + v4);
  qd += dt / 6.0 * (a1 + 2.0 * a2 + 2.0 * a3 + a4);
}

// |a - b| <= rel * max(1, |b|) element-wise.
template <int N>
void expect_near_rel(const VecN<N>& a, const VecN<N>& b, double rel) {
  for (int i = 0; i < N; ++i) {
    EXPECT_NEAR(a(i), b(i), rel * std::max(1.0, std::abs(b(i)))) << "joint " << i;
  }
}

}  // namespace cobot::control::test
