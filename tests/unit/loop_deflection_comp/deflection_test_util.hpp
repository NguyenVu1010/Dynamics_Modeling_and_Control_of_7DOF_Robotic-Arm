#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <optional>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/dynamics/dynamics.hpp"
#include "cobot/dynamics/harmonic_spring.hpp"
#include "cobot/dynamics/sim_plant.hpp"
#include "cobot/hal/joint_interface.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"

// Shared set-up of the deflection-comp tests (brief "deflection-comp", realism-2 design 4.7,
// ADR-019): the catalog Harmonic Drive gears, the nominal compensation values written into a
// controller config, and the E2E-01 inputs (tests/e2e/configs/ideal_world.yaml, URDF plant model).
namespace cobot::pipeline::test {

constexpr int kN = 7;
using Vec = common::VecN<kN>;
using State = hal::JointState<kN>;
using Command = hal::JointCommand<kN>;

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
constexpr std::array<Gear, kN> kArmGears = {kShg25, kShg25, kShg20, kShg20, kShg14, kShg14, kShg14};

inline const Gear& gear_of(int j) {
  return kArmGears[static_cast<std::size_t>(j)];
}

// Controller side: cobot.deflection_comp with the catalog values of every joint.
inline void enable_comp(Config<kN>& cfg) {
  cfg.deflection_comp.enabled = true;
  for (int j = 0; j < kN; ++j) {
    const Gear& g = gear_of(j);
    cfg.deflection_comp.stiffness_k1_nm_rad(j) = g.k1;
    cfg.deflection_comp.stiffness_k2_nm_rad(j) = g.k2;
    cfg.deflection_comp.stiffness_k3_nm_rad(j) = g.k3;
    cfg.deflection_comp.torque_t1_nm(j) = g.t1;
    cfg.deflection_comp.torque_t2_nm(j) = g.t2;
  }
}

// Plant side: every joint elastic with the catalog gear times `stiffness_scale` (breakpoint
// torques unchanged), 1 % damping, no transmission error, no lost motion, ratio 100.
inline dynamics::PlantEffects<kN> elastic_effects(double stiffness_scale) {
  dynamics::PlantEffects<kN> fx;
  for (int j = 0; j < kN; ++j) {
    const Gear& g = gear_of(j);
    fx.stiffness_k1_nm_rad(j) = stiffness_scale * g.k1;
    fx.stiffness_k2_nm_rad(j) = stiffness_scale * g.k2;
    fx.stiffness_k3_nm_rad(j) = stiffness_scale * g.k3;
    fx.torque_t1_nm(j) = g.t1;
    fx.torque_t2_nm(j) = g.t2;
    fx.damping_ratio(j) = 0.01;
    fx.error_amp_rad(j) = 0.0;
    fx.lost_motion_rad(j) = 0.0;
    fx.ratio(j) = 100.0;
    fx.motor_phase_rad(j) = 0.0;
  }
  return fx;
}

// q_hat as the brief defines it (contract 4), computed independently of the loop:
// q_hat_j = q_j - harmonic_spring_deflection(g_j(q)) with the catalog gear of joint j and g the
// gravity torque of `controller` at q.
inline Vec compensated(const model::Arm7& controller, const Vec& q) {
  const Vec g = dynamics::gravity_torque(controller, q);
  Vec q_hat;
  for (int j = 0; j < kN; ++j) {
    const Gear& gr = gear_of(j);
    q_hat(j) = q(j) - dynamics::harmonic_spring_deflection(g(j), gr.k1, gr.k2, gr.k3, gr.t1, gr.t2);
  }
  return q_hat;
}

struct Setup {
  model::Arm7 plant_model;       // URDF (torque cap M)
  model::Arm7 controller_model;  // pipeline::controller_model (peak torque P)
  Config<kN> config;             // comp as loaded; tests set it explicitly
};

inline std::optional<Setup> load_setup() {
  const auto cfg =
      load_config<kN>(std::filesystem::path(COBOT_REPO_DIR) / "tests/e2e/configs/ideal_world.yaml");
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
  const auto arm = model::make_model<kN>(chain.value(), cfg.value().gravity_b);
  if (!arm) {
    ADD_FAILURE() << "make_model: " << arm.reason();
    return std::nullopt;
  }
  Setup s{arm.value(), controller_model(arm.value(), cfg.value()), cfg.value()};
  s.config.deflection_comp.enabled = false;
  return s;
}

}  // namespace cobot::pipeline::test
