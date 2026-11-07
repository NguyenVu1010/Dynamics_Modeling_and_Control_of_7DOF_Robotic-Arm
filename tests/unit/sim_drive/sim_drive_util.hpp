#pragma once

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <optional>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/control/drive_position.hpp"
#include "cobot/dynamics/dynamics.hpp"
#include "cobot/hal/joint_interface.hpp"
#include "cobot/imperfect/config.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/sim_drive.hpp"

// Shared set-up of the SimDrive tests (brief drive-loop 6): the arm of config/default.yaml as a
// rigid model without gravity (make_model(chain, Vec3::Zero())), the ideal profile (every
// imperfection off) unless a test sets one, control rate 1 kHz.
namespace cobot::pipeline::drive_test {

constexpr int kN = 7;
using Vec = common::VecN<kN>;
using Command = hal::JointCommand<kN>;
using Profile = imperfect::ImperfectionConfig<kN>;
using Drive = SimDrive<kN>;
using Plant = dynamics::SimPlant<kN>;

constexpr double kDt = 1e-3;  // s, control period at 1 kHz

struct Arm {
  Config<kN> config;
  model::Arm7 model;  // rigid, gravity zero
  Vec home = Vec::Zero();
  Vec inertia = Vec::Zero();  // J = diag(M(home)), kg m^2 (armature included)
  Vec tau_max = Vec::Zero();  // URDF effort M, N m
};

inline std::optional<Arm> load_arm() {
  const auto cfg = load_config<kN>(std::filesystem::path(COBOT_REPO_DIR) / "config/default.yaml");
  if (!cfg) {
    ADD_FAILURE() << "load_config: " << cfg.reason();
    return std::nullopt;
  }
  const Config<kN>& c = cfg.value();
  const auto chain = model::parse_urdf_chain(c.urdf_path, c.base_link, c.tip_link);
  if (!chain) {
    ADD_FAILURE() << "parse_urdf_chain: " << chain.reason();
    return std::nullopt;
  }
  const auto arm = model::make_model<kN>(chain.value(), common::Vec3::Zero());
  if (!arm) {
    ADD_FAILURE() << "make_model: " << arm.reason();
    return std::nullopt;
  }
  Arm out{c, arm.value()};
  out.home = c.home_q_rad;
  const common::MatN<kN> m = dynamics::mass_matrix(out.model, out.home);
  out.inertia = m.diagonal();
  for (int j = 0; j < kN; ++j) {
    out.tau_max(j) = out.model.joints[static_cast<std::size_t>(j)].tau_max;
  }
  return out;
}

inline DriveConfig<kN> drive_config(double rate_hz = 4000.0,
                                    double omega_rad_s = 60.0,
                                    double zeta = 1.0,
                                    double velocity_filter_hz = 500.0) {
  DriveConfig<kN> d;
  d.rate_hz = rate_hz;
  d.omega_rad_s.setConstant(omega_rad_s);
  d.zeta = zeta;
  d.velocity_filter_hz = velocity_filter_hz;
  d.torque_feedforward = control::DriveFeedForward::kInverseDynamics;
  return d;
}

inline Command position_command(const Vec& q, const Vec& qd, const Vec& tau, std::uint32_t seq) {
  Command c;
  c.mode = hal::DriveMode::kPosition;
  c.q = q;
  c.qd = qd;
  c.tau = tau;
  c.seq = seq;
  return c;
}

inline Command torque_command(const Vec& tau) {
  Command c;
  c.tau = tau;
  return c;  // mode kTorque by default
}

inline std::uint64_t bits_of(double v) {
  std::uint64_t u = 0;
  std::memcpy(&u, &v, sizeof(u));
  return u;
}

}  // namespace cobot::pipeline::drive_test
