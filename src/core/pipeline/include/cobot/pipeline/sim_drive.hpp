#pragma once

#include <cstdint>

#include "cobot/common/types.hpp"
#include "cobot/dynamics/sim_plant.hpp"
#include "cobot/hal/joint_interface.hpp"
#include "cobot/imperfect/actuator_stage.hpp"
#include "cobot/imperfect/config.hpp"
#include "cobot/pipeline/config.hpp"

// Simulated joint drive of the algorithm drive_position (feature drive-loop, ADR-023): between the
// bus (ImperfectJointInterface) and the plant, called by SimJointInterface::advance(). It closes a
// position loop on its own motor encoder at drive.rate_hz and feeds the current loop (the plant's
// motor lag) through the profile's actuator chain. One control period of dt is substeps() equal
// sub-steps of h = dt / substeps(); per sub-step:
//   (1) a position command whose seq differs from the latched one (or the first one) is latched
//       as q_s, qd_s, tau_s with age 0;
//   (2) a torque command goes to plant.step(cmd.tau, h) as is (already shaped by the decorator;
//       the latch is cleared, the encoder filter restarts on the next position command);
//   (3) encoder q_m = round_to(plant.q_motor(), 2 pi / 2^encoder_bits) (bits 0: raw); velocity
//       v += a ((q_m - q_m_prev) / h - v), a = h / (h + 1 / (2 pi velocity_filter_hz)),
//       v = plant.qd_motor() on the first sub-step;
//   (4) q_t = q_s + qd_s min(age, T_sp), qd_t = age < T_sp ? qd_s : 0, T_sp = setpoint_period_s;
//   (5) tau = clamp(tau_s + kp (q_t - q_m) + kd (qd_t - v), +-peak_torque_nm), non-finite -> 0;
//   (6) tau = stage.limit(stage.shape(tau, plant.q()), plant.qd_motor()), an ActuatorStage at
//       period h (the slew limit of the actual sub-step) with the decorator's torque scale;
//   (7) plant.step(tau, h); (8) age += h.
// Gains kp = J omega^2 (N m/rad), kd = 2 zeta J omega (N m s/rad), J = inertia_kg_m2 (the hosts
// pass diag(M(home_q)) of the controller model). No allocation after construction.
namespace cobot::pipeline {

template <int N>
class SimDrive {
 public:
  // Throws std::invalid_argument naming cobot.drive.rate_hz (same rule as the loader).
  SimDrive(const DriveConfig<N>& drive,
           const common::VecN<N>& inertia_kg_m2,
           const common::VecN<N>& peak_torque_nm,
           const imperfect::ImperfectionConfig<N>& profile,
           const common::VecN<N>& torque_scale,
           double control_period_s,
           double setpoint_period_s);

  // One control period of dt_s (jittered or not): substeps() sub-steps of h = dt_s / substeps().
  void advance(dynamics::SimPlant<N>& plant, const hal::JointCommand<N>& cmd, double dt_s);

  int substeps() const { return substeps_; }  // rate_hz / control_rate_hz
  std::int64_t substeps_run() const { return substeps_run_; }
  const common::VecN<N>& kp() const { return kp_; }
  const common::VecN<N>& kd() const { return kd_; }
  const common::VecN<N>& target_q() const { return target_q_; }  // q_t of the last sub-step

 private:
  int substeps_ = 1;
  std::int64_t substeps_run_ = 0;
  common::VecN<N> kp_;
  common::VecN<N> kd_;
  common::VecN<N> peak_;
  common::VecN<N> encoder_bits_;
  double filter_hz_;
  double setpoint_period_s_;
  imperfect::ActuatorStage<N> stage_;
  bool latched_ = false;
  std::uint32_t seq_ = 0;
  common::VecN<N> q_s_ = common::VecN<N>::Zero();
  common::VecN<N> qd_s_ = common::VecN<N>::Zero();
  common::VecN<N> tau_s_ = common::VecN<N>::Zero();
  double age_s_ = 0.0;
  bool encoder_ready_ = false;
  common::VecN<N> q_m_prev_ = common::VecN<N>::Zero();
  common::VecN<N> v_ = common::VecN<N>::Zero();
  common::VecN<N> target_q_ = common::VecN<N>::Zero();
};

}  // namespace cobot::pipeline
