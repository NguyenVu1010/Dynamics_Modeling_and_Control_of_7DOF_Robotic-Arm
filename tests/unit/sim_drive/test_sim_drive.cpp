// pipeline::SimDrive (brief drive-loop 6): the simulated joint drive between the bus and the plant.
// Per sub-step of h = dt / substeps():
//   (1) a position command whose seq differs from the latched one (or the first one) is latched
//       as q_s, qd_s, tau_s with age 0;
//   (2) a torque command goes to plant.step(cmd.tau, h) as is (latch cleared);
//   (3) encoder q_m = round_to(plant.q_motor(), 2 pi / 2^bits) (bits 0: raw), velocity
//       v += a ((q_m - q_m_prev) / h - v), a = h / (h + 1 / (2 pi velocity_filter_hz)),
//       v = plant.qd_motor() on the first sub-step;
//   (4) q_t = q_s + qd_s min(age, T_sp), qd_t = age < T_sp ? qd_s : 0;
//   (5) tau = clamp(tau_s + kp (q_t - q_m) + kd (qd_t - v), +-peak_torque_nm), non-finite -> 0;
//   (6) tau = stage.limit(stage.shape(tau, plant.q()), plant.qd_motor()) (ActuatorStage at h);
//   (7) plant.step(tau, h); (8) age += h.
// Gains kp = J omega^2, kd = 2 zeta J omega, J = the inertia passed (diag(M(home)) by the hosts).
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "sim_drive_util.hpp"

namespace cobot::pipeline::drive_test {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

// A drive on the ideal profile with unit torque scale, set points every control period.
Drive make_drive(const Arm& arm,
                 const DriveConfig<kN>& d,
                 const Profile& profile = Profile{},
                 const Vec& torque_scale = Vec::Ones(),
                 const std::optional<Vec>& peak = std::nullopt,
                 double setpoint_period_s = kDt) {
  return {
      d, arm.inertia, peak ? *peak : arm.tau_max, profile, torque_scale, kDt, setpoint_period_s};
}

TEST(SimDrive, Rates) {
  const auto arm = load_arm();
  ASSERT_TRUE(arm);
  Plant plant(arm->model, arm->home);
  Drive drive = make_drive(*arm, drive_config(4000.0));
  EXPECT_EQ(drive.substeps(), 4);  // 4000 Hz / 1000 Hz
  EXPECT_EQ(drive.substeps_run(), 0);
  const Command hold = position_command(arm->home, Vec::Zero(), Vec::Zero(), 1);
  for (int k = 0; k < 10; ++k) {
    drive.advance(plant, hold, kDt);
  }
  EXPECT_EQ(drive.substeps_run(), 40);  // 10 advances * 4 sub-steps
  EXPECT_NEAR(plant.time_s(), 10 * kDt, 1e-15);

  // A jittered period of 1.2 ms: the same 4 sub-steps of h = 1.2e-3 / 4 = 3e-4 s.
  const double t0 = plant.time_s();
  drive.advance(plant, hold, 1.2e-3);
  EXPECT_NEAR(plant.time_s() - t0, 1.2e-3, 1e-15);
  EXPECT_EQ(drive.substeps_run(), 44);

  // 1000 Hz: one sub-step per period (ratio 1 is an integer >= 1).
  EXPECT_EQ(make_drive(*arm, drive_config(1000.0)).substeps(), 1);

  // 2500 / 1000 = 2.5 is not an integer; 500 / 1000 = 0.5 is below 1.
  for (const double rate : {2500.0, 500.0}) {
    try {
      static_cast<void>(make_drive(*arm, drive_config(rate)));
      ADD_FAILURE() << "rate " << rate << " accepted";
    } catch (const std::invalid_argument& e) {
      EXPECT_NE(std::string(e.what()).find("cobot.drive.rate_hz"), std::string::npos)
          << "rate " << rate << ": " << e.what();
    }
  }
}

TEST(SimDrive, Gains) {
  const auto arm = load_arm();
  ASSERT_TRUE(arm);
  DriveConfig<kN> d = drive_config();
  d.omega_rad_s << 20.0, 30.0, 45.0, 60.0, 80.0, 100.0, 150.0;
  d.zeta = 0.7;
  const Drive drive = make_drive(*arm, d);
  for (int j = 0; j < kN; ++j) {
    const double J = arm->inertia(j);
    const double w = d.omega_rad_s(j);
    const double kp = J * w * w;             // N m/rad
    const double kd = 2.0 * d.zeta * J * w;  // N m s/rad
    EXPECT_NEAR(drive.kp()(j), kp, 1e-12 * kp) << "joint " << j;
    EXPECT_NEAR(drive.kd()(j), kd, 1e-12 * kd) << "joint " << j;
  }
}

// Set point home with joint 7 (index 6) + 0.01 rad, qd 0, tau 0, omega 100, zeta 1, velocity
// filter 1e4 Hz, seq constant. Joint 7 is dominated by its armature (0.227 kg m^2), so with
// J = M_77(home) it is close to a decoupled critically damped second-order loop:
//   e(t) = 0.01 (1 + w t) e^(-w t); at t = 50 ms, w t = 5: 0.01 * 6 * e^-5 = 4.04e-4 rad.
// A 1-DOF simulation of the discrete law (h = 0.25 ms, filter a = 0.94, RK4) gives 4.17e-4 rad.
TEST(SimDrive, StepResponse) {
  const auto arm = load_arm();
  ASSERT_TRUE(arm);
  Plant plant(arm->model, arm->home);
  Drive drive = make_drive(*arm, drive_config(4000.0, 100.0, 1.0, 1e4));
  Vec target = arm->home;
  target(6) += 0.01;
  const Command cmd = position_command(target, Vec::Zero(), Vec::Zero(), 7);
  double err_50ms = std::numeric_limits<double>::quiet_NaN();
  double overshoot = -kInf;
  double others = 0.0;
  for (int k = 1; k <= 200; ++k) {  // 200 ms
    drive.advance(plant, cmd, kDt);
    const Vec& q = plant.q();
    ASSERT_TRUE(q.allFinite()) << "advance " << k;
    overshoot = std::max(overshoot, q(6) - target(6));
    for (int j = 0; j < 6; ++j) {
      others = std::max(others, std::abs(q(j) - arm->home(j)));
    }
    if (k == 50) {
      err_50ms = target(6) - q(6);
    }
  }
  EXPECT_GE(err_50ms, 3.0e-4);
  EXPECT_LE(err_50ms, 5.0e-4);
  EXPECT_LE(overshoot, 2e-4) << "past the target";
  EXPECT_LE(others, 1e-4) << "other joints left home";
}

// q_s = home, qd_s of joint 1 = 0.1 rad/s, T_sp = 4 ms (servo 250 Hz), h = 1e-3 / 4 = 2.5e-4 s.
// target_q() is q_t of the last sub-step of the advance:
//   advance 1: sub-steps at age 0, h, 2h, 3h -> q_t - q_s = 0.1 * 3 * 2.5e-4 = 7.5e-5 rad;
//   advance 8: last sub-step at age 31h = 7.75e-3 s > T_sp -> 0.1 * 4e-3 = 4.0e-4 rad.
// Commands with the same seq are not latched again (servo hold re-sends): advances 2-8 carry a
// different q with seq unchanged. A new seq latches the new q_s and restarts the age:
//   q_t = q_s2 + 0.1 * 0.75e-3 after its first advance.
TEST(SimDrive, Extrapolation) {
  const auto arm = load_arm();
  ASSERT_TRUE(arm);
  Plant plant(arm->model, arm->home);
  Drive drive = make_drive(*arm, drive_config(), Profile{}, Vec::Ones(), std::nullopt, 4e-3);
  Vec qd = Vec::Zero();
  qd(0) = 0.1;
  const Vec q_s = arm->home;
  drive.advance(plant, position_command(q_s, qd, Vec::Zero(), 1), kDt);
  EXPECT_NEAR(drive.target_q()(0) - q_s(0), 7.5e-5, 1e-15);
  for (int j = 1; j < kN; ++j) {
    EXPECT_EQ(drive.target_q()(j), q_s(j)) << "joint " << j << " (qd_s 0)";
  }
  const Vec moved = q_s + Vec::Constant(1.0);
  for (int k = 2; k <= 8; ++k) {
    drive.advance(plant, position_command(moved, qd, Vec::Zero(), 1), kDt);
  }
  EXPECT_NEAR(drive.target_q()(0) - q_s(0), 4.0e-4, 1e-15) << "same seq: not latched again";

  Vec q_s2 = arm->home;
  q_s2(0) += 0.05;
  drive.advance(plant, position_command(q_s2, qd, Vec::Zero(), 2), kDt);
  EXPECT_NEAR(drive.target_q()(0) - q_s2(0), 0.1 * 0.75e-3, 1e-15);
  for (int j = 1; j < kN; ++j) {
    EXPECT_EQ(drive.target_q()(j), q_s2(j)) << "joint " << j;
  }
}

// A torque command clears the latch, so the next position command is latched even with the seq
// of the one before (it is "the first one" again).
TEST(SimDrive, TorqueCommandClearsTheLatch) {
  const auto arm = load_arm();
  ASSERT_TRUE(arm);
  Plant plant(arm->model, arm->home);
  Drive drive = make_drive(*arm, drive_config(), Profile{}, Vec::Ones(), std::nullopt, 4e-3);
  drive.advance(plant, position_command(arm->home, Vec::Zero(), Vec::Zero(), 5), kDt);
  drive.advance(plant, torque_command(Vec::Zero()), kDt);
  Vec q_new = arm->home;
  q_new(2) += 0.02;
  drive.advance(plant, position_command(q_new, Vec::Zero(), Vec::Zero(), 5), kDt);
  EXPECT_EQ(drive.target_q()(2), q_new(2));
}

// Torque command c with a deadband of 0.5 N m in the profile: the command is already shaped by
// the decorator, so the drive passes it to the plant as is (|c| < deadband would be zeroed by the
// actuator stage). Ideal plant: no motor lag, |c| far below the URDF cap M.
TEST(SimDrive, TorquePassThrough) {
  const auto arm = load_arm();
  ASSERT_TRUE(arm);
  Plant plant(arm->model, arm->home);
  Profile p;
  p.actuator.deadband_nm.setConstant(0.5);
  Drive drive = make_drive(*arm, drive_config(), p);
  Vec c;
  c << 0.3, -0.2, 0.1, -0.4, 0.05, -0.05, 0.2;
  for (int k = 1; k <= 5; ++k) {
    drive.advance(plant, torque_command(c), kDt);
    for (int j = 0; j < kN; ++j) {
      EXPECT_EQ(bits_of(plant.tau_applied()(j)), bits_of(c(j)))
          << "advance " << k << " joint " << j << ": " << plant.tau_applied()(j);
    }
  }
  EXPECT_EQ(drive.substeps_run(), 20);
}

// The drive's output goes through the actuator chain of the profile.
// (a) Deadband 0.5 N m. Set point = the plant's motor position at rest, qd 0, torque feed-forward
//     0.3 N m: q_t - q_m = 0 and v = qd_motor = 0, so the PD output is 0.3 N m on every sub-step;
//     |0.3| < 0.5 -> applied 0 and the plant stays at rest.
// (b) torque_scale 1.1, peak torque 2 N m, feed-forward 1000 N m: the PD output is clamped to
//     2 N m (the kp / kd terms stay below 1 N m over one period), then scaled: applied = 2 * 1.1.
TEST(SimDrive, ActuatorChain) {
  const auto arm = load_arm();
  ASSERT_TRUE(arm);
  {
    Plant plant(arm->model, arm->home);
    Profile p;
    p.actuator.deadband_nm.setConstant(0.5);
    Drive drive = make_drive(*arm, drive_config(), p);
    const Command cmd = position_command(plant.q_motor(), Vec::Zero(), Vec::Constant(0.3), 1);
    for (int k = 1; k <= 5; ++k) {
      drive.advance(plant, cmd, kDt);
      for (int j = 0; j < kN; ++j) {
        EXPECT_EQ(plant.tau_applied()(j), 0.0) << "advance " << k << " joint " << j;
      }
    }
    EXPECT_EQ(plant.q(), arm->home) << "at rest";
  }
  {
    Plant plant(arm->model, arm->home);
    const Vec scale = Vec::Constant(1.1);
    const Vec peak = Vec::Constant(2.0);
    Drive drive = make_drive(*arm, drive_config(), Profile{}, scale, peak);
    const Command cmd = position_command(plant.q_motor(), Vec::Zero(), Vec::Constant(1000.0), 1);
    for (int k = 1; k <= 3; ++k) {
      drive.advance(plant, cmd, kDt);
      for (int j = 0; j < kN; ++j) {
        EXPECT_EQ(plant.tau_applied()(j), peak(j) * scale(j)) << "advance " << k << " joint " << j;
      }
    }
  }
}

}  // namespace
}  // namespace cobot::pipeline::drive_test
