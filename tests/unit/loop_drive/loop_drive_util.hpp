#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <optional>

#include <gtest/gtest.h>

#include "cobot/control/algorithm.hpp"
#include "cobot/control/drive_position.hpp"
#include "cobot/dynamics/dynamics.hpp"
#include "cobot/imperfect/config.hpp"
#include "cobot/pipeline/sim_drive.hpp"

#include "control_loop_test_util.hpp"

// ControlLoop with the drive_position algorithm on the rig of tests/component/control_loop
// (tests/e2e/configs/ideal_world.yaml, ideal SimPlant behind SimJointInterface), with a SimDrive
// attached to the interface as the bench attaches it (brief drive-loop 7): J = diag(M(home)) of
// the controller model, peak torque = its tau_max, ideal profile, unit torque scale, set points
// every control period (servo_rate_hz 0).
namespace cobot::pipeline::test {

inline Config<kN> drive_config(const Config<kN>& base, control::DriveFeedForward ff) {
  Config<kN> c = base;
  c.algorithm = control::Algorithm::kDrivePosition;
  c.drive.torque_feedforward = ff;
  return c;
}

inline Vec inertia_at(const model::Arm7& m, const Vec& q) {
  const common::MatN<kN> M = dynamics::mass_matrix(m, q);
  return M.diagonal();
}

struct DriveRig {
  DriveRig(const model::Arm7& model,
           const Config<kN>& config,
           std::size_t log_capacity = kRigLogCapacity)
      : storage(log_capacity, 1),
        recorder(storage.memory(), FullPolicy::kStopWhenFull),
        plant(model, config.home_q_rad),
        io(plant, 1.0 / config.control_rate_hz),
        drive(config.drive,
              inertia_at(model, config.home_q_rad),
              tau_max_of(model),
              imperfect::ImperfectionConfig<kN>{},
              Vec::Ones(),
              1.0 / config.control_rate_hz,
              1.0 / config.control_rate_hz),
        loop(model, config, &recorder) {
    io.attach_drive(&drive);
  }

  common::Status start() {
    State s;
    EXPECT_TRUE(io.read(s).is_ok());
    return loop.start(s);
  }

  // One host cycle: read -> (optional edit of the state) -> step -> write -> advance.
  template <typename Edit>
  common::Status cycle(State& s, Command& cmd, Edit edit) {
    EXPECT_TRUE(io.read(s).is_ok());
    edit(s);
    const common::Status st = loop.step(s, cmd);
    EXPECT_TRUE(io.write(cmd).is_ok());
    io.advance();
    return st;
  }
  common::Status cycle(State& s, Command& cmd) {
    return cycle(s, cmd, [](State& /*unused*/) {});
  }

  HeapStorage<kN> storage;
  Recorder<kN> recorder;
  dynamics::SimPlant<kN> plant;
  SimJointInterface<kN> io;
  SimDrive<kN> drive;
  ControlLoop<kN> loop;
};

inline std::uint64_t bits_of(double v) {
  std::uint64_t u = 0;
  std::memcpy(&u, &v, sizeof(u));
  return u;
}

// Same object representation element by element (NaN payloads and the sign of zero count).
inline bool bits_equal(const Vec& a, const Vec& b) {
  for (int j = 0; j < kN; ++j) {
    if (bits_of(a(j)) != bits_of(b(j))) {
      return false;
    }
  }
  return true;
}

// Element-wise clamp to [-tau_max, tau_max], as control::detail::clamp_or_zero does for a finite
// raw torque (non-finite raw: the all-zero command).
inline Vec clamp_or_zero(const model::Arm7& m, const Vec& raw) {
  if (!raw.allFinite()) {
    return Vec::Zero();
  }
  const Vec lim = tau_max_of(m);
  Vec out;
  for (int i = 0; i < kN; ++i) {
    out(i) = raw(i) > lim(i) ? lim(i) : (raw(i) < -lim(i) ? -lim(i) : raw(i));
  }
  return out;
}

// tau + tau_ff clamped element-wise, as ControlLoop's add_clamped (friction feed-forward).
inline Vec add_clamped(const model::Arm7& m, const Vec& tau, const Vec& tau_ff) {
  const Vec lim = tau_max_of(m);
  Vec out;
  for (int i = 0; i < kN; ++i) {
    const double t = tau(i) + tau_ff(i);
    out(i) = t > lim(i) ? lim(i) : (t < -lim(i) ? -lim(i) : t);
  }
  return out;
}

// Joint damping of the model as a vector (N m s / rad).
inline Vec damping_of(const model::Arm7& m) {
  Vec d;
  for (int i = 0; i < kN; ++i) {
    d(i) = m.joints[static_cast<std::size_t>(i)].damping;
  }
  return d;
}

// The torque feed-forward of DrivePosition before the clamp:
//   kNone 0, kGravity G(q_ref), kInverseDynamics rnea(q_ref, qd_ref, qdd_ref) + D qd_ref.
inline Vec raw_feedforward(const model::Arm7& m,
                           control::DriveFeedForward ff,
                           const Vec& q,
                           const Vec& qd,
                           const Vec& qdd) {
  switch (ff) {
    case control::DriveFeedForward::kNone:
      return Vec::Zero();
    case control::DriveFeedForward::kGravity:
      return dynamics::gravity_torque(m, q);
    case control::DriveFeedForward::kInverseDynamics: {
      const Vec raw = dynamics::rnea(m, q, qd, qdd, m.gravity_b) + damping_of(m).cwiseProduct(qd);
      return raw;
    }
  }
  return Vec::Zero();
}

}  // namespace cobot::pipeline::test
