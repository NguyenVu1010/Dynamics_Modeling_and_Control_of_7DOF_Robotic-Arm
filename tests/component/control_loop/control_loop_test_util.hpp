#pragma once

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/dynamics/sim_plant.hpp"
#include "cobot/hal/joint_interface.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/control_loop.hpp"
#include "cobot/pipeline/log_record.hpp"
#include "cobot/pipeline/sim_joint_interface.hpp"

// Shared set-up for the ControlLoop component tests: the real arm and config/default.yaml
// without friction compensation (tests/e2e/configs/ideal_world.yaml),
// an ideal SimPlant (no friction, no motor lag, exact model) behind a SimJointInterface
// without jitter, and the loop under test.
namespace cobot::pipeline::test {

using common::Mat3;
using common::Pose;
using common::Vec3;

constexpr int kN = 7;
using Vec = common::VecN<kN>;
using State = hal::JointState<kN>;
using Command = hal::JointCommand<kN>;
using Record = LogRecord<kN>;

struct Setup {
  model::Arm7 model;
  Config<kN> config;
};

// tests/e2e/configs/ideal_world.yaml (config/default.yaml with friction compensation off: the
// plant here has no friction, so compensating would inject a disturbance) and its URDF.
inline std::optional<Setup> load_setup() {
  const std::filesystem::path yaml =
      std::filesystem::path(COBOT_REPO_DIR) / "tests/e2e/configs/ideal_world.yaml";
  const auto cfg = load_config<kN>(yaml);
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
  return Setup{arm.value(), cfg.value()};
}

// Plant, HAL and loop wired as a host would wire them. The plant starts at rest at home.
struct Rig {
  Rig(const model::Arm7& model, const Config<kN>& config)
      : plant(model, config.home_q_rad),
        io(plant, 1.0 / config.control_rate_hz),
        loop(model, config) {}

  // Reads the first state and starts the loop on it.
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

  dynamics::SimPlant<kN> plant;
  SimJointInterface<kN> io;
  ControlLoop<kN> loop;
};

inline std::unique_ptr<Rig> make_rig(const Setup& setup, const Config<kN>& config) {
  return std::make_unique<Rig>(setup.model, config);
}

// Pops every record currently in the ring.
inline void drain(ControlLoop<kN>& loop, std::vector<Record>& out) {
  Record r;
  while (loop.log().pop(r)) {
    out.push_back(r);
  }
}

inline Vec q_min_of(const model::Arm7& m) {
  Vec v;
  for (int i = 0; i < kN; ++i) {
    v(i) = m.joints[static_cast<std::size_t>(i)].q_min;
  }
  return v;
}

inline Vec q_max_of(const model::Arm7& m) {
  Vec v;
  for (int i = 0; i < kN; ++i) {
    v(i) = m.joints[static_cast<std::size_t>(i)].q_max;
  }
  return v;
}

inline Vec qd_max_of(const model::Arm7& m) {
  Vec v;
  for (int i = 0; i < kN; ++i) {
    v(i) = m.joints[static_cast<std::size_t>(i)].qd_max;
  }
  return v;
}

inline Vec tau_max_of(const model::Arm7& m) {
  Vec v;
  for (int i = 0; i < kN; ++i) {
    v(i) = m.joints[static_cast<std::size_t>(i)].tau_max;
  }
  return v;
}

// Every element finite and |tau_i| <= tau_max_i.
inline bool torque_ok(const model::Arm7& m, const Vec& tau) {
  return tau.allFinite() && (tau.cwiseAbs().array() <= tau_max_of(m).array()).all();
}

// R = Rz(yaw) Ry(pitch) Rx(roll), written out element by element.
inline Mat3 rpy_matrix(const Vec3& rpy) {
  const double cr = std::cos(rpy.x());
  const double sr = std::sin(rpy.x());
  const double cp = std::cos(rpy.y());
  const double sp = std::sin(rpy.y());
  const double cy = std::cos(rpy.z());
  const double sy = std::sin(rpy.z());
  Mat3 rx;
  rx << 1.0, 0.0, 0.0, 0.0, cr, -sr, 0.0, sr, cr;
  Mat3 ry;
  ry << cp, 0.0, sp, 0.0, 1.0, 0.0, -sp, 0.0, cp;
  Mat3 rz;
  rz << cy, -sy, 0.0, sy, cy, 0.0, 0.0, 0.0, 1.0;
  const Mat3 r = rz * ry * rx;
  return r;
}

inline Pose goal_pose(const Config<kN>& config) {
  return Pose{rpy_matrix(config.trajectory.goal_rpy_rad), config.trajectory.goal_position_m};
}

}  // namespace cobot::pipeline::test
