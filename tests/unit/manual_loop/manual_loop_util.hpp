#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/lie.hpp"
#include "cobot/common/types.hpp"
#include "cobot/kinematics/kinematics.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/control_loop.hpp"
#include "cobot/pipeline/manual.hpp"

#include "loop_external_util.hpp"

// Helpers for the manual command tests (feature manual-commands, ADR-022 "Lệnh tay"): the shared
// rig of tests/component/control_loop (ideal SimPlant, tests/e2e/configs/ideal_world.yaml) in
// external mode with the cobot.manual block of config/default.yaml, 1 ms steps, and a host that
// keeps the record of every step.
//
// Numbers used by the tests (URDF of the arm, default.yaml):
//   qd_max = (2.1, 2.1, 2.1, 2.1, 3.1, 3.1, 3.1) rad/s
//   manual.joint_a_max_rad_s2 = (15, 7.5, 12.5, 12.5, 15, 20, 20) rad/s^2
//   manual TCP limits at 100 %: 0.25 m/s, 1.0 rad/s, 0.5 m/s^2, 2.0 rad/s^2
//   deadman 100 ms = 100 steps; jog band = soft_margin 3 deg + jog_margin 2 deg = 0.0872665 rad
//   home q = (0, 30, 0, -90, 0, 60, 0) deg
namespace cobot::pipeline::test {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegree = kPi / 180.0;
constexpr int kRefreshEvery = 50;  // steps between two calls of the same jog ("refresh")
constexpr double kSpeed = 0.1;     // speed fraction of the tests unless stated

// cobot.manual of config/default.yaml (ideal_world.yaml has no manual section).
inline ManualConfig<kN> default_manual() {
  const auto cfg = load_config<kN>(std::filesystem::path(COBOT_REPO_DIR) / "config/default.yaml");
  if (!cfg) {
    ADD_FAILURE() << "load_config(default.yaml): " << cfg.reason();
    return ManualConfig<kN>{};
  }
  return cfg.value().manual;
}

// ideal_world.yaml, command_source external, cobot.manual of default.yaml.
inline Config<kN> manual_config(const Setup& setup) {
  Config<kN> cfg = external_config(setup);
  cfg.manual = default_manual();
  return cfg;
}

// The loop's model for the planners (what a host passes to plan_joint_move / plan_tcp_move).
inline model::Arm7 planner_model(const Setup& setup, const Config<kN>& cfg) {
  return controller_model(setup.model, cfg);
}

// A rig driven like a host: one cycle per step(), the record of every step kept in `rows`.
struct Host {
  std::unique_ptr<Rig> rig;
  State s;
  Command cmd;
  std::vector<Record> rows;

  ControlLoop<kN>& loop() { return rig->loop; }
  // One host cycle; the record of that step goes to `rows`.
  void step() {
    EXPECT_TRUE(rig->cycle(s, cmd).is_ok());
    rows.push_back(rig->loop.last_record());
  }
  LoopMode mode() const { return rig->loop.mode(); }
};

// Starts a host on `cfg` (plant at cfg.home_q_rad) and runs `standby_steps` steps; `rows` is
// emptied afterwards, so rows[0] is the first step after the set-up.
inline Host make_host(const Setup& setup,
                      const Config<kN>& cfg,
                      int standby_steps = 100,
                      std::size_t log_capacity = kRigLogCapacity) {
  Host h;
  h.rig = make_rig(setup, cfg, log_capacity);
  EXPECT_TRUE(h.rig->start().is_ok());
  for (int k = 0; k < standby_steps; ++k) {
    h.step();
  }
  h.rows.clear();
  h.rows.reserve(16384);
  return h;
}

// Steps until the loop is in kStandby (checked after each step), at most `max_steps`; calls
// `refresh` before every kRefreshEvery-th step (k = 50, 100, ...). Returns the steps made.
template <typename Refresh>
int step_until_standby(Host& h, int max_steps, Refresh refresh) {
  for (int k = 0; k < max_steps; ++k) {
    if (k > 0 && k % kRefreshEvery == 0) {
      refresh();
    }
    h.step();
    if (h.mode() == LoopMode::kStandby) {
      return k + 1;
    }
  }
  return max_steps;
}
inline int step_until_standby(Host& h, int max_steps) {
  return step_until_standby(h, max_steps, [] {});
}

// `steps` steps with the refresh as above (the jog keeps running).
template <typename Refresh>
void step_refreshed(Host& h, int steps, Refresh refresh) {
  for (int k = 0; k < steps; ++k) {
    if (k > 0 && k % kRefreshEvery == 0) {
      refresh();
    }
    h.step();
  }
}

inline common::Mat6N<kN> jacobian_at(const model::Arm7& m, const Vec& q) {
  kinematics::FkState<kN> fk;
  kinematics::forward_kinematics(m, q, fk);
  common::Mat6N<kN> J;
  kinematics::geometric_jacobian(fk, J);
  return J;
}

// max_ij |a_ij - b_ij| over R and p.
inline double pose_max_diff(const Pose& a, const Pose& b) {
  return std::max((a.R - b.R).cwiseAbs().maxCoeff(), (a.p - b.p).cwiseAbs().maxCoeff());
}

inline bool contains(const std::string& text, const std::string& part) {
  return text.find(part) != std::string::npos;
}

}  // namespace cobot::pipeline::test
