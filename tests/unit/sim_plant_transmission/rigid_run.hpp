#pragma once

#include <array>
#include <cstdio>
#include <filesystem>
#include <optional>
#include <string>

#include <gtest/gtest.h>

#include "cobot/dynamics/sim_plant.hpp"
#include "cobot/hal/joint_interface.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/control_loop.hpp"
#include "cobot/pipeline/sim_joint_interface.hpp"

// E2E-01 inputs (tests/e2e/configs/ideal_world.yaml, ideal plant) through ControlLoop +
// SimJointInterface + SimPlant with the given plant effects, as the bench wires them (controller
// model at the peak torque, plant at the URDF one). The reference file
// tests/data/transmission/rigid_e2e01_reference.csv was written by this code before the
// transmission feature existed.
namespace cobot::dynamics::test {

struct E2e01Setup {
  model::Arm7 plant_model;
  pipeline::Config<7> config;
};

inline std::optional<E2e01Setup> load_e2e01() {
  const auto cfg = pipeline::load_config<7>(std::filesystem::path(COBOT_REPO_DIR) /
                                            "tests/e2e/configs/ideal_world.yaml");
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
  const auto arm = model::make_model<7>(chain.value(), cfg.value().gravity_b);
  if (!arm) {
    ADD_FAILURE() << "make_model: " << arm.reason();
    return std::nullopt;
  }
  return E2e01Setup{arm.value(), cfg.value()};
}

// Runs `cycles` host cycles (read -> step -> write -> advance); on_cycle(k, plant, state, cmd)
// sees every cycle. Returns one CSV line per 100 cycles:
// k, q[7], qd[7], tau_cmd[7], tau_applied[7] with 17 significant digits.
template <typename OnCycle>
std::string run_e2e01(const E2e01Setup& setup,
                      const PlantEffects<7>& fx,
                      int cycles,
                      OnCycle on_cycle) {
  SimPlant<7> plant(setup.plant_model, setup.config.home_q_rad, fx);
  pipeline::SimJointInterface<7> io(plant, 1.0 / setup.config.control_rate_hz);
  pipeline::ControlLoop<7> loop(pipeline::controller_model(setup.plant_model, setup.config),
                                setup.config);
  hal::JointState<7> s;
  EXPECT_TRUE(io.read(s).is_ok());
  EXPECT_TRUE(loop.start(s).is_ok());
  std::string csv;
  for (int k = 0; k < cycles; ++k) {
    hal::JointCommand<7> cmd;
    EXPECT_TRUE(io.read(s).is_ok());
    EXPECT_TRUE(loop.step(s, cmd).is_ok()) << "cycle " << k;
    EXPECT_TRUE(io.write(cmd).is_ok());
    io.advance();
    on_cycle(k, plant, s, cmd);
    if (k % 100 == 99) {
      csv += std::to_string(k);
      const std::array<const VecN<7>*, 4> cols = {
          &plant.q(), &plant.qd(), &cmd.tau, &plant.tau_applied()};
      for (const VecN<7>* v : cols) {
        for (int j = 0; j < 7; ++j) {
          std::array<char, 32> text{};
          std::snprintf(text.data(), text.size(), ",%.17g", (*v)(j));
          csv += text.data();
        }
      }
      csv += "\n";
    }
  }
  return csv;
}

}  // namespace cobot::dynamics::test
