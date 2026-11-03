#pragma once

#include <cstddef>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/pipeline/control_loop.hpp"
#include "cobot/pipeline/recorder.hpp"

#include "control_loop_test_util.hpp"

// Helpers of the ControlLoop + Recorder tests (feature pipeline-recorder): bitwise comparison of
// records (field by field, padding bytes excluded) and a closed-loop run on the E2E-01 set-up of
// control_loop_test_util.hpp with the recorder chosen by the caller.
namespace cobot::pipeline::test {

template <typename M>
bool same_bits(const M& a, const M& b) {
  const auto n = static_cast<std::size_t>(a.size());
  return std::memcmp(a.data(), b.data(), n * sizeof(double)) == 0;
}

inline bool same_bits(double a, double b) {
  return std::memcmp(&a, &b, sizeof(double)) == 0;
}

// Every field of the two records has the same bits.
inline bool same_record(const Record& x, const Record& y) {
  return x.t == y.t && x.mode == y.mode && x.hold_reason == y.hold_reason && same_bits(x.q, y.q) &&
         same_bits(x.qd, y.qd) && same_bits(x.q_ref, y.q_ref) && same_bits(x.qd_ref, y.qd_ref) &&
         same_bits(x.tau, y.tau) && same_bits(x.T_b_e.R, y.T_b_e.R) &&
         same_bits(x.T_b_e.p, y.T_b_e.p) && same_bits(x.T_b_d.R, y.T_b_d.R) &&
         same_bits(x.T_b_d.p, y.T_b_d.p) && same_bits(x.task_error, y.task_error) &&
         same_bits(x.step_time_us, y.step_time_us);
}

// What the host saw after each step k: the command written and last_record().
struct ClosedLoopRun {
  std::vector<Command> commands;
  std::vector<Record> last_records;
  bool all_ok = true;
};

// `steps` host cycles of the E2E-01 set-up (ideal plant, ideal_world.yaml) from home, with the
// loop recording into `recorder` (nullptr: no recorder).
inline ClosedLoopRun run_closed_loop(const Setup& setup, Recorder<kN>* recorder, int steps) {
  ClosedLoopRun run;
  run.commands.reserve(static_cast<std::size_t>(steps));
  run.last_records.reserve(static_cast<std::size_t>(steps));
  dynamics::SimPlant<kN> plant(setup.model, setup.config.home_q_rad);
  SimJointInterface<kN> io(plant, 1.0 / setup.config.control_rate_hz);
  ControlLoop<kN> loop(setup.model, setup.config, recorder);
  State s;
  EXPECT_TRUE(io.read(s).is_ok());
  run.all_ok = loop.start(s).is_ok();
  for (int k = 0; k < steps; ++k) {
    Command cmd;
    EXPECT_TRUE(io.read(s).is_ok());
    run.all_ok = loop.step(s, cmd).is_ok() && run.all_ok;
    EXPECT_TRUE(io.write(cmd).is_ok());
    io.advance();
    run.commands.push_back(cmd);
    run.last_records.push_back(loop.last_record());
  }
  return run;
}

}  // namespace cobot::pipeline::test
