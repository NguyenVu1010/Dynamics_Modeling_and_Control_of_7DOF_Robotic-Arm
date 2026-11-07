#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/lie.hpp"
#include "cobot/control/algorithm.hpp"
#include "cobot/imperfect/imperfect_joint_interface.hpp"
#include "cobot/imperfect/perturb.hpp"
#include "cobot/pipeline/overrides.hpp"

#include "control_loop_test_util.hpp"
#include "registry_util.hpp"

// ControlLoop with the tracking law chosen by cobot.control.algorithm (brief controller-registry
// 5). Runs are wired as bench/run.cpp wires one: the URDF model perturbed by the profile drives a
// SimPlant with the profile's plant effects, behind SimJointInterface (jitter, overrun) and
// ImperfectJointInterface; the loop runs on the controller model with the plant truth probe.
namespace cobot::pipeline::test {
namespace {

namespace fs = std::filesystem;

using control::registry_test::bits_equal;

constexpr double kUsPerS = 1e6;

// The full 12 s runs take about 80 s each in the unoptimised sanitizer build; there the
// bit-identity comparison runs the first 3 s of the trajectory (the Release `core` job runs all
// 12000 rows).
#if defined(__OPTIMIZE__)
constexpr int kBitIdentityCycles = 12000;  // (10 s + 2 s) * 1000 Hz
#else
constexpr int kBitIdentityCycles = 3000;
#endif

struct BenchRun {
  std::vector<Record> rows;
  model::Arm7 nominal{};
  Config<kN> config{};
  control::Algorithm algorithm = control::Algorithm::kComputedTorque;  // loop.algorithm()
};

// duration_s + hold_s of the config (12 s = 12000 cycles at 1 kHz for the default trajectory),
// or at most `max_cycles` when > 0.
std::optional<BenchRun> bench_run(const char* yaml,
                                  const std::vector<KeyOverride>& overrides,
                                  const char* profile,
                                  std::uint64_t seed,
                                  int max_cycles = 0) {
  const fs::path repo = COBOT_REPO_DIR;
  const auto cfg = overrides.empty() ? load_config<kN>(repo / yaml)
                                     : load_config_with<kN>(repo / yaml, overrides);
  if (!cfg) {
    ADD_FAILURE() << yaml << ": " << cfg.reason();
    return std::nullopt;
  }
  auto imp =
      load_imperfections<kN>(repo / "config/imperfections" / (std::string(profile) + ".yaml"));
  if (!imp) {
    ADD_FAILURE() << profile << ": " << imp.reason();
    return std::nullopt;
  }
  imperfect::ImperfectionConfig<kN>& p = imp.value();
  p.seed = seed;
  const Config<kN>& c = cfg.value();
  const auto chain = model::parse_urdf_chain(c.urdf_path, c.base_link, c.tip_link);
  const auto urdf = chain ? model::make_model<kN>(chain.value(), c.gravity_b)
                          : common::Result<model::Arm7>::error(chain.reason());
  if (!urdf) {
    ADD_FAILURE() << urdf.reason();
    return std::nullopt;
  }
  BenchRun run{{}, controller_model<kN>(urdf.value(), c), c};
  const double dt = 1.0 / c.control_rate_hz;
  const auto full =
      static_cast<int>(std::lround((c.trajectory.duration_s + c.trajectory.hold_s) / dt));
  const int cycles = max_cycles > 0 ? std::min(full, max_cycles) : full;
  const model::Arm7 plant_model = imperfect::perturb_model(urdf.value(), p.model, p.seed);
  dynamics::SimPlant<kN> plant(plant_model, c.home_q_rad, imperfect::plant_effects(p));
  SimJointInterface<kN> sim(plant,
                            dt,
                            p.timing.period_jitter_std_us / kUsPerS,
                            p.seed,
                            p.timing.overrun_prob,
                            p.timing.overrun_cycles);
  SimTruthProbe<kN> probe(plant);
  imperfect::ImperfectJointInterface<kN> io(sim, p, dt, false, c.limits.peak_torque_nm);
  const auto loop = std::make_unique<ControlLoop<kN>>(run.nominal, c);
  run.algorithm = loop->algorithm();
  State s;
  Command cmd;
  static_cast<void>(io.read(s));
  EXPECT_TRUE(loop->start(s).is_ok());
  run.rows.reserve(static_cast<std::size_t>(cycles));
  for (int k = 0; k < cycles; ++k) {
    static_cast<void>(io.read(s));
    static_cast<void>(loop->step(s, cmd, probe.sample()));
    static_cast<void>(io.write(cmd));
    sim.advance();
    run.rows.push_back(loop->last_record());
  }
  return run;
}

// Name of the first LogRecord field whose bits differ, empty when every field is equal.
std::string first_difference(const Record& a, const Record& b) {
  struct Field {
    const char* name;
    bool equal;
  };
  const Field fields[] = {
      {"t", a.t.ns == b.t.ns},
      {"mode", a.mode == b.mode},
      {"hold_reason", a.hold_reason == b.hold_reason},
      {"cmd_mode", a.cmd_mode == b.cmd_mode},
      {"step_time_us", bits_equal(a.step_time_us, b.step_time_us)},
      {"q", bits_equal(a.q, b.q)},
      {"qd", bits_equal(a.qd, b.qd)},
      {"q_ref", bits_equal(a.q_ref, b.q_ref)},
      {"qd_ref", bits_equal(a.qd_ref, b.qd_ref)},
      {"tau", bits_equal(a.tau, b.tau)},
      {"T_b_e.R", bits_equal(a.T_b_e.R, b.T_b_e.R)},
      {"T_b_e.p", bits_equal(a.T_b_e.p, b.T_b_e.p)},
      {"T_b_d.R", bits_equal(a.T_b_d.R, b.T_b_d.R)},
      {"T_b_d.p", bits_equal(a.T_b_d.p, b.T_b_d.p)},
      {"task_error", bits_equal(a.task_error, b.task_error)},
      {"qdd_ref", bits_equal(a.qdd_ref, b.qdd_ref)},
      {"q_raw", bits_equal(a.q_raw, b.q_raw)},
      {"q_true", bits_equal(a.q_true, b.q_true)},
      {"qd_true", bits_equal(a.qd_true, b.qd_true)},
      {"qdd_true", bits_equal(a.qdd_true, b.qdd_true)},
      {"q_motor_true", bits_equal(a.q_motor_true, b.q_motor_true)},
      {"tau_motor", bits_equal(a.tau_motor, b.tau_motor)},
      {"tau_friction", bits_equal(a.tau_friction, b.tau_friction)},
  };
  for (const Field& f : fields) {
    if (!f.equal) {
      return f.name;
    }
  }
  return "";
}

// Default trajectory, realistic profile, seed 7: config/default.yaml as loaded against the same
// file with computed_torque selected explicitly through the override path.
TEST(ControlLoopRegistry, ExplicitComputedTorqueIsBitIdentical) {
  const auto a = bench_run("config/default.yaml", {}, "realistic", 7, kBitIdentityCycles);
  ASSERT_TRUE(a);
  const auto b = bench_run("config/default.yaml",
                           {{"config.control.algorithm", std::string("computed_torque")}},
                           "realistic",
                           7,
                           kBitIdentityCycles);
  ASSERT_TRUE(b);
  EXPECT_EQ(a->config.algorithm, control::Algorithm::kComputedTorque);
  EXPECT_EQ(b->config.algorithm, control::Algorithm::kComputedTorque);
  EXPECT_EQ(a->algorithm, control::Algorithm::kComputedTorque);
  EXPECT_EQ(b->algorithm, control::Algorithm::kComputedTorque);
  ASSERT_EQ(a->rows.size(), static_cast<std::size_t>(kBitIdentityCycles));
  ASSERT_EQ(b->rows.size(), a->rows.size());
  int bad = 0;
  for (std::size_t k = 0; k < a->rows.size() && bad < 5; ++k) {
    const std::string field = first_difference(a->rows[k], b->rows[k]);
    if (!field.empty()) {
      ADD_FAILURE() << "row " << k << ": field " << field << " differs";
      ++bad;
    }
  }
  EXPECT_EQ(bad, 0);
}

// max over joints of RMS(q_ref - q) over the tracking and finished rows, rad.
double rms_joint_err(const std::vector<Record>& rows) {
  Vec sum = Vec::Zero();
  int n = 0;
  for (const Record& r : rows) {
    if (r.mode == LoopMode::kTracking || r.mode == LoopMode::kFinished) {
      sum += (r.q_ref - r.q).cwiseAbs2();
      ++n;
    }
  }
  return n == 0 ? 0.0 : std::sqrt((sum / n).maxCoeff());
}

TEST(ControlLoopRegistry, PdGravityTracksTheIdealPlant) {
  const char* yaml = "tests/e2e/configs/ideal_world.yaml";
  const auto pd = bench_run(yaml,
                            {{"config.control.algorithm", std::string("pd_gravity")},
                             {"config.pd_gravity.kp", 6400.0},
                             {"config.pd_gravity.kd", 160.0}},
                            "ideal",
                            1);
  ASSERT_TRUE(pd);
  const auto ct = bench_run(yaml, {}, "ideal", 1);
  ASSERT_TRUE(ct);
  ASSERT_EQ(pd->config.algorithm, control::Algorithm::kPdGravity);
  EXPECT_EQ(pd->algorithm, control::Algorithm::kPdGravity);
  EXPECT_EQ(ct->algorithm, control::Algorithm::kComputedTorque);
  ASSERT_FALSE(pd->rows.empty());
  for (std::size_t k = 0; k < pd->rows.size(); ++k) {
    const Record& r = pd->rows[k];
    ASSERT_NE(r.mode, LoopMode::kHolding) << "row " << k << " (" << to_string(r.hold_reason) << ")";
    ASSERT_NE(r.mode, LoopMode::kFault) << "row " << k;
    ASSERT_TRUE(torque_ok(pd->nominal, r.tau)) << "row " << k << ": " << r.tau.transpose();
  }
  const Record& last = pd->rows.back();
  const Pose goal = goal_pose(pd->config);
  const double pos_err = (last.T_b_e.p - goal.p).norm();
  const Mat3 R_err = last.T_b_e.R * goal.R.transpose();
  const double rot_err = common::so3_log(R_err).norm();
  EXPECT_LT(pos_err, 1e-4) << "final TCP position error, m";
  EXPECT_LT(rot_err, 1e-3) << "final TCP rotation error, rad";
  const double rms_pd = rms_joint_err(pd->rows);
  const double rms_ct = rms_joint_err(ct->rows);
  std::printf(
      "[ registry ] rms_joint_err pd_gravity %.6e rad, computed_torque %.6e rad\n", rms_pd, rms_ct);
  EXPECT_LT(rms_pd, 0.02);
  EXPECT_GT(rms_pd, rms_ct) << "the law really changed";
}

}  // namespace
}  // namespace cobot::pipeline::test
