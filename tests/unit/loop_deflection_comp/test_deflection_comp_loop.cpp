// Where the compensation applies (brief "deflection-comp" contract 4): only when
// config.deflection_comp.enabled && state.q_from_motor, the controller works on
//   q_hat_j = q_j - harmonic_spring_deflection(g_j(q), k1_j, k2_j, k3_j, t1_j, t2_j)
// with g the gravity torque of the controller's model at the measured q; qd is unchanged and the
// safety checks (soft limits) use q_hat. Otherwise the output is bit-identical to enabled = false.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/dynamics/dynamics.hpp"
#include "cobot/dynamics/harmonic_spring.hpp"
#include "cobot/dynamics/sim_plant.hpp"
#include "cobot/pipeline/control_loop.hpp"
#include "cobot/pipeline/sim_joint_interface.hpp"

#include "deflection_test_util.hpp"
#include "rigid_run.hpp"

namespace cobot::pipeline::test {
namespace {

namespace fs = std::filesystem;

// Rigid ideal plant at home behind a SimJointInterface; the loops under test all see the
// states of this plant, which is driven by `driver`.
struct RigidRig {
  explicit RigidRig(const Setup& s)
      : plant(s.plant_model, s.config.home_q_rad), io(plant, 1.0 / s.config.control_rate_hz) {}
  dynamics::SimPlant<kN> plant;
  SimJointInterface<kN> io;
};

std::unique_ptr<ControlLoop<kN>> make_loop(const Setup& s, bool enabled) {
  Config<kN> cfg = s.config;
  enable_comp(cfg);
  cfg.deflection_comp.enabled = enabled;
  return std::make_unique<ControlLoop<kN>>(s.controller_model, cfg);
}

// Loops A (enabled, flag false), B (disabled, flag false; drives the plant), C (disabled, flag
// true) give the same torque bit for bit; D (enabled, flag true) differs, so the comparison is
// not vacuous. 2 s of E2E-01 on the rigid plant.
TEST(DeflectionCompLoop, InactiveWithoutQFromMotorEvenWhenEnabled) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  RigidRig rig(*setup);
  const auto a = make_loop(*setup, true);
  const auto b = make_loop(*setup, false);
  const auto c = make_loop(*setup, false);
  const auto d = make_loop(*setup, true);
  State s;
  ASSERT_TRUE(rig.io.read(s).is_ok());
  ASSERT_FALSE(s.q_from_motor);
  for (ControlLoop<kN>* loop : {a.get(), b.get(), c.get(), d.get()}) {
    ASSERT_TRUE(loop->start(s).is_ok());
  }
  int a_diff = 0;
  int c_diff = 0;
  int d_diff = 0;
  for (int k = 0; k < 2000; ++k) {
    ASSERT_TRUE(rig.io.read(s).is_ok());
    ASSERT_FALSE(s.q_from_motor);
    Command ca;
    Command cb;
    Command cc;
    Command cd;
    ASSERT_TRUE(a->step(s, ca).is_ok());
    ASSERT_TRUE(b->step(s, cb).is_ok());
    State flagged = s;
    flagged.q_from_motor = true;
    ASSERT_TRUE(c->step(flagged, cc).is_ok());
    ASSERT_TRUE(d->step(flagged, cd).is_ok());
    a_diff += (ca.tau == cb.tau) ? 0 : 1;
    c_diff += (cc.tau == cb.tau) ? 0 : 1;
    d_diff += (cd.tau == cb.tau) ? 0 : 1;
    ASSERT_TRUE(rig.io.write(cb).is_ok());
    rig.io.advance();
  }
  EXPECT_EQ(a_diff, 0);
  EXPECT_EQ(c_diff, 0);
  EXPECT_GT(d_diff, 1000);
  EXPECT_EQ(a->mode(), b->mode());
  EXPECT_EQ(c->mode(), b->mode());
}

// The compensated loop E fed the raw (motor) q gives the same torque as an uncompensated loop F
// fed q_hat computed by the test: the loop uses exactly q - deflection(g(q)) with the
// controller-model gravity at the measured q, and qd unchanged. Both start from the same state.
// 0.5 s of E2E-01 on the rigid plant, driven by F.
TEST(DeflectionCompLoop, ControllerSeesQMinusTheDeflectionOfTheGravityTorque) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  RigidRig rig(*setup);
  const auto e = make_loop(*setup, true);
  const auto f = make_loop(*setup, false);
  State s;
  ASSERT_TRUE(rig.io.read(s).is_ok());
  ASSERT_TRUE(e->start(s).is_ok());
  ASSERT_TRUE(f->start(s).is_ok());
  double worst = 0.0;
  double shift = 0.0;
  for (int k = 0; k < 500; ++k) {
    ASSERT_TRUE(rig.io.read(s).is_ok());
    State raw = s;
    raw.q_from_motor = true;
    State hat = s;
    hat.q = compensated(setup->controller_model, s.q);
    shift = std::max(shift, (hat.q - s.q).cwiseAbs().maxCoeff());
    Command ce;
    Command cf;
    ASSERT_TRUE(e->step(raw, ce).is_ok());
    ASSERT_TRUE(f->step(hat, cf).is_ok());
    worst = std::max(worst, (ce.tau - cf.tau).cwiseAbs().maxCoeff());
    ASSERT_TRUE(rig.io.write(cf).is_ok());
    rig.io.advance();
  }
  // At home the gravity torques put joints 2 and 4 near 1e-3 rad of deflection (the
  // transmission measurement: up to 9.9e-4 rad), so the shift is real.
  EXPECT_GT(shift, 1e-4);
  EXPECT_LE(worst, 1e-9);  // N m; the same arithmetic in both paths
  EXPECT_EQ(e->mode(), LoopMode::kTracking);
}

// Soft-limit check on q_hat: a joint placed outside the soft margin by the raw q, but inside it by
// q_hat (half the deflection past the margin line), holds with comp and does not without it.
TEST(DeflectionCompLoop, SoftLimitsUseTheCompensatedPosition) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const model::Arm7& m = setup->controller_model;
  const double margin = setup->config.safety.soft_margin_rad;
  // Joint 2 (index 1), the largest gravity torque. The deflection d = deflection(g_1(q)) moves
  // q_hat = q - d towards the lower limit when d > 0, towards the upper one when d < 0.
  constexpr int kJ = 1;
  constexpr auto kJi = static_cast<std::size_t>(kJ);
  const Gear& gr = gear_of(kJ);
  const auto defl_at = [&](const Vec& q) {
    return dynamics::harmonic_spring_deflection(
        dynamics::gravity_torque(m, q)(kJ), gr.k1, gr.k2, gr.k3, gr.t1, gr.t2);
  };
  const double q_min = m.joints[kJi].q_min;
  const double q_max = m.joints[kJi].q_max;
  Vec q = setup->config.home_q_rad;
  bool placed = false;
  for (const bool lower : {true, false}) {
    // Fixed point q_j = line -/+ |d(q)| / 2 (d changes slowly with q_j; three passes are plenty).
    const double line = lower ? q_min + margin : q_max - margin;
    q(kJ) = line;
    for (int it = 0; it < 3; ++it) {
      const double d = defl_at(q);
      q(kJ) = lower ? line + (0.5 * std::abs(d)) : line - (0.5 * std::abs(d));
    }
    const double d = defl_at(q);
    // Raw q outside the margin, q_hat inside it by at least 1e-5 rad.
    const double q_hat = q(kJ) - d;
    if (std::abs(d) > 1e-4 && (lower ? (q_hat < line - 1e-5) : (q_hat > line + 1e-5))) {
      placed = true;
      break;
    }
  }
  ASSERT_TRUE(placed) << "no limit of joint 2 where the deflection points into the margin";

  for (const bool enabled : {false, true}) {
    RigidRig rig(*setup);
    const auto loop = make_loop(*setup, enabled);
    State s;
    ASSERT_TRUE(rig.io.read(s).is_ok());
    ASSERT_TRUE(loop->start(s).is_ok());
    ASSERT_TRUE(rig.io.read(s).is_ok());
    Command cmd;
    ASSERT_TRUE(loop->step(s, cmd).is_ok());
    ASSERT_EQ(loop->mode(), LoopMode::kTracking);
    rig.io.advance();
    ASSERT_TRUE(rig.io.read(s).is_ok());
    s.q = q;
    s.qd.setZero();
    s.q_from_motor = true;
    static_cast<void>(loop->step(s, cmd));
    if (enabled) {
      EXPECT_EQ(loop->mode(), LoopMode::kHolding);
      EXPECT_EQ(loop->hold_reason(), HoldReason::kJointLimit);
    } else {
      EXPECT_EQ(loop->mode(), LoopMode::kTracking);
      EXPECT_EQ(loop->hold_reason(), HoldReason::kNone);
    }
  }
}

std::vector<std::vector<double>> parse(const std::string& csv) {
  std::vector<std::vector<double>> rows;
  std::istringstream lines(csv);
  std::string line;
  while (std::getline(lines, line)) {
    std::vector<double> row;
    std::istringstream cells(line);
    std::string cell;
    while (std::getline(cells, cell, ',')) {
      row.push_back(std::strtod(cell.c_str(), nullptr));
    }
    rows.push_back(row);
  }
  return rows;
}

// Rigid plant, link source, comp enabled with the catalog values: 3 s of E2E-01 equal to the run
// recorded before the transmission feature (tests/data/transmission/rigid_e2e01_reference.csv),
// bit for bit on x86-64 (1e-9 relative elsewhere, as in the transmission tests).
TEST(DeflectionCompLoop, EnabledCompWithTheLinkSourceReproducesTheRigidReference) {
  auto setup = dynamics::test::load_e2e01();
  ASSERT_TRUE(setup);
  enable_comp(setup->config);
  int flagged = 0;
  const std::string actual = dynamics::test::run_e2e01(
      *setup,
      dynamics::PlantEffects<kN>{},
      3000,
      [&](int /*k*/, const dynamics::SimPlant<kN>&, const State& s, const Command&) {
        flagged += s.q_from_motor ? 1 : 0;
      });
  EXPECT_EQ(flagged, 0);
  const fs::path out = fs::path(COBOT_TEST_OUTPUT_DIR) / "loop_deflection_comp";
  fs::create_directories(out);
  std::ofstream(out / "rigid_comp_enabled.csv", std::ios::binary) << actual;
  const std::ifstream in(
      fs::path(COBOT_REPO_DIR) / "tests/data/transmission/rigid_e2e01_reference.csv",
      std::ios::binary);
  ASSERT_TRUE(in.good());
  std::stringstream ss;
  ss << in.rdbuf();
  const auto want = parse(ss.str());
  const auto got = parse(actual);
  ASSERT_EQ(want.size(), 30U);
  ASSERT_EQ(got.size(), want.size());
  for (std::size_t r = 0; r < want.size(); ++r) {
    ASSERT_EQ(got[r].size(), want[r].size()) << "row " << r;
    for (std::size_t c = 0; c < want[r].size(); ++c) {
#if defined(__x86_64__)
      EXPECT_EQ(got[r][c], want[r][c]) << "row " << r << " column " << c;
#else
      EXPECT_LE(std::abs(got[r][c] - want[r][c]), 1e-9 * std::max(1.0, std::abs(want[r][c])))
          << "row " << r << " column " << c;
#endif
    }
  }
}

}  // namespace
}  // namespace cobot::pipeline::test
