#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/dynamics/dynamics.hpp"

#include "control_loop_test_util.hpp"

// SimJointInterface over SimPlant: read returns the plant state stamped with the plant time
// (rounded to integer ns), write stores the command, advance() steps the plant by the period
// plus N(0, jitter) clamped to +-50 % of the period.
namespace cobot::pipeline::test {
namespace {

constexpr double kPeriod = 0.001;  // s

Vec gravity_hold(const model::Arm7& m, const Vec& q) {
  return dynamics::gravity_torque(m, q);
}

std::int64_t to_ns(double t_s) {
  return std::llround(t_s * 1e9);
}

TEST(SimJointInterface, ReadBeforeAdvanceIsTheInitialPlantState) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const Vec q0 = setup->config.home_q_rad;
  dynamics::SimPlant<kN> plant(setup->model, q0);
  SimJointInterface<kN> io(plant, kPeriod);
  State s;
  s.tau_measured.setConstant(5.0);
  ASSERT_TRUE(io.read(s).is_ok());
  EXPECT_EQ(s.t.ns, 0);
  EXPECT_EQ(s.q, q0);
  EXPECT_EQ(s.qd, Vec::Zero());
  EXPECT_EQ(s.tau_measured, Vec::Zero());
  EXPECT_TRUE(s.tau_valid);
}

// write() only stores; advance() steps the plant once by the period with that command. The
// result is bit-identical to stepping a second plant directly.
TEST(SimJointInterface, AdvanceStepsThePlantWithTheLastCommand) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const Vec q0 = setup->config.home_q_rad;
  dynamics::SimPlant<kN> plant(setup->model, q0);
  dynamics::SimPlant<kN> reference(setup->model, q0);
  SimJointInterface<kN> io(plant, kPeriod);

  Command cmd;
  cmd.tau = gravity_hold(setup->model, q0) + Vec(1.0, -2.0, 0.5, 1.5, -0.3, 0.2, 0.1);
  ASSERT_TRUE(io.write(cmd).is_ok());
  EXPECT_EQ(plant.time_s(), 0.0);  // write does not step
  EXPECT_EQ(plant.q(), q0);

  for (int k = 1; k <= 5; ++k) {
    io.advance();
    reference.step(cmd.tau, kPeriod);
    State s;
    ASSERT_TRUE(io.read(s).is_ok());
    EXPECT_EQ(plant.time_s(), reference.time_s()) << "advance " << k;
    EXPECT_EQ(s.t.ns, to_ns(reference.time_s())) << "advance " << k;
    EXPECT_EQ(s.t.ns, k * 1000000LL) << "advance " << k;
    EXPECT_EQ(s.q, reference.q()) << "advance " << k;
    EXPECT_EQ(s.qd, reference.qd()) << "advance " << k;
    EXPECT_EQ(s.tau_measured, reference.tau_applied()) << "advance " << k;
    EXPECT_TRUE(s.tau_valid);
  }
  // Ideal plant (no motor lag): the applied torque is the command.
  EXPECT_EQ(plant.tau_applied(), cmd.tau);

  // A new command replaces the old one from the next advance on.
  Command cmd2;
  cmd2.tau = gravity_hold(setup->model, plant.q());
  ASSERT_TRUE(io.write(cmd2).is_ok());
  io.advance();
  reference.step(cmd2.tau, kPeriod);
  EXPECT_EQ(plant.q(), reference.q());
  EXPECT_EQ(plant.tau_applied(), cmd2.tau);
}

// Period 1/3 ms: plant time 0.000333... s is stamped as the nearest integer ns, 333333.
TEST(SimJointInterface, StampIsPlantTimeRoundedToNanoseconds) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const double period = 1.0 / 3000.0;
  dynamics::SimPlant<kN> plant(setup->model, setup->config.home_q_rad);
  SimJointInterface<kN> io(plant, period);
  Command cmd;
  cmd.tau = gravity_hold(setup->model, setup->config.home_q_rad);
  ASSERT_TRUE(io.write(cmd).is_ok());
  io.advance();
  State s;
  ASSERT_TRUE(io.read(s).is_ok());
  EXPECT_EQ(s.t.ns, 333333);  // 333333.33 ns
  io.advance();
  ASSERT_TRUE(io.read(s).is_ok());
  EXPECT_EQ(s.t.ns, 666667);  // 666666.67 ns rounds up
  EXPECT_EQ(s.t.ns, to_ns(plant.time_s()));
}

// Collects the per-advance time steps of `n` advances.
std::vector<double> time_steps(const Setup& setup, double jitter_std, std::uint64_t seed, int n) {
  dynamics::SimPlant<kN> plant(setup.model, setup.config.home_q_rad);
  SimJointInterface<kN> io(plant, kPeriod, jitter_std, seed);
  Command cmd;
  cmd.tau = gravity_hold(setup.model, setup.config.home_q_rad);
  EXPECT_TRUE(io.write(cmd).is_ok());
  std::vector<double> dts;
  double t_prev = plant.time_s();
  for (int k = 0; k < n; ++k) {
    io.advance();
    dts.push_back(plant.time_s() - t_prev);
    t_prev = plant.time_s();
  }
  return dts;
}

TEST(SimJointInterface, JitterIsSeededAndClampedToHalfThePeriod) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  constexpr int kAdvances = 200;
  constexpr double kSlack = 1e-12;  // differences of accumulated plant times

  // No jitter: every step is the period.
  for (const double dt : time_steps(*setup, 0.0, 1, kAdvances)) {
    EXPECT_NEAR(dt, kPeriod, kSlack);
  }

  // 0.1 ms jitter: steps differ from the period, are reproducible per seed and differ
  // between seeds; all stay inside [0.5, 1.5] * period.
  const std::vector<double> a = time_steps(*setup, 1e-4, 42, kAdvances);
  const std::vector<double> b = time_steps(*setup, 1e-4, 42, kAdvances);
  const std::vector<double> c = time_steps(*setup, 1e-4, 43, kAdvances);
  EXPECT_EQ(a, b);
  EXPECT_NE(a, c);
  int off_period = 0;
  for (const double dt : a) {
    EXPECT_GE(dt, 0.5 * kPeriod - kSlack);
    EXPECT_LE(dt, 1.5 * kPeriod + kSlack);
    if (std::abs(dt - kPeriod) > 1e-6) {
      ++off_period;
    }
  }
  // P(|N(0, 0.1 ms)| <= 1 us) = 0.008 per step, so nearly all 200 steps are off the period.
  EXPECT_GT(off_period, kAdvances / 2);

  // 10 s jitter: almost every draw is clamped, to 0.5 or 1.5 periods; none outside.
  int low = 0;
  int high = 0;
  for (const double dt : time_steps(*setup, 10.0, 7, kAdvances)) {
    EXPECT_GE(dt, 0.5 * kPeriod - kSlack);
    EXPECT_LE(dt, 1.5 * kPeriod + kSlack);
    low += std::abs(dt - 0.5 * kPeriod) < 1e-9 ? 1 : 0;
    high += std::abs(dt - 1.5 * kPeriod) < 1e-9 ? 1 : 0;
  }
  EXPECT_GT(low, 0);
  EXPECT_GT(high, 0);
  EXPECT_GT(low + high, kAdvances * 9 / 10);
}

}  // namespace
}  // namespace cobot::pipeline::test
