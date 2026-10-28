#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <new>
#include <random>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "cobot/common/result.hpp"
#include "cobot/common/types.hpp"
#include "cobot/hal/joint_interface.hpp"
#include "cobot/imperfect/config.hpp"
#include "cobot/imperfect/imperfect_joint_interface.hpp"

// Counts global operator new calls while g_count_allocs is set. Eigen's own allocator is
// checked by EIGEN_RUNTIME_NO_MALLOC (test code only); this counter also catches allocations
// made inside the compiled imperfect library.
namespace {
std::atomic<bool> g_count_allocs{false};
std::atomic<long> g_alloc_count{0};
}  // namespace

void* operator new(std::size_t size) {
  if (g_count_allocs.load()) {
    g_alloc_count.fetch_add(1);
  }
  void* p = std::malloc(size == 0 ? 1 : size);
  if (p == nullptr) {
    throw std::bad_alloc();
  }
  return p;
}
void operator delete(void* p) noexcept {
  std::free(p);
}
void operator delete(void* p, std::size_t /*size*/) noexcept {
  std::free(p);
}

namespace cobot::imperfect {
namespace {

constexpr int kN = 7;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDt = 0.001;  // control period, s
using Vec = common::VecN<kN>;
using State = JointState<kN>;
using Cmd = JointCommand<kN>;
using Config = ImperfectionConfig<kN>;

// Scripted inner interface: read() returns states[i] on call i (the last one repeated once the
// script ends); write() records every command.
class FakeInner final : public JointInterface<kN> {
 public:
  Status read(State& out) override {
    if (fail) {
      return Status::error("fake inner: no fresh sample");
    }
    out = states[next < states.size() ? next : states.size() - 1];
    ++next;
    return Status::ok();
  }
  Status write(const Cmd& cmd) override {
    written.push_back(cmd);
    return Status::ok();
  }
  std::vector<State> states{State{}};
  std::size_t next = 0;
  bool fail = false;
  std::vector<Cmd> written;
};

Vec uniform_vec(std::mt19937_64& rng, double lo, double hi) {
  std::uniform_real_distribution<double> u(lo, hi);
  Vec v;
  for (int j = 0; j < kN; ++j) {
    v[j] = u(rng);
  }
  return v;
}

std::vector<State> random_states(int n, std::uint64_t seed) {
  std::mt19937_64 rng(seed);
  std::vector<State> s(static_cast<std::size_t>(n));
  for (int k = 0; k < n; ++k) {
    State& st = s[static_cast<std::size_t>(k)];
    st.t = common::Timestamp{static_cast<std::int64_t>(k + 1) * 1000000};
    st.q = uniform_vec(rng, -3.0, 3.0);
    st.qd = uniform_vec(rng, -2.0, 2.0);
    st.tau_measured = uniform_vec(rng, -50.0, 50.0);
    st.tau_valid = (k % 2) == 0;
  }
  return s;
}

State constant_state(const Vec& q, const Vec& qd) {
  State s;
  s.q = q;
  s.qd = qd;
  return s;
}

Cmd make_cmd(const Vec& tau) {
  Cmd c;
  c.tau = tau;
  return c;
}

void expect_vec_eq(const Vec& a, const Vec& b) {
  for (int j = 0; j < kN; ++j) {
    EXPECT_EQ(a[j], b[j]) << "joint " << j;
  }
}

// Multiple of `step` up to floating-point rounding of the division.
bool is_multiple(double x, double step) {
  const double r = x / step;
  return std::abs(r - std::round(r)) < 1e-9;
}

TEST(ImperfectIo, IdealConfigIsBitExactPassThrough) {
  FakeInner inner;
  inner.states = random_states(1000, 7);
  inner.written.reserve(1000);
  ImperfectJointInterface<kN> io(inner, Config{}, kDt);
  expect_vec_eq(io.q_offset_rad(), Vec::Zero());
  expect_vec_eq(io.torque_scale(), Vec::Ones());
  std::mt19937_64 rng(8);
  for (int k = 0; k < 1000; ++k) {
    const State& in = inner.states[static_cast<std::size_t>(k)];
    State out;
    ASSERT_TRUE(io.read(out).is_ok());
    EXPECT_EQ(out.t, in.t);
    expect_vec_eq(out.q, in.q);
    expect_vec_eq(out.qd, in.qd);
    expect_vec_eq(out.tau_measured, in.tau_measured);
    EXPECT_EQ(out.tau_valid, in.tau_valid);
    const Cmd cmd = make_cmd(uniform_vec(rng, -80.0, 80.0));
    ASSERT_TRUE(io.write(cmd).is_ok());
    ASSERT_EQ(inner.written.size(), static_cast<std::size_t>(k + 1));
    expect_vec_eq(inner.written.back().tau, cmd.tau);
  }
}

TEST(ImperfectIo, QuantisationTo12Bits) {
  FakeInner inner;
  inner.states = random_states(1000, 11);
  Config cfg;
  cfg.sensor.encoder_bits = Vec::Constant(12.0);
  ImperfectJointInterface<kN> io(inner, cfg, kDt);
  const double step = 2.0 * kPi / 4096.0;  // 2^12 = 4096 -> 1.5339807878856412e-3 rad
  for (int k = 0; k < 1000; ++k) {
    State out;
    ASSERT_TRUE(io.read(out).is_ok());
    const Vec& q = inner.states[static_cast<std::size_t>(k)].q;
    for (int j = 0; j < kN; ++j) {
      EXPECT_TRUE(is_multiple(out.q[j], step)) << out.q[j];
      EXPECT_LE(std::abs(out.q[j] - q[j]), step / 2.0 + 1e-15);
    }
  }
}

TEST(ImperfectIo, OffsetDrawnOnceAndSeeded) {
  FakeInner inner;
  inner.states = random_states(100, 13);
  Config cfg;
  cfg.sensor.q_offset_std_rad = Vec::Constant(0.01);
  cfg.seed = 42;
  ImperfectJointInterface<kN> io(inner, cfg, kDt);
  const Vec offset = io.q_offset_rad();
  EXPECT_GT(offset.cwiseAbs().maxCoeff(), 0.0);
  for (int k = 0; k < 100; ++k) {
    State out;
    ASSERT_TRUE(io.read(out).is_ok());
    const Vec diff = out.q - inner.states[static_cast<std::size_t>(k)].q;
    EXPECT_LT((diff - offset).cwiseAbs().maxCoeff(), 1e-12) << "cycle " << k;
  }
  FakeInner inner2;
  const ImperfectJointInterface<kN> same(inner2, cfg, kDt);
  expect_vec_eq(same.q_offset_rad(), offset);
  cfg.seed = 43;
  const ImperfectJointInterface<kN> other(inner2, cfg, kDt);
  EXPECT_GT((other.q_offset_rad() - offset).cwiseAbs().maxCoeff(), 0.0);
}

TEST(ImperfectIo, PositionNoiseStatistics) {
  const Vec q0 = Vec::Constant(0.3);
  FakeInner inner;
  inner.states = {constant_state(q0, Vec::Zero())};
  Config cfg;
  cfg.sensor.q_noise_std_rad = Vec::Constant(1e-3);
  cfg.seed = 2026;
  ImperfectJointInterface<kN> io(inner, cfg, kDt);
  constexpr int kSamples = 20000;
  Vec sum = Vec::Zero();
  Vec sum_sq = Vec::Zero();
  for (int k = 0; k < kSamples; ++k) {
    State out;
    ASSERT_TRUE(io.read(out).is_ok());
    const Vec e = out.q - q0;
    sum += e;
    sum_sq += e.cwiseProduct(e);
  }
  // 3 sigma / sqrt(n) = 3 * 1e-3 / sqrt(20000) = 3e-3 / 141.42 = 2.1213e-5 rad
  const double mean_tol = 3.0 * 1e-3 / std::sqrt(static_cast<double>(kSamples));
  for (int j = 0; j < kN; ++j) {
    const double mean = sum[j] / kSamples;
    const double var = (sum_sq[j] - kSamples * mean * mean) / (kSamples - 1);
    EXPECT_LT(std::abs(mean), mean_tol) << "joint " << j;
    EXPECT_NEAR(std::sqrt(var), 1e-3, 0.05 * 1e-3) << "joint " << j;  // within 5 %
  }
}

TEST(ImperfectIo, SensorDelayThreeCycles) {
  FakeInner inner;
  inner.states = random_states(50, 17);
  Config cfg;
  cfg.sensor.sensor_delay_cycles = 3;
  ImperfectJointInterface<kN> io(inner, cfg, kDt);
  for (int k = 0; k < 50; ++k) {
    State out;
    ASSERT_TRUE(io.read(out).is_ok());
    // Before 3 earlier reads exist the oldest available (cycle 0) is used.
    const State& want = inner.states[static_cast<std::size_t>(k >= 3 ? k - 3 : 0)];
    EXPECT_EQ(out.t, want.t) << "cycle " << k;
    expect_vec_eq(out.q, want.q);
    expect_vec_eq(out.qd, want.qd);
    expect_vec_eq(out.tau_measured, want.tau_measured);
  }
}

TEST(ImperfectIo, VelocityFromDiffOnRampWithHighCutoff) {
  FakeInner inner;
  inner.states.clear();
  for (int k = 0; k < 20; ++k) {
    inner.states.push_back(constant_state(Vec::Constant(0.5 * k * kDt), Vec::Zero()));
  }
  Config cfg;
  cfg.sensor.qd_from_diff = true;
  cfg.sensor.qd_filter_hz = Vec::Constant(1e6);
  ImperfectJointInterface<kN> io(inner, cfg, kDt);
  // alpha = dt / (dt + 1/(2 pi 1e6)) = 1e-3 / (1e-3 + 1.5915e-7); 1 - alpha = 1.5913e-4.
  // Read 0: raw 0 -> qd 0. Read k >= 1: raw 0.5, qd_k = 0.5 (1 - (1 - alpha)^k):
  // k = 1: error 8.0e-5; k = 2: 0.5 * 2.53e-8 = 1.27e-8; k = 3: 0.5 * 4.03e-12 = 2.0e-12.
  // So qd is within 1e-9 of 0.5 from read 3 on (two cycles after the first difference).
  for (int k = 0; k < 20; ++k) {
    State out;
    ASSERT_TRUE(io.read(out).is_ok());
    if (k == 0) {
      expect_vec_eq(out.qd, Vec::Zero());
    }
    if (k >= 3) {
      EXPECT_LT((out.qd - Vec::Constant(0.5)).cwiseAbs().maxCoeff(), 1e-9) << "read " << k;
    }
  }
}

TEST(ImperfectIo, VelocityFromDiffStepFollowsFirstOrderFilter) {
  FakeInner inner;
  inner.states = {constant_state(Vec::Zero(), Vec::Zero()),
                  constant_state(Vec::Constant(1.0), Vec::Zero())};  // step at read 1, then held
  Config cfg;
  cfg.sensor.qd_from_diff = true;
  cfg.sensor.qd_filter_hz = Vec::Constant(10.0);
  ImperfectJointInterface<kN> io(inner, cfg, kDt);
  // alpha = 1e-3 / (1e-3 + 1/(2 pi 10)) = 1e-3 / (1e-3 + 1.59155e-2) = 5.9117e-2.
  // raw = 0 at read 0, (1 - 0) / 1e-3 = 1000 at read 1, 0 after; f += alpha (raw - f),
  // so qd_1 = 59.117 and qd_k = 59.117 (1 - alpha)^(k-1).
  const double alpha = kDt / (kDt + 1.0 / (2.0 * kPi * 10.0));
  double f = 0.0;
  for (int k = 0; k < 50; ++k) {
    State out;
    ASSERT_TRUE(io.read(out).is_ok());
    const double raw = k == 1 ? 1.0 / kDt : 0.0;
    f += alpha * (raw - f);
    EXPECT_LT((out.qd - Vec::Constant(f)).cwiseAbs().maxCoeff(), 1e-12) << "read " << k;
  }
}

TEST(ImperfectIo, CommandDelayTwoCycles) {
  FakeInner inner;
  Config cfg;
  cfg.actuator.command_delay_cycles = 2;
  ImperfectJointInterface<kN> io(inner, cfg, kDt);
  std::mt19937_64 rng(19);
  std::vector<Cmd> sent;
  for (int k = 0; k < 30; ++k) {
    State out;
    ASSERT_TRUE(io.read(out).is_ok());
    sent.push_back(make_cmd(uniform_vec(rng, -20.0, 20.0)));
    ASSERT_TRUE(io.write(sent.back()).is_ok());
    ASSERT_EQ(inner.written.size(), static_cast<std::size_t>(k + 1));
    const Vec want = k < 2 ? Vec::Zero() : sent[static_cast<std::size_t>(k - 2)].tau;
    expect_vec_eq(inner.written.back().tau, want);
  }
}

TEST(ImperfectIo, TorqueScaleConstantPerJointAndSeeded) {
  Config cfg;
  cfg.actuator.torque_scale_std = Vec::Constant(0.05);
  cfg.seed = 5;
  FakeInner inner;
  ImperfectJointInterface<kN> io(inner, cfg, kDt);
  const Vec scale = io.torque_scale();
  EXPECT_GT((scale - Vec::Ones()).cwiseAbs().maxCoeff(), 0.0);
  std::mt19937_64 rng(23);
  for (int k = 0; k < 100; ++k) {
    State out;
    ASSERT_TRUE(io.read(out).is_ok());
    const Vec tau = uniform_vec(rng, 1.0, 30.0);
    ASSERT_TRUE(io.write(make_cmd(tau)).is_ok());
    const Vec ratio = inner.written.back().tau.cwiseQuotient(tau);
    EXPECT_LT((ratio - scale).cwiseAbs().maxCoeff(), 1e-12) << "cycle " << k;
  }
  FakeInner inner2;
  const ImperfectJointInterface<kN> same(inner2, cfg, kDt);
  expect_vec_eq(same.torque_scale(), scale);
  cfg.seed = 6;
  const ImperfectJointInterface<kN> other(inner2, cfg, kDt);
  EXPECT_GT((other.torque_scale() - scale).cwiseAbs().maxCoeff(), 0.0);
}

TEST(ImperfectIo, TorqueResolutionRounding) {
  Config cfg;
  cfg.actuator.torque_cmd_resolution_nm = Vec::Constant(0.1);
  FakeInner inner;
  ImperfectJointInterface<kN> io(inner, cfg, kDt);
  std::mt19937_64 rng(29);
  for (int k = 0; k < 500; ++k) {
    const Vec tau = uniform_vec(rng, -40.0, 40.0);
    ASSERT_TRUE(io.write(make_cmd(tau)).is_ok());
    for (int j = 0; j < kN; ++j) {
      const double w = inner.written.back().tau[j];
      EXPECT_TRUE(is_multiple(w, 0.1)) << w;
      EXPECT_LE(std::abs(w - tau[j]), 0.05 + 1e-12);
    }
  }
}

TEST(ImperfectIo, RipplePhaseFromTrueJointAngle) {
  Vec q = Vec::Constant(kPi / 12.0);
  q[0] = 0.0;
  q[2] = -kPi / 12.0;
  FakeInner inner;
  inner.states = {constant_state(q, Vec::Zero())};
  Config cfg;
  cfg.actuator.torque_ripple_frac = Vec::Constant(0.1);  // ripple_periods_per_rev default 6
  cfg.sensor.q_noise_std_rad = Vec::Constant(0.01);      // measured q differs from true q
  ImperfectJointInterface<kN> io(inner, cfg, kDt);
  State out;
  ASSERT_TRUE(io.read(out).is_ok());
  const Vec tau = Vec::LinSpaced(1.0, 7.0);
  ASSERT_TRUE(io.write(make_cmd(tau)).is_ok());
  // factor = 1 + 0.1 sin(6 q): q = pi/12 -> sin(pi/2) = 1 -> 1.1; q = 0 -> 1; q = -pi/12 -> 0.9
  Vec factor = Vec::Constant(1.1);
  factor[0] = 1.0;
  factor[2] = 0.9;
  const Vec want = tau.cwiseProduct(factor);
  EXPECT_LT((inner.written.back().tau - want).cwiseAbs().maxCoeff(), 1e-12);
}

TEST(ImperfectIo, MotorLagStepResponse) {
  Config cfg;
  cfg.actuator.motor_time_constant_s = Vec::Constant(0.01);
  FakeInner inner;
  ImperfectJointInterface<kN> plain(inner, cfg, kDt);  // plant effects off: lag ignored
  ASSERT_TRUE(plain.write(make_cmd(Vec::Ones())).is_ok());
  expect_vec_eq(inner.written.back().tau, Vec::Ones());

  ImperfectJointInterface<kN> io(inner, cfg, kDt, true);
  // dt / tau_m = 1e-3 / 1e-2 = 0.1; lag_k = lag_{k-1} + 0.1 (1 - lag_{k-1}) from lag = 0,
  // so written_k = 1 - 0.9^(k+1): 0.1, 0.19, 0.271, ...
  for (int k = 0; k < 100; ++k) {
    State out;
    ASSERT_TRUE(io.read(out).is_ok());
    ASSERT_TRUE(io.write(make_cmd(Vec::Ones())).is_ok());
    const double want = 1.0 - std::pow(0.9, k + 1);
    EXPECT_LT((inner.written.back().tau - Vec::Constant(want)).cwiseAbs().maxCoeff(), 1e-12)
        << "write " << k;
  }
}

TEST(ImperfectIo, FrictionFromLastMeasuredVelocity) {
  Vec qd = Vec::Constant(0.2);
  qd[1] = -0.2;
  qd[4] = -0.2;
  FakeInner inner;
  inner.states = {constant_state(Vec::Zero(), qd)};
  Config cfg;
  cfg.friction.coulomb_nm = Vec::Constant(0.5);
  cfg.friction.viscous_nm_s_rad = Vec::Constant(2.0);
  ImperfectJointInterface<kN> plain(inner, cfg, kDt);  // plant effects off: no friction
  State out;
  ASSERT_TRUE(plain.read(out).is_ok());
  const Vec tau = Vec::Constant(3.0);
  ASSERT_TRUE(plain.write(make_cmd(tau)).is_ok());
  expect_vec_eq(inner.written.back().tau, tau);

  ImperfectJointInterface<kN> io(inner, cfg, kDt, true);
  ASSERT_TRUE(io.read(out).is_ok());
  ASSERT_TRUE(io.write(make_cmd(tau)).is_ok());
  // 0.5 sign(qd) + 2 qd = +-(0.5 + 0.4) = +-0.9 -> written 2.1 (qd > 0) or 3.9 (qd < 0)
  Vec want = Vec::Constant(2.1);
  want[1] = 3.9;
  want[4] = 3.9;
  EXPECT_LT((inner.written.back().tau - want).cwiseAbs().maxCoeff(), 1e-12);
}

TEST(ImperfectIo, InnerReadErrorPropagatesAndLeavesOutputUntouched) {
  FakeInner inner;
  inner.fail = true;
  Config cfg;
  cfg.sensor.q_noise_std_rad = Vec::Constant(1e-3);
  ImperfectJointInterface<kN> io(inner, cfg, kDt);
  State out;
  out.t = common::Timestamp{123};
  out.q = Vec::Constant(9.0);
  out.qd = Vec::Constant(-9.0);
  out.tau_measured = Vec::Constant(4.0);
  out.tau_valid = true;
  const Status s = io.read(out);
  EXPECT_FALSE(s.is_ok());
  EXPECT_EQ(s.reason(), "fake inner: no fresh sample");
  EXPECT_EQ(out.t, common::Timestamp{123});
  expect_vec_eq(out.q, Vec::Constant(9.0));
  expect_vec_eq(out.qd, Vec::Constant(-9.0));
  expect_vec_eq(out.tau_measured, Vec::Constant(4.0));
  EXPECT_TRUE(out.tau_valid);
}

TEST(ImperfectIo, ConstructorRejectsBadDelayAndPeriod) {
  FakeInner inner;
  Config cfg;
  cfg.sensor.sensor_delay_cycles = kMaxDelayCycles;
  cfg.actuator.command_delay_cycles = kMaxDelayCycles;
  EXPECT_NO_THROW(ImperfectJointInterface<kN>(inner, cfg, kDt));
  for (const int bad : {33, -1}) {
    Config s;
    s.sensor.sensor_delay_cycles = bad;
    EXPECT_THROW(ImperfectJointInterface<kN>(inner, s, kDt), std::invalid_argument) << bad;
    Config c;
    c.actuator.command_delay_cycles = bad;
    EXPECT_THROW(ImperfectJointInterface<kN>(inner, c, kDt), std::invalid_argument) << bad;
  }
  EXPECT_THROW(ImperfectJointInterface<kN>(inner, Config{}, 0.0), std::invalid_argument);
  EXPECT_THROW(ImperfectJointInterface<kN>(inner, Config{}, -kDt), std::invalid_argument);
}

Config full_config(std::uint64_t seed) {
  Config cfg;
  cfg.sensor.encoder_bits = Vec::Constant(16.0);
  cfg.sensor.q_noise_std_rad = Vec::Constant(1e-4);
  cfg.sensor.q_offset_std_rad = Vec::Constant(1e-3);
  cfg.sensor.qd_from_diff = true;
  cfg.sensor.qd_filter_hz = Vec::Constant(100.0);
  cfg.sensor.sensor_delay_cycles = 2;
  cfg.actuator.torque_scale_std = Vec::Constant(0.02);
  cfg.actuator.torque_ripple_frac = Vec::Constant(0.01);
  cfg.actuator.torque_cmd_resolution_nm = Vec::Constant(0.01);
  cfg.actuator.motor_time_constant_s = Vec::Constant(0.005);
  cfg.actuator.command_delay_cycles = 1;
  cfg.friction.coulomb_nm = Vec::Constant(0.3);
  cfg.friction.viscous_nm_s_rad = Vec::Constant(0.1);
  cfg.seed = seed;
  return cfg;
}

TEST(ImperfectIo, FullConfigIsReproducibleBySeed) {
  FakeInner a;
  FakeInner b;
  a.states = random_states(200, 31);
  b.states = a.states;
  ImperfectJointInterface<kN> io_a(a, full_config(77), kDt, true);
  ImperfectJointInterface<kN> io_b(b, full_config(77), kDt, true);
  std::mt19937_64 rng(37);
  for (int k = 0; k < 200; ++k) {
    State out_a;
    State out_b;
    ASSERT_TRUE(io_a.read(out_a).is_ok());
    ASSERT_TRUE(io_b.read(out_b).is_ok());
    expect_vec_eq(out_a.q, out_b.q);
    expect_vec_eq(out_a.qd, out_b.qd);
    const Cmd cmd = make_cmd(uniform_vec(rng, -10.0, 10.0));
    ASSERT_TRUE(io_a.write(cmd).is_ok());
    ASSERT_TRUE(io_b.write(cmd).is_ok());
    expect_vec_eq(a.written.back().tau, b.written.back().tau);
  }
}

TEST(ImperfectIo, ReadAndWriteDoNotAllocate) {
  FakeInner inner;
  inner.states = random_states(200, 41);
  inner.written.reserve(200);
  ImperfectJointInterface<kN> io(inner, full_config(3), kDt, true);
  const Cmd cmd = make_cmd(Vec::LinSpaced(-3.0, 3.0));
  State out;
  bool all_ok = true;
  g_alloc_count.store(0);
  g_count_allocs.store(true);
  Eigen::internal::set_is_malloc_allowed(false);
  for (int k = 0; k < 200; ++k) {
    all_ok = io.read(out).is_ok() && all_ok;
    all_ok = io.write(cmd).is_ok() && all_ok;
  }
  Eigen::internal::set_is_malloc_allowed(true);
  g_count_allocs.store(false);
  EXPECT_TRUE(all_ok);
  EXPECT_EQ(g_alloc_count.load(), 0);
  EXPECT_EQ(inner.written.size(), 200U);
}

TEST(ImperfectIo, OtherDofInstantiationsExist) {
  struct Inner1 final : JointInterface<1> {
    Status read(JointState<1>& out) override {
      out.q[0] = 0.25;
      return Status::ok();
    }
    Status write(const JointCommand<1>& /*cmd*/) override { return Status::ok(); }
  } inner1;
  ImperfectJointInterface<1> io1(inner1, ImperfectionConfig<1>{}, kDt);
  JointState<1> s1;
  ASSERT_TRUE(io1.read(s1).is_ok());
  EXPECT_EQ(s1.q[0], 0.25);
  struct Inner2 final : JointInterface<2> {
    Status read(JointState<2>& /*out*/) override { return Status::ok(); }
    Status write(const JointCommand<2>& /*cmd*/) override { return Status::ok(); }
  } inner2;
  const ImperfectJointInterface<2> io2(inner2, ImperfectionConfig<2>{}, kDt);
  EXPECT_EQ(io2.torque_scale(), common::VecN<2>::Ones());
}

}  // namespace
}  // namespace cobot::imperfect
