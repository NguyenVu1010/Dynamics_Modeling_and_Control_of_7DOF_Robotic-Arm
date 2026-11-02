#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <new>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "cobot/hal/joint_interface.hpp"
#include "cobot/imperfect/config.hpp"
#include "cobot/imperfect/imperfect_joint_interface.hpp"

// Counts global operator new calls while g_count_allocs is set (catches allocations inside the
// compiled imperfect library, where EIGEN_RUNTIME_NO_MALLOC is not defined). Every form is
// replaced so all allocations and deallocations stay paired through malloc/free under ASan.
namespace {
std::atomic<bool> g_count_allocs{false};
std::atomic<long> g_alloc_count{0};

void* counted_malloc(std::size_t size) {
  if (g_count_allocs.load()) {
    g_alloc_count.fetch_add(1);
  }
  return std::malloc(size == 0 ? 1 : size);
}
void* counted_or_throw(std::size_t size) {
  void* p = counted_malloc(size);
  if (p == nullptr) {
    throw std::bad_alloc();
  }
  return p;
}
}  // namespace

void* operator new(std::size_t size) {
  return counted_or_throw(size);
}
void* operator new[](std::size_t size) {
  return counted_or_throw(size);
}
void* operator new(std::size_t size, const std::nothrow_t& /*tag*/) noexcept {
  return counted_malloc(size);
}
void* operator new[](std::size_t size, const std::nothrow_t& /*tag*/) noexcept {
  return counted_malloc(size);
}
void operator delete(void* p) noexcept {
  std::free(p);
}
void operator delete[](void* p) noexcept {
  std::free(p);
}
void operator delete(void* p, std::size_t /*size*/) noexcept {
  std::free(p);
}
void operator delete[](void* p, std::size_t /*size*/) noexcept {
  std::free(p);
}
void operator delete(void* p, const std::nothrow_t& /*tag*/) noexcept {
  std::free(p);
}
void operator delete[](void* p, const std::nothrow_t& /*tag*/) noexcept {
  std::free(p);
}

// Feature actuator-limits (realism-2 design 6.1): asymmetric scale, deadband, slew limit,
// torque-speed curve and shared bus sag in ImperfectJointInterface::write.
namespace cobot::imperfect {
namespace {

constexpr int kN = 7;
constexpr double kDt = 1e-3;  // s
using Vec = common::VecN<kN>;
using Io = ImperfectJointInterface<kN>;

Vec vec(double a, double b, double c, double d, double e, double f, double g) {
  return (Vec() << a, b, c, d, e, f, g).finished();
}

// Inner interface that always returns `state` and keeps the last command.
class ConstantInner final : public JointInterface<kN> {
 public:
  Status read(JointState<kN>& out) override {
    out = state;
    return Status::ok();
  }
  Status write(const JointCommand<kN>& cmd) override {
    last = cmd;
    return Status::ok();
  }
  JointState<kN> state;
  JointCommand<kN> last;
};

// One control cycle: read (latches the true velocity), write `tau`, return what the inner got.
Vec cycle(Io& io, ConstantInner& inner, const Vec& tau) {
  JointState<kN> s;
  EXPECT_TRUE(io.read(s).is_ok());
  JointCommand<kN> c;
  c.tau = tau;
  EXPECT_TRUE(io.write(c).is_ok());
  return inner.last.tau;
}

void expect_near(const Vec& got, const Vec& want, double tol) {
  for (int j = 0; j < kN; ++j) {
    EXPECT_NEAR(got[j], want[j], tol) << "joint " << j;
  }
}

TEST(ActuatorLimits, AsymmetricScaleOnlyOnNegativeTorque) {
  ConstantInner inner;
  ImperfectionConfig<kN> cfg;
  cfg.actuator.torque_scale_asym_frac.setConstant(0.1);
  Io io(inner, cfg, kDt);
  // -10 * (1 - 0.1) = -9; positive and zero torques keep the plain gain (scale std 0 -> 1).
  const Vec out = cycle(io, inner, vec(10, -10, 0, 3.5, -3.5, 0.25, -0.25));
  expect_near(out, vec(10, -9, 0, 3.5, -3.15, 0.25, -0.225), 1e-12);
  EXPECT_EQ(out[0], 10.0);
}

TEST(ActuatorLimits, DeadbandZeroesSmallTorques) {
  ConstantInner inner;
  ImperfectionConfig<kN> cfg;
  cfg.actuator.deadband_nm.setConstant(0.5);
  Io io(inner, cfg, kDt);
  // |tau| < 0.5 -> 0; |tau| >= 0.5 passes unchanged.
  const Vec out = cycle(io, inner, vec(0.4, 0.6, -0.4, -0.6, 0.5, -0.5, 0.0));
  const Vec want = vec(0.0, 0.6, 0.0, -0.6, 0.5, -0.5, 0.0);
  for (int j = 0; j < kN; ++j) {
    EXPECT_EQ(out[j], want[j]) << "joint " << j;
  }
}

TEST(ActuatorLimits, SlewLimitRampsOneNmPerMillisecond) {
  ConstantInner inner;
  ImperfectionConfig<kN> cfg;
  cfg.actuator.torque_slew_nm_s.setConstant(1000.0);  // 1000 N m/s * 1 ms = 1 N m per write
  Io io(inner, cfg, kDt);
  EXPECT_EQ(cycle(io, inner, Vec::Zero())[0], 0.0);
  const Vec step = vec(10, -10, 0.5, 0, 0, 0, 0);
  for (int k = 1; k <= 12; ++k) {  // write k gives min(k, 10)
    const Vec out = cycle(io, inner, step);
    const double ramp = k < 10 ? k : 10.0;
    expect_near(out, vec(ramp, -ramp, 0.5, 0, 0, 0, 0), 1e-9);
  }
  for (int k = 1; k <= 3; ++k) {  // back down from 10 towards 0 at the same rate
    EXPECT_NEAR(cycle(io, inner, Vec::Zero())[0], 10.0 - k, 1e-9) << k;
  }
}

TEST(ActuatorLimits, TorqueSpeedCurveWithBusOff) {
  ConstantInner inner;
  // Motoring (tau*w > 0): ceiling 36 up to the knee 2.2, then 36 * (3.6 - |w|) / (3.6 - 2.2),
  // 0 from 3.6. w = 2.9: 36 * 0.7 / 1.4 = 18. Braking (tau*w <= 0): capped at P = 36.
  inner.state.qd = vec(1.0, 2.9, 4.0, -2.9, -2.9, 1.0, 0.0);
  const Vec tau = vec(50, 50, 50, 50, -50, 10, -50);
  ImperfectionConfig<kN> cfg;
  cfg.actuator.knee_speed_rad_s.setConstant(2.2);
  cfg.actuator.zero_torque_speed_rad_s.setConstant(3.6);
  cfg.sensor.qd_noise_std_rad_s.setConstant(1.0);  // the curve uses the true velocity
  Io io(inner, cfg, kDt, false, Vec::Constant(36.0));
  for (int k = 0; k < 3; ++k) {
    expect_near(cycle(io, inner, tau), vec(36, 18, 0, 36, -18, 10, -36), 1e-9);
  }
}

// Joints 0-1: P 204, tau 100, w 2; joint 2: P 36, tau 50, w 2.9; knee 3, zero 6, V_nom 48.
// Write 1: no previous output -> Pw = 0 -> V = (48 + sqrt(48^2)) / 2 = 48; w < knee everywhere,
// outputs (100, 100, 36). Write 2: Pw = 100*2 + 100*2 + 36*2.9 = 504.4 W and
// V = (48 + sqrt(2304 - 4 R 504.4 / 0.85)) / 2, clamped below at 40:
//   R 0.25: disc 1710.588 -> V 44.67963, knee' 2.79248, zero' 5.58495,
//           joint 2 ceiling 36 * (5.58495 - 2.9) / (5.58495 - 2.79248) = 34.61383;
//   R 0.8:  disc 405.08 -> V 34.06 -> 40; R 100: disc < 0 -> 40.
//   V 40: knee' = 3 * 40/48 = 2.5, zero' = 5, ceiling 36 * 2.1 / 2.5 = 30.24.
TEST(ActuatorLimits, BusVoltageSagScalesTheCurve) {
  ImperfectionConfig<kN> cfg;
  cfg.actuator.knee_speed_rad_s.setConstant(3.0);
  cfg.actuator.zero_torque_speed_rad_s.setConstant(6.0);
  cfg.actuator.bus.v_nom_v = 48.0;
  cfg.actuator.bus.v_min_v = 40.0;
  const Vec peak = vec(204, 204, 36, 36, 36, 36, 36);
  const Vec tau = vec(100, 100, 50, 0, 0, 0, 0);
  for (const double r : {0.25, 0.8, 100.0}) {
    cfg.actuator.bus.r_source_ohm = r;
    ConstantInner inner;
    inner.state.qd = vec(2.0, 2.0, 2.9, 0, 0, 0, 0);
    Io io(inner, cfg, kDt, false, peak);
    expect_near(cycle(io, inner, tau), vec(100, 100, 36, 0, 0, 0, 0), 1e-9);
    EXPECT_NEAR(io.bus_voltage_v(), 48.0, 1e-12) << r;

    const Vec out = cycle(io, inner, tau);
    const double pw = 100.0 * 2.0 + 100.0 * 2.0 + 36.0 * 2.9;
    const double disc = 48.0 * 48.0 - 4.0 * r * pw / 0.85;
    const double v = disc < 0.0 ? 40.0 : std::max(40.0, (48.0 + std::sqrt(disc)) / 2.0);
    EXPECT_NEAR(io.bus_voltage_v(), v, 1e-12) << r;
    EXPECT_NEAR(io.bus_voltage_v(), r == 0.25 ? 44.67963 : 40.0, 1e-5) << r;
    const double knee = 3.0 * v / 48.0;
    const double zero = 6.0 * v / 48.0;
    const double ceiling = 36.0 * (zero - 2.9) / (zero - knee);
    EXPECT_NEAR(ceiling, r == 0.25 ? 34.61383 : 30.24, 1e-5) << r;
    expect_near(out, vec(100, 100, ceiling, 0, 0, 0, 0), 1e-9);
  }
}

TEST(ActuatorLimits, DeadbandSeesAsymScaledTorqueAndPrecedesResolution) {
  ConstantInner inner;
  ImperfectionConfig<kN> cfg;
  cfg.actuator.torque_scale_asym_frac.setConstant(0.1);
  cfg.actuator.deadband_nm.setConstant(0.5);
  Io io(inner, cfg, kDt);
  // -0.55 * 0.9 = -0.495 -> |.| < 0.5 -> 0; +0.55 passes.
  const Vec out = cycle(io, inner, vec(-0.55, 0.55, 0, 0, 0, 0, 0));
  EXPECT_EQ(out[0], 0.0);
  EXPECT_NEAR(out[1], 0.55, 1e-12);

  // Resolution 0.4 after deadband 0.5: 0.55 passes the deadband, rounds to 1 * 0.4 = 0.4
  // (rounding first would give 0.4, which the deadband would then zero).
  ImperfectionConfig<kN> cfg2;
  cfg2.actuator.deadband_nm.setConstant(0.5);
  cfg2.actuator.torque_cmd_resolution_nm.setConstant(0.4);
  Io io2(inner, cfg2, kDt);
  EXPECT_NEAR(cycle(io2, inner, Vec::Constant(0.55))[0], 0.4, 1e-12);
}

TEST(ActuatorLimits, KneeWithInfinitePeakThrowsNamingTheKey) {
  ConstantInner inner;
  ImperfectionConfig<kN> cfg;
  cfg.actuator.knee_speed_rad_s[3] = 2.2;
  cfg.actuator.zero_torque_speed_rad_s[3] = 3.6;
  const auto expect_throw = [&](const Vec& peak) {
    try {
      const Io io(inner, cfg, kDt, false, peak);
      ADD_FAILURE() << "infinite peak accepted";
    } catch (const std::invalid_argument& e) {
      EXPECT_NE(std::string(e.what()).find("actuator.knee_speed_rad_s"), std::string::npos)
          << e.what();
    }
  };
  expect_throw(Vec::Constant(std::numeric_limits<double>::infinity()));
  Vec peak = Vec::Constant(36.0);
  peak[3] = std::numeric_limits<double>::infinity();
  expect_throw(peak);
  try {
    const Io defaulted(inner, cfg, kDt);  // default peak is infinite
    ADD_FAILURE() << "default (infinite) peak accepted";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("actuator.knee_speed_rad_s"), std::string::npos);
  }
  EXPECT_NO_THROW(Io(inner, cfg, kDt, false, Vec::Constant(36.0)));
}

// Today's effects only (new keys 0): a finite peak changes nothing, bit for bit, and -0.0 stays.
TEST(ActuatorLimits, NewKeysZeroIsBitIdentical) {
  ImperfectionConfig<kN> cfg;
  cfg.seed = 9;
  cfg.actuator.torque_scale_std.setConstant(0.1);
  cfg.actuator.torque_ripple_frac.setConstant(0.05);
  cfg.actuator.torque_cmd_resolution_nm.setConstant(0.01);
  cfg.actuator.command_delay_cycles = 2;
  cfg.actuator.motor_time_constant_s.setConstant(0.003);
  cfg.friction.coulomb_nm.setConstant(1.0);
  for (const bool plant : {false, true}) {
    ConstantInner a;
    ConstantInner b;
    a.state.q = b.state.q = Vec::LinSpaced(-1.0, 1.0);
    a.state.qd = b.state.qd = vec(3, -3, 0.1, 5, -5, 0, 2);
    Io today(a, cfg, kDt, plant);
    Io capped(b, cfg, kDt, plant, Vec::Constant(1.0));
    for (int k = 0; k < 20; ++k) {
      const Vec tau = Vec::LinSpaced(-50.0, 50.0) * (1.0 + 0.1 * k);
      const Vec ta = cycle(today, a, tau);
      const Vec tb = cycle(capped, b, tau);
      for (int j = 0; j < kN; ++j) {
        EXPECT_EQ(ta[j], tb[j]) << plant << " k" << k << " j" << j;
      }
    }
  }
  ConstantInner inner;
  Io io(inner, ImperfectionConfig<kN>{}, kDt, false, Vec::Constant(36.0));
  const Vec out = cycle(io, inner, Vec::Constant(-0.0));
  EXPECT_TRUE(std::signbit(out[0]) && out[0] == 0.0);
}

ImperfectionConfig<kN> all_new_effects() {
  ImperfectionConfig<kN> cfg;
  cfg.actuator.torque_scale_asym_frac.setConstant(0.05);
  cfg.actuator.deadband_nm.setConstant(0.1);
  cfg.actuator.torque_slew_nm_s.setConstant(2e4);
  cfg.actuator.knee_speed_rad_s.setConstant(2.0);
  cfg.actuator.zero_torque_speed_rad_s.setConstant(3.0);
  cfg.actuator.bus = {48.0, 0.25, 40.0};
  cfg.actuator.command_delay_cycles = 1;
  return cfg;
}

TEST(ActuatorLimits, WriteDoesNotAllocateAndIsDeterministic) {
  ConstantInner a;
  ConstantInner b;
  a.state.qd = b.state.qd = Vec::LinSpaced(-3.5, 3.5);
  Io io_a(a, all_new_effects(), kDt, true, Vec::Constant(36.0));
  Io io_b(b, all_new_effects(), kDt, true, Vec::Constant(36.0));
  int mismatches = 0;
  g_alloc_count.store(0);
  g_count_allocs.store(true);
  Eigen::internal::set_is_malloc_allowed(false);
  for (int k = 0; k < 200; ++k) {
    // Same sign as w on every joint (motoring), so the bus carries power to the end.
    const Vec tau = Vec::LinSpaced(-60.0, 60.0) * (0.6 + 0.4 * std::sin(0.05 * k));
    mismatches += cycle(io_a, a, tau) == cycle(io_b, b, tau) ? 0 : 1;
  }
  Eigen::internal::set_is_malloc_allowed(true);
  g_count_allocs.store(false);
  EXPECT_EQ(g_alloc_count.load(), 0);
  EXPECT_EQ(mismatches, 0);
  EXPECT_EQ(io_a.bus_voltage_v(), io_b.bus_voltage_v());
  EXPECT_LT(io_a.bus_voltage_v(), 48.0);  // the arm draws power: the bus sags
}

}  // namespace
}  // namespace cobot::imperfect
