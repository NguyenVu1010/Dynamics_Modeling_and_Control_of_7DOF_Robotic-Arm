#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/dynamics/sim_plant.hpp"
#include "cobot/hal/joint_interface.hpp"
#include "cobot/imperfect/config.hpp"
#include "cobot/imperfect/imperfect_joint_interface.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/sim_joint_interface.hpp"

// Feature comm-timing, design 8: every new key at 0 gives a result bit-identical to before.
// tests/data/imperfect_comm/decorator_golden.csv was recorded by this test with the decorator and
// SimJointInterface of main before the feature (commit 6c43ec8): every effect that existed then is
// on, the comm group and the overrun keys keep their defaults (0 = off). Only the default
// constructor arguments are used, so the same source built before and after the feature compares
// the two. Doubles are written with %.17g (round trip exact).
namespace cobot::imperfect {
namespace {

namespace fs = std::filesystem;

constexpr int kN = 7;
constexpr double kDt = 1e-3;  // s, 1 kHz
constexpr int kCycles = 1500;
constexpr int kEveryCycleUntil = 50;  // every cycle recorded before this one ...
constexpr int kSampleEvery = 25;      // ... then every 25th
constexpr int kPlantSteps = 400;
constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
using Vec = common::VecN<kN>;
using hal::JointCommand;
using hal::JointInterface;
using hal::JointState;

// Deterministic inner HAL: stamp k ms, q_j = 0.3 sin(2 pi f_j t) + 0.1 j with f_j = 0.5 + 0.25 j
// Hz, qd its derivative, tau_measured a slow wave; keeps the last command.
class WaveInner final : public JointInterface<kN> {
 public:
  Status read(JointState<kN>& out) override {
    const double t = static_cast<double>(reads_) * kDt;
    out.t = common::Timestamp{reads_ * 1'000'000};
    for (int j = 0; j < kN; ++j) {
      const double w = kTwoPi * (0.5 + 0.25 * j);
      out.q[j] = 0.3 * std::sin(w * t) + 0.1 * j;
      out.qd[j] = 0.3 * w * std::cos(w * t);
      out.tau_measured[j] = 2.0 * std::cos(0.7 * t + j);
    }
    out.tau_valid = true;
    ++reads_;
    return Status::ok();
  }
  Status write(const JointCommand<kN>& cmd) override {
    last = cmd;
    return Status::ok();
  }
  JointCommand<kN> last;

 private:
  std::int64_t reads_ = 0;
};

std::string fmt(double v) {
  std::array<char, 32> buf{};
  std::snprintf(buf.data(), buf.size(), "%.17g", v);
  return buf.data();
}

void put(std::string& line, const Vec& v) {
  for (int j = 0; j < kN; ++j) {
    line += ',' + fmt(v[j]);
  }
}

// Every effect that existed before the feature, on (qd from the noisy sample).
ImperfectionConfig<kN> config_noisy_qd() {
  ImperfectionConfig<kN> c;
  c.seed = 42;
  c.sensor.encoder_bits.setConstant(16.0);
  c.sensor.q_noise_std_rad.setConstant(1e-5);
  c.sensor.q_offset_std_rad.setConstant(1e-3);
  c.sensor.qd_from_diff = false;
  c.sensor.qd_noise_std_rad_s.setConstant(1e-3);
  c.sensor.sensor_delay_cycles = 2;
  c.actuator.torque_scale_std.setConstant(0.05);
  c.actuator.torque_ripple_frac.setConstant(0.02);
  c.actuator.torque_cmd_resolution_nm.setConstant(0.01);
  c.actuator.motor_time_constant_s.setConstant(0.002);
  c.actuator.command_delay_cycles = 1;
  c.actuator.torque_scale_asym_frac.setConstant(0.03);
  c.actuator.deadband_nm.setConstant(0.05);
  c.actuator.torque_slew_nm_s.setConstant(5000.0);
  c.actuator.knee_speed_rad_s.setConstant(1.6);
  c.actuator.zero_torque_speed_rad_s.setConstant(2.6);
  c.actuator.bus.v_nom_v = 48.0;
  c.actuator.bus.r_source_ohm = 0.25;
  c.actuator.bus.v_min_v = 40.0;
  c.friction.coulomb_nm.setConstant(0.5);
  c.friction.viscous_nm_s_rad.setConstant(0.1);
  c.timing.period_jitter_std_us = 20.0;
  return c;
}

// Same, but qd from the low-passed difference, without plant effects in the decorator.
ImperfectionConfig<kN> config_diff_qd() {
  ImperfectionConfig<kN> c = config_noisy_qd();
  c.seed = 7;
  c.sensor.qd_from_diff = true;
  c.sensor.qd_filter_hz.setConstant(50.0);
  c.sensor.sensor_delay_cycles = 1;
  c.actuator.command_delay_cycles = 3;
  return c;
}

// One line per recorded cycle: case,k,t_ns,q1..7,qd1..7,taum1..7,tau_inner1..7,bus_v
void run_decorator(const char* name,
                   const ImperfectionConfig<kN>& cfg,
                   bool plant_effects,
                   std::string& out) {
  WaveInner inner;
  ImperfectJointInterface<kN> io(inner, cfg, kDt, plant_effects, Vec::Constant(36.0));
  for (int k = 0; k < kCycles; ++k) {
    JointState<kN> s;
    ASSERT_TRUE(io.read(s).is_ok());
    JointCommand<kN> cmd;
    const double t = static_cast<double>(k) * kDt;
    for (int j = 0; j < kN; ++j) {
      cmd.tau[j] = 20.0 * std::sin(kTwoPi * 1.3 * t + j) + 0.5 * j;
    }
    ASSERT_TRUE(io.write(cmd).is_ok());
    if (k >= kEveryCycleUntil && k % kSampleEvery != 0) {
      continue;
    }
    std::string line = std::string(name) + ',' + std::to_string(k) + ',' + std::to_string(s.t.ns);
    put(line, s.q);
    put(line, s.qd);
    put(line, s.tau_measured);
    put(line, inner.last.tau);
    line += ',' + fmt(io.bus_voltage_v()) + '\n';
    out += line;
  }
}

// SimJointInterface with period jitter: the plant time after each advance (the jitter stream).
void run_sim_jitter(std::string& out) {
  const fs::path yaml = fs::path(COBOT_REPO_DIR) / "config/default.yaml";
  const auto cfg = pipeline::load_config<kN>(yaml);
  ASSERT_TRUE(cfg.is_ok()) << cfg.reason();
  const auto chain =
      model::parse_urdf_chain(cfg.value().urdf_path, cfg.value().base_link, cfg.value().tip_link);
  ASSERT_TRUE(chain.is_ok()) << chain.reason();
  const auto arm = model::make_model<kN>(chain.value(), cfg.value().gravity_b);
  ASSERT_TRUE(arm.is_ok()) << arm.reason();
  dynamics::SimPlant<kN> plant(arm.value(), cfg.value().home_q_rad);
  pipeline::SimJointInterface<kN> sim(plant, kDt, 20e-6, 5);
  for (int k = 0; k < kPlantSteps; ++k) {
    sim.advance();
    if (k % kSampleEvery != 0) {
      continue;
    }
    out += "sim_jitter," + std::to_string(k) + ',' + fmt(plant.time_s()) + '\n';
  }
}

std::vector<std::string> lines_of(const std::string& text) {
  std::vector<std::string> out;
  std::stringstream ss(text);
  std::string line;
  while (std::getline(ss, line)) {
    out.push_back(line);
  }
  return out;
}

TEST(CommGolden, AllZeroCommAndOverrunMatchTheDecoratorBeforeTheFeature) {
  std::string actual;
  run_decorator("noisy_qd", config_noisy_qd(), true, actual);
  run_decorator("diff_qd", config_diff_qd(), false, actual);
  run_sim_jitter(actual);
  ASSERT_FALSE(HasFatalFailure());

  const fs::path out = fs::path(COBOT_TEST_OUTPUT_DIR) / "imperfect_comm";
  fs::create_directories(out);
  std::ofstream(out / "decorator_golden.csv", std::ios::binary) << actual;

#if !defined(__x86_64__)
  // Recorded on x86-64; other targets may contract a * b + c into one FMA, which changes the
  // last bits (and can move a value across an encoder or resolution step).
  GTEST_SKIP() << "golden recorded on x86_64";
#endif
  const fs::path ref_path =
      fs::path(COBOT_REPO_DIR) / "tests/data/imperfect_comm/decorator_golden.csv";
  const std::ifstream in(ref_path, std::ios::binary);
  ASSERT_TRUE(in.good()) << "missing " << ref_path << "; this run wrote "
                         << out / "decorator_golden.csv";
  std::stringstream ss;
  ss << in.rdbuf();
  const std::vector<std::string> want = lines_of(ss.str());
  const std::vector<std::string> got = lines_of(actual);
  ASSERT_EQ(got.size(), want.size());
  for (std::size_t l = 0; l < want.size(); ++l) {
    ASSERT_EQ(got[l], want[l]) << "first difference at line " << l + 1;
  }
}

}  // namespace
}  // namespace cobot::imperfect
