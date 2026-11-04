// Chart series (feature studio-core, cobot_studio design 5.1, ADR-021, cobot/studio/series.hpp).
// Noise lines by source: sensor q - q_true row by row; actuator tau_motor[k+1] - tau[k] (taum of
// row k+1 is the plant's response to tau of row k, so rows - 1 points); friction tau_friction;
// model, timing and comm sources have no noise line (unavailable, with a reason).
#include <cstddef>
#include <cstdint>
#include <string>

#include <gtest/gtest.h>

#include "cobot/common/timestamp.hpp"
#include "cobot/studio/param_catalog.hpp"
#include "cobot/studio/series.hpp"

#include "run_data.hpp"

namespace cobot::studio {
namespace {

constexpr std::size_t kRows = 10;
constexpr int kJoint = 2;

// Distinct values per row and joint: q = 0.1 k + j, q_true = q - 1e-3 (k + 1)(j + 1),
// tau = k + 0.5 j, tau_motor = 2 k + j, tau_friction = -0.01 k j, q_ref = q + 0.5.
bench::RunData channel_run() {
  bench::RunData run;
  run.records.resize(kRows);
  for (std::size_t k = 0; k < kRows; ++k) {
    pipeline::LogRecord<7>& r = run.records[k];
    const auto kd = static_cast<double>(k);
    r.t = common::Timestamp{static_cast<std::int64_t>(k) * 1000000};
    r.mode = pipeline::LoopMode::kTracking;
    for (int j = 0; j < 7; ++j) {
      const auto jd = static_cast<double>(j);
      r.q(j) = 0.1 * kd + jd;
      r.q_true(j) = r.q(j) - 1e-3 * (kd + 1.0) * (jd + 1.0);
      r.q_ref(j) = r.q(j) + 0.5;
      r.tau(j) = kd + 0.5 * jd;
      r.tau_motor(j) = 2.0 * kd + jd;
      r.tau_friction(j) = -0.01 * kd * jd;
    }
    r.T_b_e.p = common::Vec3(0.003 * kd, 0.0, 0.0);
  }
  run.meta.period_s = 1e-3;
  return run;
}

TEST(StudioSeries, JointChannels) {
  const bench::RunData run = channel_run();
  const Series q = extract(run, Channel::kQ, kJoint);
  EXPECT_TRUE(q.unavailable.empty()) << q.unavailable;
  ASSERT_EQ(q.y.size(), kRows);
  ASSERT_EQ(q.t_s.size(), kRows);
  const Series q_ref = extract(run, Channel::kQRef, kJoint);
  ASSERT_EQ(q_ref.y.size(), kRows);
  const Series tau = extract(run, Channel::kTau, kJoint);
  ASSERT_EQ(tau.y.size(), kRows);
  for (std::size_t k = 0; k < kRows; ++k) {
    EXPECT_NEAR(q.t_s[k], static_cast<double>(k) * 1e-3, 1e-12) << k;
    EXPECT_EQ(q.y[k], run.records[k].q(kJoint)) << k;
    EXPECT_EQ(q_ref.y[k], run.records[k].q_ref(kJoint)) << k;
    EXPECT_EQ(tau.y[k], run.records[k].tau(kJoint)) << k;
  }
}

TEST(StudioSeries, SensorNoiseIsQMinusQTrueRowByRow) {
  const bench::RunData run = channel_run();
  const Series s = extract(run, Channel::kNoise, kJoint, NoiseSource::kSensor);
  EXPECT_TRUE(s.unavailable.empty()) << s.unavailable;
  ASSERT_EQ(s.y.size(), kRows);
  ASSERT_EQ(s.t_s.size(), kRows);
  for (std::size_t k = 0; k < kRows; ++k) {
    EXPECT_EQ(s.y[k], run.records[k].q(kJoint) - run.records[k].q_true(kJoint)) << k;
  }
}

TEST(StudioSeries, ActuatorNoiseIsShiftedByOneRow) {
  const bench::RunData run = channel_run();
  const Series s = extract(run, Channel::kNoise, kJoint, NoiseSource::kActuator);
  EXPECT_TRUE(s.unavailable.empty()) << s.unavailable;
  ASSERT_EQ(s.y.size(), kRows - 1);
  ASSERT_EQ(s.t_s.size(), kRows - 1);
  for (std::size_t k = 0; k + 1 < kRows; ++k) {
    // tau_motor[k+1] - tau[k] = 2 (k + 1) + j - (k + 0.5 j) = k + 2 + 0.5 j
    EXPECT_EQ(s.y[k], run.records[k + 1].tau_motor(kJoint) - run.records[k].tau(kJoint)) << k;
  }
}

TEST(StudioSeries, FrictionNoiseIsTheFrictionTorque) {
  const bench::RunData run = channel_run();
  const Series s = extract(run, Channel::kNoise, kJoint, NoiseSource::kFriction);
  EXPECT_TRUE(s.unavailable.empty()) << s.unavailable;
  ASSERT_EQ(s.y.size(), kRows);
  for (std::size_t k = 0; k < kRows; ++k) {
    EXPECT_EQ(s.y[k], run.records[k].tau_friction(kJoint)) << k;
  }
}

TEST(StudioSeries, NoNoiseLineForModelTimingAndCommSources) {
  const bench::RunData run = channel_run();
  const Series s = extract(run, Channel::kNoise, kJoint, NoiseSource::kNone);
  EXPECT_FALSE(s.unavailable.empty());
}

TEST(StudioSeries, TaskPositionErrorIsTheNormOfPdMinusP) {
  const bench::RunData run = channel_run();
  const Series s = extract(run, Channel::kTaskPosErr, -1);
  EXPECT_TRUE(s.unavailable.empty()) << s.unavailable;
  ASSERT_EQ(s.y.size(), kRows);
  for (std::size_t k = 0; k < kRows; ++k) {
    EXPECT_NEAR(s.y[k], 0.003 * static_cast<double>(k), 1e-15) << k;  // T_b_d at the origin
  }
  const Series rot = extract(run, Channel::kTaskRotErr, -1);
  ASSERT_EQ(rot.y.size(), kRows);
  EXPECT_EQ(rot.y[5], 0.0);
}

}  // namespace
}  // namespace cobot::studio
