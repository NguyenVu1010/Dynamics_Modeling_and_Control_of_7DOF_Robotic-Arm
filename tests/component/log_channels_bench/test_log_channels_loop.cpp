// ControlLoop::step(state, out, truth) (feature log-channels, contract 2): the truth is copied
// into the record verbatim and changes nothing else; step(state, out) logs unknown (NaN) truth;
// neither allocates. Uses the E2E-01 rig of tests/component/control_loop (read only).
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <new>

#include <gtest/gtest.h>

#include "cobot/pipeline/control_loop.hpp"
#include "cobot/pipeline/log_record.hpp"
#include "cobot/pipeline/sim_joint_interface.hpp"

#include "control_loop_test_util.hpp"

// Counts every global operator new while g_counting is set (pattern of
// tests/component/control_loop_recorder/test_control_loop_recorder_no_alloc.cpp); all forms are
// replaced so that GoogleTest's nothrow allocations stay paired under ASan.
namespace {
bool g_counting = false;      // test-only, single thread
std::size_t g_new_calls = 0;  // test-only
}  // namespace

void* operator new(std::size_t size) {
  if (g_counting) {
    ++g_new_calls;
  }
  if (void* p = std::malloc(size == 0 ? 1 : size)) {
    return p;
  }
  throw std::bad_alloc();
}
void* operator new[](std::size_t size) {
  return ::operator new(size);
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
void* operator new(std::size_t size, const std::nothrow_t& /*tag*/) noexcept {
  try {
    return ::operator new(size);
  } catch (...) {
    return nullptr;
  }
}
void* operator new[](std::size_t size, const std::nothrow_t& /*tag*/) noexcept {
  try {
    return ::operator new[](size);
  } catch (...) {
    return nullptr;
  }
}
void operator delete(void* p, const std::nothrow_t& /*tag*/) noexcept {
  ::operator delete(p);
}
void operator delete[](void* p, const std::nothrow_t& /*tag*/) noexcept {
  ::operator delete[](p);
}

namespace cobot::pipeline::test {
namespace {

// Bit-equal entries (NaN payloads and the sign of zero included).
bool same(const Vec& a, const Vec& b) {
  for (int j = 0; j < kN; ++j) {
    std::uint64_t x = 0;
    std::uint64_t y = 0;
    std::memcpy(&x, &a(j), sizeof(double));
    std::memcpy(&y, &b(j), sizeof(double));
    if (x != y) {
      return false;
    }
  }
  return true;
}

bool all_nan(const Vec& v) {
  return v.array().isNaN().all();
}

// 10 000 steps (10 s of the 10 s trajectory) of three loops fed the same states: `plain` with
// step(state, out), `unknown` with a default PlantTruth, `probed` with SimTruthProbe::sample().
// The commands are bit-identical, the truth lands in the record verbatim, nothing allocates.
TEST(LogChannelsLoop, TruthIsCopiedVerbatimChangesNothingElseAndDoesNotAllocate) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  constexpr int kSteps = 10000;
  dynamics::SimPlant<kN> plant(setup->model, setup->config.home_q_rad);
  SimJointInterface<kN> io(plant, 1.0 / setup->config.control_rate_hz);
  SimTruthProbe<kN> probe(plant);
  ControlLoop<kN> plain(setup->model, setup->config);
  ControlLoop<kN> unknown(setup->model, setup->config);
  ControlLoop<kN> probed(setup->model, setup->config);
  const PlantTruth<kN> none;
  State s;
  ASSERT_TRUE(io.read(s).is_ok());
  ASSERT_TRUE(plain.start(s).is_ok());
  ASSERT_TRUE(unknown.start(s).is_ok());
  ASSERT_TRUE(probed.start(s).is_ok());

  std::size_t news = 0;
  bool qdd_ref_seen = false;
  for (int k = 0; k < kSteps; ++k) {
    ASSERT_TRUE(io.read(s).is_ok());
    Command cmd;
    Command cmd_unknown;
    Command cmd_plain;
    const std::size_t before = g_new_calls;
    g_counting = true;
    Eigen::internal::set_is_malloc_allowed(false);
    const PlantTruth<kN>& truth = probe.sample();
    const bool ok = probed.step(s, cmd, truth).is_ok();
    const bool ok_unknown = unknown.step(s, cmd_unknown, none).is_ok();
    Eigen::internal::set_is_malloc_allowed(true);
    g_counting = false;
    news += g_new_calls - before;
    ASSERT_TRUE(ok && ok_unknown) << "step " << k;
    ASSERT_TRUE(plain.step(s, cmd_plain).is_ok()) << "step " << k;
    ASSERT_TRUE(same(cmd.tau, cmd_plain.tau)) << "step " << k;
    ASSERT_TRUE(same(cmd_unknown.tau, cmd_plain.tau)) << "step " << k;

    const Record& r = probed.last_record();
    ASSERT_TRUE(same(r.q_true, truth.q) && same(r.qd_true, truth.qd) &&
                same(r.qdd_true, truth.qdd) && same(r.q_motor_true, truth.q_motor) &&
                same(r.tau_motor, truth.tau_motor) && same(r.tau_friction, truth.tau_friction))
        << "step " << k;
    const Record& p = plain.last_record();
    ASSERT_TRUE(all_nan(p.q_true) && all_nan(p.qd_true) && all_nan(p.qdd_true) &&
                all_nan(p.q_motor_true) && all_nan(p.tau_motor) && all_nan(p.tau_friction))
        << "step " << k;
    ASSERT_TRUE(same(r.q, p.q) && same(r.qdd_ref, p.qdd_ref) && same(r.q_raw, p.q_raw) &&
                same(unknown.last_record().qdd_ref, p.qdd_ref))
        << "step " << k;
    // Deflection compensation inactive (link encoder): q_raw is q.
    ASSERT_TRUE(same(p.q_raw, p.q)) << "step " << k;
    ASSERT_EQ(p.cmd_mode, CommandMode::kConfig) << "step " << k;
    // CLIK's ref.qdd while tracking (the trajectory accelerates from rest).
    qdd_ref_seen = qdd_ref_seen || (p.mode == LoopMode::kTracking && !p.qdd_ref.isZero(0.0));
    ASSERT_TRUE(io.write(cmd).is_ok());
    io.advance();
  }
  EXPECT_EQ(news, 0U);
  // The ideal arm moves: the truth really changed during the run.
  EXPECT_GT((probed.last_record().q_true - setup->config.home_q_rad).norm(), 0.0);
  EXPECT_TRUE(qdd_ref_seen);
}

}  // namespace
}  // namespace cobot::pipeline::test
