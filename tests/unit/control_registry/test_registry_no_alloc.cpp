#include <cstddef>
#include <cstdlib>
#include <memory>
#include <new>

#include <gtest/gtest.h>

#include "cobot/control/algorithm.hpp"

#include "control_loop_test_util.hpp"

// ControlLoop::step() allocates nothing with either tracking law (CONTRIBUTING "Hot path
// rules", ADR-008): every global operator new of this binary is counted while g_counting is set
// (the loop is compiled in the pipeline library without EIGEN_RUNTIME_NO_MALLOC), and Eigen's
// malloc guard is off around each step.
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

// GoogleTest allocates with the nothrow forms; route them through the replacements above so
// every allocation and deallocation pair matches (otherwise ASan reports alloc-dealloc-mismatch).
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

// `steps` host cycles with only loop.step() under both guards; returns the operator new calls.
std::size_t guarded_steps(Rig& rig, int steps) {
  State s;
  Command cmd;
  std::size_t news = 0;
  for (int k = 0; k < steps; ++k) {
    EXPECT_TRUE(rig.io.read(s).is_ok());
    const std::size_t before = g_new_calls;
    g_counting = true;
    Eigen::internal::set_is_malloc_allowed(false);
    const bool ok = rig.loop.step(s, cmd).is_ok();
    Eigen::internal::set_is_malloc_allowed(true);
    g_counting = false;
    news += g_new_calls - before;
    EXPECT_TRUE(ok) << "step " << k;
    EXPECT_TRUE(rig.io.write(cmd).is_ok());
    rig.io.advance();
  }
  return news;
}

// tests/e2e/configs/ideal_world.yaml with the given law; PD+G at omega = 80 rad/s as the
// computed torque of the file (kp = 80^2 = 6400, kd = 2 * 80 = 160).
Config<kN> with_algorithm(const Setup& setup, control::Algorithm a, CommandSource source) {
  Config<kN> c = setup.config;
  c.algorithm = a;
  c.pd_gravity.kp.setConstant(6400.0);
  c.pd_gravity.kd.setConstant(160.0);
  c.command_source = source;
  return c;
}

TEST(ControlLoopRegistry, NoAllocation) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  for (const control::Algorithm a :
       {control::Algorithm::kComputedTorque, control::Algorithm::kPdGravity}) {
    const std::unique_ptr<Rig> config_rig =
        make_rig(*setup, with_algorithm(*setup, a, CommandSource::kConfig));
    ASSERT_EQ(config_rig->loop.algorithm(), a);
    ASSERT_TRUE(config_rig->start().is_ok());
    EXPECT_EQ(guarded_steps(*config_rig, 1000), 0U) << control::to_string(a) << ", config mode";
    EXPECT_EQ(config_rig->loop.mode(), LoopMode::kTracking) << control::to_string(a);

    const std::unique_ptr<Rig> standby_rig =
        make_rig(*setup, with_algorithm(*setup, a, CommandSource::kExternal));
    ASSERT_TRUE(standby_rig->start().is_ok());
    EXPECT_EQ(guarded_steps(*standby_rig, 500), 0U) << control::to_string(a) << ", standby";
    EXPECT_EQ(standby_rig->loop.mode(), LoopMode::kStandby) << control::to_string(a);
  }
}

}  // namespace
}  // namespace cobot::pipeline::test
