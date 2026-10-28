#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <memory>
#include <new>

#include <gtest/gtest.h>

#include "control_loop_test_util.hpp"

// Counts every global operator new in this binary while g_counting is set, so a std::vector,
// std::string or dynamic Eigen matrix created inside ControlLoop::step() (compiled in the
// pipeline library, where EIGEN_RUNTIME_NO_MALLOC is not defined) is caught too.
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

// The ring holds kLogCapacity - 1 = 8191 records (one slot stays empty).
constexpr int kRingItems = static_cast<int>(kLogCapacity) - 1;

// One host cycle with only loop.step() under the allocation guards.
void guarded_cycle(Rig& rig, State& s, Command& cmd, bool inject_nan, std::size_t& news) {
  ASSERT_TRUE(rig.io.read(s).is_ok());
  if (inject_nan) {
    s.q(0) = std::numeric_limits<double>::quiet_NaN();
  }
  const std::size_t before = g_new_calls;
  g_counting = true;
  Eigen::internal::set_is_malloc_allowed(false);
  const common::Status st = rig.loop.step(s, cmd);
  Eigen::internal::set_is_malloc_allowed(true);
  g_counting = false;
  news += g_new_calls - before;
  EXPECT_TRUE(st.is_ok());
  ASSERT_TRUE(rig.io.write(cmd).is_ok());
  rig.io.advance();
}

// Tracking until the ring is full (never drained), then a NaN read into kHolding and more
// holding steps: no step allocates, a full ring drops and counts instead of blocking, and
// the ring keeps the oldest 8191 records.
TEST(ControlLoopNoAlloc, StepDoesNotAllocateInTrackingHoldingAndWithAFullRing) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const std::unique_ptr<Rig> rig = make_rig(*setup, setup->config);
  ASSERT_TRUE(rig->start().is_ok());
  State s;
  Command cmd;
  std::size_t news = 0;
  constexpr int kTracking = kRingItems + 9;  // 8200 steps: 9 records dropped
  for (int k = 0; k < kTracking; ++k) {
    guarded_cycle(*rig, s, cmd, false, news);
  }
  EXPECT_EQ(rig->loop.mode(), LoopMode::kTracking);
  EXPECT_EQ(rig->loop.log().dropped(), 9U);  // 8200 - 8191

  guarded_cycle(*rig, s, cmd, true, news);
  EXPECT_EQ(rig->loop.mode(), LoopMode::kHolding);
  constexpr int kHolding = 100;
  for (int k = 0; k < kHolding; ++k) {
    guarded_cycle(*rig, s, cmd, false, news);
  }
  EXPECT_EQ(rig->loop.mode(), LoopMode::kHolding);
  EXPECT_EQ(news, 0U);
  EXPECT_EQ(rig->loop.log().dropped(), 9U + 1U + 100U);  // NaN step + 100 holding steps

  Record r;
  int popped = 0;
  std::int64_t last_ns = -1;
  while (rig->loop.log().pop(r)) {
    if (popped == 0) {
      EXPECT_EQ(r.t.ns, 0);
    }
    last_ns = r.t.ns;
    ++popped;
  }
  EXPECT_EQ(popped, kRingItems);
  EXPECT_EQ(last_ns, static_cast<std::int64_t>(kRingItems - 1) * 1000000);  // 8190 ms
}

// deterministic: false reads steady_clock in step(); that must not allocate either.
TEST(ControlLoopNoAlloc, TimedStepDoesNotAllocate) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  Config<kN> cfg = setup->config;
  cfg.deterministic = false;
  const std::unique_ptr<Rig> rig = make_rig(*setup, cfg);
  ASSERT_TRUE(rig->start().is_ok());
  State s;
  Command cmd;
  std::size_t news = 0;
  for (int k = 0; k < 200; ++k) {
    guarded_cycle(*rig, s, cmd, false, news);
  }
  EXPECT_EQ(news, 0U);
}

}  // namespace
}  // namespace cobot::pipeline::test
