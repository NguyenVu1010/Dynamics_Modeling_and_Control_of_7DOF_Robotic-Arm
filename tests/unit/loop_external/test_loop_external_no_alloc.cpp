#include <cstddef>
#include <cstdlib>
#include <memory>
#include <new>

#include <gtest/gtest.h>

#include "loop_external_util.hpp"

// Counts every global operator new in this binary while g_counting is set, so a std::vector,
// std::string or dynamic Eigen matrix created inside step(), load(), request_stop() or reset()
// (compiled in the pipeline library, where EIGEN_RUNTIME_NO_MALLOC is not defined) is caught.
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

// Runs `fn` with both allocation guards on and adds the operator new calls it made to `news`.
template <typename Fn>
void guarded(std::size_t& news, Fn fn) {
  const std::size_t before = g_new_calls;
  g_counting = true;
  Eigen::internal::set_is_malloc_allowed(false);
  fn();
  Eigen::internal::set_is_malloc_allowed(true);
  g_counting = false;
  news += g_new_calls - before;
}

// One host cycle with only loop.step() under the guards; `spike` puts qd_2 above its limit.
void guarded_cycle(Rig& rig, State& s, Command& cmd, bool spike, std::size_t& news) {
  ASSERT_TRUE(rig.io.read(s).is_ok());
  if (spike) {
    s.qd(1) = 10.0;  // > qd_max_2 = 2.5 rad/s
  }
  bool ok = false;
  guarded(news, [&] { ok = rig.loop.step(s, cmd).is_ok(); });
  EXPECT_TRUE(ok);
  ASSERT_TRUE(rig.io.write(cmd).is_ok());
  rig.io.advance();
}

// External mode through every new path: standby, load, tracking, request_stop, the stop ramp,
// standby again, a hold (velocity spike) and reset back to standby. Nothing allocates: not the
// steps, not load() (it copies the trajectory into fixed storage), request_stop() or reset().
// 1951 steps in total stay below the ring's 8191 records, so nothing is dropped either.
TEST(LoopExternalNoAlloc, StepLoadStopAndResetDoNotAllocate) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const std::unique_ptr<Rig> rig = make_rig(*setup, external_config(*setup));
  ASSERT_TRUE(rig->start().is_ok());
  State s;
  Command cmd;
  std::size_t news = 0;
  for (int k = 0; k < 100; ++k) {
    guarded_cycle(*rig, s, cmd, false, news);
  }
  ASSERT_EQ(rig->loop.mode(), LoopMode::kStandby);

  const Pose ref = rig->loop.reference_pose();
  const auto traj = make_trajectory(ref, {waypoint(goal_pose(setup->config), 10.0, true)});
  ASSERT_TRUE(traj);
  bool loaded = false;
  guarded(news, [&] { loaded = rig->loop.load(*traj).is_ok(); });
  ASSERT_TRUE(loaded);
  for (int k = 0; k < 1000; ++k) {
    guarded_cycle(*rig, s, cmd, false, news);
  }
  ASSERT_EQ(rig->loop.mode(), LoopMode::kTracking);

  guarded(news, [&] { rig->loop.request_stop(); });
  for (int k = 0; k < 600; ++k) {
    guarded_cycle(*rig, s, cmd, false, news);
  }
  ASSERT_EQ(rig->loop.mode(), LoopMode::kStandby);

  guarded_cycle(*rig, s, cmd, true, news);
  ASSERT_EQ(rig->loop.mode(), LoopMode::kHolding);
  for (int k = 0; k < 100; ++k) {
    guarded_cycle(*rig, s, cmd, false, news);
  }
  bool reset = false;
  guarded(news, [&] { reset = rig->loop.reset(s).is_ok(); });
  ASSERT_TRUE(reset);
  ASSERT_EQ(rig->loop.mode(), LoopMode::kStandby);
  for (int k = 0; k < 150; ++k) {
    guarded_cycle(*rig, s, cmd, false, news);
  }
  EXPECT_EQ(rig->loop.mode(), LoopMode::kStandby);
  EXPECT_EQ(news, 0U);
  EXPECT_EQ(rig->loop.log().dropped(), 0U);
}

}  // namespace
}  // namespace cobot::pipeline::test
