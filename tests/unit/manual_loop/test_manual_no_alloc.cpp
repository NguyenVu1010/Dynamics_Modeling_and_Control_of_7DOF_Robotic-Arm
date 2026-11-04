#include <cstddef>
#include <cstdlib>
#include <memory>
#include <new>

#include <gtest/gtest.h>

#include "manual_loop_util.hpp"

// Counts every global operator new in this binary while g_counting is set, so a std::vector,
// std::string or dynamic Eigen matrix created inside step(), jog_joint(), jog_tcp(),
// move_joint(), load(..., kMoveTcp), request_stop(), estop() or reset() (compiled in the
// pipeline library, where EIGEN_RUNTIME_NO_MALLOC is not defined) is caught.
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

// One host cycle with only loop.step() under the guards.
void guarded_cycle(Rig& rig, State& s, Command& cmd, std::size_t& news) {
  ASSERT_TRUE(rig.io.read(s).is_ok());
  bool ok = false;
  guarded(news, [&] { ok = rig.loop.step(s, cmd).is_ok(); });
  EXPECT_TRUE(ok);
  ASSERT_TRUE(rig.io.write(cmd).is_ok());
  rig.io.advance();
}

// Guarded cycles until standby (at most `max_steps`), calling `refresh` under the guards every
// kRefreshEvery steps. Returns true when the loop reached standby.
template <typename Refresh>
bool guarded_until_standby(
    Rig& rig, State& s, Command& cmd, std::size_t& news, int max_steps, Refresh refresh) {
  for (int k = 0; k < max_steps; ++k) {
    if (k > 0 && k % kRefreshEvery == 0) {
      guarded(news, refresh);
    }
    guarded_cycle(rig, s, cmd, news);
    if (rig.loop.mode() == LoopMode::kStandby) {
      return true;
    }
  }
  return false;
}

// Every manual path on one loop: joint jog (start, 300 refreshed steps, release to standby),
// TCP jog the same, move_joint to standby, load(kMoveTcp) to standby, estop and reset, a joint
// jog to its soft limit and the rejected jog there. The plans are built off the guards (the
// planners allocate, as a host runs them off the control thread).
TEST(ManualNoAlloc, ManualCommandsAndStepsDoNotAllocate) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const Config<kN> cfg = manual_config(*setup);
  const model::Arm7 planner = planner_model(*setup, cfg);
  const std::unique_ptr<Rig> rig = make_rig(*setup, cfg);
  ASSERT_TRUE(rig->start().is_ok());
  State s;
  Command cmd;
  std::size_t news = 0;
  for (int k = 0; k < 100; ++k) {
    guarded_cycle(*rig, s, cmd, news);
  }
  ASSERT_EQ(rig->loop.mode(), LoopMode::kStandby);
  bool ok = false;

  // Joint jog.
  const auto jog1 = [&] { ok = rig->loop.jog_joint(0, +1, kSpeed).is_ok(); };
  guarded(news, jog1);
  ASSERT_TRUE(ok);
  for (int k = 0; k < 300; ++k) {
    if (k > 0 && k % kRefreshEvery == 0) {
      guarded(news, jog1);
      ASSERT_TRUE(ok);
    }
    guarded_cycle(*rig, s, cmd, news);
  }
  guarded(news, [&] { rig->loop.request_stop(); });
  ASSERT_TRUE(guarded_until_standby(*rig, s, cmd, news, 500, [] {}));
  EXPECT_EQ(news, 0U) << "joint jog";

  // TCP jog.
  const auto jog_x = [&] {
    ok = rig->loop.jog_tcp(JogAxis::kX, -1, JogFrame::kBase, kSpeed).is_ok();
  };
  guarded(news, jog_x);
  ASSERT_TRUE(ok);
  for (int k = 0; k < 300; ++k) {
    if (k > 0 && k % kRefreshEvery == 0) {
      guarded(news, jog_x);
      ASSERT_TRUE(ok);
    }
    guarded_cycle(*rig, s, cmd, news);
  }
  guarded(news, [&] { rig->loop.request_stop(); });
  ASSERT_TRUE(guarded_until_standby(*rig, s, cmd, news, 1000, [] {}));
  EXPECT_EQ(news, 0U) << "TCP jog";

  // move_joint (joint 1 back by 0.05 rad at 50 %).
  {
    const Vec q0 = rig->loop.joint_reference();
    Vec goal = q0;
    goal(0) -= 0.05;
    const auto plan = plan_joint_move<kN>(planner, cfg, q0, goal, 0.5);
    ASSERT_TRUE(plan) << plan.reason();
    guarded(news, [&] { ok = rig->loop.move_joint(plan.value()).is_ok(); });
    ASSERT_TRUE(ok);
    ASSERT_TRUE(guarded_until_standby(*rig, s, cmd, news, 2000, [] {}));
  }
  EXPECT_EQ(news, 0U) << "move_joint";

  // load(kMoveTcp) (2 cm along +x at 100 %).
  {
    const Pose start = rig->loop.reference_pose();
    const Pose goal{start.R, start.p + Vec3(0.02, 0.0, 0.0)};
    const auto plan =
        plan_tcp_move<kN>(planner, cfg, rig->loop.joint_reference(), start, goal, 1.0);
    ASSERT_TRUE(plan) << plan.reason();
    ASSERT_TRUE(plan.value().ok);
    ASSERT_TRUE(plan.value().trajectory.has_value());
    guarded(news,
            [&] { ok = rig->loop.load(*plan.value().trajectory, CommandMode::kMoveTcp).is_ok(); });
    ASSERT_TRUE(ok);
    ASSERT_TRUE(guarded_until_standby(*rig, s, cmd, news, 3000, [] {}));
  }
  EXPECT_EQ(news, 0U) << "move_tcp";

  // estop during a joint jog, hold, reset.
  guarded(news, jog1);
  ASSERT_TRUE(ok);
  for (int k = 0; k < 50; ++k) {
    guarded_cycle(*rig, s, cmd, news);
  }
  guarded(news, [&] { ok = rig->loop.estop().is_ok(); });
  ASSERT_TRUE(ok);
  for (int k = 0; k < 300; ++k) {
    guarded_cycle(*rig, s, cmd, news);
  }
  ASSERT_EQ(rig->loop.mode(), LoopMode::kHolding);
  guarded(news, [&] { ok = rig->loop.estop().is_ok(); });  // no-op in holding
  ASSERT_TRUE(ok);
  guarded(news, [&] { ok = rig->loop.reset(s).is_ok(); });
  ASSERT_TRUE(ok);
  for (int k = 0; k < 50; ++k) {
    guarded_cycle(*rig, s, cmd, news);
  }
  EXPECT_EQ(news, 0U) << "estop";

  // Joint 4 to its jog limit at 50 % (1.05 rad/s, 6.25 rad/s^2: about 1.2 s), then the
  // rejected jog in that direction (a literal reason).
  const auto jog4 = [&] { ok = rig->loop.jog_joint(3, +1, 0.5).is_ok(); };
  guarded(news, jog4);
  ASSERT_TRUE(ok);
  ASSERT_TRUE(guarded_until_standby(*rig, s, cmd, news, 4000, jog4));
  ASSERT_EQ(rig->loop.manual_stop(), ManualStop::kSoftLimit);
  guarded(news, jog4);
  EXPECT_FALSE(ok);
  EXPECT_EQ(news, 0U) << "soft limit";
  EXPECT_EQ(rig->recorder.dropped(), 0U);
}

// A TCP jog refused near a singularity (threshold below every condition number) is a literal.
TEST(ManualNoAlloc, RejectedTcpJogDoesNotAllocate) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  Config<kN> cfg = manual_config(*setup);
  cfg.clik.cond_threshold = 1.0;  // cond >= 1 always; > 1 for this arm
  const std::unique_ptr<Rig> rig = make_rig(*setup, cfg);
  ASSERT_TRUE(rig->start().is_ok());
  State s;
  Command cmd;
  std::size_t news = 0;
  for (int k = 0; k < 10; ++k) {
    guarded_cycle(*rig, s, cmd, news);
  }
  bool ok = true;
  guarded(news, [&] { ok = rig->loop.jog_tcp(JogAxis::kZ, +1, JogFrame::kTool, kSpeed).is_ok(); });
  EXPECT_FALSE(ok);
  // Rejected argument checks are literals too.
  guarded(news, [&] { ok = rig->loop.jog_joint(9, +1, kSpeed).is_ok(); });
  EXPECT_FALSE(ok);
  guarded(news, [&] { ok = rig->loop.jog_tcp(JogAxis::kX, 0, JogFrame::kBase, kSpeed).is_ok(); });
  EXPECT_FALSE(ok);
  EXPECT_EQ(news, 0U);
}

}  // namespace
}  // namespace cobot::pipeline::test
