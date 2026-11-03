// ControlLoop::step with the compensation active (enabled, q_from_motor states from the decorator
// with position_source motor) allocates nothing (brief "deflection-comp" contract 4).
#include <cstddef>
#include <cstdlib>
#include <new>

#include <gtest/gtest.h>

#include "cobot/dynamics/sim_plant.hpp"
#include "cobot/imperfect/imperfect_joint_interface.hpp"
#include "cobot/pipeline/control_loop.hpp"
#include "cobot/pipeline/sim_joint_interface.hpp"

#include "deflection_test_util.hpp"

// Counts every global operator new in this binary while g_counting is set, so an allocation
// inside ControlLoop::step() (compiled in the pipeline library, where EIGEN_RUNTIME_NO_MALLOC is
// not defined) is caught too.
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

// Rigid plant (fast) behind the decorator with the motor source: every state is flagged, so the
// compensation runs on every step (the plant does not deflect; only the code path matters here).
TEST(DeflectionCompNoAlloc, StepWithTheCompensationActiveDoesNotAllocate) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  Config<kN> cfg = setup->config;
  enable_comp(cfg);
  const double dt = 1.0 / cfg.control_rate_hz;
  dynamics::SimPlant<kN> plant(setup->plant_model, cfg.home_q_rad);
  SimJointInterface<kN> sim(plant, dt);
  imperfect::ImperfectionConfig<kN> icfg;
  icfg.sensor.position_source = imperfect::PositionSource::kMotor;
  imperfect::ImperfectJointInterface<kN> io(sim, icfg, dt, false, cfg.limits.peak_torque_nm);
  ControlLoop<kN> loop(setup->controller_model, cfg);
  State s;
  Command cmd;
  ASSERT_TRUE(io.read(s).is_ok());
  ASSERT_TRUE(loop.start(s).is_ok());
  std::size_t news = 0;
  int flagged = 0;
  for (int k = 0; k < 500; ++k) {
    ASSERT_TRUE(io.read(s).is_ok());
    flagged += s.q_from_motor ? 1 : 0;
    const std::size_t before = g_new_calls;
    g_counting = true;
    Eigen::internal::set_is_malloc_allowed(false);
    const common::Status st = loop.step(s, cmd);
    Eigen::internal::set_is_malloc_allowed(true);
    g_counting = false;
    news += g_new_calls - before;
    ASSERT_TRUE(st.is_ok()) << "cycle " << k;
    ASSERT_TRUE(io.write(cmd).is_ok());
    sim.advance();
  }
  EXPECT_EQ(flagged, 500);
  EXPECT_EQ(news, 0U);
  EXPECT_EQ(loop.mode(), LoopMode::kTracking);
}

}  // namespace
}  // namespace cobot::pipeline::test
