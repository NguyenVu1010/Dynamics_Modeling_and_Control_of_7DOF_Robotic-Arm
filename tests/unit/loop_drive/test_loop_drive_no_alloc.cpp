#include <cstddef>
#include <cstdlib>
#include <limits>
#include <memory>
#include <new>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "loop_drive_util.hpp"

// ControlLoop::step() with drive_position allocates nothing (CONTRIBUTING "Hot path rules",
// ADR-008), in config mode and while holding (the hold goes through the drive): every global
// operator new of this binary is counted while g_counting is set (the loop is compiled in the
// pipeline library without EIGEN_RUNTIME_NO_MALLOC), and Eigen's malloc guard is off around each
// step. Every form is replaced so allocations and deallocations stay paired under ASan.
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

// `steps` host cycles with only loop.step() under both guards; `nan_at` >= 0 puts a NaN in the
// state of that step (-> kHolding); `from_motor` sets q_from_motor (deflection shift active).
// Returns the operator new calls.
std::size_t guarded_steps(DriveRig& rig, int steps, int nan_at = -1, bool from_motor = false) {
  State s;
  Command cmd;
  std::size_t news = 0;
  for (int k = 0; k < steps; ++k) {
    EXPECT_TRUE(rig.io.read(s).is_ok());
    s.q_from_motor = from_motor;
    if (k == nan_at) {
      s.q(1) = std::numeric_limits<double>::quiet_NaN();
    }
    const std::size_t before = g_new_calls;
    g_counting = true;
    Eigen::internal::set_is_malloc_allowed(false);
    static_cast<void>(rig.loop.step(s, cmd));
    Eigen::internal::set_is_malloc_allowed(true);
    g_counting = false;
    news += g_new_calls - before;
    EXPECT_TRUE(rig.io.write(cmd).is_ok());
    rig.io.advance();
  }
  return news;
}

TEST(LoopDrive, NoAllocation) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  for (const control::DriveFeedForward ff : {control::DriveFeedForward::kNone,
                                             control::DriveFeedForward::kGravity,
                                             control::DriveFeedForward::kInverseDynamics}) {
    Config<kN> c = drive_config(setup->config, ff);
    c.deflection_comp.enabled = true;  // inactive without q_from_motor, constructed anyway
    c.deflection_comp.stiffness_k1_nm_rad.setConstant(31000.0);
    c.deflection_comp.stiffness_k2_nm_rad.setConstant(50000.0);
    c.deflection_comp.stiffness_k3_nm_rad.setConstant(57000.0);
    c.deflection_comp.torque_t1_nm.setConstant(14.0);
    c.deflection_comp.torque_t2_nm.setConstant(48.0);
    const auto rig = std::make_unique<DriveRig>(setup->model, c);
    ASSERT_TRUE(rig->start().is_ok());
    EXPECT_EQ(guarded_steps(*rig, 1000), 0U) << "config mode";
    EXPECT_EQ(rig->loop.mode(), LoopMode::kTracking);
    EXPECT_EQ(guarded_steps(*rig, 500, 0), 0U) << "holding";
    EXPECT_EQ(rig->loop.mode(), LoopMode::kHolding);

    const auto shifted = std::make_unique<DriveRig>(setup->model, c);
    ASSERT_TRUE(shifted->start().is_ok());
    EXPECT_EQ(guarded_steps(*shifted, 500, -1, true), 0U) << "deflection shift";
    EXPECT_EQ(guarded_steps(*shifted, 200, 0, true), 0U) << "deflection shift, holding";
  }
}

}  // namespace
}  // namespace cobot::pipeline::test
