#include <cstddef>
#include <cstdlib>
#include <limits>
#include <new>

#include <gtest/gtest.h>

#include "cobot/control/computed_torque.hpp"
#include "cobot/control/hold.hpp"

#include "control_test_util.hpp"

// Counts every global operator new in this binary, so a std::vector, std::string or dynamic
// Eigen matrix created inside compute() is caught, not only Eigen's own allocations.
namespace {
std::size_t g_new_calls = 0;  // test-only
}  // namespace

void* operator new(std::size_t size) {
  ++g_new_calls;
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

namespace cobot::control {
namespace {

TEST(ControlNoAlloc, ComputedTorqueComputeDoesNotAllocate) {
  const auto arm = test::load_arm();
  ASSERT_TRUE(arm);
  const ComputedTorque<7> ct(arm->model, ComputedTorqueGains<7>{});
  JointState<7> s;
  JointReference<7> r;
  s.q = arm->home;
  s.qd.setConstant(0.1);
  r.q = arm->home;
  r.q += VecN<7>::Constant(0.01);
  r.qdd.setConstant(0.2);
  JointReference<7> bad = r;
  bad.q(0) = std::numeric_limits<double>::quiet_NaN();
  VecN<7> sum = VecN<7>::Zero();
  const std::size_t before = g_new_calls;
  Eigen::internal::set_is_malloc_allowed(false);
  for (int k = 0; k < 10; ++k) {
    sum += ct.compute(s, r).tau;
    sum += ct.compute(s, bad).tau;
  }
  Eigen::internal::set_is_malloc_allowed(true);
  EXPECT_EQ(g_new_calls, before);
  EXPECT_TRUE(sum.allFinite());
}

TEST(ControlNoAlloc, HoldComputeDoesNotAllocate) {
  const auto arm = test::load_arm();
  ASSERT_TRUE(arm);
  HoldController<7> hold(arm->model, ComputedTorqueGains<7>{});
  hold.engage(arm->home);
  JointState<7> s;
  s.q = arm->home;
  s.q += VecN<7>::Constant(0.02);
  s.qd.setConstant(0.1);
  VecN<7> sum = VecN<7>::Zero();
  const std::size_t before = g_new_calls;
  Eigen::internal::set_is_malloc_allowed(false);
  for (int k = 0; k < 10; ++k) {
    sum += hold.compute(s).tau;
  }
  Eigen::internal::set_is_malloc_allowed(true);
  EXPECT_EQ(g_new_calls, before);
  EXPECT_TRUE(sum.allFinite());
}

}  // namespace
}  // namespace cobot::control
