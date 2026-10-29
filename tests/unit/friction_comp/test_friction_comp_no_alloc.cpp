#include <cstddef>
#include <cstdlib>
#include <limits>
#include <new>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/control/friction_compensation.hpp"

// Counts every global operator new in this binary, so a std::vector, std::string or dynamic
// Eigen matrix created inside torque() is caught, not only Eigen's own allocations.
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

namespace cobot::control {
namespace {

TEST(FrictionCompNoAlloc, TorqueDoesNotAllocate) {
  FrictionCompensationConfig<7> c;
  c.enabled = true;
  c.coulomb_nm.setConstant(1.0);
  c.viscous_nm_s_rad.setConstant(0.2);
  const FrictionCompensation<7> on(c);
  c.enabled = false;
  const FrictionCompensation<7> off(c);
  common::VecN<7> qd;
  qd << 0.3, -0.2, 0.005, 0.0, 1.0, -1.5, 0.01;
  common::VecN<7> bad = qd;
  bad(2) = std::numeric_limits<double>::quiet_NaN();
  common::VecN<7> sum = common::VecN<7>::Zero();
  const std::size_t before = g_new_calls;
  Eigen::internal::set_is_malloc_allowed(false);
  for (int k = 0; k < 10; ++k) {
    sum += on.torque(qd);
    sum += on.torque(bad);
    sum += off.torque(qd);
  }
  Eigen::internal::set_is_malloc_allowed(true);
  EXPECT_EQ(g_new_calls, before);
  EXPECT_TRUE(sum.allFinite());
}

}  // namespace
}  // namespace cobot::control
