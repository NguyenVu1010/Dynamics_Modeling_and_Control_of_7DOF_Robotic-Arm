// rnea, mass_matrix, gravity_torque, coriolis_torque and forward_dynamics run in the control
// loop, so for N = 7 they must not touch the heap. Two guards: Eigen's runtime check
// (EIGEN_RUNTIME_NO_MALLOC, aborts on an Eigen heap allocation) and a replaced global
// operator new that counts every C++ allocation while armed (std::vector, std::string, ...).
#include <atomic>
#include <cstdlib>
#include <new>
#include <random>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "cobot/dynamics/dynamics.hpp"

#include "dynamics_test_util.hpp"

namespace {

std::atomic<bool> g_counting{false};
std::atomic<long> g_allocations{0};

void* counted_alloc(std::size_t size) {
  if (g_counting.load(std::memory_order_relaxed)) {
    g_allocations.fetch_add(1, std::memory_order_relaxed);
  }
  void* p = std::malloc(size == 0 ? 1 : size);
  if (p == nullptr) {
    throw std::bad_alloc();
  }
  return p;
}

}  // namespace

void* operator new(std::size_t size) {
  return counted_alloc(size);
}
void* operator new[](std::size_t size) {
  return counted_alloc(size);
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

namespace cobot::dynamics {
namespace {

using test::Arm7;
using test::kArmDof;

constexpr int kN = kArmDof;
using Vec7 = VecN<kN>;
using Mat7 = MatN<kN>;

class HeapGuard {
 public:
  HeapGuard() {
    g_allocations.store(0);
    g_counting.store(true);
    Eigen::internal::set_is_malloc_allowed(false);
  }
  ~HeapGuard() {
    Eigen::internal::set_is_malloc_allowed(true);
    g_counting.store(false);
  }
  HeapGuard(const HeapGuard&) = delete;
  HeapGuard& operator=(const HeapGuard&) = delete;
  HeapGuard(HeapGuard&&) = delete;
  HeapGuard& operator=(HeapGuard&&) = delete;
};

TEST(DynamicsNoAlloc, CounterSeesAllocations) {
  long seen = 0;
  {
    const HeapGuard guard;
    // A direct call, not a new-expression, so the compiler may not elide it.
    void* p = ::operator new(16);
    ::operator delete(p);
    seen = g_allocations.load();
  }
  EXPECT_EQ(seen, 1);
}

TEST(DynamicsNoAlloc, HotPathFunctionsDoNotAllocate) {
  Arm7 arm;
  ASSERT_TRUE(test::load_arm(arm));
  std::mt19937 rng(23);
  const Vec7 q = test::random_q(arm, rng);
  const Vec7 qd = test::random_sym<kN>(1.0, rng);
  const Vec7 qdd = test::random_sym<kN>(2.0, rng);
  const Vec7 tau = test::random_sym<kN>(5.0, rng);
  Vec7 r_tau;
  Mat7 r_mass;
  Vec7 r_grav;
  Vec7 r_cor;
  Vec7 r_qdd;
  long allocations = -1;
  {
    const HeapGuard guard;
    r_tau = rnea(arm, q, qd, qdd, arm.gravity_b);
    r_mass = mass_matrix(arm, q);
    r_grav = gravity_torque(arm, q);
    r_cor = coriolis_torque(arm, q, qd);
    r_qdd = forward_dynamics(arm, q, qd, tau);
    allocations = g_allocations.load();
  }
  EXPECT_EQ(allocations, 0);
  EXPECT_TRUE(r_tau.allFinite());
  EXPECT_TRUE(r_mass.allFinite());
  EXPECT_TRUE(r_grav.allFinite());
  EXPECT_TRUE(r_cor.allFinite());
  EXPECT_TRUE(r_qdd.allFinite());
}

}  // namespace
}  // namespace cobot::dynamics
