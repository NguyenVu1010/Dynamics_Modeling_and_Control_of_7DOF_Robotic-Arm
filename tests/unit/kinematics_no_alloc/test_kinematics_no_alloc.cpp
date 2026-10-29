// forward_kinematics, geometric_jacobian and jdot_qdot run in every control cycle (CLIK and the
// loop's log record), so for N = 7 they must not touch the heap. Same two guards as
// tests/unit/dynamics/test_no_alloc.cpp: Eigen's runtime check and a replaced global operator
// new (every form, so allocation and deallocation always pair up under ASan).
#include <atomic>
#include <cstdlib>
#include <new>
#include <string>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "cobot/kinematics/kinematics.hpp"
#include "cobot/model/model.hpp"

namespace {

std::atomic<bool> g_counting{false};
std::atomic<long> g_allocations{0};

void* counted_alloc(std::size_t size) {
  if (g_counting.load(std::memory_order_relaxed)) {
    g_allocations.fetch_add(1, std::memory_order_relaxed);
  }
  void* p = std::malloc(size == 0 ? 1 : size);  // NOLINT(cppcoreguidelines-no-malloc)
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
  std::free(p);  // NOLINT(cppcoreguidelines-no-malloc)
}
void operator delete[](void* p) noexcept {
  std::free(p);  // NOLINT(cppcoreguidelines-no-malloc)
}
void operator delete(void* p, std::size_t /*size*/) noexcept {
  std::free(p);  // NOLINT(cppcoreguidelines-no-malloc)
}
void operator delete[](void* p, std::size_t /*size*/) noexcept {
  std::free(p);  // NOLINT(cppcoreguidelines-no-malloc)
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

namespace cobot::kinematics {
namespace {

constexpr int kN = model::kArmDof;

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

TEST(KinematicsNoAlloc, CounterSeesAllocations) {
  long seen = 0;
  {
    const HeapGuard guard;
    void* p = ::operator new(16);  // a direct call, which the compiler may not elide
    ::operator delete(p);
    seen = g_allocations.load();
  }
  EXPECT_EQ(seen, 1);
}

TEST(KinematicsNoAlloc, HotPathFunctionsDoNotAllocate) {
  const auto chain = model::parse_urdf_chain(
      std::string(COBOT_REPO_DIR) + "/sim/urdf/cobot30_5.urdf", "base_link", "link_7");
  ASSERT_TRUE(chain.is_ok()) << chain.reason();
  const auto arm = model::make_model<kN>(chain.value(), Vec3(0.0, 0.0, -9.80665));
  ASSERT_TRUE(arm.is_ok()) << arm.reason();
  const VecN<kN> q = (VecN<kN>() << 0.1, 0.5, -0.2, -1.4, 0.3, 1.0, -0.4).finished();
  const VecN<kN> qd = (VecN<kN>() << 0.3, -0.2, 0.5, 0.1, -0.4, 0.6, -0.1).finished();
  FkState<kN> fk;
  Mat6N<kN> J = Mat6N<kN>::Zero();
  Vec6 a = Vec6::Zero();
  long allocations = -1;
  {
    const HeapGuard guard;
    forward_kinematics(arm.value(), q, fk);
    geometric_jacobian(fk, J);
    a = jdot_qdot(arm.value(), fk, qd);
    allocations = g_allocations.load();
  }
  EXPECT_EQ(allocations, 0);
  EXPECT_TRUE(fk.T_b_e.p.allFinite());
  EXPECT_TRUE(J.allFinite());
  EXPECT_GT(J.norm(), 0.0);
  EXPECT_TRUE(a.allFinite());
}

}  // namespace
}  // namespace cobot::kinematics
