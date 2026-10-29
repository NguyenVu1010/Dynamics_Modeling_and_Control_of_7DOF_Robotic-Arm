// WaypointTrajectory is copied into the real-time controller and sampled every cycle, so copy
// construction and sample() must not touch the heap. Two guards, as in
// tests/unit/kinematics_no_alloc: Eigen's runtime check and a replaced global operator new
// (every form, so allocation and deallocation always pair up under ASan).
#include <atomic>
#include <cstdlib>
#include <new>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "cobot/planning/waypoint_trajectory.hpp"

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

namespace cobot::planning {
namespace {

using common::Pose;
using common::Vec3;

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

TEST(WaypointTrajectoryNoAlloc, CounterSeesAllocations) {
  long seen = 0;
  {
    const HeapGuard guard;
    void* p = ::operator new(16);  // a direct call, which the compiler may not elide
    ::operator delete(p);
    seen = g_allocations.load();
  }
  EXPECT_EQ(seen, 1);
}

TEST(WaypointTrajectoryNoAlloc, CopyAndSampleDoNotAllocate) {
  // A full list: every slot of the fixed storage is in use.
  std::vector<Waypoint> pts;
  for (std::size_t i = 0; i < kMaxWaypoints; ++i) {
    const double x = 0.01 * static_cast<double>(i);
    Waypoint w;
    w.T_b_goal.R = Eigen::AngleAxisd(10.0 * x, Vec3(0.0, 0.3, 1.0).normalized()).toRotationMatrix();
    w.T_b_goal.p = Vec3(0.3 + x, 0.1 * x, 0.5);
    w.duration_s = i % 3 == 0 ? 0.4 : 0.0;
    w.stop = i % 7 == 6;
    pts.push_back(w);
  }
  const auto built = WaypointTrajectory::create(Pose{}, pts, CartesianLimits{});
  ASSERT_TRUE(built.is_ok()) << built.reason();
  const WaypointTrajectory& traj = built.value();
  const double T = traj.duration_s();

  CartesianSample sum;
  std::size_t segments = 0;
  long allocations = -1;
  {
    const HeapGuard guard;
    const WaypointTrajectory copy(traj);  // NOLINT(performance-unnecessary-copy-initialization)
    for (int k = -10; k <= 1010; ++k) {
      const double t = T * k / 1000.0;
      const CartesianSample s = copy.sample(t);
      sum.T_b_d.p += s.T_b_d.p;
      sum.xd_d += s.xd_d;
      sum.xdd_d += s.xdd_d;
      segments += copy.segment_at(t);
    }
    allocations = g_allocations.load();
  }
  EXPECT_EQ(allocations, 0);
  EXPECT_TRUE(sum.T_b_d.p.allFinite() && sum.xd_d.allFinite() && sum.xdd_d.allFinite());
  EXPECT_GT(segments, 0U);
}

}  // namespace
}  // namespace cobot::planning
