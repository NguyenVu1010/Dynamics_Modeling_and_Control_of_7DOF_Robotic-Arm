// RobotScene::visual_poses runs at 30 Hz for every drawn run (feature studio-3d): once the caller
// has sized `out`, it must not touch the heap. Same two guards as
// tests/unit/kinematics_no_alloc: Eigen's runtime check and a replaced global operator new
// (every form, so allocation and deallocation always pair up under ASan).
#include <atomic>
#include <cstddef>
#include <cstdlib>
#include <new>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/model/model.hpp"
#include "cobot/studio/robot_scene.hpp"

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

namespace cobot::studio {
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

TEST(RobotSceneNoAlloc, CounterSeesAllocations) {
  long seen = 0;
  {
    const HeapGuard guard;
    void* p = ::operator new(16);  // a direct call, which the compiler may not elide
    ::operator delete(p);
    seen = g_allocations.load();
  }
  EXPECT_EQ(seen, 1);
}

TEST(RobotSceneNoAlloc, SecondVisualPosesCallDoesNotAllocate) {
  const auto scene = RobotScene::load(std::string(COBOT_REPO_DIR) + "/src/sim/urdf/cobot30_5.urdf",
                                      "base_link",
                                      "link_7",
                                      {{"cobot_sim", std::string(COBOT_REPO_DIR) + "/src/sim"}});
  ASSERT_TRUE(scene.is_ok()) << scene.reason();
  std::vector<common::Pose> poses(scene.value().links().size());
  const common::VecN<kN> q0 =
      (common::VecN<kN>() << 0.1, 0.5, -0.2, -1.4, 0.3, 1.0, -0.4).finished();
  const common::VecN<kN> q1 =
      (common::VecN<kN>() << -0.6, 0.2, 0.9, -2.0, -0.5, 2.5, 1.2).finished();
  scene.value().visual_poses(q0, poses);  // first call: may set up whatever it caches
  long allocations = -1;
  {
    const HeapGuard guard;
    scene.value().visual_poses(q1, poses);
    allocations = g_allocations.load();
  }
  EXPECT_EQ(allocations, 0);
  ASSERT_EQ(poses.size(), scene.value().links().size());
  for (const common::Pose& T : poses) {
    EXPECT_TRUE(T.R.allFinite());
    EXPECT_TRUE(T.p.allFinite());
  }
  // The second call really moved the arm: the tip (last link) is not where q0 put it.
  std::vector<common::Pose> at_q0(poses.size());
  scene.value().visual_poses(q0, at_q0);
  EXPECT_GT((poses.back().p - at_q0.back().p).norm(), 1e-3);
}

}  // namespace
}  // namespace cobot::studio
