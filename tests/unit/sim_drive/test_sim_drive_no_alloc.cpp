#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <new>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "cobot/pipeline/sim_joint_interface.hpp"

#include "sim_drive_util.hpp"

// SimDrive::advance allocates nothing after construction (brief drive-loop 6, hot path rules):
// every global operator new of this binary is counted while g_counting is set (the drive is
// compiled in the pipeline library without EIGEN_RUNTIME_NO_MALLOC), and Eigen's malloc guard is
// off around the calls. Every form is replaced so allocations and deallocations stay paired under
// ASan.
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

namespace cobot::pipeline::drive_test {
namespace {

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

// The budget profile's actuator group and 17-bit encoder, so the stage and the encoder rounding
// run too; set points every 4 ms (servo 250 Hz) with a moving target and a velocity feed-forward.
TEST(SimDrive, NoAllocation) {
  const auto arm = load_arm();
  ASSERT_TRUE(arm);
  const auto imp = load_imperfections<kN>(std::filesystem::path(COBOT_REPO_DIR) /
                                          "config/imperfections/budget_xarm7.yaml");
  ASSERT_TRUE(imp) << imp.reason();
  const Profile& p = imp.value();
  Plant plant(arm->model, arm->home);
  Drive drive(drive_config(),
              arm->inertia,
              Vec::Constant(30.0),
              p,
              Vec::Constant(1.05),
              kDt,
              1.0 / p.comm.servo_rate_hz);
  std::size_t news = 0;
  const Vec qd = Vec::Constant(0.05);
  for (int k = 0; k < 1000; ++k) {
    const int packet = k / 4;  // a new set point every 4 control periods
    const auto seq = static_cast<std::uint32_t>(1 + packet);
    const Vec q = arm->home + qd * (4e-3 * static_cast<double>(packet));
    const Command cmd = position_command(q, qd, Vec::Constant(0.1), seq);
    guarded(news, [&] { drive.advance(plant, cmd, kDt); });
  }
  EXPECT_EQ(news, 0U) << "operator new calls in 1000 advances";
  EXPECT_EQ(drive.substeps_run(), 4000);
  EXPECT_TRUE(plant.q().allFinite());

  // Through SimJointInterface::advance as well.
  SimJointInterface<kN> io(plant, kDt, 1e-4, 1, 0.5, 2);
  io.attach_drive(&drive);
  for (int k = 0; k < 200; ++k) {
    const Command cmd = position_command(plant.q_motor(), Vec::Zero(), Vec::Zero(), 300);
    guarded(news, [&] {
      static_cast<void>(io.write(cmd));
      io.advance();
    });
  }
  EXPECT_EQ(news, 0U) << "operator new calls through SimJointInterface";
}

}  // namespace
}  // namespace cobot::pipeline::drive_test
