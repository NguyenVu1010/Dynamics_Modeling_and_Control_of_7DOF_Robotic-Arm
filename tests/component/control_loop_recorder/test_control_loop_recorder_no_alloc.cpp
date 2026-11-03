#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <new>

#include <gtest/gtest.h>

#include "cobot/pipeline/control_loop.hpp"
#include "cobot/pipeline/recorder.hpp"

#include "loop_recorder_util.hpp"

// Counts every global operator new in this binary while g_counting is set, so an allocation in
// ControlLoop::step() or Recorder::record() (compiled in the pipeline library, where
// EIGEN_RUNTIME_NO_MALLOC is not defined) is caught too. Pattern of
// tests/component/control_loop/test_control_loop_no_alloc.cpp.
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

constexpr std::int64_t kStepNs = 1000000;  // 1 / 1000 Hz = 1 ms

// A recorder of 10 rows (kStopWhenFull) under a loop that runs 50 steps: steps 10..49 find it
// full. No step allocates, 50 - 10 = 40 records are dropped, the rows are steps 0..9, and the
// commands are bit-equal to a twin loop without a recorder fed the same states.
TEST(ControlLoopRecorderNoAlloc, StepWithAFullRecorderDoesNotAllocateOrChangeTheOutput) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  constexpr std::size_t kCapacity = 10;
  constexpr int kSteps = 50;
  HeapStorage<kN> storage(kCapacity, 1);
  Recorder<kN> recorder(storage.memory(), FullPolicy::kStopWhenFull);
  dynamics::SimPlant<kN> plant(setup->model, setup->config.home_q_rad);
  SimJointInterface<kN> io(plant, 1.0 / setup->config.control_rate_hz);
  ControlLoop<kN> loop(setup->model, setup->config, &recorder);
  ControlLoop<kN> twin(setup->model, setup->config);
  State s;
  ASSERT_TRUE(io.read(s).is_ok());
  ASSERT_TRUE(loop.start(s).is_ok());
  ASSERT_TRUE(twin.start(s).is_ok());

  std::size_t news = 0;
  for (int k = 0; k < kSteps; ++k) {
    ASSERT_TRUE(io.read(s).is_ok());
    Command cmd;
    Command cmd_twin;
    const std::size_t before = g_new_calls;
    g_counting = true;
    Eigen::internal::set_is_malloc_allowed(false);
    const bool ok = loop.step(s, cmd).is_ok();
    Eigen::internal::set_is_malloc_allowed(true);
    g_counting = false;
    news += g_new_calls - before;
    ASSERT_TRUE(ok) << "step " << k;
    ASSERT_TRUE(twin.step(s, cmd_twin).is_ok()) << "step " << k;
    ASSERT_TRUE(same_bits(cmd.tau, cmd_twin.tau)) << "step " << k;
    ASSERT_TRUE(same_record(loop.last_record(), twin.last_record())) << "step " << k;
    ASSERT_TRUE(io.write(cmd).is_ok());
    io.advance();
  }
  EXPECT_EQ(news, 0U);
  EXPECT_EQ(recorder.size(), kCapacity);
  EXPECT_EQ(recorder.dropped(), static_cast<std::uint64_t>(kSteps) - kCapacity);  // 50 - 10 = 40
  EXPECT_EQ(recorder.recorded(), kCapacity);
  const RecordSpan<kN> rows = recorder.active();
  ASSERT_EQ(rows.size(), kCapacity);
  for (std::size_t k = 0; k < rows.size(); ++k) {
    EXPECT_EQ(rows[k].t.ns, static_cast<std::int64_t>(k) * kStepNs) << "row " << k;
  }
  // last_record() is still set on every step, also when the recorder dropped the row.
  EXPECT_EQ(loop.last_record().t.ns, static_cast<std::int64_t>(kSteps - 1) * kStepNs);
}

}  // namespace
}  // namespace cobot::pipeline::test
