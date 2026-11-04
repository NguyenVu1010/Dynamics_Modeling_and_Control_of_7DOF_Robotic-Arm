// record() on ShmStorage memory never allocates (feature blackbox-log, cobot_studio design 6.1):
// only where the memory lives changes, the Recorder code is the same as on HeapStorage.
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <new>

#include <gtest/gtest.h>

#include "cobot/pipeline/recorder.hpp"
#include "cobot/pipeline/recorder_shm.hpp"

#include "shm_test_util.hpp"

// Counts every global operator new in this binary while g_counting is set, so an allocation
// inside Recorder or the storage (compiled in the pipeline library, where EIGEN_RUNTIME_NO_MALLOC
// is not defined) is caught too. Pattern of
// tests/unit/pipeline_recorder/test_recorder_no_alloc.cpp.
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

namespace cobot::pipeline::shm_test {
namespace {

constexpr int kRecords = 10000;
constexpr int kSwapEvery = 100;        // records between swap requests
constexpr std::size_t kCapacity = 64;  // < kSwapEvery: every buffer wraps

// kRecords record() calls on the two shm buffers, a swap requested every kSwapEvery records and
// the retired rows read and released, with only the recorder calls under the counters. The
// region and the records are made before counting starts.
TEST(ShmRecorderNoAlloc, RecordAndSwapOnShmMemoryDoNotAllocate) {
  const RegionGuard g;
  auto storage = Storage::create(g.name(), kCapacity, false);
  ASSERT_TRUE(storage.is_ok()) << storage.reason();
  Recorder<kDof> rec(storage.value().memory(), FullPolicy::kOverwriteOldest);
  const Rec a = make_record(1);
  const Rec b = make_record(2);
  std::int64_t sum = 0;
  int swaps = 0;
  g_new_calls = 0;
  g_counting = true;
  Eigen::internal::set_is_malloc_allowed(false);
  for (int k = 0; k < kRecords; ++k) {
    rec.record(k % 2 == 0 ? a : b);
    if (k % kSwapEvery == kSwapEvery - 1) {
      if (rec.swap_done()) {
        const RecordSpan<kDof> rows = rec.retired();
        for (std::size_t i = 0; i < rows.size(); ++i) {
          sum += rows[i].t.ns;
        }
        rec.release_retired();
        ++swaps;
      }
      static_cast<void>(rec.request_swap());
    }
  }
  static_cast<void>(rec.recorded());
  static_cast<void>(rec.overwritten());
  Eigen::internal::set_is_malloc_allowed(true);
  g_counting = false;
  // A swap requested after record k = 99, 199, ... is done by record k + 1 and collected at
  // k + 100: 10000 / 100 - 1 = 99 collected swaps.
  EXPECT_EQ(swaps, kRecords / kSwapEvery - 1);
  EXPECT_GT(sum, 0);
  EXPECT_EQ(rec.recorded(), static_cast<std::uint64_t>(kRecords));
  EXPECT_EQ(g_new_calls, 0U);
}

}  // namespace
}  // namespace cobot::pipeline::shm_test
