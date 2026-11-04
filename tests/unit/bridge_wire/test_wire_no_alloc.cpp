// to_sample and the seqlock slot never allocate (feature shm-bridge, ADR-022, ADR-008): both run
// on the control thread every step. The global operator new is replaced, so this is a binary of
// its own.
#include <cstddef>
#include <cstdint>

#include <gtest/gtest.h>

#include "cobot/bridge/seqlock.hpp"
#include "cobot/bridge/wire.hpp"
#include "cobot/pipeline/log_record.hpp"

#include "count_new.hpp"

namespace cobot::bridge {
namespace {

constexpr int kDof = 7;
constexpr int kCalls = 10000;

TEST(BridgeWireNoAlloc, ToSampleAndSeqSlotDoNotAllocate) {
  pipeline::LogRecord<kDof> r;
  r.t.ns = 5;
  r.q.setConstant(0.5);
  SeqSlot<StateSample<kDof>> slot{};
  StateSample<kDof> out{};
  double sum = 0.0;
  std::size_t reads = 0;
  {
    const count_new::Scope scope;
    Eigen::internal::set_is_malloc_allowed(false);
    for (int k = 0; k < kCalls; ++k) {
      r.t.ns = k;
      const StateSample<kDof> s = to_sample(r, static_cast<std::uint64_t>(k) + 1, 0);
      slot.write(s);
      if (slot.try_read(out)) {
        sum += out.q[0];
        ++reads;
      }
    }
    Eigen::internal::set_is_malloc_allowed(true);
    EXPECT_EQ(count_new::Scope::count(), 0U);
  }
  EXPECT_EQ(reads, static_cast<std::size_t>(kCalls));  // one thread: every read is stable
  EXPECT_EQ(out.step, static_cast<std::uint64_t>(kCalls));
  EXPECT_EQ(sum, 0.5 * kCalls);
}

}  // namespace
}  // namespace cobot::bridge
