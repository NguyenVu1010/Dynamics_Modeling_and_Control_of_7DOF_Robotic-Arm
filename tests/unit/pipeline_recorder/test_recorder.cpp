#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/pipeline/recorder.hpp"

#include "recorder_test_util.hpp"

// pipeline::Recorder<N> on HeapStorage (feature pipeline-recorder, design
// docs/design/2026-11-02-cobot-studio-design.md 6.1, ADR-020): the two full policies, the
// double-buffer swap between the control thread and one exporter thread, the storage checks.
namespace cobot::pipeline::test {
namespace {

// Records t = first .. last (inclusive) on `rec`.
void record_range(Recorder<kN>& rec, std::int64_t first, std::int64_t last) {
  for (std::int64_t t = first; t <= last; ++t) {
    rec.record(make_record(t));
  }
}

TEST(HeapStorage, ZeroCapacityOrABufferCountOtherThanOneOrTwoThrows) {
  EXPECT_THROW(HeapStorage<kN>(0, 2), std::invalid_argument);
  EXPECT_THROW(HeapStorage<kN>(0, 1), std::invalid_argument);
  EXPECT_THROW(HeapStorage<kN>(4, 3), std::invalid_argument);
  EXPECT_THROW(HeapStorage<kN>(4, 0), std::invalid_argument);
  EXPECT_NO_THROW(HeapStorage<kN>(4, 1));
  EXPECT_NO_THROW(HeapStorage<kN>(4, 2));
}

TEST(HeapStorage, MemoryDescribesTheBuffers) {
  HeapStorage<kN> one(4, 1);
  const RecorderMemory<kN> m1 = one.memory();
  EXPECT_NE(m1.control, nullptr);
  EXPECT_NE(m1.buffers[0], nullptr);
  EXPECT_EQ(m1.buffers[1], nullptr);  // one buffer
  EXPECT_EQ(m1.capacity, 4U);
  EXPECT_EQ(m1.buffer_count, 1);

  HeapStorage<kN> two(4, 2);
  const RecorderMemory<kN> m2 = two.memory();
  EXPECT_NE(m2.control, nullptr);
  EXPECT_NE(m2.buffers[0], nullptr);
  EXPECT_NE(m2.buffers[1], nullptr);
  EXPECT_NE(m2.buffers[0], m2.buffers[1]);
  EXPECT_EQ(m2.capacity, 4U);
  EXPECT_EQ(m2.buffer_count, 2);
}

// A RecorderMemory that does not describe 1 or 2 buffers of capacity > 0 is rejected.
TEST(Recorder, InvalidMemoryThrows) {
  HeapStorage<kN> two(4, 2);
  const RecorderMemory<kN> ok = two.memory();
  const auto rejects = [](RecorderMemory<kN> m) {
    EXPECT_THROW(Recorder<kN>(m, FullPolicy::kOverwriteOldest), std::invalid_argument);
  };
  RecorderMemory<kN> m = ok;
  m.control = nullptr;
  rejects(m);
  m = ok;
  m.buffers[0] = nullptr;
  rejects(m);
  m = ok;
  m.capacity = 0;
  rejects(m);
  m = ok;
  m.buffer_count = 0;
  rejects(m);
  m = ok;
  m.buffer_count = 3;
  rejects(m);
  m = ok;
  m.buffers[1] = nullptr;  // two buffers announced, one given
  rejects(m);
  EXPECT_NO_THROW(Recorder<kN>(ok, FullPolicy::kOverwriteOldest));
}

TEST(Recorder, NewRecorderIsEmpty) {
  HeapStorage<kN> storage(5, 1);
  const Recorder<kN> rec(storage.memory(), FullPolicy::kStopWhenFull);
  EXPECT_EQ(rec.size(), 0U);
  EXPECT_EQ(rec.capacity(), 5U);
  EXPECT_EQ(rec.policy(), FullPolicy::kStopWhenFull);
  EXPECT_EQ(rec.dropped(), 0U);
  EXPECT_EQ(rec.overwritten(), 0U);
  EXPECT_EQ(rec.recorded(), 0U);
  EXPECT_EQ(rec.active().size(), 0U);
}

// Capacity 5, t = 0..7 (8 records): the first 5 are kept, 8 - 5 = 3 are dropped.
TEST(Recorder, StopWhenFullKeepsTheFirstRowsAndCountsTheRest) {
  HeapStorage<kN> storage(5, 1);
  Recorder<kN> rec(storage.memory(), FullPolicy::kStopWhenFull);
  record_range(rec, 0, 7);
  EXPECT_EQ(rec.size(), 5U);
  EXPECT_EQ(rec.dropped(), 3U);
  EXPECT_EQ(rec.overwritten(), 0U);
  EXPECT_EQ(rec.recorded(), 5U);  // accepted records only
  const Span rows = rec.active();
  EXPECT_EQ(rows.size(), 5U);
  EXPECT_EQ(stamps(rows), range(0, 4));
  for (std::size_t i = 0; i < rows.size(); ++i) {
    expect_row_of_stamp(rows[i]);
  }
}

// Capacity 5, t = 0..11 (12 records): the newest 5 (t = 7..11) are kept, 12 - 5 = 7 were
// overwritten. Written in a ring, t = 10 and 11 sit before t = 7 in memory, so the ordered view
// has two parts.
TEST(Recorder, OverwriteOldestKeepsTheNewestRowsInOrder) {
  HeapStorage<kN> storage(5, 1);
  Recorder<kN> rec(storage.memory(), FullPolicy::kOverwriteOldest);
  EXPECT_EQ(rec.policy(), FullPolicy::kOverwriteOldest);
  record_range(rec, 0, 11);
  EXPECT_EQ(rec.size(), 5U);
  EXPECT_EQ(rec.overwritten(), 7U);
  EXPECT_EQ(rec.dropped(), 0U);
  EXPECT_EQ(rec.recorded(), 12U);  // every record is accepted
  const Span rows = rec.active();
  EXPECT_EQ(rows.size(), 5U);
  EXPECT_NE(rows.first, nullptr);
  EXPECT_NE(rows.second, nullptr) << "a wrapped buffer is a two-part view";
  EXPECT_GT(rows.first_size, 0U);
  EXPECT_GT(rows.second_size, 0U);
  EXPECT_EQ(rows.first_size + rows.second_size, 5U);
  EXPECT_EQ(stamps(rows), range(7, 11));
  for (std::size_t i = 0; i < rows.size(); ++i) {
    expect_row_of_stamp(rows[i]);
  }
}

// Exactly full, nothing overwritten yet: t = 0..4 in one part.
TEST(Recorder, OverwriteOldestExactlyFullIsOnePart) {
  HeapStorage<kN> storage(5, 1);
  Recorder<kN> rec(storage.memory(), FullPolicy::kOverwriteOldest);
  record_range(rec, 0, 4);
  EXPECT_EQ(rec.size(), 5U);
  EXPECT_EQ(rec.overwritten(), 0U);
  const Span rows = rec.active();
  EXPECT_EQ(rows.second_size, 0U);
  EXPECT_EQ(stamps(rows), range(0, 4));
}

// clear() empties the active buffer (the next record is its first row); recorded() is
// monotonic since construction and is not reset.
TEST(Recorder, ClearEmptiesTheActiveBufferButNotTheRecordedCount) {
  HeapStorage<kN> storage(5, 1);
  Recorder<kN> rec(storage.memory(), FullPolicy::kStopWhenFull);
  record_range(rec, 0, 2);
  rec.clear();
  EXPECT_EQ(rec.size(), 0U);
  EXPECT_EQ(rec.active().size(), 0U);
  EXPECT_EQ(rec.recorded(), 3U);
  record_range(rec, 3, 9);  // 7 records into capacity 5: t = 3..7 kept, 2 dropped
  EXPECT_EQ(rec.size(), 5U);
  EXPECT_EQ(stamps(rec.active()), range(3, 7));
  EXPECT_EQ(rec.recorded(), 3U + 5U);
}

TEST(Recorder, OneBufferNeverSwaps) {
  HeapStorage<kN> storage(4, 1);
  Recorder<kN> rec(storage.memory(), FullPolicy::kStopWhenFull);
  record_range(rec, 0, 2);
  EXPECT_FALSE(rec.request_swap());
  rec.record(make_record(3));
  EXPECT_FALSE(rec.swap_done());
  EXPECT_EQ(stamps(rec.active()), range(0, 3));
}

// 2 buffers of 4: the swap is requested by the exporter and performed by the next record().
TEST(Recorder, SwapIsPerformedByTheNextRecordAndRetiresTheOldBuffer) {
  HeapStorage<kN> storage(4, 2);
  Recorder<kN> rec(storage.memory(), FullPolicy::kStopWhenFull);
  record_range(rec, 0, 2);
  ASSERT_TRUE(rec.request_swap());
  EXPECT_FALSE(rec.swap_done()) << "no record() since the request";
  EXPECT_FALSE(rec.request_swap()) << "a swap is already pending";
  EXPECT_EQ(rec.size(), 3U);

  rec.record(make_record(3));
  ASSERT_TRUE(rec.swap_done());
  const Span retired = rec.retired();
  EXPECT_EQ(stamps(retired), range(0, 2));
  for (std::size_t i = 0; i < retired.size(); ++i) {
    expect_row_of_stamp(retired[i]);
  }
  EXPECT_EQ(stamps(rec.active()), range(3, 3));
  EXPECT_EQ(rec.size(), 1U);
  EXPECT_EQ(rec.recorded(), 4U);

  EXPECT_FALSE(rec.request_swap()) << "the retired buffer is not released yet";
  rec.release_retired();
  ASSERT_TRUE(rec.request_swap());

  rec.record(make_record(4));
  ASSERT_TRUE(rec.swap_done());
  EXPECT_EQ(stamps(rec.retired()), range(3, 3));
  EXPECT_EQ(stamps(rec.active()), range(4, 4));
  rec.release_retired();
  EXPECT_EQ(rec.recorded(), 5U);
  EXPECT_EQ(rec.dropped(), 0U);
}

// After a swap the new active buffer starts empty: capacity 4 again before the policy applies.
TEST(Recorder, SwappedInBufferHasTheFullCapacity) {
  HeapStorage<kN> storage(4, 2);
  Recorder<kN> rec(storage.memory(), FullPolicy::kStopWhenFull);
  record_range(rec, 0, 3);  // full
  ASSERT_TRUE(rec.request_swap());
  record_range(rec, 4, 9);  // t = 4..7 fill the new buffer, t = 8, 9 dropped
  ASSERT_TRUE(rec.swap_done());
  EXPECT_EQ(stamps(rec.retired()), range(0, 3));
  EXPECT_EQ(stamps(rec.active()), range(4, 7));
  EXPECT_EQ(rec.dropped(), 2U);
  EXPECT_EQ(rec.recorded(), 8U);
  rec.release_retired();
}

// The ROS node's set-up: two buffers, kOverwriteOldest. A retired buffer that wrapped comes back
// in order as a two-part view. Capacity 3, t = 0..4: buffer keeps t = 2..4, 5 - 3 = 2 overwritten.
TEST(Recorder, RetiredWrappedBufferIsInOrder) {
  HeapStorage<kN> storage(3, 2);
  Recorder<kN> rec(storage.memory(), FullPolicy::kOverwriteOldest);
  record_range(rec, 0, 4);
  ASSERT_TRUE(rec.request_swap());
  rec.record(make_record(5));
  ASSERT_TRUE(rec.swap_done());
  EXPECT_EQ(stamps(rec.retired()), range(2, 4));
  EXPECT_EQ(stamps(rec.active()), range(5, 5));
  EXPECT_EQ(rec.overwritten(), 2U);
  EXPECT_EQ(rec.recorded(), 6U);
  rec.release_retired();
}

// The producer (control thread) records t = 0 .. kCount - 1; the exporter swaps every ~0.5 ms
// and collects each retired span. Every record must come out exactly once and in order.
//
// The producer never lets the active buffer overflow (that would be a drop, i.e. a gap): when
// it is full it waits (test side only) until the exporter has requested a swap that is not
// performed yet, so its next record() swaps. `granted` counts successful request_swap() calls
// (published right after each one), `swaps` the swaps the producer saw: a record() performed a
// swap when size() did not grow (it restarts at 1 in the swapped-in buffer). At most one swap
// is outstanding, so granted > swaps means a swap is pending.
TEST(RecorderConcurrency, SwapsNeverLoseOrRepeatARecord) {
  constexpr std::int64_t kCount = 200000;
  constexpr std::size_t kCapacity = 1024;
  HeapStorage<kN> storage(kCapacity, 2);
  Recorder<kN> rec(storage.memory(), FullPolicy::kStopWhenFull);
  std::atomic<std::uint64_t> granted{0};
  std::atomic<bool> producer_done{false};
  std::vector<std::int64_t> collected;
  collected.reserve(static_cast<std::size_t>(kCount));

  // t = 0 before the threads start, so every later record() has a non-empty active buffer
  // and a swap always shows as size() not growing.
  rec.record(make_record(0));

  std::thread producer([&] {
    std::uint64_t swaps = 0;
    for (std::int64_t t = 1; t < kCount; ++t) {
      while (rec.size() == kCapacity && granted.load(std::memory_order_acquire) <= swaps) {
        std::this_thread::yield();
      }
      const std::size_t before = rec.size();
      rec.record(make_record(t));
      if (rec.size() <= before) {
        ++swaps;
      }
    }
    producer_done.store(true, std::memory_order_release);
  });

  std::thread exporter([&] {
    const auto collect = [&] {
      const Span rows = rec.retired();
      for (std::size_t i = 0; i < rows.size(); ++i) {
        collected.push_back(rows[i].t.ns);
      }
      rec.release_retired();
    };
    for (;;) {
      std::this_thread::sleep_for(std::chrono::microseconds(500));
      const bool finished = producer_done.load(std::memory_order_acquire);
      if (!rec.request_swap()) {
        ADD_FAILURE() << "request_swap refused with nothing pending or retired";
        return;
      }
      granted.fetch_add(1, std::memory_order_release);
      bool swapped = false;
      for (;;) {
        if (rec.swap_done()) {
          swapped = true;
          break;
        }
        if (producer_done.load(std::memory_order_acquire)) {
          swapped = rec.swap_done();  // the last record() may have swapped
          break;
        }
        std::this_thread::yield();
      }
      if (swapped) {
        collect();
      }
      if (finished || !swapped) {
        return;  // nothing more will be recorded; the rest is in the active buffer
      }
    }
  });

  producer.join();
  exporter.join();
  const Span rest = rec.active();
  for (std::size_t i = 0; i < rest.size(); ++i) {
    collected.push_back(rest[i].t.ns);
  }
  EXPECT_EQ(rec.dropped(), 0U);
  EXPECT_EQ(rec.recorded(), static_cast<std::uint64_t>(kCount));
  ASSERT_EQ(collected.size(), static_cast<std::size_t>(kCount));
  for (std::int64_t t = 0; t < kCount; ++t) {
    ASSERT_EQ(collected[static_cast<std::size_t>(t)], t) << "index " << t;
  }
}

// RecordSpan over caller-owned arrays: size() is first_size + second_size and operator[] runs
// through `first` and then `second`.
TEST(RecordSpan, IndexesFirstThenSecondPart) {
  const std::array<Record, 2> a = {make_record(10), make_record(11)};
  const std::array<Record, 3> b = {make_record(12), make_record(13), make_record(14)};
  Span span;
  EXPECT_EQ(span.size(), 0U);
  span.first = a.data();
  span.first_size = a.size();
  span.second = b.data();
  span.second_size = b.size();
  EXPECT_EQ(span.size(), 5U);
  EXPECT_EQ(stamps(span), range(10, 14));
  EXPECT_EQ(&span[1], &a[1]);
  EXPECT_EQ(&span[2], &b[0]);
}

}  // namespace
}  // namespace cobot::pipeline::test
