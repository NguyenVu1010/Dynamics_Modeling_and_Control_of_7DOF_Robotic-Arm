// bench::run_queue (src/bench/work_queue.hpp) on its own: every item runs once whatever the number
// of jobs, count 0 is ok, an exception becomes an error, no new item starts after a failure, and
// among several failed items the lowest index wins whatever the order they failed in.
#include <atomic>
#include <chrono>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/result.hpp"

#include "work_queue.hpp"

namespace cobot::bench {
namespace {

using common::Status;
using Clock = std::chrono::steady_clock;

// Spins until `flag` reaches `target`, at most 10 s, so a broken queue fails instead of hanging.
bool wait_for(const std::atomic<int>& flag, int target) {
  const auto deadline = Clock::now() + std::chrono::seconds(10);
  while (flag.load() < target) {
    if (Clock::now() > deadline) {
      return false;
    }
    std::this_thread::yield();
  }
  return true;
}

TEST(RunQueue, CountZeroIsOkAndRunsNothing) {
  for (const unsigned jobs : {0U, 1U, 4U}) {
    int calls = 0;
    const Status st = run_queue(0, jobs, [&calls](std::size_t) {
      ++calls;
      return Status::ok();
    });
    EXPECT_TRUE(st) << jobs;
    EXPECT_EQ(calls, 0) << jobs;
  }
}

TEST(RunQueue, EveryItemRunsExactlyOnce) {
  constexpr std::size_t kCount = 200;
  for (const unsigned jobs : {0U, 1U, 3U, 8U, 500U}) {
    std::vector<std::atomic<int>> runs(kCount);
    const Status st = run_queue(kCount, jobs, [&runs](std::size_t i) {
      runs[i].fetch_add(1);
      return Status::ok();
    });
    ASSERT_TRUE(st) << jobs << ": " << st.reason();
    for (std::size_t i = 0; i < kCount; ++i) {
      EXPECT_EQ(runs[i].load(), 1) << "jobs " << jobs << " item " << i;
    }
  }
}

TEST(RunQueue, AnExceptionIsAnErrorWithItsMessage) {
  for (const unsigned jobs : {1U, 4U}) {
    const Status st = run_queue(5, jobs, [](std::size_t i) -> Status {
      if (i == 2) {
        throw std::runtime_error("item two broke");
      }
      return Status::ok();
    });
    ASSERT_FALSE(st) << jobs;
    EXPECT_EQ(st.reason(), "item two broke") << jobs;
  }
}

TEST(RunQueue, NoNewItemStartsAfterAFailure) {
  std::vector<int> ran;
  const Status st = run_queue(10, 1, [&ran](std::size_t i) {
    ran.push_back(static_cast<int>(i));
    return i == 3 ? Status::error("item 3 failed") : Status::ok();
  });
  ASSERT_FALSE(st);
  EXPECT_EQ(st.reason(), "item 3 failed");
  EXPECT_EQ(ran, (std::vector<int>{0, 1, 2, 3}));
}

// Four items on four threads, all started before any returns: item 3 fails first, item 1 after it.
// The error of item 1 comes back.
TEST(RunQueue, TheLowestFailedIndexWinsWhateverTheOrder) {
  std::atomic<int> started{0};
  std::atomic<int> three_failed{0};
  bool timed_out = false;
  const Status st = run_queue(4, 4, [&](std::size_t i) {
    started.fetch_add(1);
    if (!wait_for(started, 4)) {
      timed_out = true;
      return Status::error("not all items started");
    }
    if (i == 3) {
      three_failed.store(1);
      return Status::error("item 3 failed");
    }
    if (i == 1) {
      wait_for(three_failed, 1);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      return Status::error("item 1 failed");
    }
    return Status::ok();
  });
  ASSERT_FALSE(timed_out);
  ASSERT_FALSE(st);
  EXPECT_EQ(st.reason(), "item 1 failed");
}

}  // namespace
}  // namespace cobot::bench
