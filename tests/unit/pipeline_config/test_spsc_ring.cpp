#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/pipeline/spsc_ring.hpp"

namespace cobot::pipeline {
namespace {

TEST(SpscRing, FifoOrderAndEmptyPop) {
  SpscRing<int, 8> ring;
  int out = -1;
  EXPECT_FALSE(ring.pop(out));
  EXPECT_EQ(out, -1);
  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(ring.push(i));
  }
  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(ring.pop(out));
    EXPECT_EQ(out, i);
  }
  EXPECT_FALSE(ring.pop(out));
  EXPECT_EQ(ring.dropped(), 0U);
}

TEST(SpscRing, HoldsCapacityMinusOneAndCountsDrops) {
  SpscRing<int, 8> ring;
  for (int i = 0; i < 7; ++i) {
    EXPECT_TRUE(ring.push(i)) << i;
  }
  EXPECT_FALSE(ring.push(7));
  EXPECT_EQ(ring.dropped(), 1U);
  EXPECT_FALSE(ring.push(8));
  EXPECT_EQ(ring.dropped(), 2U);
  int out = -1;
  for (int i = 0; i < 7; ++i) {
    ASSERT_TRUE(ring.pop(out));
    EXPECT_EQ(out, i);
  }
  EXPECT_FALSE(ring.pop(out));
}

TEST(SpscRing, WrapsAround) {
  SpscRing<int, 4> ring;
  int out = -1;
  for (int i = 0; i < 100; ++i) {
    ASSERT_TRUE(ring.push(i));
    ASSERT_TRUE(ring.push(i + 1000));
    ASSERT_TRUE(ring.pop(out));
    EXPECT_EQ(out, i);
    ASSERT_TRUE(ring.pop(out));
    EXPECT_EQ(out, i + 1000);
  }
  EXPECT_EQ(ring.dropped(), 0U);
}

TEST(SpscRing, ProducerConsumerThreads) {
  constexpr int kCount = 1000000;
  SpscRing<int, 1024> ring;
  std::vector<int> received;
  received.reserve(kCount);
  std::thread consumer([&] {
    int out = 0;
    while (static_cast<int>(received.size()) < kCount) {
      if (ring.pop(out)) {
        received.push_back(out);
      }
    }
  });
  std::thread producer([&] {
    for (int i = 0; i < kCount; ++i) {
      while (!ring.push(i)) {
      }
    }
  });
  producer.join();
  consumer.join();
  ASSERT_EQ(static_cast<int>(received.size()), kCount);
  for (int i = 0; i < kCount; ++i) {
    ASSERT_EQ(received[i], i) << "index " << i;
  }
  int out = 0;
  EXPECT_FALSE(ring.pop(out));
}

}  // namespace
}  // namespace cobot::pipeline
