// The seqlocks under load (feature shm-bridge, ADR-022 "Các vòng"): a writer thread overwrites as
// fast as it can while a reader thread copies; every sample the reader gets is whole (no field
// from another write), steps strictly increase, and the writer never waits for the reader.
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/bridge/endpoint.hpp"
#include "cobot/bridge/seqlock.hpp"
#include "cobot/bridge/wire.hpp"

#include "bridge_test_util.hpp"

namespace cobot::bridge::test {
namespace {

constexpr std::uint64_t kSamples = 2000000;
constexpr std::uint64_t kJogWrites = 1000000;
constexpr std::uint64_t kReadsAfterEnd = 1000000;

struct ReadTally {
  std::uint64_t read = 0;
  std::uint64_t mixed = 0;         // samples whose fields disagree with their step
  std::uint64_t out_of_order = 0;  // steps not strictly increasing
  std::uint64_t last_step = 0;
};

void tally(const std::vector<Sample>& got, ReadTally& t) {
  for (const Sample& s : got) {
    ++t.read;
    if (!sample_intact(s)) {
      ++t.mixed;
    }
    if (s.step <= t.last_step) {
      ++t.out_of_order;
    }
    t.last_step = s.step;
  }
}

TEST(BridgeSeqlockLoad, TwoMillionSamplesAreNeverMixed) {
  const RegionGuard g;
  auto server = Server::create(g.name(), make_session(1));
  ASSERT_TRUE(server.is_ok()) << server.reason();
  auto client = Client::open(g.name());
  ASSERT_TRUE(client.is_ok()) << client.reason();
  Server& s = server.value();
  Client& c = client.value();

  std::atomic<bool> done{false};
  ReadTally t;
  std::thread reader([&c, &done, &t] {
    std::vector<Sample> got;
    got.reserve(kStateSlots);
    for (;;) {
      const bool last = done.load(std::memory_order_acquire);
      got.clear();
      c.read_states(got);
      tally(got, t);
      if (last) {
        return;  // one final drain after the writer finished
      }
      std::this_thread::yield();
    }
  });
  for (std::uint64_t k = 1; k <= kSamples; ++k) {
    s.publish(make_sample(k));
  }
  done.store(true, std::memory_order_release);
  reader.join();

  std::printf("read %llu, lost %llu of %llu\n",
              static_cast<unsigned long long>(t.read),
              static_cast<unsigned long long>(c.lost()),
              static_cast<unsigned long long>(kSamples));
  EXPECT_EQ(t.mixed, 0U);
  EXPECT_EQ(t.out_of_order, 0U);
  EXPECT_GT(t.read, 0U);
  EXPECT_EQ(t.last_step, kSamples) << "the final drain sees the last sample";
  EXPECT_EQ(t.read + c.lost(), kSamples);
}

// Every field of jog write r is a function of r.
JogRequest jog(std::uint64_t r) {
  JogRequest j{};
  j.owner_token = r * 3 + 1;
  j.refresh = r;
  j.kind = static_cast<std::int32_t>(r % 3);
  j.joint = static_cast<std::int32_t>(r % 7);
  j.axis = static_cast<std::int32_t>(r % 6);
  j.frame = static_cast<std::int32_t>(r % 2);
  j.direction = static_cast<std::int32_t>(r % 3) - 1;
  j.speed = 0.5 * static_cast<double>(r);
  return j;
}

bool jog_intact(const JogRequest& j) {
  const JogRequest w = jog(j.refresh);
  return j.owner_token == w.owner_token && j.kind == w.kind && j.joint == w.joint &&
         j.axis == w.axis && j.frame == w.frame && j.direction == w.direction && j.speed == w.speed;
}

TEST(BridgeSeqlockLoad, JogSlotMillionWritesAreNeverMixed) {
  const auto slot = std::make_unique<SeqSlot<JogRequest>>();
  slot->write(jog(0));
  std::atomic<bool> done{false};
  std::uint64_t reads = 0;
  std::uint64_t torn = 0;
  std::uint64_t mixed = 0;
  std::uint64_t backwards = 0;
  std::uint64_t last = 0;
  std::uint64_t after = 0;
  std::thread reader([&] {
    for (;;) {
      const bool finished = done.load(std::memory_order_acquire);
      JogRequest j{};
      if (slot->try_read(j)) {
        ++reads;
        mixed += jog_intact(j) ? 0 : 1;
        backwards += j.refresh < last ? 1 : 0;
        last = j.refresh;
      } else {
        ++torn;
      }
      // Stops once the last write is seen, or a bounded number of reads after the writer ended.
      after += finished ? 1 : 0;
      if ((finished && last == kJogWrites) || after > kReadsAfterEnd) {
        return;
      }
    }
  });
  for (std::uint64_t r = 1; r <= kJogWrites; ++r) {
    slot->write(jog(r));
  }
  done.store(true, std::memory_order_release);
  reader.join();

  std::printf("jog reads %llu, torn %llu\n",
              static_cast<unsigned long long>(reads),
              static_cast<unsigned long long>(torn));
  EXPECT_EQ(mixed, 0U);
  EXPECT_EQ(backwards, 0U);
  EXPECT_EQ(last, kJogWrites);
  EXPECT_GT(reads, 0U);
}

// The jog slot of the region under load: the client writes, the server reads. Write k carries
// joint = k % 7 and speed = k / 4; one client, so its refresh counter is k.
TEST(BridgeSeqlockLoad, RegionJogSlotIsNeverMixed) {
  const RegionGuard g;
  auto server = Server::create(g.name(), make_session(1));
  ASSERT_TRUE(server.is_ok()) << server.reason();
  auto client = Client::open(g.name());
  ASSERT_TRUE(client.is_ok()) << client.reason();
  ASSERT_TRUE(client.value().claim(monotonic_ns()).is_ok());
  const Server& s = server.value();
  const std::uint64_t token = s.owner_token();
  constexpr std::uint64_t kWrites = 200000;

  std::atomic<bool> done{false};
  std::uint64_t mixed = 0;
  std::uint64_t backwards = 0;
  std::uint64_t last = 0;
  std::uint64_t after = 0;
  std::thread reader([&] {
    for (;;) {
      const bool finished = done.load(std::memory_order_acquire);
      JogRequest j{};
      if (s.read_jog(j)) {
        const bool ok = j.owner_token == token &&
                        j.joint == static_cast<std::int32_t>(j.refresh % 7) &&
                        j.speed == static_cast<double>(j.refresh) / 4.0;
        mixed += ok ? 0 : 1;
        backwards += j.refresh < last ? 1 : 0;
        last = j.refresh;
      }
      // Stops once the last write is seen, or a bounded number of reads after the writer ended.
      after += finished ? 1 : 0;
      if ((finished && last == kWrites) || after > kReadsAfterEnd) {
        return;
      }
    }
  });
  for (std::uint64_t k = 1; k <= kWrites; ++k) {
    JogRequest j{};
    j.kind = 1;
    j.joint = static_cast<std::int32_t>(k % 7);
    j.direction = 1;
    j.speed = static_cast<double>(k) / 4.0;
    client.value().write_jog(j);
  }
  done.store(true, std::memory_order_release);
  reader.join();
  EXPECT_EQ(mixed, 0U);
  EXPECT_EQ(backwards, 0U);
  EXPECT_EQ(last, kWrites);
}

}  // namespace
}  // namespace cobot::bridge::test
