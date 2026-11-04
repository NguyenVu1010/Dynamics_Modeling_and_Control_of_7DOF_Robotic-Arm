// The studio bridge between two processes (feature shm-bridge, cobot_studio design 7 "Cầu nối",
// ADR-022 "Hệ quả"): forked children play the controller or a studio; every child leaves with
// _exit and the parent always reaps it (Child kills and reaps on every exit path).
//   - order and content of the state samples across processes, read + lost == published;
//   - a slow reader loses samples but the writer never waits;
//   - a wrong layout version is refused by the other process;
//   - a stopped studio loses its heartbeat after 200 ms;
//   - a second studio client is refused until the first releases.
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <fcntl.h>
#include <string>
#include <unistd.h>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/bridge/endpoint.hpp"
#include "cobot/bridge/wire.hpp"

#include "bridge_test_util.hpp"

namespace cobot::bridge::test {
namespace {

constexpr auto kChildTimeout = std::chrono::milliseconds(60000);
constexpr int kPipeTimeoutMs = 30000;
constexpr std::uint64_t kPublished = 20000;
constexpr std::int64_t kPeriodNs = 50000;  // 50 us between samples: 20000 samples in 1 s

// What the server child measured: its longest publish() and the wall time of the whole paced run.
struct WriterReport {
  std::int64_t max_publish_ns = 0;
  std::int64_t total_ns = 0;
  std::uint64_t published = 0;
};

void sleep_until(std::int64_t t_ns) {
  timespec ts{};
  ts.tv_sec = static_cast<time_t>(t_ns / 1000000000);
  ts.tv_nsec = static_cast<long>(t_ns % 1000000000);
  while (::clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {
  }
}

// Child: creates the region, says "ready", waits for "go", publishes kPublished samples paced at
// kPeriodNs with clock_nanosleep (absolute deadlines) and reports its timing.
int server_child(const std::string& name, const Pipe& ready, const Pipe& go, const Pipe& report) {
  auto server = Server::create(name, make_session(1));
  if (!server.is_ok()) {
    return 3;
  }
  char byte = 0;
  if (!ready.put(byte) || !go.get_within(byte, kPipeTimeoutMs)) {
    return 4;
  }
  WriterReport r;
  const std::int64_t start = monotonic_ns();
  std::int64_t deadline = start;
  for (std::uint64_t k = 1; k <= kPublished; ++k) {
    const Sample s = make_sample(k);  // outside the timed window
    deadline += kPeriodNs;
    sleep_until(deadline);
    const std::int64_t t0 = monotonic_ns();
    server.value().publish(s);
    const std::int64_t dt = monotonic_ns() - t0;
    r.max_publish_ns = dt > r.max_publish_ns ? dt : r.max_publish_ns;
    ++r.published;
  }
  r.total_ns = monotonic_ns() - start;
  return report.put(r) ? 0 : 5;
}

struct ReaderOutcome {
  std::uint64_t read = 0;
  std::uint64_t lost = 0;
  std::uint64_t mixed = 0;
  std::uint64_t out_of_order = 0;
  std::uint64_t last_step = 0;
  WriterReport writer;
  int child_exit = -1;
};

void tally(const std::vector<Sample>& got, ReaderOutcome& o) {
  for (const Sample& s : got) {
    ++o.read;
    o.mixed += sample_intact(s) ? 0 : 1;
    o.out_of_order += s.step <= o.last_step ? 1 : 0;
    o.last_step = s.step;
  }
}

// Forks the server child, opens the region and reads every `period_ms` until the child reports,
// then drains once more.
ReaderOutcome run_reader(int period_ms) {
  ReaderOutcome o;
  const RegionGuard g;
  const Pipe ready;
  const Pipe go;
  const Pipe report;
  if (!ready.ok() || !go.ok() || !report.ok()) {
    ADD_FAILURE() << "pipe";
    return o;
  }
  const std::string& name = g.name();
  Child child(fork_child([&] { return server_child(name, ready, go, report); }));
  char byte = 0;
  if (!ready.get_within(byte, kPipeTimeoutMs)) {
    ADD_FAILURE() << "server child not ready, exit " << child.wait_exit(kChildTimeout);
    return o;
  }
  auto client = Client::open(name);
  if (!client.is_ok()) {
    ADD_FAILURE() << "open: " << client.reason();
    return o;
  }
  std::vector<Sample> got;
  got.reserve(kStateSlots);
  if (!go.put(byte)) {
    ADD_FAILURE() << "go";
    return o;
  }
  const auto deadline = std::chrono::steady_clock::now() + kChildTimeout;
  bool reported = false;
  while (!reported && std::chrono::steady_clock::now() < deadline) {
    reported = report.readable(period_ms);
    got.clear();
    client.value().read_states(got);  // the last pass after the report is the final drain
    tally(got, o);
  }
  EXPECT_TRUE(reported) << "the server child never reported";
  if (reported) {
    EXPECT_TRUE(report.get(o.writer));
  }
  o.lost = client.value().lost();
  o.child_exit = child.wait_exit(kChildTimeout);
  std::printf(
      "reader every %d ms: read %llu, lost %llu; writer max publish %.3f us, total %.3f s\n",
      period_ms,
      static_cast<unsigned long long>(o.read),
      static_cast<unsigned long long>(o.lost),
      static_cast<double>(o.writer.max_publish_ns) / 1e3,
      static_cast<double>(o.writer.total_ns) / 1e9);
  return o;
}

TEST(BridgeTwoProcess, OrderAndContentAcrossProcesses) {
  const ReaderOutcome o = run_reader(1);
  EXPECT_EQ(o.child_exit, 0);
  EXPECT_EQ(o.writer.published, kPublished);
  EXPECT_EQ(o.mixed, 0U) << "a sample mixed from two writes";
  EXPECT_EQ(o.out_of_order, 0U);
  EXPECT_GT(o.read, 0U);
  EXPECT_EQ(o.last_step, kPublished);
  EXPECT_EQ(o.read + o.lost, kPublished);
}

// The ring spans 4096 * 50 us = 204.8 ms of samples; a reader that comes back every 300 ms finds
// about 6000 new samples each time, so it must lose some. (The brief says "every 50 ms", which at
// 50 us per sample is 1000 samples, fewer than the 4096 slots: nothing would ever be lost.)
TEST(BridgeTwoProcess, SlowReaderLosesSamplesButTheWriterNeverWaits) {
  const ReaderOutcome o = run_reader(300);
  EXPECT_EQ(o.child_exit, 0);
  EXPECT_EQ(o.writer.published, kPublished);
  EXPECT_EQ(o.mixed, 0U);
  EXPECT_EQ(o.out_of_order, 0U);
  EXPECT_EQ(o.read + o.lost, kPublished);
  // Wall-clock figures are reported, not gated: shared CI runners (and arm64 under QEMU) wake late
  // from clock_nanosleep and preempt publish(), whatever the code does. The writer has no wait by
  // construction; its timing is gated by PF-02 (cobot_bench timing --paced) in optimised builds.
  // Losses need the writer to cover more than the ring (4096 * 50 us) between two reads, which
  // only a writer near its nominal pace does, so they are checked only then.
  const bool on_pace =
      o.writer.total_ns < static_cast<std::int64_t>(kPublished) * kPeriodNs * 3 / 2;
  if (on_pace) {
    EXPECT_GT(o.lost, 0U);
  }
  ::testing::Test::RecordProperty("max_publish_ns", std::to_string(o.writer.max_publish_ns));
  ::testing::Test::RecordProperty("writer_total_ns", std::to_string(o.writer.total_ns));
}

TEST(BridgeTwoProcess, WrongLayoutVersionIsRefusedByTheOtherProcess) {
  const RegionGuard g;
  ASSERT_TRUE(Server::create(g.name(), make_session(1)).is_ok());
  {
    RawRegion raw(g.name());
    ASSERT_TRUE(raw.ok());
    const Identity id = find_identity(raw);
    ASSERT_TRUE(id.version) << "layout version not found after the magic";
    raw.write(*id.version, static_cast<std::uint32_t>(kBridgeLayoutVersion + 1));
  }
  const std::string& name = g.name();
  Child child(fork_child([&name] {
    const auto c = Client::open(name);
    if (c.is_ok()) {
      return 0;
    }
    return contains(lower(c.reason()), "layout version") ? 3 : 4;
  }));
  ASSERT_GT(child.pid(), 0);
  EXPECT_EQ(child.wait_exit(kChildTimeout), 3) << "3: refused naming the layout version";
}

// The studio child claims, beats every 50 ms for 0.3 s and stops itself (SIGSTOP: alive but
// silent). Each beat stamp goes through a pipe; the parent polls owner_state every 10 ms and must
// see kHeartbeatLost 200..260 ms after the last beat (200 ms timeout + polling and scheduling).
TEST(BridgeTwoProcess, StoppedStudioLosesItsHeartbeatAfter200Ms) {
  const RegionGuard g;
  auto server = Server::create(g.name(), make_session(1));
  ASSERT_TRUE(server.is_ok()) << server.reason();
  const Pipe beats;
  ASSERT_TRUE(beats.ok());
  const std::string& name = g.name();
  Child child(fork_child([&name, &beats] {
    auto c = Client::open(name);
    if (!c.is_ok()) {
      return 4;
    }
    const std::int64_t t0 = monotonic_ns();
    if (!c.value().claim(t0).is_ok() || !beats.put(t0)) {
      return 3;
    }
    for (int i = 1; i <= 6; ++i) {
      sleep_until(t0 + i * kHeartbeatPeriodNs);
      const std::int64_t t = monotonic_ns();
      c.value().beat(t);
      if (!beats.put(t)) {
        return 5;
      }
    }
    ::raise(SIGSTOP);
    return 0;
  }));
  ASSERT_GT(child.pid(), 0);
  ASSERT_EQ(::fcntl(beats.read_end(), F_SETFL, O_NONBLOCK), 0);

  std::int64_t last_beat = 0;
  int stamps = 0;
  const auto drain = [&] {
    std::int64_t t = 0;
    while (beats.get(t)) {
      last_beat = t;
      ++stamps;
    }
  };
  int before_lost_bad = 0;  // states other than kNone / kAlive before the loss
  std::int64_t lost_at = 0;
  const std::int64_t give_up = monotonic_ns() + 10000 * kMs;
  while (monotonic_ns() < give_up) {
    drain();
    const std::int64_t now = monotonic_ns();
    const OwnerState st = server.value().owner_state(now);
    if (st == OwnerState::kHeartbeatLost) {
      lost_at = now;
      break;
    }
    before_lost_bad += (st == OwnerState::kNone || st == OwnerState::kAlive) ? 0 : 1;
    sleep_until(now + 10 * kMs);
  }
  drain();
  child.kill_and_reap();

  ASSERT_GT(lost_at, 0) << "kHeartbeatLost never seen";
  EXPECT_EQ(stamps, 7) << "claim + 6 beats";
  EXPECT_EQ(before_lost_bad, 0);
  const std::int64_t age = lost_at - last_beat;
  std::printf("heartbeat lost seen %.1f ms after the last beat\n", static_cast<double>(age) / 1e6);
  EXPECT_GE(age, 200 * kMs);
  EXPECT_LE(age, 260 * kMs);
}

// Studio A claims and waits; studio B is refused naming A's pid (exit 3); A releases; a new B
// claims (exit 0).
TEST(BridgeTwoProcess, SecondStudioIsRefusedUntilTheFirstReleases) {
  const RegionGuard g;
  auto server = Server::create(g.name(), make_session(1));
  ASSERT_TRUE(server.is_ok()) << server.reason();
  const Pipe from_a;
  const Pipe to_a;
  ASSERT_TRUE(from_a.ok() && to_a.ok());
  const std::string& name = g.name();
  Child a(fork_child([&] {
    auto c = Client::open(name);
    if (!c.is_ok() || !c.value().claim(monotonic_ns()).is_ok()) {
      return 3;
    }
    int msg = 1;  // claimed
    if (!from_a.put(msg) || !to_a.get_within(msg, kPipeTimeoutMs)) {
      return 4;
    }
    c.value().release();
    msg = 2;  // released
    if (!from_a.put(msg) || !to_a.get_within(msg, kPipeTimeoutMs)) {
      return 5;
    }
    return 0;
  }));
  ASSERT_GT(a.pid(), 0);
  int msg = 0;
  ASSERT_TRUE(from_a.get_within(msg, kPipeTimeoutMs)) << "A exit " << a.wait_exit(kChildTimeout);
  ASSERT_EQ(msg, 1);
  EXPECT_NE(server.value().owner_token(), 0U);

  const std::string held_by = "held by pid " + std::to_string(static_cast<long>(a.pid()));
  const auto claim_child = [&name, &held_by] {
    auto c = Client::open(name);
    if (!c.is_ok()) {
      return 5;
    }
    const common::Status s = c.value().claim(monotonic_ns());
    if (s.is_ok()) {
      return 0;
    }
    return contains(s.reason(), held_by) ? 3 : 4;
  };
  Child b(fork_child(claim_child));
  ASSERT_GT(b.pid(), 0);
  EXPECT_EQ(b.wait_exit(kChildTimeout), 3) << "3: refused with \"" << held_by << "\"";

  msg = 1;
  ASSERT_TRUE(to_a.put(msg));
  ASSERT_TRUE(from_a.get_within(msg, kPipeTimeoutMs)) << "A exit " << a.wait_exit(kChildTimeout);
  ASSERT_EQ(msg, 2);
  EXPECT_EQ(server.value().owner_token(), 0U) << "A released";

  Child b2(fork_child(claim_child));
  ASSERT_GT(b2.pid(), 0);
  EXPECT_EQ(b2.wait_exit(kChildTimeout), 0) << "B claims once A released";

  ASSERT_TRUE(to_a.put(msg));
  EXPECT_EQ(a.wait_exit(kChildTimeout), 0);
}

}  // namespace
}  // namespace cobot::bridge::test
