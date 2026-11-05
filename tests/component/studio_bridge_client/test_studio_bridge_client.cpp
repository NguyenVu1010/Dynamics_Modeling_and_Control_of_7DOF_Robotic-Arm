// studio::BridgeClient (feature studio-control, cobot/studio/bridge_client.hpp, cobot_studio design
// 6.6, ADR-022) against bench::serve on a thread of this test: config/default.yaml (absolute paths,
// 5 s Recorder buffers) + the ideal profile, region /cobot_cobot_test_client_<pid>. Real time; the
// test thread calls tick(monotonic_ns) every 50 ms as the studio's GUI timer does. Pinned:
//   - before serve starts: not connected, reason() names "no bridge"; once it runs: connected
//     within 2 s (a reconnect at most once per second), server alive, session paths = the serve
//     inputs (made absolute by serve);
//   - authority: a second client's claim fails with "held by pid"; after release the second one
//     claims; commands of a non-owner fail with "not the owner", of a client without a region
//     with "not connected"; destroying the owner releases authority;
//   - move_joint (q1 + 10 deg, speed 0.1): accepted response, one SegmentReady within 6 s whose
//     make_live_entry rows are move_joint from t_start on, end in standby, not held;
//   - jog joint 1 for 300 ms (refresh every 25 ms) then release: q1 rises by 0.063 +- 0.03 rad;
//     the jog is a live run too (cmd jog_joint, command id 0);
//   - heartbeat: 400 ms without tick during a move -> the client is no longer the owner, the
//     region is free and the samples show standby within 0.75 s of sample time after the last
//     beat;
//   - live(): at most the last 10 s of sample time; lost_samples is reported.
// Joints of jog_joint are 0-based, like the wire's JogRequest::joint and ControlLoop::jog_joint
// (the Control tab's jogPlus_1 is joint 0).
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/bridge/endpoint.hpp"
#include "cobot/bridge/wire.hpp"
#include "cobot/common/result.hpp"
#include "cobot/common/types.hpp"
#include "cobot/pipeline/log_record.hpp"
#include "cobot/studio/bridge_client.hpp"
#include "cobot/studio/live_run.hpp"
#include "cobot/studio/run_store.hpp"

#include "run_data.hpp"
#include "serve.hpp"

namespace cobot::studio {
namespace {

namespace fs = std::filesystem;
using Sample = bridge::StateSample<bench::kN>;

const fs::path kRepo = COBOT_REPO_DIR;
constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr std::int64_t kNsPerS = 1'000'000'000;
constexpr int kTickMs = 50;

std::string name_of_test_region() {
  return "cobot_test_client_" + std::to_string(::getpid());
}

std::string read_file(const fs::path& path) {
  const std::ifstream in(path, std::ios::binary);
  std::ostringstream os;
  os << in.rdbuf();
  return os.str();
}

fs::path test_dir(const std::string& name) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "studio_bridge_client" / name;
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir;
}

// config/default.yaml with absolute paths and a 5 s Recorder per buffer (the bridge exports every
// 200 ms; two 60 s buffers would touch ~120 MB).
fs::path serve_config(const fs::path& dir) {
  std::string text = read_file(kRepo / "config/default.yaml");
  const auto replace = [&text](const std::string& from, const std::string& to) {
    const std::size_t at = text.find(from);
    EXPECT_NE(at, std::string::npos) << from;
    if (at != std::string::npos) {
      text.replace(at, from.size(), to);
    }
  };
  replace("urdf_path: ../src/sim/urdf/cobot30_5.urdf",
          "urdf_path: " + (kRepo / "src/sim/urdf/cobot30_5.urdf").string());
  replace("output_dir: ../results", "output_dir: " + (dir / "results").string());
  replace("capacity_s: 60.0", "capacity_s: 5.0");
  const fs::path yaml = dir / "serve_config.yaml";
  std::ofstream(yaml, std::ios::binary) << text;
  return yaml;
}

std::uint8_t raw(pipeline::LoopMode m) {
  return static_cast<std::uint8_t>(m);
}

std::uint8_t raw(pipeline::CommandMode m) {
  return static_cast<std::uint8_t>(m);
}

// bench::serve on its own thread until stop().
class Serve {
 public:
  explicit Serve(const fs::path& dir) {
    options_.config_yaml = serve_config(dir);
    options_.imperfections_yaml = kRepo / "config/imperfections/ideal.yaml";
    options_.name = name_of_test_region();
    options_.out_dir = dir / "serve";
  }
  Serve(const Serve&) = delete;
  Serve& operator=(const Serve&) = delete;
  ~Serve() { stop(); }

  void start() {
    thread_ = std::thread([this] {
      const common::Status st = bench::serve(options_, stop_, [this](const std::string& region) {
        const std::lock_guard<std::mutex> lock(mutex_);
        ready_ = region;
        cv_.notify_all();
      });
      const std::lock_guard<std::mutex> lock(mutex_);
      result_ = st;
      cv_.notify_all();
    });
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait_for(lock, std::chrono::seconds(60), [this] { return ready_ || result_; });
    ASSERT_TRUE(ready_) << "serve never became ready"
                        << (result_ ? ": " + result_->reason() : std::string());
  }

  void stop() {
    stop_ = true;
    if (thread_.joinable()) {
      thread_.join();
      ASSERT_TRUE(result_);
      EXPECT_TRUE(result_->is_ok()) << result_->reason();
    }
  }

  const bench::ServeOptions& options() const { return options_; }

 private:
  bench::ServeOptions options_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<std::string> ready_;
  std::optional<common::Status> result_;
};

void tick(const std::vector<BridgeClient*>& clients) {
  for (BridgeClient* c : clients) {
    c->tick(bridge::monotonic_ns());
  }
}

// Ticks every 50 ms (the GUI timer) until `done` or timeout_ms; returns done().
bool drive(const std::vector<BridgeClient*>& clients,
           int timeout_ms,
           const std::function<bool()>& done) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  for (;;) {
    tick(clients);
    if (done()) {
      return true;
    }
    if (std::chrono::steady_clock::now() > end) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(kTickMs));
  }
}

bool in_standby(const BridgeClient& c) {
  return !c.live().empty() && c.live().back().mode == raw(pipeline::LoopMode::kStandby);
}

class StudioBridgeClient : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = test_dir(::testing::UnitTest::GetInstance()->current_test_info()->name());
    serve_ = std::make_unique<Serve>(dir_);
  }

  void TearDown() override { serve_.reset(); }

  // Starts serve and connects `c` (2 s).
  void start_and_connect(BridgeClient& c) {
    ASSERT_NO_FATAL_FAILURE(serve_->start());
    ASSERT_TRUE(drive({&c}, 2000, [&] { return c.state().connected; })) << c.reason();
  }

  // Claims and waits for a standby sample.
  void claim_in_standby(BridgeClient& c) {
    const common::Status claimed = c.claim();
    ASSERT_TRUE(claimed.is_ok()) << claimed.reason();
    ASSERT_TRUE(drive({&c}, 5000, [&] { return c.state().mine && in_standby(c); }))
        << "no standby sample";
  }

  // Ticks until the response to `id` arrives (5 s).
  std::optional<bridge::Response> wait_response(BridgeClient& c, std::uint64_t id) {
    std::optional<bridge::Response> r;
    drive({&c}, 5000, [&] {
      r = c.response(id);
      return r.has_value();
    });
    return r;
  }

  fs::path dir_;
  std::unique_ptr<Serve> serve_;
};

TEST_F(StudioBridgeClient, ConnectsOnceServeStartsAndReadsTheSession) {
  BridgeClient c(name_of_test_region());
  c.tick(bridge::monotonic_ns());
  EXPECT_FALSE(c.state().connected);
  EXPECT_FALSE(c.state().mine);
  EXPECT_NE(c.reason().find("no bridge"), std::string::npos) << c.reason();
  EXPECT_FALSE(c.session().has_value());
  EXPECT_TRUE(c.live().empty());

  ASSERT_NO_FATAL_FAILURE(serve_->start());
  ASSERT_TRUE(drive({&c}, 2000, [&] { return c.state().connected; })) << c.reason();
  EXPECT_TRUE(c.state().server_alive);
  EXPECT_FALSE(c.state().mine);
  EXPECT_EQ(c.state().owner_pid, 0) << "nobody holds authority yet";

  const std::optional<bridge::SessionInfo<bench::kN>> session = c.session();
  ASSERT_TRUE(session.has_value());
  EXPECT_EQ(std::string(session->config_path),
            fs::absolute(serve_->options().config_yaml).string());
  EXPECT_EQ(std::string(session->imperfections_path),
            fs::absolute(serve_->options().imperfections_yaml).string());
  EXPECT_EQ(session->control_rate_hz, 1000.0);

  // Samples flow into the live buffer; the loop starts in standby (external mode).
  ASSERT_TRUE(drive({&c}, 2000, [&] { return in_standby(c); }));
  EXPECT_EQ(c.state().mode, pipeline::LoopMode::kStandby);
}

TEST_F(StudioBridgeClient, SecondClientIsRefusedUntilTheFirstReleases) {
  BridgeClient a(name_of_test_region());
  BridgeClient b(name_of_test_region());
  ASSERT_NO_FATAL_FAILURE(start_and_connect(a));
  ASSERT_TRUE(drive({&a, &b}, 2000, [&] { return b.state().connected; })) << b.reason();

  const common::Status claimed = a.claim();
  ASSERT_TRUE(claimed.is_ok()) << claimed.reason();
  tick({&a, &b});
  EXPECT_TRUE(a.state().mine);
  EXPECT_EQ(a.state().owner_pid, static_cast<std::int64_t>(::getpid()));

  const common::Status refused = b.claim();
  ASSERT_FALSE(refused.is_ok());
  EXPECT_NE(refused.reason().find("held by pid"), std::string::npos) << refused.reason();
  EXPECT_FALSE(b.state().mine);
  EXPECT_EQ(b.state().owner_pid, static_cast<std::int64_t>(::getpid()));

  // A non-owner's command is refused before it reaches the ring.
  const common::Result<std::uint64_t> stray = b.stop();
  ASSERT_FALSE(stray.is_ok());
  EXPECT_NE(stray.reason().find("not the owner"), std::string::npos) << stray.reason();

  a.release();
  tick({&a, &b});
  EXPECT_FALSE(a.state().mine);
  const common::Status second = b.claim();
  ASSERT_TRUE(second.is_ok()) << second.reason();
  tick({&a, &b});
  EXPECT_TRUE(b.state().mine);

  // Destroying the owner releases authority.
  {
    BridgeClient c(name_of_test_region());
    ASSERT_TRUE(drive({&c}, 2000, [&] { return c.state().connected; })) << c.reason();
    b.release();
    const common::Status third = c.claim();
    ASSERT_TRUE(third.is_ok()) << third.reason();
  }
  const common::Status after = a.claim();
  EXPECT_TRUE(after.is_ok()) << after.reason();
  a.release();
}

TEST_F(StudioBridgeClient, CommandsWithoutARegionAreNotConnected) {
  BridgeClient c("cobot_test_client_none_" + std::to_string(::getpid()));
  c.tick(bridge::monotonic_ns());
  ASSERT_FALSE(c.state().connected);
  const common::Result<std::uint64_t> stop = c.stop();
  ASSERT_FALSE(stop.is_ok());
  EXPECT_NE(stop.reason().find("not connected"), std::string::npos) << stop.reason();
  const common::Status claim = c.claim();
  EXPECT_FALSE(claim.is_ok());
}

// q1 + 10 deg at speed 0.1: d = 0.1745 rad, s_v_max = 0.1 * 0.9 * 2.1 / d = 1.083 1/s,
// s_a_max = 0.1 * 15 / d = 8.59 1/s^2; s_v_max^2 / s_a_max = 0.137 < 1, trapezoid of
// 1 / 1.083 + 1.083 / 8.59 = 1.05 s; standby + 1 s settle window + the 200 ms export period
// is ~2.3 s after the response, inside 6 s.
TEST_F(StudioBridgeClient, MoveJointBecomesOneLiveRun) {
  BridgeClient c(name_of_test_region());
  ASSERT_NO_FATAL_FAILURE(start_and_connect(c));
  ASSERT_NO_FATAL_FAILURE(claim_in_standby(c));
  const std::optional<bridge::SessionInfo<bench::kN>> session = c.session();
  ASSERT_TRUE(session.has_value());

  const Sample& now = c.live().back();
  common::VecN<bench::kN> goal;
  for (int j = 0; j < bench::kN; ++j) {
    goal(j) = now.q_ref[static_cast<std::size_t>(j)];
  }
  goal(0) += 10.0 * kDeg;
  const common::Result<std::uint64_t> id = c.move_joint(goal, 0.1);
  ASSERT_TRUE(id.is_ok()) << id.reason();
  const std::optional<bridge::Response> r = wait_response(c, id.value());
  ASSERT_TRUE(r.has_value()) << "no response";
  EXPECT_EQ(r->type, bridge::CommandType::kMoveJoint);
  ASSERT_EQ(r->accepted, 1U) << r->text;

  std::vector<SegmentReady> segments;
  ASSERT_TRUE(drive({&c}, 6000, [&] {
    for (SegmentReady& s : c.take_segments()) {
      segments.push_back(std::move(s));
    }
    return !segments.empty();
  })) << "no segment within 6 s";
  drive({&c}, 500, [&] {
    for (SegmentReady& s : c.take_segments()) {
      segments.push_back(std::move(s));
    }
    return false;
  });
  ASSERT_EQ(segments.size(), 1U);
  const SegmentReady& seg = segments.front();
  EXPECT_EQ(seg.info.cmd, pipeline::CommandMode::kMoveJoint);
  EXPECT_EQ(seg.info.command_id, id.value());
  EXPECT_EQ(seg.info.t_start_ns, r->t_start_ns);
  EXPECT_GT(seg.info.t_end_ns, seg.info.t_start_ns);
  EXPECT_EQ(seg.info.settle_window_s, 1.0);
  EXPECT_FALSE(seg.info.truncated);
  ASSERT_TRUE(fs::exists(seg.csv)) << seg.csv;

  const common::Result<RunEntry> entry = make_live_entry(seg.csv, seg.info, *session, 1);
  ASSERT_TRUE(entry.is_ok()) << entry.reason();
  ASSERT_TRUE(entry.value().data);
  const auto& rows = entry.value().data->records;
  ASSERT_FALSE(rows.empty());
  for (const auto& row : rows) {
    if (row.t.ns >= seg.info.t_start_ns) {
      ASSERT_EQ(row.cmd_mode, pipeline::CommandMode::kMoveJoint) << "t_ns " << row.t.ns;
    }
  }
  EXPECT_EQ(rows.back().mode, pipeline::LoopMode::kStandby);
  EXPECT_GE(rows.back().t.ns, seg.info.t_end_ns + kNsPerS - 2'000'000)
      << "the segment runs to t_end + settle window";
  EXPECT_FALSE(entry.value().summary.held);
  EXPECT_EQ(entry.value().kind, RunKind::kGazebo);
  EXPECT_EQ(entry.value().label.rfind("live move_joint #1", 0), 0U) << entry.value().label;
  EXPECT_NEAR(rows.back().q(0), goal(0), 0.2 * kDeg);
}

// Joint jog at speed 0.1 (default.yaml, URDF joint 1: qd_max 2.1 rad/s, a_max 15 rad/s^2):
// v = 0.1 * 0.9 * 2.1 = 0.189 rad/s, a = 0.1 * 15 = 1.5 rad/s^2; ramp 0.189 / 1.5 = 0.126 s
// covering 0.5 * 1.5 * 0.126^2 = 0.0119 rad, cruise (0.300 - 0.126) * 0.189 = 0.0329 rad, ramp
// down 0.0119 rad: 0.0567 rad. The brief's 0.063 (0.0147 + 0.0336 + 0.0147, before the 0.9
// joint_speed_fraction headroom) +- 0.03 rad of real-time tolerance holds it.
TEST_F(StudioBridgeClient, JogJointMovesJoint1AndIsALiveRun) {
  BridgeClient c(name_of_test_region());
  ASSERT_NO_FATAL_FAILURE(start_and_connect(c));
  ASSERT_NO_FATAL_FAILURE(claim_in_standby(c));
  const double q_before = c.live().back().q[0];

  for (int i = 0; i < 12; ++i) {  // 12 x 25 ms = 300 ms, a refresh every 25 ms
    c.jog_joint(0, +1, 0.1);
    if (i % 2 == 0) {
      tick({&c});
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }
  c.jog_release();
  bool moved = false;
  ASSERT_TRUE(drive({&c},
                    5000,
                    [&] {
                      for (const Sample& s : c.live()) {
                        moved = moved || (s.mode == raw(pipeline::LoopMode::kTracking) &&
                                          s.cmd_mode == raw(pipeline::CommandMode::kJogJoint));
                      }
                      return moved && in_standby(c);
                    }))
      << "jog never ran or never came back to standby (moved: " << moved << ")";
  const double rise = c.live().back().q[0] - q_before;
  EXPECT_NEAR(rise, 0.063, 0.03);
  EXPECT_TRUE(c.take_jog_rejections().empty());

  std::vector<SegmentReady> segments;
  ASSERT_TRUE(drive({&c}, 4000, [&] {
    for (SegmentReady& s : c.take_segments()) {
      segments.push_back(std::move(s));
    }
    return !segments.empty();
  })) << "the jog is a live run too";
  EXPECT_EQ(segments.front().info.cmd, pipeline::CommandMode::kJogJoint);
  EXPECT_EQ(segments.front().info.command_id, 0U);
}

// No tick for 400 ms during a move_joint (1.05 s, see above): the server sees the owner lost
// 200 ms after the last beat, stops the move with deceleration, returns to standby and clears
// the owner once the motion is over.
TEST_F(StudioBridgeClient, FrozenTickStopsTheMoveAndFreesTheRegion) {
  BridgeClient c(name_of_test_region());
  ASSERT_NO_FATAL_FAILURE(start_and_connect(c));
  ASSERT_NO_FATAL_FAILURE(claim_in_standby(c));
  common::VecN<bench::kN> goal;
  for (int j = 0; j < bench::kN; ++j) {
    goal(j) = c.live().back().q_ref[static_cast<std::size_t>(j)];
  }
  goal(0) += 10.0 * kDeg;
  const common::Result<std::uint64_t> id = c.move_joint(goal, 0.1);
  ASSERT_TRUE(id.is_ok()) << id.reason();
  const std::optional<bridge::Response> r = wait_response(c, id.value());
  ASSERT_TRUE(r.has_value());
  ASSERT_EQ(r->accepted, 1U) << r->text;
  ASSERT_TRUE(drive({&c}, 1000, [&] {
    return !c.live().empty() && c.live().back().mode == raw(pipeline::LoopMode::kTracking);
  }));
  c.tick(bridge::monotonic_ns());  // the last beat
  ASSERT_FALSE(c.live().empty());
  const std::int64_t t_last_beat = c.live().back().t_ns;

  std::this_thread::sleep_for(std::chrono::milliseconds(400));  // frozen GUI

  ASSERT_TRUE(drive(
      {&c}, 3000, [&] { return !c.state().mine && c.state().owner_pid == 0 && in_standby(c); }))
      << "mine " << c.state().mine << ", owner pid " << c.state().owner_pid;
  std::optional<std::int64_t> t_standby;
  for (const Sample& s : c.live()) {
    if (s.t_ns > t_last_beat && s.mode == raw(pipeline::LoopMode::kStandby)) {
      t_standby = s.t_ns;
      break;
    }
  }
  ASSERT_TRUE(t_standby.has_value());
  // 200 ms heartbeat timeout + the stop ramp + one tick of slack.
  EXPECT_LE(*t_standby - t_last_beat, 750'000'000) << "ns of sample time after the last beat";
  // Stopped about a third of the way (lost ~0.3 s into the 1.05 s move), well short of the goal.
  EXPECT_GT(std::abs(c.live().back().q[0] - goal(0)), 1.0 * kDeg) << "the move was not stopped";
}

TEST_F(StudioBridgeClient, LiveBufferKeepsTheLastTenSeconds) {
  BridgeClient c(name_of_test_region());
  ASSERT_NO_FATAL_FAILURE(start_and_connect(c));
  drive({&c}, 11500, [] { return false; });
  ASSERT_FALSE(c.live().empty());
  const double span_s = static_cast<double>(c.live().back().t_ns - c.live().front().t_ns) /
                        static_cast<double>(kNsPerS);
  EXPECT_LE(span_s, 10.0);
  EXPECT_GE(span_s, 9.5) << "about 10 s of samples are kept";
  // Oldest first; every gap in the step numbers is a lost sample, so lost_samples (counted since
  // the client connected; 0 on a quiet machine) covers the gaps still in the buffer.
  std::uint64_t gaps = 0;
  for (std::size_t k = 1; k < c.live().size(); ++k) {
    ASSERT_GT(c.live()[k].step, c.live()[k - 1].step) << "oldest first";
    gaps += c.live()[k].step - c.live()[k - 1].step - 1;
  }
  EXPECT_GE(c.state().lost_samples, gaps);
}

}  // namespace
}  // namespace cobot::studio
