#pragma once

#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "cobot/bridge/endpoint.hpp"
#include "cobot/bridge/wire.hpp"
#include "cobot/common/result.hpp"
#include "cobot/common/types.hpp"
#include "cobot/pipeline/log_record.hpp"
#include "cobot/pipeline/manual.hpp"
#include "cobot/planning/waypoint_trajectory.hpp"
#include "cobot/studio/live_run.hpp"
#include "cobot/studio/studio_config.hpp"

#include "run_data.hpp"

// The studio side of the bridge (feature studio-control, cobot_studio design 6.6, ADR-022): one
// object on the GUI thread (not thread-safe), driven by tick() from a GUI timer. Never blocks and
// never reads files.
namespace cobot::studio {

struct LiveState {
  bool connected = false, server_alive = false, mine = false;
  std::int64_t owner_pid = 0;   // 0: free
  std::int64_t server_pid = 0;  // the controller (host) process
  pipeline::LoopMode mode = pipeline::LoopMode::kIdle;
  pipeline::CommandMode cmd = pipeline::CommandMode::kWaypoints;
  pipeline::ManualStop manual_stop = pipeline::ManualStop::kNone;
  pipeline::HoldReason hold = pipeline::HoldReason::kNone;
  std::uint64_t lost_samples = 0;
};

struct SegmentReady {
  fs::path csv;
  LiveRunInfo info;
};

class BridgeClient {
 public:
  explicit BridgeClient(std::string name);  // "cobot_controller" -> /cobot_cobot_controller
  ~BridgeClient();                          // releases authority
  BridgeClient(const BridgeClient&) = delete;
  BridgeClient& operator=(const BridgeClient&) = delete;
  BridgeClient(BridgeClient&&) = delete;
  BridgeClient& operator=(BridgeClient&&) = delete;

  // Every 50 ms from a GUI timer (on purpose: a frozen GUI stops beating and the controller stops
  // the robot): reconnect at most once per second while absent, beat while owner, drain new
  // samples into the live buffer (last 10 s of sample time), poll responses, run the live-run
  // tracker and send kExportSegment once a run's settle window has passed.
  void tick(std::int64_t now_ns);
  LiveState state() const;
  std::string reason() const;  // why not connected, last error
  const std::deque<bridge::StateSample<bench::kN>>& live() const { return live_; }
  std::optional<bridge::SessionInfo<bench::kN>> session() const { return session_; }
  common::Status claim();
  void release();
  // Commands: the command id, or "not connected" / "not the owner" / "command ring full".
  common::Result<std::uint64_t> move_joint(const common::VecN<bench::kN>& q_rad, double speed);
  common::Result<std::uint64_t> move_tcp(const common::Pose& T_b_tcp, double speed);
  common::Result<std::uint64_t> preview(const std::vector<planning::Waypoint>& tcp_waypoints);
  common::Result<std::uint64_t> run_waypoints(const std::vector<planning::Waypoint>& tcp_waypoints);
  common::Result<std::uint64_t> stop();
  common::Result<std::uint64_t> estop();
  common::Result<std::uint64_t> reset();
  common::Result<std::uint64_t> apply_config(const Materialized& files);
  // Jog slot writes (0-based joint); calling the same jog again is the refresh; release writes
  // direction 0.
  void jog_joint(int joint, int direction, double speed);
  void jog_tcp(pipeline::JogAxis axis, int direction, pipeline::JogFrame frame, double speed);
  void jog_release();
  std::optional<bridge::Response> response(std::uint64_t id) const;  // once it arrived
  std::vector<bridge::Response> take_jog_rejections();
  std::vector<SegmentReady> take_segments();  // finished live runs, to load on the worker

 private:
  using Command = bridge::Command<bench::kN>;
  struct Pending {  // a motion command sent, its run opened by an accepted response
    LiveRunInfo info;
    std::uint64_t preview_id = 0;  // run_waypoints: the preview sent with it (segment times)
  };

  void disconnect(const std::string& why);
  common::Result<std::uint64_t> send(Command& c, const char* what);
  common::Result<std::uint64_t> waypoints(const std::vector<planning::Waypoint>& list,
                                          bridge::CommandType type);
  void on_response(const bridge::Response& r);
  void track();
  void write_jog(int kind, int joint, int axis, int frame, int direction, double speed);

  std::string name_;
  std::optional<bridge::ClientEndpoint<bench::kN>> ep_;
  std::string reason_;
  std::int64_t last_attempt_ns_ = 0;
  bool attempted_ = false;
  std::int64_t last_alive_ns_ = 0;
  std::int64_t last_beat_ns_ = 0;
  std::uint64_t lost_before_ = 0;  // lost samples of earlier connections
  std::optional<bridge::SessionInfo<bench::kN>> session_;
  std::deque<bridge::StateSample<bench::kN>> live_;
  std::vector<bridge::StateSample<bench::kN>> buf_;
  std::map<std::uint64_t, bridge::Response> responses_;
  std::vector<bridge::Response> jog_rejections_;
  std::map<std::uint64_t, Pending> pending_;
  std::optional<LiveRunInfo> run_;                // the open run (tracking)
  std::map<std::uint64_t, LiveRunInfo> exports_;  // kExportSegment id -> the run
  std::vector<LiveRunInfo> to_export_;
  std::vector<SegmentReady> segments_;
  bridge::JogRequest jog_{};
  std::optional<std::int64_t> jog_armed_after_ns_;  // a jog press after this sample time
};

}  // namespace cobot::studio
