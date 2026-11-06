#include "cobot/studio/bridge_client.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstring>
#include <memory>
#include <sys/types.h>
#include <utility>

namespace cobot::studio {
namespace {

using pipeline::CommandMode;
using pipeline::LoopMode;
using Sample = bridge::StateSample<bench::kN>;

constexpr std::int64_t kNsPerS = 1'000'000'000;
constexpr std::int64_t kReconnectNs = kNsPerS;      // at most one open() per second
constexpr std::int64_t kServerGoneNs = kNsPerS;     // server heartbeat missing this long: gone
constexpr std::int64_t kLiveSpanNs = 10 * kNsPerS;  // live buffer, sample time
constexpr std::size_t kResponsesKept = 64;
constexpr int kSessionReads = 4;  // seqlock read attempts

bool is(const Sample& s, LoopMode m) {
  return s.mode == static_cast<std::uint8_t>(m);
}

std::string text_of(const char* field, std::size_t size) {
  return {field, strnlen(field, size)};
}

// False only when the process is known to be gone: a server killed without unlinking its region
// leaves the region openable, with its pid.
bool pid_alive(std::int64_t pid) {
  return pid <= 0 || ::kill(static_cast<pid_t>(pid), 0) == 0 || errno != ESRCH;
}

}  // namespace

BridgeClient::BridgeClient(std::string name) : name_(std::move(name)) {}

BridgeClient::~BridgeClient() {
  release();
}

void BridgeClient::tick(std::int64_t now_ns) {
  if (!ep_) {
    if (attempted_ && now_ns - last_attempt_ns_ < kReconnectNs) {
      return;
    }
    attempted_ = true;
    last_attempt_ns_ = now_ns;
    common::Result<bridge::ClientEndpoint<bench::kN>> opened =
        bridge::ClientEndpoint<bench::kN>::open(name_);
    if (!opened) {
      reason_ = opened.reason();
      return;
    }
    ep_.emplace(std::move(opened.value()));
    last_alive_ns_ = now_ns;
    reason_.clear();
  }
  if (ep_->server_alive(now_ns)) {
    last_alive_ns_ = now_ns;
  } else if (now_ns - last_alive_ns_ > kServerGoneNs && now_ns - last_attempt_ns_ >= kReconnectNs) {
    // A busy host (a long export) also misses beats: only a region that is gone, replaced by a
    // new server, or left behind by a dead one ends the connection; the probe runs at most once
    // per second.
    last_attempt_ns_ = now_ns;
    const common::Result<bridge::ClientEndpoint<bench::kN>> probe =
        bridge::ClientEndpoint<bench::kN>::open(name_);
    if (!probe || probe.value().server_pid() != ep_->server_pid() ||
        !pid_alive(ep_->server_pid())) {
      disconnect("bridge /cobot_" + name_ + ": the controller stopped");
      return;
    }
  }
  bridge::SessionInfo<bench::kN> s{};
  for (int k = 0; k < kSessionReads; ++k) {
    if (ep_->session(s)) {
      session_ = s;
      break;
    }
  }
  if (ep_->is_owner()) {
    // A tick gap longer than the heartbeat timeout (frozen GUI): the controller has given up on
    // this client, so authority is handed back rather than renewed.
    if (last_beat_ns_ != 0 && now_ns - last_beat_ns_ > bridge::kHeartbeatTimeoutNs) {
      ep_->release();
      jog_.direction = 0;  // the controller stopped the jog: a held button no longer jogs
      reason_ = "authority lost: no heartbeat for " +
                std::to_string((now_ns - last_beat_ns_) / 1'000'000) + " ms";
    } else {
      ep_->beat(now_ns);
      last_beat_ns_ = now_ns;
    }
  }
  buf_.clear();
  ep_->read_states(buf_);
  live_.insert(live_.end(), buf_.begin(), buf_.end());
  while (!live_.empty() && live_.back().t_ns - live_.front().t_ns > kLiveSpanNs) {
    live_.pop_front();
  }
  auto r = std::make_unique<bridge::Response>();
  while (ep_->poll_response(*r)) {
    on_response(*r);
  }
  track();
}

void BridgeClient::disconnect(const std::string& why) {
  release();
  lost_before_ += ep_->lost();
  ep_.reset();
  reason_ = why;
  session_.reset();
  live_.clear();
  // Command ids restart at 1 on every endpoint: answers of this connection must not answer the
  // next one's commands.
  responses_.clear();
  jog_rejections_.clear();
  pending_.clear();
  run_.reset();
  to_export_.clear();
  exports_.clear();
  jog_armed_after_ns_.reset();
}

LiveState BridgeClient::state() const {
  LiveState st;
  if (!ep_) {
    st.lost_samples = lost_before_;
    return st;
  }
  st.connected = true;
  st.server_alive = ep_->server_alive(bridge::monotonic_ns());
  st.mine = ep_->is_owner();
  st.owner_pid = ep_->owner_pid();
  st.server_pid = ep_->server_pid();
  st.lost_samples = lost_before_ + ep_->lost();
  if (!live_.empty()) {
    const Sample& s = live_.back();
    st.mode = static_cast<LoopMode>(s.mode);
    st.cmd = static_cast<CommandMode>(s.cmd_mode);
    st.manual_stop = static_cast<pipeline::ManualStop>(s.manual_stop);
    st.hold = static_cast<pipeline::HoldReason>(s.hold_reason);
  }
  return st;
}

std::string BridgeClient::reason() const {
  return reason_;
}

common::Status BridgeClient::claim() {
  if (!ep_) {
    return common::Status::error("bridge /cobot_" + name_ + ": not connected");
  }
  const std::int64_t now = bridge::monotonic_ns();
  common::Status s = ep_->claim(now);
  if (s) {
    last_beat_ns_ = now;
  }
  return s;
}

void BridgeClient::release() {
  jog_.direction = 0;  // without authority no button jogs
  if (ep_ && ep_->is_owner()) {
    track();  // runs whose settle window has passed are exported while still the owner
    ep_->release();
  }
  last_beat_ns_ = 0;
}

common::Result<std::uint64_t> BridgeClient::send(Command& c, const char* what) {
  using R = common::Result<std::uint64_t>;
  if (!ep_) {
    return R::error(std::string(what) + ": not connected");
  }
  if (!ep_->is_owner()) {
    return R::error(std::string(what) + ": not the owner");
  }
  return ep_->send(c);
}

namespace {

// The session's files, until the studio applies its own configuration.
StudioConfig session_config(const std::optional<bridge::SessionInfo<bench::kN>>& s) {
  StudioConfig c;
  if (s) {
    c.base = BasePair{"session",
                      text_of(s->config_path, bridge::kPathChars),
                      text_of(s->imperfections_path, bridge::kPathChars)};
  }
  return c;
}

// The tip position of a TCP pose (T_b_tip = T_b_tcp * T_tip_tcp^-1).
common::Vec3 tip_of(const common::Pose& T_b_tcp, const bridge::SessionInfo<bench::kN>& s) {
  const common::Pose T_b_tip =
      common::compose(T_b_tcp, common::inverse(bridge::from_wire(s.T_tip_tcp)));
  return T_b_tip.p;
}

}  // namespace

common::Result<std::uint64_t> BridgeClient::move_joint(const common::VecN<bench::kN>& q_rad,
                                                       double speed) {
  auto c = std::make_unique<Command>();
  c->type = bridge::CommandType::kMoveJoint;
  c->speed = speed;
  for (int j = 0; j < bench::kN; ++j) {
    c->q_goal[j] = q_rad(j);
  }
  common::Result<std::uint64_t> id = send(*c, "move_joint");
  if (id) {
    Pending& p = pending_[id.value()];
    p.info.cmd = CommandMode::kMoveJoint;
    p.info.command_id = id.value();
    p.info.config = session_config(session_);
  }
  return id;
}

common::Result<std::uint64_t> BridgeClient::move_tcp(const common::Pose& T_b_tcp, double speed) {
  auto c = std::make_unique<Command>();
  c->type = bridge::CommandType::kMoveTcp;
  c->speed = speed;
  c->T_b_tcp = bridge::to_wire(T_b_tcp);
  common::Result<std::uint64_t> id = send(*c, "move_tcp");
  if (id) {
    Pending& p = pending_[id.value()];
    p.info.cmd = CommandMode::kMoveTcp;
    p.info.command_id = id.value();
    p.info.config = session_config(session_);
    if (session_) {
      p.info.waypoint_stop = {true};
      p.info.waypoint_positions_m = {tip_of(T_b_tcp, *session_)};
    }
  }
  return id;
}

common::Result<std::uint64_t> BridgeClient::waypoints(const std::vector<planning::Waypoint>& list,
                                                      bridge::CommandType type) {
  using R = common::Result<std::uint64_t>;
  if (list.empty() || list.size() > bridge::kWireWaypoints) {
    return R::error("waypoints: " + std::to_string(list.size()) + " points, want 1 .. " +
                    std::to_string(bridge::kWireWaypoints));
  }
  auto c = std::make_unique<Command>();
  c->type = type;
  c->waypoint_count = static_cast<std::uint32_t>(list.size());
  for (std::size_t i = 0; i < list.size(); ++i) {
    c->waypoints[i].T_b_tcp = bridge::to_wire(list[i].T_b_goal);
    c->waypoints[i].duration_s = list[i].duration_s;
    c->waypoints[i].stop = list[i].stop ? 1U : 0U;
  }
  return send(*c, type == bridge::CommandType::kRunWaypoints ? "run_waypoints" : "preview");
}

common::Result<std::uint64_t> BridgeClient::preview(
    const std::vector<planning::Waypoint>& tcp_waypoints) {
  return waypoints(tcp_waypoints, bridge::CommandType::kPreviewWaypoints);
}

// A preview of the same list goes first: its response carries the segment times the run's
// accepted response does not.
common::Result<std::uint64_t> BridgeClient::run_waypoints(
    const std::vector<planning::Waypoint>& tcp_waypoints) {
  common::Result<std::uint64_t> planned = preview(tcp_waypoints);
  if (!planned) {
    return planned;
  }
  common::Result<std::uint64_t> id = waypoints(tcp_waypoints, bridge::CommandType::kRunWaypoints);
  if (id) {
    Pending& p = pending_[id.value()];
    p.preview_id = planned.value();
    p.info.cmd = CommandMode::kWaypoints;
    p.info.command_id = id.value();
    p.info.config = session_config(session_);
    for (std::size_t i = 0; session_ && i < tcp_waypoints.size(); ++i) {
      p.info.waypoint_stop.push_back(tcp_waypoints[i].stop || i + 1 == tcp_waypoints.size());
      p.info.waypoint_positions_m.push_back(tip_of(tcp_waypoints[i].T_b_goal, *session_));
    }
  }
  return id;
}

common::Result<std::uint64_t> BridgeClient::stop() {
  Command c{};
  c.type = bridge::CommandType::kStop;
  return send(c, "stop");
}

common::Result<std::uint64_t> BridgeClient::estop() {
  Command c{};
  c.type = bridge::CommandType::kEstop;
  return send(c, "estop");
}

common::Result<std::uint64_t> BridgeClient::reset() {
  Command c{};
  c.type = bridge::CommandType::kReset;
  return send(c, "reset");
}

common::Result<std::uint64_t> BridgeClient::apply_config(const Materialized& files) {
  using R = common::Result<std::uint64_t>;
  const std::string a = files.config_yaml.string();
  const std::string b = files.imperfections_yaml.string();
  if (a.size() >= bridge::kPathChars || b.size() >= bridge::kPathChars) {
    return R::error("apply config: path longer than " + std::to_string(bridge::kPathChars - 1));
  }
  auto c = std::make_unique<Command>();
  c->type = bridge::CommandType::kApplyConfig;
  bridge::copy_text(c->path_a, bridge::kPathChars, a);
  bridge::copy_text(c->path_b, bridge::kPathChars, b);
  return send(*c, "apply config");
}

void BridgeClient::write_jog(
    int kind, int joint, int axis, int frame, int direction, double speed) {
  if (!ep_ || !ep_->is_owner()) {
    if (direction == 0) {
      jog_.direction = 0;  // a release is recorded even without authority
    }
    return;
  }
  bridge::JogRequest j{};
  j.kind = kind;
  j.joint = joint;
  j.axis = axis;
  j.frame = frame;
  j.direction = direction;
  j.speed = speed;
  ep_->write_jog(j);
  jog_ = j;
  if (direction != 0 && !run_ && !jog_armed_after_ns_) {
    jog_armed_after_ns_ = live_.empty() ? 0 : live_.back().t_ns;
  }
}

void BridgeClient::jog_joint(int joint, int direction, double speed) {
  write_jog(1, joint, 0, 0, direction, speed);
}

void BridgeClient::jog_tcp(pipeline::JogAxis axis,
                           int direction,
                           pipeline::JogFrame frame,
                           double speed) {
  write_jog(2, -1, static_cast<int>(axis), static_cast<int>(frame), direction, speed);
}

void BridgeClient::jog_release() {
  if (jog_.kind != 0) {
    write_jog(jog_.kind, jog_.joint, jog_.axis, jog_.frame, 0, jog_.speed);
  }
}

std::optional<bridge::Response> BridgeClient::response(std::uint64_t id) const {
  const auto it = responses_.find(id);
  return it == responses_.end() ? std::nullopt : std::optional<bridge::Response>(it->second);
}

std::vector<bridge::Response> BridgeClient::take_jog_rejections() {
  return std::exchange(jog_rejections_, {});
}

std::vector<SegmentReady> BridgeClient::take_segments() {
  return std::exchange(segments_, {});
}

void BridgeClient::on_response(const bridge::Response& r) {
  if (r.type == bridge::CommandType::kJog) {
    jog_rejections_.push_back(r);
    return;
  }
  responses_[r.command_id] = r;
  if (responses_.size() > kResponsesKept) {
    responses_.erase(responses_.begin());
  }
  if (r.accepted == 0) {
    reason_ = text_of(r.text, bridge::kTextChars);
  }
  if (const auto e = exports_.find(r.command_id); e != exports_.end()) {
    if (r.accepted != 0) {
      e->second.truncated = r.truncated != 0;
      segments_.push_back(SegmentReady{text_of(r.out_path, bridge::kPathChars), e->second});
    }
    exports_.erase(e);
    return;
  }
  const auto p = pending_.find(r.command_id);
  if (p == pending_.end()) {
    return;
  }
  if (r.accepted != 0) {
    LiveRunInfo info = p->second.info;
    info.t_start_ns = r.t_start_ns;
    if (info.cmd == CommandMode::kMoveTcp && !info.waypoint_stop.empty()) {
      info.segment_durations_s = {r.duration_s};
    }
    const std::optional<bridge::Response> plan = response(p->second.preview_id);
    if (info.cmd == CommandMode::kWaypoints && plan && plan->accepted != 0 &&
        plan->segment_count == info.waypoint_stop.size()) {
      info.segment_durations_s.assign(plan->segment_durations_s,
                                      plan->segment_durations_s + plan->segment_count);
    } else if (info.cmd == CommandMode::kWaypoints) {
      info.waypoint_stop.clear();  // node times unknown: no waypoint fields
      info.waypoint_positions_m.clear();
    }
    if (run_ && (run_->cmd == CommandMode::kJogJoint || run_->cmd == CommandMode::kJogTcp)) {
      // A jog run still open (its standby not read yet) is closed, not dropped, by the new run.
      close_run(false);
      if (run_) {
        run_->t_end_ns = std::max(run_->t_start_ns, r.t_start_ns);
        to_export_.push_back(*run_);
      }
    }
    run_ = info;
    jog_armed_after_ns_.reset();
  }
  pending_.erase(p);
}

// Opens a jog run, closes the open run at its first standby / holding sample (a jog run: see
// below) and exports the closed runs once a sample settle_window_s past their end arrived.
void BridgeClient::track() {
  if (live_.empty()) {
    return;
  }
  if (!run_ && jog_armed_after_ns_) {
    for (const Sample& s : live_) {
      const auto cmd = static_cast<CommandMode>(s.cmd_mode);
      if (s.t_ns > *jog_armed_after_ns_ && is(s, LoopMode::kTracking) &&
          (cmd == CommandMode::kJogJoint || cmd == CommandMode::kJogTcp)) {
        LiveRunInfo info;
        info.cmd = cmd;
        info.t_start_ns = s.t_ns;
        info.config = session_config(session_);
        run_ = info;
        jog_armed_after_ns_.reset();
        break;
      }
    }
  }
  if (run_) {
    close_run(jog_.direction != 0 && ep_->is_owner());
  }
  // Exporting needs authority: without it the closed runs wait for the next claim.
  if (!ep_->is_owner()) {
    return;
  }
  const std::int64_t t_last = live_.back().t_ns;
  for (auto it = to_export_.begin(); it != to_export_.end();) {
    const std::int64_t t1 = it->t_end_ns + std::llround(it->settle_window_s * 1e9);
    if (t_last < t1) {
      ++it;
      continue;
    }
    Command c{};
    c.type = bridge::CommandType::kExportSegment;
    c.t0_ns = it->t_start_ns;
    c.t1_ns = t1;
    const common::Result<std::uint64_t> id = send(c, "export segment");
    if (!id) {
      reason_ = id.reason();
      if (id.reason().find("ring full") != std::string::npos) {
        break;  // retried on the next tick
      }
    } else {
      exports_[id.value()] = *it;
    }
    it = to_export_.erase(it);
  }
}

// A jog run is one press: the jog comes to rest while the button is still held when the refreshes
// stop for longer than the deadman (a stalled GUI) or at a soft limit, and the refreshes after
// that start it again. So a jog run ends at the first standby after its last jog sample once the
// button is released (held: pressed by the owner); a holding sample ends any run.
void BridgeClient::close_run(bool held) {
  const bool jog_run = run_->cmd == CommandMode::kJogJoint || run_->cmd == CommandMode::kJogTcp;
  held = jog_run && held;
  std::size_t from = 0;
  for (std::size_t k = 0; jog_run && k < live_.size(); ++k) {
    const Sample& s = live_[k];
    from = s.t_ns > run_->t_start_ns && is(s, LoopMode::kTracking) &&
                   static_cast<CommandMode>(s.cmd_mode) == run_->cmd
               ? k
               : from;
  }
  for (std::size_t k = from; k < live_.size(); ++k) {
    const Sample& s = live_[k];
    if (s.t_ns > run_->t_start_ns &&
        ((is(s, LoopMode::kStandby) && !held) || is(s, LoopMode::kHolding))) {
      run_->t_end_ns = s.t_ns;
      to_export_.push_back(*run_);
      run_.reset();
      break;
    }
  }
}

}  // namespace cobot::studio
