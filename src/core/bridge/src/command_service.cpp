#include "cobot/bridge/command_service.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <optional>
#include <system_error>

#include "cobot/bridge/segment.hpp"
#include "cobot/common/dof.hpp"
#include "cobot/common/types.hpp"
#include "cobot/pipeline/manual.hpp"

namespace cobot::bridge {
namespace {

using pipeline::CommandMode;
using pipeline::LoopMode;

constexpr std::size_t kMarks = 64;  // export marks kept
constexpr int kStatusReads = 4;     // seqlock read attempts (the writer never waits)
constexpr double kNsPerS = 1e9;

const char* name_of(CommandType t) {
  switch (t) {
    case CommandType::kMoveJoint:
      return "move_joint";
    case CommandType::kMoveTcp:
      return "move_tcp";
    case CommandType::kRunWaypoints:
      return "run_waypoints";
    case CommandType::kPreviewWaypoints:
      return "preview_waypoints";
    case CommandType::kStop:
      return "stop";
    case CommandType::kEstop:
      return "estop";
    case CommandType::kReset:
      return "reset";
    case CommandType::kApplyConfig:
      return "apply_config";
    case CommandType::kExportSegment:
      return "export_segment";
    case CommandType::kJog:
      return "jog";
  }
  return "command";
}

bool is_motion(CommandType t) {
  return t == CommandType::kMoveJoint || t == CommandType::kMoveTcp ||
         t == CommandType::kRunWaypoints;
}

// Command modes only the bridge starts (manual commands); waypoints may also come from the
// config or from the RViz panel.
bool is_manual(std::uint8_t cmd) {
  const auto m = static_cast<CommandMode>(cmd);
  return m == CommandMode::kJogJoint || m == CommandMode::kJogTcp || m == CommandMode::kMoveJoint ||
         m == CommandMode::kMoveTcp;
}

template <int N>
common::VecN<N> vec_of(const double (&v)[N]) {
  common::VecN<N> out;
  for (int i = 0; i < N; ++i) {
    out(i) = v[i];
  }
  return out;
}

}  // namespace

template <int N>
CommandService<N>::CommandService(ServerEndpoint<N>& endpoint,
                                  LoopPort<N>& port,
                                  model::Model<N> model,
                                  pipeline::Config<N> config,
                                  SessionInfo<N> session,
                                  std::filesystem::path segment_dir,
                                  Hooks hooks)
    : endpoint_(endpoint),
      port_(port),
      model_(std::move(model)),
      config_(std::move(config)),
      session_(session),
      segment_dir_(std::move(segment_dir)),
      hooks_(std::move(hooks)),
      period_ns_(std::llround(kNsPerS / config_.control_rate_hz)),
      command_(std::make_unique<Command<N>>()),
      response_(std::make_unique<Response>()),
      post_(std::make_unique<LoopCommand<N>>()) {}

template <int N>
void CommandService<N>::poll(std::int64_t now_ns) {
  endpoint_.beat(now_ns);
  watchdog(now_ns);
  while (endpoint_.pop_command(*command_)) {
    handle(*command_);
  }
  answer_acks();
}

template <int N>
bool CommandService<N>::studio_in_control() const {
  return endpoint_.owner_token() != 0;
}

template <int N>
bool CommandService<N>::read_status(LoopStatus<N>& out) const {
  for (int k = 0; k < kStatusReads; ++k) {
    if (port_.status(out)) {
      return true;
    }
  }
  return false;
}

// ADR-022: an owner that is lost, or replaced / released while the loop runs (or is about to
// run) a command the studio started, stops the loop with deceleration; a lost owner is cleared
// once that motion is over. A motion the studio did not start (the config's trajectory, an RViz
// goal) runs on. A motion posted but not yet applied counts as the studio's whatever the loop's
// mode (the control thread may lag the bridge thread), and the stop request is latched: kStop is
// posted on every poll until the post succeeds, then kept until the loop is out of tracking with
// no studio motion left in the command ring (FIFO: the stop is applied after it).
template <int N>
void CommandService<N>::watchdog(std::int64_t now_ns) {
  LoopStatus<N> st{};
  const bool have = read_status(st);
  const bool tracking = have && static_cast<LoopMode>(st.mode) == LoopMode::kTracking;
  if (have && !tracking) {
    studio_motion_ = false;
  }
  const bool posted = std::any_of(
      pending_.begin(), pending_.end(), [](const Pending& p) { return is_motion(p.type); });
  const bool studio = posted || (tracking && (studio_motion_ || is_manual(st.cmd)));
  std::uint64_t token = endpoint_.owner_token();
  const OwnerState state = endpoint_.owner_state(now_ns);
  const bool lost =
      token != 0 && (state == OwnerState::kHeartbeatLost || state == OwnerState::kProcessGone);
  const bool changed = last_owner_ != 0 && token != last_owner_;
  if ((lost || changed) && studio) {
    stop_wanted_ = true;
  }
  if (stop_wanted_ && !stop_posted_) {
    *post_ = LoopCommand<N>{};
    post_->kind = LoopCommand<N>::Kind::kStop;
    stop_posted_ = port_.post(*post_);
  }
  if (stop_wanted_ && stop_posted_ && !posted && have && !tracking) {
    stop_wanted_ = false;  // the stop was applied after the last studio motion: done
    stop_posted_ = false;
  }
  if (lost && have && !studio && !stop_wanted_) {
    endpoint_.clear_owner();
    token = endpoint_.owner_token();
  }
  last_owner_ = token;
}

template <int N>
Response& CommandService<N>::reply(const Command<N>& c, const std::string& text) {
  Response& r = *response_;
  r = Response{};
  r.command_id = c.id;
  r.type = c.type;
  copy_text(r.text, kTextChars, text);
  return r;
}

template <int N>
void CommandService<N>::push() {
  static_cast<void>(endpoint_.push_response(*response_));  // full: counted by the endpoint
}

template <int N>
bool CommandService<N>::motion_allowed(const Command<N>& c, const LoopStatus<N>& st) {
  const std::string name = name_of(c.type);
  if (config_.command_source != pipeline::CommandSource::kExternal) {
    reply(c, name + ": command_source is config");
  } else if (static_cast<LoopMode>(st.mode) != LoopMode::kStandby) {
    reply(c,
          name + ": not in standby (mode " + pipeline::to_string(static_cast<LoopMode>(st.mode)) +
              ")");
  } else {
    return true;
  }
  push();
  return false;
}

// The loop's stamp is read after the export returned: it bounds every exported row from above,
// so the rows before the mark's offset are all at or before its stamp.
template <int N>
void CommandService<N>::add_mark(std::uint64_t size_after) {
  LoopStatus<N> st{};
  if (!read_status(st)) {
    return;  // a later segment copy then scans from an older mark
  }
  if (marks_.size() == kMarks) {
    marks_.erase(marks_.begin());
  }
  marks_.push_back(Mark{st.t_ns, size_after});
}

template <int N>
void CommandService<N>::export_mark() {
  if (!hooks_.export_log) {
    return;
  }
  const common::Result<ExportInfo> info = hooks_.export_log();
  if (info) {
    add_mark(info.value().size_after);
  }  // else a later segment copy scans from an older mark
}

template <int N>
void CommandService<N>::post_motion(const Command<N>& c, LoopCommand<N>& lc, double duration_s) {
  export_mark();
  post_pending(c, lc, duration_s);
}

// Posts lc under a service sequence number (never 0, the watchdog's own stop): the wire ids of
// two clients may collide after a takeover, the sequence never does.
template <int N>
void CommandService<N>::post_pending(const Command<N>& c, LoopCommand<N>& lc, double duration_s) {
  lc.id = ++sequence_;
  if (!port_.post(lc)) {
    reply(c, std::string(name_of(c.type)) + ": the control loop is busy");
    push();
    return;
  }
  pending_.push_back(Pending{lc.id, c.id, c.type, duration_s});
}

template <int N>
void CommandService<N>::issues_into(const std::vector<pipeline::WaypointIssue>& issues) {
  Response& r = *response_;
  r.issue_count = static_cast<std::uint32_t>(std::min(issues.size(), kWireIssues));
  for (std::size_t i = 0; i < r.issue_count; ++i) {
    const pipeline::WaypointIssue& w = issues[i];
    r.issues[i].waypoint = static_cast<std::uint32_t>(w.waypoint);
    r.issues[i].issue = static_cast<std::uint32_t>(w.issue);
    r.issues[i].joint = w.joint;
    r.issues[i].value = w.value;
    r.issues[i].limit = w.limit;
  }
}

template <int N>
void CommandService<N>::plan_waypoints(const Command<N>& c, const LoopStatus<N>& st) {
  const std::string name = name_of(c.type);
  if (c.waypoint_count == 0 || c.waypoint_count > kWireWaypoints) {
    reply(c, name + ": " + std::to_string(c.waypoint_count) + " waypoints, want 1 .. 32");
    push();
    return;
  }
  const common::Pose& T_tip_tcp = config_.tcp_offset.T_tip_tcp;
  const common::Pose T_tcp_tip = common::inverse(T_tip_tcp);
  std::vector<planning::Waypoint> list(c.waypoint_count);
  for (std::size_t i = 0; i < list.size(); ++i) {
    list[i].T_b_goal = common::compose(from_wire(c.waypoints[i].T_b_tcp), T_tcp_tip);
    list[i].duration_s = c.waypoints[i].duration_s;
    list[i].stop = c.waypoints[i].stop != 0;
  }
  const common::Result<pipeline::PathValidation<N>> checked =
      pipeline::validate_path<N>(model_,
                                 config_.clik,
                                 config_.friction_compensation,
                                 vec_of<N>(st.standby_q),
                                 from_wire(st.standby_reference_tip),
                                 list,
                                 config_.cartesian_limits,
                                 config_.path_validation);
  if (!checked) {
    reply(c, name + ": " + checked.reason());
    push();
    return;
  }
  const pipeline::PathValidation<N>& v = checked.value();
  const bool ok = v.ok && v.trajectory.has_value();
  if (c.type == CommandType::kRunWaypoints && ok) {
    LoopCommand<N>& lc = *post_;
    lc = LoopCommand<N>{};
    lc.kind = LoopCommand<N>::Kind::kLoad;
    lc.cmd = pipeline::CommandMode::kWaypoints;
    lc.trajectory = v.trajectory;
    post_motion(c, lc, v.trajectory->duration_s());
    return;
  }
  Response& r = reply(c, name + (ok ? ": ok" : ": path validation failed"));
  issues_into(v.issues);
  if (c.type == CommandType::kPreviewWaypoints) {
    r.accepted = ok ? 1U : 0U;
    r.duration_s = ok ? v.trajectory->duration_s() : 0.0;
    r.segment_count =
        static_cast<std::uint32_t>(std::min(v.segment_durations_s.size(), kWireWaypoints));
    for (std::size_t i = 0; i < r.segment_count; ++i) {
      r.segment_durations_s[i] = v.segment_durations_s[i];
    }
    // Desired TCP path, evenly decimated to <= kPreviewPoints, the last point kept.
    const std::size_t n = v.path.size();
    r.path_count = static_cast<std::uint32_t>(std::min(n, kPreviewPoints));
    for (std::size_t k = 0; k < r.path_count; ++k) {
      const std::size_t i = r.path_count < 2 ? n - 1 : k * (n - 1) / (r.path_count - 1);
      const common::Pose tcp = common::compose(v.path[i], T_tip_tcp);
      r.path[k][0] = tcp.p.x();
      r.path[k][1] = tcp.p.y();
      r.path[k][2] = tcp.p.z();
    }
  }
  push();
}

template <int N>
void CommandService<N>::export_segment(const Command<N>& c) {
  if (!hooks_.export_log) {
    reply(c, "export: not supported by this host");
    push();
    return;
  }
  const common::Result<ExportInfo> info = hooks_.export_log();
  if (!info) {
    reply(c, info.reason());
    push();
    return;
  }
  // Rows after the latest mark before t0 start at that mark's size: only the tail is scanned.
  // Every row before m.size is at or before m.t_ns < t0 (the stamp is read after the export).
  std::uint64_t offset = 0;
  for (const Mark& m : marks_) {
    if (m.t_ns < c.t0_ns) {
      offset = m.size;
    }
  }
  add_mark(info.value().size_after);
  std::error_code ec;
  std::filesystem::create_directories(segment_dir_, ec);
  const std::filesystem::path out = segment_dir_ / ("segment-" + std::to_string(c.id) + ".csv");
  const common::Result<SegmentCopy> copy =
      copy_csv_range(info.value().csv, offset, c.t0_ns, c.t1_ns, period_ns_, out);
  if (!copy) {
    reply(c, copy.reason());
    push();
    return;
  }
  Response& r = reply(c, std::to_string(copy.value().rows) + " rows");
  r.accepted = 1;
  r.rows = copy.value().rows;
  r.truncated = copy.value().truncated ? 1U : 0U;
  copy_text(r.out_path, kPathChars, out.string());
  push();
}

template <int N>
void CommandService<N>::apply_config(const Command<N>& c, const LoopStatus<N>& st) {
  if (!motion_allowed(c, st)) {
    return;
  }
  if (!hooks_.apply_config) {
    reply(c, "apply config: not supported by this host");
    push();
    return;
  }
  const std::filesystem::path config(std::string(c.path_a, strnlen(c.path_a, kPathChars)));
  const std::filesystem::path imperfections(std::string(c.path_b, strnlen(c.path_b, kPathChars)));
  auto applied = hooks_.apply_config(config, imperfections);
  if (!applied) {
    reply(c, applied.reason());
    push();
    return;
  }
  model_ = std::move(applied.value().first);
  config_ = std::move(applied.value().second);
  period_ns_ = std::llround(kNsPerS / config_.control_rate_hz);
  SessionInfo<N>& s = session_;
  if (hooks_.seed) {
    s.seed = hooks_.seed();
  }
  copy_text(s.config_path, kPathChars, config.string());
  copy_text(s.imperfections_path, kPathChars, imperfections.string());
  s.command_source_external =
      config_.command_source == pipeline::CommandSource::kExternal ? 1U : 0U;
  s.control_rate_hz = config_.control_rate_hz;
  s.soft_margin_rad = config_.safety.soft_margin_rad;
  s.jog_margin_rad = config_.manual.jog_margin_rad;
  for (std::size_t i = 0; i < static_cast<std::size_t>(N); ++i) {
    const auto& joint = model_.joints[i];
    s.q_min[i] = joint.q_min;
    s.q_max[i] = joint.q_max;
    s.qd_max[i] = joint.qd_max;
    s.tau_max[i] = joint.tau_max;
    s.rated[i] = config_.limits.rated_torque_nm(static_cast<Eigen::Index>(i));
  }
  s.T_tip_tcp = to_wire(config_.tcp_offset.T_tip_tcp);
  endpoint_.set_session(s);
  reply(c, "applied").accepted = 1;
  push();
}

template <int N>
void CommandService<N>::handle(const Command<N>& c) {
  const std::uint64_t owner = endpoint_.owner_token();
  if (c.owner_token == 0 || c.owner_token != owner) {
    reply(c, std::string(name_of(c.type)) + ": not the owner");
    push();
    return;
  }
  LoopStatus<N> st{};
  static_cast<void>(read_status(st));
  LoopCommand<N>& lc = *post_;
  lc = LoopCommand<N>{};
  using Kind = typename LoopCommand<N>::Kind;
  switch (c.type) {
    case CommandType::kMoveJoint: {
      if (!motion_allowed(c, st)) {
        return;
      }
      const common::Result<pipeline::JointMove<N>> plan = pipeline::plan_joint_move<N>(
          model_, config_, vec_of<N>(st.joint_reference), vec_of<N>(c.q_goal), c.speed);
      if (!plan) {
        reply(c, plan.reason());
        push();
        return;
      }
      lc.kind = Kind::kMoveJoint;
      lc.move = plan.value();
      post_motion(c, lc, plan.value().duration_s);
      return;
    }
    case CommandType::kMoveTcp: {
      if (!motion_allowed(c, st)) {
        return;
      }
      const common::Pose goal_tip =
          common::compose(from_wire(c.T_b_tcp), common::inverse(config_.tcp_offset.T_tip_tcp));
      const common::Result<pipeline::PathValidation<N>> plan =
          pipeline::plan_tcp_move<N>(model_,
                                     config_,
                                     vec_of<N>(st.standby_q),
                                     from_wire(st.standby_reference_tip),
                                     goal_tip,
                                     c.speed);
      if (!plan) {
        reply(c, plan.reason());
        push();
        return;
      }
      if (!plan.value().ok || !plan.value().trajectory) {
        reply(c, "move_tcp: path validation failed");
        issues_into(plan.value().issues);
        push();
        return;
      }
      lc.kind = Kind::kLoad;
      lc.cmd = pipeline::CommandMode::kMoveTcp;
      lc.trajectory = plan.value().trajectory;
      post_motion(c, lc, lc.trajectory->duration_s());
      return;
    }
    case CommandType::kRunWaypoints:
      if (motion_allowed(c, st)) {
        plan_waypoints(c, st);
      }
      return;
    case CommandType::kPreviewWaypoints:
      plan_waypoints(c, st);
      return;
    case CommandType::kStop:
    case CommandType::kEstop:
    case CommandType::kReset:
      lc.kind = c.type == CommandType::kStop    ? Kind::kStop
                : c.type == CommandType::kEstop ? Kind::kEstop
                                                : Kind::kReset;
      post_pending(c, lc, 0.0);
      return;
    case CommandType::kApplyConfig:
      apply_config(c, st);
      return;
    case CommandType::kExportSegment:
      export_segment(c);
      return;
    case CommandType::kJog:
      break;
  }
  reply(c, "unknown command type " + std::to_string(static_cast<std::uint32_t>(c.type)));
  push();
}

template <int N>
void CommandService<N>::answer_acks() {
  LoopAck ack;
  while (port_.take_ack(ack)) {
    const auto it = std::find_if(
        pending_.begin(), pending_.end(), [&](const Pending& p) { return p.sequence == ack.id; });
    if (ack.id == 0 || it == pending_.end()) {
      continue;  // the service's own stop
    }
    Response& r = *response_;
    r = Response{};
    r.command_id = it->command_id;
    r.type = it->type;
    if (ack.status.is_ok()) {
      studio_motion_ = studio_motion_ || is_motion(it->type);
      r.accepted = 1;
      r.t_start_ns = ack.t_ns;
      r.duration_s = it->duration_s;
      copy_text(r.text, kTextChars, "accepted");
    } else {
      copy_text(r.text, kTextChars, ack.status.reason());
    }
    pending_.erase(it);
    push();
  }
  JogAck jog;
  while (port_.take_jog_ack(jog)) {
    Response& r = *response_;
    r = Response{};
    r.command_id = jog.refresh;
    r.type = CommandType::kJog;
    copy_text(r.text, kTextChars, jog.rejected != nullptr ? jog.rejected : "jog: rejected");
    push();
  }
}

#define COBOT_INSTANTIATE(N) template class CommandService<N>;
COBOT_INSTANTIATE_DOF(COBOT_INSTANTIATE)
#undef COBOT_INSTANTIATE

}  // namespace cobot::bridge
