// cobot_bridge_cli (feature controller-bridge, plan owner question B3): headless client of the
// studio bridge, so the bridge is driven end to end without a GUI (cobot_bench serve, Gazebo).
//
//   cobot_bridge_cli <name> [--speed <pct 1..100, default 10>] [--timeout-s <s, default 60>]
//                    <step>...
//
// Steps (docs/conventions.md "Studio bridge"): claim | release | pause:<s> | wait-standby | stop |
// estop | reset | jog-joint:<1..7>:<+|->:<s> | jog-tcp:<x|y|z|rx|ry|rz>:<+|->:<base|tool>:<s> |
// move-joint:<q1_deg,...,q7_deg> | move-tcp:<x,y,z,roll_deg,pitch_deg,yaw_deg> (TCP in base_link,
// R = Rz(yaw) Ry(pitch) Rx(roll)) | waypoints:<file>[:<count>] | preview:<file> | export:<csv>
// (segment of the last motion step, [t_start, standby + 1 s]). One line per step: "<k> <step>: ok
// <detail>", "<k> <step>: rejected: <reason>" or "<k> <step>: failed: <reason>". Exit 0 all ok;
// 1 a step rejected, failed or timed out (later steps skipped); 2 bad arguments, no bridge or a
// layout mismatch. Authority is released on every exit path; a thread beats every 50 ms from
// `claim` on.
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "cobot/bridge/endpoint.hpp"
#include "cobot/bridge/wire.hpp"
#include "cobot/common/types.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/log_record.hpp"
#include "cobot/pipeline/waypoint_file.hpp"

namespace {

namespace fs = std::filesystem;
namespace bridge = cobot::bridge;
using cobot::pipeline::LoopMode;

constexpr int kN = cobot::model::kArmDof;
constexpr int kExitFailed = 1;
constexpr int kExitBadInput = 2;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr std::int64_t kNsPerS = 1000000000;
constexpr auto kPoll = std::chrono::milliseconds(5);
constexpr auto kJogRefresh = std::chrono::milliseconds(20);
constexpr std::int64_t kExportTailNs = kNsPerS;  // export: standby + 1 s

const char* const kUsage =
    "usage: cobot_bridge_cli <name> [--speed <pct 1..100>] [--timeout-s <s>] <step>...\n"
    "  steps: claim | release | pause:<s> | wait-standby | stop | estop | reset |\n"
    "         jog-joint:<1..7>:<+|->:<s> | jog-tcp:<x|y|z|rx|ry|rz>:<+|->:<base|tool>:<s> |\n"
    "         move-joint:<q1_deg,...,q7_deg> | move-tcp:<x,y,z,roll_deg,pitch_deg,yaw_deg> |\n"
    "         waypoints:<file>[:<count>] | preview:<file> | export:<csv>\n";

using Client = bridge::ClientEndpoint<kN>;
using Command = bridge::Command<kN>;

std::vector<std::string> split(const std::string& text, char sep) {
  std::vector<std::string> parts;
  std::size_t start = 0;
  for (;;) {
    const std::size_t at = text.find(sep, start);
    parts.push_back(text.substr(start, at - start));
    if (at == std::string::npos) {
      return parts;
    }
    start = at + 1;
  }
}

double number(const std::string& text) {
  std::size_t used = 0;
  const double v = std::stod(text, &used);
  if (used != text.size() || !std::isfinite(v)) {
    throw std::invalid_argument("not a number: " + text);
  }
  return v;
}

int direction_of(const std::string& text) {
  if (text == "+" || text == "-") {
    return text == "+" ? 1 : -1;
  }
  throw std::invalid_argument("direction must be + or -, got " + text);
}

// A parsed step; `jog` and `command` are filled for the steps that use them.
struct Step {
  std::string text, verb;
  double seconds = 0.0;
  bridge::JogRequest jog{};
  Command command{};
  std::string path;  // export target
};

Step parse_step(const std::string& text, double speed) {
  Step s;
  s.text = text;
  const std::vector<std::string> f = split(text, ':');
  s.verb = f[0];
  const auto want = [&](std::size_t n) {
    if (f.size() != n) {
      throw std::invalid_argument("step " + text + ": expected " + std::to_string(n - 1) +
                                  " fields");
    }
  };
  if (s.verb == "claim" || s.verb == "release" || s.verb == "wait-standby" || s.verb == "stop" ||
      s.verb == "estop" || s.verb == "reset") {
    want(1);
    s.command.type = s.verb == "stop"    ? bridge::CommandType::kStop
                     : s.verb == "estop" ? bridge::CommandType::kEstop
                                         : bridge::CommandType::kReset;
  } else if (s.verb == "pause") {
    want(2);
    s.seconds = number(f[1]);
  } else if (s.verb == "jog-joint") {
    want(4);
    const double joint = number(f[1]);
    if (joint < 1 || joint > kN || joint != std::floor(joint)) {
      throw std::invalid_argument("step " + text + ": joint must be 1.." + std::to_string(kN));
    }
    s.jog.kind = 1;
    s.jog.joint = static_cast<std::int32_t>(joint) - 1;
    s.jog.direction = direction_of(f[2]);
    s.seconds = number(f[3]);
  } else if (s.verb == "jog-tcp") {
    want(5);
    const char* const axes[] = {"x", "y", "z", "rx", "ry", "rz"};
    s.jog.axis = -1;
    for (int a = 0; a < 6; ++a) {
      s.jog.axis = f[1] == axes[a] ? a : s.jog.axis;
    }
    if (s.jog.axis < 0 || (f[3] != "base" && f[3] != "tool")) {
      throw std::invalid_argument("step " + text + ": bad axis or frame");
    }
    s.jog.kind = 2;
    s.jog.direction = direction_of(f[2]);
    s.jog.frame = f[3] == "tool" ? 1 : 0;
    s.seconds = number(f[4]);
  } else if (s.verb == "move-joint") {
    want(2);
    const std::vector<std::string> q = split(f[1], ',');
    if (q.size() != static_cast<std::size_t>(kN)) {
      throw std::invalid_argument("step " + text + ": expected " + std::to_string(kN) + " angles");
    }
    s.command.type = bridge::CommandType::kMoveJoint;
    for (int i = 0; i < kN; ++i) {
      s.command.q_goal[i] = number(q[static_cast<std::size_t>(i)]) * kDegToRad;
    }
  } else if (s.verb == "move-tcp") {
    want(2);
    const std::vector<std::string> v = split(f[1], ',');
    if (v.size() != 6) {
      throw std::invalid_argument("step " + text + ": expected x,y,z,roll,pitch,yaw");
    }
    cobot::common::Pose T;
    T.p = cobot::common::Vec3(number(v[0]), number(v[1]), number(v[2]));
    const cobot::common::Mat3 R =
        (Eigen::AngleAxisd(number(v[5]) * kDegToRad, cobot::common::Vec3::UnitZ()) *
         Eigen::AngleAxisd(number(v[4]) * kDegToRad, cobot::common::Vec3::UnitY()) *
         Eigen::AngleAxisd(number(v[3]) * kDegToRad, cobot::common::Vec3::UnitX()))
            .toRotationMatrix();
    T.R = R;
    s.command.type = bridge::CommandType::kMoveTcp;
    s.command.T_b_tcp = bridge::to_wire(T);
  } else if (s.verb == "waypoints" || s.verb == "preview") {
    if (f.size() < 2) {
      throw std::invalid_argument("step " + text + ": missing file");
    }
    // waypoints:<file>[:<count>]: a trailing integer field is the count.
    std::string file = text.substr(s.verb.size() + 1);
    std::size_t count = 0;
    const std::size_t colon = file.rfind(':');
    if (s.verb == "waypoints" && colon != std::string::npos &&
        file.find_first_not_of("0123456789", colon + 1) == std::string::npos &&
        colon + 1 < file.size()) {
      count = std::stoul(file.substr(colon + 1));
      file = file.substr(0, colon);
    }
    // TCP poses as written: identity TcpOffset.
    const auto list = cobot::pipeline::load_waypoints(file, "base_link", {});
    if (!list) {
      throw std::invalid_argument(list.reason());
    }
    std::size_t n = list.value().size();
    n = count > 0 && count < n ? count : n;
    if (n == 0 || n > bridge::kWireWaypoints) {
      throw std::invalid_argument("step " + text + ": " + std::to_string(n) + " waypoints");
    }
    s.command.type = s.verb == "waypoints" ? bridge::CommandType::kRunWaypoints
                                           : bridge::CommandType::kPreviewWaypoints;
    s.command.waypoint_count = static_cast<std::uint32_t>(n);
    for (std::size_t i = 0; i < n; ++i) {
      const cobot::planning::Waypoint& w = list.value()[i];
      s.command.waypoints[i].T_b_tcp = bridge::to_wire(w.T_b_goal);
      s.command.waypoints[i].duration_s = w.duration_s;
      s.command.waypoints[i].stop = w.stop || i + 1 == n ? 1U : 0U;
    }
  } else if (s.verb == "export") {
    if (f.size() < 2 || text.size() <= s.verb.size() + 1) {
      throw std::invalid_argument("step " + text + ": missing csv");
    }
    s.path = text.substr(s.verb.size() + 1);
    s.command.type = bridge::CommandType::kExportSegment;
  } else {
    throw std::invalid_argument("unknown step " + text);
  }
  s.command.speed = speed;
  s.jog.speed = speed;
  return s;
}

std::string text_of(const char* chars, std::size_t size) {
  return {chars, strnlen(chars, size)};
}

struct Sample {
  std::int64_t t_ns = 0;
  LoopMode mode = LoopMode::kIdle;
  cobot::pipeline::HoldReason hold = cobot::pipeline::HoldReason::kNone;
};

// Outcome of one step: ok (detail), rejected or failed (reason).
struct Outcome {
  enum Kind { kOk, kRejected, kFailed } kind = kOk;
  std::string text;
};

Outcome ok(std::string detail = {}) {
  return {Outcome::kOk, std::move(detail)};
}
Outcome rejected(std::string why) {
  return {Outcome::kRejected, std::move(why)};
}
Outcome failed(std::string why) {
  return {Outcome::kFailed, std::move(why)};
}

class Session {
 public:
  Session(Client client, double timeout_s) : client_(std::move(client)), timeout_s_(timeout_s) {}
  Session(const Session&) = delete;
  Session& operator=(const Session&) = delete;
  Session(Session&&) = delete;
  Session& operator=(Session&&) = delete;
  ~Session() { release(); }

  Outcome run(const Step& s) {
    if (s.verb == "claim") {
      const cobot::common::Status st = client_.claim(bridge::monotonic_ns());
      if (!st) {
        return rejected(st.reason());
      }
      start_beating();
      return ok();
    }
    if (s.verb == "release") {
      release();
      return ok();
    }
    if (s.verb == "pause") {
      std::this_thread::sleep_for(std::chrono::duration<double>(s.seconds));
      return ok();
    }
    if (s.verb == "wait-standby") {
      return wait_standby(-1);
    }
    if (s.verb == "jog-joint" || s.verb == "jog-tcp") {
      return jog(s);
    }
    if (s.verb == "export") {
      return export_segment(s);
    }
    return command(s);
  }

 private:
  using Clock = std::chrono::steady_clock;

  Clock::time_point deadline() const {
    return Clock::now() +
           std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout_s_));
  }

  void start_beating() {
    if (beater_.joinable()) {
      return;
    }
    beater_ = std::thread([this] {
      while (!stop_beating_.load()) {
        client_.beat(bridge::monotonic_ns());
        std::this_thread::sleep_for(std::chrono::nanoseconds(bridge::kHeartbeatPeriodNs));
      }
    });
  }

  void release() {
    client_.release();
    stop_beating_.store(true);
    if (beater_.joinable()) {
      beater_.join();
    }
    stop_beating_.store(false);
  }

  void pump() {
    buffer_.clear();
    client_.read_states(buffer_);
    for (const bridge::StateSample<kN>& s : buffer_) {
      history_.push_back(Sample{s.t_ns,
                                static_cast<LoopMode>(s.mode),
                                static_cast<cobot::pipeline::HoldReason>(s.hold_reason)});
    }
  }

  // Stamp of the first standby sample later than after_ns (-1: the current state); holding
  // fails.
  Outcome wait_standby(std::int64_t after_ns, std::int64_t* standby_ns = nullptr) {
    const Clock::time_point end = deadline();
    std::size_t k = 0;
    while (Clock::now() < end) {
      pump();
      if (after_ns < 0 && !history_.empty()) {
        k = history_.size() - 1;  // the current state only
      }
      for (; k < history_.size(); ++k) {
        const Sample& s = history_[k];
        if (s.t_ns <= after_ns) {
          continue;
        }
        if (s.mode == LoopMode::kHolding) {
          return failed(std::string("held: ") + cobot::pipeline::to_string(s.hold));
        }
        if (s.mode == LoopMode::kStandby) {
          if (standby_ns != nullptr) {
            *standby_ns = s.t_ns;
          }
          return ok();
        }
      }
      if (!client_.server_alive(bridge::monotonic_ns())) {
        return failed("bridge server gone");
      }
      std::this_thread::sleep_for(kPoll);
    }
    return failed("timeout waiting for standby");
  }

  // The response to command `id` (jog responses are kept in jog_rejection_).
  std::optional<bridge::Response> await(std::uint64_t id) {
    const Clock::time_point end = deadline();
    bridge::Response r{};
    while (Clock::now() < end) {
      while (client_.poll_response(r)) {
        if (r.type == bridge::CommandType::kJog) {
          jog_rejection_ = text_of(r.text, bridge::kTextChars);
        } else if (r.command_id == id) {
          return r;
        }
      }
      pump();
      std::this_thread::sleep_for(kPoll);
    }
    return std::nullopt;
  }

  std::int64_t latest_ns() {
    pump();
    return history_.empty() ? 0 : history_.back().t_ns;
  }

  Outcome jog(const Step& s) {
    if (!client_.is_owner()) {
      return rejected("not the owner");
    }
    jog_rejection_.clear();
    const Clock::time_point end = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                                     std::chrono::duration<double>(s.seconds));
    while (Clock::now() < end && jog_rejection_.empty()) {
      client_.write_jog(s.jog);
      std::this_thread::sleep_for(kJogRefresh);
      bridge::Response r{};
      while (client_.poll_response(r)) {
        if (r.type == bridge::CommandType::kJog) {
          jog_rejection_ = text_of(r.text, bridge::kTextChars);
        }
      }
    }
    bridge::JogRequest release = s.jog;
    release.direction = 0;
    client_.write_jog(release);
    const std::int64_t released_ns = latest_ns();
    if (!jog_rejection_.empty()) {
      return rejected(jog_rejection_);
    }
    return wait_standby(released_ns);
  }

  Outcome command(const Step& s) {
    const cobot::common::Result<std::uint64_t> id = client_.send(s.command);
    if (!id) {
      return rejected(id.reason());
    }
    const std::optional<bridge::Response> r = await(id.value());
    if (!r) {
      return failed("no response within the timeout");
    }
    std::string text = text_of(r->text, bridge::kTextChars);
    for (std::uint32_t i = 0; i < r->issue_count; ++i) {
      const bridge::WireIssue& w = r->issues[i];
      text += "; waypoint " + std::to_string(w.waypoint) + " " +
              cobot::pipeline::to_string(static_cast<cobot::pipeline::PathIssue>(w.issue)) +
              (w.joint >= 0 ? " joint " + std::to_string(w.joint) : std::string()) + " " +
              std::to_string(w.value) + " vs " + std::to_string(w.limit);
    }
    if (r->accepted == 0) {
      return rejected(text);
    }
    const bridge::CommandType t = s.command.type;
    if (t == bridge::CommandType::kPreviewWaypoints) {
      return ok(std::to_string(r->path_count) + " path points, " + std::to_string(r->duration_s) +
                " s");
    }
    if (t != bridge::CommandType::kMoveJoint && t != bridge::CommandType::kMoveTcp &&
        t != bridge::CommandType::kRunWaypoints) {
      return ok();
    }
    std::int64_t standby_ns = 0;
    Outcome done = wait_standby(r->t_start_ns, &standby_ns);
    if (done.kind == Outcome::kOk) {
      motion_ = std::make_pair(r->t_start_ns, standby_ns);
      done.text = std::to_string(r->duration_s) + " s";
    }
    return done;
  }

  Outcome export_segment(const Step& s) {
    if (!motion_) {
      return failed("no motion step to export");
    }
    const std::int64_t t1 = motion_->second + kExportTailNs;
    const Clock::time_point end = deadline();
    while (latest_ns() < t1 && Clock::now() < end) {  // the rows up to t1 exist
      std::this_thread::sleep_for(kPoll);
    }
    Step e = s;
    e.command.t0_ns = motion_->first;
    e.command.t1_ns = t1;
    const cobot::common::Result<std::uint64_t> id = client_.send(e.command);
    if (!id) {
      return rejected(id.reason());
    }
    const std::optional<bridge::Response> r = await(id.value());
    if (!r) {
      return failed("no response within the timeout");
    }
    if (r->accepted == 0) {
      return rejected(text_of(r->text, bridge::kTextChars));
    }
    const fs::path from = text_of(r->out_path, bridge::kPathChars);
    const fs::path to = s.path;
    std::error_code ec;
    if (to.has_parent_path()) {
      fs::create_directories(to.parent_path(), ec);
    }
    fs::copy_file(from, to, fs::copy_options::overwrite_existing, ec);
    if (ec) {
      return failed("cannot copy " + from.string() + " to " + to.string() + ": " + ec.message());
    }
    return ok(std::to_string(r->rows) + " rows -> " + to.string() +
              (r->truncated != 0 ? " (truncated)" : ""));
  }

  Client client_;
  double timeout_s_;
  std::thread beater_;
  std::atomic<bool> stop_beating_{false};
  std::vector<bridge::StateSample<kN>> buffer_;
  std::vector<Sample> history_;
  std::string jog_rejection_;
  std::optional<std::pair<std::int64_t, std::int64_t>> motion_;  // t_start, standby of the last
};

int main_impl(int argc, char** argv) {
  if (argc < 2) {
    throw std::invalid_argument("missing <name>");
  }
  const std::string name = argv[1];
  double speed = 0.1;
  double timeout_s = 60.0;
  std::vector<std::string> texts;
  for (int i = 2; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--speed" || a == "--timeout-s") {
      if (i + 1 >= argc) {
        throw std::invalid_argument(a + " needs a value");
      }
      const double v = number(argv[++i]);
      if (a == "--speed") {
        if (v < 1.0 || v > 100.0) {
          throw std::invalid_argument("--speed must be 1..100, got " + std::to_string(v));
        }
        speed = v / 100.0;
      } else if (v > 0.0) {
        timeout_s = v;
      } else {
        throw std::invalid_argument("--timeout-s must be > 0");
      }
    } else {
      texts.push_back(a);
    }
  }
  std::vector<Step> steps;
  steps.reserve(texts.size());
  for (const std::string& t : texts) {
    steps.push_back(parse_step(t, speed));
  }
  // waypoints / preview files are read above: a bad file is a bad argument (exit 2).
  cobot::common::Result<Client> opened = Client::open(name);
  if (!opened) {
    std::fprintf(stderr, "cobot_bridge_cli: %s\n", opened.reason().c_str());
    return kExitBadInput;
  }
  Session session(std::move(opened.value()), timeout_s);
  for (std::size_t k = 0; k < steps.size(); ++k) {
    const Outcome o = session.run(steps[k]);
    const std::string head = std::to_string(k + 1) + " " + steps[k].text + ": ";
    if (o.kind == Outcome::kOk) {
      std::printf("%sok%s%s\n", head.c_str(), o.text.empty() ? "" : " ", o.text.c_str());
      std::fflush(stdout);
      continue;
    }
    std::printf("%s%s: %s\n",
                head.c_str(),
                o.kind == Outcome::kRejected ? "rejected" : "failed",
                o.text.c_str());
    std::fflush(stdout);
    return kExitFailed;  // the session's destructor releases authority
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    return main_impl(argc, argv);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "cobot_bridge_cli: %s\n%s", e.what(), kUsage);
    return kExitBadInput;
  }
}
