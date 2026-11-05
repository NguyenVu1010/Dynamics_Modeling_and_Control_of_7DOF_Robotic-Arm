#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <system_error>
#include <unistd.h>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/bridge/command_service.hpp"
#include "cobot/bridge/endpoint.hpp"
#include "cobot/bridge/loop_port.hpp"
#include "cobot/bridge/wire.hpp"
#include "cobot/common/result.hpp"
#include "cobot/common/types.hpp"
#include "cobot/dynamics/sim_plant.hpp"
#include "cobot/hal/joint_interface.hpp"
#include "cobot/kinematics/kinematics.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/control_loop.hpp"
#include "cobot/pipeline/log_record.hpp"
#include "cobot/pipeline/manual.hpp"
#include "cobot/pipeline/recorder.hpp"
#include "cobot/pipeline/sim_joint_interface.hpp"
#include "cobot/pipeline/waypoint_file.hpp"
#include "cobot/planning/waypoint_trajectory.hpp"

// Deterministic one-process host of the studio bridge (feature controller-bridge): the way
// cobot_bench serve and cobot_controller wire it, minus the threads.
//   control part, every step: SimPlant (config/default.yaml + ideal profile: the URDF model, no
//     imperfection) -> SimJointInterface -> LoopPort::before_step -> ControlLoop::step ->
//     LoopPort::after_step -> write -> advance;
//   bridge part: CommandService::poll(now) after every 5th step (the hosts poll every <= 5 ms);
//   fake clock: now = steps done x 1 ms, the stamp of row k is k ms (plant time);
//   clients: ClientEndpoint of this process; the harness beats the ones registered with
//     beat_with() every 50 steps (kHeartbeatPeriodNs) with the fake clock;
//   hooks: export_log appends the Recorder's rows to <out>/run.csv (one buffer, nothing records
//     while the hook runs, so active() + clear() is the exactly-once export of this thread);
//     with HarnessOptions::export_steps > 0 the control part first runs that many steps inside
//     the hook (no poll), as the control thread of a host keeps stepping while its bridge thread
//     exports; each call is kept as an ExportCall;
//     apply_config loads the files, checks control_rate_hz against the host's rate and builds a
//     new ControlLoop over the same Recorder that the next step start()s on its state and swaps
//     in (the plant keeps the start profile, like Gazebo physics).
namespace cobot::bridge::test {

namespace fs = std::filesystem;

constexpr int kN = 7;
using Vec = common::VecN<kN>;
using State = hal::JointState<kN>;
using JointCmd = hal::JointCommand<kN>;
using Record = pipeline::LogRecord<kN>;
using Loop = pipeline::ControlLoop<kN>;
using Service = CommandService<kN>;
using Client = ClientEndpoint<kN>;
using Status = LoopStatus<kN>;
using pipeline::LoopMode;

constexpr std::int64_t kStepNs = 1'000'000;  // 1 / control_rate_hz of default.yaml (1000 Hz)
constexpr std::int64_t kPollEverySteps = 5;  // CommandService::poll every 5 ms
constexpr std::int64_t kBeatEverySteps = kHeartbeatPeriodNs / kStepNs;  // 50
constexpr double kSpeed = 0.1;                                          // 10 %

inline fs::path repo_path(const std::string& rel) {
  return fs::path(COBOT_REPO_DIR) / rel;
}

inline fs::path default_config() {
  return repo_path("config/default.yaml");
}

inline fs::path ideal_profile() {
  return repo_path("config/imperfections/ideal.yaml");
}

struct Arm {
  model::Arm7 urdf;        // the plant's model
  model::Arm7 controller;  // pipeline::controller_model (tau_max = peak torque)
  pipeline::Config<kN> config;
};

inline common::Result<Arm> load_arm(const fs::path& yaml, pipeline::CommandSource source) {
  using Out = common::Result<Arm>;
  const auto cfg = pipeline::load_config<kN>(yaml);
  if (!cfg) {
    return Out::error(cfg.reason());
  }
  pipeline::Config<kN> config = cfg.value();
  config.command_source = source;
  const auto chain = model::parse_urdf_chain(config.urdf_path, config.base_link, config.tip_link);
  if (!chain) {
    return Out::error(chain.reason());
  }
  const auto urdf = model::make_model<kN>(chain.value(), config.gravity_b);
  if (!urdf) {
    return Out::error(urdf.reason());
  }
  return Out::ok(Arm{urdf.value(), pipeline::controller_model<kN>(urdf.value(), config), config});
}

inline common::Pose fk_tip(const model::Arm7& m, const Vec& q) {
  kinematics::FkState<kN> fk;
  kinematics::forward_kinematics(m, q, fk);
  return fk.T_b_e;
}

inline SessionInfo<kN> make_session(const Arm& arm,
                                    const fs::path& config,
                                    const fs::path& imperfections) {
  SessionInfo<kN> s{};
  s.generation = 1;
  copy_text(s.config_path, kPathChars, config.string());
  copy_text(s.imperfections_path, kPathChars, imperfections.string());
  s.seed = 1;
  s.command_source_external =
      arm.config.command_source == pipeline::CommandSource::kExternal ? 1U : 0U;
  s.control_rate_hz = arm.config.control_rate_hz;
  s.soft_margin_rad = arm.config.safety.soft_margin_rad;
  s.jog_margin_rad = arm.config.manual.jog_margin_rad;
  for (std::size_t i = 0; i < static_cast<std::size_t>(kN); ++i) {
    const auto& joint = arm.controller.joints[i];
    s.q_min[i] = joint.q_min;
    s.q_max[i] = joint.q_max;
    s.qd_max[i] = joint.qd_max;
    s.tau_max[i] = joint.tau_max;
    s.rated[i] = arm.config.limits.rated_torque_nm(static_cast<Eigen::Index>(i));
  }
  s.T_tip_tcp = to_wire(arm.config.tcp_offset.T_tip_tcp);
  return s;
}

inline std::string text_of(const char* chars, std::size_t size) {
  return {chars, strnlen(chars, size)};
}

inline std::string text_of(const Response& r) {
  return text_of(r.text, kTextChars);
}

inline LoopMode mode_of(const Status& st) {
  return static_cast<LoopMode>(st.mode);
}

inline std::uint8_t manual_stop_value(pipeline::ManualStop stop) {
  return static_cast<std::uint8_t>(stop);
}

inline Vec vec_of(const double (&values)[kN]) {
  Vec v;
  for (int i = 0; i < kN; ++i) {
    v(i) = values[i];
  }
  return v;
}

inline std::string read_file(const fs::path& path) {
  const std::ifstream in(path, std::ios::binary);
  std::ostringstream os;
  os << in.rdbuf();
  return os.str();
}

// Lines of a file without their '\n'.
inline std::vector<std::string> read_lines(const fs::path& path) {
  std::vector<std::string> lines;
  std::istringstream in(read_file(path));
  for (std::string line; std::getline(in, line);) {
    lines.push_back(line);
  }
  return lines;
}

// Leading integer (t_ns) of a CSV data line.
inline std::int64_t t_of_line(const std::string& line) {
  return std::stoll(line.substr(0, line.find(',')));
}

struct HarnessOptions {
  fs::path config = default_config();
  pipeline::CommandSource source = pipeline::CommandSource::kExternal;
  std::size_t log_rows = 20000;  // Recorder capacity (20 s at 1 kHz), one buffer, overwrite oldest
  int export_steps = 0;          // control steps run inside each export_log call, before the export
};

// One export_log call: the loop's status stamp when the hook was entered and when it returned,
// and the Run CSV size it returned.
struct ExportCall {
  std::int64_t t_entry_ns = 0;
  std::int64_t t_return_ns = 0;
  std::uint64_t size_after = 0;
};

class Harness {
 public:
  explicit Harness(const HarnessOptions& options = HarnessOptions{})
      : name_(next_name()), export_steps_(options.export_steps) {
    const auto arm = load_arm(options.config, options.source);
    if (!arm) {
      ADD_FAILURE() << "load_arm: " << arm.reason();
      return;
    }
    arm_ = arm.value();
    dir_ = fs::path(COBOT_TEST_OUTPUT_DIR) / "bridge_service" / name_;
    fs::remove_all(dir_);
    fs::create_directories(dir_ / "segments");
    run_csv_ = dir_ / "run.csv";
    const double period_s = 1.0 / arm_->config.control_rate_hz;
    plant_ = std::make_unique<dynamics::SimPlant<kN>>(arm_->urdf, arm_->config.home_q_rad);
    io_ = std::make_unique<pipeline::SimJointInterface<kN>>(*plant_, period_s);
    probe_ = std::make_unique<pipeline::SimTruthProbe<kN>>(*plant_);
    storage_ = std::make_unique<pipeline::HeapStorage<kN>>(options.log_rows, 1);
    recorder_ = std::make_unique<pipeline::Recorder<kN>>(storage_->memory(),
                                                         pipeline::FullPolicy::kOverwriteOldest);
    loop_ = std::make_unique<Loop>(arm_->controller, arm_->config, recorder_.get());
    session_ = make_session(*arm_, options.config, ideal_profile());
    auto server = ServerEndpoint<kN>::create(name_, session_);
    if (!server) {
      ADD_FAILURE() << "ServerEndpoint::create(" << name_ << "): " << server.reason();
      return;
    }
    server_.emplace(std::move(server).value());
    port_ = std::make_unique<LoopPort<kN>>(&*server_);
    Service::Hooks hooks;
    hooks.export_log = [this] { return export_log(); };
    hooks.apply_config = [this](const fs::path& config, const fs::path& imperfections) {
      return apply_config(config, imperfections);
    };
    service_ = std::make_unique<Service>(*server_,
                                         *port_,
                                         arm_->controller,
                                         arm_->config,
                                         session_,
                                         dir_ / "segments",
                                         std::move(hooks));
    State s;
    if (!io_->read(s).is_ok() || !loop_->start(s).is_ok()) {
      ADD_FAILURE() << "loop start failed";
      return;
    }
    ok_ = true;
  }

  ~Harness() {
    service_.reset();
    port_.reset();
    server_.reset();
    static_cast<void>(ServerEndpoint<kN>::remove(name_));
  }

  Harness(const Harness&) = delete;
  Harness& operator=(const Harness&) = delete;
  Harness(Harness&&) = delete;
  Harness& operator=(Harness&&) = delete;

  bool ok() const { return ok_; }

  // One control period. `guard(f)` runs f = before_step + step + after_step (the control-thread
  // part); the no-allocation test wraps it in its counters.
  template <typename Guard>
  void step(Guard&& guard) {
    control_step(guard);
    if (steps_ % kPollEverySteps == 0) {
      service_->poll(now_ns());
    }
  }

  void step() {
    step([](const auto& control) { control(); });
  }

  void steps(int n) {
    for (int k = 0; k < n; ++k) {
      step();
    }
  }

  // The control part of a step and the clients' beats, without the poll.
  template <typename Guard>
  void control_step(Guard&& guard) {
    State s;
    JointCmd cmd;
    EXPECT_TRUE(io_->read(s).is_ok());
    if (pending_) {
      swap_loop(s);
    }
    const pipeline::PlantTruth<kN>& truth = probe_->sample();
    guard([&] {
      port_->before_step(*loop_, s);
      static_cast<void>(loop_->step(s, cmd, truth));
      port_->after_step(*loop_);
    });
    EXPECT_TRUE(io_->write(cmd).is_ok());
    rows_.push_back(loop_->last_record());
    io_->advance();
    ++steps_;
    if (steps_ % kBeatEverySteps == 0) {
      for (Client* c : beaters_) {
        c->beat(now_ns());
      }
    }
  }

  // Steps until pred() or max_steps; the number of steps taken, -1 when pred never held.
  template <typename Pred>
  int run_until(Pred pred, int max_steps) {
    for (int k = 0; k < max_steps; ++k) {
      if (pred()) {
        return k;
      }
      step();
    }
    return pred() ? max_steps : -1;
  }

  // Steps (polling every 5) until the bridge answers command `id`; other responses go to
  // `others`. std::nullopt (and a test failure) after max_steps.
  std::optional<Response> await(Client& client,
                                std::uint64_t id,
                                int max_steps = 100,
                                std::vector<Response>* others = nullptr) {
    Response r{};
    for (int k = 0; k <= max_steps; ++k) {
      while (client.poll_response(r)) {
        if (r.command_id == id && r.type != CommandType::kJog) {
          return r;
        }
        if (others != nullptr) {
          others->push_back(r);
        }
      }
      if (k < max_steps) {
        step();
      }
    }
    ADD_FAILURE() << "no response to command " << id << " within " << max_steps << " steps";
    return std::nullopt;
  }

  Status status() const {
    Status st{};
    EXPECT_TRUE(port_->status(st)) << "LoopPort::status torn in a single-thread harness";
    return st;
  }

  // Index of the first row with t > after_ns satisfying pred, -1 if none.
  template <typename Pred>
  std::ptrdiff_t first_row(std::int64_t after_ns, Pred pred) const {
    for (std::size_t i = 0; i < rows_.size(); ++i) {
      if (rows_[i].t.ns > after_ns && pred(rows_[i])) {
        return static_cast<std::ptrdiff_t>(i);
      }
    }
    return -1;
  }

  void beat_with(Client* client) { beaters_.push_back(client); }
  void stop_beating() { beaters_.clear(); }

  std::int64_t now_ns() const { return steps_ * kStepNs; }
  const std::string& name() const { return name_; }
  const Arm& arm() const { return *arm_; }
  const std::vector<Record>& rows() const { return rows_; }
  const fs::path& run_csv() const { return run_csv_; }
  const fs::path& dir() const { return dir_; }
  ServerEndpoint<kN>& server() { return *server_; }
  LoopPort<kN>& port() { return *port_; }
  Service& service() { return *service_; }
  pipeline::Recorder<kN>& recorder() { return *recorder_; }
  const std::optional<Arm>& applied_arm() const { return applied_arm_; }
  // Row index of the first step of each swapped-in loop.
  const std::vector<std::size_t>& swap_rows() const { return swap_rows_; }
  int exports() const { return exports_; }
  const std::vector<ExportCall>& export_calls() const { return export_calls_; }
  int applies() const { return applies_; }

 private:
  static std::string next_name() {
    static int n = 0;
    return "test_" + std::to_string(::getpid()) + "_" + std::to_string(++n);
  }

  common::Result<Service::ExportInfo> export_log() {
    using Out = common::Result<Service::ExportInfo>;
    ++exports_;
    ExportCall call;
    call.t_entry_ns = status().t_ns;
    for (int k = 0; k < export_steps_; ++k) {
      control_step([](const auto& control) { control(); });
    }
    call.t_return_ns = status().t_ns;
    const common::Status st = pipeline::export_csv<kN>(run_csv_, recorder_->active(), true);
    if (!st) {
      return Out::error(st.reason());
    }
    recorder_->clear();
    std::error_code ec;
    const std::uintmax_t size = fs::file_size(run_csv_, ec);
    if (ec) {
      return Out::error(run_csv_.string() + ": " + ec.message());
    }
    call.size_after = static_cast<std::uint64_t>(size);
    export_calls_.push_back(call);
    return Out::ok(Service::ExportInfo{run_csv_, static_cast<std::uint64_t>(size)});
  }

  common::Result<std::pair<model::Arm7, pipeline::Config<kN>>> apply_config(
      const fs::path& config, const fs::path& imperfections) {
    using Out = common::Result<std::pair<model::Arm7, pipeline::Config<kN>>>;
    ++applies_;
    const auto imp = pipeline::load_imperfections<kN>(imperfections);
    if (!imp) {
      return Out::error(imp.reason());
    }
    const auto arm = load_arm(config, pipeline::CommandSource::kExternal);
    if (!arm) {
      return Out::error(arm.reason());
    }
    const double rate = arm.value().config.control_rate_hz;
    if (rate != arm_->config.control_rate_hz) {
      return Out::error("apply config: cobot.control_rate_hz " + std::to_string(rate) +
                        " differs from the host's rate " +
                        std::to_string(arm_->config.control_rate_hz));
    }
    pending_ = std::make_unique<Loop>(arm.value().controller, arm.value().config, recorder_.get());
    applied_arm_ = arm.value();
    return Out::ok({arm.value().controller, arm.value().config});
  }

  // At a step boundary: the new loop starts on this step's state; the old one is kept (destroyed
  // with the harness, never inside the control part).
  void swap_loop(const State& s) {
    EXPECT_TRUE(pending_->start(s).is_ok());
    retired_.push_back(std::move(loop_));
    loop_ = std::move(pending_);
    swap_rows_.push_back(rows_.size());
  }

  std::string name_;
  int export_steps_ = 0;
  bool ok_ = false;
  std::optional<Arm> arm_;
  std::optional<Arm> applied_arm_;
  fs::path dir_;
  fs::path run_csv_;
  SessionInfo<kN> session_{};
  std::unique_ptr<dynamics::SimPlant<kN>> plant_;
  std::unique_ptr<pipeline::SimJointInterface<kN>> io_;
  std::unique_ptr<pipeline::SimTruthProbe<kN>> probe_;
  std::unique_ptr<pipeline::HeapStorage<kN>> storage_;
  std::unique_ptr<pipeline::Recorder<kN>> recorder_;
  std::unique_ptr<Loop> loop_;
  std::unique_ptr<Loop> pending_;
  std::vector<std::unique_ptr<Loop>> retired_;
  std::optional<ServerEndpoint<kN>> server_;
  std::unique_ptr<LoopPort<kN>> port_;
  std::unique_ptr<Service> service_;
  std::vector<Record> rows_;
  std::vector<Client*> beaters_;
  std::vector<std::size_t> swap_rows_;
  std::int64_t steps_ = 0;
  int exports_ = 0;
  int applies_ = 0;
  std::vector<ExportCall> export_calls_;
};

inline std::optional<Client> open_client(const std::string& name) {
  auto c = Client::open(name);
  if (!c) {
    ADD_FAILURE() << "ClientEndpoint::open(" << name << "): " << c.reason();
    return std::nullopt;
  }
  return std::optional<Client>(std::move(c).value());
}

inline Command<kN> move_joint_command(const Vec& q_goal, double speed) {
  Command<kN> c{};
  c.type = CommandType::kMoveJoint;
  c.speed = speed;
  for (int i = 0; i < kN; ++i) {
    c.q_goal[i] = q_goal(i);
  }
  return c;
}

inline Command<kN> move_tcp_command(const common::Pose& T_b_tcp, double speed) {
  Command<kN> c{};
  c.type = CommandType::kMoveTcp;
  c.speed = speed;
  c.T_b_tcp = to_wire(T_b_tcp);
  return c;
}

// kRunWaypoints or kPreviewWaypoints of TCP waypoints.
inline Command<kN> waypoints_command(CommandType type,
                                     const std::vector<planning::Waypoint>& tcp_waypoints) {
  Command<kN> c{};
  c.type = type;
  c.waypoint_count = static_cast<std::uint32_t>(tcp_waypoints.size());
  for (std::size_t i = 0; i < tcp_waypoints.size(); ++i) {
    c.waypoints[i].T_b_tcp = to_wire(tcp_waypoints[i].T_b_goal);
    c.waypoints[i].duration_s = tcp_waypoints[i].duration_s;
    c.waypoints[i].stop = tcp_waypoints[i].stop ? 1U : 0U;
  }
  return c;
}

inline Command<kN> simple_command(CommandType type) {
  Command<kN> c{};
  c.type = type;
  return c;
}

inline Command<kN> export_command(std::int64_t t0_ns, std::int64_t t1_ns) {
  Command<kN> c{};
  c.type = CommandType::kExportSegment;
  c.t0_ns = t0_ns;
  c.t1_ns = t1_ns;
  return c;
}

inline Command<kN> apply_command(const fs::path& config, const fs::path& imperfections) {
  Command<kN> c{};
  c.type = CommandType::kApplyConfig;
  copy_text(c.path_a, kPathChars, config.string());
  copy_text(c.path_b, kPathChars, imperfections.string());
  return c;
}

// Joint jog request (kind 1); direction 0 releases.
inline JogRequest joint_jog(int joint, int direction, double speed) {
  JogRequest j{};
  j.kind = 1;
  j.joint = joint;
  j.direction = direction;
  j.speed = speed;
  return j;
}

// tests/e2e/waypoints/e2e_13.yaml as written (TCP poses: identity TcpOffset, as cobot_bridge_cli
// reads it).
inline std::vector<planning::Waypoint> e2e_13_waypoints() {
  const auto w = pipeline::load_waypoints(
      repo_path("tests/e2e/waypoints/e2e_13.yaml"), "base_link", pipeline::TcpOffset{});
  if (!w) {
    ADD_FAILURE() << "load_waypoints: " << w.reason();
    return {};
  }
  return w.value();
}

// The plan of manual-commands' first move_joint case: q1 + 0.5 rad, q2 - 0.25 rad from `q_ref`.
// Hand-checked duration (default.yaml at 10 %: qd_max 2.1 rad/s on joints 1 and 2 (URDF
// velocity), joint_speed_fraction 0.9, joint_a_max 15 and 7.5 rad/s^2):
//   s_v_max = min(0.1 * 0.9 * 2.1 / 0.5, 0.1 * 0.9 * 2.1 / 0.25) = min(0.378, 0.756) = 0.378 1/s
//   s_a_max = min(0.1 * 15 / 0.5, 0.1 * 7.5 / 0.25) = min(3.0, 3.0) = 3.0 1/s^2
//   s_v_max^2 / s_a_max = 0.142884 / 3 = 0.047628 < 1: trapezoid,
//   duration = 1 / 0.378 + 0.378 / 3 = 2.645503 + 0.126 = 2.771503 s.
constexpr double kMoveJointDurationS = 2.771503;
inline Vec move_joint_goal(const Vec& q_ref) {
  Vec q = q_ref;
  q(0) += 0.5;
  q(1) -= 0.25;
  return q;
}

}  // namespace cobot::bridge::test
