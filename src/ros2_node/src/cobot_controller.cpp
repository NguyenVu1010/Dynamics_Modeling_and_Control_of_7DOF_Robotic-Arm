// cobot_ros2/CobotController: ros2_control controller around pipeline::ControlLoop (design 5.5,
// ADR-012). update() runs at the controller_manager rate: read the joints' position, velocity
// and effort state interfaces through ImperfectJointInterface (plant effects on), step the loop,
// write the effort command interfaces. on_activate reads the measured state and starts the
// trajectory on it (activation fails when the state cannot be read). State stamps count update
// cycles from the activation time: the `time` argument follows the ROS clock (/clock in
// simulation), which can stall for many cycles while the interfaces are still read every cycle,
// and the loop would hold on the stale stamp. `~/status` carries the loop mode.
//
// Black-box log (feature blackbox-log, ADR-020): the loop records into a Recorder of two buffers
// of cobot.log.capacity_s each (kOverwriteOldest) in the shared-memory region
// "/cobot_log_<node name>" (ShmStorage, mlocked with cobot.log.lock_memory), which outlives a crash
// of the controller or of Gazebo (`cobot_logdump <node name>` exports it). on_activate first
// recovers the unexported rows of an old region to "<log_path>.recovered-<YYYYmmdd-HHMMSS>.csv"
// and removes it, then creates the region and starts the Run CSV at `log_path` (default:
// results/<run-id>/run.csv under the working directory) with its header. `~/dump_log`
// (std_srvs/Trigger) appends the rows not yet exported to it: while active by a buffer swap (up to
// 200 ms) and the retired buffer, while inactive directly. The ROS parameter `log_stream_ms` > 0
// does the same every log_stream_ms on an exporter thread (one export at a time). Deactivation,
// cleanup and the destructor export the rest; the region is never removed here.
// cobot.log.mode off: no region and no Run CSV.
//
// Imperfections on the Gazebo path (ADR-011, feature gazebo-realism): sensor and actuator
// effects apply through ImperfectJointInterface; friction is applied by the physics engine
// (sim.launch.py writes the profile's friction into the joints' <dynamics>), so the interface
// gets the profile with friction zeroed; model mismatch is emulated by giving the controller
// perturb_model(nominal) instead of perturbing the plant (same size of error, opposite sign);
// payload_kg (with its offset and radius) cannot be emulated that way and is rejected; timing
// jitter has no meaning on the sim clock and is ignored with a warning. `seed` >= 0 overrides
// the profile's seed.
//
// Log channels (feature log-channels): each step gets RosJointInterface's PlantTruth: qt, qdt are
// the raw Gazebo reading before the decorator, taum the last command written to Gazebo (the
// decorator's output, motor lag included); qddt, qmt and tauf are `nan`.
//
// `command_source` ("" = the config file's, "config" or "external") picks the trajectory source.
// External (feature waypoint-ros): the loop waits in standby and WaypointServer offers
// ~/follow_waypoints, ~/preview_waypoints, ~/reset and the latched ~/reference_path; update()
// hands its commands to the loop, accumulates the goal metrics from each step's record and
// publishes them in a LoopSnapshot (waypoint_server.hpp).
//
// Studio bridge (feature controller-bridge, ADR-022): the ROS parameter `log_mode` ("" = the
// config file's cobot.log.mode, else off, recorder or recorder_live) is applied like
// command_source. With recorder_live, on_activate serves the shared-memory region
// "/cobot_<node name>" (ServerEndpoint, kept until cleanup): update() calls
// LoopPort::before_step / after_step around the step (commands, jogs, state samples) and a
// bridge thread runs CommandService::poll every 5 ms (authority, heartbeat watchdog, validation,
// segment export into <tmp>/cobot_<node name>, apply config). Apply config builds a new runtime
// (build_runtime + a ControlLoop over the same Recorder) on the bridge thread and hands it to
// update() through an atomic pointer; update() swaps it in at the next step only in standby and
// starts it on that step's state; the old runtime goes back through a second atomic and is
// destroyed on the bridge thread. While the studio holds authority the RViz panel's goals and
// ~/reset are refused ("studio is in control"). on_deactivate stops the bridge thread and clears
// the owner; cleanup and the destructor remove the region.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/logging.hpp>

#include "cobot/bridge/command_service.hpp"
#include "cobot/bridge/endpoint.hpp"
#include "cobot/bridge/loop_port.hpp"
#include "cobot/common/logger.hpp"
#include "cobot/common/types.hpp"
#include "cobot/imperfect/imperfect_joint_interface.hpp"
#include "cobot/imperfect/perturb.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/control_loop.hpp"
#include "cobot/pipeline/recorder.hpp"
#include "cobot/pipeline/recorder_shm.hpp"

#include "waypoint_server.hpp"
#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rcutils/logging.h>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace cobot_ros2 {
namespace {

namespace ci = controller_interface;
using ci::CallbackReturn;
using hardware_interface::LoanedCommandInterface;
using hardware_interface::LoanedStateInterface;

constexpr double kStatusRateHz = 10.0;                  // ~/status messages per second
constexpr std::chrono::milliseconds kSwapTimeout{200};  // longer: the export fails
constexpr std::chrono::milliseconds kSwapPoll{1};
constexpr std::chrono::milliseconds kBridgePoll{5};       // CommandService::poll period
constexpr std::chrono::milliseconds kApplyTimeout{1000};  // update() takes a new runtime
enum SwapResult : int { kSwapPending, kSwapped, kSwapRefused };

// Core logger -> rcutils, the sink behind rclcpp logging.
class RosLogger final : public cobot::common::Logger {
 public:
  void log(cobot::common::LogLevel level, std::string_view msg) override {
    rcutils_log(
        nullptr, severity(level), "cobot", "%.*s", static_cast<int>(msg.size()), msg.data());
  }

 private:
  static int severity(cobot::common::LogLevel level) {
    switch (level) {
      case cobot::common::LogLevel::kDebug:
        return RCUTILS_LOG_SEVERITY_DEBUG;
      case cobot::common::LogLevel::kInfo:
        return RCUTILS_LOG_SEVERITY_INFO;
      case cobot::common::LogLevel::kWarn:
        return RCUTILS_LOG_SEVERITY_WARN;
      case cobot::common::LogLevel::kError:
        break;
    }
    return RCUTILS_LOG_SEVERITY_ERROR;
  }
};
RosLogger g_logger;

// hal::JointInterface over the loaned interfaces (ADR-012, plug-in point 1). read() fails and
// leaves `out` untouched when a state interface has no value.
class RosJointInterface final : public cobot::hal::JointInterface<kN> {
 public:
  // Binds <joint>/{position,velocity,effort} states and <joint>/effort commands by name.
  bool bind(const std::vector<std::string>& joints,
            std::vector<LoanedStateInterface>& states,
            std::vector<LoanedCommandInterface>& commands) {
    bool ok = joints.size() == static_cast<std::size_t>(kN);
    for (int i = 0; ok && i < kN; ++i) {
      const std::string& joint = joints[static_cast<std::size_t>(i)];
      q_[i] = find(states, joint, "position");
      qd_[i] = find(states, joint, "velocity");
      tau_[i] = find(states, joint, "effort");
      cmd_[i] = find(commands, joint, "effort");
      ok = q_[i] != nullptr && qd_[i] != nullptr && tau_[i] != nullptr && cmd_[i] != nullptr;
    }
    return ok;
  }
  void set_stamp(cobot::common::Timestamp t) { stamp_ = t; }
  // Plant truth on the Gazebo path (feature log-channels): q, qd of the last raw read (before the
  // decorator) and, as tau_motor, the last command written to Gazebo (the decorator's output,
  // applied over the interval ending now); qdd, q_motor and tau_friction stay NaN (friction is
  // in the physics engine).
  const cobot::pipeline::PlantTruth<kN>& truth() const { return truth_; }
  // Back to unknown (all NaN), so a re-activation never logs the previous activation's state.
  void reset_truth() { truth_ = cobot::pipeline::PlantTruth<kN>{}; }

  cobot::common::Status read(cobot::hal::JointState<kN>& out) override {
    cobot::hal::JointState<kN> s;
    s.t = stamp_;
    s.tau_valid = true;
    for (int i = 0; i < kN; ++i) {
      const std::optional<double> q = q_[i]->get_optional();
      const std::optional<double> qd = qd_[i]->get_optional();
      const std::optional<double> tau = tau_[i]->get_optional();
      if (!q || !qd || !tau) {
        // No reading: the plant state is unknown, not stale.
        truth_.q.setConstant(cobot::pipeline::kUnknownTruth);
        truth_.qd.setConstant(cobot::pipeline::kUnknownTruth);
        return cobot::common::Status::error("ros2_control: state interface not readable");
      }
      s.q(i) = *q;
      s.qd(i) = *qd;
      s.tau_measured(i) = *tau;
    }
    out = s;
    truth_.q = s.q;
    truth_.qd = s.qd;
    return cobot::common::Status::ok();
  }

  cobot::common::Status write(const cobot::hal::JointCommand<kN>& cmd) override {
    bool ok = true;
    for (int i = 0; i < kN; ++i) {
      ok = cmd_[i]->set_value(cmd.tau(i)) && ok;
    }
    if (ok) {
      truth_.tau_motor = cmd.tau;
    } else {
      // Gazebo may not apply what was asked.
      truth_.tau_motor.setConstant(cobot::pipeline::kUnknownTruth);
    }
    return ok ? cobot::common::Status::ok()
              : cobot::common::Status::error("ros2_control: command interface not writable");
  }

 private:
  template <typename Loaned>
  static Loaned* find(std::vector<Loaned>& list, const std::string& joint, const char* name) {
    const auto it = std::find_if(list.begin(), list.end(), [&](const Loaned& item) {
      return item.get_prefix_name() == joint && item.get_interface_name() == name;
    });
    return it == list.end() ? nullptr : &*it;
  }

  cobot::common::Timestamp stamp_;
  LoanedStateInterface* q_[kN] = {};
  LoanedStateInterface* qd_[kN] = {};
  LoanedStateInterface* tau_[kN] = {};
  LoanedCommandInterface* cmd_[kN] = {};
  cobot::pipeline::PlantTruth<kN> truth_;  // fixed size, updated by read() and write()
};

ci::InterfaceConfiguration individual(const std::vector<std::string>& joints,
                                      const std::vector<std::string>& names) {
  ci::InterfaceConfiguration config{ci::interface_configuration_type::INDIVIDUAL, {}};
  for (const std::string& joint : joints) {
    for (const std::string& name : names) {
      config.names.push_back(joint);
      config.names.back().append("/").append(name);
    }
  }
  return config;
}

std::string run_id() {
  const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm tm{};
  localtime_r(&now, &tm);
  char buf[32] = {};
  std::strftime(buf, sizeof(buf), "gazebo-%Y%m%d-%H%M%S", &tm);
  return buf;
}

// What on_configure builds and "apply config" replaces (loop: apply config only).
struct Runtime {
  cobot::pipeline::Config<kN> config;
  std::unique_ptr<cobot::model::Arm7> model;  // the controller's (perturbed) model
  std::unique_ptr<cobot::imperfect::ImperfectJointInterface<kN>> io;
  cobot::common::VecN<kN> tau_max = cobot::common::VecN<kN>::Ones();
  std::uint64_t seed = 0;
  std::unique_ptr<cobot::pipeline::ControlLoop<kN>> loop;
};

}  // namespace

class CobotController final : public ci::ControllerInterface {
 public:
  ~CobotController() override {
    remove_bridge();
    close_log();
  }

  ci::InterfaceConfiguration command_interface_configuration() const override {
    return individual(joints_, {"effort"});
  }
  ci::InterfaceConfiguration state_interface_configuration() const override {
    return individual(joints_, {"position", "velocity", "effort"});
  }

  CallbackReturn on_init() override {
    try {
      cobot::common::set_logger(&g_logger);
      const std::string share = ament_index_cpp::get_package_share_directory("cobot_ros2");
      auto_declare<std::string>("config_path", share + "/config/default.yaml");
      auto_declare<std::string>("imperfections_path",
                                share + "/config/imperfections/realistic.yaml");
      auto_declare<std::int64_t>("seed", -1);  // < 0: the profile's seed
      auto_declare<std::vector<std::string>>("joints", {});
      auto_declare<std::string>("log_path", "");        // empty: <cwd>/results/<run-id>/run.csv
      auto_declare<std::string>("command_source", "");  // empty: cobot.command_source
      auto_declare<std::string>("log_mode", "");        // empty: cobot.log.mode
      auto_declare<std::int64_t>("log_stream_ms", 0);   // > 0: export every log_stream_ms
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("cobot"), "on_init: %s", e.what());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
    const rclcpp::Logger log = get_node()->get_logger();
    joints_ = get_node()->get_parameter("joints").as_string_array();
    log_path_ = get_node()->get_parameter("log_path").as_string();
    stream_ms_ = get_node()->get_parameter("log_stream_ms").as_int();
    const std::string config_path = get_node()->get_parameter("config_path").as_string();
    const std::string imp_path = get_node()->get_parameter("imperfections_path").as_string();
    const std::int64_t seed = get_node()->get_parameter("seed").as_int();
    if (joints_.size() != static_cast<std::size_t>(kN)) {
      RCLCPP_ERROR(log, "joints: expected %d names, got %zu", kN, joints_.size());
      return CallbackReturn::ERROR;
    }
    cobot::common::Result<Runtime> built = build_runtime(config_path, imp_path, seed);
    if (!built) {
      RCLCPP_ERROR(log, "%s", built.reason().c_str());
      return CallbackReturn::ERROR;
    }
    Runtime rt = std::move(built.value());
    const cobot::pipeline::Config<kN>& c = rt.config;
    server_.reset();
    link_.reset();
    close_log();
    if (c.log_mode == cobot::pipeline::LogMode::kOff) {
      RCLCPP_INFO(log, "%s: cobot.log.mode off, no Run CSV", config_path.c_str());
    }
    try {
      if (!dump_srv_) {
        dump_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
            "~/dump_log",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>& /*request*/,
                   const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
              const cobot::common::Result<cobot::pipeline::DumpResult> dumped = export_log();
              response->success = dumped.is_ok();
              response->message = dumped ? describe(dumped.value()) : dumped.reason();
            });
      }
      model_ = std::move(rt.model);
      const cobot::model::Arm7& model = *model_;
      config_ = c;
      tau_max_ = rt.tau_max;
      runs_ = RunAccumulator(tau_max_);
      if (c.command_source == cobot::pipeline::CommandSource::kExternal) {
        link_ = std::make_unique<LoopLink>();
        server_ = std::make_unique<WaypointServer>(get_node(), *link_, model, c);
      }
      io_ = std::move(rt.io);
      status_pub_ = get_node()->create_publisher<std_msgs::msg::String>(
          "~/status", rclcpp::SystemDefaultsQoS());
      status_rt_ = std::make_unique<StatusPublisher>(status_pub_);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(log, "%s: %s", imp_path.c_str(), e.what());
      return CallbackReturn::ERROR;
    }
    config_path_ = config_path;
    imp_path_ = imp_path;
    seed_ = rt.seed;
    period_ns_ = std::llround(1e9 / c.control_rate_hz);
    status_every_ = std::max<std::int64_t>(1, std::llround(c.control_rate_hz / kStatusRateHz));
    RCLCPP_INFO(log,
                "configured: config %s, imperfections %s, seed %llu",
                config_path.c_str(),
                imp_path.c_str(),
                static_cast<unsigned long long>(rt.seed));
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
    const rclcpp::Logger log = get_node()->get_logger();
    if (!ros_io_.bind(joints_, state_interfaces_, command_interfaces_)) {
      RCLCPP_ERROR(log, "missing position/velocity/effort state or effort command interface");
      return CallbackReturn::ERROR;
    }
    // Start on the measured state, stamped with the activation time; update() counts cycles.
    t0_ns_ = get_node()->now().nanoseconds();
    cycle_ = 0;
    last_mode_ = cobot::pipeline::LoopMode::kIdle;  // start() enters standby again
    ros_io_.set_stamp(cobot::common::Timestamp{t0_ns_});
    ros_io_.reset_truth();
    const cobot::common::Status read = io_->read(state_);
    if (!read) {
      RCLCPP_ERROR(log, "initial state: %s", read.reason().c_str());
      return CallbackReturn::ERROR;
    }
    close_log();
    std::filesystem::path csv;
    if (config_.log_mode != cobot::pipeline::LogMode::kOff) {
      std::error_code ec;
      csv = log_path_.empty() ? std::filesystem::current_path(ec) / "results" / run_id() / "run.csv"
                              : std::filesystem::path(log_path_);
      std::filesystem::create_directories(csv.parent_path(), ec);
      if (const std::string why = open_log(csv); !why.empty()) {
        RCLCPP_ERROR(log, "black-box log: %s", why.c_str());
        return CallbackReturn::ERROR;
      }
    }
    try {
      loop_ = std::make_unique<cobot::pipeline::ControlLoop<kN>>(*model_, config_, recorder_.get());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(log, "%s", e.what());
      return CallbackReturn::ERROR;
    }
    const cobot::common::Status started = loop_->start(state_);
    if (!started) {
      RCLCPP_ERROR(log, "%s", started.reason().c_str());
      return CallbackReturn::ERROR;
    }
    if (recorder_) {
      recording_.store(true, std::memory_order_release);
      if (stream_ms_ > 0) {
        exporter_stop_ = false;
        exporter_ = std::thread([this] { stream_log(); });
      }
    }
    if (config_.log_mode == cobot::pipeline::LogMode::kRecorderLive) {
      if (const std::string why = start_bridge(); !why.empty()) {
        RCLCPP_ERROR(log, "studio bridge: %s", why.c_str());
        return CallbackReturn::ERROR;
      }
    }
    if (link_) {
      // A command or reset left in the link while inactive belongs to an aborted goal or a
      // failed request: skip it instead of running it on the new start.
      const WaypointCommand* pending = link_->command.readFromNonRT();
      if (pending != nullptr) {
        loaded_id_ = pending->id;
      }
      reset_id_ = link_->reset_requested.load(std::memory_order_acquire);
      link_->stop_requested.store(false, std::memory_order_release);
      link_->active.store(true, std::memory_order_release);
    }
    RCLCPP_INFO(log, "active, writing %s", recorder_ ? csv.c_str() : "no Run CSV");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
    for (LoanedCommandInterface& cmd : command_interfaces_) {
      static_cast<void>(cmd.set_value(0.0));
    }
    if (link_) {
      // update() stops: goals are rejected from now on and the server aborts a running one.
      link_->active.store(false, std::memory_order_release);
      const std::lock_guard<std::mutex> lock(link_->mutex);
      link_->snapshot.mode = cobot::pipeline::LoopMode::kIdle;
    }
    stop_bridge();
    stop_exporter();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) override {
    remove_bridge();
    close_log();
    return CallbackReturn::SUCCESS;
  }

  ci::return_type update(const rclcpp::Time& /*time*/,
                         const rclcpp::Duration& /*period*/) override {
    ros_io_.set_stamp(cobot::common::Timestamp{t0_ns_ + cycle_ * period_ns_});
    const bool swapped = take_runtime();
    // A failed read keeps the previous state and stamp: the loop's freshness check holds.
    static_cast<void>(io_->read(state_));
    if (swapped) {
      static_cast<void>(loop_->start(state_));
    }
    if (link_) {
      take_commands();
    }
    if (port_) {
      port_->before_step(*loop_, state_);
    }
    static_cast<void>(loop_->step(state_, cmd_, ros_io_.truth()));
    static_cast<void>(io_->write(cmd_));
    if (port_) {
      port_->after_step(*loop_);
    }
    if (link_) {
      publish_snapshot();
    }
    if (cycle_++ % status_every_ == 0) {
      status_msg_.data = cobot::pipeline::to_string(loop_->mode());  // short: no allocation
      static_cast<void>(status_rt_->try_publish(status_msg_));
    }
    return ci::return_type::OK;
  }

 private:
  using StatusPublisher = realtime_tools::RealtimePublisher<std_msgs::msg::String>;
  using Service = cobot::bridge::CommandService<kN>;

  // on_configure and "apply config": config (command_source and log_mode parameters applied),
  // the controller's model, the I/O decorator over ros_io_ and tau_max; errors as text, warnings
  // logged.
  cobot::common::Result<Runtime> build_runtime(const std::string& config_path,
                                               const std::string& imp_path,
                                               std::int64_t seed) {
    using R = cobot::common::Result<Runtime>;
    const rclcpp::Logger log = get_node()->get_logger();
    const cobot::common::Result<cobot::pipeline::Config<kN>> config =
        cobot::pipeline::load_config<kN>(config_path);
    const cobot::common::Result<cobot::imperfect::ImperfectionConfig<kN>> imp =
        cobot::pipeline::load_imperfections<kN>(imp_path);
    if (!config || !imp) {
      return R::error(config ? imp.reason() : config.reason());
    }
    cobot::pipeline::Config<kN> c = config.value();
    const std::string source = get_node()->get_parameter("command_source").as_string();
    if (source == "config" || source == "external") {
      c.command_source = source == "external" ? cobot::pipeline::CommandSource::kExternal
                                              : cobot::pipeline::CommandSource::kConfig;
    } else if (!source.empty()) {
      return R::error("command_source: expected \"\", config or external, got " + source);
    }
    const std::string log_mode = get_node()->get_parameter("log_mode").as_string();
    if (log_mode == "off") {
      c.log_mode = cobot::pipeline::LogMode::kOff;
    } else if (log_mode == "recorder") {
      c.log_mode = cobot::pipeline::LogMode::kRecorder;
    } else if (log_mode == "recorder_live") {
      c.log_mode = cobot::pipeline::LogMode::kRecorderLive;
    } else if (!log_mode.empty()) {
      return R::error("log_mode: expected \"\", off, recorder or recorder_live, got " + log_mode);
    }
    cobot::imperfect::ImperfectionConfig<kN> profile = imp.value();
    if (seed >= 0) {
      profile.seed = static_cast<std::uint64_t>(seed);
    }
    if (profile.model.payload_kg != 0.0) {
      const cobot::common::Vec3& offset = profile.model.payload_offset_m;
      std::string extra;
      std::array<char, 160> text{};
      if (!offset.isZero(0.0)) {
        std::snprintf(text.data(),
                      text.size(),
                      " (with imperfections.model.payload_offset_m = [%g, %g, %g])",
                      offset.x(),
                      offset.y(),
                      offset.z());
        extra += text.data();
      }
      if (profile.model.payload_radius_m != 0.0) {
        std::snprintf(text.data(),
                      text.size(),
                      " (with imperfections.model.payload_radius_m = %g)",
                      profile.model.payload_radius_m);
        extra += text.data();
      }
      std::snprintf(text.data(), text.size(), "%g", profile.model.payload_kg);
      return R::error(imp_path + ": imperfections.model.payload_kg = " + text.data() + extra +
                      " is not supported on the Gazebo path (model perturbation cannot emulate a "
                      "payload); set it to 0");
    }
    if (profile.timing.period_jitter_std_us != 0.0) {
      RCLCPP_WARN(log,
                  "%s: imperfections.timing.period_jitter_std_us = %g ignored on the Gazebo path "
                  "(the sim clock has no jitter)",
                  imp_path.c_str(),
                  profile.timing.period_jitter_std_us);
    }
    if (profile.sensor.position_source == cobot::imperfect::PositionSource::kMotor) {
      RCLCPP_WARN(log,
                  "%s: imperfections.sensor.position_source = motor has no motor encoder on the "
                  "Gazebo path (rigid joints); the link position is used",
                  imp_path.c_str());
    }
    if (profile.transmission.stiffness_k3_nm_rad.maxCoeff() > 0.0) {
      RCLCPP_WARN(log,
                  "%s: imperfections.transmission ignored on the Gazebo path (bench only)",
                  imp_path.c_str());
    }
    profile.friction = cobot::imperfect::FrictionConfig<kN>{};  // in the physics (<dynamics>)
    const cobot::common::Result<cobot::model::UrdfChain> chain =
        cobot::model::parse_urdf_chain(c.urdf_path, c.base_link, c.tip_link);
    const cobot::common::Result<cobot::model::Arm7> arm =
        chain ? cobot::model::make_model<kN>(chain.value(), c.gravity_b)
              : cobot::common::Result<cobot::model::Arm7>::error(chain.reason());
    if (!arm) {
      return R::error(c.urdf_path.string() + ": " + arm.reason());
    }
    if (std::llround(c.control_rate_hz) != static_cast<long long>(get_update_rate())) {
      std::array<char, 64> rates{};
      std::snprintf(rates.data(),
                    rates.size(),
                    "%g differs from the update rate %u Hz",
                    c.control_rate_hz,
                    get_update_rate());
      return R::error(config_path + ": cobot.control_rate_hz " + rates.data());
    }
    Runtime rt;
    try {
      // Model mismatch: the controller's model is perturbed instead of the plant. Its tau_max
      // is the peak torque P of cobot.limits; Gazebo enforces the URDF effort M.
      rt.model = std::make_unique<cobot::model::Arm7>(cobot::imperfect::perturb_model(
          cobot::pipeline::controller_model<kN>(arm.value(), c), profile.model, profile.seed));
      for (int i = 0; i < kN; ++i) {
        rt.tau_max(i) = rt.model->joints[static_cast<std::size_t>(i)].tau_max;
      }
      rt.io = std::make_unique<cobot::imperfect::ImperfectJointInterface<kN>>(
          ros_io_, profile, 1.0 / c.control_rate_hz, true, c.limits.peak_torque_nm);
    } catch (const std::exception& e) {
      return R::error(imp_path + ": " + e.what());
    }
    rt.config = c;
    rt.seed = profile.seed;
    return R::ok(std::move(rt));
  }

  // update(), before reading: a runtime offered by "apply config" replaces the current one when
  // the loop is in standby (pointer swaps only); the old one goes back to the bridge thread.
  bool take_runtime() {
    Runtime* next = offered_.load(std::memory_order_acquire);
    if (next == nullptr || !offered_.compare_exchange_strong(next, nullptr)) {
      return false;
    }
    const bool swap = loop_->mode() == cobot::pipeline::LoopMode::kStandby;
    if (swap) {
      std::swap(config_, next->config);
      std::swap(model_, next->model);
      std::swap(io_, next->io);
      std::swap(tau_max_, next->tau_max);
      std::swap(loop_, next->loop);
      runs_ = RunAccumulator(tau_max_);
      retired_.store(next, std::memory_order_release);
    }
    swap_result_.store(swap ? kSwapped : kSwapRefused, std::memory_order_release);
    return swap;
  }

  cobot::bridge::SessionInfo<kN> session_info() const {
    cobot::bridge::SessionInfo<kN> s{};
    cobot::bridge::copy_text(s.config_path, cobot::bridge::kPathChars, config_path_);
    cobot::bridge::copy_text(s.imperfections_path, cobot::bridge::kPathChars, imp_path_);
    s.seed = seed_;
    s.command_source_external =
        config_.command_source == cobot::pipeline::CommandSource::kExternal ? 1U : 0U;
    s.control_rate_hz = config_.control_rate_hz;
    s.soft_margin_rad = config_.safety.soft_margin_rad;
    s.jog_margin_rad = config_.manual.jog_margin_rad;
    for (std::size_t i = 0; i < static_cast<std::size_t>(kN); ++i) {
      const auto& joint = model_->joints[i];
      s.q_min[i] = joint.q_min;
      s.q_max[i] = joint.q_max;
      s.qd_max[i] = joint.qd_max;
      s.tau_max[i] = joint.tau_max;
      s.rated[i] = config_.limits.rated_torque_nm(static_cast<Eigen::Index>(i));
    }
    s.T_tip_tcp = cobot::bridge::to_wire(config_.tcp_offset.T_tip_tcp);
    return s;
  }

  // on_activate with log.mode recorder_live: the region (created once, kept until cleanup), the
  // LoopPort, the CommandService and the bridge thread. The reason of a failure, empty on success.
  std::string start_bridge() {
    const std::string name = get_node()->get_name();  // region /cobot_<node name>
    const cobot::bridge::SessionInfo<kN> session = session_info();
    if (!endpoint_) {
      auto created = cobot::bridge::ServerEndpoint<kN>::create(name, session);
      if (!created) {
        return created.reason();
      }
      endpoint_.emplace(std::move(created.value()));
      endpoint_name_ = name;
    } else {
      endpoint_->set_session(session);
    }
    port_ = std::make_unique<cobot::bridge::LoopPort<kN>>(&*endpoint_);
    Service::Hooks hooks;
    hooks.export_log = [this]() -> cobot::common::Result<Service::ExportInfo> {
      using R = cobot::common::Result<Service::ExportInfo>;
      const cobot::common::Result<cobot::pipeline::DumpResult> dumped = export_log();
      if (!dumped) {
        return R::error(dumped.reason());
      }
      std::error_code ec;
      const std::uintmax_t size = std::filesystem::file_size(csv_path_, ec);
      if (ec) {
        return R::error(csv_path_.string() + ": " + ec.message());
      }
      return R::ok(Service::ExportInfo{csv_path_, static_cast<std::uint64_t>(size)});
    };
    hooks.apply_config = [this](const std::filesystem::path& config,
                                const std::filesystem::path& imperfections) {
      return apply_runtime(config, imperfections);
    };
    hooks.seed = [this] { return seed_; };
    std::error_code ec;
    service_ =
        std::make_unique<Service>(*endpoint_,
                                  *port_,
                                  *model_,
                                  config_,
                                  session,
                                  std::filesystem::temp_directory_path(ec) / ("cobot_" + name),
                                  std::move(hooks));
    bridge_stop_.store(false);
    bridge_thread_ = std::thread([this] {
      while (!bridge_stop_.load()) {
        service_->poll(cobot::bridge::monotonic_ns());
        if (link_) {
          link_->studio_in_control.store(service_->studio_in_control(), std::memory_order_release);
        }
        std::this_thread::sleep_for(kBridgePoll);
      }
    });
    return {};
  }

  // Bridge thread ("apply config"): a new runtime over the same Recorder, swapped in by update().
  cobot::common::Result<std::pair<cobot::model::Arm7, cobot::pipeline::Config<kN>>> apply_runtime(
      const std::filesystem::path& config, const std::filesystem::path& imperfections) {
    using R = cobot::common::Result<std::pair<cobot::model::Arm7, cobot::pipeline::Config<kN>>>;
    const std::int64_t seed = get_node()->get_parameter("seed").as_int();
    cobot::common::Result<Runtime> built =
        build_runtime(config.string(), imperfections.string(), seed);
    if (!built) {
      return R::error(built.reason());
    }
    // A config-mode loop would plan and run the file's trajectory from start() on the control
    // thread right after "applied"; the studio's runtime is always external.
    if (built.value().config.command_source != cobot::pipeline::CommandSource::kExternal) {
      return R::error("apply config: command_source must be external");
    }
    auto next = std::make_unique<Runtime>(std::move(built.value()));
    try {
      next->loop = std::make_unique<cobot::pipeline::ControlLoop<kN>>(
          *next->model, next->config, recorder_.get());
    } catch (const std::exception& e) {
      return R::error(e.what());
    }
    std::pair<cobot::model::Arm7, cobot::pipeline::Config<kN>> applied{*next->model, next->config};
    const std::uint64_t seed_used = next->seed;
    // Owned by update() once it swapped it in, else taken back here.
    Runtime* const offered = next.release();
    swap_result_.store(kSwapPending, std::memory_order_release);
    offered_.store(offered, std::memory_order_release);
    const auto deadline = std::chrono::steady_clock::now() + kApplyTimeout;
    while (swap_result_.load(std::memory_order_acquire) == kSwapPending) {
      if (std::chrono::steady_clock::now() > deadline) {
        Runtime* mine = offered;
        if (offered_.compare_exchange_strong(mine, nullptr)) {
          const std::unique_ptr<Runtime> back(offered);
          return R::error("apply config: update() did not take the new runtime (inactive?)");
        }
      }
      std::this_thread::sleep_for(kSwapPoll);
    }
    if (swap_result_.load(std::memory_order_acquire) == kSwapRefused) {
      const std::unique_ptr<Runtime> back(offered);
      return R::error("apply config: not in standby");
    }
    const std::unique_ptr<Runtime> old(retired_.exchange(nullptr));  // destroyed here
    config_path_ = config.string();
    imp_path_ = imperfections.string();
    seed_ = seed_used;
    if (server_) {
      server_->set_model(applied.first, applied.second);
    }
    return R::ok(std::move(applied));
  }

  // on_deactivate: the bridge thread stops and the owner is cleared (the region stays).
  void stop_bridge() {
    bridge_stop_.store(true);
    if (bridge_thread_.joinable()) {
      bridge_thread_.join();
    }
    if (endpoint_) {
      endpoint_->clear_owner();
    }
    if (link_) {
      link_->studio_in_control.store(false, std::memory_order_release);
    }
  }

  // on_cleanup and the destructor: the bridge goes and its region is removed.
  void remove_bridge() {
    stop_bridge();
    service_.reset();
    port_.reset();
    const std::unique_ptr<Runtime> old(retired_.exchange(nullptr));
    if (endpoint_) {
      endpoint_.reset();
      static_cast<void>(cobot::bridge::ServerEndpoint<kN>::remove(endpoint_name_));
    }
  }

  // External mode, before step(): the latest goal trajectory (once per id), reset and stop
  // requests. Nothing here allocates unless load() or reset() rejects (they build the message).
  void take_commands() {
    const WaypointCommand* command = link_->command.readFromRT();
    if (command != nullptr && command->id != loaded_id_ && command->trajectory) {
      loaded_id_ = command->id;
      load_status_ = loop_->load(*command->trajectory);
    }
    const std::uint64_t reset = link_->reset_requested.load(std::memory_order_acquire);
    if (reset != reset_id_) {
      reset_id_ = reset;
      reset_status_ = loop_->reset(state_);
    }
    if (link_->stop_requested.exchange(false, std::memory_order_acq_rel)) {
      loop_->request_stop();
    }
  }

  // External mode, after step(): the goal metrics of this step, then the snapshot, skipped for
  // this cycle when the lock is busy (the metrics are cumulative, the next cycle catches up).
  void publish_snapshot() {
    runs_.add(loop_->last_record());
    const cobot::pipeline::LoopMode mode = loop_->mode();
    if (mode == cobot::pipeline::LoopMode::kStandby && last_mode_ != mode) {
      standby_q_ = state_.q;
      standby_reference_ = loop_->reference_pose();
    }
    last_mode_ = mode;
    const std::unique_lock<std::mutex> lock(link_->mutex, std::try_to_lock);
    if (!lock.owns_lock()) {
      return;
    }
    LoopSnapshot& s = link_->snapshot;
    s.mode = mode;
    s.standby_seen = s.standby_seen || mode == cobot::pipeline::LoopMode::kStandby;
    s.standby_q = standby_q_;
    s.standby_reference = standby_reference_;
    s.segment = loop_->segment();
    s.t_s = loop_->trajectory_time_s();
    s.stats = runs_.stats();
    if (s.loaded_id != loaded_id_) {
      s.loaded_id = loaded_id_;
      s.load_ok = load_status_.is_ok();
      s.load_message = load_status_.reason();  // allocates only for a rejection
    }
    if (s.reset_id != reset_id_) {
      s.reset_id = reset_id_;
      s.reset_ok = reset_status_.is_ok();
      s.reset_message = reset_status_.reason();
    }
  }

  // on_activate: recovers an old region, creates the region and the Recorder, starts the Run CSV
  // with its header. The reason of a failure, empty on success.
  std::string open_log(const std::filesystem::path& csv) {
    const rclcpp::Logger log = get_node()->get_logger();
    const std::string name = get_node()->get_name();
    const auto recovered = cobot::pipeline::recover_shm_log<kN>(name, csv);
    if (!recovered) {
      RCLCPP_WARN(log, "old black-box log not recovered (%s); removed", recovered.reason().c_str());
      static_cast<void>(cobot::pipeline::ShmStorage<kN>::remove(name));
    } else if (recovered.value()) {
      RCLCPP_WARN(log,
                  "rows of a previous run never exported: recovered to %s",
                  recovered.value()->c_str());
    }
    const long long rows = std::llround(config_.log_capacity_s * config_.control_rate_hz);
    auto storage = cobot::pipeline::ShmStorage<kN>::create(
        name, static_cast<std::size_t>(std::max(rows, 1LL)), config_.log_lock_memory);
    if (!storage) {
      return storage.reason();
    }
    if (!storage.value().lock_error().empty()) {
      RCLCPP_WARN(log, "%s", storage.value().lock_error().c_str());
    }
    std::ofstream out(csv, std::ios::binary | std::ios::trunc);
    cobot::pipeline::write_csv_header<kN>(out);
    if (!out.flush()) {
      return "cannot write " + csv.string();
    }
    const std::lock_guard<std::mutex> lock(export_mutex_);
    storage_ = std::make_unique<cobot::pipeline::ShmStorage<kN>>(std::move(storage.value()));
    recorder_ = std::make_unique<cobot::pipeline::Recorder<kN>>(
        storage_->memory(), cobot::pipeline::FullPolicy::kOverwriteOldest);
    csv_path_ = csv;
    return {};
  }

  // ~/dump_log, the stream thread and the final exports: appends the rows not yet exported to the
  // Run CSV, one export at a time. While update() records: a buffer swap (up to kSwapTimeout),
  // the retired buffer, release; otherwise every unexported row.
  cobot::common::Result<cobot::pipeline::DumpResult> export_log() {
    using R = cobot::common::Result<cobot::pipeline::DumpResult>;
    const std::lock_guard<std::mutex> lock(export_mutex_);
    if (!storage_ || !recorder_) {
      return R::error("no black-box log (cobot.log.mode off or not activated)");
    }
    if (!recording_.load(std::memory_order_acquire)) {
      return report(
          storage_->export_unexported(csv_path_, cobot::pipeline::ShmExport::kAll, false));
    }
    static_cast<void>(recorder_->request_swap());  // false while one is pending or held
    const auto deadline = std::chrono::steady_clock::now() + kSwapTimeout;
    while (!recorder_->swap_done() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(kSwapPoll);
    }
    if (!recorder_->swap_done()) {
      return R::error("no buffer swap within 200 ms (update() not running?)");
    }
    R dumped = storage_->export_unexported(csv_path_, cobot::pipeline::ShmExport::kRetired, false);
    if (dumped) {
      recorder_->release_retired();
    }
    return report(std::move(dumped));
  }

  // "<csv> <rows> rows" plus ", <n> rows lost (older than log.capacity_s)".
  static std::string describe(const cobot::pipeline::DumpResult& r) {
    std::string text = r.csv.string() + " " + std::to_string(r.rows) + " rows";
    if (r.lost > 0) {
      text += ", " + std::to_string(r.lost) + " rows lost (older than log.capacity_s)";
    }
    return text;
  }

  static cobot::common::Result<cobot::pipeline::DumpResult> report(
      cobot::common::Result<cobot::pipeline::DumpResult> r) {
    if (r && r.value().lost > 0) {
      RCLCPP_WARN(rclcpp::get_logger("cobot"), "black-box log: %s", describe(r.value()).c_str());
    }
    return r;
  }

  static void warn_if_failed(const cobot::common::Result<cobot::pipeline::DumpResult>& r) {
    if (!r) {
      RCLCPP_WARN(rclcpp::get_logger("cobot"), "black-box log: %s", r.reason().c_str());
    }
  }

  // Exporter thread (log_stream_ms > 0): export_log() every stream_ms_ until stopped.
  void stream_log() {
    std::unique_lock<std::mutex> lock(stream_mutex_);
    while (!stream_cv_.wait_for(
        lock, std::chrono::milliseconds(stream_ms_), [this] { return exporter_stop_; })) {
      lock.unlock();
      warn_if_failed(export_log());
      lock.lock();
    }
  }

  // update() no longer runs: the exporter stops and the rest is exported (the region stays).
  void stop_exporter() {
    if (exporter_.joinable()) {
      {
        const std::lock_guard<std::mutex> lock(stream_mutex_);
        exporter_stop_ = true;
      }
      stream_cv_.notify_all();
      exporter_.join();
    }
    recording_.store(false, std::memory_order_release);
    if (storage_) {
      warn_if_failed(export_log());
    }
  }

  // Final export, then the loop, the Recorder and the mapping go (the region is not removed).
  void close_log() {
    stop_exporter();
    loop_.reset();  // before the recorder it writes into
    const std::lock_guard<std::mutex> lock(export_mutex_);
    recorder_.reset();
    storage_.reset();
  }

  std::vector<std::string> joints_;
  std::string log_path_;
  std::int64_t stream_ms_ = 0;  // ROS parameter log_stream_ms
  cobot::pipeline::Config<kN> config_;
  std::unique_ptr<cobot::model::Arm7> model_;  // the controller's (perturbed) model
  // The loop records into recorder_ (null with log.mode off), over storage_ (the black-box
  // region); declared first so they outlive the loop. export_mutex_: one export at a time.
  std::unique_ptr<cobot::pipeline::ShmStorage<kN>> storage_;
  std::unique_ptr<cobot::pipeline::Recorder<kN>> recorder_;
  std::filesystem::path csv_path_;
  std::mutex export_mutex_;
  std::atomic<bool> recording_{false};  // between on_activate and on_deactivate
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr dump_srv_;
  std::unique_ptr<cobot::pipeline::ControlLoop<kN>> loop_;
  RosJointInterface ros_io_;
  std::unique_ptr<cobot::imperfect::ImperfectJointInterface<kN>> io_;
  cobot::hal::JointState<kN> state_;
  cobot::hal::JointCommand<kN> cmd_;
  std::int64_t cycle_ = 0;
  std::int64_t t0_ns_ = 0;         // activation time, the stamp of the start state
  std::int64_t period_ns_ = 1;     // control period
  std::int64_t status_every_ = 1;  // cycles between ~/status messages
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::unique_ptr<StatusPublisher> status_rt_;
  std_msgs::msg::String status_msg_;
  std::thread exporter_;
  std::mutex stream_mutex_;
  std::condition_variable stream_cv_;
  bool exporter_stop_ = false;  // under stream_mutex_
  cobot::common::VecN<kN> tau_max_ = cobot::common::VecN<kN>::Ones();
  // External mode (null otherwise); the server goes before the link it refers to.
  std::unique_ptr<LoopLink> link_;
  std::unique_ptr<WaypointServer> server_;
  RunAccumulator runs_{cobot::common::VecN<kN>::Ones()};
  std::uint64_t loaded_id_ = 0;
  std::uint64_t reset_id_ = 0;
  cobot::common::Status load_status_ = cobot::common::Status::ok();
  cobot::common::Status reset_status_ = cobot::common::Status::ok();
  cobot::pipeline::LoopMode last_mode_ = cobot::pipeline::LoopMode::kIdle;
  cobot::common::VecN<kN> standby_q_ = cobot::common::VecN<kN>::Zero();
  cobot::common::Pose standby_reference_;
  // Studio bridge (log.mode recorder_live). The thread goes before what it uses.
  std::string config_path_, imp_path_;  // of the running runtime (session info)
  std::uint64_t seed_ = 0;
  std::optional<cobot::bridge::ServerEndpoint<kN>> endpoint_;
  std::string endpoint_name_;
  std::unique_ptr<cobot::bridge::LoopPort<kN>> port_;
  std::unique_ptr<Service> service_;
  std::atomic<Runtime*> offered_{nullptr};  // bridge thread -> update()
  std::atomic<Runtime*> retired_{nullptr};  // update() -> bridge thread
  std::atomic<int> swap_result_{kSwapPending};
  std::atomic<bool> bridge_stop_{false};
  std::thread bridge_thread_;
};

}  // namespace cobot_ros2

PLUGINLIB_EXPORT_CLASS(cobot_ros2::CobotController, controller_interface::ControllerInterface)
