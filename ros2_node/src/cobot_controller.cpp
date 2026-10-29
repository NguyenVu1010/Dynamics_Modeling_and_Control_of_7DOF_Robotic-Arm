// cobot_ros2/CobotController: ros2_control controller around pipeline::ControlLoop (design 5.5,
// ADR-012). update() runs at the controller_manager rate: read the joints' position, velocity
// and effort state interfaces through ImperfectJointInterface (plant effects on), step the loop,
// write the effort command interfaces. on_activate reads the measured state and starts the
// trajectory on it (activation fails when the state cannot be read). State stamps count update
// cycles from the activation time: the `time` argument follows the ROS clock (/clock in
// simulation), which can stall for many cycles while the interfaces are still read every cycle,
// and the loop would hold on the stale stamp. A writer thread drains the log ring into the Run
// CSV at `log_path` (default: results/<run-id>/run.csv under the working directory); `~/status`
// carries the loop mode.
//
// Imperfections on the Gazebo path (ADR-011, feature gazebo-realism): sensor and actuator
// effects apply through ImperfectJointInterface; friction is applied by the physics engine
// (sim.launch.py writes the profile's friction into the joints' <dynamics>), so the interface
// gets the profile with friction zeroed; model mismatch is emulated by giving the controller
// perturb_model(nominal) instead of perturbing the plant (same size of error, opposite sign);
// payload_kg cannot be emulated that way and is rejected; timing jitter has no meaning on the
// sim clock and is ignored with a warning. `seed` >= 0 overrides the profile's seed.
//
// `command_source` ("" = the config file's, "config" or "external") picks the trajectory source.
// External (feature waypoint-ros): the loop waits in standby and WaypointServer offers
// ~/follow_waypoints, ~/preview_waypoints, ~/reset and the latched ~/reference_path; update()
// hands its commands to the loop and publishes a LoopSnapshot (waypoint_server.hpp).
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/logging.hpp>

#include "cobot/common/logger.hpp"
#include "cobot/imperfect/imperfect_joint_interface.hpp"
#include "cobot/imperfect/perturb.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/control_loop.hpp"

#include "waypoint_server.hpp"
#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rcutils/logging.h>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/string.hpp>

namespace cobot_ros2 {
namespace {

namespace ci = controller_interface;
using ci::CallbackReturn;
using hardware_interface::LoanedCommandInterface;
using hardware_interface::LoanedStateInterface;

constexpr double kStatusRateHz = 10.0;  // ~/status messages per second
constexpr std::chrono::milliseconds kWriterIdle{1};

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

  cobot::common::Status read(cobot::hal::JointState<kN>& out) override {
    cobot::hal::JointState<kN> s;
    s.t = stamp_;
    s.tau_valid = true;
    for (int i = 0; i < kN; ++i) {
      const std::optional<double> q = q_[i]->get_optional();
      const std::optional<double> qd = qd_[i]->get_optional();
      const std::optional<double> tau = tau_[i]->get_optional();
      if (!q || !qd || !tau) {
        return cobot::common::Status::error("ros2_control: state interface not readable");
      }
      s.q(i) = *q;
      s.qd(i) = *qd;
      s.tau_measured(i) = *tau;
    }
    out = s;
    return cobot::common::Status::ok();
  }

  cobot::common::Status write(const cobot::hal::JointCommand<kN>& cmd) override {
    bool ok = true;
    for (int i = 0; i < kN; ++i) {
      ok = cmd_[i]->set_value(cmd.tau(i)) && ok;
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

}  // namespace

class CobotController final : public ci::ControllerInterface {
 public:
  ~CobotController() override { stop_writer(); }

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
    const std::string config_path = get_node()->get_parameter("config_path").as_string();
    const std::string imp_path = get_node()->get_parameter("imperfections_path").as_string();
    const std::int64_t seed = get_node()->get_parameter("seed").as_int();
    if (joints_.size() != static_cast<std::size_t>(kN)) {
      RCLCPP_ERROR(log, "joints: expected %d names, got %zu", kN, joints_.size());
      return CallbackReturn::ERROR;
    }
    const cobot::common::Result<cobot::pipeline::Config<kN>> config =
        cobot::pipeline::load_config<kN>(config_path);
    const cobot::common::Result<cobot::imperfect::ImperfectionConfig<kN>> imp =
        cobot::pipeline::load_imperfections<kN>(imp_path);
    if (!config || !imp) {
      RCLCPP_ERROR(log, "%s", (config ? imp.reason() : config.reason()).c_str());
      return CallbackReturn::ERROR;
    }
    cobot::pipeline::Config<kN> c = config.value();
    const std::string source = get_node()->get_parameter("command_source").as_string();
    if (source == "config" || source == "external") {
      c.command_source = source == "external" ? cobot::pipeline::CommandSource::kExternal
                                              : cobot::pipeline::CommandSource::kConfig;
    } else if (!source.empty()) {
      RCLCPP_ERROR(
          log, "command_source: expected \"\", config or external, got %s", source.c_str());
      return CallbackReturn::ERROR;
    }
    cobot::imperfect::ImperfectionConfig<kN> profile = imp.value();
    if (seed >= 0) {
      profile.seed = static_cast<std::uint64_t>(seed);
    }
    if (profile.model.payload_kg != 0.0) {
      RCLCPP_ERROR(log,
                   "%s: imperfections.model.payload_kg = %g is not supported on the Gazebo path "
                   "(model perturbation cannot emulate a payload); set it to 0",
                   imp_path.c_str(),
                   profile.model.payload_kg);
      return CallbackReturn::ERROR;
    }
    if (profile.timing.period_jitter_std_us != 0.0) {
      RCLCPP_WARN(log,
                  "%s: imperfections.timing.period_jitter_std_us = %g ignored on the Gazebo path "
                  "(the sim clock has no jitter)",
                  imp_path.c_str(),
                  profile.timing.period_jitter_std_us);
    }
    profile.friction = cobot::imperfect::FrictionConfig<kN>{};  // in the physics (<dynamics>)
    const cobot::common::Result<cobot::model::UrdfChain> chain =
        cobot::model::parse_urdf_chain(c.urdf_path, c.base_link, c.tip_link);
    const cobot::common::Result<cobot::model::Arm7> arm =
        chain ? cobot::model::make_model<kN>(chain.value(), c.gravity_b)
              : cobot::common::Result<cobot::model::Arm7>::error(chain.reason());
    if (!arm) {
      RCLCPP_ERROR(log, "%s: %s", c.urdf_path.c_str(), arm.reason().c_str());
      return CallbackReturn::ERROR;
    }
    server_.reset();
    link_.reset();
    try {
      // Model mismatch: the controller's model is perturbed instead of the plant.
      const cobot::model::Arm7 model =
          cobot::imperfect::perturb_model(arm.value(), profile.model, profile.seed);
      loop_ = std::make_unique<cobot::pipeline::ControlLoop<kN>>(model, c);
      for (int i = 0; i < kN; ++i) {
        tau_max_(i) = model.joints[static_cast<std::size_t>(i)].tau_max;
      }
      if (c.command_source == cobot::pipeline::CommandSource::kExternal) {
        link_ = std::make_unique<LoopLink>();
        server_ = std::make_unique<WaypointServer>(get_node(), *link_, model, c);
      }
      io_ = std::make_unique<cobot::imperfect::ImperfectJointInterface<kN>>(
          ros_io_, profile, 1.0 / c.control_rate_hz, true);
      status_pub_ = get_node()->create_publisher<std_msgs::msg::String>(
          "~/status", rclcpp::SystemDefaultsQoS());
      status_rt_ = std::make_unique<StatusPublisher>(status_pub_);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(log, "%s: %s", imp_path.c_str(), e.what());
      return CallbackReturn::ERROR;
    }
    if (std::llround(c.control_rate_hz) != static_cast<long long>(get_update_rate())) {
      RCLCPP_ERROR(log,
                   "%s: cobot.control_rate_hz %g differs from the update rate %u Hz",
                   config_path.c_str(),
                   c.control_rate_hz,
                   get_update_rate());
      return CallbackReturn::ERROR;
    }
    period_ns_ = std::llround(1e9 / c.control_rate_hz);
    status_every_ = std::max<std::int64_t>(1, std::llround(c.control_rate_hz / kStatusRateHz));
    RCLCPP_INFO(log,
                "configured: config %s, imperfections %s, seed %llu",
                config_path.c_str(),
                imp_path.c_str(),
                static_cast<unsigned long long>(profile.seed));
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
    const cobot::common::Status read = io_->read(state_);
    if (!read) {
      RCLCPP_ERROR(log, "initial state: %s", read.reason().c_str());
      return CallbackReturn::ERROR;
    }
    const cobot::common::Status started = loop_->start(state_);
    if (!started) {
      RCLCPP_ERROR(log, "%s", started.reason().c_str());
      return CallbackReturn::ERROR;
    }
    std::error_code ec;
    const std::filesystem::path csv =
        log_path_.empty() ? std::filesystem::current_path(ec) / "results" / run_id() / "run.csv"
                          : std::filesystem::path(log_path_);
    std::filesystem::create_directories(csv.parent_path(), ec);
    csv_.open(csv, std::ios::binary | std::ios::trunc);
    if (!csv_) {
      RCLCPP_ERROR(log, "cannot open %s for writing", csv.c_str());
      return CallbackReturn::ERROR;
    }
    cobot::pipeline::write_csv_header<kN>(csv_);
    writer_stop_.store(false, std::memory_order_release);
    writer_ = std::thread([this] { drain_log(); });
    RCLCPP_INFO(log, "active, writing %s", csv.c_str());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
    for (LoanedCommandInterface& cmd : command_interfaces_) {
      static_cast<void>(cmd.set_value(0.0));
    }
    stop_writer();
    return CallbackReturn::SUCCESS;
  }

  ci::return_type update(const rclcpp::Time& /*time*/,
                         const rclcpp::Duration& /*period*/) override {
    ros_io_.set_stamp(cobot::common::Timestamp{t0_ns_ + cycle_ * period_ns_});
    // A failed read keeps the previous state and stamp: the loop's freshness check holds.
    static_cast<void>(io_->read(state_));
    if (link_) {
      take_commands();
    }
    static_cast<void>(loop_->step(state_, cmd_));
    static_cast<void>(io_->write(cmd_));
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

  // External mode, after step(): the snapshot, skipped for this cycle when the lock is busy.
  void publish_snapshot() {
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

  // Writer thread: flushes after every batch so the rows survive an abrupt end of Gazebo; in
  // external mode it also accumulates the goal metrics from the rows.
  void drain_log() {
    cobot::pipeline::LogRecord<kN> r;
    RunAccumulator runs(tau_max_);
    bool last = false;
    while (!last) {
      last = writer_stop_.load(std::memory_order_acquire);
      bool any = false;
      while (loop_->log().pop(r)) {
        cobot::pipeline::write_csv_row<kN>(csv_, r);
        runs.add(r);
        any = true;
      }
      if (any) {
        csv_.flush();
        if (link_) {
          const std::lock_guard<std::mutex> lock(link_->mutex);
          link_->stats = runs.stats();
        }
      } else if (!last) {
        std::this_thread::sleep_for(kWriterIdle);
      }
    }
    if (loop_->log().dropped() > 0) {
      RCLCPP_WARN(rclcpp::get_logger("cobot"), "%zu log rows dropped", loop_->log().dropped());
    }
  }

  void stop_writer() {
    if (writer_.joinable()) {
      writer_stop_.store(true, std::memory_order_release);
      writer_.join();
    }
    if (csv_.is_open()) {
      csv_.close();
    }
  }

  std::vector<std::string> joints_;
  std::string log_path_;
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
  std::ofstream csv_;
  std::thread writer_;
  std::atomic<bool> writer_stop_{false};
  cobot::common::VecN<kN> tau_max_ = cobot::common::VecN<kN>::Ones();
  // External mode (null otherwise); the server goes before the link it refers to.
  std::unique_ptr<LoopLink> link_;
  std::unique_ptr<WaypointServer> server_;
  std::uint64_t loaded_id_ = 0;
  std::uint64_t reset_id_ = 0;
  cobot::common::Status load_status_ = cobot::common::Status::ok();
  cobot::common::Status reset_status_ = cobot::common::Status::ok();
  cobot::pipeline::LoopMode last_mode_ = cobot::pipeline::LoopMode::kIdle;
  cobot::common::VecN<kN> standby_q_ = cobot::common::VecN<kN>::Zero();
  cobot::common::Pose standby_reference_;
};

}  // namespace cobot_ros2

PLUGINLIB_EXPORT_CLASS(cobot_ros2::CobotController, controller_interface::ControllerInterface)
