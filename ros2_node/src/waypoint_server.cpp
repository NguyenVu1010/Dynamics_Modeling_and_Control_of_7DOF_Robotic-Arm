#include "waypoint_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <string>
#include <thread>
#include <utility>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

namespace cobot_ros2 {
namespace {

using cobot::pipeline::LoopMode;

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
constexpr std::chrono::milliseconds kTickPeriod{100};  // feedback at 10 Hz
constexpr std::chrono::milliseconds kResetWait{1000};  // for update() to answer a reset
constexpr std::chrono::milliseconds kResetPoll{5};

// "~/name" -> "<node name>/name": the action name is resolved like the services' "~/" names.
std::string private_name(const rclcpp_lifecycle::LifecycleNode& node, const std::string& name) {
  return std::string(node.get_name()) + name.substr(1);
}

geometry_msgs::msg::Pose pose_msg(const cobot::common::Pose& T) {
  const Eigen::Quaterniond q(T.R);  // Hamilton (w, x, y, z)
  geometry_msgs::msg::Pose m;
  m.position.x = T.p.x();
  m.position.y = T.p.y();
  m.position.z = T.p.z();
  m.orientation.w = q.w();
  m.orientation.x = q.x();
  m.orientation.y = q.y();
  m.orientation.z = q.z();
  return m;
}

std::vector<cobot_interfaces::msg::WaypointIssue> issue_msgs(
    const std::vector<cobot::pipeline::WaypointIssue>& issues) {
  std::vector<cobot_interfaces::msg::WaypointIssue> out;
  for (const cobot::pipeline::WaypointIssue& i : issues) {
    cobot_interfaces::msg::WaypointIssue m;
    m.waypoint = static_cast<std::uint32_t>(i.waypoint);
    m.issue = cobot::pipeline::to_string(i.issue);
    m.joint = i.joint;
    m.value = i.value;
    m.limit = i.limit;
    out.push_back(m);
  }
  return out;
}

}  // namespace

void RunAccumulator::add(const cobot::pipeline::LogRecord<kN>& r) {
  stats_.pos_err_m = r.task_error.head<3>().norm();
  stats_.rot_err_rad = r.task_error.tail<3>().norm();
  const bool tracking = r.mode == LoopMode::kTracking;
  if (!tracking && !in_run_) {
    return;
  }
  if (tracking && !in_run_) {
    in_run_ = true;
    sum_sq_.setZero();
    count_ = 0;
    stats_.peak_tau_ratio = 0.0;
  }
  const double ratio = r.tau.cwiseAbs().cwiseQuotient(tau_max_).maxCoeff();
  stats_.peak_tau_ratio = std::max(stats_.peak_tau_ratio, ratio);
  if (tracking) {
    const cobot::common::VecN<kN> e = r.q_ref - r.q;
    sum_sq_ += e.cwiseAbs2();
    ++count_;
    return;
  }
  in_run_ = false;
  ++stats_.runs_done;
  stats_.end_mode = r.mode;
  stats_.final_pos_err_m = stats_.pos_err_m;
  stats_.final_rot_err_rad = stats_.rot_err_rad;
  stats_.rms_joint_err_rad = std::sqrt(sum_sq_.maxCoeff() / static_cast<double>(count_));
}

WaypointServer::WaypointServer(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                               LoopLink& link,
                               cobot::model::Arm7 model,
                               cobot::pipeline::Config<kN> config)
    : node_(std::move(node)), link_(link), model_(std::move(model)), config_(std::move(config)) {
  action_ = rclcpp_action::create_server<Follow>(
      node_,
      private_name(*node_, "~/follow_waypoints"),
      [this](const rclcpp_action::GoalUUID& /*uuid*/,
             const std::shared_ptr<const Follow::Goal>& /*goal*/) {
        const LoopMode mode = snapshot().mode;
        if (goal_ || mode != LoopMode::kStandby) {
          RCLCPP_WARN(node_->get_logger(),
                      "goal rejected: %s (mode %s)",
                      goal_ ? "a goal is running" : "the loop is not in standby",
                      cobot::pipeline::to_string(mode));
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandle>& /*goal*/) {
        link_.stop_requested.store(true, std::memory_order_release);  // stop along the path
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle>& goal) { accept(goal); });
  preview_ = node_->create_service<Preview>(
      "~/preview_waypoints",
      [this](const std::shared_ptr<Preview::Request>& request,
             const std::shared_ptr<Preview::Response>& response) {
        const cobot::common::Result<Validation> v = validate(request->waypoints);
        if (!v) {
          RCLCPP_WARN(node_->get_logger(), "preview: %s", v.reason().c_str());
          return;
        }
        response->ok = v.value().ok;
        response->issues = issue_msgs(v.value().issues);
        response->path = path_msg(v.value());
        response->segment_durations_s = v.value().segment_durations_s;
      });
  reset_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/reset",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>& /*request*/,
             const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
        reset(*response);
      });
  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
      "~/reference_path", rclcpp::QoS(1).reliable().transient_local());
  timer_ = node_->create_wall_timer(kTickPeriod, [this] { tick(); });
}

WaypointServer::~WaypointServer() {
  if (goal_ && goal_->is_active()) {
    try {
      auto result = std::make_shared<Follow::Result>();
      result->message = "controller shut down";
      goal_->abort(result);
    } catch (const std::exception& e) {
      RCLCPP_WARN(node_->get_logger(), "abort on shutdown: %s", e.what());
    }
  }
}

LoopSnapshot WaypointServer::snapshot() {
  const std::lock_guard<std::mutex> lock(link_.mutex);
  return link_.snapshot;
}

cobot::common::Result<WaypointServer::Validation> WaypointServer::validate(
    const std::vector<cobot_interfaces::msg::Waypoint>& waypoints) {
  const LoopSnapshot s = snapshot();
  if (!s.standby_seen) {
    return cobot::common::Result<Validation>::error("the loop has not been in standby yet");
  }
  // TCP poses in, tip poses to the core: T_b_tip = T_b_tcp * T_tip_tcp^-1.
  const cobot::common::Pose T_tcp_tip = cobot::common::inverse(config_.tcp_offset.T_tip_tcp);
  std::vector<cobot::planning::Waypoint> list;
  for (const cobot_interfaces::msg::Waypoint& w : waypoints) {
    const geometry_msgs::msg::Quaternion& o = w.pose.orientation;
    cobot::common::Pose T_b_tcp;
    T_b_tcp.p = cobot::common::Vec3(w.pose.position.x, w.pose.position.y, w.pose.position.z);
    // Hamilton (w, x, y, z), not normalized: a non-unit quaternion fails create()'s check.
    T_b_tcp.R = Eigen::Quaterniond(o.w, o.x, o.y, o.z).toRotationMatrix();
    list.push_back({cobot::common::compose(T_b_tcp, T_tcp_tip), w.duration_s, w.stop});
  }
  return cobot::pipeline::validate_path<kN>(model_,
                                            config_.clik,
                                            config_.friction_compensation,
                                            s.standby_q,
                                            s.standby_reference,
                                            list,
                                            config_.cartesian_limits,
                                            config_.path_validation);
}

nav_msgs::msg::Path WaypointServer::path_msg(const Validation& v) const {
  nav_msgs::msg::Path path;
  path.header.frame_id = config_.base_link;
  path.header.stamp = node_->now();
  for (const cobot::common::Pose& T_b_tip : v.path) {
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose = pose_msg(cobot::common::compose(T_b_tip, config_.tcp_offset.T_tip_tcp));
    path.poses.push_back(p);
  }
  return path;
}

void WaypointServer::accept(const std::shared_ptr<GoalHandle>& goal) {
  auto result = std::make_shared<Follow::Result>();
  const cobot::common::Result<Validation> v = validate(goal->get_goal()->waypoints);
  if (!v || !v.value().ok || !v.value().trajectory) {
    result->message = v ? "path validation failed" : v.reason();
    if (v) {
      result->issues = issue_msgs(v.value().issues);
    }
    RCLCPP_WARN(node_->get_logger(), "goal aborted: %s", result->message.c_str());
    goal->abort(result);
    return;
  }
  path_pub_->publish(path_msg(v.value()));
  {
    const std::lock_guard<std::mutex> lock(link_.mutex);
    runs_before_ = link_.stats.runs_done;
  }
  goal_ = goal;
  goal_duration_s_ = v.value().trajectory->duration_s();
  link_.stop_requested.store(false, std::memory_order_release);
  link_.command.writeFromNonRT(WaypointCommand{++goal_id_, v.value().trajectory});
}

void WaypointServer::tick() {
  if (!goal_) {
    return;
  }
  LoopSnapshot s;
  RunStats stats;
  {
    const std::lock_guard<std::mutex> lock(link_.mutex);
    s = link_.snapshot;
    stats = link_.stats;
  }
  if (goal_->is_canceling()) {
    link_.stop_requested.store(true, std::memory_order_release);  // until update() saw it
  }
  if (s.loaded_id != goal_id_) {
    return;  // not taken by update() yet
  }
  auto result = std::make_shared<Follow::Result>();
  if (!s.load_ok) {
    result->message = s.load_message;
    finish(result, false);
    return;
  }
  if (stats.runs_done > runs_before_) {
    result->final_pos_err_m = stats.final_pos_err_m;
    result->final_rot_err_deg = stats.final_rot_err_rad * kRadToDeg;
    result->rms_joint_err_rad = stats.rms_joint_err_rad;
    result->peak_tau_ratio = stats.peak_tau_ratio;
    const bool standby = stats.end_mode == LoopMode::kStandby;
    result->message =
        std::string("trajectory ended in mode ") + cobot::pipeline::to_string(stats.end_mode);
    finish(result, standby);
    return;
  }
  auto feedback = std::make_shared<Follow::Feedback>();
  feedback->segment = static_cast<std::uint32_t>(s.segment);
  feedback->t_s = s.t_s;
  feedback->duration_s = goal_duration_s_;
  feedback->pos_err_m = stats.pos_err_m;
  feedback->rot_err_deg = stats.rot_err_rad * kRadToDeg;
  feedback->mode = cobot::pipeline::to_string(s.mode);
  goal_->publish_feedback(feedback);
}

void WaypointServer::finish(const std::shared_ptr<Follow::Result>& result, bool success) {
  result->success = success && !goal_->is_canceling();
  if (goal_->is_canceling()) {
    goal_->canceled(result);
  } else if (result->success) {
    goal_->succeed(result);
  } else {
    RCLCPP_WARN(node_->get_logger(), "goal aborted: %s", result->message.c_str());
    goal_->abort(result);
  }
  goal_.reset();
}

void WaypointServer::reset(std_srvs::srv::Trigger::Response& response) {
  const std::uint64_t id = ++reset_id_;
  link_.reset_requested.store(id, std::memory_order_release);
  const auto deadline = std::chrono::steady_clock::now() + kResetWait;
  while (std::chrono::steady_clock::now() < deadline) {
    const LoopSnapshot s = snapshot();
    if (s.reset_id == id) {
      response.success = s.reset_ok;
      response.message = s.reset_ok ? "standby" : s.reset_message;
      return;
    }
    std::this_thread::sleep_for(kResetPoll);
  }
  response.success = false;
  response.message = "reset: no answer from the control loop (controller inactive?)";
}

}  // namespace cobot_ros2
