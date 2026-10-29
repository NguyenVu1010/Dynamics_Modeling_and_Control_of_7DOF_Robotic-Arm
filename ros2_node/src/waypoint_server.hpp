#pragma once

// External command mode of cobot_controller (feature waypoint-ros, design
// docs/design/2026-10-29-waypoint-gui-design.md 6.4), after joint_trajectory_controller: the
// action server and services live in the controller and run on the executor (non-real-time);
// update() exchanges data with them only through LoopLink:
//   non-RT -> update()  a validated trajectory (RealtimeBuffer: copy on the writer side, pointer
//                       swap on the RT side), the stop request and the reset request (atomics);
//   update() -> non-RT  LoopSnapshot, written under try_lock (update() never waits);
//   writer   -> non-RT  RunStats, accumulated by the CSV writer thread from the log rows, which
//                       carry q_ref, tau and the task error of every step.
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <nav_msgs/msg/path.hpp>

#include "cobot/common/result.hpp"
#include "cobot/common/types.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/log_record.hpp"
#include "cobot/pipeline/path_validation.hpp"
#include "cobot/planning/waypoint_trajectory.hpp"

#include <cobot_interfaces/action/follow_waypoints.hpp>
#include <cobot_interfaces/msg/waypoint.hpp>
#include <cobot_interfaces/srv/preview_waypoints.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace cobot_ros2 {

constexpr int kN = cobot::model::kArmDof;

struct WaypointCommand {
  std::uint64_t id = 0;  // 0: none
  std::optional<cobot::planning::WaypointTrajectory> trajectory;
};

struct LoopSnapshot {
  cobot::pipeline::LoopMode mode = cobot::pipeline::LoopMode::kIdle;
  bool standby_seen = false;
  cobot::common::Pose standby_reference;  // T_b_d when standby was last entered
  cobot::common::VecN<kN> standby_q = cobot::common::VecN<kN>::Zero();  // measured q then
  std::size_t segment = 0;
  double t_s = 0.0;
  std::uint64_t loaded_id = 0;  // last command passed to ControlLoop::load
  bool load_ok = false;
  std::string load_message;
  std::uint64_t reset_id = 0;  // last reset request passed to ControlLoop::reset
  bool reset_ok = false;
  std::string reset_message;
};

struct RunStats {
  std::uint64_t runs_done = 0;  // tracking phases that ended
  cobot::pipeline::LoopMode end_mode = cobot::pipeline::LoopMode::kIdle;  // mode that ended it
  double final_pos_err_m = 0.0;
  double final_rot_err_rad = 0.0;
  double rms_joint_err_rad = 0.0;  // largest per-joint RMS of q_ref - q over the tracking rows
  double peak_tau_ratio = 0.0;
  double pos_err_m = 0.0;  // latest row
  double rot_err_rad = 0.0;
};

struct LoopLink {
  realtime_tools::RealtimeBuffer<WaypointCommand> command;
  std::atomic<bool> stop_requested{false};
  std::atomic<std::uint64_t> reset_requested{0};
  std::mutex mutex;  // guards snapshot and stats
  LoopSnapshot snapshot;
  RunStats stats;
};

// Writer thread: RunStats over the log rows, one run per tracking phase. No allocation.
class RunAccumulator {
 public:
  explicit RunAccumulator(const cobot::common::VecN<kN>& tau_max) : tau_max_(tau_max) {}
  void add(const cobot::pipeline::LogRecord<kN>& r);
  const RunStats& stats() const { return stats_; }

 private:
  cobot::common::VecN<kN> tau_max_;
  cobot::common::VecN<kN> sum_sq_ = cobot::common::VecN<kN>::Zero();
  std::int64_t count_ = 0;
  bool in_run_ = false;
  RunStats stats_;
};

class WaypointServer {
 public:
  WaypointServer(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                 LoopLink& link,
                 cobot::model::Arm7 model,
                 cobot::pipeline::Config<kN> config);
  ~WaypointServer();
  WaypointServer(const WaypointServer&) = delete;
  WaypointServer& operator=(const WaypointServer&) = delete;
  WaypointServer(WaypointServer&&) = delete;
  WaypointServer& operator=(WaypointServer&&) = delete;

 private:
  using Follow = cobot_interfaces::action::FollowWaypoints;
  using Preview = cobot_interfaces::srv::PreviewWaypoints;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Follow>;
  using Validation = cobot::pipeline::PathValidation<kN>;

  LoopSnapshot snapshot();
  cobot::common::Result<Validation> validate(
      const std::vector<cobot_interfaces::msg::Waypoint>& waypoints);
  nav_msgs::msg::Path path_msg(const Validation& v) const;
  void accept(const std::shared_ptr<GoalHandle>& goal);
  void tick();  // 10 Hz: feedback, then the result once update() loaded and finished the goal
  void finish(const std::shared_ptr<Follow::Result>& result, bool success);
  void reset(std_srvs::srv::Trigger::Response& response);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  LoopLink& link_;
  cobot::model::Arm7 model_;  // the controller's model, so validation sees what the loop sees
  cobot::pipeline::Config<kN> config_;
  rclcpp_action::Server<Follow>::SharedPtr action_;
  rclcpp::Service<Preview>::SharedPtr preview_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;  // latched
  rclcpp::TimerBase::SharedPtr timer_;
  // Goal state: touched only by callbacks of the node's default (mutually exclusive) group.
  std::shared_ptr<GoalHandle> goal_;
  std::uint64_t goal_id_ = 0;
  std::uint64_t runs_before_ = 0;  // RunStats::runs_done when the goal was accepted
  double goal_duration_s_ = 0.0;
  std::uint64_t reset_id_ = 0;
};

}  // namespace cobot_ros2
