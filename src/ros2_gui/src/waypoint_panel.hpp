#pragma once

#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "cobot_gui/waypoint_table.hpp"
#include "cobot_interfaces/action/follow_waypoints.hpp"
#include "cobot_interfaces/srv/preview_waypoints.hpp"
#include <interactive_markers/interactive_marker_server.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

namespace cobot_gui {

// RViz2 panel editing a waypoint list and driving the controller's external command mode
// (design docs/design/2026-10-29-waypoint-gui-design.md 6.5). Every ROS call is asynchronous;
// the answers arrive on the RViz node's executor, which runs in the Qt thread.
class WaypointPanel : public rviz_common::Panel {
  Q_OBJECT

 public:
  explicit WaypointPanel(QWidget* parent = nullptr);
  void onInitialize() override;

 private:
  using Follow = cobot_interfaces::action::FollowWaypoints;
  using Preview = cobot_interfaces::srv::PreviewWaypoints;
  using Feedback = visualization_msgs::msg::InteractiveMarkerFeedback;

  void add_from_marker();
  void delete_row();
  void move(bool up);
  void load();
  void save();
  void preview();
  void run();
  void stop();
  void reset();
  void cell_changed(int row, int column);

  int selected() const;
  // After an edit: drop the preview findings, refill the table and markers, update buttons.
  void edited(int select);
  void refresh_table(int select);
  void refresh_markers();
  void refresh_buttons();
  void marker_feedback(const Feedback::ConstSharedPtr& feedback);
  void show_status(const QString& text);

  WaypointTable table_;
  std::vector<std::string> issues_;       // per row, from the last preview of this revision
  bool filling_ = false;                  // refresh_table() is writing cells
  geometry_msgs::msg::Pose marker_pose_;  // the 6-DOF marker, where "Add from marker" adds
  std::size_t goal_size_ = 0;             // waypoints of the running goal

  QTableWidget* grid_ = nullptr;
  QPushButton* preview_button_ = nullptr;
  QPushButton* run_button_ = nullptr;
  QLabel* status_ = nullptr;

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> markers_;
  rclcpp_action::Client<Follow>::SharedPtr follow_;
  rclcpp_action::ClientGoalHandle<Follow>::SharedPtr goal_;
  rclcpp::Client<Preview>::SharedPtr preview_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr preview_path_;
};

}  // namespace cobot_gui
