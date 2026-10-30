#include "waypoint_panel.hpp"

#include <QBrush>
#include <QColor>
#include <QFileDialog>
#include <QGridLayout>
#include <QHeaderView>
#include <QStringList>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <utility>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace cobot_gui {
namespace {

using visualization_msgs::msg::InteractiveMarker;
using visualization_msgs::msg::InteractiveMarkerControl;
using visualization_msgs::msg::Marker;

constexpr char kBaseLink[] = "base_link";  // fixed frame of the waypoint file and the goal
constexpr char kController[] = "/cobot_controller/";
constexpr char kSelected[] = "selected";
constexpr int kColumns = 8;  // x, y, z, roll, pitch, yaw, duration, stop
constexpr int kStopColumn = 7;
constexpr double kPointScaleM = 0.05;  // numbered markers
constexpr double kHandleScaleM = 0.2;  // 6-DOF marker
const QColor kIssueColor(255, 190, 190);

// Where the 6-DOF marker starts with an empty table: the first point of the design 6.6 example.
WaypointRow start_row() {
  WaypointRow r;
  r.x_m = -0.5;
  r.y_m = -0.6;
  r.z_m = 0.9;
  r.yaw_deg = 90.0;
  return r;
}

double* field(WaypointRow& r, int column) {
  double* fields[] = {&r.x_m, &r.y_m, &r.z_m, &r.roll_deg, &r.pitch_deg, &r.yaw_deg, &r.duration_s};
  return column >= 0 && column < kStopColumn ? fields[column] : nullptr;
}

// Puts each path validation finding on its row (issues, one entry per row) and returns the
// "point N: ..." lines for the status area.
QString apply_issues(const std::vector<cobot_interfaces::msg::WaypointIssue>& found,
                     std::vector<std::string>& issues) {
  QString text;
  for (const cobot_interfaces::msg::WaypointIssue& issue : found) {
    const std::string what =
        issue.issue + (issue.joint >= 0 ? " joint " + std::to_string(issue.joint + 1) : "") + " " +
        std::to_string(issue.value) + " > " + std::to_string(issue.limit);
    if (issue.waypoint < issues.size()) {
      issues[issue.waypoint] += (issues[issue.waypoint].empty() ? "" : "; ") + what;
    }
    text += QString("\npoint %1: %2").arg(issue.waypoint + 1).arg(QString::fromStdString(what));
  }
  return text;
}

Marker point_marker(std::size_t index, bool has_issue) {
  Marker m;
  m.type = Marker::TEXT_VIEW_FACING;
  m.text = std::to_string(index + 1);
  m.scale.z = kPointScaleM;
  m.pose.position.z = kPointScaleM;  // text above the sphere
  m.color.r = 1.0F;
  m.color.g = has_issue ? 0.2F : 1.0F;
  m.color.b = has_issue ? 0.2F : 1.0F;
  m.color.a = 1.0F;
  return m;
}

void add_axis_controls(InteractiveMarker& marker) {
  const double h = std::sqrt(0.5);
  // Control frames whose x axis is the base x, z and y axes: Hamilton (w, x, y, z).
  const double quats[3][4] = {{h, h, 0.0, 0.0}, {h, 0.0, h, 0.0}, {h, 0.0, 0.0, h}};
  const char* axes[3] = {"x", "z", "y"};
  for (int i = 0; i < 3; ++i) {
    InteractiveMarkerControl c;
    c.orientation.w = quats[i][0];
    c.orientation.x = quats[i][1];
    c.orientation.y = quats[i][2];
    c.orientation.z = quats[i][3];
    c.name = std::string("rotate_") + axes[i];
    c.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(c);
    c.name = std::string("move_") + axes[i];
    c.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    marker.controls.push_back(c);
  }
}

}  // namespace

WaypointPanel::WaypointPanel(QWidget* parent) : rviz_common::Panel(parent) {
  marker_pose_ = row_to_pose_msg(start_row());
  grid_ = new QTableWidget(0, kColumns, this);
  grid_->setHorizontalHeaderLabels(
      {"x m", "y m", "z m", "roll deg", "pitch deg", "yaw deg", "duration s", "stop"});
  grid_->setSelectionBehavior(QAbstractItemView::SelectRows);
  grid_->setSelectionMode(QAbstractItemView::SingleSelection);
  grid_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  status_ = new QLabel(this);
  status_->setWordWrap(true);
  status_->setTextInteractionFlags(Qt::TextSelectableByMouse);

  auto* buttons = new QGridLayout;
  const auto button = [this, buttons](const char* text, int at, auto slot) {
    auto* b = new QPushButton(text, this);
    connect(b, &QPushButton::clicked, this, slot);
    buttons->addWidget(b, at / 5, at % 5);
    return b;
  };
  button("Add from marker", 0, [this] { add_from_marker(); });
  button("Delete", 1, [this] { delete_row(); });
  button("Up", 2, [this] { move(true); });
  button("Down", 3, [this] { move(false); });
  button("Load", 4, [this] { load(); });
  button("Save", 5, [this] { save(); });
  preview_button_ = button("Preview", 6, [this] { preview(); });
  run_button_ = button("Run", 7, [this] { run(); });
  button("Stop", 8, [this] { stop(); });
  button("Reset", 9, [this] { reset(); });

  auto* layout = new QVBoxLayout(this);
  layout->addWidget(grid_);
  layout->addLayout(buttons);
  layout->addWidget(status_);
  connect(grid_, &QTableWidget::cellChanged, this, &WaypointPanel::cell_changed);
  connect(grid_, &QTableWidget::itemSelectionChanged, this, [this] {
    if (!filling_) {
      refresh_markers();
    }
  });
  refresh_buttons();
}

void WaypointPanel::onInitialize() {
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  const std::string prefix = kController;
  follow_ = rclcpp_action::create_client<Follow>(node_, prefix + "follow_waypoints");
  preview_client_ = node_->create_client<Preview>(prefix + "preview_waypoints");
  reset_client_ = node_->create_client<std_srvs::srv::Trigger>(prefix + "reset");
  preview_path_ = node_->create_publisher<nav_msgs::msg::Path>(
      "~/preview_path", rclcpp::QoS(1).reliable().transient_local());
  markers_ =
      std::make_unique<interactive_markers::InteractiveMarkerServer>("cobot_waypoints", node_);
  refresh_markers();
  show_status("Add points, Preview, then Run.");
}

int WaypointPanel::selected() const {
  const QList<QTableWidgetSelectionRange> ranges = grid_->selectedRanges();
  return ranges.isEmpty() ? -1 : ranges.first().topRow();
}

void WaypointPanel::add_from_marker() {
  const WaypointRow row = pose_msg_to_row(marker_pose_, 0.0, false);
  const int at = selected() < 0 ? static_cast<int>(table_.size()) : selected() + 1;
  table_.insert(static_cast<std::size_t>(at), row);
  edited(at);
}

void WaypointPanel::delete_row() {
  const int i = selected();
  if (i >= 0 && table_.remove(static_cast<std::size_t>(i))) {
    edited(std::min(i, static_cast<int>(table_.size()) - 1));
  }
}

void WaypointPanel::move(bool up) {
  const int i = selected();
  if (i < 0) {
    return;
  }
  const auto index = static_cast<std::size_t>(i);
  if (up ? table_.move_up(index) : table_.move_down(index)) {
    edited(up ? i - 1 : i + 1);
  }
}

void WaypointPanel::load() {
  const QString path =
      QFileDialog::getOpenFileName(this, "Load waypoints", {}, "YAML (*.yaml *.yml)");
  if (path.isEmpty()) {
    return;
  }
  const cobot::common::Status s = table_.load(path.toStdString(), kBaseLink);
  if (!s) {
    show_status(QString::fromStdString("Load failed: " + s.reason()));
    return;
  }
  edited(table_.empty() ? -1 : 0);
  show_status(QString("Loaded %1 points from %2").arg(table_.size()).arg(path));
}

void WaypointPanel::save() {
  const QString path =
      QFileDialog::getSaveFileName(this, "Save waypoints", {}, "YAML (*.yaml *.yml)");
  if (path.isEmpty()) {
    return;
  }
  const cobot::common::Status s = table_.save(path.toStdString(), kBaseLink);
  show_status(s ? "Saved " + path : QString::fromStdString("Save failed: " + s.reason()));
}

void WaypointPanel::preview() {
  if (!preview_client_ || !preview_client_->service_is_ready()) {
    show_status("Preview service not available (controller with command_source external?)");
    return;
  }
  auto request = std::make_shared<Preview::Request>();
  request->waypoints = table_.to_msg();
  const std::uint64_t revision = table_.revision();
  // rclcpp matches the callback signature exactly, so the futures are taken by value.
  // NOLINTNEXTLINE(performance-unnecessary-value-param)
  auto on_answer = [this, revision](rclcpp::Client<Preview>::SharedFuture future) {
    const Preview::Response::SharedPtr& response = future.get();
    if (revision != table_.revision()) {
      return;  // the list changed while the preview ran
    }
    table_.set_preview_result(revision, response->ok);
    issues_.assign(table_.size(), std::string());
    QString text = response->ok ? "Preview ok:" : "Preview found issues:";
    for (const double d : response->segment_durations_s) {
      text += QString(" %1 s").arg(d, 0, 'f', 2);
    }
    text += apply_issues(response->issues, issues_);
    if (!response->ok && response->issues.empty()) {
      text += " the controller refused the list (see its log)";
    }
    preview_path_->publish(response->path);
    refresh_table(selected());
    refresh_markers();
    refresh_buttons();
    show_status(text);
  };
  preview_client_->async_send_request(request, std::move(on_answer));
  show_status("Preview sent");
}

void WaypointPanel::run() {
  if (!table_.run_enabled() || !follow_->action_server_is_ready()) {
    show_status("Run needs an ok preview of the current list and the controller action");
    return;
  }
  Follow::Goal goal;
  goal.waypoints = table_.to_msg();
  goal_size_ = table_.size();
  const std::uint64_t revision = table_.revision();
  rclcpp_action::Client<Follow>::SendGoalOptions options;
  options.goal_response_callback =
      [this](const rclcpp_action::ClientGoalHandle<Follow>::SharedPtr& handle) {
        goal_ = handle;
        refresh_buttons();
        show_status(handle ? "Running" : "Goal rejected (the loop is not in standby?)");
      };
  options
      .feedback_callback = [this](
                               const rclcpp_action::ClientGoalHandle<Follow>::SharedPtr& /*handle*/,
                               const std::shared_ptr<const Follow::Feedback>& f) {
    show_status(
        QString(
            "mode %1, segment %2/%3, t %4 / %5 s\nposition error %6 mm, orientation error %7 deg")
            .arg(QString::fromStdString(f->mode))
            .arg(f->segment + 1)
            .arg(goal_size_)
            .arg(f->t_s, 0, 'f', 2)
            .arg(f->duration_s, 0, 'f', 2)
            .arg(f->pos_err_m * 1e3, 0, 'f', 2)
            .arg(f->rot_err_deg, 0, 'f', 3));
  };
  options.result_callback =
      [this, revision](const rclcpp_action::ClientGoalHandle<Follow>::WrappedResult& r) {
        goal_.reset();
        const char* code = r.code == rclcpp_action::ResultCode::SUCCEEDED  ? "succeeded"
                           : r.code == rclcpp_action::ResultCode::CANCELED ? "stopped"
                                                                           : "aborted";
        QString text =
            QString(
                "Last result: %1 (%2)\nfinal position error %3 mm, orientation error %4 deg, "
                "RMS joint error %5 rad, peak torque ratio %6")
                .arg(code)
                .arg(QString::fromStdString(r.result->message))
                .arg(r.result->final_pos_err_m * 1e3, 0, 'f', 2)
                .arg(r.result->final_rot_err_deg, 0, 'f', 3)
                .arg(r.result->rms_joint_err_rad, 0, 'g', 3)
                .arg(r.result->peak_tau_ratio, 0, 'f', 2);
        if (!r.result->issues.empty()) {
          // The controller refused the list for path issues: list them, and when the list is
          // still the one that was sent, mark its rows and keep Run off until a new preview.
          const bool current = revision == table_.revision();
          std::vector<std::string> rows(goal_size_);
          text += apply_issues(r.result->issues, rows);
          if (current) {
            issues_ = std::move(rows);
            table_.set_preview_result(revision, false);
            refresh_table(selected());
            refresh_markers();
          }
        }
        refresh_buttons();
        show_status(text);
      };
  follow_->async_send_goal(goal, options);
  show_status("Goal sent");
}

void WaypointPanel::stop() {
  if (goal_) {
    follow_->async_cancel_goal(goal_);
    show_status("Stopping along the path");
  }
}

void WaypointPanel::reset() {
  if (!reset_client_ || !reset_client_->service_is_ready()) {
    show_status("Reset service not available");
    return;
  }
  // NOLINTNEXTLINE(performance-unnecessary-value-param)
  auto on_answer = [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    const std_srvs::srv::Trigger::Response::SharedPtr& r = future.get();
    show_status(QString::fromStdString((r->success ? "Reset: " : "Reset refused: ") + r->message));
  };
  reset_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
                                    std::move(on_answer));
}

void WaypointPanel::cell_changed(int row, int column) {
  if (filling_ || row < 0 || static_cast<std::size_t>(row) >= table_.size()) {
    return;
  }
  WaypointRow r = table_.row(static_cast<std::size_t>(row));
  const QTableWidgetItem* item = grid_->item(row, column);
  if (column == kStopColumn) {
    r.stop = item->checkState() == Qt::Checked;
  } else if (double* value = field(r, column); value != nullptr) {
    bool ok = false;
    const double v = item->text().toDouble(&ok);
    if (!ok || !std::isfinite(v)) {
      refresh_table(row);  // put the old value back
      return;
    }
    *value = v;
  }
  table_.update(static_cast<std::size_t>(row), r);
  edited(row);
}

void WaypointPanel::edited(int select) {
  issues_.assign(table_.size(), std::string());
  refresh_table(select);
  refresh_markers();
  refresh_buttons();
}

void WaypointPanel::refresh_table(int select) {
  filling_ = true;
  issues_.resize(table_.size());
  grid_->setRowCount(static_cast<int>(table_.size()));
  for (std::size_t i = 0; i < table_.size(); ++i) {
    WaypointRow r = table_.row(i);
    const int row = static_cast<int>(i);
    for (int c = 0; c < kColumns; ++c) {
      auto* item = new QTableWidgetItem;
      if (c == kStopColumn) {
        item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
        item->setCheckState(r.stop ? Qt::Checked : Qt::Unchecked);
      } else {
        item->setText(QString::number(*field(r, c), 'g', 6));
      }
      if (!issues_[i].empty()) {
        item->setBackground(QBrush(kIssueColor));
        item->setToolTip(QString::fromStdString(issues_[i]));
      }
      grid_->setItem(row, c, item);
    }
  }
  if (select >= 0 && select < grid_->rowCount()) {
    grid_->selectRow(select);
  }
  filling_ = false;
}

void WaypointPanel::refresh_markers() {
  if (!markers_) {
    return;
  }
  markers_->clear();
  for (std::size_t i = 0; i < table_.size(); ++i) {
    InteractiveMarker m;
    m.header.frame_id = kBaseLink;
    m.name = "wp_" + std::to_string(i);
    m.pose = row_to_pose_msg(table_.row(i));
    m.scale = static_cast<float>(kPointScaleM);
    const bool has_issue = i < issues_.size() && !issues_[i].empty();
    InteractiveMarkerControl c;
    c.always_visible = true;
    c.interaction_mode = InteractiveMarkerControl::NONE;
    Marker sphere = point_marker(i, has_issue);
    sphere.type = Marker::SPHERE;
    sphere.pose.position.z = 0.0;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = kPointScaleM / 2.0;
    c.markers.push_back(sphere);
    c.markers.push_back(point_marker(i, has_issue));
    m.controls.push_back(c);
    markers_->insert(m);
  }
  const int i = selected();
  if (i >= 0 && static_cast<std::size_t>(i) < table_.size()) {
    marker_pose_ = row_to_pose_msg(table_.row(static_cast<std::size_t>(i)));
  }
  InteractiveMarker handle;
  handle.header.frame_id = kBaseLink;
  handle.name = kSelected;
  handle.description = i >= 0 ? "point " + std::to_string(i + 1) : "new point";
  handle.pose = marker_pose_;
  handle.scale = static_cast<float>(kHandleScaleM);
  add_axis_controls(handle);
  markers_->insert(handle, [this](const Feedback::ConstSharedPtr& f) { marker_feedback(f); });
  markers_->applyChanges();
}

void WaypointPanel::marker_feedback(const Feedback::ConstSharedPtr& feedback) {
  if (feedback->marker_name != kSelected) {
    return;
  }
  marker_pose_ = feedback->pose;
  const int i = selected();
  if (i < 0 || static_cast<std::size_t>(i) >= table_.size()) {
    return;
  }
  const auto index = static_cast<std::size_t>(i);
  if (feedback->event_type == Feedback::POSE_UPDATE) {
    const WaypointRow& old = table_.row(index);
    table_.update(index, pose_msg_to_row(feedback->pose, old.duration_s, old.stop));
    issues_.assign(table_.size(), std::string());
    refresh_table(i);
    refresh_buttons();
    markers_->setPose("wp_" + std::to_string(i), feedback->pose);
    markers_->applyChanges();
  } else if (feedback->event_type == Feedback::MOUSE_UP) {
    refresh_markers();  // issue colours of the other points were cleared by the edit
  }
}

void WaypointPanel::refresh_buttons() {
  preview_button_->setEnabled(!table_.empty());
  run_button_->setEnabled(table_.run_enabled() && !goal_);
}

void WaypointPanel::show_status(const QString& text) {
  status_->setText(text);
}

}  // namespace cobot_gui

PLUGINLIB_EXPORT_CLASS(cobot_gui::WaypointPanel, rviz_common::Panel)
