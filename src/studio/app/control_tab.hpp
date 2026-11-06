#pragma once

#include <QString>
#include <QWidget>
#include <array>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "cobot/common/types.hpp"
#include "cobot/planning/waypoint_trajectory.hpp"
#include "cobot/studio/bridge_client.hpp"

class QAbstractButton;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPlainTextEdit;
class QPushButton;
class QSlider;
class QTableWidget;
class QTimer;

// The Control tab of cobot_studio (feature studio-control, cobot_studio design 5.3, 5.4, 6.8;
// ADR-022): joint, TCP and waypoint modes over the window's BridgeClient, with Stop, E-stop and
// Reset always visible and the live robot in 3D. Widgets carry the object names of the UI
// contract. GUI thread only; refresh() runs from the window's bridge timer.
namespace cobot::studio {

class RobotView;

class ControlTab : public QWidget {
  Q_OBJECT
 public:
  // name: the bridge name (region /cobot_<name>); config_yaml names the URDF of the 3D view.
  ControlTab(BridgeClient& client,
             const std::string& name,
             const std::filesystem::path& config_yaml,
             QWidget* parent = nullptr);

  void refresh();  // labels, values, enabling, responses (every bridge tick)
  QSlider* speed_slider() const { return speed_; }
  // Shows the response to `id` in statusLine as "<what>: <text>" once it arrives.
  void watch(std::uint64_t id, const QString& what);
  void show_message(const QString& text);

 private:
  using Vec = common::VecN<bench::kN>;
  QWidget* build_joint_page();
  QWidget* build_tcp_page();
  QWidget* build_waypoint_page();
  // A press / release pair driving the jog `start` (refreshed every 25 ms while held).
  QPushButton* jog_button(const QString& name, const QString& text, std::function<void()> start);
  void sent(const common::Result<std::uint64_t>& id, const QString& what);
  common::Pose tcp_pose() const;  // of the latest sample
  std::vector<planning::Waypoint> table_points() const;
  void add_row(const common::Pose& T_b_tcp, double duration_s, bool stop);
  double speed() const;

  BridgeClient& client_;
  QString region_;
  std::optional<std::uint64_t> generation_;  // session whose jog band bounds the goals
  std::vector<std::pair<std::uint64_t, QString>> watched_;
  std::uint64_t preview_id_ = 0;
  std::vector<common::Vec3> preview_path_;
  QString message_;
  QPushButton* held_ = nullptr;  // the jog button being held
  QTimer* jog_timer_ = nullptr;
  std::function<void()> jog_;

  QLabel* connection_ = nullptr;
  QLabel* owner_ = nullptr;
  QPushButton* claim_ = nullptr;
  QPushButton* release_ = nullptr;
  std::array<QLabel*, bench::kN> joint_values_{};
  std::array<QDoubleSpinBox*, bench::kN> joint_goals_{};
  std::array<QLabel*, 6> tcp_values_{};
  std::array<QDoubleSpinBox*, 6> tcp_goals_{};
  QComboBox* frame_ = nullptr;
  std::vector<QPushButton*> jogs_;
  std::vector<QPushButton*> motion_;  // Go and Run: owner and standby
  QTableWidget* table_ = nullptr;
  QPlainTextEdit* issues_ = nullptr;
  QPushButton* preview_button_ = nullptr;
  QPushButton* add_pose_ = nullptr;
  QSlider* speed_ = nullptr;
  QPushButton* stop_ = nullptr;
  QPushButton* estop_ = nullptr;
  QPushButton* reset_ = nullptr;
  QLabel* status_ = nullptr;
  RobotView* view_ = nullptr;
};

}  // namespace cobot::studio
