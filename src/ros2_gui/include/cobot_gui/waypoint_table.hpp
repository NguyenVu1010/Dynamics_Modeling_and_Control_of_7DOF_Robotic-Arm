#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include "cobot/common/result.hpp"
#include "cobot/common/types.hpp"

#include "cobot_interfaces/msg/waypoint.hpp"

// Data model of the waypoint panel (design docs/design/2026-10-29-waypoint-gui-design.md 6.5),
// kept free of Qt so it is unit tested without a display. Rows are TCP poses in base_link, the
// format of the waypoint file (docs/conventions.md "Waypoint file").
namespace cobot_gui {

struct WaypointRow {
  double x_m = 0.0, y_m = 0.0, z_m = 0.0;
  double roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;  // R = Rz(yaw) Ry(pitch) Rx(roll)
  double duration_s = 0.0;                                // <= 0: from the Cartesian limits
  bool stop = false;
};

// Quaternions are Hamilton (w, x, y, z), as geometry_msgs. The result has unit norm.
geometry_msgs::msg::Quaternion rpy_deg_to_quaternion(double roll_deg,
                                                     double pitch_deg,
                                                     double yaw_deg);
// Normalises q first. roll and yaw in [-180, 180], pitch in [-90, 90]; at pitch +-90 roll is 0.
std::array<double, 3> quaternion_to_rpy_deg(const geometry_msgs::msg::Quaternion& q);
cobot::common::Pose row_to_pose(const WaypointRow& row);
WaypointRow pose_to_row(const cobot::common::Pose& pose, double duration_s, bool stop);
geometry_msgs::msg::Pose row_to_pose_msg(const WaypointRow& row);
WaypointRow pose_msg_to_row(const geometry_msgs::msg::Pose& pose, double duration_s, bool stop);

class WaypointTable {
 public:
  std::size_t size() const { return rows_.size(); }
  bool empty() const { return rows_.empty(); }
  const WaypointRow& row(std::size_t i) const { return rows_[i]; }  // precondition i < size()
  const std::vector<WaypointRow>& rows() const { return rows_; }

  // Edits. An index out of range returns false and changes nothing (rows and revision).
  // insert(i, r) puts r before row i; i == size() appends.
  void add(const WaypointRow& row);
  bool insert(std::size_t index, const WaypointRow& row);
  bool remove(std::size_t index);
  bool move_up(std::size_t index);    // swap with index - 1
  bool move_down(std::size_t index);  // swap with index + 1
  bool update(std::size_t index, const WaypointRow& row);

  // Waypoint file through pipeline::load_waypoints / save_waypoints with an identity TCP
  // offset (the rows are TCP poses). Load replaces all rows; a failed load changes nothing.
  // Save is not an edit.
  cobot::common::Status load(const std::filesystem::path& yaml, const std::string& base_link);
  cobot::common::Status save(const std::filesystem::path& yaml, const std::string& base_link) const;
  std::vector<cobot_interfaces::msg::Waypoint> to_msg() const;

  // Run-enabled rule: revision() changes on every successful edit and load. The panel records
  // revision() when it sends a preview and reports the answer with set_preview_result. Run is
  // enabled only when the last reported preview was ok, for the current revision, and the
  // table is not empty.
  std::uint64_t revision() const { return revision_; }
  void set_preview_result(std::uint64_t revision, bool ok);
  bool run_enabled() const;

 private:
  std::vector<WaypointRow> rows_;
  std::uint64_t revision_ = 0;
  std::uint64_t preview_revision_ = 0;
  bool preview_ok_ = false;
};

}  // namespace cobot_gui
