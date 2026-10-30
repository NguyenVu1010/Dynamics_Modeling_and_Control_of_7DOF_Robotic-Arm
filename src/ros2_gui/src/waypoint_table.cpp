#include "cobot_gui/waypoint_table.hpp"

#include <cmath>
#include <iterator>
#include <utility>

#include <Eigen/Geometry>

#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/waypoint_file.hpp"
#include "cobot/planning/waypoint_trajectory.hpp"

namespace cobot_gui {
namespace {

using cobot::common::Mat3;
using cobot::common::Vec3;

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;
// cos(pitch) below this is the gimbal lock: roll and yaw are no longer separable.
constexpr double kGimbalCos = 1e-10;

Mat3 rpy_to_matrix(double roll_deg, double pitch_deg, double yaw_deg) {
  const Eigen::Quaterniond q = Eigen::AngleAxisd(yaw_deg * kDegToRad, Vec3::UnitZ()) *
                               Eigen::AngleAxisd(pitch_deg * kDegToRad, Vec3::UnitY()) *
                               Eigen::AngleAxisd(roll_deg * kDegToRad, Vec3::UnitX());
  return q.toRotationMatrix();
}

// Inverse of rpy_to_matrix for R = Rz(yaw) Ry(pitch) Rx(roll).
std::array<double, 3> matrix_to_rpy_deg(const Mat3& R) {
  const double cos_pitch = std::hypot(R(0, 0), R(1, 0));
  const double pitch = std::atan2(-R(2, 0), cos_pitch);
  double roll = 0.0;
  double yaw = 0.0;
  if (cos_pitch > kGimbalCos) {
    roll = std::atan2(R(2, 1), R(2, 2));
    yaw = std::atan2(R(1, 0), R(0, 0));
  } else {
    // pitch +-90: R(0,1) = -sin(yaw -+ roll), R(1,1) = cos(yaw -+ roll); take roll = 0.
    yaw = std::atan2(-R(0, 1), R(1, 1));
  }
  return {roll * kRadToDeg, pitch * kRadToDeg, yaw * kRadToDeg};
}

cobot::planning::Waypoint row_to_waypoint(const WaypointRow& row) {
  cobot::planning::Waypoint w;
  w.T_b_goal = row_to_pose(row);
  w.duration_s = row.duration_s;
  w.stop = row.stop;
  return w;
}

WaypointRow make_row(const Vec3& p,
                     const std::array<double, 3>& rpy,
                     double duration_s,
                     bool stop) {
  WaypointRow row;
  row.x_m = p.x();
  row.y_m = p.y();
  row.z_m = p.z();
  row.roll_deg = rpy[0];
  row.pitch_deg = rpy[1];
  row.yaw_deg = rpy[2];
  row.duration_s = duration_s;
  row.stop = stop;
  return row;
}

}  // namespace

geometry_msgs::msg::Quaternion rpy_deg_to_quaternion(double roll_deg,
                                                     double pitch_deg,
                                                     double yaw_deg) {
  // Hamilton (w, x, y, z)
  const Eigen::Quaterniond e(rpy_to_matrix(roll_deg, pitch_deg, yaw_deg));
  geometry_msgs::msg::Quaternion q;
  q.w = e.w();
  q.x = e.x();
  q.y = e.y();
  q.z = e.z();
  return q;
}

std::array<double, 3> quaternion_to_rpy_deg(const geometry_msgs::msg::Quaternion& q) {
  const Eigen::Quaterniond e(q.w, q.x, q.y, q.z);  // Hamilton (w, x, y, z)
  return matrix_to_rpy_deg(e.normalized().toRotationMatrix());
}

cobot::common::Pose row_to_pose(const WaypointRow& row) {
  cobot::common::Pose pose;
  pose.R = rpy_to_matrix(row.roll_deg, row.pitch_deg, row.yaw_deg);
  pose.p = Vec3(row.x_m, row.y_m, row.z_m);
  return pose;
}

WaypointRow pose_to_row(const cobot::common::Pose& pose, double duration_s, bool stop) {
  return make_row(pose.p, matrix_to_rpy_deg(pose.R), duration_s, stop);
}

geometry_msgs::msg::Pose row_to_pose_msg(const WaypointRow& row) {
  geometry_msgs::msg::Pose msg;
  msg.position.x = row.x_m;
  msg.position.y = row.y_m;
  msg.position.z = row.z_m;
  msg.orientation = rpy_deg_to_quaternion(row.roll_deg, row.pitch_deg, row.yaw_deg);
  return msg;
}

WaypointRow pose_msg_to_row(const geometry_msgs::msg::Pose& pose, double duration_s, bool stop) {
  const Vec3 p(pose.position.x, pose.position.y, pose.position.z);
  return make_row(p, quaternion_to_rpy_deg(pose.orientation), duration_s, stop);
}

void WaypointTable::add(const WaypointRow& row) {
  rows_.push_back(row);
  ++revision_;
}

bool WaypointTable::insert(std::size_t index, const WaypointRow& row) {
  if (index > rows_.size()) {
    return false;
  }
  rows_.insert(rows_.begin() + static_cast<std::ptrdiff_t>(index), row);
  ++revision_;
  return true;
}

bool WaypointTable::remove(std::size_t index) {
  if (index >= rows_.size()) {
    return false;
  }
  rows_.erase(rows_.begin() + static_cast<std::ptrdiff_t>(index));
  ++revision_;
  return true;
}

bool WaypointTable::move_up(std::size_t index) {
  if (index == 0 || index >= rows_.size()) {
    return false;
  }
  std::swap(rows_[index - 1], rows_[index]);
  ++revision_;
  return true;
}

bool WaypointTable::move_down(std::size_t index) {
  if (index + 1 >= rows_.size()) {
    return false;
  }
  std::swap(rows_[index], rows_[index + 1]);
  ++revision_;
  return true;
}

bool WaypointTable::update(std::size_t index, const WaypointRow& row) {
  if (index >= rows_.size()) {
    return false;
  }
  rows_[index] = row;
  ++revision_;
  return true;
}

cobot::common::Status WaypointTable::load(const std::filesystem::path& yaml,
                                          const std::string& base_link) {
  const cobot::pipeline::TcpOffset identity{};
  const cobot::common::Result<std::vector<cobot::planning::Waypoint>> loaded =
      cobot::pipeline::load_waypoints(yaml, base_link, identity);
  if (!loaded) {
    return cobot::common::Status::error(loaded.reason());
  }
  std::vector<WaypointRow> rows;
  rows.reserve(loaded.value().size());
  for (const cobot::planning::Waypoint& w : loaded.value()) {
    rows.push_back(pose_to_row(w.T_b_goal, w.duration_s, w.stop));
  }
  rows_ = std::move(rows);
  ++revision_;
  return cobot::common::Status::ok();
}

cobot::common::Status WaypointTable::save(const std::filesystem::path& yaml,
                                          const std::string& base_link) const {
  std::vector<cobot::planning::Waypoint> tcp_poses;
  tcp_poses.reserve(rows_.size());
  for (const WaypointRow& row : rows_) {
    tcp_poses.push_back(row_to_waypoint(row));
  }
  const cobot::pipeline::TcpOffset identity{};
  return cobot::pipeline::save_waypoints(yaml, base_link, tcp_poses, identity);
}

std::vector<cobot_interfaces::msg::Waypoint> WaypointTable::to_msg() const {
  std::vector<cobot_interfaces::msg::Waypoint> out;
  out.reserve(rows_.size());
  for (const WaypointRow& row : rows_) {
    cobot_interfaces::msg::Waypoint w;
    w.pose = row_to_pose_msg(row);
    w.duration_s = row.duration_s;
    w.stop = row.stop;
    out.push_back(w);
  }
  return out;
}

void WaypointTable::set_preview_result(std::uint64_t revision, bool ok) {
  preview_revision_ = revision;
  preview_ok_ = ok;
}

bool WaypointTable::run_enabled() const {
  return preview_ok_ && preview_revision_ == revision_ && !rows_.empty();
}

}  // namespace cobot_gui
