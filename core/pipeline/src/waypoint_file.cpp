#include "cobot/pipeline/waypoint_file.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

#include "cobot/common/types.hpp"

// save_waypoints; load_waypoints lives in config.cpp next to the strict YAML reader.
namespace cobot::pipeline {
namespace {

using common::Mat3;
using common::Vec3;

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
constexpr double kGimbalEps = 1e-12;  // cos(pitch) below this: roll and yaw share one axis

// (roll, pitch, yaw) of R = Rz(yaw) Ry(pitch) Rx(roll), rad; roll = 0 at the gimbal lock.
Vec3 matrix_to_rpy(const Mat3& R) {
  const double cp = std::hypot(R(0, 0), R(1, 0));
  const double pitch = std::atan2(-R(2, 0), cp);
  if (cp < kGimbalEps) {
    return {0.0, pitch, std::atan2(-R(0, 1), R(1, 1))};
  }
  return {std::atan2(R(2, 1), R(2, 2)), pitch, std::atan2(R(1, 0), R(0, 0))};
}

std::string num(double v) {
  std::array<char, 32> buf{};
  std::snprintf(buf.data(), buf.size(), "%.17g", v);
  return buf.data();
}

std::string triple(const Vec3& v) {
  return "[" + num(v.x()) + ", " + num(v.y()) + ", " + num(v.z()) + "]";
}

}  // namespace

common::Status save_waypoints(const std::filesystem::path& yaml,
                              const std::string& base_link,
                              const std::vector<planning::Waypoint>& tip_waypoints,
                              const TcpOffset& tcp) {
  std::string text = "frame: " + base_link + "\nwaypoints:\n";
  for (const planning::Waypoint& w : tip_waypoints) {
    const common::Pose T_b_tcp = common::compose(w.T_b_goal, tcp.T_tip_tcp);
    const Vec3 rpy_deg = matrix_to_rpy(T_b_tcp.R) * kRadToDeg;
    text += "  - {position_m: " + triple(T_b_tcp.p) + ", rpy_deg: " + triple(rpy_deg) +
            ", duration_s: " + num(w.duration_s) + ", stop: " + (w.stop ? "true" : "false") + "}\n";
  }
  std::ofstream out(yaml, std::ios::binary);
  out << text;
  out.close();
  if (!out) {
    return common::Status::error("cannot write " + yaml.string());
  }
  return common::Status::ok();
}

}  // namespace cobot::pipeline
