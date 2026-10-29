#include <cstddef>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/lie.hpp"
#include "cobot/common/types.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/waypoint_file.hpp"
#include "cobot/planning/waypoint_trajectory.hpp"

#include "control_loop_test_util.hpp"

// Waypoint file (feature `loop-external`, design 6.6, docs/conventions.md "Waypoint file"):
//   frame: base_link
//   waypoints:
//     - {position_m: [x, y, z], rpy_deg: [r, p, y], duration_s: 0, stop: false}
// Poses in the file are TCP poses; the loader returns tip poses T_b_tip = T_b_tcp T_tip_tcp^-1
// and the writer stores T_b_tcp = T_b_tip T_tip_tcp.
namespace cobot::pipeline::test {
namespace {

namespace fs = std::filesystem;
using planning::Waypoint;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

fs::path out_dir() {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "loop_external_waypoints";
  fs::create_directories(dir);
  return dir;
}

fs::path write_temp(const std::string& name, const std::string& text) {
  const fs::path p = out_dir() / name;
  std::ofstream(p, std::ios::binary) << text;
  return p;
}

Waypoint wp(const Mat3& R, const Vec3& p, double duration_s, bool stop) {
  Waypoint w;
  w.T_b_goal = Pose{R, p};
  w.duration_s = duration_s;
  w.stop = stop;
  return w;
}

TcpOffset tcp_of(const Mat3& R, const Vec3& p) {
  TcpOffset t;
  t.T_tip_tcp = Pose{R, p};
  return t;
}

Mat3 rz(double yaw) {
  return rpy_matrix(Vec3(0.0, 0.0, yaw));
}

// Four tip waypoints: the design example, a general rpy, a general rotation vector (angle
// |(0.3, -1.2, 2.5)| = sqrt(7.78) = 2.789 rad) and a 180 deg yaw.
std::vector<Waypoint> sample_waypoints() {
  return {
      wp(rz(90.0 * kDeg), Vec3(-0.5, -0.6, 0.9), 0.0, false),
      wp(rpy_matrix(Vec3(10.0, -20.0, 30.0) * kDeg), Vec3(-0.3, -0.6, 0.7), 4.0, true),
      wp(common::so3_exp(Vec3(0.3, -1.2, 2.5)), Vec3(0.123456789012, -0.7, 0.55), 1.25, false),
      wp(rz(kPi), Vec3(0.2, -0.5, 0.4), 0.5, true),
  };
}

void expect_same(const std::vector<Waypoint>& a, const std::vector<Waypoint>& b, double tol) {
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_LT((a[i].T_b_goal.p - b[i].T_b_goal.p).cwiseAbs().maxCoeff(), tol) << "waypoint " << i;
    EXPECT_LT((a[i].T_b_goal.R - b[i].T_b_goal.R).cwiseAbs().maxCoeff(), tol) << "waypoint " << i;
    EXPECT_NEAR(a[i].duration_s, b[i].duration_s, tol) << "waypoint " << i;
    EXPECT_EQ(a[i].stop, b[i].stop) << "waypoint " << i;
  }
}

// ------------------------------------------------------------------ round trip

TEST(WaypointFile, RoundTripWithoutTool) {
  const std::vector<Waypoint> tip = sample_waypoints();
  const fs::path file = out_dir() / "round_trip.yaml";
  const common::Status saved = save_waypoints(file, "base_link", tip, TcpOffset{});
  ASSERT_TRUE(saved.is_ok()) << saved.reason();
  const auto loaded = load_waypoints(file, "base_link", TcpOffset{});
  ASSERT_TRUE(loaded.is_ok()) << loaded.reason();
  expect_same(loaded.value(), tip, 1e-9);
}

// With a tool: save(tip, tcp) then load(tcp) gives the tip poses back, and the file holds the
// TCP poses (loading it without the tool gives T_b_tip T_tip_tcp).
TEST(WaypointFile, RoundTripWithToolAndTheFileHoldsTcpPoses) {
  const std::vector<Waypoint> tip = sample_waypoints();
  const TcpOffset tcp = tcp_of(rpy_matrix(Vec3(5.0, -10.0, 90.0) * kDeg), Vec3(0.02, -0.01, 0.1));
  const fs::path file = out_dir() / "round_trip_tool.yaml";
  ASSERT_TRUE(save_waypoints(file, "base_link", tip, tcp).is_ok());

  const auto loaded = load_waypoints(file, "base_link", tcp);
  ASSERT_TRUE(loaded.is_ok()) << loaded.reason();
  expect_same(loaded.value(), tip, 1e-9);

  const auto raw = load_waypoints(file, "base_link", TcpOffset{});
  ASSERT_TRUE(raw.is_ok()) << raw.reason();
  std::vector<Waypoint> tcp_poses = tip;
  for (Waypoint& w : tcp_poses) {
    w.T_b_goal = common::compose(w.T_b_goal, tcp.T_tip_tcp);
  }
  expect_same(raw.value(), tcp_poses, 1e-9);
}

// ------------------------------------------------------------------ tcp offset, by hand

// File pose T_b_tcp = (Rz(90 deg), p = (0.4, -0.2, 0.6)); Rz(90 deg) maps (x, y, z) to
// (-y, x, z).
//  - tool (I, (0.1, 0, 0)): T_tip_tcp^-1 = (I, (-0.1, 0, 0)); T_b_tip = (Rz(90),
//    p + Rz(90) (-0.1, 0, 0)) = (Rz(90), (0.4, -0.2, 0.6) + (0, -0.1, 0)) = (Rz(90), (0.4, -0.3,
//    0.6)).
//  - tool (Rz(90), (0, 0, 0.1)): T_tip_tcp^-1 = (Rz(-90), -Rz(-90) (0, 0, 0.1)) =
//    (Rz(-90), (0, 0, -0.1)); T_b_tip = (Rz(90) Rz(-90), p + Rz(90) (0, 0, -0.1)) =
//    (I, (0.4, -0.2, 0.5)).
// duration_s and stop are optional: 0 and false.
TEST(WaypointFile, LoaderAppliesTheToolOffset) {
  const fs::path file = write_temp("tool.yaml",
                                   "frame: base_link\n"
                                   "waypoints:\n"
                                   "  - {position_m: [0.4, -0.2, 0.6], rpy_deg: [0, 0, 90]}\n");
  const auto a = load_waypoints(file, "base_link", tcp_of(Mat3::Identity(), Vec3(0.1, 0.0, 0.0)));
  ASSERT_TRUE(a.is_ok()) << a.reason();
  ASSERT_EQ(a.value().size(), 1U);
  EXPECT_LT((a.value()[0].T_b_goal.p - Vec3(0.4, -0.3, 0.6)).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT((a.value()[0].T_b_goal.R - rz(90.0 * kDeg)).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_EQ(a.value()[0].duration_s, 0.0);
  EXPECT_FALSE(a.value()[0].stop);

  const auto b = load_waypoints(file, "base_link", tcp_of(rz(90.0 * kDeg), Vec3(0.0, 0.0, 0.1)));
  ASSERT_TRUE(b.is_ok()) << b.reason();
  ASSERT_EQ(b.value().size(), 1U);
  EXPECT_LT((b.value()[0].T_b_goal.p - Vec3(0.4, -0.2, 0.5)).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT((b.value()[0].T_b_goal.R - Mat3::Identity()).cwiseAbs().maxCoeff(), 1e-12);
}

// Tip (I, (0.3, 0.2, 0.5)) with tool (I, (0, 0, 0.1)): the file gets T_b_tcp = (I, (0.3, 0.2,
// 0.6)), read back without the tool.
TEST(WaypointFile, WriterAppliesTheToolOffset) {
  const fs::path file = out_dir() / "writer_tool.yaml";
  const std::vector<Waypoint> tip = {wp(Mat3::Identity(), Vec3(0.3, 0.2, 0.5), 2.0, true)};
  ASSERT_TRUE(save_waypoints(file, "base_link", tip, tcp_of(Mat3::Identity(), Vec3(0.0, 0.0, 0.1)))
                  .is_ok());
  const auto raw = load_waypoints(file, "base_link", TcpOffset{});
  ASSERT_TRUE(raw.is_ok()) << raw.reason();
  ASSERT_EQ(raw.value().size(), 1U);
  EXPECT_LT((raw.value()[0].T_b_goal.p - Vec3(0.3, 0.2, 0.6)).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT((raw.value()[0].T_b_goal.R - Mat3::Identity()).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_EQ(raw.value()[0].duration_s, 2.0);
  EXPECT_TRUE(raw.value()[0].stop);
}

// rpy_deg follows the config convention R = Rz(yaw) Ry(pitch) Rx(roll), in degrees.
TEST(WaypointFile, RpyIsRollPitchYawInDegrees) {
  const fs::path file = write_temp(
      "rpy.yaml",
      "frame: base_link\n"
      "waypoints:\n"
      "  - {position_m: [0.1, 0.2, 0.3], rpy_deg: [10, 20, 30], duration_s: 1.5, stop: true}\n"
      "  - position_m: [-0.1, -0.2, 0.3]\n"
      "    rpy_deg: [-45, 0, 180]\n");
  const auto r = load_waypoints(file, "base_link", TcpOffset{});
  ASSERT_TRUE(r.is_ok()) << r.reason();
  ASSERT_EQ(r.value().size(), 2U);
  EXPECT_LT(
      (r.value()[0].T_b_goal.R - rpy_matrix(Vec3(10.0, 20.0, 30.0) * kDeg)).cwiseAbs().maxCoeff(),
      1e-12);
  EXPECT_EQ(r.value()[0].T_b_goal.p, Vec3(0.1, 0.2, 0.3));
  EXPECT_EQ(r.value()[0].duration_s, 1.5);
  EXPECT_TRUE(r.value()[0].stop);
  EXPECT_LT(
      (r.value()[1].T_b_goal.R - rpy_matrix(Vec3(-45.0, 0.0, 180.0) * kDeg)).cwiseAbs().maxCoeff(),
      1e-12);
  EXPECT_EQ(r.value()[1].duration_s, 0.0);
  EXPECT_FALSE(r.value()[1].stop);
}

// ------------------------------------------------------------------ errors

TEST(WaypointFile, FrameMustBeTheBaseLink) {
  const fs::path world = write_temp("frame_world.yaml",
                                    "frame: world\n"
                                    "waypoints:\n"
                                    "  - {position_m: [0.4, -0.2, 0.6], rpy_deg: [0, 0, 90]}\n");
  const auto r = load_waypoints(world, "base_link", TcpOffset{});
  ASSERT_FALSE(r.is_ok());
  EXPECT_NE(r.reason().find("frame"), std::string::npos) << r.reason();
  EXPECT_NE(r.reason().find("frame_world.yaml"), std::string::npos) << r.reason();

  const fs::path none = write_temp("frame_missing.yaml",
                                   "waypoints:\n"
                                   "  - {position_m: [0.4, -0.2, 0.6], rpy_deg: [0, 0, 90]}\n");
  const auto m = load_waypoints(none, "base_link", TcpOffset{});
  ASSERT_FALSE(m.is_ok());
  EXPECT_NE(m.reason().find("frame"), std::string::npos) << m.reason();

  // A file saved for one base link does not load for another.
  const fs::path saved = out_dir() / "frame_saved.yaml";
  ASSERT_TRUE(save_waypoints(saved, "base_link", sample_waypoints(), TcpOffset{}).is_ok());
  const auto other = load_waypoints(saved, "world", TcpOffset{});
  ASSERT_FALSE(other.is_ok());
  EXPECT_NE(other.reason().find("frame"), std::string::npos) << other.reason();
}

TEST(WaypointFile, MissingWaypointsListIsNamed) {
  const fs::path file = write_temp("no_list.yaml", "frame: base_link\n");
  const auto r = load_waypoints(file, "base_link", TcpOffset{});
  ASSERT_FALSE(r.is_ok());
  EXPECT_NE(r.reason().find("waypoints"), std::string::npos) << r.reason();

  const fs::path scalar = write_temp("scalar_list.yaml", "frame: base_link\nwaypoints: 3\n");
  const auto s = load_waypoints(scalar, "base_link", TcpOffset{});
  ASSERT_FALSE(s.is_ok());
  EXPECT_NE(s.reason().find("waypoints"), std::string::npos) << s.reason();
}

// A second waypoint replaced by `entry`, and the key the message must name (with the file).
struct BadEntry {
  const char* entry;
  const char* key;
};

TEST(WaypointFile, BadEntriesAreNamed) {
  const std::vector<BadEntry> cases = {
      {"{position_m: [0.1, 0.2], rpy_deg: [0, 0, 0]}", "waypoints[1].position_m"},
      {"{position_m: [0.1, .nan, 0.2], rpy_deg: [0, 0, 0]}", "waypoints[1].position_m"},
      {"{position_m: [0.1, 0.2, x], rpy_deg: [0, 0, 0]}", "waypoints[1].position_m"},
      {"{rpy_deg: [0, 0, 0]}", "waypoints[1].position_m"},
      {"{position_m: [0.1, 0.2, 0.3], rpy_deg: [0, 0]}", "waypoints[1].rpy_deg"},
      {"{position_m: [0.1, 0.2, 0.3], rpy_deg: [a, b, c]}", "waypoints[1].rpy_deg"},
      {"{position_m: [0.1, 0.2, 0.3]}", "waypoints[1].rpy_deg"},
      {"{position_m: [0.1, 0.2, 0.3], rpy_deg: [0, 0, 0], duration_s: abc}",
       "waypoints[1].duration_s"},
      {"{position_m: [0.1, 0.2, 0.3], rpy_deg: [0, 0, 0], duration_s: .inf}",
       "waypoints[1].duration_s"},
      {"{position_m: [0.1, 0.2, 0.3], rpy_deg: [0, 0, 0], stop: maybe}", "waypoints[1].stop"},
      {"{position_m: [0.1, 0.2, 0.3], rpy_deg: [0, 0, 0], speed: 1.0}", "waypoints[1].speed"},
  };
  int n = 0;
  for (const auto& bad : cases) {
    const std::string name = "bad_entry" + std::to_string(n++) + ".yaml";
    const fs::path file =
        write_temp(name,
                   std::string("frame: base_link\n"
                               "waypoints:\n"
                               "  - {position_m: [0.4, -0.2, 0.6], rpy_deg: [0, 0, 90]}\n"
                               "  - ") +
                       bad.entry + "\n");
    const auto r = load_waypoints(file, "base_link", TcpOffset{});
    ASSERT_FALSE(r.is_ok()) << bad.entry;
    EXPECT_NE(r.reason().find(bad.key), std::string::npos) << r.reason();
    EXPECT_NE(r.reason().find(name), std::string::npos) << r.reason();
  }
}

TEST(WaypointFile, MissingFileNamesThePath) {
  const auto r = load_waypoints(out_dir() / "does_not_exist.yaml", "base_link", TcpOffset{});
  ASSERT_FALSE(r.is_ok());
  EXPECT_NE(r.reason().find("does_not_exist.yaml"), std::string::npos) << r.reason();
}

TEST(WaypointFile, SaveIntoAMissingDirectoryFails) {
  const fs::path file = out_dir() / "no_such_dir" / "w.yaml";
  const common::Status st = save_waypoints(file, "base_link", sample_waypoints(), TcpOffset{});
  EXPECT_FALSE(st.is_ok());
}

}  // namespace
}  // namespace cobot::pipeline::test
