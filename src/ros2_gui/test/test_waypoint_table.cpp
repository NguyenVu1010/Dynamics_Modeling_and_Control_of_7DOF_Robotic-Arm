// Unit tests of the panel data model cobot_gui::WaypointTable (feature waypoint-gui,
// design docs/design/2026-10-29-waypoint-gui-design.md section 6.5). No Qt, no display.
//
// Contract (header ros2_gui/include/cobot_gui/waypoint_table.hpp, no Qt include):
//
//   namespace cobot_gui {
//   struct WaypointRow {          // one table row: a TCP pose in base_link
//     double x_m = 0.0, y_m = 0.0, z_m = 0.0;
//     double roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;  // R = Rz(yaw) Ry(pitch) Rx(roll)
//     double duration_s = 0.0;    // <= 0: auto
//     bool stop = false;
//   };
//   // Quaternions are Hamilton (w, x, y, z), as geometry_msgs.
//   geometry_msgs::msg::Quaternion rpy_deg_to_quaternion(double roll_deg, double pitch_deg,
//                                                        double yaw_deg);  // unit norm
//   // Normalises q first. roll and yaw in [-180, 180], pitch in [-90, 90].
//   std::array<double, 3> quaternion_to_rpy_deg(const geometry_msgs::msg::Quaternion& q);
//   cobot::common::Pose row_to_pose(const WaypointRow& row);
//   WaypointRow pose_to_row(const cobot::common::Pose& pose, double duration_s, bool stop);
//   geometry_msgs::msg::Pose row_to_pose_msg(const WaypointRow& row);
//   WaypointRow pose_msg_to_row(const geometry_msgs::msg::Pose& pose, double duration_s,
//                               bool stop);  // used by the interactive marker feedback
//
//   class WaypointTable {
//    public:
//     std::size_t size() const;
//     bool empty() const;
//     const WaypointRow& row(std::size_t i) const;            // precondition i < size()
//     const std::vector<WaypointRow>& rows() const;
//     // Edits. A call with an index out of range returns false and changes nothing (rows and
//     // revision). insert(i, r) puts r before row i, i == size() appends.
//     void add(const WaypointRow& row);                       // append
//     bool insert(std::size_t index, const WaypointRow& row);
//     bool remove(std::size_t index);
//     bool move_up(std::size_t index);                        // swap with index - 1
//     bool move_down(std::size_t index);                      // swap with index + 1
//     bool update(std::size_t index, const WaypointRow& row);
//     // Waypoint YAML file (docs/conventions.md "Waypoint file") through
//     // cobot::pipeline::load_waypoints / save_waypoints with an identity TcpOffset (the rows
//     // are TCP poses). Load replaces all rows; a failed load leaves the table (rows and
//     // revision) unchanged. Save is not an edit.
//     cobot::common::Status load(const std::filesystem::path& yaml, const std::string& base_link);
//     cobot::common::Status save(const std::filesystem::path& yaml,
//                                const std::string& base_link) const;
//     std::vector<cobot_interfaces::msg::Waypoint> to_msg() const;
//     // Run-enabled rule. revision() changes on every successful edit and load. The panel
//     // records revision() when it sends a preview and reports the answer with
//     // set_preview_result(that revision, response.ok). run_enabled() is true only when the
//     // last reported preview was ok, was for the current revision, and the table is not empty.
//     std::uint64_t revision() const;
//     void set_preview_result(std::uint64_t revision, bool ok);
//     bool run_enabled() const;
//   };
//   }  // namespace cobot_gui

#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <unistd.h>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/waypoint_file.hpp"

#include "cobot_gui/waypoint_table.hpp"
#include "cobot_interfaces/msg/waypoint.hpp"

namespace {

using cobot::common::Mat3;
using cobot::common::Vec3;
using cobot_gui::WaypointRow;
using cobot_gui::WaypointTable;

constexpr double kTol = 1e-9;
constexpr double kDegToRad = M_PI / 180.0;

WaypointRow make_row(double x, double y = 0.0, double z = 0.0) {
  WaypointRow r;
  r.x_m = x;
  r.y_m = y;
  r.z_m = z;
  return r;
}

WaypointRow full_row(const std::array<double, 3>& p,
                     const std::array<double, 3>& rpy_deg,
                     double duration_s,
                     bool stop) {
  WaypointRow r;
  r.x_m = p[0];
  r.y_m = p[1];
  r.z_m = p[2];
  r.roll_deg = rpy_deg[0];
  r.pitch_deg = rpy_deg[1];
  r.yaw_deg = rpy_deg[2];
  r.duration_s = duration_s;
  r.stop = stop;
  return r;
}

// R = Rz(yaw) Ry(pitch) Rx(roll), the convention of config/default.yaml goal_rpy_deg.
Mat3 rpy_matrix(double roll_deg, double pitch_deg, double yaw_deg) {
  const Mat3 rx = Eigen::AngleAxisd(roll_deg * kDegToRad, Vec3::UnitX()).toRotationMatrix();
  const Mat3 ry = Eigen::AngleAxisd(pitch_deg * kDegToRad, Vec3::UnitY()).toRotationMatrix();
  const Mat3 rz = Eigen::AngleAxisd(yaw_deg * kDegToRad, Vec3::UnitZ()).toRotationMatrix();
  return rz * ry * rx;
}

Mat3 quat_matrix(const geometry_msgs::msg::Quaternion& q) {
  const Eigen::Quaterniond e(q.w, q.x, q.y, q.z);  // Eigen constructor order (w, x, y, z)
  return e.normalized().toRotationMatrix();
}

double quat_norm(const geometry_msgs::msg::Quaternion& q) {
  return std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

// The x_m of every row, to read an ordering at a glance.
std::vector<double> xs(const WaypointTable& t) {
  std::vector<double> out;
  for (const WaypointRow& r : t.rows()) {
    out.push_back(r.x_m);
  }
  return out;
}

void expect_row_near(const WaypointRow& a, const WaypointRow& b, double tol) {
  EXPECT_NEAR(a.x_m, b.x_m, tol);
  EXPECT_NEAR(a.y_m, b.y_m, tol);
  EXPECT_NEAR(a.z_m, b.z_m, tol);
  EXPECT_NEAR(a.roll_deg, b.roll_deg, tol);
  EXPECT_NEAR(a.pitch_deg, b.pitch_deg, tol);
  EXPECT_NEAR(a.yaw_deg, b.yaw_deg, tol);
  EXPECT_NEAR(a.duration_s, b.duration_s, tol);
  EXPECT_EQ(a.stop, b.stop);
}

// A fresh file path per test under the system temp directory, removed at the end.
class TempFile {
 public:
  explicit TempFile(const std::string& name)
      : path_(std::filesystem::temp_directory_path() /
              ("cobot_gui_test_" + std::to_string(::getpid()) + "_" + name)) {
    std::error_code ec;
    std::filesystem::remove(path_, ec);
  }
  ~TempFile() {
    std::error_code ec;
    std::filesystem::remove(path_, ec);
  }
  TempFile(const TempFile&) = delete;
  TempFile& operator=(const TempFile&) = delete;
  TempFile(TempFile&&) = delete;
  TempFile& operator=(TempFile&&) = delete;
  const std::filesystem::path& path() const { return path_; }
  void write(const std::string& text) const {
    std::ofstream out(path_, std::ios::binary);
    out << text;
  }

 private:
  std::filesystem::path path_;
};

// ------------------------------------------------------------------ ordering

TEST(WaypointTable, StartsEmpty) {
  const WaypointTable t;
  EXPECT_TRUE(t.empty());
  EXPECT_EQ(t.size(), 0U);
  EXPECT_TRUE(t.rows().empty());
}

TEST(WaypointTable, AddAppendsInOrder) {
  WaypointTable t;
  t.add(make_row(1.0));
  t.add(make_row(2.0));
  t.add(make_row(3.0));
  EXPECT_EQ(t.size(), 3U);
  EXPECT_FALSE(t.empty());
  EXPECT_EQ(xs(t), (std::vector<double>{1.0, 2.0, 3.0}));
  EXPECT_DOUBLE_EQ(t.row(1).x_m, 2.0);
}

TEST(WaypointTable, InsertMoveRemoveUpdateOrdering) {
  WaypointTable t;
  t.add(make_row(1.0));  // A
  t.add(make_row(2.0));  // B
  t.add(make_row(3.0));  // C
  // insert before row 1: [A, D, B, C]
  ASSERT_TRUE(t.insert(1, make_row(4.0)));
  EXPECT_EQ(xs(t), (std::vector<double>{1.0, 4.0, 2.0, 3.0}));
  // insert at size() == 4 appends: [A, D, B, C, E]
  ASSERT_TRUE(t.insert(4, make_row(5.0)));
  EXPECT_EQ(xs(t), (std::vector<double>{1.0, 4.0, 2.0, 3.0, 5.0}));
  // insert at 0 prepends: [F, A, D, B, C, E]
  ASSERT_TRUE(t.insert(0, make_row(6.0)));
  EXPECT_EQ(xs(t), (std::vector<double>{6.0, 1.0, 4.0, 2.0, 3.0, 5.0}));
  // move_up(3) swaps rows 2 (D) and 3 (B): [F, A, B, D, C, E]
  ASSERT_TRUE(t.move_up(3));
  EXPECT_EQ(xs(t), (std::vector<double>{6.0, 1.0, 2.0, 4.0, 3.0, 5.0}));
  // move_down(0) swaps rows 0 (F) and 1 (A): [A, F, B, D, C, E]
  ASSERT_TRUE(t.move_down(0));
  EXPECT_EQ(xs(t), (std::vector<double>{1.0, 6.0, 2.0, 4.0, 3.0, 5.0}));
  // remove(3) drops D: [A, F, B, C, E]
  ASSERT_TRUE(t.remove(3));
  EXPECT_EQ(xs(t), (std::vector<double>{1.0, 6.0, 2.0, 3.0, 5.0}));
  // remove the last row (index 4, E): [A, F, B, C]
  ASSERT_TRUE(t.remove(4));
  EXPECT_EQ(xs(t), (std::vector<double>{1.0, 6.0, 2.0, 3.0}));
  // update(1) replaces F by G in place: [A, G, B, C]
  ASSERT_TRUE(t.update(1, make_row(7.0, 0.5, 0.25)));
  EXPECT_EQ(xs(t), (std::vector<double>{1.0, 7.0, 2.0, 3.0}));
  EXPECT_DOUBLE_EQ(t.row(1).y_m, 0.5);
  EXPECT_DOUBLE_EQ(t.row(1).z_m, 0.25);
}

TEST(WaypointTable, MovesCarryTheWholeRow) {
  WaypointTable t;
  const WaypointRow a = full_row({0.1, 0.2, 0.3}, {10.0, 20.0, 30.0}, 1.5, true);
  const WaypointRow b = full_row({-0.4, -0.5, 0.6}, {-5.0, 0.0, 90.0}, 0.0, false);
  t.add(a);
  t.add(b);
  ASSERT_TRUE(t.move_down(0));
  expect_row_near(t.row(0), b, 0.0);
  expect_row_near(t.row(1), a, 0.0);
  ASSERT_TRUE(t.move_up(1));
  expect_row_near(t.row(0), a, 0.0);
  expect_row_near(t.row(1), b, 0.0);
}

TEST(WaypointTable, OutOfRangeEditsChangeNothing) {
  WaypointTable t;
  t.add(make_row(1.0));
  t.add(make_row(2.0));
  const std::uint64_t rev = t.revision();
  EXPECT_FALSE(t.insert(3, make_row(9.0)));  // size() == 2, so 3 is past the end
  EXPECT_FALSE(t.remove(2));
  EXPECT_FALSE(t.move_up(0));    // first row has nothing above
  EXPECT_FALSE(t.move_down(1));  // last row has nothing below
  EXPECT_FALSE(t.move_up(5));
  EXPECT_FALSE(t.move_down(5));
  EXPECT_FALSE(t.update(2, make_row(9.0)));
  EXPECT_EQ(xs(t), (std::vector<double>{1.0, 2.0}));
  EXPECT_EQ(t.revision(), rev);
}

TEST(WaypointTable, EditsOnAnEmptyTableFail) {
  WaypointTable t;
  EXPECT_FALSE(t.remove(0));
  EXPECT_FALSE(t.move_up(0));
  EXPECT_FALSE(t.move_down(0));
  EXPECT_FALSE(t.update(0, make_row(1.0)));
  EXPECT_TRUE(t.insert(0, make_row(1.0)));  // 0 == size(): append
  EXPECT_EQ(xs(t), (std::vector<double>{1.0}));
  ASSERT_TRUE(t.remove(0));
  EXPECT_TRUE(t.empty());
}

// ------------------------------------------------------------------ rpy <-> quaternion

TEST(RpyQuaternion, KnownValues) {
  const double h = std::sqrt(0.5);  // cos(45 deg) = sin(45 deg)
  struct Case {
    std::array<double, 3> rpy_deg;
    std::array<double, 4> wxyz;
  };
  const std::array<Case, 5> cases{{
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}},
      // yaw 90: 90 deg about z -> (cos 45, 0, 0, sin 45)
      {{0.0, 0.0, 90.0}, {h, 0.0, 0.0, h}},
      // roll 90: about x -> (cos 45, sin 45, 0, 0)
      {{90.0, 0.0, 0.0}, {h, h, 0.0, 0.0}},
      // pitch -90: -90 deg about y -> (cos 45, 0, -sin 45, 0)
      {{0.0, -90.0, 0.0}, {h, 0.0, -h, 0.0}},
      // roll 90 and yaw 90: q = qz(90) * qx(90) = (h,0,0,h)(h,h,0,0)
      //   w = h*h - (0,0,h).(h,0,0) = 0.5
      //   v = h*(h,0,0) + h*(0,0,h) + (0,0,h)x(h,0,0) = (0.5,0,0.5) + (0,0.5,0) = (0.5,0.5,0.5)
      // (120 deg about (1,1,1)/sqrt(3): maps x to y, the first column of Rz(90) Rx(90)).
      {{90.0, 0.0, 90.0}, {0.5, 0.5, 0.5, 0.5}},
  }};
  for (const Case& c : cases) {
    const geometry_msgs::msg::Quaternion q =
        cobot_gui::rpy_deg_to_quaternion(c.rpy_deg[0], c.rpy_deg[1], c.rpy_deg[2]);
    // q and -q are the same rotation: compare up to sign.
    const double dot = q.w * c.wxyz[0] + q.x * c.wxyz[1] + q.y * c.wxyz[2] + q.z * c.wxyz[3];
    EXPECT_NEAR(std::abs(dot), 1.0, kTol)
        << "rpy " << c.rpy_deg[0] << " " << c.rpy_deg[1] << " " << c.rpy_deg[2];
    EXPECT_NEAR(quat_norm(q), 1.0, kTol);
  }
}

TEST(RpyQuaternion, QuaternionMatchesTheRpyConvention) {
  const std::array<std::array<double, 3>, 4> angles{{
      {10.0, -20.0, 30.0},
      {-170.0, 45.0, 120.0},
      {0.0, 0.0, 90.0},
      {179.0, -89.0, -179.0},
  }};
  for (const std::array<double, 3>& a : angles) {
    const Mat3 expected = rpy_matrix(a[0], a[1], a[2]);
    const Mat3 got = quat_matrix(cobot_gui::rpy_deg_to_quaternion(a[0], a[1], a[2]));
    EXPECT_LT((got - expected).cwiseAbs().maxCoeff(), kTol) << a[0] << " " << a[1] << " " << a[2];
  }
}

TEST(RpyQuaternion, RoundTripAwayFromGimbalLock) {
  // Canonical ranges: roll, yaw in [-180, 180], pitch in (-90, 90): the angles come back.
  const std::array<std::array<double, 3>, 6> angles{{
      {0.0, 0.0, 0.0},
      {10.0, -20.0, 30.0},
      {-170.0, 45.0, 120.0},
      {90.0, 0.0, 90.0},
      {-45.0, 85.0, -135.0},
      {179.5, -60.0, -179.5},
  }};
  for (const std::array<double, 3>& a : angles) {
    const std::array<double, 3> back =
        cobot_gui::quaternion_to_rpy_deg(cobot_gui::rpy_deg_to_quaternion(a[0], a[1], a[2]));
    EXPECT_NEAR(back[0], a[0], kTol);
    EXPECT_NEAR(back[1], a[1], kTol);
    EXPECT_NEAR(back[2], a[2], kTol);
  }
}

TEST(RpyQuaternion, GimbalLockKeepsTheRotation) {
  // pitch +-90: roll and yaw are not unique, the rotation is.
  for (const double pitch : {90.0, -90.0}) {
    const geometry_msgs::msg::Quaternion q = cobot_gui::rpy_deg_to_quaternion(30.0, pitch, 40.0);
    const std::array<double, 3> back = cobot_gui::quaternion_to_rpy_deg(q);
    ASSERT_TRUE(std::isfinite(back[0]) && std::isfinite(back[1]) && std::isfinite(back[2]));
    EXPECT_NEAR(back[1], pitch, 1e-5);  // asin near 1 is ill-conditioned: about 1e-6 deg
    const Mat3 again = rpy_matrix(back[0], back[1], back[2]);
    EXPECT_LT((again - rpy_matrix(30.0, pitch, 40.0)).cwiseAbs().maxCoeff(), 1e-6);
  }
}

TEST(RpyQuaternion, InputQuaternionIsNormalised) {
  // 2 * (h, 0, 0, h) is yaw 90 scaled by 2; the conversion normalises first.
  const double h = std::sqrt(0.5);
  geometry_msgs::msg::Quaternion q;
  q.w = 2.0 * h;
  q.x = 0.0;
  q.y = 0.0;
  q.z = 2.0 * h;
  const std::array<double, 3> rpy = cobot_gui::quaternion_to_rpy_deg(q);
  EXPECT_NEAR(rpy[0], 0.0, kTol);
  EXPECT_NEAR(rpy[1], 0.0, kTol);
  EXPECT_NEAR(rpy[2], 90.0, kTol);
  // -q is the same rotation.
  q.w = -h;
  q.z = -h;
  const std::array<double, 3> neg = cobot_gui::quaternion_to_rpy_deg(q);
  EXPECT_NEAR(neg[0], 0.0, kTol);
  EXPECT_NEAR(neg[1], 0.0, kTol);
  EXPECT_NEAR(neg[2], 90.0, kTol);
}

TEST(RowPose, RowToPoseUsesTheRpyConvention) {
  const WaypointRow r = full_row({-0.5, -0.6, 0.9}, {10.0, -20.0, 30.0}, 2.0, true);
  const cobot::common::Pose T = cobot_gui::row_to_pose(r);
  EXPECT_LT((T.p - Vec3(-0.5, -0.6, 0.9)).cwiseAbs().maxCoeff(), kTol);
  EXPECT_LT((T.R - rpy_matrix(10.0, -20.0, 30.0)).cwiseAbs().maxCoeff(), kTol);
  const WaypointRow back = cobot_gui::pose_to_row(T, 2.0, true);
  expect_row_near(back, r, kTol);
}

TEST(RowPose, PoseMsgRoundTrip) {
  // The interactive marker reports a geometry_msgs Pose; dragging must give back the row.
  const WaypointRow r = full_row({0.25, -0.75, 1.125}, {-170.0, 45.0, 120.0}, 3.0, false);
  const geometry_msgs::msg::Pose m = cobot_gui::row_to_pose_msg(r);
  EXPECT_DOUBLE_EQ(m.position.x, 0.25);
  EXPECT_DOUBLE_EQ(m.position.y, -0.75);
  EXPECT_DOUBLE_EQ(m.position.z, 1.125);
  EXPECT_NEAR(quat_norm(m.orientation), 1.0, kTol);
  EXPECT_LT((quat_matrix(m.orientation) - rpy_matrix(-170.0, 45.0, 120.0)).cwiseAbs().maxCoeff(),
            kTol);
  expect_row_near(cobot_gui::pose_msg_to_row(m, 3.0, false), r, kTol);
}

// ------------------------------------------------------------------ to msg

TEST(WaypointTable, ToMsgMapsEveryField) {
  WaypointTable t;
  t.add(full_row({-0.5, -0.6, 0.9}, {0.0, 0.0, 90.0}, 0.0, false));
  t.add(full_row({-0.3, -0.6, 0.7}, {10.0, -20.0, 30.0}, 4.0, true));
  const std::vector<cobot_interfaces::msg::Waypoint> msgs = t.to_msg();
  ASSERT_EQ(msgs.size(), 2U);

  EXPECT_DOUBLE_EQ(msgs[0].pose.position.x, -0.5);
  EXPECT_DOUBLE_EQ(msgs[0].pose.position.y, -0.6);
  EXPECT_DOUBLE_EQ(msgs[0].pose.position.z, 0.9);
  // yaw 90 -> (w, x, y, z) = +-(h, 0, 0, h), h = sqrt(0.5); dot with (h, 0, 0, h) is +-1.
  const double h = std::sqrt(0.5);
  const geometry_msgs::msg::Quaternion& q0 = msgs[0].pose.orientation;
  EXPECT_NEAR(std::abs(q0.w * h + q0.z * h), 1.0, kTol);
  EXPECT_NEAR(quat_norm(q0), 1.0, kTol);
  EXPECT_DOUBLE_EQ(msgs[0].duration_s, 0.0);
  EXPECT_FALSE(msgs[0].stop);

  EXPECT_DOUBLE_EQ(msgs[1].pose.position.x, -0.3);
  EXPECT_DOUBLE_EQ(msgs[1].pose.position.y, -0.6);
  EXPECT_DOUBLE_EQ(msgs[1].pose.position.z, 0.7);
  EXPECT_NEAR(quat_norm(msgs[1].pose.orientation), 1.0, kTol);
  const Mat3 rot1 = quat_matrix(msgs[1].pose.orientation);
  EXPECT_LT((rot1 - rpy_matrix(10.0, -20.0, 30.0)).cwiseAbs().maxCoeff(), kTol);
  EXPECT_DOUBLE_EQ(msgs[1].duration_s, 4.0);
  EXPECT_TRUE(msgs[1].stop);
}

TEST(WaypointTable, ToMsgOfAnEmptyTableIsEmpty) {
  const WaypointTable t;
  EXPECT_TRUE(t.to_msg().empty());
}

// ------------------------------------------------------------------ YAML through the core

TEST(WaypointTableYaml, LoadsTheDesignExampleFile) {
  // design 6.6 example plus one row with all three angles, a user duration and no stop key.
  const TempFile file("design_example.yaml");
  file.write(
      "frame: base_link\n"
      "waypoints:\n"
      "  - {position_m: [-0.5, -0.6, 0.9], rpy_deg: [0, 0, 90], duration_s: 0, stop: false}\n"
      "  - {position_m: [-0.3, -0.6, 0.7], rpy_deg: [0, 0, 90], duration_s: 4.0, stop: true}\n"
      "  - {position_m: [0.1, 0.2, 0.3], rpy_deg: [10, -20, 30], duration_s: 1.5}\n");
  WaypointTable t;
  const cobot::common::Status s = t.load(file.path(), "base_link");
  ASSERT_TRUE(s.is_ok()) << s.reason();
  ASSERT_EQ(t.size(), 3U);
  expect_row_near(t.row(0), full_row({-0.5, -0.6, 0.9}, {0.0, 0.0, 90.0}, 0.0, false), kTol);
  expect_row_near(t.row(1), full_row({-0.3, -0.6, 0.7}, {0.0, 0.0, 90.0}, 4.0, true), kTol);
  expect_row_near(t.row(2), full_row({0.1, 0.2, 0.3}, {10.0, -20.0, 30.0}, 1.5, false), kTol);
}

TEST(WaypointTableYaml, SaveThenLoadRoundTrip) {
  WaypointTable t;
  t.add(full_row({-0.5, -0.6, 0.9}, {0.0, 0.0, 90.0}, 0.0, false));
  t.add(full_row({-0.3, -0.6, 0.7}, {10.0, -20.0, 30.0}, 4.0, true));
  t.add(full_row({0.125, 0.25, 0.5}, {-170.0, 45.0, 120.0}, 2.5, false));
  const TempFile file("round_trip.yaml");
  const cobot::common::Status saved = t.save(file.path(), "base_link");
  ASSERT_TRUE(saved.is_ok()) << saved.reason();

  WaypointTable back;
  const cobot::common::Status loaded = back.load(file.path(), "base_link");
  ASSERT_TRUE(loaded.is_ok()) << loaded.reason();
  ASSERT_EQ(back.size(), t.size());
  for (std::size_t i = 0; i < t.size(); ++i) {
    SCOPED_TRACE(i);
    expect_row_near(back.row(i), t.row(i), kTol);
  }
}

TEST(WaypointTableYaml, SavedFileIsReadByTheCoreLoaderAsTcpPoses) {
  // Identity tcp offset: the core loader returns exactly the row poses.
  WaypointTable t;
  t.add(full_row({-0.5, -0.6, 0.9}, {0.0, 0.0, 90.0}, 0.0, false));
  t.add(full_row({-0.3, -0.6, 0.7}, {10.0, -20.0, 30.0}, 4.0, true));
  const TempFile file("core_reads.yaml");
  ASSERT_TRUE(t.save(file.path(), "base_link").is_ok());

  const cobot::pipeline::TcpOffset identity{};
  const cobot::common::Result<std::vector<cobot::planning::Waypoint>> loaded =
      cobot::pipeline::load_waypoints(file.path(), "base_link", identity);
  ASSERT_TRUE(loaded.is_ok()) << loaded.reason();
  ASSERT_EQ(loaded.value().size(), 2U);
  for (std::size_t i = 0; i < 2; ++i) {
    SCOPED_TRACE(i);
    const cobot::planning::Waypoint& w = loaded.value()[i];
    const cobot::common::Pose expected = cobot_gui::row_to_pose(t.row(i));
    EXPECT_LT((w.T_b_goal.p - expected.p).cwiseAbs().maxCoeff(), kTol);
    EXPECT_LT((w.T_b_goal.R - expected.R).cwiseAbs().maxCoeff(), kTol);
    EXPECT_NEAR(w.duration_s, t.row(i).duration_s, kTol);
    EXPECT_EQ(w.stop, t.row(i).stop);
  }
}

TEST(WaypointTableYaml, FileWrittenByTheCoreLoadsIntoTheTable) {
  std::vector<cobot::planning::Waypoint> tip;
  cobot::planning::Waypoint w;
  w.T_b_goal.p = Vec3(0.2, -0.4, 0.6);
  w.T_b_goal.R = rpy_matrix(-30.0, 15.0, 60.0);
  w.duration_s = 3.0;
  w.stop = true;
  tip.push_back(w);
  const TempFile file("core_writes.yaml");
  const cobot::pipeline::TcpOffset identity{};
  ASSERT_TRUE(cobot::pipeline::save_waypoints(file.path(), "base_link", tip, identity).is_ok());

  WaypointTable t;
  const cobot::common::Status s = t.load(file.path(), "base_link");
  ASSERT_TRUE(s.is_ok()) << s.reason();
  ASSERT_EQ(t.size(), 1U);
  expect_row_near(t.row(0), full_row({0.2, -0.4, 0.6}, {-30.0, 15.0, 60.0}, 3.0, true), kTol);
}

TEST(WaypointTableYaml, FailedLoadLeavesTheTableUnchanged) {
  WaypointTable t;
  t.add(make_row(1.0));
  t.add(make_row(2.0));
  const std::uint64_t rev = t.revision();

  const TempFile wrong_frame("wrong_frame.yaml");
  wrong_frame.write(
      "frame: world\n"
      "waypoints:\n"
      "  - {position_m: [0.1, 0.2, 0.3], rpy_deg: [0, 0, 0]}\n");
  const cobot::common::Status s1 = t.load(wrong_frame.path(), "base_link");
  EXPECT_FALSE(s1.is_ok());
  EXPECT_FALSE(s1.reason().empty());

  const TempFile bad_entry("bad_entry.yaml");
  bad_entry.write(
      "frame: base_link\n"
      "waypoints:\n"
      "  - {position_m: [0.1, 0.2], rpy_deg: [0, 0, 0]}\n");
  EXPECT_FALSE(t.load(bad_entry.path(), "base_link").is_ok());

  const TempFile missing("does_not_exist.yaml");
  EXPECT_FALSE(t.load(missing.path(), "base_link").is_ok());

  EXPECT_EQ(xs(t), (std::vector<double>{1.0, 2.0}));
  EXPECT_EQ(t.revision(), rev);
}

TEST(WaypointTableYaml, LoadReplacesTheRows) {
  WaypointTable t;
  t.add(make_row(9.0));
  t.add(make_row(8.0));
  const TempFile file("replace.yaml");
  file.write(
      "frame: base_link\n"
      "waypoints:\n"
      "  - {position_m: [0.5, 0.0, 0.5], rpy_deg: [0, 0, 0]}\n");
  const cobot::common::Status s = t.load(file.path(), "base_link");
  ASSERT_TRUE(s.is_ok()) << s.reason();
  ASSERT_EQ(t.size(), 1U);
  expect_row_near(t.row(0), full_row({0.5, 0.0, 0.5}, {0.0, 0.0, 0.0}, 0.0, false), kTol);
}

// ------------------------------------------------------------------ Run-enabled rule

TEST(RunEnabled, DisabledUntilAnOkPreviewOfTheCurrentList) {
  WaypointTable t;
  EXPECT_FALSE(t.run_enabled());
  t.add(make_row(1.0));
  EXPECT_FALSE(t.run_enabled());  // no preview yet
  t.set_preview_result(t.revision(), false);
  EXPECT_FALSE(t.run_enabled());  // preview found issues
  t.set_preview_result(t.revision(), true);
  EXPECT_TRUE(t.run_enabled());
  t.set_preview_result(t.revision(), false);
  EXPECT_FALSE(t.run_enabled());  // the latest answer counts
}

TEST(RunEnabled, EmptyListCannotRun) {
  WaypointTable t;
  t.set_preview_result(t.revision(), true);
  EXPECT_FALSE(t.run_enabled());
}

TEST(RunEnabled, EveryEditInvalidatesAnOkPreview) {
  // Each edit kind on a fresh 3-row table with an ok preview disables Run.
  struct Edit {
    const char* name;
    bool (*apply)(WaypointTable&);
  };
  const std::array<Edit, 6> edits{{
      {"add",
       [](WaypointTable& t) {
         t.add(make_row(4.0));
         return true;
       }},
      {"insert", [](WaypointTable& t) { return t.insert(1, make_row(4.0)); }},
      {"remove", [](WaypointTable& t) { return t.remove(2); }},
      {"move_up", [](WaypointTable& t) { return t.move_up(1); }},
      {"move_down", [](WaypointTable& t) { return t.move_down(1); }},
      {"update", [](WaypointTable& t) { return t.update(0, make_row(1.0, 0.1)); }},
  }};
  for (const Edit& e : edits) {
    SCOPED_TRACE(e.name);
    WaypointTable t;
    t.add(make_row(1.0));
    t.add(make_row(2.0));
    t.add(make_row(3.0));
    const std::uint64_t before = t.revision();
    t.set_preview_result(before, true);
    ASSERT_TRUE(t.run_enabled());
    ASSERT_TRUE(e.apply(t));
    EXPECT_NE(t.revision(), before);
    EXPECT_FALSE(t.run_enabled());
    // A new ok preview of the edited list enables Run again.
    t.set_preview_result(t.revision(), true);
    EXPECT_TRUE(t.run_enabled());
  }
}

TEST(RunEnabled, StalePreviewAnswerDoesNotEnable) {
  // Preview sent for one revision, the list is edited before the answer arrives.
  WaypointTable t;
  t.add(make_row(1.0));
  const std::uint64_t sent = t.revision();
  t.add(make_row(2.0));
  t.set_preview_result(sent, true);
  EXPECT_FALSE(t.run_enabled());
}

TEST(RunEnabled, EditBackToTheSameListStillNeedsAPreview) {
  WaypointTable t;
  t.add(make_row(1.0));
  t.add(make_row(2.0));
  t.set_preview_result(t.revision(), true);
  ASSERT_TRUE(t.move_down(0));
  ASSERT_TRUE(t.move_up(1));  // same rows as before, but edited
  EXPECT_FALSE(t.run_enabled());
}

TEST(RunEnabled, FailedEditKeepsRunEnabled) {
  WaypointTable t;
  t.add(make_row(1.0));
  t.set_preview_result(t.revision(), true);
  EXPECT_FALSE(t.remove(3));
  EXPECT_FALSE(t.move_up(0));
  EXPECT_FALSE(t.update(1, make_row(2.0)));
  EXPECT_TRUE(t.run_enabled());
}

TEST(RunEnabled, LoadInvalidatesAndFailedLoadDoesNot) {
  WaypointTable t;
  t.add(make_row(1.0));
  t.set_preview_result(t.revision(), true);
  ASSERT_TRUE(t.run_enabled());

  const TempFile missing("run_missing.yaml");
  EXPECT_FALSE(t.load(missing.path(), "base_link").is_ok());
  EXPECT_TRUE(t.run_enabled());

  const TempFile file("run_load.yaml");
  file.write(
      "frame: base_link\n"
      "waypoints:\n"
      "  - {position_m: [0.5, 0.0, 0.5], rpy_deg: [0, 0, 0]}\n");
  const cobot::common::Status s = t.load(file.path(), "base_link");
  ASSERT_TRUE(s.is_ok()) << s.reason();
  EXPECT_FALSE(t.run_enabled());
}

TEST(RunEnabled, SaveIsNotAnEdit) {
  WaypointTable t;
  t.add(make_row(0.5, 0.0, 0.5));
  t.set_preview_result(t.revision(), true);
  const TempFile file("run_save.yaml");
  const cobot::common::Status s = t.save(file.path(), "base_link");
  ASSERT_TRUE(s.is_ok()) << s.reason();
  EXPECT_TRUE(t.run_enabled());
}

}  // namespace
