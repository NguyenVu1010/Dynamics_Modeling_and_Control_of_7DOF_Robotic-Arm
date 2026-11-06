// The Control tab of cobot_studio (feature studio-control, cobot_studio design 5.3, 5.4, 6.7, 6.8,
// 6.9, 7; ADR-021, ADR-022), driven through the object names of the brief's UI contract against
// bench::serve on a thread of this test (config/default.yaml with absolute paths and 5 s Recorder
// buffers + the ideal profile, region /cobot_cobot_test_control_<pid>). A raw
// bridge::ClientEndpoint (never the owner) watches the server: its state ring, session and owner.
//
// Besides the brief's object names, this test pins the window constructor that names the bridge
// (main.cpp --bridge <name>, default cobot_controller):
//   MainWindow(std::filesystem::path config_dir, std::filesystem::path work_dir,
//              std::string bridge_name /* "cobot_controller" -> /cobot_cobot_controller */);
// and that widgets are found by object name and base class only: values are read from a QLabel,
// QLineEdit or spin box (the first number of the text, in the unit the brief gives), the
// waypoint table is a QAbstractItemView, the text widgets (statusLine, previewIssues) a QLabel,
// QLineEdit, QPlainTextEdit, QTextEdit or QListWidget. `resetButton` and `statusLine` are looked
// up inside controlTab (the Configuration tab has its own resetButton).
#include <QAbstractButton>
#include <QAbstractItemModel>
#include <QAbstractItemView>
#include <QAbstractSlider>
#include <QApplication>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QListWidgetItem>
#include <QOpenGLContext>
#include <QPlainTextEdit>
#include <QRegularExpression>
#include <QRegularExpressionMatch>
#include <QSlider>
#include <QSpinBox>
#include <QString>
#include <QTabWidget>
#include <QTest>
#include <QTextEdit>
#include <QTimer>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QTreeWidgetItemIterator>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/bridge/endpoint.hpp"
#include "cobot/bridge/wire.hpp"
#include "cobot/common/result.hpp"
#include "cobot/common/types.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/log_record.hpp"
#include "cobot/studio/robot_scene.hpp"
#include "cobot/studio/studio_config.hpp"

#include "main_window.hpp"
#include "robot_view.hpp"
#include "serve.hpp"
#include "studio_app_test_util.hpp"
#include <QMatrix4x4>
#include <Qt3DCore/QComponent>
#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>

namespace cobot::studio {
namespace {

namespace fs = std::filesystem;
using test::ModalHandler;
using test::pump;
using test::set_spin_value;
using test::spin_max;
using test::spin_min;
using test::spin_value;
using test::wait_until;
using Sample = bridge::StateSample<model::kArmDof>;

const fs::path kRepo = COBOT_REPO_DIR;
const fs::path kConfigDir = kRepo / "config";
constexpr int kN = model::kArmDof;
constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr int kMeshTimeoutMs = 120000;

std::string region_name() {
  return "cobot_test_control_" + std::to_string(::getpid());
}

std::string read_file(const fs::path& path) {
  const std::ifstream in(path, std::ios::binary);
  std::ostringstream os;
  os << in.rdbuf();
  return os.str();
}

fs::path test_dir(const std::string& name) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "studio_control" / name;
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir;
}

// config/default.yaml with absolute paths and a 5 s Recorder per buffer.
fs::path serve_config(const fs::path& dir) {
  std::string text = read_file(kRepo / "config/default.yaml");
  const auto replace = [&text](const std::string& from, const std::string& to) {
    const std::size_t at = text.find(from);
    EXPECT_NE(at, std::string::npos) << from;
    if (at != std::string::npos) {
      text.replace(at, from.size(), to);
    }
  };
  replace("urdf_path: ../src/sim/urdf/cobot30_5.urdf",
          "urdf_path: " + (kRepo / "src/sim/urdf/cobot30_5.urdf").string());
  replace("output_dir: ../results", "output_dir: " + (dir / "results").string());
  replace("capacity_s: 60.0", "capacity_s: 5.0");
  const fs::path yaml = dir / "serve_config.yaml";
  std::ofstream(yaml, std::ios::binary) << text;
  return yaml;
}

std::uint8_t raw(pipeline::LoopMode m) {
  return static_cast<std::uint8_t>(m);
}

bool gl_available() {
  QOpenGLContext context;
  return context.create();
}

// The text a widget shows.
QString text_of(const QWidget* w) {
  if (const auto* l = qobject_cast<const QLabel*>(w)) {
    return l->text();
  }
  if (const auto* e = qobject_cast<const QLineEdit*>(w)) {
    return e->text();
  }
  if (const auto* p = qobject_cast<const QPlainTextEdit*>(w)) {
    return p->toPlainText();
  }
  if (const auto* t = qobject_cast<const QTextEdit*>(w)) {
    return t->toPlainText();
  }
  if (const auto* list = qobject_cast<const QListWidget*>(w)) {
    QStringList items;
    for (int i = 0; i < list->count(); ++i) {
      items << list->item(i)->text();
    }
    return items.join(QLatin1Char('\n'));
  }
  return {};
}

// A value widget: a spin box's value, else the first number of its text; NaN when none.
double number_of(const QWidget* w) {
  if (qobject_cast<const QDoubleSpinBox*>(w) != nullptr ||
      qobject_cast<const QSpinBox*>(w) != nullptr) {
    return spin_value(w);
  }
  static const QRegularExpression number(
      QStringLiteral(R"([-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?)"));
  const QRegularExpressionMatch m = number.match(text_of(w));
  return m.hasMatch() ? m.captured(0).toDouble() : std::nan("");
}

// Product of the QTransform components from the scene root down to `entity` (studio-3d).
QMatrix4x4 world_matrix(const Qt3DCore::QEntity* entity) {
  QMatrix4x4 world;
  for (const Qt3DCore::QEntity* e = entity; e != nullptr; e = e->parentEntity()) {
    for (Qt3DCore::QComponent* c : e->components()) {
      if (const auto* t = qobject_cast<const Qt3DCore::QTransform*>(c); t != nullptr) {
        world = t->matrix() * world;
      }
    }
  }
  return world;
}

bool drawn(const Qt3DCore::QEntity* entity) {
  for (const Qt3DCore::QEntity* e = entity; e != nullptr; e = e->parentEntity()) {
    if (!e->isEnabled()) {
      return false;
    }
  }
  return true;
}

// bench::serve on its own thread until stop().
class Serve {
 public:
  explicit Serve(const fs::path& dir) {
    options_.config_yaml = serve_config(dir);
    options_.imperfections_yaml = kRepo / "config/imperfections/ideal.yaml";
    options_.name = region_name();
    options_.out_dir = dir / "serve";
  }
  Serve(const Serve&) = delete;
  Serve& operator=(const Serve&) = delete;
  ~Serve() { stop(); }

  void start() {
    thread_ = std::thread([this] {
      const common::Status st = bench::serve(options_, stop_, [this](const std::string& region) {
        const std::lock_guard<std::mutex> lock(mutex_);
        ready_ = region;
        cv_.notify_all();
      });
      const std::lock_guard<std::mutex> lock(mutex_);
      result_ = st;
      cv_.notify_all();
    });
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait_for(lock, std::chrono::seconds(60), [this] { return ready_ || result_; });
    ASSERT_TRUE(ready_) << "serve never became ready"
                        << (result_ ? ": " + result_->reason() : std::string());
  }

  void stop() {
    stop_ = true;
    if (thread_.joinable()) {
      thread_.join();
      ASSERT_TRUE(result_);
      EXPECT_TRUE(result_->is_ok()) << result_->reason();
    }
  }

 private:
  bench::ServeOptions options_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<std::string> ready_;
  std::optional<common::Status> result_;
};

// A client that never claims: the server's samples, session and owner as the test sees them.
class Observer {
 public:
  bool open() {
    common::Result<bridge::ClientEndpoint<kN>> ep = bridge::ClientEndpoint<kN>::open(region_name());
    if (!ep) {
      ADD_FAILURE() << ep.reason();
      return false;
    }
    ep_.emplace(std::move(ep.value()));
    return true;
  }

  // Reads every new sample; true when one arrived.
  bool drain() {
    buf_.clear();
    if (!ep_ || ep_->read_states(buf_) == 0) {
      return false;
    }
    for (const Sample& s : buf_) {
      saw_tracking_ = saw_tracking_ || s.mode == raw(pipeline::LoopMode::kTracking);
      if (saw_tracking_ && s.mode == raw(pipeline::LoopMode::kStandby) && !standby_after_) {
        standby_after_ = s.t_ns;
      }
    }
    last_ = buf_.back();
    return true;
  }

  const std::optional<Sample>& last() const { return last_; }
  bool in(pipeline::LoopMode m) {
    drain();
    return last_ && last_->mode == raw(m);
  }
  // Forget the motion seen so far: the next standby_after() is that of a new motion.
  void arm() {
    drain();
    saw_tracking_ = false;
    standby_after_.reset();
  }
  bool saw_tracking() const { return saw_tracking_; }
  bool back_in_standby() {
    drain();
    return standby_after_.has_value();
  }
  std::uint64_t generation() const {
    bridge::SessionInfo<kN> s{};
    for (int i = 0; i < 100; ++i) {
      if (ep_->session(s)) {
        return s.generation;
      }
    }
    ADD_FAILURE() << "session torn 100 times";
    return 0;
  }
  // Authority is free: a claim succeeds (then released at once).
  bool free() {
    if (!ep_->claim(bridge::monotonic_ns()).is_ok()) {
      return false;
    }
    ep_->release();
    return true;
  }
  std::int64_t owner_pid() const { return ep_->owner_pid(); }

 private:
  std::optional<bridge::ClientEndpoint<kN>> ep_;
  std::vector<Sample> buf_;
  std::optional<Sample> last_;
  bool saw_tracking_ = false;
  std::optional<std::int64_t> standby_after_;
};

class StudioControl : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string test = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    dir_ = test_dir(test);
    serve_ = std::make_unique<Serve>(dir_);
    window_ = std::make_unique<MainWindow>(kConfigDir, dir_ / "work", region_name());
    window_->resize(1280, 800);
    window_->show();
  }

  void TearDown() override {
    if (window_) {
      window_->close();
      window_.reset();
    }
    serve_.reset();
  }

  template <typename T>
  T& get(const char* name) const {
    T* w = window_->findChild<T*>(QString::fromLatin1(name));
    if (w == nullptr) {
      throw std::runtime_error(std::string("no widget '") + name + "' of the expected type");
    }
    return *w;
  }

  // A widget of the Control tab (names shared with the Configuration tab: resetButton).
  template <typename T>
  T& control(const char* name) const {
    T* w = get<QWidget>("controlTab").findChild<T*>(QString::fromLatin1(name));
    if (w == nullptr) {
      throw std::runtime_error(std::string("no widget '") + name + "' in controlTab");
    }
    return *w;
  }

  QAbstractButton& button(const char* name) const { return get<QAbstractButton>(name); }
  QString text(const char* name) const { return text_of(&get<QWidget>(name)); }
  QString status_line() const { return text_of(&control<QWidget>("statusLine")); }
  double value(const std::string& name) const { return number_of(&get<QWidget>(name.c_str())); }
  QListWidget& run_list() const { return get<QListWidget>("runList"); }

  void show_control(const char* page) const {
    get<QTabWidget>("tabs").setCurrentWidget(&get<QWidget>("controlTab"));
    get<QTabWidget>("controlModes").setCurrentWidget(&get<QWidget>(page));
    pump(50);
  }

  // Starts serve, waits for the Control tab to show the connection (2 s) and opens the observer.
  void connect() {
    ASSERT_NO_FATAL_FAILURE(serve_->start());
    const QString connected =
        QStringLiteral("Connected to /cobot_") + QString::fromStdString(region_name());
    ASSERT_TRUE(wait_until([&] { return text("connectionLabel").startsWith(connected); }, 2000))
        << text("connectionLabel").toStdString();
    ASSERT_TRUE(observer_.open());
    ASSERT_TRUE(wait_until([&] { return observer_.in(pipeline::LoopMode::kStandby); }, 2000));
  }

  // connect(), claimButton, "You have control" and standby.
  void claim() {
    ASSERT_NO_FATAL_FAILURE(connect());
    ASSERT_TRUE(button("claimButton").isEnabled());
    button("claimButton").click();
    ASSERT_TRUE(wait_until(
        [&] {
          return text("ownerLabel") == QStringLiteral("You have control") &&
                 button("goJointButton").isEnabled();
        },
        2000))
        << text("ownerLabel").toStdString();
  }

  // Waits for a motion the window started to end in standby (`timeout_ms`), then lets the
  // labels refresh.
  void wait_motion_done(int timeout_ms) {
    ASSERT_TRUE(wait_until([&] { return observer_.back_in_standby(); }, timeout_ms))
        << "tracked: " << observer_.saw_tracking();
    pump(300);
  }

  // copyJointButton, jointGoal_1 += delta_deg, goJointButton; standby within 10 s.
  void move_joint_1(double delta_deg) {
    show_control("jointPage");
    button("copyJointButton").click();
    pump(50);
    auto& goal = get<QWidget>("jointGoal_1");
    const double target = value("jointValue_1") + delta_deg;
    set_spin_value(&goal, target);
    ASSERT_NEAR(spin_value(&goal), target, 1e-3);
    observer_.arm();
    ASSERT_TRUE(button("goJointButton").isEnabled());
    button("goJointButton").click();
    ASSERT_NO_FATAL_FAILURE(wait_motion_done(10000));
    EXPECT_NEAR(value("jointValue_1"), target, 0.2);
  }

  int table_rows() const {
    const QAbstractItemModel* m = get<QAbstractItemView>("waypointTable").model();
    return m == nullptr ? -1 : m->rowCount();
  }

  // Two taught points: the current TCP pose, a joint-1 move of 5 deg (a live run, waited for), the
  // new pose.
  void teach_two_points() {
    show_control("waypointPage");
    const int before = table_rows();
    button("addPoseButton").click();
    pump(50);
    ASSERT_EQ(table_rows(), before + 1);
    const int runs = run_list().count();
    ASSERT_NO_FATAL_FAILURE(move_joint_1(5.0));
    ASSERT_TRUE(wait_until([&] { return run_list().count() == runs + 1; }, 4000))
        << "the joint move between the points is a live run";
    show_control("waypointPage");
    button("addPoseButton").click();
    pump(50);
    ASSERT_EQ(table_rows(), before + 2);
  }

  fs::path dir_;
  std::unique_ptr<Serve> serve_;
  std::unique_ptr<MainWindow> window_;
  Observer observer_;
  ModalHandler modal_;
};

TEST_F(StudioControl, ThreeTabsInDesignOrderAndTheContractWidgets) {
  const auto& tabs = get<QTabWidget>("tabs");
  ASSERT_EQ(tabs.count(), 3);
  EXPECT_EQ(tabs.widget(0)->objectName(), QStringLiteral("configTab"));
  EXPECT_EQ(tabs.widget(1)->objectName(), QStringLiteral("controlTab"));
  EXPECT_EQ(tabs.widget(2)->objectName(), QStringLiteral("trackingTab"));
  EXPECT_EQ(tabs.tabText(0), QStringLiteral("Configuration"));
  EXPECT_EQ(tabs.tabText(1), QStringLiteral("Control"));
  EXPECT_EQ(tabs.tabText(2), QStringLiteral("Tracking"));

  const QWidget& control_tab = get<QWidget>("controlTab");
  std::vector<std::string> names = {
      "connectionLabel",     "ownerLabel",          "claimButton",   "releaseButton",
      "controlModes",        "jointPage",           "tcpPage",       "waypointPage",
      "copyJointButton",     "goJointButton",       "frameCombo",    "copyTcpButton",
      "goTcpButton",         "waypointTable",       "addPoseButton", "removePointButton",
      "openWaypointsButton", "saveWaypointsButton", "previewButton", "runWaypointsButton",
      "previewIssues",       "controlSpeedSlider",  "stopButton",    "estopButton",
      "resetButton",         "statusLine",          "liveRobotView"};
  for (int j = 1; j <= kN; ++j) {
    for (const char* prefix : {"jointValue_", "jogMinus_", "jogPlus_", "jointGoal_"}) {
      names.push_back(prefix + std::to_string(j));
    }
  }
  for (const char* axis : {"x", "y", "z", "rx", "ry", "rz"}) {
    for (const char* prefix : {"tcpValue_", "jogMinus_", "jogPlus_", "tcpGoal_"}) {
      names.push_back(prefix + std::string(axis));
    }
  }
  for (const std::string& name : names) {
    EXPECT_NE(control_tab.findChild<QWidget*>(QString::fromStdString(name)), nullptr) << name;
  }
  EXPECT_NE(control_tab.findChild<RobotView*>(QStringLiteral("liveRobotView")), nullptr);
  const auto& modes = get<QTabWidget>("controlModes");
  ASSERT_EQ(modes.count(), 3);
  EXPECT_EQ(modes.widget(0)->objectName(), QStringLiteral("jointPage"));
  EXPECT_EQ(modes.widget(1)->objectName(), QStringLiteral("tcpPage"));
  EXPECT_EQ(modes.widget(2)->objectName(), QStringLiteral("waypointPage"));
  const auto& frame = get<QComboBox>("frameCombo");
  ASSERT_EQ(frame.count(), 2);
  EXPECT_EQ(frame.itemText(0), QStringLiteral("Base"));
  EXPECT_EQ(frame.itemText(1), QStringLiteral("Tool"));
  const auto& speed = get<QAbstractSlider>("controlSpeedSlider");
  EXPECT_EQ(speed.minimum(), 1);
  EXPECT_EQ(speed.maximum(), 100);
  EXPECT_EQ(speed.value(), 10) << "StudioConfig::speed_pct default";
  EXPECT_NE(window_->findChild<QAbstractButton*>(QStringLiteral("applyGazeboButton")), nullptr);
}

TEST_F(StudioControl, NotConnectedUntilServeStarts) {
  pump(200);
  EXPECT_TRUE(text("connectionLabel").startsWith(QStringLiteral("Not connected")))
      << text("connectionLabel").toStdString();
  EXPECT_TRUE(text("connectionLabel").contains(QString::fromStdString(region_name())));
  // Everything is disabled without a connection.
  for (const char* name : {"claimButton",
                           "releaseButton",
                           "jogPlus_1",
                           "jogMinus_1",
                           "goJointButton",
                           "goTcpButton",
                           "previewButton",
                           "runWaypointsButton",
                           "stopButton",
                           "estopButton",
                           "applyGazeboButton"}) {
    EXPECT_FALSE(button(name).isEnabled()) << name;
  }
  EXPECT_FALSE(control<QAbstractButton>("resetButton").isEnabled());
  ASSERT_NO_FATAL_FAILURE(connect());
  EXPECT_TRUE(text("connectionLabel").contains(QString::number(::getpid())))
      << "controller pid: " << text("connectionLabel").toStdString();
  EXPECT_TRUE(button("claimButton").isEnabled());
  EXPECT_EQ(text("ownerLabel"), QStringLiteral("Free"));
}

// jointGoal range = the session's jog band, in degrees: joint 4 has URDF limits
// [-3.1416, -0.4] rad, soft margin 3 deg, jog margin 2 deg: max = -0.4 rad - 5 deg =
// -22.9183118 - 5 = -27.9183118 deg; min = -3.1416 rad + 5 deg = -180.0004209 + 5 =
// -175.0004209 deg.
TEST_F(StudioControl, ClaimGivesControlAndTheJogBandBoundsTheGoals) {
  ASSERT_NO_FATAL_FAILURE(claim());
  const QWidget& goal_4 = get<QWidget>("jointGoal_4");
  EXPECT_NEAR(spin_max(&goal_4), (-0.4 - 5.0 * kDeg) / kDeg, 1e-6);
  EXPECT_NEAR(spin_min(&goal_4), (-3.1416 + 5.0 * kDeg) / kDeg, 1e-6);
  EXPECT_EQ(observer_.owner_pid(), static_cast<std::int64_t>(::getpid()));

  button("releaseButton").click();
  ASSERT_TRUE(wait_until([&] { return text("ownerLabel") == QStringLiteral("Free"); }, 1000))
      << text("ownerLabel").toStdString();
  EXPECT_TRUE(observer_.free());
}

// The joint values show the live q in degrees and the TCP values FK of it (mm, deg): default.yaml
// home q [0, 30, 0, -90, 0, 60, 0] deg puts the TCP (= tip, no tool) at (110.013, -806.959,
// 389.122) mm with R ~ [[0, -1, 0], [0, 0, -1], [1, 0, 0]], pitch -90 deg (E2E-13 header).
TEST_F(StudioControl, JointAndTcpValuesShowTheLivePose) {
  ASSERT_NO_FATAL_FAILURE(connect());
  pump(300);
  ASSERT_TRUE(observer_.last());
  const std::vector<double> home_deg = {0.0, 30.0, 0.0, -90.0, 0.0, 60.0, 0.0};
  for (int j = 1; j <= kN; ++j) {
    const double q_deg = observer_.last()->q[static_cast<std::size_t>(j - 1)] / kDeg;
    EXPECT_NEAR(value("jointValue_" + std::to_string(j)), q_deg, 0.01) << "joint " << j;
    EXPECT_NEAR(q_deg, home_deg[static_cast<std::size_t>(j - 1)], 0.05) << "joint " << j;
  }
  EXPECT_NEAR(value("tcpValue_x"), 110.013, 0.1);
  EXPECT_NEAR(value("tcpValue_y"), -806.959, 0.1);
  EXPECT_NEAR(value("tcpValue_z"), 389.122, 0.1);
  EXPECT_NEAR(value("tcpValue_ry"), -90.0, 0.1);
}

// Joint jog at the default 10 %: v = 0.1 * 0.9 * 2.1 = 0.189 rad/s, a = 0.1 * 15 = 1.5 rad/s^2;
// ramps 2 x 0.0119 rad + cruise (0.300 - 0.126) * 0.189 = 0.0329 rad: 0.0567 rad = 3.25 deg.
// Under CPU load the GUI timers fire 100-130 ms late, the 25 ms jog refresh misses the 100 ms
// deadman and the server brakes early (1.4-1.7 deg seen on CI). The test therefore checks that
// the press raises joint 1 clearly and that the release stops it near the planned 3.25 deg.
TEST_F(StudioControl, HoldingJogPlus1For300MsRaisesJoint1) {
  ASSERT_NO_FATAL_FAILURE(claim());
  show_control("jointPage");
  const double before = value("jointValue_1");
  observer_.arm();
  auto& plus = button("jogPlus_1");
  ASSERT_TRUE(plus.isEnabled());
  QTest::mousePress(&plus, Qt::LeftButton);
  pump(300);
  EXPECT_TRUE(plus.isEnabled()) << "a jog button stays enabled while its own jog runs";
  QTest::mouseRelease(&plus, Qt::LeftButton);
  ASSERT_NO_FATAL_FAILURE(wait_motion_done(5000));
  const double moved = value("jointValue_1") - before;
  EXPECT_GT(moved, 0.5) << "deg: the press did not raise joint 1";
  EXPECT_LT(moved, 3.25 + 1.0) << "deg: the release did not stop the jog";
}

TEST_F(StudioControl, GoJointReachesTheGoalAndAddsALiveRun) {
  ASSERT_NO_FATAL_FAILURE(claim());
  const int runs = run_list().count();
  ASSERT_NO_FATAL_FAILURE(move_joint_1(10.0));
  ASSERT_TRUE(wait_until([&] { return run_list().count() == runs + 1; }, 3000))
      << "no live run in runList";
  EXPECT_TRUE(run_list().item(runs)->text().startsWith(QStringLiteral("live move_joint")))
      << run_list().item(runs)->text().toStdString();
  const RunEntry& entry = window_->run_store().runs().back();
  EXPECT_EQ(entry.kind, RunKind::kGazebo);
  ASSERT_TRUE(entry.data);
  EXPECT_EQ(entry.data->records.back().mode, pipeline::LoopMode::kStandby);
  EXPECT_FALSE(entry.summary.held);
}

// copyTcpButton, tcpGoal_x += 20 mm, goTcpButton: a straight TCP move (move_tcp) to the goal.
TEST_F(StudioControl, GoTcpMovesAlongBaseX) {
  ASSERT_NO_FATAL_FAILURE(claim());
  show_control("tcpPage");
  button("copyTcpButton").click();
  pump(50);
  const double x0 = value("tcpValue_x");
  EXPECT_NEAR(value("tcpGoal_x"), x0, 0.01);
  set_spin_value(&get<QWidget>("tcpGoal_x"), x0 + 20.0);
  const int runs = run_list().count();
  observer_.arm();
  ASSERT_TRUE(button("goTcpButton").isEnabled());
  button("goTcpButton").click();
  ASSERT_NO_FATAL_FAILURE(wait_motion_done(10000));
  EXPECT_NEAR(value("tcpValue_x"), x0 + 20.0, 1.0);
  ASSERT_TRUE(wait_until([&] { return run_list().count() == runs + 1; }, 3000));
  EXPECT_TRUE(run_list().item(runs)->text().startsWith(QStringLiteral("live move_tcp")))
      << run_list().item(runs)->text().toStdString();
}

TEST_F(StudioControl, TaughtPointsPreviewCleanAndRun) {
  ASSERT_NO_FATAL_FAILURE(claim());
  ASSERT_NO_FATAL_FAILURE(teach_two_points());
  const int runs = run_list().count();
  ASSERT_TRUE(button("previewButton").isEnabled());
  button("previewButton").click();
  pump(1500);
  EXPECT_TRUE(text("previewIssues").trimmed().isEmpty()) << text("previewIssues").toStdString();
  observer_.arm();
  ASSERT_TRUE(button("runWaypointsButton").isEnabled());
  button("runWaypointsButton").click();
  ASSERT_NO_FATAL_FAILURE(wait_motion_done(15000));
  ASSERT_TRUE(wait_until([&] { return run_list().count() == runs + 1; }, 4000))
      << "no live waypoint run";
  EXPECT_TRUE(run_list().item(runs)->text().startsWith(QStringLiteral("live waypoints")))
      << run_list().item(runs)->text().toStdString();
}

TEST_F(StudioControl, EnablingFollowsAuthorityAndMode) {
  ASSERT_NO_FATAL_FAILURE(connect());
  pump(200);
  // Connected, not the owner: preview only.
  EXPECT_TRUE(button("previewButton").isEnabled());
  for (const char* name : {"jogPlus_1",
                           "goJointButton",
                           "goTcpButton",
                           "runWaypointsButton",
                           "stopButton",
                           "estopButton"}) {
    EXPECT_FALSE(button(name).isEnabled()) << name;
  }
  button("claimButton").click();
  ASSERT_TRUE(wait_until([&] { return button("goJointButton").isEnabled(); }, 2000));
  for (const char* name : {"jogPlus_1",
                           "jogMinus_7",
                           "jogPlus_x",
                           "jogMinus_rz",
                           "goTcpButton",
                           "stopButton",
                           "estopButton",
                           "previewButton"}) {
    EXPECT_TRUE(button(name).isEnabled()) << name;
  }
  EXPECT_FALSE(control<QAbstractButton>("resetButton").isEnabled()) << "only in holding";

  // During a move: no jog or Go, Stop and E-stop stay.
  show_control("jointPage");
  button("copyJointButton").click();
  pump(50);
  set_spin_value(&get<QWidget>("jointGoal_1"), value("jointValue_1") + 10.0);
  observer_.arm();
  button("goJointButton").click();
  ASSERT_TRUE(wait_until([&] { return observer_.in(pipeline::LoopMode::kTracking); }, 2000));
  pump(150);
  EXPECT_FALSE(button("goJointButton").isEnabled());
  EXPECT_FALSE(button("jogPlus_2").isEnabled());
  EXPECT_TRUE(button("stopButton").isEnabled());
  EXPECT_TRUE(button("estopButton").isEnabled());

  // Stop ends the move early, in standby.
  button("stopButton").click();
  ASSERT_NO_FATAL_FAILURE(wait_motion_done(5000));
  EXPECT_TRUE(button("goJointButton").isEnabled());
}

TEST_F(StudioControl, EstopHoldsAndResetReturnsToStandby) {
  ASSERT_NO_FATAL_FAILURE(claim());
  auto& reset = control<QAbstractButton>("resetButton");
  EXPECT_FALSE(reset.isEnabled());
  button("estopButton").click();
  ASSERT_TRUE(wait_until([&] { return observer_.in(pipeline::LoopMode::kHolding); }, 2000));
  ASSERT_TRUE(wait_until(
      [&] {
        return status_line().contains(QStringLiteral("estop"), Qt::CaseInsensitive) &&
               reset.isEnabled();
      },
      2000))
      << status_line().toStdString();
  EXPECT_EQ(observer_.last()->hold_reason, static_cast<std::uint8_t>(pipeline::HoldReason::kEstop));
  EXPECT_FALSE(button("goJointButton").isEnabled()) << "no motion while holding";

  reset.click();
  ASSERT_TRUE(wait_until([&] { return observer_.in(pipeline::LoopMode::kStandby); }, 3000));
  ASSERT_TRUE(wait_until([&] { return !reset.isEnabled(); }, 1000));
}

TEST_F(StudioControl, SpeedSlidersStayInStep) {
  auto& control_speed = get<QAbstractSlider>("controlSpeedSlider");
  auto& config_speed = get<QSlider>("speedSlider");
  EXPECT_TRUE(config_speed.isEnabled());
  control_speed.setValue(35);
  pump(50);
  EXPECT_EQ(config_speed.value(), 35);
  EXPECT_EQ(window_->studio_config().speed_pct, 35.0);
  config_speed.setValue(60);
  pump(50);
  EXPECT_EQ(control_speed.value(), 60);
  EXPECT_EQ(window_->studio_config().speed_pct, 60.0);
}

TEST_F(StudioControl, ApplyToGazeboOnlyAsOwnerInStandby) {
  ASSERT_NO_FATAL_FAILURE(connect());
  auto& apply = button("applyGazeboButton");
  pump(200);
  EXPECT_FALSE(apply.isEnabled()) << "not the owner";
  button("claimButton").click();
  ASSERT_TRUE(wait_until([&] { return apply.isEnabled(); }, 2000)) << "owner, standby, valid";

  // Not while a move runs.
  show_control("jointPage");
  button("copyJointButton").click();
  pump(50);
  set_spin_value(&get<QWidget>("jointGoal_1"), value("jointValue_1") + 10.0);
  observer_.arm();
  button("goJointButton").click();
  ASSERT_TRUE(wait_until([&] { return observer_.in(pipeline::LoopMode::kTracking); }, 2000));
  pump(150);
  EXPECT_FALSE(apply.isEnabled()) << "tracking";
  ASSERT_NO_FATAL_FAILURE(wait_motion_done(10000));
  ASSERT_TRUE(wait_until([&] { return apply.isEnabled(); }, 1000));

  // premium_fr3 materialised into the work dir and applied: a new session.
  auto& base = get<QComboBox>("baseCombo");
  const int premium = base.findText(QStringLiteral("premium_fr3"));
  ASSERT_GE(premium, 0);
  base.setCurrentIndex(premium);
  pump(100);
  ASSERT_EQ(window_->studio_config().base.name, "premium_fr3") << modal_.log();
  ASSERT_TRUE(wait_until([&] { return apply.isEnabled(); }, 1000));
  const std::uint64_t generation = observer_.generation();
  apply.click();
  ASSERT_TRUE(wait_until(
      [&] {
        for (const QWidget* w : window_->findChildren<QWidget*>(QStringLiteral("statusLine"))) {
          if (text_of(w).contains(QStringLiteral("applied"))) {
            return true;
          }
        }
        return false;
      },
      5000))
      << status_line().toStdString() << " | " << modal_.log();
  EXPECT_EQ(observer_.generation(), generation + 1);
}

// paramTree rows of keys the Gazebo path cannot honour (catalog gazebo_supported false) are
// disabled with a tooltip while connected; supported rows are not.
TEST_F(StudioControl, UnsupportedParamRowsAreGreyedWhileConnected) {
  const auto row = [this](const std::string& key) -> QTreeWidgetItem* {
    auto& tree = get<QTreeWidget>("paramTree");
    for (QTreeWidgetItemIterator it(&tree); *it != nullptr; ++it) {
      if ((*it)->text(0).toStdString() == key) {
        return *it;
      }
    }
    return nullptr;
  };
  const auto greyed = [this](QTreeWidgetItem* item) {
    auto& tree = get<QTreeWidget>("paramTree");
    bool cells_disabled = true;
    bool any_cell = false;
    bool tooltip = false;
    for (int c = 0; c < tree.columnCount(); ++c) {
      tooltip = tooltip || item->toolTip(c).contains(QStringLiteral("not supported on Gazebo"));
      if (QWidget* w = tree.itemWidget(item, c); w != nullptr) {
        any_cell = true;
        cells_disabled = cells_disabled && !w->isEnabled();
        tooltip = tooltip || w->toolTip().contains(QStringLiteral("not supported on Gazebo"));
      }
    }
    const bool disabled = item->isDisabled() || (any_cell && cells_disabled);
    return disabled && tooltip;
  };
  const std::vector<std::string> unsupported = {
      "model.payload_kg", "friction.coulomb_nm", "timing.period_jitter_std_us"};
  for (const std::string& key : unsupported) {
    ASSERT_NE(row(key), nullptr) << key;
    EXPECT_FALSE(greyed(row(key))) << key << " greyed before any connection";
  }
  ASSERT_NO_FATAL_FAILURE(connect());
  ASSERT_TRUE(wait_until(
      [&] {
        for (const std::string& key : unsupported) {
          if (!greyed(row(key))) {
            return false;
          }
        }
        return true;
      },
      1000));
  ASSERT_NE(row("sensor.q_noise_std_rad"), nullptr);
  EXPECT_FALSE(greyed(row("sensor.q_noise_std_rad")));

  ASSERT_NO_FATAL_FAILURE(serve_->stop());
  ASSERT_TRUE(wait_until(
      [&] {
        for (const std::string& key : unsupported) {
          if (greyed(row(key))) {
            return false;
          }
        }
        return true;
      },
      3000))
      << "rows enabled again once the server is gone";
  EXPECT_TRUE(text("connectionLabel").startsWith(QStringLiteral("Not connected")));
}

// The heartbeat runs on the GUI thread on purpose: a slot that blocks it for 400 ms during a move
// stops the robot (owner lost after 200 ms) and gives authority back.
TEST_F(StudioControl, BlockedGuiThreadStopsTheMoveAndFreesAuthority) {
  ASSERT_NO_FATAL_FAILURE(claim());
  show_control("jointPage");
  button("copyJointButton").click();
  pump(50);
  set_spin_value(&get<QWidget>("jointGoal_1"), value("jointValue_1") + 10.0);
  observer_.arm();
  button("goJointButton").click();
  ASSERT_TRUE(wait_until([&] { return observer_.in(pipeline::LoopMode::kTracking); }, 2000));
  QTimer::singleShot(0, [] { std::this_thread::sleep_for(std::chrono::milliseconds(400)); });
  pump(50);
  ASSERT_TRUE(wait_until(
      [&] {
        return text("ownerLabel") == QStringLiteral("Free") &&
               observer_.in(pipeline::LoopMode::kStandby);
      },
      3000))
      << text("ownerLabel").toStdString();
  EXPECT_EQ(observer_.owner_pid(), 0);
}

// No file I/O on the GUI thread for live runs: a 20 ms timer keeps firing (largest gap < 250 ms)
// from the end of a waypoint run until its live entry is in runList.
TEST_F(StudioControl, GuiStaysResponsiveWhileTheLiveRunLoads) {
  ASSERT_NO_FATAL_FAILURE(claim());
  ASSERT_NO_FATAL_FAILURE(teach_two_points());
  const int runs = run_list().count();
  observer_.arm();
  button("runWaypointsButton").click();
  ASSERT_TRUE(wait_until([&] { return observer_.back_in_standby(); }, 15000));

  QElapsedTimer clock;
  qint64 last = 0;
  qint64 max_gap = 0;
  QTimer tick;
  tick.setInterval(20);
  QObject::connect(&tick, &QTimer::timeout, &tick, [&] {
    const qint64 now = clock.elapsed();
    max_gap = std::max(max_gap, now - last);
    last = now;
  });
  clock.start();
  tick.start();
  ASSERT_TRUE(wait_until([&] { return run_list().count() == runs + 1; }, 5000));
  tick.stop();
  EXPECT_LT(max_gap, 250) << "largest timer gap in ms";
}

TEST_F(StudioControl, ClosingTheWindowReleasesAuthorityAtOnce) {
  ASSERT_NO_FATAL_FAILURE(claim());
  ASSERT_FALSE(observer_.free());
  window_->close();
  QElapsedTimer clock;
  clock.start();
  bool released = false;
  while (!released && clock.elapsed() <= 100) {
    released = observer_.free();
    if (!released) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  EXPECT_TRUE(released) << "owner token still set 100 ms after close";
}

// The live 3D view draws the robot at the latest sample (q_true when finite): after a joint move,
// link_7's world transform is RobotScene's visual pose of that q (studio-3d's GL rule).
TEST_F(StudioControl, LiveRobotViewFollowsTheLatestSample) {
  if (!gl_available()) {
    GTEST_SKIP() << "no OpenGL context (QOpenGLContext::create failed)";
  }
  ASSERT_NO_FATAL_FAILURE(claim());
  show_control("jointPage");
  auto& view = get<RobotView>("liveRobotView");
  ASSERT_TRUE(wait_until([&] { return view.meshes_loaded(); }, kMeshTimeoutMs));
  ASSERT_NO_FATAL_FAILURE(move_joint_1(10.0));
  observer_.drain();
  ASSERT_TRUE(observer_.last());
  common::VecN<kN> q;
  bool truth = true;
  for (int j = 0; j < kN; ++j) {
    truth = truth && std::isfinite(observer_.last()->q_true[static_cast<std::size_t>(j)]);
  }
  for (int j = 0; j < kN; ++j) {
    const auto i = static_cast<std::size_t>(j);
    q(j) = truth ? observer_.last()->q_true[i] : observer_.last()->q[i];
  }
  const common::Result<RobotScene> scene = RobotScene::load(kRepo / "src/sim/urdf/cobot30_5.urdf",
                                                            "base_link",
                                                            "link_7",
                                                            {{"cobot_sim", kRepo / "src/sim"}});
  ASSERT_TRUE(scene.is_ok()) << scene.reason();
  std::vector<common::Pose> poses(scene.value().links().size());
  scene.value().visual_poses(q, poses);
  const common::Pose& want = poses.back();  // link_7, the last visual link

  ASSERT_NE(view.root_entity(), nullptr);
  const Qt3DCore::QEntity* link_7 = nullptr;
  for (const Qt3DCore::QEntity* e : view.root_entity()->findChildren<Qt3DCore::QEntity*>()) {
    if (e->objectName() == QStringLiteral("link_7") && drawn(e)) {
      link_7 = e;
    }
  }
  ASSERT_NE(link_7, nullptr) << "no drawn link_7 in the live view";
  const QMatrix4x4 m = world_matrix(link_7);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      EXPECT_NEAR(m(r, c), want.R(r, c), 1e-4) << "R(" << r << "," << c << ")";
    }
    EXPECT_NEAR(m(r, 3), want.p(r), 1e-4) << "p(" << r << ")";
  }
}

}  // namespace
}  // namespace cobot::studio

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (::testing::GTEST_FLAG(list_tests)) {
    return RUN_ALL_TESTS();  // no display needed to list the tests
  }
  if (std::getenv("QT_QPA_PLATFORM") == nullptr) {
    qputenv("QT_QPA_PLATFORM", "offscreen");
  }
  const QApplication app(argc, argv);
  QApplication::setQuitOnLastWindowClosed(false);
  return RUN_ALL_TESTS();
}
