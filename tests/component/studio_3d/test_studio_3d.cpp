// The 3D playback view of the Tracking tab (feature studio-3d, cobot_studio design 6.9 and 7):
// the robot follows the shared time cursor, playback advances it, the selected run is solid and
// up to three other checked runs are ghosts, and the meshes load off the GUI thread.
//
// Widgets are found by their object names (contract of studio-app and studio-3d). The 3D scene
// is read through RobotView::root_entity(): every drawn run has one Qt3DCore::QEntity per visual
// link, named after the link (base_link, link_1 .. link_6, link_6_2, link_7), whose world
// transform (the product of the Qt3DCore::QTransform components of it and its parent entities)
// is the link's visual pose in base_link; an entity is drawn when it and every parent entity
// are enabled. The selected run's link entities carry a Qt3DExtras::QPhongMaterial, the ghosts'
// a Qt3DExtras::QPhongAlphaMaterial with alpha 0.25, both with the run's colour as diffuse.
#include <QAbstractButton>
#include <QAbstractSpinBox>
#include <QApplication>
#include <QColor>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QLabel>
#include <QListWidget>
#include <QOpenGLContext>
#include <QSlider>
#include <QSpinBox>
#include <QString>
#include <QTabWidget>
#include <QTimer>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/studio/playback.hpp"
#include "cobot/studio/robot_scene.hpp"

#include "main_window.hpp"
#include "robot_view.hpp"
#include "run_data.hpp"
#include "time_cursor.hpp"
#include <QMatrix4x4>
#include <Qt3DCore/QComponent>
#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>
#include <Qt3DExtras/QPhongAlphaMaterial>
#include <Qt3DExtras/QPhongMaterial>

namespace cobot::studio {
namespace {

namespace fs = std::filesystem;
constexpr int kN = model::kArmDof;

const fs::path kRepo = COBOT_REPO_DIR;
const std::array<QString, 9> kLinkNames = {
    "base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6", "link_6_2", "link_7"};
constexpr std::size_t kLink7 = 8;
constexpr std::size_t kLinksPerRun = 9;
constexpr int kRunTimeoutMs = 120000;   // one short bench run, Debug + ASan included
constexpr int kMeshTimeoutMs = 120000;  // 34 MB of STL, Debug + ASan included

bool gl_available() {
  QOpenGLContext context;
  return context.create();
}

// Runs the event loop for `ms` milliseconds.
void spin_for(int ms) {
  QEventLoop loop;
  QTimer::singleShot(ms, &loop, &QEventLoop::quit);
  loop.exec();
}

// Runs the event loop until `done` or `timeout_ms`; returns done().
bool wait_until(const std::function<bool()>& done, int timeout_ms) {
  QElapsedTimer clock;
  clock.start();
  while (!done()) {
    if (clock.elapsed() > timeout_ms) {
      return false;
    }
    spin_for(20);
  }
  return true;
}

// The single place that builds the window: config_dir holds default.yaml and imperfections/,
// work_dir receives the materialised runs (studio-app: --config-dir, --work-dir).
std::unique_ptr<MainWindow> make_window(const std::string& test_name) {
  const fs::path work_dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "studio_3d" / test_name;
  fs::create_directories(work_dir);
  return std::make_unique<MainWindow>(kRepo / "config", work_dir);
}

bool drawn(const Qt3DCore::QEntity* entity) {
  for (const Qt3DCore::QEntity* e = entity; e != nullptr; e = e->parentEntity()) {
    if (!e->isEnabled()) {
      return false;
    }
  }
  return true;
}

// Product of the QTransform components from the scene root down to `entity`.
QMatrix4x4 world_matrix(const Qt3DCore::QEntity* entity) {
  QMatrix4x4 world;  // identity
  for (const Qt3DCore::QEntity* e = entity; e != nullptr; e = e->parentEntity()) {
    for (Qt3DCore::QComponent* c : e->components()) {
      if (const auto* t = qobject_cast<const Qt3DCore::QTransform*>(c); t != nullptr) {
        world = t->matrix() * world;
      }
    }
  }
  return world;
}

template <typename T>
const T* component_of(const Qt3DCore::QEntity* entity) {
  for (Qt3DCore::QComponent* c : entity->components()) {
    if (const auto* m = qobject_cast<const T*>(c); m != nullptr) {
      return m;
    }
  }
  return nullptr;
}

bool is_link_name(const QString& name) {
  return std::find(kLinkNames.begin(), kLinkNames.end(), name) != kLinkNames.end();
}

// Drawn link entities with the solid and with the ghost material, and their diffuse colours.
struct MaterialCount {
  std::size_t opaque = 0;
  std::size_t alpha = 0;
  std::vector<QColor> opaque_colours;
  std::vector<QColor> alpha_colours;
  std::vector<float> alphas;
};

MaterialCount count_materials(const Qt3DCore::QEntity* root) {
  MaterialCount count;
  for (const Qt3DCore::QEntity* e : root->findChildren<Qt3DCore::QEntity*>()) {
    if (!is_link_name(e->objectName()) || !drawn(e)) {
      continue;
    }
    if (const auto* m = component_of<Qt3DExtras::QPhongMaterial>(e); m != nullptr) {
      ++count.opaque;
      count.opaque_colours.push_back(m->diffuse());
    }
    if (const auto* m = component_of<Qt3DExtras::QPhongAlphaMaterial>(e); m != nullptr) {
      ++count.alpha;
      count.alpha_colours.push_back(m->diffuse());
      count.alphas.push_back(m->alpha());
    }
  }
  return count;
}

bool all_same(const std::vector<QColor>& colours) {
  return std::all_of(
      colours.begin(), colours.end(), [&](const QColor& c) { return c == colours.front(); });
}

// The same run as the window's "ideal" base pair (config/default.yaml + ideal.yaml; the ideal
// profile draws nothing from the seed), in memory.
std::optional<bench::RunData> ideal_run(double duration_s) {
  auto imp = pipeline::load_imperfections<kN>(kRepo / "config/imperfections/ideal.yaml");
  if (!imp.is_ok()) {
    ADD_FAILURE() << imp.reason();
    return std::nullopt;
  }
  bench::RunRequest request;
  request.config_yaml = kRepo / "config/default.yaml";
  request.imperfections = imp.value();
  request.label = "ideal";
  request.duration_s = duration_s;
  auto run = bench::run_in_memory(request);
  if (!run.is_ok()) {
    ADD_FAILURE() << run.reason();
    return std::nullopt;
  }
  return std::move(run).value();
}

common::Result<RobotScene> load_scene() {
  return RobotScene::load(kRepo / "src/sim/urdf/cobot30_5.urdf",
                          "base_link",
                          "link_7",
                          {{"cobot_sim", kRepo / "src/sim"}});
}

// Visual pose of link_7 for the row the 3D view shows at cursor time t_s.
common::Pose expected_link_7(const RobotScene& scene, const bench::RunData& run, double t_s) {
  std::vector<common::Pose> poses(scene.links().size());
  scene.visual_poses(display_q(run.records[row_at(run, t_s)]), poses);
  return poses[kLink7];
}

void expect_matrix_is_pose(const QMatrix4x4& m, const common::Pose& T, const char* what) {
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      EXPECT_NEAR(m(r, c), T.R(r, c), 1e-5) << what << " R(" << r << "," << c << ")";
    }
    EXPECT_NEAR(m(r, 3), T.p(r), 1e-5) << what << " p(" << r << ")";
  }
  EXPECT_NEAR(m(3, 0), 0.0, 1e-6) << what;
  EXPECT_NEAR(m(3, 1), 0.0, 1e-6) << what;
  EXPECT_NEAR(m(3, 2), 0.0, 1e-6) << what;
  EXPECT_NEAR(m(3, 3), 1.0, 1e-6) << what;
}

class Studio3d : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!gl_available()) {
      GTEST_SKIP() << "no OpenGL context (QOpenGLContext::create failed)";
    }
    window_ = make_window(::testing::UnitTest::GetInstance()->current_test_info()->name());
    window_->resize(1280, 720);
    window_->show();
    auto* tabs = window_->findChild<QTabWidget*>("tabs");
    auto* tracking = window_->findChild<QWidget*>("trackingTab");
    ASSERT_NE(tabs, nullptr);
    ASSERT_NE(tracking, nullptr);
    tabs->setCurrentWidget(tracking);
    view_ = window_->findChild<RobotView*>("robotView");
    ASSERT_NE(view_, nullptr);
    cursor_ = window_->findChild<TimeCursor*>();
    ASSERT_NE(cursor_, nullptr);
    run_list_ = window_->findChild<QListWidget*>("runList");
    ASSERT_NE(run_list_, nullptr);
    ASSERT_TRUE(wait_until([this] { return view_->meshes_loaded(); }, kMeshTimeoutMs))
        << "meshes not loaded";
    ASSERT_NE(view_->root_entity(), nullptr);
  }

  void TearDown() override {
    if (window_) {
      window_->close();
      window_.reset();
    }
  }

  // Runs the "ideal" base pair for duration_s through runButton and checks every run.
  void run_ideal(double duration_s) {
    if (!base_chosen_) {
      auto* base = window_->findChild<QComboBox*>("baseCombo");
      ASSERT_NE(base, nullptr);
      const int ideal = base->findText("ideal");
      ASSERT_GE(ideal, 0);
      base->setCurrentIndex(ideal);
      base_chosen_ = true;
    }
    if (auto* spin = window_->findChild<QDoubleSpinBox*>("durationSpin"); spin != nullptr) {
      spin->setValue(duration_s);
    } else if (auto* ispin = window_->findChild<QSpinBox*>("durationSpin"); ispin != nullptr) {
      ispin->setValue(static_cast<int>(std::lround(duration_s)));
    } else {
      FAIL() << "durationSpin not found";
    }
    auto* run = window_->findChild<QAbstractButton*>("runButton");
    ASSERT_NE(run, nullptr);
    ASSERT_TRUE(run->isEnabled());
    const int before = run_list_->count();
    run->click();
    ASSERT_TRUE(wait_until([&] { return run_list_->count() == before + 1; }, kRunTimeoutMs))
        << "run did not arrive";
    for (int i = 0; i < run_list_->count(); ++i) {
      run_list_->item(i)->setCheckState(Qt::Checked);
    }
    spin_for(50);
  }

  // The drawn link_7 entities.
  std::vector<const Qt3DCore::QEntity*> drawn_link_7() const {
    std::vector<const Qt3DCore::QEntity*> out;
    for (const Qt3DCore::QEntity* e : view_->root_entity()->findChildren<Qt3DCore::QEntity*>()) {
      if (e->objectName() == kLinkNames[kLink7] && drawn(e)) {
        out.push_back(e);
      }
    }
    return out;
  }

  std::unique_ptr<MainWindow> window_;
  RobotView* view_ = nullptr;
  TimeCursor* cursor_ = nullptr;
  QListWidget* run_list_ = nullptr;
  bool base_chosen_ = false;
};

TEST_F(Studio3d, ControlsExist) {
  EXPECT_NE(window_->findChild<QAbstractButton*>("playButton"), nullptr);
  auto* speed = window_->findChild<QComboBox*>("speedCombo");
  ASSERT_NE(speed, nullptr);
  ASSERT_EQ(speed->count(), 3);  // 0.25x, 1x, 4x
  EXPECT_TRUE(speed->itemText(0).startsWith("0.25"));
  EXPECT_TRUE(speed->itemText(1).startsWith("1"));
  EXPECT_TRUE(speed->itemText(2).startsWith("4"));
  EXPECT_NE(window_->findChild<QSlider*>("timeSlider"), nullptr);
  EXPECT_NE(window_->findChild<QLabel*>("ghostLimitLabel"), nullptr);
  // Error magnification x1 .. x1000, default x1 (owner question Q9).
  if (auto* spin = window_->findChild<QSpinBox*>("magnifySpin"); spin != nullptr) {
    EXPECT_EQ(spin->minimum(), 1);
    EXPECT_EQ(spin->maximum(), 1000);
    EXPECT_EQ(spin->value(), 1);
  } else if (auto* dspin = window_->findChild<QDoubleSpinBox*>("magnifySpin"); dspin != nullptr) {
    EXPECT_DOUBLE_EQ(dspin->minimum(), 1.0);
    EXPECT_DOUBLE_EQ(dspin->maximum(), 1000.0);
    EXPECT_DOUBLE_EQ(dspin->value(), 1.0);
  } else {
    ADD_FAILURE() << "magnifySpin not found";
  }
}

TEST_F(Studio3d, CursorAtHalfASecondPlacesLink7AtThatRowsVisualPose) {
  ASSERT_NO_FATAL_FAILURE(run_ideal(1.0));
  const auto run = ideal_run(1.0);
  ASSERT_TRUE(run);
  const auto scene = load_scene();
  ASSERT_TRUE(scene.is_ok()) << scene.reason();

  cursor_->set_time_s(0.5);
  spin_for(100);
  const auto link_7 = drawn_link_7();
  ASSERT_EQ(link_7.size(), 1U) << "one run checked";
  expect_matrix_is_pose(
      world_matrix(link_7.front()), expected_link_7(scene.value(), *run, 0.5), "t = 0.5 s");

  // Dragging the slider moves the cursor and the robot: the slider's end is the run's end.
  auto* slider = window_->findChild<QSlider*>("timeSlider");
  ASSERT_NE(slider, nullptr);
  slider->setValue(slider->maximum());
  spin_for(100);
  const double t_end = cursor_->time_s();
  EXPECT_GE(t_end, 0.99) << "last row of a 1 s run at 1 kHz is at 0.999 s";
  EXPECT_LE(t_end, 1.0 + 1e-9);
  expect_matrix_is_pose(world_matrix(drawn_link_7().front()),
                        expected_link_7(scene.value(), *run, t_end),
                        "slider at its end");
}

TEST_F(Studio3d, PlaybackAdvancesTheCursorAtTheChosenSpeedAndPauses) {
  ASSERT_NO_FATAL_FAILURE(run_ideal(3.0));
  auto* play = window_->findChild<QAbstractButton*>("playButton");
  auto* speed = window_->findChild<QComboBox*>("speedCombo");
  ASSERT_NE(play, nullptr);
  ASSERT_NE(speed, nullptr);
  speed->setCurrentIndex(2);  // 4x
  ASSERT_TRUE(speed->currentText().startsWith("4"));

  // 4x for 0.5 s of wall time: 4 * 0.5 = 2.0 s of run time (the run is 3 s, no clamp).
  cursor_->set_time_s(0.0);
  play->click();
  spin_for(500);
  const double t_played = cursor_->time_s();
  EXPECT_NEAR(t_played, 2.0, 0.4);

  // Pause: the cursor stays where it is.
  play->click();
  const double t_paused = cursor_->time_s();
  spin_for(300);
  EXPECT_DOUBLE_EQ(cursor_->time_s(), t_paused);

  // From 2.9 s at 4x, 0.5 s of wall time would reach 4.9 s: playback stops at the run's end
  // (last row at 2.999 s) and stays there.
  cursor_->set_time_s(2.9);
  play->click();
  spin_for(500);
  const double t_end = cursor_->time_s();
  EXPECT_GE(t_end, 2.99);
  EXPECT_LE(t_end, 3.0 + 1e-9);
  spin_for(300);
  EXPECT_DOUBLE_EQ(cursor_->time_s(), t_end) << "playback stopped at the end";
}

TEST_F(Studio3d, SelectedRunIsSolidAndTheOtherCheckedRunIsAGhost) {
  ASSERT_NO_FATAL_FAILURE(run_ideal(1.0));
  ASSERT_NO_FATAL_FAILURE(run_ideal(1.0));
  ASSERT_EQ(run_list_->count(), 2);

  run_list_->setCurrentRow(0);
  spin_for(100);
  const MaterialCount first = count_materials(view_->root_entity());
  ASSERT_EQ(first.opaque, kLinksPerRun);
  ASSERT_EQ(first.alpha, kLinksPerRun);
  EXPECT_TRUE(all_same(first.opaque_colours));
  EXPECT_TRUE(all_same(first.alpha_colours));
  for (const float a : first.alphas) {
    EXPECT_NEAR(a, 0.25F, 1e-4F);  // 16-bit QColor alpha: 16384 / 65535
  }
  // Compare RGB only: QPhongAlphaMaterial keeps its alpha in the diffuse colour.
  EXPECT_NE(first.opaque_colours.front().rgb(), first.alpha_colours.front().rgb())
      << "one colour per run";

  run_list_->setCurrentRow(1);
  spin_for(100);
  const MaterialCount second = count_materials(view_->root_entity());
  ASSERT_EQ(second.opaque, kLinksPerRun);
  ASSERT_EQ(second.alpha, kLinksPerRun);
  // The newly selected run (the former ghost) is solid in its own colour, and the other way round.
  EXPECT_EQ(second.opaque_colours.front().rgb(), first.alpha_colours.front().rgb());
  EXPECT_EQ(second.alpha_colours.front().rgb(), first.opaque_colours.front().rgb());
  for (const float a : second.alphas) {
    EXPECT_NEAR(a, 0.25F, 1e-4F);  // 16-bit QColor alpha: 16384 / 65535
  }
}

TEST_F(Studio3d, AtMostThreeGhostsAndTheLabelNamesTheHiddenOne) {
  for (int i = 0; i < 5; ++i) {
    ASSERT_NO_FATAL_FAILURE(run_ideal(1.0));
  }
  ASSERT_EQ(run_list_->count(), 5);
  run_list_->setCurrentRow(4);
  spin_for(100);
  const MaterialCount count = count_materials(view_->root_entity());
  EXPECT_EQ(count.opaque, kLinksPerRun);     // 1 solid run
  EXPECT_EQ(count.alpha, 3 * kLinksPerRun);  // 3 of the 4 other checked runs
  auto* label = window_->findChild<QLabel*>("ghostLimitLabel");
  ASSERT_NE(label, nullptr);
  EXPECT_TRUE(label->isVisibleTo(window_.get()));
  EXPECT_TRUE(label->text().contains("1")) << label->text().toStdString();
  EXPECT_TRUE(label->text().contains("hidden", Qt::CaseInsensitive)) << label->text().toStdString();
}

// Mesh loading and geometry building never run on the GUI thread: a 20 ms timer started before
// the window is built never waits more than 250 ms, window construction included, until the
// meshes are loaded.
TEST(Studio3dStartup, MeshLoadingKeepsTheGuiThreadResponsive) {
  if (!gl_available()) {
    GTEST_SKIP() << "no OpenGL context (QOpenGLContext::create failed)";
  }
  std::unique_ptr<MainWindow> window;
  RobotView* view = nullptr;
  QElapsedTimer clock;
  qint64 last_ms = -1;
  qint64 max_gap_ms = 0;
  QTimer tick;
  tick.setInterval(20);
  QObject::connect(&tick, &QTimer::timeout, [&] {
    const qint64 now = clock.elapsed();
    if (last_ms >= 0) {
      max_gap_ms = std::max(max_gap_ms, now - last_ms);
    }
    last_ms = now;
  });
  clock.start();
  tick.start();
  QTimer::singleShot(100, [&] {
    window = make_window("startup");
    window->resize(1280, 720);
    window->show();
    view = window->findChild<RobotView*>("robotView");
  });
  ASSERT_TRUE(wait_until([&] { return view != nullptr && view->meshes_loaded(); }, kMeshTimeoutMs))
      << "meshes not loaded";
  spin_for(200);
  tick.stop();
  EXPECT_LT(max_gap_ms, 250) << "largest timer gap in ms";
  window->close();
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
  return RUN_ALL_TESTS();
}
