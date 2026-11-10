// The noise level selector (feature studio-noise-level, studio config UI design 3.4 and 4), run
// with QT_QPA_PLATFORM=offscreen. The `noiseLevel` card heads the Sensor / noise page (paramPages
// index 1); its five checkable buttons noiseOff, noiseProfile, noiseX2, noiseX5 and noiseCustom
// sit in one exclusive QButtonGroup. Clicking a level scales the five noise keys of the base pair
// (a key whose base is 0 on every joint gets no override); the checked button follows the values
// after every edit, reset, base switch and open. budget_xarm7 has q_noise 5e-5, q_offset 2e-3,
// qd_noise 0, torque_scale_std 0.10 and jitter 100 us, so a level overrides four keys; ideal has
// all five at 0.
#include <QAbstractButton>
#include <QApplication>
#include <QButtonGroup>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QLabel>
#include <QListWidget>
#include <QPoint>
#include <QStackedWidget>
#include <QString>
#include <QStringList>
#include <QTest>
#include <QToolButton>
#include <QWidget>
#include <algorithm>
#include <filesystem>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "cobot/pipeline/overrides.hpp"
#include "cobot/studio/param_catalog.hpp"
#include "cobot/studio/profile_catalog.hpp"
#include "cobot/studio/studio_config.hpp"

#include "main_window.hpp"
#include "studio_app_test_util.hpp"

namespace cobot::studio {
namespace {

namespace fs = std::filesystem;
using pipeline::KeyOverride;
using test::ModalHandler;
using test::pump;
using List = std::vector<double>;
using Value = decltype(KeyOverride::value);

const fs::path kConfigDir = fs::path(COBOT_REPO_DIR) / "config";
const fs::path kOutput = fs::path(COBOT_TEST_OUTPUT_DIR) / "studio_noise_selector";

constexpr const char* kQNoise = "sensor.q_noise_std_rad";
const std::vector<std::string> kFive = {kQNoise,
                                        "sensor.q_offset_std_rad",
                                        "sensor.qd_noise_std_rad_s",
                                        "actuator.torque_scale_std",
                                        "timing.period_jitter_std_us"};
const char* const kButtons[] = {"noiseOff", "noiseProfile", "noiseX2", "noiseX5", "noiseCustom"};

Value scaled(const Value& v, double k) {  // every element b * k
  if (const auto* d = std::get_if<double>(&v)) {
    return *d * k;
  }
  List out = std::get<List>(v);
  for (double& x : out) {
    x *= k;
  }
  return out;
}

BasePair pair(const std::string& name) {
  for (const BasePair& p : discover_base_pairs(kConfigDir)) {
    if (p.name == name) {
      return p;
    }
  }
  throw std::runtime_error("no base pair " + name);
}

class NoiseSelector : public ::testing::Test {
 protected:
  void SetUp() override {
    work_dir_ = kOutput / ::testing::UnitTest::GetInstance()->current_test_info()->name();
    fs::remove_all(work_dir_);
    fs::create_directories(work_dir_);
    window_ = std::make_unique<MainWindow>(kConfigDir, work_dir_);
    window_->show();
    QApplication::setActiveWindow(window_.get());
    get<QListWidget>("groupList").setCurrentRow(1);  // Sensor / noise
    pump(50);
  }
  void TearDown() override { window_.reset(); }

  template <typename T>
  T& get(const QString& name) const {
    T* w = window_->findChild<T*>(name);
    if (w == nullptr) {
      throw std::runtime_error("no widget '" + name.toStdString() + "' of the expected type");
    }
    return *w;
  }
  QAbstractButton& button(const char* name) const {
    return get<QAbstractButton>(QString::fromLatin1(name));
  }
  QFrame& card(const std::string& key) const {
    return get<QFrame>(QString::fromStdString("param:" + key));
  }

  std::string checked() const {  // the checked level button, "" when none
    std::string out;
    for (const char* name : kButtons) {
      out += button(name).isChecked() ? name : "";
    }
    return out;
  }

  void select_base(const std::string& name) {
    auto& combo = get<QComboBox>(QStringLiteral("baseCombo"));
    combo.setCurrentIndex(combo.findText(QString::fromStdString(name)));
    pump(50);
    ASSERT_EQ(window_->studio_config().base.name, name) << modal_.log();
  }

  void click(const char* name) {
    button(name).click();
    pump(20);
  }

  // Overrides of studio_config(): exactly the five keys whose budget base is not all 0, each at
  // base * k.
  void expect_scaled(const std::map<std::string, KeyOverride>& base, double k) const {
    std::set<std::string> want;
    for (const std::string& key : kFive) {
      if (!(base.at(key).value == scaled(base.at(key).value, 0.0))) {
        want.insert(key);
      }
    }
    std::set<std::string> got;
    for (const KeyOverride& o : window_->studio_config().overrides) {
      got.insert(o.key);
      EXPECT_TRUE(o.value == scaled(base.at(o.key).value, k)) << o.key << " k " << k;
    }
    EXPECT_EQ(got, want) << "k " << k;
  }

  ModalHandler modal_;
  fs::path work_dir_;
  std::unique_ptr<MainWindow> window_;
};

TEST_F(NoiseSelector, CardHeadsTheSensorPageWithFiveExclusiveButtons) {
  auto& level = get<QFrame>(QStringLiteral("noiseLevel"));
  EXPECT_TRUE(level.property("card").toBool());
  QWidget* page = get<QStackedWidget>(QStringLiteral("paramPages")).widget(1);
  ASSERT_NE(page, nullptr);
  EXPECT_TRUE(page->isAncestorOf(&level));
  const int top = level.mapTo(window_.get(), QPoint(0, 0)).y();
  for (const ParamInfo& p : param_catalog()) {
    if (p.group == "Sensor") {
      EXPECT_LT(top, card(p.key).mapTo(window_.get(), QPoint(0, 0)).y()) << p.key;
    }
  }
  std::set<QString> labels;
  for (const QLabel* l : level.findChildren<QLabel*>()) {
    labels.insert(l->text());
  }
  EXPECT_EQ(labels.count(QStringLiteral("Noise level")), 1U);
  EXPECT_EQ(labels.count(QStringLiteral("Scales every noise standard deviation of the base pair")),
            1U);
  const QStringList texts = {QStringLiteral("Off"),
                             QStringLiteral("Profile"),
                             QString::fromUtf8("×2"),
                             QString::fromUtf8("×5"),
                             QStringLiteral("Custom")};
  const QButtonGroup* group = button("noiseOff").group();
  ASSERT_NE(group, nullptr);
  EXPECT_TRUE(group->exclusive());
  for (int i = 0; i < 5; ++i) {
    auto* b = level.findChild<QToolButton*>(QString::fromLatin1(kButtons[i]));
    ASSERT_NE(b, nullptr) << kButtons[i];
    EXPECT_TRUE(b->isCheckable()) << kButtons[i];
    EXPECT_EQ(b->group(), group) << kButtons[i];
    EXPECT_EQ(b->text(), texts[i]) << kButtons[i];
  }
  EXPECT_EQ(checked(), "noiseProfile");
  EXPECT_FALSE(button("noiseCustom").isEnabled());
}

TEST_F(NoiseSelector, LevelsScaleTheBudgetPairAndHandEditsAreCustom) {
  ASSERT_NO_FATAL_FAILURE(select_base("budget_xarm7"));
  const std::map<std::string, KeyOverride> base = base_values(pair("budget_xarm7"));
  EXPECT_EQ(checked(), "noiseProfile");
  click("noiseX2");
  expect_scaled(base, 2.0);
  EXPECT_EQ(window_->studio_config().overrides.size(), 4U);  // qd_noise is 0 on every joint
  EXPECT_EQ(checked(), "noiseX2");
  EXPECT_TRUE(get<QWidget>(QStringLiteral("changeCount")).isVisible());
  EXPECT_TRUE(get<QWidget>(QStringLiteral("changeCount"))
                  .property("text")
                  .toString()
                  .contains(QStringLiteral("4")));

  // The q_noise card shows changed: undo offered, the shared field (all joints equal) amber.
  QFrame& q = card(kQNoise);
  auto* undo = q.findChild<QWidget*>(QStringLiteral("undoButton"));
  ASSERT_NE(undo, nullptr);
  EXPECT_TRUE(undo->isVisibleTo(&q));
  auto* shared = q.findChild<QDoubleSpinBox*>(QStringLiteral("value"));
  ASSERT_NE(shared, nullptr);
  EXPECT_TRUE(shared->property("changed").toBool());

  // 70 urad = 7e-5 rad = 1.4 x base 5e-5: no level matches.
  shared->setFocus();
  pump(20);
  shared->selectAll();
  QTest::keyClicks(shared, QStringLiteral("70"));
  QTest::keyClick(shared, Qt::Key_Return);
  pump(20);
  EXPECT_EQ(checked(), "noiseCustom");
  EXPECT_FALSE(button("noiseCustom").isEnabled());
  for (const KeyOverride& o : window_->studio_config().overrides) {
    if (o.key == kQNoise) {
      for (const double v : std::get<List>(o.value)) {
        EXPECT_NEAR(v, 7e-5, 1e-15);
      }
    }
  }

  click("noiseProfile");
  EXPECT_TRUE(window_->studio_config().overrides.empty());
  EXPECT_EQ(checked(), "noiseProfile");
  EXPECT_FALSE(get<QWidget>(QStringLiteral("changeCount")).isVisible());

  click("noiseX5");
  expect_scaled(base, 5.0);
  click("noiseOff");
  expect_scaled(base, 0.0);
  EXPECT_EQ(checked(), "noiseOff");
}

TEST_F(NoiseSelector, ResetAndBaseSwitchShowProfile) {
  ASSERT_NO_FATAL_FAILURE(select_base("budget_xarm7"));
  click("noiseX2");
  auto* reset = get<QWidget>(QStringLiteral("configTab"))
                    .findChild<QAbstractButton*>(QStringLiteral("resetButton"));
  ASSERT_NE(reset, nullptr);
  reset->click();
  pump(20);
  EXPECT_TRUE(window_->studio_config().overrides.empty());
  EXPECT_EQ(checked(), "noiseProfile");

  // No overrides, then with x5 (the discard question is answered Yes): Profile on the new base.
  ASSERT_NO_FATAL_FAILURE(select_base("realistic"));
  EXPECT_EQ(checked(), "noiseProfile");
  click("noiseX5");
  EXPECT_EQ(checked(), "noiseX5");
  ASSERT_NO_FATAL_FAILURE(select_base("budget_xarm7"));
  EXPECT_TRUE(window_->studio_config().overrides.empty());
  EXPECT_EQ(checked(), "noiseProfile");

  // ideal has every noise key at 0: x2 adds nothing and Profile stays shown.
  ASSERT_NO_FATAL_FAILURE(select_base("ideal"));
  click("noiseX2");
  EXPECT_TRUE(window_->studio_config().overrides.empty());
  EXPECT_EQ(checked(), "noiseProfile");
}

TEST_F(NoiseSelector, OpenOfAnX5FileShowsX5AndMaterializesBaseTimesFive) {
  ASSERT_NO_FATAL_FAILURE(select_base("budget_xarm7"));
  const std::map<std::string, KeyOverride> base = base_values(pair("budget_xarm7"));
  click("noiseX5");
  const QString file = QString::fromStdString((work_dir_ / "x5.studio.yaml").string());
  ASSERT_TRUE(window_->save_studio(file)) << modal_.log();
  click("noiseX2");
  ASSERT_EQ(checked(), "noiseX2");
  ASSERT_TRUE(window_->open_studio(file)) << modal_.log();
  pump(20);
  EXPECT_EQ(checked(), "noiseX5");
  expect_scaled(base, 5.0);

  const common::Result<Materialized> m =
      materialize(window_->studio_config(), work_dir_ / "materialized");
  ASSERT_TRUE(m) << m.reason();
  const YAML::Node sensor =
      YAML::LoadFile(m.value().imperfections_yaml.string())["imperfections"]["sensor"];
  const List want = std::get<List>(scaled(base.at(kQNoise).value, 5.0));  // 7 x 5e-5 * 5
  const YAML::Node q = sensor["q_noise_std_rad"];
  ASSERT_TRUE(q.IsDefined());
  if (q.IsSequence()) {
    EXPECT_EQ(q.as<List>(), want);
  } else {
    EXPECT_EQ(List(want.size(), q.as<double>()), want);
  }
}

}  // namespace
}  // namespace cobot::studio
