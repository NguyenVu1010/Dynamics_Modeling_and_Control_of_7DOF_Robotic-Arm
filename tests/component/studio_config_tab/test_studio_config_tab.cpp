#include <QAbstractButton>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QListWidgetItem>
#include <QPushButton>
#include <QScrollArea>
#include <QStackedWidget>
#include <QString>
#include <QTabWidget>
#include <QTest>
#include <QVariant>
#include <QWidget>
#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/pipeline/overrides.hpp"
#include "cobot/studio/param_catalog.hpp"
#include "cobot/studio/profile_catalog.hpp"
#include "cobot/studio/studio_config.hpp"

#include "main_window.hpp"
#include "param_card.hpp"
#include "param_spin.hpp"
#include "studio_app_test_util.hpp"

// The Configuration tab redesign (feature studio-config-layout; design
// docs/design/2026-11-10-studio-config-ui-design.md sections 2, 3, 3.1 to 3.3 and 4), run with
// QT_QPA_PLATFORM=offscreen. Widgets are found by the object names of the brief's contract:
//   groupList (QListWidget, 8 rows), paramPages (QStackedWidget of QScrollArea pages),
//   param:<key> (a ParamCard, a QFrame) with the children value, j1..j7, x, y, z, rangeLabel,
//   undoButton and perJoint:<key>, changeCount, applyStatusLabel.
// Besides them, it pins the two widget classes of src/studio/app:
//
//   namespace cobot::studio {
//   using Value = decltype(pipeline::KeyOverride::value);
//   class ParamCard : public QFrame {  // param_card.hpp
//    public:
//     explicit ParamCard(const ParamInfo& info, QWidget* parent = nullptr);
//     const std::string& key() const;
//     void show_value(const Value& v, bool changed);  // YAML units; never emits edited
//     void set_greyed(bool greyed);
//    signals:
//     void edited(const Value& v);
//     void undo_requested();
//   };
//   class ParamSpin : public QDoubleSpinBox {  // param_spin.hpp
//    public:
//     ParamSpin(double display_min, double display_max, const QString& display_unit,
//               QWidget* parent = nullptr);
//   };
//   }  // namespace cobot::studio
//
// The base pair of the window tests is budget_xarm7 (config/imperfections/budget_xarm7.yaml):
//   sensor.q_noise_std_rad 5.0e-5 (scalar in the file: all seven joints equal),
//   sensor.qd_from_diff true, actuator.motor_time_constant_s 0.002 (all joints equal),
//   computed_torque kp 400 -> omega 20 (config/budget_xarm7.yaml).
namespace cobot::studio {
namespace {

namespace fs = std::filesystem;
using Value = decltype(pipeline::KeyOverride::value);
using test::ModalHandler;
using test::pump;
using test::wait_until;

const fs::path kRepo = COBOT_REPO_DIR;
const fs::path kConfigDir = kRepo / "config";
const fs::path kOutput = fs::path(COBOT_TEST_OUTPUT_DIR) / "studio_config_tab";

constexpr const char* kBase = "budget_xarm7";
constexpr const char* kQNoise = "sensor.q_noise_std_rad";
constexpr const char* kHalTimeout = "config.safety.hal_timeout_ms";
constexpr int kJoints = 7;
constexpr int kPageCount = 8;
constexpr int kRunPage = 7;

// Design 3.1, in order; groupList row k selects paramPages page k.
const std::vector<QString> kPageTitles = {QStringLiteral("Controller"),
                                          QStringLiteral("Sensor / noise"),
                                          QStringLiteral("Actuator"),
                                          QStringLiteral("Friction"),
                                          QStringLiteral("Model & load"),
                                          QStringLiteral("Transmission"),
                                          QStringLiteral("Timing & comm"),
                                          QStringLiteral("Run")};

// Catalog group -> page (the brief's mapping; Timing and Comm share one page).
const std::map<std::string, int> kGroupPage = {{"Controller", 0},
                                               {"Sensor", 1},
                                               {"Actuator", 2},
                                               {"Friction", 3},
                                               {"Model", 4},
                                               {"Transmission", 5},
                                               {"Timing", 6},
                                               {"Comm", 6}};

QString qs(const std::string& s) {
  return QString::fromStdString(s);
}

const ParamInfo& info_of(const char* key) {
  const ParamInfo* p = find_param(key);
  if (p == nullptr) {
    throw std::runtime_error(std::string("no catalog entry ") + key);
  }
  return *p;
}

// The child of `parent` with this object name and type; throws (test failure) when missing.
template <typename T>
T* child(const QObject* parent, const QString& name) {
  T* w = parent->findChild<T*>(name);
  if (w == nullptr) {
    throw std::runtime_error("no child '" + name.toStdString() + "' of the expected type in '" +
                             parent->objectName().toStdString() + "'");
  }
  return w;
}

QDoubleSpinBox* spin_of(const QWidget* card, const char* name) {
  return child<QDoubleSpinBox>(card, QString::fromLatin1(name));
}

QDoubleSpinBox* joint_spin(const QWidget* card, int j) {  // j = 1..7
  return child<QDoubleSpinBox>(card, QStringLiteral("j%1").arg(j));
}

// Gives `w` the keyboard focus in its (activated) window.
void focus(QWidget* w) {
  QWidget* top = w->window();
  top->activateWindow();
  QApplication::setActiveWindow(top);
  w->setFocus(Qt::TabFocusReason);
  ASSERT_TRUE(wait_until([w] { return w->hasFocus(); }, 2000))
      << w->objectName().toStdString() << " never got the focus";
}

// Focuses the spin box, selects its number (QAbstractSpinBox::selectAll leaves the suffix
// alone), types `text` and presses Enter, as a user would.
void type_into(QDoubleSpinBox* spin, const QString& text) {
  ASSERT_NO_FATAL_FAILURE(focus(spin));
  spin->selectAll();
  QTest::keyClicks(spin, text);
  QTest::keyClick(spin, Qt::Key_Return);
  pump(30);
}

class ConfigTab : public ::testing::Test {
 protected:
  void SetUp() override {
    const ::testing::TestInfo* info = ::testing::UnitTest::GetInstance()->current_test_info();
    work_dir_ = kOutput / "work" / info->name();
    fs::remove_all(work_dir_);
    fs::create_directories(work_dir_);
    window_ = std::make_unique<MainWindow>(kConfigDir, work_dir_);
    window_->show();
    pump(20);
    ASSERT_NO_FATAL_FAILURE(select_base(kBase));
    ASSERT_TRUE(window_->studio_config().overrides.empty());
  }

  void TearDown() override { window_.reset(); }

  template <typename T>
  T* get(const char* name) const {
    return child<T>(window_.get(), QString::fromLatin1(name));
  }

  void select_base(const std::string& name) const {
    auto* combo = get<QComboBox>("baseCombo");
    const int index = combo->findText(qs(name));
    ASSERT_GE(index, 0) << name;
    combo->setCurrentIndex(index);
    pump(50);
    ASSERT_EQ(window_->studio_config().base.name, name) << modal_.log();
  }

  QFrame* card(const std::string& key) const {
    return child<QFrame>(window_.get(), qs("param:" + key));
  }

  QAbstractButton* toggle(const std::string& key) const {
    return child<QAbstractButton>(window_.get(), qs("perJoint:" + key));
  }

  QListWidget* group_list() const { return get<QListWidget>("groupList"); }
  QStackedWidget* pages() const { return get<QStackedWidget>("paramPages"); }

  // The Configuration tab and the page k of paramPages on screen.
  void show_page(int k) const {
    get<QTabWidget>("tabs")->setCurrentIndex(0);
    group_list()->setCurrentRow(k);
    pump(30);
    ASSERT_EQ(pages()->currentIndex(), k);
  }

  // The paramPages index of the page holding `w`; -1 when it is on none.
  int page_of(QWidget* w) const {
    QStackedWidget* stack = pages();
    for (QWidget* a = w; a != nullptr; a = a->parentWidget()) {
      if (a->parentWidget() == stack) {
        return stack->indexOf(a);
      }
    }
    return -1;
  }

  // The nearest ancestor QFrame with the property card=true.
  static const QFrame* card_of(const QWidget* w) {
    for (const QWidget* a = w->parentWidget(); a != nullptr; a = a->parentWidget()) {
      const auto* frame = qobject_cast<const QFrame*>(a);
      if (frame != nullptr && frame->property("card").toBool()) {
        return frame;
      }
    }
    return nullptr;
  }

  const pipeline::KeyOverride* override_of(const std::string& key) const {
    for (const pipeline::KeyOverride& o : window_->studio_config().overrides) {
      if (o.key == key) {
        return &o;
      }
    }
    return nullptr;
  }

  Value base_of(const std::string& key) const {
    const common::Result<pipeline::KeyOverride> v = base_value(window_->studio_config().base, key);
    if (!v.is_ok()) {
      throw std::runtime_error("no base value of " + key + ": " + v.reason());
    }
    return v.value().value;
  }

  std::vector<double> base_list(const std::string& key) const {
    const Value v = base_of(key);
    if (!std::holds_alternative<std::vector<double>>(v)) {
      throw std::runtime_error(key + " is not a list");
    }
    return std::get<std::vector<double>>(v);
  }

  // A widget outside every card that takes the focus (the base pair combo of the top bar).
  QWidget* elsewhere() const { return get<QComboBox>("baseCombo"); }

  ModalHandler modal_;
  fs::path work_dir_;
  std::unique_ptr<MainWindow> window_;
};

// ---------------------------------------------------------------------------------------------
// Layout

TEST_F(ConfigTab, GroupListHasTheEightPagesInOrderAndSelectsThem) {
  QListWidget* list = group_list();
  ASSERT_EQ(list->count(), kPageCount);
  ASSERT_EQ(pages()->count(), kPageCount);
  for (int k = 0; k < kPageCount; ++k) {
    // No override yet: no count suffix.
    EXPECT_EQ(list->item(k)->text(), kPageTitles.at(static_cast<std::size_t>(k))) << k;
  }
  for (const int k : {3, 0, 7, 1, 6, 2, 5, 4}) {
    list->setCurrentRow(k);
    pump(10);
    EXPECT_EQ(pages()->currentIndex(), k);
    const auto* page = qobject_cast<const QScrollArea*>(pages()->widget(k));
    ASSERT_NE(page, nullptr) << "page " << k << " is not a QScrollArea";
    EXPECT_TRUE(page->widgetResizable()) << k;
  }
}

TEST_F(ConfigTab, EveryCatalogKeySitsOnThePageOfItsGroup) {
  for (const ParamInfo& p : param_catalog()) {
    if (p.key == pipeline::kOmegaKey || p.key == kHalTimeout) {
      // Their own spin boxes, in cards at the top of the Controller page; no param: card.
      EXPECT_EQ(window_->findChild<QWidget*>(qs("param:" + p.key)), nullptr) << p.key;
      continue;
    }
    ASSERT_EQ(kGroupPage.count(p.group), 1U) << p.group;
    QFrame* c = card(p.key);
    EXPECT_EQ(page_of(c), kGroupPage.at(p.group)) << p.key;
    EXPECT_TRUE(c->property("card").toBool()) << p.key;
  }
  // Controller page top: omega with advancedGains in one card, HAL timeout in the next one.
  auto* omega = get<QWidget>("omegaSpin");
  QWidget* advanced = get<QCheckBox>("advancedGains");
  auto* hal = get<QWidget>("halTimeoutSpin");
  EXPECT_EQ(page_of(omega), 0);
  EXPECT_EQ(page_of(hal), 0);
  ASSERT_NE(card_of(omega), nullptr);
  ASSERT_NE(card_of(hal), nullptr);
  EXPECT_EQ(card_of(omega), card_of(advanced));
  EXPECT_NE(card_of(omega), card_of(hal));

  // Top to bottom: omega card, HAL timeout card, then the param: cards in catalog order (kp and
  // kd are hidden while advancedGains is off, so only the visible ones are compared).
  ASSERT_NO_FATAL_FAILURE(show_page(0));
  QWidget* content = qobject_cast<QScrollArea*>(pages()->widget(0))->widget();
  ASSERT_NE(content, nullptr);
  const auto top = [content](const QWidget* w) { return w->mapTo(content, QPoint(0, 0)).y(); };
  std::vector<int> ys = {top(card_of(omega)), top(card_of(hal))};
  for (const ParamInfo& p : param_catalog()) {
    if (p.group == "Controller" && p.key != pipeline::kOmegaKey && p.key != kHalTimeout &&
        !card(p.key)->isHidden()) {
      ys.push_back(top(card(p.key)));
    }
  }
  ASSERT_GE(ys.size(), 3U);
  EXPECT_TRUE(std::is_sorted(ys.begin(), ys.end()) &&
              std::adjacent_find(ys.begin(), ys.end()) == ys.end())
      << "Controller cards out of order";

  // The Run page holds the settings of a run.
  for (const char* name : {"speedSlider",
                           "goalPosSpin",
                           "goalRotSpin",
                           "pathPosSpin",
                           "pathRotSpin",
                           "waypointsEdit",
                           "durationSpin",
                           "seedSpin"}) {
    EXPECT_EQ(page_of(get<QWidget>(name)), kRunPage) << name;
  }
}

TEST_F(ConfigTab, FileButtonsAreFlatAndApplyIsThePrimaryButton) {
  for (const char* name : {"openButton", "saveButton", "exportButton", "resetButton"}) {
    EXPECT_TRUE(get<QPushButton>(name)->isFlat()) << name;
  }
  EXPECT_EQ(get<QPushButton>("openButton")->text(), QStringLiteral("Open..."));
  EXPECT_EQ(get<QPushButton>("saveButton")->text(), QStringLiteral("Save..."));
  EXPECT_EQ(get<QPushButton>("exportButton")->text(), QStringLiteral("Export for cobot_bench..."));
  EXPECT_EQ(get<QPushButton>("resetButton")->text(), QStringLiteral("Reset to base"));
  auto* apply = get<QPushButton>("applyGazeboButton");
  EXPECT_TRUE(apply->property("primary").toBool());
  EXPECT_FALSE(get<QPushButton>("openButton")->property("primary").toBool());

  // No overrides: the counter is hidden. No bridge server: "Gazebo: not connected".
  EXPECT_FALSE(get<QLabel>("changeCount")->isVisible());
  auto* status = get<QLabel>("applyStatusLabel");
  EXPECT_TRUE(wait_until(
      [status] { return status->text() == QStringLiteral("Gazebo: not connected"); }, 3000))
      << status->text().toStdString();
}

// ---------------------------------------------------------------------------------------------
// Display

TEST_F(ConfigTab, NoiseIsShownInMicroradiansCompactWithTheSharedFieldOnly) {
  ASSERT_NO_FATAL_FAILURE(show_page(1));
  const ParamInfo& p = info_of(kQNoise);
  ASSERT_EQ(p.display_scale, 1e6);
  ASSERT_EQ(p.display_unit, "\xC2\xB5rad");  // U+00B5 MICRO SIGN, UTF-8
  const QFrame* c = card(kQNoise);
  const std::vector<double> base = base_list(kQNoise);
  ASSERT_EQ(base.size(), static_cast<std::size_t>(kJoints));
  // budget_xarm7: 5.0e-5 rad on every joint -> 5.0e-5 * 1e6 = 50 urad, shown "50 urad".
  QDoubleSpinBox* value = spin_of(c, "value");
  EXPECT_FALSE(value->hasFocus());
  const QString expected = qs(format_display(base.at(0) * 1e6));
  EXPECT_TRUE(value->text().startsWith(expected))
      << value->text().toStdString() << " vs " << expected.toStdString();
  EXPECT_EQ(value->suffix(), QStringLiteral(" ") + QString::fromUtf8("\xC2\xB5rad"));
  EXPECT_TRUE(value->text().endsWith(QString::fromUtf8(" \xC2\xB5rad")))
      << value->text().toStdString();
  EXPECT_TRUE(value->isVisibleTo(c));
  for (int j = 1; j <= kJoints; ++j) {
    EXPECT_FALSE(joint_spin(c, j)->isVisibleTo(c)) << "j" << j << " shown while collapsed";
  }
  EXPECT_FALSE(child<QWidget>(c, QStringLiteral("rangeLabel"))->isVisibleTo(c));
  EXPECT_FALSE(child<QAbstractButton>(c, QStringLiteral("undoButton"))->isVisibleTo(c));
  EXPECT_TRUE(toggle(kQNoise)->isEnabled());
  EXPECT_FALSE(toggle(kQNoise)->isChecked());
  EXPECT_FALSE(value->property("changed").toBool());
  // The card names the key: title (label), hint (description) and the raw key.
  bool title = false;
  bool key_name = false;
  bool hint = false;
  for (const QLabel* label : c->findChildren<QLabel*>()) {
    title = title || label->text() == qs(p.label);
    key_name = key_name || (label->property("keyName").toBool() && label->text() == qs(p.key));
    hint = hint || (label->property("hint").toBool() && label->text().contains(qs(p.description)));
  }
  EXPECT_TRUE(title);
  EXPECT_TRUE(key_name);
  EXPECT_TRUE(hint);
}

// ---------------------------------------------------------------------------------------------
// No spurious override (anti-goal: a value the user did not edit is never rewritten)

TEST_F(ConfigTab, FocusingAndLeavingFieldsWithoutTypingWritesNothing) {
  ASSERT_NO_FATAL_FAILURE(show_page(1));
  const QFrame* noise = card(kQNoise);
  ASSERT_NO_FATAL_FAILURE(focus(spin_of(noise, "value")));
  ASSERT_NO_FATAL_FAILURE(focus(elsewhere()));
  pump(30);
  EXPECT_TRUE(window_->studio_config().overrides.empty())
      << describe_diff(window_->studio_config());

  toggle(kQNoise)->click();
  pump(30);
  ASSERT_TRUE(joint_spin(noise, 3)->isVisibleTo(noise));
  EXPECT_TRUE(window_->studio_config().overrides.empty())
      << describe_diff(window_->studio_config());
  ASSERT_NO_FATAL_FAILURE(focus(joint_spin(noise, 3)));
  ASSERT_NO_FATAL_FAILURE(focus(elsewhere()));
  pump(30);
  EXPECT_TRUE(window_->studio_config().overrides.empty())
      << describe_diff(window_->studio_config());

  // The same on every visible numeric field of the Actuator and Transmission pages (budget
  // deadband_nm and the stiffnesses differ by joint, so their j fields are open).
  for (const int page : {2, 5}) {
    ASSERT_NO_FATAL_FAILURE(show_page(page));
    for (const ParamInfo& p : param_catalog()) {
      if (kGroupPage.count(p.group) == 0 || kGroupPage.at(p.group) != page) {
        continue;
      }
      const QFrame* c = card(p.key);
      for (QDoubleSpinBox* s : c->findChildren<QDoubleSpinBox*>()) {
        if (s->isVisibleTo(c) && s->isEnabled()) {
          ASSERT_NO_FATAL_FAILURE(focus(s));
          ASSERT_NO_FATAL_FAILURE(focus(elsewhere()));
        }
      }
    }
    pump(30);
    EXPECT_TRUE(window_->studio_config().overrides.empty())
        << "page " << page << ": " << describe_diff(window_->studio_config());
  }
}

// ---------------------------------------------------------------------------------------------
// Shared edit

TEST_F(ConfigTab, SharedFieldSetsAllSevenJoints) {
  ASSERT_NO_FATAL_FAILURE(show_page(1));
  const QFrame* c = card(kQNoise);
  ASSERT_NO_FATAL_FAILURE(type_into(spin_of(c, "value"), QStringLiteral("5")));
  const pipeline::KeyOverride* o = override_of(kQNoise);
  ASSERT_NE(o, nullptr) << describe_diff(window_->studio_config());
  ASSERT_TRUE(std::holds_alternative<std::vector<double>>(o->value));
  const auto& list = std::get<std::vector<double>>(o->value);
  ASSERT_EQ(list.size(), static_cast<std::size_t>(kJoints));
  for (const double x : list) {
    EXPECT_NEAR(x, 5e-6, 1e-15);  // 5 urad / 1e6 = 5e-6 rad
  }
  EXPECT_EQ(window_->studio_config().overrides.size(), 1U);

  auto* count = get<QLabel>("changeCount");
  EXPECT_TRUE(count->isVisible());
  EXPECT_TRUE(count->text().startsWith(QStringLiteral("1 change"))) << count->text().toStdString();
  EXPECT_TRUE(group_list()->item(1)->text().endsWith(QStringLiteral("(1)")))
      << group_list()->item(1)->text().toStdString();
  EXPECT_EQ(group_list()->item(0)->text(), kPageTitles.at(0));
  EXPECT_TRUE(child<QAbstractButton>(c, QStringLiteral("undoButton"))->isVisibleTo(c));
  EXPECT_TRUE(spin_of(c, "value")->property("changed").toBool());

  // Typing the base value back (50 urad) is no override (MainWindow's same() rule).
  ASSERT_NO_FATAL_FAILURE(type_into(spin_of(c, "value"), QStringLiteral("50")));
  EXPECT_EQ(override_of(kQNoise), nullptr) << describe_diff(window_->studio_config());
  EXPECT_FALSE(count->isVisible());
  EXPECT_EQ(group_list()->item(1)->text(), kPageTitles.at(1));
  EXPECT_FALSE(child<QAbstractButton>(c, QStringLiteral("undoButton"))->isVisibleTo(c));
}

// ---------------------------------------------------------------------------------------------
// Per-joint edit keeps the other joints bit-for-bit
//
// No per-joint value of the six base pairs in config/ has more than three significant digits
// (the largest are e.g. 0.435, 40800, 8.7e-4), so the full-precision values are put in place
// with set_override, exactly as an earlier edit or an opened studio file would.

TEST_F(ConfigTab, EditingOneJointKeepsTheOtherJointsBitForBit) {
  const std::string key = "actuator.motor_time_constant_s";
  const ParamInfo& p = info_of(key.c_str());
  ASSERT_EQ(p.kind, ParamKind::kPerJoint);
  ASSERT_GT(p.display_scale, 0.0);
  ASSERT_NO_FATAL_FAILURE(show_page(2));
  const QFrame* c = card(key);

  // Case 1: seven equal values, expanded by the user.
  const std::vector<double> equal(kJoints, 1.2345678901234567e-3);
  // Precondition: more than three significant digits, so a 3-digit display would change it
  // (1.2345678901234567e-3 -> "0.00123" -> 1.23e-3).
  ASSERT_NE(QString::number(equal.at(0), 'g', 3).toDouble(), equal.at(0));
  ASSERT_NE(QString::number(equal.at(0) * p.display_scale, 'g', 3).toDouble() / p.display_scale,
            equal.at(0));
  window_->set_override(pipeline::KeyOverride{key, equal});
  pump(30);
  ASSERT_TRUE(toggle(key)->isEnabled());
  if (!toggle(key)->isChecked()) {
    toggle(key)->click();
    pump(30);
  }
  ASSERT_TRUE(joint_spin(c, 2)->isVisibleTo(c));
  // 0.5 in the display unit -> 0.5 / display_scale in the YAML unit.
  ASSERT_NO_FATAL_FAILURE(type_into(joint_spin(c, 2), QStringLiteral("0.5")));
  {
    const pipeline::KeyOverride* o = override_of(key);
    ASSERT_NE(o, nullptr);
    ASSERT_TRUE(std::holds_alternative<std::vector<double>>(o->value));
    const auto& v = std::get<std::vector<double>>(o->value);
    ASSERT_EQ(v.size(), static_cast<std::size_t>(kJoints));
    EXPECT_DOUBLE_EQ(v.at(1), 0.5 / p.display_scale);
    for (std::size_t j = 0; j < v.size(); ++j) {
      if (j != 1) {
        EXPECT_EQ(v.at(j), equal.at(j)) << "joint " << j + 1 << " rewritten";
      }
    }
  }

  // Case 2: seven distinct values (the card opens the joints itself); edit j4.
  const std::vector<double> distinct = {1.0123456789012345e-3,
                                        1.1234567890123457e-3,
                                        1.2345678901234567e-3,
                                        1.3456789012345678e-3,
                                        1.4567890123456789e-3,
                                        1.5678901234567890e-3,
                                        1.6789012345678901e-3};
  window_->set_override(pipeline::KeyOverride{key, distinct});
  pump(30);
  ASSERT_TRUE(joint_spin(c, 4)->isVisibleTo(c));
  ASSERT_NO_FATAL_FAILURE(type_into(joint_spin(c, 4), QStringLiteral("0.25")));
  {
    const pipeline::KeyOverride* o = override_of(key);
    ASSERT_NE(o, nullptr);
    const auto& v = std::get<std::vector<double>>(o->value);
    ASSERT_EQ(v.size(), static_cast<std::size_t>(kJoints));
    EXPECT_DOUBLE_EQ(v.at(3), 0.25 / p.display_scale);
    for (std::size_t j = 0; j < v.size(); ++j) {
      if (j != 3) {
        EXPECT_EQ(v.at(j), distinct.at(j)) << "joint " << j + 1 << " rewritten";
      }
    }
  }
}

// ---------------------------------------------------------------------------------------------
// Differs by joint, undo

TEST_F(ConfigTab, DifferentJointValuesOpenTheJointsAndLockTheToggle) {
  ASSERT_NO_FATAL_FAILURE(show_page(1));
  const QFrame* c = card(kQNoise);
  const std::vector<double> two = {1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 2e-5, 2e-5};
  window_->set_override(pipeline::KeyOverride{kQNoise, two});
  pump(30);
  EXPECT_TRUE(toggle(kQNoise)->isChecked());
  EXPECT_FALSE(toggle(kQNoise)->isEnabled());
  auto* range = child<QLabel>(c, QStringLiteral("rangeLabel"));
  EXPECT_TRUE(range->isVisibleTo(c));
  // min 1e-5 rad, max 2e-5 rad, x 1e6 -> "10" and "20" urad.
  const QString expected = QStringLiteral("differs by joint: ") + qs(format_display(1e-5 * 1e6)) +
                           QStringLiteral(" to ") + qs(format_display(2e-5 * 1e6)) +
                           QStringLiteral(" ") + qs(info_of(kQNoise).display_unit);
  EXPECT_EQ(range->text(), expected);
  EXPECT_FALSE(spin_of(c, "value")->isVisibleTo(c));
  for (int j = 1; j <= kJoints; ++j) {
    EXPECT_TRUE(joint_spin(c, j)->isVisibleTo(c)) << "j" << j;
  }
  for (const QLabel* label : c->findChildren<QLabel*>()) {
    if (label->text() == QStringLiteral("all joints")) {
      EXPECT_FALSE(label->isVisibleTo(c)) << "\"all joints\" shown while the joints differ";
    }
  }
  EXPECT_TRUE(child<QAbstractButton>(c, QStringLiteral("undoButton"))->isVisibleTo(c));
}

TEST_F(ConfigTab, UndoRemovesTheOverrideAndReturnsToTheSharedView) {
  ASSERT_NO_FATAL_FAILURE(show_page(1));
  const QFrame* c = card(kQNoise);
  const std::vector<double> two = {1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 2e-5, 2e-5};
  window_->set_override(pipeline::KeyOverride{kQNoise, two});
  pump(30);
  ASSERT_FALSE(toggle(kQNoise)->isEnabled());
  auto* undo = child<QAbstractButton>(c, QStringLiteral("undoButton"));
  ASSERT_TRUE(undo->isVisibleTo(c));
  EXPECT_EQ(undo->toolTip(), QStringLiteral("Back to the base value"));
  undo->click();
  pump(30);
  EXPECT_EQ(override_of(kQNoise), nullptr);
  EXPECT_TRUE(window_->studio_config().overrides.empty());
  EXPECT_TRUE(toggle(kQNoise)->isEnabled());
  EXPECT_FALSE(toggle(kQNoise)->isChecked());  // the user never expanded it
  EXPECT_TRUE(spin_of(c, "value")->isVisibleTo(c));
  EXPECT_FALSE(child<QWidget>(c, QStringLiteral("rangeLabel"))->isVisibleTo(c));
  for (int j = 1; j <= kJoints; ++j) {
    EXPECT_FALSE(joint_spin(c, j)->isVisibleTo(c)) << "j" << j;
  }
  EXPECT_FALSE(undo->isVisibleTo(c));
  EXPECT_FALSE(spin_of(c, "value")->property("changed").toBool());
  EXPECT_FALSE(get<QLabel>("changeCount")->isVisible());

  // Expanded by the user: it stays expanded after the values are equal again.
  toggle(kQNoise)->click();
  pump(30);
  ASSERT_TRUE(toggle(kQNoise)->isChecked());
  window_->set_override(pipeline::KeyOverride{kQNoise, two});
  pump(30);
  EXPECT_FALSE(toggle(kQNoise)->isEnabled());
  undo->click();
  pump(30);
  EXPECT_TRUE(window_->studio_config().overrides.empty());
  EXPECT_TRUE(toggle(kQNoise)->isEnabled());
  EXPECT_TRUE(toggle(kQNoise)->isChecked());
  EXPECT_TRUE(joint_spin(c, 1)->isVisibleTo(c));
  EXPECT_FALSE(child<QWidget>(c, QStringLiteral("rangeLabel"))->isVisibleTo(c));
}

// ---------------------------------------------------------------------------------------------
// xyz, advanced gains, bool

TEST_F(ConfigTab, PayloadOffsetIsThreeFieldsXYZ) {
  const std::string key = "model.payload_offset_m";
  ASSERT_TRUE(info_of(key.c_str()).xyz);
  ASSERT_NO_FATAL_FAILURE(show_page(4));
  const QFrame* c = card(key);
  for (const char* name : {"x", "y", "z"}) {
    EXPECT_TRUE(spin_of(c, name)->isVisibleTo(c)) << name;
  }
  for (const QObject* o : c->findChildren<QObject*>()) {
    EXPECT_FALSE(o->objectName().startsWith(QStringLiteral("perJoint:")))
        << o->objectName().toStdString();
  }
  EXPECT_EQ(window_->findChild<QWidget*>(qs("perJoint:" + key)), nullptr);
}

TEST_F(ConfigTab, AdvancedGainsRevealsKpKdAndRemovesTheOmegaOverride) {
  ASSERT_NO_FATAL_FAILURE(show_page(0));
  const std::string kp = "config.computed_torque.kp";
  const std::string kd = "config.computed_torque.kd";
  auto* advanced = get<QCheckBox>("advancedGains");
  ASSERT_FALSE(advanced->isChecked());
  EXPECT_TRUE(card(kp)->isHidden());
  EXPECT_TRUE(card(kd)->isHidden());

  // budget_xarm7 omega 20; 30 is an override.
  window_->set_override(pipeline::KeyOverride{pipeline::kOmegaKey, 30.0});
  pump(30);
  ASSERT_NE(override_of(pipeline::kOmegaKey), nullptr);

  advanced->click();
  pump(30);
  EXPECT_TRUE(advanced->isChecked());
  EXPECT_FALSE(card(kp)->isHidden());
  EXPECT_FALSE(card(kd)->isHidden());
  EXPECT_TRUE(card(kp)->isVisible());
  EXPECT_EQ(override_of(pipeline::kOmegaKey), nullptr) << describe_diff(window_->studio_config());
  EXPECT_FALSE(get<QWidget>("omegaSpin")->isEnabled());

  advanced->click();
  pump(30);
  EXPECT_FALSE(advanced->isChecked());
  EXPECT_TRUE(card(kp)->isHidden());
  EXPECT_TRUE(card(kd)->isHidden());
  EXPECT_TRUE(get<QWidget>("omegaSpin")->isEnabled());
}

TEST_F(ConfigTab, BoolSwitchTogglesTheOverride) {
  const std::string key = "sensor.qd_from_diff";
  ASSERT_NO_FATAL_FAILURE(show_page(1));
  ASSERT_EQ(base_of(key), Value(true));  // budget_xarm7: qd_from_diff true
  const QFrame* c = card(key);
  auto* value = child<QAbstractButton>(c, QStringLiteral("value"));
  EXPECT_TRUE(value->isCheckable());
  EXPECT_TRUE(value->isChecked());
  EXPECT_EQ(value->text(), QStringLiteral("On"));

  value->click();
  pump(30);
  const pipeline::KeyOverride* o = override_of(key);
  ASSERT_NE(o, nullptr);
  EXPECT_EQ(o->value, Value(false));
  EXPECT_FALSE(value->isChecked());
  EXPECT_EQ(value->text(), QStringLiteral("Off"));
  EXPECT_TRUE(child<QAbstractButton>(c, QStringLiteral("undoButton"))->isVisibleTo(c));

  value->click();
  pump(30);
  EXPECT_EQ(override_of(key), nullptr);  // back to the base value: no override
  EXPECT_TRUE(window_->studio_config().overrides.empty());
}

// ---------------------------------------------------------------------------------------------
// Unchanged flows: Save, Open, Reset

TEST_F(ConfigTab, SaveOpenAndResetKeepTheOverridesAndTheMarks) {
  ASSERT_NO_FATAL_FAILURE(show_page(1));
  const QFrame* c = card(kQNoise);
  ASSERT_NO_FATAL_FAILURE(type_into(spin_of(c, "value"), QStringLiteral("5")));
  ASSERT_NE(override_of(kQNoise), nullptr);
  const pipeline::KeyOverride saved = *override_of(kQNoise);

  const fs::path dir = kOutput / "flows";
  fs::remove_all(dir);
  fs::create_directories(dir);
  const fs::path file = dir / "studio.yaml";
  ASSERT_TRUE(window_->save_studio(qs(file.string())));

  get<QPushButton>("resetButton")->click();
  ASSERT_TRUE(wait_until([&] { return window_->studio_config().overrides.empty(); }, 2000))
      << modal_.log();
  EXPECT_FALSE(get<QLabel>("changeCount")->isVisible());
  EXPECT_FALSE(child<QAbstractButton>(c, QStringLiteral("undoButton"))->isVisibleTo(c));
  EXPECT_FALSE(spin_of(c, "value")->property("changed").toBool());
  EXPECT_EQ(group_list()->item(1)->text(), kPageTitles.at(1));

  ASSERT_TRUE(window_->open_studio(qs(file.string())));
  pump(50);
  ASSERT_EQ(window_->studio_config().overrides.size(), 1U);
  const pipeline::KeyOverride* o = override_of(kQNoise);
  ASSERT_NE(o, nullptr);
  ASSERT_TRUE(std::holds_alternative<std::vector<double>>(o->value));
  const auto& got = std::get<std::vector<double>>(o->value);
  const auto& want = std::get<std::vector<double>>(saved.value);
  ASSERT_EQ(got.size(), want.size());
  for (std::size_t j = 0; j < got.size(); ++j) {
    EXPECT_NEAR(got.at(j), want.at(j), 1e-18) << j;
  }
  EXPECT_TRUE(child<QAbstractButton>(c, QStringLiteral("undoButton"))->isVisibleTo(c));
  EXPECT_TRUE(spin_of(c, "value")->property("changed").toBool());
  EXPECT_TRUE(get<QLabel>("changeCount")->isVisible());
  EXPECT_TRUE(group_list()->item(1)->text().endsWith(QStringLiteral("(1)")));

  // Reset clears every override, several at once.
  window_->set_override(pipeline::KeyOverride{"sensor.qd_from_diff", false});
  window_->set_override(pipeline::KeyOverride{"model.payload_kg", 1.5});
  pump(30);
  ASSERT_EQ(window_->studio_config().overrides.size(), 3U);
  EXPECT_TRUE(get<QLabel>("changeCount")->text().startsWith(QStringLiteral("3 change")));
  EXPECT_TRUE(group_list()->item(1)->text().endsWith(QStringLiteral("(2)")));
  EXPECT_TRUE(group_list()->item(4)->text().endsWith(QStringLiteral("(1)")));
  get<QPushButton>("resetButton")->click();
  ASSERT_TRUE(wait_until([&] { return window_->studio_config().overrides.empty(); }, 2000));
  EXPECT_FALSE(get<QLabel>("changeCount")->isVisible());
  for (int k = 0; k < kPageCount; ++k) {
    EXPECT_EQ(group_list()->item(k)->text(), kPageTitles.at(static_cast<std::size_t>(k))) << k;
  }
}

// ---------------------------------------------------------------------------------------------
// ParamCard and ParamSpin on their own

TEST(ParamCard, NamesItselfAndShowsAValueWithoutEmitting) {
  const ParamInfo& p = info_of("model.payload_kg");
  ParamCard card(p);
  EXPECT_EQ(card.objectName(), QStringLiteral("param:model.payload_kg"));
  EXPECT_TRUE(card.property("card").toBool());
  EXPECT_EQ(card.key(), "model.payload_kg");
  int edits = 0;
  int undos = 0;
  QObject::connect(&card, &ParamCard::edited, [&edits](const Value&) { ++edits; });
  QObject::connect(&card, &ParamCard::undo_requested, [&undos] { ++undos; });

  card.show_value(Value(2.5), false);
  QDoubleSpinBox* value = spin_of(&card, "value");
  EXPECT_EQ(value->cleanText(), qs(format_display(2.5 * p.display_scale)));
  auto* undo = child<QAbstractButton>(&card, QStringLiteral("undoButton"));
  EXPECT_FALSE(undo->isVisibleTo(&card));
  EXPECT_FALSE(value->property("changed").toBool());

  card.show_value(Value(1.25), true);
  EXPECT_TRUE(undo->isVisibleTo(&card));
  EXPECT_TRUE(value->property("changed").toBool());
  EXPECT_EQ(edits, 0) << "show_value emitted edited";

  undo->click();
  EXPECT_EQ(undos, 1);
  EXPECT_EQ(edits, 0);

  card.set_greyed(true);
  EXPECT_FALSE(card.isEnabled());
  EXPECT_TRUE(card.toolTip().contains(QStringLiteral("not supported on Gazebo")));
  card.set_greyed(false);
  EXPECT_TRUE(card.isEnabled());
  EXPECT_TRUE(card.toolTip().isEmpty());
}

TEST(ParamCard, BoolAndChoiceEmitTheWholeValueOnChange) {
  ParamCard flag(info_of("sensor.qd_from_diff"));
  std::vector<Value> flag_edits;
  QObject::connect(
      &flag, &ParamCard::edited, [&flag_edits](const Value& v) { flag_edits.push_back(v); });
  flag.show_value(Value(true), false);
  auto* button = child<QAbstractButton>(&flag, QStringLiteral("value"));
  EXPECT_TRUE(button->isChecked());
  EXPECT_EQ(button->text(), QStringLiteral("On"));
  EXPECT_TRUE(flag_edits.empty());
  button->click();
  ASSERT_EQ(flag_edits.size(), 1U);
  EXPECT_EQ(flag_edits.front(), Value(false));

  ParamCard source(info_of("sensor.position_source"));
  std::vector<Value> source_edits;
  QObject::connect(
      &source, &ParamCard::edited, [&source_edits](const Value& v) { source_edits.push_back(v); });
  source.show_value(Value(std::string("link")), false);
  auto* combo = child<QComboBox>(&source, QStringLiteral("value"));
  EXPECT_EQ(combo->currentText(), QStringLiteral("link"));
  EXPECT_TRUE(source_edits.empty());
  combo->setCurrentIndex(combo->findText(QStringLiteral("motor")));
  ASSERT_EQ(source_edits.size(), 1U);
  EXPECT_EQ(source_edits.front(), Value(std::string("motor")));
}

TEST(ParamCard, PerJointSharedFieldReplacesEveryElement) {
  const ParamInfo& p = info_of(kQNoise);
  QWidget host;
  auto* layout = new QHBoxLayout(&host);
  auto* card = new ParamCard(p);
  auto* other = new QLineEdit;
  layout->addWidget(card);
  layout->addWidget(other);
  host.show();
  std::vector<Value> edits;
  QObject::connect(card, &ParamCard::edited, [&edits](const Value& v) { edits.push_back(v); });
  const std::vector<double> held(kJoints, 3.3333333333333333e-5);
  card->show_value(Value(held), false);
  QDoubleSpinBox* value = spin_of(card, "value");
  // Leaving without typing: nothing.
  ASSERT_NO_FATAL_FAILURE(focus(value));
  ASSERT_NO_FATAL_FAILURE(focus(other));
  EXPECT_TRUE(edits.empty());
  // 7 urad -> 7e-6 rad on all seven joints.
  ASSERT_NO_FATAL_FAILURE(type_into(value, QStringLiteral("7")));
  ASSERT_EQ(edits.size(), 1U);
  ASSERT_TRUE(std::holds_alternative<std::vector<double>>(edits.front()));
  const auto& v = std::get<std::vector<double>>(edits.front());
  ASSERT_EQ(v.size(), static_cast<std::size_t>(kJoints));
  for (const double x : v) {
    EXPECT_DOUBLE_EQ(x, 7.0 / p.display_scale);
  }
}

TEST(ParamSpin, CompactWithoutFocusFullPrecisionWithFocus) {
  const double inf = std::numeric_limits<double>::infinity();
  QWidget host;
  auto* layout = new QHBoxLayout(&host);
  auto* spin = new ParamSpin(-inf, inf, QString::fromUtf8("\xC2\xB5rad"));
  auto* other = new QLineEdit;
  layout->addWidget(spin);
  layout->addWidget(other);
  host.show();

  EXPECT_EQ(spin->minimum(), -1e300);
  EXPECT_EQ(spin->maximum(), 1e300);
  EXPECT_EQ(spin->decimals(), 12);
  EXPECT_FALSE(spin->keyboardTracking());
  EXPECT_EQ(spin->suffix(), QString::fromUtf8(" \xC2\xB5rad"));

  // 3.14159: compact "3.14" (format_display, 3 significant digits); with focus
  // QString::number(3.14159, 'g', 12) = "3.14159".
  spin->setValue(3.14159);
  EXPECT_DOUBLE_EQ(spin->value(), 3.14159);
  EXPECT_EQ(spin->cleanText(), qs(format_display(3.14159)));
  ASSERT_NO_FATAL_FAILURE(focus(spin));
  EXPECT_EQ(spin->cleanText(), QStringLiteral("3.14159"));
  ASSERT_NO_FATAL_FAILURE(focus(other));
  EXPECT_EQ(spin->cleanText(), qs(format_display(3.14159)));
  EXPECT_DOUBLE_EQ(spin->value(), 3.14159);  // leaving did not round the value

  // A typed full-precision value round-trips (9 decimals < 12).
  ASSERT_NO_FATAL_FAILURE(type_into(spin, QStringLiteral("123.456789012")));
  EXPECT_NEAR(spin->value(), 123.456789012, 1e-12);

  const ParamSpin unitless(0.0, 10.0, QString());
  EXPECT_TRUE(unitless.suffix().isEmpty());
  EXPECT_EQ(unitless.minimum(), 0.0);
  EXPECT_EQ(unitless.maximum(), 10.0);
}

}  // namespace
}  // namespace cobot::studio
