// Enter in a field the user did not type in writes nothing (design 2: an unedited value is never
// rewritten; review follow-up of feature studio-config-layout). The override carries 16
// significant digits, more than a field shows, so a write-back would change it.

#include <QAbstractSpinBox>
#include <QApplication>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QStackedWidget>
#include <QString>
#include <QTabWidget>
#include <QTest>
#include <QWidget>
#include <filesystem>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/pipeline/overrides.hpp"
#include "cobot/studio/studio_config.hpp"

#include "main_window.hpp"
#include "studio_app_test_util.hpp"

namespace fs = std::filesystem;
using cobot::pipeline::KeyOverride;
using cobot::studio::MainWindow;
using cobot::studio::test::pump;
using cobot::studio::test::wait_until;

namespace {

const fs::path kRepo = COBOT_REPO_DIR;
const fs::path kOutput = fs::path(COBOT_TEST_OUTPUT_DIR) / "studio_config_enter";
constexpr const char* kBase = "budget_xarm7";
constexpr const char* kKey = "sensor.q_noise_std_rad";

class ConfigEnter : public ::testing::Test {
 protected:
  void SetUp() override {
    fs::remove_all(kOutput);
    fs::create_directories(kOutput / "work");
    window_ = std::make_unique<MainWindow>(kRepo / "config", kOutput / "work");
    window_->show();
    pump(20);
    auto* combo = window_->findChild<QComboBox*>(QStringLiteral("baseCombo"));
    ASSERT_NE(combo, nullptr);
    const int index = combo->findText(QString::fromLatin1(kBase));
    ASSERT_GE(index, 0);
    combo->setCurrentIndex(index);
    pump(50);
    ASSERT_EQ(window_->studio_config().base.name, kBase);
    ASSERT_TRUE(window_->studio_config().overrides.empty());
    ASSERT_NO_FATAL_FAILURE(show_card());
  }

  // Bring the Configuration tab and the page holding the card to the front.
  void show_card() {
    auto* tabs = window_->findChild<QTabWidget*>(QStringLiteral("tabs"));
    auto* tab = window_->findChild<QWidget*>(QStringLiteral("configTab"));
    auto* stack = window_->findChild<QStackedWidget*>(QStringLiteral("paramPages"));
    auto* card = window_->findChild<QFrame*>(QStringLiteral("param:") + QString::fromLatin1(kKey));
    ASSERT_NE(tabs, nullptr);
    ASSERT_NE(tab, nullptr);
    ASSERT_NE(stack, nullptr);
    ASSERT_NE(card, nullptr);
    tabs->setCurrentWidget(tab);
    QWidget* page = card;
    while (page != nullptr && page->parentWidget() != stack) {
      page = page->parentWidget();
    }
    ASSERT_NE(page, nullptr);
    stack->setCurrentWidget(page);
    pump(30);
  }

  void TearDown() override { window_.reset(); }

  QDoubleSpinBox* field(const char* name) const {
    auto* card = window_->findChild<QFrame*>(QStringLiteral("param:") + QString::fromLatin1(kKey));
    return card == nullptr ? nullptr : card->findChild<QDoubleSpinBox*>(QString::fromLatin1(name));
  }

  // Focus `w` (activated window), press Enter without typing, move the focus away.
  static void enter_untouched(QWidget* w, QWidget* elsewhere) {
    QWidget* top = w->window();
    top->activateWindow();
    QApplication::setActiveWindow(top);
    w->setFocus(Qt::TabFocusReason);
    ASSERT_TRUE(wait_until([w] { return w->hasFocus(); }, 2000));
    QTest::keyClick(w, Qt::Key_Return);
    pump(30);
    elsewhere->setFocus(Qt::TabFocusReason);
    pump(30);
  }

  std::vector<double> stored() const {
    for (const KeyOverride& o : window_->studio_config().overrides) {
      if (o.key == kKey) {
        const auto* list = std::get_if<std::vector<double>>(&o.value);
        return list == nullptr ? std::vector<double>{} : *list;
      }
    }
    return {};
  }

  std::unique_ptr<MainWindow> window_;
};

TEST_F(ConfigEnter, EnterInAnUntouchedJointFieldKeepsEveryDigit) {
  // Distinct values: the card opens J1..J7 itself.
  const std::vector<double> values = {1.2345678901234567e-6,
                                      2.3456789012345678e-6,
                                      3.4567890123456789e-6,
                                      4.5678901234567891e-6,
                                      5.6789012345678912e-6,
                                      6.7890123456789123e-6,
                                      7.8901234567891234e-6};
  window_->set_override(KeyOverride{kKey, values});
  pump(30);
  ASSERT_EQ(stored(), values);
  QDoubleSpinBox* j1 = field("j1");
  QDoubleSpinBox* j2 = field("j2");
  ASSERT_NE(j1, nullptr);
  ASSERT_NE(j2, nullptr);
  ASSERT_TRUE(j1->isVisible());

  ASSERT_NO_FATAL_FAILURE(enter_untouched(j1, j2));
  ASSERT_NO_FATAL_FAILURE(enter_untouched(j2, j1));
  EXPECT_EQ(stored(), values);  // exact, element by element
  EXPECT_EQ(window_->studio_config().overrides.size(), 1U);
}

TEST_F(ConfigEnter, EnterInAnUntouchedSharedFieldKeepsEveryDigit) {
  // Equal values: the card shows the shared field.
  const std::vector<double> values(7, 1.2345678901234567e-6);
  window_->set_override(KeyOverride{kKey, values});
  pump(30);
  ASSERT_EQ(stored(), values);
  QDoubleSpinBox* shared = field("value");
  ASSERT_NE(shared, nullptr);
  ASSERT_TRUE(shared->isVisible());
  auto* other = window_->findChild<QWidget*>(QStringLiteral("baseCombo"));
  ASSERT_NE(other, nullptr);

  ASSERT_NO_FATAL_FAILURE(enter_untouched(shared, other));
  EXPECT_EQ(stored(), values);
  EXPECT_EQ(window_->studio_config().overrides.size(), 1U);
}

TEST_F(ConfigEnter, EnterInAnUntouchedFieldOnTheBaseValueAddsNoOverride) {
  QDoubleSpinBox* shared = field("value");
  ASSERT_NE(shared, nullptr);
  auto* other = window_->findChild<QWidget*>(QStringLiteral("baseCombo"));
  ASSERT_NE(other, nullptr);
  ASSERT_NO_FATAL_FAILURE(enter_untouched(shared, other));
  EXPECT_TRUE(window_->studio_config().overrides.empty());
}

}  // namespace
