// save_screenshots (feature studio-shots): one PNG per Configuration page, in groupList order,
// then the Sensor / noise page at noise level x2; the window's configuration is unchanged after.

#include <QAbstractButton>
#include <QApplication>
#include <QColor>
#include <QCoreApplication>
#include <QImage>
#include <QListWidget>
#include <QString>
#include <QStringList>
#include <cstdio>
#include <filesystem>
#include <memory>

#include <gtest/gtest.h>

#include "main_window.hpp"
#include "screenshots.hpp"
#include "theme.hpp"

namespace fs = std::filesystem;
using cobot::studio::MainWindow;

namespace {

const fs::path kRepo = COBOT_REPO_DIR;
const fs::path kOutput = fs::path(COBOT_TEST_OUTPUT_DIR) / "studio_shots";

// More than one colour in the image: a blank grab is a single colour.
bool has_content(const QImage& image) {
  const QRgb first = image.pixel(0, 0);
  for (int y = 0; y < image.height(); y += 7) {
    for (int x = 0; x < image.width(); x += 7) {
      if (image.pixel(x, y) != first) {
        return true;
      }
    }
  }
  return false;
}

class StudioShots : public ::testing::Test {
 protected:
  void SetUp() override {
    auto* app = qobject_cast<QApplication*>(QCoreApplication::instance());
    ASSERT_NE(app, nullptr);
    ASSERT_TRUE(cobot::studio::apply_theme(*app));
    fs::remove_all(kOutput);
    fs::create_directories(kOutput / "work");
    window_ = std::make_unique<MainWindow>(kRepo / "config", kOutput / "work");
    window_->resize(1400, 900);
    window_->show();
    QCoreApplication::processEvents();
  }

  std::unique_ptr<MainWindow> window_;
};

TEST_F(StudioShots, OnePngPerPageThenTheNoiseView) {
  const auto* list = window_->findChild<QListWidget*>(QStringLiteral("groupList"));
  ASSERT_NE(list, nullptr);
  ASSERT_EQ(list->count(), 8);
  const int row_before = list->currentRow();

  const QString dir = QString::fromStdString((kOutput / "shots").string());
  const QStringList files = cobot::studio::save_screenshots(*window_, dir);

  // Eight pages (config-1-controller.png .. config-8-run.png) and config-noise-x2.png.
  const QStringList expected = {QStringLiteral("config-1-controller.png"),
                                QStringLiteral("config-2-sensor-noise.png"),
                                QStringLiteral("config-3-actuator.png"),
                                QStringLiteral("config-4-friction.png"),
                                QStringLiteral("config-5-model-load.png"),
                                QStringLiteral("config-6-transmission.png"),
                                QStringLiteral("config-7-timing-comm.png"),
                                QStringLiteral("config-8-run.png"),
                                QStringLiteral("config-noise-x2.png")};
  ASSERT_EQ(files.size(), expected.size());
  for (int i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(QString::fromStdString(fs::path(files.at(i).toStdString()).filename().string()),
              expected.at(i));
    const QImage image(files.at(i));
    ASSERT_FALSE(image.isNull()) << files.at(i).toStdString();
    EXPECT_EQ(image.size(), window_->size()) << files.at(i).toStdString();
    EXPECT_TRUE(has_content(image)) << files.at(i).toStdString();
  }

  // Left as found: no override from the x2 view, Profile checked, the same page selected.
  EXPECT_TRUE(window_->studio_config().overrides.empty());
  const auto* profile = window_->findChild<QAbstractButton*>(QStringLiteral("noiseProfile"));
  ASSERT_NE(profile, nullptr);
  EXPECT_TRUE(profile->isChecked());
  EXPECT_EQ(list->currentRow(), row_before);
}

TEST_F(StudioShots, UnwritableDirectoryGivesNoFiles) {
  // A regular file where the directory should be: nothing can be written under it.
  const fs::path blocker = kOutput / "blocker";
  {
    std::FILE* f = std::fopen(blocker.string().c_str(), "w");
    ASSERT_NE(f, nullptr);
    std::fclose(f);
  }
  const QStringList files = cobot::studio::save_screenshots(
      *window_, QString::fromStdString((blocker / "shots").string()));
  EXPECT_TRUE(files.isEmpty());
}

}  // namespace
