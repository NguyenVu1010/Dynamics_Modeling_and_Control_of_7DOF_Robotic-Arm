#include <QApplication>
#include <QCommandLineParser>
#include <QStandardPaths>
#include <cstdio>
#include <filesystem>

#include "main_window.hpp"
#include "screenshots.hpp"
#include "theme.hpp"

// cobot_studio [--config-dir <dir>] [--work-dir <dir>] [--bridge <name>] [--screenshot-dir <dir>]
// (feature studio-app, ADR-021; --bridge: feature studio-control, region /cobot_<name>;
// --screenshot-dir: feature studio-shots, saves the Configuration pages as PNG and exits, works
// with QT_QPA_PLATFORM=offscreen).
int main(int argc, char** argv) {
  QApplication app(argc, argv);
  QApplication::setApplicationName(QStringLiteral("cobot_studio"));
  if (!cobot::studio::apply_theme(app)) {
    std::fputs("cobot_studio: theme resources missing, using the default Qt style\n", stderr);
  }
  QCommandLineParser parser;
  parser.setApplicationDescription(QApplication::tr("Configuration and tracking studio"));
  parser.addHelpOption();
  const QCommandLineOption config_dir(
      QStringLiteral("config-dir"),
      QApplication::tr("Directory holding default.yaml and imperfections/."),
      QStringLiteral("dir"),
      QStringLiteral(COBOT_REPO_DIR "/config"));
  const QCommandLineOption work_dir(
      QStringLiteral("work-dir"),
      QApplication::tr("Directory of the materialised runs."),
      QStringLiteral("dir"),
      QStandardPaths::writableLocation(QStandardPaths::CacheLocation) + QStringLiteral("/runs"));
  const QCommandLineOption bridge(
      QStringLiteral("bridge"),
      QApplication::tr("Studio bridge of the controller (region /cobot_<name>)."),
      QStringLiteral("name"),
      QStringLiteral("cobot_controller"));
  parser.addOption(config_dir);
  parser.addOption(work_dir);
  parser.addOption(bridge);
  const QCommandLineOption screenshot_dir(
      QStringLiteral("screenshot-dir"),
      QApplication::tr("Save the Configuration pages as PNG files into <dir> and exit."),
      QStringLiteral("dir"));
  parser.addOption(screenshot_dir);
  parser.process(app);
  cobot::studio::MainWindow window(parser.value(config_dir).toStdString(),
                                   parser.value(work_dir).toStdString(),
                                   parser.value(bridge).toStdString());
  if (parser.isSet(screenshot_dir)) {
    constexpr int kShotWidth = 1400;
    constexpr int kShotHeight = 900;
    window.resize(kShotWidth, kShotHeight);
    window.show();
    const QStringList files = cobot::studio::save_screenshots(window, parser.value(screenshot_dir));
    for (const QString& f : files) {
      std::fprintf(stdout, "%s\n", qPrintable(f));
    }
    if (files.isEmpty()) {
      std::fputs("cobot_studio: could not write the screenshots\n", stderr);
      return 1;
    }
    return 0;
  }
  window.show();
  return QApplication::exec();
}
