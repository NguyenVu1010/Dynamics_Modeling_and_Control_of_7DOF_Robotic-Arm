#include <QApplication>
#include <QCommandLineParser>
#include <QStandardPaths>
#include <cstdio>
#include <filesystem>

#include "main_window.hpp"
#include "theme.hpp"

// cobot_studio [--config-dir <dir>] [--work-dir <dir>] [--bridge <name>] (feature studio-app,
// ADR-021; --bridge: feature studio-control, region /cobot_<name>).
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
  parser.process(app);
  cobot::studio::MainWindow window(parser.value(config_dir).toStdString(),
                                   parser.value(work_dir).toStdString(),
                                   parser.value(bridge).toStdString());
  window.show();
  return QApplication::exec();
}
