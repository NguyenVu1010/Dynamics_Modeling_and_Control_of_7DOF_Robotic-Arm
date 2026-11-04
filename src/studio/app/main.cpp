#include <QApplication>
#include <QCommandLineParser>
#include <QStandardPaths>
#include <filesystem>

#include "main_window.hpp"

// cobot_studio [--config-dir <dir>] [--work-dir <dir>] (feature studio-app, ADR-021).
int main(int argc, char** argv) {
  const QApplication app(argc, argv);
  QApplication::setApplicationName(QStringLiteral("cobot_studio"));
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
  parser.addOption(config_dir);
  parser.addOption(work_dir);
  parser.process(app);
  cobot::studio::MainWindow window(parser.value(config_dir).toStdString(),
                                   parser.value(work_dir).toStdString());
  window.show();
  return QApplication::exec();
}
