#include "theme.hpp"

#include <QApplication>
#include <QFile>
#include <QIODevice>
#include <QRegularExpression>

#include <QLatin1String>

// Q_INIT_RESOURCE must be used outside any namespace; cobot_studio_ui is a static library, so its
// resources are registered on first use.
static void cobot_studio_init_theme_resources() {
  Q_INIT_RESOURCE(breeze);
  Q_INIT_RESOURCE(studio);
}

namespace cobot::studio {
namespace {

// The text of resource `path` without its /* */ comments; empty when it is missing. The Breeze
// sheet's comments name icons it does not ship (dialog_saveall.svg), so they are dropped rather
// than left as references that do not resolve.
QString read_resource(const QString& path) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return {};
  }
  static const QRegularExpression comment(QStringLiteral(R"(/\*.*?\*/)"),
                                          QRegularExpression::DotMatchesEverythingOption);
  return QString::fromUtf8(file.readAll()).remove(comment);
}

}  // namespace

QString theme_stylesheet() {
  cobot_studio_init_theme_resources();
  const QString breeze = read_resource(QStringLiteral(":/light/stylesheet.qss"));
  const QString studio = read_resource(QStringLiteral(":/studio/studio.qss"));
  if (breeze.isEmpty() || studio.isEmpty()) {
    return {};
  }
  return breeze + QLatin1String("\n") + studio;
}

bool apply_theme(QApplication& app) {
  const QString sheet = theme_stylesheet();
  if (sheet.isEmpty()) {
    return false;
  }
  app.setStyleSheet(sheet);
  return true;
}

}  // namespace cobot::studio
