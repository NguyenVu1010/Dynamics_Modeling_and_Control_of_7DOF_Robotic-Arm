#include "screenshots.hpp"

#include <QAbstractButton>
#include <QCoreApplication>
#include <QDir>
#include <QListWidget>
#include <QPixmap>
#include <QRegularExpression>
#include <QTabWidget>
#include <QWidget>

namespace cobot::studio {

namespace {

// "Sensor / noise (2)" -> "sensor-noise": the count suffix dropped, other runs as '-'.
QString slug(QString title) {
  title.remove(QRegularExpression(QStringLiteral(R"(\s*\(\d+\)$)")));
  QString s = title.toLower();
  s.replace(QRegularExpression(QStringLiteral("[^a-z0-9]+")), QStringLiteral("-"));
  s.remove(QRegularExpression(QStringLiteral("^-+|-+$")));
  return s;
}

bool grab_to(QWidget& window, const QString& path) {
  QCoreApplication::processEvents();
  return window.grab().save(path, "PNG");
}

}  // namespace

QStringList save_screenshots(QWidget& window, const QString& dir) {
  auto* tabs = window.findChild<QTabWidget*>(QStringLiteral("tabs"));
  auto* list = window.findChild<QListWidget*>(QStringLiteral("groupList"));
  auto* profile = window.findChild<QAbstractButton*>(QStringLiteral("noiseProfile"));
  auto* x2 = window.findChild<QAbstractButton*>(QStringLiteral("noiseX2"));
  auto* config_tab = window.findChild<QWidget*>(QStringLiteral("configTab"));
  if (tabs == nullptr || list == nullptr || profile == nullptr || x2 == nullptr ||
      config_tab == nullptr || !QDir().mkpath(dir)) {
    return {};
  }
  const int tab_before = tabs->currentIndex();
  const int row_before = list->currentRow();
  tabs->setCurrentWidget(config_tab);

  QStringList written;
  bool ok = true;
  for (int row = 0; row < list->count() && ok; ++row) {
    list->setCurrentRow(row);
    const QString path = QDir(dir).filePath(
        QStringLiteral("config-%1-%2.png").arg(row + 1).arg(slug(list->item(row)->text())));
    ok = grab_to(window, path);
    written << path;
  }
  if (ok && profile->isChecked()) {
    list->setCurrentRow(1);  // Sensor / noise
    x2->click();
    const QString path = QDir(dir).filePath(QStringLiteral("config-noise-x2.png"));
    ok = grab_to(window, path);
    written << path;
    profile->click();
  }
  list->setCurrentRow(row_before);
  tabs->setCurrentIndex(tab_before);
  return ok ? written : QStringList();
}

}  // namespace cobot::studio
