// The studio theme (feature studio-theme-catalog, studio config UI design 3.5): theme_stylesheet()
// is the vendored Breeze light sheet followed by studio.qss, both read from the resources of
// cobot_studio_ui; apply_theme() sets it on the QApplication; a MainWindow built under it shows
// without a "Could not parse" warning from Qt's stylesheet parser. MainWindow itself never applies
// the theme, so the other studio tests run unthemed.
#include <QApplication>
#include <QFile>
#include <QMessageLogContext>
#include <QRegularExpression>
#include <QRegularExpressionMatch>
#include <QRegularExpressionMatchIterator>
#include <QString>
#include <QStringList>
#include <QTabWidget>
#include <QtGlobal>
#include <filesystem>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "main_window.hpp"
#include "studio_app_test_util.hpp"
#include "theme.hpp"

namespace cobot::studio {
namespace {

namespace fs = std::filesystem;

const fs::path kConfigDir = COBOT_REPO_DIR "/config";
const fs::path kOutput = COBOT_TEST_OUTPUT_DIR "/studio_theme";

// The rules studio.qss holds for the later PRs (brief, theme section).
const QStringList& studio_rules() {
  static const QStringList rules = {
      QStringLiteral("QFrame[card=\"true\"]"),
      QStringLiteral("[changed=\"true\"]"),
      QStringLiteral("QPushButton[primary=\"true\"]"),
      QStringLiteral("QLabel[hint=\"true\"]"),
      QStringLiteral("QLabel[keyName=\"true\"]"),
      QStringLiteral("QListWidget#groupList"),
  };
  return rules;
}

TEST(StudioTheme, StylesheetHoldsBreezeThenTheStudioRules) {
  const QString sheet = theme_stylesheet();
  ASSERT_FALSE(sheet.isEmpty());
  for (const QString& rule : studio_rules()) {
    EXPECT_TRUE(sheet.contains(rule)) << rule.toStdString();
  }
  // A rule of the Breeze sheet.
  const int breeze = sheet.indexOf(QStringLiteral("QToolTip"));
  ASSERT_GE(breeze, 0) << "no Breeze rule (QToolTip) in the stylesheet";
  // Breeze first, then studio.qss, so the project rules win.
  EXPECT_LT(breeze, sheet.indexOf(QStringLiteral("QFrame[card=\"true\"]")));
  // The colours of the brief's studio rules.
  for (const char* colour :
       {"#e3e6eb", "#fff4d6", "#f0c24b", "#2563eb", "#6b7385", "#a0a6b2", "#eaf1ff", "#1d4ed8"}) {
    EXPECT_TRUE(sheet.contains(QLatin1String(colour), Qt::CaseInsensitive)) << colour;
  }
}

TEST(StudioTheme, ResourcesAreRegisteredAndEveryUrlResolves) {
  ASSERT_FALSE(theme_stylesheet().isEmpty());  // registers the resources (Q_INIT_RESOURCE)
  EXPECT_TRUE(QFile::exists(QStringLiteral(":/light/stylesheet.qss")));
  EXPECT_TRUE(QFile::exists(QStringLiteral(":/studio/studio.qss")));
  // Every resource the sheet names (Breeze's icons) is in breeze.qrc.
  const QRegularExpression url(QStringLiteral(R"re(url\(\s*["']?(:/[^)"'\s]+)["']?\s*\))re"));
  QRegularExpressionMatchIterator it = url.globalMatch(theme_stylesheet());
  int urls = 0;
  while (it.hasNext()) {
    const QRegularExpressionMatch m = it.next();
    EXPECT_TRUE(QFile::exists(m.captured(1))) << m.captured(1).toStdString();
    ++urls;
  }
  EXPECT_GT(urls, 0) << "the Breeze sheet references its icons";
}

TEST(StudioTheme, ApplyThemeSetsTheStylesheetOnTheApplication) {
  ASSERT_NE(qApp, nullptr);
  EXPECT_TRUE(apply_theme(*qApp));
  EXPECT_FALSE(qApp->styleSheet().isEmpty());
  EXPECT_EQ(qApp->styleSheet(), theme_stylesheet());
}

// Stylesheet parse warnings seen while the handler below is installed.
QStringList& parse_warnings() {
  static QStringList warnings;
  return warnings;
}

void record_parse_warnings(QtMsgType /*type*/,
                           const QMessageLogContext& /*context*/,
                           const QString& message) {
  if (message.contains(QStringLiteral("Could not parse"))) {
    parse_warnings().append(message);
  }
}

TEST(StudioTheme, MainWindowShowsUnderTheThemeWithoutParseWarnings) {
  parse_warnings().clear();
  const QtMessageHandler previous = qInstallMessageHandler(record_parse_warnings);
  const fs::path work_dir = kOutput / "work";
  fs::remove_all(work_dir);
  fs::create_directories(work_dir);
  {
    ASSERT_TRUE(apply_theme(*qApp));
    auto window = std::make_unique<MainWindow>(kConfigDir, work_dir);
    window->show();
    test::pump(100);
    // Polish every tab, not just the first one shown.
    if (auto* tabs = window->findChild<QTabWidget*>(QStringLiteral("tabs"))) {
      for (int i = 0; i < tabs->count(); ++i) {
        tabs->setCurrentIndex(i);
        test::pump(30);
      }
    }
    window.reset();
  }
  qInstallMessageHandler(previous);
  EXPECT_TRUE(parse_warnings().isEmpty())
      << parse_warnings().join(QStringLiteral("\n")).toStdString();
}

}  // namespace
}  // namespace cobot::studio
