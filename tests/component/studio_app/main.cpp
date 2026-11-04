#include <QApplication>
#include <QtGlobal>

#include <gtest/gtest.h>

// The widgets need a QApplication for the whole test run. ctest sets QT_QPA_PLATFORM=offscreen;
// a direct run (tools/wsl/build.sh --target) without a display gets the same default here.
int main(int argc, char** argv) {
  if (qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM")) {
    qputenv("QT_QPA_PLATFORM", "offscreen");
  }
  ::testing::InitGoogleTest(&argc, argv);
  const QApplication app(argc, argv);
  QApplication::setQuitOnLastWindowClosed(false);
  return RUN_ALL_TESTS();
}
