#pragma once

#include <QString>

class QApplication;

// The studio's look (studio config UI design 3.5): the vendored Breeze light stylesheet
// (third_party/breeze) followed by studio.qss, both from the resources of cobot_studio_ui.
// main.cpp applies it; MainWindow does not, so the tests run unthemed.
namespace cobot::studio {

// Breeze then studio.qss, so the project rules win; empty when a resource is missing.
QString theme_stylesheet();

// Sets theme_stylesheet() on `app`; false (style left alone) when a resource is missing.
bool apply_theme(QApplication& app);

}  // namespace cobot::studio
