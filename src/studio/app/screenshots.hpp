#pragma once

#include <QString>
#include <QStringList>

class QWidget;

// Screenshots of the Configuration tab for the docs (feature studio-shots, design
// docs/design/2026-11-10-studio-config-ui-design.md 5 item 4). Works on the offscreen platform.
namespace cobot::studio {

// Saves a grab of `window` (a shown MainWindow) into `dir`, created when missing:
// config-<n>-<page>.png for every groupList row in order (n from 1, the page title in lower case
// with runs of other characters as '-'), then config-noise-x2.png, the Sensor / noise page at
// noise level x2, taken only while the level is Profile and set back to Profile after. The
// selected page and tab are restored. Returns the written paths in that order; empty when the
// directory cannot be created, a file cannot be written or a contract widget is missing.
QStringList save_screenshots(QWidget& window, const QString& dir);

}  // namespace cobot::studio
