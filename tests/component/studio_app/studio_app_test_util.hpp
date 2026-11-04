#pragma once

#include <QAbstractButton>
#include <QAbstractSpinBox>
#include <QApplication>
#include <QBrush>
#include <QColor>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSpinBox>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QVariant>
#include <QWidget>
#include <functional>
#include <string>

// Helpers of the cobot_studio smoke test: waiting without blocking the event loop, answering the
// modal dialogs the window opens (SweepDialog, confirmation boxes), and spin boxes of either
// type (QSpinBox or QDoubleSpinBox; the object names are the contract, the class is not).
namespace cobot::studio::test {

// Runs the event loop in 10 ms slices until `done` is true or `timeout_ms` passed.
inline bool wait_until(const std::function<bool()>& done, int timeout_ms) {
  QElapsedTimer clock;
  clock.start();
  while (!done()) {
    if (clock.elapsed() > timeout_ms) {
      return done();
    }
    QEventLoop loop;
    QTimer::singleShot(10, &loop, &QEventLoop::quit);
    loop.exec();
  }
  return true;
}

// Lets the event loop run for `ms` (queued signals, late results).
inline void pump(int ms) {
  QEventLoop loop;
  QTimer::singleShot(ms, &loop, &QEventLoop::quit);
  loop.exec();
}

inline double spin_value(const QWidget* w) {
  if (const auto* d = qobject_cast<const QDoubleSpinBox*>(w)) {
    return d->value();
  }
  if (const auto* i = qobject_cast<const QSpinBox*>(w)) {
    return i->value();
  }
  return -1e300;
}

inline void set_spin_value(QWidget* w, double v) {
  if (auto* d = qobject_cast<QDoubleSpinBox*>(w)) {
    d->setValue(v);
  } else if (auto* i = qobject_cast<QSpinBox*>(w)) {
    i->setValue(static_cast<int>(v));
  }
}

inline double spin_min(const QWidget* w) {
  if (const auto* d = qobject_cast<const QDoubleSpinBox*>(w)) {
    return d->minimum();
  }
  if (const auto* i = qobject_cast<const QSpinBox*>(w)) {
    return i->minimum();
  }
  return -1e300;
}

inline double spin_max(const QWidget* w) {
  if (const auto* d = qobject_cast<const QDoubleSpinBox*>(w)) {
    return d->maximum();
  }
  if (const auto* i = qobject_cast<const QSpinBox*>(w)) {
    return i->maximum();
  }
  return 1e300;
}

// A background brush (QBrush or QColor) whose hue is red: hue within 15 degrees of 0 and a
// clear saturation, so both a strong and a light red qualify.
inline bool is_red(const QVariant& v) {
  if (!v.isValid()) {
    return false;
  }
  QColor c;
  if (v.userType() == QMetaType::QColor) {
    c = v.value<QColor>();
  } else if (v.userType() == QMetaType::QBrush) {
    const QBrush brush = v.value<QBrush>();
    if (brush.style() == Qt::NoBrush) {
      return false;
    }
    c = brush.color();
  } else {
    return false;
  }
  if (!c.isValid() || c.alpha() == 0) {
    return false;
  }
  const int hue = c.hslHue();  // -1 for grey
  return hue >= 0 && (hue <= 15 || hue >= 345) && c.hslSaturation() > 60;
}

// Answers every modal dialog every 20 ms:
// - QMessageBox: records its text and presses Yes, else a destructive (Discard) button, else an
//   accept (Ok) button: confirmations are accepted, errors are logged for the failure message.
// - The sweep dialog (a QDialog holding `levelsEdit`): selects `param` in `paramCombo` (by text,
//   then by item data, then by an item text containing it), types the levels and seeds and
//   presses Ok.
class ModalHandler {
 public:
  ModalHandler() {
    timer_.setInterval(20);
    QObject::connect(&timer_, &QTimer::timeout, &timer_, [this] { poll(); });
    timer_.start();
  }

  void set_sweep(const QString& param, const QString& levels, const QString& seeds) {
    param_ = param;
    levels_ = levels;
    seeds_ = seeds;
  }
  int messages() const { return messages_; }
  int sweep_dialogs() const { return sweep_dialogs_; }
  std::string log() const { return log_.join(" | ").toStdString(); }

 private:
  static QWidget* sweep_dialog() {
    for (QWidget* top : QApplication::topLevelWidgets()) {
      if (top->isVisible() && qobject_cast<QDialog*>(top) != nullptr &&
          top->findChild<QLineEdit*>(QStringLiteral("levelsEdit")) != nullptr) {
        return top;
      }
    }
    return nullptr;
  }

  void poll() {
    QWidget* w = QApplication::activeModalWidget();
    if (w == nullptr) {
      w = sweep_dialog();
    }
    if (w == nullptr) {
      return;
    }
    if (auto* box = qobject_cast<QMessageBox*>(w)) {
      answer(box);
      return;
    }
    auto* dialog = qobject_cast<QDialog*>(w);
    if (dialog != nullptr &&
        dialog->findChild<QLineEdit*>(QStringLiteral("levelsEdit")) != nullptr) {
      fill(dialog);
    }
  }

  void answer(QMessageBox* box) {
    ++messages_;
    log_ << QStringLiteral("message box: ") + box->text();
    for (const QMessageBox::ButtonRole role :
         {QMessageBox::YesRole, QMessageBox::DestructiveRole, QMessageBox::AcceptRole}) {
      for (QAbstractButton* b : box->buttons()) {
        if (box->buttonRole(b) == role) {
          b->click();
          return;
        }
      }
    }
    box->accept();
  }

  void fill(QDialog* dialog) {
    ++sweep_dialogs_;
    auto* combo = dialog->findChild<QComboBox*>(QStringLiteral("paramCombo"));
    auto* levels = dialog->findChild<QLineEdit*>(QStringLiteral("levelsEdit"));
    auto* seeds = dialog->findChild<QLineEdit*>(QStringLiteral("seedsEdit"));
    if (combo == nullptr || levels == nullptr || seeds == nullptr) {
      log_ << QStringLiteral("sweep dialog without paramCombo, levelsEdit or seedsEdit");
      dialog->reject();
      return;
    }
    int index = combo->findText(param_);
    if (index < 0) {
      index = combo->findData(param_);
    }
    for (int k = 0; index < 0 && k < combo->count(); ++k) {
      if (combo->itemText(k).contains(param_)) {
        index = k;
      }
    }
    if (index < 0) {
      log_ << QStringLiteral("paramCombo has no ") + param_;
      dialog->reject();
      return;
    }
    combo->setCurrentIndex(index);
    levels->setText(levels_);
    seeds->setText(seeds_);
    if (auto* buttons = dialog->findChild<QDialogButtonBox*>()) {
      if (QAbstractButton* ok = buttons->button(QDialogButtonBox::Ok)) {
        ok->click();
        return;
      }
    }
    dialog->accept();
  }

  QTimer timer_;
  QString param_, levels_, seeds_;
  int messages_ = 0;
  int sweep_dialogs_ = 0;
  QStringList log_;
};

}  // namespace cobot::studio::test
