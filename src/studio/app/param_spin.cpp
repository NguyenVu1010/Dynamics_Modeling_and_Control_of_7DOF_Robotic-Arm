#include "param_spin.hpp"

#include <QFocusEvent>
#include <QRegularExpression>
#include <algorithm>

#include "cobot/studio/param_catalog.hpp"

namespace cobot::studio {

namespace {

constexpr double kHuge = 1e300;  // bound for catalog limits that are infinite
constexpr int kDecimals = 12;    // setValue keeps the display value
constexpr int kFullDigits = 12;  // significant digits shown while editing

}  // namespace

ParamSpin::ParamSpin(double display_min,
                     double display_max,
                     const QString& display_unit,
                     QWidget* parent)
    : QDoubleSpinBox(parent) {
  setDecimals(kDecimals);
  setRange(std::max(display_min, -kHuge), std::min(display_max, kHuge));
  setKeyboardTracking(false);
  if (!display_unit.isEmpty()) {
    setSuffix(QStringLiteral(" ") + display_unit);
  }
}

QString ParamSpin::textFromValue(double v) const {
  return hasFocus() ? QString::number(v, 'g', kFullDigits)
                    : QString::fromStdString(format_display(v));
}

QString ParamSpin::number_text(const QString& text) const {
  QString t = text;
  if (!suffix().isEmpty() && t.endsWith(suffix())) {
    t.chop(suffix().size());
  }
  return t.trimmed();
}

double ParamSpin::valueFromText(const QString& text) const {
  return number_text(text).toDouble();
}

QValidator::State ParamSpin::validate(QString& text, int& /*pos*/) const {
  static const QRegularExpression partial(
      QStringLiteral("^[+-]?[0-9]*[.]?[0-9]*([eE][+-]?[0-9]*)?$"));
  const QString t = number_text(text);
  if (!partial.match(t).hasMatch()) {
    return QValidator::Invalid;
  }
  bool ok = false;
  const double v = t.toDouble(&ok);
  return ok && v >= minimum() && v <= maximum() ? QValidator::Acceptable : QValidator::Intermediate;
}

void ParamSpin::rerender() {
  setSuffix(suffix());  // QAbstractSpinBox re-renders the text, signals blocked
}

void ParamSpin::focusInEvent(QFocusEvent* event) {
  rerender();
  QDoubleSpinBox::focusInEvent(event);
}

void ParamSpin::focusOutEvent(QFocusEvent* event) {
  QDoubleSpinBox::focusOutEvent(event);
  rerender();
}

}  // namespace cobot::studio
