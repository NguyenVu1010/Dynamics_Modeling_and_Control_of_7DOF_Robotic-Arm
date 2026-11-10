#pragma once

#include <QDoubleSpinBox>
#include <QString>
#include <QValidator>

class QFocusEvent;

// A spin box of one catalog value in its display unit (studio config UI design 3.2): three
// significant digits (format_display) while it has no focus, full precision while it has, so any
// value a user types round-trips. Infinite catalog limits map to +-1e300. GUI thread only.
namespace cobot::studio {

class ParamSpin : public QDoubleSpinBox {
  Q_OBJECT
 public:
  ParamSpin(double display_min,
            double display_max,
            const QString& display_unit,
            QWidget* parent = nullptr);

 protected:
  QString textFromValue(double v) const override;
  double valueFromText(const QString& text) const override;
  QValidator::State validate(QString& text, int& pos) const override;
  void focusInEvent(QFocusEvent* event) override;
  void focusOutEvent(QFocusEvent* event) override;

 private:
  QString number_text(const QString& text) const;  // text without the suffix, trimmed
  void rerender();                                 // the text again, for the current focus
};

}  // namespace cobot::studio
