#include "param_card.hpp"

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLocale>
#include <QSignalBlocker>
#include <QStyle>
#include <QToolButton>
#include <QVBoxLayout>
#include <algorithm>
#include <array>
#include <variant>

#include "param_spin.hpp"

namespace cobot::studio {

namespace {

constexpr int kJoints = 7;

QString qs(const std::string& s) {
  return QString::fromStdString(s);
}

}  // namespace

// Pinned signature: cards are built from catalog references.
// NOLINTNEXTLINE(modernize-pass-by-value)
ParamCard::ParamCard(const ParamInfo& info, QWidget* parent) : QFrame(parent), info_(info) {
  setObjectName(qs("param:" + info_.key));
  setProperty("card", true);
  auto* title = new QLabel(qs(info_.label));
  QFont bold = title->font();
  bold.setBold(true);
  title->setFont(bold);
  auto* hint = new QLabel(qs(info_.description));
  hint->setProperty("hint", true);
  hint->setWordWrap(true);
  auto* key_name = new QLabel(qs(info_.key));
  key_name->setProperty("keyName", true);
  key_name->setTextInteractionFlags(Qt::TextSelectableByMouse);
  undo_ = new QToolButton;
  undo_->setObjectName(QStringLiteral("undoButton"));
  undo_->setText(QStringLiteral("↺"));
  undo_->setAutoRaise(true);
  undo_->setToolTip(tr("Back to the base value"));
  undo_->setVisible(false);
  connect(undo_, &QToolButton::clicked, this, &ParamCard::undo_requested);

  auto* head = new QHBoxLayout;
  head->addWidget(title);
  head->addStretch(1);
  auto* editors = new QHBoxLayout;
  head->addLayout(editors);
  head->addWidget(undo_);
  auto* hint_row = new QHBoxLayout;
  hint_row->addWidget(hint, 1);
  hint_row->addWidget(key_name);
  auto* layout = new QVBoxLayout(this);
  layout->addLayout(head);
  layout->addLayout(hint_row);

  if (info_.kind == ParamKind::kBool) {
    switch_ = new QToolButton;
    switch_->setObjectName(QStringLiteral("value"));
    switch_->setCheckable(true);
    switch_->setProperty("switch", true);
    switch_->setText(tr("Off"));
    connect(switch_, &QToolButton::toggled, this, [this](bool on) {
      switch_->setText(on ? tr("On") : tr("Off"));
      kept_ = on;
      emit edited(kept_);
    });
    editors->addWidget(switch_);
    editors_.push_back(switch_);
  } else if (info_.kind == ParamKind::kChoice) {
    combo_ = new QComboBox;
    combo_->setObjectName(QStringLiteral("value"));
    for (const std::string& c : info_.choices) {
      combo_->addItem(qs(c));
    }
    connect(combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this] {
      kept_ = combo_->currentText().toStdString();
      emit edited(kept_);
    });
    editors->addWidget(combo_);
    editors_.push_back(combo_);
  } else if (info_.kind == ParamKind::kNumber) {
    editors->addWidget(add_field(-1, QStringLiteral("value")));
  } else if (info_.xyz) {
    const std::array<QString, 3> names = {
        QStringLiteral("x"), QStringLiteral("y"), QStringLiteral("z")};
    for (std::size_t i = 0; i < names.size(); ++i) {
      editors->addWidget(new QLabel(names.at(i)));
      editors->addWidget(add_field(static_cast<int>(i), names.at(i)));
    }
  } else {
    editors->addWidget(add_field(-1, QStringLiteral("value")));
    all_joints_ = new QLabel(tr("all joints"));
    editors->addWidget(all_joints_);
    range_ = new QLabel;
    range_->setObjectName(QStringLiteral("rangeLabel"));
    range_->setVisible(false);
    editors->addWidget(range_);
    toggle_ = new QToolButton;
    toggle_->setObjectName(qs("perJoint:" + info_.key));
    toggle_->setCheckable(true);
    toggle_->setAutoRaise(true);
    toggle_->setText(tr("Per joint ▸"));
    connect(toggle_, &QToolButton::clicked, this, [this](bool on) {
      expanded_ = on;
      update_joints();
    });
    editors->addWidget(toggle_);
    joints_ = new QWidget;
    auto* row = new QHBoxLayout(joints_);
    row->setContentsMargins(0, 0, 0, 0);
    for (int j = 0; j < kJoints; ++j) {
      auto* cell = new QWidget;
      auto* column = new QVBoxLayout(cell);
      column->setContentsMargins(0, 0, 0, 0);
      column->addWidget(new QLabel(tr("J%1").arg(j + 1)), 0, Qt::AlignHCenter);
      column->addWidget(add_field(j, QStringLiteral("j%1").arg(j + 1)));
      row->addWidget(cell);
      joint_cells_.push_back(cell);
    }
    joints_->setVisible(false);
    layout->addWidget(joints_);
  }
}

ParamSpin* ParamCard::add_field(int index, const QString& name) {
  // J1..J7 of a per-joint card get their range and value when they first show (configure_joints):
  // a catalog range of +-1e300 costs Qt a slow exact rounding per bound, and the window has about
  // two hundred of these hidden fields at start-up.
  const bool lazy = index >= 0 && !info_.xyz;
  auto* spin = lazy ? new ParamSpin(0.0, 0.0, qs(info_.display_unit))
                    : new ParamSpin(info_.min * info_.display_scale,
                                    info_.max * info_.display_scale,
                                    qs(info_.display_unit));
  spin->setObjectName(name);
  const std::size_t f = fields_.size();
  fields_.push_back(Field{spin, index, spin->value(), lazy});
  editors_.push_back(spin);
  connect(spin, &ParamSpin::editingFinished, this, [this, f] { field_finished(f); });
  return spin;
}

// Emits only when the user changed this editor's value, and only the edited element(s): the
// other elements are the kept ones, bit for bit.
void ParamCard::field_finished(std::size_t f) {
  Field& field = fields_.at(f);
  const double display = field.spin->value();
  if (display == field.shown) {
    return;
  }
  const double v = display / info_.display_scale;
  if (info_.kind == ParamKind::kNumber) {
    kept_ = v;
  } else {
    std::vector<double> list;
    if (const auto* held = std::get_if<std::vector<double>>(&kept_)) {
      list = *held;
    }
    if (field.index < 0) {
      list.assign(list.empty() ? kJoints : list.size(), v);
    } else if (static_cast<std::size_t>(field.index) < list.size()) {
      list.at(static_cast<std::size_t>(field.index)) = v;
    } else {
      return;
    }
    kept_ = list;
  }
  field.shown = display;
  emit edited(kept_);
}

void ParamCard::set_field(Field& f, double yaml_value) {
  const QSignalBlocker block(f.spin);
  f.spin->setValue(yaml_value * info_.display_scale);
  f.shown = f.spin->value();
  set_tooltip_part(f.spin, "valueTip", full_text(yaml_value));
}

// The value in the display unit with every digit (design 3.2: the full value is in the tooltip).
QString ParamCard::full_text(double yaml_value) const {
  const QString unit =
      info_.display_unit.empty() ? QString() : QStringLiteral(" ") + qs(info_.display_unit);
  return QString::number(yaml_value * info_.display_scale, 'g', QLocale::FloatingPointShortest) +
         unit;
}

// The kept value's element of a field: the shared field shows the first one.
double ParamCard::element(const Field& f) const {
  if (const auto* list = std::get_if<std::vector<double>>(&kept_)) {
    const std::size_t i = f.index < 0 ? 0 : static_cast<std::size_t>(f.index);
    return i < list->size() ? list->at(i) : 0.0;
  }
  return std::holds_alternative<double>(kept_) ? std::get<double>(kept_) : 0.0;
}

void ParamCard::configure_joints() {
  joints_ready_ = true;
  for (Field& f : fields_) {
    if (f.lazy) {
      f.spin->set_display_range(info_.min * info_.display_scale, info_.max * info_.display_scale);
      set_field(f, element(f));
    }
  }
}

void ParamCard::show_value(const Value& v, bool changed) {
  kept_ = v;
  if (switch_ != nullptr) {
    const QSignalBlocker block(switch_);
    const bool on = std::holds_alternative<bool>(v) && std::get<bool>(v);
    switch_->setChecked(on);
    switch_->setText(on ? tr("On") : tr("Off"));
  } else if (combo_ != nullptr) {
    const QSignalBlocker block(combo_);
    combo_->setCurrentText(std::holds_alternative<std::string>(v) ? qs(std::get<std::string>(v))
                                                                  : QString());
  } else {
    for (Field& f : fields_) {
      if (f.lazy && !joints_ready_) {
        continue;  // configure_joints sets them
      }
      set_field(f, element(f));
    }
    if (toggle_ != nullptr) {
      update_joints();
    }
  }
  undo_->setVisible(changed);
  if (changed != changed_ || !polished_) {
    changed_ = changed;
    polished_ = true;
    for (QWidget* w : editors_) {
      w->setProperty("changed", changed);
      // An editor not polished yet (a page never shown) takes the property when it first shows;
      // polishing hundreds of hidden editors here would stall window construction.
      if (w->testAttribute(Qt::WA_WState_Polished)) {
        w->style()->unpolish(w);
        w->style()->polish(w);
      }
    }
  }
}

// Equal elements: the shared field, J1..J7 only when expanded by the user. Different elements:
// the joints open, the toggle locked and the range instead of the shared field (design 3.3).
void ParamCard::update_joints() {
  const auto* list = std::get_if<std::vector<double>>(&kept_);
  const std::size_t n = list == nullptr ? 0 : list->size();
  bool differ = false;
  if (list != nullptr && n > 0) {
    const auto [lo, hi] = std::minmax_element(list->begin(), list->end());
    differ = *lo != *hi;
    const QString unit =
        info_.display_unit.empty() ? QString() : QStringLiteral(" ") + qs(info_.display_unit);
    range_->setText(tr("differs by joint: %1 to %2")
                        .arg(qs(format_display(*lo * info_.display_scale)),
                             qs(format_display(*hi * info_.display_scale))) +
                    unit);
    range_->setToolTip(full_text(*lo) + tr(" to ") + full_text(*hi));
  }
  for (std::size_t j = 0; j < joint_cells_.size(); ++j) {
    joint_cells_.at(j)->setVisible(j < n);
  }
  {
    const QSignalBlocker block(toggle_);
    toggle_->setChecked(differ || expanded_);
  }
  toggle_->setEnabled(!differ);
  fields_.front().spin->setVisible(!differ);
  all_joints_->setVisible(!differ);
  range_->setVisible(differ);
  if ((differ || expanded_) && !joints_ready_) {
    configure_joints();
  }
  joints_->setVisible(differ || expanded_);
}

void ParamCard::set_greyed(bool greyed) {
  setEnabled(!greyed);
  setToolTip(greyed ? tr("not supported on Gazebo") : QString());
}

}  // namespace cobot::studio
