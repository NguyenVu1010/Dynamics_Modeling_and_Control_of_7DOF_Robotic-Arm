#pragma once

#include <QFrame>
#include <string>
#include <vector>

#include "cobot/pipeline/overrides.hpp"
#include "cobot/studio/param_catalog.hpp"

class QComboBox;
class QLabel;
class QToolButton;

// One catalog key of the Configuration tab (studio config UI design 3.2, 3.3): title, hint and
// raw key, the editors in the display unit and an undo button while the key is overridden. The
// card keeps the value last shown exactly and emits only what the user edited (anti-goal: no
// rounding writes). GUI thread only.
namespace cobot::studio {

using Value = decltype(pipeline::KeyOverride::value);

class ParamSpin;

class ParamCard : public QFrame {
  Q_OBJECT
 public:
  // objectName "param:<key>", property card=true.
  explicit ParamCard(const ParamInfo& info, QWidget* parent = nullptr);

  const std::string& key() const { return info_.key; }
  // Every editor (validate() marks).
  const std::vector<QWidget*>& editors() const { return editors_; }
  // v in YAML units; changed: the key is overridden. Never emits edited.
  void show_value(const Value& v, bool changed);
  // setEnabled(!greyed) and the tooltip "not supported on Gazebo" (empty when not greyed).
  void set_greyed(bool greyed);

 signals:
  void edited(const Value& v);  // the whole value in YAML units
  void undo_requested();

 private:
  // A numeric editor: index -1 is the shared field (every element), otherwise one element.
  struct Field {
    ParamSpin* spin = nullptr;
    int index = -1;
    double shown = 0.0;  // the spin's value after the last show_value
    bool lazy = false;   // J1..J7: range and value set by configure_joints
  };

  ParamSpin* add_field(int index, const QString& name);
  void field_finished(std::size_t f);
  void set_field(Field& f, double yaml_value);
  double element(const Field& f) const;        // the kept value's element shown by f, YAML units
  void configure_joints();                     // J1..J7 range and value, once, when they first show
  QString full_text(double yaml_value) const;  // display value, every digit, and unit
  void update_joints();                        // per-joint view: shared field, J1..J7, range

  ParamInfo info_;
  Value kept_;
  bool changed_ = false;
  bool polished_ = false;      // the changed property was set once
  bool expanded_ = false;      // by the user
  bool joints_ready_ = false;  // configure_joints ran
  std::vector<Field> fields_;
  std::vector<QWidget*> editors_;
  QToolButton* switch_ = nullptr;
  QComboBox* combo_ = nullptr;
  QToolButton* toggle_ = nullptr;
  QToolButton* undo_ = nullptr;
  QLabel* all_joints_ = nullptr;
  QLabel* range_ = nullptr;
  QWidget* joints_ = nullptr;
  std::vector<QWidget*> joint_cells_;  // caption and spin of joint j, per joint
};

}  // namespace cobot::studio
