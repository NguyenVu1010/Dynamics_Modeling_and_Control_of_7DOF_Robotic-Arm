#pragma once

#include <QObject>

namespace cobot::studio {

// The time cursor of the charts (feature studio-app), shared with the 3D view and its playback
// (feature studio-3d). Run time in s from the first row.
class TimeCursor : public QObject {
  Q_OBJECT
 public:
  using QObject::QObject;
  double time_s() const { return time_s_; }
  void set_time_s(double t_s) {
    if (t_s != time_s_) {
      time_s_ = t_s;
      emit timeChanged(t_s);
    }
  }

 signals:
  void timeChanged(double t_s);

 private:
  double time_s_ = 0.0;
};

}  // namespace cobot::studio
