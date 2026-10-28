#pragma once

#include "cobot/common/result.hpp"
#include "cobot/common/timestamp.hpp"
#include "cobot/common/types.hpp"

// Hardware abstraction: the only boundary between the controller and a robot
// (SimPlant, Gazebo through ros2_control, or EtherCAT drives on the Pi5 later).
namespace cobot::hal {

using common::Status;
using common::Timestamp;
using common::VecN;

template <int N>
struct JointState {
  Timestamp t;                                   // time of the measurement
  VecN<N> q = VecN<N>::Zero();                   // rad
  VecN<N> qd = VecN<N>::Zero();                  // rad/s
  VecN<N> tau_measured = VecN<N>::Zero();        // N m, valid only if tau_valid
  bool tau_valid = false;
};

template <int N>
struct JointCommand {
  VecN<N> tau = VecN<N>::Zero();  // N m
};

template <int N>
class JointInterface {
 public:
  virtual ~JointInterface() = default;
  // Must not allocate or throw. Error status when no fresh measurement is available.
  virtual Status read(JointState<N>& out) = 0;
  virtual Status write(const JointCommand<N>& cmd) = 0;
};

}  // namespace cobot::hal
