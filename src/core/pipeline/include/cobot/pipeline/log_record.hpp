#pragma once

#include <limits>
#include <ostream>

#include "cobot/common/timestamp.hpp"
#include "cobot/common/types.hpp"

// One row of the run CSV (contract in docs/conventions.md, feature `pipeline-config`).
// Columns: t_ns,mode,q1..qN,qd1..qdN,qref1..qrefN,qdref1..qdrefN,tau1..tauN,
//          px,py,pz,rx,ry,rz,pdx,pdy,pdz,rdx,rdy,rdz,e1..e6,step_us,hold,
//          qddref1..N,qraw1..N,qt1..N,qdt1..N,qddt1..N,qmt1..N,taum1..N,tauf1..N,cmd
// p/r = position and so3_log rotation vector of T_b_e; pd/rd the same for T_b_d.
// In holding rows q_ref is the hold target, qd_ref and e1..e6 are 0 (CLIK is not stepped).
// Doubles printed with %.17g so that runs can be compared byte for byte (NaN as `nan`); step_us
// is written as 0 when config.deterministic is true (wall time is not reproducible).
// Log channels (feature log-channels, ADR-020): the plant truth columns qt..tauf are NaN when the
// host does not know them (real robot); taum, tauf and qddt of row k describe the plant interval
// (t_{k-1}, t_k], so taum of row k+1 is the plant's response to tau of row k.
namespace cobot::pipeline {

using common::Pose;
using common::Timestamp;
using common::Vec6;
using common::VecN;

enum class LoopMode { kIdle, kTracking, kHolding, kFinished, kFault, kStandby };

// "idle", "tracking", "holding", "finished", "fault", "standby"
const char* to_string(LoopMode mode);

// Why ControlLoop entered kHolding (the first matching one in this order), kept until reset().
enum class HoldReason {
  kNone,
  kNonFinite,
  kStampBackwards,
  kStateTimeout,
  kJointLimit,
  kOverspeed
};

// "none", "non_finite", "stamp_backwards", "state_timeout", "joint_limit", "overspeed"
const char* to_string(HoldReason reason);

// Where the loop's reference comes from: config (command_source config), waypoints (external);
// the jog and move modes are reserved for the studio (phase B).
enum class CommandMode { kConfig, kWaypoints, kJogJoint, kJogTcp, kMoveJoint, kMoveTcp };

// "config", "waypoints", "jog_joint", "jog_tcp", "move_joint", "move_tcp"
const char* to_string(CommandMode mode);

inline constexpr double kUnknown = std::numeric_limits<double>::quiet_NaN();

// Plant state the host knows at the instant of a step (bench: SimTruthProbe); NaN = unknown.
// q, qd, q_motor at that instant before the sensor decorator; qdd the mean link acceleration,
// tau_motor the applied torque and tau_friction the friction torque over the plant interval
// ending at that instant.
template <int N>
struct PlantTruth {
  VecN<N> q = VecN<N>::Constant(kUnknown);
  VecN<N> qd = VecN<N>::Constant(kUnknown);
  VecN<N> qdd = VecN<N>::Constant(kUnknown);
  VecN<N> q_motor = VecN<N>::Constant(kUnknown);
  VecN<N> tau_motor = VecN<N>::Constant(kUnknown);
  VecN<N> tau_friction = VecN<N>::Constant(kUnknown);
};

template <int N>
struct LogRecord {
  Timestamp t;
  LoopMode mode = LoopMode::kIdle;
  VecN<N> q = VecN<N>::Zero();
  VecN<N> qd = VecN<N>::Zero();
  VecN<N> q_ref = VecN<N>::Zero();
  VecN<N> qd_ref = VecN<N>::Zero();
  VecN<N> tau = VecN<N>::Zero();
  Pose T_b_e;
  Pose T_b_d;
  Vec6 task_error = Vec6::Zero();
  double step_time_us = 0.0;
  HoldReason hold_reason = HoldReason::kNone;  // CSV column `hold`
  VecN<N> qdd_ref = VecN<N>::Zero();           // `qddref`: CLIK ref.qdd, 0 where qd_ref is 0
  VecN<N> q_raw = VecN<N>::Zero();             // `qraw`: measured q before deflection compensation
  // Plant truth (PlantTruth, NaN = unknown): `qt`, `qdt`, `qddt`, `qmt`, `taum`, `tauf`.
  VecN<N> q_true = VecN<N>::Constant(kUnknown);
  VecN<N> qd_true = VecN<N>::Constant(kUnknown);
  VecN<N> qdd_true = VecN<N>::Constant(kUnknown);
  VecN<N> q_motor_true = VecN<N>::Constant(kUnknown);
  VecN<N> tau_motor = VecN<N>::Constant(kUnknown);
  VecN<N> tau_friction = VecN<N>::Constant(kUnknown);
  CommandMode cmd_mode = CommandMode::kConfig;  // `cmd`
};

template <int N>
void write_csv_header(std::ostream& os);
template <int N>
void write_csv_row(std::ostream& os, const LogRecord<N>& record);

}  // namespace cobot::pipeline
