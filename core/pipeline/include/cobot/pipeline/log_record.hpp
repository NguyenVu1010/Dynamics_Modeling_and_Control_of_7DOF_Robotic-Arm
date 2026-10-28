#pragma once

#include <ostream>

#include "cobot/common/timestamp.hpp"
#include "cobot/common/types.hpp"

// One row of the run CSV (contract in docs/conventions.md, feature `pipeline-config`).
// Columns: t_ns,mode,q1..qN,qd1..qdN,qref1..qrefN,qdref1..qdrefN,tau1..tauN,
//          px,py,pz,rx,ry,rz,pdx,pdy,pdz,rdx,rdy,rdz,e1..e6,step_us
// p/r = position and so3_log rotation vector of T_b_e; pd/rd the same for T_b_d.
// Doubles printed with %.17g so that runs can be compared byte for byte; step_us is written
// as 0 when config.deterministic is true (wall time is not reproducible).
namespace cobot::pipeline {

using common::Pose;
using common::Timestamp;
using common::Vec6;
using common::VecN;

enum class LoopMode { kIdle, kTracking, kHolding, kFinished, kFault };

// "idle", "tracking", "holding", "finished", "fault"
const char* to_string(LoopMode mode);

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
};

template <int N>
void write_csv_header(std::ostream& os);
template <int N>
void write_csv_row(std::ostream& os, const LogRecord<N>& record);

}  // namespace cobot::pipeline
