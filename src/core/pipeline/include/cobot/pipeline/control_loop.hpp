#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include "cobot/control/computed_torque.hpp"
#include "cobot/control/friction_compensation.hpp"
#include "cobot/control/hold.hpp"
#include "cobot/hal/joint_interface.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/log_record.hpp"
#include "cobot/pipeline/recorder.hpp"
#include "cobot/planning/cartesian_trajectory.hpp"
#include "cobot/planning/clik.hpp"
#include "cobot/planning/waypoint_trajectory.hpp"

// One control step (feature `control-loop`, design 5.4). Owns no thread; the host calls
// step() at control_rate_hz. Each step's record goes to last_record() and, when the host passed
// one, into its Recorder (feature pipeline-recorder, ADR-020): the host owns the recorder and
// exports it off the control thread; a ControlLoop rebuilt on the same Recorder continues the
// same log.
//
// Modes: kIdle --start()--> kTracking --(t >= duration + hold)--> kFinished (keeps tracking the
// goal reference). Any of: q within soft_margin of a limit, |qd| > qd_max, a non-finite value
// in the state, a state stamp older than hal_timeout (a jump to a newer stamp, or the same
// stamp repeated for longer than hal_timeout at control_rate_hz), or a stamp going backwards
// -> kHolding (HoldController at the q of that moment; never leaves on its own). A non-finite
// computed torque -> kFault with zero torque. Output torque is always finite and within tau_max.
//
// Tracking torque: clamp(computed_torque(state, ref) + friction feed-forward of ref.qd, tau_max);
// the hold is not compensated.
//
// External command mode (command_source: external, feature `loop-external`, design
// docs/design/2026-10-29-waypoint-gui-design.md 6.3): start() -> kStandby, which keeps the
// tracking torque path on a fixed Cartesian reference (zero twist). load() runs a
// WaypointTrajectory (kTracking, trajectory time advancing by dt per step); at its end, or when
// a request_stop() ramp reaches rest, the loop returns to kStandby at the pose reached. There is
// no kFinished and no hold_s. The safety checks are the same in every mode, standby included;
// reset() is the only way out of kHolding. load(), request_stop() and reset() allocate nothing
// when they succeed (a rejection builds its message).
//
// Deflection compensation (feature deflection-comp, DeflectionCompConfig): when
// config.deflection_comp.enabled and the state has q_from_motor, start(), step() and reset() work
// on the state with q replaced by q_hat = q - harmonic_spring_deflection(g(q)) (one gravity
// evaluation of the controller's model per call, qd unchanged) before anything else: the safety
// checks, CLIK, the computed torque, the hold and the log (LogRecord::q, T_b_e) all see q_hat.
// Otherwise the state is used as given, bit for bit.
namespace cobot::pipeline {

using common::Status;
using hal::JointCommand;
using hal::JointState;

template <int N>
class ControlLoop {
 public:
  // `recorder` (may be null: no log) must outlive the loop.
  ControlLoop(const model::Model<N>& model,
              const Config<N>& config,
              Recorder<N>* recorder = nullptr);

  // Resets CLIK at initial.q. Config mode: plans the trajectory from FK(initial.q) to the
  // configured goal (kTracking). External mode: kStandby at FK(initial.q).
  Status start(const JointState<N>& initial);
  // Truth unknown: the record's plant truth columns are NaN.
  Status step(const JointState<N>& state, JointCommand<N>& out);
  // `truth` (feature log-channels) is copied into the record verbatim; nothing else uses it.
  Status step(const JointState<N>& state, JointCommand<N>& out, const PlantTruth<N>& truth);

  LoopMode mode() const { return mode_; }
  // Why the loop entered kHolding; kNone before that and after start() or reset().
  HoldReason hold_reason() const { return hold_reason_; }
  // The record of the last step (the row given to the recorder, also when it dropped the row or
  // there is none), for a host that needs per-step values on the control thread.
  const LogRecord<N>& last_record() const { return last_record_; }

  // External mode only. Accepted in kStandby when the trajectory starts at reference_pose()
  // within 1e-4 m and 1.745e-4 rad; the trajectory is copied (fixed storage) and runs from the
  // next step.
  Status load(const planning::WaypointTrajectory& trajectory);
  // In external kTracking: slow the trajectory time down to rest along the path over
  // stop_time_s (rate r = 1 - smoothstep(t / stop_time_s)). Ignored otherwise or when stopping.
  void request_stop();
  // From kHolding (external mode), when `state` is finite, its stamp fresh and every joint
  // outside the soft margin and below qd_max: kStandby at FK(state.q) with CLIK reset there.
  Status reset(const JointState<N>& state);
  const common::Pose& reference_pose() const { return reference_.T_b_d; }  // T_b_d
  std::size_t segment() const;  // segment_at(trajectory_time_s()), 0 when nothing was loaded
  double trajectory_time_s() const { return tau_; }  // trajectory time of the next step

 private:
  // External mode.
  void enter_standby(const common::Pose& pose);
  void advance_external(double dt);  // kTracking: sets reference_, advances tau_ or ends
  bool external() const { return config_.command_source == CommandSource::kExternal; }
  // `measured`, or state_hat_ holding it with q_hat when the compensation applies.
  const JointState<N>& compensated(const JointState<N>& measured);

  model::Model<N> model_;
  Config<N> config_;
  planning::ClikReference<N> clik_;
  control::ComputedTorque<N> computed_torque_;
  control::FrictionCompensation<N> friction_;
  control::HoldController<N> hold_;
  std::optional<planning::CartesianTrajectory> trajectory_;
  LoopMode mode_ = LoopMode::kIdle;
  HoldReason hold_reason_ = HoldReason::kNone;
  common::Timestamp t_start_;
  common::Timestamp t_last_;                                 // newest state stamp seen
  std::int64_t stale_steps_ = 0;                             // steps since t_last_ last advanced
  common::VecN<N> q_last_finite_ = common::VecN<N>::Zero();  // hold target for a non-finite q
  Recorder<N>* recorder_ = nullptr;
  LogRecord<N> last_record_;
  const PlantTruth<N> unknown_truth_;  // all NaN: what step() without a truth logs
  // External mode.
  std::optional<planning::WaypointTrajectory> waypoints_;
  planning::CartesianSample reference_;  // desired sample of the last step (or of standby)
  double tau_ = 0.0;                     // trajectory time, s
  bool stopping_ = false;
  std::int64_t stop_steps_ = 0;  // steps since request_stop()
  JointState<N> state_hat_;      // last compensated state (deflection-comp)
};

}  // namespace cobot::pipeline
