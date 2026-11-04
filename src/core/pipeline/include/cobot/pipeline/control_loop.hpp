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
#include "cobot/pipeline/manual.hpp"
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
//
// Manual commands (feature manual-commands, ADR-022 "Lệnh tay"; external mode only, generators in
// manual.hpp). Each is called on the control thread, allocates nothing and rejects with literals.
// - Joint jog: target velocity direction * speed * qd_max_j, acceleration speed * a_max_j
//   (semi-implicit per step: v toward the target by at most a dt, q += v dt). The other joints
//   hold their q_ref exactly. The jog brakes so that q_ref comes to rest at the jog limit
//   q_max_j - (soft_margin + jog_margin) (or q_min_j + ...) and stops there with kSoftLimit
//   (never kHolding). Deadman: when deadman_s * control_rate_hz steps pass without a refresh
//   call, ramp to 0 with kDeadman. At rest: CLIK reset at q_ref, standby at FK(q_ref).
// - Joint move: q_ref = q_ref0 + s Δ, qd_ref = s' Δ, qdd_ref = s'' Δ from the plan, Δ re-based on
//   joint_reference(); at duration_s -> standby at FK(q_goal) with kFinished.
// - Joint-mode rows (jog_joint, move_joint): T_b_d = FK(q_ref) (tip), e1..e6 = pose_error(T_b_d,
//   T_b_e) (CLIK's sign convention), qddref = qdd_ref. CLIK is not stepped.
// - TCP jog: one axis unit vector e fixed in b at the start (base frame: the base axis; tool frame:
//   that axis of R_b_tcp at the start). Scalar speed v ramps to direction * speed * v_max at
//   speed * a_max (translation tcp_v/tcp_a, rotation tcp_w/tcp_alpha). Desired TCP pose
//   p += v dt e or R = so3_exp(v dt e) R; tip reference T_b_tcp * inverse(T_tip_tcp), so rotations
//   turn about the TCP point; then CLIK and computed torque as for waypoints. After each CLIK
//   step: condition_number() > clik.cond_threshold -> ramp to 0 with kSingularity; a joint whose
//   distance to its jog limit (in the direction qd_ref moves) minus qd_ref^2 / (2 speed a_max_j)
//   is <= 0 -> ramp to 0 with kSoftLimit. At rest -> standby at the reached reference pose.
// - TCP move: load(trajectory, kMoveTcp); end -> standby with kFinished; stop as ADR-018.
// - `cmd` column = command_mode(): the mode of the command running, or of the last accepted one
//   in standby/holding rows; kWaypoints in external mode before any command; kConfig in config
//   mode.
// - The safety checks (ADR-008) are unchanged and apply in every manual mode.
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
  // `cmd` is the command mode to log: kWaypoints (RViz/studio waypoints) or kMoveTcp; anything
  // else is an error.
  Status load(const planning::WaypointTrajectory& trajectory,
              CommandMode cmd = CommandMode::kWaypoints);
  // In external kTracking of a trajectory or a joint move: slow the trajectory time (or s(t))
  // down to rest over stop_time_s (rate r = 1 - smoothstep(t / stop_time_s)); of a jog: ramp the
  // velocity to 0 at the jog's deceleration ("release"). Ends in kStandby with manual_stop() ==
  // kStopped. Ignored otherwise or when stopping.
  void request_stop();
  // From kHolding (external mode), when `state` is finite, its stamp fresh and every joint
  // outside the soft margin and below qd_max: kStandby at FK(state.q) with CLIK reset there.
  Status reset(const JointState<N>& state);
  const common::Pose& reference_pose() const { return reference_.T_b_d; }  // T_b_d
  std::size_t segment() const;  // segment_at(trajectory_time_s()), 0 when nothing was loaded
  double trajectory_time_s() const { return tau_; }  // trajectory time of the next step

  // Manual commands (see above). direction -1 or +1; joint 0-based; speed in (0, 1].
  // Accepted in kStandby (starts a jog), or while the SAME jog runs (refresh: restarts the
  // deadman count). Rejected: not standby / another command running ("jog_joint: not in
  // standby"), bad joint, direction or speed, joint already at its jog limit in that direction
  // ("jog_joint: joint at its soft limit in that direction"), a different jog running ("jog:
  // release the running jog first"), a refresh of the same jog while it brakes (release, deadman,
  // soft limit or singularity: "jog_joint: jog is stopping"; jog again from kStandby).
  Status jog_joint(int joint, int direction, double speed);
  // Same rules. Rejected at start when the condition number of the Jacobian at the current
  // reference (as CLIK computes it) > clik.cond_threshold ("jog_tcp: near a singularity") or a
  // joint is inside its jog band ("jog_tcp: a joint is at its soft limit").
  Status jog_tcp(JogAxis axis, int direction, JogFrame frame, double speed);
  // kStandby only; move.q_start within 1e-3 rad (max norm) of joint_reference(), else
  // "move_joint: plan does not start at the reference". Runs from joint_reference() (re-based).
  Status move_joint(const JointMove<N>& move);
  // From kStandby or kTracking: the next step() enters kHolding at that step's measured q with
  // HoldReason::kEstop (unless another hold reason fires in the same step: existing order wins).
  // kHolding / kFault / kIdle: no-op, returns ok. Config mode: "estop: command_source is config".
  Status estop();
  ManualStop manual_stop() const { return manual_stop_; }
  CommandMode command_mode() const { return cmd_; }  // what `cmd` logs (plan ambiguity 12)
  // q_ref of the last step (CLIK's or the joint profile's).
  const VecN<N>& joint_reference() const;

 private:
  // External mode.
  void enter_standby(const common::Pose& pose);
  void advance_external(double dt);  // kTracking: sets reference_, advances tau_ or ends
  // Stop ramp rate r and dr/dt of this step; true when the ramp has reached rest.
  bool stop_rate(double dt, double& r, double& r_dot);
  void advance_joint(double dt);       // jog_joint / move_joint: joint_ref_, reference_ or ends
  bool advance_joint_move(double dt);  // true at the end of the move
  bool joint_mode() const;             // kTracking of jog_joint or move_joint
  // Rules shared by jog_joint and jog_tcp; `refreshed` when `jog` refreshed the running one.
  Status check_jog(const ManualJog& jog, bool& refreshed);
  void start_jog(const ManualJog& jog, CommandMode cmd);
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
  // Manual commands.
  CommandMode cmd_ = CommandMode::kConfig;
  ManualStop manual_stop_ = ManualStop::kNone;
  bool estop_requested_ = false;
  ManualJog jog_;
  JointMove<N> move_;
  VecN<N> move_q0_ = VecN<N>::Zero();      // joint_reference() when the move started
  planning::JointReference<N> joint_ref_;  // reference of the joint modes
  VecN<N> jog_lo_ = VecN<N>::Zero();       // jog limits: q_min + band, q_max - band
  VecN<N> jog_hi_ = VecN<N>::Zero();
};

}  // namespace cobot::pipeline
