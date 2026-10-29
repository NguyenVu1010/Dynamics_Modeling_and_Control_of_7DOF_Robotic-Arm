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
#include "cobot/pipeline/spsc_ring.hpp"
#include "cobot/planning/cartesian_trajectory.hpp"
#include "cobot/planning/clik.hpp"
#include "cobot/planning/waypoint_trajectory.hpp"

// One control step (feature `control-loop`, design 5.4). Owns no thread; the host calls
// step() at control_rate_hz and drains log() from another thread.
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
namespace cobot::pipeline {

using common::Status;
using hal::JointCommand;
using hal::JointState;

inline constexpr std::size_t kLogCapacity = 8192;

template <int N>
class ControlLoop {
 public:
  ControlLoop(const model::Model<N>& model, const Config<N>& config);

  // Resets CLIK at initial.q. Config mode: plans the trajectory from FK(initial.q) to the
  // configured goal (kTracking). External mode: kStandby at FK(initial.q).
  Status start(const JointState<N>& initial);
  Status step(const JointState<N>& state, JointCommand<N>& out);

  LoopMode mode() const { return mode_; }
  SpscRing<LogRecord<N>, kLogCapacity>& log() { return log_; }

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

  model::Model<N> model_;
  Config<N> config_;
  planning::ClikReference<N> clik_;
  control::ComputedTorque<N> computed_torque_;
  control::FrictionCompensation<N> friction_;
  control::HoldController<N> hold_;
  std::optional<planning::CartesianTrajectory> trajectory_;
  LoopMode mode_ = LoopMode::kIdle;
  common::Timestamp t_start_;
  common::Timestamp t_last_;                                 // newest state stamp seen
  std::int64_t stale_steps_ = 0;                             // steps since t_last_ last advanced
  common::VecN<N> q_last_finite_ = common::VecN<N>::Zero();  // hold target for a non-finite q
  SpscRing<LogRecord<N>, kLogCapacity> log_;
  // External mode.
  std::optional<planning::WaypointTrajectory> waypoints_;
  planning::CartesianSample reference_;  // desired sample of the last step (or of standby)
  double tau_ = 0.0;                     // trajectory time, s
  bool stopping_ = false;
  std::int64_t stop_steps_ = 0;  // steps since request_stop()
};

}  // namespace cobot::pipeline
