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
namespace cobot::pipeline {

using common::Status;
using hal::JointCommand;
using hal::JointState;

inline constexpr std::size_t kLogCapacity = 8192;

template <int N>
class ControlLoop {
 public:
  ControlLoop(const model::Model<N>& model, const Config<N>& config);

  // Plans the trajectory from FK(initial.q) to the configured goal and resets CLIK.
  Status start(const JointState<N>& initial);
  Status step(const JointState<N>& state, JointCommand<N>& out);

  LoopMode mode() const { return mode_; }
  SpscRing<LogRecord<N>, kLogCapacity>& log() { return log_; }

 private:
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
};

}  // namespace cobot::pipeline
