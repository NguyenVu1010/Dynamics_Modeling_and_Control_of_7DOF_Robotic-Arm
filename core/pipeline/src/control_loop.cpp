#include "cobot/pipeline/control_loop.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>

#include <Eigen/Geometry>

#include "cobot/common/dof.hpp"
#include "cobot/common/lie.hpp"
#include "cobot/kinematics/kinematics.hpp"

namespace cobot::pipeline {
namespace {

using common::Mat3;
using common::Vec3;

// load(): a trajectory must start at the standby reference within 0.1 mm and 0.01 deg (design
// 6.3).
constexpr double kLoadPosTolM = 1e-4;
constexpr double kLoadRotTolRad = 1.745e-4;
constexpr double kStopEndTolerance = 1e-9;  // on the stop ramp's x = t / stop_time_s
constexpr std::size_t kMessageChars = 160;

const char* not_in_standby(LoopMode mode) {
  switch (mode) {
    case LoopMode::kIdle:
      return "load: not in standby (mode idle)";
    case LoopMode::kTracking:
      return "load: not in standby (mode tracking)";
    case LoopMode::kHolding:
      return "load: not in standby (mode holding)";
    case LoopMode::kFinished:
      return "load: not in standby (mode finished)";
    case LoopMode::kFault:
      return "load: not in standby (mode fault)";
    case LoopMode::kStandby:
      break;
  }
  return "load: not in standby";
}

// R = Rz(yaw) Ry(pitch) Rx(roll) of rpy = (roll, pitch, yaw).
Mat3 rpy_to_matrix(const Vec3& rpy) {
  const Mat3 rx = Eigen::AngleAxisd(rpy.x(), Vec3::UnitX()).toRotationMatrix();
  const Mat3 ry = Eigen::AngleAxisd(rpy.y(), Vec3::UnitY()).toRotationMatrix();
  const Mat3 rz = Eigen::AngleAxisd(rpy.z(), Vec3::UnitZ()).toRotationMatrix();
  const Mat3 r = rz * ry * rx;
  return r;
}

template <int N>
bool state_finite(const JointState<N>& s) {
  return s.q.allFinite() && s.qd.allFinite();
}

// q within soft_margin of a joint limit (or past it), or |qd| above qd_max.
template <int N>
bool outside_safe_set(const model::Model<N>& m, const JointState<N>& s, double soft_margin) {
  for (int i = 0; i < N; ++i) {
    const model::JointParams& j = m.joints[static_cast<std::size_t>(i)];
    if (s.q(i) - j.q_min < soft_margin || j.q_max - s.q(i) < soft_margin) {
      return true;
    }
    if (std::abs(s.qd(i)) > j.qd_max) {
      return true;
    }
  }
  return false;
}

// tau + tau_ff clamped element-wise to [-tau_max, tau_max]. A non-finite sum stays non-finite,
// so the loop still faults on it.
template <int N>
void add_clamped(const model::Model<N>& m, const common::VecN<N>& tau_ff, common::VecN<N>& tau) {
  for (int i = 0; i < N; ++i) {
    const double limit = m.joints[static_cast<std::size_t>(i)].tau_max;
    const double t = tau(i) + tau_ff(i);
    tau(i) = t > limit ? limit : (t < -limit ? -limit : t);
  }
}

}  // namespace

template <int N>
ControlLoop<N>::ControlLoop(const model::Model<N>& model, const Config<N>& config)
    : model_(model),
      config_(config),
      clik_(model, config.clik),
      computed_torque_(model, config.computed_torque),
      friction_(config.friction_compensation),
      hold_(model, config.computed_torque) {}

template <int N>
Status ControlLoop<N>::start(const JointState<N>& initial) {
  if (!state_finite(initial)) {
    return Status::error("control loop start: non-finite initial state");
  }
  kinematics::FkState<N> fk;
  kinematics::forward_kinematics(model_, initial.q, fk);
  clik_.reset(initial.q);
  t_start_ = initial.t;
  t_last_ = initial.t;
  stale_steps_ = 0;
  q_last_finite_ = initial.q;
  if (external()) {
    waypoints_.reset();
    tau_ = 0.0;
    enter_standby(fk.T_b_e);
    return Status::ok();
  }
  const common::Pose goal{rpy_to_matrix(config_.trajectory.goal_rpy_rad),
                          config_.trajectory.goal_position_m};
  trajectory_.emplace(fk.T_b_e, goal, config_.trajectory.duration_s);
  reference_ = trajectory_->sample(0.0);
  mode_ = LoopMode::kTracking;
  return Status::ok();
}

template <int N>
Status ControlLoop<N>::step(const JointState<N>& state, JointCommand<N>& out) {
  if (mode_ == LoopMode::kIdle) {
    out.tau.setZero();
    return Status::error("control loop: step before start");
  }
  const bool timed = !config_.deterministic;
  const std::chrono::steady_clock::time_point t0 =
      timed ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};

  const bool finite = state_finite(state);
  const double t = common::seconds_between(t_start_, state.t);
  // HAL freshness: the age of the newest stamp is the larger of the jump to a newer stamp and
  // the control periods spent on a repeated one, so a frozen HAL trips the hold while it is
  // still frozen. A stamp older than the newest one seen is a broken clock.
  double stamp_age = 0.0;
  const bool stamp_backwards = state.t < t_last_;
  if (state.t > t_last_) {
    stamp_age = common::seconds_between(t_last_, state.t);
    t_last_ = state.t;
    stale_steps_ = 0;
  } else if (state.t == t_last_) {
    ++stale_steps_;
    stamp_age = static_cast<double>(stale_steps_) / config_.control_rate_hz;
  }
  const bool hal_stale = stamp_backwards || stamp_age > config_.safety.hal_timeout_s;

  // 1. Safety: leave tracking (or standby) for the hold at the q of this moment.
  if (mode_ == LoopMode::kTracking || mode_ == LoopMode::kFinished || mode_ == LoopMode::kStandby) {
    if (!finite || hal_stale || outside_safe_set(model_, state, config_.safety.soft_margin_rad)) {
      hold_.engage(state.q.allFinite() ? state.q : q_last_finite_);
      mode_ = LoopMode::kHolding;
    }
  }
  if (state.q.allFinite()) {
    q_last_finite_ = state.q;
  }

  // 2.-3. Tracking reference and torque (standby: the same path on a fixed reference), or the
  // hold. External mode keeps the last reference while holding.
  const double dt = 1.0 / config_.control_rate_hz;
  if (!external()) {
    reference_ = trajectory_->sample(t);
  } else if (mode_ == LoopMode::kTracking) {
    advance_external(dt);
  }
  const planning::CartesianSample& x_d = reference_;
  LogRecord<N> record;
  if (mode_ == LoopMode::kTracking || mode_ == LoopMode::kFinished || mode_ == LoopMode::kStandby) {
    const planning::JointReference<N>& ref = clik_.step(x_d, dt);
    out = computed_torque_.compute(state, ref);
    add_clamped(model_, friction_.torque(ref.qd), out.tau);
    record.q_ref = ref.q;
    record.qd_ref = ref.qd;
    if (!external() && t >= config_.trajectory.duration_s + config_.trajectory.hold_s) {
      mode_ = LoopMode::kFinished;
    }
  } else if (mode_ == LoopMode::kHolding) {
    out = hold_.compute(state);
    record.q_ref = hold_.q_hold();
  }

  // 4. Never pass a non-finite torque on.
  if (mode_ == LoopMode::kFault || !out.tau.allFinite()) {
    mode_ = LoopMode::kFault;
    out.tau.setZero();
  }

  // 5. Log; a full ring drops and counts the record.
  kinematics::FkState<N> fk;
  kinematics::forward_kinematics(model_, state.q, fk);
  record.t = state.t;
  record.mode = mode_;
  record.q = state.q;
  record.qd = state.qd;
  record.tau = out.tau;
  record.T_b_e = fk.T_b_e;
  record.T_b_d = x_d.T_b_d;
  // CLIK is not stepped while holding, so its error says nothing about the hold target.
  if (mode_ != LoopMode::kHolding) {
    record.task_error = clik_.reference().task_error;
  }
  if (timed) {
    const std::chrono::duration<double, std::micro> elapsed = std::chrono::steady_clock::now() - t0;
    record.step_time_us = elapsed.count();
  }
  last_record_ = record;
  log_.push(record);

  if (mode_ == LoopMode::kFault) {
    return Status::error("control loop: fault, non-finite torque");
  }
  return Status::ok();
}

template <int N>
void ControlLoop<N>::enter_standby(const common::Pose& pose) {
  reference_ = planning::CartesianSample{pose, common::Vec6::Zero(), common::Vec6::Zero()};
  stopping_ = false;
  mode_ = LoopMode::kStandby;
}

// Time scaling (design 6.3): the trajectory is sampled at tau_, its twist scaled by the rate r
// and its twist derivative by r^2 plus the twist times dr/dt; tau_ then advances by r dt.
// Without a stop r = 1. The trajectory ends at tau_ >= T or when the stop ramp reaches r = 0.
template <int N>
void ControlLoop<N>::advance_external(double dt) {
  const planning::WaypointTrajectory& w = *waypoints_;
  double r = 1.0;
  double r_dot = 0.0;
  bool done = tau_ >= w.duration_s();
  if (stopping_) {
    const double x = static_cast<double>(stop_steps_) * dt / config_.stop_time_s;
    done = done || x >= 1.0 - kStopEndTolerance;
    r = 1.0 - x * x * (3.0 - 2.0 * x);
    r_dot = -6.0 * x * (1.0 - x) / config_.stop_time_s;
    ++stop_steps_;
  }
  const planning::CartesianSample s = w.sample(tau_);
  if (done) {
    enter_standby(s.T_b_d);
    return;
  }
  reference_.T_b_d = s.T_b_d;
  reference_.xd_d = r * s.xd_d;
  reference_.xdd_d = (r * r) * s.xdd_d + r_dot * s.xd_d;
  tau_ += r * dt;
}

template <int N>
Status ControlLoop<N>::load(const planning::WaypointTrajectory& trajectory) {
  if (!external()) {
    return Status::error("load: command_source is config");
  }
  if (mode_ != LoopMode::kStandby) {
    return Status::error(not_in_standby(mode_));
  }
  const common::Pose& start = trajectory.start_pose();
  const double dp = (start.p - reference_.T_b_d.p).norm();
  const common::Mat3 r_delta = start.R * reference_.T_b_d.R.transpose();
  const double dr = common::so3_log(r_delta).norm();
  if (dp > kLoadPosTolM || dr > kLoadRotTolRad) {
    std::array<char, kMessageChars> text{};
    std::snprintf(text.data(),
                  text.size(),
                  "load: trajectory start is %.3g m and %.3g rad from the reference pose "
                  "(limits %.3g m, %.4g rad)",
                  dp,
                  dr,
                  kLoadPosTolM,
                  kLoadRotTolRad);
    return Status::error(std::string(text.data()));
  }
  waypoints_.emplace(trajectory);
  tau_ = 0.0;
  stopping_ = false;
  mode_ = LoopMode::kTracking;
  return Status::ok();
}

template <int N>
void ControlLoop<N>::request_stop() {
  if (mode_ == LoopMode::kTracking && external() && !stopping_) {
    stopping_ = true;
    stop_steps_ = 0;
  }
}

template <int N>
Status ControlLoop<N>::reset(const JointState<N>& state) {
  if (!external()) {
    return Status::error("reset: command_source is config");
  }
  if (mode_ == LoopMode::kFault) {
    return Status::error("reset: a fault cannot be reset");
  }
  if (mode_ != LoopMode::kHolding) {
    return Status::error("reset: not holding");
  }
  if (!state_finite(state)) {
    return Status::error("reset: non-finite state");
  }
  // The stamp must be fresh by the same rule step() uses: not older than the newest one seen, and
  // a repeated newest stamp only while the periods spent on it stay within hal_timeout_s (a frozen
  // HAL is the cause of the hold, not a reason to leave it).
  if (state.t < t_last_) {
    return Status::error("reset: stale state (older than the newest stamp seen)");
  }
  if (state.t == t_last_ &&
      static_cast<double>(stale_steps_) / config_.control_rate_hz > config_.safety.hal_timeout_s) {
    return Status::error("reset: stale state (HAL stamp not advancing)");
  }
  for (int i = 0; i < N; ++i) {
    const model::JointParams& j = model_.joints[static_cast<std::size_t>(i)];
    const double margin = config_.safety.soft_margin_rad;
    if (state.q(i) - j.q_min < margin || j.q_max - state.q(i) < margin) {
      return Status::error("reset: joint " + std::to_string(i + 1) + " within soft margin");
    }
    if (std::abs(state.qd(i)) > j.qd_max) {
      return Status::error("reset: joint " + std::to_string(i + 1) + " above qd_max");
    }
  }
  kinematics::FkState<N> fk;
  kinematics::forward_kinematics(model_, state.q, fk);
  clik_.reset(state.q);
  q_last_finite_ = state.q;
  t_last_ = state.t;
  stale_steps_ = 0;
  enter_standby(fk.T_b_e);
  return Status::ok();
}

template <int N>
std::size_t ControlLoop<N>::segment() const {
  return waypoints_ ? waypoints_->segment_at(tau_) : 0U;
}

#define COBOT_PIPELINE_CONTROL_LOOP_INSTANTIATE(N) template class ControlLoop<(N)>;
COBOT_INSTANTIATE_DOF(COBOT_PIPELINE_CONTROL_LOOP_INSTANTIATE)
#undef COBOT_PIPELINE_CONTROL_LOOP_INSTANTIATE

}  // namespace cobot::pipeline
