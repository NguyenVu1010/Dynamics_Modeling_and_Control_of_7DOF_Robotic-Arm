#include "cobot/pipeline/control_loop.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>

#include <Eigen/Geometry>

#include "cobot/common/dof.hpp"
#include "cobot/kinematics/kinematics.hpp"

namespace cobot::pipeline {
namespace {

using common::Mat3;
using common::Vec3;

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
  const common::Pose goal{rpy_to_matrix(config_.trajectory.goal_rpy_rad),
                          config_.trajectory.goal_position_m};
  trajectory_.emplace(fk.T_b_e, goal, config_.trajectory.duration_s);
  clik_.reset(initial.q);
  t_start_ = initial.t;
  t_last_ = initial.t;
  stale_steps_ = 0;
  q_last_finite_ = initial.q;
  mode_ = LoopMode::kTracking;
  return Status::ok();
}

template <int N>
Status ControlLoop<N>::step(const JointState<N>& state, JointCommand<N>& out) {
  if (mode_ == LoopMode::kIdle || !trajectory_) {
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

  // 1. Safety: leave tracking for the hold at the q of this moment.
  if (mode_ == LoopMode::kTracking || mode_ == LoopMode::kFinished) {
    if (!finite || hal_stale || outside_safe_set(model_, state, config_.safety.soft_margin_rad)) {
      hold_.engage(state.q.allFinite() ? state.q : q_last_finite_);
      mode_ = LoopMode::kHolding;
    }
  }
  if (state.q.allFinite()) {
    q_last_finite_ = state.q;
  }

  // 2.-3. Tracking reference and torque, or the hold.
  const planning::CartesianSample x_d = trajectory_->sample(t);
  LogRecord<N> record;
  if (mode_ == LoopMode::kTracking || mode_ == LoopMode::kFinished) {
    const double dt = 1.0 / config_.control_rate_hz;
    const planning::JointReference<N>& ref = clik_.step(x_d, dt);
    out = computed_torque_.compute(state, ref);
    add_clamped(model_, friction_.torque(ref.qd), out.tau);
    record.q_ref = ref.q;
    record.qd_ref = ref.qd;
    if (t >= config_.trajectory.duration_s + config_.trajectory.hold_s) {
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
  log_.push(record);

  if (mode_ == LoopMode::kFault) {
    return Status::error("control loop: fault, non-finite torque");
  }
  return Status::ok();
}

#define COBOT_PIPELINE_CONTROL_LOOP_INSTANTIATE(N) template class ControlLoop<(N)>;
COBOT_INSTANTIATE_DOF(COBOT_PIPELINE_CONTROL_LOOP_INSTANTIATE)
#undef COBOT_PIPELINE_CONTROL_LOOP_INSTANTIATE

}  // namespace cobot::pipeline
