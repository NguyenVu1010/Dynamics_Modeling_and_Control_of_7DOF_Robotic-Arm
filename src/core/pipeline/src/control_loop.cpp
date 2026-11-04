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
#include "cobot/dynamics/dynamics.hpp"
#include "cobot/dynamics/harmonic_spring.hpp"
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
// Manual commands: a joint this close to its jog limit is at it; a joint move plan must start
// this close (max norm) to joint_reference().
constexpr double kAtJogLimitRad = 1e-3;
constexpr double kMoveStartTolRad = 1e-3;

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

// kJointLimit when a q is within soft_margin of a joint limit (or past it), else kOverspeed when
// a |qd| is above qd_max, else kNone.
template <int N>
HoldReason outside_safe_set(const model::Model<N>& m, const JointState<N>& s, double soft_margin) {
  for (int i = 0; i < N; ++i) {
    const model::JointParams& j = m.joints[static_cast<std::size_t>(i)];
    if (s.q(i) - j.q_min < soft_margin || j.q_max - s.q(i) < soft_margin) {
      return HoldReason::kJointLimit;
    }
  }
  for (int i = 0; i < N; ++i) {
    if (std::abs(s.qd(i)) > m.joints[static_cast<std::size_t>(i)].qd_max) {
      return HoldReason::kOverspeed;
    }
  }
  return HoldReason::kNone;
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
ControlLoop<N>::ControlLoop(const model::Model<N>& model,
                            const Config<N>& config,
                            Recorder<N>* recorder)
    : model_(model),
      config_(config),
      clik_(model, config.clik),
      computed_torque_(model, config.computed_torque),
      friction_(config.friction_compensation),
      hold_(model, config.computed_torque),
      recorder_(recorder) {
  const double band = config.safety.soft_margin_rad + config.manual.jog_margin_rad;
  for (int j = 0; j < N; ++j) {
    jog_lo_(j) = model.joints[static_cast<std::size_t>(j)].q_min + band;
    jog_hi_(j) = model.joints[static_cast<std::size_t>(j)].q_max - band;
  }
  cmd_ = external() ? CommandMode::kWaypoints : CommandMode::kConfig;
}

template <int N>
const JointState<N>& ControlLoop<N>::compensated(const JointState<N>& measured) {
  const DeflectionCompConfig<N>& d = config_.deflection_comp;
  if (!d.enabled || !measured.q_from_motor || !measured.q.allFinite()) {
    return measured;
  }
  const common::VecN<N> g = dynamics::gravity_torque(model_, measured.q);
  state_hat_ = measured;
  for (int j = 0; j < N; ++j) {
    state_hat_.q(j) -= dynamics::harmonic_spring_deflection(g(j),
                                                            d.stiffness_k1_nm_rad(j),
                                                            d.stiffness_k2_nm_rad(j),
                                                            d.stiffness_k3_nm_rad(j),
                                                            d.torque_t1_nm(j),
                                                            d.torque_t2_nm(j));
  }
  return state_hat_;
}

template <int N>
Status ControlLoop<N>::start(const JointState<N>& measured) {
  const JointState<N>& initial = compensated(measured);
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
  hold_reason_ = HoldReason::kNone;
  cmd_ = external() ? CommandMode::kWaypoints : CommandMode::kConfig;
  manual_stop_ = ManualStop::kNone;
  estop_requested_ = false;
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
Status ControlLoop<N>::step(const JointState<N>& measured, JointCommand<N>& out) {
  return step(measured, out, unknown_truth_);
}

template <int N>
Status ControlLoop<N>::step(const JointState<N>& measured,
                            JointCommand<N>& out,
                            const PlantTruth<N>& truth) {
  if (mode_ == LoopMode::kIdle) {
    out.tau.setZero();
    return Status::error("control loop: step before start");
  }
  const bool timed = !config_.deterministic;
  const std::chrono::steady_clock::time_point t0 =
      timed ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
  const JointState<N>& state = compensated(measured);

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
  HoldReason reason = HoldReason::kNone;
  if (!finite) {
    reason = HoldReason::kNonFinite;
  } else if (stamp_backwards) {
    reason = HoldReason::kStampBackwards;
  } else if (stamp_age > config_.safety.hal_timeout_s) {
    reason = HoldReason::kStateTimeout;
  } else {
    reason = outside_safe_set(model_, state, config_.safety.soft_margin_rad);
  }
  if (reason == HoldReason::kNone && estop_requested_) {
    reason = HoldReason::kEstop;
  }
  estop_requested_ = false;

  // 1. Safety: leave tracking (or standby) for the hold at the q of this moment.
  if (mode_ == LoopMode::kTracking || mode_ == LoopMode::kFinished || mode_ == LoopMode::kStandby) {
    if (reason != HoldReason::kNone) {
      hold_.engage(state.q.allFinite() ? state.q : q_last_finite_);
      mode_ = LoopMode::kHolding;
      hold_reason_ = reason;
    }
  }
  if (state.q.allFinite()) {
    q_last_finite_ = state.q;
  }

  // 2.-3. Tracking reference and torque (standby: the same path on a fixed reference), or the
  // hold. External mode keeps the last reference while holding.
  const double dt = 1.0 / config_.control_rate_hz;
  bool joint_row = false;  // joint modes: the reference comes from the profile, not CLIK
  if (!external()) {
    reference_ = trajectory_->sample(t);
  } else if (joint_mode()) {
    joint_row = true;
    advance_joint(dt);
  } else if (mode_ == LoopMode::kTracking && cmd_ == CommandMode::kJogTcp) {
    if (tcp_jog_step(jog_, dt, reference_)) {
      manual_stop_ = jog_.braking;
      enter_standby(reference_.T_b_d);
    }
  } else if (mode_ == LoopMode::kTracking) {
    advance_external(dt);
  }
  const planning::CartesianSample& x_d = reference_;
  LogRecord<N> record;
  if (mode_ == LoopMode::kTracking || mode_ == LoopMode::kFinished || mode_ == LoopMode::kStandby) {
    const planning::JointReference<N>& ref = joint_row ? joint_ref_ : clik_.step(x_d, dt);
    if (mode_ == LoopMode::kTracking && cmd_ == CommandMode::kJogTcp &&
        jog_.braking == ManualStop::kNone) {
      if (clik_.condition_number() > config_.clik.cond_threshold) {
        jog_.braking = ManualStop::kSingularity;
      } else if (tcp_jog_near_limit<N>(
                     ref, jog_lo_, jog_hi_, config_.manual.joint_a_max_rad_s2, jog_.speed)) {
        jog_.braking = ManualStop::kSoftLimit;
      }
    }
    out = computed_torque_.compute(state, ref);
    add_clamped(model_, friction_.torque(ref.qd), out.tau);
    record.q_ref = ref.q;
    record.qd_ref = ref.qd;
    record.qdd_ref = ref.qdd;
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

  // 5. Log; a full kStopWhenFull recorder drops and counts the record.
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
    record.task_error =
        joint_row ? common::pose_error(x_d.T_b_d, fk.T_b_e) : clik_.reference().task_error;
  }
  if (timed) {
    const std::chrono::duration<double, std::micro> elapsed = std::chrono::steady_clock::now() - t0;
    record.step_time_us = elapsed.count();
  }
  record.hold_reason = hold_reason_;
  record.q_raw = measured.q;
  record.q_true = truth.q;
  record.qd_true = truth.qd;
  record.qdd_true = truth.qdd;
  record.q_motor_true = truth.q_motor;
  record.tau_motor = truth.tau_motor;
  record.tau_friction = truth.tau_friction;
  record.cmd_mode = cmd_;
  last_record_ = record;
  if (recorder_ != nullptr) {
    recorder_->record(record);
  }

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
bool ControlLoop<N>::stop_rate(double dt, double& r, double& r_dot) {
  const double x = static_cast<double>(stop_steps_) * dt / config_.stop_time_s;
  r = 1.0 - x * x * (3.0 - 2.0 * x);
  r_dot = -6.0 * x * (1.0 - x) / config_.stop_time_s;
  ++stop_steps_;
  return x >= 1.0 - kStopEndTolerance;
}

template <int N>
void ControlLoop<N>::advance_external(double dt) {
  const planning::WaypointTrajectory& w = *waypoints_;
  double r = 1.0;
  double r_dot = 0.0;
  bool done = tau_ >= w.duration_s();
  if (stopping_) {
    const bool rest = stop_rate(dt, r, r_dot);
    done = done || rest;
  }
  const planning::CartesianSample s = w.sample(tau_);
  if (done) {
    manual_stop_ = stopping_ ? ManualStop::kStopped : ManualStop::kFinished;
    enter_standby(s.T_b_d);
    return;
  }
  reference_.T_b_d = s.T_b_d;
  reference_.xd_d = r * s.xd_d;
  reference_.xdd_d = (r * r) * s.xdd_d + r_dot * s.xd_d;
  tau_ += r * dt;
}

template <int N>
bool ControlLoop<N>::joint_mode() const {
  return mode_ == LoopMode::kTracking &&
         (cmd_ == CommandMode::kJogJoint || cmd_ == CommandMode::kMoveJoint);
}

template <int N>
const VecN<N>& ControlLoop<N>::joint_reference() const {
  if (joint_mode()) {
    return joint_ref_.q;
  }
  return mode_ == LoopMode::kHolding ? hold_.q_hold() : clik_.reference().q;
}

// One step of a joint mode; at rest CLIK restarts at q_ref and the loop stands by at FK(q_ref).
template <int N>
void ControlLoop<N>::advance_joint(double dt) {
  const bool jog = cmd_ == CommandMode::kJogJoint;
  const int j = jog_.joint;
  const bool rest = jog ? joint_jog_step<N>(jog_, jog_lo_(j), jog_hi_(j), dt, joint_ref_)
                        : advance_joint_move(dt);
  kinematics::FkState<N> fk;
  kinematics::forward_kinematics(model_, joint_ref_.q, fk);
  if (!rest) {
    reference_.T_b_d = fk.T_b_e;
    return;
  }
  if (jog) {
    manual_stop_ = jog_.braking;
  } else {
    manual_stop_ = stopping_ ? ManualStop::kStopped : ManualStop::kFinished;
  }
  clik_.reset(joint_ref_.q);
  enter_standby(fk.T_b_e);
}

// s(t) of the plan at the trajectory time tau_, with the request_stop() time scaling.
template <int N>
bool ControlLoop<N>::advance_joint_move(double dt) {
  double r = 1.0;
  double r_dot = 0.0;
  bool done = tau_ >= move_.duration_s;
  if (stopping_) {
    const bool rest = stop_rate(dt, r, r_dot);
    done = done || rest;
  }
  double s = 0.0;
  double s_d = 0.0;
  double s_dd = 0.0;
  move_.sample(tau_, s, s_d, s_dd);
  const VecN<N> delta = move_.q_goal - move_q0_;
  if (done) {
    joint_ref_.q = stopping_ ? VecN<N>(move_q0_ + s * delta) : move_.q_goal;
    joint_ref_.qd.setZero();
    joint_ref_.qdd.setZero();
    return true;
  }
  joint_ref_.q = move_q0_ + s * delta;
  joint_ref_.qd = (r * s_d) * delta;
  joint_ref_.qdd = (r * r * s_dd + r_dot * s_d) * delta;
  tau_ += r * dt;
  return false;
}

template <int N>
Status ControlLoop<N>::check_jog(const ManualJog& jog, bool& refreshed) {
  refreshed = false;
  const bool tcp = jog.joint < 0;
  if (!external()) {
    return Status::error(tcp ? "jog_tcp: command_source is config"
                             : "jog_joint: command_source is config");
  }
  if (jog.direction != 1 && jog.direction != -1) {
    return Status::error(tcp ? "jog_tcp: direction must be -1 or +1"
                             : "jog_joint: direction must be -1 or +1");
  }
  if (!(jog.speed > 0.0 && jog.speed <= 1.0)) {
    return Status::error(tcp ? "jog_tcp: speed outside (0, 1]" : "jog_joint: speed outside (0, 1]");
  }
  if (mode_ == LoopMode::kTracking &&
      (cmd_ == CommandMode::kJogJoint || cmd_ == CommandMode::kJogTcp)) {
    const bool same = jog_.joint == jog.joint && jog_.axis == jog.axis && jog_.frame == jog.frame &&
                      jog_.direction == jog.direction && jog_.speed == jog.speed;
    if (!same) {
      return Status::error("jog: release the running jog first");
    }
    if (jog_.braking != ManualStop::kNone) {
      return Status::error(tcp ? "jog_tcp: jog is stopping" : "jog_joint: jog is stopping");
    }
    jog_.idle_steps = 0;
    refreshed = true;
    return Status::ok();
  }
  if (mode_ != LoopMode::kStandby) {
    return Status::error(tcp ? "jog_tcp: not in standby" : "jog_joint: not in standby");
  }
  return Status::ok();
}

template <int N>
void ControlLoop<N>::start_jog(const ManualJog& jog, CommandMode cmd) {
  jog_ = jog;
  jog_.deadman_steps = std::llround(config_.manual.deadman_s * config_.control_rate_hz);
  cmd_ = cmd;
  manual_stop_ = ManualStop::kNone;
  stopping_ = false;
  mode_ = LoopMode::kTracking;
}

template <int N>
Status ControlLoop<N>::jog_joint(int joint, int direction, double speed) {
  if (joint < 0 || joint >= N) {
    return Status::error("jog_joint: joint out of range");
  }
  ManualJog jog;
  jog.joint = joint;
  jog.direction = direction;
  jog.speed = speed;
  bool refreshed = false;
  Status st = check_jog(jog, refreshed);
  if (!st.is_ok() || refreshed) {
    return st;
  }
  const double q = clik_.reference().q(joint);
  const double dist = direction > 0 ? jog_hi_(joint) - q : q - jog_lo_(joint);
  if (dist <= kAtJogLimitRad) {
    return Status::error("jog_joint: joint at its soft limit in that direction");
  }
  jog.v_max = speed * model_.joints[static_cast<std::size_t>(joint)].qd_max;
  jog.a_max = speed * config_.manual.joint_a_max_rad_s2(joint);
  joint_ref_ = clik_.reference();
  joint_ref_.qd.setZero();
  joint_ref_.qdd.setZero();
  start_jog(jog, CommandMode::kJogJoint);
  return Status::ok();
}

template <int N>
Status ControlLoop<N>::jog_tcp(JogAxis axis, int direction, JogFrame frame, double speed) {
  const int a = static_cast<int>(axis);
  if (a < 0 || a > static_cast<int>(JogAxis::kRz) ||
      (frame != JogFrame::kBase && frame != JogFrame::kTool)) {
    return Status::error("jog_tcp: bad axis or frame");
  }
  ManualJog jog;
  jog.axis = axis;
  jog.frame = frame;
  jog.direction = direction;
  jog.speed = speed;
  bool refreshed = false;
  Status st = check_jog(jog, refreshed);
  if (!st.is_ok() || refreshed) {
    return st;
  }
  // NaN (CLIK not stepped since start()) is refused too.
  if (!(clik_.condition_number() <= config_.clik.cond_threshold)) {
    return Status::error("jog_tcp: near a singularity");
  }
  const VecN<N>& q = clik_.reference().q;
  for (int j = 0; j < N; ++j) {
    if (q(j) - jog_lo_(j) <= kAtJogLimitRad || jog_hi_(j) - q(j) <= kAtJogLimitRad) {
      return Status::error("jog_tcp: a joint is at its soft limit");
    }
  }
  const ManualConfig<N>& m = config_.manual;
  const bool rotation = axis >= JogAxis::kRx;
  jog.v_max = speed * (rotation ? m.tcp_w_max_rad_s : m.tcp_v_max_m_s);
  jog.a_max = speed * (rotation ? m.tcp_alpha_max_rad_s2 : m.tcp_a_max_m_s2);
  const common::Pose& T_tip_tcp = config_.tcp_offset.T_tip_tcp;
  jog.T_b_tcp = common::compose(reference_.T_b_d, T_tip_tcp);
  jog.T_tcp_tip = common::inverse(T_tip_tcp);
  const int k = a % 3;
  jog.e = frame == JogFrame::kTool ? Vec3(jog.T_b_tcp.R.col(k)) : Vec3(Vec3::Unit(k));
  start_jog(jog, CommandMode::kJogTcp);
  return Status::ok();
}

template <int N>
Status ControlLoop<N>::move_joint(const JointMove<N>& move) {
  if (!external()) {
    return Status::error("move_joint: command_source is config");
  }
  if (mode_ != LoopMode::kStandby) {
    return Status::error("move_joint: not in standby");
  }
  const VecN<N>& q0 = clik_.reference().q;
  if (!((move.q_start - q0).cwiseAbs().maxCoeff() <= kMoveStartTolRad)) {
    return Status::error("move_joint: plan does not start at the reference");
  }
  if (!move.q_goal.allFinite() || !(move.duration_s > 0.0) || !std::isfinite(move.duration_s)) {
    return Status::error("move_joint: invalid plan");
  }
  move_ = move;
  move_q0_ = q0;
  joint_ref_.q = q0;
  joint_ref_.qd.setZero();
  joint_ref_.qdd.setZero();
  joint_ref_.task_error.setZero();
  tau_ = 0.0;
  stopping_ = false;
  cmd_ = CommandMode::kMoveJoint;
  manual_stop_ = ManualStop::kNone;
  mode_ = LoopMode::kTracking;
  return Status::ok();
}

template <int N>
Status ControlLoop<N>::estop() {
  if (!external()) {
    return Status::error("estop: command_source is config");
  }
  if (mode_ == LoopMode::kStandby || mode_ == LoopMode::kTracking) {
    estop_requested_ = true;
  }
  return Status::ok();
}

template <int N>
Status ControlLoop<N>::load(const planning::WaypointTrajectory& trajectory, CommandMode cmd) {
  if (!external()) {
    return Status::error("load: command_source is config");
  }
  if (cmd != CommandMode::kWaypoints && cmd != CommandMode::kMoveTcp) {
    return Status::error("load: command mode must be waypoints or move_tcp");
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
  cmd_ = cmd;
  manual_stop_ = ManualStop::kNone;
  mode_ = LoopMode::kTracking;
  return Status::ok();
}

template <int N>
void ControlLoop<N>::request_stop() {
  const bool jog = cmd_ == CommandMode::kJogJoint || cmd_ == CommandMode::kJogTcp;
  if (mode_ == LoopMode::kTracking && jog && jog_.braking == ManualStop::kNone) {
    jog_.braking = ManualStop::kStopped;
  } else if (mode_ == LoopMode::kTracking && external() && !jog && !stopping_) {
    stopping_ = true;
    stop_steps_ = 0;
  }
}

template <int N>
Status ControlLoop<N>::reset(const JointState<N>& measured) {
  const JointState<N>& state = compensated(measured);
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
  hold_reason_ = HoldReason::kNone;
  estop_requested_ = false;
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
