#pragma once

#include <cstdint>

#include "cobot/common/result.hpp"
#include "cobot/common/types.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/path_validation.hpp"
#include "cobot/planning/cartesian_trajectory.hpp"
#include "cobot/planning/clik.hpp"

// Manual commands of the studio (feature manual-commands, ADR-022 "Lệnh tay"): jog a joint or the
// TCP, move to a joint or TCP target. The planners below run off the control thread (they
// allocate on error); ControlLoop runs the checked plan. The generators at the end are the
// per-step motion of ControlLoop (no allocation).
namespace cobot::pipeline {

enum class JogAxis { kX, kY, kZ, kRx, kRy, kRz };  // values 0..5 (the bridge wire uses them)
enum class JogFrame { kBase, kTool };              // 0, 1
// Why the last jog or move ended (kNone while one runs or before any).
enum class ManualStop { kNone, kFinished, kStopped, kDeadman, kSoftLimit, kSingularity };
// "none", "finished", "stopped", "deadman", "soft_limit", "singularity"
const char* to_string(ManualStop stop);

// Synchronised trapezoid (ADR-022 "Đã chốt ở PR manual-commands"): one normalised profile s(t) in
// [0, 1] for every joint, q(t) = q_start + s(t) (q_goal - q_start): a straight line in joint
// space, every joint arrives at duration_s. With d_j = |q_goal_j - q_start_j| (joints with d_j = 0
// ignored):
//   s_v_max = min_j speed qd_max_j / d_j,  s_a_max = min_j speed a_max_j / d_j
//   trapezoid when s_v_max^2 / s_a_max < 1: duration = 1 / s_v_max + s_v_max / s_a_max
//   triangle otherwise:                      duration = 2 sqrt(1 / s_a_max), peak s' =
//   sqrt(s_a_max)
template <int N>
struct JointMove {
  VecN<N> q_start = VecN<N>::Zero();
  VecN<N> q_goal = VecN<N>::Zero();
  double s_v_max = 0.0, s_a_max = 0.0, duration_s = 0.0;
  // s, s', s'' at t (clamped to [0, duration_s]); no allocation.
  void sample(double t_s, double& s, double& s_d, double& s_dd) const;
};

// Off the control thread (allocates on error). Errors, in this order, name the joint 1-based:
//   speed outside (0, 1]                         "move_joint: speed 1.5 outside (0, 1]"
//   non-finite goal                              "move_joint: goal joint 3 not finite"
//   goal within soft_margin + jog_margin of a finite URDF limit
//                      "move_joint: goal joint 4 -0.45 rad is within 0.0873 rad of its limit -0.4"
//   every |q_goal - q_start| <= 1e-9             "move_joint: already at the goal"
//   torque along the profile, sampled every config.path_validation.dt_s (and at the end):
//   |RNEA(q, qd, qdd) + friction feed-forward(qd)| > path_validation.torque_ratio * tau_max_j
//                      "move_joint: torque joint 2 0.95 > 0.9 at t = 1.23 s"
template <int N>
common::Result<JointMove<N>> plan_joint_move(const model::Model<N>& model,
                                             const Config<N>& config,
                                             const VecN<N>& q_start,
                                             const VecN<N>& q_goal,
                                             double speed);

// A straight TCP move (ADR-022): validate_path from (q_start, T_b_start_tip) to one stop waypoint
// at T_b_goal_tip with an auto duration, CartesianLimits = config.manual tcp limits * speed, and
// config.path_validation except joint_margin_rad = safety.soft_margin_rad + manual.jog_margin_rad
// (plan ambiguity 11). Error for speed outside (0, 1] or what validate_path returns as an error;
// otherwise the PathValidation (ok or with issues), exactly as Preview would report them.
template <int N>
common::Result<PathValidation<N>> plan_tcp_move(const model::Model<N>& model,
                                                const Config<N>& config,
                                                const VecN<N>& q_start,
                                                const common::Pose& T_b_start_tip,
                                                const common::Pose& T_b_goal_tip,
                                                double speed);

// State of a running jog inside ControlLoop: a scalar velocity v ramped toward
// direction * v_max (or 0 once `braking` names why it stops) by at most a_max * dt per step.
struct ManualJog {
  int joint = -1;  // joint jog: 0-based joint; TCP jog: -1
  JogAxis axis = JogAxis::kX;
  JogFrame frame = JogFrame::kBase;
  int direction = 1;
  double speed = 0.0;
  double v_max = 0.0;              // speed * limit at 100 % (rad/s or m/s)
  double a_max = 0.0;              // speed * acceleration at 100 % (rad/s^2 or m/s^2)
  double v = 0.0;                  // signed, of the last step
  std::int64_t idle_steps = 0;     // steps since the start or the last refresh
  std::int64_t deadman_steps = 0;  // deadman_s * control_rate_hz
  ManualStop braking = ManualStop::kNone;
  Vec3 e = Vec3::UnitX();  // TCP jog: axis in b, fixed at the start
  common::Pose T_b_tcp;    // TCP jog: desired TCP pose
  common::Pose T_tcp_tip;  // TCP jog: inverse(T_tip_tcp)
};

// Joint jog step: the deadman count, then v toward its target, capped at the largest speed whose
// stepped ramp (v, v - a_max dt, ..., one step each) covers the distance d left to the jog limit
// [q_lo, q_hi], so that q_ref comes to rest on the limit with |qdd_ref| <= a_max (reached: set
// there, kSoftLimit); then q_ref(j) += v dt. Only joint `jog.joint` of `ref` changes. True when v
// is back at rest.
template <int N>
bool joint_jog_step(
    ManualJog& jog, double q_lo, double q_hi, double dt, planning::JointReference<N>& ref);

// TCP jog step: the deadman count, v toward its target, the desired TCP pose integrated
// (p += v dt e, or R = so3_exp(v dt e) R) and the tip sample T_b_tcp * T_tcp_tip with twist
// [v_tcp + w x r; w] and derivative [a_tcp + alpha x r + w x (w x r); alpha], r = p_tip - p_tcp.
// True when v is back at rest.
bool tcp_jog_step(ManualJog& jog, double dt, planning::CartesianSample& x_d);

// TCP jog brake after a CLIK step: true when for some joint the distance of q_ref to its jog
// limit [q_lo, q_hi], in the direction qd_ref moves, minus qd_ref^2 / (2 speed a_max_j) is <= 0.
template <int N>
bool tcp_jog_near_limit(const planning::JointReference<N>& ref,
                        const VecN<N>& q_lo,
                        const VecN<N>& q_hi,
                        const VecN<N>& a_max,
                        double speed);

}  // namespace cobot::pipeline
