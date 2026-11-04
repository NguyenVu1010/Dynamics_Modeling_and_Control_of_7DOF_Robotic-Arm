// ControlLoop::jog_tcp (feature manual-commands, ADR-022 "Lệnh tay" and "Đã chốt ở PR
// manual-commands"): one axis unit vector e fixed in base coordinates at the jog start (tool
// frame: that axis of R_b_tcp at the start); scalar speed v ramps to direction * speed * v_max at
// speed * a_max; desired TCP pose p += v dt e or R = so3_exp(v dt e) R; tip reference
// T_b_tcp * inverse(T_tip_tcp) through CLIK. Near-singularity: CLIK condition number >
// clik.cond_threshold -> refused at the start, ramp to 0 with kSingularity while running.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/lie.hpp"
#include "cobot/planning/clik.hpp"

#include "manual_loop_util.hpp"

namespace cobot::pipeline::test {
namespace {

// +x in the base frame at 10 %: v -> 0.1 * 0.25 = 0.025 m/s at 0.1 * 0.5 = 0.05 m/s^2.
// Refreshed for 1000 steps, then release:
//   ramp up 0.025 / 0.05 = 0.5 s: 0.5 * 0.05 * 0.5^2 = 6.25 mm
//   cruise 1.0 - 0.5 = 0.5 s: 0.5 * 0.025 = 12.5 mm
//   brake 0.5 s: 6.25 mm                                    total 25.0 mm
// The orientation of the desired pose never changes; y and z neither.
TEST(ManualJogTcp, TranslationAlongBaseXKeepsTheOrientation) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  Host h = make_host(*setup, manual_config(*setup));
  const Pose start = h.loop().reference_pose();

  ASSERT_TRUE(h.loop().jog_tcp(JogAxis::kX, +1, JogFrame::kBase, kSpeed).is_ok());
  step_refreshed(h, 1000, [&] {
    EXPECT_TRUE(h.loop().jog_tcp(JogAxis::kX, +1, JogFrame::kBase, kSpeed).is_ok());
  });
  EXPECT_EQ(h.mode(), LoopMode::kTracking);
  h.loop().request_stop();
  const int brake = step_until_standby(h, 1000);
  ASSERT_EQ(h.mode(), LoopMode::kStandby);
  EXPECT_LE(brake, 510);  // 500 steps of braking
  for (int k = 0; k < 20; ++k) {
    h.step();
  }
  EXPECT_EQ(h.loop().manual_stop(), ManualStop::kStopped);
  EXPECT_EQ(h.loop().command_mode(), CommandMode::kJogTcp);

  const double max_rot = 0.1 * kDegree;
  double x_last = start.p.x();
  for (std::size_t i = 0; i < h.rows.size(); ++i) {
    const Record& r = h.rows[i];
    ASSERT_NE(r.mode, LoopMode::kHolding) << i;
    EXPECT_EQ(r.cmd_mode, CommandMode::kJogTcp) << i;
    EXPECT_LT(rot_dist(r.T_b_e, start), max_rot) << "measured orientation, row " << i;
    if (r.mode != LoopMode::kTracking) {
      continue;
    }
    EXPECT_LE((r.T_b_d.R - start.R).cwiseAbs().maxCoeff(), 1e-12) << i;
    EXPECT_LE(std::abs(r.T_b_d.p.y() - start.p.y()), 1e-12) << i;
    EXPECT_LE(std::abs(r.T_b_d.p.z() - start.p.z()), 1e-12) << i;
    EXPECT_GE(r.T_b_d.p.x(), x_last - 1e-15) << "x goes back, row " << i;
    x_last = r.T_b_d.p.x();
  }
  EXPECT_NEAR(x_last - start.p.x(), 0.025, 1e-4);
  EXPECT_NEAR(h.loop().reference_pose().p.x() - start.p.x(), 0.025, 1e-4);
  EXPECT_LE(std::abs(h.loop().reference_pose().p.y() - start.p.y()), 1e-6);
  EXPECT_LE(std::abs(h.loop().reference_pose().p.z() - start.p.z()), 1e-6);
}

// rz in the tool frame with the TCP 0.1 m along the tip z axis, 10 %: w -> 0.1 * 1.0 = 0.1
// rad/s at 0.1 * 2.0 = 0.2 rad/s^2. Refreshed for 500 steps then release:
//   ramp up 0.1 / 0.2 = 0.5 s: 0.5 * 0.2 * 0.5^2 = 0.025 rad; no cruise; brake 0.025 rad
//   total 0.05 rad about the start's tool z axis.
// The rotation turns about the TCP point: (T_b_d * T_tip_tcp).p stays where it started.
TEST(ManualJogTcp, ToolRotationTurnsAboutTheTcpPoint) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  Config<kN> cfg = manual_config(*setup);
  cfg.tcp_offset.T_tip_tcp.p = Vec3(0.0, 0.0, 0.1);
  const Pose T_tip_tcp = cfg.tcp_offset.T_tip_tcp;
  Host h = make_host(*setup, cfg);
  const Pose start = h.loop().reference_pose();  // tip
  const Pose tcp0 = common::compose(start, T_tip_tcp);
  const Vec3 z_tool = tcp0.R.col(2);

  ASSERT_TRUE(h.loop().jog_tcp(JogAxis::kRz, +1, JogFrame::kTool, kSpeed).is_ok());
  step_refreshed(h, 500, [&] {
    EXPECT_TRUE(h.loop().jog_tcp(JogAxis::kRz, +1, JogFrame::kTool, kSpeed).is_ok());
  });
  h.loop().request_stop();
  step_until_standby(h, 1000);
  ASSERT_EQ(h.mode(), LoopMode::kStandby);
  EXPECT_EQ(h.loop().manual_stop(), ManualStop::kStopped);

  const Record* last = nullptr;
  for (std::size_t i = 0; i < h.rows.size(); ++i) {
    const Record& r = h.rows[i];
    ASSERT_NE(r.mode, LoopMode::kHolding) << i;
    EXPECT_EQ(r.cmd_mode, CommandMode::kJogTcp) << i;
    if (r.mode != LoopMode::kTracking) {
      continue;
    }
    last = &r;
    const Pose tcp = common::compose(r.T_b_d, T_tip_tcp);
    EXPECT_LE((tcp.p - tcp0.p).norm(), 1e-9) << "TCP point moved, row " << i;
  }
  ASSERT_NE(last, nullptr);
  const Vec3 phi = common::so3_log(last->T_b_d.R * start.R.transpose());
  EXPECT_NEAR(phi.dot(z_tool), 0.05, 5e-4);
  EXPECT_LE((phi - phi.dot(z_tool) * z_tool).norm(), 1e-9) << "axis is not the tool z axis";
}

// Start q = home with q6 = 10 deg (nearer the wrist singularity at q6 = 0). c0 = the CLIK
// condition number of the first standby step = cond(J(q_start)).
struct SingularStart {
  Config<kN> cfg;
  double c0 = 0.0;
};

SingularStart singular_start(const Setup& setup) {
  SingularStart out{manual_config(setup), 0.0};
  out.cfg.home_q_rad(5) = 10.0 * kDegree;
  out.c0 = planning::jacobian_condition_number<kN>(jacobian_at(setup.model, out.cfg.home_q_rad));
  return out;
}

// (a) threshold 0.9 c0: the TCP jog is refused at the start.
TEST(ManualJogTcp, StartAboveTheThresholdIsRefused) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  SingularStart ss = singular_start(*setup);
  ASSERT_TRUE(std::isfinite(ss.c0));
  ss.cfg.clik.cond_threshold = 0.9 * ss.c0;
  Host h = make_host(*setup, ss.cfg, 1);
  ASSERT_EQ(h.mode(), LoopMode::kStandby);
  const common::Status st = h.loop().jog_tcp(JogAxis::kX, +1, JogFrame::kBase, kSpeed);
  ASSERT_FALSE(st.is_ok());
  EXPECT_TRUE(contains(st.reason(), "near a singularity")) << st.reason();
  h.step();
  EXPECT_EQ(h.mode(), LoopMode::kStandby);
  // A joint jog is still accepted (the way out).
  EXPECT_TRUE(h.loop().jog_joint(5, +1, kSpeed).is_ok());
}

// True when, after a CLIK step that produced (q_ref, qd_ref), the TCP jog's soft-limit brake
// fires: for some joint the distance of q_ref to its jog limit in the direction qd_ref moves,
// minus qd_ref^2 / (2 speed a_max_j), is <= 0.
bool soft_limit_brake(const model::Arm7& m,
                      const Config<kN>& cfg,
                      const Vec& q_ref,
                      const Vec& qd_ref,
                      double speed) {
  const double band = cfg.safety.soft_margin_rad + cfg.manual.jog_margin_rad;
  for (int j = 0; j < kN; ++j) {
    const model::JointParams& p = m.joints[static_cast<std::size_t>(j)];
    const double v = qd_ref(j);
    if (v == 0.0) {
      continue;
    }
    const double dist = v > 0.0 ? (p.q_max - band) - q_ref(j) : q_ref(j) - (p.q_min + band);
    if (dist - v * v / (2.0 * speed * cfg.manual.joint_a_max_rad_s2(j)) <= 0.0) {
      return true;
    }
  }
  return false;
}

// (b) threshold 1.02 c0: each of the 12 (axis, direction) TCP jogs from that start (fresh loop
// each), refreshed up to 2000 steps. The condition number CLIK computes in step k is that of
// J(q_ref of step k - 1). Whenever it exceeds the threshold during the jog, the jog ends with
// kSingularity within the braking time (0.5 s = 500 steps at 10 %) + 2 steps, never holding. A
// jog whose soft-limit brake fires first ends with kSoftLimit (rx -1 turns q6 toward its jog
// limit at 0 deg); a jog with neither is released after 2000 steps and ends with kStopped. At
// least one of the 12 jogs ends with kSingularity (offline CLIK along the same desired poses:
// y -1 crosses near step 930, z +1 near step 1770).
TEST(ManualJogTcp, CrossingTheThresholdRampsToRestWithSingularity) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  SingularStart ss = singular_start(*setup);
  ss.cfg.clik.cond_threshold = 1.02 * ss.c0;
  const double threshold = ss.cfg.clik.cond_threshold;
  const std::array<JogAxis, 6> axes = {
      JogAxis::kX, JogAxis::kY, JogAxis::kZ, JogAxis::kRx, JogAxis::kRy, JogAxis::kRz};
  int singular_ends = 0;
  std::ostringstream summary;
  summary << "c0 " << ss.c0 << ", threshold " << threshold << "\n";
  for (const JogAxis axis : axes) {
    for (const int dir : {+1, -1}) {
      const std::string name =
          "axis " + std::to_string(static_cast<int>(axis)) + " dir " + std::to_string(dir);
      Host h = make_host(*setup, ss.cfg, 1);
      ASSERT_EQ(h.mode(), LoopMode::kStandby);
      Vec q_prev = h.loop().joint_reference();
      ASSERT_TRUE(h.loop().jog_tcp(axis, dir, JogFrame::kBase, kSpeed).is_ok()) << name;
      int k_cross = -1;
      int k_soft = -1;
      int k_standby = -1;
      double c_max = 0.0;
      for (int k = 0; k < 4000 && k_standby < 0; ++k) {
        if (k > 0 && k < 2000 && k % kRefreshEvery == 0) {
          h.loop().jog_tcp(axis, dir, JogFrame::kBase, kSpeed);
        }
        if (k == 2000 && k_cross < 0 && k_soft < 0) {
          h.loop().request_stop();  // release a jog that never had to brake
        }
        h.step();
        const Record& r = h.rows.back();
        ASSERT_NE(r.mode, LoopMode::kHolding) << name << " step " << k;
        if (r.mode == LoopMode::kStandby) {
          k_standby = k;
          break;
        }
        const double c = planning::jacobian_condition_number<kN>(jacobian_at(setup->model, q_prev));
        c_max = std::max(c_max, c);
        if (k_cross < 0 && k_soft < 0 && c > threshold) {
          k_cross = k;
        }
        if (k_cross < 0 && k_soft < 0 &&
            soft_limit_brake(setup->model, ss.cfg, r.q_ref, r.qd_ref, kSpeed)) {
          k_soft = k;
        }
        q_prev = r.q_ref;
      }
      summary << name << ": c_max " << c_max << ", crossed at " << k_cross << ", soft limit at "
              << k_soft << ", standby at " << k_standby << ", stop "
              << to_string(h.loop().manual_stop()) << "\n";
      ASSERT_GE(k_standby, 0) << name;
      if (k_cross >= 0) {
        EXPECT_EQ(h.loop().manual_stop(), ManualStop::kSingularity) << name;
        EXPECT_LE(k_standby - k_cross, 502) << name;
        if (h.loop().manual_stop() == ManualStop::kSingularity) {
          ++singular_ends;
        }
      } else if (k_soft >= 0) {
        EXPECT_EQ(h.loop().manual_stop(), ManualStop::kSoftLimit) << name;
      } else {
        EXPECT_EQ(h.loop().manual_stop(), ManualStop::kStopped) << name;
      }
    }
  }
  EXPECT_GE(singular_ends, 1) << summary.str();
}

// Bad arguments are rejected with nothing started.
TEST(ManualJogTcp, BadArgumentsAreRejected) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  Host h = make_host(*setup, manual_config(*setup));
  EXPECT_FALSE(h.loop().jog_tcp(JogAxis::kX, 0, JogFrame::kBase, kSpeed).is_ok());
  EXPECT_FALSE(h.loop().jog_tcp(JogAxis::kX, -2, JogFrame::kBase, kSpeed).is_ok());
  EXPECT_FALSE(h.loop().jog_tcp(JogAxis::kX, +1, JogFrame::kBase, 0.0).is_ok());
  EXPECT_FALSE(h.loop().jog_tcp(JogAxis::kX, +1, JogFrame::kBase, 1.2).is_ok());
  h.step();
  EXPECT_EQ(h.mode(), LoopMode::kStandby);
  EXPECT_EQ(h.loop().command_mode(), CommandMode::kWaypoints);
}

// A TCP jog starting with a joint inside its jog band is refused: joint 4 rests 4 deg from its
// upper limit (inside the 5 deg jog band, outside the 3 deg hold band).
TEST(ManualJogTcp, StartWithAJointInTheJogBandIsRefused) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  Config<kN> cfg = manual_config(*setup);
  cfg.home_q_rad(3) = q_max_of(setup->model)(3) - 4.0 * kDegree;
  Host h = make_host(*setup, cfg, 10);
  ASSERT_EQ(h.mode(), LoopMode::kStandby);
  const common::Status st = h.loop().jog_tcp(JogAxis::kZ, +1, JogFrame::kBase, kSpeed);
  ASSERT_FALSE(st.is_ok());
  EXPECT_TRUE(contains(st.reason(), "soft limit")) << st.reason();
}

// The TCP move: load(plan_tcp_move's trajectory, kMoveTcp). Rows log move_tcp, the loop ends in
// standby with kFinished, the final task error is below 1 mm. 0.1 m along +x at 10 %: 7.5 s.
TEST(ManualMoveTcp, LoadWithMoveTcpRunsToStandby) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const Config<kN> cfg = manual_config(*setup);
  Host h = make_host(*setup, cfg);
  const Pose start = h.loop().reference_pose();
  const Pose goal{start.R, start.p + Vec3(0.1, 0.0, 0.0)};
  const auto plan = plan_tcp_move<kN>(
      planner_model(*setup, cfg), cfg, h.loop().joint_reference(), start, goal, kSpeed);
  ASSERT_TRUE(plan) << plan.reason();
  ASSERT_TRUE(plan.value().ok);
  ASSERT_TRUE(plan.value().trajectory.has_value());
  const planning::WaypointTrajectory& traj = *plan.value().trajectory;

  // Not a trajectory command mode: rejected, nothing starts.
  for (const CommandMode bad : {CommandMode::kJogJoint,
                                CommandMode::kJogTcp,
                                CommandMode::kMoveJoint,
                                CommandMode::kConfig}) {
    EXPECT_FALSE(h.loop().load(traj, bad).is_ok()) << to_string(bad);
  }
  h.step();
  EXPECT_EQ(h.mode(), LoopMode::kStandby);

  ASSERT_TRUE(h.loop().load(traj, CommandMode::kMoveTcp).is_ok());
  EXPECT_EQ(h.loop().command_mode(), CommandMode::kMoveTcp);
  const int steps = step_until_standby(h, 9000);
  ASSERT_EQ(h.mode(), LoopMode::kStandby) << steps;
  for (int k = 0; k < 20; ++k) {
    h.step();
  }
  EXPECT_EQ(h.loop().manual_stop(), ManualStop::kFinished);
  for (std::size_t i = 0; i < h.rows.size(); ++i) {
    ASSERT_NE(h.rows[i].mode, LoopMode::kHolding) << i;
    EXPECT_EQ(h.rows[i].cmd_mode, CommandMode::kMoveTcp) << i;
  }
  EXPECT_NEAR(static_cast<double>(steps) * kDt, traj.duration_s(), 0.01);
  EXPECT_LT(h.rows.back().task_error.head<3>().norm(), 1e-3);
  EXPECT_LT(pos_dist(h.rows.back().T_b_e, goal), 1e-3);
}

// load() keeps its default command mode: kWaypoints.
TEST(ManualMoveTcp, PlainLoadLogsWaypoints) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  Host h = make_host(*setup, manual_config(*setup));
  const Pose ref = h.loop().reference_pose();
  const auto traj = make_trajectory(ref, {waypoint(short_goal(ref), 1.0, true)});
  ASSERT_TRUE(traj);
  ASSERT_TRUE(h.loop().load(*traj).is_ok());
  EXPECT_EQ(h.loop().command_mode(), CommandMode::kWaypoints);
  h.step();
  EXPECT_EQ(h.rows.back().cmd_mode, CommandMode::kWaypoints);
}

}  // namespace
}  // namespace cobot::pipeline::test
