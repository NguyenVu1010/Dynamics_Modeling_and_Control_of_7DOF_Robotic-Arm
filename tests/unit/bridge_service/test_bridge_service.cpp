// bridge::LoopPort + bridge::CommandService (feature controller-bridge, cobot_studio design 5.4 and
// 6.5, ADR-022): authority, heartbeat watchdog, validation off the control thread, responses,
// segment export and apply config, through a ClientEndpoint as the studio would use them. One
// process, fake clock (bridge_service_util.hpp); config/default.yaml + ideal profile, 10 % speed.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "bridge_service_util.hpp"

namespace cobot::bridge::test {
namespace {

using pipeline::CommandMode;
using pipeline::ManualStop;

bool contains(const std::string& text, const std::string& part) {
  return text.find(part) != std::string::npos;
}

// A harness in standby with a client that claimed authority and beats.
struct Claimed {
  Harness h;
  std::optional<Client> client;

  explicit Claimed(const HarnessOptions& options = HarnessOptions{}) : h(options) {
    if (!h.ok()) {
      return;
    }
    h.steps(20);
    client = open_client(h.name());
    if (!client) {
      return;
    }
    const common::Status st = client->claim(h.now_ns());
    EXPECT_TRUE(st.is_ok()) << st.reason();
    h.beat_with(&*client);
    h.steps(10);  // two polls: the service sees the owner
  }
  bool ok() const { return h.ok() && client.has_value() && client->is_owner(); }
};

// Sends `c` and waits for its response.
std::optional<Response> send_and_await(Harness& h,
                                       Client& client,
                                       const Command<kN>& c,
                                       int max_steps = 100) {
  const auto id = client.send(c);
  if (!id) {
    ADD_FAILURE() << "send: " << id.reason();
    return std::nullopt;
  }
  return h.await(client, id.value(), max_steps);
}

// Accepted move_joint of kMoveJointDurationS from the current reference.
std::optional<Response> start_move_joint(Claimed& c) {
  const Status st = c.h.status();
  EXPECT_EQ(mode_of(st), LoopMode::kStandby);
  const Vec goal = move_joint_goal(vec_of(st.joint_reference));
  auto r = send_and_await(c.h, *c.client, move_joint_command(goal, kSpeed));
  if (r) {
    EXPECT_EQ(r->accepted, 1U) << text_of(*r);
  }
  return r;
}

// Time (row stamp) of the first standby row after t_ns, -1 if none.
std::int64_t standby_after(const Harness& h, std::int64_t t_ns) {
  const std::ptrdiff_t i =
      h.first_row(t_ns, [](const Record& r) { return r.mode == LoopMode::kStandby; });
  return i < 0 ? -1 : h.rows()[static_cast<std::size_t>(i)].t.ns;
}

bool in_standby(const Harness& h) {
  return mode_of(h.status()) == LoopMode::kStandby;
}

// ------------------------------------------------------------------------------- motion commands

TEST(BridgeService, MoveJointIsAcceptedWithTheStepStampAndThePlannedDuration) {
  Claimed c;
  ASSERT_TRUE(c.ok());
  const auto r = start_move_joint(c);
  ASSERT_TRUE(r);
  ASSERT_EQ(r->accepted, 1U) << text_of(*r);
  EXPECT_EQ(r->type, CommandType::kMoveJoint);
  // Hand-checked in bridge_service_util.hpp: 1 / 0.42 + 0.42 / 3 = 2.520952 s.
  EXPECT_NEAR(r->duration_s, kMoveJointDurationS, 1e-5);
  // t_start_ns is the stamp of the step that applied the move: the first move_joint row.
  const std::ptrdiff_t first =
      c.h.first_row(-1, [](const Record& rec) { return rec.cmd_mode == CommandMode::kMoveJoint; });
  ASSERT_GE(first, 0);
  EXPECT_EQ(r->t_start_ns, c.h.rows()[static_cast<std::size_t>(first)].t.ns);

  ASSERT_GE(c.h.run_until([&] { return in_standby(c.h); }, 4000), 0);
  const std::int64_t t_standby = standby_after(c.h, r->t_start_ns);
  ASSERT_GE(t_standby, 0);
  // Standby at t_start + duration, within 2 steps.
  const double end_s = static_cast<double>(t_standby - r->t_start_ns) * 1e-9;
  EXPECT_NEAR(end_s, r->duration_s, 2e-3);
  EXPECT_EQ(c.h.status().manual_stop, manual_stop_value(ManualStop::kFinished));
}

TEST(BridgeService, MoveJointDuringAJogIsRejectedNotInStandby) {
  Claimed c;
  ASSERT_TRUE(c.ok());
  const Vec goal = move_joint_goal(vec_of(c.h.status().joint_reference));
  c.client->write_jog(joint_jog(0, +1, kSpeed));
  c.h.steps(20);
  ASSERT_EQ(mode_of(c.h.status()), LoopMode::kTracking);
  c.client->write_jog(joint_jog(0, +1, kSpeed));  // refresh: the jog keeps running
  const auto r = send_and_await(c.h, *c.client, move_joint_command(goal, kSpeed), 15);
  ASSERT_TRUE(r);
  EXPECT_EQ(r->accepted, 0U);
  EXPECT_TRUE(contains(text_of(*r), "not in standby")) << text_of(*r);
  c.client->write_jog(joint_jog(0, 0, kSpeed));
  EXPECT_GE(c.h.run_until([&] { return in_standby(c.h); }, 500), 0);
}

TEST(BridgeService, UnreachableMoveTcpIsRejectedWithTheIssuesOfPlanTcpMove) {
  Claimed c;
  ASSERT_TRUE(c.ok());
  const Status st = c.h.status();
  ASSERT_EQ(mode_of(st), LoopMode::kStandby);
  const Arm& arm = c.h.arm();
  const common::Pose start_tip = from_wire(st.standby_reference_tip);
  const common::Pose& T_tip_tcp = arm.config.tcp_offset.T_tip_tcp;
  common::Pose goal_tcp = common::compose(start_tip, T_tip_tcp);
  goal_tcp.p += common::Vec3(3.0, 0.0, 0.0);  // 3 m away: out of reach
  // The service's own call: goal tip = T_b_tcp * inverse(T_tip_tcp), from the standby status.
  const common::Pose goal_tip = common::compose(goal_tcp, common::inverse(T_tip_tcp));
  const auto direct = pipeline::plan_tcp_move<kN>(
      arm.controller, arm.config, vec_of(st.standby_q), start_tip, goal_tip, kSpeed);
  ASSERT_TRUE(direct.is_ok()) << direct.reason();
  ASSERT_FALSE(direct.value().ok);
  ASSERT_FALSE(direct.value().issues.empty());
  const pipeline::WaypointIssue& want = direct.value().issues.front();
  ASSERT_EQ(want.issue, pipeline::PathIssue::kUnreachable);

  const auto r = send_and_await(c.h, *c.client, move_tcp_command(goal_tcp, kSpeed));
  ASSERT_TRUE(r);
  EXPECT_EQ(r->type, CommandType::kMoveTcp);
  EXPECT_EQ(r->accepted, 0U);
  ASSERT_GE(r->issue_count, 1U);
  EXPECT_EQ(r->issue_count, direct.value().issues.size());
  const WireIssue& got = r->issues[0];
  EXPECT_EQ(got.waypoint, want.waypoint);
  EXPECT_EQ(got.issue, static_cast<std::uint32_t>(pipeline::PathIssue::kUnreachable));
  EXPECT_EQ(got.joint, want.joint);
  EXPECT_DOUBLE_EQ(got.value, want.value);
  EXPECT_DOUBLE_EQ(got.limit, want.limit);
  // Nothing was posted: the loop stays in standby.
  c.h.steps(50);
  EXPECT_EQ(mode_of(c.h.status()), LoopMode::kStandby);
}

TEST(BridgeService, RunWaypointsAndPreviewUseValidatePathFromTheStandbyStatus) {
  Claimed c;
  ASSERT_TRUE(c.ok());
  const std::vector<planning::Waypoint> tcp = e2e_13_waypoints();
  ASSERT_EQ(tcp.size(), 2U);
  const Status st = c.h.status();
  const Arm& arm = c.h.arm();
  // TCP -> tip with the config's offset (identity in default.yaml), then the panel's exact call.
  std::vector<planning::Waypoint> tip = tcp;
  for (planning::Waypoint& w : tip) {
    w.T_b_goal = common::compose(w.T_b_goal, common::inverse(arm.config.tcp_offset.T_tip_tcp));
  }
  const auto direct = pipeline::validate_path<kN>(arm.controller,
                                                  arm.config.clik,
                                                  arm.config.friction_compensation,
                                                  vec_of(st.standby_q),
                                                  from_wire(st.standby_reference_tip),
                                                  tip,
                                                  arm.config.cartesian_limits,
                                                  arm.config.path_validation);
  ASSERT_TRUE(direct.is_ok()) << direct.reason();
  ASSERT_TRUE(direct.value().ok);
  ASSERT_TRUE(direct.value().trajectory.has_value());

  // Preview: no post, the path and the segment durations.
  const auto preview =
      send_and_await(c.h, *c.client, waypoints_command(CommandType::kPreviewWaypoints, tcp));
  ASSERT_TRUE(preview);
  EXPECT_EQ(preview->type, CommandType::kPreviewWaypoints);
  EXPECT_EQ(preview->accepted, 1U) << text_of(*preview);
  EXPECT_EQ(preview->issue_count, 0U);
  EXPECT_GT(preview->path_count, 1U);
  EXPECT_LE(preview->path_count, kPreviewPoints);
  ASSERT_GE(preview->path_count, 1U);
  const double* last = preview->path[preview->path_count - 1];
  const common::Vec3 last_tcp = tcp.back().T_b_goal.p;
  EXPECT_NEAR(last[0], last_tcp.x(), 1e-9);
  EXPECT_NEAR(last[1], last_tcp.y(), 1e-9);
  EXPECT_NEAR(last[2], last_tcp.z(), 1e-9);
  ASSERT_EQ(preview->segment_count, direct.value().segment_durations_s.size());
  for (std::size_t i = 0; i < direct.value().segment_durations_s.size(); ++i) {
    EXPECT_NEAR(preview->segment_durations_s[i], direct.value().segment_durations_s[i], 1e-12)
        << "segment " << i;
  }
  c.h.steps(50);
  EXPECT_EQ(mode_of(c.h.status()), LoopMode::kStandby) << "a preview posts nothing";

  // Run: accepted with the validated trajectory's duration.
  const auto run =
      send_and_await(c.h, *c.client, waypoints_command(CommandType::kRunWaypoints, tcp));
  ASSERT_TRUE(run);
  EXPECT_EQ(run->type, CommandType::kRunWaypoints);
  ASSERT_EQ(run->accepted, 1U) << text_of(*run);
  EXPECT_NEAR(run->duration_s, direct.value().trajectory->duration_s(), 1e-9);
  const std::ptrdiff_t first = c.h.first_row(-1, [](const Record& rec) {
    return rec.cmd_mode == CommandMode::kWaypoints && rec.mode == LoopMode::kTracking;
  });
  ASSERT_GE(first, 0);
  EXPECT_EQ(run->t_start_ns, c.h.rows()[static_cast<std::size_t>(first)].t.ns);
  const int limit = static_cast<int>(std::ceil(run->duration_s * 1000.0)) + 500;
  EXPECT_GE(c.h.run_until([&] { return in_standby(c.h); }, limit), 0);
}

// ------------------------------------------------------------------------------------------ jog

TEST(BridgeService, JogThroughTheSlotMovesJoint1LikeTheLoopJog) {
  Claimed c;
  ASSERT_TRUE(c.ok());
  const double q0 = c.h.status().joint_reference[0];
  for (int k = 0; k < 500; ++k) {
    if (k % 20 == 0) {
      c.client->write_jog(joint_jog(0, +1, kSpeed));
    }
    c.h.step();
  }
  c.client->write_jog(joint_jog(0, 0, kSpeed));  // release
  ASSERT_GE(c.h.run_until([&] { return in_standby(c.h); }, 1000), 0);
  // v = 0.1 * 2.1 = 0.21 rad/s at a = 0.1 * 15 = 1.5 rad/s^2: ramp 0.14 s covers
  // 0.21^2 / (2 * 1.5) = 0.0147 rad, cruise (0.5 - 0.14) * 0.21 = 0.0756 rad, brake 0.0147 rad:
  // 0.105 rad in total.
  const double moved = c.h.rows().back().q_ref(0) - q0;
  EXPECT_NEAR(moved, 0.105, 1e-3);
  EXPECT_EQ(c.h.status().manual_stop, manual_stop_value(ManualStop::kStopped));
  for (const Record& rec : c.h.rows()) {
    if (rec.mode == LoopMode::kTracking) {
      EXPECT_EQ(rec.cmd_mode, CommandMode::kJogJoint);
    }
  }
}

TEST(BridgeService, AfterATakeoverTheOldOwnersJogWritesAreIgnored) {
  Claimed c;
  ASSERT_TRUE(c.ok());
  for (int k = 0; k < 300; ++k) {
    if (k % 20 == 0) {
      c.client->write_jog(joint_jog(0, +1, kSpeed));
    }
    c.h.step();
  }
  ASSERT_EQ(mode_of(c.h.status()), LoopMode::kTracking);
  // Refresh, then a second client takes authority between two steps.
  c.client->write_jog(joint_jog(0, +1, kSpeed));
  c.h.step();
  auto second = open_client(c.h.name());
  ASSERT_TRUE(second);
  c.client->release();
  const common::Status claimed = second->claim(c.h.now_ns());
  ASSERT_TRUE(claimed.is_ok()) << claimed.reason();
  c.h.stop_beating();
  c.h.beat_with(&*second);
  const std::size_t takeover = c.h.rows().size();
  // The first client keeps writing its jog every 20 steps: ignored (stale token).
  int to_standby = -1;
  for (int k = 1; k <= 600; ++k) {
    if (k % 20 == 0) {
      c.client->write_jog(joint_jog(0, +1, kSpeed));
    }
    c.h.step();
    if (to_standby < 0 && in_standby(c.h)) {
      to_standby = k;
    }
  }
  // The jog ramps down from 0.21 rad/s: the service stops it at its next poll, and without that
  // the deadman fires <= 100 steps after the last valid refresh (one step before the takeover).
  // 150 steps after the takeover qd_ref is below its cruise value.
  ASSERT_GT(c.h.rows().size(), takeover + 150);
  EXPECT_LT(std::abs(c.h.rows()[takeover + 150].qd_ref(0)), 0.21 - 0.01);
  // Standby within the deadman (100 steps) + braking (0.21 / 1.5 = 140 steps) + 20 steps margin.
  ASSERT_GE(to_standby, 0);
  EXPECT_LE(to_standby, 260);
  const std::uint8_t stop = c.h.status().manual_stop;
  EXPECT_TRUE(stop == manual_stop_value(ManualStop::kStopped) ||
              stop == manual_stop_value(ManualStop::kDeadman))
      << static_cast<int>(stop);
  // The stale writes never started a new jog.
  for (std::size_t i = takeover + static_cast<std::size_t>(to_standby); i < c.h.rows().size();
       ++i) {
    ASSERT_EQ(c.h.rows()[i].mode, LoopMode::kStandby) << "row " << i;
  }
  EXPECT_TRUE(c.h.service().studio_in_control());
  EXPECT_TRUE(second->is_owner());
}

TEST(BridgeService, JogAtTheSoftLimitAnswersWithAJogResponse) {
  Claimed c;
  ASSERT_TRUE(c.ok());
  // Joint 4 (index 3) from home -90 deg toward its upper limit -0.4 rad: stops at the jog band
  // -0.4 - (3 + 2) deg = -0.4873 rad, 1.08 rad away at 0.21 rad/s (~5.2 s).
  int k = 0;
  const int done = c.h.run_until(
      [&] {
        const Status st = c.h.status();
        if (mode_of(st) == LoopMode::kStandby &&
            st.manual_stop == manual_stop_value(ManualStop::kSoftLimit)) {
          return true;
        }
        if (k++ % 20 == 0) {
          c.client->write_jog(joint_jog(3, +1, kSpeed));
        }
        return false;
      },
      7000);
  ASSERT_GE(done, 0) << "joint 4 never reached its soft limit";
  const double band =
      c.h.arm().config.safety.soft_margin_rad + c.h.arm().config.manual.jog_margin_rad;
  EXPECT_NEAR(c.h.status().joint_reference[3], c.h.arm().controller.joints[3].q_max - band, 1e-3);

  c.client->write_jog(joint_jog(3, +1, kSpeed));
  JogRequest written{};
  ASSERT_TRUE(c.h.server().read_jog(written));
  std::optional<Response> jog;
  for (int step = 0; step < 30 && !jog; ++step) {
    c.h.step();
    Response r{};
    while (c.client->poll_response(r)) {
      if (r.type == CommandType::kJog) {
        jog = r;
      }
    }
  }
  ASSERT_TRUE(jog) << "no jog response";
  EXPECT_EQ(jog->command_id, written.refresh);
  EXPECT_EQ(jog->accepted, 0U);
  EXPECT_TRUE(contains(text_of(*jog), "soft limit")) << text_of(*jog);
  EXPECT_EQ(mode_of(c.h.status()), LoopMode::kStandby);
}

// ---------------------------------------------------------------------------- authority, safety

TEST(BridgeService, HeartbeatLossStopsTheMoveThenGivesAuthorityBack) {
  Claimed c;
  ASSERT_TRUE(c.ok());
  const auto r = start_move_joint(c);
  ASSERT_TRUE(r);
  ASSERT_EQ(r->accepted, 1U);
  // Last beat at a 50 ms boundary about 0.8 s into the 2.52 s move.
  ASSERT_GE(c.h.run_until(
                [&] {
                  return c.h.now_ns() >= r->t_start_ns + 800'000'000 &&
                         c.h.now_ns() % kHeartbeatPeriodNs == 0;
                },
                1000),
            0);
  const std::int64_t t_b = c.h.now_ns();  // the harness beat at this time
  c.h.stop_beating();
  std::int64_t t_cleared = -1;
  const int limit = 2000;
  for (int k = 0; k < limit && t_cleared < 0; ++k) {
    c.h.step();
    if (c.h.server().owner_token() == 0) {
      t_cleared = c.h.now_ns();
    }
  }
  const std::int64_t t_standby = standby_after(c.h, r->t_start_ns);
  ASSERT_GE(t_standby, 0) << "never back in standby";
  // Polls every 5 ms; the owner is lost once its heartbeat is older than 200 ms, i.e. at the poll
  // t_b + 205 ms; the stop applies at the next step and ramps over stop_time_s = 0.5 s.
  EXPECT_LE(t_standby, t_b + 205'000'000 + 500'000'000 + 2'000'000);
  // Not stopped before the heartbeat was lost (the ramp alone takes >= 490 steps).
  EXPECT_GE(t_standby, t_b + 200'000'000 + 490'000'000);
  EXPECT_LT(t_standby, r->t_start_ns + static_cast<std::int64_t>(r->duration_s * 1e9))
      << "the move must have been cut short";
  // Authority is given back only once the loop is in standby, then at the next poll.
  ASSERT_GE(t_cleared, 0) << "owner never cleared";
  EXPECT_GT(t_cleared, t_standby);
  EXPECT_LE(t_cleared, t_standby + 10'000'000);
  EXPECT_EQ(c.h.status().manual_stop, manual_stop_value(ManualStop::kStopped));
  EXPECT_FALSE(c.client->is_owner());
  c.h.steps(10);
  EXPECT_FALSE(c.h.service().studio_in_control());
}

TEST(BridgeService, ReleaseWhileMovingStopsAtTheNextPoll) {
  Claimed c;
  ASSERT_TRUE(c.ok());
  const auto r = start_move_joint(c);
  ASSERT_TRUE(r);
  ASSERT_EQ(r->accepted, 1U);
  c.h.steps(500);
  ASSERT_EQ(mode_of(c.h.status()), LoopMode::kTracking);
  const std::int64_t t_release = c.h.now_ns();
  c.client->release();
  c.h.stop_beating();
  ASSERT_GE(c.h.run_until([&] { return in_standby(c.h); }, 1000), 0);
  const std::int64_t t_standby = standby_after(c.h, r->t_start_ns);
  // Stop posted at the next poll (<= 5 ms), applied at the next step, ramp 0.5 s, 2 steps margin.
  EXPECT_LE(t_standby, t_release + 5'000'000 + 1'000'000 + 500'000'000 + 2'000'000);
  EXPECT_EQ(c.h.status().manual_stop, manual_stop_value(ManualStop::kStopped));
  c.h.steps(10);
  EXPECT_FALSE(c.h.service().studio_in_control());
}

TEST(BridgeService, ACommandWithAStaleTokenIsRejectedNotTheOwner) {
  // The first owner's command is in the ring when a second client takes over: the service sees a
  // token that is not the current owner's (the stale-owner case of plan risk "SPSC command ring").
  Claimed c;
  ASSERT_TRUE(c.ok());
  const Vec goal = move_joint_goal(vec_of(c.h.status().joint_reference));
  const auto id = c.client->send(move_joint_command(goal, kSpeed));
  ASSERT_TRUE(id.is_ok()) << id.reason();
  c.client->release();
  auto second = open_client(c.h.name());
  ASSERT_TRUE(second);
  ASSERT_TRUE(second->claim(c.h.now_ns()).is_ok());
  c.h.stop_beating();
  c.h.beat_with(&*second);
  const auto r = c.h.await(*second, id.value(), 20);
  ASSERT_TRUE(r);
  EXPECT_EQ(r->accepted, 0U);
  EXPECT_TRUE(contains(text_of(*r), "not the owner")) << text_of(*r);
  c.h.steps(50);
  EXPECT_EQ(mode_of(c.h.status()), LoopMode::kStandby);
}

TEST(BridgeService, ConfigModeRejectsMotionButExports) {
  HarnessOptions options;
  options.source = pipeline::CommandSource::kConfig;
  Claimed c(options);
  ASSERT_TRUE(c.ok());
  const Vec goal = move_joint_goal(vec_of(c.h.status().joint_reference));
  const auto r = send_and_await(c.h, *c.client, move_joint_command(goal, kSpeed));
  ASSERT_TRUE(r);
  EXPECT_EQ(r->accepted, 0U);
  EXPECT_TRUE(contains(text_of(*r), "command_source is config")) << text_of(*r);

  c.h.steps(100);
  const auto e = send_and_await(c.h, *c.client, export_command(0, c.h.now_ns()));
  ASSERT_TRUE(e);
  EXPECT_EQ(e->type, CommandType::kExportSegment);
  EXPECT_EQ(e->accepted, 1U) << text_of(*e);
  EXPECT_GT(e->rows, 0U);
  EXPECT_TRUE(fs::exists(text_of(e->out_path, kPathChars)));
}

TEST(BridgeService, EstopHoldsAndResetReturnsToStandby) {
  Claimed c;
  ASSERT_TRUE(c.ok());
  const auto e = send_and_await(c.h, *c.client, simple_command(CommandType::kEstop));
  ASSERT_TRUE(e);
  EXPECT_EQ(e->type, CommandType::kEstop);
  EXPECT_EQ(e->accepted, 1U) << text_of(*e);
  c.h.steps(5);
  const Status held = c.h.status();
  ASSERT_EQ(mode_of(held), LoopMode::kHolding);
  EXPECT_STREQ(pipeline::to_string(static_cast<pipeline::HoldReason>(held.hold_reason)), "estop");
  const Record& row = c.h.rows().back();
  EXPECT_EQ(row.mode, LoopMode::kHolding);
  EXPECT_STREQ(pipeline::to_string(row.hold_reason), "estop");

  const auto reset = send_and_await(c.h, *c.client, simple_command(CommandType::kReset));
  ASSERT_TRUE(reset);
  EXPECT_EQ(reset->type, CommandType::kReset);
  EXPECT_EQ(reset->accepted, 1U) << text_of(*reset);
  EXPECT_GE(c.h.run_until([&] { return in_standby(c.h); }, 20), 0);
}

TEST(BridgeService, EveryCommandIsAnsweredOnce) {
  Claimed c;
  ASSERT_TRUE(c.ok());
  std::vector<std::uint64_t> ids;
  for (int k = 0; k < 3; ++k) {
    const auto id = c.client->send(simple_command(CommandType::kStop));
    ASSERT_TRUE(id.is_ok()) << id.reason();
    ids.push_back(id.value());
  }
  std::multiset<std::uint64_t> answered;
  for (int k = 0; k < 100; ++k) {
    c.h.step();
    Response r{};
    while (c.client->poll_response(r)) {
      EXPECT_EQ(r.type, CommandType::kStop);
      answered.insert(r.command_id);
    }
  }
  ASSERT_EQ(answered.size(), ids.size());
  for (const std::uint64_t id : ids) {
    EXPECT_EQ(answered.count(id), 1U) << "command " << id;
  }
  EXPECT_EQ(c.h.server().dropped_responses(), 0U);
}

TEST(BridgeService, StudioInControlFollowsTheOwnerToken) {
  Harness h;
  ASSERT_TRUE(h.ok());
  h.steps(10);
  EXPECT_FALSE(h.service().studio_in_control());
  auto client = open_client(h.name());
  ASSERT_TRUE(client);
  ASSERT_TRUE(client->claim(h.now_ns()).is_ok());
  h.beat_with(&*client);
  h.steps(5);
  EXPECT_TRUE(h.service().studio_in_control());
  client->release();
  h.stop_beating();
  h.steps(5);
  EXPECT_FALSE(h.service().studio_in_control());
}

// ------------------------------------------------------------------------------- segment export

TEST(BridgeService, ExportAfterAMoveCopiesTheRunCsvRows) {
  Claimed c;
  ASSERT_TRUE(c.ok());
  const auto r = start_move_joint(c);
  ASSERT_TRUE(r);
  ASSERT_EQ(r->accepted, 1U);
  ASSERT_GE(c.h.run_until([&] { return in_standby(c.h); }, 4000), 0);
  c.h.steps(300);
  const std::int64_t t0 = r->t_start_ns;
  const std::int64_t t1 = standby_after(c.h, t0) + 200'000'000;
  ASSERT_LT(t1, c.h.now_ns());
  const auto e = send_and_await(c.h, *c.client, export_command(t0, t1));
  ASSERT_TRUE(e);
  ASSERT_EQ(e->accepted, 1U) << text_of(*e);
  EXPECT_EQ(e->truncated, 0U);
  const fs::path out = text_of(e->out_path, kPathChars);
  ASSERT_TRUE(fs::exists(out)) << out;
  // One row per step in [t0, t1].
  const std::int64_t steps = (t1 - t0) / kStepNs + 1;
  EXPECT_EQ(e->rows, static_cast<std::uint64_t>(steps));

  const std::vector<std::string> run = read_lines(c.h.run_csv());
  const std::vector<std::string> seg = read_lines(out);
  ASSERT_FALSE(run.empty());
  ASSERT_FALSE(seg.empty());
  EXPECT_EQ(seg.front(), run.front()) << "header";
  std::vector<std::string> want;
  for (std::size_t i = 1; i < run.size(); ++i) {
    const std::int64_t t = t_of_line(run[i]);
    if (t >= t0 && t <= t1) {
      want.push_back(run[i]);
    }
  }
  ASSERT_EQ(want.size(), static_cast<std::size_t>(steps));
  ASSERT_EQ(seg.size(), want.size() + 1);
  for (std::size_t i = 0; i < want.size(); ++i) {
    ASSERT_EQ(seg[i + 1], want[i]) << "row " << i;
  }
}

TEST(BridgeService, ExportOfRowsOverwrittenBeforeAnyExportIsTruncated) {
  // A 0.5 s Recorder: the 2.5 s move overwrites its first rows before the segment export.
  HarnessOptions options;
  options.log_rows = 500;
  Claimed c(options);
  ASSERT_TRUE(c.ok());
  const auto r = start_move_joint(c);
  ASSERT_TRUE(r);
  ASSERT_EQ(r->accepted, 1U);
  ASSERT_GE(c.h.run_until([&] { return in_standby(c.h); }, 4000), 0);
  const std::int64_t t0 = r->t_start_ns;
  const std::int64_t t1 = c.h.now_ns() - kStepNs;
  const auto e = send_and_await(c.h, *c.client, export_command(t0, t1));
  ASSERT_TRUE(e);
  ASSERT_EQ(e->accepted, 1U) << text_of(*e);
  EXPECT_EQ(e->truncated, 1U);
  EXPECT_LT(e->rows, static_cast<std::uint64_t>((t1 - t0) / kStepNs + 1));
  EXPECT_GT(e->rows, 0U);
}

// --------------------------------------------------------------------------------- apply config

TEST(BridgeService, ApplyConfigInStandbySwapsTheLoopWithoutAJump) {
  Claimed c;
  ASSERT_TRUE(c.ok());
  SessionInfo<kN> before{};
  ASSERT_TRUE(c.client->session(before));
  const fs::path premium = repo_path("config/premium_fr3.yaml");
  const auto r = send_and_await(c.h, *c.client, apply_command(premium, ideal_profile()));
  ASSERT_TRUE(r);
  EXPECT_EQ(r->type, CommandType::kApplyConfig);
  ASSERT_EQ(r->accepted, 1U) << text_of(*r);
  EXPECT_TRUE(contains(text_of(*r), "applied")) << text_of(*r);
  c.h.steps(20);
  ASSERT_EQ(c.h.swap_rows().size(), 1U);

  SessionInfo<kN> after{};
  ASSERT_TRUE(c.client->session(after));
  EXPECT_EQ(after.generation, before.generation + 1);
  EXPECT_EQ(text_of(after.config_path, kPathChars), premium.string());
  EXPECT_EQ(text_of(after.imperfections_path, kPathChars), ideal_profile().string());

  // The first row of the new loop: standby at the measured pose (CLIK reset there).
  const Record& first = c.h.rows()[c.h.swap_rows().front()];
  EXPECT_EQ(first.mode, LoopMode::kStandby);
  // One CLIK step of null-space motion moves q_ref by at most qdd dt^2 (dt = 1 ms).
  EXPECT_LT((first.q_ref - first.q).cwiseAbs().maxCoeff(), 1e-5);
  ASSERT_TRUE(c.h.applied_arm().has_value());
  const common::Pose fk = fk_tip(c.h.applied_arm()->controller, first.q);
  EXPECT_LT((first.T_b_d.p - fk.p).norm(), 1e-9);
  EXPECT_LT((first.T_b_d.R - fk.R).cwiseAbs().maxCoeff(), 1e-9);

  // The Run CSV continues across the swap: one row per ms, no gap, strictly increasing.
  const auto e = send_and_await(c.h, *c.client, export_command(0, c.h.now_ns()));
  ASSERT_TRUE(e);
  ASSERT_EQ(e->accepted, 1U) << text_of(*e);
  const std::vector<std::string> run = read_lines(c.h.run_csv());
  ASSERT_GT(run.size(), c.h.swap_rows().front() + 2);
  for (std::size_t i = 2; i < run.size(); ++i) {
    ASSERT_EQ(t_of_line(run[i]) - t_of_line(run[i - 1]), kStepNs) << "line " << i;
  }
  // A move with the new model works.
  const auto m = start_move_joint(c);
  ASSERT_TRUE(m);
  EXPECT_EQ(m->accepted, 1U);
}

TEST(BridgeService, ApplyConfigWhileTrackingIsRejectedNotInStandby) {
  Claimed c;
  ASSERT_TRUE(c.ok());
  c.client->write_jog(joint_jog(0, +1, kSpeed));
  c.h.steps(20);
  ASSERT_EQ(mode_of(c.h.status()), LoopMode::kTracking);
  c.client->write_jog(joint_jog(0, +1, kSpeed));
  const auto r = send_and_await(
      c.h, *c.client, apply_command(repo_path("config/premium_fr3.yaml"), ideal_profile()), 15);
  ASSERT_TRUE(r);
  EXPECT_EQ(r->accepted, 0U);
  EXPECT_TRUE(contains(text_of(*r), "not in standby")) << text_of(*r);
  EXPECT_EQ(c.h.applies(), 0) << "the hook must not run outside standby";
  c.client->write_jog(joint_jog(0, 0, kSpeed));
  EXPECT_GE(c.h.run_until([&] { return in_standby(c.h); }, 500), 0);
  EXPECT_TRUE(c.h.swap_rows().empty());
}

TEST(BridgeService, ApplyConfigWithAnotherControlRateIsRejectedNamingIt) {
  Claimed c;
  ASSERT_TRUE(c.ok());
  // default.yaml at 500 Hz, its URDF path made absolute.
  std::string text = read_file(default_config());
  const std::string rate = "control_rate_hz: 1000.0";
  const std::string urdf = "urdf_path: ../src/sim/urdf/cobot30_5.urdf";
  ASSERT_NE(text.find(rate), std::string::npos);
  ASSERT_NE(text.find(urdf), std::string::npos);
  text.replace(text.find(rate), rate.size(), "control_rate_hz: 500.0");
  text.replace(text.find(urdf),
               urdf.size(),
               "urdf_path: " + repo_path("src/sim/urdf/cobot30_5.urdf").string());
  const fs::path yaml = c.h.dir() / "rate_500.yaml";
  {
    std::ofstream out(yaml, std::ios::binary);
    out << text;
  }
  SessionInfo<kN> before{};
  ASSERT_TRUE(c.client->session(before));
  const auto r = send_and_await(c.h, *c.client, apply_command(yaml, ideal_profile()));
  ASSERT_TRUE(r);
  EXPECT_EQ(r->accepted, 0U);
  EXPECT_TRUE(contains(text_of(*r), "control_rate_hz")) << text_of(*r);
  SessionInfo<kN> after{};
  ASSERT_TRUE(c.client->session(after));
  EXPECT_EQ(after.generation, before.generation);
  c.h.steps(20);
  EXPECT_TRUE(c.h.swap_rows().empty());
}

}  // namespace
}  // namespace cobot::bridge::test
