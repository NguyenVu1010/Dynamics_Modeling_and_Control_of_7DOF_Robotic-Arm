// Task loader (brief hardware-catalog 3, ADR-023, ADR-009 strict keys): config/tasks/<task>.yaml ->
// pipeline::Task. Errors name the file and the full key. Exactly one of trajectory and waypoints;
// the waypoint file must exist, its parsing stays with the run. Every requirement > 0, rates in
// (0, 1], margin_frac in (0, 1); target_pass_rate defaults to 0.95, margin_frac to 0.2. payload is
// optional (0 kg, known) and known defaults to true (owner question C9).
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/task.hpp"
#include "cobot/pipeline/waypoint_file.hpp"

#include "hw_test_util.hpp"
#include "task_fixture.hpp"

namespace cobot::pipeline {
namespace {

using namespace hw_test;
using common::Vec3;

void expect_task_error(const fs::path& path, const std::vector<std::string>& parts) {
  const auto r = load_task(path);
  ASSERT_FALSE(r.is_ok()) << path;
  EXPECT_TRUE(contains(r.reason(), path.filename().string())) << r.reason();
  for (const std::string& part : parts) {
    EXPECT_TRUE(contains(r.reason(), part)) << "missing '" << part << "' in: " << r.reason();
  }
}

TEST(TaskLoader, FixtureLoadsWithItsFields) {
  const auto r = load_task(write_task(task_fixture_text()));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const Task& t = r.value();
  EXPECT_EQ(t.name, "fixture_task");
  ASSERT_TRUE(t.trajectory.has_value());
  EXPECT_TRUE(t.trajectory->goal_position_m == Vec3(-0.4, -0.6, 0.8));
  EXPECT_EQ(t.trajectory->duration_s, 7.0);
  EXPECT_EQ(t.trajectory->hold_s, 1.0);
  EXPECT_FALSE(t.waypoints.has_value());
  EXPECT_EQ(t.payload.kg, 2.0);
  EXPECT_TRUE(t.payload.offset_m == Vec3(0.0, 0.0, 0.1));
  EXPECT_EQ(t.payload.radius_m, 0.03);
  EXPECT_TRUE(t.payload.known);
  EXPECT_EQ(t.requirements.final_pos_err_m, 0.005);
  EXPECT_EQ(t.requirements.final_rot_err_deg, 2.0);
  EXPECT_EQ(t.requirements.rms_joint_err_rad, 0.02);
  EXPECT_EQ(t.requirements.cycle_time_s, 12.0);
  EXPECT_EQ(t.requirements.max_hold_rate, 0.05);
  EXPECT_EQ(t.requirements.target_pass_rate, 0.9);
  EXPECT_EQ(t.requirements.margin_frac, 0.3);
}

TEST(TaskLoader, EachMissingRequiredKeyIsNamed) {
  struct Case {
    const char* line;
    const char* key;
  };
  const std::vector<Case> cases{
      {"  name:", "task.name"},
      {"    goal_position_m:", "task.trajectory.goal_position_m"},
      {"    goal_rpy_deg:", "task.trajectory.goal_rpy_deg"},
      {"    duration_s:", "task.trajectory.duration_s"},
      {"    hold_s:", "task.trajectory.hold_s"},
      {"  requirements:", "task.requirements"},
      {"    final_pos_err_m:", "task.requirements.final_pos_err_m"},
      {"    final_rot_err_deg:", "task.requirements.final_rot_err_deg"},
      {"    rms_joint_err_rad:", "task.requirements.rms_joint_err_rad"},
      {"    cycle_time_s:", "task.requirements.cycle_time_s"},
      {"    max_hold_rate:", "task.requirements.max_hold_rate"},
  };
  int n = 0;
  for (const Case& c : cases) {
    SCOPED_TRACE(c.key);
    expect_task_error(
        write_task(drop_block(task_fixture_text(), c.line), "case" + std::to_string(n++)), {c.key});
  }
  expect_task_error(write_task("other: 1\n", "section"), {"task"});
}

TEST(TaskLoader, UnknownKeyAtEachLevelIsNamed) {
  const std::string text = task_fixture_text();
  expect_task_error(write_task(text + "bogus_key: 1\n", "root"), {"bogus_key"});
  expect_task_error(write_task(insert_after(text, "  name:", "  bogus_key: 1"), "task"),
                    {"task.bogus_key"});
  expect_task_error(
      write_task(insert_after(text, "  trajectory:", "    bogus_key: 1"), "trajectory"),
      {"task.trajectory.bogus_key"});
  expect_task_error(write_task(insert_after(text, "  payload:", "    bogus_key: 1"), "payload"),
                    {"task.payload.bogus_key"});
  expect_task_error(
      write_task(insert_after(text, "  requirements:", "    bogus_key: 1"), "requirements"),
      {"task.requirements.bogus_key"});
}

TEST(TaskLoader, BadValuesAreNamed) {
  struct Case {
    const char* line;
    const char* replacement;
    const char* key;
  };
  const std::vector<Case> cases{
      {"  name:", "  name: [a, b]", "task.name"},
      {"  trajectory:", "  trajectory: 5", "task.trajectory"},
      {"    goal_position_m:",
       "    goal_position_m: [0.0, 0.0]",
       "task.trajectory.goal_position_m"},
      {"    goal_rpy_deg:", "    goal_rpy_deg: [a, 0.0, 0.0]", "task.trajectory.goal_rpy_deg"},
      {"    duration_s:", "    duration_s: 0.0", "task.trajectory.duration_s"},
      {"    duration_s:", "    duration_s: abc", "task.trajectory.duration_s"},
      {"    hold_s:", "    hold_s: -1.0", "task.trajectory.hold_s"},
      {"  payload:", "  payload: 5", "task.payload"},
      {"    kg:", "    kg: -1.0", "task.payload.kg"},
      {"    kg:", "    kg: .nan", "task.payload.kg"},
      {"    offset_m:", "    offset_m: [0.0, 0.0]", "task.payload.offset_m"},
      {"    offset_m:", "    offset_m: [0.0, .inf, 0.0]", "task.payload.offset_m"},
      {"    radius_m:", "    radius_m: -0.01", "task.payload.radius_m"},
      {"    known:", "    known: maybe", "task.payload.known"},
      {"  requirements:", "  requirements: 5", "task.requirements"},
      {"    final_pos_err_m:", "    final_pos_err_m: abc", "task.requirements.final_pos_err_m"},
  };
  int n = 0;
  for (const Case& c : cases) {
    SCOPED_TRACE(std::string(c.replacement));
    expect_task_error(write_task(set_block(task_fixture_text(), c.line, c.replacement),
                                 "case" + std::to_string(n++)),
                      {c.key});
  }
}

TEST(TaskLoader, MissingFileIsAnErrorNamingIt) {
  const auto r = load_task(test_dir() / "no_such_task.yaml");
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(r.reason(), "no_such_task.yaml")) << r.reason();
}

TEST(TaskLoader, TrajectoryAndWaypointsTogetherAreAnError) {
  write_text(test_dir() / "base" / "list.yaml", "frame: base_link\nwaypoints: []\n");
  const std::string text = insert_after(task_fixture_text(), "  name:", "  waypoints: list.yaml");
  expect_task_error(write_task(text), {"task.trajectory", "task.waypoints"});
}

TEST(TaskLoader, NeitherTrajectoryNorWaypointsIsAnError) {
  expect_task_error(write_task(drop_block(task_fixture_text(), "  trajectory:")),
                    {"task.trajectory", "task.waypoints"});
}

TEST(TaskLoader, WaypointFileMustExist) {
  expect_task_error(write_task(task_waypoints_text("no_such_list.yaml")),
                    {"task.waypoints", "no_such_list.yaml"});
}

// Existence only: load_waypoints needs the base link and the TCP, so the run parses the list.
TEST(TaskLoader, WaypointFileIsNotParsedHere) {
  write_text(test_dir() / "base" / "list.yaml", "this is not a waypoint file\n");
  const auto r = load_task(write_task(task_waypoints_text("list.yaml")));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_FALSE(r.value().trajectory.has_value());
  ASSERT_TRUE(r.value().waypoints.has_value());
  EXPECT_TRUE(same_path(*r.value().waypoints, test_dir() / "base" / "list.yaml"));
}

TEST(TaskLoader, EveryRequirementMustBePositive) {
  const std::vector<std::string> keys{"final_pos_err_m",
                                      "final_rot_err_deg",
                                      "rms_joint_err_rad",
                                      "cycle_time_s",
                                      "max_hold_rate",
                                      "target_pass_rate",
                                      "margin_frac"};
  int n = 0;
  for (const std::string& key : keys) {
    for (const char* value : {"0.0", "-1.0"}) {
      SCOPED_TRACE(key + " " + value);
      expect_task_error(
          write_task(set_line(task_fixture_text(), "    " + key + ":", "    " + key + ": " + value),
                     "case" + std::to_string(n++)),
          {"task.requirements." + key});
    }
  }
}

TEST(TaskLoader, RatesAboveOneAreErrorsAndOneLoads) {
  int n = 0;
  for (const std::string key : {"max_hold_rate", "target_pass_rate"}) {
    SCOPED_TRACE(key);
    const std::string line = "    " + key + ":";
    expect_task_error(
        write_task(set_line(task_fixture_text(), line, line + " 1.5"), "high" + std::to_string(n)),
        {"task.requirements." + key});
    const auto r = load_task(
        write_task(set_line(task_fixture_text(), line, line + " 1.0"), "one" + std::to_string(n)));
    EXPECT_TRUE(r.is_ok()) << r.reason();
    ++n;
  }
}

TEST(TaskLoader, MarginFracIsInsideTheOpenInterval) {
  const std::string line = "    margin_frac:";
  expect_task_error(write_task(set_line(task_fixture_text(), line, line + " 1.0"), "one"),
                    {"task.requirements.margin_frac"});
  expect_task_error(write_task(set_line(task_fixture_text(), line, line + " 1.5"), "above"),
                    {"task.requirements.margin_frac"});
  const auto r = load_task(write_task(set_line(task_fixture_text(), line, line + " 0.5"), "ok"));
  EXPECT_TRUE(r.is_ok()) << r.reason();
}

TEST(TaskLoader, OptionalRequirementsHaveTheirDefaults) {
  std::string text = drop_block(task_fixture_text(), "    target_pass_rate:");
  text = drop_block(text, "    margin_frac:");
  const auto r = load_task(write_task(text));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().requirements.target_pass_rate, 0.95);
  EXPECT_EQ(r.value().requirements.margin_frac, 0.2);
}

TEST(TaskLoader, PayloadAbsentIsZeroKgKnown) {
  const auto r = load_task(write_task(drop_block(task_fixture_text(), "  payload:")));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().payload.kg, 0.0);
  EXPECT_TRUE(r.value().payload.offset_m == Vec3::Zero());
  EXPECT_EQ(r.value().payload.radius_m, 0.0);
  EXPECT_TRUE(r.value().payload.known);
}

TEST(TaskLoader, PayloadKnownDefaultsToTrueAndFalseIsKept) {
  const auto dflt = load_task(write_task(drop_block(task_fixture_text(), "    known:"), "default"));
  ASSERT_TRUE(dflt.is_ok()) << dflt.reason();
  EXPECT_TRUE(dflt.value().payload.known);
  const auto no = load_task(
      write_task(set_line(task_fixture_text(), "    known:", "    known: false"), "false"));
  ASSERT_TRUE(no.is_ok()) << no.reason();
  EXPECT_FALSE(no.value().payload.known);
}

// --- the shipped tasks ---

fs::path task_file(const char* name) {
  return kRepo / "config/tasks" / (std::string(name) + ".yaml");
}

void expect_e2e12_levels(const Task& t) {
  EXPECT_EQ(t.requirements.final_pos_err_m, 0.005);
  EXPECT_EQ(t.requirements.final_rot_err_deg, 2.0);
  EXPECT_EQ(t.requirements.rms_joint_err_rad, 0.02);
}

TEST(TaskFiles, ReferenceMoveIsTheE2e01Trajectory) {
  const auto r = load_task(task_file("reference_move"));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const Task& t = r.value();
  EXPECT_EQ(t.name, "reference_move");
  ASSERT_TRUE(t.trajectory.has_value());
  EXPECT_FALSE(t.waypoints.has_value());
  // config/default.yaml cobot.trajectory.
  EXPECT_TRUE(t.trajectory->goal_position_m == Vec3(-0.5, -0.6, 0.9));
  EXPECT_EQ(t.trajectory->duration_s, 10.0);
  EXPECT_EQ(t.trajectory->hold_s, 2.0);
  EXPECT_EQ(t.payload.kg, 0.0);
  EXPECT_TRUE(t.payload.known);
  expect_e2e12_levels(t);
  EXPECT_EQ(t.requirements.cycle_time_s, 12.0);
  EXPECT_EQ(t.requirements.max_hold_rate, 0.05);
  EXPECT_EQ(t.requirements.target_pass_rate, 0.95);
  EXPECT_EQ(t.requirements.margin_frac, 0.2);
}

TEST(TaskFiles, PickPlaceCarriesThreeKgOnItsWaypoints) {
  const auto r = load_task(task_file("pick_place_3kg"));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const Task& t = r.value();
  EXPECT_EQ(t.name, "pick_place_3kg");
  EXPECT_FALSE(t.trajectory.has_value());
  ASSERT_TRUE(t.waypoints.has_value());
  EXPECT_TRUE(same_path(*t.waypoints, kRepo / "config/tasks/pick_place_waypoints.yaml"));
  EXPECT_EQ(t.payload.kg, 3.0);
  EXPECT_TRUE(t.payload.offset_m == Vec3(0.0, 0.0, 0.05));
  EXPECT_EQ(t.payload.radius_m, 0.05);
  EXPECT_TRUE(t.payload.known);
  expect_e2e12_levels(t);
}

TEST(TaskFiles, PickPlaceWaypointsParse) {
  const auto c = load_config<kN>(kRepo / "config/premium_fr3.yaml");
  ASSERT_TRUE(c.is_ok()) << c.reason();
  const auto w = load_waypoints(
      kRepo / "config/tasks/pick_place_waypoints.yaml", c.value().base_link, c.value().tcp_offset);
  ASSERT_TRUE(w.is_ok()) << w.reason();
  EXPECT_FALSE(w.value().empty());
}

// C13: the thresholds are a proposal; a comment at the top of each new file.
TEST(TaskFiles, CommentsSayTheThresholdsAreProposed) {
  for (const char* name : {"reference_move", "pick_place_3kg"}) {
    SCOPED_TRACE(name);
    const std::string text = read_text(task_file(name));
    EXPECT_EQ(text.rfind('#', 0), 0U) << "a comment at the top of the file";
    EXPECT_TRUE(contains(text, "proposed, awaiting owner approval"));
  }
  EXPECT_EQ(read_text(kRepo / "config/tasks/pick_place_waypoints.yaml").rfind('#', 0), 0U);
}

}  // namespace
}  // namespace cobot::pipeline
