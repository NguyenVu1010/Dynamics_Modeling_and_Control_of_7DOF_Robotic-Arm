#include "cobot/pipeline/task.hpp"

#include <cmath>
#include <string>
#include <system_error>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "yaml_reader.hpp"

namespace cobot::pipeline {
namespace {

namespace fs = std::filesystem;
using common::Vec3;
using yaml_io::kAny;
using yaml_io::kNonNegative;
using yaml_io::kPositive;
using yaml_io::kUnitInterval;
using yaml_io::Reader;
using yaml_io::Rule;

const Rule kOpenUnit{[](double v) { return v > 0.0 && v < 1.0; }, "must be in (0, 1)"};

std::vector<double> to_list(const Vec3& v) {
  return {v.x(), v.y(), v.z()};
}

void read_motion(Reader& t, const fs::path& dir, Task& task) {
  const bool has_trajectory = t.has("trajectory");
  const bool has_waypoints = t.has("waypoints");
  if (has_trajectory == has_waypoints) {
    t.fail("trajectory", "give exactly one of task.trajectory and task.waypoints");
  }
  if (has_trajectory) {
    Reader x = t.map("trajectory");
    TaskTrajectory trajectory;
    x.vector<3>("goal_position_m", kAny, false, trajectory.goal_position_m);
    x.vector<3>("goal_rpy_deg", kAny, false, trajectory.goal_rpy_deg);
    x.number("duration_s", kPositive, trajectory.duration_s);
    x.number("hold_s", kNonNegative, trajectory.hold_s);
    x.finish();
    task.trajectory = trajectory;
  }
  std::string file;
  if (has_waypoints && t.scalar("waypoints", file)) {
    const fs::path path = (dir / file).lexically_normal();
    std::error_code ec;
    if (!fs::is_regular_file(path, ec)) {
      t.fail("waypoints", "cannot open " + path.string() + ": no such file");
    }
    task.waypoints = path;
  }
}

void read_payload(Reader& t, TaskPayload& p) {
  if (!t.has("payload")) {
    return;
  }
  Reader x = t.map("payload");
  x.number("kg", kNonNegative, p.kg);
  if (x.has("offset_m")) {
    x.vector<3>("offset_m", kAny, false, p.offset_m);
  }
  if (x.has("radius_m")) {
    x.number("radius_m", kNonNegative, p.radius_m);
  }
  if (x.has("known")) {
    x.scalar("known", p.known);
  }
  x.finish();
}

void read_requirements(Reader& t, TaskRequirements& q) {
  Reader x = t.map("requirements");
  x.number("final_pos_err_m", kPositive, q.final_pos_err_m);
  x.number("final_rot_err_deg", kPositive, q.final_rot_err_deg);
  x.number("rms_joint_err_rad", kPositive, q.rms_joint_err_rad);
  x.number("cycle_time_s", kPositive, q.cycle_time_s);
  x.number("max_hold_rate", kUnitInterval, q.max_hold_rate);
  if (x.has("target_pass_rate")) {
    x.number("target_pass_rate", kUnitInterval, q.target_pass_rate);
  }
  if (x.has("margin_frac")) {
    x.number("margin_frac", kOpenUnit, q.margin_frac);
  }
  x.finish();
}

}  // namespace

Result<Task> load_task(const fs::path& yaml) {
  std::error_code ec;
  const fs::path dir = fs::absolute(yaml, ec).parent_path();
  return yaml_io::load<Task>(yaml, [&dir](Reader& root, Task& task) {
    Reader t = root.map("task");
    t.scalar("name", task.name);
    read_motion(t, dir, task);
    read_payload(t, task.payload);
    read_requirements(t, task.requirements);
    t.finish();
  });
}

RunOverrides run_overrides(const HardwareModel& model,
                           control::Algorithm algorithm,
                           const Task& task) {
  RunOverrides out;
  out.config = model_overrides(model, algorithm);
  if (task.trajectory) {
    const TaskTrajectory& t = *task.trajectory;
    out.config.push_back({"config.trajectory.goal_position_m", to_list(t.goal_position_m)});
    out.config.push_back({"config.trajectory.goal_rpy_deg", to_list(t.goal_rpy_deg)});
    out.config.push_back({"config.trajectory.duration_s", t.duration_s});
    out.config.push_back({"config.trajectory.hold_s", t.hold_s});
  }
  const TaskPayload& p = task.payload;
  if (p.known) {
    out.config.push_back({"config.payload.kg", p.kg});
    out.config.push_back({"config.payload.offset_m", to_list(p.offset_m)});
    out.config.push_back({"config.payload.radius_m", p.radius_m});
  }
  out.imperfections.push_back({"model.payload_kg", p.kg});
  out.imperfections.push_back({"model.payload_offset_m", to_list(p.offset_m)});
  out.imperfections.push_back({"model.payload_radius_m", p.radius_m});
  out.waypoints = task.waypoints;
  return out;
}

}  // namespace cobot::pipeline
