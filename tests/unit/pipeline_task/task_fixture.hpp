#pragma once

#include <string>

#include "hw_test_util.hpp"

// A valid task with every key, and the values its tests expect:
//   trajectory goal (-0.4, -0.6, 0.8) m, rpy (0, 0, 45) deg, 7 s + 1 s hold;
//   payload 2 kg, 0.1 m along tip z, radius 0.03 m, known;
//   requirements as in the brief, with target_pass_rate and margin_frac off their defaults.
namespace cobot::pipeline::hw_test {

inline std::string task_fixture_text() {
  return R"(task:
  name: fixture_task
  trajectory:
    goal_position_m: [-0.4, -0.6, 0.8]
    goal_rpy_deg: [0.0, 0.0, 45.0]
    duration_s: 7.0
    hold_s: 1.0
  payload:
    kg: 2.0
    offset_m: [0.0, 0.0, 0.1]
    radius_m: 0.03
    known: true
  requirements:
    final_pos_err_m: 0.005
    final_rot_err_deg: 2.0
    rms_joint_err_rad: 0.02
    cycle_time_s: 12.0
    max_hold_rate: 0.05
    target_pass_rate: 0.9
    margin_frac: 0.3
)";
}

// The same with `waypoints: <relative>` in place of the trajectory.
inline std::string task_waypoints_text(const std::string& relative) {
  return set_block(task_fixture_text(), "  trajectory:", "  waypoints: " + relative);
}

// Writes a task into a directory of this test (`variant` tells several writes of one test apart).
inline fs::path write_task(const std::string& text, const std::string& variant = "base") {
  return write_text(test_dir() / variant / "fixture_task.yaml", text);
}

}  // namespace cobot::pipeline::hw_test
