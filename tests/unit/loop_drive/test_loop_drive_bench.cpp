// The hosts of drive_position (brief drive-loop 7, owner question C4):
// - the bench run engine (run, montecarlo, sweep through run_in_memory) builds a SimDrive after
//   the decorator and attaches it, so the plant is regulated by the drive;
// - timing() attaches a drive with the ideal profile;
// - cobot_bench serve rejects the algorithm: "cobot.control.algorithm drive_position is bench only
//   (no drive model on this host)" (cobot_controller rejects it with the same text; ros2 only).
#include <atomic>
#include <cmath>
#include <filesystem>
#include <optional>
#include <string>

#include <gtest/gtest.h>

#include "cobot/imperfect/config.hpp"
#include "cobot/pipeline/config.hpp"

#include "run.hpp"
#include "run_data.hpp"
#include "serve.hpp"

namespace cobot::bench {
namespace {

namespace fs = std::filesystem;
const fs::path kRepo = COBOT_REPO_DIR;

std::optional<imperfect::ImperfectionConfig<kN>> profile(const char* name) {
  const auto r = pipeline::load_imperfections<kN>(kRepo / "config/imperfections" /
                                                  (std::string(name) + ".yaml"));
  if (!r) {
    ADD_FAILURE() << name << ": " << r.reason();
    return std::nullopt;
  }
  return r.value();
}

TEST(LoopDriveBench, ServeRejectsDrivePosition) {
  ServeOptions options;
  options.config_yaml = kRepo / "config/budget_xarm7_drive.yaml";
  options.imperfections_yaml = kRepo / "config/imperfections/budget_xarm7.yaml";
  options.name = "cobot_drive_reject_test";
  options.out_dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "loop_drive_serve";
  options.duration_s = 0.05;
  const std::atomic<bool> stop{true};
  const common::Status st = serve(options, stop);
  ASSERT_FALSE(st.is_ok()) << "serve accepted drive_position";
  EXPECT_NE(
      st.reason().find(
          "cobot.control.algorithm drive_position is bench only (no drive model on this host)"),
      std::string::npos)
      << st.reason();
}

TEST(LoopDriveBench, TimingAttachesADrive) {
  const auto t = timing(kRepo / "config/budget_xarm7_drive.yaml", 200);
  ASSERT_TRUE(t.is_ok()) << t.reason();
  EXPECT_TRUE(std::isfinite(t.value().p50_us));
  EXPECT_GT(t.value().p50_us, 0.0);
}

// 0.3 s of the drive config on the ideal profile through run_in_memory. Without a drive the plant
// would get no torque (SimJointInterface rejects the position command) and the arm would fall
// under gravity (joint 2 ~ 0.4 rad in 0.3 s): a hold or a large joint error. With the drive it
// tracks the start of the trajectory closely.
TEST(LoopDriveBench, RunEngineAttachesTheDrive) {
  const auto p = profile("ideal");
  ASSERT_TRUE(p);
  RunRequest request;
  request.config_yaml = kRepo / "config/budget_xarm7_drive.yaml";
  request.imperfections = *p;
  request.label = "ideal";
  request.duration_s = 0.3;
  const auto run = run_in_memory(request);
  ASSERT_TRUE(run.is_ok()) << run.reason();
  const RunData& r = run.value();
  ASSERT_EQ(r.records.size(), 300U);
  EXPECT_FALSE(r.summary.held) << pipeline::to_string(r.summary.hold_reason);
  EXPECT_LT(r.summary.rms_joint_err_rad, 5e-3);
  for (const auto& rec : r.records) {
    ASSERT_TRUE(rec.tau.allFinite());
    ASSERT_TRUE(rec.tau_motor.allFinite());
  }
}

}  // namespace
}  // namespace cobot::bench
