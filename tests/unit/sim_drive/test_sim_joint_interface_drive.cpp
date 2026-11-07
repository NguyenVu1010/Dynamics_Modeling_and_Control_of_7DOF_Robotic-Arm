// SimJointInterface with an attached SimDrive (brief drive-loop 6): attach_drive(drive) is
// non-owning (nullptr = no drive). advance() then calls drive->advance(plant, last command, dt)
// instead of plant.step(last command.tau, dt), for the nominal period and for each overrun period.
// Without a drive, write() of a position command returns
// "sim joint interface: position command without a drive" and keeps the previous command.
#include <string>

#include <gtest/gtest.h>

#include "cobot/pipeline/sim_joint_interface.hpp"

#include "sim_drive_util.hpp"

namespace cobot::pipeline::drive_test {
namespace {

Drive make_drive(const Arm& arm) {
  return Drive(drive_config(), arm.inertia, arm.tau_max, Profile{}, Vec::Ones(), kDt, kDt);
}

TEST(SimJointInterfaceDrive, PositionCommandWithoutADriveIsRejected) {
  const auto arm = load_arm();
  ASSERT_TRUE(arm);
  Plant plant(arm->model, arm->home);
  SimJointInterface<kN> io(plant, kDt);
  const Vec c = Vec::Constant(0.25);
  ASSERT_TRUE(io.write(torque_command(c)).is_ok());
  const common::Status st = io.write(position_command(arm->home, Vec::Zero(), Vec::Zero(), 1));
  ASSERT_FALSE(st.is_ok());
  EXPECT_EQ(st.reason(), std::string("sim joint interface: position command without a drive"));
  io.advance();
  for (int j = 0; j < kN; ++j) {
    EXPECT_EQ(plant.tau_applied()(j), c(j)) << "joint " << j << ": previous command kept";
  }
}

TEST(SimJointInterfaceDrive, AttachedDriveRunsEveryPeriod) {
  const auto arm = load_arm();
  ASSERT_TRUE(arm);
  Plant plant(arm->model, arm->home);
  SimJointInterface<kN> io(plant, kDt);
  Drive drive = make_drive(*arm);
  io.attach_drive(&drive);
  ASSERT_TRUE(io.write(position_command(arm->home, Vec::Zero(), Vec::Zero(), 1)).is_ok());
  for (int k = 0; k < 3; ++k) {
    io.advance();
  }
  EXPECT_EQ(drive.substeps_run(), 12);  // 3 periods * 4 sub-steps
  EXPECT_NEAR(plant.time_s(), 3e-3, 1e-15);

  // A torque command through the attached drive reaches the plant unchanged.
  const Vec c = Vec::Constant(-0.125);
  ASSERT_TRUE(io.write(torque_command(c)).is_ok());
  io.advance();
  EXPECT_EQ(plant.tau_applied(), c);
  EXPECT_EQ(drive.substeps_run(), 16);

  // Detached again: the interface behaves as without a drive.
  io.attach_drive(nullptr);
  EXPECT_FALSE(io.write(position_command(arm->home, Vec::Zero(), Vec::Zero(), 2)).is_ok());
  io.advance();
  EXPECT_EQ(drive.substeps_run(), 16) << "a detached drive is not called";
  EXPECT_EQ(plant.tau_applied(), c);
}

// Overrun with probability 1 and 2 extra cycles: every advance is 1 + 2 periods, each through the
// drive: 3 * 4 = 12 sub-steps and 3 ms of plant time per advance.
TEST(SimJointInterfaceDrive, OverrunPeriodsGoThroughTheDrive) {
  const auto arm = load_arm();
  ASSERT_TRUE(arm);
  Plant plant(arm->model, arm->home);
  SimJointInterface<kN> io(plant, kDt, 0.0, 1, 1.0, 2);
  Drive drive = make_drive(*arm);
  io.attach_drive(&drive);
  ASSERT_TRUE(io.write(position_command(arm->home, Vec::Zero(), Vec::Zero(), 1)).is_ok());
  io.advance();
  io.advance();
  EXPECT_EQ(drive.substeps_run(), 24);
  EXPECT_NEAR(plant.time_s(), 6e-3, 1e-15);
}

// A jittered period reaches the drive as its dt: the plant clock moves by the same total as without
// a drive (same seed, same jitter draws).
TEST(SimJointInterfaceDrive, JitteredPeriodsReachTheDrive) {
  const auto arm = load_arm();
  ASSERT_TRUE(arm);
  Plant with(arm->model, arm->home);
  Plant without(arm->model, arm->home);
  SimJointInterface<kN> io_with(with, kDt, 1e-4, 3);
  SimJointInterface<kN> io_without(without, kDt, 1e-4, 3);
  Drive drive = make_drive(*arm);
  io_with.attach_drive(&drive);
  ASSERT_TRUE(io_with.write(position_command(arm->home, Vec::Zero(), Vec::Zero(), 1)).is_ok());
  for (int k = 0; k < 20; ++k) {
    io_with.advance();
    io_without.advance();
    EXPECT_NEAR(with.time_s(), without.time_s(), 1e-15) << "advance " << k;
  }
  EXPECT_EQ(drive.substeps_run(), 80);
}

}  // namespace
}  // namespace cobot::pipeline::drive_test
