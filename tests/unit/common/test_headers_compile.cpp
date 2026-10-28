// Every interface header compiles together and the N = 7 types instantiate.
#include <type_traits>

#include <gtest/gtest.h>

#include "cobot/common/dof.hpp"
#include "cobot/common/lie.hpp"
#include "cobot/common/logger.hpp"
#include "cobot/common/result.hpp"
#include "cobot/common/timestamp.hpp"
#include "cobot/common/types.hpp"
#include "cobot/control/computed_torque.hpp"
#include "cobot/control/hold.hpp"
#include "cobot/dynamics/dynamics.hpp"
#include "cobot/dynamics/sim_plant.hpp"
#include "cobot/hal/joint_interface.hpp"
#include "cobot/imperfect/config.hpp"
#include "cobot/imperfect/imperfect_joint_interface.hpp"
#include "cobot/imperfect/perturb.hpp"
#include "cobot/kinematics/kinematics.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/control_loop.hpp"
#include "cobot/pipeline/log_record.hpp"
#include "cobot/pipeline/sim_joint_interface.hpp"
#include "cobot/pipeline/spsc_ring.hpp"
#include "cobot/planning/cartesian_trajectory.hpp"
#include "cobot/planning/clik.hpp"

namespace {

constexpr int kN = cobot::model::kArmDof;

static_assert(kN == 7);
static_assert(std::is_abstract_v<cobot::hal::JointInterface<kN>>);
static_assert(std::is_base_of_v<cobot::hal::JointInterface<kN>,
                                cobot::imperfect::ImperfectJointInterface<kN>>);
static_assert(
    std::is_base_of_v<cobot::hal::JointInterface<kN>, cobot::pipeline::SimJointInterface<kN>>);
static_assert(cobot::common::VecN<kN>::RowsAtCompileTime == kN);
static_assert(cobot::common::Mat6N<kN>::RowsAtCompileTime == 6);
static_assert(cobot::common::Mat6N<kN>::ColsAtCompileTime == kN);
static_assert(std::is_default_constructible_v<cobot::pipeline::LogRecord<kN>>);
static_assert(std::is_default_constructible_v<cobot::pipeline::Config<kN>>);
static_assert(std::is_default_constructible_v<cobot::imperfect::ImperfectionConfig<kN>>);

TEST(Headers, Compile) {
  const cobot::imperfect::ImperfectionConfig<kN> ideal;
  EXPECT_EQ(ideal.sensor.sensor_delay_cycles, 0);
  EXPECT_EQ(ideal.actuator.torque_scale_std.norm(), 0.0);
}

}  // namespace
