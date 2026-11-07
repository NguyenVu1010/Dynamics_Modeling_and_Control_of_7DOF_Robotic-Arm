#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

#include <gtest/gtest.h>

#include "cobot/control/algorithm.hpp"
#include "cobot/control/tracking_controller.hpp"
#include "cobot/hal/joint_interface.hpp"
#include "cobot/planning/clik.hpp"

// The closed registry of tracking laws (ADR-023 "Thuật toán control.algorithm"): a constexpr
// name table, an enum with stable values and a TrackingController that is not polymorphic and
// whose compute() cannot throw.
namespace cobot::control {
namespace {

TEST(Algorithms, TableAndNames) {
  static_assert(kAlgorithms.size() == 3);  // computed_torque, pd_gravity, drive_position
  for (std::size_t i = 0; i < kAlgorithms.size(); ++i) {
    EXPECT_EQ(kAlgorithms[i].id, static_cast<Algorithm>(i)) << "entry " << i;
    EXPECT_STRNE(kAlgorithms[i].summary, "") << "entry " << i;
  }
  EXPECT_EQ(static_cast<int>(Algorithm::kComputedTorque), 0);
  EXPECT_EQ(static_cast<int>(Algorithm::kPdGravity), 1);
  EXPECT_STREQ(to_string(Algorithm::kComputedTorque), "computed_torque");
  EXPECT_STREQ(to_string(Algorithm::kPdGravity), "pd_gravity");
  for (const AlgorithmInfo& a : kAlgorithms) {
    EXPECT_STREQ(to_string(a.id), a.name);
    const std::optional<Algorithm> back = algorithm_from_string(a.name);
    ASSERT_TRUE(back.has_value()) << a.name;
    EXPECT_EQ(*back, a.id) << a.name;
  }
  EXPECT_EQ(algorithm_from_string("computed_torque"), Algorithm::kComputedTorque);
  EXPECT_EQ(algorithm_from_string("pd_gravity"), Algorithm::kPdGravity);
  for (const char* bad : {"Computed_Torque", "", "pd", "pd_gravity ", " computed_torque"}) {
    EXPECT_FALSE(algorithm_from_string(bad).has_value()) << "'" << bad << "'";
  }
  EXPECT_EQ(registered_algorithm_names(),
            std::string("computed_torque, pd_gravity, drive_position"));
}

TEST(Algorithms, StaticGuards) {
  static_assert(!std::is_polymorphic_v<TrackingController<7>>,
                "closed registry: no virtual dispatch (ADR-023)");
  static_assert(noexcept(std::declval<const TrackingController<7>&>().compute(
                    std::declval<const hal::JointState<7>&>(),
                    std::declval<const planning::JointReference<7>&>())),
                "TrackingController::compute is noexcept");
  static_assert(noexcept(std::declval<const TrackingController<7>&>().algorithm()));
  static_assert(std::is_same_v<std::underlying_type_t<Algorithm>, std::uint8_t>);
  SUCCEED();
}

}  // namespace
}  // namespace cobot::control
